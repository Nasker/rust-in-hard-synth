//! ESP-IDF audio task adapter: lifecycle, block ordering and I2S output.
//!
//! Core 1 exclusively owns the engine, arpeggiator and queue receivers. Policy,
//! demo scheduling, conversion and accounting live in the host-testable library.
//! The task retains its 24 KB internal-RAM stack and priority 10.

use crate::status;
use esp_idf_hal::{
    delay::FreeRtos,
    i2s::{I2sDriver, I2sTx},
};
use log::info;
use rinsynth_dsp::{embedded::EmbeddedEngine, Arpeggiator};
use rinsynth_esp32::{
    audio::{
        boot_demo::BootDemo,
        governor::{Decision, Governor},
        patches,
        render::BlockRenderer,
        telemetry::Telemetry,
        FRAMES, MAX_VOICES, SAMPLE_RATE, SYNTH_RATE,
    },
    control::{self, dispatch},
};
use rinsynth_model::presets::Preset;

struct AudioTaskArgs {
    tx: I2sDriver<'static, I2sTx>,
    bank: Vec<Preset>,
}

/// Transfer driver/catalog ownership to a pinned FreeRTOS task before Wi-Fi startup.
pub fn start(tx: I2sDriver<'static, I2sTx>, bank: Vec<Preset>) {
    let ctx = Box::new(AudioTaskArgs { tx, bank });
    // SAFETY: the boxed argument has a stable address and is reclaimed exactly
    // once by the trampoline. Driver and catalog move exclusively to Core 1.
    unsafe {
        esp_idf_hal::task::create(
            audio_task_trampoline,
            c"rinsynth-audio",
            24 * 1024,
            Box::into_raw(ctx).cast(),
            10,
            Some(esp_idf_hal::cpu::Core::Core1),
        )
        .expect("create audio task");
    }
}

extern "C" fn audio_task_trampoline(arg: *mut core::ffi::c_void) {
    // SAFETY: start passed this allocation to exactly one task trampoline.
    let args = unsafe { Box::from_raw(arg as *mut AudioTaskArgs) };
    if std::panic::catch_unwind(move || run(args.tx, args.bank)).is_err() {
        // A panic in the render loop would leave the box silent forever —
        // reboot into a working state instead (~1 s) like the IDF panic
        // handler would have done without the catch_unwind.
        log::error!("audio task panicked — restarting system");
        unsafe { esp_idf_sys::esp_restart() }
    }
    unsafe { esp_idf_sys::vTaskDelete(core::ptr::null_mut()) }
}

fn run(mut tx: I2sDriver<'_, I2sTx>, bank: Vec<Preset>) {
    let mut events = control::event_receiver().expect("audio event receiver already claimed");
    let mut commands = control::command_receiver().expect("audio command receiver already claimed");
    tx.tx_enable().expect("tx_enable");
    info!("I2S TX running @ {} Hz stereo 16-bit", SAMPLE_RATE);
    let free0 = unsafe { esp_idf_sys::esp_get_free_heap_size() };
    let spiram0 = unsafe { esp_idf_sys::heap_caps_get_free_size(esp_idf_sys::MALLOC_CAP_SPIRAM) };
    let mut vm = EmbeddedEngine::new(MAX_VOICES, SYNTH_RATE);
    info!(
        "engine: free heap after vm = {} (-{} bytes, spiram -{})",
        unsafe { esp_idf_sys::esp_get_free_heap_size() },
        free0 - unsafe { esp_idf_sys::esp_get_free_heap_size() },
        spiram0 - unsafe { esp_idf_sys::heap_caps_get_free_size(esp_idf_sys::MALLOC_CAP_SPIRAM) }
    );
    let preset = patches::stress();
    info!(
        "engine: applying preset '{}' (4 osc + FM + all FX + Tube)",
        preset.name
    );
    vm.apply_preset(&preset);
    let mut arp = Arpeggiator::new();
    arp.apply_preset(&preset);
    let play_preset = patches::play();
    vm.set_master_volume(0.7);
    install_profiler();
    info!("engine: stack hwm after apply_preset = {}", unsafe {
        esp_idf_sys::uxTaskGetStackHighWaterMark(core::ptr::null_mut())
    });

    // 2 s of silence so the DAC/amp settles before sound
    let mut renderer = BlockRenderer::default();
    for _ in 0..(SAMPLE_RATE / FRAMES as u32 * 2) {
        tx.write(renderer.bytes(), 1000).ok();
    }
    let mut demo = BootDemo::default();
    let mut telemetry = Telemetry::default();
    let mut governor = Governor::default();
    let mut preset_picked = false;
    vm.set_voice_cap(governor.cap());
    info!("engine: demo riff start");

    loop {
        while let Some(ev) = events.pop() {
            if matches!(ev.kind, rinsynth_model::event::SynthEventKind::NoteOn) {
                status::notify(status::Event::MidiActivity);
            }
            dispatch::receive_event(&mut vm, &mut arp, ev);
        }
        while let Some(cmd) = commands.pop() {
            dispatch::apply_cmd(&mut vm, &mut arp, cmd);
        }
        control::set_arp_enabled(arp.enabled());
        if let Some(prog) = control::take_program_change() {
            preset_picked = true;
            if let Some(idx) = dispatch::select_preset(&mut vm, &mut arp, &bank, prog) {
                let p = &bank[idx];
                info!("usb-midi: preset [{}] '{}' ({})", idx, p.name, p.category);
            }
        }
        if let Some(clock_bpm) = control::take_clock_bpm() {
            vm.set_bpm(clock_bpm);
            control::set_current_bpm(vm.bpm());
        }
        if demo.should_finish(telemetry.blocks, control::midi_seen()) {
            info!("demo riff: done");
            control::set_riff_done(true);
            if !preset_picked {
                vm.apply_preset(&play_preset);
                arp.apply_preset(&play_preset);
                info!(
                    "engine: stress riff over -> play patch '{}'",
                    play_preset.name
                );
            }
            demo.cancel(|ev| vm.receive_event(ev));
        }
        demo.tick(telemetry.blocks, |ev| vm.receive_event(ev));

        let half = control::synth_half();
        let synth_frames = if half { FRAMES / 2 } else { FRAMES };
        // Arpeggiator scheduler — emits its step events on the block
        // boundary at the *synth* rate (half rate halves sample count too).
        if arp.enabled() {
            let eff_rate = if half {
                SYNTH_RATE as f32 * 0.5
            } else {
                SYNTH_RATE as f32
            };
            arp.tick(synth_frames as u32, vm.bpm(), eff_rate, &mut |ev| {
                vm.receive_event(ev)
            });
        }
        let t0 = unsafe { esp_idf_sys::esp_timer_get_time() };
        renderer.generate(&mut vm, half, &mut telemetry.peak);
        let dt = unsafe { esp_idf_sys::esp_timer_get_time() } - t0;
        if half {
            renderer.interpolate();
        }
        control::set_active_voices(vm.active_voice_count());

        let old_cap = governor.cap();
        let decision = governor.observe(telemetry.blocks, dt, control::user_voice_cap());
        if governor.cap() != old_cap {
            vm.set_voice_cap(governor.cap());
        }
        match decision {
            Decision::Shed => {
                if vm.active_voice_count() > governor.cap() {
                    if let Some(note) = vm.shed_voice() {
                        info!(
                            "cpu-guard: sustained lates -> cap {} (shed note {})",
                            governor.cap(),
                            note
                        );
                    }
                } else {
                    info!("cpu-guard: sustained lates -> cap {}", governor.cap());
                }
                status::notify(status::Event::Overload);
            }
            Decision::Restored { worst_us } => {
                info!(
                    "cpu-guard: margin ok (worst {}us) -> cap {}",
                    worst_us,
                    governor.cap()
                );
            }
            Decision::Unchanged => {}
        }
        if tx.write(renderer.bytes(), 1000).is_err() {
            telemetry.write_failed();
        }
        telemetry.finish_block(
            dt,
            vm.active_voice_count(),
            governor.cap(),
            control::event_count(),
        );
        FreeRtos::delay_ms(0);
    }
}

fn install_profiler() {
    // Per-stage cycle profiler — install only for diagnostic builds; the
    // probes themselves cost ~150 cyc/voice-sample (five rsr.ccount + atomics)
    // which is real headroom in production. Set true when auditing.
    const PROFILE_HOOK: bool = false;
    fn ccount() -> u64 {
        unsafe { esp_idf_sys::xthal_get_ccount() as u64 }
    }
    if PROFILE_HOOK {
        rinsynth_dsp::profile::set_hook(ccount);
    }
    info!(
        "engine: profile enabled={} ccount={}",
        rinsynth_dsp::profile::enabled(),
        ccount()
    );
}
