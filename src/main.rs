//! rinsynth-esp32 — ESP32-S3 firmware port of RustInSynth.
//!
//! Phase 1: I²S to the PCM5102 (44.1 kHz / 16-bit stereo).
//! Phase 2: `rinsynth-model` on-device (param spec, preset serde, factory bank).
//! Phase 3: `rinsynth-dsp` — the actual engine renders a polyphonic
//!          factory-preset arpeggio, with per-block render-time telemetry.
//!
//! PCM5102 wiring (proven on serviSynth):
//!   BCK = GPIO6, LRCK = GPIO4, DOUT = GPIO5, MCLK = GPIO7 (256×fs)

use esp_idf_hal::delay::FreeRtos;
use esp_idf_hal::i2s::config::{
    Config, DataBitWidth, SlotMode, StdClkConfig, StdConfig, StdGpioConfig, StdSlotConfig,
};
use esp_idf_hal::i2s::{I2sDriver, I2sTx};
use esp_idf_hal::peripherals::Peripherals;
use esp_idf_sys as _;
use log::info;
use rinsynth_model as model;

const SAMPLE_RATE: u32 = 44_100;
/// Internal engine rate. Full-rate (44.1 kHz) keeps harmonics unaliased;
/// half-rate (22.05 kHz) halves voice cost but folds >11 kHz content back
/// as inharmonic partials — audible as a glassy edge on bright patches.
/// Flip SYNTH_RATE + HALF_RATE together (interpolator below handles 2x).
const SYNTH_RATE: u32 = 44_100;
const HALF_RATE: bool = SYNTH_RATE != SAMPLE_RATE;
const FRAMES: usize = 256;
const SYNTH_FRAMES: usize = if HALF_RATE { FRAMES / 2 } else { FRAMES };
/// 256 output frames / 44.1 kHz
const BLOCK_US: i64 = 5805;

fn main() -> anyhow::Result<()> {
    esp_idf_sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();
    info!("rinsynth-esp32 {} — dsp engine render", env!("CARGO_PKG_VERSION"));

    // serde recursion through ~90 Preset fields needs far more stack than
    // the 16 KB main task — run the smoke test on a dedicated thread.
    std::thread::Builder::new()
        .stack_size(64 * 1024)
        .name("model_smoke".into())
        .spawn(model_smoke)?
        .join()
        .expect("model smoke test panicked");

    let p = Peripherals::take()?;

    let std_cfg = StdConfig::new(
        Config::default(),
        StdClkConfig::from_sample_rate_hz(SAMPLE_RATE),
        StdSlotConfig::philips_slot_default(DataBitWidth::Bits16, SlotMode::Stereo),
        StdGpioConfig::default(),
    );

    let tx = I2sDriver::new_std_tx(
        p.i2s0,
        &std_cfg,
        p.pins.gpio6,       // BCK
        p.pins.gpio5,       // DOUT -> PCM5102 DIN
        Some(p.pins.gpio7), // MCLK
        p.pins.gpio4,       // LRCK/WS
    )?;

    // The engine needs a fat stack too (apply_preset + Voice init), and the
    // render loop must never share the small main-task stack with logging.
    // Spawned as a raw FreeRTOS task: the ThreadSpawnConfiguration/
    // esp_pthread_set_cfg path demonstrably did NOT size the pthread
    // (hwm read 0 with a nominal 160 KB stack), while xTaskCreatePinnedToCore
    // gives deterministic stack + priority + core pinning in one call.
    let tx = Box::new(tx);
    unsafe {
        esp_idf_hal::task::create(
            audio_task_trampoline,
            c"rinsynth-audio",
            64 * 1024,
            Box::into_raw(tx) as *mut _,
            10,
            Some(esp_idf_hal::cpu::Core::Core1),
        )
        .expect("create audio task");
    }

    // Main task idles from here on — keep it out of the audio path.
    loop {
        esp_idf_hal::delay::FreeRtos::delay_ms(60_000);
    }
}

extern "C" fn audio_task_trampoline(arg: *mut core::ffi::c_void) {
    let tx = unsafe { Box::from_raw(arg as *mut I2sDriver<'static, I2sTx>) };
    if std::panic::catch_unwind(move || audio_task(*tx)).is_err() {
        log::error!("audio task panicked — halting task");
    }
    unsafe { esp_idf_sys::vTaskDelete(core::ptr::null_mut()) }
}

fn audio_task(mut tx: I2sDriver<'_, I2sTx>) {
    use rinsynth_dsp::voice::VoiceManager;
    use rinsynth_model::event::{NoteEvent, SynthEventReceiver};
    

    tx.tx_enable().expect("tx_enable");
    info!("I2S TX running @ {} Hz stereo 16-bit", SAMPLE_RATE);

    // Heap sanity on this pthread before touching the engine
    let probe = String::from("alloc-probe");
    info!("engine: trivial alloc ok: {}", probe);
    info!(
        "engine: heap total={} internal={} default8bit={}",
        unsafe { esp_idf_sys::esp_get_free_heap_size() },
        unsafe { esp_idf_sys::heap_caps_get_free_size(esp_idf_sys::MALLOC_CAP_INTERNAL) },
        unsafe { esp_idf_sys::heap_caps_get_free_size(esp_idf_sys::MALLOC_CAP_8BIT) },
    );

    let free0 = unsafe { esp_idf_sys::esp_get_free_heap_size() };
    info!("engine: free heap before vm = {}", free0);
    // 8 voices internally — at 22.05 kHz the per-voice cost halves, so a
    // 3-4 voice overlap fits the 5.8 ms block deadline again.
    let mut vm = VoiceManager::new(8, SYNTH_RATE);
    let free1 = unsafe { esp_idf_sys::esp_get_free_heap_size() };
    info!("engine: free heap after vm = {} (-{} bytes)", free1, free0 - free1);
    info!(
        "engine: stack hwm before preset = {}",
        unsafe { esp_idf_sys::uxTaskGetStackHighWaterMark(core::ptr::null_mut()) }
    );

    // A factory pad, if the bank ships one — otherwise the default saw lead.
    // NB: factory_presets() materializes the whole 85-preset bank via one
    // giant vec! literal — at opt-level 0 that smashes the stack (and then
    // the heap). For now: default "Init" lead preset, tastefully nudged.
    let mut preset = model::presets::Preset::default();
    preset.polyphony_mode = model::voice::PolyphonyMode::Poly;
    preset.lfo_destination = model::lfo::LfoDestination::FilterCutoff;
    preset.lfo_rate = 0.4;
    preset.lfo_depth = 0.25;
    preset.osc3_level = 0.0; // two detuned saws — enough for the demo, ~140cyc/sample cheaper
    info!("engine: applying preset '{}'", preset.name);
    info!(
        "engine: stack hwm after Preset::default = {}",
        unsafe { esp_idf_sys::uxTaskGetStackHighWaterMark(core::ptr::null_mut()) }
    );
    vm.apply_preset(&preset);
    info!(
        "engine: stack hwm after apply_preset = {}",
        unsafe { esp_idf_sys::uxTaskGetStackHighWaterMark(core::ptr::null_mut()) }
    );
    vm.set_master_volume(0.7);

    // Per-stage cycle profiler — install only when needed; the probes
    // themselves cost ~150 cyc/voice-sample (five rsr.ccount + atomics).
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

    // 2 s of silence so the DAC/amp settles before sound
    let mut buf = [0i16; FRAMES * 2];
    for _ in 0..(SAMPLE_RATE / FRAMES as u32 * 2) {
        tx.write(bytemuck_cast_i16(&buf), 1000).ok();
    }
    let mut synth = [model::types::StereoSample::ZERO; SYNTH_FRAMES];
    let mut hist = [model::types::StereoSample::ZERO; 4];

    // Cmaj9-style arp loop: 8 steps, voices overlap and ring (poly mode).
    const ARP: [u8; 8] = [48, 64, 67, 71, 74, 71, 67, 64];
    const STEP_BLOCKS: u32 = 43; // ~250 ms

    info!("engine: arp start");
    let mut blocks: u32 = 0;
    let mut underruns: u32 = 0;
    let mut max_render_us: i64 = 0;

    loop {
        // Early bring-up visibility: print every block for the first 20,
        // then settle into ~1 s windows.
        let verbose = blocks < 20;
        // note on this step's key
        if blocks % STEP_BLOCKS == 0 {
            let step = (blocks / STEP_BLOCKS) as usize;
            vm.receive_event(NoteEvent::note_on_midi(ARP[step % ARP.len()], 104));
        }
        // release the key from 3 steps back — bounds voice count while
        // letting release tails overlap
        if blocks % STEP_BLOCKS == STEP_BLOCKS / 2 {
            let step = ((blocks / STEP_BLOCKS) as usize).wrapping_sub(3);
            vm.receive_event(NoteEvent::note_off(ARP[step % ARP.len()]));
        }

        let t0 = unsafe { esp_idf_sys::esp_timer_get_time() };
        for i in 0..SYNTH_FRAMES {
            synth[i] = vm.next_sample_stereo();
        }
        let dt = unsafe { esp_idf_sys::esp_timer_get_time() } - t0;

        if HALF_RATE {
            // 2x upsample: even outputs pass the synth sample (one
            // synth-frame delay), odd outputs get a Catmull-Rom midpoint
            // (-p0 + 9p1 + 9p2 - p3) / 16.
            for i in 0..SYNTH_FRAMES {
                hist[0] = hist[1];
                hist[1] = hist[2];
                hist[2] = hist[3];
                hist[3] = synth[i];
                let mid_l = (9.0 * (hist[1].left + hist[2].left)
                    - hist[0].left
                    - hist[3].left)
                    * 0.0625;
                let mid_r = (9.0 * (hist[1].right + hist[2].right)
                    - hist[0].right
                    - hist[3].right)
                    * 0.0625;
                buf[4 * i] = (hist[2].left.clamp(-1.0, 1.0) * 32767.0) as i16;
                buf[4 * i + 1] = (hist[2].right.clamp(-1.0, 1.0) * 32767.0) as i16;
                buf[4 * i + 2] = (mid_l.clamp(-1.0, 1.0) * 32767.0) as i16;
                buf[4 * i + 3] = (mid_r.clamp(-1.0, 1.0) * 32767.0) as i16;
            }
        } else {
            for i in 0..FRAMES {
                let s = synth[i];
                buf[2 * i] = (s.left.clamp(-1.0, 1.0) * 32767.0) as i16;
                buf[2 * i + 1] = (s.right.clamp(-1.0, 1.0) * 32767.0) as i16;
            }
        }
        if dt > max_render_us {
            max_render_us = dt;
        }

        if tx.write(bytemuck_cast_i16(&buf), 1000).is_err() {
            underruns += 1;
        }
        if verbose {
            info!(
                "block {} | voices: {} | render: {}us",
                blocks,
                vm.active_voice_count(),
                dt
            );
        }
        blocks += 1;
        if blocks % 172 == 0 {
            // ~1 s of blocks
            info!(
                "blocks: {} | voices: {} | render max: {}us/{}us | write errors: {}",
                blocks,
                vm.active_voice_count(),
                max_render_us,
                BLOCK_US,
                underruns
            );
            // Per-stage cycle breakdown (cycles per rendered sample, this second)
            let snap = rinsynth_dsp::profile::snapshot();
            if snap[rinsynth_dsp::profile::SAMPLES] > 0 {
                let n = snap[rinsynth_dsp::profile::SAMPLES];
                info!(
                    "profile cyc/sample: env={} ctrl={} osc={} filt={} (n={})",
                    snap[rinsynth_dsp::profile::ENV] / n,
                    snap[rinsynth_dsp::profile::CONTROL] / n,
                    snap[rinsynth_dsp::profile::OSC] / n,
                    snap[rinsynth_dsp::profile::FILTER] / n,
                    n
                );
            }
            rinsynth_dsp::profile::reset();
            max_render_us = 0;
        }
        FreeRtos::delay_ms(0);
    }
}

fn bytemuck_cast_i16(b: &[i16]) -> &[u8] {
    unsafe { core::slice::from_raw_parts(b.as_ptr() as *const u8, b.len() * 2) }
}

/// Phase 2 smoke test: prove the portable model crate works on-target —
/// param spec, CC map, preset serde roundtrip, mod matrix, factory bank.
fn model_smoke() {
    use model::types::midi_to_frequency;

    let params = model::params::SynthParam::all();
    let ccmap = model::params::CCMapping::default_mappings();
    info!(
        "model: {} params, {} default CC mappings",
        params.len(),
        ccmap.list_mappings().len()
    );

    // Preset serde roundtrip (v2 envelope)
    let preset = model::presets::Preset::new("smoke");
    let json = preset.to_json().expect("preset serialize");
    let back = model::presets::Preset::from_json(&json).expect("preset parse");
    assert_eq!(preset, back, "preset roundtrip mismatch");
    info!("model: preset serde roundtrip ok ({} bytes)", json.len());

    // Mod matrix + legacy LFO migration helper
    let slot = model::voice::legacy_lfo_slot(&back);
    let bank = model::presets::factory_presets();
    info!(
        "model: factory bank = {} presets, legacy_lfo_slot = {:?}",
        bank.len(),
        slot.map(|s| s.dest)
    );

    // Type sanity: midi→freq table
    info!("model: A4 = {:.1} Hz", midi_to_frequency(69));
}
