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

mod usb_midi;
mod web;

const SAMPLE_RATE: u32 = 44_100;
/// Internal engine rate. Full-rate (44.1 kHz) keeps harmonics unaliased;
/// half-rate (22.05 kHz) halves voice cost but folds >11 kHz content back
/// as inharmonic partials — audible as a glassy edge on bright patches.
/// Boot render rate — the web `srate` toggle switches 44100 ↔ 22050 at
/// runtime (128 synth frames + Catmull-Rom 2x upsample, interpolator below).
const SYNTH_RATE: u32 = 44_100;
const FRAMES: usize = 256;
const SYNTH_FRAMES: usize = FRAMES; // buffer size; runtime half-rate uses FRAMES/2
/// 256 output frames / 44.1 kHz
const BLOCK_US: i64 = 5805;
/// Blocks (~5.8 ms each) to let the governor shed voices. Wi-Fi/HTTP bring-up
/// starves the render core for the first seconds; don't shed during it.
const WARMUP_BLOCKS: u32 = 1_720;

fn main() -> anyhow::Result<()> {
    esp_idf_sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();
    info!("rinsynth-esp32 {} — dsp engine render", env!("CARGO_PKG_VERSION"));

    // serde recursion + factory bank materialization need far more stack
    // than the 16 KB main task — run on a 64 KB thread and return the bank.
    let bank = std::thread::Builder::new()
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

    let modem = p.modem;
    let tx = I2sDriver::new_std_tx(
        p.i2s0,
        &std_cfg,
        p.pins.gpio6,       // BCK
        p.pins.gpio5,       // DOUT -> PCM5102 DIN
        Some(p.pins.gpio7), // MCLK
        p.pins.gpio4,       // LRCK/WS
    )?;

    // Build preset JSON tables and spawn audio task FIRST (before Wi-Fi
    // fragments internal SRAM). High-water mark shows audio_task uses ~7.3 KB,
    // so 24 KB gives >3x headroom while leaving ~110 KB internal RAM for lwIP/Wi-Fi.
    web::init_presets_json(&bank);

    let ctx = Box::new(AudioTaskArgs { tx, bank });
    unsafe {
        esp_idf_hal::task::create(
            audio_task_trampoline,
            c"rinsynth-audio",
            24 * 1024,
            Box::into_raw(ctx) as *mut _,
            10,
            Some(esp_idf_hal::cpu::Core::Core1),
        )
        .expect("create audio task");
    }

    if let Err(e) = web::start(modem) {
        log::error!("web: failed to start SoftAP/HTTP server: {:?}", e);
    }

    // Install USB host on Core 0 (main task) so the USB OTG ISR lands on
    // Core 0 and never preempts the Core 1 audio render loop.
    usb_midi::start();

    // Main task idles from here on — keep it out of the audio path.
    loop {
        esp_idf_hal::delay::FreeRtos::delay_ms(60_000);
    }
}

struct AudioTaskArgs {
    tx: I2sDriver<'static, I2sTx>,
    bank: Vec<model::presets::Preset>,
}

extern "C" fn audio_task_trampoline(arg: *mut core::ffi::c_void) {
    let args = unsafe { Box::from_raw(arg as *mut AudioTaskArgs) };
    if std::panic::catch_unwind(move || audio_task(args.tx, args.bank)).is_err() {
        log::error!("audio task panicked — halting task");
    }
    unsafe { esp_idf_sys::vTaskDelete(core::ptr::null_mut()) }
}

fn audio_task(mut tx: I2sDriver<'_, I2sTx>, bank: Vec<model::presets::Preset>) {
    use rinsynth_dsp::embedded::EmbeddedEngine;
    use rinsynth_model::event::{NoteEvent};
    

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
    // Lean concrete engine — 6 voices × 3 PolyBLEP oscs + TPT SVF + ModMatrix + FX.
    let mut vm = EmbeddedEngine::new(6, SYNTH_RATE);
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
    preset.filter_cutoff = 1800.0;
    preset.filter_resonance = 0.50;
    preset.filter_attack = 0.01;
    preset.filter_decay = 0.35;
    preset.filter_sustain = 0.25;
    preset.filter_release = 0.4;
    preset.filter_env_amount = 0.40;
    preset.lfo_destination = model::lfo::LfoDestination::FilterCutoff;
    preset.lfo_rate = 0.4;
    preset.lfo_depth = 0.20;
    preset.amp_release = 0.35; // brassy stab release — keeps riff overlap bounded
    preset.osc3_level = 0.45; // 3rd oscillator: -12 st sub-square active!
    // Enable lush stereo FX chain (Chorus -> Ping-Pong Delay -> Reverb)
    preset.chorus_enabled = true;
    preset.chorus_rate = 0.8;
    preset.chorus_depth = 3.5;
    preset.chorus_mix = 0.22;
    preset.delay_enabled = true;
    preset.delay_time = 0.36;
    preset.delay_feedback = 0.35;
    preset.delay_mix = 0.20;
    preset.reverb_enabled = true;
    preset.reverb_room_size = 0.55;
    preset.reverb_damping = 0.45;
    preset.reverb_mix = 0.20;
    info!("engine: applying preset '{}' (TPT SVF + Chorus/Delay/Reverb)", preset.name);
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

    // 2 s of silence so the DAC/amp settles before sound
    let mut buf = [0i16; FRAMES * 2];
    for _ in 0..(SAMPLE_RATE / FRAMES as u32 * 2) {
        tx.write(bytemuck_cast_i16(&buf), 1000).ok();
    }
    let mut synth = [model::types::StereoSample::ZERO; SYNTH_FRAMES];
    let mut hist = [model::types::StereoSample::ZERO; 4];

    // Van Halen "Jump" intro riff: ascending triad stabs G → C → F → G in
    // steady eighths at ~130 BPM. Each stab is 3 voices + release tails
    // (peak ~5-6 active voices). Loops until any MIDI/Web note arrives.
    const RIFF: &[[u8; 3]] = &[
        [67, 71, 74], // G: G4 B4 D5
        [72, 76, 79], // C: C5 E5 G5
        [65, 69, 72], // F: F4 A4 C5
        [67, 71, 74], // G
    ];
    // Eighth-note timeline: Some(i) = strike chord i, None = let it ring.
    const SEQ: [Option<usize>; 8] = [
        Some(0), None, Some(1), None, Some(2), Some(2), Some(3), None,
    ];
    const STEP_BLOCKS: u32 = 80; // eighth note @ ~65 BPM — half-time feel
    const RIFF_VEL: u8 = 110;
    const RIFF_LOOPS: u32 = 4; // ~10 s of intro, then silence

    info!("engine: demo riff start");
    let mut blocks: u32 = 0;
    let mut underruns: u32 = 0;
    let mut overruns: u32 = 0; // blocks whose render exceeded the DMA deadline
    let mut max_render_us: i64 = 0;
    let mut peak: f32 = 0.0;   // worst |sample| this second — >1.0 = limiter squashing
    let mut arp_active: bool = true;
    let mut arp_done: bool = false;
    let mut last_stab: Option<usize> = None;

    // CPU governor: when blocks render past the deadline the DMA underruns
    // (audible as harsh crackle, not clipping). Shed the oldest voice + lower
    // the live polyphony cap instead. Restoration requires sustained margin —
    // track the worst render over the clean window and only raise the cap if
    // it stayed under ~75% of the deadline (prevents cap boundary ping-pong).
    const MAX_VOICES: usize = 6;
    let mut dyn_cap: usize = MAX_VOICES;
    let mut late_streak: u32 = 0;
    let mut cool: u32 = 0;
    let mut cool_max: i64 = 0;
    vm.set_voice_cap(MAX_VOICES);

    loop {
        // Early bring-up visibility: print every block for the first 20,
        // then settle into ~1 s windows.
        let verbose = blocks < 20;

        // Drain MIDI events queued by the USB task (block-boundary,
        // non-blocking — NoteEvents are Copy, ring is lock-free SPSC).
        while let Some(ev) = usb_midi::pop_event() {
            vm.receive_event(ev);
        }

        // Drain Web UI parameter/FX updates (lock-free SPSC ring from Core 0)
        while let Some(cmd) = web::pop_cmd() {
            web::apply_cmd(&mut vm, cmd);
        }

        if let Some(prog) = usb_midi::take_program_change() {
            if !bank.is_empty() {
                let idx = (prog as usize) % bank.len();
                let mut p = bank[idx].clone();
                p.polyphony_mode = model::voice::PolyphonyMode::Poly;
                vm.apply_preset(&p);
                web::set_current_preset_index(idx);
                web::set_current_bpm(vm.bpm());
                info!("usb-midi: preset [{}] '{}' ({})", idx, p.name, p.category);
            }
        }
        if let Some(clock_bpm) = usb_midi::take_clock_bpm() {
            vm.set_bpm(clock_bpm);
            web::set_current_bpm(vm.bpm());
        }

        if arp_active && (usb_midi::midi_seen() || blocks >= RIFF_LOOPS * SEQ.len() as u32 * STEP_BLOCKS) {
            if !arp_done {
                info!("demo riff: done");
                arp_done = true;
            }
            if arp_active {
                for chord in RIFF {
                    for &n in chord {
                        vm.receive_event(NoteEvent::note_off(n));
                    }
                }
            }
            arp_active = false;
        }

        if arp_active && blocks % STEP_BLOCKS == 0 {
            let step = (blocks / STEP_BLOCKS) as usize % SEQ.len();
            if let Some(chord) = SEQ[step] {
                // Release the previous stab, then strike the next triad —
                // tails overlap through the release so it stays brassy.
                if let Some(prev) = last_stab {
                    for &n in &RIFF[prev] {
                        vm.receive_event(NoteEvent::note_off(n));
                    }
                }
                for &n in &RIFF[chord] {
                    vm.receive_event(NoteEvent::note_on_midi(n, RIFF_VEL));
                }
                last_stab = Some(chord);
            }
        }

        let half = web::synth_half();
        let synth_frames = if half { FRAMES / 2 } else { FRAMES };
        let t0 = unsafe { esp_idf_sys::esp_timer_get_time() };
        for i in 0..synth_frames {
            let s = vm.next_sample_stereo();
            if s.left.abs() > peak {
                peak = s.left.abs();
            }
            if s.right.abs() > peak {
                peak = s.right.abs();
            }
            synth[i] = s;
        }
        let dt = unsafe { esp_idf_sys::esp_timer_get_time() } - t0;

        if half {
            // 2x upsample: even outputs pass the synth sample (one
            // synth-frame delay), odd outputs get a Catmull-Rom midpoint
            // (-p0 + 9p1 + 9p2 - p3) / 16.
            for i in 0..synth_frames {
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

        // CPU governor + overrun accounting.
        let ceiling = web::user_voice_cap().min(MAX_VOICES);
        if dyn_cap > ceiling {
            dyn_cap = ceiling;
            vm.set_voice_cap(dyn_cap);
        }
        if dt > BLOCK_US {
            // Rendering ran past the block's playback time — the I2S DMA
            // drains faster than we produce → audible underrun crackle.
            overruns += 1;
            late_streak += 1;
            cool = 0;
            // Warm-up: ignore late streaks for ~10 s after boot — the Wi-Fi
            // connect + HTTP churn burst starves the render core and would
            // shed voices before the user even plays, then take ~5 s/voice
            // to restore. Overruns are still counted for telemetry.
            if late_streak >= 4 && blocks >= WARMUP_BLOCKS {
                // Lower the cap if possible (floor 1 — a single voice always
                // fits the deadline), then shed while more voices are
                // sounding than the cap allows.
                if dyn_cap > 1 {
                    dyn_cap -= 1;
                    vm.set_voice_cap(dyn_cap);
                }
                if vm.active_voice_count() > dyn_cap {
                    if let Some(n) = vm.shed_voice() {
                        info!("cpu-guard: 4 late blocks -> cap {} (shed note {})", dyn_cap, n);
                    }
                } else {
                    info!("cpu-guard: 4 late blocks -> cap {}", dyn_cap);
                }
                late_streak = 0;
            }
        } else {
            late_streak = 0;
            if dyn_cap < ceiling {
                cool += 1;
                if dt > cool_max {
                    cool_max = dt;
                }
                // ~5 s of on-time blocks AND the worst block still had ≥25%
                // headroom — only then is one more voice likely to fit.
                if cool >= 860 {
                    if cool_max < BLOCK_US * 3 / 4 {
                        dyn_cap += 1;
                        vm.set_voice_cap(dyn_cap);
                        info!("cpu-guard: margin ok (worst {}us) -> cap {}", cool_max, dyn_cap);
                    }
                    cool = 0;
                    cool_max = 0;
                }
            } else {
                cool_max = 0;
            }
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
                "blocks: {} | voices: {}/{} | midi: {} | render max: {}us/{}us | late: {} | peak: {:.2} | write errors: {}",
                blocks,
                vm.active_voice_count(),
                dyn_cap,
                usb_midi::event_count(),
                max_render_us,
                BLOCK_US,
                overruns,
                peak,
                underruns
            );
            // Per-stage cycle breakdown (cycles per rendered sample, this second)
            let snap = rinsynth_dsp::profile::snapshot();
            if snap[rinsynth_dsp::profile::SAMPLES] > 0 {
                let n = snap[rinsynth_dsp::profile::SAMPLES];
                info!(
                    "profile cyc/sample: env={} ctrl={} osc={} filt={} fx={} (n={})",
                    snap[rinsynth_dsp::profile::ENV] / n,
                    snap[rinsynth_dsp::profile::CONTROL] / n,
                    snap[rinsynth_dsp::profile::OSC] / n,
                    snap[rinsynth_dsp::profile::FILTER] / n,
                    snap[rinsynth_dsp::profile::FX] / n,
                    n
                );
            }
            rinsynth_dsp::profile::reset();
            max_render_us = 0;
            overruns = 0;
            peak = 0.0;
        }
        FreeRtos::delay_ms(0);
    }
}

fn bytemuck_cast_i16(b: &[i16]) -> &[u8] {
    unsafe { core::slice::from_raw_parts(b.as_ptr() as *const u8, b.len() * 2) }
}

/// Phase 2 smoke test: prove the portable model crate works on-target —
/// param spec, CC map, preset serde roundtrip, mod matrix, factory bank.
fn model_smoke() -> Vec<model::presets::Preset> {
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
    bank
}
