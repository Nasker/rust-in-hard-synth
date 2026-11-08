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

mod display;
mod input;
mod status;
mod ui;
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

    // Onboard WS2812 RGB (GPIO48, free per the pin budget) — blue blink
    // until the web server is up, then dim green.
    status::init(p.pins.gpio48);

    // 1.3" SPI OLED (SH1106/SSD1306): CLK=GPIO9, MOSI=GPIO10, RES=GPIO11,
    // DC=GPIO12, CS=GPIO13 + Pot0(left)=GPIO2, Pot1=GPIO1 + Encoder=GPIO38/39/40.
    // Starts the animated SPLASH screen immediately on Core 0.
    ui::init(
        p.spi2,
        p.pins.gpio9,
        p.pins.gpio10,
        p.pins.gpio11,
        p.pins.gpio12,
        p.pins.gpio13,
        p.adc1,
        p.pins.gpio2,
        p.pins.gpio1,
        p.pins.gpio38,
        p.pins.gpio39,
        p.pins.gpio40,
    );

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

    // Warm the lazy 8 KB sine LUT on a big-stack thread — its OnceLock init
    // uses a multi-KB stack temporary, and the audio task's 24 KB stack is
    // too tight for that to be safe lazily (seen as a boot crash loop).
    std::thread::Builder::new()
        .stack_size(64 * 1024)
        .name("dsp_prewarm".into())
        .spawn(rinsynth_dsp::embedded::prewarm_dsp_tables)?
        .join()
        .expect("dsp prewarm panicked");

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
        status::set(status::Status::Error);
    } else {
        status::set(status::Status::Ready);
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
        // A panic in the render loop would leave the box silent forever —
        // reboot into a working state instead (~1 s) like the IDF panic
        // handler would have done without the catch_unwind.
        log::error!("audio task panicked — restarting system");
        unsafe { esp_idf_sys::esp_restart() }
    }
    unsafe { esp_idf_sys::vTaskDelete(core::ptr::null_mut()) }
}

fn audio_task(mut tx: I2sDriver<'_, I2sTx>, bank: Vec<model::presets::Preset>) {
    use rinsynth_dsp::embedded::{soft_clip, EmbeddedEngine};
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
    let spiram0 =
        unsafe { esp_idf_sys::heap_caps_get_free_size(esp_idf_sys::MALLOC_CAP_SPIRAM) };
    info!("engine: free heap before vm = {}", free0);
    // Lean concrete engine — 6 voices × 3 PolyBLEP oscs + TPT SVF + ModMatrix + FX.
    let mut vm = EmbeddedEngine::new(6, SYNTH_RATE);
    let free1 = unsafe { esp_idf_sys::esp_get_free_heap_size() };
    let spiram1 =
        unsafe { esp_idf_sys::heap_caps_get_free_size(esp_idf_sys::MALLOC_CAP_SPIRAM) };
    info!(
        "engine: free heap after vm = {} (-{} bytes, spiram -{})",
        free1,
        free0 - free1,
        spiram0 - spiram1
    );
    info!(
        "engine: stack hwm before preset = {}",
        unsafe { esp_idf_sys::uxTaskGetStackHighWaterMark(core::ptr::null_mut()) }
    );

    // A factory pad, if the bank ships one — otherwise the default saw lead.
    // NB: factory_presets() materializes the whole 85-preset bank via one
    // giant vec! literal — at opt-level 0 that smashes the stack (and then
    // the heap). For now: default "Init" lead preset, pushed to the
    // HEAVIEST setup the engine can run so the boot riff doubles as a
    // standing worst-case stress test — 4 sounding PolyBLEP oscs/voice,
    // Osc4 audio-rate FM + a live mod matrix, Tube distortion +
    // Chorus + Ping-Pong Delay + Reverb all on. If this plays clean, any
    // user patch fits too.
    let mut preset = model::presets::Preset::default();
    preset.name = "BootStress".to_string();
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
    preset.lfo2_rate = 5.5;
    preset.lfo2_depth = 0.45;
    preset.env3_attack = 0.02;
    preset.env3_decay = 0.5;
    preset.env3_sustain = 0.6;
    preset.env3_release = 0.4;
    preset.amp_release = 0.3; // tail rings ~half a step -> peak ~5 voices
    preset.osc3_level = 0.45; // 3rd oscillator: -12 st sub-square active!
    // 4th oscillator sounding — a detuned fifth Saw layer (also feeds the
    // Osc4 -> Osc1Pitch FM route below).
    preset.osc4_waveform = model::event::WaveformType::Saw;
    preset.osc4_level = 0.35;
    preset.osc4_semitones = 7;
    preset.osc4_pan = 0.25;
    // Live mod matrix incl. the most expensive per-sample route
    // (audio-rate Osc4 -> Osc1Pitch FM).
    preset.mod_matrix.slots[0] = model::mod_matrix::ModSlot {
        source: model::mod_matrix::ModSource::Osc4,
        dest: model::mod_matrix::ModDest::Osc1Pitch,
        amount: 0.12,
        enabled: true,
    };
    preset.mod_matrix.slots[1] = model::mod_matrix::ModSlot {
        source: model::mod_matrix::ModSource::Lfo2,
        dest: model::mod_matrix::ModDest::FilterResonance,
        amount: 0.30,
        enabled: true,
    };
    preset.mod_matrix.slots[2] = model::mod_matrix::ModSlot {
        source: model::mod_matrix::ModSource::Env3,
        dest: model::mod_matrix::ModDest::Osc3Pw,
        amount: 0.40,
        enabled: true,
    };
    // Full stereo FX chain with the priciest stage (Tube distortion) on.
    preset.distortion_enabled = true;
    preset.distortion_type = model::effects::DistortionType::Tube;
    preset.distortion_drive = 0.45;
    preset.distortion_mix = 0.55;
    preset.chorus_enabled = true;
    preset.chorus_rate = 0.8;
    preset.chorus_depth = 3.5;
    preset.chorus_mix = 0.25;
    preset.delay_enabled = true;
    preset.delay_time = 0.36;
    preset.delay_feedback = 0.35;
    preset.delay_mix = 0.22;
    preset.reverb_enabled = true;
    preset.reverb_room_size = 0.6;
    preset.reverb_damping = 0.45;
    preset.reverb_mix = 0.25;
    info!("engine: applying preset '{}' (4 osc + FM + all FX + Tube)", preset.name);
    info!(
        "engine: stack hwm after Preset::default = {}",
        unsafe { esp_idf_sys::uxTaskGetStackHighWaterMark(core::ptr::null_mut()) }
    );
    vm.apply_preset(&preset);
    info!(
        "engine: stack hwm after apply_preset = {}",
        unsafe { esp_idf_sys::uxTaskGetStackHighWaterMark(core::ptr::null_mut()) }
    );

    // Tempo-synced arpeggiator — a block-boundary note scheduler, NOT DSP.
    // It intercepts NoteOn/NoteOff (when enabled) and re-emits arpeggiated
    // events into the engine on the next block boundary. Zero per-sample cost.
    let mut arp = rinsynth_dsp::Arpeggiator::new();
    arp.apply_preset(&preset);
    // What the user actually plays once the stress riff ends — the old
    // nudged Init (3 oscs, Chorus+Delay+Reverb, no distortion, no matrix).
    // BootStress must stay a boot-only diagnostic: left as the live patch
    // it drags every later session straight into sustained overload.
    let mut play_preset = model::presets::Preset::default();
    play_preset.name = "Init+".to_string();
    play_preset.polyphony_mode = model::voice::PolyphonyMode::Poly;
    play_preset.filter_cutoff = 1800.0;
    play_preset.filter_resonance = 0.50;
    play_preset.filter_attack = 0.01;
    play_preset.filter_decay = 0.35;
    play_preset.filter_sustain = 0.25;
    play_preset.filter_release = 0.4;
    play_preset.filter_env_amount = 0.40;
    play_preset.lfo_destination = model::lfo::LfoDestination::FilterCutoff;
    play_preset.lfo_rate = 0.4;
    play_preset.lfo_depth = 0.20;
    play_preset.amp_release = 0.35;
    play_preset.osc3_level = 0.45;
    play_preset.chorus_enabled = true;
    play_preset.chorus_rate = 0.8;
    play_preset.chorus_depth = 3.5;
    play_preset.chorus_mix = 0.22;
    play_preset.delay_enabled = true;
    play_preset.delay_time = 0.36;
    play_preset.delay_feedback = 0.35;
    play_preset.delay_mix = 0.20;
    play_preset.reverb_enabled = true;
    play_preset.reverb_room_size = 0.55;
    play_preset.reverb_damping = 0.45;
    play_preset.reverb_mix = 0.20;

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

    // Boot riff — dark harmonic-minor showcase: a quick mono run climbs
    // D harmonic minor (the Bb->C# augmented second is the dark color),
    // dives back to the dominant A, then rolls strummed triads
    // i -> V -> iv -> i (V is major in harmonic minor — extra grim).
    // One ~5 s pass, then Init+.
    //
    // Voice math (BootStress = ~3.8 voices under the deadline): mono
    // steps cost 1 voice; strummed triads roll in over ~2 blocks each
    // and hold 64, so a chord's 0.3 s tail (~52 blocks) clears the next
    // strum 120 blocks later — never past ~4 voices, no crackle.
    const STRIKES: &[(&[u8], u32)] = &[
        (&[62], 18), // D4  } mono run up — one octave higher, 2x speed
        (&[64], 18), // E4
        (&[65], 18), // F4
        (&[67], 18), // G4
        (&[69], 18), // A4
        (&[70], 18), // Bb4
        (&[73], 18), // C#5
        (&[74], 18), // D5  } peak of the run
        (&[73], 18), // C#5 }
        (&[70], 18), // Bb4  } quick dive
        (&[69], 18), // A4   } lands on the dominant
        (&[50, 65, 69], 64),  // i  — Dm:  D3 F4 A4 (strummed)
        (&[57, 61, 64], 64),  // V  — A:   A3 C#4 E4 (major 3rd = darker)
        (&[55, 58, 62], 150), // iv — Gm:  G3 Bb4 D4 rings out to the end
    ];
    // Timeline as (strike, step_blocks) — the run sweeps at ~116 ms/note
    // (2x speed), chord steps hold their 120-block spacing so 0.3 s tails
    // never overlap the next strummed triad (64 hold + ~52 tail <= 120).
    const SEQ: &[(Option<usize>, u32)] = &[
        (Some(0), 20), (Some(1), 20), (Some(2), 20), (Some(3), 20),
        (Some(4), 20), (Some(5), 20), (Some(6), 20), (Some(7), 20),
        (Some(8), 20), (Some(9), 20), (Some(10), 20),
        (None, 40),
        (Some(11), 40), (None, 40), (None, 40),
        (Some(12), 40), (None, 40), (None, 40),
        (Some(13), 40), (None, 80), (None, 40),
    ];
    const SEQ_TOTAL: u32 = {
        let mut t = 0u32;
        let mut i = 0usize;
        while i < SEQ.len() {
            t += SEQ[i].1;
            i += 1;
        }
        t
    };
    const STRUM_BLOCKS: u32 = 6; // ~35 ms between chord tones — a roll
    const RIFF_VEL: u8 = 110;
    const RIFF_LOOPS: u32 = 1; // one arc — a showcase, not a loop

    info!("engine: demo riff start");
    let mut blocks: u32 = 0;
    let mut underruns: u32 = 0;
    let mut overruns: u32 = 0; // blocks whose render exceeded the DMA deadline
    let mut max_render_us: i64 = 0;
    let mut peak: f32 = 0.0;   // worst |sample| this second — >1.0 = limiter squashing
    let mut riff_active: bool = true;
    let mut riff_done_flag: bool = false;
    // (strike index, block struck) — drives the per-strike timed release.
    let mut last_stab: Option<(usize, u32)> = None;
    // SEQ position tracker: step_idx = which timeline entry fires next,
    // next_step_at = cumulative block count when it fires.
    let mut seq_step = 0usize;
    let mut next_step_at = 0u32;
    // Chord-roll state: on a strike, notes are queued and emitted one per
    // STRUM_BLOCKS (mono strikes are len 1 — they fire immediately).
    let mut strum_notes = [0u8; 4];
    let mut strum_len = 0usize;
    let mut strum_idx = 0usize;
    let mut strum_next_at = 0u32;
    // True once a factory preset was chosen (USB program change or web
    // UI) — suppresses the post-riff play-patch swap so we don't stomp
    // a deliberate selection.
    let mut preset_picked: bool = false;

    // CPU governor: when blocks render past the deadline the DMA underruns
    // (audible as harsh crackle, not clipping). Shed the oldest voice + lower
    // the live polyphony cap instead. Restoration requires sustained margin —
    // track the worst render over the clean window and only raise the cap if
    // it stayed under ~75% of the deadline (prevents cap boundary ping-pong).
    const MAX_VOICES: usize = 6;
    let mut dyn_cap: usize = MAX_VOICES;
    let mut late_streak: u32 = 0;
    // Rolling 64-block late/on-time history — sheds fire on late RATE,
    // not single streaks, so transient contention can't trigger them.
    let mut late_hist: u64 = 0;
    let mut cool: u32 = 0;
    let mut cool_max: i64 = 0;
    vm.set_voice_cap(dyn_cap);

    loop {
        // Early bring-up visibility: print every block for the first 20,
        // then settle into ~1 s windows.
        let verbose = blocks < 20;

        // Drain MIDI events queued by the USB task (block-boundary,
        // non-blocking — NoteEvents are Copy, ring is lock-free SPSC).
        while let Some(ev) = usb_midi::pop_event() {
            use model::event::SynthEventKind as K;
            if matches!(ev.kind, K::NoteOn) {
                status::notify(status::Event::MidiActivity);
            }
            // When the arpeggiator is on it swallows note events into its
            // held-chord list; everything else flows straight to the engine.
            if arp.enabled() {
                match ev.kind {
                    K::NoteOn => arp.note_on(ev.note, ev.velocity),
                    K::NoteOff => {
                        // Untracked notes (struck before the arp engaged)
                        // must still release their engine voice.
                        if let Some(fwd) = arp.note_off(ev.note) {
                            vm.receive_event(fwd);
                        }
                    }
                    _ => vm.receive_event(ev),
                }
            } else {
                vm.receive_event(ev);
            }
        }

        // Drain Web UI parameter/FX updates (lock-free SPSC ring from Core 0)
        while let Some(cmd) = web::pop_cmd() {
            match cmd {
                web::WebCmd::ArpOn(on) => arp.set_enabled(on),
                web::WebCmd::ArpMode(m) => {
                    arp.set_mode(model::arp::ArpMode::from_index(m))
                }
                web::WebCmd::ArpRate(b) => arp.set_rate_beats(b),
                web::WebCmd::ArpOct(o) => arp.set_octaves(o),
                web::WebCmd::ArpHold(h) => arp.set_hold(h),
                other => web::apply_cmd(&mut vm, other),
            }
        }

        // Publish live arp state for the OLED badge (also flips when a
        // preset that ships arp_on applies it via program change below).
        web::set_arp_enabled(arp.enabled());

        if let Some(prog) = usb_midi::take_program_change() {
            preset_picked = true;
            if !bank.is_empty() {
                let idx = (prog as usize) % bank.len();
                // No clone — apply_preset reads the preset by reference;
                // cloning allocated on the RT thread every program change.
                // apply_preset already honors preset.polyphony_mode —
                // don't override it here (that forced mono presets poly).
                vm.apply_preset(&bank[idx]);
                arp.apply_preset(&bank[idx]);
                web::set_current_preset_index(idx);
                web::set_current_bpm(vm.bpm());
                let p = &bank[idx];
                info!("usb-midi: preset [{}] '{}' ({})", idx, p.name, p.category);
            }
        }
        if let Some(clock_bpm) = usb_midi::take_clock_bpm() {
            vm.set_bpm(clock_bpm);
            web::set_current_bpm(vm.bpm());
        }

        if riff_active && (usb_midi::midi_seen() || blocks >= RIFF_LOOPS * SEQ_TOTAL) {
            if !riff_done_flag {
                info!("demo riff: done");
                riff_done_flag = true;
                web::set_riff_done(true);
                if !preset_picked {
                    vm.apply_preset(&play_preset);
                    arp.apply_preset(&play_preset);
                    info!("engine: stress riff over -> play patch '{}'", play_preset.name);
                }
            }
            if riff_active {
                for &(notes, _) in STRIKES {
                    for &n in notes {
                        vm.receive_event(NoteEvent::note_off(n));
                    }
                }
            }
            riff_active = false;
            last_stab = None;
            strum_idx = strum_len; // cancel any unstrummed chord tones
        }

        if riff_active && blocks == next_step_at {
            let (strike, step_dur) = SEQ[seq_step];
            seq_step = (seq_step + 1) % SEQ.len();
            next_step_at += step_dur;
            if let Some(strike) = strike {
                let (notes, _) = STRIKES[strike];
                let len = notes.len().min(strum_notes.len());
                strum_notes[..len].copy_from_slice(&notes[..len]);
                strum_len = len;
                strum_idx = 0;
                strum_next_at = blocks;
                last_stab = Some((strike, blocks));
            }
        }
        // Chord roll — emit one queued note every STRUM_BLOCKS (~35 ms).
        if riff_active && strum_idx < strum_len && blocks >= strum_next_at {
            vm.receive_event(NoteEvent::note_on_midi(
                strum_notes[strum_idx],
                RIFF_VEL,
            ));
            strum_idx += 1;
            strum_next_at += STRUM_BLOCKS;
        }
        // Timed strike release — voices finish their 0.3 s tail well
        // before the next strike lands, so voices never pile up.
        if let Some((strike, struck)) = last_stab {
            if blocks.wrapping_sub(struck) == STRIKES[strike].1 {
                for &n in STRIKES[strike].0 {
                    vm.receive_event(NoteEvent::note_off(n));
                }
                last_stab = None;
            }
        }

        let half = web::synth_half();
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
        if !half {
            // Full rate: render straight into the i16 buffer — the synth[]
            // intermediate only exists for the half-rate upsampler path.
            for i in 0..FRAMES {
                let s = vm.next_sample_stereo();
                if s.left.abs() > peak {
                    peak = s.left.abs();
                }
                if s.right.abs() > peak {
                    peak = s.right.abs();
                }
                buf[2 * i] = (s.left * 32767.0) as i16;
                buf[2 * i + 1] = (s.right * 32767.0) as i16;
            }
        }
        if half {
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
        }
        let dt = unsafe { esp_idf_sys::esp_timer_get_time() } - t0;

        if half {
            // 2x upsample: even outputs pass the synth sample (one
            // synth-frame delay, already soft_clip-guarded in
            // next_sample_stereo), odd outputs get a Catmull-Rom midpoint
            // (-p0 + 9p1 + 9p2 - p3) / 16 rounded by soft_clip in case CR
            // overshoots.
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
                buf[4 * i] = (hist[2].left * 32767.0) as i16;
                buf[4 * i + 1] = (hist[2].right * 32767.0) as i16;
                buf[4 * i + 2] = (soft_clip(mid_l) * 32767.0) as i16;
                buf[4 * i + 3] = (soft_clip(mid_r) * 32767.0) as i16;
            }
        }
        if dt > max_render_us {
            max_render_us = dt;
        }
        web::set_active_voices(vm.active_voice_count());

        // CPU governor + overrun accounting.
        let ceiling = web::user_voice_cap().min(MAX_VOICES);
        if dyn_cap > ceiling {
            dyn_cap = ceiling;
            vm.set_voice_cap(dyn_cap);
        }
        let late = dt > BLOCK_US;
        late_hist = (late_hist << 1) | late as u64;
        if late {
            // Rendering ran past the block's playback time — the I2S DMA
            // drains faster than we produce → audible underrun crackle.
            overruns += 1;
            late_streak += 1;
            cool = cool.saturating_sub(16);
            // Shed only on SUSTAINED overload — a transient Wi-Fi/PSRAM
            // burst makes a handful of blocks late and must not collapse
            // polyphony to mono for 10+ seconds:
            //  - 12+ consecutive lates (~70 ms of solid overrun), or
            //  - >=25% late across the last 64 blocks (~370 ms window).
            // Shed only on SUSTAINED overload — a transient Wi-Fi/PSRAM
            // burst makes a handful of blocks late and must not collapse
            // polyphony:
            //  - 12+ consecutive lates (~70 ms of solid overrun), or
            //  - >=25% late across the last 64 blocks (~370 ms window).
            if blocks >= WARMUP_BLOCKS
                && (late_streak >= 12 || late_hist.count_ones() >= 16)
            {
                // Lower the cap if possible (floor 1 — a single voice always
                // fits the deadline), then shed while more voices are
                // sounding than the cap allows.
                if dyn_cap > 1 {
                    dyn_cap -= 1;
                    vm.set_voice_cap(dyn_cap);
                }
                if vm.active_voice_count() > dyn_cap {
                    if let Some(n) = vm.shed_voice() {
                        info!("cpu-guard: sustained lates -> cap {} (shed note {})", dyn_cap, n);
                    }
                } else {
                    info!("cpu-guard: sustained lates -> cap {}", dyn_cap);
                }
                status::notify(status::Event::Overload);
                late_streak = 0;
                late_hist = 0;
            }
        } else {
            late_streak = 0;
            if dyn_cap < ceiling {
                cool += 1;
                if dt > cool_max {
                    cool_max = dt;
                }
                // ~2 s of on-time blocks AND the worst block still had ≥25%
                // headroom AND few recent lates — only then is one more
                // voice likely to fit.
                if cool >= 344 {
                    if cool_max < BLOCK_US * 3 / 4 && late_hist.count_ones() < 4 {
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
