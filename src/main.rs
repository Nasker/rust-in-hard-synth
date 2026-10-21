//! rinsynth-esp32 — Phase 1: prove I²S on the PCM5102.
//! Boots, outputs silence for 2 s, then a 440 Hz sine test tone,
//! logging underrun/error counters every 10 s.
//!
//! Phase 2 smoke test: exercises `rinsynth-model` (param spec, preset
//! schema, mod matrix, factory bank) on-device before the tone starts.
//!
//! PCM5102 wiring (proven on serviSynth):
//!   BCK = GPIO6, LRCK = GPIO4, DOUT = GPIO5, MCLK = GPIO7 (256×fs)

use esp_idf_hal::delay::FreeRtos;
use esp_idf_hal::i2s::config::{Config, DataBitWidth, SlotMode, StdClkConfig, StdConfig, StdGpioConfig, StdSlotConfig};
use esp_idf_hal::i2s::I2sDriver;
use esp_idf_hal::peripherals::Peripherals;
use esp_idf_sys as _;
use log::info;
use rinsynth_model as model;

const SAMPLE_RATE: u32 = 44_100;
const FRAMES: usize = 256;

fn main() -> anyhow::Result<()> {
    esp_idf_sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();
    info!("rinsynth-esp32 {} — I2S test tone", env!("CARGO_PKG_VERSION"));

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

    let mut tx = I2sDriver::new_std_tx(
        p.i2s0,
        &std_cfg,
        p.pins.gpio6,       // BCK
        p.pins.gpio5,       // DOUT -> PCM5102 DIN
        Some(p.pins.gpio7), // MCLK
        p.pins.gpio4,       // LRCK/WS
    )?;

    tx.tx_enable()?;
    info!("I2S TX running @ {} Hz stereo 16-bit", SAMPLE_RATE);

    // 2 s of silence so the DAC/amp settles before the tone
    let mut buf = [0i16; FRAMES * 2];
    for _ in 0..(SAMPLE_RATE / FRAMES as u32 * 2) {
        tx.write(bytemuck_cast_i16(&buf), 1000)?;
    }

    info!("tone: 440 Hz sine");
    let mut phase: f32 = 0.0;
    let inc = 440.0 / SAMPLE_RATE as f32;
    let mut blocks: u32 = 0;
    let mut underruns: u32 = 0;

    loop {
        for i in 0..FRAMES {
            let s = (phase * 2.0 * core::f32::consts::PI).sin() * 0.5;
            let v = (s * 32767.0) as i16;
            buf[2 * i] = v;
            buf[2 * i + 1] = v;
            phase += inc;
            if phase >= 1.0 {
                phase -= 1.0;
            }
        }
        match tx.write(bytemuck_cast_i16(&buf), 1000) {
            Ok(_) => {}
            Err(_) => underruns += 1,
        }
        blocks += 1;
        if blocks % 1723 == 0 {
            // ~10 s of blocks
            info!("blocks: {} | write errors: {}", blocks, underruns);
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
