//! Big-stack model initialization and DSP table prewarming.
//!
//! Complete these operations before spawning the 24 KB audio task. Factory-bank
//! materialization and sine-table initialization require larger temporary stacks.

use log::info;
use rinsynth_model as model;

pub fn load_bank() -> anyhow::Result<Vec<model::presets::Preset>> {
    // serde recursion + factory bank materialization need far more stack
    // than the 16 KB main task — run on a 64 KB thread and return the bank.
    let bank = std::thread::Builder::new()
        .stack_size(64 * 1024)
        .name("model_smoke".into())
        .spawn(model_smoke)?
        .join()
        .expect("model smoke test panicked");
    Ok(bank)
}

pub fn prewarm_dsp() -> anyhow::Result<()> {
    // Warm the lazy 8 KB sine LUT on a big-stack thread — its OnceLock init
    // uses a multi-KB stack temporary, and the audio task's 24 KB stack is
    // too tight for that to be safe lazily (seen as a boot crash loop).
    std::thread::Builder::new()
        .stack_size(64 * 1024)
        .name("dsp_prewarm".into())
        .spawn(rinsynth_dsp::embedded::prewarm_dsp_tables)?
        .join()
        .expect("dsp prewarm panicked");
    Ok(())
}

/// On-target model smoke test and factory-bank materialization, before audio starts.
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
