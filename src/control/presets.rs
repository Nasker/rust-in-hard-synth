//! Immutable preset metadata, prepared before the audio task starts.
//!
//! Strings are allocated only during initialization. Consumers borrow stable
//! catalog entries; no JSON parsing or audio-thread cloning is required.

use rinsynth_model::{presets::Preset, voice::PolyphonyMode};
use std::sync::OnceLock;

static PRESET_SUMMARIES: OnceLock<Vec<PresetSummary>> = OnceLock::new();

/// Compact, pre-extracted metadata for the OLED UI so Core 0 never parses JSON
/// or contends with Core 1 for preset strings/macro bars.
#[derive(Clone)]
pub struct PresetSummary {
    pub name: String,
    pub category: String,
    pub is_mono: bool,
    /// Category-based 4 macro bar levels in 0..=100%.
    pub macro_pct: [u8; 4],
    /// FX stage enable state [distortion, chorus, delay, reverb] — feeds the
    /// OLED bottom-row FX kill-switch readout.
    pub fx_on: [bool; 4],
}

pub fn preset_summary(idx: usize) -> Option<&'static PresetSummary> {
    PRESET_SUMMARIES.get().and_then(|v| v.get(idx))
}
pub fn preset_count() -> usize {
    PRESET_SUMMARIES.get().map_or(0, |v| v.len())
}

pub fn init(bank: &[Preset]) {
    let summaries = bank
        .iter()
        .map(|p| PresetSummary {
            name: p.name.clone(),
            category: p.category.clone(),
            is_mono: p.polyphony_mode == PolyphonyMode::Mono,
            macro_pct: macro_bars_for(p),
            fx_on: [
                p.distortion_enabled,
                p.chorus_enabled,
                p.delay_enabled,
                p.reverb_enabled,
            ],
        })
        .collect();
    let _ = PRESET_SUMMARIES.set(summaries);
}

fn macro_bars_for(p: &Preset) -> [u8; 4] {
    let pct = |x: f32| (x.clamp(0.0, 1.0) * 100.0) as u8;
    let cut = (((p.filter_cutoff.clamp(20.0, 20_000.0) / 20.0).ln() / (20_000.0f32 / 20.0).ln())
        * 100.0)
        .clamp(0.0, 100.0) as u8;
    let res = pct(p.filter_resonance);
    let fenv = pct((p.filter_env_amount + 1.0) * 0.5);
    match p.category.as_str() {
        "Bass" => [cut, res, fenv, pct(p.filter_decay / 2.0)],
        "Lead" => [cut, res, pct(p.portamento_time / 0.5), fenv],
        "Pad" => [
            pct(p.amp_attack / 2.0),
            cut,
            pct(if p.reverb_enabled { p.reverb_mix } else { 0.0 }),
            pct(if p.chorus_enabled { p.chorus_mix } else { 0.0 }),
        ],
        "Keys" => [
            cut,
            pct(p.amp_decay / 2.0),
            pct(p.amp_release / 2.0),
            pct(if p.reverb_enabled { p.reverb_mix } else { 0.0 }),
        ],
        _ => [
            cut,
            res,
            pct(if p.delay_enabled { p.delay_mix } else { 0.0 }),
            pct(if p.reverb_enabled { p.reverb_mix } else { 0.0 }),
        ],
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn macro_bars_retain_category_mapping_and_disabled_fx_values() {
        let mut p = Preset::default();
        p.category = "Pad".into();
        p.amp_attack = 1.0;
        p.filter_cutoff = 20_000.0;
        p.reverb_enabled = true;
        p.reverb_mix = 0.25;
        p.chorus_enabled = false;
        assert_eq!(macro_bars_for(&p), [50, 100, 25, 0]);
        p.category = "Bass".into();
        p.filter_resonance = 0.5;
        p.filter_env_amount = 0.0;
        p.filter_decay = 1.0;
        assert_eq!(macro_bars_for(&p), [100, 50, 50, 50]);
    }
}
