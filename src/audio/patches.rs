//! Boot-only stress patch and the post-showcase Init+ play patch.
//!
//! Constructed once during audio startup, never during steady-state rendering.

use rinsynth_model::{
    effects::DistortionType,
    event::WaveformType,
    lfo::LfoDestination,
    mod_matrix::{ModDest, ModSlot, ModSource},
    presets::Preset,
    voice::PolyphonyMode,
};

fn base() -> Preset {
    Preset {
        polyphony_mode: PolyphonyMode::Poly,
        filter_cutoff: 1800.0,
        filter_resonance: 0.50,
        filter_attack: 0.01,
        filter_decay: 0.35,
        filter_sustain: 0.25,
        filter_release: 0.4,
        filter_env_amount: 0.40,
        lfo_destination: LfoDestination::FilterCutoff,
        lfo_rate: 0.4,
        lfo_depth: 0.20,
        osc3_level: 0.45,
        chorus_enabled: true,
        chorus_rate: 0.8,
        chorus_depth: 3.5,
        delay_enabled: true,
        delay_time: 0.36,
        delay_feedback: 0.35,
        reverb_enabled: true,
        reverb_damping: 0.45,
        ..Preset::default()
    }
}

pub fn stress() -> Preset {
    let mut preset = Preset {
        name: "BootStress".to_string(),
        lfo2_rate: 5.5,
        lfo2_depth: 0.45,
        env3_attack: 0.02,
        env3_decay: 0.5,
        env3_sustain: 0.6,
        env3_release: 0.4,
        amp_release: 0.3,
        // 4th oscillator sounding — a detuned fifth Saw layer (also feeds the
        // Osc4 -> Osc1Pitch FM route below).
        osc4_waveform: WaveformType::Saw,
        osc4_level: 0.35,
        osc4_semitones: 7,
        osc4_pan: 0.25,
        // Full stereo FX chain with the priciest stage (Tube distortion) on.
        distortion_enabled: true,
        distortion_type: DistortionType::Tube,
        distortion_drive: 0.45,
        distortion_mix: 0.55,
        chorus_mix: 0.25,
        delay_mix: 0.22,
        reverb_room_size: 0.6,
        reverb_mix: 0.25,
        ..base()
    };
    // Live mod matrix incl. the most expensive per-sample route
    // (audio-rate Osc4 -> Osc1Pitch FM).
    preset.mod_matrix.slots[0] = ModSlot {
        source: ModSource::Osc4,
        dest: ModDest::Osc1Pitch,
        amount: 0.12,
        enabled: true,
    };
    preset.mod_matrix.slots[1] = ModSlot {
        source: ModSource::Lfo2,
        dest: ModDest::FilterResonance,
        amount: 0.30,
        enabled: true,
    };
    preset.mod_matrix.slots[2] = ModSlot {
        source: ModSource::Env3,
        dest: ModDest::Osc3Pw,
        amount: 0.40,
        enabled: true,
    };
    preset
}

pub fn play() -> Preset {
    Preset {
        name: "Init+".to_string(),
        amp_release: 0.35,
        chorus_mix: 0.22,
        delay_mix: 0.20,
        reverb_room_size: 0.55,
        reverb_mix: 0.20,
        ..base()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn diagnostic_and_play_patch_keep_their_intended_cost_difference() {
        let stress = stress();
        let play = play();
        assert_eq!(stress.name, "BootStress");
        assert_eq!(stress.polyphony_mode, PolyphonyMode::Poly);
        assert!(
            stress.distortion_enabled
                && stress.chorus_enabled
                && stress.delay_enabled
                && stress.reverb_enabled
        );
        assert!(stress.mod_matrix.slots[..3].iter().all(|slot| slot.enabled));
        assert_eq!(stress.osc4_level, 0.35);
        assert_eq!(play.name, "Init+");
        assert!(!play.distortion_enabled);
        assert!(play.chorus_enabled && play.delay_enabled && play.reverb_enabled);
    }
}
