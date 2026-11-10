//! Physical-control value mapping, independent of UI rendering and hardware.
//!
//! Curves retain the existing encoder/pot feel. They are not silently replaced
//! with model defaults during this extraction. Commands go only to the supplied
//! sink; in particular distortion drive emits enable before drive.

use crate::control::SynthCommand;
use rinsynth_model::params::SynthParam;

#[derive(Clone, Copy, PartialEq, Eq)]
pub enum ParamTarget {
    Cutoff,
    Resonance,
    FilterEnvAmt,
    FilterAttack,
    FilterDecay,
    FilterSustain,
    FilterRelease,
    AmpAttack,
    AmpDecay,
    AmpSustain,
    AmpRelease,
    Env3Attack,
    Env3Decay,
    Env3Sustain,
    Env3Release,
    Osc1Wave,
    Osc1Level,
    Osc2Wave,
    Osc2Level,
    Osc2Semi,
    Osc2Detune,
    Osc3Wave,
    Osc3Level,
    Osc4Wave,
    Osc4Level,
    Lfo1Rate,
    Lfo1Depth,
    Lfo2Rate,
    Lfo2Depth,
    Glide,
    StereoWidth,
    MasterVol,
    DistDrive,
    DistMix,
    ChorusRate,
    ChorusDepth,
    ChorusMix,
    DelayTime,
    DelayFb,
    DelayMix,
    RevRoom,
    RevDamp,
    RevMix,
    PolyMode,
    UnisonVoices,
    UnisonDetune,
    UnisonSpread,
    ArpOn,
    ArpHold,
    ArpMode,
    ArpRate,
    ArpOct,
}

impl ParamTarget {
    /// Convert a normalized `0..=100` UI value into a [`SynthCommand`] for the audio engine.
    pub fn to_cmd(self, pct: u8) -> SynthCommand {
        let norm = (pct.min(100) as f32) * 0.01;
        // Exponential 0.005..2.0 s envelope time mapping
        let env_t = |n: f32| 0.005 + n * n * 1.995;
        match self {
            Self::Cutoff => {
                // Logarithmic 20 Hz .. 20 kHz
                let hz = 20.0 * (1000.0f32).powf(norm);
                SynthCommand::Param(SynthParam::FilterCutoff, hz)
            }
            Self::Resonance => SynthCommand::Param(SynthParam::FilterResonance, norm * 0.95),
            Self::FilterEnvAmt => {
                SynthCommand::Param(SynthParam::FilterEnvAmount, norm * 2.0 - 1.0)
            }
            Self::FilterAttack => SynthCommand::Param(SynthParam::FilterAttack, env_t(norm)),
            Self::FilterDecay => SynthCommand::Param(SynthParam::FilterDecay, env_t(norm)),
            Self::FilterSustain => SynthCommand::Param(SynthParam::FilterSustain, norm),
            Self::FilterRelease => SynthCommand::Param(SynthParam::FilterRelease, env_t(norm)),
            Self::AmpAttack => SynthCommand::Param(SynthParam::Attack, env_t(norm)),
            Self::AmpDecay => SynthCommand::Param(SynthParam::Decay, env_t(norm)),
            Self::AmpSustain => SynthCommand::Param(SynthParam::Sustain, norm),
            Self::AmpRelease => SynthCommand::Param(SynthParam::Release, env_t(norm)),
            Self::Env3Attack => SynthCommand::Env3Attack(env_t(norm)),
            Self::Env3Decay => SynthCommand::Env3Decay(env_t(norm)),
            Self::Env3Sustain => SynthCommand::Env3Sustain(norm),
            Self::Env3Release => SynthCommand::Env3Release(env_t(norm)),
            Self::Osc1Wave => SynthCommand::Param(SynthParam::Osc1Waveform, self.index(pct) as f32),
            Self::Osc1Level => SynthCommand::Param(SynthParam::Osc1Level, norm),
            Self::Osc2Wave => SynthCommand::Param(SynthParam::Osc2Waveform, self.index(pct) as f32),
            Self::Osc2Level => SynthCommand::Param(SynthParam::Osc2Level, norm),
            Self::Osc2Semi => SynthCommand::Param(
                SynthParam::Osc2Semitones,
                ((norm * 48.0).round() - 24.0).clamp(-24.0, 24.0),
            ),
            Self::Osc2Detune => SynthCommand::Param(
                SynthParam::Osc2Cents,
                ((norm * 100.0).round() - 50.0).clamp(-50.0, 50.0),
            ),
            Self::Osc3Wave => SynthCommand::Param(SynthParam::Osc3Waveform, self.index(pct) as f32),
            Self::Osc3Level => SynthCommand::Param(SynthParam::Osc3Level, norm),
            Self::Osc4Wave => SynthCommand::Param(SynthParam::Osc4Waveform, self.index(pct) as f32),
            Self::Osc4Level => SynthCommand::Param(SynthParam::Osc4Level, norm),
            Self::Lfo1Rate => SynthCommand::Param(SynthParam::LfoRate, 0.1 + norm * norm * 19.9),
            Self::Lfo1Depth => SynthCommand::Param(SynthParam::LfoDepth, norm),
            Self::Lfo2Rate => SynthCommand::Lfo2Rate(0.1 + norm * norm * 19.9),
            Self::Lfo2Depth => SynthCommand::Lfo2Depth(norm),
            Self::Glide => SynthCommand::Param(SynthParam::PortamentoTime, norm * norm * 0.5),
            Self::StereoWidth => SynthCommand::Param(SynthParam::StereoWidth, norm * 2.0),
            Self::MasterVol => SynthCommand::Param(SynthParam::MasterVolume, norm),
            Self::DistDrive => SynthCommand::DistDrive(norm),
            Self::DistMix => SynthCommand::DistMix(norm),
            Self::ChorusRate => SynthCommand::ChorusRate(0.1 + norm * 4.9),
            Self::ChorusDepth => SynthCommand::ChorusDepth(norm * 10.0),
            Self::ChorusMix => SynthCommand::ChorusMix(norm),
            Self::DelayTime => SynthCommand::DelayTime(0.02 + norm * 1.18),
            Self::DelayFb => SynthCommand::DelayFb(norm * 0.90),
            Self::DelayMix => SynthCommand::DelayMix(norm),
            Self::RevRoom => SynthCommand::RevRoom(norm),
            Self::RevDamp => SynthCommand::RevDamp(norm),
            Self::RevMix => SynthCommand::RevMix(norm),
            Self::PolyMode => SynthCommand::PolyMode(self.index(pct) == 1),
            Self::UnisonVoices => SynthCommand::Unison(1 + self.index(pct) as u8),
            Self::UnisonDetune => SynthCommand::UniDetune(norm * 50.0),
            Self::UnisonSpread => SynthCommand::UniSpread(norm),
            Self::ArpOn => SynthCommand::ArpOn(self.index(pct) == 1),
            Self::ArpHold => SynthCommand::ArpHold(self.index(pct) == 1),
            Self::ArpMode => SynthCommand::ArpMode(self.index(pct) as u8),
            Self::ArpRate => {
                // Stepped beat divisions, slow -> fast: 1/2 .. 1/32.
                const DIVS: [f32; 7] = [2.0, 1.0, 0.75, 0.5, 0.375, 0.25, 0.125];
                SynthCommand::ArpRate(DIVS[self.index(pct)])
            }
            Self::ArpOct => SynthCommand::ArpOct(1 + self.index(pct) as u8),
        }
    }

    /// Emit all commands for one edit through the caller's sink, without global writes.
    pub fn emit_commands(self, pct: u8, mut emit: impl FnMut(SynthCommand)) {
        if matches!(self, Self::DistDrive) {
            emit(SynthCommand::DistEnable(pct > 2));
        }
        emit(self.to_cmd(pct));
    }

    /// Number of discrete positions for stepped params (binary toggles,
    /// mode selects, octave counts). `None` = continuous 0..100 control —
    /// stepped params edit one position per detent, not a % of a bar.
    pub fn steps(self) -> Option<u8> {
        match self {
            Self::ArpOn | Self::ArpHold | Self::PolyMode => Some(2),
            Self::ArpMode | Self::ArpOct => Some(4),
            Self::ArpRate => Some(7),
            Self::UnisonVoices => Some(6),
            Self::Osc1Wave | Self::Osc2Wave | Self::Osc3Wave | Self::Osc4Wave => Some(5),
            _ => None,
        }
    }

    /// Discrete index `0..steps-1` a normalized pct falls into.
    pub fn index(self, pct: u8) -> usize {
        let s = self.steps().unwrap_or(1) as usize;
        ((pct as usize * s) / 100).min(s - 1)
    }

    /// Mid-bucket pct for index `i` — maps back to the same index under
    /// [`index`](Self::index) (floor buckets don't round-trip for odd
    /// step counts, so we aim at the bucket's center).
    pub fn index_pct(self, idx: usize) -> u8 {
        let s = self.steps().unwrap_or(1) as usize;
        ((idx * 2 + 1) * 50 / s) as u8
    }

    /// Human-readable label for discrete/stepped parameters (shown instead
    /// of the raw 0-100 number on the SECTION page). `None` = numeric.
    /// Index math matches [`index`](Self::index) / [`steps`](Self::steps).
    pub fn value_label(self, pct: u8) -> Option<&'static str> {
        let i = self.index(pct);
        match self {
            Self::ArpOn | Self::ArpHold => Some(if i == 1 { "ON" } else { "OFF" }),
            Self::PolyMode => Some(if i == 1 { "POLY" } else { "MONO" }),
            Self::ArpMode => Some(["UP", "DOWN", "U/DN", "RND"][i]),
            Self::ArpRate => Some(["1/2", "1/4", "1/8D", "1/8", "16D", "1/16", "1/32"][i]),
            Self::ArpOct => Some(["1", "2", "3", "4"][i]),
            Self::UnisonVoices => Some(["1", "2", "3", "4", "5", "6"][i]),
            Self::Osc1Wave | Self::Osc2Wave | Self::Osc3Wave | Self::Osc4Wave => {
                Some(["SIN", "SQR", "SAW", "TRI", "NSE"][i])
            }
            _ => None,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn stepped_positions_roundtrip_for_all_discrete_controls() {
        for target in [
            ParamTarget::ArpOn,
            ParamTarget::ArpHold,
            ParamTarget::PolyMode,
            ParamTarget::ArpMode,
            ParamTarget::ArpOct,
            ParamTarget::ArpRate,
            ParamTarget::UnisonVoices,
            ParamTarget::Osc1Wave,
        ] {
            let steps = target.steps().unwrap() as usize;
            for index in 0..steps {
                assert_eq!(target.index(target.index_pct(index)), index);
                assert!(target.value_label(target.index_pct(index)).is_some());
            }
            assert_eq!(target.index(0), 0);
            assert_eq!(target.index(100), steps - 1);
        }
    }

    #[test]
    fn continuous_curves_preserve_existing_ranges() {
        for (pct, expected) in [(0, 20.0), (50, 632.4555), (100, 20000.0)] {
            let SynthCommand::Param(SynthParam::FilterCutoff, hz) = ParamTarget::Cutoff.to_cmd(pct)
            else {
                panic!()
            };
            assert!((hz - expected).abs() < 0.01);
        }
        let SynthCommand::Param(SynthParam::Attack, time) = ParamTarget::AmpAttack.to_cmd(50)
        else {
            panic!()
        };
        assert!((time - 0.50375).abs() < 0.00001);
    }

    #[test]
    fn distortion_edit_emits_enable_then_drive_without_a_global_queue() {
        for pct in [0, 2, 3, 100] {
            let mut commands = Vec::new();
            ParamTarget::DistDrive.emit_commands(pct, |cmd| commands.push(cmd));
            assert_eq!(commands.len(), 2);
            assert!(matches!(commands[0], SynthCommand::DistEnable(on) if on == (pct > 2)));
            assert!(
                matches!(commands[1], SynthCommand::DistDrive(value) if value == pct as f32 * 0.01)
            );
        }
    }
}
