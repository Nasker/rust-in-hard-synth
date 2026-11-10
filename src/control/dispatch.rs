//! Audio-thread command and preset application, independent of input transport.
//!
//! Call only at block boundaries. Most commands update coefficients in place;
//! SynthHalf retains the existing engine rate-change allocation exception.

use super::{state, SynthCommand as C};
use rinsynth_dsp::{effects::DistortionType, embedded::EmbeddedEngine, Arpeggiator, FilterMode};
use rinsynth_model::{
    event::{SynthEvent, SynthEventKind},
    lfo::LfoWaveform,
    presets::Preset,
    voice::PolyphonyMode,
};

/// Route musical events through the arpeggiator when enabled. Notes predating
/// engagement still receive their NoteOff directly if the arp does not track them.
pub fn receive_event(vm: &mut EmbeddedEngine, arp: &mut Arpeggiator, ev: SynthEvent) {
    if arp.enabled() {
        match ev.kind {
            SynthEventKind::NoteOn => arp.note_on(ev.note, ev.velocity),
            SynthEventKind::NoteOff => {
                if let Some(forward) = arp.note_off(ev.note) {
                    vm.receive_event(forward);
                }
            }
            _ => vm.receive_event(ev),
        }
    } else {
        vm.receive_event(ev);
    }
}

/// Apply the selected factory preset without cloning it or forcing voice mode.
/// Returns its wrapped index, or None for an empty catalog.
pub fn select_preset(
    vm: &mut EmbeddedEngine,
    arp: &mut Arpeggiator,
    bank: &[Preset],
    prog: u8,
) -> Option<usize> {
    if bank.is_empty() {
        return None;
    }
    let idx = prog as usize % bank.len();
    vm.apply_preset(&bank[idx]);
    arp.apply_preset(&bank[idx]);
    state::set_current_preset_index(idx);
    state::set_current_bpm(vm.bpm());
    Some(idx)
}

pub fn apply_cmd(vm: &mut EmbeddedEngine, arp: &mut Arpeggiator, cmd: C) {
    match cmd {
        C::Nop => {}
        C::Param(p, v) => vm.set_param(p, v),
        C::FilterMode(m) => vm.set_filter_mode(match m {
            1 => FilterMode::HighPass,
            2 => FilterMode::BandPass,
            3 => FilterMode::Notch,
            _ => FilterMode::LowPass,
        }),
        C::ModWheel(v) => vm.set_mod_wheel(v),
        C::Lfo2Rate(v) => vm.set_lfo2_rate(v),
        C::Lfo2Depth(v) => vm.set_lfo2_depth(v),
        C::Lfo2Wf(w) => vm.set_lfo2_waveform(match w {
            0 => LfoWaveform::Sine,
            1 => LfoWaveform::Triangle,
            2 => LfoWaveform::Square,
            3 => LfoWaveform::Saw,
            _ => LfoWaveform::Random,
        }),
        C::Env3Attack(v) => vm.set_env3_attack(v),
        C::Env3Decay(v) => vm.set_env3_decay(v),
        C::Env3Sustain(v) => vm.set_env3_sustain(v),
        C::Env3Release(v) => vm.set_env3_release(v),
        C::ModSlot(idx, slot) => vm.set_mod_slot(idx as usize, slot),
        C::DistEnable(on) => vm.effects.set_distortion_enabled(on),
        C::DistType(t) => vm
            .effects
            .set_distortion_type(DistortionType::from_index(t)),
        C::DistDrive(v) => vm.effects.set_distortion_drive(v),
        C::DistMix(v) => {
            // Mix>0 implicitly enables, matching the other FX convention.
            if v > 0.001 {
                vm.effects.set_distortion_enabled(true);
            }
            vm.effects.set_distortion_mix(v);
        }
        C::VoiceCap(n) => state::set_user_voice_cap(n),
        C::SynthHalf(on) => {
            state::set_synth_half(on);
            vm.set_sample_rate(if on { 22_050 } else { 44_100 });
        }
        C::ChorusRate(v) => vm.effects.set_chorus_rate(v),
        C::ChorusDepth(v) => vm.effects.set_chorus_depth(v),
        C::ChorusMix(v) => {
            vm.effects.set_chorus_enabled(v > 0.001);
            vm.effects.set_chorus_mix(v);
        }
        C::DelayTime(v) => vm.effects.set_delay_time(v),
        C::DelayFb(v) => vm.effects.set_delay_feedback(v),
        C::DelayMix(v) => {
            vm.effects.set_delay_enabled(v > 0.001);
            vm.effects.set_delay_mix(v);
        }
        C::RevRoom(v) => vm.effects.set_reverb_room_size(v),
        C::RevDamp(v) => vm.effects.set_reverb_damping(v),
        C::RevMix(v) => {
            vm.effects.set_reverb_enabled(v > 0.001);
            vm.effects.set_reverb_mix(v);
        }
        C::Bpm(v) => {
            vm.set_bpm(v);
            state::set_current_bpm(vm.bpm());
        }
        C::Lfo1Sync(on) => vm.set_lfo1_sync_enabled(on),
        C::Lfo1Beats(b) => vm.set_lfo1_beats(b),
        C::Lfo2Sync(on) => vm.set_lfo2_sync_enabled(on),
        C::Lfo2Beats(b) => vm.set_lfo2_beats(b),
        C::DelaySync(on) => vm.set_delay_sync(on),
        C::DelayBeats(b) => vm.set_delay_beats(b),
        C::PolyMode(poly) => vm.set_polyphony_mode(if poly {
            PolyphonyMode::Poly
        } else {
            PolyphonyMode::Mono
        }),
        C::Unison(n) => vm.set_unison_count(n as usize),
        C::UniDetune(c) => vm.set_unison_detune(c),
        C::UniSpread(s) => vm.set_unison_spread(s),
        C::ArpOn(on) => arp.set_enabled(on),
        C::ArpMode(m) => arp.set_mode(rinsynth_model::arp::ArpMode::from_index(m)),
        C::ArpRate(b) => arp.set_rate_beats(b),
        C::ArpOct(o) => arp.set_octaves(o),
        C::ArpHold(h) => arp.set_hold(h),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use rinsynth_model::event::NoteEvent;

    #[test]
    fn commands_control_engine_and_arp_without_transport_dependencies() {
        let mut vm = EmbeddedEngine::new(6, 44100);
        let mut arp = Arpeggiator::new();
        apply_cmd(&mut vm, &mut arp, C::PolyMode(false));
        assert_eq!(vm.polyphony_mode(), PolyphonyMode::Mono);
        apply_cmd(&mut vm, &mut arp, C::ArpOn(true));
        receive_event(&mut vm, &mut arp, NoteEvent::note_on_midi(60, 100));
        assert_eq!(vm.active_voice_count(), 0);
        arp.tick(256, 120.0, 44100.0, &mut |ev| vm.receive_event(ev));
        assert!(vm.active_voice_count() > 0);
    }

    #[test]
    fn preset_selection_preserves_mono_and_wraps_index() {
        let mut vm = EmbeddedEngine::new(6, 44100);
        let mut arp = Arpeggiator::new();
        let mut preset = Preset::default();
        preset.polyphony_mode = PolyphonyMode::Mono;
        assert_eq!(select_preset(&mut vm, &mut arp, &[], 42), None);
        assert_eq!(select_preset(&mut vm, &mut arp, &[preset], 42), Some(0));
        assert_eq!(vm.polyphony_mode(), PolyphonyMode::Mono);
    }
}
