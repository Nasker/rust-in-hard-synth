//! Transport-neutral block-boundary synth commands.

use rinsynth_model::{mod_matrix::ModSlot, params::SynthParam};

#[derive(Clone, Copy)]
pub enum SynthCommand {
    Nop,
    Param(SynthParam, f32),
    FilterMode(u8),
    ModWheel(f32),
    Lfo2Rate(f32),
    Lfo2Depth(f32),
    Lfo2Wf(u8),
    Env3Attack(f32),
    Env3Decay(f32),
    Env3Sustain(f32),
    Env3Release(f32),
    ModSlot(u8, ModSlot),
    DistEnable(bool),
    DistType(u8),
    DistDrive(f32),
    DistMix(f32),
    /// User-set polyphony ceiling (1..6) — the CPU governor never exceeds it.
    VoiceCap(u8),
    /// Runtime A/B: render the engine at 22050 Hz (true) vs 44100 Hz (false).
    SynthHalf(bool),
    ChorusRate(f32),
    ChorusDepth(f32),
    ChorusMix(f32),
    DelayTime(f32),
    DelayFb(f32),
    DelayMix(f32),
    RevRoom(f32),
    RevDamp(f32),
    RevMix(f32),
    /// Global tempo in BPM (20..300) for tempo-synced LFOs and Delay.
    Bpm(f32),
    Lfo1Sync(bool),
    Lfo1Beats(f32),
    Lfo2Sync(bool),
    Lfo2Beats(f32),
    DelaySync(bool),
    DelayBeats(f32),
    /// Voice mode: true = Poly, false = Mono (key-stack + unison).
    PolyMode(bool),
    /// Mono-mode unison stack size (1..6).
    Unison(u8),
    /// Unison pitch spread in cents (0..50).
    UniDetune(f32),
    /// Unison stereo spread 0..1.
    UniSpread(f32),
    /// Arpeggiator on/off — handled on the audio task, not inside the engine.
    ArpOn(bool),
    /// Arp pattern index (0=Up, 1=Down, 2=UpDown, 3=Random).
    ArpMode(u8),
    /// Beats per arp step (0.25 = 1/16 note).
    ArpRate(f32),
    /// Octave span of the arp sequence (1..=4).
    ArpOct(u8),
    /// Arp latch: held notes keep arpeggiating after keys release.
    ArpHold(bool),
}

/// Shared note-division table (label, quarter-note beats) used by both the Web
/// UI and any physical OLED/encoder UI. Includes dotted 1/8 (0.75) and dotted
/// 1/16 (0.375) used by factory presets.
pub const BEAT_DIVISIONS: [(&str, f32); 10] = [
    ("4 Bars", 16.0),
    ("2 Bars", 8.0),
    ("1 Bar", 4.0),
    ("1/2", 2.0),
    ("1/4", 1.0),
    ("1/8D", 0.75),
    ("1/8", 0.5),
    ("1/16D", 0.375),
    ("1/16", 0.25),
    ("1/32", 0.125),
];
