//! Independent atomic synth readouts published by the audio task.
//!
//! Relaxed loads are intentional: fields are observational and do not publish
//! other memory. They are not a coherent multi-field snapshot. Preset metadata
//! is immutable and initialized separately before audio startup.

use core::sync::atomic::{AtomicBool, AtomicU32, AtomicUsize, Ordering};

/// UI-configured polyphony ceiling — the audio-loop governor reads this every
/// block and clamps its adaptive cap to it.
static USER_VOICE_CAP: AtomicUsize = AtomicUsize::new(5);
/// Live active-voice count and demo-riff completion flag updated by Core 1
/// for lock-free readout by the Core 0 OLED UI task.
static ACTIVE_VOICES: AtomicUsize = AtomicUsize::new(0);
static RIFF_DONE: AtomicBool = AtomicBool::new(false);
/// Live arpeggiator engaged state (written by the audio task, read by UI).
static ARP_ENABLED: AtomicBool = AtomicBool::new(false);
/// Runtime rate toggle — the render loop reads this every block to choose
/// between 256 direct frames (44.1 kHz) and 128 frames + Catmull-Rom 2x
/// upsample (22.05 kHz).
static SYNTH_HALF: AtomicBool = AtomicBool::new(false);
/// Live BPM (f32 bits) and active preset index for lock-free readout by
/// Web/OLED UI tasks on Core 0 (`usize::MAX` = post-riff Init+ patch).
static CURRENT_BPM_BITS: AtomicU32 = AtomicU32::new(0x42F0_0000); // 120.0f32
static CURRENT_PRESET_IDX: AtomicUsize = AtomicUsize::new(usize::MAX);

pub fn active_voices() -> usize {
    ACTIVE_VOICES.load(Ordering::Relaxed)
}
pub fn set_active_voices(n: usize) {
    ACTIVE_VOICES.store(n, Ordering::Relaxed);
}
pub fn riff_done() -> bool {
    RIFF_DONE.load(Ordering::Relaxed)
}
pub fn set_riff_done(done: bool) {
    RIFF_DONE.store(done, Ordering::Relaxed);
}
pub fn arp_enabled() -> bool {
    ARP_ENABLED.load(Ordering::Relaxed)
}
pub fn set_arp_enabled(on: bool) {
    ARP_ENABLED.store(on, Ordering::Relaxed);
}
pub fn user_voice_cap() -> usize {
    USER_VOICE_CAP.load(Ordering::Relaxed).clamp(1, 6)
}
pub(crate) fn set_user_voice_cap(n: u8) {
    USER_VOICE_CAP.store(n as usize, Ordering::Relaxed);
}
pub fn synth_half() -> bool {
    SYNTH_HALF.load(Ordering::Relaxed)
}
pub(crate) fn set_synth_half(on: bool) {
    SYNTH_HALF.store(on, Ordering::Relaxed);
}
pub fn current_bpm() -> f32 {
    f32::from_bits(CURRENT_BPM_BITS.load(Ordering::Relaxed))
}
pub fn set_current_bpm(bpm: f32) {
    CURRENT_BPM_BITS.store(bpm.clamp(20.0, 300.0).to_bits(), Ordering::Relaxed);
}
pub fn current_preset_index() -> usize {
    CURRENT_PRESET_IDX.load(Ordering::Relaxed)
}
pub fn set_current_preset_index(idx: usize) {
    CURRENT_PRESET_IDX.store(idx, Ordering::Relaxed);
}
