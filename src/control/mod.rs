//! Transport-neutral command/event delivery and immutable preset catalog.
//!
//! Core 0 adapters may enqueue under a producer mutex. The audio task claims
//! each receiver once and consumes without locking. Preset and clock requests
//! remain latest-value mailboxes, preserving the existing delivery semantics.

mod command;
pub mod dispatch;
pub mod presets;
mod queue;
mod state;

pub use command::{SynthCommand, BEAT_DIVISIONS};
use core::sync::atomic::{AtomicBool, AtomicU32, AtomicUsize, Ordering};
pub use presets::{preset_count, preset_summary};
use queue::{Consumer, Queue};
use rinsynth_model::event::{SynthEvent, SynthEventKind};
pub use state::*;

const CMD_RING_CAP: usize = 256; // power of two
const EVENT_RING_CAP: usize = 128;
static COMMANDS: Queue<SynthCommand, CMD_RING_CAP> = Queue::new(SynthCommand::Nop);
static EVENTS: Queue<SynthEvent, EVENT_RING_CAP> = Queue::new(SynthEvent {
    kind: SynthEventKind::NoteOff,
    note: 0,
    velocity: 0.0,
});
static SEEN: AtomicBool = AtomicBool::new(false);
static EVENT_COUNT: AtomicUsize = AtomicUsize::new(0);
static PENDING_PROG: AtomicUsize = AtomicUsize::new(usize::MAX);
/// Pending BPM estimated from incoming USB-MIDI Timing Clock (0xF8, 24 PPQN).
/// `u32::MAX` means no new clock update since last audio-task poll.
static PENDING_CLOCK_BPM: AtomicU32 = AtomicU32::new(u32::MAX);

pub type CommandReceiver = Consumer<'static, SynthCommand, CMD_RING_CAP>;
pub type EventReceiver = Consumer<'static, SynthEvent, EVENT_RING_CAP>;

pub fn command_receiver() -> Option<CommandReceiver> {
    COMMANDS.take_consumer()
}
pub fn event_receiver() -> Option<EventReceiver> {
    EVENTS.take_consumer()
}

/// Enqueue from a non-real-time producer. A full queue rejects the new command.
pub fn push_cmd(cmd: SynthCommand) {
    COMMANDS.push(cmd);
}

/// Enqueue a note/control event and mark live input, cancelling the boot demo.
/// Overflow still drops new events; NoteOff recovery is a separate policy change.
pub fn push_event(ev: SynthEvent) {
    SEEN.store(true, Ordering::Relaxed);
    EVENT_COUNT.fetch_add(1, Ordering::Relaxed);
    EVENTS.push(ev);
}

/// USB traffic cancels the demo on NoteOn only; browser input cancels on any event.
pub fn push_midi_event(ev: SynthEvent) {
    if matches!(ev.kind, SynthEventKind::NoteOn) {
        SEEN.store(true, Ordering::Relaxed);
    }
    EVENT_COUNT.fetch_add(1, Ordering::Relaxed);
    EVENTS.push(ev);
}

pub fn record_program_change(prog: u8) {
    set_program_change(prog);
    EVENT_COUNT.fetch_add(1, Ordering::Relaxed);
}

pub fn midi_seen() -> bool {
    SEEN.load(Ordering::Relaxed)
}
pub fn event_count() -> usize {
    EVENT_COUNT.load(Ordering::Relaxed)
}
pub fn dropped_commands() -> usize {
    COMMANDS.dropped()
}
pub fn dropped_events() -> usize {
    EVENTS.dropped()
}

/// Latest request wins; selecting a preset alone does not cancel the boot demo.
pub fn set_program_change(prog: u8) {
    PENDING_PROG.store(prog as usize, Ordering::Release);
}
pub fn take_program_change() -> Option<u8> {
    let value = PENDING_PROG.swap(usize::MAX, Ordering::AcqRel);
    (value != usize::MAX).then_some(value as u8)
}

pub fn set_clock_bpm(bpm: f32) {
    PENDING_CLOCK_BPM.store(bpm.to_bits(), Ordering::Release);
}
pub fn take_clock_bpm() -> Option<f32> {
    let bits = PENDING_CLOCK_BPM.swap(u32::MAX, Ordering::AcqRel);
    (bits != u32::MAX).then_some(f32::from_bits(bits))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn transport_mailboxes_and_demo_cancellation_keep_existing_semantics() {
        let mut receiver = event_receiver().unwrap();
        set_program_change(2);
        record_program_change(7);
        assert_eq!(take_program_change(), Some(7));
        assert_eq!(take_program_change(), None);
        assert_eq!(event_count(), 1);
        assert!(!midi_seen());
        push_midi_event(SynthEvent::note_off(60));
        assert!(!midi_seen());
        push_event(SynthEvent::note_off(61));
        assert!(midi_seen());
        assert_eq!(event_count(), 3);
        assert_eq!(receiver.pop().unwrap().note, 60);
        assert_eq!(receiver.pop().unwrap().note, 61);
        assert!(receiver.pop().is_none());
        set_clock_bpm(120.0);
        set_clock_bpm(132.5);
        assert_eq!(take_clock_bpm(), Some(132.5));
        assert_eq!(take_clock_bpm(), None);
    }
}
