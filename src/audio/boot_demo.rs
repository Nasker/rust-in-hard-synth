//! One-shot boot showcase: fast D harmonic-minor sweep, then strummed i–V–iv.
//!
//! Scheduling is in output blocks (256 frames), independent of synth rate.
//! Cancellation releases all demo pitches and discards pending strum notes.

use rinsynth_model::event::NoteEvent;

const STRIKES: &[(&[u8], u32)] = &[
    (&[62], 18),
    (&[64], 18),
    (&[65], 18),
    (&[67], 18),
    (&[69], 18),
    (&[70], 18),
    (&[73], 18),
    (&[74], 18),
    (&[73], 18),
    (&[70], 18),
    (&[69], 18),
    (&[50, 65, 69], 64),
    (&[57, 61, 64], 64),
    (&[55, 58, 62], 150),
];
const SEQ: &[(Option<usize>, u32)] = &[
    (Some(0), 20),
    (Some(1), 20),
    (Some(2), 20),
    (Some(3), 20),
    (Some(4), 20),
    (Some(5), 20),
    (Some(6), 20),
    (Some(7), 20),
    (Some(8), 20),
    (Some(9), 20),
    (Some(10), 20),
    (None, 40),
    (Some(11), 40),
    (None, 40),
    (None, 40),
    (Some(12), 40),
    (None, 40),
    (None, 40),
    (Some(13), 40),
    (None, 80),
    (None, 40),
];
pub const TOTAL_BLOCKS: u32 = 660;
const STRUM_BLOCKS: u32 = 6;
const VELOCITY: u8 = 110;

pub struct BootDemo {
    active: bool,
    last_stab: Option<(usize, u32)>,
    seq_step: usize,
    next_step_at: u32,
    strum_notes: [u8; 4],
    strum_len: usize,
    strum_idx: usize,
    strum_next_at: u32,
}

impl Default for BootDemo {
    fn default() -> Self {
        Self {
            active: true,
            last_stab: None,
            seq_step: 0,
            next_step_at: 0,
            strum_notes: [0; 4],
            strum_len: 0,
            strum_idx: 0,
            strum_next_at: 0,
        }
    }
}

impl BootDemo {
    /// The caller performs any post-demo preset transition before calling `cancel`.
    pub fn should_finish(&self, block: u32, midi_seen: bool) -> bool {
        self.active && (midi_seen || block >= TOTAL_BLOCKS)
    }

    pub fn cancel(&mut self, mut emit: impl FnMut(NoteEvent)) {
        if self.active {
            for &(notes, _) in STRIKES {
                for &note in notes {
                    emit(NoteEvent::note_off(note));
                }
            }
        }
        self.active = false;
        self.last_stab = None;
        self.strum_idx = self.strum_len;
    }

    /// Emit scheduled strikes, strum tones and timed releases in their original order.
    /// Call once per sequential block, after checking `should_finish`.
    pub fn tick(&mut self, block: u32, mut emit: impl FnMut(NoteEvent)) {
        if self.active && block == self.next_step_at {
            let (strike, duration) = SEQ[self.seq_step];
            self.seq_step = (self.seq_step + 1) % SEQ.len();
            self.next_step_at += duration;
            if let Some(strike) = strike {
                let (notes, _) = STRIKES[strike];
                let len = notes.len().min(self.strum_notes.len());
                self.strum_notes[..len].copy_from_slice(&notes[..len]);
                self.strum_len = len;
                self.strum_idx = 0;
                self.strum_next_at = block;
                self.last_stab = Some((strike, block));
            }
        }
        if self.active && self.strum_idx < self.strum_len && block >= self.strum_next_at {
            emit(NoteEvent::note_on_midi(
                self.strum_notes[self.strum_idx],
                VELOCITY,
            ));
            self.strum_idx += 1;
            self.strum_next_at += STRUM_BLOCKS;
        }
        if let Some((strike, struck)) = self.last_stab {
            if block.wrapping_sub(struck) == STRIKES[strike].1 {
                for &note in STRIKES[strike].0 {
                    emit(NoteEvent::note_off(note));
                }
                self.last_stab = None;
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use rinsynth_model::event::SynthEventKind;

    #[test]
    fn timeline_is_the_existing_three_chord_showcase() {
        assert_eq!(SEQ.iter().map(|entry| entry.1).sum::<u32>(), TOTAL_BLOCKS);
        let mut demo = BootDemo::default();
        let mut ons = Vec::new();
        let mut offs = Vec::new();
        for block in 0..TOTAL_BLOCKS {
            demo.tick(block, |ev| match ev.kind {
                SynthEventKind::NoteOn => ons.push((block, ev.note)),
                SynthEventKind::NoteOff => offs.push((block, ev.note)),
                _ => unreachable!(),
            });
        }
        assert_eq!(
            &ons[..11],
            &[
                (0, 62),
                (20, 64),
                (40, 65),
                (60, 67),
                (80, 69),
                (100, 70),
                (120, 73),
                (140, 74),
                (160, 73),
                (180, 70),
                (200, 69)
            ]
        );
        assert_eq!(
            &ons[11..],
            &[
                (260, 50),
                (266, 65),
                (272, 69),
                (380, 57),
                (386, 61),
                (392, 64),
                (500, 55),
                (506, 58),
                (512, 62)
            ]
        );
        assert!(offs.contains(&(18, 62)));
        assert!(offs.contains(&(650, 55)));
        assert!(demo.should_finish(TOTAL_BLOCKS, false));
    }

    #[test]
    fn cancellation_discards_pending_strum_and_is_idempotent() {
        let mut demo = BootDemo::default();
        for block in 0..=260 {
            demo.tick(block, |_| {});
        }
        assert!(demo.should_finish(261, true));
        let mut releases = Vec::new();
        demo.cancel(|ev| releases.push(ev.note));
        assert_eq!(releases.len(), 20);
        for block in 261..700 {
            demo.tick(block, |_| panic!("demo emitted after cancellation"));
        }
        demo.cancel(|_| panic!("duplicate cancellation"));
        assert!(!demo.should_finish(700, true));
    }
}
