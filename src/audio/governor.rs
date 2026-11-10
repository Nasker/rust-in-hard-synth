//! Pure polyphony budget policy, evaluated once per rendered block.
//!
//! Timing is DSP render time in microseconds, not total block production time.
//! The caller applies cap changes and performs musical voice shedding.

use super::{BLOCK_US, MAX_VOICES};

/// Blocks (~5.8 ms each) before overload may reduce polyphony during startup.
pub const WARMUP_BLOCKS: u32 = 1_720;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Decision {
    Unchanged,
    Shed,
    Restored { worst_us: i64 },
}

/// Stateful overload/recovery policy. No clocks, locks, allocations or engine access.
pub struct Governor {
    cap: usize,
    late_streak: u32,
    late_hist: u64,
    cool: u32,
    cool_max: i64,
}

impl Default for Governor {
    fn default() -> Self {
        Self {
            cap: MAX_VOICES,
            late_streak: 0,
            late_hist: 0,
            cool: 0,
            cool_max: 0,
        }
    }
}

impl Governor {
    pub fn cap(&self) -> usize {
        self.cap
    }

    /// Clamp to the user ceiling, then update the rolling overload window.
    /// `block` is the zero-based block index, and `ceiling` is constrained to 1..=6.
    /// `Shed` may repeat at cap 1 so the caller can finish shedding excess voices.
    pub fn observe(&mut self, block: u32, dt: i64, ceiling: usize) -> Decision {
        let ceiling = ceiling.clamp(1, MAX_VOICES);
        self.cap = self.cap.min(ceiling);
        let late = dt > BLOCK_US;
        self.late_hist = (self.late_hist << 1) | late as u64;
        if late {
            self.late_streak += 1;
            self.cool = self.cool.saturating_sub(16);
            if block >= WARMUP_BLOCKS
                && (self.late_streak >= 12 || self.late_hist.count_ones() >= 16)
            {
                self.cap = self.cap.saturating_sub(1).max(1);
                self.late_streak = 0;
                self.late_hist = 0;
                return Decision::Shed;
            }
        } else {
            self.late_streak = 0;
            if self.cap < ceiling {
                self.cool += 1;
                self.cool_max = self.cool_max.max(dt);
                if self.cool >= 344 {
                    let worst_us = self.cool_max;
                    let restore = worst_us < BLOCK_US * 3 / 4 && self.late_hist.count_ones() < 4;
                    self.cool = 0;
                    self.cool_max = 0;
                    if restore {
                        self.cap += 1;
                        return Decision::Restored { worst_us };
                    }
                }
            } else {
                self.cool_max = 0;
            }
        }
        Decision::Unchanged
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn warmup_preserves_cap_despite_sustained_lates() {
        let mut g = Governor::default();
        for block in 0..WARMUP_BLOCKS {
            assert_eq!(g.observe(block, BLOCK_US + 1, 6), Decision::Unchanged);
        }
        assert_eq!(g.cap(), 6);
        assert_eq!(g.observe(WARMUP_BLOCKS, BLOCK_US + 1, 6), Decision::Shed);
        assert_eq!(g.cap(), 5);
    }

    #[test]
    fn twelve_consecutive_lates_shed_but_transients_do_not() {
        let mut g = Governor::default();
        for i in 0..11 {
            assert_eq!(
                g.observe(WARMUP_BLOCKS + i, BLOCK_US + 1, 6),
                Decision::Unchanged
            );
        }
        assert_eq!(
            g.observe(WARMUP_BLOCKS + 11, BLOCK_US + 1, 6),
            Decision::Shed
        );
        assert_eq!(g.cap(), 5);
    }

    #[test]
    fn rolling_rate_sheds_without_a_consecutive_streak() {
        let mut g = Governor::default();
        for i in 0..60 {
            assert_eq!(
                g.observe(WARMUP_BLOCKS + i, if i % 4 == 0 { 6000 } else { 1000 }, 6),
                Decision::Unchanged
            );
        }
        assert_eq!(g.observe(WARMUP_BLOCKS + 60, 6000, 6), Decision::Shed);
    }

    #[test]
    fn recovery_requires_344_clean_blocks_and_strict_margin() {
        let mut g = Governor::default();
        g.observe(0, 1000, 3);
        for i in 0..343 {
            g.observe(i + 1, 1000, 6);
        }
        assert_eq!(g.cap(), 3);
        assert_eq!(
            g.observe(344, 1000, 6),
            Decision::Restored { worst_us: 1000 }
        );
        assert_eq!(g.cap(), 4);
        for i in 0..344 {
            g.observe(i + 345, BLOCK_US * 3 / 4, 6);
        }
        assert_eq!(g.cap(), 4);
    }

    #[test]
    fn deadline_is_inclusive_and_cap_has_a_floor() {
        let mut g = Governor::default();
        for i in 0..100 {
            g.observe(WARMUP_BLOCKS + i, BLOCK_US, 6);
        }
        assert_eq!(g.cap(), 6);
        for i in 0..100 {
            g.observe(WARMUP_BLOCKS + i, BLOCK_US + 1, 1);
        }
        assert_eq!(g.cap(), 1);
    }
}
