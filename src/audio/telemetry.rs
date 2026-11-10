//! Audio accounting and diagnostic log formatting, outside sample generation.
//!
//! Render maxima, late counts and peak reset every 172 blocks (~1 second).
//! Write failures remain cumulative, matching the original telemetry contract.

use super::BLOCK_US;
use log::info;

#[derive(Default)]
pub struct Telemetry {
    pub blocks: u32,
    pub peak: f32,
    write_errors: u32,
    overruns: u32,
    max_render_us: i64,
}

impl Telemetry {
    pub fn write_failed(&mut self) {
        self.write_errors += 1;
    }

    /// Called after the I2S write; logging remains on the audio task for now.
    /// `render_us` excludes interpolation, command dispatch and I2S waits.
    pub fn finish_block(&mut self, render_us: i64, voices: usize, cap: usize, events: usize) {
        self.max_render_us = self.max_render_us.max(render_us);
        if render_us > BLOCK_US {
            self.overruns += 1;
        }
        if self.blocks < 20 {
            info!(
                "block {} | voices: {} | render: {}us",
                self.blocks, voices, render_us
            );
        }
        self.blocks += 1;
        if self.blocks.is_multiple_of(172) {
            info!(
                "blocks: {} | voices: {}/{} | midi: {} | render max: {}us/{}us | late: {} | peak: {:.2} | write errors: {}",
                self.blocks, voices, cap, events, self.max_render_us, BLOCK_US,
                self.overruns, self.peak, self.write_errors
            );
            // Per-stage cycle breakdown (cycles per rendered sample, this second)
            let snap = rinsynth_dsp::profile::snapshot();
            if snap[rinsynth_dsp::profile::SAMPLES] > 0 {
                let n = snap[rinsynth_dsp::profile::SAMPLES];
                info!(
                    "profile cyc/sample: env={} ctrl={} osc={} filt={} fx={} (n={})",
                    snap[rinsynth_dsp::profile::ENV] / n,
                    snap[rinsynth_dsp::profile::CONTROL] / n,
                    snap[rinsynth_dsp::profile::OSC] / n,
                    snap[rinsynth_dsp::profile::FILTER] / n,
                    snap[rinsynth_dsp::profile::FX] / n,
                    n
                );
            }
            rinsynth_dsp::profile::reset();
            self.max_render_us = 0;
            self.overruns = 0;
            self.peak = 0.0;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn window_resets_but_write_errors_remain_cumulative() {
        let mut t = Telemetry::default();
        t.write_failed();
        t.peak = 0.75;
        for _ in 0..171 {
            t.finish_block(BLOCK_US + 1, 3, 5, 10);
        }
        assert_eq!(t.overruns, 171);
        assert_eq!(t.max_render_us, BLOCK_US + 1);
        t.finish_block(1000, 3, 5, 10);
        assert_eq!(
            (t.blocks, t.overruns, t.max_render_us, t.write_errors),
            (172, 0, 0, 1)
        );
        assert_eq!(t.peak, 0.0);
    }
}
