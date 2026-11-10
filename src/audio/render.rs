//! PCM conversion and the optional half-rate interpolation path.
//!
//! Buffers are inline and reused. Generation and interpolation remain separate
//! so the existing DSP-only timing measurement keeps its original scope.

use super::FRAMES;
use rinsynth_dsp::embedded::{soft_clip, EmbeddedEngine};
use rinsynth_model::types::StereoSample;

pub struct BlockRenderer {
    pub pcm: [i16; FRAMES * 2],
    synth: [StereoSample; FRAMES],
    hist: [StereoSample; 4],
}

impl Default for BlockRenderer {
    fn default() -> Self {
        Self {
            pcm: [0; FRAMES * 2],
            synth: [StereoSample::ZERO; FRAMES],
            hist: [StereoSample::ZERO; 4],
        }
    }
}

impl BlockRenderer {
    /// Generate 256 full-rate or 128 half-rate samples; update the pre-conversion peak.
    /// Half-rate samples are staged until `interpolate` is called.
    pub fn generate(&mut self, vm: &mut EmbeddedEngine, half: bool, peak: &mut f32) {
        let frames = if half { FRAMES / 2 } else { FRAMES };
        for i in 0..frames {
            let s = vm.next_sample_stereo();
            if s.left.abs() > *peak {
                *peak = s.left.abs();
            }
            if s.right.abs() > *peak {
                *peak = s.right.abs();
            }
            if half {
                self.synth[i] = s;
            } else {
                self.pcm[2 * i] = (s.left * 32767.0) as i16;
                self.pcm[2 * i + 1] = (s.right * 32767.0) as i16;
            }
        }
    }

    /// 2x Catmull-Rom midpoint interpolation with persistent four-sample history.
    /// Even outputs pass through with one synth-frame delay; odd outputs are
    /// soft-clipped midpoints. History is retained across rate toggles as before.
    pub fn interpolate(&mut self) {
        for i in 0..FRAMES / 2 {
            self.hist[0] = self.hist[1];
            self.hist[1] = self.hist[2];
            self.hist[2] = self.hist[3];
            self.hist[3] = self.synth[i];
            let mid_l = (9.0 * (self.hist[1].left + self.hist[2].left)
                - self.hist[0].left
                - self.hist[3].left)
                * 0.0625;
            let mid_r = (9.0 * (self.hist[1].right + self.hist[2].right)
                - self.hist[0].right
                - self.hist[3].right)
                * 0.0625;
            self.pcm[4 * i] = (self.hist[2].left * 32767.0) as i16;
            self.pcm[4 * i + 1] = (self.hist[2].right * 32767.0) as i16;
            self.pcm[4 * i + 2] = (soft_clip(mid_l) * 32767.0) as i16;
            self.pcm[4 * i + 3] = (soft_clip(mid_r) * 32767.0) as i16;
        }
    }

    /// Native-endian PCM bytes for the little-endian ESP32 I2S driver.
    pub fn bytes(&self) -> &[u8] {
        // SAFETY: i16 storage is initialized and contiguous; u8 has alignment 1.
        // The returned immutable slice cannot outlive the borrowed PCM buffer.
        unsafe {
            core::slice::from_raw_parts(self.pcm.as_ptr().cast(), core::mem::size_of_val(&self.pcm))
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use rinsynth_model::event::SynthEvent;

    #[test]
    fn full_rate_matches_the_original_sample_conversion() {
        let mut expected = EmbeddedEngine::new(6, 44100);
        let mut actual = EmbeddedEngine::new(6, 44100);
        expected.receive_event(SynthEvent::note_on_midi(62, 110));
        actual.receive_event(SynthEvent::note_on_midi(62, 110));
        let mut renderer = BlockRenderer::default();
        let mut peak = 0.0;
        renderer.generate(&mut actual, false, &mut peak);
        for frame in renderer.pcm.chunks_exact(2) {
            let s = expected.next_sample_stereo();
            assert_eq!(
                frame,
                &[(s.left * 32767.0) as i16, (s.right * 32767.0) as i16]
            );
        }
        assert_eq!(renderer.bytes().len(), FRAMES * 4);
        assert!(peak > 0.0);
    }

    #[test]
    fn interpolation_retains_history_across_blocks() {
        let mut renderer = BlockRenderer::default();
        renderer.synth.fill(StereoSample {
            left: 0.25,
            right: -0.25,
        });
        renderer.interpolate();
        assert_eq!(&renderer.pcm[..2], &[0, 0]);
        assert_eq!(&renderer.pcm[4..6], &[8191, -8191]);
        renderer.interpolate();
        assert_eq!(&renderer.pcm[..2], &[8191, -8191]);
        assert_eq!(renderer.pcm[2], (soft_clip(0.25) * 32767.0) as i16);
    }
}
