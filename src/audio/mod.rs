//! Audio policies and startup patches; hardware task orchestration lives in the binary.
//!
//! Scheduling, rendering and governor updates allocate nothing. Patch constructors
//! allocate preset strings during startup, before steady-state rendering begins.

pub mod boot_demo;
pub mod governor;
pub mod patches;
pub mod render;
pub mod telemetry;

pub const SAMPLE_RATE: u32 = 44_100;
/// Boot render rate; runtime half-rate mode uses half as many synth frames.
pub const SYNTH_RATE: u32 = 44_100;
pub const FRAMES: usize = 256;
/// Integer microsecond deadline retained from the original audio runtime.
pub const BLOCK_US: i64 = 5805;
pub const MAX_VOICES: usize = 6;
