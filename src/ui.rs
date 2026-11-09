//! Physical UI state machine, parameter page tables, and screen renderer.
//!
//! Strict SRP separation:
//! - [`SystemSnapshot`]: read-only view of synth/network/MIDI health gathered
//!   via lock-free atomics from `status`, `web`, and `usb_midi`.
//! - [`UiState`]: pure state machine (`Splash` -> `Home` <-> `Menu` <-> `Section`)
//!   driven by [`InputFrame`] events and emitting [`WebCmd`]s via a sink closure.
//! - [`render`]: pure view function `(UiState, SystemSnapshot) -> Canvas`.
//! - [`init`]: wires SPI OLED (`GPIO 9..13`), FX kill buttons (`GPIO 41/42/2/1`),
//!   and PCNT encoder (`GPIO 38..40`) into the Core 0 UI task (~25 FPS).

use crate::display::{Canvas, Display, Sh1106Spi, HEIGHT, WIDTH};
use crate::input::{InputFrame, InputSource, PhysicalInputs, POTS_ENABLED};
use crate::status::{self, Status};
use crate::usb_midi;
use crate::web::{self, WebCmd};
use esp_idf_hal::delay::FreeRtos;
use esp_idf_hal::gpio::{Gpio1, Gpio2, Gpio38, Gpio39, Gpio40, Gpio41, Gpio42, OutputPin};
use esp_idf_hal::spi::SpiAnyPins;
use log::{info, warn};
use rinsynth_model::params::SynthParam;

const FPS_DELAY_MS: u32 = 40; // 25 FPS
const SPLASH_MIN_FRAMES: u32 = 135; // ~5.4 s presentation animation

// ---------------------------------------------------------------------------
// Declarative Section & Parameter Page Tables (SRP: data, not code)
// ---------------------------------------------------------------------------

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
    /// Convert a normalized `0..=100` UI value into a [`WebCmd`] for the audio engine.
    pub fn to_cmd(self, pct: u8) -> WebCmd {
        let norm = (pct.min(100) as f32) * 0.01;
        // Exponential 0.005..2.0 s envelope time mapping
        let env_t = |n: f32| 0.005 + n * n * 1.995;
        match self {
            Self::Cutoff => {
                // Logarithmic 20 Hz .. 20 kHz
                let hz = 20.0 * (1000.0f32).powf(norm);
                WebCmd::Param(SynthParam::FilterCutoff, hz)
            }
            Self::Resonance => WebCmd::Param(SynthParam::FilterResonance, norm * 0.95),
            Self::FilterEnvAmt => {
                WebCmd::Param(SynthParam::FilterEnvAmount, norm * 2.0 - 1.0)
            }
            Self::FilterAttack => WebCmd::Param(SynthParam::FilterAttack, env_t(norm)),
            Self::FilterDecay => WebCmd::Param(SynthParam::FilterDecay, env_t(norm)),
            Self::FilterSustain => WebCmd::Param(SynthParam::FilterSustain, norm),
            Self::FilterRelease => WebCmd::Param(SynthParam::FilterRelease, env_t(norm)),
            Self::AmpAttack => WebCmd::Param(SynthParam::Attack, env_t(norm)),
            Self::AmpDecay => WebCmd::Param(SynthParam::Decay, env_t(norm)),
            Self::AmpSustain => WebCmd::Param(SynthParam::Sustain, norm),
            Self::AmpRelease => WebCmd::Param(SynthParam::Release, env_t(norm)),
            Self::Env3Attack => WebCmd::Env3Attack(env_t(norm)),
            Self::Env3Decay => WebCmd::Env3Decay(env_t(norm)),
            Self::Env3Sustain => WebCmd::Env3Sustain(norm),
            Self::Env3Release => WebCmd::Env3Release(env_t(norm)),
            Self::Osc1Wave => WebCmd::Param(SynthParam::Osc1Waveform, self.index(pct) as f32),
            Self::Osc1Level => WebCmd::Param(SynthParam::Osc1Level, norm),
            Self::Osc2Wave => WebCmd::Param(SynthParam::Osc2Waveform, self.index(pct) as f32),
            Self::Osc2Level => WebCmd::Param(SynthParam::Osc2Level, norm),
            Self::Osc2Semi => {
                let semi = ((norm * 48.0).round() - 24.0).clamp(-24.0, 24.0);
                WebCmd::Param(SynthParam::Osc2Semitones, semi)
            }
            Self::Osc2Detune => {
                let cents = ((norm * 100.0).round() - 50.0).clamp(-50.0, 50.0);
                WebCmd::Param(SynthParam::Osc2Cents, cents)
            }
            Self::Osc3Wave => WebCmd::Param(SynthParam::Osc3Waveform, self.index(pct) as f32),
            Self::Osc3Level => WebCmd::Param(SynthParam::Osc3Level, norm),
            Self::Osc4Wave => WebCmd::Param(SynthParam::Osc4Waveform, self.index(pct) as f32),
            Self::Osc4Level => WebCmd::Param(SynthParam::Osc4Level, norm),
            Self::Lfo1Rate => WebCmd::Param(SynthParam::LfoRate, 0.1 + norm * norm * 19.9),
            Self::Lfo1Depth => WebCmd::Param(SynthParam::LfoDepth, norm),
            Self::Lfo2Rate => WebCmd::Lfo2Rate(0.1 + norm * norm * 19.9),
            Self::Lfo2Depth => WebCmd::Lfo2Depth(norm),
            Self::Glide => WebCmd::Param(SynthParam::PortamentoTime, norm * norm * 0.5),
            Self::StereoWidth => WebCmd::Param(SynthParam::StereoWidth, norm * 2.0),
            Self::MasterVol => WebCmd::Param(SynthParam::MasterVolume, norm),
            Self::DistDrive => {
                web::push_cmd(WebCmd::DistEnable(pct > 2));
                WebCmd::DistDrive(norm)
            }
            Self::DistMix => WebCmd::DistMix(norm),
            Self::ChorusRate => WebCmd::ChorusRate(0.1 + norm * 4.9),
            Self::ChorusDepth => WebCmd::ChorusDepth(norm * 10.0),
            Self::ChorusMix => WebCmd::ChorusMix(norm),
            Self::DelayTime => WebCmd::DelayTime(0.02 + norm * 1.18),
            Self::DelayFb => WebCmd::DelayFb(norm * 0.90),
            Self::DelayMix => WebCmd::DelayMix(norm),
            Self::RevRoom => WebCmd::RevRoom(norm),
            Self::RevDamp => WebCmd::RevDamp(norm),
            Self::RevMix => WebCmd::RevMix(norm),
            Self::PolyMode => WebCmd::PolyMode(self.index(pct) == 1),
            Self::UnisonVoices => WebCmd::Unison(1 + self.index(pct) as u8),
            Self::UnisonDetune => WebCmd::UniDetune(norm * 50.0),
            Self::UnisonSpread => WebCmd::UniSpread(norm),
            Self::ArpOn => WebCmd::ArpOn(self.index(pct) == 1),
            Self::ArpHold => WebCmd::ArpHold(self.index(pct) == 1),
            Self::ArpMode => WebCmd::ArpMode(self.index(pct) as u8),
            Self::ArpRate => {
                // Stepped beat divisions, slow -> fast: 1/2 .. 1/32.
                const DIVS: [f32; 7] = [2.0, 1.0, 0.75, 0.5, 0.375, 0.25, 0.125];
                WebCmd::ArpRate(DIVS[self.index(pct)])
            }
            Self::ArpOct => WebCmd::ArpOct(1 + self.index(pct) as u8),
        }
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
            Self::ArpRate => {
                Some(["1/2", "1/4", "1/8D", "1/8", "16D", "1/16", "1/32"][i])
            }
            Self::ArpOct => Some(["1", "2", "3", "4"][i]),
            Self::UnisonVoices => Some(["1", "2", "3", "4", "5", "6"][i]),
            Self::Osc1Wave | Self::Osc2Wave | Self::Osc3Wave | Self::Osc4Wave => {
                Some(["SIN", "SQR", "SAW", "TRI", "NSE"][i])
            }
            _ => None,
        }
    }
}

#[derive(Clone, Copy)]
pub struct PageSlot {
    pub label: &'static str,
    pub target: ParamTarget,
    pub default_pct: u8,
}

#[derive(Clone, Copy)]
pub struct SectionDef {
    pub name: &'static str,
    pub icon: [u8; 8],
    /// Parameter pages; each page holds up to 4 column slots aligned
    /// under the top pots (shorter pages leave their columns empty).
    pub pages: &'static [&'static [PageSlot]],
}

const ICON_OSC: [u8; 8] = [
    0b00011000,
    0b00100100,
    0b01000010,
    0b10000001,
    0b10000001,
    0b01000010,
    0b00100100,
    0b00011000,
];

const ICON_FILT: [u8; 8] = [
    0b11110000,
    0b00001000,
    0b00000100,
    0b00000100,
    0b00000010,
    0b00000010,
    0b00000001,
    0b00000001,
];

const ICON_ENV: [u8; 8] = [
    0b00010000,
    0b00101000,
    0b01000100,
    0b01000011,
    0b10000001,
    0b10000001,
    0b11111111,
    0b00000000,
];

const ICON_LFO: [u8; 8] = [
    0b00110000,
    0b01001000,
    0b10000100,
    0b00000101,
    0b00000010,
    0b00110000,
    0b01001001,
    0b00000110,
];

const ICON_FX: [u8; 8] = [
    0b10011001,
    0b01000010,
    0b00111100,
    0b11100111,
    0b11100111,
    0b00111100,
    0b01000010,
    0b10011001,
];

const ICON_VOICE: [u8; 8] = [
    0b00011000,
    0b00111100,
    0b00011000,
    0b01111110,
    0b11011011,
    0b00011000,
    0b00100100,
    0b01000010,
];

const ICON_ARP: [u8; 8] = [
    0b00000010,
    0b00000110,
    0b00001110,
    0b00011010,
    0b00110010,
    0b01100010,
    0b11000010,
    0b00000000,
];

pub const SECTIONS: [SectionDef; 7] = [
    SectionDef {
        name: "OSC",
        icon: ICON_OSC,
        pages: &[
            &[
                PageSlot { label: "O1WF", target: ParamTarget::Osc1Wave, default_pct: 25 },
                PageSlot { label: "O1LV", target: ParamTarget::Osc1Level, default_pct: 85 },
                PageSlot { label: "O2WF", target: ParamTarget::Osc2Wave, default_pct: 25 },
                PageSlot { label: "O2LV", target: ParamTarget::Osc2Level, default_pct: 65 },
            ],
            &[
                PageSlot { label: "O2SM", target: ParamTarget::Osc2Semi, default_pct: 50 },
                PageSlot { label: "O2DT", target: ParamTarget::Osc2Detune, default_pct: 55 },
                PageSlot { label: "O3LV", target: ParamTarget::Osc3Level, default_pct: 45 },
                PageSlot { label: "O4LV", target: ParamTarget::Osc4Level, default_pct: 0 },
            ],
        ],
    },
    SectionDef {
        name: "FILT",
        icon: ICON_FILT,
        pages: &[
            &[
                PageSlot { label: "CUT", target: ParamTarget::Cutoff, default_pct: 68 },
                PageSlot { label: "RES", target: ParamTarget::Resonance, default_pct: 45 },
                PageSlot { label: "FENV", target: ParamTarget::FilterEnvAmt, default_pct: 70 },
                PageSlot { label: "FDEC", target: ParamTarget::FilterDecay, default_pct: 40 },
            ],
            &[
                PageSlot { label: "FATK", target: ParamTarget::FilterAttack, default_pct: 10 },
                PageSlot { label: "FDEC", target: ParamTarget::FilterDecay, default_pct: 40 },
                PageSlot { label: "FSUS", target: ParamTarget::FilterSustain, default_pct: 30 },
                PageSlot { label: "FREL", target: ParamTarget::FilterRelease, default_pct: 40 },
            ],
        ],
    },
    SectionDef {
        name: "ENV",
        icon: ICON_ENV,
        pages: &[
            &[
                PageSlot { label: "AATK", target: ParamTarget::AmpAttack, default_pct: 8 },
                PageSlot { label: "ADEC", target: ParamTarget::AmpDecay, default_pct: 35 },
                PageSlot { label: "ASUS", target: ParamTarget::AmpSustain, default_pct: 80 },
                PageSlot { label: "AREL", target: ParamTarget::AmpRelease, default_pct: 35 },
            ],
            &[
                PageSlot { label: "E3AT", target: ParamTarget::Env3Attack, default_pct: 10 },
                PageSlot { label: "E3DC", target: ParamTarget::Env3Decay, default_pct: 40 },
                PageSlot { label: "E3SU", target: ParamTarget::Env3Sustain, default_pct: 50 },
                PageSlot { label: "E3RL", target: ParamTarget::Env3Release, default_pct: 40 },
            ],
        ],
    },
    SectionDef {
        name: "LFO",
        icon: ICON_LFO,
        pages: &[
            &[
                PageSlot { label: "L1RT", target: ParamTarget::Lfo1Rate, default_pct: 25 },
                PageSlot { label: "L1DP", target: ParamTarget::Lfo1Depth, default_pct: 20 },
                PageSlot { label: "L2RT", target: ParamTarget::Lfo2Rate, default_pct: 50 },
                PageSlot { label: "L2DP", target: ParamTarget::Lfo2Depth, default_pct: 0 },
            ],
        ],
    },
    SectionDef {
        name: "FX",
        icon: ICON_FX,
        pages: &[
            &[
                PageSlot { label: "DRV", target: ParamTarget::DistDrive, default_pct: 0 },
                PageSlot { label: "DMIX", target: ParamTarget::DistMix, default_pct: 50 },
                PageSlot { label: "CHMX", target: ParamTarget::ChorusMix, default_pct: 25 },
                PageSlot { label: "CHDP", target: ParamTarget::ChorusDepth, default_pct: 35 },
            ],
            &[
                PageSlot { label: "DLTM", target: ParamTarget::DelayTime, default_pct: 30 },
                PageSlot { label: "DLFB", target: ParamTarget::DelayFb, default_pct: 38 },
                PageSlot { label: "DLMX", target: ParamTarget::DelayMix, default_pct: 22 },
                PageSlot { label: "RVMX", target: ParamTarget::RevMix, default_pct: 25 },
            ],
        ],
    },
    SectionDef {
        name: "VOICE",
        icon: ICON_VOICE,
        pages: &[
            &[
                PageSlot { label: "GLID", target: ParamTarget::Glide, default_pct: 15 },
                PageSlot { label: "UNIS", target: ParamTarget::UnisonVoices, default_pct: 0 },
                PageSlot { label: "UDET", target: ParamTarget::UnisonDetune, default_pct: 25 },
                PageSlot { label: "MVOL", target: ParamTarget::MasterVol, default_pct: 75 },
            ],
        ],
    },
    SectionDef {
        name: "ARP",
        icon: ICON_ARP,
        pages: &[
            &[
                PageSlot { label: "ON", target: ParamTarget::ArpOn, default_pct: 0 },
                PageSlot { label: "HOLD", target: ParamTarget::ArpHold, default_pct: 0 },
                PageSlot { label: "MODE", target: ParamTarget::ArpMode, default_pct: 0 },
                PageSlot { label: "RATE", target: ParamTarget::ArpRate, default_pct: 78 },
            ],
            &[
                PageSlot { label: "OCT", target: ParamTarget::ArpOct, default_pct: 0 },
            ],
        ],
    },
];

/// Category-based default HOME macro targets aligned to the 4 top pots.
pub const fn category_macro_targets(category: &str) -> [ParamTarget; 4] {
    let b = category.as_bytes();
    if b.len() == 4 && b[0] == b'B' && b[1] == b'a' {
        [
            ParamTarget::Cutoff,
            ParamTarget::Resonance,
            ParamTarget::FilterEnvAmt,
            ParamTarget::FilterDecay,
        ]
    } else if b.len() == 4 && b[0] == b'L' && b[1] == b'e' {
        [
            ParamTarget::Cutoff,
            ParamTarget::Resonance,
            ParamTarget::Glide,
            ParamTarget::FilterEnvAmt,
        ]
    } else if b.len() == 3 && b[0] == b'P' && b[1] == b'a' {
        [
            ParamTarget::AmpAttack,
            ParamTarget::Cutoff,
            ParamTarget::RevMix,
            ParamTarget::ChorusMix,
        ]
    } else if b.len() == 4 && b[0] == b'K' && b[1] == b'e' {
        [
            ParamTarget::Cutoff,
            ParamTarget::AmpDecay,
            ParamTarget::AmpRelease,
            ParamTarget::RevMix,
        ]
    } else {
        [
            ParamTarget::Cutoff,
            ParamTarget::Resonance,
            ParamTarget::DelayMix,
            ParamTarget::RevMix,
        ]
    }
}

/// Category-based default HOME macro labels aligned to the 4 top pots.
pub const fn category_macro_labels(category: &str) -> [&'static str; 4] {
    let b = category.as_bytes();
    if b.len() == 4 && b[0] == b'B' && b[1] == b'a' {
        ["CUT", "RES", "FENV", "DEC"]
    } else if b.len() == 4 && b[0] == b'L' && b[1] == b'e' {
        ["CUT", "RES", "GLID", "FENV"]
    } else if b.len() == 3 && b[0] == b'P' && b[1] == b'a' {
        ["ATK", "CUT", "RMIX", "CMIX"]
    } else if b.len() == 4 && b[0] == b'K' && b[1] == b'e' {
        ["CUT", "DEC", "REL", "RMIX"]
    } else {
        ["CUT", "RES", "DMIX", "RMIX"]
    }
}

// ---------------------------------------------------------------------------
// Read-only System Snapshot
// ---------------------------------------------------------------------------

#[derive(Clone)]
pub struct SystemSnapshot {
    pub status: Status,
    pub midi_connected: bool,
    pub midi_flash: bool,
    pub overload_flash: bool,
    pub riff_done: bool,
    pub active_voices: usize,
    pub voice_cap: usize,
    pub bpm: u16,
    pub preset_idx: Option<usize>,
    pub preset_count: usize,
    pub preset_name: &'static str,
    pub preset_category: &'static str,
    pub is_mono: bool,
    /// Arpeggiator engaged — live state from the audio task.
    pub arp_on: bool,
    pub macro_labels: [&'static str; 4],
    pub macro_targets: [ParamTarget; 4],
    pub macro_pct: [u8; 4],
    /// FX stage state [distortion, chorus, delay, reverb] for the bottom row.
    pub fx_on: [bool; 4],
}

impl SystemSnapshot {
    pub fn collect() -> Self {
        let st = status::snapshot();
        let riff_done = web::riff_done();
        let raw_idx = web::current_preset_index();

        let (preset_idx, preset_name, preset_category, is_mono, macro_pct, fx_on) =
            if let Some(summary) = web::preset_summary(raw_idx) {
                (
                    Some(raw_idx),
                    summary.name.as_str(),
                    summary.category.as_str(),
                    summary.is_mono,
                    summary.macro_pct,
                    summary.fx_on,
                )
            } else if !riff_done {
                // BootStress riff runs the whole FX chain.
                (None, "BOOTSTRESS", "STRESS", false, [68, 50, 70, 35], [true; 4])
            } else {
                (None, "INIT+", "INIT", false, [68, 50, 22, 25], [false; 4])
            };

        Self {
            status: st.status,
            midi_connected: usb_midi::device_connected() || usb_midi::midi_seen(),
            midi_flash: st.midi_active,
            overload_flash: st.overload_active,
            riff_done,
            active_voices: web::active_voices(),
            voice_cap: web::user_voice_cap(),
            bpm: web::current_bpm().round() as u16,
            preset_idx,
            preset_count: web::preset_count().max(1),
            preset_name,
            preset_category,
            is_mono,
            arp_on: web::arp_enabled(),
            macro_labels: category_macro_labels(preset_category),
            macro_targets: category_macro_targets(preset_category),
            macro_pct,
            fx_on,
        }
    }
}

// ---------------------------------------------------------------------------
// Pure UI State Machine
// ---------------------------------------------------------------------------

#[derive(Clone, Copy, PartialEq, Eq)]
pub enum ScreenMode {
    /// Boot presentation + animated waveform while Wi-Fi and the demo riff run.
    Splash,
    /// Main performance view: 4 top pot macros, status glyphs, preset hero, 4 softkeys.
    Home,
    /// Overview carousel of synth modules (OSC, FILT, ENV, LFO, FX, VOICE).
    Menu { selected: usize },
    /// 4-column parameter pages inside a module.
    /// `cursor` 0..4 selects a parameter column; `editing` locks encoder to value adjust.
    Section {
        sec: usize,
        page: usize,
        cursor: usize,
        editing: bool,
    },
}

pub struct UiState {
    pub mode: ScreenMode,
    pub frame: u32,
    /// Live macro bar values on HOME (initialized from preset, updated by pots).
    pub home_macros: [u8; 4],
    pub last_preset_idx: Option<usize>,
    pub mono_override: Option<bool>,
    /// FX kill-switch state [dist, chorus, delay, reverb] once toggled from
    /// the preset's declared state; `None` until a button/encoder acts.
    pub fx_override: Option<[bool; 4]>,
    /// Last non-zero mix (0..=100) per FX stage, restored when un-killing
    /// chorus/delay/reverb (their enable = mix > 0). Distortion has a real
    /// enable flag and does not need a slot here.
    pub fx_mix_mem: [u8; 4],
    /// Live parameter values per `[section][page][col]`.
    pub section_vals: [[[u8; 4]; 2]; SECTIONS.len()],
    /// Latest physical pot positions `0..=100` for visual tick markers.
    pub pot_pos: [u8; 2],
    /// Transient overlay message `(text, deadline_frame)` — e.g. the
    /// Wi-Fi radio defeat feedback on encoder double-click (~2 s).
    pub toast: Option<(&'static str, u32)>,
}

impl Default for UiState {
    fn default() -> Self {
        Self::new()
    }
}

impl UiState {
    pub fn new() -> Self {
        let mut section_vals = [[[50u8; 4]; 2]; SECTIONS.len()];
        for (si, sec) in SECTIONS.iter().enumerate() {
            for (pi, page) in sec.pages.iter().enumerate() {
                for (ci, slot) in page.iter().enumerate() {
                    section_vals[si][pi][ci] = slot.default_pct;
                }
            }
        }
        Self {
            mode: ScreenMode::Splash,
            frame: 0,
            home_macros: [68, 50, 70, 35],
            last_preset_idx: None,
            mono_override: None,
            fx_override: None,
            fx_mix_mem: [50; 4],
            section_vals,
            pot_pos: [0, 0],
            toast: None,
        }
    }

    /// Effective FX on/off state: preset-declared until toggled locally.
    pub fn fx_on(&self, snap: &SystemSnapshot) -> [bool; 4] {
        self.fx_override.unwrap_or(snap.fx_on)
    }

    /// Toggle one FX stage (0=DIST, 1=CHRS, 2=DLY, 3=REV) — driven by the
    /// physical kill buttons on GPIO 41/42/2/1. Chorus/Delay/Reverb have no
    /// enable flag: killing them stores their mix and pushes 0.0; un-killing
    /// restores the stored mix. Distortion uses `DistEnable` directly.
    pub fn toggle_fx(
        &mut self,
        idx: usize,
        snap: &SystemSnapshot,
        emit: &mut impl FnMut(WebCmd),
    ) {
        let mut on = self.fx_on(snap);
        on[idx] = !on[idx];
        self.fx_override = Some(on);
        let mix = self.fx_mix_mem[idx] as f32 / 100.0;
        match idx {
            0 => emit(WebCmd::DistEnable(on[0])),
            1 => emit(WebCmd::ChorusMix(if on[1] { mix } else { 0.0 })),
            2 => emit(WebCmd::DelayMix(if on[2] { mix } else { 0.0 })),
            _ => emit(WebCmd::RevMix(if on[3] { mix } else { 0.0 })),
        }
    }

    pub fn step(
        &mut self,
        inp: &InputFrame,
        snap: &SystemSnapshot,
        mut emit: impl FnMut(WebCmd),
    ) {
        self.frame = self.frame.wrapping_add(1);
        self.pot_pos = inp.pot_pct;

        if let Some((_, until)) = self.toast {
            if self.frame >= until {
                self.toast = None;
            }
        }

        // Rotary double-click toggles the Wi-Fi radio globally — an A/B
        // test for the ~10 Hz beacon-burst EMI click in the DAC path.
        // Single clicks are deferred ~400 ms in input.rs so this gesture
        // doesn't also fire the normal click action.
        if inp.encoder_dbl_click {
            let on = web::toggle_wifi_radio();
            self.toast = Some((
                if on { "WIFI ON" } else { "WIFI OFF" },
                self.frame + 50,
            ));
            return;
        }

        // Physical FX kill buttons (GPIO 41/42/2/1, left→right) — global on
        // every screen so they work mid-performance. Toast names the new
        // state; the bottom FX row reflects it via `fx_override`.
        const FX_TOAST: [[&str; 2]; 4] = [
            ["DIST OFF", "DIST ON"],
            ["CHRS OFF", "CHRS ON"],
            ["DLY OFF", "DLY ON"],
            ["REV OFF", "REV ON"],
        ];
        for (i, pressed) in inp.button_press.iter().enumerate() {
            if *pressed {
                // Breadboard row is mirrored vs. the on-screen
                // DIST·CHRS·DLY·REV order — 3-i puts DIST on the left.
                let fx = 3 - i;
                self.toggle_fx(fx, snap, &mut emit);
                self.toast = Some((FX_TOAST[fx][self.fx_on(snap)[fx] as usize], self.frame + 50));
            }
        }

        // Sync HOME macro bars whenever the preset changes
        if snap.preset_idx != self.last_preset_idx {
            self.last_preset_idx = snap.preset_idx;
            self.home_macros = snap.macro_pct;
            self.mono_override = None;
            self.fx_override = None;
        }

        match self.mode {
            ScreenMode::Splash => {
                if self.frame >= SPLASH_MIN_FRAMES
                    || snap.riff_done
                    || inp.encoder_click
                    || inp.encoder_long_press
                    || inp.encoder_delta != 0
                {
                    self.mode = ScreenMode::Home;
                    self.home_macros = snap.macro_pct;
                }
            }
            ScreenMode::Home => {
                // 1. Rotary encoder rotation scrolls through presets (0..preset_count)
                if inp.encoder_delta != 0 {
                    let count = snap.preset_count as i32;
                    let cur = snap.preset_idx.map_or(0, |i| i as i32);
                    let next = (cur + inp.encoder_delta).rem_euclid(count) as u8;
                    usb_midi::set_program_change(next);
                }

                // 2. Pot 0 (left, GPIO 2) & Pot 1 (second, GPIO 1) drive Macro 0 & Macro 1
                for pot_idx in 0..2 {
                    if let Some(pct) = inp.pot_moved[pot_idx] {
                        self.home_macros[pot_idx] = pct;
                        let cmd = snap.macro_targets[pot_idx].to_cmd(pct);
                        emit(cmd);
                    }
                }

                // 3. Encoder short click -> enter MENU
                if inp.encoder_click {
                    self.mode = ScreenMode::Menu { selected: 0 };
                } else if inp.encoder_long_press {
                    // Long press on HOME toggles Mono/Poly mode
                    let now_mono = self.mono_override.unwrap_or(snap.is_mono);
                    let new_mono = !now_mono;
                    self.mono_override = Some(new_mono);
                    emit(WebCmd::PolyMode(!new_mono));
                }
            }
            ScreenMode::Menu { selected } => {
                let mut sel = selected;
                if inp.encoder_delta != 0 {
                    sel = (sel as i32 + inp.encoder_delta)
                        .rem_euclid(SECTIONS.len() as i32) as usize;
                    self.mode = ScreenMode::Menu { selected: sel };
                }
                if inp.encoder_long_press {
                    self.mode = ScreenMode::Home;
                } else if inp.encoder_click {
                    self.mode = ScreenMode::Section {
                        sec: sel,
                        page: 0,
                        cursor: 0,
                        editing: false,
                    };
                }
            }
            ScreenMode::Section {
                sec,
                mut page,
                mut cursor,
                mut editing,
            } => {
                let section = &SECTIONS[sec];

                // Pots 0 and 1 always control Columns 0 and 1 of the active page!
                for pot_idx in 0..2 {
                    if let Some(pct) = inp.pot_moved[pot_idx] {
                        if pot_idx < section.pages[page].len() {
                            self.section_vals[sec][page][pot_idx] = pct;
                            let target = section.pages[page][pot_idx].target;
                            emit(target.to_cmd(pct));
                        }
                    }
                }

                if inp.encoder_long_press {
                    // Long press returns to MENU
                    self.mode = ScreenMode::Menu { selected: sec };
                    return;
                }

                if inp.encoder_click {
                    // Click toggles between moving the column cursor and editing the focused column
                    editing = !editing;
                }

                if inp.encoder_delta != 0 {
                    if editing {
                        let target = section.pages[page][cursor].target;
                        let cur_val = self.section_vals[sec][page][cursor] as i32;
                        let new_val = if let Some(steps) = target.steps() {
                            // Stepped params: one detent = one position
                            // (ON/OFF flips with a single click — no bar ride).
                            let cur_idx = target.index(cur_val as u8) as i32;
                            let new_idx = (cur_idx + inp.encoder_delta)
                                .clamp(0, steps as i32 - 1);
                            target.index_pct(new_idx as usize) as i32
                        } else {
                            // Continuous: 4% per detent, speed-scaled
                            // via param_delta (x2/x4/x8 on fast spins).
                            cur_val + inp.param_delta * 4
                        };
                        let new_val = new_val.clamp(0, 100) as u8;
                        self.section_vals[sec][page][cursor] = new_val;
                        emit(target.to_cmd(new_val));
                    } else {
                        // Move column cursor across all columns, wrapping
                        // across pages — pages may hold <4 slots (ARP pg 2).
                        let total: usize = section.pages.iter().map(|p| p.len()).sum();
                        let flat = section.pages[..page]
                            .iter()
                            .map(|p| p.len())
                            .sum::<usize>()
                            + cursor;
                        let next = (flat as i32 + inp.encoder_delta)
                            .rem_euclid(total as i32) as usize;
                        let mut rem = next;
                        page = 0;
                        while rem >= section.pages[page].len() {
                            rem -= section.pages[page].len();
                            page += 1;
                        }
                        cursor = rem;
                    }
                }

                self.mode = ScreenMode::Section {
                    sec,
                    page,
                    cursor,
                    editing,
                };
            }
        }
    }
}

// ---------------------------------------------------------------------------
// 8×8 status glyphs (MSB = left pixel)
// ---------------------------------------------------------------------------

const GLYPH_WIFI_ON: [u8; 8] = [
    0b00111100,
    0b01000010,
    0b10011001,
    0b00100100,
    0b01000010,
    0b00011000,
    0b00011000,
    0b00000000,
];

const GLYPH_WIFI_WAIT: [u8; 8] = [
    0b00100100,
    0b00000000,
    0b00011000,
    0b00000000,
    0b00000000,
    0b00011000,
    0b00011000,
    0b00000000,
];

const GLYPH_MIDI: [u8; 8] = [
    0b00111100,
    0b01011010,
    0b10100101,
    0b10000001,
    0b10100101,
    0b01000010,
    0b00111100,
    0b00000000,
];

const GLYPH_VOICE: [u8; 8] = [
    0b00001000,
    0b00011010,
    0b01111001,
    0b01111011,
    0b01111001,
    0b00011010,
    0b00001000,
    0b00000000,
];

// ---------------------------------------------------------------------------
// Pure screen renderer
// ---------------------------------------------------------------------------

pub fn render(state: &UiState, snap: &SystemSnapshot, canvas: &mut Canvas) {
    canvas.clear();
    // Bottom row is the same on every screen: the 4 physical buttons map to
    // DIST / CHRS / DLY / REV kill switches (filled = FX active).
    let fx = state.fx_on(snap);
    match state.mode {
        ScreenMode::Splash => render_splash(state.frame, snap, canvas),
        ScreenMode::Home => render_home(state, snap, canvas),
        ScreenMode::Menu { selected } => render_menu(selected, canvas, fx),
        ScreenMode::Section {
            sec,
            page,
            cursor,
            editing,
        } => render_section(state, sec, page, cursor, editing, canvas, fx),
    }

    // Transient toast overlay (Wi-Fi radio toggle feedback, ~2 s).
    if let Some((msg, until)) = state.toast {
        if state.frame < until {
            let w = Canvas::str_width(msg, 1) + 10;
            let x0 = (WIDTH as i32 - w) / 2;
            canvas.fill_rect(x0, 26, w, 12, true);
            canvas.draw_str_centered(28, msg, 1, false);
        }
    }
}

fn render_splash(frame: u32, snap: &SystemSnapshot, c: &mut Canvas) {
    c.rect(0, 0, WIDTH as i32, HEIGHT as i32, true);
    c.fill_rect(0, 0, 4, 4, true);
    c.fill_rect(WIDTH as i32 - 4, 0, 4, 4, true);

    c.draw_str_centered(4, "RUST IN", 2, true);
    c.draw_str_centered(18, "SYNTH", 2, true);

    let phase_shift = (frame * 5) as i32;
    let morph = ((frame % 80) as i32 - 40).abs();
    let mut prev_y = 38i32;
    for x in 4..(WIDTH as i32 - 4) {
        let p = ((x * 6 + phase_shift) & 0xFF) as i32;
        let half = (p & 0x7F) - 64;
        let parabola = 64 - ((half * half) >> 6);
        let sin_val = if (p & 0x80) != 0 { -parabola } else { parabola };
        let saw_val = (p >> 1) - 64;
        let blended = (sin_val * morph + saw_val * (40 - morph)) / 40;
        let y = (38 - (blended / 8)).clamp(29, 47);

        if x > 4 {
            let y0 = prev_y.min(y);
            let y1 = prev_y.max(y);
            c.vline(x, y0, (y1 - y0 + 1).max(1), true);
        } else {
            c.set_pixel(x, y, true);
        }
        prev_y = y;
    }

    let prog_w = ((frame.min(SPLASH_MIN_FRAMES) * 116) / SPLASH_MIN_FRAMES) as i32;
    c.rect(6, 49, 116, 5, true);
    c.fill_rect(7, 50, prog_w.clamp(1, 114), 3, true);

    let status_txt = match snap.status {
        Status::Ready => "WEB UP 192.168.71.1",
        Status::Error => "WEB SERVER ERROR",
        Status::Booting => "STARTING WIFI AP...",
    };
    c.draw_str_centered(55, status_txt, 1, true);
}

fn render_home(state: &UiState, snap: &SystemSnapshot, c: &mut Canvas) {
    // 1. Top 4 Pot Macro Columns (x = 0, 32, 64, 96)
    for col in 0..4 {
        let x0 = (col as i32) * 32;
        let label = snap.macro_labels[col];
        let lw = Canvas::str_width(label, 1);
        let lx = x0 + (32 - lw) / 2;
        c.draw_str(lx, 0, label, 1, true);

        // Mini pot value bar (26 px wide, 4 px tall)
        c.rect(x0 + 3, 8, 26, 4, true);
        let fill = ((state.home_macros[col] as i32) * 24) / 100;
        if fill > 0 {
            c.fill_rect(x0 + 4, 9, fill.clamp(1, 24), 2, true);
        }
        // For wired Pots 0 & 1, draw a small physical-position tick under the bar
        if col < 2 && POTS_ENABLED {
            let tick_x = x0 + 4 + ((state.pot_pos[col] as i32) * 23) / 100;
            c.set_pixel(tick_x, 12, true);
        }
        if col < 3 {
            c.vline(x0 + 31, 0, 12, true);
        }
    }
    c.hline(0, 13, WIDTH as i32, true);

    // 2. Status Glyph Strip (y = 15..23)
    match snap.status {
        Status::Ready => {
            c.draw_icon8(2, 15, &GLYPH_WIFI_ON, true);
            c.draw_str(12, 15, "WEB", 1, true);
        }
        Status::Booting => {
            if (state.frame / 8) % 2 == 0 {
                c.draw_icon8(2, 15, &GLYPH_WIFI_WAIT, true);
            }
            c.draw_str(12, 15, "AP..", 1, true);
        }
        Status::Error => {
            c.fill_rect(1, 14, 29, 9, true);
            c.draw_str(3, 15, "!WEB", 1, false);
        }
    }

    if snap.midi_flash {
        c.fill_rect(36, 14, 34, 9, true);
        c.draw_icon8(38, 15, &GLYPH_MIDI, false);
        c.draw_str(48, 15, "MIDI", 1, false);
    } else if snap.midi_connected {
        c.draw_icon8(38, 15, &GLYPH_MIDI, true);
        c.draw_str(48, 15, "MIDI", 1, true);
    } else {
        c.draw_icon8(38, 15, &GLYPH_MIDI, true);
        c.draw_str(48, 15, "--", 1, true);
    }

    if snap.overload_flash {
        c.fill_rect(76, 14, 51, 9, true);
        c.draw_str(80, 15, "!SHED!", 1, false);
    } else {
        c.draw_icon8(74, 15, &GLYPH_VOICE, true);
        let mut vbuf = [0u8; 10];
        let vstr = format_voices_bpm(snap.active_voices, snap.voice_cap, snap.bpm, &mut vbuf);
        c.draw_str(84, 15, vstr, 1, true);
    }

    c.hline(0, 24, WIDTH as i32, true);

    // 3. Center Preset Hero Area (y = 26..51)
    let is_mono = state.mono_override.unwrap_or(snap.is_mono);
    let mut hdr_buf = [0u8; 22];
    let hdr = format_preset_header(
        snap.preset_idx,
        snap.preset_category,
        is_mono,
        snap.arp_on,
        &mut hdr_buf,
    );
    c.draw_str_centered(27, hdr, 1, true);

    let name = snap.preset_name;
    let char_count = name.chars().count();
    if char_count <= 10 {
        c.draw_str_centered(37, name, 2, true);
    } else {
        let total_w = Canvas::str_width(name, 2);
        let max_scroll = (total_w - 120).max(0);
        let cycle = (max_scroll as u32) + 40;
        let pos = state.frame % (cycle * 2);
        let phase = if pos < cycle { pos } else { cycle * 2 - pos };
        let offset = (phase as i32 - 20).clamp(0, max_scroll);
        c.draw_str(4 - offset, 37, name, 2, true);
    }

    // 4. Bottom row: FX kill-switch readout for the 4 buttons
    //    (DIST / CHRS / DLY / REV — filled = stage active)
    draw_fx_row(c, state.fx_on(snap));
}

/// MENU view: 3-up horizontal card carousel of synth modules with icons + names.
fn render_menu(selected: usize, c: &mut Canvas, fx: [bool; 4]) {
    // Top title bar
    c.fill_rect(0, 0, WIDTH as i32, 11, true);
    c.draw_str(3, 2, "SYNTH MODULES", 1, false);
    let mut pg_buf = [0u8; 5];
    pg_buf[0] = b'0' + ((selected + 1) as u8);
    pg_buf[1] = b'/';
    pg_buf[2] = b'0' + (SECTIONS.len() as u8);
    if let Ok(s) = core::str::from_utf8(&pg_buf[..3]) {
        c.draw_str(106, 2, s, 1, false);
    }

    // Show 3 adjacent cards (window of 3 out of 6)
    let start = if selected == 0 {
        0
    } else if selected + 1 >= SECTIONS.len() {
        SECTIONS.len() - 3
    } else {
        selected - 1
    };

    for slot in 0..3 {
        let idx = start + slot;
        let sec = &SECTIONS[idx];
        let x0 = 4 + (slot as i32) * 41;
        let is_sel = idx == selected;

        if is_sel {
            c.fill_rect(x0, 15, 38, 34, true);
            c.draw_icon8(x0 + 15, 19, &sec.icon, false);
            let nw = Canvas::str_width(sec.name, 1);
            c.draw_str(x0 + (38 - nw) / 2, 36, sec.name, 1, false);
        } else {
            c.rect(x0, 15, 38, 34, true);
            c.draw_icon8(x0 + 15, 19, &sec.icon, true);
            let nw = Canvas::str_width(sec.name, 1);
            c.draw_str(x0 + (38 - nw) / 2, 36, sec.name, 1, true);
        }
    }

    draw_fx_row(c, fx);
}

/// SECTION view: 4 parameter columns aligned under the 4 top pots!
fn render_section(
    state: &UiState,
    sec_idx: usize,
    page_idx: usize,
    cursor: usize,
    editing: bool,
    c: &mut Canvas,
    fx: [bool; 4],
) {
    let sec = &SECTIONS[sec_idx];
    let page = sec.pages[page_idx];

    // Header bar (y = 0..10) showing section name + page number + edit mode hint
    c.fill_rect(0, 0, WIDTH as i32, 11, true);
    c.draw_str(3, 2, sec.name, 1, false);
    let mut pbuf = [0u8; 8];
    pbuf[0] = b'P';
    pbuf[1] = b'G';
    pbuf[2] = b' ';
    pbuf[3] = b'0' + ((page_idx + 1) as u8);
    pbuf[4] = b'/';
    pbuf[5] = b'0' + (sec.pages.len() as u8);
    if let Ok(ps) = core::str::from_utf8(&pbuf[..6]) {
        c.draw_str(44, 2, ps, 1, false);
    }
    c.draw_str(92, 2, if editing { "[EDIT]" } else { "[NAV]" }, 1, false);

    // Parameter Columns (x = 0, 32, 64, 96), y = 13..51 — up to 4 slots
    // per page, aligned under the top pots (ARP pg 2 carries only OCT).
    for (col, slot) in page.iter().enumerate() {
        let x0 = (col as i32) * 32;
        let val = state.section_vals[sec_idx][page_idx][col];
        let focused = col == cursor;

        // Column header (inverted when focused)
        if focused {
            c.fill_rect(x0 + 1, 13, 30, 9, true);
        }
        let lw = Canvas::str_width(slot.label, 1);
        c.draw_str(x0 + (32 - lw) / 2, 14, slot.label, 1, !focused);

        // Vertical bar gauge (x0+10..x0+22, y=24..42, 18 px tall)
        c.rect(x0 + 9, 24, 14, 18, true);
        let fill_h = ((val as i32) * 16) / 100;
        if fill_h > 0 {
            let fy = 24 + 17 - fill_h;
            c.fill_rect(x0 + 10, fy, 12, fill_h, true);
        }

        // Physical pot position tick on the left side of the bar for Pots 0 & 1
        if col < 2 && POTS_ENABLED {
            let ty = 24 + 16 - ((state.pot_pos[col] as i32) * 15) / 100;
            c.hline(x0 + 6, ty, 3, true);
        }

        // Value readout at y = 44 — text label for discrete params,
        // numeric 0..100 for continuous ones.
        let mut nbuf = [0u8; 4];
        let nstr = slot
            .target
            .value_label(val)
            .unwrap_or_else(|| format_u8(val, &mut nbuf));
        let nw = Canvas::str_width(nstr, 1);
        c.draw_str(x0 + (32 - nw) / 2, 44, nstr, 1, true);

        if col < 3 {
            c.vline(x0 + 31, 12, 40, true);
        }
    }

    draw_fx_row(c, fx);
}

/// Bottom row on every screen: the 4 physical buttons are FX kill switches
/// (DIST / CHRS / DLY / REV). Filled box = stage active, hollow = killed.
fn draw_fx_row(c: &mut Canvas, fx_on: [bool; 4]) {
    const LABELS: [&str; 4] = ["DIST", "CHRS", "DLY", "REV"];
    for (i, &lbl) in LABELS.iter().enumerate() {
        let x0 = (i as i32) * 32;
        let on = fx_on[i];
        if on {
            c.fill_rect(x0 + 1, 54, 30, 10, true);
        } else {
            c.rect(x0 + 1, 54, 30, 10, true);
        }
        let lw = Canvas::str_width(lbl, 1);
        let lx = x0 + (32 - lw) / 2;
        c.draw_str(lx, 56, lbl, 1, !on);
    }
}

fn format_u8<'a>(v: u8, buf: &'a mut [u8; 4]) -> &'a str {
    let mut n = 0;
    if v >= 100 {
        buf[n] = b'1';
        buf[n + 1] = b'0';
        buf[n + 2] = b'0';
        n = 3;
    } else if v >= 10 {
        buf[n] = b'0' + (v / 10);
        buf[n + 1] = b'0' + (v % 10);
        n = 2;
    } else {
        buf[n] = b'0' + v;
        n = 1;
    }
    core::str::from_utf8(&buf[..n]).unwrap_or("0")
}

fn format_voices_bpm<'a>(active: usize, cap: usize, bpm: u16, buf: &'a mut [u8; 10]) -> &'a str {
    buf[0] = b'0' + (active.min(9) as u8);
    buf[1] = b'/';
    buf[2] = b'0' + (cap.min(9) as u8);
    buf[3] = b' ';
    let b = bpm.clamp(20, 999);
    let mut n = 4;
    if b >= 100 {
        buf[n] = b'0' + ((b / 100) as u8);
        n += 1;
    }
    buf[n] = b'0' + (((b / 10) % 10) as u8);
    n += 1;
    buf[n] = b'0' + ((b % 10) as u8);
    n += 1;
    core::str::from_utf8(&buf[..n]).unwrap_or("0/5")
}

fn format_preset_header<'a>(
    idx: Option<usize>,
    cat: &str,
    is_mono: bool,
    arp_on: bool,
    buf: &'a mut [u8; 22],
) -> &'a str {
    let mut n = 0usize;
    let mut push = |bytes: &[u8]| {
        for &b in bytes {
            if n < buf.len() {
                buf[n] = b;
                n += 1;
            }
        }
    };
    match idx {
        Some(i) => {
            let num = (i + 1).min(99);
            push(&[b'P', b'0' + ((num / 10) as u8), b'0' + ((num % 10) as u8), b' ']);
        }
        None => push(b"BOOT "),
    }
    push(b"[");
    push(&cat.as_bytes()[..cat.len().min(6)]);
    push(b"] ");
    push(if is_mono { b"MONO" } else { b"POLY" });
    if arp_on {
        push(b" +ARP");
    }
    core::str::from_utf8(&buf[..n]).unwrap_or("SYNTH")
}

fn ui_task_loop(display: &mut impl Display, inputs: &mut Option<PhysicalInputs<'static>>) -> ! {
    let mut state = UiState::new();
    let mut canvas = Canvas::new();
    info!("oled-ui: task running @ 25 FPS (Splash -> Home <-> Menu <-> Section)");
    loop {
        let inp = inputs
            .as_mut()
            .map_or_else(InputFrame::default, |src| src.poll());
        let snap = SystemSnapshot::collect();
        state.step(&inp, &snap, web::push_cmd);
        render(&state, &snap, &mut canvas);
        display.flush(&canvas);
        FreeRtos::delay_ms(FPS_DELAY_MS);
    }
}

/// Initialize the 1.3" SPI OLED (`GPIO 9..13`), FX kill buttons
/// (`GPIO 41`, `42`, `2`, `1`), and PCNT rotary encoder (`GPIO 38..40`),
/// then spawn the Core 0 UI task.
#[allow(clippy::too_many_arguments)]
pub fn init<SPI, SCLK, MOSI, RES, DC, CS>(
    spi: SPI,
    sclk: SCLK,
    mosi: MOSI,
    res: RES,
    dc: DC,
    cs: CS,
    btn0_gpio41: Gpio41<'static>,
    btn1_gpio42: Gpio42<'static>,
    btn2_gpio2: Gpio2<'static>,
    btn3_gpio1: Gpio1<'static>,
    enc_clk_gpio38: Gpio38<'static>,
    enc_dt_gpio39: Gpio39<'static>,
    enc_sw_gpio40: Gpio40<'static>,
) where
    SPI: SpiAnyPins + Send + 'static,
    SCLK: OutputPin + Send + 'static,
    MOSI: OutputPin + Send + 'static,
    RES: OutputPin + Send + 'static,
    DC: OutputPin + Send + 'static,
    CS: OutputPin + Send + 'static,
{
    let mut inputs = match PhysicalInputs::new(
        btn0_gpio41,
        btn1_gpio42,
        btn2_gpio2,
        btn3_gpio1,
        enc_clk_gpio38,
        enc_dt_gpio39,
        enc_sw_gpio40,
    ) {
        Ok(inp) => Some(inp),
        Err(e) => {
            warn!("oled-ui: physical inputs init failed: {e:?}");
            None
        }
    };

    match Sh1106Spi::new(spi, sclk, mosi, res, dc, cs) {
        Ok(mut drv) => {
            info!("oled-ui: SH1106 SPI + inputs initialized");
            if let Err(e) = std::thread::Builder::new()
                .stack_size(8 * 1024)
                .name("oled-ui".into())
                .spawn(move || ui_task_loop(&mut drv, &mut inputs))
            {
                warn!("oled-ui: failed to spawn task: {e}");
            }
        }
        Err(e) => warn!("oled-ui: SPI display init failed: {e:?}"),
    }
}
