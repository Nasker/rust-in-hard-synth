//! Physical UI state machine and screen renderer (Milestone 1: Splash + Home + Menu preview).
//!
//! Strict SRP separation:
//! - [`SystemSnapshot`]: read-only view of synth/network/MIDI health gathered
//!   via lock-free atomics from `status`, `web`, and `usb_midi`.
//! - [`UiState`]: pure state machine (`Splash` -> `Home` <-> `Menu`) + frame
//!   counter. Zero hardware dependencies; unit-testable in isolation.
//! - [`render`]: pure view function `(UiState, SystemSnapshot) -> Canvas`.
//! - [`init`]: wires the SPI pins into [`Sh1106Spi`](crate::display::Sh1106Spi)
//!   and spawns the low-priority Core 0 refresh task (~25 FPS).

use crate::display::{Canvas, Display, Sh1106Spi, HEIGHT, WIDTH};
use crate::status::{self, Status};
use crate::usb_midi;
use crate::web;
use esp_idf_hal::delay::FreeRtos;
use esp_idf_hal::gpio::OutputPin;
use esp_idf_hal::spi::SpiAnyPins;
use log::{info, warn};

const FPS_DELAY_MS: u32 = 40; // 25 FPS
const SPLASH_MIN_FRAMES: u32 = 135; // ~5.4 s presentation animation

/// Read-only snapshot of live engine + connectivity state for a single UI frame.
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
    pub preset_name: &'static str,
    pub preset_category: &'static str,
    pub is_mono: bool,
    pub macro_labels: [&'static str; 4],
    pub macro_pct: [u8; 4],
}

impl SystemSnapshot {
    pub fn collect() -> Self {
        let st = status::snapshot();
        let riff_done = web::riff_done();
        let raw_idx = web::current_preset_index();

        let (preset_idx, preset_name, preset_category, is_mono, macro_pct) =
            if let Some(summary) = web::preset_summary(raw_idx) {
                (
                    Some(raw_idx),
                    summary.name.as_str(),
                    summary.category.as_str(),
                    summary.is_mono,
                    summary.macro_pct,
                )
            } else if !riff_done {
                (None, "BOOTSTRESS", "STRESS", false, [68, 50, 70, 35])
            } else {
                (None, "INIT+", "INIT", false, [68, 50, 22, 25])
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
            preset_name,
            preset_category,
            is_mono,
            macro_labels: category_macro_labels(preset_category),
            macro_pct,
        }
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

/// High-level screen state machine (per PHYSICAL_UI.md).
#[derive(Clone, Copy, PartialEq, Eq)]
pub enum ScreenMode {
    /// Boot presentation + animated waveform while Wi-Fi and the demo riff run.
    Splash,
    /// Main performance view: 4 top pot macros, status glyphs, preset hero, 4 softkeys.
    Home,
}

pub struct UiState {
    pub mode: ScreenMode,
    pub frame: u32,
}

impl Default for UiState {
    fn default() -> Self {
        Self::new()
    }
}

impl UiState {
    pub const fn new() -> Self {
        Self {
            mode: ScreenMode::Splash,
            frame: 0,
        }
    }

    pub fn step(&mut self, snap: &SystemSnapshot) {
        self.frame = self.frame.wrapping_add(1);
        if self.mode == ScreenMode::Splash
            && (self.frame >= SPLASH_MIN_FRAMES || snap.riff_done)
        {
            self.mode = ScreenMode::Home;
        }
    }
}

// ---------------------------------------------------------------------------
// 8×8 status glyphs (MSB = left pixel)
// ---------------------------------------------------------------------------

/// Radiating Wi-Fi / Web-server link icon (8×8).
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

/// Dim / searching Wi-Fi icon (8×8).
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

/// 5-pin DIN MIDI connector icon (8×8).
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

/// Speaker / voice activity icon (8×8).
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
    match state.mode {
        ScreenMode::Splash => render_splash(state.frame, snap, canvas),
        ScreenMode::Home => render_home(state.frame, snap, canvas),
    }
}

/// Boot presentation screen: border, bold title, live morphing synth wave,
/// boot progress bar, and live Wi-Fi / engine status line.
fn render_splash(frame: u32, snap: &SystemSnapshot, c: &mut Canvas) {
    // Outer frame with corner accents
    c.rect(0, 0, WIDTH as i32, HEIGHT as i32, true);
    c.fill_rect(0, 0, 4, 4, true);
    c.fill_rect(WIDTH as i32 - 4, 0, 4, 4, true);

    // Title in 2× scale (10×14 px per char)
    c.draw_str_centered(4, "RINSYNTH", 2, true);
    c.draw_str_centered(20, "ESP32-S3 VA SYNTH", 1, true);

    // Animated oscilloscope waveform across y = 29..47 (center y = 38)
    let phase_shift = (frame * 5) as i32;
    let morph = ((frame % 80) as i32 - 40).abs(); // 0..40 triangle morph
    let mut prev_y = 38i32;
    for x in 4..(WIDTH as i32 - 4) {
        let p = ((x * 6 + phase_shift) & 0xFF) as i32; // 0..255 cycle
        // Integer sine approximation + saw harmonic blend
        let half = (p & 0x7F) - 64;
        let parabola = 64 - ((half * half) >> 6);
        let sin_val = if (p & 0x80) != 0 { -parabola } else { parabola }; // -64..64
        let saw_val = (p >> 1) - 64; // -64..63
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

    // Progress bar (y = 49..53)
    let prog_w = ((frame.min(SPLASH_MIN_FRAMES) * 116) / SPLASH_MIN_FRAMES) as i32;
    c.rect(6, 49, 116, 5, true);
    c.fill_rect(7, 50, prog_w.clamp(1, 114), 3, true);

    // Status footer (y = 55)
    let status_txt = match snap.status {
        Status::Ready => "WEB UP 192.168.71.1",
        Status::Error => "WEB SERVER ERROR",
        Status::Booting => "STARTING WIFI AP...",
    };
    c.draw_str_centered(55, status_txt, 1, true);
}

/// Main HOME screen:
/// - Top row (y=0..12): 4 pot macro columns (32 px each) with label + level bar
/// - Status strip (y=15..24): Web glyph, MIDI glyph, Voice count + Mono/Poly + BPM
/// - Hero card (y=26..51): Category/Preset # + bold 2× Preset Name
/// - Bottom row (y=54..63): 4 contextual softkey pills aligned to the 4 buttons
fn render_home(frame: u32, snap: &SystemSnapshot, c: &mut Canvas) {
    // 1. Top 4 Pot Macro Columns (x = 0, 32, 64, 96)
    for col in 0..4 {
        let x0 = (col as i32) * 32;
        let label = snap.macro_labels[col];
        let lw = Canvas::str_width(label, 1);
        let lx = x0 + (32 - lw) / 2;
        c.draw_str(lx, 0, label, 1, true);

        // Mini pot value bar (26 px wide, 4 px tall)
        c.rect(x0 + 3, 8, 26, 4, true);
        let fill = ((snap.macro_pct[col] as i32) * 24) / 100;
        if fill > 0 {
            c.fill_rect(x0 + 4, 9, fill.clamp(1, 24), 2, true);
        }
        if col < 3 {
            c.vline(x0 + 31, 0, 12, true);
        }
    }
    c.hline(0, 13, WIDTH as i32, true);

    // 2. Status Glyph Strip (y = 15..23)
    // Web server glyph + label
    match snap.status {
        Status::Ready => {
            c.draw_icon8(2, 15, &GLYPH_WIFI_ON, true);
            c.draw_str(12, 15, "WEB", 1, true);
        }
        Status::Booting => {
            if (frame / 8) % 2 == 0 {
                c.draw_icon8(2, 15, &GLYPH_WIFI_WAIT, true);
            }
            c.draw_str(12, 15, "AP..", 1, true);
        }
        Status::Error => {
            c.fill_rect(1, 14, 29, 9, true);
            c.draw_str(3, 15, "!WEB", 1, false);
        }
    }

    // MIDI device / note activity glyph
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

    // Right side of status bar: Overload warning OR Voice count + BPM
    if snap.overload_flash {
        c.fill_rect(76, 14, 51, 9, true);
        c.draw_str(80, 15, "!SHED!", 1, false);
    } else {
        c.draw_icon8(74, 15, &GLYPH_VOICE, true);
        let mut vbuf = [0u8; 10];
        let vstr = format_voices_bpm(
            snap.active_voices,
            snap.voice_cap,
            snap.bpm,
            &mut vbuf,
        );
        c.draw_str(84, 15, vstr, 1, true);
    }

    c.hline(0, 24, WIDTH as i32, true);

    // 3. Center Preset Hero Area (y = 26..51)
    let mut hdr_buf = [0u8; 22];
    let hdr = format_preset_header(
        snap.preset_idx,
        snap.preset_category,
        snap.is_mono,
        &mut hdr_buf,
    );
    c.draw_str_centered(27, hdr, 1, true);

    // Preset Name in 2× bold font (auto-scrolls if > 10 chars)
    let name = snap.preset_name;
    let char_count = name.chars().count();
    if char_count <= 10 {
        c.draw_str_centered(37, name, 2, true);
    } else {
        // Smooth horizontal marquee for long preset names
        let total_w = Canvas::str_width(name, 2);
        let max_scroll = (total_w - 120).max(0);
        let cycle = (max_scroll as u32) + 40;
        let pos = frame % (cycle * 2);
        let phase = if pos < cycle { pos } else { cycle * 2 - pos };
        let offset = (phase as i32 - 20).clamp(0, max_scroll);
        c.draw_str(4 - offset, 37, name, 2, true);
    }

    // 4. Bottom 4 Softkey Buttons (y = 54..63, 32 px columns aligned to bottom buttons)
    let softkeys = [
        "PREV",
        "NEXT",
        if snap.is_mono { "MONO" } else { "POLY" },
        "MENU",
    ];
    for (i, &lbl) in softkeys.iter().enumerate() {
        let x0 = (i as i32) * 32;
        let inverted = i == 3; // Highlight MENU as primary navigation entry
        if inverted {
            c.fill_rect(x0 + 1, 54, 30, 10, true);
        } else {
            c.rect(x0 + 1, 54, 30, 10, true);
        }
        let lw = Canvas::str_width(lbl, 1);
        let lx = x0 + (32 - lw) / 2;
        c.draw_str(lx, 56, lbl, 1, !inverted);
    }
}

/// Zero-alloc formatter for `"3/5 120"` (voices + BPM) in the status strip.
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

/// Zero-alloc formatter for `"P01 [BASS] MONO"` header above the preset title.
fn format_preset_header<'a>(
    idx: Option<usize>,
    cat: &str,
    is_mono: bool,
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
    core::str::from_utf8(&buf[..n]).unwrap_or("SYNTH")
}

fn ui_task_loop(display: &mut impl Display) -> ! {
    let mut state = UiState::new();
    let mut canvas = Canvas::new();
    info!("oled-ui: task running @ 25 FPS (Splash -> Home)");
    loop {
        let snap = SystemSnapshot::collect();
        state.step(&snap);
        render(&state, &snap, &mut canvas);
        display.flush(&canvas);
        FreeRtos::delay_ms(FPS_DELAY_MS);
    }
}

/// Initialize the 1.3" SPI OLED on GPIO 9..13 and spawn the Core 0 UI task.
/// Logs a warning and returns if hardware init fails so audio is never blocked.
pub fn init<SPI, SCLK, MOSI, RES, DC, CS>(
    spi: SPI,
    sclk: SCLK,
    mosi: MOSI,
    res: RES,
    dc: DC,
    cs: CS,
) where
    SPI: SpiAnyPins + Send + 'static,
    SCLK: OutputPin + Send + 'static,
    MOSI: OutputPin + Send + 'static,
    RES: OutputPin + Send + 'static,
    DC: OutputPin + Send + 'static,
    CS: OutputPin + Send + 'static,
{
    match Sh1106Spi::new(spi, sclk, mosi, res, dc, cs) {
        Ok(mut drv) => {
            info!("oled-ui: SH1106 SPI initialized (CLK=9 MOSI=10 RES=11 DC=12 CS=13)");
            if let Err(e) = std::thread::Builder::new()
                .stack_size(8 * 1024)
                .name("oled-ui".into())
                .spawn(move || ui_task_loop(&mut drv))
            {
                warn!("oled-ui: failed to spawn task: {e}");
            }
        }
        Err(e) => warn!("oled-ui: SPI display init failed: {e:?}"),
    }
}
