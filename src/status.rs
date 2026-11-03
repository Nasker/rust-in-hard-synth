//! Onboard RGB status indicator (WS2812 "NeoPixel", GPIO48 on DevKitC-1).
//!
//! Single responsibility: turn [`Status`]/`Event` into light. Producers call
//! [`set`]/[`notify`] — plain atomic stores, safe from any thread including
//! the Core 1 audio loop (no alloc, no lock). A low-priority task owns the
//! state→color mapping and refresh loop; the WS2812 wire protocol lives in
//! [`Ws2812`] behind the [`Indicator`] trait so the renderer never touches
//! RMT details and the driver can be swapped (second LED, OLED icon, mock).
//!
//! States:   Booting = blue blink, Ready (web/AP up) = dim green,
//!           Error (web server failed) = red.
//! Overlays: MidiActivity = ~90 ms white flash,
//!           Overload (governor shed) = ~1.5 s amber pulse — outranks MIDI.

use core::sync::atomic::{AtomicU32, AtomicU8, Ordering};
use esp_idf_hal::gpio::OutputPin;
use esp_idf_hal::rmt::config::{TransmitConfig, TxChannelConfig};
use esp_idf_hal::rmt::encoder::simple_encoder::{
    EncoderCallback, NotEnoughSpace, SimpleEncoder, SimpleEncoderConfig, SymbolBuffer,
};
use esp_idf_hal::rmt::{PinState, Pulse, PulseTicks, Symbol, TxChannelDriver};
use esp_idf_hal::units::FromValueType;
use esp_idf_sys::{esp_timer_get_time, EspError};
use log::warn;

/// Steady system states — the LED's base color.
#[derive(Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum Status {
    Booting = 0,
    /// Web server + SoftAP are up — the "go play/edit" signal the user asked for.
    Ready = 1,
    /// Something boot-critical failed (e.g. web::start) — synth still runs.
    Error = 2,
}

/// Transient overlays, shown briefly on top of the base status.
#[derive(Clone, Copy)]
pub enum Event {
    /// USB-MIDI note activity — short white flash.
    MidiActivity,
    /// CPU governor shed a voice / sustained late blocks — amber pulse.
    Overload,
}

const MIDI_FLASH_MS: u32 = 90;
const OVERLOAD_FLASH_MS: u32 = 1_500;
const REFRESH_MS: u32 = 50;

static STATUS: AtomicU8 = AtomicU8::new(Status::Booting as u8);
static MIDI_UNTIL_MS: AtomicU32 = AtomicU32::new(0);
static OVERLOAD_UNTIL_MS: AtomicU32 = AtomicU32::new(0);

/// Millisecond timebase — wraps every ~49.7 days; comparisons below are
/// wrap-safe via `deadline.wrapping_sub(now) < i32::MAX`.
fn now_ms() -> u32 {
    unsafe { (esp_timer_get_time() / 1_000) as u32 }
}

fn pending(deadline: &AtomicU32, now: u32) -> bool {
    deadline.load(Ordering::Relaxed).wrapping_sub(now) < (i32::MAX as u32)
        && deadline.load(Ordering::Relaxed) != 0
}

/// Set the steady-state system status (idempotent, atomic, RT-safe).
pub fn set(status: Status) {
    STATUS.store(status as u8, Ordering::Relaxed);
}

/// Fire a transient overlay event (idempotent, atomic, RT-safe).
pub fn notify(event: Event) {
    let until = now_ms().wrapping_add(match event {
        Event::MidiActivity => MIDI_FLASH_MS,
        Event::Overload => OVERLOAD_FLASH_MS,
    });
    match event {
        Event::MidiActivity => &MIDI_UNTIL_MS,
        Event::Overload => &OVERLOAD_UNTIL_MS,
    }
    .store(until, Ordering::Relaxed);
}

/// What the renderer asks of a physical indicator — the only thing the
/// state machine needs to know about the outside world.
pub trait Indicator: Send {
    fn set_rgb(&mut self, r: u8, g: u8, b: u8);
}

// ---------------------------------------------------------------------------
// WS2812 wire driver (RMT TX, 10 MHz ticks = 100 ns resolution)
// ---------------------------------------------------------------------------

/// One pixel of GRB data over the new-IDF RMT API. `send_and_wait` leaves the
/// line low after the last symbol, which doubles as the >50 µs latch/reset.
struct Ws2812<'d> {
    tx: TxChannelDriver<'d>,
}

impl<'d> Ws2812<'d> {
    fn new(pin: impl OutputPin + 'd) -> Result<Self, EspError> {
        let config = TxChannelConfig {
            resolution: 10.MHz().into(),
            ..Default::default()
        };
        Ok(Self {
            tx: TxChannelDriver::new(pin, &config)?,
        })
    }
}

impl Indicator for Ws2812<'_> {
    fn set_rgb(&mut self, r: u8, g: u8, b: u8) {
        // WS2812 wants GRB order. The encoder is cheap (one IDF object) and
        // sends run at ~20 Hz max — recreated per frame for simplicity.
        let enc = match SimpleEncoder::with_config(
            Ws2812Encoder::new(),
            &SimpleEncoderConfig {
                min_chunk_size: 8,
                ..Default::default()
            },
        ) {
            Ok(e) => e,
            Err(e) => {
                warn!("status-led: encoder alloc failed: {e:?}");
                return;
            }
        };
        let _ = self
            .tx
            .send_and_wait(enc, &[g, r, b], &TransmitConfig::default());
    }
}

/// Bytes → RMT symbols. 1 byte = 8 symbols; position tracks bytes consumed.
struct Ws2812Encoder {
    zero: Symbol,
    one: Symbol,
}

impl Ws2812Encoder {
    fn new() -> Self {
        // T0H 400 ns + T0L 900 ns / T1H 800 ns + T1L 500 ns (spec ±150 ns).
        let sym = |hi: u16, lo: u16| {
            Symbol::new(
                Pulse::new(PinState::High, PulseTicks::new(hi).unwrap()),
                Pulse::new(PinState::Low, PulseTicks::new(lo).unwrap()),
            )
        };
        Self {
            zero: sym(4, 9),
            one: sym(8, 5),
        }
    }
}

impl EncoderCallback for Ws2812Encoder {
    type Item = u8;

    fn encode(&mut self, data: &[u8], buf: &mut SymbolBuffer<'_>) -> Result<(), NotEnoughSpace> {
        let mut byte_pos = buf.position() / 8;
        while byte_pos < data.len() && buf.remaining() >= 8 {
            let byte = data[byte_pos];
            let mut syms = [Symbol::new(Pulse::zero(), Pulse::zero()); 8];
            for (i, s) in syms.iter_mut().enumerate() {
                *s = if byte & (0x80 >> i) != 0 { self.one } else { self.zero };
            }
            buf.write_all(&syms)?;
            byte_pos += 1;
        }
        if byte_pos == data.len() {
            Ok(())
        } else {
            Err(NotEnoughSpace)
        }
    }
}

// ---------------------------------------------------------------------------
// Renderer: Status + Events → color, ~20 Hz refresh on a low-priority task
// ---------------------------------------------------------------------------

/// State → (r, g, b). Kept dim — the onboard WS2812 is blinding at 255.
fn color_for(now: u32) -> (u8, u8, u8) {
    if pending(&OVERLOAD_UNTIL_MS, now) {
        return (14, 6, 0); // amber pulse — shed/overload
    }
    if pending(&MIDI_UNTIL_MS, now) {
        return (10, 9, 14); // brief white flash — note activity
    }
    match STATUS.load(Ordering::Relaxed) {
        s if s == Status::Ready as u8 => (0, 14, 0),          // dim green
        s if s == Status::Error as u8 => (16, 0, 0),          // red
        _ => {
            if (now / 500) % 2 == 0 {
                (0, 0, 16) // Booting — blue 500 ms square blink
            } else {
                (0, 0, 0)
            }
        }
    }
}

fn render_loop(led: &mut impl Indicator) -> ! {
    let mut last = (0xFF, 0xFF, 0xFF); // force a first write
    loop {
        let rgb = color_for(now_ms());
        if rgb != last {
            led.set_rgb(rgb.0, rgb.1, rgb.2);
            last = rgb;
        }
        esp_idf_hal::delay::FreeRtos::delay_ms(REFRESH_MS);
    }
}

/// Bring up the status LED. Returns silently (with a warn) on failure —
/// the indicator is a nicety, never a boot blocker.
pub fn init(pin: impl OutputPin + Send + 'static) {
    match Ws2812::new(pin) {
        Ok(mut led) => {
            if let Err(e) = std::thread::Builder::new()
                .stack_size(4 * 1024)
                .name("status-led".into())
                .spawn(move || render_loop(&mut led))
            {
                warn!("status-led: spawn failed: {e}");
            }
        }
        Err(e) => warn!("status-led: init failed: {e:?}"),
    }
}
