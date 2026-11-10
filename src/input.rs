//! Physical input drivers: PCNT quadrature rotary encoder + switch (GPIO 38/39/40)
//! and four FX kill buttons (GPIO 41, 42, 2, 1 — active-low, wired to GND).
//!
//! Single responsibility: turn raw hardware signals into debounced, jitter-free
//! [`InputFrame`] snapshots at the UI frame rate (~25 Hz).
//!
//! Confirmed wiring:
//! - FX buttons (L→R):         `GPIO 41`, `GPIO 42`, `GPIO 2`, `GPIO 1`
//!   (to GND, internal pull-ups; displaced the never-enabled pots on 1/2)
//! - Rotary Encoder CLK:       `GPIO 38`
//! - Rotary Encoder DT:        `GPIO 39`
//! - Rotary Encoder Switch:    `GPIO 40` (active-low with internal pull-up)

use core::time::Duration;
use esp_idf_hal::gpio::{Gpio1, Gpio2, Gpio38, Gpio39, Gpio40, Gpio41, Gpio42, Input, PinDriver, Pull};
use esp_idf_hal::pcnt::config::{
    ChannelConfig, ChannelEdgeAction, ChannelLevelAction, GlitchFilterConfig, UnitConfig,
};
use esp_idf_hal::pcnt::PcntUnitDriver;
use esp_idf_sys::{esp_timer_get_time, gpio_pullup_en, EspError};
use log::info;

/// Debounced, smoothed input snapshot produced once per UI tick (~40 ms).
#[derive(Clone, Copy, Default)]
pub struct InputFrame {
    /// Detent steps since last frame (+ = clockwise, - = counter-clockwise).
    /// Always 1:1 — use for discrete navigation (presets, menus, cursors).
    pub encoder_delta: i32,
    /// Speed-scaled detent steps for continuous parameters: ×1 for slow
    /// turns, ×2 under ~120 ms/detent, ×4 under ~80 ms, ×8 under ~45 ms
    /// or when several detents land in one frame. Always real steps.
    pub param_delta: i32,
    /// True on the frame a short press (<500 ms) is confirmed — dispatched
    /// ~`DBL_CLICK_MS` after release so a double-click can be caught first.
    pub encoder_click: bool,
    /// True on the frame two short clicks land within `DBL_CLICK_MS`
    /// (Wi-Fi radio defeat for the RF-EMI A/B test — no click fires too).
    pub encoder_dbl_click: bool,
    /// True on the frame a hold crosses 500 ms (acts as BACK / mode toggle).
    pub encoder_long_press: bool,
    /// Debounced FX kill-button presses, edge-triggered once per actuation.
    /// Raw GPIO order [41, 42, 2, 1]; the UI reverses this for the mirrored mounting.
    pub button_press: [bool; 4],
    /// Current physical pot positions in 0..=100% — no pots wired (GPIO 1/2
    /// host the FX buttons); kept at 0 for future re-wire on new pins.
    pub pot_pct: [u8; 2],
    /// `Some(new_pct)` when a pot intentionally moved outside the deadband this frame.
    pub pot_moved: [Option<u8>; 2],
}

/// Abstract input source so `UiState` never touches ADC or PCNT registers.
pub trait InputSource: Send {
    fn poll(&mut self) -> InputFrame;
}

/// One debounced FX kill button — active-low (wired to GND, internal
/// pull-up), edge-triggered with a re-trigger lockout that absorbs any
/// contact bounce still visible across the ~40 ms poll frames.
struct FxButton {
    pin: PinDriver<'static, Input>,
    was_low: bool,
    last_fire_ms: u32,
}

impl FxButton {
    /// Min gap between two reported presses — covers multi-frame bounce
    /// bursts and prevents accidental double-toggles on a sloppy tap.
    const REFIRE_MS: u32 = 200;
}

const LONG_PRESS_MS: u32 = 480;

/// Max gap between two short clicks that counts as a double-click.
/// Single clicks are deferred by this window so the double gesture
/// doesn't also trigger the normal click action.
const DBL_CLICK_MS: u32 = 400;

/// No potentiometers are wired — GPIO 1/2 now host FX kill buttons. Keep
/// false: restoring pots needs NEW pins plus the ADC path (see git history).
pub const POTS_ENABLED: bool = false;

/// Hardware input bundle: PCNT encoder + switch + the four FX kill buttons.
pub struct PhysicalInputs<'d> {
    pcnt: PcntUnitDriver<'d>,
    last_pcnt_count: i32,
    rem_pulses: i32,
    last_detent_ms: u32,
    sw: PinDriver<'d, Input>,
    sw_down: bool,
    sw_down_ms: u32,
    sw_long_fired: bool,
    /// A released short click waiting out the double-click window before
    /// being dispatched as `encoder_click` (cancelled by a 2nd click).
    pending_click: bool,
    pending_click_ms: u32,
    /// Raw FX button inputs on GPIO 41, 42, 2, 1. Logical FX assignment
    /// belongs to the UI; it currently reverses these indices.
    buttons: [FxButton; 4],
}

fn now_ms() -> u32 {
    unsafe { (esp_timer_get_time() / 1_000) as u32 }
}

impl PhysicalInputs<'static> {
    pub fn new(
        btn0_gpio41: Gpio41<'static>,
        btn1_gpio42: Gpio42<'static>,
        btn2_gpio2: Gpio2<'static>,
        btn3_gpio1: Gpio1<'static>,
        enc_clk_gpio38: Gpio38<'static>,
        enc_dt_gpio39: Gpio39<'static>,
        enc_sw_gpio40: Gpio40<'static>,
    ) -> Result<Self, EspError> {
        // 1. Hardware PCNT quadrature decoder on GPIO 38 (CLK) + GPIO 39 (DT)
        let mut pcnt = PcntUnitDriver::new(&UnitConfig {
            low_limit: -16384,
            high_limit: 16384,
            ..Default::default()
        })?;
        pcnt.set_glitch_filter(Some(&GlitchFilterConfig {
            max_glitch: Duration::from_nanos(1_000),
            ..Default::default()
        }))?;
        {
            let ch = pcnt.add_channel(
                Some(enc_clk_gpio38),
                Some(enc_dt_gpio39),
                &ChannelConfig::default(),
            )?;
            ch.set_edge_action(ChannelEdgeAction::Increase, ChannelEdgeAction::Decrease)?;
            ch.set_level_action(ChannelLevelAction::Keep, ChannelLevelAction::Inverse)?;
        }
        // Ensure internal pull-ups are enabled on CLK (38) and DT (39).
        unsafe {
            gpio_pullup_en(38);
            gpio_pullup_en(39);
        }
        pcnt.enable()?;
        pcnt.clear_count()?;
        pcnt.start()?;

        // 2. Encoder push-switch on GPIO 40 (active-low, internal pull-up)
        let sw = PinDriver::input(enc_sw_gpio40, Pull::Up)?;

        // 3. FX kill buttons on GPIO 41, 42, 2, 1 (active-low, internal
        //    pull-ups) — physical left→right = FX index [DIST,CHRS,DLY,REV].
        fn mk_btn<P: esp_idf_hal::gpio::InputPin + 'static>(pin: P) -> Result<FxButton, EspError> {
            Ok(FxButton {
                pin: PinDriver::input(pin, Pull::Up)?,
                was_low: false,
                last_fire_ms: 0,
            })
        }
        let buttons = [
            mk_btn(btn0_gpio41)?,
            mk_btn(btn1_gpio42)?,
            mk_btn(btn2_gpio2)?,
            mk_btn(btn3_gpio1)?,
        ];

        let this = Self {
            pcnt,
            last_pcnt_count: 0,
            rem_pulses: 0,
            last_detent_ms: 0,
            sw,
            sw_down: false,
            sw_down_ms: 0,
            sw_long_fired: false,
            pending_click: false,
            pending_click_ms: 0,
            buttons,
        };
        info!("input: ready — Encoder(GPIO38/39/40) FXBtns(GPIO41/42/2/1)");
        Ok(this)
    }
}

impl InputSource for PhysicalInputs<'static> {
    fn poll(&mut self) -> InputFrame {
        let now = now_ms();

        // 1. Poll PCNT quadrature counter
        let raw_count = self.pcnt.get_count().unwrap_or(self.last_pcnt_count);
        let diff = raw_count - self.last_pcnt_count;
        self.last_pcnt_count = raw_count;

        // Standard encoders produce 2 edges per detent on a single edge-channel
        // (rising + falling of CLK). Accumulate so both 1-edge and 2-edge
        // encoders feel responsive without losing half-steps.
        // The PCNT decode counts clockwise as negative on this encoder
        // (CLK leads DT); negate so +delta = clockwise = "next" in the UI.
        self.rem_pulses += diff;
        let encoder_delta = if self.rem_pulses.abs() >= 2 {
            let steps = self.rem_pulses / 2;
            self.rem_pulses -= steps * 2;
            -steps
        } else {
            0
        };

        // Velocity-sensitive scaling for continuous params: several detents
        // inside one ~40 ms frame, or detents arriving on back-to-back
        // frames, mean a fast spin. Multiplied steps stay on real values.
        // Curve: 1x slow -> 2x <120 ms -> 4x <80 ms -> 8x <45 ms per detent.
        let param_delta = if encoder_delta != 0 {
            let dt = now.wrapping_sub(self.last_detent_ms);
            self.last_detent_ms = now;
            let mult = if encoder_delta.abs() > 2 || dt <= 45 {
                8
            } else if encoder_delta.abs() > 1 || dt <= 80 {
                4
            } else if dt <= 120 {
                2
            } else {
                1
            };
            encoder_delta * mult
        } else {
            0
        };

        // 2. Poll encoder push switch (active-low). Single clicks are
        // deferred by DBL_CLICK_MS so a second click can be recognized as
        // a double-click instead of firing the click action first.
        let pressed = self.sw.is_low();
        let mut encoder_click = false;
        let mut encoder_dbl_click = false;
        let mut encoder_long_press = false;

        if pressed {
            if !self.sw_down {
                self.sw_down = true;
                self.sw_down_ms = now;
                self.sw_long_fired = false;
            } else if !self.sw_long_fired
                && now.wrapping_sub(self.sw_down_ms) >= LONG_PRESS_MS
            {
                self.sw_long_fired = true;
                encoder_long_press = true;
                self.pending_click = false; // hold during window aborts the dbl
            }
        } else if self.sw_down {
            self.sw_down = false;
            if !self.sw_long_fired && now.wrapping_sub(self.sw_down_ms) >= 25 {
                if self.pending_click
                    && now.wrapping_sub(self.pending_click_ms) <= DBL_CLICK_MS
                {
                    // Second click inside the window → double-click, and
                    // neither click dispatches a normal `encoder_click`.
                    encoder_dbl_click = true;
                    self.pending_click = false;
                } else {
                    self.pending_click = true;
                    self.pending_click_ms = now;
                }
            }
        }

        // Window expired with no second click → dispatch the deferred click.
        if self.pending_click && now.wrapping_sub(self.pending_click_ms) > DBL_CLICK_MS {
            self.pending_click = false;
            encoder_click = true;
        }

        // 3. FX kill buttons — fire once on the high→low edge; a 200 ms
        // re-fire lockout absorbs any bounce surviving across poll frames.
        let mut button_press = [false; 4];
        for (i, b) in self.buttons.iter_mut().enumerate() {
            let low = b.pin.is_low();
            if low && !b.was_low && now.wrapping_sub(b.last_fire_ms) >= FxButton::REFIRE_MS {
                b.last_fire_ms = now;
                button_press[i] = true;
            }
            b.was_low = low;
        }

        InputFrame {
            encoder_delta,
            param_delta,
            encoder_click,
            encoder_dbl_click,
            encoder_long_press,
            button_press,
            pot_pct: [0, 0],
            pot_moved: [None, None],
        }
    }
}
