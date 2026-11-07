//! Physical input drivers: PCNT quadrature rotary encoder + switch (GPIO 38/39/40)
//! and ADC1 potentiometers with EMA smoothing + hysteresis deadband (GPIO 2, GPIO 1).
//!
//! Single responsibility: turn raw hardware signals into debounced, jitter-free
//! [`InputFrame`] snapshots at the UI frame rate (~25 Hz).
//!
//! Confirmed wiring:
//! - Pot 0 (leftmost column):  `GPIO 2` (ADC1_CH1)
//! - Pot 1 (second column):    `GPIO 1` (ADC1_CH0)
//! - Rotary Encoder CLK:       `GPIO 38`
//! - Rotary Encoder DT:        `GPIO 39`
//! - Rotary Encoder Switch:    `GPIO 40` (active-low with internal pull-up)

use core::time::Duration;
use esp_idf_hal::adc::attenuation::DB_12;
use esp_idf_hal::adc::oneshot::config::AdcChannelConfig;
use esp_idf_hal::adc::oneshot::{AdcChannelDriver, AdcDriver};
use esp_idf_hal::adc::ADC1;
use esp_idf_hal::gpio::{Gpio1, Gpio2, Gpio38, Gpio39, Gpio40, Input, PinDriver, Pull};
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
    /// Current physical pot positions in 0..=100% (`[pot0_left_gpio2, pot1_gpio1]`).
    pub pot_pct: [u8; 2],
    /// `Some(new_pct)` when a pot intentionally moved outside the deadband this frame.
    pub pot_moved: [Option<u8>; 2],
}

/// Abstract input source so `UiState` never touches ADC or PCNT registers.
pub trait InputSource: Send {
    fn poll(&mut self) -> InputFrame;
}

/// EMA + hysteresis state for one analog potentiometer (12-bit 0..4095).
struct PotFilter {
    ema_raw: i32,
    committed_raw: i32,
    pct: u8,
    initialized: bool,
}

impl PotFilter {
    /// ~1.4% of full scale (58 / 4095) required to break out of rest — kills
    /// Wi-Fi TX rail noise while feeling instantaneous to a finger turn.
    const HYSTERESIS_RAW: i32 = 58;
    /// Clamp endpoints slightly inside 0..4095 so every pot reaches 0% and 100%.
    const MIN_RAW: i32 = 48;
    const MAX_RAW: i32 = 4030;

    const fn new() -> Self {
        Self {
            ema_raw: 0,
            committed_raw: 0,
            pct: 0,
            initialized: false,
        }
    }

    fn raw_to_pct(raw: i32) -> u8 {
        let clamped = raw.clamp(Self::MIN_RAW, Self::MAX_RAW) - Self::MIN_RAW;
        let span = Self::MAX_RAW - Self::MIN_RAW;
        ((clamped * 100 + span / 2) / span).clamp(0, 100) as u8
    }

    fn update(&mut self, sample_raw: u16) -> Option<u8> {
        let r = sample_raw as i32;
        if !self.initialized {
            self.ema_raw = r;
            self.committed_raw = r;
            self.pct = Self::raw_to_pct(r);
            self.initialized = true;
            return None;
        }
        // 50/50 EMA at 25 Hz (~40 ms time constant)
        self.ema_raw = (self.ema_raw + r) / 2;
        if (self.ema_raw - self.committed_raw).abs() >= Self::HYSTERESIS_RAW {
            self.committed_raw = self.ema_raw;
            let new_pct = Self::raw_to_pct(self.committed_raw);
            if new_pct != self.pct {
                self.pct = new_pct;
                return Some(new_pct);
            }
        }
        None
    }
}

const LONG_PRESS_MS: u32 = 480;

/// Max gap between two short clicks that counts as a double-click.
/// Single clicks are deferred by this window so the double gesture
/// doesn't also trigger the normal click action.
const DBL_CLICK_MS: u32 = 400;

/// Flip to `true` when reliable potentiometers are wired back in — the ADC
/// channels and filters are still constructed below, so this is the only
/// switch needed to re-enable Pot 0 (GPIO 2) and Pot 1 (GPIO 1).
pub const POTS_ENABLED: bool = false;

/// Self-owning wrapper: holds `AdcDriver` in a `Box::leak` static reference so
/// multiple `AdcChannelDriver`s can borrow it without lifetime self-reference.
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
    adc: &'static AdcDriver<'static, esp_idf_hal::adc::ADCU1>,
    pot0_ch: AdcChannelDriver<
        'static,
        esp_idf_hal::adc::ADCCH1<esp_idf_hal::adc::ADCU1>,
        &'static AdcDriver<'static, esp_idf_hal::adc::ADCU1>,
    >,
    pot1_ch: AdcChannelDriver<
        'static,
        esp_idf_hal::adc::ADCCH0<esp_idf_hal::adc::ADCU1>,
        &'static AdcDriver<'static, esp_idf_hal::adc::ADCU1>,
    >,
    pot_filters: [PotFilter; 2],
}

fn now_ms() -> u32 {
    unsafe { (esp_timer_get_time() / 1_000) as u32 }
}

impl PhysicalInputs<'static> {
    pub fn new(
        adc1: ADC1<'static>,
        pot0_gpio2: Gpio2<'static>,
        pot1_gpio1: Gpio1<'static>,
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

        // 3. ADC1 oneshot channels for Pot 0 (GPIO 2, left) and Pot 1 (GPIO 1, second)
        let adc: &'static AdcDriver<'static, _> = Box::leak(Box::new(AdcDriver::new(adc1)?));
        let ch_cfg = AdcChannelConfig {
            attenuation: DB_12,
            ..Default::default()
        };
        let pot0_ch = AdcChannelDriver::new(adc, pot0_gpio2, &ch_cfg)?;
        let pot1_ch = AdcChannelDriver::new(adc, pot1_gpio1, &ch_cfg)?;

        let mut this = Self {
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
            adc,
            pot0_ch,
            pot1_ch,
            pot_filters: [PotFilter::new(), PotFilter::new()],
        };
        // Prime initial pot positions so boot doesn't fire synthetic pot moves.
        let _ = this.sample_pots();
        info!(
            "input: ready — Encoder(GPIO38/39/40) Pots(GPIO2/GPIO1) {}",
            if POTS_ENABLED { "enabled" } else { "DISABLED" }
        );
        Ok(this)
    }

    fn sample_pots(&mut self) -> [Option<u8>; 2] {
        // 4x oversample average per channel to suppress high-frequency switching noise
        let mut sum0 = 0u32;
        let mut sum1 = 0u32;
        for _ in 0..4 {
            sum0 += self.adc.read_raw(&mut self.pot0_ch).unwrap_or(0) as u32;
            sum1 += self.adc.read_raw(&mut self.pot1_ch).unwrap_or(0) as u32;
        }
        let m0 = self.pot_filters[0].update((sum0 / 4) as u16);
        let m1 = self.pot_filters[1].update((sum1 / 4) as u16);
        [m0, m1]
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

        // 3. Poll ADC1 pots (disabled until reliable pots are wired — see
        // POTS_ENABLED). Reporting None keeps every pot-driven param frozen.
        let pot_moved = if POTS_ENABLED {
            self.sample_pots()
        } else {
            [None, None]
        };

        InputFrame {
            encoder_delta,
            param_delta,
            encoder_click,
            encoder_dbl_click,
            encoder_long_press,
            pot_pct: [self.pot_filters[0].pct, self.pot_filters[1].pct],
            pot_moved,
        }
    }
}
