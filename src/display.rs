//! 128×64 monochrome OLED framebuffer, font/glyph primitives, and SH1106 SPI driver.
//!
//! Single responsibility split:
//! - [`Canvas`]: pure 1 KB page-packed framebuffer (`[u8; 1024]`) + zero-alloc
//!   pixel/line/rect/text/bitmap primitives. No hardware dependencies.
//! - [`Display`]: minimal sink trait (`flush(&mut self, &Canvas)`) so the UI
//!   renderer never knows about SPI, GPIO, or SH1106 page addressing.
//! - [`Sh1106Spi`]: concrete 4-wire SPI + RES driver for the 1.3" 128×64 OLED
//!   (absorbs the SH1106 +2 column RAM offset and 8-page flush protocol).
//!
//! Pinout (confirmed on hardware):
//!   CLK = GPIO 9, MOSI = GPIO 10, RES = GPIO 11, DC = GPIO 12, CS = GPIO 13.

use esp_idf_hal::delay::FreeRtos;
use esp_idf_hal::gpio::{AnyInputPin, Output, OutputPin, PinDriver};
use esp_idf_hal::spi::config::{Config as SpiConfig, DriverConfig as SpiDriverConfig, MODE_0};
use esp_idf_hal::spi::{Dma, SpiAnyPins, SpiDeviceDriver, SpiDriver};
use esp_idf_hal::units::FromValueType;
use esp_idf_sys::EspError;

pub const WIDTH: usize = 128;
pub const HEIGHT: usize = 64;
pub const PAGES: usize = HEIGHT / 8;

/// Abstract monochrome display sink — the only boundary the UI task sees.
pub trait Display: Send {
    fn flush(&mut self, canvas: &Canvas);
}

/// Pure 128×64 monochrome framebuffer (8 horizontal pages × 128 vertical byte columns).
#[derive(Clone)]
pub struct Canvas {
    buf: [u8; WIDTH * PAGES],
}

impl Default for Canvas {
    fn default() -> Self {
        Self::new()
    }
}

impl Canvas {
    pub const fn new() -> Self {
        Self {
            buf: [0u8; WIDTH * PAGES],
        }
    }

    pub fn clear(&mut self) {
        self.buf.fill(0);
    }

    pub fn page_slice(&self, page: usize) -> &[u8] {
        let start = page * WIDTH;
        &self.buf[start..start + WIDTH]
    }

    #[inline]
    pub fn set_pixel(&mut self, x: i32, y: i32, on: bool) {
        if x < 0 || x >= WIDTH as i32 || y < 0 || y >= HEIGHT as i32 {
            return;
        }
        let ux = x as usize;
        let uy = y as usize;
        let idx = (uy >> 3) * WIDTH + ux;
        let bit = 1u8 << (uy & 7);
        if on {
            self.buf[idx] |= bit;
        } else {
            self.buf[idx] &= !bit;
        }
    }

    pub fn hline(&mut self, x: i32, y: i32, w: i32, on: bool) {
        for dx in 0..w.max(0) {
            self.set_pixel(x + dx, y, on);
        }
    }

    pub fn vline(&mut self, x: i32, y: i32, h: i32, on: bool) {
        for dy in 0..h.max(0) {
            self.set_pixel(x, y + dy, on);
        }
    }

    pub fn rect(&mut self, x: i32, y: i32, w: i32, h: i32, on: bool) {
        if w <= 0 || h <= 0 {
            return;
        }
        self.hline(x, y, w, on);
        self.hline(x, y + h - 1, w, on);
        self.vline(x, y, h, on);
        self.vline(x + w - 1, y, h, on);
    }

    pub fn fill_rect(&mut self, x: i32, y: i32, w: i32, h: i32, on: bool) {
        for dy in 0..h.max(0) {
            self.hline(x, y + dy, w, on);
        }
    }

    /// Draw an 8-pixel-wide bitmap where each byte in `rows` is one horizontal row
    /// (MSB = leftmost pixel).
    pub fn draw_icon8(&mut self, x: i32, y: i32, rows: &[u8], on: bool) {
        for (ry, &row) in rows.iter().enumerate() {
            for rx in 0..8 {
                if (row & (0x80 >> rx)) != 0 {
                    self.set_pixel(x + rx, y + ry as i32, on);
                }
            }
        }
    }

    /// Draw a single 5×7 ASCII character with integer `scale` (1 = 5×7, 2 = 10×14).
    pub fn draw_char(&mut self, x: i32, y: i32, ch: char, scale: i32, on: bool) {
        let cols = glyph_5x7(ch);
        let s = scale.max(1);
        for (col_idx, &bits) in cols.iter().enumerate() {
            for row_idx in 0..7 {
                if (bits & (1 << row_idx)) != 0 {
                    let px = x + (col_idx as i32) * s;
                    let py = y + row_idx * s;
                    if s == 1 {
                        self.set_pixel(px, py, on);
                    } else {
                        self.fill_rect(px, py, s, s, on);
                    }
                }
            }
        }
    }

    /// Draw an ASCII string with 1-column spacing (scaled by `scale`).
    pub fn draw_str(&mut self, x: i32, y: i32, text: &str, scale: i32, on: bool) {
        let s = scale.max(1);
        let step = 6 * s;
        for (i, ch) in text.chars().enumerate() {
            self.draw_char(x + (i as i32) * step, y, ch, s, on);
        }
    }

    /// Width in pixels of `text` at `scale`.
    pub fn str_width(text: &str, scale: i32) -> i32 {
        let n = text.chars().count() as i32;
        if n == 0 {
            0
        } else {
            let s = scale.max(1);
            n * 6 * s - s
        }
    }

    /// Draw an ASCII string horizontally centered on the 128-px display.
    pub fn draw_str_centered(&mut self, y: i32, text: &str, scale: i32, on: bool) {
        let w = Self::str_width(text, scale);
        let x = ((WIDTH as i32) - w) / 2;
        self.draw_str(x, y, text, scale, on);
    }
}

/// Compact 5×7 column-major ASCII font (LSB = top row).
const fn glyph_5x7(ch: char) -> [u8; 5] {
    let c = if ch >= 'a' && ch <= 'z' {
        (ch as u8 - b'a' + b'A') as char
    } else {
        ch
    };
    match c {
        ' ' => [0x00, 0x00, 0x00, 0x00, 0x00],
        '!' => [0x00, 0x00, 0x5F, 0x00, 0x00],
        '+' => [0x08, 0x08, 0x3E, 0x08, 0x08],
        '-' => [0x08, 0x08, 0x08, 0x08, 0x08],
        '.' => [0x00, 0x60, 0x60, 0x00, 0x00],
        '/' => [0x20, 0x10, 0x08, 0x04, 0x02],
        ':' => [0x00, 0x36, 0x36, 0x00, 0x00],
        '<' => [0x08, 0x14, 0x22, 0x41, 0x00],
        '>' => [0x00, 0x41, 0x22, 0x14, 0x08],
        '[' => [0x00, 0x7F, 0x41, 0x41, 0x00],
        ']' => [0x00, 0x41, 0x41, 0x7F, 0x00],
        '0' => [0x3E, 0x51, 0x49, 0x45, 0x3E],
        '1' => [0x00, 0x42, 0x7F, 0x40, 0x00],
        '2' => [0x42, 0x61, 0x51, 0x49, 0x46],
        '3' => [0x21, 0x41, 0x45, 0x4B, 0x31],
        '4' => [0x18, 0x14, 0x12, 0x7F, 0x10],
        '5' => [0x27, 0x45, 0x45, 0x45, 0x39],
        '6' => [0x3C, 0x4A, 0x49, 0x49, 0x30],
        '7' => [0x01, 0x71, 0x09, 0x05, 0x03],
        '8' => [0x36, 0x49, 0x49, 0x49, 0x36],
        '9' => [0x06, 0x49, 0x49, 0x29, 0x1E],
        'A' => [0x7E, 0x11, 0x11, 0x11, 0x7E],
        'B' => [0x7F, 0x49, 0x49, 0x49, 0x36],
        'C' => [0x3E, 0x41, 0x41, 0x41, 0x22],
        'D' => [0x7F, 0x41, 0x41, 0x22, 0x1C],
        'E' => [0x7F, 0x49, 0x49, 0x49, 0x41],
        'F' => [0x7F, 0x09, 0x09, 0x09, 0x01],
        'G' => [0x3E, 0x41, 0x49, 0x49, 0x7A],
        'H' => [0x7F, 0x08, 0x08, 0x08, 0x7F],
        'I' => [0x00, 0x41, 0x7F, 0x41, 0x00],
        'J' => [0x20, 0x40, 0x41, 0x3F, 0x01],
        'K' => [0x7F, 0x08, 0x14, 0x22, 0x41],
        'L' => [0x7F, 0x40, 0x40, 0x40, 0x40],
        'M' => [0x7F, 0x02, 0x0C, 0x02, 0x7F],
        'N' => [0x7F, 0x04, 0x08, 0x10, 0x7F],
        'O' => [0x3E, 0x41, 0x41, 0x41, 0x3E],
        'P' => [0x7F, 0x09, 0x09, 0x09, 0x06],
        'Q' => [0x3E, 0x41, 0x51, 0x21, 0x5E],
        'R' => [0x7F, 0x09, 0x19, 0x29, 0x46],
        'S' => [0x46, 0x49, 0x49, 0x49, 0x31],
        'T' => [0x01, 0x01, 0x7F, 0x01, 0x01],
        'U' => [0x3F, 0x40, 0x40, 0x40, 0x3F],
        'V' => [0x1F, 0x20, 0x40, 0x20, 0x1F],
        'W' => [0x3F, 0x40, 0x38, 0x40, 0x3F],
        'X' => [0x63, 0x14, 0x08, 0x14, 0x63],
        'Y' => [0x07, 0x08, 0x70, 0x08, 0x07],
        'Z' => [0x61, 0x51, 0x49, 0x45, 0x43],
        _ => [0x3E, 0x41, 0x5D, 0x55, 0x1E],
    }
}

// ---------------------------------------------------------------------------
// SH1106 / SSD1306 1.3" 4-wire SPI hardware driver
// ---------------------------------------------------------------------------

/// SH1106 RAM is 132 columns wide; visible 128-px panel starts at column 2.
const SH1106_COL_OFFSET: u8 = 2;

pub struct Sh1106Spi<'d> {
    spi: SpiDeviceDriver<'d, SpiDriver<'d>>,
    dc: PinDriver<'d, Output>,
    _res: PinDriver<'d, Output>,
    /// Shadow buffer per page for dirty-page diffing — unchanged pages cost
    /// zero SPI transactions and zero bus contention during audio rendering.
    prev: [[u8; WIDTH]; PAGES],
    first_flush: bool,
}

impl<'d> Sh1106Spi<'d> {
    pub fn new<SPI: SpiAnyPins + 'd>(
        spi: SPI,
        sclk: impl OutputPin + 'd,
        mosi: impl OutputPin + 'd,
        res: impl OutputPin + 'd,
        dc: impl OutputPin + 'd,
        cs: impl OutputPin + 'd,
    ) -> Result<Self, EspError> {
        let bus_cfg = SpiDriverConfig::new().dma(Dma::Auto(1024));
        // Polling mode avoids 16 ISR + FreeRTOS queue context switches per
        // frame (a 128-byte page @ 20 MHz finishes in ~51 µs inline on Core 0).
        let dev_cfg = SpiConfig::new()
            .baudrate(20.MHz().into())
            .data_mode(MODE_0)
            .write_only(true)
            .polling(true);

        let spi_dev = SpiDeviceDriver::new_single(
            spi,
            sclk,
            mosi,
            Option::<AnyInputPin>::None,
            Some(cs),
            &bus_cfg,
            &dev_cfg,
        )?;

        let mut res_pin = PinDriver::output(res)?;
        let dc_pin = PinDriver::output(dc)?;

        // Hardware reset pulse required by SH1106/SSD1306 on power-up.
        res_pin.set_high()?;
        FreeRtos::delay_ms(5);
        res_pin.set_low()?;
        FreeRtos::delay_ms(15);
        res_pin.set_high()?;
        FreeRtos::delay_ms(15);

        let mut drv = Self {
            spi: spi_dev,
            dc: dc_pin,
            _res: res_pin,
            prev: [[0u8; WIDTH]; PAGES],
            first_flush: true,
        };
        drv.init_panel()?;
        Ok(drv)
    }

    fn write_cmds(&mut self, cmds: &[u8]) -> Result<(), EspError> {
        self.dc.set_low()?;
        self.spi.write(cmds)
    }

    fn write_data(&mut self, data: &[u8]) -> Result<(), EspError> {
        self.dc.set_high()?;
        self.spi.write(data)
    }

    fn init_panel(&mut self) -> Result<(), EspError> {
        // Init sequence compatible with 1.3" SH1106 v1.0 (and 0.96" SSD1306).
        let init_seq: &[u8] = &[
            0xAE,       // Display OFF
            0xD5, 0x80, // Display clock divide ratio / oscillator freq
            0xA8, 0x3F, // Multiplex ratio = 64-1
            0xD3, 0x00, // Display offset = 0
            0x40,       // Start line = 0
            0xAD, 0x8B, // SH1106 internal DC-DC charge pump ON
            0x8D, 0x14, // SSD1306 charge pump ON (no-op on SH1106)
            0xA0,       // Segment remap normal (flipped 180° from 0xA1)
            0xC0,       // COM output scan normal (flipped 180° from 0xC8)
            0xDA, 0x12, // COM pins hardware configuration (alternative)
            0x81, 0xCF, // Contrast control
            0xD9, 0xF1, // Pre-charge period
            0xDB, 0x40, // VCOMH deselect level
            0x32,       // SH1106 pump voltage 8.0V
            0xA4,       // Entire display ON from RAM
            0xA6,       // Normal display (not inverted)
        ];
        self.write_cmds(init_seq)?;

        // Clear all 8 pages (including the 132-col SH1106 margin) before turning on.
        let zeros = [0u8; 132];
        for page in 0..8u8 {
            self.write_cmds(&[0xB0 | page, 0x00, 0x10])?;
            self.write_data(&zeros)?;
        }
        self.write_cmds(&[0xAF]) // Display ON
    }
}

impl Display for Sh1106Spi<'_> {
    fn flush(&mut self, canvas: &Canvas) {
        let col_lo = 0x00 | (SH1106_COL_OFFSET & 0x0F);
        let col_hi = 0x10 | ((SH1106_COL_OFFSET >> 4) & 0x0F);
        for page in 0..PAGES {
            let slice = canvas.page_slice(page);
            if !self.first_flush && self.prev[page] == slice {
                continue;
            }
            let cmd = [0xB0 | (page as u8), col_lo, col_hi];
            if self.write_cmds(&cmd).is_err() {
                return;
            }
            if self.write_data(slice).is_err() {
                return;
            }
            self.prev[page].copy_from_slice(slice);
        }
        self.first_flush = false;
    }
}
