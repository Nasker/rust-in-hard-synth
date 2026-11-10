//! ESP32-S3 synth composition root: acquire hardware and start task adapters.
//!
//! Prepare immutable metadata and start Core 1 audio before Wi-Fi fragments
//! internal SRAM. USB host installation stays on Core 0 to keep its ISR off
//! the audio core. DSP and control policies live in the host-testable library.

use esp_idf_hal::i2s::config::{
    Config, DataBitWidth, SlotMode, StdClkConfig, StdConfig, StdGpioConfig, StdSlotConfig,
};
use esp_idf_hal::i2s::I2sDriver;
use esp_idf_hal::peripherals::Peripherals;
use log::info;
use rinsynth_esp32::{audio::SAMPLE_RATE, control};

mod audio_task;
mod display;
mod input;
mod startup;
mod status;
mod ui;
mod usb_midi;
mod web;

fn main() -> anyhow::Result<()> {
    esp_idf_sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();
    info!(
        "rinsynth-esp32 {} — dsp engine render",
        env!("CARGO_PKG_VERSION")
    );
    let bank = startup::load_bank()?;
    let p = Peripherals::take()?;

    // Onboard WS2812 RGB (GPIO48, free per the pin budget) — blue blink
    // until the web server is up, then dim green.
    status::init(p.pins.gpio48);

    // 1.3" SPI OLED (SH1106/SSD1306): CLK=GPIO9, MOSI=GPIO10, RES=GPIO11,
    // DC=GPIO12, CS=GPIO13 + FX buttons=GPIO41/42/2/1 + Encoder=GPIO38/39/40.
    // Starts the animated SPLASH screen immediately on Core 0.
    ui::init(
        p.spi2,
        p.pins.gpio9,
        p.pins.gpio10,
        p.pins.gpio11,
        p.pins.gpio12,
        p.pins.gpio13,
        p.pins.gpio41,
        p.pins.gpio42,
        p.pins.gpio2,
        p.pins.gpio1,
        p.pins.gpio38,
        p.pins.gpio39,
        p.pins.gpio40,
    );
    let std_cfg = StdConfig::new(
        Config::default(),
        StdClkConfig::from_sample_rate_hz(SAMPLE_RATE),
        StdSlotConfig::philips_slot_default(DataBitWidth::Bits16, SlotMode::Stereo),
        StdGpioConfig::default(),
    );
    let modem = p.modem;
    let tx = I2sDriver::new_std_tx(
        p.i2s0,
        &std_cfg,
        p.pins.gpio6,       // BCK
        p.pins.gpio5,       // DOUT -> PCM5102 DIN
        Some(p.pins.gpio7), // MCLK
        p.pins.gpio4,       // LRCK/WS
    )?;

    // Prepare immutable metadata and start audio before Wi-Fi fragments
    // internal SRAM. Retain the 24 KB audio stack; remeasure its high-water
    // mark on hardware after changes to the runtime or buffer layout.
    control::presets::init(&bank);
    web::init_presets_json(&bank);
    startup::prewarm_dsp()?;
    audio_task::start(tx, bank);

    if let Err(e) = web::start(modem) {
        log::error!("web: failed to start SoftAP/HTTP server: {:?}", e);
        status::set(status::Status::Error);
    } else {
        status::set(status::Status::Ready);
    }

    // Install USB host on Core 0 (main task) so the USB OTG ISR lands on
    // Core 0 and never preempts the Core 1 audio render loop.
    usb_midi::start();

    // Main task idles from here on — keep it out of the audio path.
    loop {
        esp_idf_hal::delay::FreeRtos::delay_ms(60_000);
    }
}
