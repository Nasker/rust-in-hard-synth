# rinsynth-esp32

ESP32-S3 firmware port of [RustInSynth](https://github.com/Nasker/RustInSynth) —
a standalone hardware synth driven by the RustInSynth DSP engine, with a
browser-based editor served by the device itself. Companion to the C-based
serviSynth project; same board, same PCM5102 DAC wiring.

Reference plan: `RustInSynth_ESP32S3_Port_Plan.md`.

## Hardware

- ESP32-S3 with octal PSRAM (R8, 8 MB) and 16 MB flash
- PCM5102 DAC (I2S0, standard Philips, 44.1 kHz / 16-bit stereo):

  | Signal | GPIO |
  |--------|------|
  | BCK    | 6    |
  | LRCK   | 4    |
  | DIN    | 5    |
  | MCLK   | 7    |

- Flash/console via the `COM` port (UART bridge, `/dev/ttyACM0`)

## Toolchain

One-time setup:

```sh
cargo install espup espflash ldproxy
espup install            # Xtensa 'esp' toolchain
. ~/export-esp.sh        # env vars (LIBCLANG_PATH etc.) every session
```

The build reuses the system ESP-IDF via `ESP_IDF` in `.cargo/config.toml`
(esp-idf-sys `native` feature) — currently `/home/oscarmartinez/esp/v5.5.1/esp-idf`.

## Build & flash

```sh
. ~/export-esp.sh
cargo build --release
espflash flash --port /dev/ttyACM0 target/xtensa-esp32s3-espidf/release/rinsynth-esp32
espflash monitor --port /dev/ttyACM0        # or any 115200 serial reader
```

## Status — Phase 1 complete

- [x] ESP-IDF Rust firmware boots on the board (16 KB main-task stack needed —
      Rust logging overflows the 3.5 K default)
- [x] I2S TX: 2 s silence, then 440 Hz stereo sine, underrun counter at zero
- [ ] next: Phase 2 — extract `rinsynth-model` (param spec, preset schema)
  from the desktop repo, then Phase 3 DSP extraction and the
  incremental feature ladder
