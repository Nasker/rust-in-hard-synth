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
espflash flash -p /dev/ttyACM0 target/xtensa-esp32s3-espidf/release/rinsynth-esp32 \
    --partition-table partitions.csv
espflash monitor -p /dev/ttyACM0            # or any 115200 serial reader
```

## Status

- [x] Phase 1 — firmware boots (16 KB main-task stack; the 3.5 K default
      crashes under Rust logging), I2S TX verified: 2 s silence → 440 Hz
      stereo sine, zero write errors
- [x] Phase 2 — `rinsynth-model` extracted to
      `../RustInSynth/crates/rinsynth-model` (workspace crate shared with
      the desktop host: param spec, CC map, preset schema, mod matrix,
      factory bank). Boot runs `model_smoke()`: param/CC counts, preset
      serde roundtrip, `legacy_lfo_slot`, factory bank, A4=440 sanity
- [x] Phase 3 — `rinsynth-dsp` renders a real arpeggio on hardware
      (8-voice `VoiceManager`, factory-Init preset, LFO on cutoff).

### Render architecture

The engine runs **internally at 22.05 kHz** and the firmware 2x-upsamples
to the 44.1 kHz I2S stream with a Catmull-Rom midpoint interpolator
(`src/main.rs`). Rationale: per-voice cost on Xtensa measured ~1500
cycles/voice-sample (in-order core + FPU latency chains + `dyn`
dispatch) — half-rate halves it, and 3-voice blocks dropped from
7.3 ms to 3.8 ms vs the 5.8 ms deadline. Trade-off: internal Nyquist
is 11 kHz (brighter patches may alias); revisit with static dispatch
or dual-core voice splitting if needed.

Telemetry: `rinsynth_dsp::profile` reports per-stage cycles/sample
(env/ctrl/osc/filter) via `xthal_get_ccount` once per second.

## Known toolchain landmine

The esp 1.97 toolchain's LLVM-21 Xtensa backend cannot lower aggregate
constant-pool entries — any serde `f32` `Deserialize` impl dies with
`Cannot select XtensaISD::PCREL_WRAPPER` (esp-rs/rust#277). Fixed upstream
(llvm-project@39a59933) but the released toolchains carrying it (1.98.x)
have a different global-address miscompile — so the model crate is built
with `opt-level = 0` in this firmware's profile (see `Cargo.toml`). The
model is not on the audio hot path; revisit when a clean toolchain lands.
