# RustInHardSynth (`rinsynth-esp32`)

<p align="center">
  <img src="RustInSynth.png" alt="Rust In Hard Synth" width="480">
</p>

**RustInHardSynth — A Standalone ESP32-S3 Virtual Analog Synthesizer in Rust**

Hardware firmware port of [RustInSynth](https://github.com/Nasker/RustInSynth) running natively on an **ESP32-S3** microcontroller with a **PCM5102 I²S DAC**, **USB-MIDI Host**, and a **self-hosted Wi-Fi Web UI** (SoftAP + Station mode with captive portal).

---

## Features

**Sound Engine (`rinsynth-dsp` EmbeddedEngine)**
- **4-oscillator bank per voice** with 5 waveforms (Sine, Triangle, Saw, Square/Pulse with PW, Noise) + PolyBLEP anti-aliasing. **OSC 4** doubles as an audio-rate modulator (**FM / Ring Mod**)
- **Zavalishin TPT State-Variable Filter (SVF)** — Low-Pass, High-Pass, Band-Pass, and Notch modes with 6-octave filter envelope sweep and automatic zero-overhead transparent bypass when wide open
- **3 ADSR envelopes per voice** — Amplitude (VCA), Filter (VCF), and auxiliary **Env 3** matrix source; preset changes update ADSR times in place without cutting active notes
- **2 LFOs** — 5 waveforms each (Sine, Triangle, Square, Saw, Sample & Hold), free-running (`0.05–20 Hz`) or **tempo-synced** (`4 Bars` down to `1/32`, including dotted `1/8D` and `1/16D`)
- **6-voice polyphonic pool** (default user cap of 5, adjustable `1–6` at runtime) with bass-note protection on voice stealing and portamento (glide)
- **Adaptive CPU Governor** — monitors per-block render time against the `5.805 ms` DMA deadline; gracefully fast-fades (`~70 ms`) the quietest excess voice under extreme patches instead of crackling, and restores polyphony when headroom returns
- **Runtime Sample-Rate A/B (`44.1 kHz` / `22.05 kHz`)** — live switchable between full-rate 44.1 kHz and 2× Catmull-Rom interpolated half-rate for 2× DSP headroom without retriggering held notes

**Modulation**
- **8-slot Modulation Matrix** evaluated at control rate (every 64 samples), plus audio-rate **OSC 4 → Pitch (FM)** and **OSC 4 → Level/Amp (RM)**
- **Sources**: LFO 1–3, Amp Env, Filter Env, Env 3–4, Velocity, Key Track, Mod Wheel, Aftertouch, Pitch Bend, Osc 4 (FM/RM), Constant
- **Destinations**: All Pitch, Osc 1–3 Pitch/Level/Pan/PW, Filter Cutoff/Resonance, Amplitude, LFO 1 Rate
- **Tempo Sync** — Global BPM (`20–300 BPM`), browser **TAP** tempo, and automatic **USB-MIDI Timing Clock (`0xF8`, 24 PPQN)** tracking lock LFO 1, LFO 2, and Ping-Pong Delay to musical subdivisions

**Effects & Stereo**
- **True stereo path** with equal-power per-oscillator panning and stereo width control
- **Stereo Effects Chain** (`Distortion → Chorus → Ping-Pong Delay → Reverb`):
  - **Distortion**: 4 algorithms (*Tube* Padé waveshaper, *Fuzz* asymmetric clip, *Bitcrush*, *Wavefold* LUT) with Drive and Mix
  - **Chorus**: Quadrature stereo modulated delay line
  - **Ping-Pong Delay**: Stereo cross-feedback delay with free time (`0.02–1.5 s`) or **Tempo Sync** (`4 Bars` – `1/32`)
  - **Reverb**: Schroeder/Freeverb stereo reverb (4 parallel combs + 2 series allpasses in internal SRAM)
- **Stereo Bus Peak Compressor/Limiter** + final soft clipper

**Connectivity & Control**
- **Native USB-MIDI Host (USB OTG port)** — plug-and-play with class-compliant USB MIDI keyboards/sequencers (USB hub support enabled). Handles Note On/Off, Pitch Bend, Aftertouch, Program Change, 30+ MIDI CCs, and MIDI Timing Clock (`0xF8`)
- **Self-Hosted Web UI over Wi-Fi** — always-on SoftAP (`rinsynth-esp32` @ `http://192.168.71.1`) with captive-portal DNS + simultaneous **AP+STA Mixed Mode** (joins home Wi-Fi with credentials saved in NVS)
  - Four tabs: **SYNTH** | **ROUTING MATRIX** | **PRESETS** (85 factory presets across themed shelves) | **WIFI / NET**
  - Built-in 2-octave on-screen touch/QWERTY keyboard (`Z..M` / `Q..I`)
  - Low-latency batched WebSocket transport (`/ws`) with server-side ping/pong watchdog and HTTP `/cmd` fallback
- **Hardware UI Ready** — lock-free multi-producer command ring (`web::push_cmd`) and atomic state readouts (`current_bpm`, `current_preset_index`, `user_voice_cap`) ready for a Core 0 physical control panel (SSD1306/SH1106 OLED, PCNT quadrature encoder, ADC1 pots, tactile buttons)

---

## Architecture

```
            Core 0 (Comms & UI)                        Core 1 (Hard Real-Time DSP)
 ┌────────────────────────────────────────┐     ┌────────────────────────────────────────┐
 │ USB OTG Host (MIDI IN + 0xF8 Clock)    │     │ EmbeddedEngine (6 voices × 4 PolyBLEP) │
 │   └─▶ Lock-Free SPSC MIDI Ring ────────┼────▶│   ├─▶ Control-Rate ModMatrix (÷64)     │
 │                                        │     │   ├─▶ Zavalishin TPT SVF (w/ bypass)   │
 │ Wi-Fi SoftAP + STA / Captive DNS       │     │   ├─▶ Dist → Chorus → Delay → Reverb   │
 │ HTTP + Batched WebSocket Server (/ws)  │     │   ├─▶ Bus Limiter + CPU Voice Governor │
 │   └─▶ Lock-Free Command Ring ──────────┼────▶│   └─▶ I²S0 DMA TX (44.1 kHz / 256 fr)  │
 └────────────────────────────────────────┘     └───────────────────┬────────────────────┘
                                                                    │
                                                                    ▼
                                                          PCM5102 Stereo DAC
```

---

## Hardware & Pinout

- **MCU**: ESP32-S3 with **16 MB Flash** and **8 MB Octal PSRAM** (240 MHz dual-core Xtensa LX7)
- **DAC**: PCM5102 I²S stereo DAC (`44.1 kHz`, 16-bit stereo, 256-frame DMA blocks ≈ `5.805 ms` deadline)

| Function | Signal | ESP32-S3 GPIO |
|----------|--------|---------------|
| **PCM5102 I²S** | `BCK` | `GPIO 6` |
| **PCM5102 I²S** | `LRCK / WS` | `GPIO 4` |
| **PCM5102 I²S** | `DIN (DOUT)` | `GPIO 5` |
| **PCM5102 I²S** | `MCLK (256×fs)` | `GPIO 7` |
| **USB-MIDI Host** | `USB OTG D- / D+` | `GPIO 19 / GPIO 20` |
| **UART Console / Flash** | `TX / RX` | `GPIO 43 / GPIO 44` (`/dev/ttyACM0`) |
| **Reserved (Internal)** | `Flash / Octal PSRAM` | `GPIO 26–37` (do not use) |
| **Future Physical UI** | `ADC1 Pots` | `GPIO 1–10` (*never* `ADC2` with Wi-Fi) |
| **Future Physical UI** | `OLED (I²C) / Encoder (PCNT)` | `GPIO 8–9` / `GPIO 38–40` |

---

## MIDI CC Mappings

| CC | Parameter | Range |
|----|-----------|-------|
| `1` | Mod Wheel | `0% → 100%` |
| `5` | Portamento Time | `0.0 → 3.0 s` (exp) |
| `7` | Master Volume | `0% → 100%` |
| `70` | Preset Select (or MIDI Program Change) | `0 → 84` (85 factory presets) |
| `71` | Filter Resonance | `0% → 100%` |
| `72` | Amp Release | `1 ms → 5 s` |
| `73` | Amp Attack | `1 ms → 2 s` |
| `74` | Filter Cutoff | `20 Hz → 20 kHz` (exp) |
| `75–77` | OSC 1–3 Waveform | Sine / Square / Saw / Tri / Noise |
| `78–79` | OSC 2–3 Semitones | `-24 → +24 st` (`CC 64 = 0`) |
| `80–82` | OSC 1–3 Level | `0% → 100%` |
| `83–84` | Amp Decay / Sustain | `1 ms → 5 s` / `0% → 100%` |
| `85–86` | OSC 2–3 Cents | `-100 → +100 ¢` (`CC 64 = 0`) |
| `91` | Reverb Mix (auto-enables `> 0`) | `0% → 60%` |
| `93` | Chorus Mix (auto-enables `> 0`) | `0% → 60%` |
| `94` | Delay Mix (auto-enables `> 0`) | `0% → 60%` |
| `102` | Pitch Bend Range | `1 → 24 st` |
| `103–107` | Filter Attack / Decay / Sustain / Release / Env Amt | `1 ms → 5 s` / `-100% → +100%` |
| `108–111` | LFO 1 Rate / Depth / Waveform / Destination | `0.1 → 20 Hz` / `0–100%` |

---

## Building & Flashing

### Prerequisites

This firmware builds against the portable `rinsynth-model` and `rinsynth-dsp` workspace crates from a sibling [RustInSynth](https://github.com/Nasker/RustInSynth) checkout (`../RustInSynth/crates/rinsynth-*`) and the Xtensa `esp` Rust toolchain:

```bash
cargo install espup espflash ldproxy
espup install
. ~/export-esp.sh
```

### Run Host DSP Tests

```bash
cd ../RustInSynth && cargo test -p rinsynth-dsp --lib
```

### Build & Flash to ESP32-S3

```bash
cargo build --release
~/.cargo/bin/espflash flash --port /dev/ttyACM0 target/xtensa-esp32s3-espidf/release/rinsynth-esp32
```

Once booted:
1. Connect a USB-MIDI controller to the ESP32-S3 **USB OTG** port, and/or
2. Join the **`rinsynth-esp32`** Wi-Fi network and open **`http://192.168.71.1`** (or configure your home Wi-Fi under the **WIFI-NET** tab to access it on your LAN).

---

## Project Structure

```
rinsynth-esp32/
├── src/
│   ├── main.rs            # Core 1 I²S audio loop, CPU voice governor, boot riff, telemetry
│   ├── usb_midi.rs        # Core 0 USB-MIDI Host driver, SPSC event ring, 0xF8 MIDI Clock BPM
│   ├── web.rs             # Core 0 SoftAP+STA Wi-Fi, Captive DNS, HTTP/WS server, CmdRing
│   └── index.html         # Single-file themed Web UI (SYNTH / MATRIX / PRESETS / WIFI-NET)
├── sdkconfig.defaults     # ESP-IDF config (240 MHz, Octal PSRAM, lwIP/Wi-Fi in SPIRAM, USB Host)
├── partitions.csv         # 16 MB flash partition table
├── Cargo.toml             # Firmware dependencies & Xtensa profile settings
└── AGENTS.md              # Detailed hardware, DSP audit, and architecture notes
```

---

## Related Repositories

- **[RustInSynth](https://github.com/Nasker/RustInSynth)** — Desktop standalone & VST3/CLAP plugin synthesizer + shared `rinsynth-model` and `rinsynth-dsp` crates
