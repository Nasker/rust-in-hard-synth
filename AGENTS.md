# rinsynth-esp32 — Agent Context

ESP32-S3 standalone synthesizer in Rust (esp-idf-svc / cargo-esp). Audio is
synthesized on-device and output through a PCM5102 DAC. USB-MIDI host via the
native USB OTG port. Browser control over Wi-Fi SoftAP + HTTP/WebSocket.

Related repos:
- DSP + model: `/home/oscarmartinez/Documents/PlatformIO/Projects/RustInSynth`
  (`rinsynth-dsp` crate, `src/embedded.rs` = lean static-dispatch engine,
  `src/effects.rs` = Distortion→Chorus→PingPongDelay→Reverb chain)
- Legacy C reference synth: `/home/oscarmartinez/Documents/PlatformIO/Projects/synth/main/synth.c`

## Hardware

- ESP32-S3 rev v0.2, 16 MB flash, 8 MB octal PSRAM
- PCM5102: BCK=GPIO6, LRCK/WS=GPIO4, DIN=GPIO5, MCLK=GPIO7
- I2S: 44100 Hz, stereo 16-bit, 256-frame blocks (~5.805 ms deadline)
- UART/flash: `/dev/ttyACM0` (CH340). USB OTG port = USB-MIDI host (hub support on)
- Task layout: audio render pinned to Core 1; USB host + Wi-Fi/HTTP on Core 0;
  MIDI→audio via lock-free SPSC ring (`usb_midi.rs`)

## Build / flash / test

```bash
cd ~/Documents/PlatformIO/Projects/RustInSynth && cargo test -p rinsynth-dsp
cd ~/Documents/PlatformIO/Projects/rinsynth-esp32 && cargo build --release
~/.cargo/bin/espflash flash --port /dev/ttyACM0 --monitor target/xtensa-esp32s3-espidf/release/rinsynth-esp32
# if auto-reset fails: hold BOOT, tap RESET, release BOOT; use --before no-reset
```

## Engine status (what works)

- 4-voice+ polyphony (pool of 6), 4× PolyBLEP oscillators/voice (`MAX_OSCS=4`),
  TPT SVF (Zavalishin), amp+filter+env3 ADSRs, LFO1/LFO2, glide, velocity
- Control-rate ModMatrix (every 64 samples); Osc4 usable as audio-rate FM/RM source
- Bus peak limiter + soft clip; voice stealing protects held bass notes
- USB-MIDI: Note On/Off, PitchBend, CC, Program Change; full 30-CC map incl.
  CC70/ProgramChange = preset select (85 factory presets)
- SoftAP `rinsynth-esp32` @ http://192.168.71.1 — themed Web UI (SYNTH /
  ROUTING MATRIX / PRESETS / WIFI-NET tabs), captive portal (DNS :53 + probe
  handlers), AP+STA mode with NVS-persisted credentials

## Open issue (top priority)

Web UI becomes unresponsive after a while: page loads slowly or slider/preset
changes stop taking effect. Already fixed: esp-idf-svc ws wrapper bug (now raw
`httpd_*` API in `src/web.rs`), CLOSE/PING frames, socket leaks, 40 Hz JS
coalescing, ping/pong watchdog, `WIFI_PS_NONE`, `CONFIG_FREERTOS_HZ=1000`.
Still recurring on hardware. Next suspects to verify on serial monitor while
reproducing: httpd socket exhaustion (count `httpd_sock_err` logs), AP+STA
reconnect side-effects, DNS task starvation, stale-session handling with
multiple clients. Reproduce: connect phone/laptop to AP, drag sliders for a
few minutes, watch `esp monitor` output.

## Conventions

- No allocations / preset materialization in the audio loop
- Prefer concrete static dispatch in `EmbeddedEngine` (no `dyn`)
- Preset changes update ADSR params in place (never reset active voices)
- Never log Wi-Fi passwords; bound NVS strings
