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

## Open issue (top priority) — under mitigation, needs on-device soak test

Web UI becomes unresponsive after a while: page loads slowly or slider/preset
changes stop taking effect. Previously fixed: esp-idf-svc ws wrapper bug (now
raw `httpd_*` API in `src/web.rs`), CLOSE/PING frames, socket leaks, JS
coalescing, ping/pong watchdog, `WIFI_PS_NONE`, `CONFIG_FREERTOS_HZ=1000`.

Round-2 hardening (2026-10-23, in tree, unverified on hardware):
- `CONFIG_LWIP_MAX_SOCKETS=16` + `max_open_sockets=12` (was 10/7 — httpd alone
  could occupy 9 of 10 sockets, starving DNS/clients)
- TCP keep-alive on all httpd sockets (idle 10s, 2×5s probes) so dead peers'
  session slots are freed automatically
- `ws_ping` watchdog task: every 10 s queues a server-side WS PING to each
  WS client via `httpd_queue_work`; failed sends trigger `httpd_sess_trigger_close`;
  client PONGs refresh the session LRU so a live WS is never evicted
- `recv_wait_timeout` 5→2 s, `send_wait_timeout` 5→3 s (bounded httpd stalls)
- JS: removed the `bufferedAmount`-based `/cmd` fetch fallback (was a
  positive-feedback fetch storm under WS congestion); all dirty params now go
  out as ONE `;`-batched WS frame per 25 ms tick (server `handle_query` splits
  on `;`); WS frame cap raised 256→1024 B for batches
- `usb_midi` SPSC ring actually had two producers (usb_client + httpd tasks);
  pushes now serialized under `PUSH_LOCK` (consumer pop stays lock-free)

Still to verify on hardware with `esp monitor`: whether UI stays responsive
after minutes of slider drags; if it recurs, count `httpd_sock_err` logs and
check AP+STA channel re-anchor on STA connect (deauths AP clients once).
Reproduce: connect phone/laptop to AP, drag sliders for a few minutes.

## Conventions

- No allocations / preset materialization in the audio loop
- Prefer concrete static dispatch in `EmbeddedEngine` (no `dyn`)
- Preset changes update ADSR params in place (never reset active voices)
- Never log Wi-Fi passwords; bound NVS strings
