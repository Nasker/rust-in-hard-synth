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

## Resolved: Web UI unresponsiveness (root-caused & fixed on hardware 2026-10-23)

Two distinct root causes were found and fixed; the UI stall no longer
reproduces under sustained load.

### Root cause 1 — internal-RAM exhaustion (the page-load stall)

`CONFIG_SPIRAM_MALLOC_ALWAYSINTERNAL=16384` forced every heap alloc ≤16 KB —
including all lwIP/Wi-Fi buffers — into the ~180 KB internal pool. Only ~13 KB
remained free at idle; each TCP response pinned ~7-9 KB of DMA-capable TX
pbufs, and 2-3 concurrent transfers emptied the pool. Symptom on the wire:
any downlink frame >~1315 B was silently dropped, so the 38 KB index stalled
after ~1 TCP window while small requests still worked.

Fix (`sdkconfig.defaults`):
- `CONFIG_SPIRAM_TRY_ALLOCATE_WIFI_LWIP=y` — Wi-Fi/lwIP mallocs prefer PSRAM
  (Wi-Fi copies TX pbufs into its own internal DMA buffers at send time, so
  this is safe)
- `CONFIG_SPIRAM_MALLOC_ALWAYSINTERNAL=16384→1024`
- `CONFIG_SPIRAM_MALLOC_RESERVE_INTERNAL=32768` (untouched — DMA reserve)
- `CONFIG_ESP_WIFI_AMPDU_TX_ENABLED=n` — not the root cause (large frames
  died with or without it); kept off to simplify the TX path
- `CONFIG_LWIP_MAX_SOCKETS=10→16`, `max_open_sockets=7→12`

Result: free internal RAM ~13 KB → ~50 KB, index loads in ~120 ms.

### Root cause 2 — WS stream desync on control frames (the 10s death)

`httpd` reads each frame's first byte itself, then calls the URI handler only
for data frames (`ws_type < CLOSE`) unless `handle_ws_control_frames` is set.
Without it, a client PONG's remaining bytes stayed in the socket, desynced the
stream, and the next parse failed "not properly masked" → session killed. It
was latent until the server-side PING watchdog made browsers reply PONG — so
every WS connection died ~10 s in. Fixed by `handle_ws_control_frames: true`
on the `/ws` URI (handler consumes whole frames; CLOSE handled by httpd too).

### Other hardening from the same round

- `ws_ping` watchdog: every 10 s, `httpd_queue_work` sends a server PING to
  each WS session (fd re-validated inside the work fn); send failure →
  `httpd_sess_trigger_close`; client PONGs refresh session LRU
- TCP keep-alive on httpd sockets (idle 10 s, 2×5 s probes); `lru_purge_enable`
- `recv_wait_timeout` 5→2 s, `send_wait_timeout` 5→3 s
- JS: dropped the `bufferedAmount`→`/cmd` fetch fallback (fetch-storm under
  congestion); dirty params go out as ONE `;`-batched WS frame per 25 ms tick;
  `handle_query` splits `;`; WS frame cap 256→1024 B
- `usb_midi` SPSC ring had two producers (usb_client + httpd tasks) — pushes
  serialized under `PUSH_LOCK`, consumer pop stays lock-free
- Telemetry: `mon:` log every ~10 s prints free internal heap + httpd session
  count (watch it during soaks)

### Verified on hardware

- 15/15 + 10/10 index fetches, ~120 ms each; heap flat, sessions→0
- 60 s WS soak: 187 batched `;` commands, 6 watchdog pings PONG'd, no reset
- Multi-client: 3 concurrent WS + 526 preset fetches + 2 RST-aborted stale
  sessions — all cleaned up, heap recovered to baseline
- Audio: render ≤ ~5.3 ms/block idle, ~6.7 ms peak under WS load, 0 I2S
  write errors (PSRAM bus contention raises render max slightly — watch it)

### Still to verify

- Real browser drag-a-thon (minutes of slider drags on a phone)
- AP+STA mode after the PSRAM change; captive-DNS under load
- USB host enumeration sometimes logs `ESP_ERR_NO_MEM` endpoint allocs before
  reporting the interface ready — monitor separately

`httpd_sock_err ... recv : 104/128` + "not properly masked" after a client
closes without a WS CLOSE frame is expected teardown noise, not a bug.

## Conventions

- No allocations / preset materialization in the audio loop
- Prefer concrete static dispatch in `EmbeddedEngine` (no `dyn`)
- Preset changes update ADSR params in place (never reset active voices)
- Never log Wi-Fi passwords; bound NVS strings
