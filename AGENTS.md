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

## DSP budget audit + overload control (2026-10-23)

The "awful distortion depending on patch/voices" was **I2S underrun crackle**,
not bus clipping: at 6 voices + heavy FX the render passed the 5.805 ms
deadline and DMA starved (measured up to ~10 ms). A `peak:` meter in the
per-second telemetry now shows the post-limiter level directly (>1.0 would be
i16-conversion clipping — observed 0.2-0.3 under load, so the limiter works).

### Measured cost (instrumented build, cyc/sample; budget = 5442 @240 MHz)

- Per voice ≈ 900 cyc/sample (osc ~590 for 4 PolyBLEPs, filt ~200, env ~105)
- FX chain flat ~1115 cyc/sample idle → ~1400 with Tube distortion on;
  Fuzz/Bitcrush are much cheaper (one preset measured fx=303 total)
- Control/mod-matrix ~90-180 cyc/sample
- Profiler probes add ~150 cyc/sample total overhead — an uninstrumented
  build has ~5% more headroom
- Extrapolated worst case (6 voices + all FX + distortion): ~6800 cyc/sample
  ≈ 7.3 ms — **over deadline** → that is why 6-voice complex patches crackled

### CPU governor (main.rs audio loop)

- Warm-up: no shedding for the first `WARMUP_BLOCKS` (~10 s) — Wi-Fi/HTTP
  bring-up starves the render core and would otherwise shed before the
  user plays a note.
- Every late block (`dt > BLOCK_US`) counts; 4 in a row → `dyn_cap -= 1`
  (floor 1 — a single voice always fits) and `shed_voice()` keeps firing
  while *active* voices exceed the cap, fading the least-important voice
  over ~70 ms. Priority: quietest release tail (>100 ms), then quietest
  held non-bass above the cap, then quietest held non-bass anywhere;
  Attack voices are never held-candidates (fresh notes mid-ramp read as
  quiet but are the most audible loss). `LeanVoice.cfgd_release` stores
  the configured release, restored on `note_on`.
- CAUTION: `ADSREnvelope::release()` recomputes its decrement from the
  *current* amplitude — re-releasing a fading voice restarts a full
  fade, which pinned the governor in a zombie-shed loop until the
  <=100 ms skip was added. Keep the shed fade below that threshold or
  adjust both.
- Restore: after ~5 s of on-time blocks AND the worst block < 75% of the
  deadline → cap +1 (margin check prevents boundary ping-pong)
- User ceiling: `Poly` slider (1-6) in the UI → `pcap` key →
  `WebCmd::VoiceCap` → `USER_VOICE_CAP` atomic; governor clamps to it.
  Default 5 (Prophet-5-style safe ceiling; 6 stays reachable per-patch)
- `CONFIG_ESP_TASK_WDT_CHECK_IDLE_TASK_CPU1=n` — Core 1 is a dedicated RT
  render loop that legitimately starves IDLE1; the WDT backtrace itself
  stalled render ~20 ms. Real overruns are reported via `late:` instead.

### Distortion: already ported, now exposed

The desktop `EffectsChain` (Distortion→Chorus→Delay→Reverb) already runs in
`EmbeddedEngine`; 18 factory presets enable distortion (Tube/Fuzz/Bitcrush/
Fold all used) with no UI control — that alone read as "distorts awfully".
- UI: DISTORTION group in the effects column — On/Off, type buttons
  (Tube/Fuzz/Crush/Fold), Drive + Mix sliders; preset JSON carries
  `dt_e/dt_t/dt_d/dt_m` so loading a preset updates the controls
- Hot path: `tanh` → Padé [3/3] (|err|≤0.02, saturates |x|>3), Fold `sin` →
  `sine_lut` (explicit TAU wrap first — sine_lut only wraps ±1 turn),
  per-sample `sqrt` level compensation cached in `set_drive`
- Per-stage profiling: `profile::add` instrumentation in `embedded.rs`
  (env/ctrl/osc/filt/fx counters, `pstage!` = one predictable branch when the
  hook is off; `PROFILE_HOOK` const in main.rs installs the Xtensa `ccount`
  reader — set false to reclaim probe overhead in production)

### RF interference (audible ~10 Hz click — confirmed, mitigated)

A periodic click audible in the DAC output was confirmed to be Wi-Fi TX
bursts coupling into the analog path: dead silent during a `wstop`
radio-off window. AP beacons fire ~every 102 ms (hardcoded 100-TU in
esp-idf-svc) — hence ~8-10 Hz. Mitigations: `esp_wifi_set_max_tx_power(40)`
(10 dBm) at boot — ~10x less burst energy; `wpow`/`wstop` cmd keys allow
live tuning (wpow = quarter-dBm 4..80). If it persists: decouple the
PCM5102 rail (ferrite bead + 100 nF/10 µF at VCC) and keep USB/I2S
wiring away from the module antenna.

### Remaining DSP notes

- SVF bypass: `LeanVoice.filter_bypass` skips the per-sample TPT update
  when the filter is audibly transparent — lowpass, cutoff at top clamp,
  reso ≈ 0, AND no effective modulator (fenv ≈ 0, no LFO/mod-matrix route
  to cutoff/reso; the implicit mod-wheel LFO1→cutoff route counts only
  when wheel+depth > 0.01). ~150 cyc/sample/voice saved (filt 402→96 on
  2 voices). Filter state resets on re-entry; conditions re-evaluated
  each control tick so it never flaps mid-note.
- In the instrumented build the governor settles ~cap 2-3 under 4-osc +
  all-FX load; uninstrumented should hold ~4. Half-rate rendering is now a
  runtime A/B: `SynthRate` UI buttons / `srate` cmd flip the engine 44100 ↔
  22050 at the block boundary (`vm.set_sample_rate` re-derives coefficients
  in place; 128 synth frames + Catmull-Rom 2x upsample, ~half render cost).
- `peak:` telemetry answers "is it still clipping" at a glance; `late:`
  answers "is it still underrunning". Both reset each ~1 s window.

## Conventions

- No allocations / preset materialization in the audio loop
- Prefer concrete static dispatch in `EmbeddedEngine` (no `dyn`)
- Preset changes update ADSR params in place (never reset active voices)
- Never log Wi-Fi passwords; bound NVS strings
