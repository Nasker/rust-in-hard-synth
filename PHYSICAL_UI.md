# Physical UI plan — state-driven OLED + contextual controls

Vision (discussed 2026-11-03): a small OLED + rotary encoder + pots + buttons
whose meaning depends on the current screen state. Display layout mirrors
the physical control row, so each on-screen parameter sits next to the pot
that owns it — no printed legend needed. Same "navigation + contextual
pages" model as Nord/Peak/Hydrasynth.

## Screen state machine

```
        boot
         │
         ▼
   ┌───────────┐   riff ends or
   │  SPLASH   │   any click
   │ animation │──────────────┐
   └───────────┘              ▼
                     ┌──────────────┐   rotate → MENU
                     │    HOME      │◄──────────────┐
                     │ glyphs+name  │               │ auto-return
                     │ 4 macros     │   ENTER       │ timeout (~30 s)
                     └──────────────┘────────┐      │ or BACK
                           ▲                 ▼      │
                           │           ┌───────────┐│
                           │  BTN1=BACK│   MENU    ││
                           └───────────┤ glyph nav ││
                                       └───────────┘│
                            ENTER/click a glyph ▼   │
                                       ┌───────────┐│
                                       │  SECTION  ├┘
                                       │ 4-col     │
                                       │ pages     │
                                       └───────────┘
```

- **SPLASH** — logo/anim while Wi-Fi/HTTP + riff come up. Exits on riff
  end (or click skips it). Doubles as a boot-progress readout.
- **HOME** — status glyph row up top (web server up, MIDI device present;
  room for tempo / voice-count / governor-warn later), preset name large
  in the middle, 4 macro labels over the pots (each pot live on a main
  macro — e.g. CUTOFF / RESO / ATTACK / RELEASE or a per-preset pick).
- **MENU** — glyph grid/carousel of engine sections: OSC, FILTER, ENV,
  LFO, MATRIX, FX, VOICE, **ARP**, SYSTEM. Rotate to roll the focus,
  click/ENTER to enter. This is the "map of the synth".
- **SECTION** — paginated parameters, 4 per page in pot-aligned columns
  (label on top, value bar under it, pot i edits column i). Middle
  softkeys step pages (◄PAGE / PAGE►) — encoder also steps when pots are
  idle. Encoder adjusts the focused column. BACK returns to MENU.

## Architecture — same SRP/interface-first split as `status.rs`

```
┌─ producers ──────────────────────────────────────────────┐
│ PCNT encoder / ADC1 pots / GPIO buttons → UiEvent stream │
└────────────────────┬─────────────────────────────────────┘
                     ▼
┌─ ui_state (PURE — no hardware, host-testable) ───────────┐
│  state machine + focus + page tables.                    │
│  In: UiEvent, SystemStatus(status.rs), engine readouts   │
│  Out: Screen model (what to draw) + ParamChange list     │
└────────┬───────────────────────────────┬─────────────────┘
         ▼                               ▼
┌─ renderer ─────────────┐    ┌─ control sink ─────────────┐
│ Screen → framebuffer    │    │ ParamChange → web::push_cmd │
│ glyphs/text/parameter   │    │ (existing serialized ring — │
│ bars → SSD1306 driver   │    │  same path the browser uses)│
└────────────────────────┘    └─────────────────────────────┘
```

- **ui_state is pure Rust** — unit-testable on the host, no ESP deps.
  This is the interface-first part: `fn step(&mut self, ev: UiEvent) ->
  (Screen, Vec<ParamChange>)`.
- **Param changes reuse `WebCmd`** — the physical UI is just another
  Core 0 producer on the already-serialized ring (`CMD_PUSH_LOCK`).
  Zero new plumbing into the audio loop.
- **Sections/pages are static data** — a `PAGE_TABLE: &[Section]` of
  `{glyph, name, pages: &[[PageSlot; N]]}` where `PageSlot =
  {param: SynthParam, label}` — adding a page is data, not code.
  `SynthParam` already carries identity/ranges via the model crate.
- **Display refresh decoupled from control rate** — UI task ~25-30 fps;
  parameter edits go out immediately, animation is cosmetic.

## Concrete hardware layout (confirmed)

- **4 pots** along the TOP of the screen → every parameter page is exactly
  4 columns, each column aligned under its pot. Label/value live above
  the pot line.
- **4 buttons** along the BOTTOM → contextual softkeys, labels rendered
  on the screen's bottom row. This replaces the long-press grammar:
  always-on-screen hints (e.g. `BACK · ◄PAGE · PAGE► · ENTER` on MENU,
  `BACK · · · RESET` on a param page). Function changes per screen —
  the screen IS the legend.
- **Rotary encoder with click** on the right side → primary navigation:
  rotate = move focus / adjust value, click = confirm/enter.
- **Display**: 1.3" SPI monochrome OLED — almost certainly **SH1106**
  (SSD1306 is the 0.96" variant). Driver consequences:
  - **Page-addressing only** (no horizontal mode like SSD1306) — flush
    must iterate 8 pages × 132 cols; a dirty-rect that isn't page-aligned
    still costs a full page row. Framebuffer = 128×64/8 = 1 KB static.
  - **+2 column RAM offset** — its internal RAM is 132 wide; column 0 of
    the visible panel starts at RAM column 2. One offset in the flush
    routine, invisible to everything above it.
  - SPI + DMA on GPIO10–13 → full-frame 1 KB writes are cheap; ~25-30 fps
    refresh is trivial headroom.

## HOME macros — category defaults + per-preset override (confirmed)

Two-tier model:
1. **Category table** (`HOME_MACRO_TABLE[category]` in the page data):
   every preset gets sensible macros for free — e.g. Bass →
   CUTOFF/RESO/FENV/DECAY, Lead → CUTOFF/RESO/GLIDE/FENV, Pad →
   ATTACK/CUTOFF/REVMIX/CHOMIX. No schema churn, works for all 85 today.
2. **`macro_params: [u16; 4]` preset field** (serde-defaulted → "use
   category") — per-patch override so the user can pin *their* four.
   Editing flow: on HOME, a softkey opens MACRO-ASSIGN — 4 slots aligned
   to the pots, encoder scrolls the SynthParam list per slot. Stored in
   the preset struct; persistence is session-level until preset save
   exists (file save stays a separate feature — flag: unsaved macro
   assignments die on power-off, same as any edited param today).

## Arpeggiator — a SECTION page + a note-generator (CPU cost ~zero)

An arp is **not DSP** — it's a block-boundary note scheduler, the same
mechanism the boot riff already uses. No per-sample cost; the CPU budget
is untouched. Design:

- **`Arpeggiator` as a pure portable struct** (rinsynth-dsp or -model —
  same testable-pure philosophy as `ui_state`): tracks held notes,
  emits `NoteEvent`s on a schedule derived from `bpm` + division. The
  audio loop calls `arp.tick(block, bpm) -> events` and feeds them to
  `vm.receive_event` — held-note tracking means NoteOff semantics just
  work, and it functions in poly or mono (mono+arp = classic bass lines).
- **ARP page = 4 params**: ON/OFF · MODE (up/down/up-down/random) ·
  RATE (tempo-synced division: 1/4..1/32 incl. dotted/triplet) ·
  OCTAVES (1-3). Gate-length is a future 5th if wanted — the page model
  handles multi-page trivially.
- Preset fields `arp_on/arp_mode/arp_rate/arp_oct` (serde-defaulted off)
  so arp state travels with patches — a "Dream Seq"-style preset can
  ship arpeggiated out of the box.
- Steal interaction: arp notes are notes — they join the voice pool and
  obey the governor like any other. Poly+arp at 5 voices is the
  honest ceiling, same as hands.

## Decisions to pin down (the interesting ones)

1. **Pot takeover semantics** — absolute pots vs. page-switched params
   mean stale positions. Standard fix is **catch/pickup**: a pot doesn't
   edit until it crosses the current value. Recommend pickup everywhere
   (prevents jumps when entering a page); cheap to implement per-slot.
2. **Button grammar** — 4 softkeys gives each screen its own labelled
   actions. Proposed defaults: BTN1 = BACK everywhere (consistent muscle
   memory), BTN4 = ENTER/confirm, middle two = screen-specific (page
   prev/next, reset, fine/coarse). Only rule: never leave a button's
   label blank — unlabeled = dead button is a UX bug.
3. **Glyph set** — 16×16 icon tiles: Wi-Fi link, MIDI plug, OSC waveforms,
   filter curve, ENV shape, LFO wave, matrix grid, FX bubbles, voices,
   gear. Small bitmap font (5×7 or 8×8) for text + preset names.
4. **Auto-return to HOME** — ~30 s idle in MENU/SECTION goes home
   (standard on hardware; keeps the "is it working?" readout visible).
   Arguable: keep last section on stage? Recommend auto-home, configurable.
5. **Splash exit** — riff-done OR click. If web isn't up yet, HOME shows
   a dim/unlit web glyph — state visible without blocking the transition.
6. **Parameter source of truth** — engine params flow up via existing
   telemetry (or a cheap `web::` snapshot) so the screen shows engine
   truth, not UI-side guesses — matters when browser + pots both edit.

## Hardware budget (per AGENTS.md pin rules)

- Pots ×4: ADC1 GPIO1-4 (100 nF at wiper, EMA + ~1% hysteresis — Wi-Fi-safe)
- Encoder: PCNT on GPIO38/39 + click on GPIO40 — zero CPU
- Buttons ×4: GPIO41,42,45,46 (or 47 — all free per budget)
- Display: SH1106 SPI — GPIO10(SCK) 11(MOSI) 12(DC) 13(CS) (+RESET any free)
- Status LED: already done (GPIO48)

## Milestones (each independently flashable)

1. **M1 display driver + splash** — framebuffer, font, one animation;
   exits to a stub HOME. Proves the screen + task on hardware.
2. **M2 input layer** — PCNT encoder → UiEvent, ADC1 pots with
   EMA/hysteresis, buttons with debounce. Debug screen echoing events.
3. **M3 ui_state + HOME** — real state machine, status glyphs wired to
   `status.rs` state + `usb_midi` presence, preset name, pot macros.
4. **M4 MENU + SECTION pages** — glyph carousel, page table for every
   section, encoder-adjust + pot pickup semantics.
5. **M5 arpeggiator** — pure `Arpeggiator` note-scheduler (block-boundary,
   tempo-synced), `arp_*` preset fields, ARP section page.
6. **M6 polish** — animations, auto-return, param flash on web-side
   edits, MACRO-ASSIGN editor on HOME.

Non-goals for now: preset save to flash (file system work — macro
assignments and arp settings are session-level until it exists), patch
browser beyond next/prev, per-part screens (multitimbral is still
discussion).
