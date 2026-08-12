# HDMI extension — specification

Optional HDMI output showing the VIC-II picture, rendered on the Pi from shadow
memory. Status: **specification only, nothing implemented.**

## 1. What makes this cheap

The architectural fact that the whole design rests on:

> The Pi's shadow is authoritative. The emulated CPU reads from it, the mirror is
> write-through, and physical C64 DRAM is **write-only** from our side — its only
> consumer is the VIC-II.

Everything a renderer needs is therefore already in Pi memory. **The renderer
issues no bus cycles at all.** That is what separates this from
`DISPLAY_SCRUB`, which failed because it added physical traffic (see the K233–K238
sequence in `CHT`).

## 2. Modes

Config key `HDMI_MODE`, matching the existing `UPPER_SNAKE value` style.

| Value | Mode | Shows | Per-frame cost |
|---|---|---|---|
| `0` | **off** | Text info — boot diagnostics, status, reset snapshot | none |
| `1` | **on** | VIC mirror **plus** a status line | rendering |
| `2` | **mirror** | Clean VIC picture, no overlay | rendering |

Default `0`.

### Why `off` still lights the console

`off` disables the *mirror*, not the display. Every hardware result in this
project has been read off the HDMI console — the K236 scrub counters, the
`NO-STABLE-WINDOW` calibration lines, the reset snapshot. A truly dark mode would
let someone configure away their own visibility.

It also makes a better control. Because the framebuffer and VideoCore come up in
all three modes, switching `0` ↔ `2` isolates exactly one variable — per-frame
rendering — rather than confounding it with bringing the video subsystem up. If
the renderer is ever suspected of perturbing bus timing, that A/B is available on
the same image, same boot, no rebuild.

## 3. Invariants

Non-negotiable, and the first one should be enforced by construction rather than
by discipline.

1. **The renderer cannot issue a physical bus access.** Not "must not" — give it a
   read-only view of shadow plus the register shadow and *no bus handle*, so a
   later well-meaning change cannot add a read-back "just to verify". The scrub
   work is the argument: accidental bus traffic cost days.
2. **Never block the emulator.** No lock the emulator can wait on. If a frame
   overruns, drop it.
3. **Skip, do not delay.** A late frame is invisible; a stalled emulator is not.
4. **Mode `2` implies mode `1` until takeover** — console through boot, picture
   once the core starts. Otherwise the boot diagnostics are lost.

## 4. Where it runs

`Source/Bus/RAD/c128_refresh.cpp` already establishes the pattern via
`CMultiCoreSupport`, and its `Run()` records that **cores 1 and 2 are
intentionally unused**, parked in `wfe`:

| Core | Today | With this feature |
|---|---|---|
| 0 | emulator | unchanged |
| 1 | parked (`wfe`) | **renderer** |
| 2 | parked (`wfe`) | parked |
| 3 | `C128_REFRESH_CORE` | unchanged |

Core 1 is chosen over 2 arbitrarily; if cache-partitioning experiments ever
suggest otherwise, it is a constant.

## 5. Data sources

All local. Nothing here requires a bus cycle.

| Needed | Where it lives |
|---|---|
| Screen matrix, bitmap, charset, sprite data | bank-0 shadow (`CC64Memory`) |
| `$D011` `$D016` `$D018` `$D01x` `$D02x` | `m_VICRegShadow[0x40]`, `m_LastD018` |
| VIC bank | CIA2 `$DD00` tracking, already maintained |
| Colour RAM | the bank-1 I/O cache (`$D800-$DBFF`) |
| Sprite pointers / hot blocks | the relocation-aware hot-block map |
| Changed regions | the write buffer's dirty bitmap |

The dirty bitmap is worth using from the start: re-rendering only changed regions
turns a fixed per-frame cost into one proportional to what the guest actually
drew, which for a static GEOS desktop is nearly nothing.

## 6. Rendering model

**Capture VIC registers per scanline.** A frame-at-a-time renderer that samples
the registers once will get every raster effect wrong — FLI, split screens,
sprite multiplexing, opened borders. Per-line capture is ~263 lines × ~20 bytes
≈ **5 KB/frame**, which is nothing, and it is painful to retrofit.

The emulator already tracks the raster line and delivers display-register writes
immediately, so the capture point exists.

Scope, in order: text and hires bitmap first, then multicolour, then sprites,
then border effects. Each is independently useful.

## 7. Cost budget

| Item | Figure |
|---|---|
| Source data read per frame | ~9 KB (matrix + bitmap) |
| Pixels produced | 320×200, or 384×272 with border |
| Framebuffer, 8bpp indexed | 64 KB/frame, ~3.8 MB/s at 60 Hz |
| …with border | 104 KB/frame, ~6.3 MB/s |
| Pixel work | tens of thousands of instructions/frame; **well under 1% of one A53 core** |

Render at native resolution and let the VideoCore scale to the HDMI mode rather
than producing scaled pixels in software.

> **To verify during implementation:** that Circle exposes independent
> virtual/physical framebuffer sizes so the GPU does the scaling. If not, render
> to a small buffer and blit — the cost changes, the design does not.

## 8. Cache discipline

The real risk is not compute. This project has already been bitten twice by cache
behaviour: byte zero was an instruction-fetch stall, and the RAD notes record that
an L2 miss inside a bus window is fatal — which is why `busTiming` is a packed,
128-byte-aligned struct and why `CACHING_L1_WINDOW_KB`, `CACHING_L2_OFFSET_KB` and
`CACHING_L2_PRELOADS_PER_CYCLE` exist as tunables.

Rules:

- **Non-temporal stores (`STNP`) for the framebuffer.** Streaming 64–104 KB of
  writes per frame through L2 would evict the emulator's working set. Reading
  shadow is comparatively benign — clean shared lines.
- **No allocation, no locks, no logging on the render path** once running.
- Treat any measured change in bus behaviour between `HDMI_MODE 0` and `2` as a
  defect in this feature, not as noise.

## 9. Staging, with acceptance per stage

| Stage | Deliverable | Acceptance |
|---|---|---|
| 1 | `HDMI_MODE` parsed; `0` reproduces today exactly | Boot diagnostics unchanged; bus behaviour bit-identical to the current build |
| 2 | Renderer on core 1, text mode only, no overlay | Picture matches the composite output on a static BASIC screen |
| 3 | Hires bitmap + per-line register capture | GEOS desktop renders; a raster split renders in the right place |
| 4 | Multicolour, sprites, border | Metal Dust recognisable |
| 5 | Mode `1` status line | Live counters legible without a reset snapshot |

Stage 1 is the one not to skip. Proving `HDMI_MODE 0` is indistinguishable from
today establishes the baseline that every later stage is measured against.

## 10. The diagnostic dividend

Two reasons this earns its place beyond being a feature.

**A shadow-sourced picture is immune to the DRAM corruption.** The sparkle lives
in physical DRAM; shadow is clean. Composite output next to HDMI makes any
divergence a continuous, zero-cost visualisation of the exact fault the K222–K238
sequence has been chasing — and it separates "our emulation is wrong" from "the
DRAM got disturbed" at a glance, with no diagnostic build.

**Mode `1`'s status line ends the photograph workflow.** Several results this week
were obtained by pressing reset and photographing one line before the counters
cleared. A live status line shows a rate changing as it changes.

## 11. Open questions

- Circle's virtual/physical framebuffer sizing (§7).
- Whether the colour-RAM cache is populated on all machine paths or only v2 —
  affects whether colour is correct in every configuration.
- Whether per-line capture wants a ring buffer or a fixed 263-entry array; the
  array is simpler and 5 KB is affordable.
- Whether mode `1`'s overlay should be suppressible at runtime for photography.

## 12. Related

- `Docs/SuperCPU64/architecture.md` — where the cycles go, and why reads are free.
- `Docs/SuperCPU64/timing-notes.md` — the cache and bus-window constraints.
- `CHT`, K233–K238 — why "it only costs a little bus traffic" is not a safe
  assumption on this hardware.
