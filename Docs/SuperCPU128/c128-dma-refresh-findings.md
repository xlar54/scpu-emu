# C128 under sustained expansion-port DMA: established findings

Working record of what has been **measured** on a real Commodore 128 while the RAD
cartridge holds `/DMA` and drives the bus. Companion to
`supercpu128-hardware-findings.md`, which covers a real CMD SuperCPU 128 accelerator;
this file covers the **C128 motherboard itself** as a DMA host.

Same discipline as that file: raw measurement first, interpretation clearly separated,
and nothing recorded here that is still open. Open questions, candidate schedules and
untrusted scan results are deliberately excluded — they live in the `CHT` collaboration
log.

## Test provenance

- Machine: physical C128, NTSC, held in **C64 mode** (C= key at power-on)
- Cartridge: RAD Expansion Unit, Raspberry Pi 3B (BCM2837, Cortex-A53), `arm_freq=1400`
- Measurements come from purpose-built diagnostic kernels; the kernel number is cited
  for every figure so it can be re-derived from the archived `busdiag-*.txt` logs
- Oracles are seeded byte patterns in physical DRAM, verified by triple readback, and
  cross-checked against the visible screen where possible

## 1. Measured machine parameters

| Quantity | Measured value | Source |
|---|---|---|
| PHI2 period | 1368–1370 ARM cycles = **978.6 ns** (1.0219 MHz) | E15–E17 |
| Raster line | **65 PHI cycles** = 63.6 µs | E16 (`linecycles=65`) |
| Frame | 263 lines = **16.73 ms**, 59.78 Hz | derived, E16/E18F |
| Full-frame release | 263 × 65 = **17,095** `/DMA` pulses, 16,714–16,715 µs measured | E18L |

The 2-second exposures used throughout the diagnostics correspond to exactly
**31,560 raster lines = 120 frames = 2.006 s**, verified by exact pulse accounting
(2,051,400 pulses at W=65) in E18F and E18L.

## 2. Sustained `/DMA` suppresses C128 DRAM refresh

**This is the central finding.** On a C128, holding `/DMA` asserted continuously causes
physical DRAM to lose its contents. Screen memory decays visibly within tens of seconds.

Evidence, in order of decisiveness:

- **kernel144.** With *all* RAD address/data/RW drivers disabled and no bus traffic at
  all, releasing `/DMA` for 400 ARM cycles (~286 ns) at each verified PHI2 falling edge
  preserved **255 of 256** oracle cells over 60 s, with all eight control cells exact.
  The single failure (`$04FF`) occurred at the tail of a capture pass during which the
  pulse loop was paused. `host_rw_low = 0` across 61,363,160 sampled CPU half-cycles.
- **E11 halt test.** After entering SCPU64 mode, halting *all* physical bus traffic and
  switching to continuous refresh stopped the accumulation of spurious screen characters
  outright. This distinguishes refresh decay from mirroring errors or rogue writes in a
  single observation.
- **Contrast.** Under the same conditions *without* the release, a 60-second exposure
  produced 21 target failures plus destroyed controls (kernel143).

The release is **sub-cycle** — 286 ns is shorter than one PHI2 half-cycle, so the 8502
never completes a bus cycle and remains parked. This was confirmed by sampling R/W across
61 million CPU half-cycles with no low observed.

### Interpretation

The C128's DRAM refresh depends on activity that sustained `/DMA` prevents from reaching
the array. Releasing `/DMA` inside the VIC-owned half-cycle restores it without handing
the bus back to the host CPU.

**This is a motherboard-level property, not specific to C64 mode**, and applies to any
cartridge that holds the C128 bus for extended periods.

## 3. Retention limits (measured)

kernel154 ran a suspension ladder: normal refresh, then a deliberate gap, then resume and
capture.

| Suspension | Result |
|---|---|
| 2 ms | pass |
| 4 ms | 3 stable failures |
| 8 ms | 2 stable failures |

Interpretation: with a normal refresh sweep period of ~3.27 ms, the corresponding
worst-case row intervals are 5.27 ms (passed) and 7.27 ms (failed). **Retention onset on
this machine lies between roughly 5 and 7 ms of row interval.**

Caveats recorded honestly: the 4 ms and 8 ms counts are non-monotonic, indicating the
measurement sits near its noise floor, and the oracle used was not polarity-aware (see
§5), so sensitivity was roughly half of achievable.

## 4. What does not work (measured negative results)

These are recorded so they are not re-attempted.

**Synthetic read sweeps do not refresh.** Reading addresses to "touch" DRAM rows does not
preserve it:

- reading the 16 target cells directly ~30,000×/s for 60 s — all 16 decayed (kernel141 X1)
- reading the full page `$0400–$04FF` continuously — catastrophic, target *and* control
  destroyed (kernel142 F1)
- kernel144 achieved 255/256 with **zero** synthetic reads, proving reads were never the
  mechanism

**Fixed-period epoch schedules fail at every duty tested.**

| Schedule | Result |
|---|---|
| 250 µs traffic / 250 µs refresh (50%) | decays |
| 250 µs / 1000 µs (80% refresh) | decays; flood of corruption under CPU load |
| dithered 8–23 µs traffic / 70 µs refresh | materially reduced, still not zero |

Notably 80% duty implies a ~4.10 ms row interval — *below* the 5.27 ms that passed the
ladder — and it still decayed.

**Per-line fixed-phase traffic windows fail at every width tested.** A window of W consecutive
PHI cycles opened at the same offset on every raster line:

| Width | Blocked per line | Result (adapter-equipped machine) |
|---|---|---|
| 32 | 31.3 µs (49%) | heavy corruption |
| 16 | 15.7 µs (25%) | marginal; clean on 1 of 3 cold boots |
| 8 | 7.8 µs (12%) | sparse but nonzero |
| 4 | 3.9 µs (6%) | sparse but nonzero |

Every rung is 64–511× *inside* the measured suspension bound, so contiguous gap length is not
the mechanism.

**Phase avoidance does not fix it either.** A single-edge omission map (65 rungs, `W = 64`, run
in a coprime-stride order to decorrelate scan position) locates the refresh-critical omitted
edges at **anchor-relative 5..11** — seven edges. The production schedule at the time opened
traffic on edges 24..55, which **already protects 5..11 with zero overlap**, and it still
corrupted catastrophically.

That measured band does reconcile with the documented VIC-II refresh position: an anchor offset
of `L ≈ 4..6` maps 5..11 onto absolute ~9..17, containing the documented 11..15 burst with about
one edge of margin at each end. It is the only independent measurement on this machine that has
agreed with schematic 310378.

### Interpretation

Average refresh duty is not the governing quantity, and neither is contiguous gap length. The
decisive comparison is between two schedules of **essentially identical duty**:

```
per-line width 4   6.2% duty  -> corrupts
coarse epoch       5.9% duty  -> no continuing corruption
  (250 µs traffic / 4250 µs cycle)
```

The distinguishing property is **periodicity**. A per-line window's blackout period is exactly
the raster-line period, which is the natural period of the refresh counter's advance — so the
same rows sit under the window on *every* sweep and are permanently starved, never refreshed at
all. Shrinking the window only shrinks how many rows are permanently dead. The coarse epoch's
period-to-sweep ratio is 4250/3270 = 1.300, not an integer, so the blocked set rotates: each row
is skipped occasionally, waits one extra sweep (~6.5 ms, inside the marginal band), and is
refreshed normally on the next pass.

A single-edge omission map therefore cannot predict multi-edge blocking. It measures sensitivity
at 1/65 loss; a production window blocks tens of consecutive edges, and the two regimes are not
related by extrapolation.

The non-aliased duty ceiling has been bracketed on hardware but not resolved:

```
250 µs traffic / 4250 µs cycle   =  5.9%  ->  no continuing corruption
750 µs traffic / 4250 µs cycle   = 17.6%  ->  corruption returns
```

Underlying all of it: the VIC's refresh counter **free-runs**. A row whose refresh slot falls
inside a traffic window is *skipped for that entire sweep* rather than deferred within it. So an
80% duty means roughly 80% of rows refreshed normally and 20% skipped — not every row at 80%
rate. That is why duty arithmetic consistently under-predicts the damage.

Dithering a schedule's period removes the aliasing (multi-sweep starvation) but cannot remove
single-skip damage, because one missed sweep already lands inside the measured 5–7 ms failure
band. The dithered-epoch hardware result matched that prediction: materially less corruption,
not zero.

## 5. Decay signature (measured)

From kernel142 F1, the only phase with a full both-parity sample. Of 21 non-stable cells,
**20 conform exactly** to a true/complement rail split on address bit A0:

- **odd addresses rail toward `$FF`** — only 0→1 bit flips:
  `$0401 02→FF`, `$0403 04→3C`, `$0405 06→FF`, `$0407 08→FF`, `$0429 0A→9A`,
  `$042B 0C→FF`, `$042D 0E→FF`, `$042F 10→FF`, `$0701 12→FF`, `$0703 14→3D`,
  `$0705 16→FF`, `$0707 18→FF`
- **even addresses rail toward `$00`** — only 1→0 bit flips:
  `$0402 03→00`, `$0404 05→00`, `$042A 0B→08`, `$042C 0D→09`, `$042E 0F→00`,
  `$0702 13→00`, `$0704 15→00`, `$0706 17→00`
- sole violation: `$0406 07→08`

Two consequences, both practical:

1. The intermediate values (`$3C`, `$9A`, `$3D`, `$08`, `$09`) establish that **`$FF` is a
   genuine decay endpoint**, not a floating-bus read artifact. Earlier hedging on that
   point can be dropped.
2. **Seed oracles against the rail.** Odd addresses seeded `$00` and even addresses seeded
   `$FF` put all 8 bits of every cell at risk, versus ~4.5 for an arbitrary pattern. This
   roughly doubles sensitivity and makes "a decayed cell coincidentally equals its expected
   value" impossible by construction.

A separate stochastic floor was measured under otherwise-correct continuous refresh:
approximately **0.35 failures per 2-second exposure** across a 264-cell oracle
(6 failures across 32 s of 8-second controls, E18F/E18I/E18J). Any acceptance criterion
applied to an exposure-bearing measurement must account for it; zero-exposure controls
should still require exactly zero.

## 6. Host-side faults discovered on this hardware

These are Pi-side, not C128-side, but were found while bringing up the C128 and cost
eighteen firmware revisions. They generalise to any cycle-accurate bus code.

### Discarded "priming" reads caused the fault they were added to cure

The C128 path inserted two discarded reads after every write, on the theory that the
buffered expansion port returned stale data on the first read. A same-boot A/B disproved
it: a raw probe with priming **disabled** returned the written value on read 0, **16 of 16
times at three separate targets, 24/24 trials**, while the ordinary primed path failed its
first byte in the same boot. Setting the prime count to zero fixed the ordinary path
outright (0 errors, repeat failures 1/20 → 0/20).

Priming does not clear bus residue — it replaces it.

### Position-locked, value-random transfer failures were instruction-fetch stalls

A failure locked to byte 0 of every transfer group, with a nondeterministic wrong value,
was not electrical. It was an L1 instruction-cache miss delaying the GPIO access that
immediately follows a `WAIT_UP_TO_CYCLE` deadline.

Supporting measurements:

- margin arithmetic: a PHI2 half-cycle is ~684 ARM cycles; the read eye's margin to its
  upper edge was ~120 cycles = **86 ns**. An L1I miss served from L2 costs ~15–25 cycles
  (11–18 ns, harmless); one served from DRAM costs ~140–210 cycles (**100–150 ns**, fatal).
  Branch misprediction (~3 cycles, and the A53 has no BTB), store-buffer drain, register
  allocation and PMU reads are all excluded on magnitude alone.
- the six-byte groups are **loops, not unrolled**: 18 textual `RAD_SPEEK` sites produced
  exactly 18 inlined `busReadByte_p1` bodies (33 if unrolled), with back-edges confirmed in
  disassembly. Iteration 0 is therefore the first-ever execution of a 600–950 byte body
  inside a 25,148-byte function; iterations 1–5 run warm.
- the Cortex-A53 has **no branch target buffer**, so a `WAIT_UP_TO_CYCLE` loop exit is a
  guaranteed fetch redirect landing on the fall-through line with no run-ahead. In the
  known-good out-of-line primitive, that fall-through crosses a 64-byte cache-line boundary
  — the vulnerability exists there too and simply never fires, because the function is
  called hundreds of times per boot and stays resident.

**Practical rules that follow:**

- keep timing-critical bus primitives in a single shared **out-of-line** implementation
  that is called often enough to stay resident; do not inline them at many sites
- `WAIT_UP_TO_CYCLE` should **detect and report** a missed deadline rather than falling
  through silently — one compare converts an entire class of silent corruption into a counter
- any code added anywhere re-rolls cache-line placement for every wait-exit in the image;
  re-verify timing calibration after unrelated changes

### Cross-core `/DMA` handoff cost ~16 PHI cycles

When refresh pulses ran on core 3 and bus traffic on core 0 with a negotiated handshake,
the measured handoff wait and the measured late-close were **the same event**: 22,199 vs
22,231 ARM cycles in one boot and 22,235 vs 22,270 in another — differing by 32 and 35
cycles, and stable to 0.16% across independent boots. Both are **16.2–16.3 PHI cycles**.

A store-then-load handshake between two cores is the AArch64 store-buffer litmus pattern:
release/acquire semantics do **not** prevent store-load reordering, and a full `DMB` is
required on both sides.

## 7. Documented behaviour that our measurements contradict

Recorded because the discrepancy is itself a fact, and because anyone reading the
schematics will hit it.

Per the C128/128D Service Manual and schematic 310378:

- the C128 DRAM **row** address is `A0–A7` (the low byte); the column is `TA8–TA15`
- VIC-IIe refresh is internal — VMA0-7 → MA0-7, `/RAS` from the VIC — and active only
  while AEC is low, with the 74LS257 address multiplexers tri-stated by `/AEC`
- expansion-port `/DMA` pulls the 8502's AEC/RDY, the Z80's `/BUSRQST`, and GAEC into
  the MMU

**That model predicts that holding `/DMA` cannot block refresh. Section 2 measures that it
does.** The mechanism is therefore not fully explained; the empirical result stands on its
own and should be trusted over the schematic model until the discrepancy is resolved.

Also noted from research and not verified here: the real CMD SuperCPU 128 used an MMU
adapter connection that the RAD cartridge does not have. Anything requiring native C128
mode should investigate that before assuming expansion-port access is sufficient.

### Machine-to-machine margin differs

Stated precisely, because it is easy to over-read. The same card at a per-line traffic width of
4 was run on two different C128s:

| Machine | Result |
|---|---|
| C128 with a CMD MMU adapter fitted (8722 seated in the adapter, SuperCPU-side header unconnected) | sparse cells still change on a settled static screen |
| A different, stock C128 | static screen holds — though slow, with a corrupted boot animation, which is a bandwidth symptom rather than decay |

**This establishes that refresh margin differs between machines. It does not isolate the adapter
as the cause.** DRAM lot, VIC-IIe revision, board revision, bus loading and calibration all
differ between the two boards; the adapter is one candidate among several. The test that would
isolate it — removing the adapter from the *same* board, reseating the 8722, and re-running an
identical image — has not been done.

Note also that every schedule figure in §4 was measured on the adapter-equipped machine. The
stock machine has exactly one data point and no upper bound.

## 8. Operational notes

- The C128 must be put into C64 mode physically (hold C= at power-on). The `C128_MODE`
  config key only selects the emulator's C64-compatible SCPU path; it cannot change the
  physical MMU.
- C128 detection: `SPEEK($D030)` returns `$FE` or `$FC` on a C128 (upstream
  `checkForC128()`).
- Read-timing calibration doubles as an **incidental retention test**. It performs
  65 sample points × 12 repetitions × 7 reads = 5,460 reads over 5.5–11 ms against a
  seeded oracle. If DRAM is decaying while it runs, the oracle rots and no sample point is
  ever clean — so a `NO-STABLE-WINDOW` verdict has repeatedly correlated with a boot whose
  refresh regime was already bad, and an eye being found has correlated with a healthy one.
  Treat it as an early refresh-health warning, not only as a calibration result.
- When a read eye *is* reported, check whether its edges are real: a bound equal to the
  configured scan floor or ceiling is censored, not measured.
- **A running screen is not a clean retention oracle.** `CWriteBuffer::resyncSweep()` delivers
  *clean* shadow bytes — it rewrites screen cells that have not changed — and a write refreshes
  the cell it targets, so mirroring repairs decay while you are watching for it. Its ceiling is
  modest (64 bytes per raster-safe opportunity at 60 Hz ≈ 3,840 bytes/s, roughly 260 ms per
  1000-byte pass), so it shortens an error's visible lifetime rather than hiding it outright —
  but any retention verdict taken from a live screen should first halt mirroring.
  `MIRROR_HALT_AFTER_S` does exactly that and gates all three runtime mirror sites; immediate
  VIC/CIA traffic and the refresh service keep running.
- Halting the mirror does **not** flatter the result: the refresh scheduler opens its traffic
  interval unconditionally on its own timer, with no reference to whether the emulator has work
  pending, so the refresh blackout occurs whether or not mirroring is active.
- **K198 stock-C128 no-repair observation.** With the permuted width-8 scheduler still running,
  the same static A-Z screen showed no visually identifiable changed cells in photographs taken
  about ten minutes apart after `MIRROR_HALT_AFTER_S` fired. This establishes that the width-8
  schedule preserves idle physical DRAM on that machine. It also confines the remaining runtime
  corruption to traffic suppressed by the mirror halt. K199 therefore disables only the
  background `resyncSweep()` on a detected physical C128, leaves ordinary dirty-buffer delivery
  enabled, and removes the timed halt. This is an isolation experiment, not yet a general claim
  that all C128 background resynchronization is unsafe.
- **K199 result.** The boot animation became correct, but an otherwise blank idle BASIC screen
  acquired five reverse-video cells and the `PRINT TI` loop still stuttered slightly. Disabling
  `resyncSweep()` therefore did not remove the fault. Reverse-video cells are a useful signature:
  they may be cursor-blink values delivered to wrong addresses, or bit-7 retention failures caused
  by traffic-window overrun. Read the K199 per-phase overrun counters before changing timing; zero
  overruns selects the address-latch hypothesis, while busy extensions select scheduler closure.
- **K202 resolved that fork.** During ten real seconds of boot/BASIC/partial command traffic, the
  line scheduler opened 258,433 epochs. It recorded 1,517 deadline recoveries and a maximum busy
  extension of 60,582 ARM cycles: about 44 PHI periods at the measured 1,367 ARM cycles/PHI. The
  maximum busy wait (60,542) independently matches that extension. This is not address-latch
  timing: a transaction claims the epoch, encounters BA low, and prevents core 3 from resuming
  refresh through a VIC DMA interval. One screen cell corrupted during the same run.
- **K203 corrective handshake.** Physical C128 reads and writes now align and sample BA before
  claiming `g_C128TrafficBusy`, and claim only with three measured PHI periods remaining before
  the published epoch deadline. BA-blocked or late operations retry in another traffic window
  without holding refresh closed. The calibrated address/data waveform is unchanged.

## 9. Diagnostic methodology that proved necessary

Recorded because each of these was learned by losing hardware rounds.

- **Never gate a measurement behind the subsystem it measures.** A retention ladder gated
  behind a self-test that had not passed in ten revisions produced no data for ten
  revisions. A gate should *label* a result `UNTRUSTED`, not discard it.
- **Exact accounting proves emission, not effect.** A refresh sweep emitting exactly the
  expected 17,095 pulses at exactly the expected duration still failed to preserve DRAM
  (E18L). Count and duration are necessary, not sufficient.
- **Print `NOT RUN` rather than zeros for unexecuted diagnostic sections.** An unrun section
  dumping zero-initialised arrays reads as a perfect result, and an all-`$00` expected
  pattern trivially matches an all-`$00` read.
- **Retain per-pass counts, not just classified totals.** A `0 → 5 → 8` progression across
  three read passes cannot be produced by a zero-exposure transition, and that signature is
  what localised a harness bug that aggregate counts had hidden for three revisions.
- **A control built from the machinery it is checking cannot catch a defect in that
  machinery.** Validate against an independently proven primitive.
- **Scan order is a confound.** A result whose banding aligns with execution position rather
  than the swept parameter must be re-run in a decorrelated order — a coprime stride
  distinguishes by shape, whereas a simple reversal may leave only a handful of
  discriminating points. This is not hypothetical: an ascending-order scan produced a broad
  48-edge "critical band" that a coprime-stride re-run reduced to seven edges, and the broad
  band would have driven a production schedule.
- **A metric that yields physically impossible values is measuring noise.** A pulse-width scan
  reported two of seven rungs with *negative* damage — fewer failures after exposure than at
  baseline — which refresh cannot produce. That invalidates the comparison rather than merely
  weakening it.
- **A between-machine comparison does not isolate a component.** Swapping cards between two
  boards that differ in several respects measures the boards, not the one part you were
  thinking about. Isolation requires changing that part on a single board.
- **Prefer a deterministic permutation to a PRNG** when decorrelating a swept parameter: a step
  coprime to the modulus visits every value exactly once per cycle, has no modulo bias or
  unverified short cycle, and is auditable from the log.
