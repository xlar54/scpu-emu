# Timing notes

## The budget

| | |
|---|---|
| PAL C64 cycle | 1016ns (985248Hz) |
| NTSC C64 cycle | 978ns (1022727Hz) |
| RPi Zero 2 / 3A+ core cycle | ~0.7ns at 1.4GHz |
| Emulated 65816 cycle, interpreted | ~50ns |

About 1400 ARM cycles per C64 cycle. That is the window every bus access has to
fit in, and it is why `Source/Bus/RAD/` times against `PMCCNTR_EL0` rather than
any OS facility.

At ~50ns per emulated cycle an interpreter clears 20MHz with room to spare, so
the 65816 core does not need dynamic recompilation. **A bus access costs about
twenty emulated cycles**, which is the entire reason `CWriteBuffer` exists.

## Rasterline arithmetic

| Standard | Lines | Cycles/line | Cycles/frame |
|---|---|---|---|
| PAL | 312 | 63 | 19656 |
| NTSC 6567R56A | 262 | 64 | 16768 |
| NTSC 6567R8 | 263 | 65 | 17095 |

A badline steals 40-43 cycles, so eight consecutive lines cost about
`63*7 + 23 = 464` cycles rather than 504. RAD-Doom uses exactly that figure to
predict how long a blit will take and schedule it into the vertical blank.
`CWriteBuffer` will want the same trick once cycle pacing lands.

## Detection heuristics carried over from RAD

- **Machine running**: time 1000 C64 cycles; expect 1.2-1.6M ARM cycles. Outside
  that window, the machine is not up.
- **PAL vs NTSC**: sample `$D012` plus `$D011` bit 7 for a frame and keep the
  maximum. PAL reaches 311, NTSC only 261 or 262.
- **C64 vs C128**: `$D030` exists on a C128 and reads `$FF` on a C64.

All three are in `Source/Bus/RAD/cpu_hijack.cpp`.

## Where /DMA may be asserted

Only during a badline, when the VIC-II has taken the bus from the 6510 anyway.
RAD waits for BA low, then asserts `/DMA` 80ns after the falling edge of PHI2.

## Constants that must not be touched casually

From `Source/Bus/RAD/lowlevel_dma.h`, inherited from RAD:

- R/W is driven ~40 cycles *before* the address latch OE and DIR. The original
  comment records that this took a long time to work out and that old VIC-II
  revisions need it; newer ones work without.
- Writes re-check BA mid-burst and resync if a badline starts partway through.
- Timing constants were tuned at `-Ofast`. Changing optimisation flags moves them.

## Historical issue, fixed: burst transient `(lo, lo)` address

An earlier `busWriteByteBurst_p1()` enabled `bLATCH_A_OE` before both address
halves had been clocked. Because the two transparent latches shared D0-D7, the
C64 briefly saw `(lo, lo)` -- for example, a write to `$04D0` exposed `$D0D0`.
This was a real defect and the original section correctly warned that increased
mirror traffic could turn it into VIC-II fetch corruption.

The current `Source/Bus/RAD/lowlevel_dma.h` has fixed it. The burst path now:

1. clocks the low address into `LATCH_A0`;
2. clocks the high address into `LATCH_A8`;
3. waits through BA arbitration; and
4. enables `bLATCH_A_OE` only at
   `TIMING_ENABLE_RWOUT_ADDR_LATCH_WRITING`.

The latch outputs therefore never expose the shared GPIO byte until both halves
hold the intended address. Single reads and writes retain their corresponding
timed-enable behavior. Keep this ordering invariant; moving address OE back into
the opening GPIO operation would recreate the defect.

This correction matters diagnostically: the equal bitmap-sparkle rate measured
with the K238 scrub off and on cannot be attributed to the old transient. K238
added roughly 118KB/s of burst writes without changing the fault rate.

The stock RAD firmware has no burst path at all -- `emuWriteByteMany` does not
appear in `_radmain/`. It is a RAD-Doom addition, so hardware validation still
needs to cover the SCPU-EMU implementation directly.

## Physical-C64 display-RAM investigation

K240 ran the same composite bitmap oracle on two real C64s. One machine
reliably changed `$xxF8/$xxF9` only when those locations contained mixed
high-entropy data; the other completed all twelve idle/read/write arms with no
changed byte. The write-resume bias sweep was flat. This makes the observed
sparkle a machine-specific marginal-DRAM or motherboard condition, not a
general RAD write-timing defect. Do not tune production bus timing around the
failing machine. The complete results are retained on the test card as
`SCPU/busdiag-k240-machine-a.txt` and `SCPU/busdiag-k240-machine-b.txt`.

## Read calibration observation

The ordinary-read calibration currently preserves its original selection rule:
twelve repetitions each read six RAM bytes and then VIC register `$D020`, and
only a zero-error combined sample is accepted. Two physical C64s produced an
irreducible floor of exactly twelve errors across a wide range of sample points
while their subsequent RAM self-tests passed.

K242 recorded, without changing selection:

- RAM and mixed-sequence VIC errors separately at every sample point;
- an isolated `$D020` sweep, primed so no measured VIC read follows RAM;
- distinct and dominant VIC values for both sequences; and
- all twelve mixed and isolated VIC values at the best RAM sample.

Machine B showed `$D020 = $F6` in all twelve mixed reads and all twelve isolated
reads from sample 330 through 620. Its masked low nibble was therefore the
expected `$06`; every one of the twelve plateau errors came from RAM instead.
Because each repetition ended with `$D020` and then began again at `$0334`, the
shape suggests the first RAM access after a VIC-device boundary rather than a
failed VIC sample point.

K243 retains production selection unchanged and adds three observation-only
checks:

- a separately primed RAM-only error curve at every sample point;
- rotation of `$0334-$0339` through all six positions in the mixed chain, with
  failures attributed independently by position and physical address; and
- the actual first byte, immediate reread, and copied byte zero for both the
  `$E000` KERNAL and `$A000` BASIC physical ROM snapshots.

If mixed failures follow position zero while the RAM-only chain gains an
error-free window, calibration is manufacturing a VIC-to-RAM turnaround error.
If they follow one address instead, the fault is address-specific. The ROM
probe deliberately preserves the first unprimed byte in the snapshot before
performing its side-effect-free reread; it observes whether snapshot byte zero
needs a block-local prime without silently applying that fix.

Machine B's K243 rotation produced `2 2 2 2 2 2` errors by position but
`12 0 0 0 0 0` by address. The later raw probe read `$0334 = $00` sixteen times,
then the ordinary self-test successfully wrote and read `$3C` there. This was
not a weak address: the calibration's first `$0334 <- $3C` seed write had been
lost. After machine detection ended with reads, the C64 acquisition branch had
incorrectly cleared `busWriteTurnaroundNeeded` before calibration; the C128
branch already initialized it correctly. K244 sets the C64 flag as well, causing
the existing single-write primitive to pay its configured one idle turnaround
pass. It changes no timing constant or runtime policy.

K244 showed that the direction-state correction and idle pass alone did not
make that first write land: `$0334` remained `$00` and the rotation was
unchanged. K245 therefore consumes the unreliable first transaction with one
sacrificial write to harmless cassette-buffer RAM at `$02FE` before seeding the
oracle. This follows the already hardware-proven self-test pattern. It neither
duplicates a potentially side-effecting I/O write nor changes the calibrated
addresses, selector, timing constants, or runtime access policy.

Machine B confirmed K245: the combined eye was error-free from 330 through 620
and retained the configured midpoint 475; mixed RAM, RAM-only, rotated-position,
and rotated-address errors were all zero. The ordinary single and burst
self-tests also passed. This closes the former 12-error calibration floor as a
lost first seed write, not a read-sampling defect.

The same K245 image and universal configuration then cold-booted the FPGA
Commodore 64U flawlessly. Its first SuperCPU animation was stable and it no
longer required a second RAD reset. Separate C64U firmware/configuration is not
part of the production path; older `kernel8-64u.img` and `config-64u.txt` files
on development cards are historical artifacts only.
