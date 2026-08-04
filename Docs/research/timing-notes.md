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
