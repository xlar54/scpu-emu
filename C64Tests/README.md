# SCPU-EMU hardware test programs

These programs run on a real C64 with SCPU-EMU (and should also be useful on a
CMD SuperCPU or VICE's SCPU64 target). They complement the host-side C++ tests:
the programs here exercise the complete Pi/RAD/C64 path, including VIC-II
visibility and wall-clock timing.

## Quick start

Build the PRG files on Windows:

```powershell
.\C64Tests\build.ps1
```

Bundled copies of `petcat` and `c1541` are used automatically; `ca65` and `ld65`
must be on `PATH` for the ML sources. The generated programs are placed in
`C64Tests/bin`, and the script creates the ready-to-mount
`C64Tests/SCPU-TESTS.d64` disk image. Load and run each program normally:

```basic
LOAD"00-DETECT",8,1
RUN
```

Machine-language tests contain their own one-line BASIC loader, so `RUN` works
for every PRG. With the physical speed switch in **Turbo**, run the programs in
the order below. A normal C64 can run the display tests, but the SCPU-specific
tests will report that no accelerator was detected.

## Test matrix

| Program | Type | What it checks | Passing result |
|---|---|---|---|
| `00-DETECT` | BASIC | `$D0BC` presence and `$D0B0/$D0B2/$D0B5/$D0B8` status | `SCPU DETECTED` and status bytes with a bit legend |
| `01-SPEED` | BASIC | software Normal/Turbo requests, elapsed speed, and effective MHz estimate | Turbo jiffies are lower and estimated MHz is substantially above 1.0 |
| `02-PRIVATE` | ML | write protection and private SRAM at `$D200-$D2FF` and `$D300-$D3FF` | `PASS` for the gate and both pages |
| `03-SCREEN` | BASIC | screen RAM, colour RAM, borders, and visible mirroring | coloured 16x16 character field, then `PASS` |
| `04-RASTER` | BASIC | stable 9-bit VIC raster reads and jiffy-clock progress | range near 0-311 PAL or 0-262 NTSC, and roughly 50/60 frames |
| `05-ANIMATION` | BASIC | resident SuperCPU DOS 2.04 startup-animation entry | animation replays, then BASIC cold-starts |
| `06-CUBE` | ML | caches 16 hires wireframe frames in SuperRAM banks `$02-$03`, then animates with `MVN` | white wireframe cube rotates on black; key exits |
| `07-MANDELBROT` | ML | Q8.8 fixed-point Mandelbrot renderer with selectable 1 or 20 MHz mode | monochrome set and escape-time contours; Space exits |
| `08-SOUNDTEST` | ML | SID voices, triangle/saw/pulse/noise waveforms, chord, and filter sweep | each labelled sound is audible; Space stops |
| `10-CPU816` | ML | native mode, 16-bit accumulator/index, ADC, stack, and return to emulation | `CPU 65816: PASS` |
| `11-SUPERRAM` | ML | 24-bit long reads/writes in banks `$01,$02,$7F,$F5` | one `PASS` per bank (16 MB configuration expected) |
| `12-RASTERIRQ` | ML | VIC raster IRQ delivery while running Turbo | stable bars and increasing IRQ count; key exits |
| `13-VICBANKS` | ML | VIC banks 0-3 and accelerator write mirroring | four stable solid-colour screens; key advances |
| `14-SPRITEBALLS` | ML | all eight sprites, ninth X bits, frame pacing and VIC writes | eight differently coloured balls bounce smoothly; key exits |
| `15-SCPU128PROBE` | ML/C128 | read-only SCPU128, 8722 MMU, ROM-window and 24-bit bank fingerprints | creates `SCPU128 LOG` as a sequential file on device 8 |
| `16-NMITIMING` | ML | CIA2 Timer-A one-shot timing, native-window delivery and 1024-edge stress | repeatable sweep signatures, zero misses, and native/deferred totals adding to `$40` |

`01-SPEED` reports raw jiffies, the Normal/Turbo ratio, and an effective MHz
estimate obtained by treating the forced-Normal run as the 1.0 MHz reference.
It is a workload estimate, not a frequency-counter reading: BASIC overhead,
video standard, firmware build, and attached devices all affect the result.
Turbo should nevertheless take fewer jiffies. If it does not, check that the
physical switch permits Turbo and that `$D0B8` bit 6 is clear.

`02-PRIVATE` masks interrupts while testing because `$D200-$D2FF` is live
SuperCPU DOS system RAM. It opens the hardware-register bank only for one
write/read/restore transaction at a time and closes it before advancing, so the
KERNAL window is never left switched while BASIC or an IRQ handler is running.
The original contents of both pages are preserved.

`05-ANIMATION` is specifically for SuperCPU DOS 2.04. Because the animation's
RAM image is self-modifying, the launcher uses a five-byte 65816 trampoline to
enter the ROM's `$F8:01E0` copy-and-launch routine. That refreshes
`$0E00-$1C00` before every replay. The ROM cold-starts BASIC when it finishes
instead of returning to the program. See `research/startup-animation.md`.

`06-CUBE` is a 65816 ML demo. It renders 16 perspective-projected wireframe
frames once, caches the resulting 128 KB of bitmap data in SuperRAM banks
`$02-$03`, then animates by using `MVN` to copy complete cached frames into the
visible bitmap at `$2000`. It temporarily selects the mirror-everything policy
so the VIC sees each copied frame, and restores the prior policy on exit. It
requires at least 128 KB of SuperRAM. Press any key to return to text mode.

`07-MANDELBROT` asks whether to render at forced 1 MHz or Turbo speed before
entering hires mode. Its 65816 renderer uses Q8.8 fixed-point arithmetic and a
runtime-generated square table to draw a 160x100 sample grid expanded to the
320x200 bitmap. Interior points are solid and every fourth escape iteration
forms a monochrome contour band. It scans the keyboard matrix directly while
rendering, so holding Space restores the machine and returns to BASIC without
waiting for the image to finish.

`08-SOUNDTEST` is a short observational SID check. It plays a triangle scale on
voice 1, a sawtooth scale on voice 2, noise on voice 3, a three-voice chord,
and a resonant low-pass filter sweep. It finishes automatically, silences all
SID registers, and returns to BASIC. Press Space to stop early.

`11-SUPERRAM` expects a 16 MB SuperRAM configuration. Bank `$01` is internal
SRAM and must always pass. A failure above the installed SIMM size is expected;
the displayed bank tells you where it occurred. The test restores the original
byte after each probe.

`12-RASTERIRQ` is observational because accelerated raster code is not
cycle-exact by design. The display should remain stable, the border bands should
alternate, and the hexadecimal IRQ counter should continue changing. Press any
key to restore the VIC/CIA state and return to BASIC.

`13-VICBANKS` fills one screen and a small RAM character set in each 16 KB VIC
bank, switches CIA2's VIC-bank bits, and asks the accelerator to use the
matching optimization mode. This is a direct end-to-end test of the shadow-RAM
write policy. Press a key after each solid-colour screen.

`14-SPRITEBALLS` uses all eight hardware sprites over a black screen. Each ball
has its own colour, position and velocity; several cross X=255 to exercise
`$D010`. Press any key to restore the VIC state and return to BASIC.

`15-SCPU128PROBE` is a native C128-mode program with a BASIC 7 loader at
`$1C01`; unlike the other tests, it must be run before `GO64`. It is deliberately
read-only apart from briefly opening and closing the documented SuperCPU
hardware-register gate. It captures the closed/open `$D0B0-$D0BF` views, the
8722 registers at `$D500-$D50B` and `$FF00-$FF04`, visible ROM-window CRCs, and
256-byte fingerprints reached through 65816 banks `$00` and `$01`. The report
is written as `SCPU128 LOG` on device 8, replacing the previous report. Make a
fresh copy after each run and repeat after cold boots in C128 40-column, C128
80-column, Normal and Turbo configurations. Also record the machine revision,
PAL/NTSC standard, SCPU ROM revision, physical switch position and SIMM size.

`16-NMITIMING` is for side-by-side RAD, VICE SCPU64 and real-SuperCPU
comparison. Select forced 1 MHz or Turbo at its start screen. It temporarily
banks out the KERNAL and installs direct handlers in RAM under `$FFEA/$FFFA`,
removing the KERNAL prologue from the measurement; it also puts a temporary
`JML` at `$C003` for the SuperCPU EPROM native-vector route. Five CIA2 Timer-A
latch values are sampled 32 times. `MIN`, `MAX`, and `SUM` identify the
instruction boundary where each NMI arrived. Exact signatures can differ
between implementations, but repeated runs on one implementation should be
tight. The native-window line classifies 64 NMIs as taken while `E=0`, deferred
until `E=1`, or missed. The final 1024 varying one-shots must report `OK $0400`
and `MISS $0000`; `SUM` is a comparison signature. Space is checked between
trials, then saved vectors, banking and CIA state are restored before BASIC.

Reference capture from 2026-08-07, before the emulated-cycle deadline fix
(RAD kernel 096): both implementations delivered all 1024 stress edges. VICE
reported native `$40`, deferred `$00`; RAD with `NMI_NATIVE_DEFER=1` reported
native `$00`, deferred `$40`. At Turbo, the latch `$10->$20` captured-X slope
was `$1E->$38` on VICE but only `$0D->$1A` on RAD. Since the measured loop is
about 12 CPU cycles, those slopes correspond to approximately 19.5 MHz and
9.75 MHz respectively. This identified the old RAD wall-clock deadline: an
armed timer forced the interpreter onto its unbatched path, so real time
expired after only half the intended emulated instructions. Kernel 097 changes
CIA deadlines to emulated cycles; I/O stretch now accounts for physical bus
time separately. This reference note preserves the distinguishing signatures
needed to recognize that regression.

## Interpreting failures

- Detection/status failures isolate SuperCPU register decoding.
- A good Normal time but no Turbo improvement isolates speed-request handling.
- A CPU816 failure isolates native 65816 execution before involving SuperRAM.
- A bank `$01` pass followed by SuperRAM failures isolates the 24-bit map/SIMM.
- Correct CPU results but corrupt/stale screens isolate write mirroring or bus
  timing, not the instruction core.
- Raster freezes or missed keys suggest IRQ/NMI sampling or fine-grained C64
  bus pacing.

Sources are intentionally checked in beside generated binaries. Edit the
`.bas`/`.asm` files and rerun `build.ps1`; do not hand-edit files under `bin`.
