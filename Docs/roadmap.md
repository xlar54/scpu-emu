# Roadmap

The ordering principle: get the *hardware* risk retired before taking on the
*emulation* risk. A 65816 core is a large, well-understood piece of work that
can be written and tested entirely on a PC. Whether a Raspberry Pi can hold a
C64's bus while pretending to be its CPU is neither large nor well understood,
and nothing else matters if it does not work.

## Milestone 1 — stock C64 running on the 6502 core *(code complete, untested on hardware)*

Prove the takeover, the banking model and the mirroring path by booting a stock
KERNAL with the Pi as the CPU.

- [x] RAD framework promoted out of `_radclone` into `Source/Bus/RAD/`
- [x] DMA hijack with badline wait (`cpu_hijack.cpp`)
- [x] `IC64Bus` with RAD and Host backends
- [x] C64 PLA banking model
- [x] Shadowed bank 0 with I/O routed to the bus
- [x] MOS 6502 core, all 151 documented opcodes, NMOS decimal mode
- [x] Write buffer with coalescing and optimization modes
- [x] SuperCPU register block
- [x] Host test suite
- [x] **A genuine C64 KERNAL boots to the BASIC READY prompt** through the core,
      banking model and shadow memory, on the host bus:

      ```
          **** COMMODORE 64 BASIC V2 ****
       64K RAM SYSTEM  38911 BASIC BYTES FREE
      READY.
      ```

      The whole cold start costs 61571 bus cycles — the RAM test, vector setup,
      screen init and BASIC banner all run out of shadow RAM, and only I/O and
      mirrored writes reach the machine. This validates the 6502 core, the PLA
      table, the `$00`/`$01` port emulation and the mirroring path together.
- [x] **Firmware compiles.** `make firmware` fetches the AArch64 toolchain and
      Circle 44.3, applies RAD's Circle settings and produces a ~156KB
      `kernel8.img`. `make sdcard` stages a complete card.
- [x] **Runs on real hardware.** A physical C64 boots to a normal screen with
      the Pi as its CPU. The bus self-test passes: reads are live, single
      read/write is stable 20/20, bursts are clean, and every address line
      verifies. The fix that first made the display hold was suppressing unsafe
      mirrored writes to `$D000-$DFFF` while the physical host still had I/O
      banked in. The completed C64 path now installs the real SuperCPU-style
      `$01=$34` all-RAM map after ROM capture and selects I/O with `/GAME` only
      for genuine chip accesses, so VIC-visible DRAM under I/O is coherent too.
- [x] Cycle pacing so raster code behaves, plus the IEC auto-throttle on
      `$DD00` writes
- [ ] Timing constants validated per Pi model

## Milestone 2 — the 65816 *(code complete, untested on hardware)*

- [x] **Reference data derived and cross-checked before writing any code.**
      `Docs/SuperCPU64/65816-reference.md` — the WDC datasheet, Bruce Clark's
      *65C816 Opcodes*, VICE's `65816core.c` and SingleStepTests, reconciled,
      with the disagreements recorded as open questions rather than papered
      over. Three of its findings contradicted what was already in the repo.
- [x] `CW65C816`: emulation and native modes, 8/16-bit M and X, the full
      addressing-mode set, bank wrapping, 65C02-style decimal mode
- [x] Its own opcode table (`w65c816_opcodes.cpp`), never derived from the
      6502's. 50 of the 105 encodings the 6502 leaves undocumented have a
      *different length* on a 65816, and getting one wrong desynchronises the
      instruction stream into corruption a long way from its cause.
- [x] Differential testing against `CM6502` — in emulation mode the two cores
      agree instruction for instruction on registers, flags, cycle counts and
      every byte of bus traffic. The exemption list is explicit
      (`Tests/CPU/test_w65c816_diff.cpp`); anything not on it that differs is a
      bug.
- [x] Banks 1+ and the SuperRAM model (`SuperCPU/fast_ram`, `memory_map`)
- [x] Long addressing, `MVN`/`MVP` block moves
- [x] **A genuine C64 KERNAL boots to READY with the 65816 driving**, then
      switches to native mode and reaches 8MB up in SuperRAM without touching
      the expansion port (`Tests/Integration/test_kernal_65816.cpp`).
- [x] Selectable from the SD card via `CPU_CORE` in `scpu.cfg`, so a bad run can
      be backed out without a rebuild. Defaults to the 65816.
- [x] **Runs on real hardware.** The full stack: the CMD splash animation
      (double-buffered, raster-interrupt driven), the SuperCPU DOS banner with
      JiffyDOS, 16MB detected by CMD's own sizing code, a stable display, and
      WORKING IEC DISK ACCESS against both a JiffyDOS FD-4000 and a stock 1571.
      The disk chain alone consumed eleven firmware builds; the fixes that
      mattered: the $D0B2 kernal-window trampoline, immediate speed-register
      pacing, serial-activity-gated mirror suppression (pauses have protocol
      meaning: >200us means EOI), immediate display-register writes, and
      raster-scheduled mirror drains.
- [ ] `MVN`/`MVP` range invalidation into the write buffer. A block move into
      shadowed bank 0 can dirty 64KB in one instruction; the buffer currently
      handles that as 65536 individual mirrored writes.
- [ ] The open questions in the reference doc, section 1.3 — chiefly **U1**
      (the direct-page pointer wrap, decided one way and pinned by a test) and
      **U2** (whether the SuperCPU's gate array forwards the emulation-mode RMW
      dummy write). Both are testable on hardware in minutes.

## Milestone 3 — fidelity

- [ ] Speed switching actually changing execution rate
- [ ] Optimization modes exercised by real software (GEOS is the interesting case)
- [ ] `$D0B0`/`$D0BC` detection confirmed against hardware or VICE — the sources
      disagree on which address in `$D0Bx` carries the flag
- [ ] Character ROM capture via an injected Ultimax stub
- [x] SuperCPU ROM image loaded from the SD card and readable at `$F80000`
      (`SCPU/scpu.rom`, staged from SuperCPU DOS 2.04)
- [ ] **Bootmap**: mapping the SuperCPU ROM over bank 0 at reset so its own code
      runs before the C64's KERNAL, which is what a real card does. Held back
      deliberately — it changes power-on behaviour before the machine can read
      `scpu.cfg`, so a mistake could not be backed out from the SD card. Needs
      the reset sequence verified against VICE's `scpu64mem.c` first.

## Milestone 4 — beyond a stock machine

- [ ] C128 support
- [ ] NTSC timing validation
- [ ] REU coexistence, or reuse of RAD's REU emulation alongside the accelerator

## Deliberately out of scope for now

- Cycle-exact VIC-II behaviour. The real SuperCPU is not cycle-exact either;
  raster-split demos are documented as unreliable on it.
- Ultimax-mode cartridges. Architecturally incompatible with an accelerator, on
  real hardware as much as here.
- Fastloaders that bypass the KERNAL. Documented as failing on real hardware.
