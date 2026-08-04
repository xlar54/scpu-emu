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
- [x] Host test suite (1203 checks)
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
- [ ] **Runs on real hardware.** Nothing below this line is meaningful until a
      physical C64 reaches that prompt with the Pi driving it. The host bus is
      not a C64: it has no CIAs, no real VIC-II (only a synthesised raster
      counter), and no timing.
- [ ] Timing constants validated per Pi model
- [ ] Cycle pacing (`SuperCPU/timing`) so raster code behaves

## Milestone 2 — the 65816

- [ ] `CW65C816`: emulation and native modes, 8/16-bit M and X, the full
      addressing-mode set, bank wrapping, decimal mode
- [ ] Differential testing against `CM6502` — in emulation mode the two cores
      must agree instruction for instruction, which is what
      `Tools/trace_compare` is for
- [ ] Banks 1+ and the SuperRAM model (`SuperCPU/fast_ram`, `memory_map`)
- [ ] Long addressing, `MVN`/`MVP` block moves with range invalidation into the
      write buffer

## Milestone 3 — fidelity

- [ ] Speed switching actually changing execution rate
- [ ] Optimization modes exercised by real software (GEOS is the interesting case)
- [ ] `$D0B0`/`$D0BC` detection confirmed against hardware or VICE — the sources
      disagree on which address in `$D0Bx` carries the flag
- [ ] Character ROM capture via an injected Ultimax stub
- [ ] SuperCPU ROM image support (SuperCPU DOS, JiffyDOS)

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
