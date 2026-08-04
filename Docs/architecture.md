# Architecture

## The idea in one paragraph

A CMD SuperCPU works by asserting `/DMA` so the C64's 6510 stops driving the
bus, then executing the program itself out of fast RAM it owns, only touching
the C64 for I/O and to keep VIC-visible memory coherent. A RAD Expansion Unit
can do all of that: it is a Raspberry Pi with level shifters on the expansion
port, and RAD-Doom already proved the takeover works by running Doom on the ARM
and POKEing frames into screen memory. SCPU-EMU keeps the takeover and replaces
the payload: instead of a game, the Pi runs a CPU emulator.

## Dependency flow

```
App
 └── SuperCPU                     registers, mirroring policy, speed
      ├── CPU/                    M6502 now, W65C816 next
      ├── C64/                    banking, shadowed bank 0
      └── Bus/                    IC64Bus
           ├── Bus/RAD/           real hardware
           └── Bus/Host/          in-memory, for PC tests
```

Boundaries that are enforced, not just intended:

- **`CPU/` knows nothing about GPIO, RAD or the C64.** It reads and writes
  through `cpu_bus.h` and observes two interrupt lines. That is the whole
  interface.
- **`C64/` models the machine**, not the cartridge: the PLA banking table and
  the 64KB the program sees.
- **`SuperCPU/` is the product** — registers, write buffering, optimization
  modes, speed policy.
- **`Bus/RAD/` holds everything timing-critical.** It is the only place that
  touches GPIO or the ARM cycle counter.
- **`Bus/Host/` means none of the above needs hardware to be tested.**

## Where the cycles go

This is the only performance fact that matters:

| Operation | Cost |
|---|---|
| Emulated CPU cycle | ~50ns (interpreted on a 1GHz A53) |
| C64 bus access | ~1µs |

A bus access costs about twenty emulated cycles. So:

- **Reads of RAM and ROM never leave the Pi.** Bank 0 is shadowed
  (`C64/c64_memory`). Instruction fetch is free.
- **I/O reads and writes must leave the Pi.** VIC-II, SID, CIA and colour RAM
  are real chips. This is unavoidable and the real SuperCPU pays it too.
- **RAM writes must be mirrored**, because the VIC-II fetches from DRAM. This is
  the bottleneck, and `SuperCPU/write_buffer` is the mitigation:
  - *coalescing* — a dirty-address bitmap, so a hundred writes to one address
    cost one bus cycle, and a screen clear costs 1000 no matter how it was done;
  - *optimization modes* — the SuperCPU's own mechanism for declaring which
    regions the VIC actually reads, so the rest is never mirrored at all.

One ordering rule keeps this correct: **the buffer is flushed before any I/O
access.** Without it a program could stage a screen and then flip `$D011` to it
while the data is still buffered. Enforced in `c64_memory.cpp` and tested in
`Tests/Integration/test_kernal_boot.cpp`.

## Boot sequence

1. Circle brings up GPIO, the ARM cycle counter, SD card (`App/main.cpp`).
2. Read `SCPU/scpu.cfg` for RAD bus timings, snapshot into the cache-resident
   `busTiming` block.
3. Optionally load `basic.rom` / `kernal.rom` / `chargen.rom` from SD.
4. Unmount the SD card — the FAT driver takes interrupts and allocates, neither
   of which is safe once we are driving the bus to a cycle budget.
5. Reset the C64 and let its KERNAL boot, so `$01` settles at `$37`.
6. Wait for a badline, assert `/DMA`. The 6510 is now off the bus.
7. Detect C64 vs C128, PAL vs NTSC.
8. If no KERNAL image was supplied, read `$A000-$BFFF` and `$E000-$FFFF` off the
   live machine into shadow RAM (16384 bus cycles, ~16ms).
9. Reset the emulated CPU, which fetches its vector from shadow RAM, and run.

## Interrupts

The VIC-II and CIAs still raise `/IRQ` and `/NMI` on the real machine. The RAD
backend reads those lines straight off GPIO, which costs nothing, so the
emulated core can sample them at every instruction boundary. NMI is edge
triggered and IRQ level triggered, as on real silicon.

## Known gaps

- The 65816 core is not written yet; milestone 1 runs the 6502 core. Banks other
  than 0 therefore do not exist yet.
- Cycle *pacing* is not implemented: `runFrame()` computes a budget but nothing
  yet holds the emulated CPU to real time, so raster-dependent code will not
  behave until `SuperCPU/timing` lands.
- The character ROM cannot be snapshotted off the machine — see
  [research/supercpu-memory-map.md](research/supercpu-memory-map.md).
- None of the hardware paths have been run on real hardware yet. See
  [roadmap.md](roadmap.md).
