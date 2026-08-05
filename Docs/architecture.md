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

Ordinary I/O never flushes this buffer synchronously: doing that issued bursts
across visible VIC-II fetches and caused flicker. The frame scheduler drains
queued RAM in at most 64-byte chunks, rechecking the raster before every chunk;
display-register writes remain immediate, so raster programs keep their exact
timing. Rewriting one of the possible sprite-pointer bytes moves that commit
behind the newer shape data, so the physical VIC cannot select that data before
it has reached C64 DRAM.

## Boot sequence

1. Circle brings up GPIO, the ARM cycle counter, SD card (`App/main.cpp`).
2. Read `SCPU/scpu.cfg` for RAD bus timings and the `CPU_CORE`, `BOOTMAP`, and
   `JIFFYDOS` selections, then snapshot timings into the cache-resident
   `busTiming` block.
3. Optionally load `basic.rom` / `kernal.rom` / `chargen.rom` from SD.
4. Unmount the SD card, emit the final startup diagnostics, then mask ARM IRQs.
   The FAT driver and timer interrupts may preempt or allocate, neither of which
   is safe once GPIO bus phases have sub-microsecond deadlines.
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

- The 65816 core has not yet driven physical hardware — only the 6502 core has.
  `CPU_CORE` in `scpu.cfg` selects between them.
- `MVN`/`MVP` into shadowed bank 0 produce one mirrored write per byte. A 64KB
  block move is a single instruction that dirties the whole bank, and the write
  buffer needs a range-invalidate path rather than 65536 individual entries.
- Two questions about the chip are open rather than settled, and both are
  testable on hardware in minutes: whether the direct-page pointer of `(d)`,
  `(d,X)` and `(d),Y` wraps in-page in emulation mode, and whether the
  SuperCPU's gate array forwards the emulation-mode read-modify-write dummy
  write. See [research/65816-reference.md](research/65816-reference.md)
  section 1.3.
- The character ROM cannot be snapshotted off the machine — see
  [research/supercpu-memory-map.md](research/supercpu-memory-map.md).
- None of the hardware paths have been run on real hardware yet. See
  [roadmap.md](roadmap.md).
