# VICE snapshot tooling

Ground-truth forensics against a VICE SCPU64 `.vsf` snapshot. Built for the
3D Pool ball-flicker investigation; general enough to reuse for the next
"works in VICE, glitches on hardware" case.

The snapshot used here is `C64Tests/vice-snapshot-balls.vsf`, taken by the
user in VICE SCPU64 at 3D Pool /SCPU's table screen.

## vsfextract.py

Parses the module directory and writes the two 64K RAM views next to the
snapshot:

- `snap_ram_cpu.bin` — SRAM bank 0, what the 65816 sees (module `C64MEM`,
  52 scalar bytes, then board RAM 64K, SRAM 128K, SIMM 16MB).
- `snap_ram_board.bin` — the C64 board DRAM, what the VIC fetches. In VICE
  these two agree wherever the SCPU's write-through mirrors; diffing them
  against OUR delivered stream is the ground-truth test.

## vsfballs.py

The analysis pass that cracked the flicker: reads the VIC register file out
of the `VIC-II` module (registers start at offset 1), derives screen/bitmap/
pointer-row addresses from `$D018`/`$DD00`, dumps the pointer row and shape
blocks from both RAM views, and scans for the code that writes them
(`STA $D018`, `STA $xxF8,X`, ...). Finding: pointer row `$CFF8`, shapes at
`$D0C0/$D480/$D4C0` — RAM under I/O — and the raster IRQ at `$3096` flips
`$DD00` between VIC banks 3 and 1 every frame (double-buffered bitmaps at
`$E000` and `$6000`).

## snapballs.cpp

Host-rig validation that seeds the emulator's shadow RAM from
`snap_ram_cpu.bin` and lets the game's OWN raster IRQ handler run against
the full delivery stack (writeFast -> write buffer -> border flushes ->
resync sweep). One VIC IRQ per frame; the handler flips banks, rewrites the
active pointer row from `$021D`, and self-modifies its store target -- the
complete ball engine, no game main loop needed (a spin stub at `$0334`
stands in; `$40` is the tables-ready handshake, re-cleared per frame).

Build from the repo root after `make build-tests`:

    g++ -O2 -std=c++14 -fno-rtti -fno-exceptions -DSCPU_HOST_BUILD -I. \
        Tools/snapshot/snapballs.cpp \
        build/host/Source/CPU/M6502/*.o build/host/Source/CPU/W65C816/*.o \
        build/host/Source/C64/*.o build/host/Source/SuperCPU/*.o \
        build/host/Source/Bus/Host/*.o -o build/host/snapballs
    ./build/host/snapballs 600

PASS criteria: every bank-3 frame's delivered pointer row carries relocated
blocks (< $30), bank-1 rows stay raw, and the relocated copies byte-match
both the game's shape source and VICE's board RAM.

Note the seeding caveat inside: memcpy into shadow RAM bypasses the mirror
sink, so the harness must `discard()` the write buffer's synced bitmap and
re-queue everything -- real programs cannot create that state.
