# ROMs

**No ROM images are committed to this repository, and none are required.**

## Local development set

For testing, this directory is populated locally (and git-ignored). To recreate
it:

```sh
C=http://www.zimmers.net/anonftp/pub/cbm/firmware/computers/c64
curl -sSLO $C/kernal.901227-03.bin      # sha1 1d503e56df85a62fee696e7618dc5b4e781df1bb
curl -sSLO $C/basic.901226-01.bin       # sha1 79015323128650c742a3694c9429aa91f355905e
curl -sSLO $C/characters.901225-01.bin  # sha1 adc7c31e18c7c7413d54802ef2f4193da14711aa
cp kernal.901227-03.bin kernal.rom
cp basic.901226-01.bin  basic.rom
cp characters.901225-01.bin chargen.rom

M=https://www.zimmers.net/anonftp/pub/cbm/firmware/misc/cmd
curl -sSLO $M/scpu-dos-2.04.bin         # 128K SuperCPU DOS, sha1 6aa529a7b1b6de53e8979e407a77b4d5657727f5
curl -sSLO $M/scpu-dos-1.4.bin          # 128K, earlier revision, sha1 3422a7735f0f959d990ab39512d8815a5d8eab7a
```

`Tests/Integration/test_real_kernal.cpp` uses `kernal.rom` and `basic.rom` to
boot a genuine KERNAL through the CPU core. Those tests report as *skipped*
rather than failing when the files are absent, so a fresh clone still goes
green.

The SuperCPU DOS images are 128KB and are not used yet — they become relevant in
milestone 3, when SCPU-EMU can run the accelerator's own ROM instead of the
host's KERNAL. See [../Docs/roadmap.md](../Docs/roadmap.md).

## Default: no files at all

SCPU-EMU snapshots BASIC and KERNAL off the running machine over the bus at
start-up. After a KERNAL cold start `$01 = $37` leaves both banked in, so while
holding DMA we can simply read `$A000-$BFFF` and `$E000-$FFFF` into shadow RAM.
Costs 16384 bus cycles, about 16ms.

This is the recommended path: the KERNAL you run is the one actually fitted to
your machine.

## Optional files

Place in `SCPU/` on the SD card:

| File | Size | Notes |
|---|---|---|
| `kernal.rom` | 8192 | overrides the snapshot |
| `basic.rom` | 8192 | overrides the snapshot |
| `chargen.rom` | 4096 | cannot be snapshotted — see below |

Use these to pin a specific KERNAL revision, or to run a real SuperCPU ROM image
(SuperCPU DOS, which brings JiffyDOS with it).

## Why chargen is different

Exposing the character ROM needs CHAREN low, and with the 6510 held off the bus
nothing can rewrite its I/O port. So it cannot be captured the way BASIC and
KERNAL are.

This only affects programs that read the character set *through the CPU*. The
VIC-II fetches it directly on the C64 side and is unaffected, so the display is
correct either way. A future option is injecting a short 6502 stub over Ultimax
to copy it into RAM before takeover — see
[../Docs/research/supercpu-memory-map.md](../Docs/research/supercpu-memory-map.md).

## Legal

C64 ROMs are copyrighted by Commodore's successors; the SuperCPU ROM is
copyrighted by CMD. Dump them from hardware you own, or obtain them from a
source you are entitled to use. Do not commit them here — `.gitignore` is set up
to prevent it.
