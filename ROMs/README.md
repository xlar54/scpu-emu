# ROMs

**No ROM images are committed to this repository. The four runtime images are
required on the SD card.**

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
cp scpu-dos-2.04.bin scpu.rom
```

`Tests/Integration/test_real_kernal.cpp` uses `kernal.rom` and `basic.rom` to
boot a genuine KERNAL through the CPU core. Those host-side tests report as
*skipped* rather than failing when the local files are absent, so a fresh clone
still goes green. That development convenience does not change the firmware's
runtime requirement.

The SuperCPU DOS images are 128KB. `make sdcard` stages **2.04** as
`SCPU/scpu.rom`, and the firmware maps it at `$F80000`. The runtime preflight
requires this exact 128KB image along with the three C64 ROMs. An incomplete set
is reported on HDMI and leaves the physical machine to boot normally.

**The two images are for different machines.** This is the thing to get right:

| | machine | contents |
|---|---|---|
| `scpu-dos-1.4.bin` | **SuperCPU 64** | first 64KB populated, rest `$FF`. No C128 content. |
| `scpu-dos-2.04.bin` | **SuperCPU 128** | all 128KB, and carries a C128 KERNAL and BASIC — the strings `(C)1986 COMMODORE ELECTRONICS, LTD.` and `(C)1977 MICROSOFT CORP.` are in there. |

**Correction — an earlier version of this file was wrong here.** It said that
staging 2.04 on a C64 gets you to the SuperCPU boot screen and then
`SUPERCPU INITIALIZATION ERROR: 06` "because its boot code is checking for a
machine that is not there." Both halves are wrong:

- **2.04 works.** That attribution was made during an early debugging round and
  was retracted on 2026-08-04; the real causes were emulation bugs since fixed
  (KERNAL shadow, `$D0B2` window moves). `make sdcard` stages 2.04, and that is
  deliberate — do not "helpfully" revert it to 1.4.
- **Error 06 is not a machine-detection failure.** On real hardware it is the
  documented symptom of an **inadequate power supply** (CMD SuperCPU FAQ). The
  accelerator already exceeds the nominal cartridge-port current specification
  on its own. It has nothing to say about which DOS image is installed.

Both corrections point the same way: if error 06 appears, suspect power on
hardware and the emulation elsewhere — not the ROM version.

1.4's boot chain, for reference: reset reads `$FFFC` under bootmap and gets
`$FC90`, which is `JML $F800FC`, which is `JML $F80100`, which is the real
start — `SEI`, set up the stack, then enable the hardware registers at `$D07E`.
So both the bootmap window and the `$F80000` mapping have to be right for it to
get anywhere.

`BOOTMAP 1` maps this ROM over bank 0 at reset so its code runs before the C64
KERNAL, matching the real accelerator. `BOOTMAP 0` remains the SD-card recovery
setting if that boot path is being diagnosed.

## Required runtime set

Place all four files in `SCPU/` on the SD card:

| File | Size | Notes |
|---|---|---|
| `kernal.rom` | 8192 | C64 KERNAL 901227-03 |
| `basic.rom` | 8192 | C64 BASIC 901226-01 |
| `chargen.rom` | 4096 | C64 character ROM 901225-01 |
| `scpu.rom` | 131072 | SuperCPU DOS 2.04, mapped at `$F80000` |

If any file is absent or has an unusable size, startup prints the complete
required-file checklist, does not assert `/DMA`, and reports `BOOTING NORMALLY`.
`scpu.rom` is the real SuperCPU ROM image — SuperCPU DOS, which brings JiffyDOS
with it.

## Why chargen is different

Exposing the character ROM needs CHAREN low, and with the 6510 held off the bus
nothing can rewrite its I/O port. So it cannot be captured the way BASIC and
KERNAL are.

This only affects programs that read the character set *through the CPU*. The
VIC-II fetches it directly on the C64 side and is unaffected, so the display is
correct either way. A future option is injecting a short 6502 stub over Ultimax
to copy it into RAM before takeover — see
[../Docs/SuperCPU64/supercpu-memory-map.md](../Docs/SuperCPU64/supercpu-memory-map.md).

## Legal

Commodore ROMs are copyright Commodore International Corporation. The SuperCPU
ROM is **copyright CMD**. CMD is a trademark of Creative Micro Designs.

Dump them from hardware you own, or obtain them from a source you are entitled
to use. Do not commit them here — `.gitignore` is set up to prevent it.

The SuperCPU ROM is available from **CoreI64 — Thomas Christoph** at
<https://www.corei64.com/shop/>.
