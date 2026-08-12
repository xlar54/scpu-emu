# CMD SuperCPU — memory map

## Hardware summary

| | |
|---|---|
| CPU | WDC 65C816S at 20MHz, 8/16-bit |
| Opcodes | documented 6510 opcodes only — undocumented encodings are reused for 65816 instructions |
| On-board RAM | 128KB SRAM |
| ROM | SuperCPU DOS (modified KERNAL/BASIC + JiffyDOS). c64-wiki says 64KB; the circulating dumps (`scpu-dos-1.4.bin`, `scpu-dos-2.04.bin`) are 128KB, which matches the "internal ROM of 128KB" figure given elsewhere. Assume 128KB with the low 64KB the active image. |
| Expansion | SuperRAM card, 1–16MB via a 72-pin fast-page-mode SIMM, 70ns or faster; EDO and SDRAM are not supported |
| Glue | Altera CPLD (larger part on V2, which fixed most of V1's bugs) |

## How it takes over

The SuperCPU asserts `/DMA` on the expansion port. The 6510 tri-states its
address, data and R/W pins and stops driving the bus; the SuperCPU drives it
instead. The VIC-II is untouched and keeps generating the display, refreshing
DRAM and raising raster interrupts.

Three consequences fall out of this, and they shape the whole emulator:

1. **The 6510's on-chip I/O port at `$00`/`$01` is no longer the authority on
   banking.** It is inside a chip that is now inert. The accelerator has to
   emulate the port and resolve the memory map itself — see
   `Source/C64/banking.cpp`.
2. **C64 DRAM goes stale.** The program reads the accelerator's own copy, but
   the VIC-II reads DRAM. Writes the VIC could observe must be mirrored back,
   at one C64 cycle each. This is what the optimization modes exist to reduce.
3. **Ultimax cartridges are incompatible.** A freezer that pulls `/GAME` low with
   `/EXROM` high replaces the KERNAL and leaves the accelerator inoperable. The
   documentation calls out Action Replay and Super Snapshot by name.

## Speed

20MHz in turbo, dropping to 1MHz (2MHz on a C128 in fast mode) when software
asks for it or when the access requires the C64.

Notable behaviours documented for the real hardware:

- Disk access always throttles to 1MHz regardless of the speed setting.
- VIC-II raster interrupt timing is unaffected by acceleration, so games that
  depend on cycle-exact raster code are hit-and-miss.
- Custom fastloaders that bypass the KERNAL usually fail. The documented
  workaround is to switch to Normal for the load and back to Turbo afterwards.

## 24-bit address space

The 65816 addresses 16MB as 256 banks of 64KB.

| Range | Contents | Confidence |
|---|---|---|
| `$000000-$00FFFF` | Bank 0 — active computer RAM image; C64-visible and shadowed | **documented** |
| `$010000-$01FFFF` | Bank 1 — writable PseudoROM/RAM image | **documented** |
| `$020000-$F5FFFF` | User SuperRAM (SIMM) | **documented** |
| `$F60000-$F7FFFF` | System RAM: physical SIMM banks 0 and 1, relocated and reserved | **documented** |
| `$F80000-$FFFFFF` | SuperCPU ROM | **inferred** |

The [CMD SuperRAM manual](../SuperRam.md) documents the expansion layout. The
SIMM starts physically at offset zero, but the SuperCPU's SRAM occupies logical
banks `$00-$01`. The corresponding first 128 KB of the SIMM is therefore
relocated to logical banks `$F6-$F7` and reserved for future system use. Bank
`$F5` is the highest user bank on a 16 MB installation.

| Installed SIMM | User expansion banks |
|---:|---:|
| 1 MB | `$02-$0F` |
| 4 MB | `$02-$3F` |
| 8 MB | `$02-$7F` |
| 16 MB | `$02-$F5` |

### Detecting and using SuperRAM

From the CMD SuperRAM manual:

- Check the ROM version at `$00E487` (64 mode) or `$00F6DD` (128 mode) for
  "1.40" or higher.
- Read expansion pointers at `$00D27C-$00D27F` — note these sit inside the
  `$D200-$D2FF` private RAM window.
- Access via 65816 long addressing (`LDA $01D27C` = `AF 7C D2 01`). Contemporary
  6502 assemblers needed the opcodes entered as raw bytes.
- Sequential access outperforms random access, because of the fast-page-mode
  DRAM controller.

## Bank 0 and the C64 PLA

Within bank 0 the map follows the normal C64 rules, driven by the *emulated*
`$01` plus the real `/GAME` and `/EXROM`. With no cartridge:

| Region | Contents |
|---|---|
| `$A000-$BFFF` | BASIC if LORAM and HIRAM, else RAM |
| `$D000-$DFFF` | RAM if LORAM and HIRAM are both low; else character ROM if CHAREN low; else I/O |
| `$E000-$FFFF` | KERNAL if HIRAM, else RAM |
| everything else | RAM |

Implemented and unit-tested in `Source/C64/banking.cpp` and
`Tests/C64/test_banking.cpp`.

## The character ROM problem

SCPU-EMU can snapshot BASIC and KERNAL straight off the running machine, because
after a KERNAL cold start `$01 = $37` leaves both banked in and we can simply
read them while holding DMA.

The character ROM cannot be captured this way. Exposing it needs CHAREN low, and
with the 6510 held off the bus nothing can rewrite its port. Options:

1. Supply `chargen.rom` on the SD card (what SCPU-EMU does today).
2. Before asserting DMA, inject a short 6502 stub over Ultimax that sets
   `$01 = $33`, copies `$D000-$DFFF` to `$C000-$CFFF`, restores `$01 = $37`, then
   spins — and read it back from `$C000` afterwards. The machinery for injecting
   such a stub already exists in `Source/Bus/C64Side/`.

This only affects programs that read the character set *through the CPU*. The
VIC-II fetches it directly on the C64 side and is unaffected, so the display is
correct either way.

## Known incompatibility classes

From the SuperCPU compatibility notes, these are properties of the real
hardware, not of this emulation — software that fails here largely fails on a
genuine SuperCPU too:

- **Undocumented 6510 opcodes.** The 65816 gives those encodings real, different
  meanings, so software using them crashes. SCPU-EMU's milestone-1 core executes
  them as NOPs; the 65816 core will execute the actual 65816 instructions.
- **`$FFFF` wraparound assumptions.** The 65816 addresses 16MB, so an init loop
  that writes the IRQ vectors at `$FFFx` and then runs on into zero page lands in
  bank 1 at `$010000+` instead of wrapping to `$0000`. A real trap for the
  65816 core to reproduce faithfully.
- **Timing-dependent code** — fastloaders that bypass the KERNAL, FLI, raster
  splits and border effects.
- **Undocumented I/O mirrors** such as `$D220`; the SuperCPU does not pass them
  through.
- **C128 `$D030` 2MHz tricks.**
- **IRQ-heavy code** sees no benefit: the work finishes before the next
  interrupt regardless of speed.

## Sources

- [CMD SuperRAM Installation Guide and User's Reference](../SuperRam.md)
- [SuperCPU register map](supercpu-registers.md)
- [Software compatibility rules and conformance audit](software-compatibility.md)
- [Programming the SuperCPU — 65816 reference and conformance audit](programming-the-scpu.md)
