# CMD SuperCPU — register reference

What SCPU-EMU implements, and how confident we are in each entry. Anything
marked **inferred** should be confirmed against real hardware or VICE before it
is relied on.

Implemented in `Source/SuperCPU/registers.cpp`.

## Write-sensitive switches

Almost every SuperCPU register is a *write-sensitive switch*: storing any value
triggers the action and the value is discarded. They cannot be read back. This
is why BASIC toggles speed with `POKE 53370,0` — the `0` carries no meaning.

### Optimization mode select

Selects which writes get mirrored into C64 DRAM for the VIC-II. Requires the
hardware register bank to be open (`$D07E`).

| Address | Decimal | Function | Confidence |
|---|---|---|---|
| `$D074` | 53364 | VIC bank 2 (`$8000-$BFFF`) — documented as "GEOS optimization" | **confirmed** |
| `$D075` | 53365 | VIC bank 1 (`$4000-$7FFF`) | **confirmed** |
| `$D076` | 53366 | BASIC — mirror only `$0400-$07FF` | **confirmed** |
| `$D077` | 53367 | No optimization — mirror everything | **confirmed** |
| `$D078` | 53368 | VIC bank 0 (`$0000-$3FFF`), V2 only | **inferred** |
| `$D079` | 53369 | VIC bank 3 (`$C000-$FFFF`), V2 only | **inferred** |

V2 hardware is documented as adding VIC banks 0 and 3 plus per-bank and full
optimization with granular zero-page/stack control via a "Z flag". That the two
new banks landed at `$D078`/`$D079` is the obvious reading but is not stated
outright in the sources consulted.

The documented mode semantics, which drive `CWriteBuffer`:

| Mode | Mirrors |
|---|---|
| DEFAULT | everything except zero page and stack |
| NONE | everything, including zero page and stack |
| BASIC | `$0400-$07FF` only |
| FULL | nothing (C128 80-column: the VDC has its own RAM) |

### Speed select

Always available, whether or not the register bank is open.

| Address | Decimal | Function | Confidence |
|---|---|---|---|
| `$D07A` | 53370 | Normal — 1MHz (2MHz in C128 fast mode) | **confirmed** |
| `$D07B` | 53371 | Turbo — 20MHz | **confirmed** |

### Register bank enable

| Address | Decimal | Function | Confidence |
|---|---|---|---|
| `$D07E` | 53374 | Enable hardware registers | **confirmed** |
| `$D07F` | 53375 | Disable hardware registers (restore stock map) | **confirmed** |

Documentation warns that software should toggle these promptly to avoid
conflicts — while open, `$D074-$D079` are stolen from whatever else might decode
there.

## Status block, `$D0B0-$D0BF` (read)

**This is not a mirrored block** — each address carries distinct flags. Earlier
notes here treated it as a single status byte; the SuperCPU 128 register list
corrected that. Implemented in full.

| Address | Dec | Contents | Confidence |
|---|---|---|---|
| `$D0B0` | 53424 | version / mode, bits 7-6 | **confirmed** |
| `$D0B2` | 53426 | bit7 hardware registers enabled, bit6 system at 1MHz | **confirmed** |
| `$D0B3` | 53427 | v2 enhanced optimization; readable always, writable while the bank is open | **partial** |
| `$D0B4` | 53428 | bits 7-6 current optimization mode | **confirmed** |
| `$D0B5` | 53429 | bit7 JiffyDOS switch, bit6 speed switch (1 = Normal) | **confirmed** |
| `$D0B6` | 53430 | bit7 processor emulation mode, bit6 reset switch (v1) | **confirmed** |
| `$D0B8` | 53432 | bit7 software speed flag, bit6 master speed flag (1 = Normal) | **confirmed** |
| `$D0BC` | 53436 | bit7 DOS extension mode, bit6 RAMLink registers | **confirmed** |

`$D0B0` bits 7-6:

| Value | Meaning |
|---|---|
| `00xxxxxx` | V2 in C128 mode |
| `01xxxxxx` | V2 in C64 mode |
| `11xxxxxx` | V1, no SuperCPU, or disabled |

`$D0B4` bits 7-6:

| Value | Optimization |
|---|---|
| `00xxxxxx` | VIC bank 2 / GEOS |
| `01xxxxxx` | VIC bank 1 |
| `10xxxxxx` | BASIC |
| `11xxxxxx` | none |

### Detection

**Check bit 7 of `$D0BC` (53436).** It reads 1 on a stock C64/C128 because
nothing decodes the address, and 0 on a SuperCPU. Then read `$D0B0` for the
version.

Resolved discrepancy: an earlier note here quoted the idiom as `PEEK(53433)`,
which is `$D0B9`. Both the register list and the compatibility notes give
`$D0BC`/53436, so 53433 appears to be a transposition in the scanned manual.
SCPU-EMU answers on the whole block regardless, so either reading works.

### The speed switch is a permission, not a command

From the 1541 Ultimate discussion, and confirmed by the register list: the
physical switch is *force 1MHz, or merely allow 20MHz*. In the NORMAL position
the machine is locked to 1MHz and a write to `$D07B` is accepted and discarded.
In TURBO it does not accelerate anything by itself — software still has to ask.

Detection is independent of speed: "simply having the thing turned on is all
that's required".

### Not yet modelled

Enabling the hardware registers is documented to also change the KERNAL ROM
memory map at `$E000-$FFFF`, which is why software is told not to leave them
enabled longer than necessary. SCPU-EMU does not do this.

## Private RAM windows

Not registers — genuine RAM inside the cartridge, which must never reach the C64.

| Range | Size | Purpose | Confidence |
|---|---|---|---|
| `$D200-$D2FF` | 256 B | SuperCPU DOS / kernel scratch | **confirmed** |
| `$D300-$D3FF` | 256 B | free for user programs | **confirmed** |

The three ranges the SuperCPU steals inside I/O space are documented as
`$D070-$D07F`, `$D0B0-$D0BF` and `$D200-$D3FF`. SCPU-EMU claims all of
`$D0B0-$D0BF` and all of `$D200-$D3FF`; in `$D070-$D07F` it decodes the
documented addresses and lets the rest fall through to the machine.

A side effect worth noting: the compatibility notes record that programs writing
to undocumented I/O *mirrors* fail on a SuperCPU, giving `$D220` (a VIC mirror)
as the example. `$D220` falls inside the `$D200-$D2FF` private RAM window, so
SCPU-EMU already swallows it rather than passing it to the VIC — matching the
real hardware without any special case.

## Sources

- CMD SuperCPU 128 V2 User's Guide —
  <https://archive.org/stream/CMD_SuperCPU_128_V2_Users_Guide/CMD_SuperCPU_128_V2_Users_Guide_djvu.txt>
- c64-wiki, *SuperCPU* — <https://www.c64-wiki.com/wiki/SuperCPU>
- SuperCPU programming info — <http://www.elysium.filety.pl/tools/supercpu/superprog.html>
- Commodore Hacking #13, *Exploiting the 65C816S CPU* —
  <http://mclauchlan.site.net.au/scott/C=Hacking/C-Hacking13/cpu.html>
- SuperCPU 128 register list, c-128.freeforums.net —
  <https://c-128.freeforums.net/thread/559/c128-super-cpu-registers>
- SuperCPU compatibility notes — <https://supercpu.cbm8bit.com/comp.htm>
- 1541 Ultimate issue #654, on detection and speed-switch semantics —
  <https://github.com/GideonZ/1541ultimate/issues/654>
- Scanned manuals — <https://www.zimmers.net/anonftp/pub/cbm/manuals/cmd/>
