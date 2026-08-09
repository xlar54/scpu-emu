# CMD SuperRAM Installation Guide and User's Reference

Cleaned Markdown edition of the 1997 Creative Micro Designs, Inc. manual.
This edition removes OCR page furniture, repairs obvious recognition errors,
and reformats the material for project reference. Technical statements remain
those of the original manual unless placed under **SCPU-EMU implementation
notes**.

## Copyright notice

Copyright © 1997 Creative Micro Designs, Inc.

The original manual and accompanying programs were protected by international
copyright law and were not to be copied or distributed without CMD's express
written permission. This repository copy is retained as local technical
reference material.

Original publisher contact information:

- Creative Micro Designs, Inc.
- P.O. Box 646, East Longmeadow, MA 01028-0646
- Support: 413-525-0023
- Sales: 800-638-3263
- Historical website: <http://www.cmdweb.com/>

## Contents

1. [General information](#general-information)
2. [Installation](#installation)
3. [Troubleshooting](#troubleshooting)
4. [SIMM information](#simm-information)
5. [Programming information](#superram-programming-information)
6. [Utilities disk](#the-utilities-disk)
7. [SCPU-EMU implementation notes](#scpu-emu-implementation-notes)

## General information

The CMD SuperRAM card is an add-in card for the CMD SuperCPU 64 and SuperCPU
128 accelerator cartridges. It accepts a single 72-pin fast-page-mode SIMM and
provides 1, 4, 8, or 16 MB of memory that the SuperCPU's 65816 can address
directly.

### Uses

Using SuperRAM requires software written or modified for the 65816. It does not
automatically make existing BASIC or machine-language programs use the extra
memory. CMD supplied GEOS patches that could use approximately 2 MB as RAM
disks, and expected other software to be adapted over time.

### Power requirements

CMD recommended a heavy-duty power supply for a C64 or C64C fitted with a
SuperCPU. The SuperCPU already exceeds Commodore's nominal cartridge-port
current specification, and SuperRAM increases the load. Inadequate voltage can
cause incorrect operation or hardware damage.

With the SuperCPU and SuperRAM installed, the +5 V supply should remain near
5 V. Some variation is normal, but CMD considered less than 4.8 V evidence of
an inadequate or faulty supply.

### Utilities disk

The replacement SuperCPU Utilities disk included programs for testing,
detecting, configuring, and developing for SuperRAM. Those programs were also
covered by CMD's copyright and were not redistributable without permission.

## Installation

Take normal antistatic precautions. If you are not comfortable handling socketed
chips and circuit boards, have the installation performed by a qualified person.

1. Turn off the computer and all peripherals, then remove the SuperCPU from the
   cartridge port.
2. Remove the four case screws and separate the two halves of the SuperCPU
   enclosure. Angle the upper half slightly to clear the switches.
3. If the SuperRAM kit includes an updated SuperCPU ROM, note the old ROM's
   orientation, remove it, and install the replacement with every pin aligned.
4. Install the 72-pin SIMM in the SuperRAM socket. The module is keyed and fits
   in only one orientation.
5. Align the two connectors on the rear of the SuperRAM card with the matching
   SuperCPU headers and press the boards together firmly.
6. Reassemble the SuperCPU.
7. Power up and confirm that the startup display reports the expected expansion
   memory. Run `SUPERRAMTEST` before relying on the installation.

## Troubleshooting

### Basic isolation procedure

1. Remove other cartridge-port devices and retest.
2. Remove the SuperRAM card and test the SuperCPU alone.
3. Reinstall and reseat the SuperRAM card and SIMM.
4. Clean and align cartridge-port and SuperCPU edge contacts.
5. If possible, test the SuperCPU/SuperRAM on another computer.
6. If possible, test the SuperRAM card in another SuperCPU.

### Dirty contacts or bad connections

Verify that:

- the SIMM is fully seated;
- the SuperRAM card is fully seated on both headers;
- cartridge-port contacts align with the connector opening;
- all contacts are clean and free of residue.

### Low-voltage conditions

An inadequate supply can cause unusual and intermittent failures. Measure the
computer's +5 V rail with the SuperCPU and SuperRAM attached. A reading below
4.8 V should be investigated before further testing.

### Bus loading and noise

Some Commodore computers cannot reliably drive multiple cartridge-port loads.
Symptoms may include random screen characters, freezes, or total failure to
start. Clean and tighten connections and test on a second computer to separate
a host-specific signal problem from a SuperCPU or SuperRAM fault.

### Early SuperCPU compatibility

Some SuperCPU units earlier than revision 1E may be incompatible with
SuperRAM because of component loading characteristics. Reported symptoms
include:

- freezes;
- unexpected returns to BASIC;
- SuperCPU initialization failures;
- SRAM or SuperRAM test failures;
- unexplained resets.

CMD produced a SuperCPU 64 CPLD upgrade to address this condition.

## SIMM information

The SuperRAM card contains a clock oscillator, bus driver, GAL, digital delay,
CPLD, supporting passive components, two SuperCPU connectors, and one 72-pin
SIMM socket. The CPLD implements most memory mapping, control, and refresh
logic.

Only standard **fast-page-mode** 72-pin SIMMs rated at **70 ns or faster** are
supported. EDO and SDRAM modules are not compatible. Faster speed ratings do
not improve performance because the controller uses fixed access timing.

### Supported organizations

| Capacity | Organization | Row size | Row/column address bits |
|---:|---:|---:|---:|
| 1 MB | 256K × 32/36 | 2 KB | 9/9 |
| 4 MB | 1M × 32/36 | 4 KB | 10/10 |
| 8 MB | 2M × 32/36 | 4 KB | 10/10 |
| 8 MB | 2M × 32/36 | 4 KB | 11/10 |
| 16 MB | 4M × 32/36 | 4 KB | 11/10 |
| 16 MB | 4M × 32/36 | 4 KB | 12/10 |
| 16 MB | 4M × 32/36 | 8 KB | 11/11 |

## SuperRAM programming information

The 65816 can address up to 16 MB. Unlike an REU or RAMLink, SuperRAM is part
of the processor's address space: programs can read, write, and execute code
directly in it. The processor does not have to be in native mode to use long
addressing, although native mode is required for other 65816 features.

### Common SuperCPU 64/128 memory map

| Banks | Function |
|---|---|
| `$00` | Active computer RAM image; always present |
| `$01` | Writable PseudoROM/RAM image; always present |
| `$02-$F5` | User expansion RAM when fitted |
| `$F6-$F7` | System RAM: physical SIMM banks `$00-$01`, relocated below ROM and reserved for system use |
| `$F8-$FF` | SuperCPU system ROM or ROM-reserved space |

The SIMM's physical addressing begins at offset `$000000`. Because SuperCPU
SRAM already occupies logical banks `$00` and `$01`, the corresponding first
two SIMM banks are relocated to logical banks `$F6` and `$F7`. On a 16 MB
system, bank `$F5` is therefore the highest bank available to user programs.

The installed SIMM capacity determines the last user expansion bank:

| Installed SIMM | User expansion banks |
|---:|---:|
| 1 MB | `$02-$0F` |
| 4 MB | `$02-$3F` |
| 8 MB | `$02-$7F` |
| 16 MB | `$02-$F5` |

The capacity names describe the fitted SIMM. Banks `$F6-$F7` hold the
relocated system portion and are not part of the user ranges shown above.

The SuperCPU 128 has two additional hidden SRAM banks that are swapped into
logical banks `$00` and `$01` as needed; this does not change the common
SuperRAM bank layout.

### Detecting expansion RAM

First verify a SuperCPU ROM with expansion support. In C64 mode, read four
PETSCII bytes beginning at `$00E487`. Version `1.40` was the first release with
initial SuperRAM support.

If the ROM supports SuperRAM, read the allocation pointers at
`$00D27C-$00D27F`:

| Address | Meaning |
|---|---|
| `$00D27C` | First available page |
| `$00D27D` | Bank containing the first available page |
| `$00D27E` | Last available page + 1 |
| `$00D27F` | Bank containing the last available page + 1 |

All four bytes are zero when no expansion RAM is available.

These variables are valid in bank `$00` while I/O is mapped in. With I/O
mapped out, use the corresponding locations in bank `$01`.

Applications that reserve SuperRAM must update the allocation pointers rather
than assuming that all fitted memory is free. To change them through bank `$00`:

1. Open the SuperCPU hardware-register bank by storing any value to `$00D07E`
   (decimal 53374).
2. Update the allocation pointers.
3. Close the hardware-register bank by storing any value to `$00D07F`
   (decimal 53375).

Future system extensions or previously loaded programs may have reserved part
of SuperRAM. Relocatable code and data are therefore strongly recommended.

### Speed considerations

SIMM DRAM is slower than the SuperCPU's on-board SRAM. Place latency-critical
routines in SRAM when possible. The SuperRAM controller is optimized for
sequential access; random reads and writes incur the largest slowdown.

### Programming examples

65816 assemblers can express 24-bit absolute-long accesses directly:

```asm
lda $01d27c        ; absolute long read
sta $01d27c        ; absolute long write
```

In a 6502 assembler without 65816 syntax, encode the instructions manually:

```asm
; LDA $01D27C
.byte $af
.byte $7c,$d2,$01

; STA $01D27C
.byte $8f
.byte $7c,$d2,$01
```

`LDA` and `STA` also have absolute-long,X forms. Other 65816 instructions have
additional long addressing modes.

## The Utilities Disk

The updated utilities disk contained:

- `RUNME.BAS`
- `Superinstall`
- `calculator`
- `64CONFIG 2.1s`
- `SRBOOT`
- `SUPERRAMTEST`
- `SUPERRAMDETECT`
- `SUPERRAMFAKE`

### SUPERRAMTEST

Load and run with:

```text
LOAD"SUPERRAMTEST",8
RUN
```

After confirmation, the utility tests host verification memory and then the
installed SIMM with several bit patterns. A complete test can take several
minutes. On failure, record the failing test and pattern, then work through the
connection, power, and cross-system checks in [Troubleshooting](#troubleshooting).

### SUPERRAMDETECT

Example software showing how to detect SuperRAM and calculate the available
capacity from the allocation pointers.

The original OCR called this program `SUPERRAMPETECT` in one heading; the disk
listing and surrounding text establish that `SUPERRAMDETECT` is correct.

### SUPERRAMFAKE

Modifies the SuperRAM allocation registers to test software against simulated
memory configurations. It can also place a chosen version string in the ROM
version workspace. Pressing the SuperCPU reset button restores the real values.

### 64CONFIG 2.1s

A replacement GEOS 64 v2.0 `CONFIGURE` supporting SuperRAM-backed RAM disks
and CMD HD/FD drives. It is not compatible with GEORAM versions of GEOS.

Operational limits documented by CMD:

- one physical drive permits up to two RAM disks;
- two physical drives permit one RAM disk;
- three physical drives permit no RAM disk;
- at least one of GEOS drive A or B must be physical;
- two RAM disks of the same type should be renamed immediately;
- the Shadow 1581 Directory option is unavailable with the HD/FD 1581 driver;
- there is no “DMA for MoveData” option because SuperRAM has no DMA controller.

GEOS retains its declared SuperRAM allocation after exiting to BASIC so it can
be re-entered. If GEOS will not be resumed, press the SuperCPU reset button to
release that allocation.

### SRBOOT

Works with GEOS 64 v2.0 and `64CONFIG 2.1s` to reboot GEOS from SuperRAM after
exiting to BASIC. It requires uninterrupted power, unchanged device numbers,
and an intact GEOS kernel image in SuperRAM.

Typical use from BASIC:

```text
LOAD"SRBOOT",8
RUN
```

### calculator

Updated GEOS calculator desk accessory that fixes a crash in the standard
version when math functions run on accelerated systems.

## SCPU-EMU implementation notes

These conclusions are project notes derived from the manual; they are not text
from CMD.

1. **Bank `$05` is valid SuperRAM.** `$05:0000` lies squarely in the documented
   user range `$02:0000-$F5:FFFF` on a 16 MB configuration.
2. **Banks `$F6-$F7` are not ordinary top-of-user-memory banks.** They expose
   the SIMM storage displaced by the SuperCPU's logical banks `$00-$01` and are
   reserved for system use.
3. **Instruction-width state matters when executing code from SuperRAM.** If
   the M flag is clear, `LDA #imm` consumes a 16-bit operand. Code assembled as
   `A9 02` must execute after `SEP #$20`; otherwise the following opcode becomes
   the high immediate byte.
4. **A near `RTS` cannot return across program banks.** A monitor transfer from
   bank `$00` into bank `$05` must return through a long-call convention or a
   monitor trap. With SuperMON, use `G` and terminate test code with `BRK`.
5. **Do not use the routine itself as the write target when testing RAM.** A
   store to `$05:0000` overwrites the first opcode. Use a separate cell such as
   `$05:0100`.
6. **Allocation pointers are ownership boundaries, not merely size fields.** A
   general-purpose program should reserve memory by updating them rather than
   assuming that every detected SIMM byte is unclaimed.
7. **Sequential-access timing is architecturally visible.** The emulator's
   SuperRAM timing model should retain a page/row locality benefit rather than
   applying one constant penalty to every access.

### Minimal SuperMON execution test

Assemble this at `$05:0000`:

```asm
sep #$20           ; 8-bit accumulator
lda #$02
sta $050100        ; data cell, separate from code
brk                 ; return to SuperMON through its BRK handler
```

Then run and inspect it with:

```text
G 050000
M 050100 050100
```

The final dump should contain `$02`.
