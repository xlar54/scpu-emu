# SuperCPU 128 hardware findings

This is the working record for observations made on a real CMD SuperCPU 128.
Keep raw measurements separate from interpretation: the purpose of this file is
to guide C128 support without silently importing SCPU64 assumptions.

## Test provenance

- Date received: 2026-08-06
- Accelerator: real SuperCPU 128 running SuperCPU DOS 2.04
- Probe: `C64Tests/ml/15-scpu128probe.asm`, version 1
- Source disk: `C64Tests/SCPU-TESTS-log.d64`
- Report length: 690 bytes each; all reports end normally and are not splat files
- The probe captures state before opening its output file
- The probe only writes the documented `$D07E/$D07F` register-gate controls;
  all mapped-memory tests are reads and CRC calculations

The disk contains four complete sequential reports:

| File | User-reported configuration |
|---|---|
| `log128-40` | C128 mode, 40-column display, Turbo |
| `log128-80` | C128 mode, 80-column display, Turbo |
| `log128n-80` | C128 mode, 80-column display, physical speed switch in Normal |
| `log64` | C64 mode; user reports that this mode did not run normally |

The user also reports that physical-Normal 40-column mode did not run. The
complete `log64` proves that the ML body executed at least once in C64 mode, but
the exact invocation and visible behavior of that run remain to be established.

## Published SCPU128 v2 architecture

Doug Cotton's contemporary article, *The SuperCPU Steps Up to Version 2*, adds
important details that are not present in VICE's SCPU64 implementation:

- SCPU128 v2 has **256K of onboard SRAM**, enough to shadow the RAM and ROM
  images of both a C64 and C128. SCPU64 has only the original 128K pair.
- Only two 64K images are presented to the 65816 at once. **65816 bank `$00`
  always contains the active computer RAM image**; in C128 mode this is C128
  physical RAM bank 0 or bank 1 according to the current C128 selection.
- **65816 bank `$01` contains the writable ROM image.** It normally holds the
  C64 ROM image, with C128 ROM segments switched into the composite when those
  segments are mapped in C128 mode.
- The extra 128K unique to SCPU128 is switched into banks `$00/$01` as needed;
  it is not exposed as additional 65816 banks. SuperRAM consequently still
  starts at bank `$02`, preserving the SCPU64/SCPU128 application memory map.

This published description matches the broad pattern in the captured CRCs and
replaces the earlier tentative description of bank `$01` as generic private
SRAM. It is specifically writable ROM-shadow SRAM.

Source: [Commodore World issue 22, *The SuperCPU Steps Up to Version
2*](https://electronicsandbooks.com/edt/manual/Magazine/C/Commodore%20World%201994%201999/Commodore_World_Issue_22.pdf).

### Enhanced optimization register

The same article supplies the complete `$D0B3` v2 encoding. The important new
low bits are conventionally named `B` and `Z`:

- `B` assigns a per-bank optimization to C128 RAM bank 0 or bank 1.
- `Z=0` mirrors zero page and stack (`$0000-$01FF`) to the host; `Z=1` stops
  that mirroring for faster direct-page and stack access.
- `B` is meaningful only in C128 mode on SCPU128.
- Relocating zero page or stack with the C128 MMU does not relocate the range
  controlled by `Z`; the optimization applies to the true physical
  `$0000-$01FF` addresses.
- Writes to the enhanced and legacy optimization controls are reflected into
  each other for backward compatibility.
- I/O is always mirrored whenever I/O is mapped in, independently of the RAM
  optimization mode.

The documented base patterns are:

| `$D0B3` pattern | Meaning |
|---|---|
| `00xxx1BZ` | VIC bank 0, `$0000-$3FFF` |
| `01xxx0B0` | VIC bank 1, `$4000-$7FFF` |
| `00xxx0B0` | VIC bank 2/GEOS, `$8000-$BFFF` |
| `01xxx1B0` | VIC bank 3, `$C000-$FFFF` |
| `10xxx0B0` | BASIC, `$0400-$07FF` |
| `11xxx00Z` | no optimization, both banks `$00:0000-$01:FFFF` |
| `11xxx1BZ` | no optimization, selected C128 bank `$0000-$FFFF` |
| `10xxx100` | full optimization, no RAM mirroring |

The observed `$D0B3=$C1` is therefore the documented v2 default: no
optimization across both banks, with `Z=1` disabling zero-page/stack
mirroring. The low bit repeated in every captured `$D0Bx` value is specifically
this `Z` state, not merely an unidentified optimization flag.

### C128-specific timing rules

The contemporary programming notes identify bus synchronizations that should
be modeled explicitly:

- In C128 mode, reads **and writes** to `$0001` and `$FF00` wait for the next
  1 MHz bus cycle.
- Reads from VDC ports `$D600/$D601` and MMU load registers `$FF01-$FF04` also
  wait for the next 1 MHz cycle; writes to those locations use the cache.
- After a real VDC access, another VDC access is blocked during the following
  1 MHz cycle so the VDC can complete the operation.
- On v2, color RAM is treated as ordinary mirrored memory rather than slow
  I/O. Reads are full speed and writes are full speed when the cache is ready.
- `$D200-$D3FF` SRAM and the `$D07x/$D0Bx` registers operate at full processor
  speed on v2. In C64 mode `$0001` is also full speed.

These are access-class rules, not a blanket throttle for `$D000-$DFFF`.

The v2 documentation also says `$D0B6` bit 6 no longer reports the reset switch;
that bit is a v1-only feature.

### C128 startup and MMU constraints

The C128 does not begin a cold boot on the 8502. Its Z80 executes first, checks
the cartridge signals, selects C128 or C64 operation through the 8722, and then
hands control to the 8502. SCPU128 cold-start emulation therefore cannot assume
the C64 sequence of immediately fetching a 65xx reset vector. The practical
implementation choices are either to let the physical C128 complete the Z80
handoff before asserting DMA, or to reproduce the observable handoff state.

The 8722 also imposes rules that the SCPU memory layer must retain:

- `$FF00` is the always-visible configuration register; `$FF01-$FF04` load the
  four preset configurations represented by `$D501-$D504`.
- The captured `$D506=$04` means 1K of common RAM at the bottom of memory
  (`$0000-$03FF`). That is why the next bank-switch probe can safely execute
  from `$0300` while changing the selected C128 RAM bank.
- Page-zero and stack relocation are distinct from common RAM and from the
  SCPU `$D0B3` `Z` optimization. They must be applied in the correct order.
- The processor's built-in port at `$0000/$0001` is special and is not ordinary
  relocated RAM.

Sources: the [Commodore 128 service
manual](https://retro-bobbel.de/zimmers/cbm/schematics/computers/c128/servicemanuals/Commodore_128_Service_Manual_Preliminary_314001-07_%281985_Aug%29_%28missing_pages_26-45%29.pdf)
and the [8722 MMU programming
notes](https://www.zimmers.net/anonftp/pub/cbm/documents/projects/memory/c128/1028/1028.html).

## Raw register snapshots

### C128 40-column Turbo

```text
CPU PORT 00/01: 2F 43
VIC D02F/D030: FF FC

SCPU D0B0-D0BF CLOSED
01 01 01 C1 C1 81 81 01 01 01 01 01 01 01 01 01

SCPU D0B0-D0BF OPEN
01 01 81 C1 C1 81 81 01 01 01 01 01 01 01 01 01

MMU D500-D50B
00 3F 7F 01 41 B7 04 00 F0 01 F0 20

MMU FF00-FF04
00 3F 7F 01 41
```

### C128 80-column Turbo

```text
CPU PORT 00/01: 2F 43
VIC D02F/D030: FF FC

SCPU D0B0-D0BF CLOSED
01 01 01 C1 C1 81 81 01 01 01 01 01 01 01 01 01

SCPU D0B0-D0BF OPEN
01 01 81 C1 C1 81 81 01 01 01 01 01 01 01 01 01

MMU D500-D50B
00 3F 7F 01 41 37 04 00 F0 01 F0 20

MMU FF00-FF04
00 3F 7F 01 41
```

### C128 80-column, physical switch in Normal

This register snapshot is identical to the 80-column Turbo snapshot above.

### C64 mode

```text
CPU PORT 00/01: 2F E7
VIC D02F/D030: FF FC

SCPU D0B0-D0BF CLOSED
41 01 01 C1 C1 81 81 01 01 01 01 01 01 01 01 01

SCPU D0B0-D0BF OPEN
41 01 81 C1 C1 81 81 01 01 01 01 01 01 01 01 01

MMU D500-D50B
00 00 00 00 00 00 00 00 00 00 00 00

MMU FF00-FF04
DD 8D 07 DD 4C
```

## Confirmed findings

### `$D0B0` distinguishes C128 and C64 modes

After removing the optimization low bits repeated across the `$D0Bx` block:

| Mode | `$D0B0` | Bits 7-6 |
|---|---:|---:|
| C128 | `$01` | `00` |
| C64 | `$41` | `01` |

This confirms the documented SuperCPU v2 mode encoding on real SCPU128
hardware. The emulator must not report the SCPU64 value `$40` while operating
as a SuperCPU128 in native C128 mode.

### `$D0B2` reports the register gate

Every run returned `$01` with the hardware registers closed and `$81` after a
write to `$D07E`. Bit 7 therefore reports the gate state on the real SCPU128,
matching the SCPU64 behavior already modeled.

### `$D505` exposes the physical 40/80-column selection

The only stable MMU difference between C128 40- and 80-column runs was:

| Display selection | `$D505` |
|---|---:|
| 40-column | `$B7` |
| 80-column | `$37` |

Bit 7 is the 8722 MMU's 40/80-key sense. Bits 6 and 0 report C128 mode and the
8502 selected respectively. The MMU therefore continues to describe the C128
host-side processor selection even while the SuperCPU is executing in its
place.

`$D50B=$20` reports two 64K RAM banks and MMU revision zero.

### C64 mode removes the MMU view

In C64 mode `$D500-$D50B` returned zero and `$FF00-$FF04` returned
`DD 8D 07 DD 4C`, bytes from the C64 memory map rather than MMU registers.
C128 MMU decoding must therefore be disabled when the SuperCPU reports C64
mode.

### Opening the SCPU register bank does not change the other sampled flags

Across all four logs, only `$D0B2` bit 7 changed between the closed and open
snapshots. The optimization, switch, processor-mode, speed and DOS-extension
values remained stable.

## Physical Normal-switch contradiction

The `log128n-80` run was made with the physical SuperCPU speed switch in
Normal. Nevertheless it reported the same bytes as the Turbo run:

- `$D0B5=$81`: JiffyDOS bit set, documented physical-Normal bit 6 clear
- `$D0B8=$01`: documented software-Normal and master-Normal bits clear
- `$D0B2=$01`: documented system-1MHz bit 6 clear

The common low bit is an optimization flag and does not affect this decoding.
The result therefore says Turbo according to every documented status flag.

Published v2 material clarifies that software Normal (`$D07A`) means 1 MHz, or
2 MHz when the C128 itself is in Fast mode. That changes the expected measured
frequency but not the flag contradiction: `$D0B5/$D0B8` should still report a
Normal constraint. The unit's Turbo LED is documented to light whenever the
accelerator is actually operating at 20 MHz, so its state should be recorded in
the follow-up test alongside register values and elapsed timing.

Do not change the emulator's polarity from this one observation. A follow-up
probe must measure elapsed speed while sampling these registers live in both
physical switch positions. Possible explanations still include:

1. SCPU128 native mode does not report the physical switch through these bits
   in the same way as SCPU64.
2. The switch limits actual execution speed but the status flags fail to show
   it in this C128 configuration.
3. The particular hardware's switch or switch-sense path behaves differently.

The software Turbo request cannot explain the result under the published
model, because the physical Normal switch is supposed to override it.

## Memory fingerprints

### Visible-window CRC16-CCITT

| Mode | `$A000-$BFFF` | `$C000-$CFFF` | `$E000-$FFFF` |
|---|---:|---:|---:|
| C128 40 Turbo | `$8D5F` | `$07F1` | `$4474` |
| C128 80 Turbo | `$8D5F` | `$07F1` | `$4474` |
| C128 80 physical Normal | `$8D5F` | `$07F1` | `$4474` |
| C64 | `$A512` | `$A014` | `$AE28` |

Changing 40/80 selection did not change the CPU-visible C128 ROM windows.
C64 mode produced a completely different, internally consistent map.

### 256-byte long-address CRC16-CCITT

| Address | C128 40 T | C128 80 T | C128 80 N | C64 |
|---|---:|---:|---:|---:|
| `$00:0000` | `$D2F8` | `$B1E6` | `$F958` | `$80AB` |
| `$00:0400` | `$B0C1` | `$F413` | `$F413` | `$B0C1` |
| `$00:1C00` | `$C2DB` | `$C2DB` | `$C2DB` | `$C2DB` |
| `$00:A000` | `$D784` | `$D784` | `$D784` | `$6409` |
| `$00:C000` | `$3986` | `$3986` | `$3986` | `$5292` |
| `$00:E000` | `$29ED` | `$29ED` | `$29ED` | `$D9A1` |
| `$01:0000` | `$9EA4` | `$9EA4` | `$9EA4` | `$4625` |
| `$01:0400` | `$021C` | `$021C` | `$021C` | `$F10F` |
| `$01:1C00` | `$41E8` | `$41E8` | `$41E8` | `$9505` |
| `$01:A000` | `$D784` | `$D784` | `$D784` | `$6409` |
| `$01:C000` | `$3986` | `$3986` | `$3986` | `$32AE` |
| `$01:E000` | `$29ED` | `$29ED` | `$29ED` | `$D9A1` |

In C128 mode, banks `$00` and `$01` differ in RAM regions but receive identical
overlays at `$A000`, `$C000`, and `$E000`. Bank `$01` fingerprints are also
stable across 40/80 and reported speed configurations. This agrees with the
published architecture: bank `$00` is the selected C128 RAM shadow, while bank
`$01` is the writable composite ROM shadow. ROM segments are also copied into
bank `$00` as necessary to form the CPU-visible machine map.

The reports do not yet verify the swap to the C128's second physical 64K bank.
All three native-mode reports used MMU configuration `$00`, which selects C128
RAM bank 0. Published material says an MMU bank change must switch the matching
RAM shadow into 65816 bank `$00`; hardware still needs to establish the exact
switch and write-mirroring sequence.

In C64 mode, banks `$00` and `$01` again differ in RAM. `$A000` and `$E000`
match across the two banks because ROM is visible there; `$C000` differs because
that window is RAM in the C64 map. This supports applying the active machine's
ROM/I/O overlay after selecting accelerator bank-zero or bank-one SRAM.

## Probe limitations and failed modes

The version-1 probe is not a general 8502 program:

- Its BASIC loader is at the C128 address `$1C01`, so C64 BASIC `RUN` cannot
  find it at the C64 BASIC start `$0801`. A direct `SYS7181` can enter it.
- It uses 65816-only instructions (`BRA`, `PHX`, `PLX`, and indirect-long
  addressing). It cannot safely execute if a switch position genuinely returns
  control to the C128's 8502.

These limitations can explain a failure to launch; they do not by themselves
explain why physical-Normal 80-column mode completed while physical-Normal
40-column mode reportedly did not.

## Implementation consequences

1. Add an SCPU128 identity/machine mode distinct from SCPU64. `$D0B0` must
   report bits 7-6 as `00` in C128 mode and `01` in C64 mode.
2. Add the 8722 MMU as a layer in the bank-zero host memory map. Disable it in
   C64 mode.
3. Model all 256K of SCPU128 SRAM. Swap the selected C128 physical RAM shadow
   into 65816 bank `$00`; keep the writable composite ROM shadow in bank `$01`.
4. Map C128 ROM segments into bank `$01` and mirror them into bank `$00` as
   needed to reproduce the active C128 memory configuration.
5. Treat `$D505` bit 7 as host 40/80-key state, not as an SCPU optimization
   control.
6. Do not alter physical-speed-switch polarity until timing confirms what the
   real hardware actually did.
7. Expand `$D0B3` handling to include its C128 bank-select (`B`) and physical
   zero-page/stack (`Z`) semantics.
8. Add the individual C128 synchronization rules for `$0001`, `$FF00`,
   `$FF01-$FF04`, and `$D600/$D601`; do not approximate them as one general I/O
   delay.
9. Define the takeover point relative to the C128's Z80-to-8502 cold-start
   handoff. A C64-style reset-vector start is not sufficient for native C128
   mode.

## Next hardware experiments

### 1. 8502-compatible switch and timing probe

Use only documented 6502/8502 opcodes. Measure a fixed loop against a CIA timer
and record `$D0B2`, `$D0B5`, `$D0B8`, and `$D030` before and after the physical
switch changes. Also record the physical Turbo LED. Supply separate C128
`$1C01` and C64 `$0801` loaders, and distinguish C128 1 MHz from C128 Fast
2 MHz when the switch is Normal.

### 2. Controlled C128 MMU bank-1 probe

The current `$D506=$04` configures 1K of common RAM at the bottom of memory. A
small routine can run from that common area, temporarily select the bank-1
preconfiguration (`$FF02`, currently `$7F`), fingerprint safe RAM pages, and
restore configuration `$00`. Preserve every modified byte.

This will verify the published claim that an MMU bank change switches the
corresponding C128 RAM shadow into 65816 bank `$00`. It must also check which
C128 ROM segments in bank `$01` change with each MMU configuration and how the
`B` optimization flag affects write mirroring.

### 3. Interrupt/register-window test

Open the SCPU hardware registers, trigger VIC and CIA interrupts separately,
and record the gate state and MMU configuration inside and after each handler.

### 4. VDC access timing

Measure `$D600/$D601` access delays and optimization status in both display
modes. Determine whether 80-column selection changes mirroring policy or merely
changes the host display path.

### 5. Reset-state capture

Use a minimal resident logger to capture the MMU, SCPU flags, vectors and ROM
mapping earlier than a disk-loaded program can. Repeat cold reset, warm reset,
`GO64`, and SuperCPU reset-button cases.

## Reference material

- `Docs/research/supercpu-registers.md` documents the existing SCPU64-derived
  register model.
- `C64Tests/README.md` documents the probe and the rest of the hardware disk.
- CMD SuperCPU programming information gives the published register meanings:
  <https://www.elysium.filety.pl/tools/supercpu/superprog.html>
- Doug Cotton, *The SuperCPU Steps Up to Version 2*, gives the SCPU128 SRAM
  swapping, enhanced optimization and C128 timing details:
  <https://electronicsandbooks.com/edt/manual/Magazine/C/Commodore%20World%201994%201999/Commodore_World_Issue_22.pdf>
- Scans of the original CMD SuperCPU128 v1 and v2 user guides are indexed at:
  <https://www.zimmers.net/anonftp/pub/cbm/manuals/cmd/index.html>
- CMD's general specifications identify the three physical switches and the
  Turbo LED behavior:
  <https://elysium.filety.pl/tools/supercpu/superspec.html>
- The Commodore 8722 register definitions are summarized from the C128
  programming documentation:
  <https://www.commodore.ca/manuals/funet/cbm/documents/projects/memory/c128/1028/1028.html>
