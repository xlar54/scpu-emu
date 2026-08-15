# Programming the SuperCPU — 65816 reference and conformance audit

What a programmer needs to know to write code that uses the SuperCPU, and what
SCPU-EMU does about each item.

Distilled from the **Go64! SuperCPU tutorial series** (10 parts, originally
published in Go64! magazine, translated from German), together with the
memory-locations and memory-mirroring reference pages, hosted at
<https://supercpu.cbm8bit.com/scpu/>. Summarised in our own words. Companion to
[software-compatibility.md](software-compatibility.md), which covers what *not*
to do; this document covers what you *can* do.

## Conformance summary

| Area | SCPU-EMU | Evidence |
|---|---|---|
| All 256 opcodes as 65816 instructions | **conforms** | [w65c816.cpp](../../Source/CPU/W65C816/w65c816.cpp) |
| Native-mode vectors `$FFE4/E6/E8/EA/EE` | **conforms** | [w65c816.h:68-72](../../Source/CPU/W65C816/w65c816.h#L68-L72) |
| Decimal mode computes N and Z correctly | **conforms** | [w65c816.cpp:96](../../Source/CPU/W65C816/w65c816.cpp#L96) |
| RMW acknowledges `$D019` in emulation mode, not in native | **conforms** | [w65c816_addressing.h:233-262](../../Source/CPU/W65C816/w65c816_addressing.h#L233-L262) |
| Optimization mirror ranges | **conforms exactly** | [write_buffer.cpp:100-116](../../Source/SuperCPU/write_buffer.cpp#L100-L116) |
| `MVN`/`MVP` interruptible, one byte per execution | **conforms** | [w65c816.cpp:1031-1076](../../Source/CPU/W65C816/w65c816.cpp#L1031-L1076) |
| Bank-1 ROM shadow, KERNAL and BASIC | **conforms** | [c64_memory.h:221](../../Source/C64/c64_memory.h#L221) |
| DOS-extension mapping `$1000-$5FFF`, `$8000-$9FFF` | **conforms** | [c64_memory.h:348-357](../../Source/C64/c64_memory.h#L348-L357) |
| KERNAL window moves to `$6000` when h/w registers open | **conforms** | [registers.h:267](../../Source/SuperCPU/registers.h#L267) |
| `$D200-$D3FF` read always, write only when registers open | **conforms** | [registers.cpp:274-287](../../Source/SuperCPU/registers.cpp#L274-L287) |
| Interrupt vector reroute to bank 1 | **conforms** | [c64_memory.h:361-372](../../Source/C64/c64_memory.h#L361-L372) |
| Emulation-mode `dp,X` wrap at `$FF` | **conforms** | [w65c816_addressing.h:236](../../Source/CPU/W65C816/w65c816_addressing.h#L236) |
| 24-bit `abs,X` carries into the next bank | **conforms** | [w65c816_addressing.h:366](../../Source/CPU/W65C816/w65c816_addressing.h#L366) |

Nothing in this source turned up a behaviour we do not model. Three
documentation corrections fell out of the audit — see the last section.

---

## 1. Detecting the accelerator

Two independent methods.

**Register test** — read `$D0BC` (53436) and test bit 7. Zero means SuperCPU;
a stock machine floats the unused I/O read high and returns one.

**Processor test** — the NMOS 6502/6510 computes the N flag incorrectly after a
decimal-mode add. The 65816 does not:

```asm
        SED
        LDA #$99
        CLC
        ADC #$01        ; N set  -> 6510 ; N clear -> 65816
        CLD
```

**SCPU-EMU:** both work. The core computes N and Z the 65C02 way in decimal
mode, and the comment records a further refinement the tutorial does not
mention — unlike the 65C02 there is **no extra cycle** in decimal mode.

**Speed test** — bit 6 of `$D0B8` reads 1 in 1 MHz mode and 0 in turbo.

## 2. Processor modes

The E flag selects emulation (6502-compatible, 8-bit, 64K) or native. It is not
directly accessible; it swaps with the carry:

```asm
        CLC
        XCE             ; -> native mode
        SEC
        XCE             ; -> emulation mode
```

In native mode two new status bits set register width:

| Bit | Flag | Meaning |
|---|---|---|
| 5 | **M** | 0 = 16-bit accumulator/memory, 1 = 8-bit |
| 4 | **X** | 0 = 16-bit index registers, 1 = 8-bit |

```asm
        REP #$20        ; 16-bit accumulator
        SEP #$20        ; 8-bit accumulator
        REP #$10        ; 16-bit X and Y
        SEP #$10        ; 8-bit X and Y
        REP #$30        ; both 16-bit
```

Width is a per-region decision, not a permanent one — the tutorial is explicit
that best performance comes from switching where it pays rather than staying
16-bit throughout.

## 3. Registers the 6502 did not have

| Register | Purpose | Access |
|---|---|---|
| **B** | hidden high half of the accumulator | `XBA` swaps A and B |
| **D** | Direct Page register — relocates "zero page" anywhere in bank 0 | `TCD`, `TDC`, `PHD`, `PLD` |
| **DBR** | Data Bank — supplies the bank byte for 16-bit addresses | `PHB`, `PLB` |
| **PBR** | Program Bank — where code is executing | set by long jumps |
| **S** | 16-bit stack pointer; the stack can live anywhere in bank 0 | `TCS`, `TSC`, `TXS`, `TSX` |

Setting the data bank is done through the stack, since there is no direct load:

```asm
        LDA #$03
        PHA
        PLB             ; DBR = 3, so INC $2000 now touches $032000
```

## 4. New addressing modes

- **Direct page indirect** — `LDA ($FD)`, no Y needed. Removes the standard
  `LDY #$00 : LDA ($FD),Y` idiom and the need to preserve Y.
- **Long** — `LDA $058100`, `STA $058100`. Full 24-bit, no mode switch required.
- **Long indirect** — `LDA [$FC],Y` with a **three-byte** pointer: low, high,
  bank. Reaches all 16 MB through a zero-page pointer.
- **Absolute indexed indirect jump** — `JMP ($7000,X)` and `JSR ($7000,X)`,
  which makes a jump table a two-instruction affair (entries are two bytes, so
  the index advances by two).
- **Stack relative** — `LDA 3,S`. The stack pointer is the base of an array;
  indices start at 1 because S points at the next free byte.
- **Stack relative indirect indexed** — `LDA (3,S),Y`, for dereferencing a
  pointer that was passed on the stack.

Not every instruction takes the long forms: `LDA` and `STA` do, `INC` and `ROL`
do not.

## 5. New instructions worth knowing

| Instruction | Effect |
|---|---|
| `BRA` | branch always, ±127 bytes, no flag needed |
| `BRL` | branch long, 3 bytes, full 64K range, still position-independent |
| `STZ` | store zero without touching the accumulator (abs, dp, and X-indexed forms — no Y-indexed form) |
| `TSB` / `TRB` | test-and-set / test-and-reset bits in memory; only Z is affected |
| `PEA` | push a 16-bit constant, always 16-bit regardless of M |
| `PEI` | push a 16-bit value read from a direct-page location |
| `PER` | push a PC-relative address — the basis of position-independent code |
| `PHX`/`PLX`, `PHY`/`PLY` | index registers straight to and from the stack |
| `MVN` / `MVP` | block move, see §7 |
| `JSL` / `RTL` | long call and return across banks |
| `XBA` | swap A and B |

`PHP` is always 8 bits. 16-bit pushes go **high byte first**.

## 6. Interrupts

Vector addresses differ by mode:

| Vector | Emulation | Native |
|---|---|---|
| IRQ | `$FFFE/FF` | `$FFEE/EF` |
| NMI | `$FFFA/FB` | `$FFEA/EB` |
| BRK | (via IRQ) | `$FFE6/E7` |
| COP | `$FFF4/F5` | `$FFE4/E5` |
| ABORT | `$FFF8/F9` | `$FFE8/E9` |
| RESET | `$FFFC/FD` | — |

**The KERNAL's `$0314/$0315` indirection is bypassed in native mode**, because
the hardware vector no longer points at the KERNAL dispatcher.

**Save the status register.** An interrupt can arrive with M and X in any state,
so a native-mode handler must `PHP` on entry and `PLP` before its pulls, or the
`PLA`/`PLX`/`PLY` will restore the wrong number of bytes.

### The `$D019` rule

Acknowledge a VIC-II raster interrupt with:

```asm
        LDA $D019
        STA $D019
```

`DEC $D019`, `ASL $D019` and friends **work in emulation mode but fail in
native mode**. The reason is the read-modify-write cycle pattern: the NMOS part
writes the original value back before the modified one, and that write-back is
what clears the latch. The CMOS core drops it in native mode.

**SCPU-EMU models all three cases**, and the code comment says why:

```
E=1        read, write, write -- the original byte goes back first.
E=0, m=1   read, read, write  -- an internal cycle instead.
E=0, m=0   the two writes go HIGH BYTE FIRST, the reverse of the reads.
```

So `INC $D019` acknowledges here in emulation mode and does not in native mode,
matching the documented hardware exactly. This is the same fault class as the
`$D011` read-modify-write bug found in K218 — worth remembering that it is a
*class*, not a one-off.

### What the speed is actually for

At 20 MHz a raster handler no longer has to busy-wait for the beam. Update the
VIC registers and return; the tutorial puts the recovered time at roughly 80% of
what a wait-loop handler burns.

## 7. Block moves

`MVN` and `MVP` move up to 64 KB between any two banks at about 7 cycles per
byte. Set the accumulator to **count minus one**, X to the source address and Y
to the destination.

```asm
        MVN $target_bank, $source_bank
```

Four traps, all of which SCPU-EMU models:

1. **The operand order in the object code is destination bank, then source
   bank** — the reverse of how the mnemonic reads.
2. **The direction is the opposite of WDC's names.** `$54` "Move Negative"
   increments. Go by increment/decrement and ignore the names.
3. **DBR is destroyed** — it is set to the destination bank. `PHB`/`PLB` around
   the move.
4. **It is interruptible.** One byte executes per pass with the PC parked on the
   opcode, so an interrupt taken mid-move pushes the address of the `MVN` itself
   and `RTI` resumes from X, Y and C. **A handler must therefore preserve X, Y
   and the accumulator**, or the move resumes corrupted.

With 8-bit index registers the move is confined to page 0 of each bank, because
XH and YH are architecturally zero. `C = 0` moves one byte; `C = $FFFF` moves
65,536.

## 8. Running code outside bank 0

`JSL`/`RTL` and long `JMP` update the Program Bank register. The stack, zero
page and direct page stay in bank 0 regardless.

**KERNAL routines only work when called from bank 0**, because they return with
a two-byte `RTS`. And hardware stays where it is: the SID is always at
`$00D400`, so a player running from bank 2 still has to write its registers in
bank 0.

## 9. Two different things both called "mirroring"

This tripped us up before, so it is worth stating plainly.

**(a) The SuperCPU's bank-1 ROM shadow.** The accelerator keeps ROM images in
its own fast SRAM at `$010000+` and serves bank-0 reads from there, so the CPU
never waits on the C64's slow ROM.

**Read the table bank 1 → bank 0.** The first column is where the data physically
lives in the accelerator's SRAM; "mirrored at" is the bank-0 address at which it
appears. Getting this backwards is easy and I did it in the first draft.

| Bank 1 address | Reference | Mirrored at | State | Mirrored if |
|---|---|---|---|---|
| `$E000-$FFFF` | KERNAL | `$E000-$FFFF` | read only | h/w registers **disabled** and KERNAL ROM enabled |
| `$D400-$DFFF` | — | *no mirror* | | |
| `$D200-$D3FF` | extra RAM for I/O area | `$D200-$D3FF` | read always, write if h/w registers enabled | I/O enabled |
| `$C000-$D1FF` | — | *no mirror* | | |
| `$A000-$BFFF` | BASIC | `$A000-$BFFF` | read only | BASIC ROM enabled |
| `$8000-$9FFF` | RLDOS | `$8000-$9FFF` | read only | CPU DOS extensions enabled |
| `$6000-$7FFF` | **ALT. KERNAL** | `$E000-$FFFF` | read only | h/w registers **enabled** and KERNAL ROM enabled |
| `$1000-$5FFF` | RLDOS | `$1000-$5FFF` | read only | CPU DOS extensions enabled |
| `$0000-$0FFF` | — | *no mirror* | | |

The `$6000` row is the one to read carefully. Bank-0 `$E000-$FFFF` **always**
serves the KERNAL; what the register bank changes is *which image*. Closed, it
comes from bank 1 `$E000-$FFFF`; open, from a different image at bank 1
`$6000-$7FFF`. So `$D07E` changes the code running at the KERNAL entry points.

`CC64Memory::read8()` implements exactly this — `REG_KERNAL` returns
`m_ROMShadow[ m_KernalShadowBase + ( a - 0xE000 ) ]`, with the base switched
between `$6000` and `$E000` by `applyKernalShadow()`.

Interrupt vector fetch is a **three-way** decision, not two-way:

| Vector address | Condition |
|---|---|
| `$00FFE4-$00FFFF` | KERNAL ROM switched out |
| **CPU ROM** | any one of: 65816 in native mode · CPU h/w registers enabled · RAMLink h/w enabled · CPU DOS extensions enabled · system 1 MHz mode enabled |
| `$017FE4-$017FFF` | all other conditions |

SCPU-EMU matches this exactly: `interruptRerouteActive()` requires the KERNAL to
be mapped and then tests `!emulationMode || interruptRerouteRequested()`, where
the latter is `m_HWRegsEnabled || m_Sys1MHz || m_DOSExt || m_RAMLink` — the same
five conditions.

**SCPU-EMU models all of this**: `m_ROMShadow` into bank 1,
`dosExtensionMapsBank1()` for the two RLDOS ranges, `m_KernalShadowBase`
switching between `$E000` and `$6000` as the register bank opens and closes, and
`interruptRerouteActive()` for the vector split.

**(b) SCPU-EMU's write-through to real C64 DRAM.** Entirely separate concept,
and not something a real SuperCPU does — it exists because our CPU lives on a
Pi and the real VIC-II still has to fetch from real DRAM. When our source or CHT
says "mirror," it usually means this one. See
[architecture.md](architecture.md).

## 10. Speed and optimization

```asm
        STA $D07E       ; enable the hardware registers
        STA $D07A       ; 1 MHz
        STA $D07B       ; turbo
        STA $D07F       ; disable the hardware registers again
```

The optimization registers declare which region the VIC actually reads, so the
accelerator can skip keeping the rest coherent:

| Register | Declares | Region |
|---|---|---|
| `$D074` | VIC bank 2 / GEOS | `$8000-$BFFF` |
| `$D075` | VIC bank 1 | `$4000-$7FFF` |
| `$D076` | BASIC | `$0400-$07FF` |
| `$D077` | none — keep everything coherent | all (V1 default) |

**SCPU-EMU implements these ranges exactly**, plus VIC bank 0 and 3 and a
`DEFAULT` mode that mirrors everything except zero page and stack.

On a verified C64-class host, SCPU-EMU now reproduces the real card's
RAM-under-I/O mechanism. After ROM capture and the bus self-test, a bounded
takeover stub leaves the halted 6510 at `$01=$34`. `/GAME` remains released for
ordinary RAM and mirror traffic, making physical DRAM at `$D000-$DFFF`
reachable, and is asserted only for a genuine VIC/SID/CIA/colour-RAM access.
This is required for a bank-3 bitmap at `$C000`, whose lower 3904 bytes occupy
`$D000-$DF3F`. The old blanket suppression remains the fail-safe until that
physical map verifies, and on host types where it is not yet enabled.

**Note for the C128:** the memory-locations reference describes `$D07A` as
selecting *"Normal (1 MHz or 2 MHz in 128 Fast mode)"*. So on a SuperCPU 128 the
non-turbo speed follows the C128's own 2 MHz state rather than being forced to
1 MHz. This bears directly on the open `$D030` question in
[software-compatibility.md](software-compatibility.md#6-there-is-no-2-mhz--d030):
it suggests letting `$D030` reach the machine is correct, and that "normal"
speed should track it — but it does not settle what the accelerator does with
the VIC-II blanking that C128 2 MHz mode causes.

---

## Corrections this audit produced

1. **[supercpu-registers.md](supercpu-registers.md) "Not yet modelled" is
   stale.** It says enabling the hardware registers "is documented to also
   change the KERNAL ROM memory map at `$E000-$FFFF`… SCPU-EMU does not do
   this." We do: `applyKernalShadow()` moves the window between `$E000` and
   `$6000`. The source also locates the change at `$6000-$7FFF`, not
   `$E000-$FFFF`.
2. **[software-compatibility.md](software-compatibility.md) open item 1** gains
   evidence — see the C128 note in §10 above.
3. The `$D019` read-modify-write behaviour is **mode-dependent**, not simply
   "CMOS differs from NMOS". The standalone *"A solved secret"* article states
   the general CMOS difference; tutorial part 7 is the one that pins it to
   native mode, and that is what our core implements.

## Sources

- Go64! SuperCPU tutorial, parts 1-10, and the memory-locations and
  memory-mirroring reference pages — <https://supercpu.cbm8bit.com/scpu/>
- *"A solved secret"* (Go64! 03/98) — the `$D019` read-modify-write cycle counts.
- [supercpu-registers.md](supercpu-registers.md) — register reference, from VICE.
- [software-compatibility.md](software-compatibility.md) — the "what not to do" side.
- [65816-reference.md](65816-reference.md), [supercpu-memory-map.md](supercpu-memory-map.md).
