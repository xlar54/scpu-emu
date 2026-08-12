# Writing SuperCPU-compatible C64 software — and what SCPU-EMU does about it

The rules below are what a C64 program has to respect to keep working when a
SuperCPU is fitted. They matter to this project from the other side: **each rule
describes a behaviour real software depends on, so each one is a conformance
requirement for the emulation.**

Technical content distilled from *"Learn programming in a SuperCPU-compatible
way"* by **Malte Mundt (ThunderBlade / Protovision), 1999** — summarised here in
our own words, not reproduced. Register-level detail lives in
[supercpu-registers.md](supercpu-registers.md); this document is the
software-behaviour view and the audit against it.

## Conformance summary

| # | Rule | SCPU-EMU | Evidence |
|---|---|---|---|
| 1 | Illegal 6510 opcodes do not exist — all 256 are real 65816 instructions | **conforms** | all 256 opcodes decoded, [w65c816.cpp](../../Source/CPU/W65C816/w65c816.cpp) |
| 2 | `STA $D07A` → 1 MHz, `STA $D07B` → turbo, value ignored | **conforms** | [registers.h:57-58](../../Source/SuperCPU/registers.h#L57-L58) |
| 3 | Disk access must run at 1 MHz | **conforms, and exceeds** | [c64_memory.h:675-703](../../Source/C64/c64_memory.h#L675-L703) |
| 4 | Timing-critical raster code must run at 1 MHz | **conforms** (same switch as #2) | [registers.cpp](../../Source/SuperCPU/registers.cpp) |
| 5 | Address space does not wrap at `$FFFF` — it continues into bank 1 | **conforms** | [w65c816_addressing.h:366](../../Source/CPU/W65C816/w65c816_addressing.h#L366) |
| 6 | There is no 2 MHz mode (`$D030`) | **gap — see below** | [registers.cpp:292](../../Source/SuperCPU/registers.cpp#L292) |
| 7 | Undocumented I/O mirrors (`$D220` for `$D020`) do not work | **conforms** | [registers.cpp:274-287](../../Source/SuperCPU/registers.cpp#L274-L287) |
| 8 | `$D200-$D2FF` is system RAM; `$D300-$D3FF` is free for programs | **conforms** | [registers.h:100-103](../../Source/SuperCPU/registers.h#L100-L103) |
| 9 | Detection: bit 7 of `$D0BC` reads 0 on a SuperCPU, 1 on a stock machine | **conforms** | [registers.cpp:224-233](../../Source/SuperCPU/registers.cpp#L224-L233) |

---

## 1. No illegal opcodes

The 6510's undocumented opcodes save a cycle or two by fusing operations. The
65816 defines **every** value from `$00` to `$FF` as a real instruction, so those
byte values now mean something entirely different and a program using them
crashes rather than misbehaves quietly.

**Emulation requirement:** the core must decode all 256 opcodes as 65816
instructions, with no 6502-illegal fallback and no "unknown opcode → NOP" path.

**Status: conforms.** The dispatch in `w65c816.cpp` has a `case` for all 256
values, including the ones a 6510 programmer would recognise as illegal —
`$EB` is `XBA`, `$42` is `WDM`, `$FF` is `SBC long,X`.

## 2. Speed control

```asm
STA $D07A   ; 53370 — drop to 1 MHz
STA $D07B   ; 53371 — return to turbo
```

Both are **write-sensitive switches**: the store triggers the action and the
value is discarded. That is why BASIC uses `POKE 53370,0`.

**Status: conforms.** `SCPU_REG_SOFT_1MHZ_ON` / `SCPU_REG_SOFT_1MHZ_OFF`, with
`$D079` as the documented mirror of `$D07B`.

One subtlety the article does not cover, from the register reference: the
**physical switch is a permission, not a command.** In NORMAL it locks the
machine to 1 MHz and a write to `$D07B` is accepted and discarded; in TURBO it
accelerates nothing by itself until software asks.

## 3. Disk access at 1 MHz

Fast loaders and JiffyDOS encode data in the *timing* between edges, so they
break at 20 MHz. On real hardware it is the **KERNAL**, not the accelerator
hardware, that drops the speed when its own disk routines are used. A program
with its own IRQ loader or fastloader therefore has to do it by hand — `$D07A`
before, `$D07B` after.

**Emulation requirement:** honour the manual switch, and ideally do not depend on
software remembering to use it.

**Status: conforms, and goes further.** SCPU-EMU throttles automatically:
changes to CIA2's IEC outputs, receive-side activity and DDRA transitions arm a
hold-off during which pacing runs at 1 MHz regardless of the selected speed
(`iecThrottleActive()`). CMD document the real hardware as *"always throttles to
1 MHz for disk access regardless of the speed setting"*, so this is the behaviour
being matched rather than an invention. A separate `fineTicksRequired()` also
stops the core batching instructions while a transfer is live, because the slow
protocol assigns meaning to the gaps between individual instructions.

This is the one place where the emulation is *more* forgiving than the article
implies, and deliberately so: a program with a hand-rolled loader that forgets
`$D07A` still works here.

## 4. Timing-critical code at 1 MHz

The same applies to anything built on 1 MHz cycle counting — FLI, side-border
opening, raster bars. Drop to 1 MHz for the effect, return to turbo for the
work. Routines written *for* 20 MHz can of course do far more per frame.

**Status: conforms** — same mechanism as rule 2.

## 5. The address space does not wrap at `$FFFF`

The classic bug. A single loop initialising the NMI/IRQ vectors at `$FFFA-$FFFF`
and then continuing into zero page:

```asm
        LDX #$00
loop    LDA data,X
        STA $FFFA,X
        INX
        CPX #$2A
        BNE loop
```

On a 6510 the store wraps to `$0000` and the zero-page half of the table lands
where the programmer expected. On a 65816 `abs,X` indexing is a **24-bit**
computation, so it carries into bank 1 — `$010000`, which on a SuperCPU is the
SRAM holding the ROM images. Zero page is never written and the program fails
later, somewhere unrelated.

The fix in software is two separate loops. The fix in an emulator is to get the
arithmetic right.

**Status: conforms.** `eaAbsoluteIndexed()` performs a full 24-bit add:

```c
return ( ( ( (u32)m_DBR << 16 ) | base ) + index ) & SCPU_ADDR_MASK;
```

and bank 1 is a real 64 KB array (`m_Bank1`, [memory_map.h:46](../../Source/SuperCPU/memory_map.h#L46)),
so the stray write lands there exactly as it would on hardware.

**Related, and not in the article:** the sibling rule for direct-page indexing
goes the *other* way — `dp,X` wraps at `$FF` in emulation mode with an aligned
direct page, and does not otherwise. `eaDirectIndexed()` implements both cases,
with a comment calling it "the rule most often wrong."

## 6. There is no 2 MHz — `$D030`

Some C64 programs poke `$D030` to get 2 MHz on a C128 in C64 mode with the
screen blanked. That does nothing useful with a SuperCPU fitted, so the advice is
to offer 20 MHz instead of chasing 2 MHz.

**Status: gap.** `$D030` is not claimed by the SuperCPU register interceptor
(only `$D071-$D07F`, `$D0B0-$D0BF` and `$D200-$D3FF` are), so a write falls
through to the machine. On a real C128 that engages 2 MHz mode, which blanks the
VIC-II.

This is *faithful to the machine* but we have not established that it is
**faithful to the accelerator** — what a SuperCPU 128 does with a C64-mode write
to `$D030` is unverified. It is a small, bounded question and the failure mode is
visible (a blanked screen on a C128 running a title that pokes `$D030`), so it is
worth resolving before the C128 work is called done rather than guessing now.

**Evidence since:** the Go64! memory-locations reference describes `$D07A` as
selecting *"Normal (1 MHz or 2 MHz in 128 Fast mode)"* — so on a SuperCPU 128 the
non-turbo speed follows the C128's own 2 MHz state rather than being forced to
1 MHz. That points towards letting `$D030` reach the machine being correct, and
towards "normal" speed needing to track it.

**Stronger evidence, and it points the other way.** *C=Hacking* 17's software-repair
notes record that on a 128D a SuperCPU meeting a write to `$D030` can **lock the
machine outright**, and the recommended repair is to patch the instruction out —
`INC $D030` (`EE 30 D0`) becomes `BIT $D030` (`2C 30 D0`), same length, harmless.

So on real hardware `$D030` is not merely useless with an accelerator fitted, it is
actively dangerous. SCPU-EMU currently passes the write through, which on a C128
engages 2 MHz and blanks the VIC-II rather than locking up — arguably a kinder
outcome than the real thing, but a *different* one, and the difference is now
documented rather than assumed. Deciding whether to intercept it is still open;
what has changed is that "pass it through and match the hardware" is no longer the
obvious answer.

## 7. Only documented I/O locations

The C64 decodes the VIC incompletely, so `$D220` behaves like `$D020` and
`$D212` like `$D012`. Programs that rely on this break on a SuperCPU, which does
not decode the mirrors — and worse, uses that space itself.

**Status: conforms, without a special case.** `$D220` falls inside the
`$D200-$D2FF` private RAM window, so SCPU-EMU already swallows it rather than
passing it to the VIC. The same structural reason the real hardware breaks these
programs is the reason ours does.

## 8. The private RAM windows

| Range | Size | Purpose |
|---|---|---|
| `$D200-$D2FF` | 256 B | SuperCPU DOS / system scratch |
| `$D300-$D3FF` | 256 B | **free for SuperCPU-aware programs** |

Both always read back; writes normally take effect only while the hardware
register bank is open (`$D07E`). `$D27E` is the single exception, writable with
the bank closed.

**Status: conforms**, including the write gate. That gate is not cosmetic — CMD's
DOS 2.04 startup writes `$FF` to `$D20C` with the bank closed, *expects the write
to be discarded*, and branches on reading it back. Honouring the write sends the
boot down the wrong path.

## 9. Detecting a SuperCPU

**Read `$D0BC` (53436) and test bit 7.** A stock machine returns 1 because
nothing decodes the address and an unused I/O read floats high. A SuperCPU
returns 0. Then read `$D0B0` for the hardware version.

**Status: conforms**, and the read path documents the reasoning inline. Note that
detection is independent of speed — the accelerator is detectable whether or not
turbo is engaged.

---

## Not a conformance rule, but worth recording

**Do not calculate inside the IRQ.** Frame-driven code gains nothing from an
accelerator: a heavier calculation simply finishes earlier and the CPU idles
until the next raster interrupt. Work belongs in the main loop, running as far
as it can get, rather than being sliced across frames.

There is nothing for the emulation to *implement* here, but there is something to
**measure**: if SCPU-EMU is working, main-loop-bound code should scale with the
selected speed and IRQ-bound code should not. We have an unanchored BASIC figure
(`TI-T ≈ 60` for 10,000 `FOR/NEXT`) with no unaccelerated reference captured
alongside it — that pairing is still outstanding and would make the claim
checkable.

## Open items from this audit

1. **`$D030` on a C128 in C64 mode** (rule 6) — decide whether to intercept or
   keep passing it through, and record why.
2. ~~Hardware-register enable remaps the KERNAL ROM and we do not model it.~~
   **Resolved.** The Go64! memory-mirroring reference locates the alternative
   window at `$6000-$7FFF`, not `$E000-$FFFF`, and SCPU-EMU does model it —
   `applyKernalShadow()` switches `m_KernalShadowBase` between the two. See
   [programming-the-scpu.md](programming-the-scpu.md#9-two-different-things-both-called-mirroring).
3. **Bootmap** (`$D0B6`/`$D0B7`) is decoded but the SuperCPU ROM is not executed
   from reset — deliberate, see [../roadmap.md](../roadmap.md).

## Sources

- Malte Mundt (ThunderBlade / Protovision), *"Learn programming in a
  SuperCPU-compatible way"*, 1999 — the rules in sections 1-9.
- [supercpu-registers.md](supercpu-registers.md) — register-level reference,
  sourced from VICE's SCPU64 implementation.
- [65816-reference.md](65816-reference.md), [supercpu-memory-map.md](supercpu-memory-map.md).
