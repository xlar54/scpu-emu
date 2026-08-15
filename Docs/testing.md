# Testing

```sh
make tests
```

Around 530,000 checks, all running on the host with no hardware. The bulk of
that is the differential test, which runs both CPU cores over the same programs
and compares everything they do.

| Suite | Covers |
|---|---|
| `Tests/CPU/test_m6502.cpp` | NMOS decimal ADC/SBC, the indirect-JMP page-wrap bug, page-cross cycle penalties, BRK vs IRQ stack layout, edge-triggered NMI, undocumented-opcode lengths |
| `Tests/C64/test_banking.cpp` | the PLA table for every interesting `$01` value, Ultimax, DDR-dependent port readback |
| `Tests/SuperCPU/test_write_buffer.cpp` | coalescing, each optimization mode, sprite-pointer commit ordering, pending-value preservation across policy changes, bounded flushes, reset discard |
| `Tests/Integration/test_kernal_boot.cpp` | the seams: ROM execution costing zero bus cycles, immediate VIC/CIA I/O, raster-scheduled mirroring with repeated guards, IEC activity and pacing, register interception, partial ROM snapshotting |
| `Tests/CPU/test_w65c816.cpp` | everything that makes a 65816 not a 6502: XCE and the mode invariants, each address-wrapping rule, the emulation-mode old/new stack split, `MVN`/`MVP`, 16-bit decimal, the read-modify-write behaviours, `WAI`/`STP` |
| `Tests/CPU/test_w65c816_diff.cpp` | runs `CW65C816` and `CM6502` over the same programs and compares registers, flags, cycle counts and every byte of bus traffic — see below |
| `Tests/SuperCPU/test_memory_map.cpp` | the 24-bit space: bank 0 delegation, private SRAM, SIMM sizing and aliasing, open bus |
| `Tests/Integration/test_real_kernal.cpp` | boots a **genuine Commodore KERNAL** to the BASIC READY prompt on the 6502 core, checks the RAM test finds 38911 bytes free, and asserts the whole cold start stays under 100k bus cycles. Reports as skipped when the ROM images are absent — see [../ROMs/README.md](../ROMs/README.md) |
| `Tests/Integration/test_kernal_65816.cpp` | the same boot with the **65816** driving and the full 24-bit map underneath, then switches the booted machine into native mode and reaches 8MB up in SuperRAM |

## What the tests actually assert

Where it matters they assert on **bus cycle counts**, not just on values. Bus
bandwidth is the scarce resource in this design, so a change that keeps the
right bytes in the right places but doubles the traffic is a regression, and the
tests are written to catch that:

```cpp
for ( int i = 0; i < 1000; i++ ) f.poke( 0x0400, (u8)i );
f.wb.flush();
CHECK_EQ( f.bus.m_Cycles, 1 );   // 1000 writes, one bus cycle
```

`CHostBus` logs every access, so ordering can be asserted directly. The tests pin
down that I/O remains immediate while staged RAM stays queued for a safe raster
opportunity, rather than forcing a burst across the visible display.

## The differential test

In emulation mode a 65816 *is* a 6502, so the two cores must agree instruction
for instruction. `test_w65c816_diff.cpp` generates random programs of documented
opcodes, runs both cores over identical memory, and after every instruction
compares registers, flags, cycle count, the full 64K, and the exact sequence of
writes. That is a far stronger check than any hand-written expectation, because
it exercises paths nobody thought to write a test for.

The agreement is not total, and the differences are real hardware behaviour
rather than tolerance — decimal `ADC` sets N by the 65C02's rule, interrupts
clear the D flag, `abs,X` carries into the next bank, `JMP ($xxFF)` is fixed. Each
is an **explicit exemption** with its own targeted test asserting that it
*does* differ; anything not on that list that differs is a bug. The list, and the
evidence behind each entry, is in
[research/65816-reference.md](research/65816-reference.md) section 10.

## Conformance against outside references

The differential test above has a structural blind spot: it compares our 65816
against our 6502, so it proves the two agree, not that either is right. It is
also emulation-mode only by construction. Four tools exist to check the same
code against authorities that have never seen this source. All are built by
`make viceconf`, from the same objects as the test suite, so nothing can pass
here and behave differently on the card.

| tool | reference | what it establishes |
|---|---|---|
| `render_state` | VICE screenshot | one machine state renders identically — modes, geometry, sprites, collisions |
| `replay_state` | VICE screenshot | a *running* program does: write log → anchors → band planner → renderer, end to end |
| `sid_trace` | VICE state dump + our bus log | every SID write reaches the chip, once, in order, unmerged |
| `ss816` | SingleStepTests vectors | the 65816 in **native** mode, against hardware-derived truth |
| `scpu_trace` | CMD's own SuperCPU ROM | every register access CMD's firmware makes is answered by something we model |

`Tools/viceconf/capture.sh` drives VICE headless and diffs the result;
`REPLAY=1` selects the running-program path, `COLLISIONS=1` the collision check.
`Tools/viceconf/fetch_ss816.sh` gets the CPU vectors, which are not committed.

Two things these are deliberately strict about, both learned the expensive way.
An instrument must refuse to answer questions outside its competence: a
collision check with no stored result reports "no signature" rather than
comparing against uninitialised memory, and a block-move vector captured
mid-instruction is skipped loudly rather than reported as 10,000 CPU failures.
And a clean result is only worth what its coverage is worth — `scpu_trace`
covers the boot path, so it says nothing about routines only particular
software reaches.

## What is not covered

Everything under `Source/Bus/RAD/` and `Source/App/`. It needs a Pi and a C64,
and its correctness is a question of nanosecond timing against a real VIC-II
rather than of logic. `Tools/hardware_tests/` is where that work goes.

Two limits are worth stating explicitly because they are contracts rather than
gaps. Mid-frame rewrites of screen, charset or bitmap memory are outside the
HDMI fidelity contract — source and colour RAM are one coherent snapshot per
frame. Sprite pointers are the exception: they are timestamped, because every
multiplexer rewrites them and the cost is a few ring entries per frame. And the
whole physical-delivery half — bus timing, the read eye, contention with the
HDMI blit — cannot be reached from host tests at all. `28-iecload` exists to
put a number on that half, but it needs hardware to run.

## Adding a test

Tests self-register; there is no list to update.

```cpp
#include "../test_framework.h"

TEST( my_new_test )
{
    CHECK( something );
    CHECK_EQ( value, expected );
}
```

Add the file to `TEST_SRCS` in the top-level `Makefile`.
