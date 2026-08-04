# Testing

```sh
make tests
```

1203 checks across five areas, all running on the host with no hardware.

| Suite | Covers |
|---|---|
| `Tests/CPU/test_m6502.cpp` | NMOS decimal ADC/SBC, the indirect-JMP page-wrap bug, page-cross cycle penalties, BRK vs IRQ stack layout, edge-triggered NMI, undocumented-opcode lengths |
| `Tests/C64/test_banking.cpp` | the PLA table for every interesting `$01` value, Ultimax, DDR-dependent port readback |
| `Tests/SuperCPU/test_write_buffer.cpp` | coalescing, each optimization mode, flush-on-mode-change, auto-flush |
| `Tests/Integration/test_kernal_boot.cpp` | the seams: ROM execution costing zero bus cycles, I/O reaching the machine, flush-before-I/O ordering, register interception, partial ROM snapshotting |
| `Tests/Integration/test_real_kernal.cpp` | boots a **genuine Commodore KERNAL** to the BASIC READY prompt, checks the RAM test finds 38911 bytes free, and asserts the whole cold start stays under 100k bus cycles. Reports as skipped when the ROM images are absent — see [../ROMs/README.md](../ROMs/README.md) |

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

`CHostBus` logs every access, so ordering can be asserted directly — which is how
the flush-before-I/O rule is pinned down.

## What is not covered

Everything under `Source/Bus/RAD/` and `Source/App/`. It needs a Pi and a C64,
and its correctness is a question of nanosecond timing against a real VIC-II
rather than of logic. `Tools/hardware_tests/` is where that work goes.

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
