# RAD Expansion Unit — findings

Notes taken while reading [frntc/RAD](https://github.com/frntc/RAD) and
[frntc/RAD-Doom](https://github.com/frntc/RAD-Doom), which SCPU-EMU is built on.
This is the "why it works" background for the code under `Source/Bus/RAD/`.

## What the hardware is

A Raspberry Pi (3A+/3B+ or Zero 2) on a small carrier that plugs into the
C64/C128 expansion port. Deliberately dumb: level shifters (CBTD3861), a pair of
address latches, a bus transceiver, and a 74LVC257 multiplexer. No CPLD, no
microcontroller. **All behaviour is software on the Pi**, which is exactly what
makes a SuperCPU port possible at all.

GPIO assignment (`Source/Bus/RAD/gpio_defs.h`):

| Signal | GPIO | Notes |
|---|---|---|
| PHI2 | 9 | the clock everything is timed against |
| R/W | 18 | driven when we are bus master |
| /DMA | 19 | the takeover line |
| /RESET | 8 | |
| /IRQ | 4 | input when we listen, output when we inject |
| /GAME | 5 | |
| MPLEX_SEL | 7 | selects what the 74LVC257 presents |
| BA | 10 | VIC-II badline indicator |
| D0–D7 | 20–27 | all in GPIO bank 2, so direction flips in one register write |

D0–D7 are also used to *emit* the address through the two latches, which is why
`gpio_defs.h` aliases `A0..A15` onto the same pins.

## The key discovery

RAD-Doom does not run Doom on the C64. It **replaces the C64's CPU**: it asserts
/DMA, the 6510 tri-states, and the Pi drives the bus. From there it POKEs a
converted framebuffer into screen memory 50 times a second.

That is architecturally the same manoeuvre a CMD SuperCPU performs. The
SuperCPU also holds /DMA so the 6510 stops driving the bus, and then executes
from its own fast SRAM, only touching the C64 for I/O and for keeping VIC-visible
memory coherent. **RAD-Doom is therefore not a rough starting point for a
SuperCPU — it is the same machine doing a different job.**

## Where /DMA can safely be asserted

From `waitAndHijack()` in `rad_doom_hijack.cpp`: spin until BA goes low, i.e.
until the VIC-II announces a badline and takes the bus from the 6510 anyway,
then assert /DMA 80ns after the falling edge of PHI2 (`TIMING_TRIGGER_DMA`).
Asserting it at an arbitrary moment is not clean.

Reproduced in `Source/Bus/RAD/cpu_hijack.cpp` with the badline wait intact.

## Access cost

One access per C64 cycle, ~1µs on PAL. The primitives are split into phases so
the caller can interleave other work:

- `busReadByte_p1/p2/p3` — latch address, wait for the CPU half-cycle, sample data
- `busWriteByte_p1/p2` — latch address and data, hold through the VIC half-cycle
- `busWriteByteBurst_p1/p2` between `busBeginBurstWrites`/`busEndBurstWrites` —
  keeps the latch and transceiver configured across a whole run of writes

The burst path is what makes mirroring affordable: roughly one C64 cycle per
byte instead of the two or three a sequence of standalone writes costs. RAD-Doom
uses it to blit ~10KB per frame. `CWriteBuffer` uses it for the same reason.

Renamed from RAD's `emu*REU*` names in `Source/Bus/RAD/lowlevel_dma.h`; nothing
about them is REU-specific.

## Things that bit RAD and will bite us

- **BA must be re-checked mid-write.** `busWriteByteBurst_p1` contains a resync
  loop for when a badline starts partway through a burst. Left intact.
- **R/W must be driven slightly before the address latch OE and DIR.** There is a
  comment in the original to the effect that this took a long time to work out,
  and that old VIC-II revisions need it. Left intact, including the magic `-40`.
- **Timing constants are per-Pi-model and per-machine** and live in a config
  file. They were tuned at `-Ofast`; changing optimisation flags moves them.
- **Cache preloading matters.** RAD's REU loop spends real effort on `prfm`
  instructions because an L2 miss inside a bus window is fatal. `busTiming` is a
  packed, 128-byte-aligned struct for exactly this reason.

## What could not be carried over

`ultimax_init.h` / `ultimax_memcfg.h` are small 6502 stubs RAD injects over
Ultimax mode to bring a C128 into C64 mode and set `$01`. They are kept in
`Source/Bus/C64Side/` and are currently unused: SCPU-EMU resets the machine and
lets its own KERNAL set `$01 = $37` instead. They become relevant again for
C128 support and for capturing the character ROM (see
[supercpu-memory-map.md](supercpu-memory-map.md)).

## Sources

- <https://github.com/frntc/RAD>
- <https://github.com/frntc/RAD-Doom>
