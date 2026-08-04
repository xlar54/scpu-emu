# SCPU-EMU

CMD SuperCPU emulation for the Commodore 64/128, running on a Raspberry Pi in a
[RAD Expansion Unit](https://github.com/frntc/RAD).

The Pi asserts `/DMA`, the C64's 6510 stops driving the bus, and the Pi becomes
the machine's CPU — emulating a WDC 65C816 out of its own fast RAM and only
touching the C64 for I/O and to keep VIC-visible memory coherent. That is the
same manoeuvre the real CMD SuperCPU performs.

Derived from [RAD-Doom](https://github.com/frntc/RAD-Doom) by Carsten
Dachsbacher, which established that a Pi can replace a C64's CPU over the
expansion port. GPLv3.

> **Status: it works.** A real C64 runs with the Pi as its 65816 — booting
> CMD's own SuperCPU DOS ROM to its splash and banner, JiffyDOS KERNAL, 16MB of
> SuperRAM detected by CMD's own code, stable display, and working IEC disk
> access against real drives. Speed is currently ~8-12MHz of the 20MHz target;
> the interpreter is the remaining bottleneck.
>
> On the host side the same stack runs on a PC: the CMD ROM boots against a
> fake IEC drive that serves a directory over the full slow serial protocol
> (`Tools/host_cmdboot/`), and the 65816 core agrees with the 6502 core
> instruction for instruction across half a million checks.
> See [Docs/roadmap.md](Docs/roadmap.md).

## Hardware

- A RAD Expansion Unit ([build info](https://github.com/frntc/RAD))
- Raspberry Pi 3A+, 3B+ or Zero 2
- A PAL C64 or C128 (NTSC is modelled but unvalidated)

## Quick start

```sh
make tests          # host test suite — needs only g++, no hardware
make firmware       # Raspberry Pi kernel image — needs a Circle tree
```

See [Docs/build.md](Docs/build.md) for the Circle setup, and
[Docs/hardware-setup.md](Docs/hardware-setup.md) for the SD card layout.

## ROMs

None ship with this project. SCPU-EMU works with no ROM files at all: it
snapshots BASIC and KERNAL off the running machine over the bus. Supply files
only if you want a specific KERNAL revision or a real SuperCPU ROM image. See
[ROMs/README.md](ROMs/README.md).

## Layout

```
Source/
  App/          start-up, Circle entry point
  CPU/          CPU cores, isolated behind cpu_bus.h
    M6502/      milestone-1 core and test oracle
    W65C816/    the real core (not yet written)
  C64/          PLA banking, shadowed bank 0
  SuperCPU/     registers, write mirroring, optimization modes
  Bus/
    RAD/        timing-critical hardware code
    Host/       in-memory backend so everything above is testable on a PC
    C64Side/    6502 stubs injected over Ultimax
  Common/
Tests/          host test suite
Docs/
  research/     what was learned about the SuperCPU and RAD, with sources
```

## Why this is tractable

RAD is not a generic cartridge that happens to be programmable — it is a
Raspberry Pi that can be bus master on a C64, and RAD-Doom already used it to
replace the CPU. The gap between "run Doom on the ARM and POKE the results in"
and "run a 65816 on the ARM and POKE the results in" is smaller than it looks.

The hard part is not the CPU emulation. A 65816 interpreter on a 1GHz A53 clears
20MHz comfortably. The hard part is that every write the VIC-II might read has
to be pushed back over the expansion port at roughly 1µs each, against ~50ns per
emulated cycle. Managing that traffic is what the SuperCPU's optimization modes
were for, and it is what most of the code here is about. See
[Docs/architecture.md](Docs/architecture.md).

## Credits

- Carsten Dachsbacher for the RAD Expansion Unit and RAD-Doom, which this is
  built on and which contains the hard-won bus timing.
- CMD, for the SuperCPU.

Commodore, CMD and Raspberry Pi trademarks belong to their respective owners.

## License

GPLv3, inherited from RAD-Doom. See [LICENSE](LICENSE).
