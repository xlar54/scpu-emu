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

None ship with this project. Firmware startup requires `basic.rom`,
`kernal.rom`, `chargen.rom`, and the 128KB SuperCPU DOS 2.04 image as
`scpu.rom` under `SCPU/` on the SD card. If the set is incomplete, the RAD does
not acquire the bus and the physical Commodore boots normally. See
[ROMs/README.md](ROMs/README.md).

**The SuperCPU ROM is copyright CMD.** It is not included here and never will
be. It is available from CoreI64 — Thomas Christoph — at
<https://www.corei64.com/shop/>.

Commodore ROMs are copyright Commodore International Corporation. CMD is a
trademark of Creative Micro Designs. Dump ROMs from hardware you own, or obtain
them from a source you are entitled to use.

## Configuration — `SCPU/scpu.cfg`

The card is configured by a single plain-text file, `SCPU/scpu.cfg`, staged from
[Config/default.cfg](Config/default.cfg) by `make sdcard`. One `KEY value` per
line, `#` for comments. The Pi reads it before any emulation starts, so a
setting always takes effect no matter how badly the emulated machine behaves —
which is what makes it a safe recovery path.

**`Config/default.cfg` is the authoritative reference.** Every key carries a
comment there explaining what it does and why its default is what it is. The
table below covers the settings you would normally choose between; the rest are
timing and cache parameters that should be left alone unless you are working on
bus timing itself.

### Machine

| Key | Values | Notes |
|---|---|---|
| `JIFFYDOS` | `1` on, `0` off | The virtual replacement for the physical switch on a real SuperCPU. `POKE 53429,128` / `POKE 53429,0` changes it until the Pi reboots. |
| `REUSIZE` | `1` none, `2` 128K, `3` 256K, `4` 512K, `5` 2MB, `6` 4MB, `7` 16MB | RAM Expansion Unit. 1MB is deliberately absent — selectors are appended, never renumbered, so a card in the field cannot change machine because the table grew. |
| `CPU_CORE` | `1` 65816, `0` 6502 | `0` is a fallback: everything a normal C64 does still works, but SuperRAM is unreachable because a 6502 cannot name an address above `$FFFF`. Quickest way to find out whether the CPU core is responsible for a regression. |
| `BOOTMAP` | `1` on, `0` off | Runs the SuperCPU ROM at reset. Firmware startup requires the 128KB SuperCPU DOS 2.04 image at `SCPU/scpu.rom`. **If the machine does not boot, set this to 0.** |
| `C128_MODE` | `0` auto, `1` force C64 path, `2` force native C128 | `2` is experimental. On a C128 the physical machine type and the operating mode are different things, and `$0001` is internal to the 8502 so a DMA master cannot read it. |
| `BOOT_ANIMATION` | `1` on, `0` faithful | `1` runs the C64 startup animation that SuperCPU DOS 2.04 carries but skips. A deliberate one-byte deviation from strict fidelity. |

### Video

`VIDEO_MODE` selects what the **Pi's HDMI output** shows. It does not change how
the C64's own screen is driven — the physical VIC-II does that in every mode.

| Value | HDMI output |
|---|---|
| `0` | Firmware text console. The safe baseline; the runtime path is unchanged. |
| `1` | The VIC-II picture, rendered by the Pi from its memory shadow. |
| `2` | C128 VDC picture. Reserved for future native C128 support. |

> **`VIDEO_MODE 1` is experimental.** The Pi has to reconstruct the picture from
> its shadow while simultaneously meeting the C64's bus timing, and the two
> compete for the same scarce resource. Because of those timing and bandwidth
> limits it may not give the output you expect — expect artefacts, missing
> updates, or reduced fidelity on demanding software. `VIDEO_MODE 0` is the
> baseline to fall back to, and the mode to use when diagnosing anything else.

### Display mirroring

The Pi's shadow RAM is authoritative and physical DRAM is write-only, existing
purely so the VIC-II has something to fetch. These control how much of that
shadow gets pushed back out, and when.

| Key | Default | Notes |
|---|---|---|
| `MIRROR_DISPLAY_BYTES` | `1024` | Visible-display delivery allowance, roughly 8KB/frame, spread over 128 points so no single pause is long. `0` restores strict border-only mirroring. |
| `MIRROR_D000_RELOCATE` | `1` | Translates sprite shape pointers that land under `$D000-$DFFF`, which is DRAM to the VIC but I/O to any bus master and therefore unreachable. Without it, 3D Pool/SCPU's balls alternate between correct and garbage at frame rate. |
| `DISPLAY_SCRUB` | `0` | Targeted repair for sparse stored-data changes on bitmap screens. Text and charset modes are deliberately excluded. Leave disabled until bitmap-only hardware trials validate it. |
| `VECTOR_REROUTE` | `1` | Fetches interrupt vectors from the accelerator's ROM window under the conditions VICE models as `scpu64_interrupt_reroute()`. Ordinary C64 operation in emulation mode is unaffected. |

### Timing and caching

`IO_STRETCH`, the `WAIT_*` family and the `CACHING_*` family describe the bus
protocol and the Pi's cache behaviour. These are the values the whole design
rests on, and a wrong one produces symptoms — phantom keypresses, half
directories, corrupted loads — that look nothing like a timing problem. Read the
comments in `Config/default.cfg` and
[Docs/SuperCPU64/timing-notes.md](Docs/SuperCPU64/timing-notes.md) before
changing any of them.

## Third-party dependencies

Nothing is vendored. This records what to fetch and why.

### Circle

Bare-metal C++ environment for the Raspberry Pi. **Version 44.3**, which is what
RAD targets.

- <https://github.com/rsta2/circle>

Use RAD's Circle build settings; its README is explicit that other settings
probably will not work correctly. Not vendored because Circle is large, has its
own build configuration, and pinning it here would either fork it or silently
drift from what RAD expects. See [Docs/build.md](Docs/build.md).

### RAD Expansion Unit

- <https://github.com/frntc/RAD>
- <https://github.com/frntc/RAD-Doom>

GPLv3. The parts SCPU-EMU uses have been promoted into `Source/Bus/RAD/` with
attribution intact, rather than pulled in as a dependency, because they needed
renaming and decoupling from RAD's REU-specific state. What was taken and what
was changed is recorded in
[Docs/SuperCPU64/rad-notes.md](Docs/SuperCPU64/rad-notes.md).

### Toolchain

`aarch64-none-elf` GCC for the firmware; any host C++ compiler for the tests.

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
  SuperCPU64/   SCPU64 architecture and hardware research
  SuperCPU128/  SCPU128 hardware findings
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
[Docs/SuperCPU64/architecture.md](Docs/SuperCPU64/architecture.md).

## Credits

- Carsten Dachsbacher for the RAD Expansion Unit and RAD-Doom, which this is
  built on and which contains the hard-won bus timing.
- CMD, for the SuperCPU.

Commodore, CMD and Raspberry Pi trademarks belong to their respective owners.

## License

GPLv3, inherited from RAD-Doom. See [LICENSE](LICENSE).
