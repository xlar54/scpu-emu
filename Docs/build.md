# Building

## Host tests

Needs nothing but a C++ compiler.

```sh
make tests
```

This builds everything platform independent — CPU cores, C64 banking and memory
model, the SuperCPU layer, and the host bus backend — and runs the suite.
`Source/Bus/RAD/` and `Source/App/` are excluded; they need Circle and a Pi.

### Compiler flags

The host build uses `-fno-rtti -fno-exceptions`. Two reasons:

1. The firmware does not use RTTI or exceptions, so the tests should exercise
   the same code generation.
2. Without them, MSYS2 / mingw-w64 GCC 14 fails to link this code: `ld` exits
   with status 116 and prints nothing at all, even under `-Wl,--verbose`. It is
   reproducible with a two-object link of `m6502.o` plus a stub `main`, and goes
   away entirely with the flags above. If you hit a silent link failure on
   another toolchain, check this first.

## Firmware

```sh
make firmware
```

On first run this fetches the AArch64 bare-metal toolchain and Circle 44.3 into
`_toolchain/` (a few hundred MB, once), applies RAD's Circle build settings,
builds the Circle libraries, and then builds `kernel8.img`. Subsequent runs skip
everything already present and just rebuild the firmware.

`Tools/setup_buildchain.sh` does the work and can be run directly if you want to
pass extra make arguments.

### What the script assembles

RAD is explicit that its own Circle settings must be used — the stock ones are
not expected to work for hard-real-time bus driving. So the tree ends up as:

```
_toolchain/circle/                    <- CIRCLEHOME
  Config.mk                           RASPPI=3, AARCH=64, PREFIX64=aarch64-none-elf-
  include/circle/sysconfig.h          <- Firmware/Circle/sysconfig.h   (RAD's)
  Source/
    Rules.mk                          <- Firmware/Circle/Rules.mk      (RAD's)
    Firmware/                         <- a copy of this repo's Source/
```

`Source/Makefile` expects exactly that: `CIRCLEHOME = ../..` and
`include ../Rules.mk`.

Note this follows the **RAD Expansion Unit** layout, not RAD-Doom's. RAD-Doom
uses a newlib-based tree because Doom needs stdio and libm; SCPU-EMU needs
neither and links against Circle plus `liblinuxemu`, exactly as the stock RAD
firmware does. If you were following RAD-Doom's `Makefile` as a model, that is
the difference.

### Toolchain

`aarch64-none-elf-gcc`, from the ARM GNU toolchain releases. If you already have
one on `PATH` the script uses it and skips the download.

### Optimisation level

`-Ofast`, matching RAD. The bus timing constants in `Config/*.cfg` were tuned at
that level; changing it will move them.

### Why Circle is not vendored

It is large, has its own build configuration, and RAD is specific about which
settings to use. Vendoring would mean either shipping a fork or shipping
something that silently drifts from what RAD expects. The dependency section of
the [top-level README](../README.md#third-party-dependencies) records what to
fetch; the script does the fetching.

## SD card

```sh
make sdcard
```

Stages everything into `SDCard/`: the Pi boot firmware, `config.txt`,
`kernel8.img` if it has been built, `SCPU/scpu.cfg`, and any ROM images present
in `ROMs/`. Copy the contents to the root of a FAT32 card.

See [hardware-setup.md](hardware-setup.md).
