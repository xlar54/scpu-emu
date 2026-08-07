# Third-party dependencies

Nothing is vendored here. This file records what to fetch and why.

## Circle

Bare-metal C++ environment for the Raspberry Pi. **Version 44.3**, which is what
RAD targets.

- <https://github.com/rsta2/circle>

Use RAD's Circle build settings; its README is explicit that other settings
probably will not work correctly. Not vendored because Circle is large, has its
own build configuration, and pinning it here would either fork it or silently
drift from what RAD expects. See [../Docs/build.md](../Docs/build.md).

## RAD Expansion Unit

- <https://github.com/frntc/RAD>
- <https://github.com/frntc/RAD-Doom>

GPLv3. The parts SCPU-EMU uses have been promoted into `Source/Bus/RAD/` with
attribution intact, rather than pulled in as a dependency, because they needed
renaming and decoupling from RAD's REU-specific state. What was taken and what
was changed is recorded in
[../Docs/SuperCPU64/rad-notes.md](../Docs/SuperCPU64/rad-notes.md).

## Toolchain

`aarch64-none-elf` GCC for the firmware; any host C++ compiler for the tests.
