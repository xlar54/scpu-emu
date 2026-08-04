# host_cmdboot — the CMD boot + fake-drive rig

Boots the real SuperCPU DOS ROM through the emulator on a PC, self-types
`LOAD"$",10`, and serves the directory from a fake IEC drive implementing the
standard slow serial protocol — ATN commands, filename with EOI, talker
turnaround, byte clocking with debounce-stable phases, EOI on the last byte,
UNTALK. Includes coarse VIC raster and CIA1 interrupt synthesis (the splash
waits on raster interrupts) and a CIA1 timer-B one-shot (the KERNAL's EOI
timeout).

This rig is how the disk stack got debugged: every protocol-level bug became a
local edit-compile-run loop instead of an SD-card swap. Keep it working.

Build (from the repo root, after `make build-tests`):

    g++ -O2 -std=c++14 -fno-rtti -fno-exceptions -DSCPU_HOST_BUILD -I. \
        -o cmdload Tools/host_cmdboot/cmdload.cpp \
        build/host/Source/CPU/M6502/*.o build/host/Source/CPU/W65C816/*.o \
        build/host/Source/C64/*.o build/host/Source/SuperCPU/*.o \
        build/host/Source/Bus/Host/*.o

Run from the repo root (needs ROMs/ populated). Pass any argument for a
verbose per-transition protocol trace from the drive's point of view.

Known limitation: the run ends with the host waiting at the post-UNTALK
close-out — the fake drive's final CLOSE handshake is unfinished. The real
hardware path is complete; finishing the fake drive's tail is cosmetic.
