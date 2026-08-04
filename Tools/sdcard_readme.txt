SCPU-EMU - SD card contents
===========================

Copy everything in this folder to the root of a FAT32-formatted SD card,
keeping the SCPU/ subdirectory.

  bootcode.bin       Raspberry Pi GPU bootloader
  start.elf          Raspberry Pi GPU firmware
  fixup.dat          ARM/GPU memory split
  LICENCE.broadcom   Broadcom licence for the three files above
  config.txt         Pi boot configuration -- READ THE NOTE BELOW
  kernel8.img        SCPU-EMU firmware

  SCPU/scpu.cfg      RAD bus timings
  SCPU/kernal.rom    stock C64 KERNAL
  SCPU/basic.rom     stock C64 BASIC
  SCPU/chargen.rom   stock C64 character generator


POWER-UP ORDER
--------------
1. Power the Raspberry Pi from an EXTERNAL supply. RAD can be powered from the
   expansion port via a jumper, but its author reports this needs a very strong
   supply, often does not work, and causes instability.

2. NEVER power the RAD from the computer and externally at the same time.

3. Boot the Pi FIRST, then switch on the C64.

4. Never insert or remove the cartridge with power on.

SCPU-EMU resets the C64 itself and waits for it to come up before taking the
bus, so it does not matter whether the C64 has finished booting first.


CLOCK SPEED -- IMPORTANT
------------------------
The timings in SCPU/scpu.cfg are counts of ARM clock cycles. They are only
valid at the clock config.txt pins, so changing arm_freq silently changes every
bus timing.

The supplied config.txt assumes a Raspberry Pi 3A+/3B+ at its stock 1400MHz.

A Pi Zero 2 W is stock 1000MHz, and RAD overclocks it substantially. If you are
running a Zero 2, use the config.txt from the official RAD release archive
instead of this one -- it carries the clock RAD's timing constants were
actually tuned against.


ABOUT THE ROM FILES
-------------------
Keep them. With kernal.rom present, SCPU-EMU runs that image and never executes
your machine's own KERNAL, so it does not matter what is fitted -- including
JiffyDOS, which does not need to be disabled.

Deleting them makes SCPU-EMU read BASIC and KERNAL off your running machine
instead. That is the more faithful option eventually, but not for a first
bring-up: if the machine has JiffyDOS, its fast-transfer routines are heavily
cycle-timed and will not survive the current unpaced execution.


WHAT TO EXPECT
--------------
This is not a finished SuperCPU. There is no 65816 core yet (a 6502 stands in),
no SuperRAM, and no cycle pacing -- so the machine runs far faster than 1MHz at
a speed nobody selected, and POKE 53370,0 will not slow it down.

Expect: a normal C64 startup, then the screen clearing and starting again. The
second one is SCPU-EMU.

Do not expect disk loading, games using undocumented opcodes, or anything
raster-timed to work yet.


IF IT DOES NOT WORK
-------------------
Bus timings vary between Raspberry Pi models and between individual machines.
The reliable procedure is to get the stock RAD software running first and tune
its timings there, then copy the working values into SCPU/scpu.cfg -- the
parameter names are identical.

See Docs/hardware-setup.md in the source repository.
