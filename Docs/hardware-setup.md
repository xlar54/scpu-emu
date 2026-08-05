# Hardware setup

## What you need

- A RAD Expansion Unit — see the [RAD project page](https://github.com/frntc/RAD)
  for PCB, BOM and assembly.
- A Raspberry Pi 3A+, 3B+ or Zero 2.
- A PAL C64 or C128. NTSC is modelled but has not been validated.

## SD card

FAT-formatted. Layout:

```
/kernel8-32.img        (or kernel8.img, per your Pi model)
/config.txt
/SCPU/scpu.cfg         bus timings -- start from Config/default.cfg
/SCPU/basic.rom        optional, 8192 bytes
/SCPU/kernal.rom       optional, 8192 bytes
/SCPU/chargen.rom      optional, 4096 bytes
```

`SCPU/scpu.cfg` also selects the virtual JiffyDOS switch. `JIFFYDOS 1` enables
it (the default), while `JIFFYDOS 0` disables it. The setting is applied before
the SuperCPU boot ROM runs.

No ROM files are required. Without them SCPU-EMU snapshots BASIC and KERNAL off
the running machine. See [../ROMs/README.md](../ROMs/README.md).

## Powering up

Follow the RAD project's guidance, which is specific about this:

1. **Use an external power supply for the Raspberry Pi.** RAD can be powered
   from the expansion port via a jumper, but its author reports that this needs
   a very strong and stable supply, "often does not work at all", and is a
   likely cause of instability.
2. **NEVER power the RAD from the computer and externally at the same time.**
   RAD's documentation is emphatic about this.
3. **Boot the Pi first, then switch on the C64.** The circuitry has pull-ups and
   pull-downs so it should not disturb the bus while booting, but this order is
   the safe one.
4. **Never insert or remove the cartridge with power on.** Hot-plugging the
   expansion port can damage the machine, the Pi, or both.

SCPU-EMU pulses `/RESET` itself and waits for the machine to come up before
taking the bus, so it does not matter whether the C64 has finished booting when
the Pi is ready — it will be reset regardless.

To retry after a failure: power off the C64, power-cycle the Pi, then bring the
C64 back up.

## Clock speed and bus timings

They are coupled. The timing constants in `SCPU/scpu.cfg` are counts of ARM
clock cycles, so they are only valid at the clock `config.txt` pins. Changing
`arm_freq` changes every bus timing at once.

`Config/default.cfg` carries RAD's `AUTO_TIMING_RPI3PLUS_C64C128` values, which
assume a Pi 3A+/3B+ at its stock 1400MHz. A **Pi Zero 2 W is stock 1000MHz** and
RAD overclocks it substantially; if that is your board, take `config.txt` from
the official RAD release archive rather than the one here, so the clock matches
what those timings were tuned against.

## Bus timings

The RAD bus timings depend on the Pi model and on the individual machine. If the
display corrupts or the C64 hangs on takeover, adjust them.

The reliable procedure is to get the **stock RAD software** working first and
tune timings there using its built-in test tooling, then copy the values into
`SCPU/scpu.cfg`. The parameter names are identical.

Starting points are in [../Config/](../Config/).

## Do not stack cartridges

An accelerator drives the bus. Anything else that drives `/GAME`, `/EXROM`,
`/DMA` or the data lines will conflict. Ultimax-mode cartridges (Action Replay,
Super Snapshot) are architecturally incompatible with an accelerator — this is
true of the real SuperCPU too, and is documented in the CMD manual.

As the RAD project puts it: be careful not to damage your Pi or your 8-bit
machine. Use at your own risk.
