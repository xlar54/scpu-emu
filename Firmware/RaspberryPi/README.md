# Raspberry Pi boot firmware

The Pi's GPU bootloader, which loads `kernel8.img` (our firmware) and starts the
ARM cores. Nothing to do with the C64 side.

## Files

| File | Purpose |
|---|---|
| `bootcode.bin` | second-stage bootloader, run by the GPU from ROM |
| `start.elf` | GPU firmware; loads and starts the ARM |
| `fixup.dat` | memory split between ARM and GPU, paired with `start.elf` |
| `config.txt` | boot configuration — this one is ours, see below |
| `LICENCE.broadcom` | Broadcom's licence; must accompany redistribution |

These are ignored by git. They are freely redistributable, but `start.elf` alone
is 3MB and it is better fetched than vendored. To (re)fetch:

```sh
B=https://raw.githubusercontent.com/raspberrypi/firmware/master/boot
for f in bootcode.bin start.elf fixup.dat LICENCE.broadcom; do
  curl -sSL -o "Firmware/RaspberryPi/$f" "$B/$f"
done
```

Pi 3A+/3B+ and Zero 2 W are all BCM2837-family and use `start.elf`/`fixup.dat`.
`start4.elf`/`fixup4.dat` are for the Pi 4 and are not needed — RAD does not
support it.

## Installing

Copy these plus the built `kernel8.img` to the root of a FAT-formatted SD card,
then add the `SCPU/` directory. See
[../../Docs/hardware-setup.md](../../Docs/hardware-setup.md).

## Source

<https://github.com/raspberrypi/firmware/tree/master/boot>
