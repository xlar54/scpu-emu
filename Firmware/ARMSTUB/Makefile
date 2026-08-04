#
# RAD's custom ARM stub for the Raspberry Pi 3 / Zero 2.
#
# The GPU firmware runs this before our kernel. RAD's version disables L1 data
# prefetching (CPUACTLR_EL1.L1PCTL), because speculative prefetches introduce
# nondeterministic memory stalls -- and every C64 bus access is driven inside a
# window counted in ARM cycles. RAD's bus timings were tuned with this stub in
# place, so running without it applies those timings under different conditions.
#
# It is also the Pi 3 variant: LOCAL_CONTROL/LOCAL_PRESCALER at 0x40000000 and a
# 19.2MHz oscillator, versus the Pi 4 values in the stock armstub8.S alongside.
#
# Select it from config.txt with:   armstub=rad-prefetch.bin
#
# Derived from the Raspberry Pi armstub (BSD), modified by Carsten Dachsbacher.
#
# The upstream RAD Makefile named its target no-prefetch.bin while the recipe
# produced rad-prefetch.bin, so make never saw the target appear and rebuilt it
# every time. Corrected here.
#

PREFIX ?= aarch64-none-elf-

all: rad-prefetch.bin

# -Ttext=0 is essential. The GPU firmware loads the armstub at physical address
# 0 and the ARM begins executing there in EL3. Linking anywhere else leaves the
# literal pools and absolute references pointing into nothing, start.elf fails
# to launch it, and the Pi reports four ACT-LED flashes without ever reaching
# our kernel.
#
# The upstream RAD Makefile omits this because RAD ships a prebuilt binary; the
# omission only bites when you rebuild it from source.
rad-prefetch.bin: rad-prefetch.S
	$(PREFIX)gcc -mcpu=cortex-a53 -c rad-prefetch.S -o rad-prefetch.o
	$(PREFIX)ld -Ttext=0 -o rad-prefetch.elf rad-prefetch.o
	$(PREFIX)objdump -D rad-prefetch.elf > rad-prefetch.lst
	$(PREFIX)objcopy rad-prefetch.elf -O binary rad-prefetch.bin

clean:
	rm -f *.o *.elf *.bin *.lst

.PHONY: all clean
