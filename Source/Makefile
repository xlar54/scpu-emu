#
# SCPU-EMU - Circle bare-metal build for the Raspberry Pi
#
# Layout follows the RAD Expansion Unit, not RAD-Doom: this directory is
# expected to sit two levels below a Circle root, with RAD's Rules.mk one level
# up. Tools/setup_buildchain.sh assembles that tree for you; see Docs/build.md.
#
#   <circle>/
#     Source/
#       Rules.mk         <- Firmware/Circle/Rules.mk
#       Firmware/        <- this directory
#
# RAD-Doom used a newlib-based tree because Doom needs stdio and libm. SCPU-EMU
# needs neither, so it links against Circle plus liblinuxemu exactly as the
# stock RAD firmware does.
#

CIRCLEHOME = ../..

OBJS = \
	App/main.o \
	App/boot.o \
	CPU/M6502/m6502.o \
	CPU/M6502/m6502_opcodes.o \
	C64/banking.o \
	C64/c64_memory.o \
	SuperCPU/supercpu.o \
	SuperCPU/registers.o \
	SuperCPU/write_buffer.o \
	SuperCPU/fast_ram.o \
	SuperCPU/memory_map.o \
	Bus/RAD/rad_bus.o \
	Bus/RAD/cpu_hijack.o \
	Bus/RAD/bus_timing.o \
	Bus/RAD/lowlevel_dma.o \
	Bus/RAD/lowlevel_arm64.o \
	Bus/RAD/gpio_defs.o \
	Common/config.o \
	Common/helpers.o

LIBS = $(CIRCLEHOME)/addon/linux/liblinuxemu.a

# -Ofast matches RAD: the bus timing constants were tuned at this optimisation
# level and changing it will move them.
CFLAGS += -fno-threadsafe-statics
CFLAGS += -Wno-comment -Ofast -g0

LIBS += $(CIRCLEHOME)/addon/SDCard/libsdcard.a \
	    $(CIRCLEHOME)/lib/fs/libfs.a \
		$(CIRCLEHOME)/addon/fatfs/libfatfs.a \
	    $(CIRCLEHOME)/lib/sched/libsched.a \
        $(CIRCLEHOME)/lib/libcircle.a

include ../Rules.mk

-include $(DEPS)

# Rules.mk's clean only sweeps the current directory; ours are in subdirectories.
EXTRACLEAN = App/*.o App/*.d CPU/M6502/*.o CPU/M6502/*.d C64/*.o C64/*.d \
             SuperCPU/*.o SuperCPU/*.d Bus/RAD/*.o Bus/RAD/*.d \
             Common/*.o Common/*.d
