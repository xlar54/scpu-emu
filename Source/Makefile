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
	CPU/W65C816/w65c816.o \
	CPU/W65C816/w65c816_opcodes.o \
	C64/banking.o \
	C64/c64_memory.o \
	SuperCPU/supercpu.o \
	SuperCPU/registers.o \
	SuperCPU/write_buffer.o \
	SuperCPU/fast_ram.o \
	SuperCPU/memory_map.o \
	Bus/RAD/rad_bus.o \
	Bus/RAD/c128_refresh.o \
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

# Rules.mk appends "$(OPTIMIZE) -g" AFTER our CFLAGS, and the LAST -O flag
# wins -- so despite the -Ofast above, every object in this tree has always
# compiled at -O2. (Upstream RAD has the identical ordering, so its shipped
# bus timings were tuned at -O2 too; the comment above claiming -Ofast is
# wrong, and is left as history.) That accident is also why it is SAFE to
# leave Bus/RAD, App and Common exactly as they are: they keep building at
# the flags the working hardware timings were actually tuned with.
#
# The interpreter is different: measured on hardware it runs at ~0.3
# instructions per clock, stall-bound, with step() compiling to ~41KB against
# a 32K two-way L1I. Smaller code IS faster code here, so the dispatch unit
# gets -Os -- measured to shrink step() to ~17KB, comfortably inside the
# cache -- and the small hot memory/banking units get -O3. Target-specific
# appends land after Rules.mk's -O2 on the command line, so they win.
CPU/W65C816/w65c816.o:        CPPFLAGS += -Os
CPU/W65C816/w65c816_opcodes.o: CPPFLAGS += -O3
CPU/M6502/m6502.o:            CPPFLAGS += -O3
C64/c64_memory.o:             CPPFLAGS += -O3
C64/banking.o:                CPPFLAGS += -O3
SuperCPU/memory_map.o:        CPPFLAGS += -O3
SuperCPU/write_buffer.o:      CPPFLAGS += -O3
SuperCPU/supercpu.o:          CPPFLAGS += -O3

-include $(DEPS)

# Rules.mk's clean only sweeps the current directory; ours are in subdirectories.
EXTRACLEAN = App/*.o App/*.d CPU/M6502/*.o CPU/M6502/*.d \
             CPU/W65C816/*.o CPU/W65C816/*.d C64/*.o C64/*.d \
             SuperCPU/*.o SuperCPU/*.d Bus/RAD/*.o Bus/RAD/*.d \
             Common/*.o Common/*.d
