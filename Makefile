#
# SCPU-EMU - top-level Makefile
#
#   make tests    build and run the host test suite (needs only g++)
#   make firmware build the Raspberry Pi kernel image (needs a Circle tree,
#                 see Docs/build.md -- delegates to Source/Makefile)
#   make clean
#
# The host build compiles everything that is platform independent: the CPU
# cores, the C64 banking and memory model, the SuperCPU layer, and the host bus
# backend. Source/Bus/RAD and Source/App are excluded -- they need Circle and a
# Raspberry Pi.
#

CXX      ?= g++

# -fno-rtti -fno-exceptions: this is embedded-oriented code that uses neither,
# and the firmware build does not have them either, so the host tests should
# exercise the same code generation. It also sidesteps a silent ld failure
# (exit 116, no diagnostic) when linking weak typeinfo from these headers under
# MSYS2/mingw-w64 GCC 14 -- see Docs/build.md.
CXXFLAGS ?= -O2 -g -Wall -Wextra -Wno-unused-parameter -std=c++14 \
            -fno-rtti -fno-exceptions -DSCPU_HOST_BUILD

BUILDDIR  = build/host

HOST_SRCS = \
	Source/CPU/M6502/m6502.cpp \
	Source/CPU/M6502/m6502_opcodes.cpp \
	Source/CPU/W65C816/w65c816.cpp \
	Source/CPU/W65C816/w65c816_opcodes.cpp \
	Source/C64/banking.cpp \
	Source/C64/c64_memory.cpp \
	Source/SuperCPU/write_buffer.cpp \
	Source/SuperCPU/registers.cpp \
	Source/SuperCPU/supercpu.cpp \
	Source/SuperCPU/fast_ram.cpp \
	Source/SuperCPU/memory_map.cpp \
	Source/Bus/Host/host_bus.cpp

TEST_SRCS = \
	Tests/test_main.cpp \
	Tests/CPU/test_m6502.cpp \
	Tests/CPU/test_w65c816.cpp \
	Tests/CPU/test_w65c816_diff.cpp \
	Tests/C64/test_banking.cpp \
	Tests/SuperCPU/test_write_buffer.cpp \
	Tests/Integration/test_kernal_boot.cpp \
	Tests/Integration/test_real_kernal.cpp \
	Tests/Integration/test_kernal_65816.cpp \
	Tests/SuperCPU/test_memory_map.cpp

HOST_OBJS = $(HOST_SRCS:%.cpp=$(BUILDDIR)/%.o)
TEST_OBJS = $(TEST_SRCS:%.cpp=$(BUILDDIR)/%.o)

TEST_BIN = $(BUILDDIR)/scpu-tests

.PHONY: all tests build-tests firmware clean

all: tests

tests: build-tests
	@$(TEST_BIN)

build-tests: $(TEST_BIN)

$(TEST_BIN): $(HOST_OBJS) $(TEST_OBJS)
	@mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS) -o $@ $^

$(BUILDDIR)/%.o: %.cpp
	@mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS) -MMD -MP -c -o $@ $<

-include $(HOST_OBJS:.o=.d) $(TEST_OBJS:.o=.d)

# Builds the Raspberry Pi kernel image. Fetches the AArch64 toolchain and
# Circle 44.3 into _toolchain/ on first run, applies RAD's Circle settings, then
# builds. See Docs/build.md.
firmware:
	@bash Tools/setup_buildchain.sh

# Rebuild the firmware only, assuming the chain is already set up.
firmware-quick:
	@bash Tools/setup_buildchain.sh

# Assemble everything that goes on the SD card into SDCard/.
# kernel8.img comes from `make firmware` and is copied in if it exists.
SDDIR = SDCard

.PHONY: sdcard
sdcard:
	@mkdir -p $(SDDIR)/SCPU
	@cp -f Firmware/RaspberryPi/bootcode.bin      $(SDDIR)/ 2>/dev/null || echo "  MISSING bootcode.bin      (see Firmware/RaspberryPi/README.md)"
	@cp -f Firmware/RaspberryPi/start.elf         $(SDDIR)/ 2>/dev/null || echo "  MISSING start.elf         (see Firmware/RaspberryPi/README.md)"
	@cp -f Firmware/RaspberryPi/fixup.dat         $(SDDIR)/ 2>/dev/null || echo "  MISSING fixup.dat         (see Firmware/RaspberryPi/README.md)"
	@# config.txt sets gpu_mem=16, which makes the firmware load the CUT-DOWN
	@# loader instead of start.elf/fixup.dat. Omitting these two reports as
	@# "start.elf not found" (3 ACT-LED flashes) even though start.elf is present.
	@cp -f Firmware/RaspberryPi/start_cd.elf      $(SDDIR)/ 2>/dev/null || echo "  MISSING start_cd.elf      -- REQUIRED, gpu_mem=16 selects it over start.elf"
	@cp -f Firmware/RaspberryPi/fixup_cd.dat      $(SDDIR)/ 2>/dev/null || echo "  MISSING fixup_cd.dat      -- REQUIRED, pairs with start_cd.elf"
	@cp -f Firmware/RaspberryPi/LICENCE.broadcom  $(SDDIR)/ 2>/dev/null || true
	@cp -f Firmware/RaspberryPi/config.txt        $(SDDIR)/
	@cp -f Firmware/ARMSTUB/rad-prefetch.bin      $(SDDIR)/ 2>/dev/null \
		|| echo "  MISSING rad-prefetch.bin  -- run 'make firmware'; config.txt selects it via armstub= and the bus timings depend on it"
	@cp -f Config/default.cfg                     $(SDDIR)/SCPU/scpu.cfg
	@cp -f ROMs/kernal.rom  $(SDDIR)/SCPU/ 2>/dev/null || true
	@cp -f ROMs/basic.rom   $(SDDIR)/SCPU/ 2>/dev/null || true
	@cp -f ROMs/chargen.rom $(SDDIR)/SCPU/ 2>/dev/null || true
	@# The SuperCPU's own ROM: SuperCPU DOS and its JiffyDOS support. Optional --
	@# the accelerator boots the machine's own KERNAL without it.
	@#
	@# 1.4 is the SuperCPU 64 image. 2.04 is for the SuperCPU 128 -- it carries a
	@# C128 KERNAL and BASIC inside it, and on a 64 its boot code gets as far as
	@# "SUPERCPU INITIALIZATION ERROR: 06". Do not stage 2.04 for a C64.
	@cp -f ROMs/scpu-dos-1.4.bin $(SDDIR)/SCPU/scpu.rom 2>/dev/null \
		|| echo "  (no SuperCPU ROM staged -- optional, see ROMs/README.md)"
	@cp -f Source/kernel8.img $(SDDIR)/ 2>/dev/null \
		|| echo "  MISSING kernel8.img       -- run 'make firmware' first (needs a Circle tree, see Docs/build.md)"
	@cp -f Tools/sdcard_readme.txt $(SDDIR)/README.txt 2>/dev/null || true
	@echo ""
	@echo "SD card staged in $(SDDIR)/ :"
	@cd $(SDDIR) && find . -type f | sort | sed 's|^\./|  |'

clean:
	rm -rf build
	-$(MAKE) -C Source clean 2>/dev/null || true

clean-sdcard:
	rm -rf $(SDDIR)
