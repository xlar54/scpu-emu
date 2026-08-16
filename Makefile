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
	Source/Bus/Host/host_bus.cpp \
	Source/Video/vic_raster.cpp \
	Source/Video/vic_renderer.cpp \
	Source/REU/reu.cpp

TEST_SRCS = \
	Tests/test_main.cpp \
	Tests/CPU/test_m6502.cpp \
	Tests/CPU/test_w65c816.cpp \
	Tests/CPU/test_w65c816_diff.cpp \
	Tests/C64/test_banking.cpp \
	Tests/Video/test_vic_raster.cpp \
	Tests/Video/test_vic_renderer.cpp \
	Tests/SuperCPU/test_write_buffer.cpp \
	Tests/Integration/test_kernal_boot.cpp \
	Tests/Integration/test_real_kernal.cpp \
	Tests/Integration/test_kernal_65816.cpp \
	Tests/SuperCPU/test_memory_map.cpp \
	Tests/REU/test_reu.cpp

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

# --- VICE conformance tools -------------------------------------------------
# Host programs that drive the SHIPPING renderer, band planner and memory map
# against a reference emulator. Built from the same objects as the test suite,
# so a fix cannot pass here and behave differently on the card.
#
#   render_state  one machine state -> one frame        (geometry, modes, sprites)
#   replay_state  a running program -> a replayed frame (write log, anchors, bands)
#   sid_trace     a running program -> its bus traffic  (every SID write delivered)
#   ss816         SingleStepTests vectors -> our 65816   (external CPU truth)
#   scpu_trace    CMD's own ROM -> our register layer   (coverage, not semantics)
#
# Tools/viceconf/capture.sh drives xscpu64 and diffs the result.
VICECONF_BIN = $(BUILDDIR)/render_state $(BUILDDIR)/replay_state \
               $(BUILDDIR)/sid_trace $(BUILDDIR)/ss816 \
               $(BUILDDIR)/scpu_trace

.PHONY: viceconf
viceconf: $(VICECONF_BIN)

$(BUILDDIR)/render_state: $(BUILDDIR)/Tools/viceconf/render_state.o $(HOST_OBJS)
	@mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS) -o $@ $^

$(BUILDDIR)/replay_state: $(BUILDDIR)/Tools/viceconf/replay_state.o $(HOST_OBJS)
	@mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS) -o $@ $^

$(BUILDDIR)/sid_trace: $(BUILDDIR)/Tools/viceconf/sid_trace.o $(HOST_OBJS)
	@mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS) -o $@ $^

$(BUILDDIR)/ss816: $(BUILDDIR)/Tools/viceconf/ss816.o $(HOST_OBJS)
	@mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS) -o $@ $^

$(BUILDDIR)/scpu_trace: $(BUILDDIR)/Tools/viceconf/scpu_trace.o $(HOST_OBJS)
	@mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS) -o $@ $^

-include $(HOST_OBJS:.o=.d) $(TEST_OBJS:.o=.d) \
         $(BUILDDIR)/Tools/viceconf/render_state.d \
         $(BUILDDIR)/Tools/viceconf/replay_state.d \
         $(BUILDDIR)/Tools/viceconf/sid_trace.d \
         $(BUILDDIR)/Tools/viceconf/ss816.d \
         $(BUILDDIR)/Tools/viceconf/scpu_trace.d

# Re-render the compendium PDF from its HTML source.
#
# The PDF is the artifact people read and the HTML is the source, so they go
# stale apart silently -- which is exactly what happened, because there was no
# command to run. Chromium's print-to-PDF is what produced the original
# (Producer: Skia/PDF), so using it again keeps pagination and fonts consistent.
.PHONY: docs-pdf
docs-pdf:
	@src="$(CURDIR)/Docs/CMD-SuperCPU-Compendium.html"; \
	out="$(CURDIR)/Docs/CMD-SuperCPU-Compendium.pdf"; \
	for c in "/c/Program Files/Google/Chrome/Application/chrome.exe" \
	         "/c/Program Files (x86)/Microsoft/Edge/Application/msedge.exe" \
	         "/c/Program Files/Microsoft/Edge/Application/msedge.exe" \
	         "$$(command -v google-chrome 2>/dev/null)" \
	         "$$(command -v chromium 2>/dev/null)"; do \
	  [ -n "$$c" ] && [ -x "$$c" ] || continue; \
	  echo "  rendering with $$c"; \
	  "$$c" --headless --disable-gpu --no-sandbox --no-pdf-header-footer \
	        --print-to-pdf="$$(cygpath -m "$$out" 2>/dev/null || echo $$out)" \
	        "file:///$$(cygpath -m "$$src" 2>/dev/null || echo $$src)" || exit 1; \
	  echo "  wrote $$out"; exit 0; \
	done; \
	echo "  no Chromium-based browser found -- install one, or print"; \
	echo "  Docs/CMD-SuperCPU-Compendium.html to PDF by hand"; exit 1

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
	@# SuperCPU DOS 2.04, confirmed working on this machine. An early debugging
	@# round blamed 2.04 for "INITIALIZATION ERROR: 06" and switched to 1.4; the
	@# real culprits were since-fixed emulation bugs, and 2.04 is preferred.
	@cp -f ROMs/scpu-dos-2.04.bin $(SDDIR)/SCPU/scpu.rom 2>/dev/null \
		|| echo "  (no SuperCPU ROM staged -- optional, see ROMs/README.md)"
	@cp -f Source/kernel8.img $(SDDIR)/ 2>/dev/null \
		|| echo "  MISSING kernel8.img       -- run 'make firmware' first (needs a Circle tree, see Docs/build.md)"
	@echo ""
	@echo "SD card staged in $(SDDIR)/ :"
	@cd $(SDDIR) && find . -type f | sort | sed 's|^\./|  |'

clean:
	rm -rf build
	-$(MAKE) -C Source clean 2>/dev/null || true

clean-sdcard:
	rm -rf $(SDDIR)
