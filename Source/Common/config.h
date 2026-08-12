/*

  {_______            {_          {______
        {__          {_ __               {__
        {__         {_  {__               {__
     {__           {__   {__               {__
 {______          {__     {__              {__
       {__       {__       {__            {__   
         {_________         {______________		Expansion Unit
                
 RADExp - A framework for DMA interfacing with Commodore C64/C128 computers using a Raspberry Pi Zero 2 or 3A+/3B+
 Copyright (c) 2022 Carsten Dachsbacher <frenetic@dachsbacher.de>

 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/>.

*/
#ifndef _config_h
#define _config_h


#define TIMING_NAMES 41
const char timingNames[TIMING_NAMES][32] = {
	"WAIT_FOR_SIGNALS", 
	"WAIT_CYCLE_READ", 
	"WAIT_CYCLE_WRITEDATA", 
	"WAIT_CYCLE_READ_BADLINE", 
	"WAIT_CYCLE_READ_VIC2", 
	"WAIT_CYCLE_WRITEDATA_VIC2", 
	"WAIT_CYCLE_MULTIPLEXER", 
	"WAIT_CYCLE_MULTIPLEXER_VIC2", 
	"WAIT_TRIGGER_DMA", 
	"WAIT_RELEASE_DMA",
	"WAIT_OFFSET_CBTD",
	"WAIT_DATA_HOLD",
	// Was a second "WAIT_TRIGGER_DMA" upstream. The parser stops at the first
	// name that matches, so this entry could never be reached and
	// TIMING_TRIGGER_DMA was unsettable from the config file. Renamed so it
	// works; index 8 remains WAIT_TRIGGER_DMA.
	"TIMING_TRIGGER_DMA",
	"WAIT_ENABLE_ADDRLATCH",
	"WAIT_READ_BA_WRITING",
	"WAIT_ENABLE_RW_ADDRLATCH",
	"WAIT_ENABLE_DATA_WRITING", 
	"WAIT_BA_SIGNAL_AVAIL",
	"CACHING_L1_WINDOW_KB",
	"CACHING_L2_OFFSET_KB",
	"CACHING_L2_PRELOADS_PER_CYCLE",
	// Not a timing: 0 selects the 6502 core, 1 the 65816. It rides in the same
	// table because that is the whole config mechanism, and having it on the SD
	// card means switching cores -- or falling back after a bad run -- needs no
	// rebuild and no toolchain.
	"CPU_CORE",
	// Also not a timing: 1 maps the SuperCPU's own ROM over bank 0 at reset.
	"BOOTMAP",
	// Virtual replacement for the cartridge's physical JiffyDOS switch.
	"JIFFYDOS",
	// Also not a timing: mirrored bytes allowed per drain while the VIC-II is
	// drawing the picture. 0 restores strict border-only mirroring.
	"MIRROR_DISPLAY_BYTES",
	// The SID drives the data bus later than every other chip; reads of
	// $D400-$D7FF sample at this point instead of the shared one.
	"WAIT_CYCLE_READ_SID",
	// Not a timing: run 2.04's carried-but-skipped C64 startup animation.
	"BOOT_ANIMATION",
	// Diagnostic: after this many seconds of uptime, STOP all mirror bus
	// traffic (flushes and the resync sweep) so the display shows whatever
	// real DRAM holds, undisturbed. Separates "our traffic corrupts fetches"
	// from "our writes corrupt DRAM". 0 = never halt (normal operation).
	"MIRROR_HALT_AFTER_S",
	// Not a timing: when a VIC-bank-3 sprite pointer selects a shape block at
	// $D000-$DFFF (RAM to the VIC, chip registers to our bus writes), deliver
	// a translated pointer into a relocated copy at $C000-$CBFF instead of
	// silently showing stale DRAM. 3D Pool needs it; see CC64Memory.
	"MIRROR_D000_RELOCATE",
	// Not a timing: print the once-per-second status line. The write goes out
	// through the logger with IRQs briefly unmasked, and that window is long
	// enough to show as a momentary display stutter -- so it defaults OFF.
	// Stall self-detection stays armed either way; it only prints when a
	// stall is actually found.
	"HEARTBEAT",
	// Not a timing: deliver CIA2 timer NMIs at the exact emulated cycle the
	// timer writes imply, instead of at the sampled line's µs-jittered
	// arrival. Winter Games /SCPU cycle-times a 20-cycle NMI around
	// native-mode windows; sampling slack landed one inside and the machine
	// BRK-stormed through the KERNAL's accidental $6C03 vector.
	"NMI_RETIME",
	// Not a timing: fetch interrupt vectors from the accelerator's own EPROM
	// ($F80000+vector) when it is acting as an accelerator, as a real
	// SuperCPU does and VICE models (scpu64_interrupt_reroute).
	"VECTOR_REROUTE",
	// Not a timing: charge a C64-bus access a whole C64 cycle, the way a real
	// SuperCPU pays for one. Without it, I/O-bound code runs several times
	// too fast in emulated time. See CC64Memory::tickFast.
	"IO_STRETCH",
	// Compatibility: hold CIA2 NMIs across short native-mode windows. See
	// CW65C816::m_DeferNativeNMI and the hardware note in default.cfg.
	"NMI_NATIVE_DEFER",
	// Model the real card's one-deep mirrored-write buffer in emulated time.
	// Off by default on RAD because physical deferred delivery already consumes
	// that time and charging it here too causes visible scheduler starvation.
	"MIRROR_STRETCH",
	// Physical C128 operating-mode selection: 0 auto, 1 force C64, 2 native.
	"C128_MODE",
	// Diagnostic: stop every physical expansion-bus read and write after N
	// seconds while leaving the emulated CPU alive. 0 disables it.
	"BUS_HALT_AFTER_S",
	// Diagnostic-only C64 image mode. 1 runs the broad non-display RAM oracle;
	// 2 runs the displayed-hires VIC-fetch oracle; 3 distinguishes immediate
	// write/address failure from delayed retention at $2078; 4 maps display-RAM
	// row sensitivity with high-entropy single/repeated writes and independent
	// retention delays; 5 crosses the same oracle with active text/bitmap VIC
	// fetches and controlled bus traffic. All save and halt before CPU-core
	// startup. 0 keeps the normal boot path.
	"BUS_ACCESS_SENTINEL",
	// Repair the empirically vulnerable quarter of the active bitmap fetch set
	// from bank-0 shadow during raster-safe opportunities. Text mode is excluded.
	"DISPLAY_SCRUB",
	// 1 uses the measured A1=0/A3=1 address mask; 0 scrubs full coverage.
	"DISPLAY_SCRUB_MASK",
	// Trial instrumentation: when nonzero, alternate scrub off/on at this
	// interval. 0 is static mode. The production scheduler remains write-only:
	// some physical machines have no stable expansion-bus read window.
	"DISPLAY_SCRUB_PERIOD_S"
};

// Which CPU core boot.cpp should install. Set from CPU_CORE in the config file;
// SCPU_CFG_CORE_DEFAULT applies when the key is absent.
extern int cfgCPUCore;

#define SCPU_CFG_CORE_6502      0
#define SCPU_CFG_CORE_65816     1
#define SCPU_CFG_CORE_DEFAULT   SCPU_CFG_CORE_65816

// Whether to let the SuperCPU's own ROM run at reset. Set from BOOTMAP in the
// config file. Ignored unless SCPU/scpu.rom is present and at least 64K.
extern int cfgBootmap;

#define SCPU_CFG_BOOTMAP_DEFAULT 1

// Initial position of the virtual JiffyDOS switch. The config file is read
// before the SuperCPU ROM runs, so CMD's boot code sees this value when it
// chooses which KERNAL to install.
extern int cfgJiffyDOS;

#define SCPU_CFG_JIFFYDOS_DEFAULT 1

// Mirrored bytes per drain call while the beam is in the visible display.
// Off by default: bus-safe, but it can replay a game's sprite writes while the
// VIC is fetching that sprite, tearing shapes that the game itself had timed
// correctly. Raise it only for games that write more per frame than the border
// can carry. See CSuperCPU::setMirrorDisplayBudget.
extern int cfgMirrorDisplayBytes;

// Run SuperCPU DOS 2.04's C64 startup animation, which the ROM carries but
// skips on the faithful path. One-byte read deviation; see
// CSuperCPURegisters::ioRead.
extern int cfgBootAnimation;

// Diagnostic; see MIRROR_HALT_AFTER_S in the names table.
extern int cfgMirrorHaltAfterS;

// Under-I/O sprite-shape relocation; see MIRROR_D000_RELOCATE in the names
// table. Default on.
extern int cfgMirrorD000Relocate;

// Once-per-second status line; see HEARTBEAT in the names table. Default off:
// the logger write stutters the display for a visible instant.
extern int cfgHeartbeat;

// CIA2 timer NMI retiming; see NMI_RETIME in the names table. Default on.
extern int cfgNMIRetime;

// Interrupt vector reroute; see VECTOR_REROUTE in the names table.
extern int cfgVectorReroute;

// C64-bus access cost; see IO_STRETCH in the names table. Default off on RAD.
extern int cfgIOStretch;
extern int cfgNMINativeDefer;
extern int cfgMirrorStretch;
extern int cfgC128Mode;
extern int cfgBusHaltAfterS;
extern int cfgBusAccessSentinel;
extern int cfgDisplayScrub;
extern int cfgDisplayScrubMask;
extern int cfgDisplayScrubPeriodS;

#define SCPU_CFG_MIRROR_DISPLAY_DEFAULT 1024

extern int readConfig( CLogger *logger, const char *DRIVE, const char *FILENAME );

#endif
