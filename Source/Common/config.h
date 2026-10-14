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


#define TIMING_NAMES 57
const char timingNames[][32] = {
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
	"DISPLAY_SCRUB_PERIOD_S",
	// Selects the eventual HDMI presentation source. This first-stage parser
	// deliberately does not alter runtime behaviour; mode 0 remains the exact
	// master-branch text-console path until the renderer lands separately.
	"VIDEO_MODE",
	// Size of the emulated RAM Expansion Unit. A SELECTOR, not a size:
	// 1 none, 2 128K, 3 256K, 4 512K, 5 2MB, 6 4MB, 7 16MB. 1 rather than
	// 0 means "no REU" so a missing or mistyped key is distinguishable
	// from a deliberate off. See Source/REU/reu.h.
	"REUSIZE",
	// Physical RAM-under-I/O policy. 0 always uses the conventional-map
	// compatibility fallback, 1 forces the verified $01=$34 map, and 2 probes
	// automatically. See cfgRAMUnderIO below.
	//
	// THE COMMA AFTER THIS ENTRY IS LOAD-BEARING. It was missing from K421 to
	// K425: C++ concatenated the two adjacent literals into one key named
	// RAM_UNDER_IOVSP_AIM_READ, the card's "RAM_UNDER_IO 1" stopped parsing,
	// boot silently fell back from the physical under-I/O map, and every
	// under-$D000 charset rendered from wrong DRAM -- six hardware builds and
	// four wrong theories chased what one byte of punctuation caused. A
	// static_assert on the initializer count follows in the guard commit.
	"RAM_UNDER_IO",
	// Diagnostic calibration, cycle-exact VSP work: move the REAL-TIME deadline
	// of each placed VIC read/write by this many whole C64 cycles (-6..+6,
	// default 0). A physical phase trim only: the cycles the guest is charged,
	// the bus ledger and the pacer's grid never see it (CW65C816::busPhaseTrim).
	// Positive lands the access LATER. The horizontal shift of a VSP screen is
	// the readout: sweep VSP_AIM_WRITE until the shift is zero, and that value
	// is the machine's measured end-to-end placement constant.
	"VSP_AIM_READ",
	"VSP_AIM_WRITE",
	// Stop retiring emulated instructions while the VIC-II holds BA, the way a
	// real 6510 is frozen on a badline. 0 off, 1 (default) on except while a
	// serial transaction window is armed, 2 always. See CC64Memory::
	// baStallSettle and the hardware note in default.cfg.
	"BA_STALL",
	// Ceiling on ONE such freeze, in C64 cycles. 8..80, default 80 -- the top of
	// the range IS RAD_BA_STALL_CEILING, so nothing configurable is unreachable.
	"BA_STALL_MAX",
	// How much evidence a serial-transaction window needs. 0 legacy, 1 (default)
	// ignore a level on a line the guest is itself pulling down, 2 positive
	// evidence only. See CC64Memory::m_IECWindowMode and default.cfg.
	"IEC_WINDOW",
	// Hypothesis-free hang detector: freeze the interrupt ring when the set of
	// executed code addresses stops growing. 1 (default) on, 0 off. See the
	// W65_PROG_ block in Source/CPU/W65C816/w65c816.h.
	"HANG_WATCH",
	// Two RAM addresses whose writes are logged with value, writer PC and
	// cycle, in DECIMAL because this parser is atoi. 0 disarms a slot, which
	// makes $0000 the one address not watchable from the config. Default
	// 40840 = $9F88, Mario's frame-phase index: the byte the 2026-10-12 hang
	// spins on. A watch puts an out-of-line call on every write to that
	// address's 4K page, so do not point one at page $0 without measuring.
	"RAM_WATCH",
	"RAM_WATCH2",
	// Diagnostic-minimal mode. 0 (DEFAULT) keeps every instrument, because the
	// 2026-10-12 hang is NOT solved: HANG_WATCH, the interrupt ring, RAM_WATCH
	// and the VSP trace are all still producing the evidence the investigation
	// runs on, and a default that quietly disarmed them would be the worst kind
	// of speed-up. 1 drops only instruments that answer CLOSED questions, which
	// today means the K449 per-stage bus timers.
	//
	// WHY THOSE AND NOTHING ELSE. Verified in the rebuilt rad_bus.o with the
	// project toolchain: radAccumRead is inlined TWICE into radDirectRead --
	// once for radBusStages and, under the ( addr & 0xF000 ) == 0xD000 test, a
	// second straight-line copy for radBusStagesIO -- so every $Dxxx transfer
	// executes two ~40-instruction accumulator bodies against a measured
	// unstalled transfer floor of 639 ARM plain and 2001 turnaround. Every guest
	// I/O access pays it, because CRADBus::read/write reach the bus through
	// RAD_SPEEK/RAD_SPOKE and those are radDirectRead/radDirectWrite.
	//
	// AND WHY IT IS A CONFIG KEY RATHER THAN A REBUILD: the TI benchmark cannot
	// see this cost at all -- its loop executes out of Pi RAM and never touches
	// the expansion port -- so the A/B that matters is a Mario run, which means
	// it has to be selectable in two boots on the card. The per-write counters,
	// by contrast, are 3 instructions each and a runtime gate would cost 2 to
	// save 3; those use the compile-time SCPU_DIAG_WRITE_COUNTERS gate in
	// Source/C64/c64_memory.h instead.
	"DIAG_MINIMAL",
	// THE LATE-CASE VSP FIX, gated (K500). 1 serves a guest $D011 read that
	// falls inside the raster window (a $D012 anchor read at 1MHz within the
	// last 200 cycles) from the VIC register shadow instead of the bus, when
	// the anchor's low byte proves the raster MSB is 0. Bits 6..0 are already
	// shadow-served (the GEOS fix), so the value is identical; what goes is
	// the transfer (2.3 cycles) and the turnaround it forces on the write that
	// follows, so a DEC/INC $D011 pair reaches its count target early and the
	// catch-up places every frame on it. Exact value 1 only; default 0.
	"VSP_D011_SHADOW_READ",
	// THE WRITE LEAD (K502), quarter C64 cycles 0..3, default 2. The settle
	// that places a VIC write exits this much before the pacer's lock phase so
	// the write's entry into the bus primitive lands mid-cycle instead of on
	// the falling edge it must precede. Measured need: the write path from a
	// settle exit to the primitive is ~630 ARM heavier than the read path the
	// lock phase was tuned for (catch= -0.46 cycle on every DEC, letters
	// flickering 0/+1 on the oracle disk). 0 reproduces the pre-K502 placement.
	"VSP_WRITE_LEAD",
	// BISECT KEY (K505). 1 (default): K501's placement inside a raster window
	// -- the K243 pass paid inside the write's resync, lag 1, the catch-up
	// target anchored on the anchor read's cycle start. 0: K500's placement
	// for those three (resync then pass, lag 2 on a pending turnaround, target
	// on the read's return). Metal Dust's intro regressed somewhere in
	// K498-K504 while Mario was fixed; this splits the two without a rebuild.
	"VSP_PLACEMENT_K501",
	// TRACE FILTER (K506). 1: the vsp trace ring retains only VIC writes placed
	// inside a raster window (a $D012 anchor at 1MHz within 200 cycles). The
	// ring holds 128 events and a game writes $D011 every frame, so by the
	// time the card is pulled the intro that mattered is long overwritten;
	// with this on, the ring is the intro's windowed writes. Default 0.
	"VSP_TRACE_WINDOW_ONLY",
};

// A missing comma between two entries concatenates the literals, silently
// turning two keys into one plus an empty row -- exactly what cost six
// hardware builds on 2026-08-18 (RAM_UNDER_IOVSP_AIM_READ). With the first
// dimension inferred, that mistake now changes the row count and this line
// refuses to compile instead of shipping.
static_assert( sizeof( timingNames ) / sizeof( timingNames[ 0 ] )
               == TIMING_NAMES,
               "timingNames entry count != TIMING_NAMES -- missing comma or "
               "stale count" );

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

// What the Pi's HDMI output will show. Stage 1 only parses and validates this
// value; no runtime path consumes it yet.
extern int cfgVideoMode;

// Emulated REU size, as a REUSIZE_* selector from Source/REU/reu.h.
// Defaults to "no REU fitted", so an unconfigured card behaves exactly as
// it did before the unit existed.
extern int cfgREUSize;

// Physical RAM-under-I/O selection. The deterministic default is the clean
// conventional-map fallback. AUTO remains available for diagnostics, but
// takeover acceptance was proven not to predict whether a host's VIC tolerates
// per-access /GAME selection (the tested 64U is a false-full result).
extern int cfgRAMUnderIO;

// Diagnostic VSP placement calibration, cycles, -6..+6, default 0.
extern int cfgVSPAimRead;
extern int cfgVSPAimWrite;

// Emulated BA stall; see BA_STALL in the names table. Default 1.
extern int cfgBAStall;
extern int cfgBAStallMax;

#define SCPU_CFG_BA_STALL_DEFAULT     1
#define SCPU_CFG_BA_STALL_MAX_DEFAULT 80

// Evidence required to arm the serial-transaction window; see IEC_WINDOW in the
// names table. Default 1: a line the guest is pulling down is its own output
// looping back, not a drive answering.
extern int cfgIECWindow;

#define SCPU_CFG_IEC_WINDOW_DEFAULT   1

// Hang detector and watched-RAM write forensics; see HANG_WATCH and RAM_WATCH
// in the names table. cfgRAMWatch/cfgRAMWatch2 are 0 for "disarmed".
extern int cfgHangWatch;
extern int cfgRAMWatch;
extern int cfgRAMWatch2;

#define SCPU_CFG_HANG_WATCH_DEFAULT  1
// $9F88. Decimal because the config parser is atoi.
#define SCPU_CFG_RAM_WATCH_DEFAULT   40840
#define SCPU_CFG_RAM_WATCH2_DEFAULT  0

// See DIAG_MINIMAL in the names table. The default is OFF in the sense that
// nothing is disarmed: we are mid-investigation and the hang instruments must
// survive a default boot.
extern int cfgDiagMinimal;

// See VSP_D011_SHADOW_READ in the names table. 0 off (default), 1 on.
extern int cfgVSPD011ShadowRead;

// See VSP_WRITE_LEAD in the names table. Quarter cycles, 0..3.
extern int cfgVSPWriteLead;
#define SCPU_CFG_VSP_WRITE_LEAD_DEFAULT 2

// See VSP_PLACEMENT_K501 in the names table. 1 (default) or 0.
extern int cfgVSPPlacementK501;

// See VSP_TRACE_WINDOW_ONLY in the names table. 0 (default) or 1.
extern int cfgVSPTraceWindowOnly;

#define SCPU_CFG_DIAG_MINIMAL_DEFAULT 0

#define SCPU_CFG_RAM_UNDER_IO_OFF      0
#define SCPU_CFG_RAM_UNDER_IO_FORCE    1
#define SCPU_CFG_RAM_UNDER_IO_AUTO     2
#define SCPU_CFG_RAM_UNDER_IO_DEFAULT  SCPU_CFG_RAM_UNDER_IO_OFF

#define SCPU_CFG_VIDEO_CONSOLE  0
#define SCPU_CFG_VIDEO_VICII    1
#define SCPU_CFG_VIDEO_VDC      2
#define SCPU_CFG_VIDEO_DEFAULT  SCPU_CFG_VIDEO_CONSOLE

#define SCPU_CFG_MIRROR_DISPLAY_DEFAULT 1024

extern int readConfig( CLogger *logger, const char *DRIVE, const char *FILENAME );

#endif
