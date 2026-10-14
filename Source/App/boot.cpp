/*
   SCPU-EMU - CMD SuperCPU emulation for the C64/C128 using a RAD Expansion Unit
   Copyright (c) 2026 SCPU-EMU contributors

   Start-up sequence.

   ROM policy: basic.rom, kernal.rom, chargen.rom and the SuperCPU DOS 2.04
   scpu.rom are required under SCPU/. If any image is missing or unusable, the
   firmware leaves the Commodore bus untouched so the machine boots normally.

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
#include "boot.h"
#include "hdmi_display.h"
#include <circle/actled.h>
#include <circle/bcmpropertytags.h>
#include <circle/startup.h>
#include <circle/synchronize.h>

#include "../Common/config.h"
#include "../Common/helpers.h"
#include "../Common/runtime_diagnostic.h"
#include "../Bus/RAD/gpio_defs.h"
#include "../Bus/RAD/lowlevel_arm64.h"
#include "../Bus/RAD/bus_timing.h"
#include "../Bus/RAD/rad_bus.h"
#include "../Bus/RAD/c128_refresh.h"
#include "../Bus/RAD/cpu_hijack.h"
#include "../SuperCPU/supercpu.h"

// Defined in main.cpp, which owns the FAT filesystem object.
extern "C" void radUnmountFileSystem();

// Diagnostics have to run with interrupts ENABLED to reach the screen: Circle
// cannot complete a logger write while they are masked. The frame hook runs
// between frames with no bus access in flight, so briefly unmasking there
// cannot disturb a timed bus window.
class CScopedLoggingIRQs
{
public:
	CScopedLoggingIRQs()  { EnableIRQs(); }
	~CScopedLoggingIRQs() { DisableIRQs(); }
};

// Bus waits that hit their ceiling instead of seeing the signal they wanted.
// Defined in Bus/RAD/gpio_defs.cpp; declared here rather than including
// lowlevel_dma.h, which needs the whole GPIO header stack ahead of it.
extern volatile u32 radBAWaitTimeouts;
extern volatile u32 radPHIWaitTimeouts;

// How many times those ceilings were NEWLY reached while the emulator was
// running, and the first pair of values that did it. Latched here instead of
// logged there: see the note in scpuCheckButton for why a console write cannot
// happen inside the run loop. Printed on the guest-stopped diagnostic line,
// beside the stop it would otherwise have caused.
static u32 s_BusWaitTimeoutReports = 0;
static u32 s_BusWaitTimeoutFirstBA = 0;
static u32 s_BusWaitTimeoutFirstPHI = 0;

// Heartbeat: proves the frame loop is still running, and reports what the
// emulated machine is doing while it does so.
static u32 s_HeartbeatFrames = 0;
static u64 s_MirrorHaltStartCycles = 0;
static u32 s_BusHaltFrames = 0;
static u32 s_HeartbeatPCLow = 0xFFFFFFFF;
static u32 s_HeartbeatPCHigh = 0;
static u32 s_HeartbeatStalledRuns = 0;
static bool s_HeartbeatDumped = false;
static u32 s_DisplayScrubPhaseGeneration = 0;
static u32 s_ResyncDiagnosticGeneration = 0;
// Native-C128 bring-up may execute garbage continuously rather than settling
// into a tight loop, so the ordinary stall detector never fires. Keep a small
// sampled PC history and force one bounded capture early in startup.
static u32 s_C128DiagnosticFrames = 0;
static u32 s_C128PCTrace[ 8 ] = {};
static u32 s_C128PCTracePos = 0;

static bool radBusButtonPressed();
static bool radBusHardwareResetPressed();
static bool radBusRAMUnderIOReady();
static bool radBusPrepareRAMUnderIOAccess();
static void radBusSampleInterrupts( bool &irq, bool &nmi );
static bool radBusTrafficHalted();
static SCPURuntimeResult scpuCurrentRuntimeResult( CSuperCPU *scpu );
static void scpuBeginRuntimeDiagnostic( CSuperCPU *scpu );
static bool scpuRuntimeDiagnosticActive();
static bool scpuSaveRuntimeBusDiag( const SCPURuntimeResult &runtime );
static void radBusHaltAndProbe( u32 &eligiblePhases, u32 &addrChanges,
	                           u32 &distinctAddrs, u32 &sequentialAddrs,
	                           u32 &writePhases, u8 &firstAddr, u8 &lastAddr );

// Bare metal: no snprintf. Keep every freeze-dump append explicitly bounded:
// a CIA timing row can be about 240 characters, substantially more than the
// old 128-byte scratch buffer allowed.
static const u32 SCPU_DUMP_LINE_SIZE = 256;
// Spelled with the drive, like SCPU_BUS_DIAG_FILE. writeFile() passes DRIVE
// only to f_mount and this string verbatim to f_open, so unprefixed it reaches
// the card only because FatFs here has SD as volume 0 with FF_FS_RPATH 0 --
// a coincidence of the configuration, and this file is the only record a VSP
// run leaves behind.
static const char *SCPU_VSP_TRACE_FILE = "SD:SCPU/vsptrace-e18.txt";
static C64VideoStandard s_DetectedVideo = VIDEO_PAL;
static bool s_DetectedVideoValid = false;

static void scpuLineStart( char *dst, u32 capacity, u32 &length )
{
	length = 0;
	if ( capacity ) dst[ 0 ] = 0;
}

static bool scpuLineChar( char *dst, u32 capacity, u32 &length, char c )
{
	if ( !capacity || length + 1 >= capacity )
		return false;
	dst[ length++ ] = c;
	dst[ length ] = 0;
	return true;
}

static bool scpuLineHexByte( char *dst, u32 capacity, u32 &length, u8 v )
{
	static const char *h = "0123456789ABCDEF";
	return scpuLineChar( dst, capacity, length, h[ v >> 4 ] )
	    && scpuLineChar( dst, capacity, length, h[ v & 15 ] );
}

static bool scpuLineText( char *dst, u32 capacity, u32 &length, const char *s )
{
	while ( s && *s )
		if ( !scpuLineChar( dst, capacity, length, *s++ ) ) return false;
	return true;
}

static bool scpuLineHexU32( char *dst, u32 capacity, u32 &length, u32 v )
{
	static const char *h = "0123456789ABCDEF";
	for ( s32 shift = 28; shift >= 0; shift -= 4 )
		if ( !scpuLineChar( dst, capacity, length,
		                     h[ ( v >> shift ) & 15 ] ) ) return false;
	return true;
}

static bool scpuLineDecimal( char *dst, u32 capacity, u32 &length, u32 v )
{
	char reversed[ 10 ];
	u32 digits = 0;
	do
	{
		reversed[ digits++ ] = (char)( '0' + v % 10 );
		v /= 10;
	} while ( v && digits < sizeof reversed );

	while ( digits )
		if ( !scpuLineChar( dst, capacity, length, reversed[ --digits ] ) )
			return false;
	return true;
}

static bool scpuLineDecimal64( char *dst, u32 capacity, u32 &length, u64 v )
{
	char reversed[ 20 ];
	u32 digits = 0;
	do
	{
		reversed[ digits++ ] = (char)( '0' + v % 10 );
		v /= 10;
	} while ( v && digits < sizeof reversed );

	while ( digits )
		if ( !scpuLineChar( dst, capacity, length, reversed[ --digits ] ) )
			return false;
	return true;
}
static void scpuWaitForButtonThenReboot();
static CLogger *s_Logger;
static CHDMIDisplay *s_HDMIDisplay = 0;
static bool s_RebootRequested = false;

// Frame hook: reboot the Pi when the RAD's button is pressed, so the card can
// be swapped and retried without power-cycling anything.
// Runs once per emulated frame.
//
//   LEFT button  (GPIO 3)      -> reboot the Pi, so a freshly written card is
//                                 picked up without a power cycle
//   RIGHT button (hardware)    -> wired straight to the C64's /RESET line and
//                                 invisible to the Pi as a button, but we can
//                                 see the line go low. Treat it as a reset of
//                                 the emulated machine, which is what it would
//                                 do on a real SuperCPU.
// Sampled every ~50 frames, so the freeze dump can report the CIA state as it
// was BEFORE the reset button was pressed. The button is wired to the C64's
// physical /RESET line, so by the time the dump reads the CIAs they have
// already been reset -- DDR reads $00 regardless of what it was, which
// contaminated several rounds of serial forensics before this existed.
static u8  s_PreResetDD00 = 0, s_PreResetDD02 = 0;
static u32 s_PreResetSampleFrame = 0;
static bool s_PreResetSampleValid = false;
static const u32 SCPU_RESET_RELEASE_STABLE_SAMPLES = 100000;

// A reset press also resets the physical VIC and CIAs.  Keep the time between
// noticing /RESET and waiting for its release as short as possible: emitting a
// multi-line logger dump here used to leave the physical machine released
// while the emulated CPU was still stopped if the user let go during logging.
// Everything needed by the dump is therefore copied into this fixed-size
// snapshot first, and the frame hook emits it later in an IEC-idle frame.
struct SCPUResetDiagnostic
{
	bool pending;
	bool haveCPU;
	bool haveCore;
	bool haveCode;
	bool preResetSampleValid;
	u32 pc;
	u16 sp;
	u64 cycles;
	u8  p;
	u8  e;
	u32 irqsTaken;
	u32 nmisTaken;
	bool liveIRQ;
	bool liveNMI;
	// Interrupt-entry ring copied out of the core, plus the guest's live
	// interrupt vectors.  Mario bypasses the KERNAL ($0314 = $0000) and
	// re-points $FFFE between $0813, $0846 and $0898, so which one is loaded at
	// death says which rung the chain stopped on without any per-write
	// instrumentation.  These are RAM reads, not bus reads: the value is the
	// byte UNDER the KERNAL, which is the one the 6510 fetches only while $01
	// has the KERNAL banked out -- true for Mario, not in general, so do not
	// read this field as "the vector" on another title.
	CW65C816::W65InterruptEntry intRing[ CW65C816::W65_INT_RING ];
	u32 intRingHead;
	u32 intRingFrozen;
	u32 intRingFrozenCycles;
	u32 stackAnomalies;
	// The progress detector's SUSPECT copy, taken ~0.35s into the stall, plus
	// its counters.  Two rings and not one because the transition into a hang
	// and its steady state are different evidence and the live ring can only
	// hold one of them: 64 entries is about five frames.
	CW65C816::W65InterruptEntry intRingSuspect[ CW65C816::W65_INT_RING ];
	u32 intRingSuspectHead;
	u32 progEpochs;
	u32 progStallEpochsMax;
	u32 progSuspects;
	u32 progConfirmed;
	u32 progSuspectCycles;
	u32 progConfirmCycles;
	u32 progCellCount;
	u16 progCells[ CW65C816::W65_PROG_CELL_REPORT ];
	u16 progPC;
	// Watched-RAM write forensics, newest last.
	CC64Memory::WatchedWrite watchRing[ CC64Memory::SCPU_WATCH_RING ];
	u32 watchPos;
	u32 watchWrites;
	u32 watchChanges;
	u32 watchAddr[ 2 ];
	// $9F88 read two ways.  m_RAM is the byte in DRAM; the memory map is what
	// the guest's CMP at $A765 actually fetches.  If those two disagree the
	// hang is case (c) -- written correctly, read from somewhere else -- and
	// this pair proves it without any further instrumentation.
	u8  phaseViaRAM;
	u8  phaseViaMap;
	u8  phaseShadow;	// $9F87, which $09B3 writes with the same Y
	// What the guest last saw of the raster, and what it is waiting for.
	u8  d012Reads[ CC64Memory::SCPU_D012_RING ];
	u32 d012ReadCyc[ CC64Memory::SCPU_D012_RING ];
	u32 d012ReadPos;
	u32 d012ReadCount;
	u16 rasterCompare;
	// RST8 forensics. The whole second hang was one $D011 read merging a live
	// raster bit into the control latch, so these say outright which arm of
	// d011RasterHighBit() ran instead of leaving it to be inferred.
	u32 rst8Samples;
	u32 rst8Rejected;
	u8  rst8LastLow;
	u8  rst8LastRejLow;
	u32 rst8Writes;
	u32 compareImpossible;
	u32 anchorRefused;
	// The interrupt-control write ring, live and latched. This is the one
	// instrument that records what ARMED a compare rather than what the
	// handler did afterwards; see VICIntCtlWrite in c64_memory.h.
	VICIntCtlWrite ictl[ SCPU_ICTL_RING ];
	VICIntCtlWrite ictlFrozen[ SCPU_ICTL_RING ];
	u32 ictlHead;
	u32 ictlFrozenHead;
	u32 ictlFrozenReason;
	u32 ictlFrozenCycles;
	u32 d019Writes;
	u32 d019AckHeld;
	u32 d019AckHeldLate;
	u8  vectors[ 6 ];	// $FFFA..$FFFF
	u8  softVectors[ 2 ];	// $0314..$0315
	// CIA2 timer-NMI model state at snapshot time.
	u32 nmiSynth;
	u8  nmiHoldActive;
	u32 nmiRescales;
	u32 nmiTraceArm;
	u32 nmiTraceLatch;
	u32 nmiTraceExpose;
	u32 nmiTraceTake;
	u8  nmiOwned;
	u8  nmiMask;
	u32 nmiFactor;
	u8  nmiDeadlineArmed;
	u8 stack[ 16 ];
	u8 code[ 16 ];
	u32 cia[ 64 ];
	u32 ciaIntervalEntry[ 16 ];
	u32 ciaIntervalUS[ 16 ];
	u32 io[ 64 ];
	u8 preResetDD00;
	u8 preResetDD02;
	u8 vic11;
	u8 vic15;
	u8 vic16;
	u8 vic18;
	u8 vic20;
	u8 vic21;
	u32 vicScreenBase;
	u32 vicBitmapBase;
	u8 mirrorMode;
	u32 mirrorPending;
	u8 mirrorScreenPending;
	u8 mirrorBitmapPending;
	u64 mirrorAccepted;
	u64 mirrorSkipped;
	u64 mirrorFlushed;
	u64 mirrorEntriesExamined;
	u64 mirrorZeroProgressScans;
	u64 mirrorNoProgressCacheHits;
	u8 scrubPhaseOn;
	u64 scrubSampledText;
	u64 scrubSampledBitmap;
	u64 scrubMismatchText;
	u64 scrubMismatchBitmap;
	u64 scrubRepaired;
	VSPTimingEvent vsp[ SCPU_VSP_TRACE_SIZE ];
	u32 vspCount;
	u32 vspTotal;
	u32 vspOverwritten;
	u8 vspGamePhase;
	u8 vspVideo;
	u16 vspCyclesPerLine;
	u16 vspRasterLines;
	bool vspVideoValid;
	bool vspValid;
	SCPUVSPMatrixDiagnostic vspMatrix;
	SCPURuntimeResult runtime;
};

static SCPUResetDiagnostic s_ResetDiagnostic = {};
static u32 s_ResetDiagnosticQuietFrames = 0;
static u32 s_HardwareResetGeneration = 0;

static void scpuSnapshotResetDiagnostic( CSuperCPU *scpu )
{
	SCPUResetDiagnostic &d = s_ResetDiagnostic;
	d.pending = false;
	d.haveCPU = scpu && scpu->cpu();
	d.haveCore = false;
	d.haveCode = false;
	d.preResetSampleValid = s_PreResetSampleValid;
	d.preResetDD00 = s_PreResetDD00;
	d.preResetDD02 = s_PreResetDD02;
	d.runtime = scpuCurrentRuntimeResult( scpu );
	d.vspCount = d.vspTotal = d.vspOverwritten = 0;
	d.vspGamePhase = 0;
	d.vspVideo = (u8)s_DetectedVideo;
	d.vspCyclesPerLine = (u16)c64CyclesPerLine( s_DetectedVideo );
	d.vspRasterLines = (u16)c64RasterLines( s_DetectedVideo );
	d.vspVideoValid = s_DetectedVideoValid;
	d.vspValid = false;
	scpuVSPMatrixDiagnostic( d.vspMatrix );

	if ( !d.haveCPU )
	{
		d.pending = true;
		s_ResetDiagnosticQuietFrames = 0;
		return;
	}

	d.pc = (u32)scpu->cpu()->pc();
	d.sp = scpu->cpu()->stackPointer();
	d.cycles = scpu->cpu()->cycles();

	if ( CW65C816 *core = scpu->core65816() )
	{
		d.haveCore = true;
		d.p = core->m_P;
		d.e = core->m_E ? 1 : 0;
		d.irqsTaken = core->m_IRQsTaken;
		d.nmisTaken = core->m_NMIsTaken;
		d.intRingHead = core->m_IntRingHead;
		d.intRingFrozen = core->m_IntRingFrozen;
		d.intRingFrozenCycles = core->m_IntRingFrozenCycles;
		d.stackAnomalies = core->m_StackAnomalies;
		for ( u32 i = 0; i < CW65C816::W65_INT_RING; i++ )
			d.intRing[ i ] = core->m_IntRing[ i ];
		for ( u32 i = 0; i < CW65C816::W65_INT_RING; i++ )
			d.intRingSuspect[ i ] = core->m_IntRingSuspect[ i ];
		d.intRingSuspectHead = core->m_IntRingSuspectHead;
		d.progEpochs = core->m_ProgEpochs;
		d.progStallEpochsMax = core->m_ProgStallEpochsMax;
		d.progSuspects = core->m_ProgSuspects;
		d.progConfirmed = core->m_ProgConfirmed;
		d.progSuspectCycles = core->m_ProgSuspectCycles;
		d.progConfirmCycles = core->m_ProgConfirmCycles;
		d.progCellCount = core->m_ProgCellCount;
		d.progPC = core->m_ProgPC;
		for ( u32 i = 0; i < CW65C816::W65_PROG_CELL_REPORT; i++ )
			d.progCells[ i ] = core->m_ProgCells[ i ];
		for ( u32 i = 0; i < 6; i++ )
			d.vectors[ i ] = scpu->memory().m_RAM[ 0xFFFA + i ];
		d.softVectors[ 0 ] = scpu->memory().m_RAM[ 0x0314 ];
		d.softVectors[ 1 ] = scpu->memory().m_RAM[ 0x0315 ];
		radBusSampleInterrupts( d.liveIRQ, d.liveNMI );
	}

	{
		CC64Memory &m = scpu->memory();
		d.vspCount = m.copyVSPTrace( d.vsp, SCPU_VSP_TRACE_SIZE,
		                              d.vspTotal, d.vspOverwritten );
		d.vspGamePhase = m.m_RAM[ 0x9F88 ];
		d.vspValid = true;
		for ( u32 i = 0; i < CC64Memory::SCPU_WATCH_RING; i++ )
			d.watchRing[ i ] = m.m_WatchRing[ i ];
		d.watchPos = m.m_WatchPos;
		d.watchWrites = m.m_WatchWrites;
		d.watchChanges = m.m_WatchChanges;
		d.watchAddr[ 0 ] = cfgRAMWatch;
		d.watchAddr[ 1 ] = cfgRAMWatch2;
		for ( u32 i = 0; i < CC64Memory::SCPU_D012_RING; i++ )
		{
			d.d012Reads[ i ] = m.m_D012Reads[ i ];
			d.d012ReadCyc[ i ] = m.m_D012ReadCyc[ i ];
		}
		d.d012ReadPos = m.m_D012ReadPos;
		d.d012ReadCount = m.m_D012ReadCount;
		d.rasterCompare = m.rasterCompare();
		d.rst8Samples = m.m_D011RST8Samples;
		d.rst8Rejected = m.m_D011RST8Rejected;
		d.rst8LastLow = m.m_D011RST8LastLow;
		d.rst8LastRejLow = m.m_D011RST8LastRejLow;
		d.rst8Writes = m.m_D011RST8Writes;
		d.compareImpossible = m.m_RasterCompareImpossible;
		d.anchorRefused = m.m_RasterAnchorRefused;
		for ( u32 i = 0; i < SCPU_ICTL_RING; i++ )
		{
			d.ictl[ i ] = m.m_ICtlRing[ i ];
			d.ictlFrozen[ i ] = m.m_ICtlFrozen[ i ];
		}
		d.ictlHead = m.m_ICtlHead;
		d.ictlFrozenHead = m.m_ICtlFrozenHead;
		d.ictlFrozenReason = m.m_ICtlFrozenReason;
		d.ictlFrozenCycles = m.m_ICtlFrozenCycles;
		d.d019Writes = m.m_D019Writes;
		d.d019AckHeld = m.m_D019AckHeldIRQ;
		d.d019AckHeldLate = m.m_D019AckHeldIRQLate;
		// $9F88 through the DRAM array and through the map the guest reads.
		// Both are RAM, so neither touches a chip register and both are safe
		// while /RESET is asserted.
		d.phaseViaRAM = m.m_RAM[ 0x9F88 ];
		d.phaseShadow = m.m_RAM[ 0x9F87 ];
		d.phaseViaMap = (u8)scpu->memoryMap().read8( 0x9F88 );
		d.nmiSynth = (u32)m.m_SynthNMIsDelivered;
		d.nmiHoldActive = m.iecThrottleActive() ? 1 : 0;
		d.nmiRescales = (u32)m.m_CIA2Rescales;
		d.nmiOwned = m.m_CIA2NMIOwned ? 1 : 0;
		d.nmiMask = m.m_CIA2ICRMask;
		d.nmiFactor = (u32)m.m_CIA2FactorInUse;
		d.nmiDeadlineArmed = m.m_CIA2NMIDeadline ? 1 : 0;
		// Lifecycle deltas of the LAST timer NMI, all relative to the arm:
		// where the time went between the deadline and the core's dispatch.
		// Microseconds since the arm, per stage, for the CURRENT arm only;
		// 0xFFFFFFFF means that stage never happened for it.
		d.nmiTraceLatch  = m.nmiTraceStageUS( m.m_NMITraceLatchAt,  m.m_NMITraceLatchGen );
		d.nmiTraceExpose = m.nmiTraceStageUS( m.m_NMITraceExposeAt, m.m_NMITraceExposeGen );
		d.nmiTraceTake   = m.nmiTraceStageUS( m.m_NMITraceTakeAt,   m.m_NMITraceTakeGen );
		d.nmiTraceArm = (u32)m.m_NMITraceGen;
		d.vic11 = m.m_VICRegShadow[ 0x11 ];
		d.vic15 = m.m_VICRegShadow[ 0x15 ];
		d.vic16 = m.m_VICRegShadow[ 0x16 ];
		d.vic18 = m.m_VICRegShadow[ 0x18 ];
		d.vic20 = m.m_VICRegShadow[ 0x20 ];
		d.vic21 = m.m_VICRegShadow[ 0x21 ];
		d.vicScreenBase = m.activeScreenBase();
		d.vicBitmapBase = m.activeBitmapBase();
	}
	{
		CWriteBuffer &w = scpu->writeBuffer();
		d.mirrorMode = (u8)w.optMode();
		d.mirrorPending = w.pending();
		d.mirrorScreenPending = d.vicScreenBase < 0x10000
		                      && w.hasPendingInRange( d.vicScreenBase, 1000 );
		d.mirrorBitmapPending = d.vicBitmapBase < 0x10000
		                      && w.hasPendingInRange( d.vicBitmapBase, 8000 );
		d.mirrorAccepted = w.m_WritesAccepted;
		// Both halves of "the policy discarded this write": the ones the sink
		// examined, and the ones CC64Memory's mirror floor rejected before the
		// call. Reporting only the first would have hidden ~96% of them -- see
		// IMirrorSink::m_MirrorFloor.
		d.mirrorSkipped = w.m_WritesSkipped + w.m_MirrorFloorSkipped;
		d.mirrorFlushed = w.m_BytesFlushed;
		d.mirrorEntriesExamined = w.m_PolicyEntriesExamined;
		d.mirrorZeroProgressScans = w.m_PolicyZeroProgressScans;
		d.mirrorNoProgressCacheHits = w.m_PolicyNoProgressCacheHits;
		d.scrubPhaseOn = scpu->displayScrubPhaseOn() ? 1 : 0;
		d.scrubSampledText = w.m_DisplayScrubSampled[ 0 ];
		d.scrubSampledBitmap = w.m_DisplayScrubSampled[ 1 ];
		d.scrubMismatchText = w.m_DisplayScrubMismatches[ 0 ];
		d.scrubMismatchBitmap = w.m_DisplayScrubMismatches[ 1 ];
		d.scrubRepaired = w.m_DisplayScrubBytes;
	}

	for ( u32 i = 0; i < 16; i++ )
	{
		// The 6502 and emulation-mode 65816 wrap S inside page $01. In
		// native mode S is a full 16-bit bank-0 address.
		const u16 stackAddr = ( d.haveCore && !d.e )
		                    ? (u16)( d.sp + i + 1 )
		                    : (u16)( 0x0100 | ( ( d.sp + i + 1 ) & 0xFF ) );
		d.stack[ i ] = scpu->memory().m_RAM[ stackAddr ];
	}

	// This deliberately excludes bank 0's I/O page. No diagnostic snapshot may
	// add a physical CIA access while /RESET is asserted. Check the whole window,
	// not just PC: a PC near $CFFF can make PC+11 cross into $D000. Code in the
	// 65816's private banks is safe and retains its full 24-bit address.
	bool codeWindowSafe = true;
	for ( int i = -4; i < 12; i++ )
	{
		const u32 addr = ( d.pc + i ) & SCPU_ADDR_MASK;
		if ( ( addr & 0xFF0000 ) == 0 && ( addr & 0xF000 ) == 0xD000 )
		{
			codeWindowSafe = false;
			break;
		}
	}
	if ( codeWindowSafe )
	{
		d.haveCode = true;
		for ( int i = -4; i < 12; i++ )
			d.code[ i + 4 ] = scpu->memoryMap().read8(
				( d.pc + i ) & SCPU_ADDR_MASK );
	}

	for ( u32 i = 0; i < 64; i++ )
	{
		d.cia[ i ] = scpu->memory().m_CIALog[
			( scpu->memory().m_CIALogPos + i ) & 63 ];
		d.io[ i ] = scpu->memory().m_IOLog[
			( scpu->memory().m_IOLogPos + i ) & 63 ];
	}

	for ( u32 i = 0; i < 16; i++ )
	{
		const u32 age = i + 1;
		const u32 idx  = ( scpu->memory().m_CIALogPos + 64 - age ) & 63;
		const u32 prev = ( scpu->memory().m_CIALogPos + 64 - age - 1 ) & 63;
		d.ciaIntervalEntry[ i ] = scpu->memory().m_CIALog[ idx ];
		u32 dt = ( scpu->memory().m_CIALogCyc[ idx ]
		         - scpu->memory().m_CIALogCyc[ prev ] )
		       / ( SCPU_ARM_CLOCK_HZ / 1000000u );
		if ( dt > 99999 ) dt = 99999;
		d.ciaIntervalUS[ i ] = dt;
	}

	// Publish the snapshot only after every field is complete.
	d.pending = true;
	s_ResetDiagnosticQuietFrames = 0;
}

static void scpuEmitResetDiagnostic()
{
	SCPUResetDiagnostic &d = s_ResetDiagnostic;
	if ( !d.pending || !s_Logger )
		return;

	// Clear first so this is one bounded emission even if logging itself is
	// interrupted. A later reset press will simply publish a fresh snapshot.
	d.pending = false;
	s_ResetDiagnosticQuietFrames = 0;

	if ( !d.haveCPU )
	{
		s_Logger->Write( "SCPU", LogNotice,
		                 "reset pressed: CPU diagnostic unavailable" );
		return;
	}

	s_Logger->Write( "SCPU", LogNotice,
	               "reset pressed: PC=$%06X S=$%04X cycles=%llu",
	               (unsigned)d.pc, (unsigned)d.sp,
	               (unsigned long long)d.cycles );

	if ( d.haveCore )
		s_Logger->Write( "SCPU", LogNotice,
		               "  P=$%02X (I=%u) E=%u  irqsTaken=%u nmisTaken=%u  line: IRQ=%u NMI=%u",
		               d.p, ( d.p & W65_I ) ? 1 : 0, d.e,
		               (unsigned)d.irqsTaken, (unsigned)d.nmisTaken,
		               d.liveIRQ ? 1 : 0, d.liveNMI ? 1 : 0 );

	// The CIA2 timer-NMI model's view at snapshot time -- which mode
	// delivered the NMIs, and how the deadline units moved with speed.
	s_Logger->Write( "SCPU", LogNotice,
	               "  nmi model: synth=%u hold=%u rescales=%u owned=%u mask=$%02X factor=%u deadline=%s",
	               (unsigned)d.nmiSynth, (unsigned)d.nmiHoldActive,
	               (unsigned)d.nmiRescales, (unsigned)d.nmiOwned,
	               d.nmiMask, (unsigned)d.nmiFactor,
	               d.nmiDeadlineArmed ? "armed" : "none" );
	{
		// Absent stages print as "-", so an armed-but-never-fired timer (or an
		// NMI held back by the native-mode deferral) reads as exactly that
		// instead of as a wrapped subtraction.
		char lat[ 16 ], exp[ 16 ], tak[ 16 ];
		const u32 stages[ 3 ] = { d.nmiTraceLatch, d.nmiTraceExpose, d.nmiTraceTake };
		char *outs[ 3 ] = { lat, exp, tak };
		for ( u32 i = 0; i < 3; i++ )
		{
			u32 n = 0;
			if ( stages[ i ] == 0xFFFFFFFF )
			{
				outs[ i ][ 0 ] = '-'; outs[ i ][ 1 ] = 0;
			} else {
				scpuLineStart( outs[ i ], 16, n );
				scpuLineChar( outs[ i ], 16, n, '+' );
				scpuLineDecimal( outs[ i ], 16, n, stages[ i ] );
			}
		}
		s_Logger->Write( "SCPU", LogNotice,
			               "  nmi trace: arm#%u latch=%sus expose=%sus take=%sus",
			               (unsigned)d.nmiTraceArm, lat, exp, tak );
	}

	char line[ SCPU_DUMP_LINE_SIZE ];
	u32 n = 0;
	scpuLineStart( line, sizeof line, n );
	for ( u32 i = 0; i < 16; i++ )
	{
		scpuLineHexByte( line, sizeof line, n, d.stack[ i ] );
		scpuLineChar( line, sizeof line, n, ' ' );
	}
	s_Logger->Write( "SCPU", LogNotice, "  stack: %s", line );

	if ( d.haveCode )
	{
		scpuLineStart( line, sizeof line, n );
		for ( u32 i = 0; i < 16; i++ )
		{
			scpuLineHexByte( line, sizeof line, n, d.code[ i ] );
			scpuLineChar( line, sizeof line, n, ' ' );
		}
		s_Logger->Write( "SCPU", LogNotice, "  code@PC-4: %s", line );
	}

	for ( u32 row = 0; row < 4; row++ )
	{
		scpuLineStart( line, sizeof line, n );
		for ( u32 i = 0; i < 16; i++ )
		{
			const u32 e = d.cia[ row * 16 + i ];
			if ( !scpuLineChar( line, sizeof line, n, ( e >> 24 ) ? 'w' : 'r' )
			     || !scpuLineHexByte( line, sizeof line, n, (u8)( ( e >> 8 ) & 0xFF ) )
			     || !scpuLineHexByte( line, sizeof line, n, (u8)( e & 0xFF ) )
			     || !scpuLineChar( line, sizeof line, n, '=' )
			     || !scpuLineHexByte( line, sizeof line, n, (u8)( ( e >> 16 ) & 0xFF ) )
			     || !scpuLineChar( line, sizeof line, n, ' ' ) )
				break;
		}
		s_Logger->Write( "SCPU", LogNotice, "  cia%u: %s", row, line );
	}

	scpuLineStart( line, sizeof line, n );
	for ( u32 i = 0; i < 16; i++ )
	{
		const u32 e = d.ciaIntervalEntry[ i ];
		if ( !scpuLineChar( line, sizeof line, n, ( e >> 24 ) ? 'w' : 'r' )
		     || !scpuLineHexByte( line, sizeof line, n, (u8)( ( e >> 8 ) & 0xFF ) )
		     || !scpuLineHexByte( line, sizeof line, n, (u8)( e & 0xFF ) )
		     || !scpuLineChar( line, sizeof line, n, '=' )
		     || !scpuLineHexByte( line, sizeof line, n, (u8)( ( e >> 16 ) & 0xFF ) )
		     || !scpuLineChar( line, sizeof line, n, '+' )
		     || !scpuLineDecimal( line, sizeof line, n, d.ciaIntervalUS[ i ] )
		     || !scpuLineChar( line, sizeof line, n, ' ' ) )
			break;
	}
	s_Logger->Write( "SCPU", LogNotice, "  ciaT: %s", line );

	if ( d.preResetSampleValid )
		s_Logger->Write( "SCPU", LogNotice,
		               "  pre-reset (last IEC-idle sample): $DD00=$%02X $DD02=$%02X",
		               d.preResetDD00, d.preResetDD02 );
	else
		s_Logger->Write( "SCPU", LogNotice,
		               "  pre-reset: no IEC-idle CIA sample available" );
	s_Logger->Write( "SCPU", LogNotice,
	               "  live CIA state not sampled while physical /RESET was asserted" );
	s_Logger->Write( "SCPU", LogNotice,
	               "  vic shadow: D011=$%02X D016=$%02X D018=$%02X D015=$%02X D020/21=$%02X/$%02X screen=$%04X bitmap=$%04X",
	               d.vic11, d.vic16, d.vic18, d.vic15, d.vic20, d.vic21,
	               (unsigned)( d.vicScreenBase & 0xFFFF ),
	               (unsigned)( d.vicBitmapBase & 0xFFFF ) );
	if ( d.vspValid )
		s_Logger->Write( "SCPU", LogNotice,
		               "  vsp trace: retained=%u total=%u overwritten=%u signature D016=$%02X x=%u D018=$%02X phase=$%02X",
		               (unsigned)d.vspCount, (unsigned)d.vspTotal,
		               (unsigned)d.vspOverwritten, d.vic16,
		               (unsigned)( d.vic16 & 7 ), d.vic18,
		               d.vspGamePhase );
	for ( u32 i = 0; i < SCPU_VSP_MATRIX_CAPTURE_SLOTS; i++ )
	{
		const SCPUVSPMatrixCapture &capture = d.vspMatrix.capture[ i ];
		if ( capture.state != 2 ) continue;
		s_Logger->Write( "SCPU", LogNotice,
		               "  vsp matrix%u%s: mismatch=%u direct=%u/1024 shift=%d score=%u/%u pending=%u raster=%u-%u safe=%u",
		               (unsigned)i, capture.kind ? "-stall" : "",
		               (unsigned)capture.mismatchCount,
		               (unsigned)capture.directMatches,
		               (int)capture.bestShift,
		               (unsigned)capture.bestShiftMatches,
		               (unsigned)capture.bestShiftCompared,
		               (unsigned)capture.pendingAtCapture,
		               (unsigned)capture.rasterBefore,
		               (unsigned)capture.rasterAfter,
		               (unsigned)capture.safeWindow );
	}
	s_Logger->Write( "SCPU", LogNotice,
	               "  mirror: mode=%u pending=%u screen/bitmap=%u/%u accepted=%llu skipped=%llu flushed=%llu",
	               (unsigned)d.mirrorMode, (unsigned)d.mirrorPending,
	               (unsigned)d.mirrorScreenPending,
	               (unsigned)d.mirrorBitmapPending,
	               (unsigned long long)d.mirrorAccepted,
	               (unsigned long long)d.mirrorSkipped,
	               (unsigned long long)d.mirrorFlushed );
	s_Logger->Write( "SCPU", LogNotice,
	               "  mirror scan: examined=%llu zero=%llu cached=%llu",
	               (unsigned long long)d.mirrorEntriesExamined,
	               (unsigned long long)d.mirrorZeroProgressScans,
	               (unsigned long long)d.mirrorNoProgressCacheHits );
	s_Logger->Write( "SCPU", LogNotice,
	               "  display scrub: phase=%s repaired=%llu (physical sampling disabled)",
	               d.scrubPhaseOn ? "ON" : "OFF",
	               (unsigned long long)d.scrubRepaired );
	if ( s_HDMIDisplay )
		s_Logger->Write( "SCPU", LogNotice,
		               "  hdmi renderer: max-band=%uus missed=%u raster-resync=%u lost/reset/fallback=%u/%u/%u",
		               (unsigned)s_HDMIDisplay->maxBandUS(),
		               (unsigned)s_HDMIDisplay->missedBandDeadlines(),
		               (unsigned)s_HDMIDisplay->rasterResyncs(),
		               (unsigned)s_HDMIDisplay->rasterLostResyncs(),
		               (unsigned)s_HDMIDisplay->rasterResetResyncs(),
		               (unsigned)s_HDMIDisplay->rasterFallbacks() );

	// Both should read zero. Either one non-zero means a bus wait hit its
	// ceiling instead of seeing the signal it wanted -- which before those
	// ceilings existed was an unrecoverable hang, not a counter.
	s_Logger->Write( "SCPU", LogNotice,
	               "  bus wait timeouts: BA=%u PHI=%u",
	               (unsigned)radBAWaitTimeouts, (unsigned)radPHIWaitTimeouts );

	for ( u32 row = 0; row < 4; row++ )
	{
		scpuLineStart( line, sizeof line, n );
		for ( u32 i = 0; i < 16; i++ )
		{
			const u32 e = d.io[ row * 16 + i ];
			if ( !scpuLineChar( line, sizeof line, n,
			                    ( e & ( 1u << 25 ) ) ? 'W' : ( ( e >> 24 ) ? 'w' : 'r' ) )
			     || !scpuLineHexByte( line, sizeof line, n, (u8)( ( e >> 8 ) & 0xFF ) )
			     || !scpuLineHexByte( line, sizeof line, n, (u8)( e & 0xFF ) )
			     || !scpuLineChar( line, sizeof line, n, '=' )
			     || !scpuLineHexByte( line, sizeof line, n, (u8)( ( e >> 16 ) & 0xFF ) )
			     || !scpuLineChar( line, sizeof line, n, ' ' ) )
				break;
		}
		s_Logger->Write( "SCPU", LogNotice, "  io%u: %s", row, line );
	}
}

// Put the real C128's VIC into a predictable 40-column configuration without
// touching screen, character, or colour RAM.  An earlier diagnostic stamped a
// report into every possible screen matrix because the adapter's private MMU
// signals are unavailable to RAD.  That also overwrote character data and made
// the resulting video useless.  The border code below is map-independent, so
// preserving all display RAM is both safer and more informative.
static void scpuShowC128FreezeDiagnostic( CSuperCPU *scpu )
{
	if ( !scpu || !scpu->registers().scpu128Mode() || !scpu->hostBus() )
		return;

	IC64Bus *bus = scpu->hostBus();

	// Configuration zero is the captured native-C128 startup map: RAM bank 0,
	// I/O and system ROM visible. Put the VIC in its ordinary 40-column text
	// layout with screen RAM at $0400 and the character generator at $1000.
	bus->write( 0xFF00, 0x00 );
	u8 ddra = bus->read( 0xDD02 );
	u8 pra  = bus->read( 0xDD00 );
	bus->write( 0xDD02, (u8)( ddra | 0x03 ) );
	bus->write( 0xDD00, (u8)( pra  | 0x03 ) );
	bus->write( 0xD011, 0x1B );
	bus->write( 0xD016, 0x08 );
	bus->write( 0xD018, 0x14 );
	bus->write( 0xD020, 0x06 );
	bus->write( 0xD021, 0x06 );
}

static bool scpuC128DiagnosticWait( IC64Bus *bus, u32 hostCycles )
{
	const u64 until = bus->hostCycles() + hostCycles;
	while ( bus->hostCycles() < until )
	{
		if ( radBusButtonPressed() ) return false;
		asm volatile( "yield" );
	}
	return true;
}

// Screen RAM may be unreachable without the real SCPU128 adapter cable. As a
// map-independent fallback, continuously send the captured 24-bit PC through
// the VIC border: GREEN marker, then 24 bits MSB-first (WHITE=1, BLACK=0),
// with RED separators, followed by a YELLOW end marker.  The complete sequence
// is about 9.5 seconds so a short phone video is sufficient.
static void scpuHoldC128FreezeDiagnostic( CSuperCPU *scpu )
{
	IC64Bus *bus = scpu ? scpu->hostBus() : 0;
	if ( !bus ) return;
	const u32 oneSecond = bus->hostCyclesPerSec();
	const u32 bitTime = oneSecond / 5;       // 200 ms
	const u32 separatorTime = oneSecond / 10; // 100 ms
	const u32 pc = s_ResetDiagnostic.pc & 0xFFFFFF;

	while ( !radBusButtonPressed() )
	{
		bus->write( 0xD020, 0x05 );       // green: sequence begins
		if ( !scpuC128DiagnosticWait( bus, oneSecond ) ) break;
		bus->write( 0xD020, 0x02 );       // red: separator
		if ( !scpuC128DiagnosticWait( bus, bitTime ) ) break;
		for ( int bit = 23; bit >= 0; bit-- )
		{
			bus->write( 0xD020, ( pc & ( 1u << bit ) ) ? 0x01 : 0x00 );
			if ( !scpuC128DiagnosticWait( bus, bitTime ) ) return;
			bus->write( 0xD020, 0x02 );
			if ( !scpuC128DiagnosticWait( bus, separatorTime ) ) return;
		}
		bus->write( 0xD020, 0x07 );       // yellow: sequence ends
		if ( !scpuC128DiagnosticWait( bus, oneSecond ) ) break;
	}
}

static void scpuUpdateVideoSelection( CSuperCPU *scpu );

static bool scpuCheckButton( void *ctx )
{
	CSuperCPU *scpu = (CSuperCPU *)ctx;
	scpuUpdateVideoSelection( scpu );
	const bool hardwareResetPressed = radBusHardwareResetPressed();

	// Do not wait for a formal stall during initial native-C128 bring-up. A bad
	// MMU map commonly makes the processor roam through changing garbage, which
	// corrupts the display but never satisfies the narrow-PC-span detector.
	// Sample every ten frames and force the report at about 1.5 seconds.
	if ( scpu && scpu->cpu() && scpu->registers().scpu128Mode() )
	{
		if ( ( s_C128DiagnosticFrames % 10 ) == 0 )
			s_C128PCTrace[ s_C128PCTracePos++ & 7 ] = (u32)scpu->cpu()->pc();
		if ( ++s_C128DiagnosticFrames == 90 )
		{
			scpuSnapshotResetDiagnostic( scpu );
			{
				CScopedLoggingIRQs dumpIRQs;
				s_Logger->Write( "SCPU", LogNotice,
				               "C128 STARTUP CAPTURE at PC=$%06X (forced, no stall required)",
				               (unsigned)scpu->cpu()->pc() );
				scpuEmitResetDiagnostic();
			}
			scpuShowC128FreezeDiagnostic( scpu );
			scpuHoldC128FreezeDiagnostic( scpu );
			s_RebootRequested = true;
			return true;
		}
	}

	// Note the FIRST time a bus wait ever hits its ceiling. Before those
	// ceilings existed this was the freeze that took the whole firmware down
	// -- no frame hook, no button, power cycle only -- so it left no trace at
	// all. Knowing it happened, once, is the difference between a mystery and a
	// lead.
	//
	// IT NO LONGER WRITES A CONSOLE LINE, and that is a correctness change, not
	// a tidy-up. This runs in the frame hook, which means the emulated CPU is
	// STOPPED, and s_Logger's device is the Circle console: helpers.h's
	// STANDARD_SETUP falls back to m_Screen when no logdev is given, and
	// Firmware/RaspberryPi/cmdline.txt is "width=720 height=480". By run time the
	// console has long scrolled, so every line reaches CScreenDevice::Scroll,
	// which copies 720 x 464 x 4 = 1,336,320 bytes and -- with
	// SCREEN_DMA_BURST_LENGTH defaulting to 2 in circle/sysconfig.h -- then
	// BLOCKS core 0 in CDMAChannel::Wait(). This file already states the
	// magnitude twice from its own experience: "one heartbeat write costs
	// milliseconds" and "costs enough real time to stutter the display for a
	// visible instant", which is why HEARTBEAT defaults to 0.
	//
	// AND IT IS NOT HYPOTHETICAL ON THIS CARD. CHT.3 records this very line
	// firing twice over: "BUS WAIT TIMEOUT: BA=3034 PHI=0" in the K403 freeze
	// and "BA=3 PHI=0" over 38 otherwise quiet minutes. So the write does fire,
	// it fires rarely in a healthy run, and it fires from inside a guest stop --
	// which is the shape of the 2026-10-12 Mario displacement, one multi-
	// millisecond guest stop out of 63 captured events.
	//
	// NOTE THE SECOND MECHANISM THE SAME COUNTERS IMPLY, because it is at least
	// as likely as the scroll and this latch is how it gets read: each timeout
	// means one bus access spun the full RAD_BA_WAIT_LIMIT = 4000 C64 cycles
	// (lowlevel_dma.h:160) and then proceeded anyway. Three or four of those
	// inside one mirror drain is ~12ms on their own with no console involved --
	// which is what CHT.3 diagnosed as K403's "all=29669us/260" epoch. buswait=
	// and the existing guest-stop cause mask separate the two: a FRAMEEND-tagged
	// peak points at the frame hook, a BORDER/FLUSH-tagged one at the BA ceiling.
	//
	// NOTHING IS LOST, and that is the condition this edit had to meet. The
	// detection predicate is unchanged and still runs HERE, at the same point,
	// on the same two counters and behind the same IEC-quiet guard -- only the
	// cost of reporting it moves. The live totals are already printed verbatim
	// by the diagnostic dump ("  bus wait timeouts: BA=%u PHI=%u"), which runs
	// with the machine already stopped, and the first-change pair plus the
	// change count are latched below and printed on the guest-stopped line.
	if ( !scpu || !scpu->memory().iecBusActive() )
	{
		static u32 lastBA = 0, lastPHI = 0;
		if ( radBAWaitTimeouts != lastBA || radPHIWaitTimeouts != lastPHI )
		{
			if ( !s_BusWaitTimeoutReports )
			{
				s_BusWaitTimeoutFirstBA = radBAWaitTimeouts;
				s_BusWaitTimeoutFirstPHI = radPHIWaitTimeouts;
			}
			s_BusWaitTimeoutReports++;
			lastBA = radBAWaitTimeouts;
			lastPHI = radPHIWaitTimeouts;
		}
	}

	// Preserve the already-tracked CIA2 state for a later reset diagnostic.
	// Do not call read8() here: a guest-visible $DD00/$DD02 poll deliberately
	// re-arms the SuperCPU's 1MHz IEC hold, so the old diagnostic introduced a
	// periodic hitch into the machine it was observing.
	if ( scpu && !hardwareResetPressed
	     && ( ++s_PreResetSampleFrame % 50 ) == 0
	     && !scpu->memory().iecBusActive() )
	{
		s_PreResetSampleValid = scpu->memory().trackedCIA2Snapshot(
			s_PreResetDD00, s_PreResetDD02 );
	}

	if ( radBusButtonPressed() )
	{
		// The RAD's Pi-reboot button is the convenient way to return the card,
		// but it is not the Commodore /RESET line handled below. Persist the
		// same post-handoff record on both exits so the diagnostic cannot be
		// silently lost merely because the other physical button was used.
		if ( !s_RebootRequested )
		{
			// Snapshot the passive VSP sidecar as well.  This reads only Pi-owned
			// state; it does not add a physical VIC/CIA access before the reboot.
			scpuSnapshotResetDiagnostic( scpu );
			const SCPURuntimeResult runtime = s_ResetDiagnostic.runtime;
			CScopedLoggingIRQs saveIRQs;
			if ( !scpuSaveRuntimeBusDiag( runtime ) )
				s_Logger->Write( "SCPU", LogError,
							               "K362 could not save post-handoff runtime to %s",
				               SCPU_BUS_DIAG_FILE );
			else
				s_Logger->Write( "SCPU", LogNotice,
							               "K362 saved post-handoff runtime to %s",
				               SCPU_BUS_DIAG_FILE );
		}
		s_RebootRequested = true;
		return true;					// stop the run loop; caller reboots
	}

	if ( hardwareResetPressed )
	{
		// Take a bounded snapshot first; do not log or sample a physical CIA
		// until the machine has been released and normal execution has resumed.
		scpuSnapshotResetDiagnostic( scpu );

		// The physical VIC and CIAs remain in reset while the line is low, so the
		// emulator must not resume bus traffic until release. Require a long run
		// of agreeing high samples; any bounce resets the count. This remains
		// responsive to however long the user holds the button while preventing a
		// single press from becoming several emulated resets.
		u32 released = 0;
		while ( released < SCPU_RESET_RELEASE_STABLE_SAMPLES )
		{
			if ( radBusHardwareResetPressed() ) released = 0;
			else                                  released++;
		}

		// Persist the counters from the workload that just ended while the
		// emulated CPU is stopped and before reset clears its accounting. This
		// is deliberately after physical /RESET is released so FAT/SD logging
		// cannot extend the time the VIC and CIAs are held in reset.
		{
			CScopedLoggingIRQs saveIRQs;
			if ( !scpuSaveRuntimeBusDiag( s_ResetDiagnostic.runtime ) )
				s_Logger->Write( "SCPU", LogError,
							               "K362 could not save post-handoff runtime to %s",
				               SCPU_BUS_DIAG_FILE );
			else
				s_Logger->Write( "SCPU", LogNotice,
							               "K362 saved post-handoff runtime to %s",
				               SCPU_BUS_DIAG_FILE );
		}

		// RESET restores the physical 6510's processor port. If this run installed
		// the all-RAM under-I/O map, put it back before the emulated CPU resumes;
		// otherwise its next legitimate VIC/CIA access would hit DRAM instead.
		const bool restoreRAMUnderIO = radBusRAMUnderIOReady();
		if ( restoreRAMUnderIO && !radBusPrepareRAMUnderIOAccess() )
		{
			CScopedLoggingIRQs resetMapIRQs;
			s_Logger->Write( "SCPU", LogError,
			               "could not restore the C64 RAM-under-I/O bus map after RESET" );
			s_RebootRequested = true;
			return true;
		}

		if ( scpu ) scpu->reset();
		if ( scpuRuntimeDiagnosticActive() ) scpuBeginRuntimeDiagnostic( scpu );
		s_HardwareResetGeneration++;
		s_C128DiagnosticFrames = 0;
		s_C128PCTracePos = 0;
		for ( u32 i = 0; i < 8; i++ ) s_C128PCTrace[ i ] = 0;
		s_PreResetSampleFrame = 0;
		s_PreResetSampleValid = false;
		return false;
	}

	// --- diagnostic mirror halt ---------------------------------------------
	// MIRROR_HALT_AFTER_S: after N seconds, stop all mirror bus traffic so
	// the display shows whatever real DRAM holds, undisturbed. If corruption
	// SNAPS CLEAN when traffic stops, our ongoing writes were disturbing the
	// VIC's fetches; if it PERSISTS with the bus quiet, the DRAM content
	// itself is wrong -- our writes must be landing somewhere they should not.
	if ( cfgMirrorHaltAfterS > 0 && scpu && !scpu->mirrorHalted() )
	{
		IC64Bus *host = scpu->hostBus();
		const u64 now = host ? host->hostCycles() : 0;
		const u32 hz = host ? host->hostCyclesPerSec() : 0;
		if ( hz && !s_MirrorHaltStartCycles ) s_MirrorHaltStartCycles = now;
		if ( hz && now - s_MirrorHaltStartCycles
		          >= (u64)cfgMirrorHaltAfterS * hz )
		{
			CScopedLoggingIRQs irqs;
			s_Logger->Write( "SCPU", LogNotice,
				               "C128 K204 permutation: line=%u step=%u visited=%u opens-min/max=%u/%u",
			               (unsigned)c128RefreshLineCyclesUsed(),
			               (unsigned)c128RefreshPermutationStep(),
			               (unsigned)c128RefreshPhasesVisited(),
			               (unsigned)c128RefreshPhaseMinOpens(),
			               (unsigned)c128RefreshPhaseMaxOpens() );
			s_Logger->Write( "SCPU", LogNotice,
				               "C128 K204 overrun: count=%u total=%u max=%u worst-phase/max=%u/%u deadline/recovery/lines=%u/%u/%u",
			               (unsigned)c128RefreshLineOverrunCount(),
			               (unsigned)c128RefreshLineTotalOverrunCycles(),
			               (unsigned)c128RefreshLineMaxLateCycles(),
			               (unsigned)c128RefreshWorstOverrunPhase(),
			               (unsigned)c128RefreshWorstPhaseOverrunCycles(),
			               (unsigned)c128RefreshLineDeadlineMisses(),
			               (unsigned)c128RefreshLineDeadlineRecoveries(),
			               (unsigned)c128RefreshLineRecoveryLines() );

			// The historical pulse-scan section can consume the old diagnostic
			// formatter before its later topology report. Persist the live runtime
			// counters at the moment traffic stops, after the requested workload,
			// rather than asking the user to transcribe transient HDMI output.
			if ( !scpuSaveRuntimeBusDiag( scpuCurrentRuntimeResult( scpu ) ) )
			{
				s_Logger->Write( "SCPU", LogError,
				               "K204 could not save live counters to %s; retrying",
				               SCPU_BUS_DIAG_FILE );
				return false;
			}

			// Make the visible freeze the acknowledgement: f_close and unmount
			// have completed, so power can now be removed without losing the
			// counters that explain this run.
			s_Logger->Write( "SCPU", LogNotice,
			               "K204 saved live counters to %s", SCPU_BUS_DIAG_FILE );
			scpu->setMirrorHalted( true );
			s_Logger->Write( "SCPU", LogNotice,
			               "MIRROR HALTED (diagnostic): log is safely on SD" );
		}
	}

	// Stronger companion diagnostic: stop every physical expansion-bus access,
	// including immediate VIC/CIA reads and writes that bypass the mirror queue.
	// The emulator keeps executing against $FF reads, so its behaviour may no
	// longer be useful; the only observation sought is whether physical-screen
	// corruption stops when the RAD becomes electrically quiet.
	if ( cfgBusHaltAfterS > 0 && !radBusTrafficHalted() )
	{
		if ( ++s_BusHaltFrames >= (u32)cfgBusHaltAfterS * 60 )
		{
			if ( scpu ) scpu->setMirrorHalted( true );
			u32 eligiblePhases = 0, addrChanges = 0, distinctAddrs = 0;
			u32 sequentialAddrs = 0, writePhases = 0;
			u8 firstAddr = 0, lastAddr = 0;
			radBusHaltAndProbe( eligiblePhases, addrChanges, distinctAddrs,
			                    sequentialAddrs, writePhases, firstAddr, lastAddr );
			CScopedLoggingIRQs irqs;
			s_Logger->Write( "SCPU", LogNotice,
			               "ALL PHYSICAL BUS TRAFFIC HALTED (diagnostic)" );
			s_Logger->Write( "SCPU", LogNotice,
			               "idle host: eligible=%u distinct=%u changes=%u sequential=%u",
			               (unsigned)eligiblePhases, (unsigned)distinctAddrs,
			               (unsigned)addrChanges, (unsigned)sequentialAddrs );
			s_Logger->Write( "SCPU", LogNotice,
			               "idle host: first=%02X last=%02X", firstAddr, lastAddr );
			s_Logger->Write( "SCPU", writePhases ? LogError : LogNotice,
			               "idle host: R/W-low=%u/%u%s", (unsigned)writePhases,
			               (unsigned)eligiblePhases,
			               writePhases ? " -- ANOTHER BUS MASTER IS WRITING" : "" );
		}
	}

	// A timed display-scrub trial is self-controlled: report each off/on edge
	// without touching VIC state. Physical read/compare sampling is disabled on
	// normal hardware because this machine has no calibrated stable read window.
	if ( scpu && scpu->displayScrubPeriodSeconds()
	     && scpu->displayScrubPhaseGeneration() != s_DisplayScrubPhaseGeneration )
	{
		s_DisplayScrubPhaseGeneration = scpu->displayScrubPhaseGeneration();
		CWriteBuffer &wb = scpu->writeBuffer();
		CScopedLoggingIRQs irqs;
		s_Logger->Write( "SCPU", LogNotice,
		               "display scrub phase: %s; repaired=%llu (physical sampling disabled)",
		               scpu->displayScrubPhaseOn() ? "ON" : "OFF",
		               (unsigned long long)wb.m_DisplayScrubBytes );
	}

	// K381 is deliberately self-timed so one hardware run contains both the
	// no-repair and restored-repair observations. These messages occur only
	// after IEC is quiet; logging must never lengthen a live serial handshake.
	if ( scpu
	     && scpu->resyncDiagnosticGeneration() != s_ResyncDiagnosticGeneration )
	{
		s_ResyncDiagnosticGeneration = scpu->resyncDiagnosticGeneration();
		CScopedLoggingIRQs irqs;
		if ( scpu->resyncDiagnosticPhase() == SCPU_RESYNC_DIAG_HOLD )
			s_Logger->Write( "SCPU", LogNotice,
			               "K381: IEC quiet and dirty queue drained; background resync HELD for about 10 seconds" );
		else if ( scpu->resyncDiagnosticPhase() == SCPU_RESYNC_DIAG_REPAIRING )
			s_Logger->Write( "SCPU", LogNotice,
			               "K381: targeted active-display rewrite STARTED; shadow changed during hold=%s",
			               scpu->resyncDiagnosticShadowChanged() ? "YES -- RUN CONFOUNDED" : "no" );
		else if ( scpu->resyncDiagnosticPhase() == SCPU_RESYNC_DIAG_COMPLETE )
			s_Logger->Write( "SCPU", LogNotice,
			               "K381: targeted active-display rewrite COMPLETE" );
	}

	// --- heartbeat and stall self-detection ---------------------------------
	// A freeze used to leave nothing behind. The bounded bus waits keep the
	// firmware alive through one, but "no output" is still ambiguous: a silent
	// log looks identical whether the Pi is wedged or simply has nothing to
	// report. So say something every second, and say what the emulated machine
	// is doing while saying it.
	//
	// The PC range is the useful part. A machine that is working sweeps a wide
	// range of addresses in a second; one parked in a KERNAL wait loop spins
	// over a handful of bytes. When that happens for several seconds running,
	// dump the full diagnostic unprompted -- the reset button is a poor way to
	// collect it, because pressing it resets the physical CIAs and destroys the
	// serial state that explains the stall.
	if ( scpu && scpu->cpu() )
	{
		const u32 pc = (u32)scpu->cpu()->pc();
		if ( pc < s_HeartbeatPCLow )  s_HeartbeatPCLow  = pc;
		if ( pc > s_HeartbeatPCHigh ) s_HeartbeatPCHigh = pc;

		if ( ++s_HeartbeatFrames >= 60 )
		{
			// Never log mid-transaction: the serial protocol reads a pause as
			// EOI, and one heartbeat write costs milliseconds. Hold the window
			// open until a quiet second comes around; the accumulated PC range
			// still tells the story. This is the rule the reset dump has
			// always followed (three quiet frames) -- the first heartbeat cut
			// ignored it and broke JiffyDOS loads once per second.
			if ( scpu->memory().iecBusActive() )
				return false;
			const u32 span = s_HeartbeatPCHigh - s_HeartbeatPCLow;

			// A machine sitting at READY legitimately spins in the KERNAL's
			// keyboard wait ($E5CD-$E5D4), which a span heuristic cannot tell
			// from a wedge. Exempt exactly that window; a machine stuck
			// anywhere else -- including the $FF48 BRK loop this detector was
			// built for -- still trips.
			const bool kernalIdle = ( s_HeartbeatPCLow  >= 0x00E5C0
			                       && s_HeartbeatPCHigh <= 0x00E5D8 );
			const bool stalled = ( span < 64 ) && !kernalIdle;

			// The print is optional (HEARTBEAT, default off): one logger write
			// with IRQs unmasked costs enough real time to stutter the display
			// for a visible instant, once per second. The sampling above and
			// the stall detection below stay armed regardless -- they cost
			// nothing until a stall is actually found, and a found stall is
			// worth a stutter.
			if ( cfgHeartbeat )
			{
				CScopedLoggingIRQs irqs;
				s_Logger->Write( "SCPU", LogNotice,
				               "hb PC=$%06X..$%06X wb=%u iec=%u BA=%u PHI=%u%s",
				               (unsigned)s_HeartbeatPCLow, (unsigned)s_HeartbeatPCHigh,
				               (unsigned)scpu->writeBuffer().pending(),
				               (unsigned)( scpu->memory().iecBusActive() ? 1 : 0 ),
				               (unsigned)radBAWaitTimeouts,
				               (unsigned)radPHIWaitTimeouts,
				               stalled ? "  <-- STALLED" : "" );
			}

			if ( stalled )
			{
				// Three seconds in the same few bytes is not a busy loop doing
				// work; dump once and then stay quiet until it moves again.
				if ( ++s_HeartbeatStalledRuns == 3 && !s_HeartbeatDumped )
				{
					s_HeartbeatDumped = true;
					// Preserve a fourth matrix sample at the failure itself. This
					// diagnostic waits for a hidden physical raster interval but does
					// not advance the guest or drain outstanding mirror bytes.
					scpuCaptureVSPStallMatrixDiagnostic( scpu );
					scpuSnapshotResetDiagnostic( scpu );
					{
						CScopedLoggingIRQs dumpIRQs;
						// Self-contained: with HEARTBEAT off there were no hb
						// lines before this, so name the loop here.
						s_Logger->Write( "SCPU", LogNotice,
						               "STALL DETECTED at PC=$%06X..$%06X -- dumping state (no reset pressed)",
						               (unsigned)s_HeartbeatPCLow,
						               (unsigned)s_HeartbeatPCHigh );
						scpuEmitResetDiagnostic();
					}

					// Native C128 startup has no trustworthy KERNAL display yet.
					// Paint the captured state through the physical VIC and hold it
					// until the user has photographed it. The left RAD button then
					// follows the ordinary clean reboot path.
					if ( scpu->registers().scpu128Mode() )
					{
						scpuShowC128FreezeDiagnostic( scpu );
						scpuHoldC128FreezeDiagnostic( scpu );
						s_RebootRequested = true;
						return true;
					}
				}
			}
			else
			{
				s_HeartbeatStalledRuns = 0;
				s_HeartbeatDumped = false;
			}

			s_HeartbeatFrames = 0;
			s_HeartbeatPCLow  = 0xFFFFFFFF;
			s_HeartbeatPCHigh = 0;
		}
	}

	// Emit the saved pre-reset state on a later hook invocation, after several
	// complete IEC-idle frames. No values are read from the reset CIAs here.
	if ( s_ResetDiagnostic.pending && scpu )
	{
		if ( scpu->memory().iecBusActive() )
			s_ResetDiagnosticQuietFrames = 0;
		else if ( ++s_ResetDiagnosticQuietFrames >= 3 )
		{
			CScopedLoggingIRQs irqs;
			scpuEmitResetDiagnostic();
		}
	}

	return false;
}

// runFrame() does not invoke the normal frame hook. Startup and diagnostic
// loops use this wrapper so the physical reset button is still serviced before
// the emulator enters its permanent run() loop. A reset aborts the diagnostic
// currently in progress; normal execution can then resume immediately.
static bool scpuRunFrameAndCheckButtons( CSuperCPU &scpu )
{
	const u32 resetGeneration = s_HardwareResetGeneration;
	scpu.runFrame();
	const bool stop = scpuCheckButton( &scpu );
	return stop || resetGeneration != s_HardwareResetGeneration;
}

static CRADBus   radBus;
static CActLED   actLED;
static CSuperCPU superCPU;
static bool s_HDMIPictureActive = false;
static bool s_HDMIExclusive = false;
enum SCPURuntimeVideoTransition
{
	SCPU_VIDEO_TRANSITION_NONE,
	SCPU_VIDEO_TRANSITION_TO_HDMI,
	SCPU_VIDEO_TRANSITION_TO_VIC
};
static SCPURuntimeVideoTransition s_VideoTransition =
	SCPU_VIDEO_TRANSITION_NONE;
static u32 s_VideoTransitionFrame = 0;
static u32 s_VideoFrames = 0;
static u32 s_VideoLastSwitchFrame = 0;
static bool s_VideoHasSwitched = false;
static SCPURuntimeSample s_RuntimeBaseline = {};
static u32 s_PiThrottledStartup = 0;
static u32 s_PiThrottledPostSelfTest = 0;
static u32 s_PiThrottledCore1 = 0;
static u32 s_PiARMClockStartup = 0;
static u32 s_PiARMClockPostSelfTest = 0;
static u32 s_PiARMClockCore1 = 0;
static bool s_PiThrottledStartupValid = false;
static bool s_PiThrottledPostSelfTestValid = false;
static bool s_PiThrottledCore1Valid = false;
static bool s_PiARMClockStartupValid = false;
static bool s_PiARMClockPostSelfTestValid = false;
static bool s_PiARMClockCore1Valid = false;

static const u32 SCPU_PI_UNDERVOLTAGE_NOW = 1u << 0;
static const u32 SCPU_PI_UNDERVOLTAGE_OCCURRED = 1u << 16;

static void scpuUpdateVideoSelection( CSuperCPU *scpu )
{
	// Runtime selection exists only when VIDEO_MODE 1 created core 1. A
	// deliberate two-access register knock changes the request; this frame hook
	// performs the electrical handoff only at an IEC-quiet frame boundary.
	if ( !scpu || !s_HDMIPictureActive || !s_HDMIDisplay )
		return;
	s_VideoFrames++;
	const bool wantHDMI = scpu->registers().videoHDMIRequested();
	static const u32 debounceFrames = 30;
	const bool debounceDone = !s_VideoHasSwitched
	                       || (u32)( s_VideoFrames - s_VideoLastSwitchFrame )
	                          >= debounceFrames;

	if ( s_VideoTransition == SCPU_VIDEO_TRANSITION_TO_VIC )
	{
		// A reversed request cancels cleanly because physical delivery has not
		// yet been re-enabled. Wake core 1 and leave the active mode unchanged.
		if ( wantHDMI )
		{
			s_HDMIDisplay->setPictureEnabled( true );
			s_VideoTransition = SCPU_VIDEO_TRANSITION_NONE;
			return;
		}
		if ( !s_HDMIDisplay->pictureParked()
		     || scpu->memory().iecBusActive() || !debounceDone )
			return;

		// Core 1 now generates no shared-memory traffic. Restore the physical
		// write path and queue only the currently visible working set; the
		// existing raster scheduler converges it in bounded installments.
		scpu->memory().enableBusKeepalive( false );
		scpu->enablePhysicalMirror();
		scpu->memory().enableVICLog( false );
		s_HDMIExclusive = false;
		s_VideoTransition = SCPU_VIDEO_TRANSITION_NONE;
		s_VideoLastSwitchFrame = s_VideoFrames;
		s_VideoHasSwitched = true;
		return;
	}

	if ( s_VideoTransition == SCPU_VIDEO_TRANSITION_TO_HDMI )
	{
		if ( !wantHDMI )
		{
			s_HDMIDisplay->setPictureEnabled( false );
			scpu->memory().enableVICLog( false );
			s_VideoTransition = SCPU_VIDEO_TRANSITION_NONE;
			return;
		}
		// Never detach physical presentation until core 1 proves it completed
		// a frame newer than the one visible when the request was accepted.
		if ( s_HDMIDisplay->presentedFrames() == s_VideoTransitionFrame
		     || scpu->memory().iecBusActive() || !debounceDone )
			return;

		scpu->memory().enableBusKeepalive( true );
		scpu->disablePhysicalMirror();
		s_HDMIExclusive = true;
		s_VideoTransition = SCPU_VIDEO_TRANSITION_NONE;
		s_VideoLastSwitchFrame = s_VideoFrames;
		s_VideoHasSwitched = true;
		scpuBeginRuntimeDiagnostic( scpu );
		return;
	}

	if ( wantHDMI == s_HDMIExclusive || !debounceDone
	     || scpu->memory().iecBusActive() )
		return;

	if ( wantHDMI )
	{
		// Capture starts before core 1 wakes; after one complete new frame the
		// transition above may safely make HDMI authoritative.
		scpu->memory().enableVICLog( true );
		s_HDMIDisplay->resetCollisionLatches();
		s_VideoTransitionFrame = s_HDMIDisplay->presentedFrames();
		s_HDMIDisplay->setPictureEnabled( true );
		s_VideoTransition = SCPU_VIDEO_TRANSITION_TO_HDMI;
	}
	else
	{
		// Stop presentation first. The next frame hook observes the parked
		// acknowledgement before it lets any physical mirror burst resume.
		s_HDMIDisplay->setPictureEnabled( false );
		s_VideoTransition = SCPU_VIDEO_TRANSITION_TO_VIC;
	}
}

static SCPURuntimeSample scpuCaptureRuntimeSample( CSuperCPU *scpu,
	                                               bool handoff )
{
	SCPURuntimeSample s = {};
	s.handoff = handoff && scpu && scpu->hostBus();
	if ( !s.handoff ) return s;

	s.hostCycles = scpu->hostBus()->hostCycles();
	s.emuCycles = scpu->memory().m_EmuCycles;
	s.ramWrites = scpu->memory().m_RamWrites;
	s.acceptedWrites = scpu->writeBuffer().m_WritesAccepted;
	s.skippedWrites = scpu->writeBuffer().m_WritesSkipped
	                + scpu->writeBuffer().m_MirrorFloorSkipped;
	s.postedWaitCycles = scpu->memory().m_MirrorStretchCycles;
	s.slowCycles = scpu->memory().m_SlowPacedCycles;
	s.iecThrottledCycles = scpu->memory().m_IECThrottledCycles;
	s.transfers = radBus.transfers();
	s.serialTransfers = radBus.serialTransfers();
	s.idlePrimes = radBus.readPrimes();
	if ( s_HDMIDisplay )
	{
		s.maxBandUS = s_HDMIDisplay->maxBandUS();
		s.missedBandDeadlines = s_HDMIDisplay->missedBandDeadlines();
	}
	return s;
}

static void scpuBeginRuntimeDiagnostic( CSuperCPU *scpu )
{
	if ( s_HDMIDisplay ) s_HDMIDisplay->resetRuntimeDiagnostics();
	s_RuntimeBaseline = scpuCaptureRuntimeSample( scpu, true );
}

static SCPURuntimeResult scpuCurrentRuntimeResult( CSuperCPU *scpu )
{
	const SCPURuntimeSample current =
		scpuCaptureRuntimeSample( scpu, s_HDMIExclusive );
	const u32 rate = ( scpu && scpu->hostBus() )
	               ? scpu->hostBus()->hostCyclesPerSec() : 0;
	return scpuRuntimeDelta( s_RuntimeBaseline, current, rate );
}

static bool scpuRuntimeDiagnosticActive()
{
	return s_HDMIExclusive;
}

static bool scpuReadPiThrottled( u32 &state )
{
	CBcmPropertyTags tags;
	TPropertyTagSimple tag;
	tag.nValue = 0;
	if ( !tags.GetTag( PROPTAG_GET_THROTTLED, &tag, sizeof tag, 4 ) )
	{
		state = 0;
		return false;
	}
	state = tag.nValue;
	return true;
}

static bool scpuReadPiARMClock( u32 &rateHz )
{
	CBcmPropertyTags tags;
	TPropertyTagClockRate tag;
	tag.nClockId = CLOCK_ID_ARM;
	tag.nRate = 0;
	if ( !tags.GetTag( PROPTAG_GET_CLOCK_RATE_MEASURED,
	                   &tag, sizeof tag, 4 ) || tag.nRate == 0 )
	{
		rateHz = 0;
		return false;
	}
	rateHz = tag.nRate;
	return true;
}

static bool scpuPiUndervoltageSeen( bool valid, u32 state )
{
	return valid && ( state & ( SCPU_PI_UNDERVOLTAGE_NOW
	                           | SCPU_PI_UNDERVOLTAGE_OCCURRED ) ) != 0;
}

static void scpuLogPiThrottled( CLogger *logger, const char *when,
	                            bool valid, u32 state )
{
	if ( !valid )
	{
		logger->Write( "SCPU", LogWarning,
		               "Pi power %s: GET_THROTTLED query failed", when );
		return;
	}

	const bool warning = ( state & 0x000F000Fu ) != 0;
	logger->Write( "SCPU", warning ? LogWarning : LogNotice,
	               "Pi power %s: throttled=$%08X "
	               "live[uv/cap/throttle/temp]=%u/%u/%u/%u "
	               "sticky=%u/%u/%u/%u",
	               when, (unsigned)state,
	               (unsigned)( ( state >> 0 ) & 1 ),
	               (unsigned)( ( state >> 1 ) & 1 ),
	               (unsigned)( ( state >> 2 ) & 1 ),
	               (unsigned)( ( state >> 3 ) & 1 ),
	               (unsigned)( ( state >> 16 ) & 1 ),
	               (unsigned)( ( state >> 17 ) & 1 ),
	               (unsigned)( ( state >> 18 ) & 1 ),
	               (unsigned)( ( state >> 19 ) & 1 ) );
}

static void scpuLogPiARMClock( CLogger *logger, const char *when,
	                           bool valid, u32 rateHz )
{
	if ( !valid )
	{
		logger->Write( "SCPU", LogWarning,
		               "Pi ARM clock %s: measured-rate query failed", when );
		return;
	}
	// Every cycle count in this project -- bus waits, the pacing grid, the
	// speed verdict, every microsecond column in every diagnostic -- is
	// denominated in SCPU_ARM_CLOCK_HZ, and nothing has ever checked that the
	// Pi agrees. config.txt pins arm_freq=1400 with force_turbo=1, so a
	// disagreement here is a fault (throttling, or an edited config.txt), not
	// ordinary idle clock scaling.
	const u32 assumed = SCPU_ARM_CLOCK_HZ;
	const u32 delta = rateHz > assumed ? rateHz - assumed : assumed - rateHz;
	if ( delta > assumed / 100u )
		logger->Write( "SCPU", LogWarning,
		               "Pi ARM clock %s: measured %u Hz but this firmware assumes "
		               "%u Hz -- EVERY cycle count is off by %u%%",
		               when, (unsigned)rateHz, (unsigned)assumed,
		               (unsigned)( (u64)delta * 100u / assumed ) );
	else
		logger->Write( "SCPU", LogNotice,
		               "Pi ARM clock %s: %u Hz (assumed %u Hz)",
		               when, (unsigned)rateHz, (unsigned)assumed );
}

// The mailbox query above reports what the FIRMWARE believes the ARM clock
// is. This measures the counter we actually time with: PMCCNTR_EL0 against
// CNTPCT_EL0, whose rate is fixed by CNTFRQ_EL0 and does not move with
// arm_freq or throttling. It cannot catch a mid-run thermal step (config.txt
// has temp_soft_limit=70, and initial_turbo=2 pins the clock across this
// window anyway) -- what it CAN catch is our cycle unit not being
// SCPU_ARM_CLOCK_HZ at all: a counter divider, or a config.txt edited after
// the bus timings were tuned. Runs before the bus is acquired, so there is no
// timed window to disturb.
static u32 s_MeasuredCycleCounterHz = 0;

static void scpuLogMeasuredCycleCounter( CLogger *logger )
{
	u64 frq = 0;
	asm volatile( "mrs %0, CNTFRQ_EL0" : "=r" ( frq ) );
	if ( !frq )
	{
		logger->Write( "SCPU", LogWarning,
		               "cycle counter: CNTFRQ_EL0 reads 0; cannot verify the ARM clock" );
		return;
	}

	u64 t0 = 0, t1 = 0;
	asm volatile( "mrs %0, CNTPCT_EL0" : "=r" ( t0 ) );
	const u64 p0 = radBus.hostCycles();
	// 10ms: long enough that the register reads around the window are noise.
	const u64 window = frq / 100u;
	do {
		asm volatile( "mrs %0, CNTPCT_EL0" : "=r" ( t1 ) );
	} while ( t1 - t0 < window );
	const u64 p1 = radBus.hostCycles();
	const u64 elapsed = t1 - t0;
	if ( !elapsed ) return;
	s_MeasuredCycleCounterHz = (u32)( ( p1 - p0 ) * frq / elapsed );

	const u32 assumed = SCPU_ARM_CLOCK_HZ;
	const u32 delta = s_MeasuredCycleCounterHz > assumed
	                ? s_MeasuredCycleCounterHz - assumed
	                : assumed - s_MeasuredCycleCounterHz;
	if ( delta > assumed / 100u )
		logger->Write( "SCPU", LogWarning,
		               "cycle counter: PMCCNTR runs at %u Hz, not the assumed %u Hz "
		               "-- EVERY cycle count in this firmware is off by %u%%",
		               (unsigned)s_MeasuredCycleCounterHz, (unsigned)assumed,
		               (unsigned)( (u64)delta * 100u / assumed ) );
	else
		logger->Write( "SCPU", LogNotice,
		               "cycle counter: PMCCNTR measured %u Hz against assumed %u Hz",
		               (unsigned)s_MeasuredCycleCounterHz, (unsigned)assumed );
}

static void scpuAppendPiThrottledDiag( char *dst, u32 capacity, u32 &length )
{
	if ( length && dst[ length - 1 ] != '\n' )
	{
		scpuLineChar( dst, capacity, length, '\r' );
		scpuLineChar( dst, capacity, length, '\n' );
	}
	scpuLineText( dst, capacity, length, "pi throttled: startup=" );
	if ( s_PiThrottledStartupValid )
	{
		scpuLineText( dst, capacity, length, "$" );
		scpuLineHexU32( dst, capacity, length, s_PiThrottledStartup );
	}
	else
		scpuLineText( dst, capacity, length, "QUERY-FAILED" );
	scpuLineText( dst, capacity, length, " post-selftest=" );
	if ( s_PiThrottledPostSelfTestValid )
	{
		scpuLineText( dst, capacity, length, "$" );
		scpuLineHexU32( dst, capacity, length, s_PiThrottledPostSelfTest );
	}
	else
		scpuLineText( dst, capacity, length, "QUERY-FAILED" );
	scpuLineText( dst, capacity, length,
	              " bits 0-3=live 16-19=sticky\r\n" );
	scpuLineText( dst, capacity, length, "pi measured ARM Hz: startup=" );
	if ( s_PiARMClockStartupValid )
		scpuLineDecimal( dst, capacity, length, s_PiARMClockStartup );
	else
		scpuLineText( dst, capacity, length, "QUERY-FAILED" );
	scpuLineText( dst, capacity, length, " post-selftest=" );
	if ( s_PiARMClockPostSelfTestValid )
		scpuLineDecimal( dst, capacity, length, s_PiARMClockPostSelfTest );
	else
		scpuLineText( dst, capacity, length, "QUERY-FAILED" );
	scpuLineText( dst, capacity, length, " core1=" );
	if ( s_PiARMClockCore1Valid )
		scpuLineDecimal( dst, capacity, length, s_PiARMClockCore1 );
	else
		scpuLineText( dst, capacity, length, "NOT-SAMPLED" );
	// The ASSUMED rate belongs in the file beside the measured ones. A trace
	// read weeks later cannot otherwise tell whether its microsecond columns
	// were computed against the clock the Pi was actually running at.
	scpuLineText( dst, capacity, length, " assumed=" );
	scpuLineDecimal( dst, capacity, length, SCPU_ARM_CLOCK_HZ );
	scpuLineText( dst, capacity, length, " pmccntr-measured=" );
	if ( s_MeasuredCycleCounterHz )
		scpuLineDecimal( dst, capacity, length, s_MeasuredCycleCounterHz );
	else
		scpuLineText( dst, capacity, length, "NOT-SAMPLED" );
	scpuLineText( dst, capacity, length, "\r\npi throttled core1=" );
	if ( s_PiThrottledCore1Valid )
	{
		scpuLineText( dst, capacity, length, "$" );
		scpuLineHexU32( dst, capacity, length, s_PiThrottledCore1 );
	}
	else
		scpuLineText( dst, capacity, length, "NOT-SAMPLED" );
	scpuLineText( dst, capacity, length, "\r\n" );
}

static void scpuHDMIShowFailure()
{
	if ( !s_HDMIDisplay ) return;
	s_HDMIPictureActive = false;
	s_HDMIDisplay->showFailure();
}

static void scpuAppendRuntimeBusDiag( char *dst, u32 capacity, u32 &length,
	                                  const SCPURuntimeResult &r )
{
	if ( length && dst[ length - 1 ] != '\n' )
	{
		scpuLineChar( dst, capacity, length, '\r' );
		scpuLineChar( dst, capacity, length, '\n' );
	}
	scpuLineText( dst, capacity, length,
	              "post-handoff runtime: build=362 handoff=" );
	scpuLineDecimal( dst, capacity, length, r.valid ? 1 : 0 );
	if ( r.valid )
	{
		scpuLineText( dst, capacity, length, " host-us=" );
		scpuLineDecimal64( dst, capacity, length, r.hostUS );
		scpuLineText( dst, capacity, length, " emu=" );
		scpuLineDecimal64( dst, capacity, length, r.emuCycles );
		scpuLineText( dst, capacity, length, " achieved-khz=" );
		scpuLineDecimal64( dst, capacity, length, r.achievedKHz );
		scpuLineText( dst, capacity, length, " ram-writes=" );
		scpuLineDecimal64( dst, capacity, length, r.ramWrites );
		scpuLineText( dst, capacity, length, " accepted=" );
		scpuLineDecimal64( dst, capacity, length, r.acceptedWrites );
		scpuLineText( dst, capacity, length, " skipped=" );
		scpuLineDecimal64( dst, capacity, length, r.skippedWrites );
		scpuLineText( dst, capacity, length, " posted-wait=" );
		scpuLineDecimal64( dst, capacity, length, r.postedWaitCycles );
		scpuLineText( dst, capacity, length, " slow=" );
		scpuLineDecimal64( dst, capacity, length, r.slowCycles );
		scpuLineText( dst, capacity, length, " iec-slow=" );
		scpuLineDecimal64( dst, capacity, length, r.iecThrottledCycles );
		scpuLineText( dst, capacity, length, " transfers=" );
		scpuLineDecimal64( dst, capacity, length, r.transfers );
		scpuLineText( dst, capacity, length, " iec=" );
		scpuLineDecimal64( dst, capacity, length, r.serialTransfers );
		scpuLineText( dst, capacity, length, " idle-primes=" );
		scpuLineDecimal64( dst, capacity, length, r.idlePrimes );
		scpuLineText( dst, capacity, length, " hdmi-max-band-us=" );
		scpuLineDecimal( dst, capacity, length, r.maxBandUS );
		scpuLineText( dst, capacity, length, " hdmi-missed=" );
		scpuLineDecimal( dst, capacity, length, r.missedBandDeadlines );
	}
	scpuLineText( dst, capacity, length, "\r\n" );
}

static void scpuAppendRuntimePiDiag( char *dst, u32 capacity, u32 &length )
{
	u32 throttled = 0, clockHz = 0;
	const bool throttledValid = scpuReadPiThrottled( throttled );
	const bool clockValid = scpuReadPiARMClock( clockHz );
	scpuLineText( dst, capacity, length, "post-handoff pi throttled=" );
	if ( throttledValid )
	{
		scpuLineText( dst, capacity, length, "$" );
		scpuLineHexU32( dst, capacity, length, throttled );
	}
	else
		scpuLineText( dst, capacity, length, "QUERY-FAILED" );
	scpuLineText( dst, capacity, length, " measured-arm-hz=" );
	if ( clockValid ) scpuLineDecimal( dst, capacity, length, clockHz );
	else              scpuLineText( dst, capacity, length, "QUERY-FAILED" );
	scpuLineText( dst, capacity, length,
	              " bits 0-3=live 16-19=sticky\r\n" );
}

static void scpuAppendVSPTraceDiag( char *dst, u32 capacity, u32 &length )
{
	const SCPUResetDiagnostic &d = s_ResetDiagnostic;
	if ( !d.vspValid )
	{
		scpuLineText( dst, capacity, length,
		              "vsp trace: no reset-time snapshot available\r\n" );
		return;
	}

	scpuLineText( dst, capacity, length, "vsp trace: retained=" );
	scpuLineDecimal( dst, capacity, length, d.vspCount );
	scpuLineText( dst, capacity, length, " total=" );
	scpuLineDecimal( dst, capacity, length, d.vspTotal );
	scpuLineText( dst, capacity, length, " overwritten=" );
	scpuLineDecimal( dst, capacity, length, d.vspOverwritten );
	scpuLineText( dst, capacity, length, " signature D016=$" );
	scpuLineHexByte( dst, capacity, length, d.vic16 );
	scpuLineText( dst, capacity, length, " x=" );
	scpuLineDecimal( dst, capacity, length, d.vic16 & 7 );
	scpuLineText( dst, capacity, length, " D018=$" );
	scpuLineHexByte( dst, capacity, length, d.vic18 );
	scpuLineText( dst, capacity, length, " phase=$" );
	scpuLineHexByte( dst, capacity, length, d.vspGamePhase );
	scpuLineText( dst, capacity, length, "\r\n" );
	scpuLineText( dst, capacity, length, "video: detected=" );
	if ( !d.vspVideoValid )
		scpuLineText( dst, capacity, length, "UNKNOWN" );
	else if ( d.vspVideo == VIDEO_PAL )
		scpuLineText( dst, capacity, length, "PAL" );
	else if ( d.vspVideo == VIDEO_NTSC_R56A )
		scpuLineText( dst, capacity, length, "NTSC-R56A" );
	else
		scpuLineText( dst, capacity, length, "NTSC-R8" );
	scpuLineText( dst, capacity, length, " cycles/line=" );
	scpuLineDecimal( dst, capacity, length, d.vspCyclesPerLine );
	scpuLineText( dst, capacity, length, " lines=" );
	scpuLineDecimal( dst, capacity, length, d.vspRasterLines );
	scpuLineText( dst, capacity, length, "\r\n" );

	const SCPUVSPMatrixDiagnostic &matrix = d.vspMatrix;
	scpuLineText( dst, capacity, length, "matrix compare: state=" );
	scpuLineDecimal( dst, capacity, length, matrix.state );
	scpuLineText( dst, capacity, length, " gameplay=" );
	scpuLineDecimal( dst, capacity, length, matrix.gameplaySeen );
	scpuLineText( dst, capacity, length, " phase-changes=" );
	scpuLineDecimal( dst, capacity, length, matrix.phaseChanges );
	scpuLineText( dst, capacity, length, " qualifying=" );
	scpuLineDecimal( dst, capacity, length, matrix.qualifyingFrames );
	scpuLineText( dst, capacity, length, " quiet=" );
	scpuLineDecimal( dst, capacity, length, matrix.quietFrames );
	scpuLineText( dst, capacity, length, " trace=" );
	scpuLineDecimal( dst, capacity, length, matrix.traceTotal );
	scpuLineText( dst, capacity, length, " captured=" );
	scpuLineDecimal( dst, capacity, length, matrix.capturedCount );
	scpuLineText( dst, capacity, length, "\r\n" );
	for ( u32 slot = 0; slot < SCPU_VSP_MATRIX_CAPTURE_SLOTS; slot++ )
	{
		const SCPUVSPMatrixCapture &capture = matrix.capture[ slot ];
		scpuLineText( dst, capacity, length, "matrix slot" );
		scpuLineDecimal( dst, capacity, length, slot );
		scpuLineText( dst, capacity, length, capture.kind ? " stall" : " clean" );
		scpuLineText( dst, capacity, length, ": state=" );
		scpuLineDecimal( dst, capacity, length, capture.state );
		scpuLineText( dst, capacity, length, " target/frame=" );
		scpuLineDecimal( dst, capacity, length, capture.targetFrame );
		scpuLineChar( dst, capacity, length, '/' );
		scpuLineDecimal( dst, capacity, length, capture.qualifyingFrame );
		scpuLineText( dst, capacity, length, " trace=" );
		scpuLineDecimal( dst, capacity, length, capture.traceTotal );
		scpuLineText( dst, capacity, length, " phase=$" );
		scpuLineHexByte( dst, capacity, length, capture.gamePhase );
		scpuLineText( dst, capacity, length, " attempts=" );
		scpuLineDecimal( dst, capacity, length, capture.captureAttempts );
		scpuLineText( dst, capacity, length, " wait unsafe/pending=" );
		scpuLineDecimal( dst, capacity, length, capture.unsafeWindowWaits );
		scpuLineChar( dst, capacity, length, '/' );
		scpuLineDecimal( dst, capacity, length, capture.pendingWaits );
		scpuLineText( dst, capacity, length, "\r\n" );
		if ( capture.state != 2 ) continue;

		scpuLineText( dst, capacity, length, "matrix bases" );
		scpuLineDecimal( dst, capacity, length, slot );
		scpuLineText( dst, capacity, length, ": logical=$" );
		scpuLineHexByte( dst, capacity, length,
		                 (u8)( capture.logicalBase >> 8 ) );
		scpuLineHexByte( dst, capacity, length, (u8)capture.logicalBase );
		scpuLineText( dst, capacity, length, " physical=$" );
		scpuLineHexByte( dst, capacity, length,
		                 (u8)( capture.physicalBase >> 8 ) );
		scpuLineHexByte( dst, capacity, length, (u8)capture.physicalBase );
		scpuLineText( dst, capacity, length, " raster=" );
		scpuLineDecimal( dst, capacity, length, capture.rasterBefore );
		scpuLineChar( dst, capacity, length, '-' );
		scpuLineDecimal( dst, capacity, length, capture.rasterAfter );
		scpuLineText( dst, capacity, length, " safe=" );
		scpuLineDecimal( dst, capacity, length, capture.safeWindow );
		scpuLineText( dst, capacity, length, " pending=" );
		scpuLineDecimal( dst, capacity, length, capture.pendingAtCapture );
		scpuLineText( dst, capacity, length, "\r\n" );

		scpuLineText( dst, capacity, length, "matrix result" );
		scpuLineDecimal( dst, capacity, length, slot );
		scpuLineText( dst, capacity, length, ": direct=" );
		scpuLineDecimal( dst, capacity, length, capture.directMatches );
		scpuLineText( dst, capacity, length, "/1024 mismatch=" );
		scpuLineDecimal( dst, capacity, length, capture.mismatchCount );
		scpuLineText( dst, capacity, length, " first/last=$" );
		if ( capture.firstMismatch == 0xFFFF )
			scpuLineText( dst, capacity, length, "----/----" );
		else
		{
			scpuLineHexByte( dst, capacity, length,
			                 (u8)( capture.firstMismatch >> 8 ) );
			scpuLineHexByte( dst, capacity, length,
			                 (u8)capture.firstMismatch );
			scpuLineChar( dst, capacity, length, '/' );
			scpuLineHexByte( dst, capacity, length,
			                 (u8)( capture.lastMismatch >> 8 ) );
			scpuLineHexByte( dst, capacity, length,
			                 (u8)capture.lastMismatch );
		}
		scpuLineText( dst, capacity, length, " best-shift=" );
		if ( capture.bestShift < 0 )
		{
			scpuLineChar( dst, capacity, length, '-' );
			scpuLineDecimal( dst, capacity, length,
			                 (u32)( -capture.bestShift ) );
		}
		else
		{
			scpuLineChar( dst, capacity, length, '+' );
			scpuLineDecimal( dst, capacity, length,
			                 (u32)capture.bestShift );
		}
		scpuLineText( dst, capacity, length, " score=" );
		scpuLineDecimal( dst, capacity, length, capture.bestShiftMatches );
		scpuLineChar( dst, capacity, length, '/' );
		scpuLineDecimal( dst, capacity, length, capture.bestShiftCompared );
		scpuLineText( dst, capacity, length, "\r\n" );

		for ( u32 i = 0; i < capture.mismatchSampleCount; i++ )
		{
			const SCPUVSPMatrixMismatch &m = capture.mismatch[ i ];
			scpuLineText( dst, capacity, length, "matrix mismatch" );
			scpuLineDecimal( dst, capacity, length, slot );
			scpuLineText( dst, capacity, length, " +$" );
			scpuLineHexByte( dst, capacity, length, (u8)( m.offset >> 8 ) );
			scpuLineHexByte( dst, capacity, length, (u8)m.offset );
			scpuLineText( dst, capacity, length, " expected/physical=$" );
			scpuLineHexByte( dst, capacity, length, m.expected );
			scpuLineChar( dst, capacity, length, '/' );
			scpuLineHexByte( dst, capacity, length, m.physical );
			scpuLineText( dst, capacity, length, "\r\n" );
		}
	}

	for ( u32 i = 0; i < d.vspCount; i++ )
	{
		const VSPTimingEvent &e = d.vsp[ i ];
		scpuLineText( dst, capacity, length, "vsp" );
		if ( i < 10 ) scpuLineChar( dst, capacity, length, '0' );
		scpuLineDecimal( dst, capacity, length, i );
		scpuLineText( dst, capacity, length, " seq=" );
		scpuLineDecimal( dst, capacity, length, e.sequence );
		scpuLineText( dst, capacity, length, " w$" );
		scpuLineHexByte( dst, capacity, length, (u8)( e.address >> 8 ) );
		scpuLineHexByte( dst, capacity, length, (u8)e.address );
		scpuLineChar( dst, capacity, length, '=' );
		scpuLineHexByte( dst, capacity, length, e.value );
		scpuLineChar( dst, capacity, length, '/' );
		scpuLineHexByte( dst, capacity, length, e.physicalValue );
		scpuLineText( dst, capacity, length, " raster=" );
		if ( e.flags & SCPU_VSP_RASTER_VALID )
		{
			scpuLineHexByte( dst, capacity, length, e.raster );
			scpuLineChar( dst, capacity, length, '+' );
			if ( e.hostCycle >= e.rasterHostCycle )
				scpuLineDecimal64( dst, capacity, length,
				                   e.hostCycle - e.rasterHostCycle );
			else
				scpuLineText( dst, capacity, length, "WRAP" );
		}
		else
			scpuLineText( dst, capacity, length, "--" );
		scpuLineText( dst, capacity, length, " host=" );
		scpuLineDecimal64( dst, capacity, length, e.hostCycle );
		// The one derived number this sidecar exists to produce: the interval
		// since the previous write TO THE SAME REGISTER. Across Mario's
		// DEC/INC pair on $D011 that interval IS how long the register holds
		// its decremented value, which NMOS gives as exactly 6 -- against the
		// real card's 90us/92 cycles and our measured 100us/102. It is
		// deliberately NOT "since the previous retained event": the handler
		// also writes $D016 and $D018, and an intervening one would silently
		// turn this into a different measurement with nothing to say so.
		//
		// Printed in HUNDREDTHS of a C64 cycle against the hostPerC64 in the
		// grid= field below, so 600 is exactly six. Whole cycles would hide a
		// one-cycle error whenever that denominator is off: on the nominal
		// 1,000,000Hz grid (1400 ARM at 1.4GHz) a true NTSC 7 divides to 6.84
		// and floors to the 6 this trace is read to confirm.
		scpuLineText( dst, capacity, length, " dprev=" );
		u32 prev = i;
		while ( prev && d.vsp[ prev - 1 ].address != e.address )
			prev--;
		if ( prev && e.hostPerC64
		     && e.hostCycle >= d.vsp[ prev - 1 ].hostCycle )
		{
			const u64 dhost = e.hostCycle - d.vsp[ prev - 1 ].hostCycle;
			scpuLineDecimal64( dst, capacity, length, dhost );
			scpuLineText( dst, capacity, length, "h=" );
			scpuLineDecimal64( dst, capacity, length,
			                   dhost * 100u / e.hostPerC64 );
			scpuLineText( dst, capacity, length, "cc" );
		}
		else
			scpuLineText( dst, capacity, length, "--" );
		// The stall charged between the previous retained event and this one.
		// On a scrolling Mario capture the $D011 INC must show a nonzero value
		// here whenever its DEC landed early in the line, and dprev must equal
		// 59 - DECcycle; a zero here with a 6-cycle dprev is the K451 failure
		// reappearing.
		scpuLineText( dst, capacity, length, " bastall=" );
		scpuLineDecimal( dst, capacity, length, e.baStallCycles );
		scpuLineChar( dst, capacity, length, '/' );
		scpuLineDecimal( dst, capacity, length, e.baStallEvents );
		// WHICH END MOVED, in whole PHI cycles: the anchor $D012 read's own BA
		// spins, then this write's. Printed adjacently because the reading is
		// the PAIR -- these are the only two numbers in the trace that can say
		// whether a whole-cycle placement slip happened on the read end or the
		// write end, and every other placement field here averages over both.
		// Not a duration and not hundredths: one unit is one whole C64 cycle.
		// THE BADLINE REFERENCE. Host cycles back to the last badline-class
		// release and that release's length. The length is not decoration: a
		// release of 40+ is the only thing that licenses treating the stamp as
		// in-line cycle ~54, and reconstructing its entry as release minus
		// length must land near in-line 12 or the reference is not a badline.
		scpuLineText( dst, capacity, length, " badref=" );
		if ( e.sinceBadlineHost == 0xFFFFFFFFu )
			scpuLineText( dst, capacity, length, "--" );
		else
		{
			scpuLineDecimal64( dst, capacity, length, e.sinceBadlineHost );
			scpuLineChar( dst, capacity, length, 'h' );
			scpuLineText( dst, capacity, length, "/len" );
			scpuLineDecimal( dst, capacity, length, e.badlineReleaseLen );
		}
		// counted/elapsed since the guest's own $D012 anchor, for THIS event: the
		// guest cycles the core charged against the C64 cycles that really passed.
		// Equal means the pacer held 1:1 across the span; counted > elapsed means
		// cycles ran unpaced inside it. One unit is one whole C64 cycle; elapsed
		// is printed in hundredths like dprev=.
		scpuLineText( dst, capacity, length, " anc=" );
		if ( ( e.flags & SCPU_VSP_RASTER_VALID ) && e.c64Cycle >= e.rasterC64Cycle
		     && e.hostCycle >= e.rasterHostCycle && e.hostPerC64 )
		{
			scpuLineDecimal64( dst, capacity, length, e.c64Cycle - e.rasterC64Cycle );
			scpuLineText( dst, capacity, length, "c/" );
			scpuLineDecimal64( dst, capacity, length,
			                   ( e.hostCycle - e.rasterHostCycle ) * 100u / e.hostPerC64 );
			scpuLineText( dst, capacity, length, "cc" );
		}
		else
			scpuLineText( dst, capacity, length, "--" );
		// catch= (K499): the catch-up's deficit at ENTRY, countTarget - entry,
		// in signed hundredths of a cycle, then the wait it applied. Positive
		// is early-and-corrected; negative is late-and-UNcorrected, the half
		// of the placement no other field shows. -- when the gate did not run
		// for this write (no anchor, not factor 1, inside a serial window).
		scpuLineText( dst, capacity, length, " catch=" );
		if ( ( e.flags & SCPU_VSP_CATCH_VALID ) && e.hostPerC64 )
		{
			const s64 d = e.catchDeficit;
			scpuLineChar( dst, capacity, length, d < 0 ? '-' : '+' );
			scpuLineDecimal64( dst, capacity, length,
			                   (u64)( d < 0 ? -d : d ) * 100u / e.hostPerC64 );
			scpuLineChar( dst, capacity, length, '/' );
			scpuLineDecimal64( dst, capacity, length,
			                   (u64)e.catchWait * 100u / e.hostPerC64 );
			scpuLineText( dst, capacity, length, "cc" );
		}
		else
			scpuLineText( dst, capacity, length, "--" );
		scpuLineText( dst, capacity, length, " spins=" );
		scpuLineDecimal( dst, capacity, length, e.anchorBASpins );
		scpuLineChar( dst, capacity, length, 'r' );
		scpuLineChar( dst, capacity, length, '/' );
		scpuLineDecimal( dst, capacity, length, e.writeBASpins );
		scpuLineChar( dst, capacity, length, 'w' );
		// postrel= is the assumption-free half of this trace: host cycles from
		// the BA release to this write, then the same in hundredths of a C64
		// cycle like dprev=. The trailing b/s says whether the release it is
		// measured from was badline-class or a sprite steal -- an 's' makes the
		// number meaningless for the invariant, see SCPU_VSP_RELEASE_BADLINE.
		// Against the invariant's INC at 59, and with the two in-cycle offsets
		// VSPTimingEvent::sinceReleaseHost spells out, roughly 380..580 says our
		// placement is right and K456's +3..+4 hold error lives in the raster=
		// reconstruction, whose anchor cycle is assumed and which the stall
		// itself moved; roughly 730..930 says the write really is about 3.5
		// cycles late. The bands do not touch; a point value read out of this
		// would be over-reading it by a cycle.
		scpuLineText( dst, capacity, length, " postrel=" );
		if ( e.sinceReleaseHost == 0xFFFF || !e.hostPerC64 )
			scpuLineText( dst, capacity, length, "--" );
		else
		{
			scpuLineDecimal( dst, capacity, length, e.sinceReleaseHost );
			scpuLineText( dst, capacity, length, "h=" );
			scpuLineDecimal( dst, capacity, length,
			                 (u32)e.sinceReleaseHost * 100u / e.hostPerC64 );
			scpuLineText( dst, capacity, length, "cc" );
			scpuLineChar( dst, capacity, length,
			              ( e.flags & SCPU_VSP_RELEASE_BADLINE ) ? 'b' : 's' );
		}
		scpuLineText( dst, capacity, length, " emu=" );
		scpuLineDecimal64( dst, capacity, length, e.emuCycle );
		scpuLineText( dst, capacity, length, " c64=" );
		scpuLineDecimal64( dst, capacity, length, e.c64Cycle );
		scpuLineText( dst, capacity, length, " pace=" );
		if ( e.hostCycle >= e.pacingAnchor )
		{
			scpuLineChar( dst, capacity, length, '+' );
			scpuLineDecimal64( dst, capacity, length,
			                   e.hostCycle - e.pacingAnchor );
		}
		else
		{
			scpuLineChar( dst, capacity, length, '-' );
			scpuLineDecimal64( dst, capacity, length,
			                   e.pacingAnchor - e.hostCycle );
		}
		scpuLineText( dst, capacity, length, " debt=" );
		scpuLineDecimal64( dst, capacity, length, e.pacingDebt );
		scpuLineText( dst, capacity, length, " grid=" );
		if ( e.hostPerC64 )
		{
			scpuLineDecimal64( dst, capacity, length,
			                   e.hostCycle % e.hostPerC64 );
			scpuLineChar( dst, capacity, length, '/' );
			scpuLineDecimal64( dst, capacity, length,
			                   e.pacingAnchor % e.hostPerC64 );
			scpuLineChar( dst, capacity, length, '/' );
			scpuLineDecimal( dst, capacity, length, e.hostPerC64 );
		}
		else
			scpuLineText( dst, capacity, length, "--" );
		// THE DEC-CYCLE-FREE DECOMPOSITION. dprev= above is the physical spacing
		// between the two writes and bastall= is what was charged inside it, so
		// this is the REMAINDER: the real time from release to second write that
		// the freeze does not account for. VICE gives Mario hold = 59 - DECcycle
		// and the DEC/INC pair is six NMOS cycles of code, so the steal between
		// them is 53 - DECcycle; subtract and the DEC cycle CANCELS, leaving 6 at
		// every point of the reference sweep (45-39, 29-23, 9-3, 6-0). No raster
		// anchor, no assumed line start, no cycle-in-line. It must read 600.
		//   ~600  the release-to-write path is clean and the +3..+4 is NOT outside
		//         the charge. The live candidates are then an over-long freeze and
		//         the raster= reconstruction, and max(bastall=) over the $D011 INC
		//         events separates them: the reference bounds that steal by
		//         53 - 14 = 39, so a 42..46 there is the freeze running past the
		//         release
		//   ~900  three cycles of real time between release and write that were
		//         never charged -- the physical path, most likely
		//         HANDLE_BUS_AVAILABLE re-waiting a steal the emulated stall had
		//         already paid for. Handing stall cycles back would then be fixing
		//         the wrong end, straight out of the sled the stall repaired
		// WHAT IT CANNOT SEE, so it is not over-read: an over-long freeze leaves
		// this at 600, because the extra cycles sit inside BOTH terms -- we really
		// did spin for them and we really did charge them. It LOCATES the error,
		// it does not size it.
		//
		// The stall is SUMMED over the dprev span, because baStallCycles is
		// charged since the previous RETAINED event while dprev= is deliberately
		// since the previous write to the SAME register; an intervening $D016 or
		// $D018 write would otherwise move part of the freeze outside the field
		// and make it read ~900 for no reason but the handler's register order.
		// seq= adjacency is REQUIRED for that sum to be complete: the ring drops
		// events by overwriting them, and a dropped event's charge is lost from
		// the sum while its real time stays inside dprev. Unguarded that reads as
		// a clean ~900 -- the same silent failure the postrel= attribution flag
		// exists to prevent, so it gets the same treatment and prints "seq".
		scpuLineText( dst, capacity, length, " code=" );
		if ( prev && e.hostPerC64
		     && e.hostCycle >= d.vsp[ prev - 1 ].hostCycle )
		{
			if ( e.sequence - d.vsp[ prev - 1 ].sequence != i - ( prev - 1 ) )
				scpuLineText( dst, capacity, length, "seq" );
			else
			{
				u64 spanStall = 0;
				for ( u32 j = prev; j <= i; j++ )
					spanStall += d.vsp[ j ].baStallCycles;
				const s64 cc =
					(s64)( ( e.hostCycle - d.vsp[ prev - 1 ].hostCycle )
					       * 100u / e.hostPerC64 )
					- (s64)( spanStall * 100u );
				if ( cc < 0 )
				{
					scpuLineChar( dst, capacity, length, '-' );
					scpuLineDecimal64( dst, capacity, length, (u64)-cc );
				}
				else
					scpuLineDecimal64( dst, capacity, length, (u64)cc );
				scpuLineText( dst, capacity, length, "cc" );
			}
		}
		else
			scpuLineText( dst, capacity, length, "--" );
		// --- THE DISPLACEMENT FIELDS ---------------------------------------
		// stall= is the longest interval since the previous retained event in
		// which the emulated CPU executed nothing while the physical raster
		// kept moving, in host cycles and then in hundredths of a C64 cycle
		// like dprev= and postrel=. The trailing hex is the SCPU_GUEST_STOP_*
		// bitmask for that interval. 201 NTSC lines is about 1,300,000cc here,
		// so a stall= that explains the K478 raster=$FE events must read seven
		// figures; anything in the low thousands is a normal mirror
		// installment and is NOT a candidate, whatever its cause bits say.
		//
		// d012= is reads/first: how many times the guest read $D012 since the
		// previous retained event, and the value the FIRST of them returned.
		// raster= above is the LAST one, which is why this is the half that
		// settles it -- a handler whose poll walked down from the bottom
		// border reads many times starting near 248, while one entered 201
		// lines late reads a handful starting near 254.
		//
		// irq= is host cycles from the rising edge of the PHYSICAL /IRQ
		// assertion we observed to this write. The healthy $35 events fix the
		// normal handler-entry-to-VSP distance; a bad event matching it says
		// the hardware asserted late (so look at the raster compare, not at
		// our scheduler), and a bad event far above it says we were stopped
		// between the assertion and the handler -- which stall= must then
		// corroborate, or the two fields disagree and neither is believed.
		scpuLineText( dst, capacity, length, " stall=" );
		scpuLineDecimal( dst, capacity, length, e.stallHost );
		if ( e.hostPerC64 )
		{
			scpuLineChar( dst, capacity, length, 'h' );
			scpuLineChar( dst, capacity, length, '=' );
			scpuLineDecimal64( dst, capacity, length,
			                   (u64)e.stallHost * 100u / e.hostPerC64 );
			scpuLineText( dst, capacity, length, "cc" );
		}
		scpuLineChar( dst, capacity, length, '/' );
		scpuLineHexByte( dst, capacity, length, e.stallCause );
		scpuLineText( dst, capacity, length, " d012=" );
		scpuLineDecimal( dst, capacity, length, e.d012Reads );
		scpuLineChar( dst, capacity, length, '/' );
		scpuLineHexByte( dst, capacity, length, e.d012First );
		scpuLineText( dst, capacity, length, " irq=" );
		if ( e.irqToWriteHost == 0xFFFFFFFFu )
			scpuLineText( dst, capacity, length, "--" );
		else
		{
			scpuLineDecimal( dst, capacity, length, e.irqToWriteHost );
			scpuLineChar( dst, capacity, length, 'h' );
		}
		scpuLineText( dst, capacity, length, " factor=" );
		scpuLineDecimal( dst, capacity, length, e.factor );
		if ( e.address == 0xD011 )
		{
			// This is deliberately labelled as the nearest raster ANCHOR,
			// not the exact write line. The raw host stamps above let the
			// analysis reconstruct that line using the measured C64 clock.
			scpuLineText( dst, capacity, length,
			              " bl(anchor/ysc/match/range/den)=" );
			scpuLineDecimal( dst, capacity, length, e.raster & 7 );
			scpuLineChar( dst, capacity, length, '/' );
			scpuLineDecimal( dst, capacity, length, e.value & 7 );
			scpuLineChar( dst, capacity, length, '/' );
			scpuLineChar( dst, capacity, length,
			              ( e.flags & SCPU_VSP_ANCHOR_BADLINE_MOD ) ? 'Y' : 'N' );
			scpuLineChar( dst, capacity, length, '/' );
			scpuLineChar( dst, capacity, length,
			              ( e.flags & SCPU_VSP_ANCHOR_BADLINE_RANGE ) ? 'Y' : 'N' );
			scpuLineChar( dst, capacity, length, '/' );
			scpuLineChar( dst, capacity, length,
			              ( e.flags & SCPU_VSP_D011_DEN ) ? 'Y' : 'N' );
		}
		scpuLineText( dst, capacity, length, " sig=" );
		scpuLineHexByte( dst, capacity, length, e.d016 );
		scpuLineChar( dst, capacity, length, '/' );
		scpuLineHexByte( dst, capacity, length, e.d018 );
		scpuLineChar( dst, capacity, length, '/' );
		scpuLineHexByte( dst, capacity, length, e.gamePhase );
		scpuLineText( dst, capacity, length, "\r\n" );
	}
}

// Pacer totals since the last emulated reset, for both diagnostic files. A
// guest run ends with a button press and a console that has long scrolled
// away, so the SD files are the only place these can be read afterwards.
// count:avg/min/max, in ARM cycles. The average is emitted already computed
// because this line is read by eye first and by a parser second, and min/max
// are what distinguish a stage with a bimodal phase distribution from a slow
// one -- the distinction the whole K448 $D011 question turns on.
static void scpuLineStage( char *dst, u32 capacity, u32 &length,
                           const char *name, const RADBusStageStats &s )
{
	scpuLineText( dst, capacity, length, name );
	scpuLineDecimal64( dst, capacity, length, s.count );
	scpuLineChar( dst, capacity, length, ':' );
	scpuLineDecimal64( dst, capacity, length,
	                   s.count ? ( s.sum / s.count ) : 0 );
	scpuLineChar( dst, capacity, length, '/' );
	scpuLineDecimal( dst, capacity, length, s.count ? s.min : 0 );
	scpuLineChar( dst, capacity, length, '/' );
	scpuLineDecimal( dst, capacity, length, s.max );
}

static void scpuAppendStageDiag( char *dst, u32 capacity, u32 &length,
                                 const char *tag,
                                 const RADBusAccessStats &s )
{
	scpuLineText( dst, capacity, length, tag );
	scpuLineStage( dst, capacity, length, " rsync=", s.readResync );
	scpuLineStage( dst, capacity, length, " rvic=", s.readVIC );
	scpuLineStage( dst, capacity, length, " rcpu=", s.readCPU );
	scpuLineStage( dst, capacity, length, " wsync=", s.writeResync );
	scpuLineStage( dst, capacity, length, " wp1turn=", s.writeP1Turn );
	scpuLineStage( dst, capacity, length, " wp1plain=", s.writeP1Plain );
	scpuLineStage( dst, capacity, length, " wp2=", s.writeP2 );
	scpuLineText( dst, capacity, length, " bastall-r/w=" );
	scpuLineDecimal64( dst, capacity, length, s.readBAStalls );
	scpuLineChar( dst, capacity, length, '/' );
	scpuLineDecimal64( dst, capacity, length, s.writeBAStalls );
	// A separate token rather than a third field inside bastall-r/w: that key
	// is already quoted verbatim in CHT and in the K449 result, and a changed
	// shape would silently break any grep comparing a later run against those
	// numbers.
	scpuLineText( dst, capacity, length, " bastall-wturn=" );
	scpuLineDecimal64( dst, capacity, length, s.writeTurnBAStalls );
	scpuLineText( dst, capacity, length, "\r\n" );
}

static void scpuAppendPacerDiag( char *dst, u32 capacity, u32 &length )
{
	// Where a physical access spends its time, beside the pacer totals it has
	// to be read against. K448 measured the DEC/INC $D011 pair at 10.99 C64
	// cycles of real time in its clean mode for 6 charged; these stages break
	// the unbudgeted 6830 ARM out by PHI entry phase. The io: line is
	// $D000-$DFFF only, which is the traffic the artifact is made of; the all:
	// line averages in the RAM and ROM accesses that outnumber it, and is the
	// one to read for Problem B throughput.
	//
	// Calibrated here rather than lazily from an access tail: nothing is on
	// the bus while a diagnostic file is being formatted, so the 128 counter
	// reads cannot land between two transfers and the primitives keep no
	// per-access branch for it. stamp= bounds every figure below it.
	// THE GUEST-STOPPED LEDGER, read off the ledger CC64Memory ALREADY keeps.
	// noteGuestStopped()/noteGuestResumed() bracket every interpreter exit in
	// runFrame(), and the stop that stays open across the frame hook is tagged
	// SCPU_GUEST_STOP_FRAMEEND -- so the phase this investigation cares about is
	// already instrumented and no second ledger is needed. What was missing is a
	// figure that OUTLIVES the VSP appender, which drains m_GuestStopMaxHost into
	// every retained event and zeroes it; a run whose worst stop is not followed
	// by a $D011 write therefore lost it. peak= is that same measurement, never
	// drained.
	//
	// peak= is host cycles at arm_freq=1400, so one jiffy is 23,333,333 and the
	// ~12.7ms displacement to explain is ~17,800,000. cause= is the
	// SCPU_GUEST_STOP_* mask for that peak, and it is what discriminates the two
	// candidate mechanisms: $40 (FRAMEEND) alone points at the frame hook and so
	// at a console write, while $02/$01 (BORDER/FLUSH) points at a mirror drain
	// paying RAD_BA_WAIT_LIMIT. buswait= is reports/firstBA/firstPHI from the
	// latch in scpuCheckButton, which is what makes the second reading possible.
	scpuLineText( dst, capacity, length, "guest-stopped: peak=" );
	scpuLineDecimal( dst, capacity, length,
	                 superCPU.memory().guestStopPeakHost() );
	scpuLineText( dst, capacity, length, " cause=$" );
	scpuLineHexByte( dst, capacity, length,
	                 (u8)superCPU.memory().guestStopPeakCauses() );
	scpuLineText( dst, capacity, length, " buswait=" );
	scpuLineDecimal( dst, capacity, length, s_BusWaitTimeoutReports );
	scpuLineChar( dst, capacity, length, '/' );
	scpuLineDecimal( dst, capacity, length, s_BusWaitTimeoutFirstBA );
	scpuLineChar( dst, capacity, length, '/' );
	scpuLineDecimal( dst, capacity, length, s_BusWaitTimeoutFirstPHI );
	scpuLineText( dst, capacity, length, "\r\n" );
	radCalibrateStampCost();
	scpuLineText( dst, capacity, length, "busstage: stamp=" );
	scpuLineDecimal( dst, capacity, length, radBusStampCost );
	scpuLineText( dst, capacity, length, "\r\n" );
	scpuAppendStageDiag( dst, capacity, length, "busstage all:",
	                     radBusStages );
	scpuAppendStageDiag( dst, capacity, length, "busstage io:",
	                     radBusStagesIO );
	scpuLineText( dst, capacity, length, "pacer: wait-host=" );
	scpuLineDecimal64( dst, capacity, length,
	                   superCPU.memory().m_PacerWaitHostCycles );
	scpuLineText( dst, capacity, length, " late-1mhz-entries=" );
	scpuLineDecimal64( dst, capacity, length,
	                   superCPU.memory().m_PacerLateEntries );
	scpuLineText( dst, capacity, length, " late-1mhz-probes=" );
	scpuLineDecimal64( dst, capacity, length,
	                   superCPU.memory().m_PacerLateProbes );
	scpuLineText( dst, capacity, length, " late-snap-host=" );
	scpuLineDecimal64( dst, capacity, length,
	                   superCPU.memory().m_PacerLateSnapHostCycles );
	scpuLineText( dst, capacity, length, " credit-entries=" );
	scpuLineDecimal64( dst, capacity, length,
	                   superCPU.memory().m_PacerCreditEntries );
	scpuLineText( dst, capacity, length, " credit-host=" );
	scpuLineDecimal64( dst, capacity, length,
	                   superCPU.memory().m_PacerCreditHostCycles );
	scpuLineText( dst, capacity, length, " credit-clamped=" );
	scpuLineDecimal64( dst, capacity, length,
	                   superCPU.memory().m_PacerClampEntries );
	scpuLineText( dst, capacity, length, " slow-cycles=" );
	scpuLineDecimal64( dst, capacity, length,
	                   superCPU.memory().m_SlowPacedCycles );
	scpuLineText( dst, capacity, length, "\r\n" );

	// The emulated BA stall, as a population that can be divided: events are
	// freezes and cycles are what the guest was charged for them, so
	// cycles/events is the MEAN STEAL and has to land near 43 on a badline
	// machine. iec-skipped counts the samples mode 1 declined inside a serial
	// transaction -- if a load misbehaves, that number is the first thing to
	// read, because a zero there means the suppression never engaged.
	// pin-events/pin-cycles come from the bus side, so the two must agree
	// exactly: both sides count only freezes with stolen >= 1, so a difference
	// means samples were taken and then not charged.
	//
	// AND A DIVISOR WARNING, because this file's own populations are easy to
	// mix: slow-cycles above is incremented in tickFast from the INSTRUCTION's
	// cycles only, so it does NOT include these stalled cycles, while
	// m_EmuCycles and c64Now() DO. Any "ARM per slow cycle" figure computed
	// across a run with the stall engaged is therefore over a different
	// population than the K448/K449 figures archived in CHT. Add
	// bastall-emu cycles= to slow-cycles before comparing, or do not compare.
	scpuLineText( dst, capacity, length, "bastall-emu: events=" );
	scpuLineDecimal64( dst, capacity, length,
	                   superCPU.memory().m_BAStallEvents );
	scpuLineText( dst, capacity, length, " cycles=" );
	scpuLineDecimal64( dst, capacity, length,
	                   superCPU.memory().m_BAStallCycles );
	scpuLineText( dst, capacity, length, " iec-skipped=" );
	scpuLineDecimal64( dst, capacity, length,
	                   superCPU.memory().m_BAStallSuppressedIEC );
	scpuLineText( dst, capacity, length, " pin-events=" );
	scpuLineDecimal64( dst, capacity, length, radBus.m_BAStallEvents );
	scpuLineText( dst, capacity, length, " pin-cycles=" );
	scpuLineDecimal64( dst, capacity, length, radBus.m_BAStallCycles );
	scpuLineText( dst, capacity, length, " capped=" );
	scpuLineDecimal64( dst, capacity, length, radBus.m_BAStallCaps );
	scpuLineText( dst, capacity, length, " faulted=" );
	scpuLineDecimal( dst, capacity, length, radBus.m_BAStallFaulted ? 1 : 0 );
	// Re-entry spacing: freezes that began within 8 C64 cycles of the previous
	// release, the cycles they were charged, how many were within 2 cycles, and
	// how many were sprite-class in LENGTH. Read against events= above.
	//
	// b2b near zero means every freeze was a cleanly separated BA-low run, each
	// charged exactly what the guest lost, so no merged sprite lead-in can be
	// inflating the hold and the physical path is where to look. b2b-tight is
	// deliberately the WEAKER of the two: we sample BA only at an access
	// boundary, so a re-entry sooner than one guest instruction is unobservable
	// and a tight zero on its own proves nothing. b2b-cycles is the exact size
	// of the only correction this mechanism would justify.
	scpuLineText( dst, capacity, length, " b2b=" );
	scpuLineDecimal64( dst, capacity, length,
	                   superCPU.memory().m_BAStallB2B );
	scpuLineText( dst, capacity, length, " b2b-cycles=" );
	scpuLineDecimal64( dst, capacity, length,
	                   superCPU.memory().m_BAStallB2BCycles );
	scpuLineText( dst, capacity, length, " b2b-tight=" );
	scpuLineDecimal64( dst, capacity, length,
	                   superCPU.memory().m_BAStallB2BTight );
	scpuLineText( dst, capacity, length, " b2b-short=" );
	scpuLineDecimal64( dst, capacity, length,
	                   superCPU.memory().m_BAStallB2BShort );
	// THE RASTER-WINDOW CATCH-UP READOUT, and it is not optional: the whole
	// first reading of that mechanism is "did it fire at all". catchup=0 means
	// the window was never open at factor 1, so the write is LATE rather than
	// early and the repayment diagnosis is dead -- which is free information
	// worth a card round on its own, and completely invisible without this.
	//
	// Read the MEAN, catchup-host divided by catchup, in ARM cycles against
	// the grid= denominator: near one C64 cycle says the repaid lag came from
	// the handler's own physical accesses; near six says a slice-boundary
	// rasterLine() landed inside the window, which is the recorded dissent and
	// points at a different fix. declined= counts deficits above the 8-cycle
	// cap that were refused rather than spun out; a large declined with a
	// small catchup means the cap is in the wrong place, not that the
	// mechanism is absent.
	scpuLineText( dst, capacity, length, " catchup=" );
	scpuLineDecimal64( dst, capacity, length,
	                   superCPU.memory().m_VSPCatchupEvents );
	scpuLineText( dst, capacity, length, " catchup-host=" );
	scpuLineDecimal64( dst, capacity, length,
	                   superCPU.memory().m_VSPCatchupHost );
	scpuLineText( dst, capacity, length, " catchup-declined=" );
	scpuLineDecimal64( dst, capacity, length,
	                   superCPU.memory().m_VSPCatchupDeclined );
	// d011shadow=<served>/<declined> (K500): $D011 reads served from the
	// shadow with no transfer, and reads inside the window the anchor could
	// not vouch for. 0/0 with the key on means the window never opened.
	scpuLineText( dst, capacity, length, " d011shadow=" );
	scpuLineDecimal64( dst, capacity, length,
	                   superCPU.memory().m_VSPD011ShadowReads );
	scpuLineChar( dst, capacity, length, '/' );
	scpuLineDecimal64( dst, capacity, length,
	                   superCPU.memory().m_VSPD011ShadowDeclined );
	// irqhist= / anchist= (K503): run-wide, first $D011 write after each
	// anchor read. irqhist buckets are whole cycles of IRQ->write from -5 to
	// +5 relative to the first sample (irqbase=, host cycles); anchist buckets
	// are the anchor's low byte -3..+3 relative to the first (ancbase=). With
	// a working absorber, irqhist has two adjacent populations and anchist
	// two adjacent values; any count elsewhere is an unabsorbed frame.
	// Absolute (K504): irqhist=<cycles>:<count>,... for the non-zero whole-cycle
	// buckets of IRQ->first placed $D011 write (0..127), then anchist=
	// <anchor byte>:<count>,... A working absorber shows two adjacent cycle
	// buckets of similar size and two adjacent anchor bytes; anything else is
	// an unabsorbed frame.
	scpuLineText( dst, capacity, length, " irqhist=" );
	{
		bool first = true;
		for ( u32 i = 0; i < 128; i++ )
		{
			const u32 n = superCPU.memory().m_VSPHistIrq[ i ];
			if ( !n ) continue;
			if ( !first ) scpuLineChar( dst, capacity, length, ',' );
			first = false;
			scpuLineDecimal( dst, capacity, length, i );
			scpuLineChar( dst, capacity, length, ':' );
			scpuLineDecimal64( dst, capacity, length, n );
		}
	}
	scpuLineText( dst, capacity, length, " irqhist-128plus=" );
	scpuLineDecimal64( dst, capacity, length,
	                   superCPU.memory().m_VSPHistIrqOther );
	scpuLineText( dst, capacity, length, " anchist=" );
	{
		bool first = true;
		for ( u32 i = 0; i < 256; i++ )
		{
			const u32 n = superCPU.memory().m_VSPHistAnchor[ i ];
			if ( !n ) continue;
			if ( !first ) scpuLineChar( dst, capacity, length, ',' );
			first = false;
			scpuLineDecimal( dst, capacity, length, i );
			scpuLineChar( dst, capacity, length, ':' );
			scpuLineDecimal64( dst, capacity, length, n );
		}
	}
	// Mirror opportunities skipped because the guest was inside a cycle-counted
	// raster window. This is the did-it-fire readout for the raster-critical
	// guard, and it is the FIRST thing to read if the screen does not change:
	// zero means the window was never recognised and the guard is inert, which
	// is a different failure from the guard firing and not helping.
	scpuLineText( dst, capacity, length, " rdefer=" );
	scpuLineDecimal64( dst, capacity, length, superCPU.m_RasterDeferred );
	scpuLineText( dst, capacity, length, "\r\n" );

	// THE DISTRIBUTION, on its own line so nothing that parses the line above
	// has to change. 16 bins of 5 C64 cycles, 0-4 first, 75+ last. Read this
	// instead of cycles/events: a sprite steal is 2-3 (bin 0), eight sprites
	// are 3+2N=19 (bin 3) and a badline is 43 (bin 8), so K456's 31.6 mean
	// describes no event that ever happened. Bin 15 holds the tail censored by
	// RAD_BA_STALL_CEILING -- compare it with capped= above before dividing
	// anything by anything.
	scpuLineText( dst, capacity, length, "bastall-hist5:" );
	for ( u32 bin = 0; bin < 16; bin++ )
	{
		scpuLineChar( dst, capacity, length, bin ? ',' : ' ' );
		scpuLineDecimal( dst, capacity, length,
		                 superCPU.memory().m_BAStallHist[ bin ] );
	}
	scpuLineText( dst, capacity, length, "\r\n" );

	// ARMED TIME AGAINST TRANSACTION TIME, which iec-skipped above could only
	// imply. All three are emulated-cycle counts over the same run, so each
	// divides by bastall slow-cycles + bastall cycles to give its armed
	// FRACTION -- and they are not disjoint: poll and hold can both be armed.
	//   hold=   the 100ms speed hold. This is the one measured at 98% during
	//           Mario gameplay with no drive access, and the only one of the
	//           three that can be armed by a static level.
	//   poll=   the 1000-cycle top-up, counted only where it forces the rate.
	//   txn=    the 2ms EDGE window. This is TRANSACTION time: with no drive in
	//           the conversation it should be near zero, and hold/txn is the
	//           over-arm ratio stated directly.
	// arms are the events, by the evidence that caused them. read-refused is
	// what mode 0 WOULD have armed on and this mode did not, so a run with
	// read-refused large and hold small is the direct proof that the level arm
	// was the over-arm. latch/ddra are printed because an over-arm capture is
	// only actionable with the CIA2 output state that produced it: bit 4 or 5
	// set there means the guest was pulling CLK or DATA down itself.
	scpuLineText( dst, capacity, length, "iecwin: mode=" );
	scpuLineDecimal( dst, capacity, length,
	                 (int)superCPU.memory().m_IECWindowMode );
	scpuLineText( dst, capacity, length, " hold=" );
	scpuLineDecimal64( dst, capacity, length,
	                   superCPU.memory().m_IECHoldArmedCycles );
	scpuLineText( dst, capacity, length, " poll=" );
	scpuLineDecimal64( dst, capacity, length,
	                   superCPU.memory().m_IECPollArmedCycles );
	scpuLineText( dst, capacity, length, " txn=" );
	scpuLineDecimal64( dst, capacity, length,
	                   superCPU.memory().m_IECActivityArmedCycles );
	scpuLineText( dst, capacity, length, " arms: edge=" );
	scpuLineDecimal64( dst, capacity, length,
	                   superCPU.memory().m_IECArmReadEdge );
	scpuLineText( dst, capacity, length, " level=" );
	scpuLineDecimal64( dst, capacity, length,
	                   superCPU.memory().m_IECArmReadLevel );
	scpuLineText( dst, capacity, length, " read-refused=" );
	scpuLineDecimal64( dst, capacity, length,
	                   superCPU.memory().m_IECArmReadRefused );
	scpuLineText( dst, capacity, length, " write=" );
	scpuLineDecimal64( dst, capacity, length,
	                   superCPU.memory().m_IECArmWrite );
	scpuLineText( dst, capacity, length, " write-refused=" );
	scpuLineDecimal64( dst, capacity, length,
	                   superCPU.memory().m_IECArmWriteRefused );
	scpuLineText( dst, capacity, length, " poll-arms=" );
	scpuLineDecimal64( dst, capacity, length,
	                   superCPU.memory().m_IECPollArms );
	scpuLineText( dst, capacity, length, " latch=" );
	scpuLineHexByte( dst, capacity, length,
	                 superCPU.memory().cia2PortALatch() );
	scpuLineText( dst, capacity, length, " ddra=" );
	scpuLineHexByte( dst, capacity, length, superCPU.memory().cia2DDRA() );
	scpuLineText( dst, capacity, length, "\r\n" );
}

// The CPU post-mortem was previously formatted only into s_Logger by
// scpuEmitResetDiagnostic(), which the LEFT-button path never calls and whose
// output the reboot destroys anyway.  S after a hang is the single most
// diagnostic byte in the machine, so it has to reach the card.
// One printer for both rings. The live ring and the progress detector's SUSPECT
// copy are the same structure and must format identically, or a reader comparing
// the transition against the steady state has to normalise two layouts by hand.
// target= is the handler this interrupt ENTERED: with Mario re-pointing $FFFE
// three times a frame, the entry PC alone cannot say which rung ran.
static void scpuAppendOneInterruptRing(
	char *dst, u32 capacity, u32 &length, const char *tag,
	const CW65C816::W65InterruptEntry *entries, u32 head )
{
	const u32 ring = CW65C816::W65_INT_RING;
	const u32 n = head < ring ? head : ring;
	for ( u32 i = 0; i < n; i++ )
	{
		const CW65C816::W65InterruptEntry &e = entries[ ( head - n + i ) % ring ];
		scpuLineText( dst, capacity, length, "  " );
		scpuLineText( dst, capacity, length, tag );
		scpuLineChar( dst, capacity, length, ' ' );
		scpuLineDecimal( dst, capacity, length, i );
		scpuLineText( dst, capacity, length,
		              e.nmi ? " NMI pc=$" : " IRQ pc=$" );
		scpuLineHexU32( dst, capacity, length, e.pc );
		scpuLineText( dst, capacity, length, " ->$" );
		scpuLineHexByte( dst, capacity, length, (u8)( e.target >> 8 ) );
		scpuLineHexByte( dst, capacity, length, (u8)( e.target & 0xFF ) );
		scpuLineText( dst, capacity, length, " s=$" );
		scpuLineHexByte( dst, capacity, length, (u8)( e.s >> 8 ) );
		scpuLineHexByte( dst, capacity, length, (u8)( e.s & 0xFF ) );
		scpuLineText( dst, capacity, length, " p=$" );
		scpuLineHexByte( dst, capacity, length, e.p );
		scpuLineText( dst, capacity, length, " cyc=" );
		scpuLineDecimal( dst, capacity, length, e.cycles );
		// WHY this interrupt happened, as far as it can be seen from inside.
		// cmp= is OUR decode of the armed compare and ras= is the guest's own
		// last physical $D012 read with its age in WHOLE C64 CYCLES -- not the
		// hundredths dprev= and stall= print in the VSP trace below. An entry
		// whose ras= is nowhere near its cmp= was not let in by the compare
		// this field names.
		scpuLineText( dst, capacity, length, " cmp=" );
		scpuLineDecimal( dst, capacity, length, e.compare );
		scpuLineText( dst, capacity, length, " mask=$" );
		scpuLineHexByte( dst, capacity, length, e.d01a );
		scpuLineText( dst, capacity, length, " ras=" );
		if ( e.iflags & SCPU_IRQCTX_RASTER_VALID )
		{
			scpuLineDecimal( dst, capacity, length, e.raster );
			scpuLineChar( dst, capacity, length, '@' );
			if ( e.rasterAge4 == 0xFF )
				scpuLineText( dst, capacity, length, "--" );
			else
				scpuLineDecimal( dst, capacity, length,
				                 (u32)e.rasterAge4 * 4 );
			scpuLineChar( dst, capacity, length, 'c' );
		}
		else
			scpuLineText( dst, capacity, length, "--" );
		// L sampled level, F freshly sampled at this boundary, A an ack since
		// the previous entry left /IRQ low immediately, K it was still low at
		// the next control event, R the raster anchor is valid, X the armed
		// compare cannot be reached on this machine, C the asserted line was
		// carried in from an earlier sample rather than found at this
		// boundary -- see SCPU_IRQCTX_LINE_CARRIED for how L, F and C are
		// read together.
		scpuLineText( dst, capacity, length, " i=" );
		{
			static const char letters[ 7 ] =
				{ 'L', 'F', 'A', 'K', 'R', 'X', 'C' };
			for ( u32 b = 0; b < 7; b++ )
				scpuLineChar( dst, capacity, length,
				              ( e.iflags & ( 1u << b ) ) ? letters[ b ] : '-' );
		}
		scpuLineText( dst, capacity, length, "\r\n" );
	}
}

// --- the interrupt-control write ring ---------------------------------------
// ONE LINE PER WRITE to $D012, $D019, $D01A or a raster-bit-8 $D011, newest
// last, with the writer's PC and a cycle stamp in the same clock domain the
// interrupt ring above stamps -- so "which rung armed this compare" is a join
// on cyc= and not a guess.
//
// HOW TO READ IT, and this is the whole point of the capture:
//   cmp= is OUR composition. val=/other= are the bytes that reached the chip.
//   Recompose ( other & $80 ? 256 : 0 ) | low-half and check it against cmp=
//   before believing either. A '!' on cmp= marks a compare this machine cannot
//   reach, which is where K471's 318 would appear.
//   gras= is the guest's own last physical $D012 read; pras= is the physical
//   $D012 read d011RasterHighBit() already makes when a guest $D011 read comes
//   back claiming bit 8. BOTH ages are WHOLE C64 CYCLES. "--" is no
//   observation, not a small one.
//   irq=BAL: the cached /IRQ level Before the write, the pin sampled After it
//   (only ever taken on $D019, '-' elsewhere), and the cached level at the
//   next control event, Late, with lateh= the host cycles to that sample.
//   A $D019 line reading irq=1 1 1 is a plain store that did not clear the
//   source -- and that reproduces the short entry gap and the nesting with
//   every compare in this ring perfectly correct.
static void scpuAppendOneIntCtlRing(
	char *dst, u32 capacity, u32 &length, const char *tag,
	const VICIntCtlWrite *entries, u32 head, u32 limit )
{
	const u32 retained = head < SCPU_ICTL_RING ? head : SCPU_ICTL_RING;
	const u32 n = retained < limit ? retained : limit;
	for ( u32 i = 0; i < n; i++ )
	{
		const VICIntCtlWrite &e =
			entries[ ( head - n + i ) & ( SCPU_ICTL_RING - 1 ) ];
		scpuLineText( dst, capacity, length, "  " );
		scpuLineText( dst, capacity, length, tag );
		scpuLineChar( dst, capacity, length, ' ' );
		scpuLineDecimal( dst, capacity, length, head - n + i );
		scpuLineText( dst, capacity, length, " w$D0" );
		scpuLineHexByte( dst, capacity, length, e.reg );
		scpuLineText( dst, capacity, length, "=$" );
		scpuLineHexByte( dst, capacity, length, e.value );
		scpuLineText( dst, capacity, length, " prev=$" );
		scpuLineHexByte( dst, capacity, length, e.prev );
		scpuLineText( dst, capacity, length, " other=$" );
		scpuLineHexByte( dst, capacity, length, e.other );
		scpuLineText( dst, capacity, length, " cmp=" );
		scpuLineDecimal( dst, capacity, length, e.compare );
		if ( e.flags & SCPU_ICTL_COMPARE_IMPOSSIBLE )
			scpuLineChar( dst, capacity, length, '!' );
		scpuLineText( dst, capacity, length, " mask=$" );
		scpuLineHexByte( dst, capacity, length, e.d01a );
		scpuLineText( dst, capacity, length, " by=$" );
		scpuLineHexU32( dst, capacity, length, e.pc );
		scpuLineText( dst, capacity, length, " cyc=" );
		scpuLineDecimal( dst, capacity, length, e.cycles );
		scpuLineText( dst, capacity, length, " gras=" );
		if ( ( e.flags & SCPU_ICTL_GRASTER_VALID ) && e.grasterAge != 0xFFFF )
		{
			scpuLineDecimal( dst, capacity, length, e.graster );
			scpuLineChar( dst, capacity, length, '@' );
			scpuLineDecimal( dst, capacity, length, e.grasterAge );
			scpuLineChar( dst, capacity, length, 'c' );
		}
		else
			scpuLineText( dst, capacity, length, "--" );
		scpuLineText( dst, capacity, length, " pras=" );
		if ( ( e.flags & SCPU_ICTL_PRASTER_VALID ) && e.prasterAge != 0xFFFF )
		{
			scpuLineDecimal( dst, capacity, length, e.praster );
			scpuLineChar( dst, capacity, length, '@' );
			scpuLineDecimal( dst, capacity, length, e.prasterAge );
			scpuLineChar( dst, capacity, length, 'c' );
		}
		else
			scpuLineText( dst, capacity, length, "--" );
		scpuLineText( dst, capacity, length, " irq=" );
		scpuLineChar( dst, capacity, length,
		              ( e.flags & SCPU_ICTL_IRQ_BEFORE ) ? '1' : '0' );
		scpuLineChar( dst, capacity, length,
		              !( e.flags & SCPU_ICTL_IRQ_AFTER_VALID ) ? '-'
		              : ( e.flags & SCPU_ICTL_IRQ_AFTER ) ? '1' : '0' );
		scpuLineChar( dst, capacity, length,
		              !( e.flags & SCPU_ICTL_IRQ_LATE_VALID ) ? '-'
		              : ( e.flags & SCPU_ICTL_IRQ_LATE ) ? '1' : '0' );
		scpuLineText( dst, capacity, length, " lateh=" );
		if ( e.lateHost != 0xFFFF )
			scpuLineDecimal( dst, capacity, length, e.lateHost );
		else
			scpuLineText( dst, capacity, length, "--" );
		scpuLineText( dst, capacity, length, "\r\n" );
	}
}

static void scpuAppendIntCtlDiag( char *dst, u32 capacity, u32 &length )
{
	const SCPUResetDiagnostic &d = s_ResetDiagnostic;
	// THE COUNTERS FIRST, because they are the cheap answer to the leading
	// structural suspect and they are meaningful even if both rings lapped.
	// ack-held counts acknowledgements after which the physical /IRQ pin was
	// STILL asserted; late= counts those still asserted at the next control
	// event, well past any pull-up propagation. acks= against the irqs= on the
	// post-mortem line above is the count test CODEX asked for: one plain-store
	// ack per interrupt entry is the healthy shape.
	scpuLineText( dst, capacity, length, "  ictl: acks=" );
	scpuLineDecimal( dst, capacity, length, d.d019Writes );
	scpuLineText( dst, capacity, length, " ack-held=" );
	scpuLineDecimal( dst, capacity, length, d.d019AckHeld );
	scpuLineText( dst, capacity, length, " ack-held-late=" );
	scpuLineDecimal( dst, capacity, length, d.d019AckHeldLate );
	scpuLineText( dst, capacity, length, " writes=" );
	scpuLineDecimal( dst, capacity, length, d.ictlHead );
	scpuLineText( dst, capacity, length, " latch=" );
	// 1 = the first impossible compare, which is this hang's measured
	// signature and locks the copy. 2 = a progress-detector SUSPECT episode,
	// which knows nothing about rasters and is re-copied every episode.
	scpuLineDecimal( dst, capacity, length, d.ictlFrozenReason );
	scpuLineText( dst, capacity, length, " at-cyc=" );
	scpuLineDecimal( dst, capacity, length, d.ictlFrozenCycles );
	scpuLineText( dst, capacity, length, " ages=C64cyc\r\n" );
	if ( d.ictlFrozenReason )
		scpuAppendOneIntCtlRing( dst, capacity, length, "ictlF",
		                         d.ictlFrozen, d.ictlFrozenHead,
		                         SCPU_ICTL_RING );
	// The live tail as well: the latched copy holds the failure, this holds
	// what the machine is doing now, and the entry the latch caught last has
	// no late sample in the frozen copy but does have one here.
	scpuAppendOneIntCtlRing( dst, capacity, length, "ictl",
	                         d.ictl, d.ictlHead, 48 );

	// --- the CIA conversation, newest last ---------------------------------
	// ON THE CARD, which is new: this ring was already captured but only ever
	// printed to the logger, where the reboot destroys it and the LEFT-button
	// path never looks.
	//
	// IT IS HERE BECAUSE IT IS THE ONLY NON-DESTRUCTIVE CIA EVIDENCE WE HAVE,
	// and the limit it describes is as important as the data. $DC0D and $DD0D
	// CLEAR ON READ, so nothing in this firmware may sample CIA interrupt
	// status for observation -- doing so would destroy the state the guest's
	// own handler is about to act on. What this block therefore carries is:
	// every CIA WRITE (w), including the $DC0D mask writes that say whether a
	// CIA interrupt was ever enabled, and every CIA READ (r) the GUEST itself
	// made, with the value it got -- edge-compressed, so a polling loop shows
	// once per change. If the guest reads $DC0D in its handler, that value IS
	// the CIA's interrupt status at that moment and it cost us nothing. If it
	// does not, a VIC raster IRQ and a CIA IRQ are indistinguishable to this
	// firmware on the shared open-collector /IRQ line, and the next instrument
	// is a scope on /IRQ rather than more code.
	scpuLineText( dst, capacity, length, "  cia (w=write r=guest read, "
	              "newest last, reads edge-compressed):\r\n    " );
	for ( u32 i = 32; i < 64; i++ )
	{
		const u32 e = d.cia[ i ];
		if ( !e ) continue;
		scpuLineChar( dst, capacity, length, ( e >> 24 ) ? 'w' : 'r' );
		scpuLineChar( dst, capacity, length, '$' );
		scpuLineHexByte( dst, capacity, length, (u8)( ( e >> 8 ) & 0xFF ) );
		scpuLineHexByte( dst, capacity, length, (u8)( e & 0xFF ) );
		scpuLineChar( dst, capacity, length, '=' );
		scpuLineHexByte( dst, capacity, length, (u8)( ( e >> 16 ) & 0xFF ) );
		scpuLineChar( dst, capacity, length, ' ' );
		if ( i == 47 )
			scpuLineText( dst, capacity, length, "\r\n    " );
	}
	scpuLineText( dst, capacity, length, "\r\n" );
}

static void scpuAppendInterruptRingDiag( char *dst, u32 capacity, u32 &length )
{
	const SCPUResetDiagnostic &d = s_ResetDiagnostic;
	if ( !d.haveCore )
	{
		scpuLineText( dst, capacity, length,
		              "post-mortem: no core snapshot available\r\n" );
		return;
	}
	scpuLineText( dst, capacity, length, "post-mortem: PC=$" );
	scpuLineHexU32( dst, capacity, length, d.pc );
	scpuLineText( dst, capacity, length, " S=$" );
	scpuLineHexByte( dst, capacity, length, (u8)( d.sp >> 8 ) );
	scpuLineHexByte( dst, capacity, length, (u8)( d.sp & 0xFF ) );
	scpuLineText( dst, capacity, length, " P=$" );
	scpuLineHexByte( dst, capacity, length, d.p );
	scpuLineText( dst, capacity, length, " I=" );
	scpuLineDecimal( dst, capacity, length, ( d.p & W65_I ) ? 1 : 0 );
	scpuLineText( dst, capacity, length, " E=" );
	scpuLineDecimal( dst, capacity, length, d.e );
	scpuLineText( dst, capacity, length, " irqs=" );
	scpuLineDecimal( dst, capacity, length, d.irqsTaken );
	scpuLineText( dst, capacity, length, " nmis=" );
	scpuLineDecimal( dst, capacity, length, d.nmisTaken );
	scpuLineText( dst, capacity, length, " line-irq=" );
	scpuLineDecimal( dst, capacity, length, d.liveIRQ ? 1 : 0 );
	// now-cyc and frozen-at-cyc are the same counter (cpu()->cycles() is
	// m_Cycles, which is what the ring stamps). A freeze whose stamp is far
	// behind now-cyc happened during loading, not at the hang, and that run is
	// void rather than a confirmation.
	scpuLineText( dst, capacity, length, "\r\n  now-cyc=" );
	scpuLineDecimal64( dst, capacity, length, d.cycles );
	scpuLineText( dst, capacity, length, " stack-anomalies=" );
	scpuLineDecimal( dst, capacity, length, d.stackAnomalies );
	scpuLineText( dst, capacity, length, " ring-frozen=" );
	scpuLineDecimal( dst, capacity, length, d.intRingFrozen );
	scpuLineText( dst, capacity, length, " frozen-at-cyc=" );
	scpuLineDecimal( dst, capacity, length, d.intRingFrozenCycles );
	scpuLineText( dst, capacity, length, "\r\n  vectors: FFFA=$" );
	scpuLineHexByte( dst, capacity, length, d.vectors[ 1 ] );
	scpuLineHexByte( dst, capacity, length, d.vectors[ 0 ] );
	scpuLineText( dst, capacity, length, " FFFC=$" );
	scpuLineHexByte( dst, capacity, length, d.vectors[ 3 ] );
	scpuLineHexByte( dst, capacity, length, d.vectors[ 2 ] );
	scpuLineText( dst, capacity, length, " FFFE=$" );
	scpuLineHexByte( dst, capacity, length, d.vectors[ 5 ] );
	scpuLineHexByte( dst, capacity, length, d.vectors[ 4 ] );
	scpuLineText( dst, capacity, length, " 0314=$" );
	scpuLineHexByte( dst, capacity, length, d.softVectors[ 1 ] );
	scpuLineHexByte( dst, capacity, length, d.softVectors[ 0 ] );
	scpuLineText( dst, capacity, length, "\r\n  stack: " );
	for ( u32 i = 0; i < 16; i++ )
	{
		scpuLineHexByte( dst, capacity, length, d.stack[ i ] );
		scpuLineChar( dst, capacity, length, ' ' );
	}
	if ( d.haveCode )
	{
		scpuLineText( dst, capacity, length, "\r\n  code@PC-4: " );
		for ( u32 i = 0; i < 16; i++ )
		{
			scpuLineHexByte( dst, capacity, length, d.code[ i ] );
			scpuLineChar( dst, capacity, length, ' ' );
		}
	}
	scpuLineText( dst, capacity, length, "\r\n" );

	// --- the hypothesis-free detector's own verdict -----------------------
	// Printed whether or not it tripped, and that is deliberate: the 2026-10-12
	// run reported stack-anomalies=0 on a visibly hung machine, and a silent
	// instrument is indistinguishable from a working one unless it says what it
	// measured. epochs/stall-max say how close it came even when it did not
	// fire, which is the number that calibrates the confirm threshold.
	scpuLineText( dst, capacity, length, "  progress: confirmed=" );
	scpuLineDecimal( dst, capacity, length, d.progConfirmed );
	scpuLineText( dst, capacity, length, " suspects=" );
	scpuLineDecimal( dst, capacity, length, d.progSuspects );
	scpuLineText( dst, capacity, length, " epochs=" );
	scpuLineDecimal( dst, capacity, length, d.progEpochs );
	scpuLineText( dst, capacity, length, " stall-max=" );
	scpuLineDecimal( dst, capacity, length, d.progStallEpochsMax );
	scpuLineText( dst, capacity, length, " suspect-cyc=" );
	scpuLineDecimal( dst, capacity, length, d.progSuspectCycles );
	scpuLineText( dst, capacity, length, " confirm-cyc=" );
	scpuLineDecimal( dst, capacity, length, d.progConfirmCycles );
	scpuLineText( dst, capacity, length, " cells=" );
	scpuLineDecimal( dst, capacity, length, d.progCellCount );
	scpuLineText( dst, capacity, length, " pc=$" );
	scpuLineHexByte( dst, capacity, length, (u8)( d.progPC >> 8 ) );
	scpuLineHexByte( dst, capacity, length, (u8)( d.progPC & 0xFF ) );
	if ( d.progConfirmed && d.progCellCount )
	{
		// EVERY 16-byte code cell the machine was still executing in the
		// stalled epoch. This is the whole hang, stated as a set of addresses,
		// with no theory about why. Capped at W65_PROG_CELL_REPORT; cells=
		// above is the untruncated count.
		scpuLineText( dst, capacity, length, "\r\n  live code cells:" );
		const u32 shown = d.progCellCount < CW65C816::W65_PROG_CELL_REPORT
		                ? d.progCellCount : CW65C816::W65_PROG_CELL_REPORT;
		for ( u32 i = 0; i < shown; i++ )
		{
			const u32 addr = (u32)d.progCells[ i ] << 4;
			scpuLineText( dst, capacity, length, " $" );
			scpuLineHexByte( dst, capacity, length, (u8)( addr >> 8 ) );
			scpuLineHexByte( dst, capacity, length, (u8)( addr & 0xFF ) );
		}
	}
	scpuLineText( dst, capacity, length, "\r\n" );

	// --- the watched RAM bytes -------------------------------------------
	// (a) never written, (b) written the same value, (c) written correctly and
	// read from somewhere else. The three readings are: an empty or stale ring
	// with a large now-cyc gap; writes=high with changes=0; and via-ram
	// disagreeing with via-map.
	scpuLineText( dst, capacity, length, "  ram-watch: $" );
	scpuLineHexU32( dst, capacity, length, d.watchAddr[ 0 ] );
	scpuLineText( dst, capacity, length, " $" );
	scpuLineHexU32( dst, capacity, length, d.watchAddr[ 1 ] );
	scpuLineText( dst, capacity, length, " writes=" );
	scpuLineDecimal( dst, capacity, length, d.watchWrites );
	scpuLineText( dst, capacity, length, " changes=" );
	scpuLineDecimal( dst, capacity, length, d.watchChanges );
	scpuLineText( dst, capacity, length, "\r\n  phase 9F88: via-ram=$" );
	scpuLineHexByte( dst, capacity, length, d.phaseViaRAM );
	scpuLineText( dst, capacity, length, " via-map=$" );
	scpuLineHexByte( dst, capacity, length, d.phaseViaMap );
	scpuLineText( dst, capacity, length, " 9F87=$" );
	scpuLineHexByte( dst, capacity, length, d.phaseShadow );
	scpuLineText( dst, capacity, length, "\r\n" );
	{
		const u32 wring = CC64Memory::SCPU_WATCH_RING;
		const u32 wn = d.watchPos < wring ? d.watchPos : wring;
		for ( u32 i = 0; i < wn; i++ )
		{
			const CC64Memory::WatchedWrite &w =
				d.watchRing[ ( d.watchPos - wn + i ) % wring ];
			scpuLineText( dst, capacity, length, "  wr " );
			scpuLineDecimal( dst, capacity, length, i );
			scpuLineText( dst, capacity, length, " $" );
			scpuLineHexByte( dst, capacity, length, (u8)( w.addr >> 8 ) );
			scpuLineHexByte( dst, capacity, length, (u8)( w.addr & 0xFF ) );
			scpuLineText( dst, capacity, length, " <- $" );
			scpuLineHexByte( dst, capacity, length, w.value );
			scpuLineText( dst, capacity, length, " was=$" );
			scpuLineHexByte( dst, capacity, length, w.old );
			// PC points just PAST the writing instruction: $09B0 STY $9F88
			// records $09B3, because the operand bytes are already fetched.
			scpuLineText( dst, capacity, length, " by=$" );
			scpuLineHexU32( dst, capacity, length, w.pc );
			scpuLineText( dst, capacity, length, " cyc=" );
			scpuLineDecimal( dst, capacity, length, w.cycles );
			scpuLineText( dst, capacity, length, "\r\n" );
		}
	}

	// --- raster state the chain is waiting on -----------------------------
	// A chain parked on one rung forever is either not being reached or waiting
	// for a compare that cannot match. raster-compare is the line the guest
	// last programmed; the $D012 reads are what it actually saw, with ages in
	// the same clock the ring stamps.
	scpuLineText( dst, capacity, length, "  raster-compare=" );
	scpuLineDecimal( dst, capacity, length, d.rasterCompare );
	scpuLineText( dst, capacity, length, " d012-reads=" );
	scpuLineDecimal( dst, capacity, length, d.d012ReadCount );
	scpuLineText( dst, capacity, length, " last:" );
	{
		const u32 dring = CC64Memory::SCPU_D012_RING;
		const u32 dn = d.d012ReadPos < dring ? d.d012ReadPos : dring;
		for ( u32 i = 0; i < dn; i++ )
		{
			const u32 idx = ( d.d012ReadPos - dn + i ) % dring;
			scpuLineText( dst, capacity, length, " " );
			scpuLineDecimal( dst, capacity, length, d.d012Reads[ idx ] );
			scpuLineText( dst, capacity, length, "@" );
			scpuLineDecimal( dst, capacity, length, d.d012ReadCyc[ idx ] );
		}
	}
	// RST8, the one bit that parked the chain. READ THESE AS A SET:
	//   rst8-writes>0           a read-modify-write still put raster bit 8 in
	//                           the latch -- the fix did not hold.
	//   samples>0 rejected=0    the physical samples were COHERENT, so the beam
	//                           really was in 256..262 when the guest read
	//                           $D011, and the remaining defect is that the
	//                           access happened there at all. rejlow is then
	//                           meaningless and low= is the line's low byte.
	//   rejected>0              the bit did not come from the raster: rejlow is
	//                           the confirming $D012, and low|256 was not a
	//                           line this machine has.
	// impossible-compare and anchor-refused are the two downstream guards; both
	// should stay 0 once rst8-writes does.
	scpuLineText( dst, capacity, length, "\r\n  rst8: samples=" );
	scpuLineDecimal( dst, capacity, length, d.rst8Samples );
	scpuLineText( dst, capacity, length, " rejected=" );
	scpuLineDecimal( dst, capacity, length, d.rst8Rejected );
	scpuLineText( dst, capacity, length, " low=" );
	scpuLineDecimal( dst, capacity, length, d.rst8LastLow );
	scpuLineText( dst, capacity, length, " rejlow=" );
	scpuLineDecimal( dst, capacity, length, d.rst8LastRejLow );
	scpuLineText( dst, capacity, length, " writes=" );
	scpuLineDecimal( dst, capacity, length, d.rst8Writes );
	scpuLineText( dst, capacity, length, " impossible-compare=" );
	scpuLineDecimal( dst, capacity, length, d.compareImpossible );
	scpuLineText( dst, capacity, length, " anchor-refused=" );
	scpuLineDecimal( dst, capacity, length, d.anchorRefused );

	// Newest write to each of the four registers Mario's chain drives, pulled
	// out of the I/O ring that was already being captured and never emitted
	// anywhere the card could read it. Bit 24 marks a write.
	{
		static const u8 want[ 4 ] = { 0x11, 0x16, 0x18, 0x19 };
		scpuLineText( dst, capacity, length, "\r\n  last VIC writes:" );
		for ( u32 k = 0; k < 4; k++ )
		{
			u32 found = 0xFFFFFFFF;
			for ( u32 back = 1; back <= 64; back++ )
			{
				const u32 e = d.io[ 64 - back ];
				if ( !( e & ( 1u << 24 ) ) ) continue;
				const u32 a = e & 0xFFFF;
				if ( a < 0xD000 || a >= 0xD400 ) continue;
				if ( ( a & 0x3F ) != want[ k ] ) continue;
				found = ( e >> 16 ) & 0xFF;
				break;
			}
			scpuLineText( dst, capacity, length, " D0" );
			scpuLineHexByte( dst, capacity, length, want[ k ] );
			scpuLineText( dst, capacity, length, "=" );
			if ( found == 0xFFFFFFFF )
				scpuLineText( dst, capacity, length, "--" );
			else
				scpuLineHexByte( dst, capacity, length, (u8)found );
		}
		scpuLineText( dst, capacity, length, "\r\n" );
	}

	scpuAppendIntCtlDiag( dst, capacity, length );

	scpuAppendOneInterruptRing( dst, capacity, length, "int",
	                            d.intRing, d.intRingHead );
	// The transition, which the live ring above cannot also hold: 64 entries is
	// about five frames and the confirm is 8.4s later.
	if ( d.progConfirmed )
		scpuAppendOneInterruptRing( dst, capacity, length, "sus",
		                            d.intRingSuspect, d.intRingSuspectHead );
}

static bool scpuSaveRuntimeBusDiag( const SCPURuntimeResult &runtime )
{
	// Raised from 65536: the post-mortem now carries a second 64-entry ring, a
	// 32-entry watched-write ring and the live-cell list, about 7K more.
	// scpuLineChar truncates the TAIL, which is the newest and most
	// interesting end of every one of those blocks.
	// Raised from 81920: the interrupt-control ring adds a latched 128-entry
	// block and a 48-entry live tail at ~130 characters, about 23K.
	static char haltBusDiag[ 131072 ];
	u32 size =
		radBus.formatSelfTestResults( haltBusDiag, sizeof haltBusDiag );
	if ( !size ) return false;
	scpuAppendRuntimeBusDiag( haltBusDiag, sizeof haltBusDiag, size, runtime );
	scpuAppendPiThrottledDiag( haltBusDiag, sizeof haltBusDiag, size );
	scpuAppendRuntimePiDiag( haltBusDiag, sizeof haltBusDiag, size );
	scpuAppendPacerDiag( haltBusDiag, sizeof haltBusDiag, size );
	scpuAppendInterruptRingDiag( haltBusDiag, sizeof haltBusDiag, size );
	scpuAppendVSPTraceDiag( haltBusDiag, sizeof haltBusDiag, size );
	const bool busDiagSaved = size
	                       && writeFile( s_Logger, SCPU_DRIVE,
	                                     SCPU_BUS_DIAG_FILE,
	                                     (u8 *)haltBusDiag, size );

	// The cold-start self-test rewrites busdiag-e18.txt on every Pi boot. Keep
	// the runtime VSP capture in its own file so the convenient RAD/Pi reboot
	// button cannot successfully save the trace and then immediately erase it.
	// Do not touch an older trace when no runtime snapshot exists.
	bool vspTraceSaved = true;
	if ( s_ResetDiagnostic.vspValid )
	{
		// Already too small before this change: 128 retained events at ~217
		// characters is 27.8K, plus ~6.2K of matrix block and the header, so
		// this buffer has been silently losing events. scpuLineChar truncates
		// the TAIL -- the newest and most interesting end of the ring.
		// Raised from 49152: the comment above already records that 128 events
		// at ~217 characters plus the matrix block overran it, and the
		// post-mortem block below adds ~5.5K in front of the trace (64 ring
		// lines at ~72 characters plus the fixed dumps).
		// Raised from 90112 for the same ~7K of new post-mortem blocks.
		// Raised from 106496 for the same ~23K of interrupt-control ring.
		static char vspDiag[ 131072 ];
		u32 vspSize = 0;
		scpuLineText( vspDiag, sizeof vspDiag, vspSize,
		              "SCPU-EMU passive VSP timing trace\r\n" );
		scpuAppendPacerDiag( vspDiag, sizeof vspDiag, vspSize );
		scpuAppendInterruptRingDiag( vspDiag, sizeof vspDiag, vspSize );
		scpuAppendVSPTraceDiag( vspDiag, sizeof vspDiag, vspSize );
		vspTraceSaved = vspSize
		              && writeFile( s_Logger, SCPU_DRIVE,
		                            SCPU_VSP_TRACE_FILE,
		                            (u8 *)vspDiag, vspSize );
	}
	return busDiagSaved && vspTraceSaved;
}

// Shared scratch for the three C64 images, safe to reuse because CC64Memory
// COPIES what it is given.
static u8 romBuffer[ C64_BASIC_SIZE ];

// The SuperCPU's own ROM needs its own home for the life of the program:
// CSuperCPUMemoryMap::setROM() stores a POINTER rather than copying, so this
// cannot share romBuffer above. Sized for the largest image the address space
// can hold; the shipped SuperCPU DOS images are 128K.
static u8 scpuROMBuffer[ SCPU_ROM_MAXSIZE ];

static bool radBusButtonPressed() { return radBus.buttonPressed(); }
static bool radBusHardwareResetPressed() { return radBus.hardwareResetPressed(); }
static bool radBusRAMUnderIOReady() { return radBus.ramUnderIOReady(); }
static bool radBusPrepareRAMUnderIOAccess()
{
	return radBus.prepareRAMUnderIOAccess();
}
static void radBusSampleInterrupts( bool &irq, bool &nmi ) { radBus.sampleInterrupts( irq, nmi ); }
static bool radBusTrafficHalted() { return radBus.trafficHalted(); }
static void radBusHaltAndProbe( u32 &eligiblePhases, u32 &addrChanges,
	                           u32 &distinctAddrs, u32 &sequentialAddrs,
	                           u32 &writePhases, u8 &firstAddr, u8 &lastAddr )
{
	radBus.setTrafficHalted( true );
	radBus.probeIdleMaster( eligiblePhases, addrChanges, distinctAddrs,
	                       sequentialAddrs, writePhases, firstAddr, lastAddr );
}

// Park here on any failure. Without this a failed start-up never reaches the
// run loop, so nothing ever polls the button and the only way out is a power
// cycle -- which is exactly what the button exists to avoid.
static void scpuWaitForButtonThenReboot()
{
	while ( !radBus.buttonPressed() )
		;
	reboot();
}

bool scpuBootLoadConfig( CLogger *logger )
{
	// The RAD's bus timings depend on the Raspberry Pi model and the machine.
	// Start from the built-in defaults, then let scpu.cfg override.
	setDefaultTimings( AUTO_TIMING_RPI3PLUS_C64C128 );

	bool haveConfig = readConfig( logger, SCPU_DRIVE, SCPU_CONFIG_FILE ) != 0;
	if ( !haveConfig )
		logger->Write( "SCPU", LogWarning,
		               "no %s found, using built-in timings", SCPU_CONFIG_FILE );

	// Snapshot into the cache-resident block the DMA primitives read from.
	initBusTiming();
	return haveConfig;
}

static bool loadROMFile( CLogger *logger, const char *name, u8 *dst, u32 expected )
{
	u32 size = 0;

	// 'expected' is also the buffer capacity, so readFile refuses anything
	// larger before reading a single byte. That matters because the SuperCPU
	// DOS images are 128K and dropping one in as kernal.rom by mistake would
	// otherwise overrun an 8K buffer.
	if ( !readFile( logger, SCPU_DRIVE, name, dst, &size, expected ) )
		return false;

	if ( size != expected )
	{
		logger->Write( "SCPU", LogWarning, "%s is %u bytes, expected %u -- ignored",
		               name, size, expected );
		return false;
	}

	logger->Write( "SCPU", LogNotice, "loaded %s", name );
	return true;
}

// Same, but for an image whose size is not known in advance. The SuperCPU DOS
// images ship at 128K, but the ROM region is larger and CMD produced more than
// one build, so the size is reported rather than demanded -- an image that is
// not the size we expected is far more useful loaded than refused.
// Returns 0 if absent or unusable.
static u32 loadROMFileSized( CLogger *logger, const char *name, u8 *dst, u32 capacity )
{
	u32 size = 0;

	if ( !readFile( logger, SCPU_DRIVE, name, dst, &size, capacity ) )
		return 0;

	if ( size == 0 )
		return 0;

	// A ROM that is not a power of two cannot mirror cleanly into its region,
	// which is how the address decode fills the space above the image.
	if ( size & ( size - 1 ) )
	{
		logger->Write( "SCPU", LogWarning,
		               "%s is %u bytes, not a power of two -- ignored", name, size );
		return 0;
	}

	logger->Write( "SCPU", LogNotice, "loaded %s (%u KB)", name, size / 1024 );
	return size;
}


// Screen codes -> ASCII, enough to read what the emulated KERNAL has drawn.
static char scpuScreenChar( u8 c )
{
	if ( c == 0x20 || c == 0x00 ) return ' ';
	if ( c >= 0x01 && c <= 0x1A ) return (char)( 'A' + c - 1 );
	if ( c >= 0x30 && c <= 0x39 ) return (char)( '0' + c - 0x30 );
	if ( c == 0x2E ) return '.';
	if ( c == 0x2A ) return '*';
	if ( c == 0x2C ) return ',';
	if ( c == 0x2F ) return '/';
	if ( c == 0x28 ) return '(';
	if ( c == 0x29 ) return ')';
	return '?';
}

// Defined below. The benchmark inside the dump now waits a serial transaction
// out with the same helper its caller uses, instead of abandoning the
// measurement the moment one starts.
static bool scpuWaitForDiagnosticWindow( CSuperCPU &scpu );

// Report what the emulator believes the screen contains, straight out of shadow
// RAM. This separates two very different faults that look identical on the C64:
// a CPU core that is running wrong, versus a core that is running correctly
// while the mirroring fails to get its output across the bus.
static void scpuDumpEmulatedScreen( CLogger *logger, CSuperCPU &scpu )
{
	// Logging stops the emulated CPU while the physical drive keeps running.
	// The caller seeks a quiet window, but keep this defensive check here so a
	// future caller cannot accidentally pause an active IEC transaction.
	if ( scpu.memory().iecBusActive() )
		return;

	const u8 *ram = scpu.memory().m_RAM;

	logger->Write( "SCPU", LogNotice, "--- emulated screen, from SHADOW RAM $0400 ---" );

	for ( u32 row = 0; row < 25; row++ )
	{
		char line[ 41 ];
		bool blank = true;
		for ( u32 col = 0; col < 40; col++ )
		{
			line[ col ] = scpuScreenChar( ram[ 0x0400 + row * 40 + col ] );
			if ( line[ col ] != ' ' ) blank = false;
		}
		line[ 40 ] = 0;
		if ( !blank )
			logger->Write( "SCPU", LogNotice, "|%s|", line );
	}

	logger->Write( "SCPU", LogNotice,
	               "PC=$%06X  cycles=%llu  memtop($37/$38)=$%02X%02X",
	               (unsigned)scpu.cpu()->pc(),
	               (unsigned long long)scpu.cpu()->cycles(),
	               ram[ 0x38 ], ram[ 0x37 ] );

	// Achieved speed. The selected speed is only a request: if the interpreter
	// plus its pacing overhead cannot retire instructions fast enough, the
	// machine silently runs slower than it claims and everything time-based
	// drifts. This is the number that says whether that is happening.
	{
		u64 c0 = 0, h0 = 0, io0 = 0, mir0 = 0;
		u64 wait0 = 0, late0 = 0, probe0 = 0, lateHost0 = 0;
		u64 slow0 = 0, iec0 = 0;
		const u32 benchResetGeneration = s_HardwareResetGeneration;

		// A transaction can begin during the benchmark, and nothing may be
		// logged while one is in progress. This used to RETURN when that
		// happened, which is why the verdict below never printed on a machine
		// whose SuperCPU DOS keeps probing drives: one serial edge anywhere in
		// sixty frames discarded the whole measurement, silently. The guard is
		// untouched and still consulted at the same two points per frame; only
		// the response changed -- wait the transaction out with the caller's
		// own quiet-window helper, re-baseline, try again, and finally report a
		// SHORTER span rather than nothing.
		//
		// Retry only while the span is still too short to report: a later
		// attempt can end WORSE than an earlier one, and re-baselining means
		// the deltas belong to the last attempt alone, so a usable span must
		// never be thrown away for the chance of a longer one.
		static const u32 BENCH_FRAMES = 60;
		static const u32 BENCH_ATTEMPTS = 3;
		static const u32 BENCH_FRAMES_MIN = 10;
		u32 frames = 0;
		for ( u32 attempt = 0; attempt < BENCH_ATTEMPTS; attempt++ )
		{
			c0 = scpu.cpu()->cycles();
			h0 = radBus.hostCycles();
			io0 = scpu.memory().m_IOReads + scpu.memory().m_IOWrites;
			mir0 = scpu.writeBuffer().m_BytesFlushed;
			wait0 = scpu.memory().m_PacerWaitHostCycles;
			late0 = scpu.memory().m_PacerLateEntries;
			probe0 = scpu.memory().m_PacerLateProbes;
			lateHost0 = scpu.memory().m_PacerLateSnapHostCycles;
			slow0 = scpu.memory().m_SlowPacedCycles;
			iec0 = scpu.memory().m_IECThrottledCycles;

			frames = 0;
			while ( frames < BENCH_FRAMES )
			{
				if ( scpu.memory().iecBusActive() )
					break;
				if ( scpuRunFrameAndCheckButtons( scpu ) )
					return;
				if ( scpu.memory().iecBusActive() )
					break;
				frames++;
			}
			if ( frames >= BENCH_FRAMES_MIN )
				break;

			// Never wait after the LAST attempt: its partial span is about to
			// be reported, and frames run now would land inside the measured
			// host interval without being counted in it.
			if ( attempt + 1 < BENCH_ATTEMPTS
			     && !scpuWaitForDiagnosticWindow( scpu ) )
				break;
		}

		// A reset or a reboot request during those attempts zeroed the very
		// counters the baselines were read from, so the deltas below would be
		// nonsense rather than merely imprecise.
		if ( s_RebootRequested
		     || s_HardwareResetGeneration != benchResetGeneration )
			return;
		if ( frames < BENCH_FRAMES_MIN )
		{
			logger->Write( "SCPU", LogNotice,
			               "speed: not measured -- the serial bus was never quiet for %u frames",
			               (unsigned)BENCH_FRAMES_MIN );
			return;
		}

		const u64 dc  = scpu.cpu()->cycles() - c0;
		const u64 dh  = radBus.hostCycles() - h0;
		const u64 dio = ( scpu.memory().m_IOReads + scpu.memory().m_IOWrites ) - io0;
		const u64 dmir = scpu.writeBuffer().m_BytesFlushed - mir0;
		const u64 dwait = scpu.memory().m_PacerWaitHostCycles - wait0;
		const u64 dlate = scpu.memory().m_PacerLateEntries - late0;
		const u64 dprobe = scpu.memory().m_PacerLateProbes - probe0;
		const u64 dlateHost =
			scpu.memory().m_PacerLateSnapHostCycles - lateHost0;
		const u64 dslow = scpu.memory().m_SlowPacedCycles - slow0;
		const u64 diec = scpu.memory().m_IECThrottledCycles - iec0;
		const u64 dfast = dc > dslow ? dc - dslow : 0;

		const u32 achievedKHz = dh ? (u32)( ( dc * ( SCPU_ARM_CLOCK_HZ / 1000 ) ) / dh ) : 0;
		const u64 expectedHost = dfast * ( SCPU_ARM_CLOCK_HZ / SCPU_TURBO_HZ )
		                       + dslow * ( SCPU_ARM_CLOCK_HZ / SCPU_NORMAL_HZ );

		logger->Write( "SCPU", LogNotice,
		               "speed: aggregate %u kHz over %u frames, schedule %llu%% (%s)",
		               (unsigned)achievedKHz, (unsigned)frames,
		               (unsigned long long)( dh ? ( expectedHost * 100 ) / dh : 0 ),
		               ( expectedHost * 100 >= dh * 90 )
		                   ? "keeping up" : "FALLING BEHIND -- timing will drift" );
		logger->Write( "SCPU", LogNotice,
		               "  modes: %llu turbo + %llu slow cycles (%llu IEC-forced)",
		               (unsigned long long)dfast, (unsigned long long)dslow,
		               (unsigned long long)diec );

		// Where the time actually went. Without this the speed figure says only
		// THAT we are behind, never WHY, and the two candidate explanations --
		// a slow interpreter versus too much traffic on the expansion port --
		// call for completely different fixes.
		//
		// A bus access costs roughly one microsecond, i.e. about 1400 ARM
		// cycles at 1.4GHz, so it takes very few of them to dominate. Compare
		// "arm/emucycle" against "bus%" to tell the two apart: a high
		// arm/emucycle with a low bus% means the interpreter is the limit.
		const u64 busAccesses = dio + dmir;
		const u64 busArmCycles = busAccesses * ( SCPU_ARM_CLOCK_HZ / 1000000 );
		logger->Write( "SCPU", LogNotice,
		               "  %llu emu cycles in %llu arm cycles = %llu arm/emucycle",
		               (unsigned long long)dc, (unsigned long long)dh,
		               (unsigned long long)( dc ? dh / dc : 0 ) );
		logger->Write( "SCPU", LogNotice,
		               "  pacer waited %llu arm cycles; excluding waits: %llu arm/emucycle",
		               (unsigned long long)dwait,
		               (unsigned long long)( dc && dh > dwait ? ( dh - dwait ) / dc : 0 ) );
		// The premise the +10-cycle VSP arithmetic rests on, finally measured.
		// Counted only while the guest selected 1MHz, so this will read zero at
		// a turbo prompt by construction -- it is Mario's regime that matters,
		// and the SD files carry the same totals for a run that cannot be
		// watched on the console.
		logger->Write( "SCPU", LogNotice,
		               "  pacer late at 1MHz %llu times, %llu probed, +%llu arm cycles (%llu arm per probe)",
		               (unsigned long long)dlate,
		               (unsigned long long)dprobe,
		               (unsigned long long)dlateHost,
		               (unsigned long long)( dprobe ? dlateHost / dprobe : 0 ) );
		if ( superCPU.benchArmPerEmuCycle() )
		{
			logger->Write( "SCPU", LogNotice,
			               "  interpreter pure: %u arm/emucycle (70 = 20MHz)",
			               (unsigned)superCPU.benchArmPerEmuCycle() );
			// Per 1000 emulated cycles: the stall diagnosis. A 32K two-way
			// L1I refilling constantly, a D-cache thrashing, or a mispredict
			// per dispatch each point at a different fix.
			logger->Write( "SCPU", LogNotice,
			               "  bench PMU/1k emu: instr=%u l1i=%u l1d=%u brmiss=%u",
			               (unsigned)superCPU.m_BenchInstrPer1k,
			               (unsigned)superCPU.m_BenchL1IRefillPer1k,
			               (unsigned)superCPU.m_BenchL1DRefillPer1k,
			               (unsigned)superCPU.m_BenchBranchMissPer1k );
		}
		logger->Write( "SCPU", LogNotice,
		               "  bus: %llu io + %llu mirrored = %llu accesses, ~%llu%% of the time",
		               (unsigned long long)dio, (unsigned long long)dmir,
		               (unsigned long long)busAccesses,
		               (unsigned long long)( dh ? ( busArmCycles * 100 ) / dh : 0 ) );
	}

	logger->Write( "SCPU", LogNotice,
	               "bus: ioReads=%u ioWrites=%u ramWrites=%u mirrored=%u",
	               (unsigned)scpu.memory().m_IOReads,
	               (unsigned)scpu.memory().m_IOWrites,
	               (unsigned)scpu.memory().m_RamWrites,
	               (unsigned)scpu.writeBuffer().m_BytesFlushed );
}

// Startup diagnostics are useful, but logger output and the snapshot benchmark
// both stop normal execution at frame boundaries. Wait only by running normal
// frames, require IEC to remain quiet across several of them, and give up after
// a bounded interval if the machine is still probing or loading from a drive.
static bool scpuWaitForDiagnosticWindow( CSuperCPU &scpu )
{
	// 180 frames (three seconds) was not enough on a machine whose SuperCPU
	// DOS probes drives through the whole boot: the window never opened and
	// every diagnostic behind it was skipped in silence. Seven seconds costs
	// nothing measurable -- this runs once, before the permanent run loop, and
	// only ever advances ordinary frames -- and the quiet requirement itself is
	// deliberately unchanged, so nothing is ever logged into a live transaction.
	static const u32 MAX_WAIT_FRAMES = 420;
	static const u32 QUIET_FRAMES_REQUIRED = 3;
	u32 quietFrames = 0;

	for ( u32 i = 0; i < MAX_WAIT_FRAMES; i++ )
	{
		if ( !scpu.memory().iecBusActive() )
		{
			if ( ++quietFrames >= QUIET_FRAMES_REQUIRED )
				return true;
		}
		else
			quietFrames = 0;

		if ( scpuRunFrameAndCheckButtons( scpu ) )
			return false;
	}

	return false;
}

void scpuBootRun( CLogger *logger )
{
	s_PiThrottledStartupValid = scpuReadPiThrottled( s_PiThrottledStartup );
	s_PiARMClockStartupValid = scpuReadPiARMClock( s_PiARMClockStartup );
	scpuLogPiThrottled( logger, "at startup", s_PiThrottledStartupValid,
	                    s_PiThrottledStartup );
	scpuLogPiARMClock( logger, "at startup", s_PiARMClockStartupValid,
	                   s_PiARMClockStartup );
	scpuLogMeasuredCycleCounter( logger );

	// One ACT-LED blink per milestone, so there is a signal even with no
	// monitor attached. The Pi's own error codes are all >= 3 flashes, so a
	// single blink cannot be mistaken for one.
	actLED.Blink( 1 );

	// Validate the complete ROM set before touching the Commodore bus. Loading
	// all four first also lets a missing file produce one clear checklist rather
	// than failing later with a partly configured accelerator.
	const bool basicLoaded =
		loadROMFile( logger, SCPU_ROM_DIR "basic.rom", romBuffer, C64_BASIC_SIZE );
	if ( basicLoaded )
		superCPU.setBasicROM( romBuffer );

	const bool kernalLoaded =
		loadROMFile( logger, SCPU_ROM_DIR "kernal.rom", romBuffer, C64_KERNAL_SIZE );
	if ( kernalLoaded )
		superCPU.setKernalROM( romBuffer );

	const bool chargenLoaded =
		loadROMFile( logger, SCPU_ROM_DIR "chargen.rom", romBuffer, C64_CHARROM_SIZE );
	if ( chargenLoaded )
		superCPU.setCharROM( romBuffer );

	const u32 scpuROMSize = loadROMFileSized( logger, SCPU_ROM_DIR "scpu.rom",
	                                          scpuROMBuffer, SCPU_ROM_MAXSIZE );
	const bool scpuROMLoaded = scpuROMSize == 128u * 1024u;
	if ( scpuROMLoaded )
		superCPU.memoryMap().setROM( scpuROMBuffer, scpuROMSize );

	if ( !basicLoaded || !kernalLoaded || !chargenLoaded || !scpuROMLoaded )
	{
		logger->Write( "SCPU", LogError,
		               "ERROR: MISSING ROMS IN SCPU DIRECTORY:" );
		logger->Write( "SCPU", LogNotice, "-> basic.rom (901226-01)" );
		logger->Write( "SCPU", LogNotice, "-> kernal.rom (901227-03)" );
		logger->Write( "SCPU", LogNotice, "-> chargen.rom (901225-01)" );
		logger->Write( "SCPU", LogNotice, "-> scpu.rom (2.04)" );
		logger->Write( "SCPU", LogNotice, "" );
		logger->Write( "SCPU", LogNotice, "BOOTING NORMALLY" );
		radUnmountFileSystem();
		return;
	}

	// From here on the SD card is not touched again: the FAT driver takes
	// interrupts and allocates, neither of which is safe once we are holding
	// DMA and driving the bus to a cycle budget.
	radUnmountFileSystem();
	actLED.Blink( 2 );		// milestone: ROMs handled, SD released

	s_Logger = logger;
	radBus.setLogger( logger );
	logger->Write( "SCPU", LogNotice,
	               "takeover: automatic legacy-first with C128 Ultimax fallback" );

	// Which core: the 65816 is the real accelerator, the 6502 is milestone-1
	// scaffolding kept as a fallback. CPU_CORE in scpu.cfg decides, so a bad run
	// can be backed out by editing one line on the SD card.
	const SCPUCoreType core = ( cfgCPUCore == SCPU_CFG_CORE_6502 )
	                        ? SCPU_CORE_6502 : SCPU_CORE_65816;

	logger->Write( "SCPU", LogNotice, "CPU core: %s",
	               ( core == SCPU_CORE_6502 ) ? "MOS 6502 (fallback)" : "WDC 65C816" );

	// This replaces the physical JiffyDOS switch on the cartridge. Set it before
	// init() resets and starts the SuperCPU ROM; reset deliberately preserves the
	// switch position, so CMD's boot code sees the configured value when it
	// chooses which KERNAL to install. A later $D0B5 POKE remains in effect until
	// the Pi is rebooted and this configuration is read again.
	superCPU.registers().setJiffyDOSSwitch( cfgJiffyDOS != 0 );
	// VSP PLACEMENT CALIBRATION, re-wired. The K425 bisect suspended this
	// along with the bias-8 aim encoding and the discard was never undone,
	// so scpu.cfg has carried VSP_AIM_READ / VSP_AIM_WRITE -- parsed,
	// range-checked, documented as the way to measure the machine's
	// placement constant -- into a (void) ever since. The bias encoding is
	// long gone (the aim point is the plain K420 decode again), so the
	// reason for the suspension no longer holds, and the knob is the only
	// instrument here that reads placement off the VIC's own c-access
	// counter instead of off a software anchor whose in-line position is
	// unknown. Both default to 0, so a machine that does not set them in
	// scpu.cfg is bit-identical to before.
	// APPLIED AFTER init(), NOT HERE. superCPU.cpu() is NULL at this point --
	// CSuperCPU's constructor sets m_CPU to 0 (supercpu.cpp:386) and only
	// init() selects m_CPU = &m_Core65816 (supercpu.cpp:563), which happens
	// roughly 170 lines below. The guard therefore skipped the call on every
	// boot and the aim stayed at its default zero in every build that has ever
	// shipped, while the log line that used to sit here printed the CONFIG
	// values OUTSIDE the guard -- so the boot log reported "write +6" for a
	// setting that was never applied, and three hardware sweeps (+2, +4, +6)
	// were all no-ops read as results. Confirmed in the compiled image, not
	// just the source: in the preserved kernel8.lst the null check branches
	// past the indirect call.
	//
	// The lesson that belongs next to the code rather than in a log file: a
	// configuration value that is PARSED is not a value that is APPLIED, and
	// logging the parsed one proves nothing. See the readback after init().
	// A deliberate $D0BA knock may request runtime VIC/HDMI selection, but only
	// when this boot actually creates the passive HDMI renderer. Track the live
	// handoff flag so its unlocked read reports completed hardware state.
	superCPU.registers().configureVideoSwitch( false, false );
	superCPU.registers().trackVideoHDMIActive( &s_HDMIExclusive );
	logger->Write( "SCPU", LogNotice, "JiffyDOS: %s (scpu.cfg)",
	               cfgJiffyDOS ? "enabled" : "disabled" );

	// Bootmap runs CMD's own boot code before the C64's KERNAL, which is where
	// the SuperCPU banner comes from. It is read from the config here, on the
	// Pi, well before any emulation starts -- so BOOTMAP 0 on the SD card is
	// always effective, however badly the emulated machine behaves with it on.
	superCPU.setBootmapEnabled( cfgBootmap != 0 );
	superCPU.setC128Mode( cfgC128Mode == 1 ? SCPU_C128_MODE_C64
	                    : cfgC128Mode == 2 ? SCPU_C128_MODE_NATIVE
	                                      : SCPU_C128_MODE_AUTO );
	logger->Write( "SCPU", LogNotice, "C128 mode: %s (scpu.cfg)",
	               cfgC128Mode == 1 ? "forced C64"
	             : cfgC128Mode == 2 ? "forced native" : "auto" );
	superCPU.registers().setBootAnimationHack( cfgBootAnimation != 0 );
	if ( cfgBootAnimation )
		logger->Write( "SCPU", LogNotice,
		               "boot animation: enabled (one-byte $D20C deviation)" );

	// Mirroring inside the visible picture. Border-only delivery (0) cannot
	// keep up with a game that redraws moving objects every frame, which shows
	// as coherent scenery with mixed-frame garbage on exactly the moving parts.
	superCPU.setMirrorDisplayBudget( (u32)cfgMirrorDisplayBytes );
	logger->Write( "SCPU", LogNotice, "mirror: %u bytes/drain inside the display%s",
	               (unsigned)cfgMirrorDisplayBytes,
	               cfgMirrorDisplayBytes ? "" : " (border-only)" );
	superCPU.setDisplayScrub( cfgDisplayScrub != 0, cfgDisplayScrubMask != 0,
	                         (u32)cfgDisplayScrubPeriodS );
	logger->Write( "SCPU", LogNotice, "display scrub: %s (%s coverage), period=%us%s",
	               cfgDisplayScrub ? "on" : "off",
	               cfgDisplayScrubMask ? "A1=0/A3=1 masked" : "full",
	               (unsigned)cfgDisplayScrubPeriodS,
	               cfgDisplayScrubPeriodS
	                 ? " (bitmap-only; clock starts at first bitmap; write-only A/B)"
	                 : " (bitmap-only)" );

	// Shape blocks a bank-3 game keeps under $D000-$DFFF are unreachable to
	// bus writes (the halted 6510 keeps real I/O banked in), so their sprite
	// pointers are delivered translated into relocated copies at $C000-$CBFF.
	superCPU.setUnderIORelocate( cfgMirrorD000Relocate != 0 );
	logger->Write( "SCPU", LogNotice, "mirror: under-I/O sprite relocation %s",
	               cfgMirrorD000Relocate ? "on" : "off" );

	// VIDEO_MODE 1 replaces the console with a small indexed framebuffer. The
	// GPU scales it to the configured HDMI mode; core 1 is not started until
	// all physical-bus calibration and self-test work is complete. Diagnostic
	// images retain the firmware console because their text is the result.
	if ( cfgVideoMode == SCPU_CFG_VIDEO_VICII )
	{
		if ( cfgBusAccessSentinel != 0 )
			logger->Write( "SCPU", LogNotice,
			               "video: diagnostic image retains firmware console" );
		else
		{
			CHDMIDisplay *display = new CHDMIDisplay( superCPU.memory() );
			if ( display && display->initialize( logger ) )
			{
					display->watchBusActivity( radBus.serialCounter() );
					s_HDMIDisplay = display;
					superCPU.memory().attachHDMICollisionLatches(
						display->spriteSpriteCollisionLatch(),
						display->spriteBackgroundCollisionLatch(),
						&s_HDMIExclusive );
					superCPU.registers().configureVideoSwitch( true, true );
				if ( scpuPiUndervoltageSeen( s_PiThrottledStartupValid,
				                             s_PiThrottledStartup ) )
					s_HDMIDisplay->showFailure();
				else
					s_HDMIDisplay->showReady();
				// The old CScreenDevice owns the superseded console buffer. Stop
				// the logger writing into it; mode 1 now communicates by colour.
				logger->SetNewTarget( 0 );
			}
			else
				logger->Write( "SCPU", LogError,
				               "video: HDMI renderer unavailable; retaining console" );
		}
	}
	else if ( cfgVideoMode == SCPU_CFG_VIDEO_VDC )
		logger->Write( "SCPU", LogWarning,
		               "video: VDC HDMI output is not implemented yet" );
	else
		logger->Write( "SCPU", LogNotice, "video: firmware text console" );

	// Say once whether the per-second line will follow, so a quiet log reads
	// as configured rather than wedged. Stall detection is armed either way.
	logger->Write( "SCPU", LogNotice, "heartbeat: %s (stall detector always armed)",
	               cfgHeartbeat ? "on" : "off" );

	// CIA2 timer NMIs delivered on the emulated clock instead of the sampled
	// line; cycle-timed loader NMIs (Winter Games /SCPU) depend on it.
	superCPU.setNMIRetime( cfgNMIRetime != 0 );
	superCPU.setDeferNativeNMI( cfgNMINativeDefer != 0 );
	superCPU.setVectorReroute( cfgVectorReroute != 0 );
	superCPU.setIOStretch( cfgIOStretch != 0 );
	superCPU.setMirrorStretch( cfgMirrorStretch != 0 );
	// The VIC freezing the emulated CPU on a badline. Mode 0 is the K451
	// behaviour, in which Mario's IRQ stabiliser sled cannot absorb a steal
	// and the machine eventually hangs at $0849 with S destroyed.
	superCPU.setBAStall( (u32)cfgBAStall, (u32)cfgBAStallMax );
	// How much evidence a CIA2 access needs before it counts as serial traffic.
	// Mode 0 is the legacy predicate measured at 98% armed during Mario
	// gameplay with no drive access at all.
	superCPU.setIECWindow( (u32)cfgIECWindow );
	// Forward progress, not any particular register: the hang detector that is
	// NOT derived from the hypothesis it tests. See the W65_PROG_ block in
	// Source/CPU/W65C816/w65c816.h.
	superCPU.setHangWatch( cfgHangWatch != 0 );
	// 0 disarms a slot, and the memory layer's sentinel is 0x10000, so a
	// disarmed slot must be translated and not passed through: passing 0 would
	// watch $0000 and put the notify call on every zero-page write.
	superCPU.setRAMWatch( cfgRAMWatch ? (u32)cfgRAMWatch : 0x10000u,
	                      cfgRAMWatch2 ? (u32)cfgRAMWatch2 : 0x10000u );
	logger->Write( "SCPU", LogNotice,
	               "nmi retime: %s, native defer: %s, vector reroute: %s",
	               cfgNMIRetime ? "on" : "off",
	               cfgNMINativeDefer ? "on" : "off",
	               cfgVectorReroute ? "on" : "off" );
	logger->Write( "SCPU", LogNotice,
	               "access cost: io=%s mirror=%s",
	               cfgIOStretch ? "on" : "off",
	               cfgMirrorStretch ? "on" : "off" );
	// A separate line, not a third field in the one above: that format is
	// quoted verbatim in CHT and a changed shape breaks every grep against an
	// archived run.
	logger->Write( "SCPU", LogNotice,
	               "ba stall: mode=%d max=%d cycles (1MHz only)",
	               cfgBAStall, cfgBAStallMax );
	logger->Write( "SCPU", LogNotice, "iec window: mode=%d", cfgIECWindow );
	logger->Write( "SCPU", LogNotice,
	               "hang watch: %s, ram watch: $%04X $%04X",
	               cfgHangWatch ? "on" : "off",
	               (unsigned)cfgRAMWatch, (unsigned)cfgRAMWatch2 );
	// DIAG_MINIMAL keeps every instrument on the line above and drops the K449
	// per-stage bus timers: two ~40-instruction accumulator bodies riding on a
	// transfer whose measured unstalled floor is 639 ARM, paid on every physical
	// access the guest makes -- and invisible to the TI benchmark, whose loop
	// never leaves Pi RAM. bastall-r/w stays live in both modes.
	radBusStageTimingEnabled = ( cfgDiagMinimal == 0 );
	logger->Write( "SCPU", LogNotice,
	               "diag minimal: %s (busstage timers %s)",
	               cfgDiagMinimal ? "on" : "off",
	               radBusStageTimingEnabled ? "on" : "off" );

	// From superCPU.init() onward the Pi owns and accesses the C64 bus in
	// sub-microsecond GPIO windows. Circle's timer/SD IRQs may pre-empt at any
	// instruction, so leaving them enabled can stretch a nominally valid bus
	// phase into the VIC-II's half-cycle. Match upstream RAD and keep ARM IRQs
	// masked for timed bus operation.
	//
	// This masking is LOAD-BEARING, established by bisection: moving it later
	// so that startup logging would work stopped games loading at all, because
	// init(), the bus self-test and the first frames then ran with interrupts
	// live. Logging is bought back the other way instead -- CScopedLoggingIRQs
	// unmasks around individual log writes, which happen between bus
	// operations, never inside one.
	logger->Write( "SCPU", LogNotice, "READY." );
	DisableIRQs();

	if ( !superCPU.init( &radBus, core, SCPU_SIMM_16MB, (u8)cfgREUSize ) )
	{
		// init() releases /DMA on every failure path, so the C64 is running its
		// own CPU again and is usable -- it simply has no accelerator.
		{
		CScopedLoggingIRQs failIRQs;
		scpuHDMIShowFailure();
		logger->Write( "SCPU", LogError,
		               "startup failed: no machine detected, no safe DMA handover "
		               "point found, or no usable KERNAL. The C64 has been released "
		               "and continues to run normally." );
		logger->Write( "SCPU", LogNotice,
		               "press the RAD button to reboot and retry." );
		}
		scpuWaitForButtonThenReboot();
	}

	// THE VSP PLACEMENT CALIBRATION, applied where the core actually exists.
	// init() has selected m_CPU above, and no guest instruction has run yet, so
	// this is the first moment the setting can take effect and the last moment
	// before it would matter. Unconditional: there is no pointer to guard.
	superCPU.cpu()->setPreciseAim( cfgVSPAimRead, cfgVSPAimWrite );
	// K498: the write aim is applied by CC64Memory at the catch-up launch of a
	// placed VIC write, not by the core's settle trim. Same moment, same rule.
	superCPU.memory().setVSPAimWrite( cfgVSPAimWrite );
	// K500: the gated shadow-served $D011 read, same moment, same rule.
	superCPU.memory().setVSPD011ShadowRead( cfgVSPD011ShadowRead != 0 );
	// K502: the write lead, quarter cycles, same moment, same rule.
	superCPU.cpu()->setWriteLead( cfgVSPWriteLead );
	superCPU.memory().setVSPPlacementK501( cfgVSPPlacementK501 != 0 );	// K505 bisect key
	superCPU.memory().setVSPTraceWindowOnly( cfgVSPTraceWindowOnly != 0 );	// K506 trace filter

	actLED.Blink( 3 );		// milestone: bus acquired

	// Interrupts stay masked from init() to the end of the run loop. Each
	// message BLOCK below unmasks just for its writes -- never across a bus
	// operation. The first cut of this used one scope over all of startup,
	// which quietly included the bus self-test's timed reads and writes:
	// exactly the exposure the bisect proved fatal.
	{
	CScopedLoggingIRQs startupIRQs;

	// READBACK, not a repeat of the config. These numbers come from the running
	// core, so a mismatch against scpu.cfg is visible instead of silent -- which
	// is exactly how the previous version hid a dead control for three hardware
	// sweeps. Printed whenever EITHER the request or the applied value is
	// nonzero, so "requested 6, applied 0" cannot go unreported.
	{
		const int aimR = superCPU.cpu()->preciseAimRead();
		const int aimW = superCPU.memory().vspAimWrite();	// K498: applied there, read back from there
		if ( aimR || aimW || cfgVSPAimRead || cfgVSPAimWrite )
			logger->Write( "SCPU", LogNotice,
			               "VSP aim APPLIED: read %+d write %+d "
			               "(scpu.cfg requested %+d/%+d) -- CALIBRATION, "
			               "not a normal setting",
			               aimR, aimW, cfgVSPAimRead, cfgVSPAimWrite );
		if ( cfgVSPD011ShadowRead )
			logger->Write( "SCPU", LogNotice,
			               "VSP $D011 shadow read: ON (raster window only, "
			               "RST8 proven 0 by the anchor) -- K500" );
		logger->Write( "SCPU", LogNotice,
		               "VSP write lead APPLIED: %d quarter cycles (scpu.cfg %d) -- K502",
		               superCPU.cpu()->writeLeadQuarters(), cfgVSPWriteLead );
	}

	// The pure-interpreter figure, no bus and no scheduler: 70 means 20MHz.
	if ( superCPU.benchArmPerEmuCycle() )
	{
		logger->Write( "SCPU", LogNotice,
		               "interpreter: %u arm/emucycle pure (70 = 20MHz)",
		               (unsigned)superCPU.benchArmPerEmuCycle() );
		logger->Write( "SCPU", LogNotice,
		               "pmu/1k emucycles: insn=%u irefill=%u drefill=%u brmiss=%u",
		               (unsigned)superCPU.m_BenchInstrPer1k,
		               (unsigned)superCPU.m_BenchL1IRefillPer1k,
		               (unsigned)superCPU.m_BenchL1DRefillPer1k,
		               (unsigned)superCPU.m_BenchBranchMissPer1k );
		logger->Write( "SCPU", LogNotice,
		               "workloads arm/emucycle: b0=%u screen=%u io=%u super=%u",
		               (unsigned)superCPU.m_BenchBank0ArmPerCycle,
		               (unsigned)superCPU.m_BenchScreenArmPerCycle,
		               (unsigned)superCPU.m_BenchIOArmPerCycle,
		               (unsigned)superCPU.m_BenchSuperRAMArmPerCycle );
		logger->Write( "SCPU", LogNotice,
		               "superram added emucycles/1k base: %u",
		               (unsigned)superCPU.m_BenchSuperRAMStretchPer1k );
	}

	const C64Signals &sig = radBus.signals();
	s_DetectedVideo = sig.video;
	s_DetectedVideoValid = true;
	// Cross-check the PHI2 period the pacer is already using. setPacing()
	// derives it from sig.video so the grid is reproducible between boots;
	// radWaitForMachineRunning() measured the same quantity on the live
	// machine before takeover, and they agreed to 19ppm on an NTSC C64
	// (CHT 1024). A percent-scale disagreement means the standard is
	// mis-detected or arm_freq is not 1400MHz, and every paced cycle is wrong
	// by that amount -- so warn rather than print two numbers and leave the
	// comparison to whoever reads the log.
	{
		const u64 rate1k   = radLastMeasuredMachineRate();
		const u32 assumed  = c64CpuHz( sig.video );
		const u32 measured = rate1k
		                   ? (u32)( 1000ULL * SCPU_ARM_CLOCK_HZ / rate1k ) : 0;
		const u32 delta    = measured > assumed
		                   ? ( measured - assumed ) : ( assumed - measured );
		logger->Write( "SCPU",
		               ( measured && delta * 100u <= assumed )
		                   ? LogNotice : LogWarning,
		               "PHI2: %s %u Hz assumed, %u Hz measured",
		               sig.video == VIDEO_PAL ? "PAL" : "NTSC",
		               (unsigned)assumed, (unsigned)measured );
	}
	if ( s_HDMIDisplay )
		s_HDMIDisplay->setVideoTiming( c64CyclesPerLine( sig.video ),
		                               c64RasterLines( sig.video ),
		                               c64DisplayFirstLine( sig.video ) );
	logger->Write( "SCPU", LogNotice, "SuperRAM: %u MB%s",
	               (unsigned)superCPU.fastRAM().sizeMB(),
	               superCPU.fastRAM().present()
	                   ? " (unreachable until the 65816 core lands)" : " -- none fitted" );

	const u32 reuBytes = superCPU.reu().sizeBytes();
	if ( reuBytes == 0 )
		logger->Write( "SCPU", LogNotice, "REU: disabled" );
	else if ( reuBytes < 1024u * 1024u )
	{
		const char *model = reuBytes == 128u * 1024u ? " (1700)"
		                  : reuBytes == 256u * 1024u ? " (1764)"
		                  : reuBytes == 512u * 1024u ? " (1750)" : "";
		logger->Write( "SCPU", LogNotice, "REU: %u KB%s",
		               (unsigned)( reuBytes / 1024u ), model );
	}
	else
		logger->Write( "SCPU", LogNotice, "REU: %u MB (expanded)",
		               (unsigned)( reuBytes / ( 1024u * 1024u ) ) );

	logger->Write( "SCPU", LogNotice, "attached to %s, %s",
	               sig.machine == MACHINE_C128 ? "C128" : "C64",
	               sig.video == VIDEO_PAL ? "PAL" : "NTSC" );

	logger->Write( "SCPU", LogNotice, "running bus self-test..." );
	}	// remask: the self-test is real, timed bus traffic

	const bool busSelfTestPassed = radBus.selfTest();
	const bool c128KnownFirstTransferOnly =
		radBus.c128KnownFirstTransferOnly();
	s_PiThrottledPostSelfTestValid =
		scpuReadPiThrottled( s_PiThrottledPostSelfTest );
	s_PiARMClockPostSelfTestValid =
		scpuReadPiARMClock( s_PiARMClockPostSelfTest );
	{
		CScopedLoggingIRQs powerLogIRQs;
		scpuLogPiThrottled( logger, "after bus self-test",
		                    s_PiThrottledPostSelfTestValid,
		                    s_PiThrottledPostSelfTest );
		scpuLogPiARMClock( logger, "after bus self-test",
		                   s_PiARMClockPostSelfTestValid,
		                   s_PiARMClockPostSelfTest );
	}
	if ( s_HDMIDisplay
	     && scpuPiUndervoltageSeen( s_PiThrottledPostSelfTestValid,
		                              s_PiThrottledPostSelfTest ) )
		s_HDMIDisplay->showFailure();
	// Kernel154 established the physical retention bound: perfect through 2 ms,
	// first stable decay at 4 ms. Production epochs are conservatively sized
	// from that result, so do not repeat the minute-long ladder on every boot.
	{
		// Always replace the previous run's file, whether this run passes or
		// fails. The SD write happens between bus operations with /DMA held and
		// IRQs temporarily enabled; it cannot alter the C128-side measurement.
		static char busDiag[ 65536 ];
		u32 busDiagSize = radBus.formatSelfTestResults( busDiag, sizeof busDiag );
		scpuAppendPiThrottledDiag( busDiag, sizeof busDiag, busDiagSize );
		CScopedLoggingIRQs diagFileIRQs;
		if ( busDiagSize && writeFile( logger, SCPU_DRIVE, SCPU_BUS_DIAG_FILE,
		                               (u8 *)busDiag, busDiagSize ) )
			logger->Write( "SCPU", LogNotice, "saved %s", SCPU_BUS_DIAG_FILE );
		else
			logger->Write( "SCPU", LogError, "could not save %s", SCPU_BUS_DIAG_FILE );
	}

	// On the physical C128 the first transfer after a direction change is still
	// occasionally lost, but the saved kernel144 result was narrowly bounded:
	// only element zero of the first-transfer series and single-write series,
	// zero failures in all 20 repeated runs, and a perfect burst. Permit exactly
	// that known C128 signature for this boot trial; every broader error remains
	// a hard stop and the raw failure is retained in busdiag.txt.
	if ( !busSelfTestPassed && !c128KnownFirstTransferOnly )
	{
		// Do not start the core over a bus we know is unreliable. It produces a
		// dramatic display and no information, and it leaves the machine
		// unusable. Hand the C64 back instead: it returns to its own CPU and
		// the log above stays on screen to be read.
		CScopedLoggingIRQs failIRQs;
		scpuHDMIShowFailure();
		radBus.logSelfTestResults( logger );
		if ( cfgBusHaltAfterS > 0 )
		{
			u32 eligiblePhases = 0, addrChanges = 0, distinctAddrs = 0;
			u32 sequentialAddrs = 0, writePhases = 0;
			u8 firstAddr = 0, lastAddr = 0;
			radBusHaltAndProbe( eligiblePhases, addrChanges, distinctAddrs,
			                    sequentialAddrs, writePhases, firstAddr, lastAddr );
			logger->Write( "SCPU", LogNotice,
			               "idle host: eligible=%u distinct=%u changes=%u sequential=%u",
			               (unsigned)eligiblePhases, (unsigned)distinctAddrs,
			               (unsigned)addrChanges, (unsigned)sequentialAddrs );
			logger->Write( "SCPU", LogNotice,
			               "idle host: first=%02X last=%02X", firstAddr, lastAddr );
			logger->Write( "SCPU", writePhases ? LogError : LogNotice,
			               "idle host: R/W-low=%u/%u%s", (unsigned)writePhases,
			               (unsigned)eligiblePhases,
			               writePhases ? " -- ANOTHER BUS MASTER IS WRITING" : "" );
		}
		logger->Write( "SCPU", LogError,
		               "BUS SELF-TEST FAILED -- not starting the CPU core." );
		logger->Write( "SCPU", LogError,
		               "Releasing /DMA; the C64 gets its own CPU back and should "
		               "behave normally. Fix the failing operation above first." );
		radBus.release();
		// Final diagnostic group:
		//   6 blinks = single R/W failed only
		//   7 blinks = burst write failed only
		//   8 blinks = both failed
		// The earlier startup groups are 1, 2 and 3 blinks, so a user counting
		// every flash sees totals of 12, 13 or 14 respectively.
		const u8 busFailure = radBus.selfTestFailure();
		actLED.Blink( busFailure == 1 ? 6 : busFailure == 2 ? 7 : 8 );
		logger->Write( "SCPU", LogNotice,
		               "press the RAD button to reboot and retry." );
		scpuWaitForButtonThenReboot();
	}
	if ( c128KnownFirstTransferOnly )
	{
		CScopedLoggingIRQs toleratedIRQs;
		logger->Write( "SCPU", LogNotice,
		               "C128: tolerating isolated first-transfer self-test signature;"
		               " repeated single and burst paths passed" );
	}

	// K221-K233 diagnostic-only path. It deliberately runs before the CPU core and
	// never boots afterward: the broad physical-RAM sentinel overwrites normal
	// machine contents. Normal firmware behavior is unchanged unless the SD
	// config explicitly selects a BUS_ACCESS_SENTINEL mode.
	if ( cfgBusAccessSentinel )
	{
		const bool displaySentinel = cfgBusAccessSentinel == 2;
		const bool displayAddress = cfgBusAccessSentinel == 3;
		const bool displayRow = cfgBusAccessSentinel == 4;
		const bool displayFetch = cfgBusAccessSentinel == 5;
		const bool displayPersistence = cfgBusAccessSentinel == 6;
		const bool displayTiming = cfgBusAccessSentinel == 7;
		const bool displayBoundary = cfgBusAccessSentinel == 8;
		const bool displayRefreshCore3 = cfgBusAccessSentinel == 9;
		const bool displayRefreshCore0 = cfgBusAccessSentinel == 10;
		const bool displayRW = cfgBusAccessSentinel == 11;
		const bool displayScrub = cfgBusAccessSentinel == 12;
		const bool displayBAGuard = cfgBusAccessSentinel == 13;
		const bool displayRefresh = displayRefreshCore3 || displayRefreshCore0
		                         || displayRW || displayScrub;
		{
		CScopedLoggingIRQs sentinelStartIRQs;
			logger->Write( "SCPU", LogNotice,
			               displayBAGuard
			                 ? "K239 display BA guard: identical oracle and timed write-resume A/B"
			               : displayRefresh
				                 ? ( displayScrub
				                       ? "K233 display scrub: control versus masked shadow repair"
				                     : displayRW
				                       ? "K232 display R/W: floating versus actively-held-high idle"
			                     : displayRefreshCore0
			                       ? "K231 display refresh: sustained DMA versus core0 VIC-half release"
			                       : "K230 display refresh: sustained DMA versus core3 VIC-half release" )
		                 : displayBoundary
		                 ? "K229 display boundary: transition-only versus active dwell"
		                 : displayTiming
		                 ? "K228 display timing: exposed-map sample sweep and repair"
		                 : displayPersistence
		                 ? "K227 display persistence: repeated active arms and three readbacks"
		                 : displayFetch
		                 ? "K226 display fetch: active text/bitmap and traffic matrix"
		                 : displayRow
		                 ? "K225 display row: high-entropy write and retention maps"
		                 : displayAddress
		                 ? "K223 display address: immediate $2078 and map discriminator"
		                 : ( displaySentinel
		                       ? "K222 display sentinel: nine hires fetch arms"
		                       : "K221 access sentinel: starting control/read/write arms" ) );
			logger->Write( "SCPU", LogNotice,
			               displayBAGuard
			                 ? "K239: about one minute; do not reset or remove the card"
			               : displayRefresh
				                 ? ( displayScrub
				                       ? "K233: about 3 minutes; do not reset or remove the card"
				                     : displayRW
				                       ? "K232: about 70 seconds; do not reset or remove the card"
			                     : displayRefreshCore0
			                       ? "K231: about 35 seconds; do not reset or remove the card"
			                       : "K230: about 35 seconds; do not reset or remove the card" )
		                 : displayBoundary
		                 ? "K229: about 40 seconds; do not reset or remove the card"
		                 : displayTiming
		                 ? "K228: about 30 seconds; do not reset or remove the card"
		                 : displayPersistence
		                 ? "K227: about 60 seconds; do not reset or remove the card"
		                 : displayFetch
		                 ? "K226: about 75 seconds; do not reset or remove the card"
		                 : displayRow
		                 ? "K225: about 40 seconds; do not reset or remove the card"
		                 : displayAddress
		                 ? "K223: under 20 seconds; do not reset or remove the card"
		                 : ( displaySentinel
		                       ? "K222: about 75 seconds; border identifies each arm"
		                       : "K221: about one minute; do not reset or remove the card" ) );
		}

		const bool sentinelCompleted = displayBAGuard
			? radBus.runDisplayBAGuardDiagnostic()
			: displayScrub
			? radBus.runDisplayScrubDiagnostic( superCPU.writeBuffer() )
			: displayRW
			? radBus.runDisplayRWDiagnostic()
			: displayRefreshCore0
			? radBus.runDisplayCore0RefreshDiagnostic()
			: displayRefreshCore3
			? radBus.runDisplayRefreshDiagnostic()
			: displayBoundary
			? radBus.runDisplayBoundaryDiagnostic()
			: displayTiming
			? radBus.runDisplayTimingDiagnostic()
			: displayPersistence
			? radBus.runDisplayPersistenceDiagnostic()
			: displayFetch
			? radBus.runDisplayFetchDiagnostic()
			: displayRow
			? radBus.runDisplayRowDiagnostic()
			: displayAddress
			? radBus.runDisplayAddressDiagnostic()
			: ( displaySentinel
			      ? radBus.runDisplaySentinelDiagnostic()
			      : radBus.runAccessSentinelDiagnostic() );
		static char sentinelDiag[ 196608 ];
		const u32 sentinelDiagSize =
			radBus.formatSelfTestResults( sentinelDiag, sizeof sentinelDiag );
		{
		CScopedLoggingIRQs sentinelResultIRQs;
		if ( displayBAGuard ) radBus.logDisplayBAGuardResults( logger );
		else if ( displayRefresh ) radBus.logDisplayRefreshResults( logger );
		else if ( displayBoundary ) radBus.logDisplayBoundaryResults( logger );
		else if ( displayTiming ) radBus.logDisplayTimingResults( logger );
		else if ( displayPersistence ) radBus.logDisplayPersistenceResults( logger );
		else if ( displayFetch ) radBus.logDisplayFetchResults( logger );
		else if ( displayRow ) radBus.logDisplayRowResults( logger );
		else if ( displayAddress ) radBus.logDisplayAddressResults( logger );
		else if ( displaySentinel ) radBus.logDisplaySentinelResults( logger );
		else radBus.logAccessSentinelResults( logger );
		if ( sentinelDiagSize
		     && writeFile( logger, SCPU_DRIVE, SCPU_BUS_DIAG_FILE,
		                   (u8 *)sentinelDiag, sentinelDiagSize ) )
			logger->Write( "SCPU", LogNotice,
			               displayBAGuard
			                 ? "K239: saved display BA guard result to %s"
			               : displayRefresh
				                 ? ( displayScrub
				                       ? "K233: saved display scrub result to %s"
				                     : displayRW
				                       ? "K232: saved display R/W result to %s"
			                     : displayRefreshCore0
			                       ? "K231: saved display refresh result to %s"
			                       : "K230: saved display refresh result to %s" )
			                 : displayBoundary
			                 ? "K229: saved display boundary result to %s"
			                 : displayTiming
			                 ? "K228: saved display timing result to %s"
			                 : displayPersistence
			                 ? "K227: saved display persistence result to %s"
			                 : displayFetch
			                 ? "K226: saved display fetch result to %s"
			                 : displayRow
			                 ? "K225: saved display row result to %s"
			                 : displayAddress
			                 ? "K223: saved display address result to %s"
			                 : ( displaySentinel
			                       ? "K222: saved display sentinel to %s"
			                       : "K221: saved access sentinel to %s" ),
			               SCPU_BUS_DIAG_FILE );
		else
			logger->Write( "SCPU", LogError,
			               displayBAGuard
			                 ? "K239: could not save display BA guard result to %s"
			               : displayRefresh
				                 ? ( displayScrub
				                       ? "K233: could not save display scrub result to %s"
				                     : displayRW
				                       ? "K232: could not save display R/W result to %s"
			                     : displayRefreshCore0
			                       ? "K231: could not save display refresh result to %s"
			                       : "K230: could not save display refresh result to %s" )
			                 : displayBoundary
			                 ? "K229: could not save display boundary result to %s"
			                 : displayTiming
			                 ? "K228: could not save display timing result to %s"
			                 : displayPersistence
			                 ? "K227: could not save display persistence result to %s"
			                 : displayFetch
			                 ? "K226: could not save display fetch result to %s"
			                 : displayRow
			                 ? "K225: could not save display row result to %s"
			                 : displayAddress
			                 ? "K223: could not save display address result to %s"
			                 : ( displaySentinel
			                       ? "K222: could not save display sentinel to %s"
			                       : "K221: could not save access sentinel to %s" ),
			               SCPU_BUS_DIAG_FILE );
		logger->Write( "SCPU", sentinelCompleted ? LogNotice : LogError,
			               sentinelCompleted
			                 ? ( displayBAGuard
			                       ? "K239 COMPLETE -- cyan border, thirteen blinks; return card"
			                     : displayRefresh
				                       ? ( displayScrub
				                             ? "K233 COMPLETE -- cyan border, twelve blinks; return card"
				                           : displayRW
				                             ? "K232 COMPLETE -- cyan border, eleven blinks; return card"
			                           : displayRefreshCore0
			                             ? "K231 COMPLETE -- cyan border, ten blinks; return card"
			                             : "K230 COMPLETE -- cyan border, nine blinks; return card" )
			                       : displayBoundary
			                       ? "K229 COMPLETE -- cyan border, eight blinks; return card"
			                       : displayTiming
			                       ? "K228 COMPLETE -- cyan border, seven blinks; return card"
			                       : displayPersistence
			                       ? "K227 COMPLETE -- cyan border, six blinks; return card"
			                       : displayFetch
			                       ? "K226 COMPLETE -- cyan border, five blinks; return card"
			                       : displayRow
			                       ? "K225 COMPLETE -- cyan border, five blinks; return card"
			                       : displayAddress
			                       ? "K223 COMPLETE -- cyan border, five blinks; return card"
		                       : ( displaySentinel
		                             ? "K222 COMPLETE -- cyan border, five blinks; return card"
		                             : "K221 COMPLETE -- cyan border, five blinks; return card" ) )
			                 : ( displayBAGuard
			                       ? "K239 NOT RUN -- requires a detected C64; releasing machine"
			                     : displayRefresh
				                       ? ( displayScrub
				                             ? "K233 NOT RUN -- requires a detected C64; releasing machine"
				                           : displayRW
				                             ? "K232 NOT RUN -- requires a detected C64; releasing machine"
			                           : displayRefreshCore0
			                             ? "K231 NOT RUN -- requires a detected C64; releasing machine"
			                             : "K230 NOT RUN -- requires a detected C64; releasing machine" )
			                       : displayBoundary
			                       ? "K229 NOT RUN -- requires a detected C64; releasing machine"
			                       : displayTiming
			                       ? "K228 NOT RUN -- requires a detected C64; releasing machine"
			                       : displayPersistence
			                       ? "K227 NOT RUN -- requires a detected C64; releasing machine"
			                       : displayFetch
			                       ? "K226 NOT RUN -- requires a detected C64; releasing machine"
			                       : displayRow
			                       ? "K225 NOT RUN -- requires a detected C64; releasing machine"
			                       : displayAddress
			                       ? "K223 NOT RUN -- requires a detected C64; releasing machine"
		                       : ( displaySentinel
		                             ? "K222 NOT RUN -- requires a detected C64; releasing machine"
		                             : "K221 NOT RUN -- requires a detected C64; releasing machine" ) ) );
		}

		if ( !sentinelCompleted )
		{
			radBus.release();
			actLED.Blink( 8 );
			scpuWaitForButtonThenReboot();
		}

		// Final two physical writes are only a completion marker. The result was
		// already captured and saved, and no access follows them.
		radBus.write( 0xD020, 0x03 );
		radBus.write( 0xD020, 0x03 );
		radBus.setTrafficHalted( true );
		actLED.Blink( displayBAGuard ? 13 : displayScrub ? 12 : displayRW ? 11 : displayRefreshCore0 ? 10
		            : displayRefreshCore3 ? 9
		            : displayBoundary ? 8
		            : displayTiming ? 7 : displayPersistence ? 6 : 5 );
		for ( ;; ) asm volatile( "wfe" );
	}

	// E18N is a diagnostic-only C128 image. It deliberately stops before the
	// SuperCPU boot animation: starting the emulator would mix mirror traffic,
	// guest timing and further DRAM damage into a test whose sole question is
	// which protected PHI slots preserve the physical RAM.
	static const bool C128_REFRESH_DIAGNOSTIC_IMAGE = false;
	if ( C128_REFRESH_DIAGNOSTIC_IMAGE
	     && radBus.signals().machine == MACHINE_C128 )
	{
		{
		CScopedLoggingIRQs scanIRQs;
			logger->Write( "SCPU", LogNotice,
			               "E18N: core3 DMA release-width scan (280..560 ARM cycles)" );
			logger->Write( "SCPU", LogNotice,
			               "E18N: read0 is decision; later reads audit observation decay" );
		}
		const bool scanCompleted = radBus.runRefreshPhaseScan();
		const bool scanTrusted = radBus.refreshPhaseScanTrusted();

		// Persist raw vectors before drawing the completion screen. The renderer
		// is ordinary physical bus traffic; even if it exposes a new hardware
		// problem, the expensive scan result is already recoverable from SD.
		static char phaseDiag[ 65536 ];
		const u32 phaseDiagSize =
			radBus.formatSelfTestResults( phaseDiag, sizeof phaseDiag );
		{
		CScopedLoggingIRQs phaseFileIRQs;
		if ( phaseDiagSize
		     && writeFile( logger, SCPU_DRIVE, SCPU_BUS_DIAG_FILE,
		                   (u8 *)phaseDiag, phaseDiagSize ) )
				logger->Write( "SCPU", LogNotice,
				               "E18N: saved diagnostic data to %s",
			               SCPU_BUS_DIAG_FILE );
		else
				logger->Write( "SCPU", LogError,
				               "E18N: could not save pulse scan to %s",
			               SCPU_BUS_DIAG_FILE );
		radBus.logSelfTestResults( logger );
		if ( !scanCompleted )
			logger->Write( "SCPU", LogError,
			               "E18N mechanical abort -- return the SD card for the partial log" );
		else if ( scanTrusted )
			logger->Write( "SCPU", LogNotice,
			               "E18N complete -- return the SD card" );
		else
			logger->Write( "SCPU", LogWarning,
			               "E18N incomplete -- partial pulse scan saved; return the SD card" );
		}

		radBus.renderRefreshPhaseScanSummary();
		// No ordinary physical access is allowed after this point. Core 3 changes
		// to continuous VIC-half /DMA release and preserves the displayed result
		// until the user powers down and returns the card.
		radBus.setTrafficHalted( true );
		actLED.Blink( scanTrusted ? 5 : 8 );
		for ( ;; ) asm volatile( "wfe" );
	}

	// A real SuperCPU keeps the stopped 6510 in an all-RAM processor-port map
	// and selects Ultimax only for genuine I/O cycles. Install that arrangement
	// only now: ROM snapshotting, calibration, the bus self-test and destructive
	// diagnostics above all depend on the conventional host map. The RAD's C128
	// refresh ownership is deliberately excluded until its equivalent has been
	// validated independently.
	if ( radBus.signals().machine == MACHINE_C64 )
	{
		// RAM_UNDER_IO selects whether to attempt that arrangement at all. It
		// defaults to OFF, and the short-circuit matters: with OFF we never probe,
		// so the second $01=$34 takeover never happens and no per-access /GAME is
		// ever asserted. Displayed data under $D000-$DFFF is then delivered by
		// backing-store relocation instead, which needs neither.
		//
		// The default is OFF for determinism rather than caution. Per-access /GAME
		// is carried into VIC fetch state by at least one FPGA host, and a machine
		// whose physical map depends on how a boot-time probe happened to go makes
		// every field report ambiguous about which path was actually running.
		const bool underIOAttempted =
			cfgRAMUnderIO != SCPU_CFG_RAM_UNDER_IO_OFF;
		const bool underIOReady =
			underIOAttempted && radBus.prepareRAMUnderIOAccess();
		const u8 underIOFailure =
			underIOAttempted ? radBus.ramUnderIOFailure() : 0;
		if ( !underIOReady )
		{
			// Some FPGA hosts accept the ordinary badline acquisition but not a
			// second GAME/Ultimax reset takeover. That means the native under-I/O
			// optimisation is unavailable, not that SuperCPU emulation is unsafe.
			// The failed probe has reset and released the host; reacquire the exact
			// K371 conventional map and retain the established D-window suppression
			// plus sprite-relocation fallback.
			if ( !radBus.acquired() && !radBus.acquire() )
			{
				CScopedLoggingIRQs underIOFatalIRQs;
				scpuHDMIShowFailure();
				logger->Write( "SCPU", LogError,
				               "RAM-under-I/O probe failed and legacy reacquisition failed" );
				logger->Write( "SCPU", LogNotice,
				               "press the RAD button to reboot and retry." );
				scpuWaitForButtonThenReboot();
			}
			superCPU.setRAMUnderIOAccessible( false );
			superCPU.setUnderIORelocate( cfgMirrorD000Relocate != 0 );
			{
				// Distinguish "declined by configuration" from "probed and failed".
				// Reporting a failure code we never obtained would send anyone
				// reading this log hunting a takeover fault that did not occur.
				CScopedLoggingIRQs underIOFallbackIRQs;
				if ( !underIOAttempted )
					logger->Write( "SCPU", LogNotice,
					               "physical RAM under I/O: not attempted "
					               "(RAM_UNDER_IO 0); displayed data under $D000-$DFFF "
					               "is delivered by backing-store relocation" );
				else
					logger->Write( "SCPU", LogWarning,
					               "physical RAM under I/O unavailable (failure=$%02X); "
					               "continuing with compatibility fallback",
					               (unsigned)underIOFailure );
			}
		}
		else
		{
			superCPU.setRAMUnderIOAccessible( true );
			CScopedLoggingIRQs underIOLogIRQs;
			logger->Write( "SCPU", LogNotice,
			               "physical RAM under I/O: enabled; /GAME selects chip accesses" );
		}
	}

	// Measurements are finished. Core 1 may now read Pi shadow and write the
	// indexed framebuffer without sharing any physical-bus path with core 0.
	if ( s_HDMIDisplay )
	{
		superCPU.memory().enableVICLog( true );
		if ( s_HDMIDisplay->start() )
		{
			// The first passive frame naturally replaces the green ready screen.
			// K356 makes one bounded wait before emulation begins: prove that
			// core 1 is actively rendering
			// and repeat the read-eye scan under that exact electrical/thermal
			// load. The quiet acquisition curve is retained alongside it.
			s_HDMIPictureActive = true;
			const u64 core1Deadline = radBus.hostCycles()
			                        + SCPU_ARM_CLOCK_HZ / 4u;
			while ( !s_HDMIDisplay->firstFrameReady()
			        && radBus.hostCycles() < core1Deadline )
				asm volatile( "yield" );
			if ( s_HDMIDisplay->firstFrameReady() )
			{
				radBus.calibrateReadTimingUnderLoad();
				static char loadedEyeDiag[ 65536 ];
				u32 loadedEyeDiagSize = 0;
				{
					CScopedLoggingIRQs loadedEyeIRQs;
					s_PiThrottledCore1Valid =
						scpuReadPiThrottled( s_PiThrottledCore1 );
					s_PiARMClockCore1Valid =
						scpuReadPiARMClock( s_PiARMClockCore1 );
					loadedEyeDiagSize = radBus.formatSelfTestResults(
						loadedEyeDiag, sizeof loadedEyeDiag );
					scpuAppendPiThrottledDiag( loadedEyeDiag,
					                            sizeof loadedEyeDiag,
					                            loadedEyeDiagSize );
					if ( loadedEyeDiagSize )
						writeFile( logger, SCPU_DRIVE, SCPU_BUS_DIAG_FILE,
						           (u8 *)loadedEyeDiag, loadedEyeDiagSize );
				}
			}
		}
		else
		{
			superCPU.memory().enableVICLog( false );
			CScopedLoggingIRQs failIRQs;
			scpuHDMIShowFailure();
			logger->Write( "SCPU", LogError,
			               "HDMI renderer did not start; retaining the console" );
		}
	}

	actLED.Blink( 4 );		// milestone: handing over to the CPU core
	if ( !s_HDMIPictureActive )
	{
	CScopedLoggingIRQs handoverIRQs;
	logger->Write( "SCPU", LogNotice, "starting the CPU core" );
	logger->Write( "SCPU", LogNotice,
	               "buttons: LEFT = reboot the Pi, RIGHT = reset the emulated C64" );

	}	// startup reporting done; masked again for the run

	// Run about two seconds of emulated time, then report what the emulator
	// thinks it has drawn only after a bounded wait for an IEC-quiet window.
	// If boot-time drive probing never settles, skip diagnostics rather than
	// stopping the emulated CPU while the physical drive continues.
	bool startupInterrupted = false;
	for ( u32 i = 0; i < 120; i++ )
	{
		if ( scpuRunFrameAndCheckButtons( superCPU ) )
		{
			startupInterrupted = true;
			break;
		}
	}

	if ( !s_HDMIPictureActive && !startupInterrupted
	     && scpuWaitForDiagnosticWindow( superCPU ) )
		scpuDumpEmulatedScreen( logger, superCPU );

	// Calibration runs before the successful boot and its original log lines
	// have scrolled away by now. Repeat the retained results last so hardware
	// testing does not require photographing a transient early boot screen.
	if ( !s_HDMIPictureActive && !startupInterrupted )
	{
		CScopedLoggingIRQs resultIRQs;
		radBus.logSelfTestResults( logger );
		// The first diagnostic snapshot predates the CPU-core startup. Refresh
		// phase/deadline faults tend to appear under animation traffic, so replace
		// it after the bounded 120-frame run with the live counters. This remains
		// outside any timed physical access and makes runtime fallback diagnosable
		// from the returned card rather than from a transient HDMI log.
		static char runtimeBusDiag[ 16384 ];
		const u32 runtimeBusDiagSize =
			radBus.formatSelfTestResults( runtimeBusDiag, sizeof runtimeBusDiag );
		if ( runtimeBusDiagSize
		     && writeFile( logger, SCPU_DRIVE, SCPU_BUS_DIAG_FILE,
		                   (u8 *)runtimeBusDiag, runtimeBusDiagSize ) )
			logger->Write( "SCPU", LogNotice,
			               "updated %s with runtime refresh counters",
			               SCPU_BUS_DIAG_FILE );
		else
			logger->Write( "SCPU", LogError,
			               "could not update %s after startup",
			               SCPU_BUS_DIAG_FILE );
		logger->Write( "SCPU", LogNotice,
		               "calibration results retained above for hardware testing" );
	}

	// A short LEFT press can happen inside one of the startup wrappers above.
	// Preserve that request instead of consuming it merely as "stop this
	// diagnostic" and then entering the permanent run loop.
	if ( !s_RebootRequested )
	{
		superCPU.setFrameHook( scpuCheckButton, &superCPU );
		superCPU.run();
	}

	// The button was pressed. Release the machine and restart the Pi so a fresh
	// kernel8.img on the card is picked up.
	logger->Write( "SCPU", LogNotice, "button pressed -- releasing /DMA and rebooting" );
	radBus.release();
	actLED.Blink( 2 );
	reboot();
}
