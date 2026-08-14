/*
   SCPU-EMU - CMD SuperCPU emulation for the C64/C128 using a RAD Expansion Unit
   Copyright (c) 2026 SCPU-EMU contributors

   Start-up sequence.

   ROM policy. Two paths, in order of preference:

     1. Files on the SD card under SCPU/. Use this to run a real SuperCPU ROM
        image (which brings JiffyDOS and the SuperCPU DOS with it), or to pin a
        specific KERNAL revision. No ROM images ship with this project.
     2. Snapshot the ROMs off the live machine over the bus. Needs no files at
        all and gives you the KERNAL actually fitted to the computer. The
        character ROM cannot be captured this way -- exposing it needs CHAREN
        low and with the 6510 held off the bus nothing can rewrite its port --
        so chargen stays blank unless supplied as a file. That only matters for
        programs that read the character set through the CPU; the VIC-II reads
        it directly on the C64 side and is unaffected.

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
// Native-C128 bring-up may execute garbage continuously rather than settling
// into a tight loop, so the ordinary stall detector never fires. Keep a small
// sampled PC history and force one bounded capture early in startup.
static u32 s_C128DiagnosticFrames = 0;
static u32 s_C128PCTrace[ 8 ] = {};
static u32 s_C128PCTracePos = 0;

static bool radBusButtonPressed();
static bool radBusHardwareResetPressed();
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
	u8 scrubPhaseOn;
	u64 scrubSampledText;
	u64 scrubSampledBitmap;
	u64 scrubMismatchText;
	u64 scrubMismatchBitmap;
	u64 scrubRepaired;
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
		radBusSampleInterrupts( d.liveIRQ, d.liveNMI );
	}

	{
		CC64Memory &m = scpu->memory();
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
		d.mirrorSkipped = w.m_WritesSkipped;
		d.mirrorFlushed = w.m_BytesFlushed;
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
	s_Logger->Write( "SCPU", LogNotice,
	               "  mirror: mode=%u pending=%u screen/bitmap=%u/%u accepted=%llu skipped=%llu flushed=%llu",
	               (unsigned)d.mirrorMode, (unsigned)d.mirrorPending,
	               (unsigned)d.mirrorScreenPending,
	               (unsigned)d.mirrorBitmapPending,
	               (unsigned long long)d.mirrorAccepted,
	               (unsigned long long)d.mirrorSkipped,
	               (unsigned long long)d.mirrorFlushed );
	s_Logger->Write( "SCPU", LogNotice,
	               "  display scrub: phase=%s repaired=%llu (physical sampling disabled)",
	               d.scrubPhaseOn ? "ON" : "OFF",
	               (unsigned long long)d.scrubRepaired );
	if ( s_HDMIDisplay )
		s_Logger->Write( "SCPU", LogNotice,
		               "  hdmi renderer: max-band=%uus missed-deadlines=%u",
		               (unsigned)s_HDMIDisplay->maxBandUS(),
		               (unsigned)s_HDMIDisplay->missedBandDeadlines() );

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

static void scpuPromoteHDMIExclusive( CSuperCPU *scpu );

static bool scpuCheckButton( void *ctx )
{
	CSuperCPU *scpu = (CSuperCPU *)ctx;
	scpuPromoteHDMIExclusive( scpu );
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

	// Say so the FIRST time a bus wait ever hits its ceiling. Before those
	// ceilings existed this was the freeze that took the whole firmware down
	// -- no frame hook, no button, power cycle only -- so it left no trace at
	// all. One line, once, is the difference between a mystery and a lead.
	if ( !scpu || !scpu->memory().iecBusActive() )
	{
		static u32 lastBA = 0, lastPHI = 0;
		if ( radBAWaitTimeouts != lastBA || radPHIWaitTimeouts != lastPHI )
		{
			lastBA = radBAWaitTimeouts;
			lastPHI = radPHIWaitTimeouts;
			CScopedLoggingIRQs irqs;
			s_Logger->Write( "SCPU", LogNotice,
			               "BUS WAIT TIMEOUT: BA=%u PHI=%u (bus stopped answering)",
			               (unsigned)lastBA, (unsigned)lastPHI );
		}
	}

	// These reads go to the physical CIA. Do not insert them into an active
	// serial transaction merely for diagnostics; take the next scheduled
	// sample once IEC is quiet instead.
	if ( scpu && !hardwareResetPressed
	     && ( ++s_PreResetSampleFrame % 50 ) == 0
	     && !scpu->memory().iecBusActive() )
	{
		s_PreResetDD00 = scpu->memory().read8( 0xDD00 );
		s_PreResetDD02 = scpu->memory().read8( 0xDD02 );
		s_PreResetSampleValid = true;
	}

	if ( radBusButtonPressed() )
	{
		// The RAD's Pi-reboot button is the convenient way to return the card,
		// but it is not the Commodore /RESET line handled below. Persist the
		// same post-handoff record on both exits so the diagnostic cannot be
		// silently lost merely because the other physical button was used.
		if ( !s_RebootRequested )
		{
			const SCPURuntimeResult runtime =
				scpuCurrentRuntimeResult( scpu );
			CScopedLoggingIRQs saveIRQs;
			if ( !scpuSaveRuntimeBusDiag( runtime ) )
				s_Logger->Write( "SCPU", LogError,
					               "K355 could not save post-handoff runtime to %s",
				               SCPU_BUS_DIAG_FILE );
			else
				s_Logger->Write( "SCPU", LogNotice,
					               "K355 saved post-handoff runtime to %s",
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
					               "K355 could not save post-handoff runtime to %s",
				               SCPU_BUS_DIAG_FILE );
			else
				s_Logger->Write( "SCPU", LogNotice,
					               "K355 saved post-handoff runtime to %s",
				               SCPU_BUS_DIAG_FILE );
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

static void scpuPromoteHDMIExclusive( CSuperCPU *scpu )
{
	// Do not give up the known physical presentation path merely because the
	// HDMI task was scheduled. Wait for proof that core 1 completed a frame;
	// the next core-0 frame hook then performs the mirror handoff without ever
	// waiting on core 1. This path exists only for VIDEO_MODE 1.
	if ( !scpu || s_HDMIExclusive || !s_HDMIPictureActive
	     || !s_HDMIDisplay || !s_HDMIDisplay->firstFrameReady() )
		return;

	// Physical mirroring was also keeping the expansion bus electrically warm.
	// Arm its low-bandwidth replacement before detaching it so the first idle
	// interval cannot expose another cold production access. This promotion is
	// reachable only in VIDEO_MODE 1; VIDEO_MODE 0 remains byte-for-byte on its
	// established physical-mirror path.
	scpu->memory().enableBusKeepalive( true );
	scpu->disablePhysicalMirror();
	s_HDMIExclusive = true;
	scpuBeginRuntimeDiagnostic( scpu );
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
	s.skippedWrites = scpu->writeBuffer().m_WritesSkipped;
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
	logger->Write( "SCPU", LogNotice, "Pi ARM clock %s: %u Hz",
	               when, (unsigned)rateHz );
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
	              "post-handoff runtime: build=355 handoff=" );
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

static bool scpuSaveRuntimeBusDiag( const SCPURuntimeResult &runtime )
{
	static char haltBusDiag[ 32768 ];
	u32 size =
		radBus.formatSelfTestResults( haltBusDiag, sizeof haltBusDiag );
	if ( !size ) return false;
	scpuAppendRuntimeBusDiag( haltBusDiag, sizeof haltBusDiag, size, runtime );
	scpuAppendPiThrottledDiag( haltBusDiag, sizeof haltBusDiag, size );
	scpuAppendRuntimePiDiag( haltBusDiag, sizeof haltBusDiag, size );
	return size && writeFile( s_Logger, SCPU_DRIVE, SCPU_BUS_DIAG_FILE,
	                          (u8 *)haltBusDiag, size );
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
		const u64 c0 = scpu.cpu()->cycles();
		const u64 h0 = radBus.hostCycles();
		const u64 io0 = scpu.memory().m_IOReads + scpu.memory().m_IOWrites;
		const u64 mir0 = scpu.writeBuffer().m_BytesFlushed;
		const u64 wait0 = scpu.memory().m_PacerWaitHostCycles;
		const u64 slow0 = scpu.memory().m_SlowPacedCycles;
		const u64 iec0 = scpu.memory().m_IECThrottledCycles;

		// A transaction can begin during the benchmark. Abort before producing
		// any more synchronous log output and return to the free-running loop.
		for ( u32 i = 0; i < 60; i++ )
		{
			if ( scpu.memory().iecBusActive() )
				return;
			if ( scpuRunFrameAndCheckButtons( scpu ) )
				return;
			if ( scpu.memory().iecBusActive() )
				return;
		}

		const u64 dc  = scpu.cpu()->cycles() - c0;
		const u64 dh  = radBus.hostCycles() - h0;
		const u64 dio = ( scpu.memory().m_IOReads + scpu.memory().m_IOWrites ) - io0;
		const u64 dmir = scpu.writeBuffer().m_BytesFlushed - mir0;
		const u64 dwait = scpu.memory().m_PacerWaitHostCycles - wait0;
		const u64 dslow = scpu.memory().m_SlowPacedCycles - slow0;
		const u64 diec = scpu.memory().m_IECThrottledCycles - iec0;
		const u64 dfast = dc > dslow ? dc - dslow : 0;

		const u32 achievedKHz = dh ? (u32)( ( dc * ( SCPU_ARM_CLOCK_HZ / 1000 ) ) / dh ) : 0;
		const u64 expectedHost = dfast * ( SCPU_ARM_CLOCK_HZ / SCPU_TURBO_HZ )
		                       + dslow * ( SCPU_ARM_CLOCK_HZ / SCPU_NORMAL_HZ );

		logger->Write( "SCPU", LogNotice,
		               "speed: aggregate %u kHz, schedule %llu%% (%s)",
		               (unsigned)achievedKHz,
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
	static const u32 MAX_WAIT_FRAMES = 180;
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
	logger->Write( "SCPU", LogNotice, "SCPU-EMU starting" );
	logger->Write( "SCPU", LogNotice, "if you are reading this on HDMI, the Pi"
	               " firmware and our kernel both loaded correctly" );
	s_PiThrottledStartupValid = scpuReadPiThrottled( s_PiThrottledStartup );
	s_PiARMClockStartupValid = scpuReadPiARMClock( s_PiARMClockStartup );
	scpuLogPiThrottled( logger, "at startup", s_PiThrottledStartupValid,
	                    s_PiThrottledStartup );
	scpuLogPiARMClock( logger, "at startup", s_PiARMClockStartupValid,
	                   s_PiARMClockStartup );

	// One ACT-LED blink per milestone, so there is a signal even with no
	// monitor attached. The Pi's own error codes are all >= 3 flashes, so a
	// single blink cannot be mistaken for one.
	actLED.Blink( 1 );

	// Optional ROM images. Anything not supplied here is taken off the machine.
	if ( loadROMFile( logger, SCPU_ROM_DIR "basic.rom", romBuffer, C64_BASIC_SIZE ) )
		superCPU.setBasicROM( romBuffer );

	if ( loadROMFile( logger, SCPU_ROM_DIR "kernal.rom", romBuffer, C64_KERNAL_SIZE ) )
		superCPU.setKernalROM( romBuffer );

	if ( loadROMFile( logger, SCPU_ROM_DIR "chargen.rom", romBuffer, C64_CHARROM_SIZE ) )
		superCPU.setCharROM( romBuffer );

	// The SuperCPU's own ROM -- SuperCPU DOS and its JiffyDOS support. Entirely
	// optional: SCPU-EMU boots the machine's own KERNAL out of bank 0, so
	// without this the accelerator still works and $F80000 simply reads open
	// bus. With it, software that knows about the accelerator can reach the
	// code CMD shipped.
	//
	// Note this only makes the ROM READABLE. It does not map it over bank 0 at
	// reset -- see the bootmap note in registers.h. Changing what runs at
	// power-on is a different decision, and one that cannot be backed out from
	// the config file, because the machine would never get far enough to read
	// it.
	{
		const u32 scpuROMSize = loadROMFileSized( logger, SCPU_ROM_DIR "scpu.rom",
		                                          scpuROMBuffer, SCPU_ROM_MAXSIZE );
		if ( scpuROMSize )
			superCPU.memoryMap().setROM( scpuROMBuffer, scpuROMSize );
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
	logger->Write( "SCPU", LogNotice,
	               "nmi retime: %s, native defer: %s, vector reroute: %s",
	               cfgNMIRetime ? "on" : "off",
	               cfgNMINativeDefer ? "on" : "off",
	               cfgVectorReroute ? "on" : "off" );
	logger->Write( "SCPU", LogNotice,
	               "access cost: io=%s mirror=%s",
	               cfgIOStretch ? "on" : "off",
	               cfgMirrorStretch ? "on" : "off" );

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
	DisableIRQs();

	if ( !superCPU.init( &radBus, core, SCPU_SIMM_16MB ) )
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

	actLED.Blink( 3 );		// milestone: bus acquired

	// Interrupts stay masked from init() to the end of the run loop. Each
	// message BLOCK below unmasks just for its writes -- never across a bus
	// operation. The first cut of this used one scope over all of startup,
	// which quietly included the bus self-test's timed reads and writes:
	// exactly the exposure the bisect proved fatal.
	{
	CScopedLoggingIRQs startupIRQs;

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
	logger->Write( "SCPU", LogNotice, "SuperRAM: %u MB%s",
	               (unsigned)superCPU.fastRAM().sizeMB(),
	               superCPU.fastRAM().present()
	                   ? " (unreachable until the 65816 core lands)" : " -- none fitted" );

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

	// Measurements are finished. Core 1 may now read Pi shadow and write the
	// indexed framebuffer without sharing any physical-bus path with core 0.
	if ( s_HDMIDisplay )
	{
		if ( s_HDMIDisplay->start() )
		{
			// The first passive frame naturally replaces the blue ready screen.
			// K355 makes one bounded wait before emulation begins: prove that
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
