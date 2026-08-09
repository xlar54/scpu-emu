/*
   SCPU-EMU - CMD SuperCPU emulation for the C64/C128 using a RAD Expansion Unit

   CRADBus - IC64Bus over the real expansion port.

   Copyright (c) 2026 SCPU-EMU contributors
   Built on the RAD Expansion Unit framework
   Copyright (c) 2022, 2023 Carsten Dachsbacher <frenetic@dachsbacher.de>

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
#include <circle/memio.h>
#include <circle/bcm2835.h>
#include <circle/types.h>
#include <circle/logger.h>

#include "gpio_defs.h"
#include "lowlevel_arm64.h"
#include "bus_timing.h"

static u64 armCycleCounter;

#include "rad_lowlevel.h"
#include "rad_bus.h"
#include "cpu_hijack.h"
#include "c128_refresh.h"

CRADBus::CRADBus()
	: m_Reads( 0 ), m_Writes( 0 ), m_BurstWrites( 0 ), m_Acquired( false ),
	  m_TrafficHalted( false ),
	  m_SelfTestFailure( 0 ),
	  m_ReadTimingConfigured( 0 ), m_ReadTimingStart( 0 ), m_ReadTimingEnd( 0 ),
	  m_ReadTimingSelected( 0 ),
	  m_WriteTimingConfiguredAddr( 0 ), m_WriteTimingConfiguredData( 0 ),
	  m_WriteTimingSelectedAddr( 0 ), m_WriteTimingSelectedData( 0 ),
	  m_WriteTimingPassingPoints( 0 ), m_TestBankA( 0 ), m_TestBankE( 0 ),
	  m_TestRasterChanged( 0 ), m_TestAddrErrors( 0 ), m_TestSingleErrors( 0 ),
	  m_TestBurstErrors( 0 ), m_TestRepeatedFailures( 0 ), m_Logger( 0 )
{
	m_Signals.machine       = MACHINE_UNKNOWN;
	m_Signals.video         = VIDEO_PAL;
	m_Signals.gameAsserted  = false;
	m_Signals.exromAsserted = false;
	for ( u32 i = 0; i < 6; i++ )
		m_TestRaster[ i ] = m_TestAddrLines[ i ] = m_TestSingle[ i ]
		                  = m_TestBurst[ i ] = 0;
	m_TestAddrFirstReread = m_TestSingleFirstReread =
		m_TestBurstFirstReread = 0;
	m_PosRawBeforeSingle[ 0 ] = m_PosRawBeforeSingle[ 1 ] = 0;
	for ( u32 p = 0; p < 3; p++ )
	{
		for ( u32 i = 0; i < 16; i++ )
			m_TurnBaseTarget[ p ][ i ] = m_TurnSingleTarget[ p ][ i ]
			                              = m_TurnDoubleTarget[ p ][ i ] = 0;
		for ( u32 i = 0; i < 3; i++ )
			m_TurnSinglePostFlush[ p ][ i ] = m_TurnDoublePostFlush[ p ][ i ] = 0;
		for ( u32 i = 0; i < 10; i++ )
		{
			m_TurnBaseNeighbor[ p ][ i ] = m_TurnSingleNeighbor[ p ][ i ]
			                                  = m_TurnDoubleNeighbor[ p ][ i ] = 0;
		}
		for ( u32 i = 0; i < 14; i++ ) m_TurnFlags[ p ][ i ] = 0;
		m_TurnSingleFlush[ p ] = m_TurnDoubleFlush[ p ] = 0;
	}
	m_TurnDetectedMachine = 0;
	for ( u32 cell = 0; cell < 256; cell++ )
	{
		m_RefreshMapExpected[ cell ] = 0;
		for ( u32 pass = 0; pass < 3; pass++ )
			m_RefreshMapReadback[ pass ][ cell ] = 0;
	}
	for ( u32 cell = 0; cell < 8; cell++ )
	{
		m_RefreshControlExpected[ cell ] = 0;
		for ( u32 pass = 0; pass < 3; pass++ )
			m_RefreshControlReadback[ pass ][ cell ] = 0;
	}
	for ( u32 i = 0; i < 32; i++ ) m_RefreshFailureBitmap[ i ] = 0;
	for ( u32 target = 0; target < 3; target++ )
	{
		for ( u32 i = 0; i < 8; i++ ) m_FirstDiscSample[ target ][ i ] = 0;
		for ( u32 i = 0; i < 6; i++ ) m_FirstDiscCount[ target ][ i ] = 0;
	}
	m_RefreshSweepChecksum = 0;
	m_RefreshSweepCount = 0;
	m_RefreshElapsedMs = 0;
	m_RefreshPassUsec = 0;
	m_RefreshFailureCount = 0;
	m_RefreshUnstableCount = 0;
	m_RefreshHostWritePhases = 0;
	for ( u32 i = 0; i < 6; i++ )
		m_RefreshGapFailures[ i ] = m_RefreshGapUnstable[ i ] = 0xFFFF;
	for ( u32 pass = 0; pass < 2; pass++ )
		for ( u32 i = 0; i < 8; i++ )
		{
			m_PhaseScanStage1Failures[ pass ][ i ] = 0xFFFF;
			m_PhaseScanStage1Unstable[ pass ][ i ] = 0xFFFF;
			m_PhaseScanStage1AnchorArm[ pass ][ i ] = 0;
			m_PhaseScanStage1AnchorPhi[ pass ][ i ] = 0;
		}
	for ( u32 i = 0; i < 65; i++ )
	{
		m_PhaseScanStage2Failures[ i ] = 0xFFFF;
		m_PhaseScanStage2BaselineFailures[ i ] = 0xFFFF;
		m_PhaseScanStage2BaselineUnstable[ i ] = 0xFFFF;
		m_PhaseScanStage2BaselineAttempts[ i ] = 0;
		m_PhaseScanStage2ExecutionIndex[ i ] = 0xFF;
		m_PhaseScanExecutionK[ i ] = 0xFF;
		m_PhaseScanStage2BaselineSeedUsec[ i ] = 0;
		m_PhaseScanStage2BaselineCaptureUsec[ i ] = 0;
		m_PhaseScanStage2CaptureUsec[ i ] = 0;
		m_PhaseScanStage2Unstable[ i ] = 0xFFFF;
		m_PhaseScanStage2Class[ i ] = '!';
		m_PhaseScanStage2AnchorArm[ i ] = 0;
		m_PhaseScanStage2Pulses[ i ] = 0;
		m_PhaseScanStage2Lines[ i ] = 0;
		m_PhaseScanStage2AnchorPhi[ i ] = 0;
		for ( u32 pass = 0; pass < 3; pass++ )
		{
			m_PhaseScanStage2PassFailures[ i ][ pass ] = 0xFFFF;
			for ( u32 byte = 0; byte < 33; byte++ )
				m_PhaseScanStage2MismatchMask[ i ][ pass ][ byte ] = 0;
		}
	}
	m_PhaseScanRan = m_PhaseScanStage1Agree = m_PhaseScanStage2Ran = 0;
	m_PhaseScanStage2Contiguous = m_PhaseScanStage2PassCount = 0;
	m_PhaseScanStage2FailureCount = m_PhaseScanStage2FlagCount = 0;
	m_PhaseScanStage2RungsCompleted = 0;
	m_PhaseScanStage2BandStart = 0xFF;
	m_PhaseScanLineCycles = 0;
	m_PhaseScanBaselineFailures = m_PhaseScanBaselineUnstable = 0;
	m_PhaseScanBaselineK = m_PhaseScanBaselineW =
		m_PhaseScanBaselineAttempts = 0;
	for ( u32 rung = 0; rung < REFRESH_CONTROL_COUNT; rung++ )
	{
		m_RefreshControlValid[ rung ] = 0;
		m_RefreshControlBaselineAttempts[ rung ] = 0;
		m_RefreshControlBaselineFailures[ rung ] = 0xFFFF;
		m_RefreshControlBaselineUnstable[ rung ] = 0xFFFF;
		m_RefreshControlUnstable01[ rung ] = 0xFFFF;
		m_RefreshControlUnstable12[ rung ] = 0xFFFF;
		m_RefreshControlPulses[ rung ] = 0;
		m_RefreshControlExpectedPulses[ rung ] = 0;
		m_RefreshControlExposureLines[ rung ] = 0;
		m_RefreshControlExposureUsec[ rung ] = 0;
		m_RefreshControlCaptureUsec[ rung ] = 0;
		m_RefreshControlAnchorArm[ rung ] = 0;
		m_RefreshControlAnchorPhi[ rung ] = 0;
		for ( u32 attempt = 0; attempt < 2; attempt++ )
		{
			m_RefreshControlBaselineFramePulses[ rung ][ attempt ] = 0;
			m_RefreshControlBaselineFrameExpected[ rung ][ attempt ] = 0;
			m_RefreshControlBaselineFrameSweeps[ rung ][ attempt ] = 0;
			m_RefreshControlBaselineSeedUsec[ rung ][ attempt ] = 0;
			m_RefreshControlBaselineCaptureUsec[ rung ][ attempt ] = 0;
			for ( u32 pass = 0; pass < 3; pass++ )
			{
				m_RefreshControlBaselinePassFailures
					[ rung ][ attempt ][ pass ] = 0xFFFF;
				m_RefreshControlBaselinePassUsec
					[ rung ][ attempt ][ pass ] = 0;
			}
			for ( u32 pair = 0; pair < 2; pair++ )
				m_RefreshControlBaselinePassUnstable
					[ rung ][ attempt ][ pair ] = 0xFFFF;
		}
		for ( u32 pass = 0; pass < 3; pass++ )
		{
			m_RefreshControlPassFailures[ rung ][ pass ] = 0xFFFF;
			m_RefreshControlCapturePassUsec[ rung ][ pass ] = 0;
			for ( u32 byte = 0; byte < 33; byte++ )
				m_RefreshControlMismatchMask[ rung ][ pass ][ byte ] = 0;
		}
	}
	m_RefreshWarmupPulses = 0;
	m_LastRefreshSeedUsec = m_LastRefreshCaptureUsec = 0;
	for ( u32 pass = 0; pass < 3; pass++ )
		m_LastRefreshCapturePassUsec[ pass ] = 0;
	m_BlackoutLongDelayUsec = 0;
	m_Core0FrameSweeps = m_Core0FramePulses =
		m_Core0FrameExpectedPulses = m_Core0FrameAccountingErrors = 0;
	m_Core0FrameMinUsec = 0xFFFFFFFFu;
	m_Core0FrameMaxUsec = 0;
	m_PulseScanRan = m_PulseScanCompleted = 0;
	for ( u32 i = 0; i < REFRESH_PULSE_SCAN_COUNT; i++ )
		m_PulseScanWidth[ i ] = 0;
}

#define RADLOG( ... ) do { if ( m_Logger ) m_Logger->Write( "RADbus", LogNotice, __VA_ARGS__ ); } while ( 0 )

// Small bounded formatter for the SD-card diagnostic. This is bare metal, so
// do not depend on snprintf or a hosted C library.
static bool busDiagChar( char *dst, u32 capacity, u32 &length, char c )
{
	if ( !dst || !capacity || length + 1 >= capacity ) return false;
	dst[ length++ ] = c;
	dst[ length ] = 0;
	return true;
}

static bool busDiagText( char *dst, u32 capacity, u32 &length, const char *s )
{
	while ( s && *s )
		if ( !busDiagChar( dst, capacity, length, *s++ ) ) return false;
	return true;
}

static bool busDiagHex( char *dst, u32 capacity, u32 &length, u8 v )
{
	static const char h[] = "0123456789ABCDEF";
	return busDiagChar( dst, capacity, length, h[ v >> 4 ] )
	    && busDiagChar( dst, capacity, length, h[ v & 15 ] );
}

static bool busDiagHexWord( char *dst, u32 capacity, u32 &length, u16 v )
{
	return busDiagHex( dst, capacity, length, (u8)( v >> 8 ) )
	    && busDiagHex( dst, capacity, length, (u8)v );
}

static bool busDiagDecimal( char *dst, u32 capacity, u32 &length, u32 v )
{
	char reverse[ 10 ];
	u32 digits = 0;
	do
	{
		reverse[ digits++ ] = (char)( '0' + v % 10 );
		v /= 10;
	} while ( v && digits < sizeof reverse );
	while ( digits )
		if ( !busDiagChar( dst, capacity, length, reverse[ --digits ] ) ) return false;
	return true;
}

static bool busDiagBytes( char *dst, u32 capacity, u32 &length, const u8 *v, u32 n )
{
	for ( u32 i = 0; i < n; i++ )
	{
		if ( i && !busDiagChar( dst, capacity, length, ' ' ) ) return false;
		if ( !busDiagHex( dst, capacity, length, v[ i ] ) ) return false;
	}
	return true;
}

static bool busDiagEndLine( char *dst, u32 capacity, u32 &length )
{
	return busDiagChar( dst, capacity, length, '\r' )
	    && busDiagChar( dst, capacity, length, '\n' );
}

// These helpers bypass RAD_PEEK/RAD_SPEEK so the diagnostic can compare a
// minimal direct transfer with the ordinary wrapper. E7 intentionally makes
// the normal read primitive itself the same direct p1/p2/p3 sequence.
__attribute__((noinline)) u8 radDirectRead( u16 addr )
{
	register u32 g2;
	u8 v = 0;
	// K202's hardware-validated blocking path. A transaction claims the open
	// C128 traffic epoch before it aligns to the physical bus; C64-class hosts
	// return immediately because the refresh interlock is disabled.
	c128TrafficBegin();
	BUS_RESYNC
	busReadByte_p1( g2, addr );
	busReadByte_p2( g2 );
	const u32 sampleAt = ( ( addr & 0xFC00 ) == 0xD400 )
	                   ? (u32)busTiming.WAIT_CYCLE_READ_SID
	                   : (u32)busTiming.WAIT_CYCLE_READ;
	busReadByte_p3( g2, v, false, sampleAt );
	c128TrafficEnd();
	return v;
}

static inline u8 busDiagRawRead( u16 addr )
{
	return radDirectRead( addr );
}

__attribute__((noinline)) void radDirectWrite( u16 addr, u8 value )
{
	register u32 g2;
	c128TrafficBegin();
	BUS_RESYNC
	busWriteByte_p1( g2, addr, value );
	busWriteByte_p2( g2, false );
	c128TrafficEnd();
}

static inline void busDiagRawWrite( u16 addr, u8 value )
{
	radDirectWrite( addr, value );
}

// bit 0: the next write will pay the read->write turnaround phase.
// Read-policy state deliberately no longer exists: raw and normal reads use
// the same direct p1/p2/p3 sequence.
static u8 busDiagTurnFlags()
{
	return busWriteTurnaroundNeeded ? 1 : 0;
}

bool CRADBus::acquire()
{
	// Order matters, and it used to be wrong. We reset the machine first and
	// only then waited for a valid clock -- so on a cold power-on, where the
	// Pi and the C64 come up together, the reset landed while the C64 was still
	// powering up. On real hardware that showed as an eight second struggle to
	// get a stable PHI2 measurement, and a takeover that happened before the
	// KERNAL had settled $01 to $37. A warm restart worked first time because
	// the machine was already running.
	//
	// So: confirm it is alive, THEN reset it, THEN confirm it came back, THEN
	// give the KERNAL room to finish before taking the bus.

	RADLOG( "1/6 waiting for the C64 to power up..." );
	if ( !radWaitForMachineRunning() )
	{
		u64 measured = radMeasureMachineRate();
		if ( measured == 0 )
			RADLOG( "    FAILED: PHI2 never changed state. Check the cartridge is"
			        " seated and the C64 is powered on." );
		else
			RADLOG( "    FAILED: 1000 C64 cycles measured %u ARM cycles; expected"
			        " 1200000-1600000. If this is a clean multiple or fraction of"
			        " the expected value, arm_freq does not match the timings.",
			        (unsigned)measured );
		return false;
	}

	RADLOG( "2/6 resetting for a clean start..." );
	radResetMachine();

	RADLOG( "3/6 waiting for restart..." );
	if ( !radWaitForMachineRunning() )
	{
		RADLOG( "    FAILED: no clock after reset." );
		return false;
	}

	RADLOG( "4/6 letting the host KERNAL boot (3s)..." );
	idleHold( 3 );

	// Prepare the fallback while the host is still running freely. If the
	// badline probe identifies a C128, /DMA is already held and the measured
	// refresh margin is only a few milliseconds; no cache warm-up or table
	// construction may be deferred into that interval.
	radPrepareUltimaxTakeover();

	RADLOG( "5/6 taking /DMA on a VIC badline..." );
	if ( radHijackCPU() )
	{
		RADLOG( "    badline takeover complete; detecting host..." );
		m_Signals.machine = radDetectMachine();
		if ( m_Signals.machine == MACHINE_C64 )
		{
			// The original RAD ordering, now proven on both a physical C64 and
			// the FPGA 64U: keep this acquisition and never expose those hosts to
			// the GAME/Ultimax reset sequence.
			busWriteTurnaroundPasses = 1;
			busWriteTurnaroundNeeded = 0;
			m_Acquired = true;
			calibrateReadTiming();
			calibrateWriteTiming();
			m_Signals.video = radDetectVideoStandard();

			RADLOG( "6/6 C64, %s -- legacy bus acquired",
			        m_Signals.video == VIDEO_PAL ? "PAL" : "NTSC" );
			return true;
		}

		// Enter with /DMA still asserted. radHijackCPUWithUltimax() asserts
		// RESET before it ever releases /DMA, so the physical 8502/Z80 cannot
		// execute in the transition. This ordering is load-bearing.
		RADLOG( "    C128 detected; escalating without releasing /DMA..." );
	}
	else
	{
		// A native C128 may have its VIC-IIe display blanked and provide no
		// badline. radHijackCPU() is bounded and has not asserted /DMA here, so
		// fall back to the reset-time Ultimax handoff instead of failing startup.
		// The bounded attempts may have displaced cached takeover code, and the
		// host still owns the bus, so it is safe to warm the fallback once more.
		radPrepareUltimaxTakeover();
		RADLOG( "    no badline; trying the bounded C128 Ultimax fallback..." );
	}

	RADLOG( "6/6 starting the GAME/Ultimax reset takeover..." );
	if ( !radHijackCPUWithUltimax() )
	{
		RADLOG( "    FAILED: the host CPU did not fetch the Ultimax NOP stream;"
		        " /DMA was released and the machine was reset normally." );
		return false;
	}

	RADLOG( "    /DMA asserted on a known read cycle; detecting the machine..." );
	// Detect first using repeated reads at the configured baseline. On a C128,
	// the former order spent the entire read scan and 513-point write grid with
	// /DMA continuously low before the refresh core even started. Kernel154's
	// ladder measured real decay beginning between 2 and 4 ms, so that ordering
	// made cold-start success probabilistic by construction.
	m_Signals.machine = radDetectMachine();
	busWriteTurnaroundPasses =
		m_Signals.machine == MACHINE_C128 ? 2 : 1;
	// The first calibration write follows the Ultimax takeover/read detector,
	// but the static direction flag starts clear. Seed the real bus state so
	// that first C128 write receives the same two idle phases as later
	// read-to-write transitions.
	busWriteTurnaroundNeeded =
		m_Signals.machine == MACHINE_C128 ? 1 : 0;
	// The raw C128 discriminator reads physical read zero correctly. E3-E6
	// proved that even a configured-zero policy preamble perturbs the strict
	// read path, so normal reads now enter p1/p2/p3 directly as raw reads do.
	// Keep GAME asserted through both probes, matching upstream
	// waitAndHijackMenu().  Only now is the C128 cartridge-start mapping no
	// longer needed; /DMA remains asserted throughout.
	SET_GPIO( bGAME_OUT );
	m_Acquired        = true;
	if ( m_Signals.machine == MACHINE_C128 && !c128RefreshStart() )
	{
		RADLOG( "    FAILED: C128 refresh helper core did not start" );
		radReleaseCPU();
		m_Acquired = false;
		return false;
	}
	if ( m_Signals.machine == MACHINE_C128 )
	{
		RADLOG( "    C128 refresh: core 3 active before calibration" );
		// Calibrate exactly once, under the final core3/core0 ownership regime.
		calibrateReadTiming();
		calibrateWriteTiming();
		RADLOG( "    C128 refresh: loaded timing calibration complete" );
	}
	else
	{
		calibrateReadTiming();
		calibrateWriteTiming();
	}
	m_Signals.video = radDetectVideoStandard();
	if ( m_Signals.machine == MACHINE_C128 )
	{
		if ( !startC128LineRefresh() )
		{
			RADLOG( "    FAILED: C128 permuted per-line refresh did not lock" );
			release();
			return false;
		}
		RADLOG( "    C128 permuted line refresh: width=8 step=%u phi=%u",
		        (unsigned)c128RefreshPermutationStep(),
		        (unsigned)c128RefreshMeasuredPhiCycles() );
	}

	RADLOG( "    %s, %s -- Ultimax bus acquired",
	        m_Signals.machine == MACHINE_C128 ? "C128" : "C64",
	        m_Signals.video == VIDEO_PAL ? "PAL" : "NTSC" );

	return true;
}

bool CRADBus::startC128LineRefresh()
{
	const u32 cyclesPerLine = c64CyclesPerLine( m_Signals.video );
	const u32 rasterLines = c64RasterLines( m_Signals.video );
	if ( !c128RefreshBeginLineSync( cyclesPerLine, rasterLines ) )
		return false;

	// Core 3 now holds an exclusive traffic interval open. Consecutive reads
	// therefore sample every raster line rather than being separated by a
	// refresh epoch. A single low-byte change identifies the line transition;
	// the following high-bit read distinguishes 255->256 from frame wrap.
	u8 previous = 0;
	RAD_SPEEK( 0xD012, previous );
	m_Reads++;
	for ( u32 attempt = 0; attempt < 512; attempt++ )
	{
		u8 low = 0;
		RAD_SPEEK( 0xD012, low );
		m_Reads++;
		if ( low == previous ) continue;

		u8 high = 0;
		RAD_SPEEK( 0xD011, high );
		m_Reads++;
		const u16 line = (u16)( low | ( ( high & 0x80 ) ? 0x100 : 0 ) );
		if ( line < rasterLines )
			return c128RefreshCommitLineSync( line );
		previous = low;
	}

	c128RefreshCancelLineSync();
	return false;
}

void CRADBus::calibrateReadTiming()
{
	register u32 g2;
	static const u8 patterns[] = { 0x3C, 0xC3, 0x5A, 0xA5, 0x69, 0x96 };
	static const u16 firstSample = 300;
	static const u16 lastSample  = 620;
	static const u16 sampleStep  = 5;
	static const u32 repetitions = 12;

	const u16 configured = busTiming.WAIT_CYCLE_READ;
	m_ReadTimingConfigured = configured;

	// These bytes are in the cassette buffer and the existing self-test uses
	// the same locations. Writes do not depend on the read sample we are about
	// to calibrate. The VIC border register adds a non-DRAM device to the test;
	// its low nibble is a stable read-back value on every VIC-II revision.
	for ( u32 i = 0; i < sizeof( patterns ); i++ )
		RAD_SPOKE( (u16)( 0x0334 + i ), patterns[ i ] );
	RAD_SPOKE( 0xD020, 0x06 );
	m_Writes += sizeof( patterns ) + 1;

	u16 runStart = 0, runEnd = 0;
	u16 bestStart = 0, bestEnd = 0;
	u32 bestRunSamples = 0;

	for ( u16 sample = firstSample; sample <= lastSample; sample += sampleStep )
	{
		busTiming.WAIT_CYCLE_READ = sample;
		busTiming.WAIT_CYCLE_READ2 = (u16)( sample + 20 );

		u32 errors = 0;
		for ( u32 r = 0; r < repetitions; r++ )
		{
			for ( u32 i = 0; i < sizeof( patterns ); i++ )
			{
				u8 v;
				RAD_SPEEK( (u16)( 0x0334 + i ), v );
				if ( v != patterns[ i ] ) errors++;
			}

			u8 border;
		RAD_SPEEK( 0xD020, border );
		if ( ( border & 0x0F ) != 0x06 ) errors++;
		}
		m_Reads += repetitions * ( sizeof( patterns ) + 1 );

		if ( errors == 0 )
		{
			if ( runStart == 0 ) runStart = sample;
			runEnd = sample;
		}
		else if ( runStart != 0 )
		{
			const u32 runSamples = ( runEnd - runStart ) / sampleStep + 1;
			if ( runSamples > bestRunSamples )
			{
				bestRunSamples = runSamples;
				bestStart = runStart;
				bestEnd = runEnd;
			}
			runStart = runEnd = 0;
		}
	}

	if ( runStart != 0 )
	{
		const u32 runSamples = ( runEnd - runStart ) / sampleStep + 1;
		if ( runSamples > bestRunSamples )
		{
			bestRunSamples = runSamples;
			bestStart = runStart;
			bestEnd = runEnd;
		}
	}

	if ( bestRunSamples )
	{
		// Stay on the five-cycle grid while choosing the midpoint. The middle of
		// the eye has margin in both directions; using its first passing point
		// recreates the intermittent edge failure this calibration is fixing.
		const u16 chosen = (u16)( bestStart
		                 + ( ( ( bestEnd - bestStart ) / sampleStep ) / 2 ) * sampleStep );
		busTiming.WAIT_CYCLE_READ = chosen;
		busTiming.WAIT_CYCLE_READ2 = (u16)( chosen + 20 );
		m_ReadTimingStart = bestStart;
		m_ReadTimingEnd = bestEnd;
		m_ReadTimingSelected = chosen;
		RADLOG( "    read timing: configured %u, stable %u..%u, selected %u",
		        (unsigned)configured, (unsigned)bestStart,
		        (unsigned)bestEnd, (unsigned)chosen );
	}
	else
	{
		// Preserve an explicitly configured value if the probe cannot establish
		// a perfect window. The normal self-test will then reject an unsafe bus.
		busTiming.WAIT_CYCLE_READ = configured;
		busTiming.WAIT_CYCLE_READ2 = (u16)( configured + 20 );
		m_ReadTimingStart = m_ReadTimingEnd = 0;
		m_ReadTimingSelected = configured;
		RADLOG( "    read timing: no error-free window from %u..%u; retaining %u",
		        (unsigned)firstSample, (unsigned)lastSample,
		        (unsigned)configured );
	}

	// Return the border to the normal C64 power-on colour. The boot ROM will
	// shortly initialise the VIC itself, but leaving diagnostics unobtrusive is
	// useful when acquisition later fails for an unrelated reason.
	RAD_SPOKE( 0xD020, 0x0E );
	m_Writes++;
}

void CRADBus::calibrateWriteTiming()
{
	register u32 g2;
	static const u16 addrFirst = 340, addrLast = 520, timingStep = 10;
	static const u16 dataFirst = 540, dataLast = 800;
	static const u32 addrCount = ( addrLast - addrFirst ) / timingStep + 1;
	static const u32 dataCount = ( dataLast - dataFirst ) / timingStep + 1;
	static const u8 p0[ 4 ] = { 0x96, 0x69, 0x3C, 0xC3 };
	static const u8 p1[ 4 ] = { 0x5A, 0xA5, 0x0F, 0xF0 };
	u8 grid[ addrCount ][ dataCount ];

	m_WriteTimingConfiguredAddr = (u16)busTiming.TIMING_ENABLE_RWOUT_ADDR_LATCH_WRITING;
	m_WriteTimingConfiguredData = (u16)busTiming.TIMING_ENABLE_DATA_WRITING;
	m_WriteTimingSelectedAddr = m_WriteTimingConfiguredAddr;
	m_WriteTimingSelectedData = m_WriteTimingConfiguredData;
	m_WriteTimingPassingPoints = 0;

	for ( u32 ai = 0; ai < addrCount; ai++ )
	{
		const u16 at = (u16)( addrFirst + ai * timingStep );
		for ( u32 di = 0; di < dataCount; di++ )
		{
			const u16 dt = (u16)( dataFirst + di * timingStep );
			u32 errors = 0;
			busTiming.TIMING_ENABLE_RWOUT_ADDR_LATCH_WRITING = at;
			busTiming.TIMING_ENABLE_DATA_WRITING = dt;

			// Alternate write/read pairs exercise the direction transition that
			// failed on the C128, rather than hiding it in a run of writes.
			for ( u32 round = 0; round < 2; round++ )
			{
				const u8 *pat = round ? p1 : p0;
				for ( u32 i = 0; i < 4; i++ )
				{
					u8 got;
					RAD_SPOKE( (u16)( 0x0340 + i ), pat[ i ] );
					RAD_SPEEK( (u16)( 0x0340 + i ), got );
					if ( got != pat[ i ] ) errors++;
				}
			}

			// The mirror uses a separately sequenced burst path, so a candidate
			// is valid only if that path also works in both data states.
			for ( u32 round = 0; round < 2; round++ )
			{
				const u8 *pat = round ? p0 : p1;
				C64BusWrite burst[ 4 ];
				for ( u32 i = 0; i < 4; i++ )
				{
					burst[ i ].addr = (u16)( 0x0348 + i );
					burst[ i ].value = pat[ i ];
				}
				writeBurst( burst, 4 );
				for ( u32 i = 0; i < 4; i++ )
				{
					u8 got;
					RAD_SPEEK( burst[ i ].addr, got );
					if ( got != burst[ i ].value ) errors++;
				}
			}

			grid[ ai ][ di ] = (u8)( errors > 254 ? 254 : errors );
			if ( errors == 0 ) m_WriteTimingPassingPoints++;
		}
	}

	if ( m_WriteTimingPassingPoints )
	{
		// Pick the zero-error point with the largest zero-error neighbourhood,
		// not merely the first edge of the eye. This supplies useful margin in
		// both independently-scanned dimensions.
		u32 bestScore = 0, bestDistance = ~0u, bestA = 0, bestD = 0;
		for ( u32 ai = 0; ai < addrCount; ai++ )
		for ( u32 di = 0; di < dataCount; di++ )
		{
			if ( grid[ ai ][ di ] != 0 ) continue;
			u32 score = 0;
			for ( int da = -2; da <= 2; da++ )
			for ( int dd = -2; dd <= 2; dd++ )
			{
				const int na = (int)ai + da, nd = (int)di + dd;
				if ( na >= 0 && nd >= 0 && na < (int)addrCount
				     && nd < (int)dataCount && grid[ na ][ nd ] == 0 ) score++;
			}
			const u16 at = (u16)( addrFirst + ai * timingStep );
			const u16 dt = (u16)( dataFirst + di * timingStep );
			const int ad = (int)at - (int)m_WriteTimingConfiguredAddr;
			const int dtd = (int)dt - (int)m_WriteTimingConfiguredData;
			const u32 distance = (u32)( ad * ad + dtd * dtd );
			if ( score > bestScore || ( score == bestScore && distance < bestDistance ) )
			{
				bestScore = score; bestDistance = distance; bestA = ai; bestD = di;
			}
		}
		m_WriteTimingSelectedAddr = (u16)( addrFirst + bestA * timingStep );
		m_WriteTimingSelectedData = (u16)( dataFirst + bestD * timingStep );
	}

	busTiming.TIMING_ENABLE_RWOUT_ADDR_LATCH_WRITING = m_WriteTimingSelectedAddr;
	busTiming.TIMING_ENABLE_DATA_WRITING = m_WriteTimingSelectedData;
	RADLOG( "    write timing: configured %u/%u, %u passing points, selected %u/%u",
	        (unsigned)m_WriteTimingConfiguredAddr, (unsigned)m_WriteTimingConfiguredData,
	        (unsigned)m_WriteTimingPassingPoints, (unsigned)m_WriteTimingSelectedAddr,
	        (unsigned)m_WriteTimingSelectedData );
}

void CRADBus::release()
{
	if ( !m_Acquired )
		return;

	// The helper is the only owner allowed to release /DMA during normal C128
	// operation. Stop it and wait for /DMA-low acknowledgement before handing
	// the physical CPU back to the machine.
	c128RefreshStop();
	radReleaseCPU();
	m_Acquired = false;
}

void CRADBus::seedRefreshMap( u8 border )
{
	// Re-establish the complete VIC view before every retention phase. These
	// register writes are idempotent and deliberately repeated; this rule does
	// not apply to general CIA/VIC writes elsewhere in the emulator.
	static const u16 vicSetupAddr[ 7 ] =
		{ 0xDD02, 0xDD00, 0xD011, 0xD016, 0xD018, 0xD020, 0xD021 };
	const u8 vicSetupValue[ 7 ] =
		{ 0x03,   0x03,   0x1B,   0x08,   0x14,   border, 0x00 };
	for ( u32 i = 0; i < 7; i++ )
	{
		busDiagRawWrite( vicSetupAddr[ i ], vicSetupValue[ i ] );
		busDiagRawWrite( vicSetupAddr[ i ], vicSetupValue[ i ] );
	}

	// Clear the complete visible matrix first. Page $04 occupies the first 256
	// bytes (six full screen rows plus sixteen cells), while the independent
	// page-$07 control remains visible at screen row 19, column 8.
	for ( u32 i = 0; i < 1000; i++ )
	{
		busDiagRawWrite( (u16)( 0x0400 + i ), 0x20 );
		busDiagRawWrite( (u16)( 0x0400 + i ), 0x20 );
		busDiagRawWrite( (u16)( 0xD800 + i ), 0x01 );
		busDiagRawWrite( (u16)( 0xD800 + i ), 0x01 );
	}

	// This odd-multiplier permutation uses every possible byte value exactly
	// once. That exercises all data bits and prevents a decayed byte from being
	// hidden by a low-diversity or uniform screen pattern. The expected copy is
	// retained in Pi memory, never in the C128 DRAM under test.
	for ( u32 i = 0; i < 256; i++ )
		m_RefreshMapExpected[ i ] = (u8)( i * 73u + 0x35u );
	for ( u32 i = 0; i < 8; i++ )
		m_RefreshControlExpected[ i ] = (u8)( 17u + i );

	// Double-seed every oracle cell. The C128 occasionally loses the first
	// transfer after bus turnaround; two writes make this retention experiment
	// independent of that already-measured phenomenon.
	for ( u32 pass = 0; pass < 2; pass++ )
	{
		for ( u32 i = 0; i < 256; i++ )
			busDiagRawWrite( (u16)( 0x0400 + i ), m_RefreshMapExpected[ i ] );
		for ( u32 i = 0; i < 8; i++ )
			busDiagRawWrite( (u16)( 0x0700 + i ), m_RefreshControlExpected[ i ] );
	}
	m_Writes += 4542;
}

void CRADBus::seedRefreshOracle( u8 salt )
{
	const u64 started = hostCycles();
	// The phase scan does not need a readable display during every rung. The
	// display-oriented seeder above spends 4000 physical writes clearing screen
	// and colour cells which are not part of the oracle; that was long enough
	// for the already-bad production phase to damage the baseline itself.
	static const u16 vicSetupAddr[ 7 ] =
		{ 0xDD02, 0xDD00, 0xD011, 0xD016, 0xD018, 0xD020, 0xD021 };
	const u8 vicSetupValue[ 7 ] =
		{ 0x03,   0x03,   0x1B,   0x08,   0x14,   0x00,   0x00 };
	for ( u32 i = 0; i < 7; i++ )
	{
		busDiagRawWrite( vicSetupAddr[ i ], vicSetupValue[ i ] );
		busDiagRawWrite( vicSetupAddr[ i ], vicSetupValue[ i ] );
	}

	for ( u32 i = 0; i < 256; i++ )
		m_RefreshMapExpected[ i ] = (u8)( ( i * 73u + 0x35u ) ^ salt );
	for ( u32 i = 0; i < 8; i++ )
		m_RefreshControlExpected[ i ] = (u8)( ( 0xA7u + i * 29u ) ^ salt );
	for ( u32 pass = 0; pass < 2; pass++ )
	{
		for ( u32 i = 0; i < 256; i++ )
			busDiagRawWrite( (u16)( 0x0400 + i ), m_RefreshMapExpected[ i ] );
		for ( u32 i = 0; i < 8; i++ )
			busDiagRawWrite( (u16)( 0x0700 + i ), m_RefreshControlExpected[ i ] );
	}
	m_Writes += 542;
	const u64 elapsed = hostCycles() - started;
	m_LastRefreshSeedUsec =
		(u32)( elapsed / ( SCPU_ARM_CLOCK_HZ / 1000000u ) );
}

void CRADBus::captureRefreshMap()
{
	// E18L proved that the core-0 full-frame helper emitted the expected number
	// of pulses but was not effective refresh: later passes acquired failures.
	// E18M therefore makes observation a short, measured blackout with no
	// unproven pulse primitive between seed, read0, read1, or read2.
	const u64 captureStarted = hostCycles();
	for ( u32 pass = 0; pass < 3; pass++ )
	{
		const u64 passStarted = hostCycles();
		for ( u32 cell = 0; cell < 256; cell++ )
			m_RefreshMapReadback[ pass ][ cell ] =
				busDiagRawRead( (u16)( 0x0400 + cell ) );
		for ( u32 cell = 0; cell < 8; cell++ )
			m_RefreshControlReadback[ pass ][ cell ] =
				busDiagRawRead( (u16)( 0x0700 + cell ) );
		m_LastRefreshCapturePassUsec[ pass ] =
			(u32)( ( hostCycles() - passStarted )
			       / ( SCPU_ARM_CLOCK_HZ / 1000000u ) );
	}
	m_LastRefreshCaptureUsec =
		(u32)( ( hostCycles() - captureStarted )
		       / ( SCPU_ARM_CLOCK_HZ / 1000000u ) );
	m_Reads += 792;

	m_RefreshFailureCount = 0;
	m_RefreshUnstableCount = 0;
	for ( u32 i = 0; i < 32; i++ ) m_RefreshFailureBitmap[ i ] = 0;
	for ( u32 cell = 0; cell < 256; cell++ )
	{
		const u8 a = m_RefreshMapReadback[ 1 ][ cell ];
		const u8 b = m_RefreshMapReadback[ 2 ][ cell ];
		const bool pass = a == m_RefreshMapExpected[ cell ]
		               && b == m_RefreshMapExpected[ cell ];
		if ( !pass )
		{
			m_RefreshFailureBitmap[ cell >> 3 ] |= (u8)( 1u << ( cell & 7 ) );
			m_RefreshFailureCount++;
		}
		if ( a != b ) m_RefreshUnstableCount++;
	}
}

bool CRADBus::runVICRefreshSlot()
{
	register u32 g2;

	// /DMA is low on entry. Wait for the falling PHI2 edge that starts the
	// VIC-owned half, then release it only inside that half. Reassert it after
	// 400 ARM cycles (~286ns at 1.4GHz), leaving over 200ns before the next CPU
	// edge. The 8502 therefore sees /DMA asserted whenever it could own the bus,
	// while the C128 MMU/VIC sees an ordinary refresh interval.
	WAIT_FOR_VIC_HALFCYCLE_EDGE
	RESTART_CYCLE_COUNTER
	SET_GPIO( bDMA_OUT );
	WAIT_UP_TO_CYCLE( 400 );
	CLR_GPIO( bDMA_OUT );

	// Sample R/W once the nominal CPU half begins. It must never be low: that
	// would mean the 8502 escaped the DMA hold and attempted a write.
	WAIT_FOR_CPU_HALFCYCLE
	g2 = read32( ARM_GPIO_GPLEV0 );
	return !( g2 & bRW_OUT );
}

u32 CRADBus::core0RefreshRasterLine()
{
	// Used only while core 3 is stopped and /DMA is asserted. This advances
	// through one complete physical raster line. It includes that line's VIC
	// refresh slots, but it is NOT a complete DRAM-row refresh sweep: a VIC-IIe
	// supplies only a small subset of its row refresh addresses on each line.
	const u32 cycles = c64CyclesPerLine( m_Signals.video );
	for ( u32 i = 0; i < cycles; i++ ) (void)runVICRefreshSlot();
	return cycles;
}

u32 CRADBus::core0RefreshFullFrame()
{
	// A complete video frame is deliberately stronger than the inferred
	// minimum number of raster lines. It covers all refresh-row addresses even
	// if the number issued per line differs from the topology being measured,
	// so the diagnostic harness does not assume its own answer.
	const u32 lines = c64RasterLines( m_Signals.video );
	const u32 expected = lines * c64CyclesPerLine( m_Signals.video );
	const u64 started = hostCycles();
	u32 pulses = 0;
	for ( u32 line = 0; line < lines; line++ )
		pulses += core0RefreshRasterLine();
	const u64 elapsed = hostCycles() - started;
	const u32 usec = (u32)( elapsed / ( SCPU_ARM_CLOCK_HZ / 1000000u ) );
	m_Core0FrameSweeps++;
	m_Core0FramePulses += pulses;
	m_Core0FrameExpectedPulses += expected;
	if ( pulses != expected ) m_Core0FrameAccountingErrors++;
	if ( usec < m_Core0FrameMinUsec ) m_Core0FrameMinUsec = usec;
	if ( usec > m_Core0FrameMaxUsec ) m_Core0FrameMaxUsec = usec;
	return pulses;
}

void CRADBus::runRefreshFiniteTests()
{
	// Green marks the experimental fix phase in kernel144. After seeding, every
	// RAD bus driver is disabled and the only activity is /DMA being released
	// during the safe interior of each VIC-owned half-cycle.
	seedRefreshMap( 5 );
	DISABLE_ADDRESS_LATCH_AND_BUSTRANSCEIVER( false )
	CLR_GPIO( bMPLEX_SEL );

	const u64 start = hostCycles();
	const u64 target = (u64)60 * (u64)SCPU_ARM_CLOCK_HZ;
	u64 elapsed = 0;
	u32 sweeps = 0;
	u32 hostWrites = 0;
	do
	{
		if ( runVICRefreshSlot() ) hostWrites++;
		sweeps++;
		elapsed = hostCycles() - start;
	} while ( elapsed < target );

	m_RefreshSweepCount = sweeps;
	m_RefreshElapsedMs =
		(u32)( elapsed / ( SCPU_ARM_CLOCK_HZ / 1000u ) );
	const u64 elapsedUsec = elapsed / ( SCPU_ARM_CLOCK_HZ / 1000000u );
	m_RefreshPassUsec = sweeps ? (u32)( elapsedUsec / sweeps ) : 0;
	m_RefreshSweepChecksum = 0;
	m_RefreshHostWritePhases = hostWrites;

	// Read the physical RAM before drawing the result map. This separates true
	// content loss from a VIC pointer/fetch failure.
	captureRefreshMap();
	RADLOG( "  refresh VIC-half DMA release: %u slots in %u ms, %u us/slot, failures=%u unstable=%u host-writes=%u",
	        (unsigned)sweeps, (unsigned)m_RefreshElapsedMs,
	        (unsigned)m_RefreshPassUsec, (unsigned)m_RefreshFailureCount,
	        (unsigned)m_RefreshUnstableCount, (unsigned)m_RefreshHostWritePhases );
}

void CRADBus::runRefreshGapLadder()
{
	if ( m_Signals.machine != MACHINE_C128 || !c128RefreshActive() )
		return;

	static const u32 gapUsec[ 6 ] = { 250, 500, 1000, 2000, 4000, 8000 };
	const u64 cyclesPerUsec = SCPU_ARM_CLOCK_HZ / 1000000u;
	const u64 normalCycles = (u64)10 * (u64)SCPU_ARM_CLOCK_HZ;

	for ( u32 rung = 0; rung < 6; rung++ )
	{
		// Every rung begins from a fresh oracle. While seeding/capturing, core 0
		// supplies inline refresh pulses after each physical access; during the
		// ten-second quiet phase core 3 owns every available VIC half.
		seedRefreshMap( (u8)( 5 + rung ) );
		const u64 normalStart = hostCycles();
		while ( hostCycles() - normalStart < normalCycles )
			asm volatile( "yield" );

		// No bus operation occurs while the helper is stopped. /DMA stays low,
		// so this is a clean refresh-suspension interval rather than contention.
		c128RefreshStop();
		const u64 gapStart = hostCycles();
		const u64 gapCycles = (u64)gapUsec[ rung ] * cyclesPerUsec;
		while ( hostCycles() - gapStart < gapCycles )
			asm volatile( "yield" );
		if ( !c128RefreshStart() )
		{
			RADLOG( "  refresh gap %u us: helper restart FAILED",
			        (unsigned)gapUsec[ rung ] );
			return;
		}

		captureRefreshMap();
		m_RefreshGapFailures[ rung ] = (u16)m_RefreshFailureCount;
		m_RefreshGapUnstable[ rung ] = (u16)m_RefreshUnstableCount;
		RADLOG( "  refresh gap %u us: failures=%u unstable=%u",
		        (unsigned)gapUsec[ rung ], (unsigned)m_RefreshFailureCount,
		        (unsigned)m_RefreshUnstableCount );
	}
}

bool CRADBus::prepareRefreshControl( u32 rung, u8 salt )
{
	if ( rung >= REFRESH_CONTROL_COUNT
	     || m_Signals.machine != MACHINE_C128 ) return false;
	c128RefreshCancelLineSync();
	c128RefreshStop();
	bool seeded = false;
	for ( u32 attempt = 0; attempt < 2 && !seeded; attempt++ )
	{
		m_RefreshControlBaselineAttempts[ rung ] = (u8)( attempt + 1 );
		// A retry repeats the identical physical data pattern. In E18M the two
		// long-duration controls must remain comparable even if only one needs a
		// second baseline attempt.
		const u32 sweepStart = m_Core0FrameSweeps;
		const u32 pulseStart = m_Core0FramePulses;
		const u32 expectedStart = m_Core0FrameExpectedPulses;
		seedRefreshOracle( salt );
		captureRefreshMap();
		m_RefreshControlBaselineSeedUsec[ rung ][ attempt ] =
			m_LastRefreshSeedUsec;
		m_RefreshControlBaselineCaptureUsec[ rung ][ attempt ] =
			m_LastRefreshCaptureUsec;
		for ( u32 pass = 0; pass < 3; pass++ )
			m_RefreshControlBaselinePassUsec[ rung ][ attempt ][ pass ] =
				m_LastRefreshCapturePassUsec[ pass ];
		m_RefreshControlBaselineFrameSweeps[ rung ][ attempt ] =
			(u16)( m_Core0FrameSweeps - sweepStart );
		m_RefreshControlBaselineFramePulses[ rung ][ attempt ] =
			m_Core0FramePulses - pulseStart;
		m_RefreshControlBaselineFrameExpected[ rung ][ attempt ] =
			m_Core0FrameExpectedPulses - expectedStart;

		for ( u32 pass = 0; pass < 3; pass++ )
		{
			u32 passFailures = 0;
			for ( u32 cell = 0; cell < 264; cell++ )
			{
				const u8 expected = cell < 256
				                  ? m_RefreshMapExpected[ cell ]
				                  : m_RefreshControlExpected[ cell - 256 ];
				const u8 actual = cell < 256
				                ? m_RefreshMapReadback[ pass ][ cell ]
				                : m_RefreshControlReadback[ pass ][ cell - 256 ];
				if ( actual != expected ) passFailures++;
			}
			m_RefreshControlBaselinePassFailures
				[ rung ][ attempt ][ pass ] = (u16)passFailures;
		}
		for ( u32 pair = 0; pair < 2; pair++ )
		{
			u32 pairUnstable = 0;
			for ( u32 cell = 0; cell < 264; cell++ )
			{
				const u8 a = cell < 256
				           ? m_RefreshMapReadback[ pair ][ cell ]
				           : m_RefreshControlReadback[ pair ][ cell - 256 ];
				const u8 b = cell < 256
				           ? m_RefreshMapReadback[ pair + 1 ][ cell ]
				           : m_RefreshControlReadback[ pair + 1 ][ cell - 256 ];
				if ( a != b ) pairUnstable++;
			}
			m_RefreshControlBaselinePassUnstable
				[ rung ][ attempt ][ pair ] = (u16)pairUnstable;
		}
		u32 failures = m_RefreshFailureCount;
		u32 unstable = m_RefreshUnstableCount;
		for ( u32 cell = 0; cell < 8; cell++ )
		{
			const u8 a = m_RefreshControlReadback[ 1 ][ cell ];
			const u8 b = m_RefreshControlReadback[ 2 ][ cell ];
			if ( a != m_RefreshControlExpected[ cell ]
			     || b != m_RefreshControlExpected[ cell ] ) failures++;
			if ( a != b ) unstable++;
		}
		m_RefreshControlBaselineFailures[ rung ] = (u16)failures;
		m_RefreshControlBaselineUnstable[ rung ] = (u16)unstable;
		seeded = failures == 0 && unstable == 0;
	}
	// A failed baseline makes the eventual map UNTRUSTED, but it is not a
	// mechanical failure. Keep core 3 stopped so the selected exposure can run
	// and preserve the raw result instead of suppressing the entire scan.
	return true;
}

void CRADBus::recordRefreshControl( u32 rung )
{
	if ( rung >= REFRESH_CONTROL_COUNT ) return;
	m_RefreshControlCaptureUsec[ rung ] = m_LastRefreshCaptureUsec;
	for ( u32 pass = 0; pass < 3; pass++ )
		m_RefreshControlCapturePassUsec[ rung ][ pass ] =
			m_LastRefreshCapturePassUsec[ pass ];
	for ( u32 pass = 0; pass < 3; pass++ )
	{
		u32 failures = 0;
		for ( u32 byte = 0; byte < 33; byte++ )
			m_RefreshControlMismatchMask[ rung ][ pass ][ byte ] = 0;
		for ( u32 cell = 0; cell < 264; cell++ )
		{
			const u8 expected = cell < 256
			                  ? m_RefreshMapExpected[ cell ]
			                  : m_RefreshControlExpected[ cell - 256 ];
			const u8 actual = cell < 256
			                ? m_RefreshMapReadback[ pass ][ cell ]
			                : m_RefreshControlReadback[ pass ][ cell - 256 ];
			if ( actual != expected )
			{
				m_RefreshControlMismatchMask[ rung ][ pass ][ cell >> 3 ]
					|= (u8)( 1u << ( cell & 7 ) );
				failures++;
			}
		}
		m_RefreshControlPassFailures[ rung ][ pass ] = (u16)failures;
	}
	u32 unstable01 = 0, unstable12 = 0;
	for ( u32 cell = 0; cell < 264; cell++ )
	{
		const u8 a = cell < 256 ? m_RefreshMapReadback[ 0 ][ cell ]
		                        : m_RefreshControlReadback[ 0 ][ cell - 256 ];
		const u8 b = cell < 256 ? m_RefreshMapReadback[ 1 ][ cell ]
		                        : m_RefreshControlReadback[ 1 ][ cell - 256 ];
		const u8 c = cell < 256 ? m_RefreshMapReadback[ 2 ][ cell ]
		                        : m_RefreshControlReadback[ 2 ][ cell - 256 ];
		if ( a != b ) unstable01++;
		if ( b != c ) unstable12++;
	}
	m_RefreshControlUnstable01[ rung ] = (u16)unstable01;
	m_RefreshControlUnstable12[ rung ] = (u16)unstable12;
	m_RefreshControlValid[ rung ] = 1;
}

bool CRADBus::refreshControlStrict( u32 rung ) const
{
	if ( rung >= REFRESH_CONTROL_COUNT || !m_RefreshControlValid[ rung ]
	     || m_RefreshControlBaselineFailures[ rung ] != 0
	     || m_RefreshControlBaselineUnstable[ rung ] != 0
	     || m_RefreshControlUnstable01[ rung ] != 0
	     || m_RefreshControlUnstable12[ rung ] != 0 )
		return false;
	for ( u32 pass = 0; pass < 3; pass++ )
		if ( m_RefreshControlPassFailures[ rung ][ pass ] != 0 ) return false;
	return true;
}

bool CRADBus::refreshWindowAccountingExact( u32 rung, u32 expectedPulses,
	                                         u32 expectedLines ) const
{
	return rung < REFRESH_CONTROL_COUNT
	    && m_RefreshControlPulses[ rung ] == expectedPulses
	    && m_RefreshControlExpectedPulses[ rung ] == expectedPulses
	    && m_RefreshControlExposureLines[ rung ] == expectedLines;
}

bool CRADBus::refreshExposureControlAcceptable( u32 rung,
	                                             u32 expectedPulses,
	                                             u32 expectedLines ) const
{
	if ( rung >= REFRESH_CONTROL_COUNT || !m_RefreshControlValid[ rung ]
	     || m_RefreshControlBaselineFailures[ rung ] != 0
	     || m_RefreshControlBaselineUnstable[ rung ] != 0
	     || m_RefreshControlUnstable01[ rung ] != 0
	     || m_RefreshControlUnstable12[ rung ] != 0
	     || !refreshWindowAccountingExact(
	            rung, expectedPulses, expectedLines ) )
		return false;

	// Exposure-bearing controls use the same measured background allowance as
	// topology P: at most five unique stable cells. Zero-exposure floors never
	// call this helper and retain strict all-zero acceptance.
	u32 uniqueFailures = 0;
	for ( u32 byte = 0; byte < 33; byte++ )
	{
		u8 v = 0;
		for ( u32 pass = 0; pass < 3; pass++ )
			v |= m_RefreshControlMismatchMask[ rung ][ pass ][ byte ];
		while ( v ) { uniqueFailures += v & 1u; v >>= 1; }
	}
	return uniqueFailures <= 5;
}

bool CRADBus::runRefreshWindowControl( u32 rung, u32 k, u32 w,
	                                   u32 exposureLines )
{
	if ( rung >= REFRESH_CONTROL_COUNT || !c128RefreshActive() ) return false;
	m_PhaseScanBaselineK = (u8)k;
	m_PhaseScanBaselineW = (u8)w;
	const u8 salt =
		(u8)( 0x31u + rung * 0x27u + k * 29u + w * 17u );
	if ( !prepareRefreshControl( rung, salt ) )
		return false;
	// The baseline capture is observational. Re-seed immediately before the
	// timed exposure so every rung starts at the same measured cell age.
	seedRefreshOracle( salt );
	if ( !c128RefreshStart() ) return false;
	if ( !c128RefreshConfigureWindowScan( k, w, exposureLines )
	     || !startC128LineRefresh() )
	{
		c128RefreshCancelLineSync();
		return false;
	}

	const u32 linesPerSecond = c64RasterLines( m_Signals.video ) * 60u;
	const u32 nominalSeconds = linesPerSecond
	                         ? ( exposureLines + linesPerSecond - 1 )
	                           / linesPerSecond : 0;
	const u64 timeout = (u64)( nominalSeconds + 4u )
	                  * (u64)SCPU_ARM_CLOCK_HZ;
	const u64 started = hostCycles();
	while ( !c128RefreshWindowScanComplete()
	        && hostCycles() - started < timeout )
		asm volatile( "yield" );
	m_RefreshControlExposureUsec[ rung ] =
		(u32)( ( hostCycles() - started )
		       / ( SCPU_ARM_CLOCK_HZ / 1000000u ) );
	if ( !c128RefreshWindowScanComplete() )
	{
		c128RefreshCancelLineSync();
		return false;
	}

	m_RefreshControlAnchorArm[ rung ] =
		c128RefreshWindowScanAnchorArmCycles();
	const u32 phi = c128RefreshWindowScanAnchorPhiCycles();
	m_RefreshControlAnchorPhi[ rung ] =
		(u8)( phi > 255u ? 255u : phi );
	m_RefreshControlPulses[ rung ] =
		(u32)c128RefreshWindowScanExposurePulses();
	m_RefreshControlExpectedPulses[ rung ] = w * exposureLines;
	m_RefreshControlExposureLines[ rung ] =
		c128RefreshWindowScanExposureLines();

	c128RefreshCancelLineSync();
	c128RefreshStop();
	captureRefreshMap();
	recordRefreshControl( rung );
	return c128RefreshStart();
}

bool CRADBus::runRefreshContinuousControl( u32 rung, u32 exposureSeconds,
	                                       u8 salt )
{
	if ( rung >= REFRESH_CONTROL_COUNT || !c128RefreshActive()
	     || exposureSeconds == 0 )
		return false;
	if ( !prepareRefreshControl( rung, salt ) )
		return false;
	seedRefreshOracle( salt );

	// Set the force flag before starting core 3, so its first no-line-sync
	// epoch enters the exact kernel144 tight loop and never opens traffic.
	c128RefreshForceContinuous( true );
	if ( !c128RefreshStart() )
	{
		c128RefreshForceContinuous( false );
		return false;
	}
	const u64 pulseStart = c128RefreshSlots();
	const u64 started = hostCycles();
	const u64 duration = (u64)exposureSeconds * (u64)SCPU_ARM_CLOCK_HZ;
	while ( hostCycles() - started < duration ) asm volatile( "yield" );
	m_RefreshControlExposureUsec[ rung ] =
		(u32)( ( hostCycles() - started )
		       / ( SCPU_ARM_CLOCK_HZ / 1000000u ) );
	c128RefreshStop();
	const u64 pulseEnd = c128RefreshSlots();
	c128RefreshForceContinuous( false );
	m_RefreshControlPulses[ rung ] = (u32)( pulseEnd - pulseStart );
	m_RefreshControlExpectedPulses[ rung ] = 0; // wall-time control, not line-counted
	m_RefreshControlExposureLines[ rung ] = 0;
	captureRefreshMap();
	recordRefreshControl( rung );
	return c128RefreshStart();
}

bool CRADBus::runRefreshCore0ContinuousControl( u32 rung,
	                                             u32 exposureSeconds,
	                                             u8 salt )
{
	if ( rung >= REFRESH_CONTROL_COUNT || !c128RefreshActive()
	     || exposureSeconds == 0 ) return false;
	if ( !prepareRefreshControl( rung, salt ) ) return false;
	seedRefreshOracle( salt );

	// Exercise the exact core-0 full-frame helper that E18L used inside every
	// baseline. Core 3 remains stopped, so emitter core is the only variable
	// relative to the matched literal-continuous control.
	const u32 pulseStart = m_Core0FramePulses;
	const u64 started = hostCycles();
	const u64 duration = (u64)exposureSeconds * (u64)SCPU_ARM_CLOCK_HZ;
	do { (void)core0RefreshFullFrame(); }
	while ( hostCycles() - started < duration );
	m_RefreshControlExposureUsec[ rung ] =
		(u32)( ( hostCycles() - started )
		       / ( SCPU_ARM_CLOCK_HZ / 1000000u ) );
	m_RefreshControlPulses[ rung ] = m_Core0FramePulses - pulseStart;
	m_RefreshControlExpectedPulses[ rung ] = 0;
	m_RefreshControlExposureLines[ rung ] = 0;
	captureRefreshMap();
	recordRefreshControl( rung );
	return c128RefreshStart();
}

bool CRADBus::runRefreshBlackoutLongControl( u32 rung, u8 salt )
{
	if ( rung >= REFRESH_CONTROL_COUNT || !c128RefreshActive() ) return false;
	if ( !prepareRefreshControl( rung, salt ) ) return false;
	const u32 attempt = m_RefreshControlBaselineAttempts[ rung ]
	                  ? m_RefreshControlBaselineAttempts[ rung ] - 1u : 0u;
	const u32 shortSpanUsec =
		m_RefreshControlBaselineSeedUsec[ rung ][ attempt ]
		+ m_RefreshControlBaselineCaptureUsec[ rung ][ attempt ];
	m_BlackoutLongDelayUsec = shortSpanUsec;

	// This is separately seeded from SHORT. The deliberate no-refresh delay is
	// equal to SHORT's measured seed-plus-capture span, placing the oldest cell
	// at approximately twice the ordinary observation age without a reseed in
	// the interval being tested.
	seedRefreshOracle( salt );
	const u64 delayStarted = hostCycles();
	const u64 delayCycles = (u64)shortSpanUsec
	                      * ( SCPU_ARM_CLOCK_HZ / 1000000u );
	while ( hostCycles() - delayStarted < delayCycles ) asm volatile( "yield" );
	m_RefreshControlExposureUsec[ rung ] =
		(u32)( ( hostCycles() - delayStarted )
		       / ( SCPU_ARM_CLOCK_HZ / 1000000u ) );
	m_RefreshControlPulses[ rung ] = 0;
	m_RefreshControlExpectedPulses[ rung ] = 0;
	m_RefreshControlExposureLines[ rung ] = 0;
	captureRefreshMap();
	recordRefreshControl( rung );
	return c128RefreshStart();
}

bool CRADBus::runRefreshContinuousWarmup( u32 exposureSeconds )
{
	if ( !c128RefreshActive() || exposureSeconds == 0 ) return false;
	// Eliminate every inherited coarse/line-scheduler state before measuring
	// the cold boot. No oracle exists yet, so this changes no diagnostic data;
	// it only establishes the exact kernel144 refresh regime deterministically.
	c128RefreshCancelLineSync();
	c128RefreshStop();
	c128RefreshForceContinuous( true );
	if ( !c128RefreshStart() )
	{
		c128RefreshForceContinuous( false );
		return false;
	}
	const u64 pulseStart = c128RefreshSlots();
	const u64 started = hostCycles();
	const u64 duration = (u64)exposureSeconds * (u64)SCPU_ARM_CLOCK_HZ;
	while ( hostCycles() - started < duration ) asm volatile( "yield" );
	c128RefreshStop();
	m_RefreshWarmupPulses = (u32)( c128RefreshSlots() - pulseStart );
	c128RefreshForceContinuous( false );
	return c128RefreshStart();
}

static u32 refreshMaskBitCount( const u8 *mask, u32 bytes )
{
	u32 count = 0;
	for ( u32 i = 0; i < bytes; i++ )
	{
		u8 v = mask[ i ];
		while ( v ) { count += v & 1u; v >>= 1; }
	}
	return count;
}

bool CRADBus::runRefreshPhaseScan()
{
	if ( m_Signals.machine != MACHINE_C128 || !c128RefreshActive() )
		return false;

	// E18N: E18M showed a genuine phase-local loss spike, but that spike is
	// already inside production's protected interval. It also showed residual
	// loss under both continuous emitters. Scan the one parameter common to
	// those paths: how long /DMA is released inside the VIC half. Every rung
	// uses the same salt, two-second exposure, ordinary core-3 tight loop, and
	// retains all three reads. Read 0 is the decision measurement; later reads
	// are an audit of observation-time decay.
	static const u16 pulseWidths[ REFRESH_PULSE_SCAN_COUNT ] =
		{ 280, 340, 400, 440, 480, 520, 560 };
	m_PulseScanRan = 1;
	m_PulseScanCompleted = 0;
	m_PhaseScanRan = 1;
	m_PhaseScanStage1Agree = 0;
	m_PhaseScanStage2Ran = 0;
	m_PhaseScanStage2RungsCompleted = 0;
	for ( u32 rung = 0; rung < REFRESH_CONTROL_COUNT; rung++ )
		m_RefreshControlValid[ rung ] = 0;

	for ( u32 rung = 0; rung < REFRESH_PULSE_SCAN_COUNT; rung++ )
	{
		m_PulseScanWidth[ rung ] = pulseWidths[ rung ];
		c128RefreshCancelLineSync();
		c128RefreshStop();
		c128RefreshSetDMAReleaseCycles( pulseWidths[ rung ] );
		if ( !c128RefreshStart() ) goto pulseScanFailed;
		if ( !runRefreshContinuousControl( rung, 2, 0xD6 ) )
			goto pulseScanFailed;
		m_PhaseScanStage2RungsCompleted = (u8)( rung + 1 );
	}

	c128RefreshCancelLineSync();
	c128RefreshStop();
	c128RefreshSetDMAReleaseCycles( 480 );
	if ( !c128RefreshStart() ) return false;
	m_PulseScanCompleted = 1;
	m_PhaseScanStage1Agree = 1;
	m_PhaseScanStage2Ran = 1;
	return true;

pulseScanFailed:
	c128RefreshCancelLineSync();
	c128RefreshStop();
	c128RefreshSetDMAReleaseCycles( 480 );
	(void)c128RefreshStart();
	return false;

	// E18M topology implementation is retained below for audit and can be
	// re-enabled after the pulse-width result is incorporated.
#if 0
	m_PhaseScanRan = 1;
	m_PhaseScanStage1Agree = 0;
	m_PhaseScanStage2Ran = 0;
	m_PhaseScanStage2Contiguous = 0;
	m_PhaseScanStage2PassCount = 0;
	m_PhaseScanStage2FailureCount = 0;
	m_PhaseScanStage2FlagCount = 0;
	m_PhaseScanStage2RungsCompleted = 0;
	m_PhaseScanStage2BandStart = 0xFF;
	m_PhaseScanLineCycles = (u8)c64CyclesPerLine( m_Signals.video );
	for ( u32 rung = 0; rung < REFRESH_CONTROL_COUNT; rung++ )
		m_RefreshControlValid[ rung ] = 0;
	for ( u32 k = 0; k < 65; k++ )
	{
		m_PhaseScanStage2Failures[ k ] = 0xFFFF;
		m_PhaseScanStage2BaselineFailures[ k ] = 0xFFFF;
		m_PhaseScanStage2BaselineUnstable[ k ] = 0xFFFF;
		m_PhaseScanStage2BaselineAttempts[ k ] = 0;
		m_PhaseScanStage2ExecutionIndex[ k ] = 0xFF;
		m_PhaseScanExecutionK[ k ] = 0xFF;
		m_PhaseScanStage2BaselineSeedUsec[ k ] = 0;
		m_PhaseScanStage2BaselineCaptureUsec[ k ] = 0;
		m_PhaseScanStage2CaptureUsec[ k ] = 0;
		m_PhaseScanStage2Unstable[ k ] = 0xFFFF;
		m_PhaseScanStage2Class[ k ] = '!';
		for ( u32 pass = 0; pass < 3; pass++ )
		{
			m_PhaseScanStage2PassFailures[ k ][ pass ] = 0xFFFF;
			for ( u32 byte = 0; byte < 33; byte++ )
				m_PhaseScanStage2MismatchMask[ k ][ pass ][ byte ] = 0;
		}
	}
	m_RefreshWarmupPulses = 0;
	if ( !runRefreshContinuousWarmup( 2 ) ) return false;

	const u32 exposureLines = c64RasterLines( m_Signals.video ) * 60u * 2u;
	const u32 fullPulses = (u32)m_PhaseScanLineCycles * exposureLines;
	const u8 stressSalt = 0x2A;

	// The two immediate floors validate the refresh-free observation path. LONG
	// doubles its measured blackout age. Matched two-second core-3/core-0
	// controls isolate emitter effectiveness. Continuous controls remain rate
	// diagnostics; K/W-full2 exercises the production-relevant scheduler.
	if ( !runRefreshWindowControl( REFRESH_PRE_FLOOR0, 0,
	                              m_PhaseScanLineCycles, 0 ) ) return false;
	if ( !runRefreshWindowControl( REFRESH_PRE_FLOOR1, 0,
	                              m_PhaseScanLineCycles, 0 ) ) return false;
	if ( !runRefreshBlackoutLongControl(
	         REFRESH_PRE_BLACKOUT_LONG, 0xB7 ) ) return false;
	const u8 emitterSalt = 0xD6;
	if ( !runRefreshContinuousControl(
	         REFRESH_PRE_CONT2, 2, emitterSalt ) ) return false;
	if ( !runRefreshCore0ContinuousControl(
	         REFRESH_PRE_CORE0_CONT2, 2, emitterSalt ) ) return false;
	if ( !runRefreshContinuousControl(
	         REFRESH_PRE_CONT8_A, 8, stressSalt ) ) return false;
	if ( !runRefreshWindowControl( REFRESH_PRE_KWFULL2, 0,
	                              m_PhaseScanLineCycles,
	                              exposureLines ) ) return false;
	if ( !runRefreshContinuousControl(
	         REFRESH_PRE_CONT8_B, 8, stressSalt ) ) return false;

	bool controlsTrusted =
		refreshControlStrict( REFRESH_PRE_FLOOR0 )
		&& refreshControlStrict( REFRESH_PRE_FLOOR1 )
		&& refreshWindowAccountingExact( REFRESH_PRE_FLOOR0, 0, 0 )
		&& refreshWindowAccountingExact( REFRESH_PRE_FLOOR1, 0, 0 )
		&& refreshExposureControlAcceptable(
		       REFRESH_PRE_KWFULL2, fullPulses, exposureLines )
		&& m_Core0FrameAccountingErrors == 0;

	// Omit one edge at a time. Classification is informational and operates on
	// the unique union of all three masks: P <=5, F >=20, ? otherwise. Any
	// instability or accounting error is always ?, never P.
	const u32 protectedWidth = m_PhaseScanLineCycles - 1;
	const u32 expectedPulses = protectedWidth * exposureLines;
	for ( u32 executionIndex = 0;
	      executionIndex < m_PhaseScanLineCycles; executionIndex++ )
	{
		const u32 k = ( executionIndex * 31u ) % m_PhaseScanLineCycles;
		m_PhaseScanExecutionK[ executionIndex ] = (u8)k;
		m_PhaseScanStage2ExecutionIndex[ k ] = (u8)executionIndex;
		if ( !runRefreshWindowControl( REFRESH_TOPOLOGY_SCRATCH, k,
		                              protectedWidth, exposureLines ) ) return false;
		m_PhaseScanStage2BaselineAttempts[ k ] =
			m_RefreshControlBaselineAttempts[ REFRESH_TOPOLOGY_SCRATCH ];
		m_PhaseScanStage2BaselineFailures[ k ] =
			m_RefreshControlBaselineFailures[ REFRESH_TOPOLOGY_SCRATCH ];
		m_PhaseScanStage2BaselineUnstable[ k ] =
			m_RefreshControlBaselineUnstable[ REFRESH_TOPOLOGY_SCRATCH ];
		const u32 baselineAttempt =
			m_PhaseScanStage2BaselineAttempts[ k ]
			? m_PhaseScanStage2BaselineAttempts[ k ] - 1u : 0u;
		m_PhaseScanStage2BaselineSeedUsec[ k ] =
			m_RefreshControlBaselineSeedUsec
				[ REFRESH_TOPOLOGY_SCRATCH ][ baselineAttempt ];
		m_PhaseScanStage2BaselineCaptureUsec[ k ] =
			m_RefreshControlBaselineCaptureUsec
				[ REFRESH_TOPOLOGY_SCRATCH ][ baselineAttempt ];
		m_PhaseScanStage2CaptureUsec[ k ] =
			m_RefreshControlCaptureUsec[ REFRESH_TOPOLOGY_SCRATCH ];
		const bool baselineClean =
			m_PhaseScanStage2BaselineFailures[ k ] == 0
			&& m_PhaseScanStage2BaselineUnstable[ k ] == 0;
		if ( !baselineClean ) controlsTrusted = false;

		u8 unionMask[ 33 ];
		for ( u32 byte = 0; byte < 33; byte++ ) unionMask[ byte ] = 0;
		for ( u32 pass = 0; pass < 3; pass++ )
		{
			m_PhaseScanStage2PassFailures[ k ][ pass ] =
				m_RefreshControlPassFailures[ REFRESH_TOPOLOGY_SCRATCH ][ pass ];
			for ( u32 byte = 0; byte < 33; byte++ )
			{
				const u8 v = m_RefreshControlMismatchMask
					[ REFRESH_TOPOLOGY_SCRATCH ][ pass ][ byte ];
				m_PhaseScanStage2MismatchMask[ k ][ pass ][ byte ] = v;
				unionMask[ byte ] |= v;
			}
		}
		const u32 uniqueFailures = refreshMaskBitCount( unionMask, 33 );
		const u32 unstable =
			m_RefreshControlUnstable01[ REFRESH_TOPOLOGY_SCRATCH ]
		  + m_RefreshControlUnstable12[ REFRESH_TOPOLOGY_SCRATCH ];
		m_PhaseScanStage2Failures[ k ] = (u16)uniqueFailures;
		m_PhaseScanStage2Unstable[ k ] =
			(u16)( unstable > 0xFFFFu ? 0xFFFFu : unstable );
		m_PhaseScanStage2Pulses[ k ] =
			m_RefreshControlPulses[ REFRESH_TOPOLOGY_SCRATCH ];
		m_PhaseScanStage2Lines[ k ] =
			m_RefreshControlExposureLines[ REFRESH_TOPOLOGY_SCRATCH ];
		m_PhaseScanStage2AnchorArm[ k ] =
			m_RefreshControlAnchorArm[ REFRESH_TOPOLOGY_SCRATCH ];
		m_PhaseScanStage2AnchorPhi[ k ] =
			m_RefreshControlAnchorPhi[ REFRESH_TOPOLOGY_SCRATCH ];

		const bool exact = m_PhaseScanStage2Pulses[ k ] == expectedPulses
		                && m_PhaseScanStage2Lines[ k ] == exposureLines;
		char classification = '?';
		if ( exact && baselineClean && unstable == 0 )
			classification = uniqueFailures <= 5 ? 'P'
			               : uniqueFailures >= 20 ? 'F' : '?';
		m_PhaseScanStage2Class[ k ] = (u8)classification;
		if ( classification == 'P' ) m_PhaseScanStage2PassCount++;
		else if ( classification == 'F' ) m_PhaseScanStage2FailureCount++;
		else m_PhaseScanStage2FlagCount++;
		m_PhaseScanStage2RungsCompleted = (u8)( executionIndex + 1 );

		// Bound a scheduler change to at most sixteen omission rungs. Every
		// checkpoint uses the same anchored K/W path as the map, not the
		// separately reported unanchored continuous loop.
		if ( ( executionIndex & 15u ) == 15u )
		{
			const u32 checkpoint = executionIndex >> 4;
			const u32 slot = REFRESH_INTERLEAVE_16 + checkpoint;
			if ( !runRefreshWindowControl( slot, 0,
			                              m_PhaseScanLineCycles,
			                              exposureLines ) ) return false;
			if ( !refreshExposureControlAcceptable(
			        slot, fullPulses, exposureLines ) ) controlsTrusted = false;
		}
	}

	// Bracket the complete map with the same representative-duration controls.
	if ( !runRefreshWindowControl( REFRESH_POST_FLOOR0, 0,
	                              m_PhaseScanLineCycles, 0 ) ) return false;
	if ( !runRefreshWindowControl( REFRESH_POST_FLOOR1, 0,
	                              m_PhaseScanLineCycles, 0 ) ) return false;
	if ( !runRefreshContinuousControl( REFRESH_POST_CONT2, 2, 0xE3 ) ) return false;
	if ( !runRefreshWindowControl( REFRESH_POST_KWFULL2, 0,
	                              m_PhaseScanLineCycles,
	                              exposureLines ) ) return false;
	controlsTrusted = controlsTrusted
		&& refreshControlStrict( REFRESH_POST_FLOOR0 )
		&& refreshControlStrict( REFRESH_POST_FLOOR1 )
		&& refreshWindowAccountingExact( REFRESH_POST_FLOOR0, 0, 0 )
		&& refreshWindowAccountingExact( REFRESH_POST_FLOOR1, 0, 0 )
		&& refreshExposureControlAcceptable(
		       REFRESH_POST_KWFULL2, fullPulses, exposureLines )
		&& m_Core0FrameAccountingErrors == 0;

	// A contiguous critical band is meaningful only when no rung is flagged.
	const u32 failedCount = m_PhaseScanStage2FailureCount;
	if ( m_PhaseScanStage2FlagCount == 0 && failedCount > 0
	     && failedCount < m_PhaseScanLineCycles )
	{
		for ( u32 start = 0; start < m_PhaseScanLineCycles; start++ )
		{
			const u32 before =
				( start + m_PhaseScanLineCycles - 1 ) % m_PhaseScanLineCycles;
			if ( m_PhaseScanStage2Class[ before ] != 'P' ) continue;
			bool oneRun = true;
			for ( u32 n = 0; n < failedCount; n++ )
			{
				const u32 pos = ( start + n ) % m_PhaseScanLineCycles;
				if ( m_PhaseScanStage2Class[ pos ] != 'F' ) oneRun = false;
			}
			const u32 after = ( start + failedCount ) % m_PhaseScanLineCycles;
			if ( oneRun && m_PhaseScanStage2Class[ after ] == 'P' )
			{
				m_PhaseScanStage2Contiguous = 1;
				m_PhaseScanStage2BandStart = (u8)start;
				break;
			}
		}
	}
	m_PhaseScanStage1Agree = controlsTrusted ? 1 : 0;
	m_PhaseScanStage2Ran = 1;
	return true;
#endif
}

static u8 refreshScreenCode( char c )
{
	if ( c >= 'A' && c <= 'Z' ) return (u8)( c - 'A' + 1 );
	return (u8)c;
}

void CRADBus::renderRefreshPhaseScanSummary()
{
	if ( m_PulseScanRan )
	{
		const u8 border = m_PulseScanCompleted ? 5 : 2;
		busDiagRawWrite( 0xD020, border );
		busDiagRawWrite( 0xD020, border );
		busDiagRawWrite( 0xD021, 0 );
		busDiagRawWrite( 0xD021, 0 );
		for ( u32 i = 0; i < 1000; i++ )
		{
			busDiagRawWrite( (u16)( 0x0400 + i ), 0x20 );
			busDiagRawWrite( (u16)( 0xD800 + i ), 0x01 );
		}
		static const char title[] = "C128 DMA PULSE WIDTH E18N";
		static const char heading[] = "WIDTH BASE0 READ0 READ1 READ2";
		static const char saved[] = "RESULT SAVED TO SD - RETURN CARD";
		static const char hex[] = "0123456789ABCDEF";
		for ( u32 i = 0; title[ i ]; i++ )
			busDiagRawWrite( (u16)( 0x0400 + 2 * 40 + 7 + i ),
			                 refreshScreenCode( title[ i ] ) );
		for ( u32 i = 0; heading[ i ]; i++ )
			busDiagRawWrite( (u16)( 0x0400 + 5 * 40 + 5 + i ),
			                 refreshScreenCode( heading[ i ] ) );
		for ( u32 rung = 0;
		      rung < m_PhaseScanStage2RungsCompleted
		      && rung < REFRESH_PULSE_SCAN_COUNT; rung++ )
		{
			const u16 base = (u16)( 0x0400 + ( 7 + rung * 2 ) * 40 + 5 );
			const u32 width = m_PulseScanWidth[ rung ];
			const u32 attempt = m_RefreshControlBaselineAttempts[ rung ]
			                  ? m_RefreshControlBaselineAttempts[ rung ] - 1u : 0u;
			const u16 values[ 4 ] =
			{
				m_RefreshControlBaselinePassFailures[ rung ][ attempt ][ 0 ],
				m_RefreshControlPassFailures[ rung ][ 0 ],
				m_RefreshControlPassFailures[ rung ][ 1 ],
				m_RefreshControlPassFailures[ rung ][ 2 ]
			};
			busDiagRawWrite( base + 0,
			                 refreshScreenCode( (char)( '0' + ( width / 100 ) % 10 ) ) );
			busDiagRawWrite( base + 1,
			                 refreshScreenCode( (char)( '0' + ( width / 10 ) % 10 ) ) );
			busDiagRawWrite( base + 2,
			                 refreshScreenCode( (char)( '0' + width % 10 ) ) );
			for ( u32 value = 0; value < 4; value++ )
			{
				const u16 pos = (u16)( base + 6 + value * 6 );
				busDiagRawWrite( pos + 0,
				                 refreshScreenCode( hex[ values[ value ] >> 12 ] ) );
				busDiagRawWrite( pos + 1,
				                 refreshScreenCode( hex[ ( values[ value ] >> 8 ) & 15 ] ) );
				busDiagRawWrite( pos + 2,
				                 refreshScreenCode( hex[ ( values[ value ] >> 4 ) & 15 ] ) );
				busDiagRawWrite( pos + 3,
				                 refreshScreenCode( hex[ values[ value ] & 15 ] ) );
			}
		}
		for ( u32 i = 0; saved[ i ]; i++ )
			busDiagRawWrite( (u16)( 0x0400 + 22 * 40 + 3 + i ),
			                 refreshScreenCode( saved[ i ] ) );
		return;
	}
	bool success = m_PhaseScanRan && m_PhaseScanStage1Agree
	            && m_PhaseScanStage2Ran;
	const u8 border = success ? 5 : 2;
	busDiagRawWrite( 0xD020, border );
	busDiagRawWrite( 0xD020, border );
	busDiagRawWrite( 0xD021, 0 );
	busDiagRawWrite( 0xD021, 0 );
	for ( u32 i = 0; i < 1000; i++ )
	{
		busDiagRawWrite( (u16)( 0x0400 + i ), 0x20 );
		busDiagRawWrite( (u16)( 0xD800 + i ), 0x01 );
	}

	static const char title[] = "C128 REFRESH TOPOLOGY E18M";
	static const char saved[] = "RESULT SAVED TO SD - RETURN CARD";
	for ( u32 i = 0; title[ i ]; i++ )
		busDiagRawWrite( (u16)( 0x0400 + 2 * 40 + 7 + i ),
		                 refreshScreenCode( title[ i ] ) );
	static const char complete[] = "COMPLETE";
	static const char abort[] = "ABORT";
	static const char untrusted[] = "UNTRUSTED";
	const char *state = success ? complete
	                  : m_PhaseScanStage2Ran ? untrusted : abort;
	for ( u32 i = 0; state[ i ]; i++ )
		busDiagRawWrite( (u16)( 0x0400 + 4 * 40 + 16 + i ),
		                 refreshScreenCode( state[ i ] ) );
	static const char heading[] = "RUNG             READ0 READ1 READ2";
	for ( u32 i = 0; heading[ i ]; i++ )
		busDiagRawWrite( (u16)( 0x0400 + 6 * 40 + 3 + i ),
		                 refreshScreenCode( heading[ i ] ) );
	static const char *names[ 5 ] =
		{ "SHORT FLOOR A", "SHORT FLOOR B", "BLACKOUT LONG",
		  "CORE3 CONT 2S", "CORE0 FRAME 2S" };
	static const char hex[] = "0123456789ABCDEF";
	for ( u32 rung = 0; rung < 5; rung++ )
	{
		const u16 base = (u16)( 0x0400 + ( 8 + rung * 2 ) * 40 + 3 );
		for ( u32 i = 0; names[ rung ][ i ]; i++ )
			busDiagRawWrite( (u16)( base + i ),
			                 refreshScreenCode( names[ rung ][ i ] ) );
		for ( u32 pass = 0; pass < 3; pass++ )
		{
			const u16 value = m_RefreshControlPassFailures[ rung ][ pass ];
			const u16 pos = (u16)( base + 17 + pass * 6 );
			busDiagRawWrite( pos + 0, refreshScreenCode( hex[ value >> 12 ] ) );
			busDiagRawWrite( pos + 1, refreshScreenCode( hex[ ( value >> 8 ) & 15 ] ) );
			busDiagRawWrite( pos + 2, refreshScreenCode( hex[ ( value >> 4 ) & 15 ] ) );
			busDiagRawWrite( pos + 3, refreshScreenCode( hex[ value & 15 ] ) );
		}
	}
	static const char topology[] = "MAP P=$00 F=$00 ?=$00 CONTIG=N";
	for ( u32 i = 0; topology[ i ]; i++ )
		busDiagRawWrite( (u16)( 0x0400 + 18 * 40 + 4 + i ),
		                 refreshScreenCode( topology[ i ] ) );
	busDiagRawWrite( 0x0400 + 18 * 40 + 11,
	                 refreshScreenCode( hex[ m_PhaseScanStage2PassCount >> 4 ] ) );
	busDiagRawWrite( 0x0400 + 18 * 40 + 12,
	                 refreshScreenCode( hex[ m_PhaseScanStage2PassCount & 15 ] ) );
	busDiagRawWrite( 0x0400 + 18 * 40 + 17,
	                 refreshScreenCode( hex[ m_PhaseScanStage2FailureCount >> 4 ] ) );
	busDiagRawWrite( 0x0400 + 18 * 40 + 18,
	                 refreshScreenCode( hex[ m_PhaseScanStage2FailureCount & 15 ] ) );
	busDiagRawWrite( 0x0400 + 18 * 40 + 23,
	                 refreshScreenCode( hex[ m_PhaseScanStage2FlagCount >> 4 ] ) );
	busDiagRawWrite( 0x0400 + 18 * 40 + 24,
	                 refreshScreenCode( hex[ m_PhaseScanStage2FlagCount & 15 ] ) );
	busDiagRawWrite( 0x0400 + 18 * 40 + 33,
	                 refreshScreenCode(
		                 m_PhaseScanStage2Contiguous ? 'Y' : 'N' ) );

	for ( u32 i = 0; saved[ i ]; i++ )
		busDiagRawWrite( (u16)( 0x0400 + 19 * 40 + 3 + i ),
		                 refreshScreenCode( saved[ i ] ) );
	static const char safe[] = "SAFE CONTINUOUS REFRESH ACTIVE";
	for ( u32 i = 0; safe[ i ]; i++ )
		busDiagRawWrite( (u16)( 0x0400 + 21 * 40 + 5 + i ),
		                 refreshScreenCode( safe[ i ] ) );
}

void CRADBus::renderRefreshMap()
{
	// Cyan means measurement is complete. Replace the oracle with a compact
	// 16x16 map: dot=stable, X=repeatable mismatch, ?=unstable capture.
	busDiagRawWrite( 0xD020, 3 );
	busDiagRawWrite( 0xD020, 3 );
	for ( u32 i = 0; i < 1000; i++ )
	{
		busDiagRawWrite( (u16)( 0x0400 + i ), 0x20 );
		busDiagRawWrite( (u16)( 0xD800 + i ), 0x01 );
	}

	static const char title[] = "PAGE 04 VIC REFRESH MAP";
	static const char legend[] = ".=OK X=FAIL ?=UNSTABLE";
	for ( u32 i = 0; title[ i ]; i++ )
		busDiagRawWrite( (u16)( 0x0400 + 1 * 40 + 5 + i ),
		                 refreshScreenCode( title[ i ] ) );
	for ( u32 i = 0; i < 16; i++ )
	{
		const char digit = (char)( i < 10 ? '0' + i : 'A' + i - 10 );
		busDiagRawWrite( (u16)( 0x0400 + 3 * 40 + 5 + i ),
		                 refreshScreenCode( digit ) );
		busDiagRawWrite( (u16)( 0x0400 + ( 4 + i ) * 40 + 3 ),
		                 refreshScreenCode( digit ) );
	}
	for ( u32 row = 0; row < 16; row++ )
		for ( u32 col = 0; col < 16; col++ )
		{
			const u32 cell = row * 16 + col;
			const u8 a = m_RefreshMapReadback[ 1 ][ cell ];
			const u8 b = m_RefreshMapReadback[ 2 ][ cell ];
			char mark = '.';
			u8 color = 5;
			if ( a != b ) { mark = '?'; color = 7; }
			else if ( a != m_RefreshMapExpected[ cell ] ) { mark = 'X'; color = 2; }
			const u16 offset = (u16)( ( 4 + row ) * 40 + 5 + col );
			busDiagRawWrite( (u16)( 0x0400 + offset ), refreshScreenCode( mark ) );
			busDiagRawWrite( (u16)( 0xD800 + offset ), color );
		}
	for ( u32 i = 0; legend[ i ]; i++ )
		busDiagRawWrite( (u16)( 0x0400 + 22 * 40 + 5 + i ),
		                 refreshScreenCode( legend[ i ] ) );
	m_Writes += 2593;
}

void CRADBus::runRefreshDiagnosticHold()
{
	renderRefreshMap();
	// Preserve the displayed result with the same VIC-half DMA release while
	// the user photographs it. The experiment itself ended before rendering.
	for ( ;; ) (void)runVICRefreshSlot();
}

void CRADBus::runFirstTransferDiscriminator()
{
	static const u16 targets[ 3 ] = { 0x02FF, 0x0334, 0x0400 };
	static const u32 trials = 24;

	for ( u32 p = 0; p < 3; p++ )
	{
		for ( u32 c = 0; c < 6; c++ ) m_FirstDiscCount[ p ][ c ] = 0;
		const u16 target = targets[ p ];
		const u16 control = (u16)( target ^ 1 );

		for ( u32 trial = 0; trial < trials; trial++ )
		{
			const u8 oldValue = (u8)( 0x31 + p * 0x23 + trial * 0x11 );
			const u8 newValue = (u8)( oldValue ^ 0xA5 );
			const u8 controlValue = (u8)( 0xC6 ^ trial );

			// Double-seed both cells so the classifier begins from known DRAM.
			// The operation under test below is deliberately exactly one raw write.
			busDiagRawWrite( control, controlValue );
			busDiagRawWrite( control, controlValue );
			busDiagRawWrite( target, oldValue );
			busDiagRawWrite( target, oldValue );

			(void)busDiagRawRead( control );
			const u8 preLag = busDiagRawRead( target );
			const u8 pre = busDiagRawRead( target );

			busDiagRawWrite( target, newValue );
			const u8 immediate = busDiagRawRead( target );
			const u8 controlSeen = busDiagRawRead( control );
			const u8 finalValue = busDiagRawRead( target );
			const u8 settledValue = busDiagRawRead( target );

			if ( trial == 0 )
			{
				m_FirstDiscSample[p][0] = oldValue;
				m_FirstDiscSample[p][1] = newValue;
				m_FirstDiscSample[p][2] = preLag;
				m_FirstDiscSample[p][3] = pre;
				m_FirstDiscSample[p][4] = immediate;
				m_FirstDiscSample[p][5] = controlSeen;
				m_FirstDiscSample[p][6] = finalValue;
				m_FirstDiscSample[p][7] = settledValue;
			}

			// The first target read after the control is retained as preLag, then a
			// second target read establishes the actual pre-content. Likewise,
			// finalValue exposes a one-read-behind control value while settledValue
			// decides whether the write itself landed.
			if ( pre != oldValue )
				m_FirstDiscCount[p][5]++;                 // other / bad pre-read
			else if ( settledValue == newValue )
			{
				if ( finalValue == controlValue && controlSeen == controlValue )
					m_FirstDiscCount[p][4]++;             // direct one-read lag
				else if ( immediate == newValue ) m_FirstDiscCount[p][0]++;
				else                              m_FirstDiscCount[p][2]++;
			}
			else if ( settledValue == oldValue )
			{
				if ( immediate == newValue ) m_FirstDiscCount[p][3]++;
				else                         m_FirstDiscCount[p][1]++;
			}
			else
				m_FirstDiscCount[p][5]++;
		}
	}
	// Four seed writes and seven reads per trial, for three targets.
	m_Writes += trials * 3 * 5;
	m_Reads  += trials * 3 * 7;
}

bool CRADBus::selfTest()
{
	register u32 g2;
	u8 v;
	u32 romErr = 0, addrErr = 0, wrErr = 0, burstErr = 0;

	// --- is the read path alive at all? ------------------------------------
	// $D012 is the VIC-II raster counter. On a running machine it MUST change
	// between samples taken a few hundred microseconds apart -- the beam does
	// not stop. If every sample is identical we are not reading the bus at all,
	// which is a completely different fault from reading the wrong address.
	{
		u8 r[ 6 ];
		for ( u32 i = 0; i < 6; i++ )
		{
			RAD_SPEEK( 0xD012, r[ i ] );
			DELAY( 1 << 14 );
		}
		m_Reads += 6;

		bool changed = false;
		for ( u32 i = 1; i < 6; i++ )
			if ( r[ i ] != r[ 0 ] ) changed = true;
		for ( u32 i = 0; i < 6; i++ ) m_TestRaster[ i ] = r[ i ];
		m_TestRasterChanged = changed ? 1 : 0;

		RADLOG( "  self-test: $D012 raster %02X %02X %02X %02X %02X %02X -- %s",
		        r[0], r[1], r[2], r[3], r[4], r[5],
		        changed ? "advancing, reads are live"
		                : "FROZEN: the read path is returning nothing" );
	}

	// --- raw read/write turnaround probe -----------------------------------
	// Do not use RAD_SPEEK here: it automatically discards two reads after a
	// write on a detected C128 and would hide the very first physical samples
	// this probe is meant to expose. All three targets and their A0-A9 XOR
	// neighbours remain in low RAM exposed during takeover. Sixteen target
	// reads expose any fixed-depth residue pipeline instead of assuming the
	// second or third read is trustworthy. The neighbour snapshots catch a
	// write displaced by any one of the ten low address lines.
	{
		static const u16 targets[ 3 ] = { 0x02FF, 0x0334, 0x0400 };
		const u8 savedWriteTurnaround = busWriteTurnaroundNeeded;
		m_TurnDetectedMachine = (u8)m_Signals.machine;
		// A distinct address/data marker for the post-neighbour flush. Two
		// consecutive writes avoid assuming the first write after the raster
		// reads landed. The marker is trusted only when a later read says C6.
		busDiagRawWrite( 0x02FC, 0xC6 );
		busDiagRawWrite( 0x02FC, 0xC6 );

		for ( u32 p = 0; p < 3; p++ )
		{
			const u16 target = targets[ p ];
			m_TurnFlags[ p ][ 0 ] = busDiagTurnFlags();
			for ( u32 i = 0; i < 16; i++ )
				m_TurnBaseTarget[ p ][ i ] = busDiagRawRead( target );
			m_TurnFlags[ p ][ 1 ] = busDiagTurnFlags();
			for ( u32 bit = 0; bit < 10; bit++ )
				m_TurnBaseNeighbor[ p ][ bit ] = busDiagRawRead( (u16)( target ^ ( 1u << bit ) ) );
			m_TurnFlags[ p ][ 2 ] = busDiagTurnFlags();

			busDiagRawWrite( target, 0x5A );
			m_TurnFlags[ p ][ 3 ] = busDiagTurnFlags();
			for ( u32 i = 0; i < 16; i++ )
				m_TurnSingleTarget[ p ][ i ] = busDiagRawRead( target );
			m_TurnFlags[ p ][ 4 ] = busDiagTurnFlags();
			for ( u32 bit = 0; bit < 10; bit++ )
				m_TurnSingleNeighbor[ p ][ bit ] = busDiagRawRead( (u16)( target ^ ( 1u << bit ) ) );
			m_TurnFlags[ p ][ 5 ] = busDiagTurnFlags();
			m_TurnSingleFlush[ p ] = busDiagRawRead( 0x02FC );
			m_TurnFlags[ p ][ 6 ] = busDiagTurnFlags();
			for ( u32 i = 0; i < 3; i++ )
				m_TurnSinglePostFlush[ p ][ i ] = busDiagRawRead( target );
			m_TurnFlags[ p ][ 7 ] = busDiagTurnFlags();

			busDiagRawWrite( target, 0x37 );
			m_TurnFlags[ p ][ 8 ] = busDiagTurnFlags();
			busDiagRawWrite( target, 0x37 );
			m_TurnFlags[ p ][ 9 ] = busDiagTurnFlags();
			for ( u32 i = 0; i < 16; i++ )
				m_TurnDoubleTarget[ p ][ i ] = busDiagRawRead( target );
			m_TurnFlags[ p ][ 10 ] = busDiagTurnFlags();
			for ( u32 bit = 0; bit < 10; bit++ )
				m_TurnDoubleNeighbor[ p ][ bit ] = busDiagRawRead( (u16)( target ^ ( 1u << bit ) ) );
			m_TurnFlags[ p ][ 11 ] = busDiagTurnFlags();
			m_TurnDoubleFlush[ p ] = busDiagRawRead( 0x02FC );
			m_TurnFlags[ p ][ 12 ] = busDiagTurnFlags();
			for ( u32 i = 0; i < 3; i++ )
				m_TurnDoublePostFlush[ p ][ i ] = busDiagRawRead( target );
			m_TurnFlags[ p ][ 13 ] = busDiagTurnFlags();
		}

		m_Reads += 258;
		m_Writes += 11;
		// Restore the entry write-turn flag exactly. The raw probe must not alter
		// the state inherited by the ordinary self-test.
		busWriteTurnaroundNeeded = savedWriteTurnaround;

		RADLOG( "  raw turn: machine=%s direct-read-policy",
		        m_TurnDetectedMachine == MACHINE_C128 ? "C128" : "C64" );
		for ( u32 p = 0; p < 3; p++ )
		{
			u16 singleChanged = 0, doubleChanged = 0;
			for ( u32 bit = 0; bit < 10; bit++ )
			{
				if ( m_TurnSingleNeighbor[p][bit] != m_TurnBaseNeighbor[p][bit] )
					singleChanged |= (u16)( 1u << bit );
				if ( m_TurnDoubleNeighbor[p][bit] != m_TurnSingleNeighbor[p][bit] )
					doubleChanged |= (u16)( 1u << bit );
			}
			RADLOG( "    %04X base=%02X/%02X/%02X single=%02X/%02X/%02X post=%02X/%02X/%02X f=%02X",
			        targets[p], m_TurnBaseTarget[p][0], m_TurnBaseTarget[p][1],
			        m_TurnBaseTarget[p][2], m_TurnSingleTarget[p][0],
			        m_TurnSingleTarget[p][1], m_TurnSingleTarget[p][2],
			        m_TurnSinglePostFlush[p][0], m_TurnSinglePostFlush[p][1],
			        m_TurnSinglePostFlush[p][2], m_TurnSingleFlush[p] );
			RADLOG( "         double=%02X/%02X/%02X post=%02X/%02X/%02X f=%02X neighchg=%03X/%03X",
			        m_TurnDoubleTarget[p][0], m_TurnDoubleTarget[p][1],
			        m_TurnDoubleTarget[p][2], m_TurnDoublePostFlush[p][0],
			        m_TurnDoublePostFlush[p][1], m_TurnDoublePostFlush[p][2],
			        m_TurnDoubleFlush[p], singleChanged, doubleChanged );
		}
	}

	// --- reads, against values we already know -----------------------------
	// The machine's own ROMs are banked in ($01 = $37 after a KERNAL start), so
	// these addresses have documented contents on every C64 ever made. If these
	// come back wrong, reads are broken and nothing else can be trusted.
	RAD_SPEEK( 0xFFFC, v ); if ( v != 0xE2 ) romErr++;	// reset vector lo
	RAD_SPEEK( 0xFFFD, v ); if ( v != 0xFC ) romErr++;	// reset vector hi -> $FCE2
	RAD_SPEEK( 0xA000, v ); if ( v != 0x94 ) romErr++;	// BASIC cold start lo
	RAD_SPEEK( 0xA001, v ); if ( v != 0xE3 ) romErr++;	// BASIC cold start hi -> $E394
	m_Reads += 4;

	// Informational only. The C64's PLA does not select BASIC/KERNAL while the
	// 6510 is off the bus, so these ranges return open bus to a DMA device --
	// which is exactly why an REU only ever transfers RAM. Nothing depends on
	// it: KERNAL and BASIC come from the SD card.
	RADLOG( "  self-test: ROM reads  %s (%u/4) -- expected to fail; the PLA"
	        " deselects ROM during DMA, so ROM is unreadable this way",
	        romErr ? "unreadable" : "readable!", romErr );

	if ( romErr )
	{
		u8 a, b, c, d;
		RAD_SPEEK( 0xFFFC, a ); RAD_SPEEK( 0xFFFD, b );
		RAD_SPEEK( 0xA000, c ); RAD_SPEEK( 0xA001, d );
		RADLOG( "    got $FFFC=%02X $FFFD=%02X (expect E2 FC), "
		        "$A000=%02X $A001=%02X (expect 94 E3)",
		        a, b, c, d );
	}

	// --- first-transfer series ----------------------------------------------
	// Earlier builds called this an address-line test. Cleaner results showed
	// only the first write/read pair failing, with no alias duplicate, so that
	// label overstated the evidence. Keep the multi-address series because it
	// remains a useful correctness gate, but report exactly what it measures.
	//
	// These six addresses are RAM under any non-Ultimax configuration, so
	// banking cannot influence the result. Each differs from the others in
	// exactly one address line (A10..A15). If a line is stuck, two of them
	// collide and the earlier value is overwritten by the later one.
	{
		static const u16 aline[ 6 ] = { 0x0400, 0x0800, 0x1000, 0x2000, 0x4000, 0x8000 };
		for ( u32 i = 0; i < 6; i++ )
			{ RAD_SPOKE( aline[ i ], (u8)( 0xE0 | i ) ); }

		u8 got[ 6 ];
		for ( u32 i = 0; i < 6; i++ )
		{
			RAD_SPEEK( aline[ i ], got[ i ] );
			if ( i == 0 ) RAD_SPEEK( aline[ i ], m_TestAddrFirstReread );
			if ( got[ i ] != (u8)( 0xE0 | i ) ) addrErr++;
		}
		for ( u32 i = 0; i < 6; i++ ) m_TestAddrLines[ i ] = got[ i ];
		m_TestAddrErrors = (u8)addrErr;

		m_Writes += 6; m_Reads += 7;

		RADLOG( "  self-test: first-transfer series %s -- 0400=%02X 0800=%02X 1000=%02X "
		        "2000=%02X 4000=%02X 8000=%02X (want E0 E1 E2 E3 E4 E5)",
		        addrErr ? "FAIL" : "ok",
		        got[0], got[1], got[2], got[3], got[4], got[5] );

		if ( addrErr )
			RADLOG( "    one or more transfers failed; duplicates would indicate"
			        " aliasing, while an isolated first byte indicates turnaround" );
	}

	// --- is anything actually banked in? -----------------------------------
	// Decisive test for the ROM failures. Writes always fall through to the RAM
	// underneath, whatever is banked in for reads. So:
	//   read back the ROM byte  -> ROM is banked in, reads are fine
	//   read back what we wrote -> ROM is NOT banked in; $01 is not $37 and we
	//                              have been reading blank RAM all along
	{
		u8 a, b;
		RAD_SPOKE( 0xA000, 0xA5 );
		RAD_SPEEK( 0xA000, a );
		RAD_SPOKE( 0xE000, 0x5A );
		RAD_SPEEK( 0xE000, b );
		m_TestBankA = a;
		m_TestBankE = b;
		m_Writes += 2; m_Reads += 2;

		RADLOG( "  self-test: banking   $A000 reads %02X after writing A5,"
		        " $E000 reads %02X after writing 5A", a, b );

		if ( a == 0xA5 || b == 0x5A )
			RADLOG( "    -> RAM is visible where ROM should be: the halted 6510"
			        " did not leave $01 at $37" );
		else if ( a == 0x00 && b == 0x00 )
			RADLOG( "    -> neither ROM nor the RAM we just wrote: this address"
			        " range is not responding at all" );
		else
			RADLOG( "    -> ROM is banked in and readable" );
	}

	// --- single writes, read back ------------------------------------------
	// $0334-$033F is the tail of the cassette buffer: RAM on every machine and
	// not used while we hold the bus.
	static const u8 patterns[] = { 0x3C, 0xC3, 0x5A, 0xA5, 0x69, 0x96 };

	// Seed the harmless read-prime address with a distinctive value. The first
	// write is deliberately sacrificial because that is the disputed C128
	// turnaround cycle; the following $02FF write is then not first in its run.
	// If a later target read returns $5A despite two discarded $02FF reads, the
	// fault is stale read residue. A different old target value instead points
	// to a lost write.
	RAD_SPOKE( 0x02FE, 0xA6 );
	RAD_SPOKE( 0x02FF, 0x5A );
	m_Writes += 2;

	// E8 positional discriminator: execute the known-good out-of-line raw
	// primitive at the exact point where the following inlined strict series
	// has been failing. The strict writes immediately replace this marker, so
	// it remains diagnostic and does not relax the boot gate.
	busDiagRawWrite( 0x0334, 0x6D );
	m_PosRawBeforeSingle[ 0 ] = busDiagRawRead( 0x0334 );
	m_PosRawBeforeSingle[ 1 ] = busDiagRawRead( 0x0334 );
	m_Writes++; m_Reads += 2;

	for ( u32 i = 0; i < 6; i++ )
	{
		RAD_SPOKE( (u16)( 0x0334 + i ), patterns[ i ] );
	}
	u8 rb[ 6 ];
	for ( u32 i = 0; i < 6; i++ )
	{
		RAD_SPEEK( (u16)( 0x0334 + i ), rb[ i ] );
		if ( i == 0 )
		{
			RAD_SPEEK( 0x0334, m_TestSingleFirstReread );
		}
		if ( rb[ i ] != patterns[ i ] ) wrErr++;
	}
	for ( u32 i = 0; i < 6; i++ ) m_TestSingle[ i ] = rb[ i ];
	m_TestSingleErrors = (u8)wrErr;
	m_Writes += 6; m_Reads += 7;

	// Report the bytes, not just a count. A bit that is always wrong points at
	// a data line; a byte that differs run to run points at timing margin.
	RADLOG( "  self-test: single r/w  %s (%u/6) got %02X %02X %02X %02X %02X %02X"
	        " want 3C C3 5A A5 69 96; reread0 %02X",
	        wrErr ? "FAIL" : "ok", wrErr,
	        rb[0], rb[1], rb[2], rb[3], rb[4], rb[5],
	        m_TestSingleFirstReread );

	// Repeat it a few times: an intermittent fault is a timing problem and a
	// consistent one is not, and that distinction decides what to change next.
	{
		u32 reruns = 0, runsFailed = 0;
		for ( reruns = 0; reruns < 20; reruns++ )
		{
			u32 e = 0;
			for ( u32 i = 0; i < 6; i++ ) { RAD_SPOKE( (u16)( 0x0334 + i ), patterns[ i ] ); }
			for ( u32 i = 0; i < 6; i++ )
			{
				RAD_SPEEK( (u16)( 0x0334 + i ), v );
				if ( v != patterns[ i ] ) e++;
			}
			if ( e ) runsFailed++;
		}
		m_Writes += 120; m_Reads += 120;
		m_TestRepeatedFailures = (u8)runsFailed;

		RADLOG( "  self-test: single r/w repeated 20x -- %u runs had errors%s",
		        runsFailed,
		        runsFailed == 0 ? " (stable)"
		                        : ( runsFailed >= 18 ? " (consistent: not timing)"
		                                             : " (INTERMITTENT: timing margin)" ) );
	}

	// --- burst writes, read back -------------------------------------------
	// This is the path the VIC mirroring uses, and it is entirely separate from
	// single writes: the address latch and transceiver stay configured across
	// the whole run, with a mid-burst resync when a badline starts.
	C64BusWrite burst[ 6 ];
	for ( u32 i = 0; i < 6; i++ )
	{
		burst[ i ].addr  = (u16)( 0x033A + i );
		burst[ i ].value = (u8)( patterns[ 5 - i ] );
	}
	writeBurst( burst, 6 );

	for ( u32 i = 0; i < 6; i++ )
	{
		RAD_SPEEK( burst[ i ].addr, v );
		if ( i == 0 )
		{
			RAD_SPEEK( burst[ i ].addr, m_TestBurstFirstReread );
		}
		m_TestBurst[ i ] = v;
		if ( v != burst[ i ].value ) burstErr++;
	}
	m_TestBurstErrors = (u8)burstErr;
	m_Reads += 7;

	RADLOG( "  self-test: burst write %s (%u/6 wrong)",
	        burstErr ? "FAIL" : "ok", burstErr );

	if ( burstErr && !wrErr )
		RADLOG( "    single writes work but bursts do not -- the mirroring path"
		        " is the problem, not the bus itself" );

	// Raw, policy-free classification of the first transfer. It is diagnostic,
	// not a tolerance: the ordinary checks above remain the boot gate.
	runFirstTransferDiscriminator();

	// Preserve the failure class for a monitor-free LED diagnostic. Native C128
	// reset overwrites the cassette buffer, so PEEK-after-reset cannot recover
	// these results on that machine.
	m_SelfTestFailure = ( wrErr ? 1 : 0 ) | ( burstErr ? 2 : 0 )
	                  | ( addrErr ? 4 : 0 );

	// romErr deliberately excluded: see the note above.
	return ( addrErr + wrErr + burstErr ) == 0;
}

bool CRADBus::c128KnownFirstTransferOnly() const
{
	return m_Signals.machine == MACHINE_C128
	    && m_TestAddrErrors == 1
	    && m_TestSingleErrors == 1
	    && m_TestRepeatedFailures == 0
	    && m_TestBurstErrors == 0
	    && m_TestAddrLines[ 1 ] == 0xE1
	    && m_TestAddrLines[ 2 ] == 0xE2
	    && m_TestAddrLines[ 3 ] == 0xE3
	    && m_TestAddrLines[ 4 ] == 0xE4
	    && m_TestAddrLines[ 5 ] == 0xE5
	    && m_TestSingle[ 1 ] == 0xC3
	    && m_TestSingle[ 2 ] == 0x5A
	    && m_TestSingle[ 3 ] == 0xA5
	    && m_TestSingle[ 4 ] == 0x69
	    && m_TestSingle[ 5 ] == 0x96;
}

void CRADBus::logSelfTestResults( CLogger *logger ) const
{
	if ( !logger ) return;
	if ( m_Signals.machine == MACHINE_C128 )
	{
		logger->Write( "RADbus", c128RefreshActive() ? LogNotice : LogError,
			               "C128 refresh E18: core3=%s line=%s fallback=%u reason=%u slots=%u traffic/refresh epochs=%u/%u",
		               c128RefreshActive() ? "ACTIVE" : "NOT ACTIVE",
		               c128RefreshLineActive() ? "ACTIVE" : "NOT ACTIVE",
		               c128RefreshLineFallback() ? 1u : 0u,
		               (unsigned)c128RefreshLineFallbackReason(),
		               (unsigned)c128RefreshSlots(),
		               (unsigned)c128TrafficEpochs(),
		               (unsigned)c128RefreshEpochs() );
		logger->Write( "RADbus", LogNotice,
			               "C128 line K204: line-cycles=%u traffic-cycles=8 perm-step=%u phases=%u min/max=%u/%u runtime-guarded-ba=1 guard-phi=3 startup-legacy=1 single-writes=1 background-resync=0 phi=%u resync=%u phase-errors=%u deadline/recovery/lines=%u/%u/%u overrun-count/total/max=%u/%u/%u worst-phase/max=%u/%u max-wait=%u",
		               (unsigned)c128RefreshLineCyclesUsed(),
		               (unsigned)c128RefreshPermutationStep(),
		               (unsigned)c128RefreshPhasesVisited(),
		               (unsigned)c128RefreshPhaseMinOpens(),
		               (unsigned)c128RefreshPhaseMaxOpens(),
		               (unsigned)c128RefreshMeasuredPhiCycles(),
		               (unsigned)c128RefreshLineResyncs(),
		               (unsigned)c128RefreshLinePhaseErrors(),
		               (unsigned)c128RefreshLineDeadlineMisses(),
		               (unsigned)c128RefreshLineDeadlineRecoveries(),
		               (unsigned)c128RefreshLineRecoveryLines(),
		               (unsigned)c128RefreshLineOverrunCount(),
		               (unsigned)c128RefreshLineTotalOverrunCycles(),
		               (unsigned)c128RefreshLineMaxLateCycles(),
		               (unsigned)c128RefreshWorstOverrunPhase(),
		               (unsigned)c128RefreshWorstPhaseOverrunCycles(),
		               (unsigned)c128MaxTrafficBusyCycles() );
		if ( m_PhaseScanRan )
		{
			if ( m_PulseScanRan )
				logger->Write( "RADbus", LogNotice,
				               "C128 pulse E18N: complete=%u rungs=%u final-width=%u",
				               (unsigned)m_PulseScanCompleted,
				               (unsigned)m_PhaseScanStage2RungsCompleted,
				               (unsigned)c128RefreshDMAReleaseCycles() );
			else logger->Write( "RADbus", LogNotice,
			               "C128 phase E18M: trusted=%u scan-complete=%u rungs=%u P/F/?=%u/%u/%u contiguous=%u start=%u",
			               (unsigned)m_PhaseScanStage1Agree,
			               (unsigned)m_PhaseScanStage2Ran,
			               (unsigned)m_PhaseScanStage2RungsCompleted,
			               (unsigned)m_PhaseScanStage2PassCount,
			               (unsigned)m_PhaseScanStage2FailureCount,
			               (unsigned)m_PhaseScanStage2FlagCount,
			               (unsigned)m_PhaseScanStage2Contiguous,
			               (unsigned)m_PhaseScanStage2BandStart );
		}
	}

	if ( m_ReadTimingStart )
		logger->Write( "RADbus", LogNotice,
		               "read timing: configured %u stable %u..%u selected %u",
		               (unsigned)m_ReadTimingConfigured, (unsigned)m_ReadTimingStart,
		               (unsigned)m_ReadTimingEnd, (unsigned)m_ReadTimingSelected );
	else
		logger->Write( "RADbus", LogNotice,
		               "read timing: NO stable window; retained %u",
		               (unsigned)m_ReadTimingConfigured );

	logger->Write( "RADbus", LogNotice,
	               "write timing: configured %u/%u pass=%u selected %u/%u",
	               (unsigned)m_WriteTimingConfiguredAddr,
	               (unsigned)m_WriteTimingConfiguredData,
	               (unsigned)m_WriteTimingPassingPoints,
	               (unsigned)m_WriteTimingSelectedAddr,
	               (unsigned)m_WriteTimingSelectedData );

	logger->Write( "RADbus", LogNotice,
	               "raster: %02X %02X %02X %02X %02X %02X %s",
	               m_TestRaster[0], m_TestRaster[1], m_TestRaster[2],
	               m_TestRaster[3], m_TestRaster[4], m_TestRaster[5],
	               m_TestRasterChanged ? "advancing" : "FROZEN" );
	logger->Write( "RADbus", LogNotice,
	               "raw turn: machine=%s direct-read-policy",
	               m_TurnDetectedMachine == MACHINE_C128 ? "C128" : "C64" );
	static const u16 turnTargets[ 3 ] = { 0x02FF, 0x0334, 0x0400 };
	for ( u32 p = 0; p < 3; p++ )
		logger->Write( "RADbus", LogNotice,
		               "raw %04X base=%02X/%02X/%02X single=%02X/%02X/%02X double=%02X/%02X/%02X",
		               turnTargets[p], m_TurnBaseTarget[p][0], m_TurnBaseTarget[p][1],
		               m_TurnBaseTarget[p][2], m_TurnSingleTarget[p][0],
		               m_TurnSingleTarget[p][1], m_TurnSingleTarget[p][2],
		               m_TurnDoubleTarget[p][0], m_TurnDoubleTarget[p][1],
		               m_TurnDoubleTarget[p][2] );
	logger->Write( "RADbus", LogNotice,
	               "first-transfer: %02X %02X %02X %02X %02X %02X errors=%u reread0=%02X",
	               m_TestAddrLines[0], m_TestAddrLines[1], m_TestAddrLines[2],
	               m_TestAddrLines[3], m_TestAddrLines[4], m_TestAddrLines[5],
	               (unsigned)m_TestAddrErrors, m_TestAddrFirstReread );
	logger->Write( "RADbus", LogNotice,
	               "banking after A5/5A: A000=%02X E000=%02X",
	               m_TestBankA, m_TestBankE );
	logger->Write( "RADbus", LogNotice,
	               "positional raw before single: %02X %02X expected=6D 6D",
	               m_PosRawBeforeSingle[0], m_PosRawBeforeSingle[1] );
	logger->Write( "RADbus", LogNotice,
	               "single: %02X %02X %02X %02X %02X %02X err=%u repeats=%u/20 reread0=%02X",
	               m_TestSingle[0], m_TestSingle[1], m_TestSingle[2],
	               m_TestSingle[3], m_TestSingle[4], m_TestSingle[5],
	               (unsigned)m_TestSingleErrors,
	               (unsigned)m_TestRepeatedFailures,
	               m_TestSingleFirstReread );
	logger->Write( "RADbus", LogNotice,
	               "burst: %02X %02X %02X %02X %02X %02X err=%u reread0=%02X",
	               m_TestBurst[0], m_TestBurst[1], m_TestBurst[2],
	               m_TestBurst[3], m_TestBurst[4], m_TestBurst[5],
	               (unsigned)m_TestBurstErrors,
	               m_TestBurstFirstReread );
	static const u16 discTargets[ 3 ] = { 0x02FF, 0x0334, 0x0400 };
	for ( u32 p = 0; p < 3; p++ )
		logger->Write( "RADbus", LogNotice,
		               "disc %04X s=%02X/%02X prelag/pre=%02X/%02X imm=%02X ctl=%02X fin/set=%02X/%02X counts ok/lost/first/false/lagctl/other=%u/%u/%u/%u/%u/%u",
		               discTargets[p], m_FirstDiscSample[p][0], m_FirstDiscSample[p][1],
		               m_FirstDiscSample[p][2], m_FirstDiscSample[p][3],
		               m_FirstDiscSample[p][4], m_FirstDiscSample[p][5],
		               m_FirstDiscSample[p][6], m_FirstDiscSample[p][7],
		               (unsigned)m_FirstDiscCount[p][0],
		               (unsigned)m_FirstDiscCount[p][1],
		               (unsigned)m_FirstDiscCount[p][2],
		               (unsigned)m_FirstDiscCount[p][3],
		               (unsigned)m_FirstDiscCount[p][4],
		               (unsigned)m_FirstDiscCount[p][5] );
}

u32 CRADBus::formatSelfTestResults( char *dst, u32 capacity ) const
{
	u32 n = 0;
	if ( !dst || capacity == 0 ) return 0;
	dst[ 0 ] = 0;

	busDiagText( dst, capacity, n, "SCPU-EMU RAD bus diagnostic\r\nverdict: " );
	busDiagText( dst, capacity, n, m_SelfTestFailure ? "FAIL mask=" : "PASS mask=" );
	busDiagDecimal( dst, capacity, n, m_SelfTestFailure );
	busDiagEndLine( dst, capacity, n );
	if ( m_Signals.machine == MACHINE_C128 )
	{
		// Always place the live scheduler record before the optional historical
		// scan sections. The pulse-width formatter returns after its retained
		// vectors, which previously made these runtime counters disappear from
		// the SD-card file even though they were printed on HDMI.
		busDiagText( dst, capacity, n,
		             "K204 runtime line/phi/step/visited/open-min/open-max/overrun-count/total/max/worst-phase/worst-max/deadline/recovery/lines/max-busy: " );
		busDiagDecimal( dst, capacity, n, c128RefreshLineCyclesUsed() );
		busDiagChar( dst, capacity, n, '/' );
		busDiagDecimal( dst, capacity, n, c128RefreshMeasuredPhiCycles() );
		busDiagChar( dst, capacity, n, '/' );
		busDiagDecimal( dst, capacity, n, c128RefreshPermutationStep() );
		busDiagChar( dst, capacity, n, '/' );
		busDiagDecimal( dst, capacity, n, c128RefreshPhasesVisited() );
		busDiagChar( dst, capacity, n, '/' );
		busDiagDecimal( dst, capacity, n, c128RefreshPhaseMinOpens() );
		busDiagChar( dst, capacity, n, '/' );
		busDiagDecimal( dst, capacity, n, c128RefreshPhaseMaxOpens() );
		busDiagChar( dst, capacity, n, '/' );
		busDiagDecimal( dst, capacity, n, (u32)c128RefreshLineOverrunCount() );
		busDiagChar( dst, capacity, n, '/' );
		busDiagDecimal( dst, capacity, n,
		                (u32)c128RefreshLineTotalOverrunCycles() );
		busDiagChar( dst, capacity, n, '/' );
		busDiagDecimal( dst, capacity, n,
		                (u32)c128RefreshLineMaxLateCycles() );
		busDiagChar( dst, capacity, n, '/' );
		busDiagDecimal( dst, capacity, n, c128RefreshWorstOverrunPhase() );
		busDiagChar( dst, capacity, n, '/' );
		busDiagDecimal( dst, capacity, n,
		                c128RefreshWorstPhaseOverrunCycles() );
		busDiagChar( dst, capacity, n, '/' );
		busDiagDecimal( dst, capacity, n,
		                (u32)c128RefreshLineDeadlineMisses() );
		busDiagChar( dst, capacity, n, '/' );
		busDiagDecimal( dst, capacity, n,
		                (u32)c128RefreshLineDeadlineRecoveries() );
		busDiagChar( dst, capacity, n, '/' );
		busDiagDecimal( dst, capacity, n,
		                (u32)c128RefreshLineRecoveryLines() );
		busDiagChar( dst, capacity, n, '/' );
		busDiagDecimal( dst, capacity, n, (u32)c128MaxTrafficBusyCycles() );
		busDiagEndLine( dst, capacity, n );
		if ( m_PulseScanRan )
		{
			busDiagText( dst, capacity, n,
			             "protocol: C128-PULSE-WIDTH-E18N core3-literal-continuous=1 exposure_s=2 same-salt=D6 widths-arm-cycles=280,340,400,440,480,520,560 decision=read0 later-passes=observation-decay-audit baseline-retained=1 masks-retained=3x33 production-width-restored=480 diagnostic_only=1\r\n" );
			busDiagText( dst, capacity, n,
			             "pulse-scan complete/rungs/final-width/phi-cycles: " );
			busDiagDecimal( dst, capacity, n, m_PulseScanCompleted );
			busDiagChar( dst, capacity, n, '/' );
			busDiagDecimal( dst, capacity, n,
			                m_PhaseScanStage2RungsCompleted );
			busDiagChar( dst, capacity, n, '/' );
			busDiagDecimal( dst, capacity, n,
			                c128RefreshDMAReleaseCycles() );
			busDiagChar( dst, capacity, n, '/' );
			busDiagDecimal( dst, capacity, n,
			                c128RefreshMeasuredPhiCycles() );
			busDiagEndLine( dst, capacity, n );
			for ( u32 rung = 0;
			      rung < m_PhaseScanStage2RungsCompleted
			      && rung < REFRESH_PULSE_SCAN_COUNT; rung++ )
			{
				const u32 attempt = m_RefreshControlBaselineAttempts[ rung ]
				                  ? m_RefreshControlBaselineAttempts[ rung ] - 1u
				                  : 0u;
				busDiagText( dst, capacity, n, "width " );
				busDiagDecimal( dst, capacity, n, m_PulseScanWidth[ rung ] );
				busDiagText( dst, capacity, n,
				             " base-attempt/pass0/pass1/pass2 exposure-pass0/pass1/pass2 delta0 pulses exposure-us: " );
				busDiagDecimal( dst, capacity, n,
				                m_RefreshControlBaselineAttempts[ rung ] );
				for ( u32 pass = 0; pass < 3; pass++ )
				{
					busDiagChar( dst, capacity, n, '/' );
					busDiagDecimal( dst, capacity, n,
					                m_RefreshControlBaselinePassFailures
					                  [ rung ][ attempt ][ pass ] );
				}
				for ( u32 pass = 0; pass < 3; pass++ )
				{
					busDiagChar( dst, capacity, n, '/' );
					busDiagDecimal( dst, capacity, n,
					                m_RefreshControlPassFailures[ rung ][ pass ] );
				}
				busDiagChar( dst, capacity, n, '/' );
				const u32 baseline0 =
					m_RefreshControlBaselinePassFailures[ rung ][ attempt ][ 0 ];
				const u32 exposure0 = m_RefreshControlPassFailures[ rung ][ 0 ];
				if ( exposure0 >= baseline0 )
					busDiagDecimal( dst, capacity, n, exposure0 - baseline0 );
				else
				{
					busDiagChar( dst, capacity, n, '-' );
					busDiagDecimal( dst, capacity, n, baseline0 - exposure0 );
				}
				busDiagChar( dst, capacity, n, '/' );
				busDiagDecimal( dst, capacity, n,
				                m_RefreshControlPulses[ rung ] );
				busDiagChar( dst, capacity, n, '/' );
				busDiagDecimal( dst, capacity, n,
				                m_RefreshControlExposureUsec[ rung ] );
				busDiagEndLine( dst, capacity, n );
				busDiagText( dst, capacity, n,
				             "  timing-us baseline-seed/capture exposure-capture/pass0/pass1/pass2: " );
				busDiagDecimal( dst, capacity, n,
				                m_RefreshControlBaselineSeedUsec[ rung ][ attempt ] );
				busDiagChar( dst, capacity, n, '/' );
				busDiagDecimal( dst, capacity, n,
				                m_RefreshControlBaselineCaptureUsec[ rung ][ attempt ] );
				busDiagChar( dst, capacity, n, '/' );
				busDiagDecimal( dst, capacity, n,
				                m_RefreshControlCaptureUsec[ rung ] );
				for ( u32 pass = 0; pass < 3; pass++ )
				{
					busDiagChar( dst, capacity, n, '/' );
					busDiagDecimal( dst, capacity, n,
					                m_RefreshControlCapturePassUsec[ rung ][ pass ] );
				}
				busDiagEndLine( dst, capacity, n );
				for ( u32 pass = 0; pass < 3; pass++ )
				{
					busDiagText( dst, capacity, n, "  mismatch-mask-read" );
					busDiagChar( dst, capacity, n, (char)( '0' + pass ) );
					busDiagText( dst, capacity, n, ": " );
					busDiagBytes( dst, capacity, n,
					              m_RefreshControlMismatchMask[ rung ][ pass ], 33 );
					busDiagEndLine( dst, capacity, n );
				}
			}
			return n;
		}
		busDiagText( dst, capacity, n,
		             "protocol: C128-TOPOLOGY-E18M controls-label-trust-never-suppress-map=1 precondition=literal-continuous-2s-before-seed observation=no-refresh-seed-and-capture measured-seed-pass-total-usec=1 blackout-short=immediate blackout-long=delay-equal-short-seed-plus-capture emitter-ab=same-salt-core3-cont2-vs-core0-fullframes2 pre-controls=floor0,floor1,blackoutlong,core3cont2,core0cont2,continuous8a,kwfull2,continuous8b zero-exposure-floor-gate=all-zero-stable-exact exposure-kw-gate=unique-union-le5-stable-exact topology-order=K-i-times-31-mod-line execution-index-retained=1 interleave-kwfull2-after-execution-count=16,32,48,64 post-controls=floor0,floor1,continuous2-warning,kwfull2-gate topology=Wline-1-all-K exposure_s=2 K=first-protected-edge omitted-edge=K-1-mod-line classification=unique-union-le5-P-ge20-F-middle-or-unstable-or-accounting-error-FLAG masks-retained=65x3x33 kw-pulses=counted-inside-exposure-only oracle=page04-plus-page07 read_passes=3 diagnostic_only=1 dmb=1 single_rw=out_of-line\r\n" );
		busDiagText( dst, capacity, n, "C128 refresh core3: " );
		busDiagText( dst, capacity, n,
		             c128RefreshActive() ? "ACTIVE slots=" : "NOT-ACTIVE slots=" );
		busDiagDecimal( dst, capacity, n, (u32)c128RefreshSlots() );
		busDiagText( dst, capacity, n, " skipped=" );
		busDiagDecimal( dst, capacity, n, (u32)c128RefreshSkippedSlots() );
		busDiagText( dst, capacity, n, " core0-slots=" );
		busDiagDecimal( dst, capacity, n, (u32)c128RefreshCore0Slots() );
		busDiagEndLine( dst, capacity, n );
		busDiagText( dst, capacity, n, "epochs traffic/refresh/max_handoff_wait_cycles: " );
		busDiagDecimal( dst, capacity, n, (u32)c128TrafficEpochs() );
		busDiagChar( dst, capacity, n, '/' );
		busDiagDecimal( dst, capacity, n, (u32)c128RefreshEpochs() );
		busDiagChar( dst, capacity, n, '/' );
		busDiagDecimal( dst, capacity, n, (u32)c128MaxTrafficBusyCycles() );
		busDiagEndLine( dst, capacity, n );
		busDiagText( dst, capacity, n,
		             "line active/fallback/reason/linecycles/phi/resync/phaseerr/deadline/recovery/lines/maxlate/predicted: " );
		busDiagDecimal( dst, capacity, n, c128RefreshLineActive() ? 1u : 0u );
		busDiagChar( dst, capacity, n, '/' );
		busDiagDecimal( dst, capacity, n, c128RefreshLineFallback() ? 1u : 0u );
		busDiagChar( dst, capacity, n, '/' );
		busDiagDecimal( dst, capacity, n, c128RefreshLineFallbackReason() );
		busDiagChar( dst, capacity, n, '/' );
		busDiagDecimal( dst, capacity, n, c128RefreshLineCyclesUsed() );
		busDiagChar( dst, capacity, n, '/' );
		busDiagDecimal( dst, capacity, n, c128RefreshMeasuredPhiCycles() );
		busDiagChar( dst, capacity, n, '/' );
		busDiagDecimal( dst, capacity, n, (u32)c128RefreshLineResyncs() );
		busDiagChar( dst, capacity, n, '/' );
		busDiagDecimal( dst, capacity, n, (u32)c128RefreshLinePhaseErrors() );
		busDiagChar( dst, capacity, n, '/' );
		busDiagDecimal( dst, capacity, n, (u32)c128RefreshLineDeadlineMisses() );
		busDiagChar( dst, capacity, n, '/' );
		busDiagDecimal( dst, capacity, n, (u32)c128RefreshLineDeadlineRecoveries() );
		busDiagChar( dst, capacity, n, '/' );
		busDiagDecimal( dst, capacity, n, (u32)c128RefreshLineRecoveryLines() );
		busDiagChar( dst, capacity, n, '/' );
		busDiagDecimal( dst, capacity, n, (u32)c128RefreshLineMaxLateCycles() );
		busDiagChar( dst, capacity, n, '/' );
		busDiagDecimal( dst, capacity, n, c128RefreshPredictedRaster() );
		busDiagEndLine( dst, capacity, n );
		busDiagText( dst, capacity, n,
		             "K198 permutation step/visited/open-min/open-max/overrun-count/total/worst-phase/worst-max: " );
		busDiagDecimal( dst, capacity, n, c128RefreshPermutationStep() );
		busDiagChar( dst, capacity, n, '/' );
		busDiagDecimal( dst, capacity, n, c128RefreshPhasesVisited() );
		busDiagChar( dst, capacity, n, '/' );
		busDiagDecimal( dst, capacity, n, c128RefreshPhaseMinOpens() );
		busDiagChar( dst, capacity, n, '/' );
		busDiagDecimal( dst, capacity, n, c128RefreshPhaseMaxOpens() );
		busDiagChar( dst, capacity, n, '/' );
		busDiagDecimal( dst, capacity, n, (u32)c128RefreshLineOverrunCount() );
		busDiagChar( dst, capacity, n, '/' );
		busDiagDecimal( dst, capacity, n,
		                (u32)c128RefreshLineTotalOverrunCycles() );
		busDiagChar( dst, capacity, n, '/' );
		busDiagDecimal( dst, capacity, n, c128RefreshWorstOverrunPhase() );
		busDiagChar( dst, capacity, n, '/' );
		busDiagDecimal( dst, capacity, n,
		                c128RefreshWorstPhaseOverrunCycles() );
		busDiagEndLine( dst, capacity, n );
		busDiagText( dst, capacity, n, "K198 phase open-count: " );
		for ( u32 phase = 0; phase < c128RefreshLineCyclesUsed(); phase++ )
		{
			if ( phase ) busDiagChar( dst, capacity, n, ' ' );
			busDiagDecimal( dst, capacity, n,
			                c128RefreshPhaseOpenCount( phase ) );
		}
		busDiagEndLine( dst, capacity, n );
		busDiagText( dst, capacity, n, "K198 phase overrun-count/max: " );
		for ( u32 phase = 0; phase < c128RefreshLineCyclesUsed(); phase++ )
		{
			if ( phase ) busDiagChar( dst, capacity, n, ' ' );
			busDiagDecimal( dst, capacity, n,
			                c128RefreshPhaseOverrunCount( phase ) );
			busDiagChar( dst, capacity, n, '/' );
			busDiagDecimal( dst, capacity, n,
			                c128RefreshPhaseMaxOverrunCycles( phase ) );
		}
		busDiagEndLine( dst, capacity, n );
		const bool controlsComplete = m_PhaseScanStage2Ran != 0;
		busDiagText( dst, capacity, n,
		             "control E18M complete/trusted/linecycles/warmup-pulses: " );
		busDiagDecimal( dst, capacity, n, controlsComplete ? 1u : 0u );
		busDiagChar( dst, capacity, n, '/' );
		busDiagDecimal( dst, capacity, n, m_PhaseScanStage1Agree );
		busDiagChar( dst, capacity, n, '/' );
		busDiagDecimal( dst, capacity, n, m_PhaseScanLineCycles );
		busDiagChar( dst, capacity, n, '/' );
		busDiagDecimal( dst, capacity, n, m_RefreshWarmupPulses );
		busDiagEndLine( dst, capacity, n );
		busDiagText( dst, capacity, n,
		             "core0 full-frame sweeps/pulses/expected/errors/min-us/max-us: " );
		busDiagDecimal( dst, capacity, n, m_Core0FrameSweeps );
		busDiagChar( dst, capacity, n, '/' );
		busDiagDecimal( dst, capacity, n, m_Core0FramePulses );
		busDiagChar( dst, capacity, n, '/' );
		busDiagDecimal( dst, capacity, n, m_Core0FrameExpectedPulses );
		busDiagChar( dst, capacity, n, '/' );
		busDiagDecimal( dst, capacity, n, m_Core0FrameAccountingErrors );
		busDiagChar( dst, capacity, n, '/' );
		busDiagDecimal( dst, capacity, n,
		                m_Core0FrameSweeps ? m_Core0FrameMinUsec : 0 );
		busDiagChar( dst, capacity, n, '/' );
		busDiagDecimal( dst, capacity, n, m_Core0FrameMaxUsec );
		busDiagEndLine( dst, capacity, n );
		busDiagText( dst, capacity, n,
		             "blackout-long intentional-delay-us: " );
		busDiagDecimal( dst, capacity, n, m_BlackoutLongDelayUsec );
		busDiagEndLine( dst, capacity, n );
		static const char *controlNames[ REFRESH_CONTROL_COUNT ] =
		{
			"pre-floor0", "pre-floor1", "pre-blackout-long",
			"pre-core3-continuous2", "pre-core0-fullframes2",
			"pre-continuous8-a", "pre-kwfull2", "pre-continuous8-b",
			"topology-scratch", "interleave-kwfull2-after-exec15",
			"interleave-kwfull2-after-exec31", "interleave-kwfull2-after-exec47",
			"interleave-kwfull2-after-exec63", "post-floor0", "post-floor1",
			"post-continuous2", "post-kwfull2"
		};
		for ( u32 rung = 0; rung < REFRESH_CONTROL_COUNT; rung++ )
		{
			busDiagText( dst, capacity, n, "control " );
			busDiagText( dst, capacity, n, controlNames[ rung ] );
			busDiagText( dst, capacity, n,
			             " valid/base-attempt/base-fail/base-unstable/read0/read1/read2/unstable01/unstable12: " );
			busDiagDecimal( dst, capacity, n, m_RefreshControlValid[ rung ] );
			busDiagChar( dst, capacity, n, '/' );
			busDiagDecimal( dst, capacity, n,
			                m_RefreshControlBaselineAttempts[ rung ] );
			busDiagChar( dst, capacity, n, '/' );
			busDiagDecimal( dst, capacity, n,
			                m_RefreshControlBaselineFailures[ rung ] );
			busDiagChar( dst, capacity, n, '/' );
			busDiagDecimal( dst, capacity, n,
			                m_RefreshControlBaselineUnstable[ rung ] );
			for ( u32 pass = 0; pass < 3; pass++ )
			{
				busDiagChar( dst, capacity, n, '/' );
				busDiagDecimal( dst, capacity, n,
				                m_RefreshControlPassFailures[ rung ][ pass ] );
			}
			busDiagChar( dst, capacity, n, '/' );
			busDiagDecimal( dst, capacity, n,
			                m_RefreshControlUnstable01[ rung ] );
			busDiagChar( dst, capacity, n, '/' );
			busDiagDecimal( dst, capacity, n,
			                m_RefreshControlUnstable12[ rung ] );
			busDiagEndLine( dst, capacity, n );
			for ( u32 attempt = 0; attempt < 2; attempt++ )
			{
				busDiagText( dst, capacity, n, "  baseline-attempt" );
				busDiagChar( dst, capacity, n, (char)( '1' + attempt ) );
				busDiagText( dst, capacity, n,
				             " pass0/pass1/pass2/unstable01/unstable12/frame-sweeps/pulses/expected: " );
				for ( u32 pass = 0; pass < 3; pass++ )
				{
					if ( pass ) busDiagChar( dst, capacity, n, '/' );
					busDiagDecimal( dst, capacity, n,
					                m_RefreshControlBaselinePassFailures
					                  [ rung ][ attempt ][ pass ] );
				}
				for ( u32 pair = 0; pair < 2; pair++ )
				{
					busDiagChar( dst, capacity, n, '/' );
					busDiagDecimal( dst, capacity, n,
					                m_RefreshControlBaselinePassUnstable
					                  [ rung ][ attempt ][ pair ] );
				}
				busDiagChar( dst, capacity, n, '/' );
				busDiagDecimal( dst, capacity, n,
				                m_RefreshControlBaselineFrameSweeps
				                  [ rung ][ attempt ] );
				busDiagChar( dst, capacity, n, '/' );
				busDiagDecimal( dst, capacity, n,
				                m_RefreshControlBaselineFramePulses
				                  [ rung ][ attempt ] );
				busDiagChar( dst, capacity, n, '/' );
				busDiagDecimal( dst, capacity, n,
				                m_RefreshControlBaselineFrameExpected
				                  [ rung ][ attempt ] );
				busDiagEndLine( dst, capacity, n );
				busDiagText( dst, capacity, n,
				             "    timing-us seed/capture/pass0/pass1/pass2: " );
				busDiagDecimal( dst, capacity, n,
				                m_RefreshControlBaselineSeedUsec
				                  [ rung ][ attempt ] );
				busDiagChar( dst, capacity, n, '/' );
				busDiagDecimal( dst, capacity, n,
				                m_RefreshControlBaselineCaptureUsec
				                  [ rung ][ attempt ] );
				for ( u32 pass = 0; pass < 3; pass++ )
				{
					busDiagChar( dst, capacity, n, '/' );
					busDiagDecimal( dst, capacity, n,
					                m_RefreshControlBaselinePassUsec
					                  [ rung ][ attempt ][ pass ] );
				}
				busDiagEndLine( dst, capacity, n );
			}
			busDiagText( dst, capacity, n,
			             "  pulses/expected/exposure-lines/anchor-phi/anchor-arm: " );
			busDiagDecimal( dst, capacity, n, m_RefreshControlPulses[ rung ] );
			busDiagChar( dst, capacity, n, '/' );
			if ( rung == REFRESH_PRE_CONT2
			     || rung == REFRESH_PRE_CORE0_CONT2
			     || rung == REFRESH_PRE_CONT8_A
			     || rung == REFRESH_PRE_CONT8_B
			     || rung == REFRESH_POST_CONT2 )
				busDiagText( dst, capacity, n, "NA" );
			else
				busDiagDecimal( dst, capacity, n,
				                m_RefreshControlExpectedPulses[ rung ] );
			busDiagChar( dst, capacity, n, '/' );
			busDiagDecimal( dst, capacity, n,
			                m_RefreshControlExposureLines[ rung ] );
			busDiagChar( dst, capacity, n, '/' );
			busDiagDecimal( dst, capacity, n,
			                m_RefreshControlAnchorPhi[ rung ] );
			busDiagChar( dst, capacity, n, '/' );
			busDiagDecimal( dst, capacity, n,
			                m_RefreshControlAnchorArm[ rung ] );
			busDiagEndLine( dst, capacity, n );
			busDiagText( dst, capacity, n,
			             "  timing-us exposure/capture/pass0/pass1/pass2: " );
			busDiagDecimal( dst, capacity, n,
			                m_RefreshControlExposureUsec[ rung ] );
			busDiagChar( dst, capacity, n, '/' );
			busDiagDecimal( dst, capacity, n,
			                m_RefreshControlCaptureUsec[ rung ] );
			for ( u32 pass = 0; pass < 3; pass++ )
			{
				busDiagChar( dst, capacity, n, '/' );
				busDiagDecimal( dst, capacity, n,
				                m_RefreshControlCapturePassUsec
				                  [ rung ][ pass ] );
			}
			busDiagEndLine( dst, capacity, n );
			for ( u32 pass = 0; pass < 3; pass++ )
			{
				busDiagText( dst, capacity, n, "  mismatch-mask-read" );
				busDiagChar( dst, capacity, n, (char)( '0' + pass ) );
				busDiagText( dst, capacity, n, ": " );
				busDiagBytes( dst, capacity, n,
				              m_RefreshControlMismatchMask[ rung ][ pass ], 33 );
				busDiagEndLine( dst, capacity, n );
			}
			u8 controlUnion[ 33 ];
			for ( u32 byte = 0; byte < 33; byte++ )
			{
				controlUnion[ byte ] = 0;
				for ( u32 pass = 0; pass < 3; pass++ )
					controlUnion[ byte ] |=
						m_RefreshControlMismatchMask[ rung ][ pass ][ byte ];
			}
			busDiagText( dst, capacity, n, "  unique-union-failures: " );
			busDiagDecimal( dst, capacity, n,
			                refreshMaskBitCount( controlUnion, 33 ) );
			busDiagEndLine( dst, capacity, n );
		}
		u8 stressUnionA[ 33 ], stressUnionB[ 33 ];
		bool stressSame = true;
		for ( u32 byte = 0; byte < 33; byte++ )
		{
			stressUnionA[ byte ] = stressUnionB[ byte ] = 0;
			for ( u32 pass = 0; pass < 3; pass++ )
			{
				stressUnionA[ byte ] |= m_RefreshControlMismatchMask
					[ REFRESH_PRE_CONT8_A ][ pass ][ byte ];
				stressUnionB[ byte ] |= m_RefreshControlMismatchMask
					[ REFRESH_PRE_CONT8_B ][ pass ][ byte ];
			}
			if ( stressUnionA[ byte ] != stressUnionB[ byte ] ) stressSame = false;
		}
		busDiagText( dst, capacity, n,
		             "continuous8 duplicate valid/unique-a/unique-b/same-union-mask: " );
		busDiagDecimal( dst, capacity, n,
		                m_RefreshControlValid[ REFRESH_PRE_CONT8_A ]
		             && m_RefreshControlValid[ REFRESH_PRE_CONT8_B ] ? 1u : 0u );
		busDiagChar( dst, capacity, n, '/' );
		busDiagDecimal( dst, capacity, n, refreshMaskBitCount( stressUnionA, 33 ) );
		busDiagChar( dst, capacity, n, '/' );
		busDiagDecimal( dst, capacity, n, refreshMaskBitCount( stressUnionB, 33 ) );
		busDiagChar( dst, capacity, n, '/' );
		busDiagDecimal( dst, capacity, n, stressSame ? 1u : 0u );
		busDiagEndLine( dst, capacity, n );
		u8 emitterUnionCore3[ 33 ], emitterUnionCore0[ 33 ];
		bool emitterSame = true;
		for ( u32 byte = 0; byte < 33; byte++ )
		{
			emitterUnionCore3[ byte ] = emitterUnionCore0[ byte ] = 0;
			for ( u32 pass = 0; pass < 3; pass++ )
			{
				emitterUnionCore3[ byte ] |= m_RefreshControlMismatchMask
					[ REFRESH_PRE_CONT2 ][ pass ][ byte ];
				emitterUnionCore0[ byte ] |= m_RefreshControlMismatchMask
					[ REFRESH_PRE_CORE0_CONT2 ][ pass ][ byte ];
			}
			if ( emitterUnionCore3[ byte ] != emitterUnionCore0[ byte ] )
				emitterSame = false;
		}
		busDiagText( dst, capacity, n,
		             "emitter-ab valid/core3-unique/core0-unique/same-union-mask: " );
		busDiagDecimal( dst, capacity, n,
		                m_RefreshControlValid[ REFRESH_PRE_CONT2 ]
		             && m_RefreshControlValid[ REFRESH_PRE_CORE0_CONT2 ] ? 1u : 0u );
		busDiagChar( dst, capacity, n, '/' );
		busDiagDecimal( dst, capacity, n,
		                refreshMaskBitCount( emitterUnionCore3, 33 ) );
		busDiagChar( dst, capacity, n, '/' );
		busDiagDecimal( dst, capacity, n,
		                refreshMaskBitCount( emitterUnionCore0, 33 ) );
		busDiagChar( dst, capacity, n, '/' );
		busDiagDecimal( dst, capacity, n, emitterSame ? 1u : 0u );
		busDiagEndLine( dst, capacity, n );
		busDiagText( dst, capacity, n,
		             "topology E18M trusted/scan-complete/rungs/linecycles/W/P/F/?/contiguous/fail-band-start: " );
		busDiagDecimal( dst, capacity, n, m_PhaseScanStage1Agree );
		busDiagChar( dst, capacity, n, '/' );
		busDiagDecimal( dst, capacity, n, m_PhaseScanStage2Ran );
		busDiagChar( dst, capacity, n, '/' );
		busDiagDecimal( dst, capacity, n, m_PhaseScanStage2RungsCompleted );
		busDiagChar( dst, capacity, n, '/' );
		busDiagDecimal( dst, capacity, n, m_PhaseScanLineCycles );
		busDiagChar( dst, capacity, n, '/' );
		busDiagDecimal( dst, capacity, n,
		                m_PhaseScanLineCycles ? m_PhaseScanLineCycles - 1 : 0 );
		busDiagChar( dst, capacity, n, '/' );
		busDiagDecimal( dst, capacity, n, m_PhaseScanStage2PassCount );
		busDiagChar( dst, capacity, n, '/' );
		busDiagDecimal( dst, capacity, n, m_PhaseScanStage2FailureCount );
		busDiagChar( dst, capacity, n, '/' );
		busDiagDecimal( dst, capacity, n, m_PhaseScanStage2FlagCount );
		busDiagChar( dst, capacity, n, '/' );
		busDiagDecimal( dst, capacity, n, m_PhaseScanStage2Contiguous );
		busDiagChar( dst, capacity, n, '/' );
		if ( m_PhaseScanStage2BandStart == 0xFF )
			busDiagText( dst, capacity, n, "NA" );
		else
			busDiagDecimal( dst, capacity, n, m_PhaseScanStage2BandStart );
		busDiagEndLine( dst, capacity, n );
		if ( m_PhaseScanStage2RungsCompleted )
		{
			const u32 phases = m_PhaseScanLineCycles;
			busDiagText( dst, capacity, n,
			             "topology K=0..line-1 vector P=unique<=5 F=unique>=20 ?=flag !=not-run: " );
			for ( u32 k = 0; k < phases; k++ )
				busDiagChar( dst, capacity, n,
				             (char)m_PhaseScanStage2Class[ k ] );
			busDiagEndLine( dst, capacity, n );
			busDiagText( dst, capacity, n, "  execution-index-to-K: " );
			for ( u32 executionIndex = 0; executionIndex < phases;
			      executionIndex++ )
			{
				if ( executionIndex ) busDiagChar( dst, capacity, n, ' ' );
				busDiagDecimal( dst, capacity, n,
				                m_PhaseScanExecutionK[ executionIndex ] );
			}
			busDiagEndLine( dst, capacity, n );
			busDiagText( dst, capacity, n, "  K-to-execution-index: " );
			for ( u32 k = 0; k < phases; k++ )
			{
				if ( k ) busDiagChar( dst, capacity, n, ' ' );
				busDiagDecimal( dst, capacity, n,
				                m_PhaseScanStage2ExecutionIndex[ k ] );
			}
			busDiagEndLine( dst, capacity, n );
			busDiagText( dst, capacity, n,
			             "  execution-order-unique-failures: " );
			for ( u32 executionIndex = 0; executionIndex < phases;
			      executionIndex++ )
			{
				if ( executionIndex ) busDiagChar( dst, capacity, n, ' ' );
				const u32 k = m_PhaseScanExecutionK[ executionIndex ];
				busDiagDecimal( dst, capacity, n,
				                k < 65 ? m_PhaseScanStage2Failures[ k ]
				                       : 0xFFFFu );
			}
			busDiagEndLine( dst, capacity, n );
			busDiagText( dst, capacity, n, "  unique-failures: " );
			for ( u32 k = 0; k < phases; k++ )
			{
				if ( k ) busDiagChar( dst, capacity, n, ' ' );
				busDiagDecimal( dst, capacity, n,
				                m_PhaseScanStage2Failures[ k ] );
			}
			busDiagEndLine( dst, capacity, n );
			busDiagText( dst, capacity, n, "  baseline-attempts: " );
			for ( u32 k = 0; k < phases; k++ )
			{
				if ( k ) busDiagChar( dst, capacity, n, ' ' );
				busDiagDecimal( dst, capacity, n,
				                m_PhaseScanStage2BaselineAttempts[ k ] );
			}
			busDiagEndLine( dst, capacity, n );
			busDiagText( dst, capacity, n, "  baseline-failures: " );
			for ( u32 k = 0; k < phases; k++ )
			{
				if ( k ) busDiagChar( dst, capacity, n, ' ' );
				busDiagDecimal( dst, capacity, n,
				                m_PhaseScanStage2BaselineFailures[ k ] );
			}
			busDiagEndLine( dst, capacity, n );
			busDiagText( dst, capacity, n, "  baseline-unstable: " );
			for ( u32 k = 0; k < phases; k++ )
			{
				if ( k ) busDiagChar( dst, capacity, n, ' ' );
				busDiagDecimal( dst, capacity, n,
				                m_PhaseScanStage2BaselineUnstable[ k ] );
			}
			busDiagEndLine( dst, capacity, n );
			busDiagText( dst, capacity, n,
			             "  baseline-timing-us-seed: " );
			for ( u32 k = 0; k < phases; k++ )
			{
				if ( k ) busDiagChar( dst, capacity, n, ' ' );
				busDiagDecimal( dst, capacity, n,
				                m_PhaseScanStage2BaselineSeedUsec[ k ] );
			}
			busDiagEndLine( dst, capacity, n );
			busDiagText( dst, capacity, n,
			             "  baseline-timing-us-capture: " );
			for ( u32 k = 0; k < phases; k++ )
			{
				if ( k ) busDiagChar( dst, capacity, n, ' ' );
				busDiagDecimal( dst, capacity, n,
				                m_PhaseScanStage2BaselineCaptureUsec[ k ] );
			}
			busDiagEndLine( dst, capacity, n );
			busDiagText( dst, capacity, n,
			             "  exposure-capture-timing-us: " );
			for ( u32 k = 0; k < phases; k++ )
			{
				if ( k ) busDiagChar( dst, capacity, n, ' ' );
				busDiagDecimal( dst, capacity, n,
				                m_PhaseScanStage2CaptureUsec[ k ] );
			}
			busDiagEndLine( dst, capacity, n );
			for ( u32 pass = 0; pass < 3; pass++ )
			{
				busDiagText( dst, capacity, n, "  pass-failures-read" );
				busDiagChar( dst, capacity, n, (char)( '0' + pass ) );
				busDiagText( dst, capacity, n, ": " );
				for ( u32 k = 0; k < phases; k++ )
				{
					if ( k ) busDiagChar( dst, capacity, n, ' ' );
					busDiagDecimal( dst, capacity, n,
					                m_PhaseScanStage2PassFailures[ k ][ pass ] );
				}
				busDiagEndLine( dst, capacity, n );
			}
			busDiagText( dst, capacity, n, "  unstable-sums: " );
			for ( u32 k = 0; k < phases; k++ )
			{
				if ( k ) busDiagChar( dst, capacity, n, ' ' );
				busDiagDecimal( dst, capacity, n,
				                m_PhaseScanStage2Unstable[ k ] );
			}
			busDiagEndLine( dst, capacity, n );
			busDiagText( dst, capacity, n, "  exposure-pulses: " );
			for ( u32 k = 0; k < phases; k++ )
			{
				if ( k ) busDiagChar( dst, capacity, n, ' ' );
				busDiagDecimal( dst, capacity, n,
				                m_PhaseScanStage2Pulses[ k ] );
			}
			busDiagEndLine( dst, capacity, n );
			busDiagText( dst, capacity, n, "  exposure-lines: " );
			for ( u32 k = 0; k < phases; k++ )
			{
				if ( k ) busDiagChar( dst, capacity, n, ' ' );
				busDiagDecimal( dst, capacity, n,
				                m_PhaseScanStage2Lines[ k ] );
			}
			busDiagEndLine( dst, capacity, n );
			busDiagText( dst, capacity, n, "  anchor-phi: " );
			for ( u32 k = 0; k < phases; k++ )
			{
				if ( k ) busDiagChar( dst, capacity, n, ' ' );
				busDiagDecimal( dst, capacity, n,
				                m_PhaseScanStage2AnchorPhi[ k ] );
			}
			busDiagEndLine( dst, capacity, n );
			busDiagText( dst, capacity, n, "  anchor-arm: " );
			for ( u32 k = 0; k < phases; k++ )
			{
				if ( k ) busDiagChar( dst, capacity, n, ' ' );
				busDiagDecimal( dst, capacity, n,
				                m_PhaseScanStage2AnchorArm[ k ] );
			}
			busDiagEndLine( dst, capacity, n );
			for ( u32 executionIndex = 0;
			      executionIndex < m_PhaseScanStage2RungsCompleted;
			      executionIndex++ )
			{
				const u32 k = m_PhaseScanExecutionK[ executionIndex ];
				busDiagText( dst, capacity, n, "topology-mask K=" );
				busDiagHex( dst, capacity, n, (u8)k );
				busDiagText( dst, capacity, n, " execution-index=" );
				busDiagDecimal( dst, capacity, n, executionIndex );
				busDiagText( dst, capacity, n, " class=" );
				busDiagChar( dst, capacity, n,
				             (char)m_PhaseScanStage2Class[ k ] );
				busDiagText( dst, capacity, n, " unique/unstable=" );
				busDiagDecimal( dst, capacity, n,
				                m_PhaseScanStage2Failures[ k ] );
				busDiagChar( dst, capacity, n, '/' );
				busDiagDecimal( dst, capacity, n,
				                m_PhaseScanStage2Unstable[ k ] );
				busDiagEndLine( dst, capacity, n );
				for ( u32 pass = 0; pass < 3; pass++ )
				{
					busDiagText( dst, capacity, n, "  mismatch-mask-read" );
					busDiagChar( dst, capacity, n, (char)( '0' + pass ) );
					busDiagText( dst, capacity, n, ": " );
					busDiagBytes( dst, capacity, n,
					              m_PhaseScanStage2MismatchMask[ k ][ pass ], 33 );
					busDiagEndLine( dst, capacity, n );
				}
			}
		}
#if 0 // Superseded E18C-E phase/topology report; retained until controls settle.
		busDiagText( dst, capacity, n,
		             "phase E18E ran/full-protect-pass/omit-scan-ran/linecycles/omit-fail-count/contiguous/fail-band-start: " );
		busDiagDecimal( dst, capacity, n, m_PhaseScanRan );
		busDiagChar( dst, capacity, n, '/' );
		busDiagDecimal( dst, capacity, n, m_PhaseScanStage1Agree );
		busDiagChar( dst, capacity, n, '/' );
		busDiagDecimal( dst, capacity, n, m_PhaseScanStage2Ran );
		busDiagChar( dst, capacity, n, '/' );
		busDiagDecimal( dst, capacity, n, m_PhaseScanLineCycles );
		busDiagChar( dst, capacity, n, '/' );
		busDiagDecimal( dst, capacity, n, m_PhaseScanStage2PassCount );
		busDiagChar( dst, capacity, n, '/' );
		busDiagDecimal( dst, capacity, n, m_PhaseScanStage2Contiguous );
		busDiagChar( dst, capacity, n, '/' );
		if ( m_PhaseScanStage2BandStart == 0xFF )
			busDiagText( dst, capacity, n, "NA" );
		else
			busDiagDecimal( dst, capacity, n, m_PhaseScanStage2BandStart );
		busDiagEndLine( dst, capacity, n );
		busDiagText( dst, capacity, n,
		             "phase E18E last baseline K/W/attempts/failures/unstable: " );
		busDiagDecimal( dst, capacity, n, m_PhaseScanBaselineK );
		busDiagChar( dst, capacity, n, '/' );
		busDiagDecimal( dst, capacity, n, m_PhaseScanBaselineW );
		busDiagChar( dst, capacity, n, '/' );
		busDiagDecimal( dst, capacity, n, m_PhaseScanBaselineAttempts );
		busDiagChar( dst, capacity, n, '/' );
		busDiagDecimal( dst, capacity, n, m_PhaseScanBaselineFailures );
		busDiagChar( dst, capacity, n, '/' );
		busDiagDecimal( dst, capacity, n, m_PhaseScanBaselineUnstable );
		busDiagEndLine( dst, capacity, n );
		busDiagText( dst, capacity, n,
		             "phase E18E full-protect W=line K=0 failures/unstable/anchor-phi/anchor-arm: " );
		busDiagDecimal( dst, capacity, n, m_PhaseScanStage1Failures[ 0 ][ 0 ] );
		busDiagChar( dst, capacity, n, '/' );
		busDiagDecimal( dst, capacity, n, m_PhaseScanStage1Unstable[ 0 ][ 0 ] );
		busDiagChar( dst, capacity, n, '/' );
		busDiagDecimal( dst, capacity, n, m_PhaseScanStage1AnchorPhi[ 0 ][ 0 ] );
		busDiagChar( dst, capacity, n, '/' );
		busDiagDecimal( dst, capacity, n, m_PhaseScanStage1AnchorArm[ 0 ][ 0 ] );
		busDiagEndLine( dst, capacity, n );
		if ( m_PhaseScanStage1Agree )
		{
			const u32 phases = m_PhaseScanLineCycles;
			busDiagText( dst, capacity, n,
			             "phase E18E omit-one K=0..linecycles-1 vector P=safe-omit F=critical-omit: " );
			for ( u32 k = 0; k < phases; k++ )
			{
				const u16 f = m_PhaseScanStage2Failures[ k ];
				const u16 u = m_PhaseScanStage2Unstable[ k ];
				busDiagChar( dst, capacity, n,
				             f == 0xFFFF ? '!' : ( f == 0 && u == 0 ? 'P' : 'F' ) );
			}
			busDiagEndLine( dst, capacity, n );
			busDiagText( dst, capacity, n, "  failures: " );
			for ( u32 k = 0; k < phases; k++ )
			{
				if ( k ) busDiagChar( dst, capacity, n, ' ' );
				busDiagDecimal( dst, capacity, n,
				                m_PhaseScanStage2Failures[ k ] );
			}
			busDiagEndLine( dst, capacity, n );
			busDiagText( dst, capacity, n, "  unstable: " );
			for ( u32 k = 0; k < phases; k++ )
			{
				if ( k ) busDiagChar( dst, capacity, n, ' ' );
				busDiagDecimal( dst, capacity, n,
				                m_PhaseScanStage2Unstable[ k ] );
			}
			busDiagEndLine( dst, capacity, n );
			busDiagText( dst, capacity, n, "  anchor-phi: " );
			for ( u32 k = 0; k < phases; k++ )
			{
				if ( k ) busDiagChar( dst, capacity, n, ' ' );
				busDiagDecimal( dst, capacity, n,
				                m_PhaseScanStage2AnchorPhi[ k ] );
			}
			busDiagEndLine( dst, capacity, n );
			busDiagText( dst, capacity, n, "  anchor-arm: " );
			for ( u32 k = 0; k < phases; k++ )
			{
				if ( k ) busDiagChar( dst, capacity, n, ' ' );
				busDiagDecimal( dst, capacity, n,
				                m_PhaseScanStage2AnchorArm[ k ] );
			}
			busDiagEndLine( dst, capacity, n );
		}
#endif
		busDiagText( dst, capacity, n,
		             "refresh gap us/fail/unstable: " );
		static const u16 gapUsec[ 6 ] = { 250, 500, 1000, 2000, 4000, 8000 };
		for ( u32 i = 0; i < 6; i++ )
		{
			if ( i ) busDiagText( dst, capacity, n, " | " );
			busDiagDecimal( dst, capacity, n, gapUsec[ i ] );
			busDiagChar( dst, capacity, n, '/' );
			if ( m_RefreshGapFailures[ i ] == 0xFFFF )
				busDiagText( dst, capacity, n, "NA/NA" );
			else
			{
				busDiagDecimal( dst, capacity, n, m_RefreshGapFailures[ i ] );
				busDiagChar( dst, capacity, n, '/' );
				busDiagDecimal( dst, capacity, n, m_RefreshGapUnstable[ i ] );
			}
		}
		busDiagEndLine( dst, capacity, n );
	}

	busDiagText( dst, capacity, n, "read timing: configured=" );
	busDiagDecimal( dst, capacity, n, m_ReadTimingConfigured );
	if ( m_ReadTimingStart )
	{
		busDiagText( dst, capacity, n, " stable=" );
		busDiagDecimal( dst, capacity, n, m_ReadTimingStart );
		busDiagText( dst, capacity, n, ".." );
		busDiagDecimal( dst, capacity, n, m_ReadTimingEnd );
		busDiagText( dst, capacity, n, " selected=" );
		busDiagDecimal( dst, capacity, n, m_ReadTimingSelected );
	}
	else
		busDiagText( dst, capacity, n, " NO-STABLE-WINDOW" );
	busDiagEndLine( dst, capacity, n );

	busDiagText( dst, capacity, n, "write timing: configured=" );
	busDiagDecimal( dst, capacity, n, m_WriteTimingConfiguredAddr );
	busDiagChar( dst, capacity, n, '/' );
	busDiagDecimal( dst, capacity, n, m_WriteTimingConfiguredData );
	busDiagText( dst, capacity, n, " pass=" );
	busDiagDecimal( dst, capacity, n, m_WriteTimingPassingPoints );
	busDiagText( dst, capacity, n, " selected=" );
	busDiagDecimal( dst, capacity, n, m_WriteTimingSelectedAddr );
	busDiagChar( dst, capacity, n, '/' );
	busDiagDecimal( dst, capacity, n, m_WriteTimingSelectedData );
	busDiagEndLine( dst, capacity, n );

	busDiagText( dst, capacity, n, "raster: " );
	busDiagBytes( dst, capacity, n, m_TestRaster, 6 );
	busDiagText( dst, capacity, n, m_TestRasterChanged ? " advancing" : " FROZEN" );
	busDiagEndLine( dst, capacity, n );

	busDiagText( dst, capacity, n, "raw turnaround: machine=" );
	busDiagText( dst, capacity, n,
	             m_TurnDetectedMachine == MACHINE_C128 ? "C128" : "C64" );
	busDiagText( dst, capacity, n, " read_policy=direct" );
	busDiagText( dst, capacity, n, " flags: bit0=write-turn" );
	busDiagEndLine( dst, capacity, n );
	static const u16 turnTargets[ 3 ] = { 0x02FF, 0x0334, 0x0400 };
	for ( u32 p = 0; p < 3; p++ )
	{
		busDiagText( dst, capacity, n, "probe target=" );
		busDiagHexWord( dst, capacity, n, turnTargets[ p ] );
		busDiagText( dst, capacity, n, " neighbor-order=A0,A1,A2,A3,A4,A5,A6,A7,A8,A9" );
		busDiagEndLine( dst, capacity, n );

		busDiagText( dst, capacity, n, "  flags entry/baseT/baseN/w5A/singleT/singleN/sFlush/sPost/w37a/w37b/doubleT/doubleN/dFlush/dPost: " );
		busDiagBytes( dst, capacity, n, m_TurnFlags[ p ], 14 );
		busDiagEndLine( dst, capacity, n );

		busDiagText( dst, capacity, n, "  base target16: " );
		busDiagBytes( dst, capacity, n, m_TurnBaseTarget[ p ], 16 );
		busDiagText( dst, capacity, n, " neighbors: " );
		busDiagBytes( dst, capacity, n, m_TurnBaseNeighbor[ p ], 10 );
		busDiagEndLine( dst, capacity, n );

		busDiagText( dst, capacity, n, "  single5A target16: " );
		busDiagBytes( dst, capacity, n, m_TurnSingleTarget[ p ], 16 );
		busDiagText( dst, capacity, n, " neighbors: " );
		busDiagBytes( dst, capacity, n, m_TurnSingleNeighbor[ p ], 10 );
		busDiagEndLine( dst, capacity, n );
		busDiagText( dst, capacity, n, "  single flush02FC=" );
		busDiagHex( dst, capacity, n, m_TurnSingleFlush[ p ] );
		busDiagText( dst, capacity, n, " post-target: " );
		busDiagBytes( dst, capacity, n, m_TurnSinglePostFlush[ p ], 3 );
		busDiagEndLine( dst, capacity, n );

		busDiagText( dst, capacity, n, "  double37 target16: " );
		busDiagBytes( dst, capacity, n, m_TurnDoubleTarget[ p ], 16 );
		busDiagText( dst, capacity, n, " neighbors: " );
		busDiagBytes( dst, capacity, n, m_TurnDoubleNeighbor[ p ], 10 );
		busDiagEndLine( dst, capacity, n );
		busDiagText( dst, capacity, n, "  double flush02FC=" );
		busDiagHex( dst, capacity, n, m_TurnDoubleFlush[ p ] );
		busDiagText( dst, capacity, n, " post-target: " );
		busDiagBytes( dst, capacity, n, m_TurnDoublePostFlush[ p ], 3 );
		busDiagEndLine( dst, capacity, n );
	}
	if ( !m_RefreshSweepCount )
	{
		busDiagText( dst, capacity, n, "refresh R1: NOT RUN" );
		busDiagEndLine( dst, capacity, n );
	}
	else
	{
		busDiagText( dst, capacity, n,
		             "refresh R1: page04 map, VIC-half DMA release for 60s, no RAM sweep" );
		busDiagText( dst, capacity, n, " slots=" );
		busDiagDecimal( dst, capacity, n, m_RefreshSweepCount );
		busDiagText( dst, capacity, n, " elapsed_ms=" );
		busDiagDecimal( dst, capacity, n, m_RefreshElapsedMs );
		busDiagText( dst, capacity, n, " pass_us=" );
		busDiagDecimal( dst, capacity, n, m_RefreshPassUsec );
		busDiagText( dst, capacity, n, " host_rw_low=" );
		busDiagDecimal( dst, capacity, n, m_RefreshHostWritePhases );
		busDiagText( dst, capacity, n, " failures=" );
		busDiagDecimal( dst, capacity, n, m_RefreshFailureCount );
		busDiagText( dst, capacity, n, " unstable=" );
		busDiagDecimal( dst, capacity, n, m_RefreshUnstableCount );
		busDiagEndLine( dst, capacity, n );
		busDiagText( dst, capacity, n,
		             "expected[lo]=(lo*$49+$35) mod $100; classification uses read1/read2" );
		busDiagEndLine( dst, capacity, n );
		busDiagText( dst, capacity, n, "failure bitmap (bit=low-byte, LSB first): " );
		busDiagBytes( dst, capacity, n, m_RefreshFailureBitmap, 32 );
		busDiagEndLine( dst, capacity, n );
		for ( u32 row = 0; row < 16; row++ )
		{
			busDiagText( dst, capacity, n, "map row " );
			busDiagHex( dst, capacity, n, (u8)row );
			busDiagText( dst, capacity, n, " expected: " );
			busDiagBytes( dst, capacity, n, &m_RefreshMapExpected[ row * 16 ], 16 );
			busDiagEndLine( dst, capacity, n );
			for ( u32 pass = 0; pass < 3; pass++ )
			{
				busDiagText( dst, capacity, n, "  read" );
				busDiagChar( dst, capacity, n, (char)( '0' + pass ) );
				busDiagText( dst, capacity, n, ": " );
				busDiagBytes( dst, capacity, n,
				              &m_RefreshMapReadback[ pass ][ row * 16 ], 16 );
				busDiagEndLine( dst, capacity, n );
			}
		}
		busDiagText( dst, capacity, n, "control expected: " );
		busDiagBytes( dst, capacity, n, m_RefreshControlExpected, 8 );
		busDiagEndLine( dst, capacity, n );
		for ( u32 pass = 0; pass < 3; pass++ )
		{
			busDiagText( dst, capacity, n, "control read" );
			busDiagChar( dst, capacity, n, (char)( '0' + pass ) );
			busDiagText( dst, capacity, n, ": " );
			busDiagBytes( dst, capacity, n, m_RefreshControlReadback[ pass ], 8 );
			busDiagEndLine( dst, capacity, n );
		}
		busDiagText( dst, capacity, n,
		             "R2 cyan: 16x16 map .=stable X=mismatch ?=unstable; VIC refresh hold" );
		busDiagEndLine( dst, capacity, n );
	}

	busDiagText( dst, capacity, n, "first-transfer actual: " );
	busDiagBytes( dst, capacity, n, m_TestAddrLines, 6 );
	busDiagText( dst, capacity, n, " expected: E0 E1 E2 E3 E4 E5 errors=" );
	busDiagDecimal( dst, capacity, n, m_TestAddrErrors );
	busDiagText( dst, capacity, n, " reread0=" );
	busDiagHex( dst, capacity, n, m_TestAddrFirstReread );
	busDiagEndLine( dst, capacity, n );

	busDiagText( dst, capacity, n, "banking after A5/5A: A000=" );
	busDiagHex( dst, capacity, n, m_TestBankA );
	busDiagText( dst, capacity, n, " E000=" );
	busDiagHex( dst, capacity, n, m_TestBankE );
	busDiagEndLine( dst, capacity, n );

	busDiagText( dst, capacity, n, "single actual: " );
	busDiagBytes( dst, capacity, n, m_TestSingle, 6 );
	busDiagText( dst, capacity, n, " expected: 3C C3 5A A5 69 96 errors=" );
	busDiagDecimal( dst, capacity, n, m_TestSingleErrors );
	busDiagText( dst, capacity, n, " repeats=" );
	busDiagDecimal( dst, capacity, n, m_TestRepeatedFailures );
	busDiagText( dst, capacity, n, "/20" );
	busDiagText( dst, capacity, n, " reread0=" );
	busDiagHex( dst, capacity, n, m_TestSingleFirstReread );
	busDiagEndLine( dst, capacity, n );
	busDiagText( dst, capacity, n, "positional raw before single: " );
	busDiagBytes( dst, capacity, n, m_PosRawBeforeSingle, 2 );
	busDiagText( dst, capacity, n, " expected: 6D 6D" );
	busDiagEndLine( dst, capacity, n );

	busDiagText( dst, capacity, n, "burst actual: " );
	busDiagBytes( dst, capacity, n, m_TestBurst, 6 );
	busDiagText( dst, capacity, n, " expected: 96 69 A5 5A C3 3C errors=" );
	busDiagDecimal( dst, capacity, n, m_TestBurstErrors );
	busDiagText( dst, capacity, n, " reread0=" );
	busDiagHex( dst, capacity, n, m_TestBurstFirstReread );
	busDiagEndLine( dst, capacity, n );

	static const u16 discTargets[ 3 ] = { 0x02FF, 0x0334, 0x0400 };
	for ( u32 p = 0; p < 3; p++ )
	{
		busDiagText( dst, capacity, n, "disc target=" );
		busDiagHexWord( dst, capacity, n, discTargets[p] );
		busDiagText( dst, capacity, n,
		             " sample old/new/prelag/pre/immediate/control/final/settled: " );
		busDiagBytes( dst, capacity, n, m_FirstDiscSample[p], 8 );
		busDiagEndLine( dst, capacity, n );
		busDiagText( dst, capacity, n,
		             "  counts ok/write-lost/first-read/false-pass/lag-control/other: " );
		for ( u32 c = 0; c < 6; c++ )
		{
			if ( c ) busDiagChar( dst, capacity, n, '/' );
			busDiagDecimal( dst, capacity, n, m_FirstDiscCount[p][c] );
		}
		busDiagEndLine( dst, capacity, n );
	}

	return n;
}

void CRADBus::probeIdleMaster( u32 &eligiblePhases, u32 &addrChanges,
	                           u32 &distinctAddrs, u32 &sequentialAddrs,
	                           u32 &writePhases, u8 &firstAddr, u8 &lastAddr )
{
	register u32 g2 = read32( ARM_GPIO_GPLEV0 );
	register u32 g3;
	eligiblePhases = addrChanges = distinctAddrs = sequentialAddrs = writePhases = 0;
	firstAddr = lastAddr = 0;
	u8 seen[ 32 ];
	for ( u32 i = 0; i < sizeof seen; i++ ) seen[ i ] = 0;

	// Do not release /DMA. Disable every RAD-facing driver and observe only
	// CPU half-cycles with BA high. VIC addresses during its own half-cycle are
	// expected, and BA low lets it extend ownership into the nominal CPU half.
	// R/W-low in an eligible phase is decisive: a correctly parked 8502/Z80
	// cannot issue a write.
	DISABLE_ADDRESS_LATCH_AND_BUSTRANSCEIVER( false )
	CLR_GPIO( bMPLEX_SEL );
	WAIT_FOR_VIC_HALFCYCLE

	bool haveAddr = false;
	for ( u32 i = 0; i < 4096; i++ )
	{
		WAIT_FOR_CPU_HALFCYCLE
		RESTART_CYCLE_COUNTER
		WAIT_UP_TO_CYCLE( WAIT_FOR_SIGNALS + TIMING_OFFSET_CBTD );
		g2 = read32( ARM_GPIO_GPLEV0 );
		if ( !( g2 & bBA ) )
		{
			WAIT_FOR_VIC_HALFCYCLE
			continue;
		}
		eligiblePhases++;
		if ( !( g2 & bRW_OUT ) ) writePhases++;

		SET_GPIO( bMPLEX_SEL );
		WAIT_UP_TO_CYCLE( WAIT_CYCLE_MULTIPLEXER );
		g3 = read32( ARM_GPIO_GPLEV0 );
		CLR_GPIO( bMPLEX_SEL );
		const u8 addr = (u8)( ( ( g3 >> 10 ) & 15 ) | ( ( g3 & 15 ) << 4 ) );
		if ( !( seen[ addr >> 3 ] & ( 1u << ( addr & 7 ) ) ) )
		{
			seen[ addr >> 3 ] |= (u8)( 1u << ( addr & 7 ) );
			distinctAddrs++;
		}
		if ( !haveAddr ) { firstAddr = addr; haveAddr = true; }
		else
		{
			if ( addr != lastAddr ) addrChanges++;
			if ( addr == (u8)( lastAddr + 1 ) ) sequentialAddrs++;
		}
		lastAddr = addr;
		WAIT_FOR_VIC_HALFCYCLE
	}
}

void CRADBus::setTrafficHalted( bool halted )
{
	m_TrafficHalted = halted;
	// Diagnostic isolation: once all ordinary RAD accesses stop, remove the
	// epoch suspension gap too. If physical DRAM still changes while core3
	// releases /DMA on every VIC half-cycle, the pulse primitive rather than
	// mirror/traffic scheduling is at fault.
	c128RefreshForceContinuous( halted );
}

bool CRADBus::buttonPressed()
{
	// The button is GPIO 3, which the 74LVC257 shares with address bit A7. The
	// button is only the value present while MPLEX_SEL is LOW, so the select
	// line has to be driven before sampling -- reading it blind samples A7
	// instead and reports phantom presses.
	//
	// That is not academic: it used to cut the post-reset settle short, so the
	// bus was taken while the KERNAL still had the screen blanked, and with no
	// badlines the handover could never find a safe point.
	CLR_GPIO( bMPLEX_SEL );

	// Let the multiplexer settle, then require two agreeing samples.
	DELAY( 64 );
	u32 a = read32( ARM_GPIO_GPLEV0 );
	DELAY( 64 );
	u32 b = read32( ARM_GPIO_GPLEV0 );

	return ( !( a & bBUTTON ) && !( b & bBUTTON ) );
}

bool CRADBus::hardwareResetPressed()
{
	// RESET_OUT is left as an input after radResetMachine(), so a low level
	// here means someone else is asserting reset -- the RAD's own reset button.
	u32 g2 = read32( ARM_GPIO_GPLEV0 );
	return CPU_RESET ? true : false;
}

void CRADBus::idleHold( u32 seconds )
{
	// Uninterruptible. This is used for the post-reset settle, where cutting the
	// wait short is exactly the failure we are avoiding. DELAY( 1 << 27 ) is
	// roughly one second at 1.4GHz.
	for ( u32 i = 0; i < seconds; i++ )
		DELAY( 1 << 27 );
}

void CRADBus::idleHoldInterruptible( u32 seconds )
{
	// For observation windows, where responding to the button is worth more
	// than the exact duration.
	for ( u32 i = 0; i < seconds * 8; i++ )
	{
		DELAY( 1 << 24 );
		if ( buttonPressed() )
			return;
	}
}

void CRADBus::signalAlive()
{
	register u32 g2;

	// Eight colours, three passes, about a third of a second each -- roughly
	// eight seconds in total.
	//
	// The first version of this used DELAY( 1 << 22 ), which is about 36ms per
	// colour: the whole sequence finished in a fifth of a second and was easy to
	// miss entirely, especially on a display that is not stable. A diagnostic
	// nobody can see is worse than none, because "no flash" then gets read as
	// "writes are broken".
	static const u8 colours[] = { 2, 7, 5, 13, 14, 3, 1, 0 };

	for ( u32 pass = 0; pass < 3; pass++ )
	{
		for ( u32 i = 0; i < 8; i++ )
		{
			RAD_SPOKE( 0xD020, colours[ i ] );
			DELAY( 1 << 25 );
		}
	}

	m_Writes += 24;
}

u8 CRADBus::read( u16 addr )
{
	if ( m_TrafficHalted ) return 0xFF;
	u8 v = 0xFF;

	RAD_SPEEK( addr, v );
	m_Reads++;
	return v;
}

void CRADBus::write( u16 addr, u8 value )
{
	if ( m_TrafficHalted ) return;

	RAD_SPOKE( addr, value );
	m_Writes++;
}

void CRADBus::writeBurst( const C64BusWrite *writes, u32 count )
{
	if ( count == 0 || m_TrafficHalted )
		return;

	register u32 g2;
	const bool epochC128 =
		__atomic_load_n( &g_C128EpochEnabled, __ATOMIC_RELAXED ) != 0;
	if ( epochC128 )
	{
		// Kernel190 proved that refresh-only operation and the isolated bus
		// self-test are not representative of the corruption produced by a
		// sustained mirrored screen drain. Remove every burst-only variable on
		// the physical C128: reused latches, grouped traffic ownership, internal
		// BA resync and the first-byte seed/repeat workarounds. The ordinary
		// self-synchronising write is the path that passed repeated testing.
		// This is deliberately slower; correctness establishes the primitive
		// before C128 burst throughput is reintroduced.
		for ( u32 i = 0; i < count; i++ )
			RAD_SPOKE( writes[ i ].addr, writes[ i ].value );
		m_BurstWrites += count;
		return;
	}
	u32 offset = 0;
	while ( offset < count )
	{
		// Bound the time core3 can be held at REQUEST. Four physical writes plus
		// the stable-path seed leave ample margin before E14's guarded refresh
		// deadline; the cap will be re-sized after the phase window is measured.
		const u32 remaining = count - offset;
		const u32 chunk = epochC128 && remaining > 4 ? 4 : remaining;
		// The C128 has intermittently swallowed byte zero of an optimized burst
		// while the same single-write primitive passes 20/20 repeated runs. Seed
		// that RAM byte through the stable path; the following burst still writes
		// the complete chunk, so queue and ordering semantics do not change.
		if ( epochC128 )
			RAD_SPOKE( writes[ offset ].addr, writes[ offset ].value );
		// The buffered C128 may swallow the first physical write after a read
		// direction run. Repeating that RAM write is side-effect free.
		const bool repeatFirst = busWriteTurnaroundNeeded != 0;

		busBeginBurstWrites( g2 );
		BUS_RESYNC
		for ( u32 i = 0; i < chunk; i++ )
		{
			RAD_BURST_POKE( writes[ offset + i ].addr,
			                writes[ offset + i ].value );
			if ( i == 0 && repeatFirst )
				RAD_BURST_POKE( writes[ offset ].addr, writes[ offset ].value );
		}
		busEndBurstWrites( g2 );
		offset += chunk;
	}

	m_BurstWrites += count;
}

void CRADBus::readBlock( u16 addr, u8 *dst, u32 length )
{
	if ( m_TrafficHalted )
	{
		for ( u32 i = 0; i < length; i++ ) dst[ i ] = 0xFF;
		return;
	}
	for ( u32 i = 0; i < length; i++ )
	{
		u8 v = 0xFF;
		RAD_SPEEK( (u16)( addr + i ), v );
		dst[ i ] = v;
	}

	m_Reads += length;
}

bool CRADBus::irqAsserted()
{
	// /IRQ is a direct pin on the level shifter, readable whenever we have it
	// configured as an input (which the hijack sequence leaves it as).
	u32 g2 = read32( ARM_GPIO_GPLEV0 );
	return CPU_IRQ_LOW ? true : false;
}

bool CRADBus::nmiAsserted()
{
	// /NMI shares its GPIO with address bit A4 through the multiplexer, and is
	// the value present while MPLEX_SEL is low -- which is where the bus is
	// left outside of an access.
	u32 g2 = read32( ARM_GPIO_GPLEV0 );
	return CPU_NMI_LOW ? true : false;
}

void CRADBus::sampleInterrupts( bool &irq, bool &nmi )
{
	// One MMIO read serves both lines; see the note in c64_bus.h.
	u32 g2 = read32( ARM_GPIO_GPLEV0 );
	irq = CPU_IRQ_LOW ? true : false;
	nmi = CPU_NMI_LOW ? true : false;
}

u64 CRADBus::hostCycles()
{
	u64 c;
	READ_CYCLE_COUNTER( c );
	return c;
}

u16 CRADBus::rasterLine()
{
	if ( m_TrafficHalted ) return 0xFFFF;
	const u16 rasterLines = (u16)c64RasterLines( m_Signals.video );

	// $D012 contains the low eight raster bits and $D011 bit 7 contains bit 8.
	// Reading low then high can straddle either the 255 -> 256 transition or
	// the end-of-frame wrap and manufacture a line which never existed (most
	// notably $1FF). Bracket the low byte with two high-bit reads and accept the
	// sample only when both sides agree. The range check is a second line of
	// defence and also keeps a bad bus read out of the transfer scheduler.
	for ( u32 attempt = 0; attempt < 4; attempt++ )
	{
		u8 hiBefore = 0, lo = 0, hiAfter = 0;

		RAD_SPEEK( 0xD011, hiBefore );
		RAD_SPEEK( 0xD012, lo );
		RAD_SPEEK( 0xD011, hiAfter );
		m_Reads += 3;

		if ( ( ( hiBefore ^ hiAfter ) & 0x80 ) != 0 )
			continue;

		const u16 line = (u16)( lo | ( ( hiAfter & 0x80 ) ? 0x100 : 0 ) );
		if ( line < rasterLines )
		{
			if ( m_Signals.machine == MACHINE_C128 )
				c128RefreshObserveRaster( line );
			return line;
		}
	}

	return 0xFFFF;
}
