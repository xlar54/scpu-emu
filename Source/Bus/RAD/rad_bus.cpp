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
// Armed only after the post-snapshot $01=$34 takeover verifies. Ordinary
// $D000-$DFFF accesses then pulse /GAME for one CPU half-cycle; force-RAM
// operations and mirror bursts leave it released.
static bool radSelectIOWithGame = false;

#include "rad_lowlevel.h"
#include "rad_bus.h"
#include "cpu_hijack.h"
#include "c128_refresh.h"
#include "../../SuperCPU/write_buffer.h"

CRADBus::CRADBus()
	: m_Reads( 0 ), m_Writes( 0 ), m_BurstWrites( 0 ),
	  m_LastTransferCycles( 0 ), m_Transfers( 0 ), m_SerialTransfers( 0 ),
	  m_ReadPrimes( 0 ), m_Acquired( false ), m_RAMUnderIOReady( false ),
	  m_RAMUnderIOFailure( RAMUNDERIO_OK ),
	  m_TrafficHalted( false ),
	  m_SelfTestFailure( 0 ),
	  m_ReadTimingConfigured( 0 ), m_ReadTimingStart( 0 ), m_ReadTimingEnd( 0 ),
	  m_ReadTimingSelected( 0 ), m_ReadTimingBestError( 0 ),
	  m_ReadTimingBestErrorSample( 0 ), m_LoadedReadTimingRan( false ),
	  m_QuietReadTimingConfigured( 0 ), m_QuietReadTimingStart( 0 ),
	  m_QuietReadTimingEnd( 0 ), m_QuietReadTimingSelected( 0 ),
	  m_QuietReadTimingBestError( 0 ), m_QuietReadTimingBestErrorSample( 0 ),
	  m_ReadTimingRAMBestError( 0 ),
	  m_ReadTimingRAMBestSample( 0 ), m_ReadTimingRAMOnlyBestError( 0 ),
	  m_ReadTimingRAMOnlyBestSample( 0 ),
	  m_SnapshotKernalFirst( 0 ), m_SnapshotKernalReread( 0 ),
	  m_SnapshotKernalCopied( 0 ), m_SnapshotBasicFirst( 0 ),
	  m_SnapshotBasicReread( 0 ), m_SnapshotBasicCopied( 0 ),
	  m_SnapshotKernalObserved( 0 ), m_SnapshotBasicObserved( 0 ),
	  m_WriteTimingConfiguredAddr( 0 ), m_WriteTimingConfiguredData( 0 ),
	  m_WriteTimingSelectedAddr( 0 ), m_WriteTimingSelectedData( 0 ),
	  m_WriteTimingPassingPoints( 0 ),
	  m_SIDTimingConfigured( 0 ), m_SIDTimingStart( 0 ), m_SIDTimingEnd( 0 ),
	  m_SIDTimingSelected( 0 ), m_SIDTimingBestSample( 0 ), m_SIDTimingDistinct( 0 ),
	  m_SIDTimingDominant( 0 ), m_SIDTimingRamp( 0 ), m_SIDTimingEdge( 0 ),
	  m_SIDPhysicalReliable( false ), m_SIDModelFreq( 0 ), m_SIDModelControl( 0 ),
	  m_SIDModelPhase( 0 ), m_SIDModelLastHost( 0 ), m_SIDModelRemainder( 0 ),
	  m_TestBankA( 0 ), m_TestBankE( 0 ),
	  m_TestRasterChanged( 0 ), m_TestAddrErrors( 0 ), m_TestSingleErrors( 0 ),
	  m_TestBurstErrors( 0 ), m_TestRepeatedFailures( 0 ),
	  m_AccessSentinelRan( false ), m_AccessSentinelTrafficCount( 0 ),
	  m_DisplaySentinelRan( false ), m_DisplayAddressRan( false ),
	  m_DisplayRowRan( false ), m_DisplayFetchRan( false ),
	  m_DisplayPersistenceRan( false ), m_DisplayTimingRan( false ),
	  m_DisplayBoundaryRan( false ),
	  m_DisplayRefreshRan( false ),
	  m_DisplayDiagnosticVariant( 0 ),
	  m_DisplayBAGuardRan( false ),
	  m_Logger( 0 )
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
	for ( u32 i = 0; i < READ_TIMING_SCORE_COUNT; i++ )
	{
		m_ReadTimingErrors[ i ] = 0;
		m_QuietReadTimingErrors[ i ] = 0;
		m_ReadTimingRAMErrors[ i ] = 0;
		m_ReadTimingRAMOnlyErrors[ i ] = 0;
		m_ReadTimingMixedVICErrors[ i ] = 0;
		m_ReadTimingIsolatedVICErrors[ i ] = 0;
		m_ReadTimingMixedVICDistinct[ i ] = 0;
		m_ReadTimingMixedVICDominantValue[ i ] = 0;
		m_ReadTimingMixedVICDominantCount[ i ] = 0;
		m_ReadTimingIsolatedVICDistinct[ i ] = 0;
		m_ReadTimingIsolatedVICDominantValue[ i ] = 0;
		m_ReadTimingIsolatedVICDominantCount[ i ] = 0;
		for ( u32 r = 0; r < READ_TIMING_REPETITIONS; r++ )
		{
			m_ReadTimingMixedVICActual[ i ][ r ] = 0;
			m_ReadTimingIsolatedVICActual[ i ][ r ] = 0;
		}
		for ( u32 p = 0; p < 6; p++ )
		{
			m_ReadTimingMixedRAMPositionErrors[ i ][ p ] = 0;
			m_ReadTimingMixedRAMAddressErrors[ i ][ p ] = 0;
		}
	}
	for ( u32 arm = 0; arm < 3; arm++ )
	{
		m_AccessSentinelBaselineRetainedErrors[ arm ] = 0;
		m_AccessSentinelArmAddedErrors[ arm ] = 0;
		m_AccessSentinelArmRemovedErrors[ arm ] = 0;
		m_AccessSentinelSameErrors[ arm ] = 0;
		m_AccessSentinelNewErrors[ arm ] = 0;
		m_AccessSentinelClearedErrors[ arm ] = 0;
		m_AccessSentinelElapsedUsec[ arm ] = 0;
		for ( u32 pass = 0; pass < 2; pass++ )
		{
			m_AccessSentinelBaselineErrors[ arm ][ pass ] = 0;
			m_AccessSentinelExposureErrors[ arm ][ pass ] = 0;
			m_AccessSentinelFirstAddr[ arm ][ pass ] = 0;
			m_AccessSentinelFirstExpected[ arm ][ pass ] = 0;
			m_AccessSentinelFirstActual[ arm ][ pass ] = 0;
		}
	}
	for ( u32 arm = 0; arm < 9; arm++ )
	{
		m_DisplaySentinelDisplayOn[ arm ] = 0;
		m_DisplaySentinelOperation[ arm ] = 0;
		m_DisplaySentinelRate[ arm ] = 0;
		m_DisplaySentinelTrafficCount[ arm ] = 0;
		m_DisplaySentinelElapsedUsec[ arm ] = 0;
		m_DisplaySentinelBaselineRetainedErrors[ arm ] = 0;
		m_DisplaySentinelArmAddedErrors[ arm ] = 0;
		m_DisplaySentinelArmRemovedErrors[ arm ] = 0;
		for ( u32 pass = 0; pass < 2; pass++ )
		{
			m_DisplaySentinelBaselineErrors[ arm ][ pass ] = 0;
			m_DisplaySentinelVerifySameErrors[ arm ][ pass ] = 0;
			m_DisplaySentinelVerifyNewErrors[ arm ][ pass ] = 0;
			m_DisplaySentinelVerifyClearedErrors[ arm ][ pass ] = 0;
		}
		for ( u32 pass = 0; pass < 3; pass++ )
		{
			m_DisplaySentinelExposureErrors[ arm ][ pass ] = 0;
			m_DisplaySentinelFirstAddr[ arm ][ pass ] = 0;
			m_DisplaySentinelFirstExpected[ arm ][ pass ] = 0;
			m_DisplaySentinelFirstActual[ arm ][ pass ] = 0;
		}
	}
	m_DisplayAddressLadderRan = 0;
	for ( u32 mode = 0; mode < 2; mode++ )
	{
		for ( u32 style = 0; style < 2; style++ )
		{
			m_DisplayAddressImmediateFirstTrial[ mode ][ style ] = 0;
			m_DisplayAddressImmediateFirstExpected[ mode ][ style ] = 0;
			m_DisplayAddressMapErrors[ mode ][ style ] = 0;
			m_DisplayAddressMapFirst[ mode ][ style ] = 0;
			m_DisplayAddressMapLast[ mode ][ style ] = 0;
			m_DisplayAddressMapAND[ mode ][ style ] = 0;
			m_DisplayAddressMapOR[ mode ][ style ] = 0;
			m_DisplayAddressMapRuns[ mode ][ style ] = 0;
			m_DisplayAddressMapMaxRun[ mode ][ style ] = 0;
			for ( u32 read = 0; read < 3; read++ )
			{
				m_DisplayAddressImmediateErrors[ mode ][ style ][ read ] = 0;
				m_DisplayAddressImmediateFirstActual[ mode ][ style ][ read ] = 0;
			}
		}
		for ( u32 rung = 0; rung < 9; rung++ )
		{
			m_DisplayAddressDelayUsec[ rung ] = 0;
			m_DisplayAddressElapsedUsec[ mode ][ rung ] = 0;
			m_DisplayAddressLadderExpected[ mode ][ rung ] = 0;
			for ( u32 read = 0; read < 3; read++ )
			{
				m_DisplayAddressLadderInitial[ mode ][ rung ][ read ] = 0;
				m_DisplayAddressLadderDelayed[ mode ][ rung ][ read ] = 0;
			}
		}
	}
	for ( u32 arm = 0; arm < 10; arm++ )
	{
		m_DisplayRowMode[ arm ] = m_DisplayRowKind[ arm ] =
			m_DisplayRowSalt[ arm ] = 0;
		m_DisplayRowPrefillAttempts[ arm ] = 0;
		for ( u32 phase = 0; phase < 2; phase++ )
		{
			m_DisplayRowSeedUsec[ arm ][ phase ] = 0;
			m_DisplayRowCaptureUsec[ arm ][ phase ] = 0;
			m_DisplayRowErrors[ arm ][ phase ] = 0;
			m_DisplayRowFirst[ arm ][ phase ] = 0;
			m_DisplayRowLast[ arm ][ phase ] = 0;
			m_DisplayRowAND[ arm ][ phase ] = 0;
			m_DisplayRowOR[ arm ][ phase ] = 0;
			m_DisplayRowRuns[ arm ][ phase ] = 0;
			m_DisplayRowMaxRun[ arm ][ phase ] = 0;
		}
	}
	for ( u32 rung = 0; rung < 6; rung++ )
	{
		m_DisplayRowRetentionSalt[ rung ] = 0;
		m_DisplayRowRetentionAttempts[ rung ] = 0;
		m_DisplayRowRetentionDelayUsec[ rung ] = 0;
		m_DisplayRowRetentionElapsedUsec[ rung ] = 0;
		m_DisplayRowRetentionSeedUsec[ rung ] = 0;
		for ( u32 phase = 0; phase < 2; phase++ )
		{
			m_DisplayRowRetentionCaptureUsec[ rung ][ phase ] = 0;
			m_DisplayRowRetentionErrors[ rung ][ phase ] = 0;
			m_DisplayRowRetentionFirst[ rung ][ phase ] = 0;
			m_DisplayRowRetentionLast[ rung ][ phase ] = 0;
			m_DisplayRowRetentionAND[ rung ][ phase ] = 0;
			m_DisplayRowRetentionOR[ rung ][ phase ] = 0;
			m_DisplayRowRetentionRuns[ rung ][ phase ] = 0;
			m_DisplayRowRetentionMaxRun[ rung ][ phase ] = 0;
		}
	}
	for ( u32 arm = 0; arm < 12; arm++ )
	{
		m_DisplayPersistenceState[ arm ] =
			m_DisplayPersistenceOperation[ arm ] =
			m_DisplayPersistenceSalt[ arm ] =
			m_DisplayPersistencePrefillAttempts[ arm ] = 0;
		m_DisplayPersistenceTrafficCount[ arm ] =
			m_DisplayPersistenceElapsedUsec[ arm ] = 0;
		for ( u32 phase = 0; phase < 5; phase++ )
		{
			if ( phase < 2 ) m_DisplayPersistenceSeedUsec[ arm ][ phase ] = 0;
			m_DisplayPersistenceCaptureUsec[ arm ][ phase ] = 0;
			m_DisplayPersistenceErrors[ arm ][ phase ] = 0;
			m_DisplayPersistenceFirst[ arm ][ phase ] = 0;
			m_DisplayPersistenceLast[ arm ][ phase ] = 0;
			m_DisplayPersistenceAND[ arm ][ phase ] = 0;
			m_DisplayPersistenceOR[ arm ][ phase ] = 0;
			m_DisplayPersistenceRuns[ arm ][ phase ] = 0;
			m_DisplayPersistenceMaxRun[ arm ][ phase ] = 0;
			m_DisplayPersistenceFirstExpected[ arm ][ phase ] = 0;
			m_DisplayPersistenceFirstActual[ arm ][ phase ] = 0;
			m_DisplayPersistenceXorAND[ arm ][ phase ] = 0;
			m_DisplayPersistenceXorOR[ arm ][ phase ] = 0;
		}
		for ( u32 pair = 0; pair < 2; pair++ )
			m_DisplayPersistenceSame[ arm ][ pair ] =
				m_DisplayPersistenceAdded[ arm ][ pair ] =
				m_DisplayPersistenceRemoved[ arm ][ pair ] = 0;
	}
	for ( u32 phase = 0; phase < 10; phase++ )
		m_DisplayTimingSample[ phase ] = 0;
	for ( u32 arm = 0; arm < 6; arm++ )
	{
		m_DisplayTimingState[ arm ] = m_DisplayTimingOperation[ arm ] =
			m_DisplayTimingSalt[ arm ] = m_DisplayTimingPrefillAttempts[ arm ] = 0;
		m_DisplayTimingPrefillErrors[ arm ] =
			m_DisplayTimingBaselineErrors[ arm ] = 0;
		m_DisplayTimingTrafficCount[ arm ] =
			m_DisplayTimingElapsedUsec[ arm ] =
			m_DisplayTimingRepairWrites[ arm ] = 0;
		for ( u32 phase = 0; phase < 10; phase++ )
		{
			m_DisplayTimingCaptureUsec[ arm ][ phase ] = 0;
			m_DisplayTimingErrors[ arm ][ phase ] = 0;
			m_DisplayTimingFirst[ arm ][ phase ] = 0;
			m_DisplayTimingLast[ arm ][ phase ] = 0;
			m_DisplayTimingAND[ arm ][ phase ] = 0;
			m_DisplayTimingOR[ arm ][ phase ] = 0;
			m_DisplayTimingRuns[ arm ][ phase ] = 0;
			m_DisplayTimingMaxRun[ arm ][ phase ] = 0;
			m_DisplayTimingFirstExpected[ arm ][ phase ] = 0;
			m_DisplayTimingFirstActual[ arm ][ phase ] = 0;
			m_DisplayTimingXorAND[ arm ][ phase ] = 0;
			m_DisplayTimingXorOR[ arm ][ phase ] = 0;
			m_DisplayTimingSame[ arm ][ phase ] = 0;
			m_DisplayTimingAdded[ arm ][ phase ] = 0;
			m_DisplayTimingRemoved[ arm ][ phase ] = 0;
		}
	}
	for ( u32 arm = 0; arm < 16; arm++ )
	{
		m_DisplayBoundaryState[ arm ] = m_DisplayBoundarySafe[ arm ] =
			m_DisplayBoundaryDwell[ arm ] = m_DisplayBoundarySalt[ arm ] =
			m_DisplayBoundaryPrefillAttempts[ arm ] = 0;
		m_DisplayBoundaryPrefillErrors[ arm ] =
			m_DisplayBoundaryElapsedUsec[ arm ] = 0;
		m_DisplayBoundarySame[ arm ] = m_DisplayBoundaryAdded[ arm ] =
			m_DisplayBoundaryRemoved[ arm ] = 0;
		for ( u32 phase = 0; phase < 2; phase++ )
		{
			m_DisplayBoundaryWaitReads[ arm ][ phase ] = 0;
			m_DisplayBoundaryCaptureUsec[ arm ][ phase ] = 0;
			m_DisplayBoundaryErrors[ arm ][ phase ] = 0;
			m_DisplayBoundaryFirst[ arm ][ phase ] = 0;
			m_DisplayBoundaryLast[ arm ][ phase ] = 0;
			m_DisplayBoundaryAND[ arm ][ phase ] = 0;
			m_DisplayBoundaryOR[ arm ][ phase ] = 0;
			m_DisplayBoundaryRuns[ arm ][ phase ] = 0;
			m_DisplayBoundaryMaxRun[ arm ][ phase ] = 0;
			m_DisplayBoundaryFirstExpected[ arm ][ phase ] = 0;
			m_DisplayBoundaryFirstActual[ arm ][ phase ] = 0;
			m_DisplayBoundaryXorAND[ arm ][ phase ] = 0;
			m_DisplayBoundaryXorOR[ arm ][ phase ] = 0;
		}
	}
	for ( u32 arm = 0; arm < 8; arm++ )
	{
		m_DisplayRefreshEnabled[ arm ] = 0;
		m_DisplayRefreshStartOK[ arm ] = 0;
		m_DisplayRefreshSlots[ arm ] = 0;
		m_DisplayScrubOpportunities[ arm ] = 0;
		m_DisplayScrubMaxChunkCycles[ arm ] = 0;
	}
	for ( u32 arm = 0; arm < DISPLAY_BA_ARM_COUNT; arm++ )
	{
		m_DisplayBAOperation[ arm ] = m_DisplayBABias[ arm ] =
			m_DisplayBARotation[ arm ] = m_DisplayBAPrefillAttempts[ arm ] =
			m_DisplayBATargetAttempts[ arm ] = 0;
		m_DisplayBATrafficCount[ arm ] = m_DisplayBAElapsedUsec[ arm ] = 0;
		m_DisplayBABaselinePersistent[ arm ] =
			m_DisplayBABaselineUnion[ arm ] = m_DisplayBAPostPersistent[ arm ] =
			m_DisplayBAPostUnion[ arm ] = m_DisplayBAAddedPersistent[ arm ] =
			m_DisplayBAUnstableOnly[ arm ] = 0;
		for ( u32 pass = 0; pass < 6; pass++ )
			m_DisplayBACaptureUsec[ arm ][ pass ] = 0;
		for ( u32 family = 0; family < DISPLAY_BA_FAMILY_COUNT; family++ )
			m_DisplayBAFamilyAdded[ arm ][ family ] = 0;
	}
	for ( u32 arm = 0; arm < 9; arm++ )
	{
		m_DisplayFetchState[ arm ] = m_DisplayFetchOperation[ arm ] =
			m_DisplayFetchSalt[ arm ] = m_DisplayFetchPrefillAttempts[ arm ] = 0;
		m_DisplayFetchRate[ arm ] = m_DisplayFetchTrafficCount[ arm ] =
			m_DisplayFetchElapsedUsec[ arm ] = 0;
		for ( u32 phase = 0; phase < 3; phase++ )
		{
			if ( phase < 2 ) m_DisplayFetchSeedUsec[ arm ][ phase ] = 0;
			m_DisplayFetchCaptureUsec[ arm ][ phase ] = 0;
			m_DisplayFetchErrors[ arm ][ phase ] = 0;
			m_DisplayFetchFirst[ arm ][ phase ] = 0;
			m_DisplayFetchLast[ arm ][ phase ] = 0;
			m_DisplayFetchAND[ arm ][ phase ] = 0;
			m_DisplayFetchOR[ arm ][ phase ] = 0;
			m_DisplayFetchRuns[ arm ][ phase ] = 0;
			m_DisplayFetchMaxRun[ arm ][ phase ] = 0;
		}
	}
	for ( u32 rung = 0; rung < 6; rung++ )
	{
		m_DisplayFetchRetentionSalt[ rung ] =
			m_DisplayFetchRetentionPrefillAttempts[ rung ] = 0;
		m_DisplayFetchRetentionDelayUsec[ rung ] =
			m_DisplayFetchRetentionElapsedUsec[ rung ] = 0;
		for ( u32 phase = 0; phase < 3; phase++ )
		{
			if ( phase < 2 )
				m_DisplayFetchRetentionSeedUsec[ rung ][ phase ] = 0;
			m_DisplayFetchRetentionCaptureUsec[ rung ][ phase ] = 0;
			m_DisplayFetchRetentionErrors[ rung ][ phase ] = 0;
			m_DisplayFetchRetentionFirst[ rung ][ phase ] = 0;
			m_DisplayFetchRetentionLast[ rung ][ phase ] = 0;
			m_DisplayFetchRetentionAND[ rung ][ phase ] = 0;
			m_DisplayFetchRetentionOR[ rung ][ phase ] = 0;
			m_DisplayFetchRetentionRuns[ rung ][ phase ] = 0;
			m_DisplayFetchRetentionMaxRun[ rung ][ phase ] = 0;
		}
	}
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
	m_SIDPotFiltered[ 0 ] = m_SIDPotFiltered[ 1 ] = 0x80;
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
static bool sidReadAtFallingEdge = false;

__attribute__((noinline)) u8 radDirectRead( u16 addr, bool forceRAM )
{
	register u32 g2;
	u8 v = 0;
	const bool selectIO = radSelectIOWithGame && !forceRAM
	                   && addr >= 0xD000 && addr <= 0xDFFF;
	// K202's hardware-validated blocking path. A transaction claims the open
	// C128 traffic epoch before it aligns to the physical bus; C64-class hosts
	// return immediately because the refresh interlock is disabled.
	c128TrafficBegin();
	BUS_RESYNC
	busReadByte_p1( g2, addr );
	busReadByte_p2( g2, selectIO );
	const u32 sampleAt = ( ( addr & 0xFC00 ) == 0xD400 )
	                   ? (u32)busTiming.WAIT_CYCLE_READ_SID
	                   : (u32)busTiming.WAIT_CYCLE_READ;
	if ( ( addr & 0xFC00 ) == 0xD400 && sidReadAtFallingEdge )
		busReadByte_p3FallingEdge( g2, v, false, selectIO );
	else
		busReadByte_p3( g2, v, false, sampleAt, selectIO );
	c128TrafficEnd();
	return v;
}

static inline u8 busDiagRawRead( u16 addr )
{
	return radDirectRead( addr );
}

__attribute__((noinline)) void radDirectWrite( u16 addr, u8 value,
	                                           bool forceRAM )
{
	register u32 g2;
	const bool selectIO = radSelectIOWithGame && !forceRAM
	                   && addr >= 0xD000 && addr <= 0xDFFF;
	c128TrafficBegin();
	BUS_RESYNC
	busWriteByte_p1( g2, addr, value, selectIO );
	busWriteByte_p2( g2, false, selectIO );
	c128TrafficEnd();
}

static inline void busDiagRawWrite( u16 addr, u8 value )
{
	radDirectWrite( addr, value );
}

// C64 access-induced-corruption diagnostic. Keep the oracle out of zero page,
// the stack, the visible screen, ROM/I/O windows and the takeover workspace.
// $0800-$9FFF plus $C000-$CFFF covers 43,008 physical RAM bytes (about two
// thirds of the machine), making sparse stray writes much more likely to hit
// a sentinel than a traditional one-page bus test.
static const u32 ACCESS_SENTINEL_FIRST_LENGTH = 0x9800;
static const u32 ACCESS_SENTINEL_SECOND_LENGTH = 0x1000;
static const u32 ACCESS_SENTINEL_LENGTH =
	ACCESS_SENTINEL_FIRST_LENGTH + ACCESS_SENTINEL_SECOND_LENGTH;
static const u32 ACCESS_SENTINEL_BITMAP_BYTES =
	( ACCESS_SENTINEL_LENGTH + 7 ) / 8;
static u8 accessSentinelMismatch[ 2 ][ ACCESS_SENTINEL_BITMAP_BYTES ];

static inline u16 accessSentinelAddr( u32 index )
{
	return index < ACCESS_SENTINEL_FIRST_LENGTH
	     ? (u16)( 0x0800u + index )
	     : (u16)( 0xC000u + index - ACCESS_SENTINEL_FIRST_LENGTH );
}

static inline u8 accessSentinelExpected( u16 addr, u8 salt )
{
	const u32 mixed = (u32)addr * 73u + (u32)( addr >> 8 ) * 151u
	                + (u32)salt * 29u;
	return (u8)( mixed ^ ( addr >> 3 ) ^ ( addr >> 11 ) );
}

static void accessSentinelSeed( u8 salt )
{
	for ( u32 index = 0; index < ACCESS_SENTINEL_LENGTH; index++ )
	{
		const u16 addr = accessSentinelAddr( index );
		busDiagRawWrite( addr, accessSentinelExpected( addr, salt ) );
	}
}

static u32 accessSentinelCapture( u8 salt, u8 *mismatch,
	                              u16 &firstAddr, u8 &firstExpected,
	                              u8 &firstActual )
{
	// Keep a first-read-after-write residue out of the sentinel itself. These
	// two sacrificial reads are identical for every baseline/exposure pass and
	// target the cassette-buffer scratch already used by the ordinary self-test.
	(void)busDiagRawRead( 0x02FC );
	(void)busDiagRawRead( 0x02FC );
	for ( u32 byte = 0; byte < ACCESS_SENTINEL_BITMAP_BYTES; byte++ )
		mismatch[ byte ] = 0;

	u32 errors = 0;
	firstAddr = 0;
	firstExpected = firstActual = 0;
	for ( u32 index = 0; index < ACCESS_SENTINEL_LENGTH; index++ )
	{
		const u16 addr = accessSentinelAddr( index );
		const u8 expected = accessSentinelExpected( addr, salt );
		const u8 actual = busDiagRawRead( addr );
		if ( actual == expected ) continue;

		mismatch[ index >> 3 ] |= (u8)( 1u << ( index & 7 ) );
		if ( errors++ == 0 )
		{
			firstAddr = addr;
			firstExpected = expected;
			firstActual = actual;
		}
	}
	return errors;
}

static void accessSentinelCompare( const u8 *before, const u8 *after,
	                               u32 &same, u32 &added, u32 &removed )
{
	same = added = removed = 0;
	for ( u32 index = 0; index < ACCESS_SENTINEL_LENGTH; index++ )
	{
		const u8 bit = (u8)( 1u << ( index & 7 ) );
		const bool wasWrong = ( before[ index >> 3 ] & bit ) != 0;
		const bool isWrong = ( after[ index >> 3 ] & bit ) != 0;
		if ( wasWrong && isWrong ) same++;
		else if ( !wasWrong && isWrong ) added++;
		else if ( wasWrong && !isWrong ) removed++;
	}
}

static void displaySentinelCompare( const u8 *before, const u8 *after,
	                                u32 &same, u32 &added, u32 &removed );

// K222 displayed oracle. The matrix and bitmap are both ordinary physical RAM
// in VIC bank 0. DEN is always clear while these buffers are seeded or read;
// it is raised only for a display-on exposure arm.
static const u32 DISPLAY_SENTINEL_MATRIX_LENGTH = 1024;
static const u32 DISPLAY_SENTINEL_BITMAP_LENGTH = 8192;
static const u32 DISPLAY_SENTINEL_LENGTH =
	DISPLAY_SENTINEL_MATRIX_LENGTH + DISPLAY_SENTINEL_BITMAP_LENGTH;
static const u32 DISPLAY_SENTINEL_BITMAP_BYTES =
	( DISPLAY_SENTINEL_LENGTH + 7 ) / 8;
static u8 displaySentinelMismatch[ 2 ][ DISPLAY_SENTINEL_BITMAP_BYTES ];

static inline u16 displaySentinelAddr( u32 index )
{
	return index < DISPLAY_SENTINEL_MATRIX_LENGTH
	     ? (u16)( 0x0400u + index )
	     : (u16)( 0x2000u + index - DISPLAY_SENTINEL_MATRIX_LENGTH );
}

static inline u8 displaySentinelExpected( u32 index )
{
	if ( index < DISPLAY_SENTINEL_MATRIX_LENGTH )
		return 0x10; // white foreground, black background in hires mode
	const u32 bitmapOffset = index - DISPLAY_SENTINEL_MATRIX_LENGTH;
	const u32 rasterRow = bitmapOffset / 40u;
	return ( ( bitmapOffset ^ rasterRow ) & 1u ) ? 0x55 : 0xAA;
}

static void displaySentinelSetMode( bool displayOn, u8 border )
{
	static const u16 addrs[ 7 ] =
		{ 0xDD02, 0xDD00, 0xD015, 0xD016, 0xD018, 0xD020, 0xD021 };
	const u8 values[ 7 ] =
		{ 0x03,   0x03,   0x00,   0x08,   0x18,   border, 0x00 };
	for ( u32 i = 0; i < 7; i++ )
	{
		busDiagRawWrite( addrs[ i ], values[ i ] );
		busDiagRawWrite( addrs[ i ], values[ i ] );
	}
	const u8 d011 = displayOn ? 0x3B : 0x2B;
	busDiagRawWrite( 0xD011, d011 );
	busDiagRawWrite( 0xD011, d011 );
}

static void displaySentinelSetDEN( bool displayOn )
{
	const u8 d011 = displayOn ? 0x3B : 0x2B;
	busDiagRawWrite( 0xD011, d011 );
	busDiagRawWrite( 0xD011, d011 );
}

static void displaySentinelSeed()
{
	for ( u32 index = 0; index < DISPLAY_SENTINEL_LENGTH; index++ )
		busDiagRawWrite( displaySentinelAddr( index ),
		                 displaySentinelExpected( index ) );
}

static u32 displaySentinelCapture( u8 *mismatch,
	                               u16 &firstAddr, u8 &firstExpected,
	                               u8 &firstActual )
{
	(void)busDiagRawRead( 0x02FC );
	(void)busDiagRawRead( 0x02FC );
	for ( u32 byte = 0; byte < DISPLAY_SENTINEL_BITMAP_BYTES; byte++ )
		mismatch[ byte ] = 0;

	u32 errors = 0;
	firstAddr = 0;
	firstExpected = firstActual = 0;
	for ( u32 index = 0; index < DISPLAY_SENTINEL_LENGTH; index++ )
	{
		const u16 addr = displaySentinelAddr( index );
		const u8 expected = displaySentinelExpected( index );
		const u8 actual = busDiagRawRead( addr );
		if ( actual == expected ) continue;

		mismatch[ index >> 3 ] |= (u8)( 1u << ( index & 7 ) );
		if ( errors++ == 0 )
		{
			firstAddr = addr;
			firstExpected = expected;
			firstActual = actual;
		}
	}
	return errors;
}

static void displaySentinelCompare( const u8 *before, const u8 *after,
	                                u32 &same, u32 &added, u32 &removed )
{
	same = added = removed = 0;
	for ( u32 index = 0; index < DISPLAY_SENTINEL_LENGTH; index++ )
	{
		const u8 bit = (u8)( 1u << ( index & 7 ) );
		const bool wasWrong = ( before[ index >> 3 ] & bit ) != 0;
		const bool isWrong = ( after[ index >> 3 ] & bit ) != 0;
		if ( wasWrong && isWrong ) same++;
		else if ( !wasWrong && isWrong ) added++;
		else if ( wasWrong && !isWrong ) removed++;
	}
}

// K223 retains four complete maps: text/bitmap mode crossed with a one-pass
// or two-pass bulk seed. They are file-scope scratch so the diagnostic result
// formatter can preserve the raw evidence without bloating CRADBus itself.
static u8 displayAddressMismatch[ 2 ][ 2 ][ DISPLAY_SENTINEL_BITMAP_BYTES ];

static void displayAddressSetMode( bool bitmap, u8 border )
{
	static const u16 addrs[ 7 ] =
		{ 0xDD02, 0xDD00, 0xD015, 0xD016, 0xD018, 0xD020, 0xD021 };
	const u8 values[ 7 ] =
		{ 0x03,   0x03,   0x00,   0x08,   0x18,   border, 0x00 };
	for ( u32 i = 0; i < 7; i++ )
	{
		busDiagRawWrite( addrs[ i ], values[ i ] );
		busDiagRawWrite( addrs[ i ], values[ i ] );
	}
	// DEN is deliberately clear in both modes. Bit 5 is the only variable.
	const u8 d011 = bitmap ? 0x2B : 0x0B;
	busDiagRawWrite( 0xD011, d011 );
	busDiagRawWrite( 0xD011, d011 );
}

static void displayAddressSeed( u32 passes )
{
	for ( u32 pass = 0; pass < passes; pass++ )
		for ( u32 index = 0; index < DISPLAY_SENTINEL_LENGTH; index++ )
			busDiagRawWrite( displaySentinelAddr( index ),
			                 displaySentinelExpected( index ) );
}

static void displayAddressAnalyzeMap( const u8 *mismatch,
	                                  u32 &errors, u16 &first, u16 &last,
	                                  u16 &addrAND, u16 &addrOR,
	                                  u16 &runs, u16 &maxRun )
{
	errors = 0;
	first = last = runs = maxRun = 0;
	addrAND = 0xFFFF;
	addrOR = 0;
	u16 previous = 0;
	u16 currentRun = 0;
	for ( u32 index = 0; index < DISPLAY_SENTINEL_LENGTH; index++ )
	{
		if ( ( mismatch[ index >> 3 ]
		       & (u8)( 1u << ( index & 7 ) ) ) == 0 )
			continue;
		const u16 addr = displaySentinelAddr( index );
		if ( errors == 0 )
		{
			first = addr;
			currentRun = 1;
			runs = 1;
		}
		else if ( addr == (u16)( previous + 1u ) )
			currentRun++;
		else
		{
			if ( currentRun > maxRun ) maxRun = currentRun;
			currentRun = 1;
			runs++;
		}
		errors++;
		last = previous = addr;
		addrAND &= addr;
		addrOR |= addr;
	}
	if ( currentRun > maxRun ) maxRun = currentRun;
	if ( errors == 0 ) addrAND = 0;
}

static const u32 DISPLAY_ROW_ARM_COUNT = 10;
static const u32 DISPLAY_ROW_RETENTION_COUNT = 6;
static u8 displayRowPrefillMismatch
	[ DISPLAY_ROW_ARM_COUNT ][ DISPLAY_SENTINEL_BITMAP_BYTES ];
static u8 displayRowTestMismatch
	[ DISPLAY_ROW_ARM_COUNT ][ DISPLAY_SENTINEL_BITMAP_BYTES ];
static u8 displayRowRetentionBaselineMismatch
	[ DISPLAY_ROW_RETENTION_COUNT ][ DISPLAY_SENTINEL_BITMAP_BYTES ];
static u8 displayRowRetentionMismatch
	[ DISPLAY_ROW_RETENTION_COUNT ][ DISPLAY_SENTINEL_BITMAP_BYTES ];

static const u32 DISPLAY_FETCH_ARM_COUNT = 9;
static const u32 DISPLAY_FETCH_RETENTION_COUNT = 6;
static u8 displayFetchPrefillMismatch
	[ DISPLAY_FETCH_ARM_COUNT ][ DISPLAY_SENTINEL_BITMAP_BYTES ];
static u8 displayFetchBaselineMismatch
	[ DISPLAY_FETCH_ARM_COUNT ][ DISPLAY_SENTINEL_BITMAP_BYTES ];
static u8 displayFetchPostMismatch
	[ DISPLAY_FETCH_ARM_COUNT ][ DISPLAY_SENTINEL_BITMAP_BYTES ];
static u8 displayFetchRetentionPrefillMismatch
	[ DISPLAY_FETCH_RETENTION_COUNT ][ DISPLAY_SENTINEL_BITMAP_BYTES ];
static u8 displayFetchRetentionBaselineMismatch
	[ DISPLAY_FETCH_RETENTION_COUNT ][ DISPLAY_SENTINEL_BITMAP_BYTES ];
static u8 displayFetchRetentionPostMismatch
	[ DISPLAY_FETCH_RETENTION_COUNT ][ DISPLAY_SENTINEL_BITMAP_BYTES ];

static const u32 DISPLAY_PERSISTENCE_ARM_COUNT = 12;
static const u32 DISPLAY_PERSISTENCE_PHASE_COUNT = 5;
static u8 displayPersistenceMismatch
	[ DISPLAY_PERSISTENCE_ARM_COUNT ][ DISPLAY_PERSISTENCE_PHASE_COUNT ]
	[ DISPLAY_SENTINEL_BITMAP_BYTES ];
static const u32 DISPLAY_TIMING_ARM_COUNT = 6;
static const u32 DISPLAY_TIMING_PHASE_COUNT = 10;
static u8 displayTimingMismatch
	[ DISPLAY_TIMING_ARM_COUNT ][ DISPLAY_TIMING_PHASE_COUNT ]
	[ DISPLAY_SENTINEL_BITMAP_BYTES ];
static const u32 DISPLAY_BOUNDARY_ARM_COUNT = 16;
static const u32 DISPLAY_BOUNDARY_PHASE_COUNT = 2;
static u8 displayBoundaryMismatch
	[ DISPLAY_BOUNDARY_ARM_COUNT ][ DISPLAY_BOUNDARY_PHASE_COUNT ]
	[ DISPLAY_SENTINEL_BITMAP_BYTES ];
// K233 uses the production CWriteBuffer scrub against this authoritative
// oracle image. Only the matrix and bitmap ranges are populated.
static u8 displayScrubShadow[ 0x10000 ];

// K239 keeps every arm on the same composite oracle. Four bitmap families are
// interleaved by 64-byte block; the second repetition rotates the assignment so
// an address-sensitive weakness cannot masquerade as a data-pattern effect.
static const u32 DISPLAY_BA_LOCAL_ARM_COUNT = 12;
static const u32 DISPLAY_BA_CAPTURE_COUNT = 6;
static u8 displayBAMismatch
	[ DISPLAY_BA_LOCAL_ARM_COUNT ][ DISPLAY_BA_CAPTURE_COUNT ]
	[ DISPLAY_SENTINEL_BITMAP_BYTES ];
static u8 displayBAAddedMap
	[ DISPLAY_BA_LOCAL_ARM_COUNT ][ DISPLAY_SENTINEL_BITMAP_BYTES ];

static inline u8 displayBAFamily( u32 index, u8 rotation )
{
	if ( index < DISPLAY_SENTINEL_MATRIX_LENGTH ) return 0;
	const u32 offset = index - DISPLAY_SENTINEL_MATRIX_LENGTH;
	return (u8)( 1u + ( ( ( offset >> 6 ) + rotation ) & 3u ) );
}

static inline u8 displayBAExpected( u32 index, u8 rotation )
{
	const u8 family = displayBAFamily( index, rotation );
	if ( family == 0 ) return 0x10;
	if ( family == 1 ) return 0x00;
	if ( family == 2 ) return 0xFF;
	const u16 addr = displaySentinelAddr( index );
	if ( family == 3 )
	{
		const u32 offset = index - DISPLAY_SENTINEL_MATRIX_LENGTH;
		return ( ( ( offset >> 6 ) ^ ( offset / 320u ) ) & 1u ) ? 0xFF : 0x00;
	}
	const u32 mixed = (u32)addr * 73u + index * 29u + 0x6Du;
	return (u8)( mixed ^ ( mixed >> 8 ) ^ ( addr >> 3 ) ^ ( addr >> 11 ) );
}

static u32 displayBASeed( u8 rotation, bool complement, bool repeat )
{
	u64 started, finished;
	READ_CYCLE_COUNTER( started );
	for ( u32 index = 0; index < DISPLAY_SENTINEL_LENGTH; index++ )
	{
		u8 value = displayBAExpected( index, rotation );
		if ( complement ) value = (u8)~value;
		busDiagRawWrite( displaySentinelAddr( index ), value );
		if ( repeat ) busDiagRawWrite( displaySentinelAddr( index ), value );
	}
	READ_CYCLE_COUNTER( finished );
	return (u32)( ( finished - started )
	              / ( SCPU_ARM_CLOCK_HZ / 1000000u ) );
}

static u32 displayBACapture( u8 rotation, bool complement, u8 *mismatch,
	                         u32 &errors )
{
	u64 started, finished;
	READ_CYCLE_COUNTER( started );
	(void)busDiagRawRead( 0x02FC );
	(void)busDiagRawRead( 0x02FC );
	for ( u32 byte = 0; byte < DISPLAY_SENTINEL_BITMAP_BYTES; byte++ )
		mismatch[ byte ] = 0;
	errors = 0;
	for ( u32 index = 0; index < DISPLAY_SENTINEL_LENGTH; index++ )
	{
		u8 expected = displayBAExpected( index, rotation );
		if ( complement ) expected = (u8)~expected;
		if ( busDiagRawRead( displaySentinelAddr( index ) ) == expected ) continue;
		mismatch[ index >> 3 ] |= (u8)( 1u << ( index & 7 ) );
		errors++;
	}
	READ_CYCLE_COUNTER( finished );
	return (u32)( ( finished - started )
	              / ( SCPU_ARM_CLOCK_HZ / 1000000u ) );
}

static inline u8 displayRowExpected( u32 index, u8 salt )
{
	const u16 addr = displaySentinelAddr( index );
	const u32 mixed = (u32)addr * 73u + (u32)( addr >> 8 ) * 151u
	                + index * 29u + (u32)salt * 47u;
	return (u8)( mixed ^ ( mixed >> 8 ) ^ ( addr >> 3 )
	             ^ ( addr >> 11 ) );
}

static u32 displayRowSeed( u8 salt, bool complement, bool adjacentRepeat )
{
	u64 started, finished;
	READ_CYCLE_COUNTER( started );
	for ( u32 index = 0; index < DISPLAY_SENTINEL_LENGTH; index++ )
	{
		u8 value = displayRowExpected( index, salt );
		if ( complement ) value = (u8)~value;
		busDiagRawWrite( displaySentinelAddr( index ), value );
		if ( adjacentRepeat )
			busDiagRawWrite( displaySentinelAddr( index ), value );
	}
	READ_CYCLE_COUNTER( finished );
	return (u32)( ( finished - started )
	              / ( SCPU_ARM_CLOCK_HZ / 1000000u ) );
}

static u32 displayRowCapture( u8 salt, bool complement, u8 *mismatch )
{
	u64 started, finished;
	READ_CYCLE_COUNTER( started );
	(void)busDiagRawRead( 0x02FC );
	(void)busDiagRawRead( 0x02FC );
	for ( u32 byte = 0; byte < DISPLAY_SENTINEL_BITMAP_BYTES; byte++ )
		mismatch[ byte ] = 0;
	for ( u32 index = 0; index < DISPLAY_SENTINEL_LENGTH; index++ )
	{
		u8 expected = displayRowExpected( index, salt );
		if ( complement ) expected = (u8)~expected;
		if ( busDiagRawRead( displaySentinelAddr( index ) ) != expected )
			mismatch[ index >> 3 ] |= (u8)( 1u << ( index & 7 ) );
	}
	READ_CYCLE_COUNTER( finished );
	return (u32)( ( finished - started )
	              / ( SCPU_ARM_CLOCK_HZ / 1000000u ) );
}

static u32 displayPersistenceCapture( u8 salt, bool complement, u8 *mismatch,
	                                  u8 &firstExpected, u8 &firstActual,
	                                  u8 &xorAND, u8 &xorOR )
{
	u64 started, finished;
	READ_CYCLE_COUNTER( started );
	// Keep the same two-read priming as K225/K226 so only the number and content
	// of recorded passes changes between protocols.
	(void)busDiagRawRead( 0x02FC );
	(void)busDiagRawRead( 0x02FC );
	for ( u32 byte = 0; byte < DISPLAY_SENTINEL_BITMAP_BYTES; byte++ )
		mismatch[ byte ] = 0;
	firstExpected = firstActual = 0;
	xorAND = 0xFF;
	xorOR = 0;
	u32 errors = 0;
	for ( u32 index = 0; index < DISPLAY_SENTINEL_LENGTH; index++ )
	{
		u8 expected = displayRowExpected( index, salt );
		if ( complement ) expected = (u8)~expected;
		const u8 actual = busDiagRawRead( displaySentinelAddr( index ) );
		if ( actual == expected ) continue;
		mismatch[ index >> 3 ] |= (u8)( 1u << ( index & 7 ) );
		const u8 delta = (u8)( actual ^ expected );
		xorAND &= delta;
		xorOR |= delta;
		if ( errors++ == 0 )
		{
			firstExpected = expected;
			firstActual = actual;
		}
	}
	if ( errors == 0 ) xorAND = 0;
	READ_CYCLE_COUNTER( finished );
	return (u32)( ( finished - started )
	              / ( SCPU_ARM_CLOCK_HZ / 1000000u ) );
}

static void displayFetchSetState( u8 state, u8 border )
{
	static const u16 addrs[ 7 ] =
		{ 0xDD02, 0xDD00, 0xD015, 0xD016, 0xD018, 0xD020, 0xD021 };
	const u8 values[ 7 ] =
		{ 0x03,   0x03,   0x00,   0x08,   0x18,   border, 0x00 };
	for ( u32 i = 0; i < 7; i++ )
	{
		busDiagRawWrite( addrs[ i ], values[ i ] );
		busDiagRawWrite( addrs[ i ], values[ i ] );
	}
	const u8 d011 = state == 2 ? 0x3B : state == 1 ? 0x1B : 0x0B;
	busDiagRawWrite( 0xD011, d011 );
	busDiagRawWrite( 0xD011, d011 );
}

static void displayFetchSetDEN( u8 state )
{
	const u8 d011 = state == 2 ? 0x3B : state == 1 ? 0x1B : 0x0B;
	busDiagRawWrite( 0xD011, d011 );
	busDiagRawWrite( 0xD011, d011 );
}

static void displayFetchBlank( u8 state )
{
	const u8 d011 = state == 2 ? 0x2B : 0x0B;
	busDiagRawWrite( 0xD011, d011 );
	busDiagRawWrite( 0xD011, d011 );
}

static bool displayBoundaryRasterWindow( u8 raster, bool safe )
{
	// $F0-$FF is in the lower border on both NTSC and PAL. $80-$87 is well
	// inside the visible picture. We deliberately use broad windows rather
	// than one exact line so a slightly late raw sample cannot deadlock K229.
	return safe ? raster >= 0xF0
	            : raster >= 0x80 && raster <= 0x87;
}

static u32 displayBoundaryWaitRaster( bool safe )
{
	u64 now;
	READ_CYCLE_COUNTER( now );
	const u64 deadline = now + SCPU_ARM_CLOCK_HZ / 10u;
	u32 reads = 0;
	u8 raster;
	// If execution starts inside the requested window, leave it first. This
	// makes every arm synchronize to a fresh window rather than using a random
	// point near its trailing edge.
	do
	{
		raster = busDiagRawRead( 0xD012 );
		reads++;
		READ_CYCLE_COUNTER( now );
		if ( now >= deadline ) return reads | 0x80000000u;
	} while ( displayBoundaryRasterWindow( raster, safe ) );
	do
	{
		raster = busDiagRawRead( 0xD012 );
		reads++;
		READ_CYCLE_COUNTER( now );
		if ( now >= deadline ) return reads | 0x80000000u;
	} while ( !displayBoundaryRasterWindow( raster, safe ) );
	return reads;
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
	// A new acquisition always begins in the conventional host-KERNAL map.
	// The special all-RAM port state is installed only after ROM snapshotting
	// and the complete baseline bus self-test have succeeded.
	radSelectIOWithGame = false;
	m_RAMUnderIOReady = false;

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
			// Machine detection has just ended with physical reads. K243 rotated
			// the following calibration reads and proved that the first seed
			// write ($0334 <- $3C) was lost while every later write and read
			// passed. Represent that real read->write direction here so the
			// ordinary single-write primitive pays its existing one C64 idle
			// pass. The C128 path below already does the same with two passes.
			busWriteTurnaroundNeeded = 1;
			m_Acquired = true;
			calibrateReadTiming();
			calibrateWriteTiming();
			calibrateSIDReadTiming();
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
		calibrateSIDReadTiming();
		RADLOG( "    C128 refresh: loaded timing calibration complete" );
	}
	else
	{
		calibrateReadTiming();
		calibrateWriteTiming();
		calibrateSIDReadTiming();
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

static void summarizeReadTimingValues( const u8 *values, u32 count,
	                                   u8 &distinct, u8 &dominantValue,
	                                   u8 &dominantCount )
{
	distinct = dominantValue = dominantCount = 0;
	for ( u32 i = 0; i < count; i++ )
	{
		bool seen = false;
		for ( u32 j = 0; j < i; j++ )
			if ( values[ j ] == values[ i ] ) seen = true;
		if ( !seen ) distinct++;

		u8 matches = 0;
		for ( u32 j = 0; j < count; j++ )
			if ( values[ j ] == values[ i ] ) matches++;
		if ( matches > dominantCount )
		{
			dominantCount = matches;
			dominantValue = values[ i ];
		}
	}
}

void CRADBus::calibrateReadTiming()
{
	register u32 g2;
	static const u8 patterns[] = { 0x3C, 0xC3, 0x5A, 0xA5, 0x69, 0x96 };
	static const u16 firstSample = 300;
	static const u16 lastSample  = 620;
	static const u16 sampleStep  = 5;

	const u16 configured = busTiming.WAIT_CYCLE_READ;
	m_ReadTimingConfigured = configured;

	// These bytes are in the cassette buffer and the existing self-test uses
	// the same locations. Writes do not depend on the read sample we are about
	// to calibrate. The VIC border register adds a non-DRAM device to the test;
	// its low nibble is a stable read-back value on every VIC-II revision.
	// K243/K244 proved that the first post-acquisition write can be swallowed
	// even after the configured direction-settle pass: $0334 stayed $00 while
	// every subsequent write passed. Consume that one unreliable transaction
	// at a disposable RAM byte, exactly as the later self-test already does,
	// rather than duplicating an oracle or potentially side-effecting I/O write.
	RAD_SPOKE( 0x02FE, 0xA6 );
	for ( u32 i = 0; i < sizeof( patterns ); i++ )
		RAD_SPOKE( (u16)( 0x0334 + i ), patterns[ i ] );
	RAD_SPOKE( 0xD020, 0x06 );
	m_Writes += sizeof( patterns ) + 2;

	u16 runStart = 0, runEnd = 0;
	u16 bestStart = 0, bestEnd = 0;
	u32 bestRunSamples = 0;
	m_ReadTimingBestError = 0xFFFF;
	m_ReadTimingBestErrorSample = firstSample;
	m_ReadTimingRAMBestError = 0xFFFF;
	m_ReadTimingRAMBestSample = firstSample;
	m_ReadTimingRAMOnlyBestError = 0xFFFF;
	m_ReadTimingRAMOnlyBestSample = firstSample;
	for ( u32 i = 0; i < READ_TIMING_SCORE_COUNT; i++ )
	{
		m_ReadTimingErrors[ i ] = 0xFFFF;
		m_ReadTimingRAMErrors[ i ] = 0xFFFF;
		m_ReadTimingRAMOnlyErrors[ i ] = 0xFFFF;
		m_ReadTimingMixedVICErrors[ i ] = 0xFFFF;
		m_ReadTimingIsolatedVICErrors[ i ] = 0xFFFF;
		m_ReadTimingMixedVICDistinct[ i ] = 0;
		m_ReadTimingMixedVICDominantValue[ i ] = 0;
		m_ReadTimingMixedVICDominantCount[ i ] = 0;
		m_ReadTimingIsolatedVICDistinct[ i ] = 0;
		m_ReadTimingIsolatedVICDominantValue[ i ] = 0;
		m_ReadTimingIsolatedVICDominantCount[ i ] = 0;
		for ( u32 p = 0; p < sizeof( patterns ); p++ )
		{
			m_ReadTimingMixedRAMPositionErrors[ i ][ p ] = 0;
			m_ReadTimingMixedRAMAddressErrors[ i ][ p ] = 0;
		}
	}

	for ( u16 sample = firstSample; sample <= lastSample; sample += sampleStep )
	{
		busTiming.WAIT_CYCLE_READ = sample;
		busTiming.WAIT_CYCLE_READ2 = (u16)( sample + 20 );

		const u32 scoreIndex = ( sample - firstSample ) / sampleStep;
		u32 ramErrors = 0, mixedVICErrors = 0;
		for ( u32 r = 0; r < READ_TIMING_REPETITIONS; r++ )
		{
			// Two complete rotations across twelve repetitions distinguish a
			// first-position turnaround error from a weak physical address.
			for ( u32 position = 0; position < sizeof( patterns ); position++ )
			{
				const u32 addressIndex = ( r + position ) % sizeof( patterns );
				u8 v;
				RAD_SPEEK( (u16)( 0x0334 + addressIndex ), v );
				if ( v != patterns[ addressIndex ] )
				{
					ramErrors++;
					m_ReadTimingMixedRAMPositionErrors
						[ scoreIndex ][ position ]++;
					m_ReadTimingMixedRAMAddressErrors
						[ scoreIndex ][ addressIndex ]++;
				}
			}

			u8 border;
			RAD_SPEEK( 0xD020, border );
			m_ReadTimingMixedVICActual[ scoreIndex ][ r ] = border;
			if ( ( border & 0x0F ) != 0x06 ) mixedVICErrors++;
		}

		// The mixed loop changes from DRAM to VIC once per repetition. Prime with
		// an unscored VIC read, then measure a VIC-only chain at the same sample
		// point. This is observation only and cannot affect timing selection.
		u8 isolatedPrime;
		RAD_SPEEK( 0xD020, isolatedPrime );
		(void)isolatedPrime;
		u32 isolatedVICErrors = 0;
		for ( u32 r = 0; r < READ_TIMING_REPETITIONS; r++ )
		{
			u8 border;
			RAD_SPEEK( 0xD020, border );
			m_ReadTimingIsolatedVICActual[ scoreIndex ][ r ] = border;
			if ( ( border & 0x0F ) != 0x06 ) isolatedVICErrors++;
		}

		// Absorb the VIC-to-RAM transition in an unscored read, then measure a
		// RAM-only chain. This curve is observation-only: production selection
		// below deliberately continues to use the original mixed score.
		u8 ramOnlyPrime;
		RAD_SPEEK( 0x0334, ramOnlyPrime );
		(void)ramOnlyPrime;
		u32 ramOnlyErrors = 0;
		for ( u32 r = 0; r < READ_TIMING_REPETITIONS; r++ )
		{
			for ( u32 i = 0; i < sizeof( patterns ); i++ )
			{
				u8 v;
				RAD_SPEEK( (u16)( 0x0334 + i ), v );
				if ( v != patterns[ i ] ) ramOnlyErrors++;
			}
		}
		m_Reads += READ_TIMING_REPETITIONS * ( sizeof( patterns ) * 2 + 2 ) + 2;
		const u32 errors = ramErrors + mixedVICErrors;
		if ( scoreIndex < READ_TIMING_SCORE_COUNT )
		{
			m_ReadTimingErrors[ scoreIndex ] = (u16)errors;
			m_ReadTimingRAMErrors[ scoreIndex ] = (u16)ramErrors;
			m_ReadTimingRAMOnlyErrors[ scoreIndex ] = (u16)ramOnlyErrors;
			m_ReadTimingMixedVICErrors[ scoreIndex ] = (u16)mixedVICErrors;
			m_ReadTimingIsolatedVICErrors[ scoreIndex ] = (u16)isolatedVICErrors;
			summarizeReadTimingValues(
				m_ReadTimingMixedVICActual[ scoreIndex ], READ_TIMING_REPETITIONS,
				m_ReadTimingMixedVICDistinct[ scoreIndex ],
				m_ReadTimingMixedVICDominantValue[ scoreIndex ],
				m_ReadTimingMixedVICDominantCount[ scoreIndex ] );
			summarizeReadTimingValues(
				m_ReadTimingIsolatedVICActual[ scoreIndex ], READ_TIMING_REPETITIONS,
				m_ReadTimingIsolatedVICDistinct[ scoreIndex ],
				m_ReadTimingIsolatedVICDominantValue[ scoreIndex ],
				m_ReadTimingIsolatedVICDominantCount[ scoreIndex ] );
		}
		if ( errors < m_ReadTimingBestError )
		{
			m_ReadTimingBestError = (u16)errors;
			m_ReadTimingBestErrorSample = sample;
		}
		if ( ramErrors < m_ReadTimingRAMBestError )
		{
			m_ReadTimingRAMBestError = (u16)ramErrors;
			m_ReadTimingRAMBestSample = sample;
		}
		if ( ramOnlyErrors < m_ReadTimingRAMOnlyBestError )
		{
			m_ReadTimingRAMOnlyBestError = (u16)ramOnlyErrors;
			m_ReadTimingRAMOnlyBestSample = sample;
		}

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

	const u32 ramBestIndex =
		( m_ReadTimingRAMBestSample - firstSample ) / sampleStep;
	RADLOG( "    read observe: mixed RAM best %u/%u, RAM-only %u/%u, mixed VIC %u, isolated VIC %u",
	        (unsigned)m_ReadTimingRAMBestSample,
	        (unsigned)m_ReadTimingRAMBestError,
	        (unsigned)m_ReadTimingRAMOnlyBestSample,
	        (unsigned)m_ReadTimingRAMOnlyBestError,
	        (unsigned)m_ReadTimingMixedVICErrors[ ramBestIndex ],
	        (unsigned)m_ReadTimingIsolatedVICErrors[ ramBestIndex ] );

	// Return the border to the normal C64 power-on colour. The boot ROM will
	// shortly initialise the VIC itself, but leaving diagnostics unobtrusive is
	// useful when acquisition later fails for an unrelated reason.
	RAD_SPOKE( 0xD020, 0x0E );
	m_Writes++;
}

void CRADBus::calibrateReadTimingUnderLoad()
{
	// Preserve the acquisition-time curve before calibrateReadTiming() reuses
	// its working arrays. That first pass ran while core 1 was still parked;
	// this pass runs only after the HDMI renderer has completed a whole frame.
	m_QuietReadTimingConfigured = m_ReadTimingConfigured;
	m_QuietReadTimingStart = m_ReadTimingStart;
	m_QuietReadTimingEnd = m_ReadTimingEnd;
	m_QuietReadTimingSelected = m_ReadTimingSelected;
	m_QuietReadTimingBestError = m_ReadTimingBestError;
	m_QuietReadTimingBestErrorSample = m_ReadTimingBestErrorSample;
	for ( u32 i = 0; i < READ_TIMING_SCORE_COUNT; i++ )
		m_QuietReadTimingErrors[ i ] = m_ReadTimingErrors[ i ];

	calibrateReadTiming();
	m_LoadedReadTimingRan = true;
	RADLOG( "    read timing core1 A/B: quiet=%u..%u/%u loaded=%u..%u/%u",
	        (unsigned)m_QuietReadTimingStart,
	        (unsigned)m_QuietReadTimingEnd,
	        (unsigned)m_QuietReadTimingSelected,
	        (unsigned)m_ReadTimingStart,
	        (unsigned)m_ReadTimingEnd,
	        (unsigned)m_ReadTimingSelected );
}

bool CRADBus::verifyC64CIA2DDRA( u8 expected )
{
	// A successful ordinary-read calibration gives us an eye rather than one
	// lucky point. Verify the CIA at its start, selected midpoint and end. If no
	// eye was established, leave the higher-level shadow unknown: accepting one
	// configured-point read would recreate the residue/RMW failure this guard is
	// intended to remove.
	if ( m_ReadTimingStart == 0 || m_ReadTimingEnd < m_ReadTimingStart )
	{
		RADLOG( "    CIA2 DDRA seed: NOT VERIFIED (no calibrated read window)" );
		return false;
	}

	u16 points[ 3 ];
	u32 pointCount = 0;
	const u16 candidates[ 3 ] = {
		m_ReadTimingStart, m_ReadTimingSelected, m_ReadTimingEnd
	};
	for ( u32 i = 0; i < 3; i++ )
	{
		bool duplicate = false;
		for ( u32 j = 0; j < pointCount; j++ )
			if ( points[ j ] == candidates[ i ] ) duplicate = true;
		if ( !duplicate ) points[ pointCount++ ] = candidates[ i ];
	}
	if ( pointCount < 2 )
	{
		RADLOG( "    CIA2 DDRA seed: NOT VERIFIED (read window has only one sample point)" );
		return false;
	}

	const u16 savedRead = (u16)busTiming.WAIT_CYCLE_READ;
	const u16 savedRead2 = (u16)busTiming.WAIT_CYCLE_READ2;
	u8 observed[ 3 ][ 3 ];
	bool verified = true;
	for ( u32 p = 0; p < pointCount; p++ )
	{
		busTiming.WAIT_CYCLE_READ = points[ p ];
		busTiming.WAIT_CYCLE_READ2 = (u16)( points[ p ] + 20 );

		// The first physical reads after write->read turnaround are known to be
		// unreliable on buffered C128 expansion ports. Discard two complete
		// transfers at every timing group before any sample can vote.
		for ( u32 prime = 0; prime < 2; prime++ )
		{
			u8 ignored;
			RAD_SPEEK( 0xDD02, ignored );
		}
		for ( u32 sample = 0; sample < 3; sample++ )
		{
			RAD_SPEEK( 0xDD02, observed[ p ][ sample ] );
			if ( observed[ p ][ sample ] != expected ) verified = false;
		}
	}
	m_Reads += pointCount * 5;
	busTiming.WAIT_CYCLE_READ = savedRead;
	busTiming.WAIT_CYCLE_READ2 = savedRead2;

	for ( u32 p = 0; p < pointCount; p++ )
		RADLOG( "    CIA2 DDRA @%u: %02X %02X %02X",
		        (unsigned)points[ p ], (unsigned)observed[ p ][ 0 ],
		        (unsigned)observed[ p ][ 1 ], (unsigned)observed[ p ][ 2 ] );
	RADLOG( "    CIA2 DDRA seed $%02X: %s",
	        (unsigned)expected, verified ? "VERIFIED" : "NOT VERIFIED; shadow left unknown" );
	return verified;
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

void CRADBus::calibrateSIDReadTiming()
{
	static const u16 firstSample = 300;
	static const u16 lastSample  = 675;
	static const u16 sampleStep  = 5;
	static const u32 samplesPerPoint = 64;
	static const u32 minimumDistinct = 12;
	static const u32 maximumDominant = 24;
	static const u32 minimumRampSteps = 40;

	const u16 configured = (u16)busTiming.WAIT_CYCLE_READ_SID;
	m_SIDTimingConfigured = configured;
	m_SIDTimingStart = m_SIDTimingEnd = 0;
	m_SIDTimingSelected = configured;
	m_SIDTimingDistinct = m_SIDTimingDominant = 0;
	m_SIDTimingRamp = 0;
	m_SIDTimingEdge = 0;
	m_SIDPhysicalReliable = false;
	sidReadAtFallingEdge = false;

	// Reset voice 3's oscillator, select the maximum frequency, then release
	// it into the sawtooth waveform. OSC3 ($D41B) is the only useful physical SID
	// read oracle: a valid sample produces many values, while a missed sample
	// on the expansion bus is overwhelmingly one open-bus value (typically FC).
	// This is the same oscillator sequence used by upstream RAD detectSID().
	// Noise is unsuitable here: its LFSR clocks from oscillator bit 19 and thus
	// changes only every several reads even at $FFFF, causing a valid window to
	// fail an entropy test.
	RAD_SPOKE( 0xD412, 0xFF );
	RAD_SPOKE( 0xD40E, 0xFF );
	RAD_SPOKE( 0xD40F, 0xFF );
	RAD_SPOKE( 0xD412, 0x20 );
	m_Writes += 4;

	// Give the LFSR time to advance after TEST is released. These discarded
	// reads also pay the initial write-to-read bus turnaround before scoring.
	busTiming.WAIT_CYCLE_READ_SID = configured;
	for ( u32 i = 0; i < 32; i++ )
	{
		u8 ignored;
		RAD_SPEEK( 0xD41B, ignored );
	}
	m_Reads += 32;

	u16 runStart = 0, runEnd = 0;
	u16 bestStart = 0, bestEnd = 0;
	u32 bestRunSamples = 0;
	u16 bestObservedSample = firstSample;
	u32 bestObservedDistinct = 0, bestObservedDominant = samplesPerPoint;
	u32 bestObservedRamp = 0;

	for ( u16 sample = firstSample; sample <= lastSample; sample += sampleStep )
	{
		busTiming.WAIT_CYCLE_READ_SID = sample;
		u32 distinct = 255, dominant = 0, ramp = 255;
		for ( u32 pass = 0; pass < 2; pass++ )
		{
			u8 counts[ 256 ];
			for ( u32 i = 0; i < sizeof counts; i++ ) counts[ i ] = 0;
			u32 blockDistinct = 0, blockDominant = 0, blockRamp = 0;
			u8 previous = 0;
			for ( u32 i = 0; i < samplesPerPoint; i++ )
			{
				u8 value;
				RAD_SPEEK( 0xD41B, value );
				if ( counts[ value ]++ == 0 ) blockDistinct++;
				if ( counts[ value ] > blockDominant )
					blockDominant = counts[ value ];
				if ( i )
				{
					const u8 delta = (u8)( value - previous );
					if ( delta >= 1 && delta <= 16 ) blockRamp++;
				}
				previous = value;
			}
			if ( blockDistinct < distinct ) distinct = blockDistinct;
			if ( blockDominant > dominant ) dominant = blockDominant;
			if ( blockRamp < ramp ) ramp = blockRamp;
		}
		m_Reads += samplesPerPoint * 2;
		if ( ramp > bestObservedRamp
		     || ( ramp == bestObservedRamp && distinct > bestObservedDistinct )
		     || ( ramp == bestObservedRamp && distinct == bestObservedDistinct
		          && dominant < bestObservedDominant ) )
		{
			bestObservedSample = sample;
			bestObservedDistinct = distinct;
			bestObservedDominant = dominant;
			bestObservedRamp = ramp;
		}

		const bool valid = distinct >= minimumDistinct
		                && dominant <= maximumDominant
		                && ramp >= minimumRampSteps;
		if ( valid )
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
		const u16 chosen = (u16)( bestStart
		                 + ( ( ( bestEnd - bestStart ) / sampleStep ) / 2 ) * sampleStep );
		busTiming.WAIT_CYCLE_READ_SID = chosen;
		m_SIDTimingStart = bestStart;
		m_SIDTimingEnd = bestEnd;
		m_SIDTimingSelected = chosen;
		m_SIDTimingBestSample = chosen;
		m_SIDPhysicalReliable = true;

		// Record a fresh score at the selected midpoint for the persistent log.
		u8 counts[ 256 ];
		for ( u32 i = 0; i < sizeof counts; i++ ) counts[ i ] = 0;
		u8 previous = 0;
		for ( u32 i = 0; i < samplesPerPoint; i++ )
		{
			u8 value;
			RAD_SPEEK( 0xD41B, value );
			if ( counts[ value ]++ == 0 ) m_SIDTimingDistinct++;
			if ( counts[ value ] > m_SIDTimingDominant )
				m_SIDTimingDominant = counts[ value ];
			if ( i )
			{
				const u8 delta = (u8)( value - previous );
				if ( delta >= 1 && delta <= 16 ) m_SIDTimingRamp++;
			}
			previous = value;
		}
		m_Reads += samplesPerPoint;
		RADLOG( "    SID timing: configured %u, saw window %u..%u, selected %u, distinct/dominant/ramp=%u/%u/%u",
		        (unsigned)configured, (unsigned)bestStart, (unsigned)bestEnd,
		        (unsigned)chosen, (unsigned)m_SIDTimingDistinct,
		        (unsigned)m_SIDTimingDominant, (unsigned)m_SIDTimingRamp );
	}
	else
	{
		// Numeric offsets all missed. Validate the signal at the event where a
		// physical 6510 samples it: PHI2's falling edge. This is selected only
		// when the same entropy/dominance oracle passes there.
		sidReadAtFallingEdge = true;
		u32 edgeDistinct = 255, edgeDominant = 0, edgeRamp = 255;
		bool edgeValid = true;
		for ( u32 pass = 0; pass < 2; pass++ )
		{
			u8 counts[ 256 ];
			for ( u32 i = 0; i < sizeof counts; i++ ) counts[ i ] = 0;
			u32 distinct = 0, dominant = 0, ramp = 0;
			u8 previous = 0;
			for ( u32 i = 0; i < samplesPerPoint; i++ )
			{
				u8 value;
				RAD_SPEEK( 0xD41B, value );
				if ( counts[ value ]++ == 0 ) distinct++;
				if ( counts[ value ] > dominant ) dominant = counts[ value ];
				if ( i )
				{
					const u8 delta = (u8)( value - previous );
					if ( delta >= 1 && delta <= 16 ) ramp++;
				}
				previous = value;
			}
			m_Reads += samplesPerPoint;
			if ( distinct < edgeDistinct ) edgeDistinct = distinct;
			if ( dominant > edgeDominant ) edgeDominant = dominant;
			if ( ramp < edgeRamp ) edgeRamp = ramp;
			if ( distinct < minimumDistinct || dominant > maximumDominant
			     || ramp < minimumRampSteps ) edgeValid = false;
		}
		if ( edgeValid )
		{
			m_SIDTimingEdge = 1;
			m_SIDPhysicalReliable = true;
			m_SIDTimingBestSample = 0;
			m_SIDTimingDistinct = (u8)edgeDistinct;
			m_SIDTimingDominant = (u8)edgeDominant;
			m_SIDTimingRamp = (u8)edgeRamp;
			RADLOG( "    SID timing: numeric scan missed; selected last PHI2-high sample distinct/dominant/ramp=%u/%u/%u",
			        (unsigned)edgeDistinct, (unsigned)edgeDominant,
			        (unsigned)edgeRamp );
		}
		else
		{
			sidReadAtFallingEdge = false;
			busTiming.WAIT_CYCLE_READ_SID = configured;
			m_SIDTimingBestSample = bestObservedSample;
			m_SIDTimingDistinct = (u8)bestObservedDistinct;
			m_SIDTimingDominant = (u8)bestObservedDominant;
			m_SIDTimingRamp = (u8)bestObservedRamp;
			RADLOG( "    SID timing: no numeric or edge saw window; retaining %u; best %u distinct/dominant/ramp=%u/%u/%u edge=%u/%u/%u",
			        (unsigned)configured, (unsigned)bestObservedSample,
			        (unsigned)bestObservedDistinct, (unsigned)bestObservedDominant,
			        (unsigned)bestObservedRamp, (unsigned)edgeDistinct,
			        (unsigned)edgeDominant, (unsigned)edgeRamp );
		}
	}

	// Leave the physical SID quiet for the boot ROM, which will initialise it
	// normally. No readable register state existed to preserve at takeover.
	RAD_SPOKE( 0xD412, 0x08 );
	RAD_SPOKE( 0xD40E, 0x00 );
	RAD_SPOKE( 0xD40F, 0x00 );
	RAD_SPOKE( 0xD412, 0x00 );
	m_Writes += 4;
}

void CRADBus::sidModelAdvance()
{
	const u64 now = hostCycles();
	if ( m_SIDModelLastHost == 0 )
	{
		m_SIDModelLastHost = now;
		return;
	}

	const u64 elapsed = now - m_SIDModelLastHost;
	m_SIDModelLastHost = now;
	// The oscillator is clocked by the machine's PHI2, independently of the
	// accelerated CPU. Keep the fractional conversion in ARM-Hz units so a
	// tight polling loop neither freezes nor artificially races the model.
	const u32 sidHz = m_Signals.video == VIDEO_PAL ? 985248u : 1022727u;
	const u64 scaled = elapsed * sidHz + m_SIDModelRemainder;
	const u64 sidClocks = scaled / SCPU_ARM_CLOCK_HZ;
	m_SIDModelRemainder = scaled % SCPU_ARM_CLOCK_HZ;
	if ( !( m_SIDModelControl & 0x08 ) )
		m_SIDModelPhase = (u32)( ( m_SIDModelPhase
		                   + sidClocks * m_SIDModelFreq ) & 0xFFFFFFu );
}

void CRADBus::sidObserveWrite( u16 addr, u8 value )
{
	if ( addr != 0xD40E && addr != 0xD40F && addr != 0xD412 )
		return;

	sidModelAdvance();
	if ( addr == 0xD40E )
		m_SIDModelFreq = (u16)( ( m_SIDModelFreq & 0xFF00 ) | value );
	else if ( addr == 0xD40F )
		m_SIDModelFreq = (u16)( ( m_SIDModelFreq & 0x00FF )
		                           | ( (u16)value << 8 ) );
	else
	{
		m_SIDModelControl = value;
		if ( value & 0x08 ) m_SIDModelPhase = 0;
	}
}

u8 CRADBus::sidReadOSC3Model()
{
	sidModelAdvance();
	if ( ( m_SIDModelControl & 0x08 ) || !( m_SIDModelControl & 0xF0 ) )
		return 0;
	// A real SID applies the selected waveform (and has revision-dependent
	// combined-waveform behaviour). The fallback's contract is narrower: keep
	// the voice-3 oscillator free-running and deterministic when the expansion
	// bus cannot sample the real register. Returning the accumulator's top byte
	// preserves phase/frequency and visits every value, which is what software
	// such as Metal Dust uses OSC3 for.
	return (u8)( m_SIDModelPhase >> 16 );
}

u8 CRADBus::sidReadPOTFiltered( u16 addr )
{
	static const u32 samples = 17;
	u8 counts[ 256 ];
	for ( u32 i = 0; i < sizeof counts; i++ ) counts[ i ] = 0;

	for ( u32 i = 0; i < samples; i++ )
	{
		u8 value;
		RAD_SPEEK( addr, value );
		counts[ value ]++;
	}
	// CRADBus::read accounts for the logical access; retain the physical bus
	// statistic for the sixteen additional observations made by the filter.
	m_Reads += samples - 1;

	const u32 pot = addr - 0xD419;
	const u8 previous = m_SIDPotFiltered[ pot ];
	u32 bestCount = 0;
	u8 best = previous;
	for ( u32 value = 0; value < 256; value++ )
	{
		if ( counts[ value ] > bestCount )
		{
			bestCount = counts[ value ];
			best = (u8)value;
		}
		else if ( counts[ value ] == bestCount && bestCount != 0 )
		{
			// At a near-even valid/residue split, prefer continuity. The 1351's
			// POT value moves gradually; open-bus values do not.
			const u32 oldDistance = best > previous ? best - previous
			                                      : previous - best;
			const u32 newDistance = value > previous ? value - previous
			                                       : previous - value;
			if ( newDistance < oldDistance ) best = (u8)value;
		}
	}

	// Fewer than three repeats means there was no stable POT value in this
	// group. Hold the last coordinate rather than converting transition noise
	// into a violent mouse jump.
	if ( bestCount >= 3 ) m_SIDPotFiltered[ pot ] = best;
	return m_SIDPotFiltered[ pot ];
}

bool CRADBus::prepareRAMUnderIOAccess()
{
	m_RAMUnderIOFailure = RAMUNDERIO_OK;
	if ( !m_Acquired || m_TrafficHalted
	     || m_Signals.machine != MACHINE_C64 )
	{
		m_RAMUnderIOFailure = RAMUNDERIO_PRECONDITION;
		return false;
	}

	// The acquisition path deliberately leaves the physical host in its normal
	// $01=$37/$35 map until BASIC and KERNAL have been snapshotted and the bus
	// self-test has exercised those ROM windows. Now reset through the same
	// bounded Ultimax injector once more, but leave the stopped 6510 at $34:
	// CHAREN/HIRAM/LORAM=100. With /GAME released the PLA exposes all RAM;
	// asserting /GAME selects Ultimax, whose $D000-$DFFF window is real I/O.
	radSelectIOWithGame = false;
	m_RAMUnderIOReady = false;
	radPrepareUltimaxTakeover( 0x34 );
	if ( !radHijackCPUWithUltimax() )
	{
		// radHijackCPUWithUltimax() already released /DMA and reset the machine
		// normally on failure. Reflect that ownership change so a later release()
		// cannot drive a bus we no longer own.
		m_Acquired = false;
		m_RAMUnderIOFailure = RAMUNDERIO_TAKEOVER;
		RADLOG( "RAM-under-I/O capability: takeover failed" );
		return false;
	}

	// Success returns with /GAME asserted and /DMA held. Release /GAME to select
	// physical RAM, then prove both halves of the mapping before enabling any
	// queued under-I/O write. The stub blanked D011/D020/D021 through Ultimax,
	// so the chip reads must differ from the deliberately chosen RAM sentinels.
	SET_GPIO( bGAME_OUT );
	busWriteTurnaroundNeeded = 1;
	static const u16 probeAddr[ 3 ] = { 0xD011, 0xD020, 0xD021 };
	static const u8  probeData[ 3 ] = { 0x5A, 0xA5, 0x3C };
	bool ramOK = true;
	for ( u32 i = 0; i < 3; i++ )
	{
		radDirectWrite( probeAddr[ i ], probeData[ i ], true );
		if ( radDirectRead( probeAddr[ i ], true ) != probeData[ i ] )
			ramOK = false;
	}

	// Arm the selector only after the all-RAM side has verified. Each following
	// read asserts /GAME inside the timed primitive and releases it exactly at
	// the CPU-to-VIC falling edge.
	radSelectIOWithGame = true;
	bool ioOK = false;
	for ( u32 i = 0; i < 3; i++ )
		if ( radDirectRead( probeAddr[ i ] ) != probeData[ i ] ) ioOK = true;

	if ( !ramOK || !ioOK )
	{
		m_RAMUnderIOFailure = ( ramOK ? 0 : RAMUNDERIO_RAM_MAP )
		                        | ( ioOK ? 0 : RAMUNDERIO_IO_MAP );
		RADLOG( "RAM-under-I/O capability: RAM map=%s I/O select=%s",
		        ramOK ? "pass" : "FAIL", ioOK ? "pass" : "FAIL" );
		radSelectIOWithGame = false;
		SET_GPIO( bGAME_OUT );
		// Do not let the physical CPU execute even one instruction with $01=$34.
		// Reset it while /DMA is held, then release it into the normal reset path.
		radResetMachine();
		radReleaseCPU();
		m_Acquired = false;
		return false;
	}

	m_RAMUnderIOReady = true;
	m_RAMUnderIOFailure = RAMUNDERIO_OK;
	m_LastTransferCycles = hostCycles();
	return true;
}

void CRADBus::release()
{
	if ( !m_Acquired )
		return;

	// The helper is the only owner allowed to release /DMA during normal C128
	// operation. Stop it and wait for /DMA-low acknowledgement before handing
	// the physical CPU back to the machine.
	c128RefreshStop();
	const bool restoreHost = m_RAMUnderIOReady;
	m_RAMUnderIOReady = false;
	radSelectIOWithGame = false;
	// $34 is intentionally useful only while the accelerator owns the bus. If
	// the Pi is rebooting or startup aborts, give the physical CPU a clean reset
	// so its KERNAL restores the conventional $2F/$37 processor-port state.
	// Assert/reset it while /DMA is still held; otherwise it can execute briefly
	// from all-RAM $E000 before RESET reaches the pin.
	if ( restoreHost ) radResetMachine();
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

bool CRADBus::runAccessSentinelDiagnostic()
{
	if ( !m_Acquired || m_TrafficHalted
	     || m_Signals.machine != MACHINE_C64 )
		return false;

	m_AccessSentinelRan = true;
	m_AccessSentinelTrafficCount = 1u << 18;
	static const u32 trafficRatePerSecond = 16000;
	const u64 trafficInterval = SCPU_ARM_CLOCK_HZ / trafficRatePerSecond;

	// Make completion/progress visible without touching the sentinel range.
	// Every write arm then repeats this exact value, so its intentional effect
	// is a no-op at the VIC register.
	busDiagRawWrite( 0xD020, 0x06 );
	busDiagRawWrite( 0xD020, 0x06 );
	m_Writes += 2;

	for ( u32 arm = 0; arm < 3; arm++ )
	{
		const u8 salt = (u8)( 0x35u + arm * 0x49u );
		accessSentinelSeed( salt );
		m_Writes += ACCESS_SENTINEL_LENGTH;

		u16 ignoredAddr;
		u8 ignoredExpected, ignoredActual;
		m_AccessSentinelBaselineErrors[ arm ][ 0 ] =
			accessSentinelCapture( salt, accessSentinelMismatch[ 0 ],
			                       ignoredAddr, ignoredExpected, ignoredActual );
		m_AccessSentinelBaselineErrors[ arm ][ 1 ] =
			accessSentinelCapture( salt, accessSentinelMismatch[ 1 ],
			                       ignoredAddr, ignoredExpected, ignoredActual );

		const u64 started = hostCycles();
		u64 deadline = started + trafficInterval;
		for ( u32 access = 0; access < m_AccessSentinelTrafficCount; access++ )
		{
			while ( hostCycles() < deadline ) asm volatile( "nop" );
			if ( arm == 1 )
				(void)busDiagRawRead( 0xD020 );
			else if ( arm == 2 )
				busDiagRawWrite( 0xD020, 0x06 );
			deadline += trafficInterval;
		}
		m_AccessSentinelElapsedUsec[ arm ] =
			(u32)( ( hostCycles() - started )
			       / ( SCPU_ARM_CLOCK_HZ / 1000000u ) );

		m_AccessSentinelExposureErrors[ arm ][ 0 ] =
			accessSentinelCapture(
				salt, accessSentinelMismatch[ 0 ],
				m_AccessSentinelFirstAddr[ arm ][ 0 ],
				m_AccessSentinelFirstExpected[ arm ][ 0 ],
				m_AccessSentinelFirstActual[ arm ][ 0 ] );

		accessSentinelCompare(
			accessSentinelMismatch[ 1 ], accessSentinelMismatch[ 0 ],
			m_AccessSentinelBaselineRetainedErrors[ arm ],
			m_AccessSentinelArmAddedErrors[ arm ],
			m_AccessSentinelArmRemovedErrors[ arm ] );

		m_AccessSentinelExposureErrors[ arm ][ 1 ] =
			accessSentinelCapture(
				salt, accessSentinelMismatch[ 1 ],
				m_AccessSentinelFirstAddr[ arm ][ 1 ],
				m_AccessSentinelFirstExpected[ arm ][ 1 ],
				m_AccessSentinelFirstActual[ arm ][ 1 ] );
		accessSentinelCompare(
			accessSentinelMismatch[ 0 ], accessSentinelMismatch[ 1 ],
			m_AccessSentinelSameErrors[ arm ],
			m_AccessSentinelNewErrors[ arm ],
			m_AccessSentinelClearedErrors[ arm ] );

		m_Reads += ACCESS_SENTINEL_LENGTH * 4u + 8u;
		if ( arm == 1 ) m_Reads += m_AccessSentinelTrafficCount;
		if ( arm == 2 ) m_Writes += m_AccessSentinelTrafficCount;
	}
	return true;
}

bool CRADBus::runDisplaySentinelDiagnostic()
{
	if ( !m_Acquired || m_TrafficHalted
	     || m_Signals.machine != MACHINE_C64 )
		return false;

	m_DisplaySentinelRan = true;
	static const u8 displayOn[ 9 ] = { 1, 1, 1, 1, 1, 1, 1, 0, 0 };
	static const u8 operation[ 9 ] = { 0, 1, 1, 1, 2, 2, 2, 1, 2 };
	static const u32 rate[ 9 ] =
		{ 0, 4000, 16000, 64000, 4000, 16000, 64000, 64000, 64000 };
	static const u8 border[ 9 ] = { 1, 5, 7, 2, 3, 4, 8, 13, 10 };
	static const u32 exposureSeconds = 8;

	for ( u32 arm = 0; arm < 9; arm++ )
	{
		m_DisplaySentinelDisplayOn[ arm ] = displayOn[ arm ];
		m_DisplaySentinelOperation[ arm ] = operation[ arm ];
		m_DisplaySentinelRate[ arm ] = rate[ arm ];
		m_DisplaySentinelTrafficCount[ arm ] = rate[ arm ] * exposureSeconds;

		// Blanked setup is repeated for every arm, so no state from the previous
		// exposure can leak into the next baseline.
		displaySentinelSetMode( false, 0 );
		displaySentinelSeed();
		m_Writes += 16u + DISPLAY_SENTINEL_LENGTH;

		u16 ignoredAddr;
		u8 ignoredExpected, ignoredActual;
		m_DisplaySentinelBaselineErrors[ arm ][ 0 ] =
			displaySentinelCapture( displaySentinelMismatch[ 0 ],
			                        ignoredAddr, ignoredExpected, ignoredActual );
		m_DisplaySentinelBaselineErrors[ arm ][ 1 ] =
			displaySentinelCapture( displaySentinelMismatch[ 1 ],
			                        ignoredAddr, ignoredExpected, ignoredActual );

		// A distinct border identifies every exposure in a video. D011 is last:
		// for display-on arms, the bitmap becomes fetch-active only after all
		// other setup traffic has completed.
		busDiagRawWrite( 0xD020, border[ arm ] );
		busDiagRawWrite( 0xD020, border[ arm ] );
		displaySentinelSetDEN( displayOn[ arm ] != 0 );
		m_Writes += 4;

		const u64 started = hostCycles();
		if ( rate[ arm ] == 0 )
		{
			const u64 stop = started
			               + (u64)exposureSeconds * SCPU_ARM_CLOCK_HZ;
			while ( hostCycles() < stop ) asm volatile( "nop" );
		}
		else
		{
			const u64 interval = SCPU_ARM_CLOCK_HZ / rate[ arm ];
			u64 deadline = started + interval;
			for ( u32 access = 0;
			      access < m_DisplaySentinelTrafficCount[ arm ]; access++ )
			{
				while ( hostCycles() < deadline ) asm volatile( "nop" );
				if ( operation[ arm ] == 1 )
					(void)busDiagRawRead( 0xD020 );
				else
					busDiagRawWrite( 0xD020, border[ arm ] );
				deadline += interval;
			}
		}
		m_DisplaySentinelElapsedUsec[ arm ] =
			(u32)( ( hostCycles() - started )
			       / ( SCPU_ARM_CLOCK_HZ / 1000000u ) );

		// Blank before the first oracle read. The two idempotent D011 writes are
		// counted as part of the exposure protocol and are identical in every arm.
		displaySentinelSetDEN( false );
		m_Writes += 2;
		m_DisplaySentinelExposureErrors[ arm ][ 0 ] =
			displaySentinelCapture(
				displaySentinelMismatch[ 0 ],
				m_DisplaySentinelFirstAddr[ arm ][ 0 ],
				m_DisplaySentinelFirstExpected[ arm ][ 0 ],
				m_DisplaySentinelFirstActual[ arm ][ 0 ] );
		displaySentinelCompare(
			displaySentinelMismatch[ 1 ], displaySentinelMismatch[ 0 ],
			m_DisplaySentinelBaselineRetainedErrors[ arm ],
			m_DisplaySentinelArmAddedErrors[ arm ],
			m_DisplaySentinelArmRemovedErrors[ arm ] );

		m_DisplaySentinelExposureErrors[ arm ][ 1 ] =
			displaySentinelCapture(
				displaySentinelMismatch[ 1 ],
				m_DisplaySentinelFirstAddr[ arm ][ 1 ],
				m_DisplaySentinelFirstExpected[ arm ][ 1 ],
				m_DisplaySentinelFirstActual[ arm ][ 1 ] );
		displaySentinelCompare(
			displaySentinelMismatch[ 0 ], displaySentinelMismatch[ 1 ],
			m_DisplaySentinelVerifySameErrors[ arm ][ 0 ],
			m_DisplaySentinelVerifyNewErrors[ arm ][ 0 ],
			m_DisplaySentinelVerifyClearedErrors[ arm ][ 0 ] );

		m_DisplaySentinelExposureErrors[ arm ][ 2 ] =
			displaySentinelCapture(
				displaySentinelMismatch[ 0 ],
				m_DisplaySentinelFirstAddr[ arm ][ 2 ],
				m_DisplaySentinelFirstExpected[ arm ][ 2 ],
				m_DisplaySentinelFirstActual[ arm ][ 2 ] );
		displaySentinelCompare(
			displaySentinelMismatch[ 1 ], displaySentinelMismatch[ 0 ],
			m_DisplaySentinelVerifySameErrors[ arm ][ 1 ],
			m_DisplaySentinelVerifyNewErrors[ arm ][ 1 ],
			m_DisplaySentinelVerifyClearedErrors[ arm ][ 1 ] );

		m_Reads += DISPLAY_SENTINEL_LENGTH * 5u + 10u;
		if ( operation[ arm ] == 1 )
			m_Reads += m_DisplaySentinelTrafficCount[ arm ];
		else if ( operation[ arm ] == 2 )
			m_Writes += m_DisplaySentinelTrafficCount[ arm ];
	}
	return true;
}

bool CRADBus::runDisplayAddressDiagnostic()
{
	if ( !m_Acquired || m_TrafficHalted
	     || m_Signals.machine != MACHINE_C64 )
		return false;

	m_DisplayAddressRan = true;
	static const u16 target = 0x2078;
	static const u32 immediateTrials = 256;

	// First answer Claude's decisive question at the exact K222 address. Each
	// iteration writes a changing value and records the first three physical
	// target reads. Read 0 is intentionally the immediate next transaction;
	// reads 1 and 2 reveal any turnaround depth rather than hiding it behind a
	// sacrificial read policy. One- and two-write styles are kept separate.
	for ( u32 mode = 0; mode < 2; mode++ )
	{
		displayAddressSetMode( mode != 0, mode ? 7 : 5 );
		m_Writes += 16;
		for ( u32 style = 0; style < 2; style++ )
		{
			for ( u32 trial = 0; trial < immediateTrials; trial++ )
			{
				const u8 expected =
					(u8)( trial * 73u + mode * 0x31u + style * 0x57u + 0x35u );
				busDiagRawWrite( target, expected );
				if ( style != 0 ) busDiagRawWrite( target, expected );
				u8 actual[ 3 ];
				for ( u32 read = 0; read < 3; read++ )
				{
					actual[ read ] = busDiagRawRead( target );
					if ( actual[ read ] != expected )
						m_DisplayAddressImmediateErrors[ mode ][ style ][ read ]++;
				}
				if ( actual[ 2 ] != expected
				     && m_DisplayAddressImmediateFirstTrial[ mode ][ style ] == 0 )
				{
					// Store trial+1 so zero remains the unambiguous "no failure" marker.
					m_DisplayAddressImmediateFirstTrial[ mode ][ style ] =
						(u16)( trial + 1u );
					m_DisplayAddressImmediateFirstExpected[ mode ][ style ] = expected;
					for ( u32 read = 0; read < 3; read++ )
						m_DisplayAddressImmediateFirstActual[ mode ][ style ][ read ] =
							actual[ read ];
				}
			}
			m_Writes += immediateTrials * ( style ? 2u : 1u );
			m_Reads += immediateTrials * 3u;
		}
	}

	// Reproduce K222's bulk seed/capture under both D011 modes. The repeated
	// seed is the direct A/B for a swallowed or marginal write. Preserve every
	// mismatch bit plus fixed-address and run summaries.
	for ( u32 mode = 0; mode < 2; mode++ )
	{
		for ( u32 style = 0; style < 2; style++ )
		{
			displayAddressSetMode( mode != 0, mode ? 4 : 3 );
			displayAddressSeed( style ? 2u : 1u );
			u16 ignoredAddr;
			u8 ignoredExpected, ignoredActual;
			(void)displaySentinelCapture(
				displayAddressMismatch[ mode ][ style ],
				ignoredAddr, ignoredExpected, ignoredActual );
			displayAddressAnalyzeMap(
				displayAddressMismatch[ mode ][ style ],
				m_DisplayAddressMapErrors[ mode ][ style ],
				m_DisplayAddressMapFirst[ mode ][ style ],
				m_DisplayAddressMapLast[ mode ][ style ],
				m_DisplayAddressMapAND[ mode ][ style ],
				m_DisplayAddressMapOR[ mode ][ style ],
				m_DisplayAddressMapRuns[ mode ][ style ],
				m_DisplayAddressMapMaxRun[ mode ][ style ] );
			m_Writes += 16u + DISPLAY_SENTINEL_LENGTH * ( style ? 2u : 1u );
			m_Reads += DISPLAY_SENTINEL_LENGTH + 2u;
		}
	}

	// If the third immediate read was reliable in all four arms, a temporal
	// check is still required. Fresh double seeds make each rung independent;
	// text versus bitmap mode is the only variable. The 0-us rung measures the
	// ladder's own overhead, then the spacing grows to four seconds.
	bool immediateClean = true;
	for ( u32 mode = 0; mode < 2; mode++ )
		for ( u32 style = 0; style < 2; style++ )
			if ( m_DisplayAddressImmediateErrors[ mode ][ style ][ 2 ] != 0 )
				immediateClean = false;

	static const u32 delayUsec[ 9 ] =
		{ 0, 100, 1000, 4000, 16000, 64000, 256000, 1000000, 4000000 };
	if ( immediateClean )
	{
		m_DisplayAddressLadderRan = 1;
		for ( u32 rung = 0; rung < 9; rung++ )
			m_DisplayAddressDelayUsec[ rung ] = delayUsec[ rung ];
		for ( u32 mode = 0; mode < 2; mode++ )
		{
			displayAddressSetMode( mode != 0, mode ? 8 : 13 );
			m_Writes += 16;
			for ( u32 rung = 0; rung < 9; rung++ )
			{
				const u8 expected =
					(u8)( 0x55u + mode * 0x39u + rung * 0x1Du );
				m_DisplayAddressLadderExpected[ mode ][ rung ] = expected;
				busDiagRawWrite( target, expected );
				busDiagRawWrite( target, expected );
				for ( u32 read = 0; read < 3; read++ )
					m_DisplayAddressLadderInitial[ mode ][ rung ][ read ] =
						busDiagRawRead( target );
				const u64 started = hostCycles();
				const u64 duration =
					(u64)delayUsec[ rung ] * ( SCPU_ARM_CLOCK_HZ / 1000000u );
				while ( hostCycles() - started < duration ) asm volatile( "nop" );
				m_DisplayAddressElapsedUsec[ mode ][ rung ] =
					(u32)( ( hostCycles() - started )
					       / ( SCPU_ARM_CLOCK_HZ / 1000000u ) );
				for ( u32 read = 0; read < 3; read++ )
					m_DisplayAddressLadderDelayed[ mode ][ rung ][ read ] =
						busDiagRawRead( target );
				m_Writes += 2;
				m_Reads += 6;
			}
		}
	}
	return true;
}

bool CRADBus::runDisplayRowDiagnostic()
{
	if ( !m_Acquired || m_TrafficHalted
	     || m_Signals.machine != MACHINE_C64 )
		return false;

	m_DisplayRowRan = true;
	// Eight single-write maps in a deliberately non-alternating permutation,
	// followed by one genuine adjacent-repeat control in each mode. This keeps
	// mode from being a synonym for execution order or starting raster phase.
	static const u8 modes[ DISPLAY_ROW_ARM_COUNT ] =
		{ 0, 1, 1, 0, 1, 0, 0, 1, 0, 1 };
	static const u8 kinds[ DISPLAY_ROW_ARM_COUNT ] =
		{ 0, 0, 0, 0, 0, 0, 0, 0, 1, 1 };
	static const u8 borders[ DISPLAY_ROW_ARM_COUNT ] =
		{ 5, 7, 2, 3, 4, 8, 13, 10, 14, 6 };

	for ( u32 arm = 0; arm < DISPLAY_ROW_ARM_COUNT; arm++ )
	{
		const u8 salt = (u8)( 0x21u + arm * 0x17u );
		m_DisplayRowMode[ arm ] = modes[ arm ];
		m_DisplayRowKind[ arm ] = kinds[ arm ];
		m_DisplayRowSalt[ arm ] = salt;
		displayAddressSetMode( modes[ arm ] != 0, borders[ arm ] );
		m_Writes += 16;

		// Establish a verified complement at every address. These are true
		// adjacent repeats, not two whole-array passes. Retry only the setup;
		// the tested pass is always issued exactly once.
		for ( u32 attempt = 0; attempt < 3; attempt++ )
		{
			m_DisplayRowPrefillAttempts[ arm ] = (u8)( attempt + 1u );
			m_DisplayRowSeedUsec[ arm ][ 0 ] =
				displayRowSeed( salt, true, true );
			m_DisplayRowCaptureUsec[ arm ][ 0 ] =
				displayRowCapture( salt, true,
				                   displayRowPrefillMismatch[ arm ] );
			displayAddressAnalyzeMap(
				displayRowPrefillMismatch[ arm ],
				m_DisplayRowErrors[ arm ][ 0 ],
				m_DisplayRowFirst[ arm ][ 0 ],
				m_DisplayRowLast[ arm ][ 0 ],
				m_DisplayRowAND[ arm ][ 0 ],
				m_DisplayRowOR[ arm ][ 0 ],
				m_DisplayRowRuns[ arm ][ 0 ],
				m_DisplayRowMaxRun[ arm ][ 0 ] );
			m_Writes += DISPLAY_SENTINEL_LENGTH * 2u;
			m_Reads += DISPLAY_SENTINEL_LENGTH + 2u;
			if ( m_DisplayRowErrors[ arm ][ 0 ] == 0 ) break;
		}

		m_DisplayRowSeedUsec[ arm ][ 1 ] =
			displayRowSeed( salt, false, kinds[ arm ] != 0 );
		m_DisplayRowCaptureUsec[ arm ][ 1 ] =
			displayRowCapture( salt, false, displayRowTestMismatch[ arm ] );
		displayAddressAnalyzeMap(
			displayRowTestMismatch[ arm ],
			m_DisplayRowErrors[ arm ][ 1 ],
			m_DisplayRowFirst[ arm ][ 1 ],
			m_DisplayRowLast[ arm ][ 1 ],
			m_DisplayRowAND[ arm ][ 1 ],
			m_DisplayRowOR[ arm ][ 1 ],
			m_DisplayRowRuns[ arm ][ 1 ],
			m_DisplayRowMaxRun[ arm ][ 1 ] );
		m_Writes += DISPLAY_SENTINEL_LENGTH * ( kinds[ arm ] ? 2u : 1u );
		m_Reads += DISPLAY_SENTINEL_LENGTH + 2u;
	}

	// Independent retention arms. Each rung gets a fresh, verified, adjacent-
	// repeat seed and one baseline capture. No physical access occurs between
	// the end of that capture and its delayed capture, so an earlier rung's
	// readback cannot refresh a later rung.
	static const u32 delays[ DISPLAY_ROW_RETENTION_COUNT ] =
		{ 500000, 1000000, 2000000, 4000000, 8000000, 16000000 };
	for ( u32 rung = 0; rung < DISPLAY_ROW_RETENTION_COUNT; rung++ )
	{
		const u8 salt = (u8)( 0xB3u + rung * 0x25u );
		m_DisplayRowRetentionSalt[ rung ] = salt;
		m_DisplayRowRetentionDelayUsec[ rung ] = delays[ rung ];
		displayAddressSetMode( true, (u8)( 1u + rung ) );
		m_Writes += 16;

		for ( u32 attempt = 0; attempt < 3; attempt++ )
		{
			m_DisplayRowRetentionAttempts[ rung ] = (u8)( attempt + 1u );
			m_DisplayRowRetentionSeedUsec[ rung ] =
				displayRowSeed( salt, false, true );
			m_DisplayRowRetentionCaptureUsec[ rung ][ 0 ] =
				displayRowCapture(
					salt, false, displayRowRetentionBaselineMismatch[ rung ] );
			displayAddressAnalyzeMap(
				displayRowRetentionBaselineMismatch[ rung ],
				m_DisplayRowRetentionErrors[ rung ][ 0 ],
				m_DisplayRowRetentionFirst[ rung ][ 0 ],
				m_DisplayRowRetentionLast[ rung ][ 0 ],
				m_DisplayRowRetentionAND[ rung ][ 0 ],
				m_DisplayRowRetentionOR[ rung ][ 0 ],
				m_DisplayRowRetentionRuns[ rung ][ 0 ],
				m_DisplayRowRetentionMaxRun[ rung ][ 0 ] );
			m_Writes += DISPLAY_SENTINEL_LENGTH * 2u;
			m_Reads += DISPLAY_SENTINEL_LENGTH + 2u;
			if ( m_DisplayRowRetentionErrors[ rung ][ 0 ] == 0 ) break;
		}

		const u64 started = hostCycles();
		const u64 duration =
			(u64)delays[ rung ] * ( SCPU_ARM_CLOCK_HZ / 1000000u );
		while ( hostCycles() - started < duration ) asm volatile( "nop" );
		m_DisplayRowRetentionElapsedUsec[ rung ] =
			(u32)( ( hostCycles() - started )
			       / ( SCPU_ARM_CLOCK_HZ / 1000000u ) );
		m_DisplayRowRetentionCaptureUsec[ rung ][ 1 ] =
			displayRowCapture( salt, false,
			                   displayRowRetentionMismatch[ rung ] );
		displayAddressAnalyzeMap(
			displayRowRetentionMismatch[ rung ],
			m_DisplayRowRetentionErrors[ rung ][ 1 ],
			m_DisplayRowRetentionFirst[ rung ][ 1 ],
			m_DisplayRowRetentionLast[ rung ][ 1 ],
			m_DisplayRowRetentionAND[ rung ][ 1 ],
			m_DisplayRowRetentionOR[ rung ][ 1 ],
			m_DisplayRowRetentionRuns[ rung ][ 1 ],
			m_DisplayRowRetentionMaxRun[ rung ][ 1 ] );
		m_Reads += DISPLAY_SENTINEL_LENGTH + 2u;
	}
	return true;
}

bool CRADBus::runDisplayFetchDiagnostic()
{
	if ( !m_Acquired || m_TrafficHalted
	     || m_Signals.machine != MACHINE_C64 )
		return false;

	m_DisplayFetchRan = true;
	static const u8 states[ DISPLAY_FETCH_ARM_COUNT ] =
		{ 0, 2, 1, 2, 0, 1, 1, 2, 0 };
	static const u8 operations[ DISPLAY_FETCH_ARM_COUNT ] =
		{ 0, 0, 0, 1, 1, 1, 2, 2, 2 };
	static const u8 borders[ DISPLAY_FETCH_ARM_COUNT ] =
		{ 5, 7, 2, 3, 4, 8, 13, 10, 14 };
	static const u32 trafficRate = 16000;
	static const u32 exposureSeconds = 4;

	for ( u32 arm = 0; arm < DISPLAY_FETCH_ARM_COUNT; arm++ )
	{
		const u8 salt = (u8)( 0x19u + arm * 0x2Bu );
		const u32 rate = operations[ arm ] ? trafficRate : 0;
		m_DisplayFetchState[ arm ] = states[ arm ];
		m_DisplayFetchOperation[ arm ] = operations[ arm ];
		m_DisplayFetchSalt[ arm ] = salt;
		m_DisplayFetchRate[ arm ] = rate;
		m_DisplayFetchTrafficCount[ arm ] = rate * exposureSeconds;

		// Setup and both oracle captures are always blanked. Only the fixed
		// exposure interval enables text or bitmap fetches.
		displayFetchSetState( 0, borders[ arm ] );
		m_Writes += 16;
		for ( u32 attempt = 0; attempt < 3; attempt++ )
		{
			m_DisplayFetchPrefillAttempts[ arm ] = (u8)( attempt + 1u );
			m_DisplayFetchSeedUsec[ arm ][ 0 ] =
				displayRowSeed( salt, true, true );
			m_DisplayFetchCaptureUsec[ arm ][ 0 ] =
				displayRowCapture( salt, true,
				                   displayFetchPrefillMismatch[ arm ] );
			displayAddressAnalyzeMap(
				displayFetchPrefillMismatch[ arm ],
				m_DisplayFetchErrors[ arm ][ 0 ],
				m_DisplayFetchFirst[ arm ][ 0 ],
				m_DisplayFetchLast[ arm ][ 0 ],
				m_DisplayFetchAND[ arm ][ 0 ],
				m_DisplayFetchOR[ arm ][ 0 ],
				m_DisplayFetchRuns[ arm ][ 0 ],
				m_DisplayFetchMaxRun[ arm ][ 0 ] );
			m_Writes += DISPLAY_SENTINEL_LENGTH * 2u;
			m_Reads += DISPLAY_SENTINEL_LENGTH + 2u;
			if ( m_DisplayFetchErrors[ arm ][ 0 ] == 0 ) break;
		}

		m_DisplayFetchSeedUsec[ arm ][ 1 ] =
			displayRowSeed( salt, false, false );
		m_DisplayFetchCaptureUsec[ arm ][ 1 ] =
			displayRowCapture( salt, false,
			                   displayFetchBaselineMismatch[ arm ] );
		displayAddressAnalyzeMap(
			displayFetchBaselineMismatch[ arm ],
			m_DisplayFetchErrors[ arm ][ 1 ],
			m_DisplayFetchFirst[ arm ][ 1 ],
			m_DisplayFetchLast[ arm ][ 1 ],
			m_DisplayFetchAND[ arm ][ 1 ],
			m_DisplayFetchOR[ arm ][ 1 ],
			m_DisplayFetchRuns[ arm ][ 1 ],
			m_DisplayFetchMaxRun[ arm ][ 1 ] );
		m_Writes += DISPLAY_SENTINEL_LENGTH;
		m_Reads += DISPLAY_SENTINEL_LENGTH + 2u;

		displayFetchSetDEN( states[ arm ] );
		m_Writes += 2;
		const u64 started = hostCycles();
		if ( operations[ arm ] == 0 )
		{
			const u64 stop = started
			               + (u64)exposureSeconds * SCPU_ARM_CLOCK_HZ;
			while ( hostCycles() < stop ) asm volatile( "nop" );
		}
		else
		{
			const u64 interval = SCPU_ARM_CLOCK_HZ / trafficRate;
			u64 deadline = started + interval;
			for ( u32 access = 0;
			      access < m_DisplayFetchTrafficCount[ arm ]; access++ )
			{
				while ( hostCycles() < deadline ) asm volatile( "nop" );
				if ( operations[ arm ] == 1 )
					(void)busDiagRawRead( 0xD020 );
				else
					busDiagRawWrite( 0xD020, borders[ arm ] );
				deadline += interval;
			}
		}
		m_DisplayFetchElapsedUsec[ arm ] =
			(u32)( ( hostCycles() - started )
			       / ( SCPU_ARM_CLOCK_HZ / 1000000u ) );
		displayFetchBlank( states[ arm ] );
		m_Writes += 2;
		m_DisplayFetchCaptureUsec[ arm ][ 2 ] =
			displayRowCapture( salt, false, displayFetchPostMismatch[ arm ] );
		displayAddressAnalyzeMap(
			displayFetchPostMismatch[ arm ],
			m_DisplayFetchErrors[ arm ][ 2 ],
			m_DisplayFetchFirst[ arm ][ 2 ],
			m_DisplayFetchLast[ arm ][ 2 ],
			m_DisplayFetchAND[ arm ][ 2 ],
			m_DisplayFetchOR[ arm ][ 2 ],
			m_DisplayFetchRuns[ arm ][ 2 ],
			m_DisplayFetchMaxRun[ arm ][ 2 ] );
		m_Reads += DISPLAY_SENTINEL_LENGTH + 2u;
		if ( operations[ arm ] == 1 )
			m_Reads += m_DisplayFetchTrafficCount[ arm ];
		else if ( operations[ arm ] == 2 )
			m_Writes += m_DisplayFetchTrafficCount[ arm ];
	}

	// Independent single-write retention controls. Unlike K225, only the
	// complement prefill is repeated; every tested target is written once.
	static const u32 delays[ DISPLAY_FETCH_RETENTION_COUNT ] =
		{ 500000, 1000000, 2000000, 4000000, 8000000, 16000000 };
	for ( u32 rung = 0; rung < DISPLAY_FETCH_RETENTION_COUNT; rung++ )
	{
		const u8 salt = (u8)( 0xA7u + rung * 0x31u );
		m_DisplayFetchRetentionSalt[ rung ] = salt;
		m_DisplayFetchRetentionDelayUsec[ rung ] = delays[ rung ];
		displayAddressSetMode( true, (u8)( rung + 1u ) );
		m_Writes += 16;
		for ( u32 attempt = 0; attempt < 3; attempt++ )
		{
			m_DisplayFetchRetentionPrefillAttempts[ rung ] =
				(u8)( attempt + 1u );
			m_DisplayFetchRetentionSeedUsec[ rung ][ 0 ] =
				displayRowSeed( salt, true, true );
			m_DisplayFetchRetentionCaptureUsec[ rung ][ 0 ] =
				displayRowCapture(
					salt, true, displayFetchRetentionPrefillMismatch[ rung ] );
			displayAddressAnalyzeMap(
				displayFetchRetentionPrefillMismatch[ rung ],
				m_DisplayFetchRetentionErrors[ rung ][ 0 ],
				m_DisplayFetchRetentionFirst[ rung ][ 0 ],
				m_DisplayFetchRetentionLast[ rung ][ 0 ],
				m_DisplayFetchRetentionAND[ rung ][ 0 ],
				m_DisplayFetchRetentionOR[ rung ][ 0 ],
				m_DisplayFetchRetentionRuns[ rung ][ 0 ],
				m_DisplayFetchRetentionMaxRun[ rung ][ 0 ] );
			m_Writes += DISPLAY_SENTINEL_LENGTH * 2u;
			m_Reads += DISPLAY_SENTINEL_LENGTH + 2u;
			if ( m_DisplayFetchRetentionErrors[ rung ][ 0 ] == 0 ) break;
		}

		m_DisplayFetchRetentionSeedUsec[ rung ][ 1 ] =
			displayRowSeed( salt, false, false );
		m_DisplayFetchRetentionCaptureUsec[ rung ][ 1 ] =
			displayRowCapture(
				salt, false, displayFetchRetentionBaselineMismatch[ rung ] );
		displayAddressAnalyzeMap(
			displayFetchRetentionBaselineMismatch[ rung ],
			m_DisplayFetchRetentionErrors[ rung ][ 1 ],
			m_DisplayFetchRetentionFirst[ rung ][ 1 ],
			m_DisplayFetchRetentionLast[ rung ][ 1 ],
			m_DisplayFetchRetentionAND[ rung ][ 1 ],
			m_DisplayFetchRetentionOR[ rung ][ 1 ],
			m_DisplayFetchRetentionRuns[ rung ][ 1 ],
			m_DisplayFetchRetentionMaxRun[ rung ][ 1 ] );
		m_Writes += DISPLAY_SENTINEL_LENGTH;
		m_Reads += DISPLAY_SENTINEL_LENGTH + 2u;

		const u64 started = hostCycles();
		const u64 duration =
			(u64)delays[ rung ] * ( SCPU_ARM_CLOCK_HZ / 1000000u );
		while ( hostCycles() - started < duration ) asm volatile( "nop" );
		m_DisplayFetchRetentionElapsedUsec[ rung ] =
			(u32)( ( hostCycles() - started )
			       / ( SCPU_ARM_CLOCK_HZ / 1000000u ) );
		m_DisplayFetchRetentionCaptureUsec[ rung ][ 2 ] =
			displayRowCapture(
				salt, false, displayFetchRetentionPostMismatch[ rung ] );
		displayAddressAnalyzeMap(
			displayFetchRetentionPostMismatch[ rung ],
			m_DisplayFetchRetentionErrors[ rung ][ 2 ],
			m_DisplayFetchRetentionFirst[ rung ][ 2 ],
			m_DisplayFetchRetentionLast[ rung ][ 2 ],
			m_DisplayFetchRetentionAND[ rung ][ 2 ],
			m_DisplayFetchRetentionOR[ rung ][ 2 ],
			m_DisplayFetchRetentionRuns[ rung ][ 2 ],
			m_DisplayFetchRetentionMaxRun[ rung ][ 2 ] );
		m_Reads += DISPLAY_SENTINEL_LENGTH + 2u;
	}
	return true;
}

bool CRADBus::runDisplayPersistenceDiagnostic()
{
	if ( !m_Acquired || m_TrafficHalted
	     || m_Signals.machine != MACHINE_C64 )
		return false;

	m_DisplayPersistenceRan = true;
	// Each active-display condition appears twice, in a different position and
	// with a different salt. This catches both run/order dependence and an
	// oracle-data-dependent address family without spending time on the K226
	// display-off and retention controls that were clean in both hardware runs.
	static const u8 states[ DISPLAY_PERSISTENCE_ARM_COUNT ] =
		{ 1, 2, 1, 2, 1, 2, 2, 1, 2, 1, 2, 1 };
	static const u8 operations[ DISPLAY_PERSISTENCE_ARM_COUNT ] =
		{ 1, 0, 2, 1, 0, 2, 2, 0, 1, 2, 0, 1 };
	static const u8 borders[ DISPLAY_PERSISTENCE_ARM_COUNT ] =
		{ 2, 3, 4, 5, 6, 7, 8, 9, 10, 13, 14, 15 };
	static const u32 trafficRate = 16000;
	static const u32 exposureSeconds = 4;

	for ( u32 arm = 0; arm < DISPLAY_PERSISTENCE_ARM_COUNT; arm++ )
	{
		const u8 salt = (u8)( 0x31u + arm * 0x37u );
		const u32 trafficCount = operations[ arm ]
		                       ? trafficRate * exposureSeconds : 0;
		m_DisplayPersistenceState[ arm ] = states[ arm ];
		m_DisplayPersistenceOperation[ arm ] = operations[ arm ];
		m_DisplayPersistenceSalt[ arm ] = salt;
		m_DisplayPersistenceTrafficCount[ arm ] = trafficCount;

		displayFetchSetState( 0, borders[ arm ] );
		m_Writes += 16;
		for ( u32 attempt = 0; attempt < 3; attempt++ )
		{
			m_DisplayPersistencePrefillAttempts[ arm ] =
				(u8)( attempt + 1u );
			m_DisplayPersistenceSeedUsec[ arm ][ 0 ] =
				displayRowSeed( salt, true, true );
			m_DisplayPersistenceCaptureUsec[ arm ][ 0 ] =
				displayPersistenceCapture(
					salt, true, displayPersistenceMismatch[ arm ][ 0 ],
					m_DisplayPersistenceFirstExpected[ arm ][ 0 ],
					m_DisplayPersistenceFirstActual[ arm ][ 0 ],
					m_DisplayPersistenceXorAND[ arm ][ 0 ],
					m_DisplayPersistenceXorOR[ arm ][ 0 ] );
			displayAddressAnalyzeMap(
				displayPersistenceMismatch[ arm ][ 0 ],
				m_DisplayPersistenceErrors[ arm ][ 0 ],
				m_DisplayPersistenceFirst[ arm ][ 0 ],
				m_DisplayPersistenceLast[ arm ][ 0 ],
				m_DisplayPersistenceAND[ arm ][ 0 ],
				m_DisplayPersistenceOR[ arm ][ 0 ],
				m_DisplayPersistenceRuns[ arm ][ 0 ],
				m_DisplayPersistenceMaxRun[ arm ][ 0 ] );
			m_Writes += DISPLAY_SENTINEL_LENGTH * 2u;
			m_Reads += DISPLAY_SENTINEL_LENGTH + 2u;
			if ( m_DisplayPersistenceErrors[ arm ][ 0 ] == 0 ) break;
		}

		m_DisplayPersistenceSeedUsec[ arm ][ 1 ] =
			displayRowSeed( salt, false, false );
		m_DisplayPersistenceCaptureUsec[ arm ][ 1 ] =
			displayPersistenceCapture(
				salt, false, displayPersistenceMismatch[ arm ][ 1 ],
				m_DisplayPersistenceFirstExpected[ arm ][ 1 ],
				m_DisplayPersistenceFirstActual[ arm ][ 1 ],
				m_DisplayPersistenceXorAND[ arm ][ 1 ],
				m_DisplayPersistenceXorOR[ arm ][ 1 ] );
		displayAddressAnalyzeMap(
			displayPersistenceMismatch[ arm ][ 1 ],
			m_DisplayPersistenceErrors[ arm ][ 1 ],
			m_DisplayPersistenceFirst[ arm ][ 1 ],
			m_DisplayPersistenceLast[ arm ][ 1 ],
			m_DisplayPersistenceAND[ arm ][ 1 ],
			m_DisplayPersistenceOR[ arm ][ 1 ],
			m_DisplayPersistenceRuns[ arm ][ 1 ],
			m_DisplayPersistenceMaxRun[ arm ][ 1 ] );
		m_Writes += DISPLAY_SENTINEL_LENGTH;
		m_Reads += DISPLAY_SENTINEL_LENGTH + 2u;

		displayFetchSetDEN( states[ arm ] );
		m_Writes += 2;
		const u64 started = hostCycles();
		if ( operations[ arm ] == 0 )
		{
			const u64 stop = started
			               + (u64)exposureSeconds * SCPU_ARM_CLOCK_HZ;
			while ( hostCycles() < stop ) asm volatile( "nop" );
		}
		else
		{
			const u64 interval = SCPU_ARM_CLOCK_HZ / trafficRate;
			u64 deadline = started + interval;
			for ( u32 access = 0; access < trafficCount; access++ )
			{
				while ( hostCycles() < deadline ) asm volatile( "nop" );
				if ( operations[ arm ] == 1 )
					(void)busDiagRawRead( 0xD020 );
				else
					busDiagRawWrite( 0xD020, borders[ arm ] );
				deadline += interval;
			}
		}
		m_DisplayPersistenceElapsedUsec[ arm ] =
			(u32)( ( hostCycles() - started )
			       / ( SCPU_ARM_CLOCK_HZ / 1000000u ) );
		displayFetchBlank( states[ arm ] );
		m_Writes += 2;

		for ( u32 post = 0; post < 3; post++ )
		{
			const u32 phase = post + 2u;
			m_DisplayPersistenceCaptureUsec[ arm ][ phase ] =
				displayPersistenceCapture(
					salt, false, displayPersistenceMismatch[ arm ][ phase ],
					m_DisplayPersistenceFirstExpected[ arm ][ phase ],
					m_DisplayPersistenceFirstActual[ arm ][ phase ],
					m_DisplayPersistenceXorAND[ arm ][ phase ],
					m_DisplayPersistenceXorOR[ arm ][ phase ] );
			displayAddressAnalyzeMap(
				displayPersistenceMismatch[ arm ][ phase ],
				m_DisplayPersistenceErrors[ arm ][ phase ],
				m_DisplayPersistenceFirst[ arm ][ phase ],
				m_DisplayPersistenceLast[ arm ][ phase ],
				m_DisplayPersistenceAND[ arm ][ phase ],
				m_DisplayPersistenceOR[ arm ][ phase ],
				m_DisplayPersistenceRuns[ arm ][ phase ],
				m_DisplayPersistenceMaxRun[ arm ][ phase ] );
			m_Reads += DISPLAY_SENTINEL_LENGTH + 2u;
		}
		for ( u32 pair = 0; pair < 2; pair++ )
			displaySentinelCompare(
				displayPersistenceMismatch[ arm ][ pair + 2u ],
				displayPersistenceMismatch[ arm ][ pair + 3u ],
				m_DisplayPersistenceSame[ arm ][ pair ],
				m_DisplayPersistenceAdded[ arm ][ pair ],
				m_DisplayPersistenceRemoved[ arm ][ pair ] );
		if ( operations[ arm ] == 1 ) m_Reads += trafficCount;
		else if ( operations[ arm ] == 2 ) m_Writes += trafficCount;
	}
	return true;
}

bool CRADBus::runDisplayTimingDiagnostic()
{
	if ( !m_Acquired || m_TrafficHalted
	     || m_Signals.machine != MACHINE_C64 )
		return false;

	m_DisplayTimingRan = true;
	static const u8 states[ DISPLAY_TIMING_ARM_COUNT ] = { 1, 2, 1, 2, 1, 2 };
	static const u8 operations[ DISPLAY_TIMING_ARM_COUNT ] = { 1, 0, 2, 1, 0, 2 };
	// These are K227 arms 0..5 exactly, including all four salts that produced
	// stable dirty maps and both clean bitmap controls.
	static const u8 salts[ DISPLAY_TIMING_ARM_COUNT ] =
		{ 0x31, 0x68, 0x9F, 0xD6, 0x0D, 0x44 };
	static const u8 borders[ DISPLAY_TIMING_ARM_COUNT ] = { 2, 3, 4, 5, 6, 7 };
	static const u16 samples[ DISPLAY_TIMING_PHASE_COUNT ] =
		{ 475, 350, 400, 450, 475, 500, 550, 600, 475, 475 };
	static const u32 trafficRate = 16000;
	static const u32 exposureSeconds = 4;
	const u16 configured = (u16)busTiming.WAIT_CYCLE_READ;
	for ( u32 phase = 0; phase < DISPLAY_TIMING_PHASE_COUNT; phase++ )
		m_DisplayTimingSample[ phase ] = samples[ phase ];

	for ( u32 arm = 0; arm < DISPLAY_TIMING_ARM_COUNT; arm++ )
	{
		const u8 salt = salts[ arm ];
		const u32 trafficCount = operations[ arm ]
		                       ? trafficRate * exposureSeconds : 0;
		m_DisplayTimingState[ arm ] = states[ arm ];
		m_DisplayTimingOperation[ arm ] = operations[ arm ];
		m_DisplayTimingSalt[ arm ] = salt;
		m_DisplayTimingTrafficCount[ arm ] = trafficCount;

		displayFetchSetState( 0, borders[ arm ] );
		m_Writes += 16;
		busTiming.WAIT_CYCLE_READ = configured;
		busTiming.WAIT_CYCLE_READ2 = (u16)( configured + 20u );
		for ( u32 attempt = 0; attempt < 3; attempt++ )
		{
			m_DisplayTimingPrefillAttempts[ arm ] = (u8)( attempt + 1u );
			(void)displayRowSeed( salt, true, true );
			m_DisplayTimingCaptureUsec[ arm ][ 9 ] =
				displayPersistenceCapture(
					salt, true, displayTimingMismatch[ arm ][ 9 ],
					m_DisplayTimingFirstExpected[ arm ][ 9 ],
					m_DisplayTimingFirstActual[ arm ][ 9 ],
					m_DisplayTimingXorAND[ arm ][ 9 ],
					m_DisplayTimingXorOR[ arm ][ 9 ] );
			displayAddressAnalyzeMap(
				displayTimingMismatch[ arm ][ 9 ],
				m_DisplayTimingErrors[ arm ][ 9 ],
				m_DisplayTimingFirst[ arm ][ 9 ],
				m_DisplayTimingLast[ arm ][ 9 ],
				m_DisplayTimingAND[ arm ][ 9 ],
				m_DisplayTimingOR[ arm ][ 9 ],
				m_DisplayTimingRuns[ arm ][ 9 ],
				m_DisplayTimingMaxRun[ arm ][ 9 ] );
			m_DisplayTimingPrefillErrors[ arm ] =
				m_DisplayTimingErrors[ arm ][ 9 ];
			m_Writes += DISPLAY_SENTINEL_LENGTH * 2u;
			m_Reads += DISPLAY_SENTINEL_LENGTH + 2u;
			if ( m_DisplayTimingPrefillErrors[ arm ] == 0 ) break;
		}

		(void)displayRowSeed( salt, false, false );
		m_DisplayTimingCaptureUsec[ arm ][ 9 ] =
			displayPersistenceCapture(
				salt, false, displayTimingMismatch[ arm ][ 9 ],
				m_DisplayTimingFirstExpected[ arm ][ 9 ],
				m_DisplayTimingFirstActual[ arm ][ 9 ],
				m_DisplayTimingXorAND[ arm ][ 9 ],
				m_DisplayTimingXorOR[ arm ][ 9 ] );
		displayAddressAnalyzeMap(
			displayTimingMismatch[ arm ][ 9 ],
			m_DisplayTimingErrors[ arm ][ 9 ],
			m_DisplayTimingFirst[ arm ][ 9 ],
			m_DisplayTimingLast[ arm ][ 9 ],
			m_DisplayTimingAND[ arm ][ 9 ],
			m_DisplayTimingOR[ arm ][ 9 ],
			m_DisplayTimingRuns[ arm ][ 9 ],
			m_DisplayTimingMaxRun[ arm ][ 9 ] );
		m_DisplayTimingBaselineErrors[ arm ] = m_DisplayTimingErrors[ arm ][ 9 ];
		m_Writes += DISPLAY_SENTINEL_LENGTH;
		m_Reads += DISPLAY_SENTINEL_LENGTH + 2u;

		displayFetchSetDEN( states[ arm ] );
		m_Writes += 2;
		const u64 started = hostCycles();
		if ( operations[ arm ] == 0 )
		{
			const u64 stop = started
			               + (u64)exposureSeconds * SCPU_ARM_CLOCK_HZ;
			while ( hostCycles() < stop ) asm volatile( "nop" );
		}
		else
		{
			const u64 interval = SCPU_ARM_CLOCK_HZ / trafficRate;
			u64 deadline = started + interval;
			for ( u32 access = 0; access < trafficCount; access++ )
			{
				while ( hostCycles() < deadline ) asm volatile( "nop" );
				if ( operations[ arm ] == 1 )
					(void)busDiagRawRead( 0xD020 );
				else
					busDiagRawWrite( 0xD020, borders[ arm ] );
				deadline += interval;
			}
		}
		m_DisplayTimingElapsedUsec[ arm ] =
			(u32)( ( hostCycles() - started )
			       / ( SCPU_ARM_CLOCK_HZ / 1000000u ) );
		displayFetchBlank( states[ arm ] );
		m_Writes += 2;

		// Phase 0 is the configured-point reference. Phases 1..7 sweep the
		// sample point, and phase 8 returns to 475 to prove the sweep itself did
		// not alter the map. Phase 9 is captured after targeted repair below.
		for ( u32 phase = 0; phase < 9; phase++ )
		{
			busTiming.WAIT_CYCLE_READ = samples[ phase ];
			busTiming.WAIT_CYCLE_READ2 = (u16)( samples[ phase ] + 20u );
			m_DisplayTimingCaptureUsec[ arm ][ phase ] =
				displayPersistenceCapture(
					salt, false, displayTimingMismatch[ arm ][ phase ],
					m_DisplayTimingFirstExpected[ arm ][ phase ],
					m_DisplayTimingFirstActual[ arm ][ phase ],
					m_DisplayTimingXorAND[ arm ][ phase ],
					m_DisplayTimingXorOR[ arm ][ phase ] );
			displayAddressAnalyzeMap(
				displayTimingMismatch[ arm ][ phase ],
				m_DisplayTimingErrors[ arm ][ phase ],
				m_DisplayTimingFirst[ arm ][ phase ],
				m_DisplayTimingLast[ arm ][ phase ],
				m_DisplayTimingAND[ arm ][ phase ],
				m_DisplayTimingOR[ arm ][ phase ],
				m_DisplayTimingRuns[ arm ][ phase ],
				m_DisplayTimingMaxRun[ arm ][ phase ] );
			displaySentinelCompare(
				displayTimingMismatch[ arm ][ 0 ],
				displayTimingMismatch[ arm ][ phase ],
				m_DisplayTimingSame[ arm ][ phase ],
				m_DisplayTimingAdded[ arm ][ phase ],
				m_DisplayTimingRemoved[ arm ][ phase ] );
			m_Reads += DISPLAY_SENTINEL_LENGTH + 2u;
		}

		busTiming.WAIT_CYCLE_READ = configured;
		busTiming.WAIT_CYCLE_READ2 = (u16)( configured + 20u );
		for ( u32 index = 0; index < DISPLAY_SENTINEL_LENGTH; index++ )
		{
			if ( ( displayTimingMismatch[ arm ][ 0 ][ index >> 3 ]
			       & (u8)( 1u << ( index & 7 ) ) ) == 0 ) continue;
			busDiagRawWrite( displaySentinelAddr( index ),
			                 displayRowExpected( index, salt ) );
			m_DisplayTimingRepairWrites[ arm ]++;
			m_Writes++;
		}
		m_DisplayTimingCaptureUsec[ arm ][ 9 ] =
			displayPersistenceCapture(
				salt, false, displayTimingMismatch[ arm ][ 9 ],
				m_DisplayTimingFirstExpected[ arm ][ 9 ],
				m_DisplayTimingFirstActual[ arm ][ 9 ],
				m_DisplayTimingXorAND[ arm ][ 9 ],
				m_DisplayTimingXorOR[ arm ][ 9 ] );
		displayAddressAnalyzeMap(
			displayTimingMismatch[ arm ][ 9 ],
			m_DisplayTimingErrors[ arm ][ 9 ],
			m_DisplayTimingFirst[ arm ][ 9 ],
			m_DisplayTimingLast[ arm ][ 9 ],
			m_DisplayTimingAND[ arm ][ 9 ],
			m_DisplayTimingOR[ arm ][ 9 ],
			m_DisplayTimingRuns[ arm ][ 9 ],
			m_DisplayTimingMaxRun[ arm ][ 9 ] );
		displaySentinelCompare(
			displayTimingMismatch[ arm ][ 0 ],
			displayTimingMismatch[ arm ][ 9 ],
			m_DisplayTimingSame[ arm ][ 9 ],
			m_DisplayTimingAdded[ arm ][ 9 ],
			m_DisplayTimingRemoved[ arm ][ 9 ] );
		m_Reads += DISPLAY_SENTINEL_LENGTH + 2u;
		if ( operations[ arm ] == 1 ) m_Reads += trafficCount;
		else if ( operations[ arm ] == 2 ) m_Writes += trafficCount;
	}
	busTiming.WAIT_CYCLE_READ = configured;
	busTiming.WAIT_CYCLE_READ2 = (u16)( configured + 20u );
	return true;
}

bool CRADBus::runDisplayBoundaryDiagnostic()
{
	if ( !m_Acquired || m_TrafficHalted
	     || m_Signals.machine != MACHINE_C64 )
		return false;

	m_DisplayBoundaryRan = true;
	// Slots 0..7 are the full state x transition-raster x duration matrix.
	// Slots 8..15 repeat the exact same oracles in a permuted order. The two
	// salts are patterns already known to fail on hardware (K228 text/none and
	// K226 bitmap/none), avoiding a false clean result from pattern sensitivity.
	static const u8 slots[ DISPLAY_BOUNDARY_ARM_COUNT ] =
		{ 0, 1, 2, 3, 4, 5, 6, 7, 7, 2, 5, 0, 3, 6, 1, 4 };
	static const u8 borders[ DISPLAY_BOUNDARY_ARM_COUNT ] =
		{ 2, 3, 4, 5, 6, 7, 8, 9, 10, 13, 14, 15, 2, 3, 4, 5 };
	static const u32 exposureSeconds = 4;

	for ( u32 arm = 0; arm < DISPLAY_BOUNDARY_ARM_COUNT; arm++ )
	{
		const u8 slot = slots[ arm ];
		const u8 state = slot < 4 ? 1 : 2;
		const bool safe = ( slot & 1u ) != 0;
		const bool dwell = ( slot & 2u ) != 0;
		const u8 salt = state == 1 ? 0x0D : 0x44;
		m_DisplayBoundaryState[ arm ] = state;
		m_DisplayBoundarySafe[ arm ] = safe ? 1 : 0;
		m_DisplayBoundaryDwell[ arm ] = dwell ? 1 : 0;
		m_DisplayBoundarySalt[ arm ] = salt;

		displayFetchSetState( 0, borders[ arm ] );
		m_Writes += 16;
		for ( u32 attempt = 0; attempt < 3; attempt++ )
		{
			m_DisplayBoundaryPrefillAttempts[ arm ] = (u8)( attempt + 1u );
			(void)displayRowSeed( salt, true, true );
			(void)displayPersistenceCapture(
				salt, true, displayBoundaryMismatch[ arm ][ 0 ],
				m_DisplayBoundaryFirstExpected[ arm ][ 0 ],
				m_DisplayBoundaryFirstActual[ arm ][ 0 ],
				m_DisplayBoundaryXorAND[ arm ][ 0 ],
				m_DisplayBoundaryXorOR[ arm ][ 0 ] );
			displayAddressAnalyzeMap(
				displayBoundaryMismatch[ arm ][ 0 ],
				m_DisplayBoundaryPrefillErrors[ arm ],
				m_DisplayBoundaryFirst[ arm ][ 0 ],
				m_DisplayBoundaryLast[ arm ][ 0 ],
				m_DisplayBoundaryAND[ arm ][ 0 ],
				m_DisplayBoundaryOR[ arm ][ 0 ],
				m_DisplayBoundaryRuns[ arm ][ 0 ],
				m_DisplayBoundaryMaxRun[ arm ][ 0 ] );
			m_Writes += DISPLAY_SENTINEL_LENGTH * 2u;
			m_Reads += DISPLAY_SENTINEL_LENGTH + 2u;
			if ( m_DisplayBoundaryPrefillErrors[ arm ] == 0 ) break;
		}

		(void)displayRowSeed( salt, false, false );
		m_DisplayBoundaryCaptureUsec[ arm ][ 0 ] =
			displayPersistenceCapture(
				salt, false, displayBoundaryMismatch[ arm ][ 0 ],
				m_DisplayBoundaryFirstExpected[ arm ][ 0 ],
				m_DisplayBoundaryFirstActual[ arm ][ 0 ],
				m_DisplayBoundaryXorAND[ arm ][ 0 ],
				m_DisplayBoundaryXorOR[ arm ][ 0 ] );
		displayAddressAnalyzeMap(
			displayBoundaryMismatch[ arm ][ 0 ],
			m_DisplayBoundaryErrors[ arm ][ 0 ],
			m_DisplayBoundaryFirst[ arm ][ 0 ],
			m_DisplayBoundaryLast[ arm ][ 0 ],
			m_DisplayBoundaryAND[ arm ][ 0 ],
			m_DisplayBoundaryOR[ arm ][ 0 ],
			m_DisplayBoundaryRuns[ arm ][ 0 ],
			m_DisplayBoundaryMaxRun[ arm ][ 0 ] );
		m_Writes += DISPLAY_SENTINEL_LENGTH;
		m_Reads += DISPLAY_SENTINEL_LENGTH + 2u;

		m_DisplayBoundaryWaitReads[ arm ][ 0 ] =
			displayBoundaryWaitRaster( safe );
		m_Reads += m_DisplayBoundaryWaitReads[ arm ][ 0 ] & 0x7FFFFFFFu;
		const u64 started = hostCycles();
		displayFetchSetDEN( state );
		m_Writes += 2;
		if ( dwell )
		{
			const u64 stop = started
			               + (u64)exposureSeconds * SCPU_ARM_CLOCK_HZ;
			while ( hostCycles() < stop ) asm volatile( "nop" );
			m_DisplayBoundaryWaitReads[ arm ][ 1 ] =
				displayBoundaryWaitRaster( safe );
			m_Reads += m_DisplayBoundaryWaitReads[ arm ][ 1 ] & 0x7FFFFFFFu;
		}
		displayFetchBlank( state );
		m_Writes += 2;
		m_DisplayBoundaryElapsedUsec[ arm ] =
			(u32)( ( hostCycles() - started )
			       / ( SCPU_ARM_CLOCK_HZ / 1000000u ) );

		m_DisplayBoundaryCaptureUsec[ arm ][ 1 ] =
			displayPersistenceCapture(
				salt, false, displayBoundaryMismatch[ arm ][ 1 ],
				m_DisplayBoundaryFirstExpected[ arm ][ 1 ],
				m_DisplayBoundaryFirstActual[ arm ][ 1 ],
				m_DisplayBoundaryXorAND[ arm ][ 1 ],
				m_DisplayBoundaryXorOR[ arm ][ 1 ] );
		displayAddressAnalyzeMap(
			displayBoundaryMismatch[ arm ][ 1 ],
			m_DisplayBoundaryErrors[ arm ][ 1 ],
			m_DisplayBoundaryFirst[ arm ][ 1 ],
			m_DisplayBoundaryLast[ arm ][ 1 ],
			m_DisplayBoundaryAND[ arm ][ 1 ],
			m_DisplayBoundaryOR[ arm ][ 1 ],
			m_DisplayBoundaryRuns[ arm ][ 1 ],
			m_DisplayBoundaryMaxRun[ arm ][ 1 ] );
		displaySentinelCompare(
			displayBoundaryMismatch[ arm ][ 0 ],
			displayBoundaryMismatch[ arm ][ 1 ],
			m_DisplayBoundarySame[ arm ],
			m_DisplayBoundaryAdded[ arm ],
			m_DisplayBoundaryRemoved[ arm ] );
		m_Reads += DISPLAY_SENTINEL_LENGTH + 2u;
	}
	return true;
}

bool CRADBus::runDisplayRefreshDiagnostic()

{
	return runDisplayRefreshDiagnosticVariant( 0 );
}

bool CRADBus::runDisplayCore0RefreshDiagnostic()
{
	return runDisplayRefreshDiagnosticVariant( 1 );
}

bool CRADBus::runDisplayRWDiagnostic()
{
	return runDisplayRefreshDiagnosticVariant( 2 );
}

bool CRADBus::runDisplayScrubDiagnostic( CWriteBuffer &scrubBuffer )
{
	return runDisplayRefreshDiagnosticVariant( 3, &scrubBuffer );
}

bool CRADBus::runDisplayBAGuardDiagnostic()
{
	if ( !m_Acquired || m_TrafficHalted
	     || m_Signals.machine != MACHINE_C64 )
		return false;

	m_DisplayBAGuardRan = true;
	static const u8 operations[ DISPLAY_BA_LOCAL_ARM_COUNT ] =
		{ 0, 2, 1, 2, 2, 2, 2, 0, 2, 1, 2, 2 };
	static const u8 biases[ DISPLAY_BA_LOCAL_ARM_COUNT ] =
		{ 0, 40, 0, 0, 60, 20, 20, 0, 60, 0, 0, 40 };
	static const u32 trafficRate = 16000;
	static const u32 exposureSeconds = 4;

	for ( u32 arm = 0; arm < DISPLAY_BA_LOCAL_ARM_COUNT; arm++ )
	{
		const u8 operation = operations[ arm ];
		const u8 bias = biases[ arm ];
		const u8 rotation = arm >= DISPLAY_BA_LOCAL_ARM_COUNT / 2 ? 1 : 0;
		const u8 border = (u8)( 1u + arm );
		m_DisplayBAOperation[ arm ] = operation;
		m_DisplayBABias[ arm ] = bias;
		m_DisplayBARotation[ arm ] = rotation;
		m_DisplayBATrafficCount[ arm ] = operation
			? trafficRate * exposureSeconds : 0;

		// All setup and verification use the production timing point and DEN=0.
		// The complement pass clears any retained state from the previous arm.
		displaySentinelSetMode( false, border );
		m_Writes += 16;
		for ( u32 attempt = 0; attempt < 3; attempt++ )
		{
			m_DisplayBAPrefillAttempts[ arm ] = (u8)( attempt + 1u );
			(void)displayBASeed( rotation, true, true );
			u32 errors = 0;
			(void)displayBACapture( rotation, true,
			                      displayBAMismatch[ arm ][ 0 ], errors );
			m_Writes += DISPLAY_SENTINEL_LENGTH * 2u;
			m_Reads += DISPLAY_SENTINEL_LENGTH + 2u;
			if ( errors == 0 ) break;
		}

		// A stored baseline error must survive all three separately primed maps.
		// Retry the target seed if that conservative baseline is not clean.
		for ( u32 attempt = 0; attempt < 3; attempt++ )
		{
			m_DisplayBATargetAttempts[ arm ] = (u8)( attempt + 1u );
			(void)displayBASeed( rotation, false, false );
			m_Writes += DISPLAY_SENTINEL_LENGTH;
			for ( u32 pass = 0; pass < 3; pass++ )
			{
				u32 ignoredErrors = 0;
				m_DisplayBACaptureUsec[ arm ][ pass ] =
					displayBACapture( rotation, false,
					                  displayBAMismatch[ arm ][ pass ],
					                  ignoredErrors );
				m_Reads += DISPLAY_SENTINEL_LENGTH + 2u;
			}
			u32 persistent = 0;
			for ( u32 index = 0; index < DISPLAY_SENTINEL_LENGTH; index++ )
			{
				const u8 bit = (u8)( 1u << ( index & 7 ) );
				if ( ( displayBAMismatch[ arm ][ 0 ][ index >> 3 ] & bit )
				     && ( displayBAMismatch[ arm ][ 1 ][ index >> 3 ] & bit )
				     && ( displayBAMismatch[ arm ][ 2 ][ index >> 3 ] & bit ) )
					persistent++;
			}
			if ( persistent == 0 || attempt == 2 ) break;
		}

		u32 baselinePersistent = 0, baselineUnion = 0;
		for ( u32 index = 0; index < DISPLAY_SENTINEL_LENGTH; index++ )
		{
			const u8 bit = (u8)( 1u << ( index & 7 ) );
			const bool p0 = ( displayBAMismatch[ arm ][ 0 ][ index >> 3 ] & bit ) != 0;
			const bool p1 = ( displayBAMismatch[ arm ][ 1 ][ index >> 3 ] & bit ) != 0;
			const bool p2 = ( displayBAMismatch[ arm ][ 2 ][ index >> 3 ] & bit ) != 0;
			if ( p0 || p1 || p2 ) baselineUnion++;
			if ( p0 && p1 && p2 ) baselinePersistent++;
		}
		m_DisplayBABaselinePersistent[ arm ] = baselinePersistent;
		m_DisplayBABaselineUnion[ arm ] = baselineUnion;

		// Bias only the timed address-enable point of paced single writes. It is
		// restored before DEN is blanked, so setup/capture can never inherit it.
		displaySentinelSetDEN( true );
		m_Writes += 2;
		const u32 productionEnable =
			busTiming.TIMING_ENABLE_RWOUT_ADDR_LATCH_WRITING;
		if ( operation == 2 )
			busTiming.TIMING_ENABLE_RWOUT_ADDR_LATCH_WRITING =
				productionEnable + bias;
		const u64 started = hostCycles();
		if ( operation == 0 )
		{
			const u64 stop = started
			               + (u64)exposureSeconds * SCPU_ARM_CLOCK_HZ;
			while ( hostCycles() < stop ) asm volatile( "nop" );
		}
		else
		{
			const u64 interval = SCPU_ARM_CLOCK_HZ / trafficRate;
			u64 deadline = started + interval;
			for ( u32 access = 0;
			      access < m_DisplayBATrafficCount[ arm ]; access++ )
			{
				while ( hostCycles() < deadline ) asm volatile( "nop" );
				if ( operation == 1 ) (void)busDiagRawRead( 0xD020 );
				else busDiagRawWrite( 0xD020, border );
				deadline += interval;
			}
		}
		m_DisplayBAElapsedUsec[ arm ] =
			(u32)( ( hostCycles() - started )
			       / ( SCPU_ARM_CLOCK_HZ / 1000000u ) );
		busTiming.TIMING_ENABLE_RWOUT_ADDR_LATCH_WRITING = productionEnable;
		if ( operation == 1 ) m_Reads += m_DisplayBATrafficCount[ arm ];
		else if ( operation == 2 ) m_Writes += m_DisplayBATrafficCount[ arm ];

		displaySentinelSetDEN( false );
		m_Writes += 2;
		for ( u32 pass = 0; pass < 3; pass++ )
		{
			u32 ignoredErrors = 0;
			m_DisplayBACaptureUsec[ arm ][ pass + 3 ] =
				displayBACapture( rotation, false,
				                  displayBAMismatch[ arm ][ pass + 3 ],
				                  ignoredErrors );
			m_Reads += DISPLAY_SENTINEL_LENGTH + 2u;
		}

		u32 postPersistent = 0, postUnion = 0, addedPersistent = 0;
		for ( u32 byte = 0; byte < DISPLAY_SENTINEL_BITMAP_BYTES; byte++ )
			displayBAAddedMap[ arm ][ byte ] = 0;
		for ( u32 index = 0; index < DISPLAY_SENTINEL_LENGTH; index++ )
		{
			const u8 bit = (u8)( 1u << ( index & 7 ) );
			const bool b0 = ( displayBAMismatch[ arm ][ 0 ][ index >> 3 ] & bit ) != 0;
			const bool b1 = ( displayBAMismatch[ arm ][ 1 ][ index >> 3 ] & bit ) != 0;
			const bool b2 = ( displayBAMismatch[ arm ][ 2 ][ index >> 3 ] & bit ) != 0;
			const bool p0 = ( displayBAMismatch[ arm ][ 3 ][ index >> 3 ] & bit ) != 0;
			const bool p1 = ( displayBAMismatch[ arm ][ 4 ][ index >> 3 ] & bit ) != 0;
			const bool p2 = ( displayBAMismatch[ arm ][ 5 ][ index >> 3 ] & bit ) != 0;
			const bool postAny = p0 || p1 || p2;
			const bool postAll = p0 && p1 && p2;
			if ( postAny ) postUnion++;
			if ( postAll ) postPersistent++;
			// Baseline UNION is deliberately conservative: a target that looked
			// wrong even once before exposure is never counted as newly stored.
			if ( postAll && !( b0 || b1 || b2 ) )
			{
				displayBAAddedMap[ arm ][ index >> 3 ] |= bit;
				addedPersistent++;
				const u8 family = displayBAFamily( index, rotation );
				if ( family < DISPLAY_BA_FAMILY_COUNT )
					m_DisplayBAFamilyAdded[ arm ][ family ]++;
			}
		}
		m_DisplayBAPostPersistent[ arm ] = postPersistent;
		m_DisplayBAPostUnion[ arm ] = postUnion;
		m_DisplayBAAddedPersistent[ arm ] = addedPersistent;
		m_DisplayBAUnstableOnly[ arm ] = postUnion - postPersistent;
	}

	displaySentinelSetMode( false, 0 );
	m_Writes += 16;
	return true;
}

bool CRADBus::runDisplayRefreshDiagnosticVariant( u8 variant,
	                                                CWriteBuffer *scrubBuffer )
{
	if ( !m_Acquired || m_TrafficHalted
	     || m_Signals.machine != MACHINE_C64 )
		return false;
	if ( variant == 3 && !scrubBuffer )
		return false;

	m_DisplayRefreshRan = true;
	m_DisplayDiagnosticVariant = variant;
	const bool core0 = variant == 1;
	const bool rwHold = variant == 2;
	const bool scrubTest = variant == 3;
	// Each state/control/intervention condition is repeated in a permuted second
	// half. All variants retain K229's displayed oracle, border-synchronised
	// transitions, and absence of ordinary physical transactions during exposure.
	static const u8 slots[ 8 ] = { 0, 1, 2, 3, 3, 0, 1, 2 };
	static const u8 scrubStates[ 8 ] = { 1, 1, 2, 2, 2, 1, 2, 1 };
	static const u8 scrubEnabled[ 8 ] = { 0, 1, 0, 1, 1, 0, 0, 1 };
	static const u8 borders[ 8 ] = { 2, 3, 4, 5, 6, 7, 8, 9 };
	const u32 exposureSeconds = scrubTest ? 16u : rwHold ? 8u : 4u;

	for ( u32 arm = 0; arm < 8; arm++ )
	{
		const u8 slot = slots[ arm ];
		const u8 state = scrubTest ? scrubStates[ arm ]
		                           : slot < 2 ? 1 : 2;
		const bool refresh = scrubTest ? scrubEnabled[ arm ] != 0
		                               : ( slot & 1u ) != 0;
		const u8 salt = state == 1 ? 0x0D : 0x44;
		m_DisplayBoundaryState[ arm ] = state;
		m_DisplayBoundarySafe[ arm ] = 1;
		m_DisplayBoundaryDwell[ arm ] = 1;
		m_DisplayBoundarySalt[ arm ] = salt;
		m_DisplayRefreshEnabled[ arm ] = refresh ? 1 : 0;
		if ( scrubTest )
		{
			for ( u32 index = 0; index < DISPLAY_SENTINEL_LENGTH; index++ )
				displayScrubShadow[ displaySentinelAddr( index ) ] =
					displayRowExpected( index, salt );
			scrubBuffer->attach( this, displayScrubShadow );
			scrubBuffer->setOptMode( SCPU_OPT_NONE );
			scrubBuffer->resetStats();
		}

		displayFetchSetState( 0, borders[ arm ] );
		m_Writes += 16;
		for ( u32 attempt = 0; attempt < 3; attempt++ )
		{
			m_DisplayBoundaryPrefillAttempts[ arm ] = (u8)( attempt + 1u );
			(void)displayRowSeed( salt, true, true );
			(void)displayPersistenceCapture(
				salt, true, displayBoundaryMismatch[ arm ][ 0 ],
				m_DisplayBoundaryFirstExpected[ arm ][ 0 ],
				m_DisplayBoundaryFirstActual[ arm ][ 0 ],
				m_DisplayBoundaryXorAND[ arm ][ 0 ],
				m_DisplayBoundaryXorOR[ arm ][ 0 ] );
			displayAddressAnalyzeMap(
				displayBoundaryMismatch[ arm ][ 0 ],
				m_DisplayBoundaryPrefillErrors[ arm ],
				m_DisplayBoundaryFirst[ arm ][ 0 ],
				m_DisplayBoundaryLast[ arm ][ 0 ],
				m_DisplayBoundaryAND[ arm ][ 0 ],
				m_DisplayBoundaryOR[ arm ][ 0 ],
				m_DisplayBoundaryRuns[ arm ][ 0 ],
				m_DisplayBoundaryMaxRun[ arm ][ 0 ] );
			m_Writes += DISPLAY_SENTINEL_LENGTH * 2u;
			m_Reads += DISPLAY_SENTINEL_LENGTH + 2u;
			if ( m_DisplayBoundaryPrefillErrors[ arm ] == 0 ) break;
		}

		(void)displayRowSeed( salt, false, false );
		m_DisplayBoundaryCaptureUsec[ arm ][ 0 ] =
			displayPersistenceCapture(
				salt, false, displayBoundaryMismatch[ arm ][ 0 ],
				m_DisplayBoundaryFirstExpected[ arm ][ 0 ],
				m_DisplayBoundaryFirstActual[ arm ][ 0 ],
				m_DisplayBoundaryXorAND[ arm ][ 0 ],
				m_DisplayBoundaryXorOR[ arm ][ 0 ] );
		displayAddressAnalyzeMap(
			displayBoundaryMismatch[ arm ][ 0 ],
			m_DisplayBoundaryErrors[ arm ][ 0 ],
			m_DisplayBoundaryFirst[ arm ][ 0 ],
			m_DisplayBoundaryLast[ arm ][ 0 ],
			m_DisplayBoundaryAND[ arm ][ 0 ],
			m_DisplayBoundaryOR[ arm ][ 0 ],
			m_DisplayBoundaryRuns[ arm ][ 0 ],
			m_DisplayBoundaryMaxRun[ arm ][ 0 ] );
		m_Writes += DISPLAY_SENTINEL_LENGTH;
		m_Reads += DISPLAY_SENTINEL_LENGTH + 2u;

		m_DisplayBoundaryWaitReads[ arm ][ 0 ] =
			displayBoundaryWaitRaster( true );
		m_Reads += m_DisplayBoundaryWaitReads[ arm ][ 0 ] & 0x7FFFFFFFu;
		displayFetchSetDEN( state );
		m_Writes += 2;
		const u64 started = hostCycles();
		const u64 stop = started
		               + (u64)exposureSeconds * SCPU_ARM_CLOCK_HZ;
		if ( refresh && scrubTest )
		{
			m_DisplayRefreshStartOK[ arm ] = 1;
			const u64 opportunityCycles = SCPU_ARM_CLOCK_HZ / ( 60u * 128u );
			u64 nextOpportunity = started;
			u64 now = started;
			do
			{
				READ_CYCLE_COUNTER( now );
				if ( now >= stop ) break;
				if ( now < nextOpportunity )
					continue;
				nextOpportunity += opportunityCycles;
				m_DisplayScrubOpportunities[ arm ]++;
				const u16 line = rasterLine();
				if ( !c64RasterIsSafeForBulkTransfer( m_Signals.video, line ) )
					continue;
				u64 chunkStart, chunkEnd;
				READ_CYCLE_COUNTER( chunkStart );
				const u32 sent = scrubBuffer->resyncDisplayed(
					0x0400, state == 2 ? 0x2000 : 0xFFFFFFFF,
					64, true );
				READ_CYCLE_COUNTER( chunkEnd );
				m_DisplayRefreshSlots[ arm ] += sent;
				const u32 chunkCycles = (u32)( chunkEnd - chunkStart );
				if ( chunkCycles > m_DisplayScrubMaxChunkCycles[ arm ] )
					m_DisplayScrubMaxChunkCycles[ arm ] = chunkCycles;
			} while ( now < stop );
		}
		else if ( refresh && rwHold )
		{
			// The output latch is set before changing the GPIO direction, so R/W
			// can never glitch low. The VIC only reads during its owned cycles.
			// Restore high impedance before the first following physical access.
			SET_GPIO( bRW_OUT );
			OUT_GPIO_RW();
			m_DisplayRefreshStartOK[ arm ] = 1;
			while ( hostCycles() < stop ) asm volatile( "nop" );
			INP_GPIO_RW();
		}
		else if ( refresh && core0 )
		{
			m_DisplayRefreshStartOK[ arm ] = 1;
			u64 pulses = 0;
			do
			{
				pulses += core0RefreshFullFrame();
			} while ( hostCycles() < stop );
			m_DisplayRefreshSlots[ arm ] = pulses;
		}
		else if ( refresh )
		{
			c128RefreshForceContinuous( true );
			m_DisplayRefreshStartOK[ arm ] = c128RefreshStart() ? 1 : 0;
			const u64 slotsBefore = c128RefreshSlots();
			while ( hostCycles() < stop ) asm volatile( "nop" );
			m_DisplayRefreshSlots[ arm ] = c128RefreshSlots() - slotsBefore;
			c128RefreshStop();
			c128RefreshForceContinuous( false );
		}
		else
		{
			while ( hostCycles() < stop ) asm volatile( "nop" );
		}
		m_DisplayBoundaryWaitReads[ arm ][ 1 ] =
			displayBoundaryWaitRaster( true );
		m_Reads += m_DisplayBoundaryWaitReads[ arm ][ 1 ] & 0x7FFFFFFFu;
		displayFetchBlank( state );
		m_Writes += 2;
		m_DisplayBoundaryElapsedUsec[ arm ] =
			(u32)( ( hostCycles() - started )
			       / ( SCPU_ARM_CLOCK_HZ / 1000000u ) );

		m_DisplayBoundaryCaptureUsec[ arm ][ 1 ] =
			displayPersistenceCapture(
				salt, false, displayBoundaryMismatch[ arm ][ 1 ],
				m_DisplayBoundaryFirstExpected[ arm ][ 1 ],
				m_DisplayBoundaryFirstActual[ arm ][ 1 ],
				m_DisplayBoundaryXorAND[ arm ][ 1 ],
				m_DisplayBoundaryXorOR[ arm ][ 1 ] );
		displayAddressAnalyzeMap(
			displayBoundaryMismatch[ arm ][ 1 ],
			m_DisplayBoundaryErrors[ arm ][ 1 ],
			m_DisplayBoundaryFirst[ arm ][ 1 ],
			m_DisplayBoundaryLast[ arm ][ 1 ],
			m_DisplayBoundaryAND[ arm ][ 1 ],
			m_DisplayBoundaryOR[ arm ][ 1 ],
			m_DisplayBoundaryRuns[ arm ][ 1 ],
			m_DisplayBoundaryMaxRun[ arm ][ 1 ] );
		displaySentinelCompare(
			displayBoundaryMismatch[ arm ][ 0 ],
			displayBoundaryMismatch[ arm ][ 1 ],
			m_DisplayBoundarySame[ arm ],
			m_DisplayBoundaryAdded[ arm ],
			m_DisplayBoundaryRemoved[ arm ] );
		m_Reads += DISPLAY_SENTINEL_LENGTH + 2u;
	}
	c128RefreshStop();
	c128RefreshForceContinuous( false );
	return true;
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
	{
		const u32 i = m_ReadTimingRAMBestSample >= 300u
		           && m_ReadTimingRAMBestSample <= 620u
		           ? ( m_ReadTimingRAMBestSample - 300u ) / 5u : 0u;
		logger->Write( "RADbus", LogNotice,
		               "read observe: mixed RAM=%u/%u RAM-only=%u/%u mixed VIC err/dist/dom=%u/%u/%02Xx%u isolated=%u/%u/%02Xx%u",
		               (unsigned)m_ReadTimingRAMBestSample,
		               (unsigned)m_ReadTimingRAMBestError,
		               (unsigned)m_ReadTimingRAMOnlyBestSample,
		               (unsigned)m_ReadTimingRAMOnlyBestError,
		               (unsigned)m_ReadTimingMixedVICErrors[ i ],
		               (unsigned)m_ReadTimingMixedVICDistinct[ i ],
		               m_ReadTimingMixedVICDominantValue[ i ],
		               (unsigned)m_ReadTimingMixedVICDominantCount[ i ],
		               (unsigned)m_ReadTimingIsolatedVICErrors[ i ],
		               (unsigned)m_ReadTimingIsolatedVICDistinct[ i ],
		               m_ReadTimingIsolatedVICDominantValue[ i ],
		               (unsigned)m_ReadTimingIsolatedVICDominantCount[ i ] );
		logger->Write( "RADbus", LogNotice,
		               "read rotate @%u: position=%u,%u,%u,%u,%u,%u address=%u,%u,%u,%u,%u,%u",
		               (unsigned)m_ReadTimingRAMBestSample,
		               (unsigned)m_ReadTimingMixedRAMPositionErrors[ i ][ 0 ],
		               (unsigned)m_ReadTimingMixedRAMPositionErrors[ i ][ 1 ],
		               (unsigned)m_ReadTimingMixedRAMPositionErrors[ i ][ 2 ],
		               (unsigned)m_ReadTimingMixedRAMPositionErrors[ i ][ 3 ],
		               (unsigned)m_ReadTimingMixedRAMPositionErrors[ i ][ 4 ],
		               (unsigned)m_ReadTimingMixedRAMPositionErrors[ i ][ 5 ],
		               (unsigned)m_ReadTimingMixedRAMAddressErrors[ i ][ 0 ],
		               (unsigned)m_ReadTimingMixedRAMAddressErrors[ i ][ 1 ],
		               (unsigned)m_ReadTimingMixedRAMAddressErrors[ i ][ 2 ],
		               (unsigned)m_ReadTimingMixedRAMAddressErrors[ i ][ 3 ],
		               (unsigned)m_ReadTimingMixedRAMAddressErrors[ i ][ 4 ],
		               (unsigned)m_ReadTimingMixedRAMAddressErrors[ i ][ 5 ] );
	}
	if ( m_SnapshotKernalObserved || m_SnapshotBasicObserved )
		logger->Write( "RADbus", LogNotice,
		               "snapshot byte0: kernal=%02X/%02X/%02X basic=%02X/%02X/%02X (first/reread/copied)",
		               m_SnapshotKernalFirst, m_SnapshotKernalReread,
		               m_SnapshotKernalCopied, m_SnapshotBasicFirst,
		               m_SnapshotBasicReread, m_SnapshotBasicCopied );

	logger->Write( "RADbus", LogNotice,
	               "write timing: configured %u/%u pass=%u selected %u/%u",
	               (unsigned)m_WriteTimingConfiguredAddr,
	               (unsigned)m_WriteTimingConfiguredData,
	               (unsigned)m_WriteTimingPassingPoints,
	               (unsigned)m_WriteTimingSelectedAddr,
	               (unsigned)m_WriteTimingSelectedData );

	if ( m_SIDTimingEdge )
		logger->Write( "RADbus", LogNotice,
		               "SID timing: last PHI2-high selected distinct/dominant/ramp=%u/%u/%u",
		               (unsigned)m_SIDTimingDistinct,
		               (unsigned)m_SIDTimingDominant,
		               (unsigned)m_SIDTimingRamp );
	else if ( m_SIDTimingStart )
		logger->Write( "RADbus", LogNotice,
		               "SID timing: configured %u saw %u..%u selected %u distinct/dominant/ramp=%u/%u/%u",
		               (unsigned)m_SIDTimingConfigured,
		               (unsigned)m_SIDTimingStart,
		               (unsigned)m_SIDTimingEnd,
		               (unsigned)m_SIDTimingSelected,
		               (unsigned)m_SIDTimingDistinct,
		               (unsigned)m_SIDTimingDominant,
		               (unsigned)m_SIDTimingRamp );
	else
		logger->Write( "RADbus", LogNotice,
		               "SID timing: NO saw window; retained %u best %u distinct/dominant/ramp=%u/%u/%u",
		               (unsigned)m_SIDTimingConfigured,
		               (unsigned)m_SIDTimingBestSample,
		               (unsigned)m_SIDTimingDistinct,
		               (unsigned)m_SIDTimingDominant,
		               (unsigned)m_SIDTimingRamp );
	if ( !m_SIDPhysicalReliable )
		logger->Write( "RADbus", LogNotice,
		               "SID fallback: OSC3 host-clock model; POTX/POTY 17-read mode filter" );

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

void CRADBus::logAccessSentinelResults( CLogger *logger ) const
{
	if ( !logger ) return;
	if ( !m_AccessSentinelRan )
	{
		logger->Write( "RADbus", LogWarning,
		               "access sentinel: NOT RUN (requires acquired C64 bus)" );
		return;
	}

	static const char *armNames[ 3 ] = { "control", "read-D020", "write-D020" };
	logger->Write( "RADbus", LogNotice,
	               "access sentinel: bytes=%u traffic/arm=%u rate=16000/s two-pass oracle",
	               (unsigned)ACCESS_SENTINEL_LENGTH,
	               (unsigned)m_AccessSentinelTrafficCount );
	for ( u32 arm = 0; arm < 3; arm++ )
	{
		logger->Write( "RADbus", LogNotice,
		               "  %s base=%u/%u exposure=%u/%u arm retained/added/removed=%u/%u/%u verify same/new/cleared=%u/%u/%u elapsed-us=%u",
		               armNames[ arm ],
		               (unsigned)m_AccessSentinelBaselineErrors[ arm ][ 0 ],
		               (unsigned)m_AccessSentinelBaselineErrors[ arm ][ 1 ],
		               (unsigned)m_AccessSentinelExposureErrors[ arm ][ 0 ],
		               (unsigned)m_AccessSentinelExposureErrors[ arm ][ 1 ],
		               (unsigned)m_AccessSentinelBaselineRetainedErrors[ arm ],
		               (unsigned)m_AccessSentinelArmAddedErrors[ arm ],
		               (unsigned)m_AccessSentinelArmRemovedErrors[ arm ],
		               (unsigned)m_AccessSentinelSameErrors[ arm ],
		               (unsigned)m_AccessSentinelNewErrors[ arm ],
		               (unsigned)m_AccessSentinelClearedErrors[ arm ],
		               (unsigned)m_AccessSentinelElapsedUsec[ arm ] );
		logger->Write( "RADbus", LogNotice,
		               "    first pass0/pass1: %04X %02X>%02X / %04X %02X>%02X",
		               m_AccessSentinelFirstAddr[ arm ][ 0 ],
		               m_AccessSentinelFirstExpected[ arm ][ 0 ],
		               m_AccessSentinelFirstActual[ arm ][ 0 ],
		               m_AccessSentinelFirstAddr[ arm ][ 1 ],
		               m_AccessSentinelFirstExpected[ arm ][ 1 ],
		               m_AccessSentinelFirstActual[ arm ][ 1 ] );
	}
}

void CRADBus::logDisplaySentinelResults( CLogger *logger ) const
{
	if ( !logger ) return;
	if ( !m_DisplaySentinelRan )
	{
		logger->Write( "RADbus", LogWarning,
		               "display sentinel: NOT RUN (requires acquired C64 bus)" );
		return;
	}

	static const char *armNames[ 9 ] =
	{
		"on-none", "on-read-4k", "on-read-16k", "on-read-64k",
		"on-write-4k", "on-write-16k", "on-write-64k",
		"off-read-64k", "off-write-64k"
	};
	logger->Write( "RADbus", LogNotice,
	               "display sentinel K222: bytes=%u arms=9 exposure=8s blanked-readback=3",
	               (unsigned)DISPLAY_SENTINEL_LENGTH );
	for ( u32 arm = 0; arm < 9; arm++ )
	{
		logger->Write( "RADbus", LogNotice,
		               "  %s rate/count=%u/%u base=%u/%u exposure=%u/%u/%u arm kept/add/rem=%u/%u/%u verify01 s/n/c=%u/%u/%u verify12=%u/%u/%u us=%u",
		               armNames[ arm ],
		               (unsigned)m_DisplaySentinelRate[ arm ],
		               (unsigned)m_DisplaySentinelTrafficCount[ arm ],
		               (unsigned)m_DisplaySentinelBaselineErrors[ arm ][ 0 ],
		               (unsigned)m_DisplaySentinelBaselineErrors[ arm ][ 1 ],
		               (unsigned)m_DisplaySentinelExposureErrors[ arm ][ 0 ],
		               (unsigned)m_DisplaySentinelExposureErrors[ arm ][ 1 ],
		               (unsigned)m_DisplaySentinelExposureErrors[ arm ][ 2 ],
		               (unsigned)m_DisplaySentinelBaselineRetainedErrors[ arm ],
		               (unsigned)m_DisplaySentinelArmAddedErrors[ arm ],
		               (unsigned)m_DisplaySentinelArmRemovedErrors[ arm ],
		               (unsigned)m_DisplaySentinelVerifySameErrors[ arm ][ 0 ],
		               (unsigned)m_DisplaySentinelVerifyNewErrors[ arm ][ 0 ],
		               (unsigned)m_DisplaySentinelVerifyClearedErrors[ arm ][ 0 ],
		               (unsigned)m_DisplaySentinelVerifySameErrors[ arm ][ 1 ],
		               (unsigned)m_DisplaySentinelVerifyNewErrors[ arm ][ 1 ],
		               (unsigned)m_DisplaySentinelVerifyClearedErrors[ arm ][ 1 ],
		               (unsigned)m_DisplaySentinelElapsedUsec[ arm ] );
		logger->Write( "RADbus", LogNotice,
		               "    first p0/p1/p2: %04X %02X>%02X / %04X %02X>%02X / %04X %02X>%02X",
		               m_DisplaySentinelFirstAddr[ arm ][ 0 ],
		               m_DisplaySentinelFirstExpected[ arm ][ 0 ],
		               m_DisplaySentinelFirstActual[ arm ][ 0 ],
		               m_DisplaySentinelFirstAddr[ arm ][ 1 ],
		               m_DisplaySentinelFirstExpected[ arm ][ 1 ],
		               m_DisplaySentinelFirstActual[ arm ][ 1 ],
		               m_DisplaySentinelFirstAddr[ arm ][ 2 ],
		               m_DisplaySentinelFirstExpected[ arm ][ 2 ],
		               m_DisplaySentinelFirstActual[ arm ][ 2 ] );
	}
}

void CRADBus::logDisplayAddressResults( CLogger *logger ) const
{
	if ( !logger ) return;
	if ( !m_DisplayAddressRan )
	{
		logger->Write( "RADbus", LogWarning,
		               "display address K223: NOT RUN (requires acquired C64 bus)" );
		return;
	}

	static const char *modeNames[ 2 ] = { "text-DEN0", "bitmap-DEN0" };
	static const char *styleNames[ 2 ] = { "single", "double" };
	logger->Write( "RADbus", LogNotice,
	               "display address K223: target=$2078 immediate=256 trials; bulk maps=9216 bytes" );
	for ( u32 mode = 0; mode < 2; mode++ )
	{
		for ( u32 style = 0; style < 2; style++ )
		{
			const u16 fixedMask = m_DisplayAddressMapErrors[ mode ][ style ]
				? (u16)~( m_DisplayAddressMapAND[ mode ][ style ]
				          ^ m_DisplayAddressMapOR[ mode ][ style ] )
				: 0;
			logger->Write( "RADbus", LogNotice,
			               "  %s %s immediate r0/r1/r2=%u/%u/%u first=%u exp=%02X got=%02X/%02X/%02X map=%u first/last=%04X/%04X and/or=%04X/%04X fixed=%04X:%04X runs/max=%u/%u",
			               modeNames[ mode ], styleNames[ style ],
			               (unsigned)m_DisplayAddressImmediateErrors[ mode ][ style ][ 0 ],
			               (unsigned)m_DisplayAddressImmediateErrors[ mode ][ style ][ 1 ],
			               (unsigned)m_DisplayAddressImmediateErrors[ mode ][ style ][ 2 ],
			               (unsigned)m_DisplayAddressImmediateFirstTrial[ mode ][ style ],
			               m_DisplayAddressImmediateFirstExpected[ mode ][ style ],
			               m_DisplayAddressImmediateFirstActual[ mode ][ style ][ 0 ],
			               m_DisplayAddressImmediateFirstActual[ mode ][ style ][ 1 ],
			               m_DisplayAddressImmediateFirstActual[ mode ][ style ][ 2 ],
			               (unsigned)m_DisplayAddressMapErrors[ mode ][ style ],
			               m_DisplayAddressMapFirst[ mode ][ style ],
			               m_DisplayAddressMapLast[ mode ][ style ],
			               m_DisplayAddressMapAND[ mode ][ style ],
			               m_DisplayAddressMapOR[ mode ][ style ],
			               fixedMask,
			               (u16)( m_DisplayAddressMapAND[ mode ][ style ] & fixedMask ),
			               m_DisplayAddressMapRuns[ mode ][ style ],
			               m_DisplayAddressMapMaxRun[ mode ][ style ] );
		}
	}
	logger->Write( "RADbus", LogNotice, "  delay ladder: %s",
	               m_DisplayAddressLadderRan ? "ran" : "skipped (immediate r2 failed)" );
	if ( m_DisplayAddressLadderRan )
	{
		for ( u32 mode = 0; mode < 2; mode++ )
			for ( u32 rung = 0; rung < 9; rung++ )
				logger->Write( "RADbus", LogNotice,
				               "    %s %uus/%uus exp=%02X initial=%02X/%02X/%02X delayed=%02X/%02X/%02X",
				               modeNames[ mode ],
				               (unsigned)m_DisplayAddressDelayUsec[ rung ],
				               (unsigned)m_DisplayAddressElapsedUsec[ mode ][ rung ],
				               m_DisplayAddressLadderExpected[ mode ][ rung ],
				               m_DisplayAddressLadderInitial[ mode ][ rung ][ 0 ],
				               m_DisplayAddressLadderInitial[ mode ][ rung ][ 1 ],
				               m_DisplayAddressLadderInitial[ mode ][ rung ][ 2 ],
				               m_DisplayAddressLadderDelayed[ mode ][ rung ][ 0 ],
				               m_DisplayAddressLadderDelayed[ mode ][ rung ][ 1 ],
				               m_DisplayAddressLadderDelayed[ mode ][ rung ][ 2 ] );
	}
}

void CRADBus::logDisplayRowResults( CLogger *logger ) const
{
	if ( !logger ) return;
	if ( !m_DisplayRowRan )
	{
		logger->Write( "RADbus", LogWarning,
		               "display row K225: NOT RUN (requires acquired C64 bus)" );
		return;
	}
	logger->Write( "RADbus", LogNotice,
	               "display row K225: 10 high-entropy write maps + 6 independent retention maps" );
	for ( u32 arm = 0; arm < DISPLAY_ROW_ARM_COUNT; arm++ )
	{
		const u16 fixedMask = m_DisplayRowErrors[ arm ][ 1 ]
			? (u16)~( m_DisplayRowAND[ arm ][ 1 ] ^ m_DisplayRowOR[ arm ][ 1 ] )
			: 0;
		logger->Write( "RADbus", LogNotice,
		               "  arm%u %s-%s salt=%02X pre-attempt/errors=%u/%u test-errors=%u first/last=%04X/%04X and/or=%04X/%04X fixed=%04X:%04X runs/max=%u/%u seed-us=%u/%u capture-us=%u/%u",
		               (unsigned)arm,
		               m_DisplayRowMode[ arm ] ? "bitmap" : "text",
		               m_DisplayRowKind[ arm ] ? "repeat" : "single",
		               m_DisplayRowSalt[ arm ],
		               m_DisplayRowPrefillAttempts[ arm ],
		               (unsigned)m_DisplayRowErrors[ arm ][ 0 ],
		               (unsigned)m_DisplayRowErrors[ arm ][ 1 ],
		               m_DisplayRowFirst[ arm ][ 1 ],
		               m_DisplayRowLast[ arm ][ 1 ],
		               m_DisplayRowAND[ arm ][ 1 ],
		               m_DisplayRowOR[ arm ][ 1 ], fixedMask,
		               (u16)( m_DisplayRowAND[ arm ][ 1 ] & fixedMask ),
		               m_DisplayRowRuns[ arm ][ 1 ],
		               m_DisplayRowMaxRun[ arm ][ 1 ],
		               (unsigned)m_DisplayRowSeedUsec[ arm ][ 0 ],
		               (unsigned)m_DisplayRowSeedUsec[ arm ][ 1 ],
		               (unsigned)m_DisplayRowCaptureUsec[ arm ][ 0 ],
		               (unsigned)m_DisplayRowCaptureUsec[ arm ][ 1 ] );
	}
	for ( u32 rung = 0; rung < DISPLAY_ROW_RETENTION_COUNT; rung++ )
	{
		const u16 fixedMask = m_DisplayRowRetentionErrors[ rung ][ 1 ]
			? (u16)~( m_DisplayRowRetentionAND[ rung ][ 1 ]
			          ^ m_DisplayRowRetentionOR[ rung ][ 1 ] )
			: 0;
		logger->Write( "RADbus", LogNotice,
		               "  retain%u delay/actual=%u/%u salt=%02X attempts=%u base/post=%u/%u first/last=%04X/%04X and/or=%04X/%04X fixed=%04X:%04X runs/max=%u/%u seed-us=%u capture-us=%u/%u",
		               (unsigned)rung,
		               (unsigned)m_DisplayRowRetentionDelayUsec[ rung ],
		               (unsigned)m_DisplayRowRetentionElapsedUsec[ rung ],
		               m_DisplayRowRetentionSalt[ rung ],
		               m_DisplayRowRetentionAttempts[ rung ],
		               (unsigned)m_DisplayRowRetentionErrors[ rung ][ 0 ],
		               (unsigned)m_DisplayRowRetentionErrors[ rung ][ 1 ],
		               m_DisplayRowRetentionFirst[ rung ][ 1 ],
		               m_DisplayRowRetentionLast[ rung ][ 1 ],
		               m_DisplayRowRetentionAND[ rung ][ 1 ],
		               m_DisplayRowRetentionOR[ rung ][ 1 ], fixedMask,
		               (u16)( m_DisplayRowRetentionAND[ rung ][ 1 ] & fixedMask ),
		               m_DisplayRowRetentionRuns[ rung ][ 1 ],
		               m_DisplayRowRetentionMaxRun[ rung ][ 1 ],
		               (unsigned)m_DisplayRowRetentionSeedUsec[ rung ],
		               (unsigned)m_DisplayRowRetentionCaptureUsec[ rung ][ 0 ],
		               (unsigned)m_DisplayRowRetentionCaptureUsec[ rung ][ 1 ] );
	}
}

void CRADBus::logDisplayFetchResults( CLogger *logger ) const
{
	if ( !logger ) return;
	if ( !m_DisplayFetchRan )
	{
		logger->Write( "RADbus", LogWarning,
		               "display fetch K226: NOT RUN (requires acquired C64 bus)" );
		return;
	}
	static const char *stateNames[ 3 ] = { "off", "text", "bitmap" };
	static const char *operationNames[ 3 ] = { "none", "read", "write" };
	logger->Write( "RADbus", LogNotice,
	               "display fetch K226: 9 active-fetch arms + 6 single-write retention controls" );
	for ( u32 arm = 0; arm < DISPLAY_FETCH_ARM_COUNT; arm++ )
	{
		logger->Write( "RADbus", LogNotice,
		               "  arm%u %s-%s salt=%02X rate/count=%u/%u pre/base/post=%u/%u/%u elapsed=%u seed-us=%u/%u capture-us=%u/%u/%u",
		               (unsigned)arm, stateNames[ m_DisplayFetchState[ arm ] ],
		               operationNames[ m_DisplayFetchOperation[ arm ] ],
		               m_DisplayFetchSalt[ arm ],
		               (unsigned)m_DisplayFetchRate[ arm ],
		               (unsigned)m_DisplayFetchTrafficCount[ arm ],
		               (unsigned)m_DisplayFetchErrors[ arm ][ 0 ],
		               (unsigned)m_DisplayFetchErrors[ arm ][ 1 ],
		               (unsigned)m_DisplayFetchErrors[ arm ][ 2 ],
		               (unsigned)m_DisplayFetchElapsedUsec[ arm ],
		               (unsigned)m_DisplayFetchSeedUsec[ arm ][ 0 ],
		               (unsigned)m_DisplayFetchSeedUsec[ arm ][ 1 ],
		               (unsigned)m_DisplayFetchCaptureUsec[ arm ][ 0 ],
		               (unsigned)m_DisplayFetchCaptureUsec[ arm ][ 1 ],
		               (unsigned)m_DisplayFetchCaptureUsec[ arm ][ 2 ] );
	}
	for ( u32 rung = 0; rung < DISPLAY_FETCH_RETENTION_COUNT; rung++ )
	{
		logger->Write( "RADbus", LogNotice,
		               "  retain%u delay/actual=%u/%u salt=%02X attempts=%u pre/base/post=%u/%u/%u seed-us=%u/%u capture-us=%u/%u/%u",
		               (unsigned)rung,
		               (unsigned)m_DisplayFetchRetentionDelayUsec[ rung ],
		               (unsigned)m_DisplayFetchRetentionElapsedUsec[ rung ],
		               m_DisplayFetchRetentionSalt[ rung ],
		               m_DisplayFetchRetentionPrefillAttempts[ rung ],
		               (unsigned)m_DisplayFetchRetentionErrors[ rung ][ 0 ],
		               (unsigned)m_DisplayFetchRetentionErrors[ rung ][ 1 ],
		               (unsigned)m_DisplayFetchRetentionErrors[ rung ][ 2 ],
		               (unsigned)m_DisplayFetchRetentionSeedUsec[ rung ][ 0 ],
		               (unsigned)m_DisplayFetchRetentionSeedUsec[ rung ][ 1 ],
		               (unsigned)m_DisplayFetchRetentionCaptureUsec[ rung ][ 0 ],
		               (unsigned)m_DisplayFetchRetentionCaptureUsec[ rung ][ 1 ],
		               (unsigned)m_DisplayFetchRetentionCaptureUsec[ rung ][ 2 ] );
	}
}

void CRADBus::logDisplayPersistenceResults( CLogger *logger ) const
{
	if ( !logger ) return;
	if ( !m_DisplayPersistenceRan )
	{
		logger->Write( "RADbus", LogWarning,
		               "display persistence K227: NOT RUN (requires acquired C64 bus)" );
		return;
	}
	static const char *stateNames[ 3 ] = { "off", "text", "bitmap" };
	static const char *operationNames[ 3 ] = { "none", "read", "write" };
	logger->Write( "RADbus", LogNotice,
	               "display persistence K227: 12 repeated active-fetch arms, 3 post captures" );
	for ( u32 arm = 0; arm < DISPLAY_PERSISTENCE_ARM_COUNT; arm++ )
		logger->Write( "RADbus", LogNotice,
		               "  arm%u %s-%s salt=%02X errors pre/base/p0/p1/p2=%u/%u/%u/%u/%u persist01 s/a/r=%u/%u/%u persist12=%u/%u/%u",
		               (unsigned)arm,
		               stateNames[ m_DisplayPersistenceState[ arm ] ],
		               operationNames[ m_DisplayPersistenceOperation[ arm ] ],
		               m_DisplayPersistenceSalt[ arm ],
		               (unsigned)m_DisplayPersistenceErrors[ arm ][ 0 ],
		               (unsigned)m_DisplayPersistenceErrors[ arm ][ 1 ],
		               (unsigned)m_DisplayPersistenceErrors[ arm ][ 2 ],
		               (unsigned)m_DisplayPersistenceErrors[ arm ][ 3 ],
		               (unsigned)m_DisplayPersistenceErrors[ arm ][ 4 ],
		               (unsigned)m_DisplayPersistenceSame[ arm ][ 0 ],
		               (unsigned)m_DisplayPersistenceAdded[ arm ][ 0 ],
		               (unsigned)m_DisplayPersistenceRemoved[ arm ][ 0 ],
		               (unsigned)m_DisplayPersistenceSame[ arm ][ 1 ],
		               (unsigned)m_DisplayPersistenceAdded[ arm ][ 1 ],
		               (unsigned)m_DisplayPersistenceRemoved[ arm ][ 1 ] );
}

void CRADBus::logDisplayTimingResults( CLogger *logger ) const
{
	if ( !logger ) return;
	if ( !m_DisplayTimingRan )
	{
		logger->Write( "RADbus", LogWarning,
		               "display timing K228: NOT RUN (requires acquired C64 bus)" );
		return;
	}
	static const char *stateNames[ 3 ] = { "off", "text", "bitmap" };
	static const char *operationNames[ 3 ] = { "none", "read", "write" };
	logger->Write( "RADbus", LogNotice,
	               "display timing K228: multi-sample readback and targeted repair" );
	for ( u32 arm = 0; arm < DISPLAY_TIMING_ARM_COUNT; arm++ )
		logger->Write( "RADbus", LogNotice,
		               "  arm%u %s-%s salt=%02X pre/base=%u/%u initial/final/repaired=%u/%u/%u repair-writes=%u",
		               (unsigned)arm, stateNames[ m_DisplayTimingState[ arm ] ],
		               operationNames[ m_DisplayTimingOperation[ arm ] ],
		               m_DisplayTimingSalt[ arm ],
		               (unsigned)m_DisplayTimingPrefillErrors[ arm ],
		               (unsigned)m_DisplayTimingBaselineErrors[ arm ],
		               (unsigned)m_DisplayTimingErrors[ arm ][ 0 ],
		               (unsigned)m_DisplayTimingErrors[ arm ][ 8 ],
		               (unsigned)m_DisplayTimingErrors[ arm ][ 9 ],
		               (unsigned)m_DisplayTimingRepairWrites[ arm ] );
}

void CRADBus::logDisplayBoundaryResults( CLogger *logger ) const
{
	if ( !logger ) return;
	if ( !m_DisplayBoundaryRan )
	{
		logger->Write( "RADbus", LogWarning,
		               "display boundary K229: NOT RUN (requires acquired C64 bus)" );
		return;
	}
	logger->Write( "RADbus", LogNotice,
	               "display boundary K229: transition-only versus four-second dwell" );
	for ( u32 arm = 0; arm < DISPLAY_BOUNDARY_ARM_COUNT; arm++ )
		logger->Write( "RADbus", LogNotice,
		               "  arm%u %s-%s-%s salt=%02X pre/base/post=%u/%u/%u added=%u waits=%u/%u",
		               (unsigned)arm,
		               m_DisplayBoundaryState[ arm ] == 1 ? "text" : "bitmap",
		               m_DisplayBoundarySafe[ arm ] ? "border" : "visible",
		               m_DisplayBoundaryDwell[ arm ] ? "dwell" : "transition",
		               m_DisplayBoundarySalt[ arm ],
		               (unsigned)m_DisplayBoundaryPrefillErrors[ arm ],
		               (unsigned)m_DisplayBoundaryErrors[ arm ][ 0 ],
		               (unsigned)m_DisplayBoundaryErrors[ arm ][ 1 ],
		               (unsigned)m_DisplayBoundaryAdded[ arm ],
		               (unsigned)m_DisplayBoundaryWaitReads[ arm ][ 0 ],
		               (unsigned)m_DisplayBoundaryWaitReads[ arm ][ 1 ] );
}

void CRADBus::logDisplayRefreshResults( CLogger *logger ) const
{
	if ( !logger ) return;
	if ( !m_DisplayRefreshRan )
	{
		logger->Write( "RADbus", LogWarning,
		               "display refresh %s: NOT RUN (requires acquired C64 bus)",
		               m_DisplayDiagnosticVariant == 3 ? "K233"
		             : m_DisplayDiagnosticVariant == 2 ? "K232"
		             : m_DisplayDiagnosticVariant == 1 ? "K231" : "K230" );
		return;
	}
	logger->Write( "RADbus", LogNotice,
	               m_DisplayDiagnosticVariant == 3
	                 ? "display scrub K233: control versus masked shadow repair"
	               : m_DisplayDiagnosticVariant == 2
	                 ? "display R/W K232: floating versus actively-held-high idle"
	                 : m_DisplayDiagnosticVariant == 1
	                 ? "display refresh K231: sustained-DMA versus core0 VIC-half release"
	                 : "display refresh K230: sustained-DMA versus core3 VIC-half release" );
	for ( u32 arm = 0; arm < 8; arm++ )
		logger->Write( "RADbus", LogNotice,
		               "  arm%u %s intervention=%u started=%u salt=%02X pre/base/post=%u/%u/%u added=%u bytes/opp/maxcycles=%llu/%u/%u",
		               (unsigned)arm,
		               m_DisplayBoundaryState[ arm ] == 1 ? "text" : "bitmap",
		               (unsigned)m_DisplayRefreshEnabled[ arm ],
		               (unsigned)m_DisplayRefreshStartOK[ arm ],
		               m_DisplayBoundarySalt[ arm ],
		               (unsigned)m_DisplayBoundaryPrefillErrors[ arm ],
		               (unsigned)m_DisplayBoundaryErrors[ arm ][ 0 ],
		               (unsigned)m_DisplayBoundaryErrors[ arm ][ 1 ],
		               (unsigned)m_DisplayBoundaryAdded[ arm ],
		               (unsigned long long)m_DisplayRefreshSlots[ arm ],
		               (unsigned)m_DisplayScrubOpportunities[ arm ],
		               (unsigned)m_DisplayScrubMaxChunkCycles[ arm ] );
}

void CRADBus::logDisplayBAGuardResults( CLogger *logger ) const
{
	if ( !logger ) return;
	if ( !m_DisplayBAGuardRan )
	{
		logger->Write( "RADbus", LogWarning,
		               "display BA guard K239: NOT RUN (requires acquired C64 bus)" );
		return;
	}
	static const char *operationNames[ 3 ] = { "idle", "read", "write" };
	logger->Write( "RADbus", LogNotice,
	               "display BA guard K239: identical composite bitmap, repeated timed arms" );
	logger->Write( "RADbus", LogNotice,
	               "  read eye best=%u errors=%u configured/selected=%u/%u stable=%u..%u",
	               (unsigned)m_ReadTimingBestErrorSample,
	               (unsigned)m_ReadTimingBestError,
	               (unsigned)m_ReadTimingConfigured,
	               (unsigned)m_ReadTimingSelected,
	               (unsigned)m_ReadTimingStart, (unsigned)m_ReadTimingEnd );
	for ( u32 arm = 0; arm < DISPLAY_BA_ARM_COUNT; arm++ )
		logger->Write( "RADbus", LogNotice,
		               "  arm%u %s bias=%u rot=%u base p/u=%u/%u post p/u/unstable=%u/%u/%u added=%u families matrix/00/ff/alt/entropy=%u/%u/%u/%u/%u",
		               (unsigned)arm, operationNames[ m_DisplayBAOperation[ arm ] ],
		               (unsigned)m_DisplayBABias[ arm ],
		               (unsigned)m_DisplayBARotation[ arm ],
		               (unsigned)m_DisplayBABaselinePersistent[ arm ],
		               (unsigned)m_DisplayBABaselineUnion[ arm ],
		               (unsigned)m_DisplayBAPostPersistent[ arm ],
		               (unsigned)m_DisplayBAPostUnion[ arm ],
		               (unsigned)m_DisplayBAUnstableOnly[ arm ],
		               (unsigned)m_DisplayBAAddedPersistent[ arm ],
		               (unsigned)m_DisplayBAFamilyAdded[ arm ][ 0 ],
		               (unsigned)m_DisplayBAFamilyAdded[ arm ][ 1 ],
		               (unsigned)m_DisplayBAFamilyAdded[ arm ][ 2 ],
		               (unsigned)m_DisplayBAFamilyAdded[ arm ][ 3 ],
		               (unsigned)m_DisplayBAFamilyAdded[ arm ][ 4 ] );
}

static void displayRowFormatMapBits( char *dst, u32 capacity, u32 &n,
	                                  const u8 *mismatch )
{
	for ( u32 byte = 0; byte < DISPLAY_SENTINEL_BITMAP_BYTES; byte++ )
	{
		if ( ( byte & 31u ) == 0 ) busDiagEndLine( dst, capacity, n );
		busDiagHex( dst, capacity, n, mismatch[ byte ] );
	}
	busDiagEndLine( dst, capacity, n );
}

void CRADBus::formatDisplayBAGuardResults( char *dst, u32 capacity,
	                                        u32 &n ) const
{
	static const char *operationNames[ 3 ] = { "idle", "read-D020", "write-D020" };
	busDiagText( dst, capacity, n,
	             "protocol: C64-DISPLAY-BA-GUARD-K239 ranges=0400-07FF,2000-3FFF bytes=9216 mode=bitmap exposure-seconds=4 traffic-rate=16000 oracle=interleaved-64-byte-families-matrix,00,FF,alternating,entropy rotation=0,1 arms=idle,w40,read,w0,w60,w20,w20,idle,w60,read,w0,w40 write-address-enable-bias=0,20,40,60 bias-applies-to-production-address-parameter data-enable-unchanged triple-primed-baseline-and-post persistent=intersection union=or added=post-intersection-minus-baseline-union diagnostic_only=1\r\n" );
	for ( u32 arm = 0; arm < DISPLAY_BA_ARM_COUNT; arm++ )
	{
		busDiagText( dst, capacity, n,
		             "ba arm/op/bias/rotation/prefill-attempts/target-attempts/count/elapsed-us/base-persistent/base-union/post-persistent/post-union/post-unstable/added/family-matrix/00/ff/alt/entropy: " );
		const u32 first[ 6 ] =
		{
			arm, m_DisplayBABias[ arm ], m_DisplayBARotation[ arm ],
			m_DisplayBAPrefillAttempts[ arm ],
			m_DisplayBATargetAttempts[ arm ], m_DisplayBATrafficCount[ arm ]
		};
		busDiagDecimal( dst, capacity, n, first[ 0 ] );
		busDiagChar( dst, capacity, n, '/' );
		busDiagText( dst, capacity, n,
		             operationNames[ m_DisplayBAOperation[ arm ] ] );
		for ( u32 i = 1; i < 6; i++ )
		{
			busDiagChar( dst, capacity, n, '/' );
			busDiagDecimal( dst, capacity, n, first[ i ] );
		}
		const u32 scores[ 12 ] =
		{
			m_DisplayBAElapsedUsec[ arm ],
			m_DisplayBABaselinePersistent[ arm ],
			m_DisplayBABaselineUnion[ arm ],
			m_DisplayBAPostPersistent[ arm ],
			m_DisplayBAPostUnion[ arm ],
			m_DisplayBAUnstableOnly[ arm ],
			m_DisplayBAAddedPersistent[ arm ],
			m_DisplayBAFamilyAdded[ arm ][ 0 ],
			m_DisplayBAFamilyAdded[ arm ][ 1 ],
			m_DisplayBAFamilyAdded[ arm ][ 2 ],
			m_DisplayBAFamilyAdded[ arm ][ 3 ],
			m_DisplayBAFamilyAdded[ arm ][ 4 ]
		};
		for ( u32 i = 0; i < 12; i++ )
		{
			busDiagChar( dst, capacity, n, '/' );
			busDiagDecimal( dst, capacity, n, scores[ i ] );
		}
		busDiagEndLine( dst, capacity, n );
		busDiagText( dst, capacity, n,
		             "  capture-us base0/base1/base2/post0/post1/post2: " );
		for ( u32 pass = 0; pass < 6; pass++ )
		{
			if ( pass ) busDiagChar( dst, capacity, n, '/' );
			busDiagDecimal( dst, capacity, n,
			                m_DisplayBACaptureUsec[ arm ][ pass ] );
		}
		busDiagEndLine( dst, capacity, n );
		if ( m_DisplayBAAddedPersistent[ arm ] )
		{
			busDiagText( dst, capacity, n, "  added-map-bits:" );
			displayRowFormatMapBits( dst, capacity, n,
			                         displayBAAddedMap[ arm ] );
		}
	}
}

static void displayRowFormatSummary( char *dst, u32 capacity, u32 &n,
	                                  const char *label, u32 errors,
	                                  u16 first, u16 last, u16 addrAND,
	                                  u16 addrOR, u16 runs, u16 maxRun )
{
	const u16 fixedMask = errors ? (u16)~( addrAND ^ addrOR ) : 0;
	busDiagText( dst, capacity, n, label );
	busDiagText( dst, capacity, n,
	             " errors/first/last/and/or/fixed-mask/fixed-value/runs/max-run: " );
	busDiagDecimal( dst, capacity, n, errors );
	const u16 values[ 8 ] =
		{ first, last, addrAND, addrOR, fixedMask,
		  (u16)( addrAND & fixedMask ), runs, maxRun };
	for ( u32 i = 0; i < 8; i++ )
	{
		busDiagChar( dst, capacity, n, '/' );
		busDiagHexWord( dst, capacity, n, values[ i ] );
	}
	busDiagEndLine( dst, capacity, n );
}

void CRADBus::formatDisplayRowResults( char *dst, u32 capacity, u32 &n ) const
{
	busDiagText( dst, capacity, n,
	             "protocol: C64-DISPLAY-ROW-K225 ranges=0400-07FF,2000-3FFF bytes=9216 oracle=address-salt-high-entropy prefill=complement-adjacent-repeat-verified single-arms=8 mode-order=text,bitmap,bitmap,text,bitmap,text,text,bitmap repeat-arms=text,bitmap retention-mode=bitmap-DEN0 retention-us=500000,1000000,2000000,4000000,8000000,16000000 raw-maps=errors-only diagnostic_only=1\r\n" );
	for ( u32 arm = 0; arm < DISPLAY_ROW_ARM_COUNT; arm++ )
	{
		busDiagText( dst, capacity, n, "row-map arm/mode/kind/salt/prefill-attempts: " );
		busDiagDecimal( dst, capacity, n, arm );
		busDiagChar( dst, capacity, n, '/' );
		busDiagText( dst, capacity, n,
		             m_DisplayRowMode[ arm ] ? "bitmap" : "text" );
		busDiagChar( dst, capacity, n, '/' );
		busDiagText( dst, capacity, n,
		             m_DisplayRowKind[ arm ] ? "repeat" : "single" );
		busDiagChar( dst, capacity, n, '/' );
		busDiagHex( dst, capacity, n, m_DisplayRowSalt[ arm ] );
		busDiagChar( dst, capacity, n, '/' );
		busDiagDecimal( dst, capacity, n, m_DisplayRowPrefillAttempts[ arm ] );
		busDiagEndLine( dst, capacity, n );
		busDiagText( dst, capacity, n,
		             "  seed-us prefill/test capture-us prefill/test: " );
		busDiagDecimal( dst, capacity, n, m_DisplayRowSeedUsec[ arm ][ 0 ] );
		busDiagChar( dst, capacity, n, '/' );
		busDiagDecimal( dst, capacity, n, m_DisplayRowSeedUsec[ arm ][ 1 ] );
		busDiagChar( dst, capacity, n, '/' );
		busDiagDecimal( dst, capacity, n, m_DisplayRowCaptureUsec[ arm ][ 0 ] );
		busDiagChar( dst, capacity, n, '/' );
		busDiagDecimal( dst, capacity, n, m_DisplayRowCaptureUsec[ arm ][ 1 ] );
		busDiagEndLine( dst, capacity, n );
		for ( u32 phase = 0; phase < 2; phase++ )
		{
			displayRowFormatSummary(
				dst, capacity, n, phase ? "  test" : "  prefill",
				m_DisplayRowErrors[ arm ][ phase ],
				m_DisplayRowFirst[ arm ][ phase ],
				m_DisplayRowLast[ arm ][ phase ],
				m_DisplayRowAND[ arm ][ phase ],
				m_DisplayRowOR[ arm ][ phase ],
				m_DisplayRowRuns[ arm ][ phase ],
				m_DisplayRowMaxRun[ arm ][ phase ] );
			if ( m_DisplayRowErrors[ arm ][ phase ] )
			{
				busDiagText( dst, capacity, n,
				             phase ? "  test-map-bits:" : "  prefill-map-bits:" );
				displayRowFormatMapBits(
					dst, capacity, n,
					phase ? displayRowTestMismatch[ arm ]
					      : displayRowPrefillMismatch[ arm ] );
			}
		}
	}
	for ( u32 rung = 0; rung < DISPLAY_ROW_RETENTION_COUNT; rung++ )
	{
		busDiagText( dst, capacity, n,
		             "row-retention rung/salt/attempts/requested-us/elapsed-us/seed-us/capture-base/post: " );
		busDiagDecimal( dst, capacity, n, rung );
		busDiagChar( dst, capacity, n, '/' );
		busDiagHex( dst, capacity, n, m_DisplayRowRetentionSalt[ rung ] );
		busDiagChar( dst, capacity, n, '/' );
		busDiagDecimal( dst, capacity, n, m_DisplayRowRetentionAttempts[ rung ] );
		busDiagChar( dst, capacity, n, '/' );
		busDiagDecimal( dst, capacity, n, m_DisplayRowRetentionDelayUsec[ rung ] );
		busDiagChar( dst, capacity, n, '/' );
		busDiagDecimal( dst, capacity, n, m_DisplayRowRetentionElapsedUsec[ rung ] );
		busDiagChar( dst, capacity, n, '/' );
		busDiagDecimal( dst, capacity, n, m_DisplayRowRetentionSeedUsec[ rung ] );
		for ( u32 phase = 0; phase < 2; phase++ )
		{
			busDiagChar( dst, capacity, n, '/' );
			busDiagDecimal( dst, capacity, n,
			                m_DisplayRowRetentionCaptureUsec[ rung ][ phase ] );
		}
		busDiagEndLine( dst, capacity, n );
		for ( u32 phase = 0; phase < 2; phase++ )
		{
			displayRowFormatSummary(
				dst, capacity, n, phase ? "  post" : "  baseline",
				m_DisplayRowRetentionErrors[ rung ][ phase ],
				m_DisplayRowRetentionFirst[ rung ][ phase ],
				m_DisplayRowRetentionLast[ rung ][ phase ],
				m_DisplayRowRetentionAND[ rung ][ phase ],
				m_DisplayRowRetentionOR[ rung ][ phase ],
				m_DisplayRowRetentionRuns[ rung ][ phase ],
				m_DisplayRowRetentionMaxRun[ rung ][ phase ] );
			if ( m_DisplayRowRetentionErrors[ rung ][ phase ] )
			{
				busDiagText( dst, capacity, n,
				             phase ? "  post-map-bits:" : "  baseline-map-bits:" );
				displayRowFormatMapBits(
					dst, capacity, n,
					phase ? displayRowRetentionMismatch[ rung ]
					      : displayRowRetentionBaselineMismatch[ rung ] );
			}
		}
	}
}

void CRADBus::formatDisplayFetchResults( char *dst, u32 capacity, u32 &n ) const
{
	static const char *stateNames[ 3 ] = { "off", "text", "bitmap" };
	static const char *operationNames[ 3 ] = { "none", "read", "write" };
	static const char *phaseNames[ 3 ] = { "  prefill", "  baseline", "  post" };
	busDiagText( dst, capacity, n,
	             "protocol: C64-DISPLAY-FETCH-K226 ranges=0400-07FF,2000-3FFF bytes=9216 oracle=address-salt-high-entropy setup=DEN0-complement-adjacent-repeat-verified target=single-write exposure-seconds=4 traffic-rate=16000 states=off,text,bitmap operations=none,read-D020,write-same-D020 retention=single-write-DEN0 delays-us=500000,1000000,2000000,4000000,8000000,16000000 raw-maps=errors-only diagnostic_only=1\r\n" );
	for ( u32 arm = 0; arm < DISPLAY_FETCH_ARM_COUNT; arm++ )
	{
		busDiagText( dst, capacity, n,
		             "fetch arm/state/op/salt/prefill-attempts/rate/count/elapsed-us: " );
		busDiagDecimal( dst, capacity, n, arm );
		busDiagChar( dst, capacity, n, '/' );
		busDiagText( dst, capacity, n, stateNames[ m_DisplayFetchState[ arm ] ] );
		busDiagChar( dst, capacity, n, '/' );
		busDiagText( dst, capacity, n,
		             operationNames[ m_DisplayFetchOperation[ arm ] ] );
		busDiagChar( dst, capacity, n, '/' );
		busDiagHex( dst, capacity, n, m_DisplayFetchSalt[ arm ] );
		busDiagChar( dst, capacity, n, '/' );
		busDiagDecimal( dst, capacity, n, m_DisplayFetchPrefillAttempts[ arm ] );
		busDiagChar( dst, capacity, n, '/' );
		busDiagDecimal( dst, capacity, n, m_DisplayFetchRate[ arm ] );
		busDiagChar( dst, capacity, n, '/' );
		busDiagDecimal( dst, capacity, n, m_DisplayFetchTrafficCount[ arm ] );
		busDiagChar( dst, capacity, n, '/' );
		busDiagDecimal( dst, capacity, n, m_DisplayFetchElapsedUsec[ arm ] );
		busDiagEndLine( dst, capacity, n );
		busDiagText( dst, capacity, n,
		             "  seed-us complement/target capture-us prefill/baseline/post: " );
		busDiagDecimal( dst, capacity, n, m_DisplayFetchSeedUsec[ arm ][ 0 ] );
		busDiagChar( dst, capacity, n, '/' );
		busDiagDecimal( dst, capacity, n, m_DisplayFetchSeedUsec[ arm ][ 1 ] );
		for ( u32 phase = 0; phase < 3; phase++ )
		{
			busDiagChar( dst, capacity, n, '/' );
			busDiagDecimal( dst, capacity, n,
			                m_DisplayFetchCaptureUsec[ arm ][ phase ] );
		}
		busDiagEndLine( dst, capacity, n );
		for ( u32 phase = 0; phase < 3; phase++ )
		{
			displayRowFormatSummary(
				dst, capacity, n, phaseNames[ phase ],
				m_DisplayFetchErrors[ arm ][ phase ],
				m_DisplayFetchFirst[ arm ][ phase ],
				m_DisplayFetchLast[ arm ][ phase ],
				m_DisplayFetchAND[ arm ][ phase ],
				m_DisplayFetchOR[ arm ][ phase ],
				m_DisplayFetchRuns[ arm ][ phase ],
				m_DisplayFetchMaxRun[ arm ][ phase ] );
			if ( m_DisplayFetchErrors[ arm ][ phase ] )
			{
				const u8 *map = phase == 0
					? displayFetchPrefillMismatch[ arm ]
					: phase == 1 ? displayFetchBaselineMismatch[ arm ]
					             : displayFetchPostMismatch[ arm ];
				busDiagText( dst, capacity, n, "  map-bits:" );
				displayRowFormatMapBits( dst, capacity, n, map );
			}
		}
	}
	for ( u32 rung = 0; rung < DISPLAY_FETCH_RETENTION_COUNT; rung++ )
	{
		busDiagText( dst, capacity, n,
		             "fetch-retention rung/salt/prefill-attempts/requested-us/elapsed-us: " );
		busDiagDecimal( dst, capacity, n, rung );
		busDiagChar( dst, capacity, n, '/' );
		busDiagHex( dst, capacity, n, m_DisplayFetchRetentionSalt[ rung ] );
		busDiagChar( dst, capacity, n, '/' );
		busDiagDecimal( dst, capacity, n,
		                m_DisplayFetchRetentionPrefillAttempts[ rung ] );
		busDiagChar( dst, capacity, n, '/' );
		busDiagDecimal( dst, capacity, n,
		                m_DisplayFetchRetentionDelayUsec[ rung ] );
		busDiagChar( dst, capacity, n, '/' );
		busDiagDecimal( dst, capacity, n,
		                m_DisplayFetchRetentionElapsedUsec[ rung ] );
		busDiagEndLine( dst, capacity, n );
		busDiagText( dst, capacity, n,
		             "  seed-us complement/target capture-us prefill/baseline/post: " );
		busDiagDecimal( dst, capacity, n,
		                m_DisplayFetchRetentionSeedUsec[ rung ][ 0 ] );
		busDiagChar( dst, capacity, n, '/' );
		busDiagDecimal( dst, capacity, n,
		                m_DisplayFetchRetentionSeedUsec[ rung ][ 1 ] );
		for ( u32 phase = 0; phase < 3; phase++ )
		{
			busDiagChar( dst, capacity, n, '/' );
			busDiagDecimal( dst, capacity, n,
			                m_DisplayFetchRetentionCaptureUsec[ rung ][ phase ] );
		}
		busDiagEndLine( dst, capacity, n );
		for ( u32 phase = 0; phase < 3; phase++ )
		{
			displayRowFormatSummary(
				dst, capacity, n, phaseNames[ phase ],
				m_DisplayFetchRetentionErrors[ rung ][ phase ],
				m_DisplayFetchRetentionFirst[ rung ][ phase ],
				m_DisplayFetchRetentionLast[ rung ][ phase ],
				m_DisplayFetchRetentionAND[ rung ][ phase ],
				m_DisplayFetchRetentionOR[ rung ][ phase ],
				m_DisplayFetchRetentionRuns[ rung ][ phase ],
				m_DisplayFetchRetentionMaxRun[ rung ][ phase ] );
			if ( m_DisplayFetchRetentionErrors[ rung ][ phase ] )
			{
				const u8 *map = phase == 0
					? displayFetchRetentionPrefillMismatch[ rung ]
					: phase == 1 ? displayFetchRetentionBaselineMismatch[ rung ]
					             : displayFetchRetentionPostMismatch[ rung ];
				busDiagText( dst, capacity, n, "  map-bits:" );
				displayRowFormatMapBits( dst, capacity, n, map );
			}
		}
	}
}

void CRADBus::formatDisplayPersistenceResults( char *dst, u32 capacity,
	                                            u32 &n ) const
{
	static const char *stateNames[ 3 ] = { "off", "text", "bitmap" };
	static const char *operationNames[ 3 ] = { "none", "read", "write" };
	static const char *phaseNames[ 5 ] =
		{ "prefill", "baseline", "post0", "post1", "post2" };
	busDiagText( dst, capacity, n,
	             "protocol: C64-DISPLAY-PERSISTENCE-K227 ranges=0400-07FF,2000-3FFF bytes=9216 oracle=address-salt-high-entropy setup=DEN0-complement-adjacent-repeat-verified target=single-write exposure-seconds=4 traffic-rate=16000 active-states=text,bitmap operations=none,read-D020,write-same-D020 repeats-per-condition=2 blank-before-post=1 post-captures=3 xor-signature=1 raw-maps=errors-only diagnostic_only=1\r\n" );
	for ( u32 arm = 0; arm < DISPLAY_PERSISTENCE_ARM_COUNT; arm++ )
	{
		busDiagText( dst, capacity, n,
		             "persistence arm/state/op/salt/prefill-attempts/count/elapsed-us: " );
		busDiagDecimal( dst, capacity, n, arm );
		busDiagChar( dst, capacity, n, '/' );
		busDiagText( dst, capacity, n,
		             stateNames[ m_DisplayPersistenceState[ arm ] ] );
		busDiagChar( dst, capacity, n, '/' );
		busDiagText( dst, capacity, n,
		             operationNames[ m_DisplayPersistenceOperation[ arm ] ] );
		busDiagChar( dst, capacity, n, '/' );
		busDiagHex( dst, capacity, n, m_DisplayPersistenceSalt[ arm ] );
		busDiagChar( dst, capacity, n, '/' );
		busDiagDecimal( dst, capacity, n,
		                m_DisplayPersistencePrefillAttempts[ arm ] );
		busDiagChar( dst, capacity, n, '/' );
		busDiagDecimal( dst, capacity, n,
		                m_DisplayPersistenceTrafficCount[ arm ] );
		busDiagChar( dst, capacity, n, '/' );
		busDiagDecimal( dst, capacity, n,
		                m_DisplayPersistenceElapsedUsec[ arm ] );
		busDiagEndLine( dst, capacity, n );
		busDiagText( dst, capacity, n,
		             "  seed-us complement/target capture-us pre/base/p0/p1/p2: " );
		busDiagDecimal( dst, capacity, n,
		                m_DisplayPersistenceSeedUsec[ arm ][ 0 ] );
		busDiagChar( dst, capacity, n, '/' );
		busDiagDecimal( dst, capacity, n,
		                m_DisplayPersistenceSeedUsec[ arm ][ 1 ] );
		for ( u32 phase = 0; phase < DISPLAY_PERSISTENCE_PHASE_COUNT; phase++ )
		{
			busDiagChar( dst, capacity, n, '/' );
			busDiagDecimal( dst, capacity, n,
			                m_DisplayPersistenceCaptureUsec[ arm ][ phase ] );
		}
		busDiagEndLine( dst, capacity, n );
		busDiagText( dst, capacity, n,
		             "  persistence p0-p1 same/added/removed p1-p2 same/added/removed: " );
		for ( u32 pair = 0; pair < 2; pair++ )
		{
			if ( pair ) busDiagChar( dst, capacity, n, '/' );
			busDiagDecimal( dst, capacity, n,
			                m_DisplayPersistenceSame[ arm ][ pair ] );
			busDiagChar( dst, capacity, n, '/' );
			busDiagDecimal( dst, capacity, n,
			                m_DisplayPersistenceAdded[ arm ][ pair ] );
			busDiagChar( dst, capacity, n, '/' );
			busDiagDecimal( dst, capacity, n,
			                m_DisplayPersistenceRemoved[ arm ][ pair ] );
		}
		busDiagEndLine( dst, capacity, n );
		for ( u32 phase = 0; phase < DISPLAY_PERSISTENCE_PHASE_COUNT; phase++ )
		{
			displayRowFormatSummary(
				dst, capacity, n, phaseNames[ phase ],
				m_DisplayPersistenceErrors[ arm ][ phase ],
				m_DisplayPersistenceFirst[ arm ][ phase ],
				m_DisplayPersistenceLast[ arm ][ phase ],
				m_DisplayPersistenceAND[ arm ][ phase ],
				m_DisplayPersistenceOR[ arm ][ phase ],
				m_DisplayPersistenceRuns[ arm ][ phase ],
				m_DisplayPersistenceMaxRun[ arm ][ phase ] );
			busDiagText( dst, capacity, n,
			             "  first expected/actual xor-and/or: " );
			busDiagHex( dst, capacity, n,
			            m_DisplayPersistenceFirstExpected[ arm ][ phase ] );
			busDiagChar( dst, capacity, n, '/' );
			busDiagHex( dst, capacity, n,
			            m_DisplayPersistenceFirstActual[ arm ][ phase ] );
			busDiagChar( dst, capacity, n, '/' );
			busDiagHex( dst, capacity, n,
			            m_DisplayPersistenceXorAND[ arm ][ phase ] );
			busDiagChar( dst, capacity, n, '/' );
			busDiagHex( dst, capacity, n,
			            m_DisplayPersistenceXorOR[ arm ][ phase ] );
			busDiagEndLine( dst, capacity, n );
			if ( m_DisplayPersistenceErrors[ arm ][ phase ] )
			{
				busDiagText( dst, capacity, n, "  map-bits:" );
				displayRowFormatMapBits(
					dst, capacity, n,
					displayPersistenceMismatch[ arm ][ phase ] );
			}
		}
	}
}

void CRADBus::formatDisplayTimingResults( char *dst, u32 capacity,
	                                      u32 &n ) const
{
	static const char *stateNames[ 3 ] = { "off", "text", "bitmap" };
	static const char *operationNames[ 3 ] = { "none", "read", "write" };
	busDiagText( dst, capacity, n,
	             "protocol: C64-DISPLAY-TIMING-K228 ranges=0400-07FF,2000-3FFF bytes=9216 K227-arms=0..5 exposure-seconds=4 traffic-rate=16000 samples=475,350,400,450,475,500,550,600,475,repair-475 compare-to-phase0=1 targeted-repair=phase0-errors-only raw-maps=phase0,phase8,phase9-errors-only diagnostic_only=1\r\n" );
	for ( u32 arm = 0; arm < DISPLAY_TIMING_ARM_COUNT; arm++ )
	{
		busDiagText( dst, capacity, n,
		             "timing arm/state/op/salt/prefill-attempts/prefill-errors/baseline-errors/count/elapsed-us/repair-writes: " );
		busDiagDecimal( dst, capacity, n, arm );
		busDiagChar( dst, capacity, n, '/' );
		busDiagText( dst, capacity, n, stateNames[ m_DisplayTimingState[ arm ] ] );
		busDiagChar( dst, capacity, n, '/' );
		busDiagText( dst, capacity, n,
		             operationNames[ m_DisplayTimingOperation[ arm ] ] );
		busDiagChar( dst, capacity, n, '/' );
		busDiagHex( dst, capacity, n, m_DisplayTimingSalt[ arm ] );
		busDiagChar( dst, capacity, n, '/' );
		busDiagDecimal( dst, capacity, n,
		                m_DisplayTimingPrefillAttempts[ arm ] );
		busDiagChar( dst, capacity, n, '/' );
		busDiagDecimal( dst, capacity, n,
		                m_DisplayTimingPrefillErrors[ arm ] );
		busDiagChar( dst, capacity, n, '/' );
		busDiagDecimal( dst, capacity, n,
		                m_DisplayTimingBaselineErrors[ arm ] );
		busDiagChar( dst, capacity, n, '/' );
		busDiagDecimal( dst, capacity, n,
		                m_DisplayTimingTrafficCount[ arm ] );
		busDiagChar( dst, capacity, n, '/' );
		busDiagDecimal( dst, capacity, n,
		                m_DisplayTimingElapsedUsec[ arm ] );
		busDiagChar( dst, capacity, n, '/' );
		busDiagDecimal( dst, capacity, n,
		                m_DisplayTimingRepairWrites[ arm ] );
		busDiagEndLine( dst, capacity, n );
		for ( u32 phase = 0; phase < DISPLAY_TIMING_PHASE_COUNT; phase++ )
		{
			busDiagText( dst, capacity, n,
			             "  phase/sample/errors/same/added/removed/first/last/and/or/first-expected/actual/xor-and/or/capture-us: " );
			busDiagDecimal( dst, capacity, n, phase );
			busDiagChar( dst, capacity, n, '/' );
			busDiagDecimal( dst, capacity, n,
			                m_DisplayTimingSample[ phase ] );
			busDiagChar( dst, capacity, n, '/' );
			busDiagDecimal( dst, capacity, n,
			                m_DisplayTimingErrors[ arm ][ phase ] );
			busDiagChar( dst, capacity, n, '/' );
			busDiagDecimal( dst, capacity, n,
			                m_DisplayTimingSame[ arm ][ phase ] );
			busDiagChar( dst, capacity, n, '/' );
			busDiagDecimal( dst, capacity, n,
			                m_DisplayTimingAdded[ arm ][ phase ] );
			busDiagChar( dst, capacity, n, '/' );
			busDiagDecimal( dst, capacity, n,
			                m_DisplayTimingRemoved[ arm ][ phase ] );
			busDiagChar( dst, capacity, n, '/' );
			busDiagHexWord( dst, capacity, n,
			                m_DisplayTimingFirst[ arm ][ phase ] );
			busDiagChar( dst, capacity, n, '/' );
			busDiagHexWord( dst, capacity, n,
			                m_DisplayTimingLast[ arm ][ phase ] );
			busDiagChar( dst, capacity, n, '/' );
			busDiagHexWord( dst, capacity, n,
			                m_DisplayTimingAND[ arm ][ phase ] );
			busDiagChar( dst, capacity, n, '/' );
			busDiagHexWord( dst, capacity, n,
			                m_DisplayTimingOR[ arm ][ phase ] );
			busDiagChar( dst, capacity, n, '/' );
			busDiagHex( dst, capacity, n,
			            m_DisplayTimingFirstExpected[ arm ][ phase ] );
			busDiagChar( dst, capacity, n, '/' );
			busDiagHex( dst, capacity, n,
			            m_DisplayTimingFirstActual[ arm ][ phase ] );
			busDiagChar( dst, capacity, n, '/' );
			busDiagHex( dst, capacity, n,
			            m_DisplayTimingXorAND[ arm ][ phase ] );
			busDiagChar( dst, capacity, n, '/' );
			busDiagHex( dst, capacity, n,
			            m_DisplayTimingXorOR[ arm ][ phase ] );
			busDiagChar( dst, capacity, n, '/' );
			busDiagDecimal( dst, capacity, n,
			                m_DisplayTimingCaptureUsec[ arm ][ phase ] );
			busDiagEndLine( dst, capacity, n );
			if ( m_DisplayTimingErrors[ arm ][ phase ]
			     && ( phase == 0 || phase >= 8 ) )
			{
				busDiagText( dst, capacity, n, "  map-bits:" );
				displayRowFormatMapBits(
					dst, capacity, n,
					displayTimingMismatch[ arm ][ phase ] );
			}
		}
	}
}

void CRADBus::formatDisplayBoundaryResults( char *dst, u32 capacity,
	                                        u32 &n ) const
{
	busDiagText( dst, capacity, n,
	             "protocol: C64-DISPLAY-BOUNDARY-K229 ranges=0400-07FF,2000-3FFF bytes=9216 states=text,bitmap transition-windows=visible-80-87,border-F0-FF durations=transition-only,4-second-dwell repeats=2 salts=text-0D,bitmap-44 prefill=complement-repeat-verified capture-DEN0 raw-maps=errors-only diagnostic_only=1\r\n" );
	for ( u32 arm = 0; arm < DISPLAY_BOUNDARY_ARM_COUNT; arm++ )
	{
		busDiagText( dst, capacity, n,
		             "boundary arm/state/window/duration/salt/prefill-attempts/prefill-errors/elapsed-us/wait-enable/wait-blank/same/added/removed: " );
		busDiagDecimal( dst, capacity, n, arm );
		busDiagChar( dst, capacity, n, '/' );
		busDiagText( dst, capacity, n,
		             m_DisplayBoundaryState[ arm ] == 1 ? "text" : "bitmap" );
		busDiagChar( dst, capacity, n, '/' );
		busDiagText( dst, capacity, n,
		             m_DisplayBoundarySafe[ arm ] ? "border" : "visible" );
		busDiagChar( dst, capacity, n, '/' );
		busDiagText( dst, capacity, n,
		             m_DisplayBoundaryDwell[ arm ] ? "dwell" : "transition" );
		busDiagChar( dst, capacity, n, '/' );
		busDiagHex( dst, capacity, n, m_DisplayBoundarySalt[ arm ] );
		const u32 values[ 8 ] =
		{
			m_DisplayBoundaryPrefillAttempts[ arm ],
			m_DisplayBoundaryPrefillErrors[ arm ],
			m_DisplayBoundaryElapsedUsec[ arm ],
			m_DisplayBoundaryWaitReads[ arm ][ 0 ],
			m_DisplayBoundaryWaitReads[ arm ][ 1 ],
			m_DisplayBoundarySame[ arm ],
			m_DisplayBoundaryAdded[ arm ],
			m_DisplayBoundaryRemoved[ arm ]
		};
		for ( u32 i = 0; i < 8; i++ )
		{
			busDiagChar( dst, capacity, n, '/' );
			busDiagDecimal( dst, capacity, n, values[ i ] );
		}
		busDiagEndLine( dst, capacity, n );
		for ( u32 phase = 0; phase < DISPLAY_BOUNDARY_PHASE_COUNT; phase++ )
		{
			displayRowFormatSummary(
				dst, capacity, n, phase == 0 ? "  baseline" : "  post",
				m_DisplayBoundaryErrors[ arm ][ phase ],
				m_DisplayBoundaryFirst[ arm ][ phase ],
				m_DisplayBoundaryLast[ arm ][ phase ],
				m_DisplayBoundaryAND[ arm ][ phase ],
				m_DisplayBoundaryOR[ arm ][ phase ],
				m_DisplayBoundaryRuns[ arm ][ phase ],
				m_DisplayBoundaryMaxRun[ arm ][ phase ] );
			busDiagText( dst, capacity, n,
			             "  first expected/actual xor-and/or capture-us: " );
			busDiagHex( dst, capacity, n,
			            m_DisplayBoundaryFirstExpected[ arm ][ phase ] );
			busDiagChar( dst, capacity, n, '/' );
			busDiagHex( dst, capacity, n,
			            m_DisplayBoundaryFirstActual[ arm ][ phase ] );
			busDiagChar( dst, capacity, n, '/' );
			busDiagHex( dst, capacity, n,
			            m_DisplayBoundaryXorAND[ arm ][ phase ] );
			busDiagChar( dst, capacity, n, '/' );
			busDiagHex( dst, capacity, n,
			            m_DisplayBoundaryXorOR[ arm ][ phase ] );
			busDiagChar( dst, capacity, n, '/' );
			busDiagDecimal( dst, capacity, n,
			                m_DisplayBoundaryCaptureUsec[ arm ][ phase ] );
			busDiagEndLine( dst, capacity, n );
			if ( m_DisplayBoundaryErrors[ arm ][ phase ] )
			{
				busDiagText( dst, capacity, n, "  map-bits:" );
				displayRowFormatMapBits(
					dst, capacity, n,
					displayBoundaryMismatch[ arm ][ phase ] );
			}
		}
	}
}

void CRADBus::formatDisplayRefreshResults( char *dst, u32 capacity,
	                                       u32 &n ) const
{
	busDiagText( dst, capacity, n,
	             m_DisplayDiagnosticVariant == 3
	               ? "protocol: C64-DISPLAY-SCRUB-K233 ranges=0400-07FF,2000-3FFF bytes=9216 states=text,bitmap exposure-seconds=16 scrub=off,production-CWriteBuffer-masked-A1-low-A3-high repeats=2 cadence=128-opportunities-per-frame chunk=64 hidden-raster-only ordinary-queue-empty prefill=complement-repeat-verified capture-DEN0 raw-maps=errors-only diagnostic_only=1\r\n"
	             : m_DisplayDiagnosticVariant == 2
	               ? "protocol: C64-DISPLAY-RW-K232 ranges=0400-07FF,2000-3FFF bytes=9216 states=text,bitmap exposure-seconds=8 rw-idle=floating,actively-held-high repeats=2 salts=text-0D,bitmap-44 transitions=border-F0-FF traffic=none dma=continuously-asserted prefill=complement-repeat-verified capture-DEN0 raw-maps=errors-only diagnostic_only=1\r\n"
	             : m_DisplayDiagnosticVariant == 1
	               ? "protocol: C64-DISPLAY-REFRESH-K231 ranges=0400-07FF,2000-3FFF bytes=9216 states=text,bitmap exposure-seconds=4 refresh=none,core0-continuous-VIC-half-DMA-release repeats=2 salts=text-0D,bitmap-44 transitions=border-F0-FF prefill=complement-repeat-verified capture-DEN0 raw-maps=errors-only diagnostic_only=1\r\n"
	               : "protocol: C64-DISPLAY-REFRESH-K230 ranges=0400-07FF,2000-3FFF bytes=9216 states=text,bitmap exposure-seconds=4 refresh=none,core3-continuous-VIC-half-DMA-release repeats=2 salts=text-0D,bitmap-44 transitions=border-F0-FF prefill=complement-repeat-verified capture-DEN0 raw-maps=errors-only diagnostic_only=1\r\n" );
	for ( u32 arm = 0; arm < 8; arm++ )
	{
		busDiagText( dst, capacity, n,
		             m_DisplayDiagnosticVariant == 3
		               ? "scrub arm/state/enabled/start-ok/salt/prefill-attempts/prefill-errors/elapsed-us/wait-enable/wait-blank/bytes/same/added/removed/opportunities/max-chunk-cycles: "
		             : m_DisplayDiagnosticVariant == 2
		               ? "rw arm/state/held-high/start-ok/salt/prefill-attempts/prefill-errors/elapsed-us/wait-enable/wait-blank/slots/same/added/removed: "
		               : "refresh arm/state/enabled/start-ok/salt/prefill-attempts/prefill-errors/elapsed-us/wait-enable/wait-blank/slots/same/added/removed: " );
		busDiagDecimal( dst, capacity, n, arm );
		busDiagChar( dst, capacity, n, '/' );
		busDiagText( dst, capacity, n,
		             m_DisplayBoundaryState[ arm ] == 1 ? "text" : "bitmap" );
		busDiagChar( dst, capacity, n, '/' );
		busDiagDecimal( dst, capacity, n, m_DisplayRefreshEnabled[ arm ] );
		busDiagChar( dst, capacity, n, '/' );
		busDiagDecimal( dst, capacity, n, m_DisplayRefreshStartOK[ arm ] );
		busDiagChar( dst, capacity, n, '/' );
		busDiagHex( dst, capacity, n, m_DisplayBoundarySalt[ arm ] );
		const u32 values[ 5 ] =
		{
			m_DisplayBoundaryPrefillAttempts[ arm ],
			m_DisplayBoundaryPrefillErrors[ arm ],
			m_DisplayBoundaryElapsedUsec[ arm ],
			m_DisplayBoundaryWaitReads[ arm ][ 0 ],
			m_DisplayBoundaryWaitReads[ arm ][ 1 ]
		};
		for ( u32 i = 0; i < 5; i++ )
		{
			busDiagChar( dst, capacity, n, '/' );
			busDiagDecimal( dst, capacity, n, values[ i ] );
		}
		busDiagChar( dst, capacity, n, '/' );
		busDiagDecimal( dst, capacity, n,
		                (u32)m_DisplayRefreshSlots[ arm ] );
		const u32 comparison[ 3 ] =
		{
			m_DisplayBoundarySame[ arm ], m_DisplayBoundaryAdded[ arm ],
			m_DisplayBoundaryRemoved[ arm ]
		};
		for ( u32 i = 0; i < 3; i++ )
		{
			busDiagChar( dst, capacity, n, '/' );
			busDiagDecimal( dst, capacity, n, comparison[ i ] );
		}
		if ( m_DisplayDiagnosticVariant == 3 )
		{
			busDiagChar( dst, capacity, n, '/' );
			busDiagDecimal( dst, capacity, n,
			                m_DisplayScrubOpportunities[ arm ] );
			busDiagChar( dst, capacity, n, '/' );
			busDiagDecimal( dst, capacity, n,
			                m_DisplayScrubMaxChunkCycles[ arm ] );
		}
		busDiagEndLine( dst, capacity, n );
		for ( u32 phase = 0; phase < 2; phase++ )
		{
			displayRowFormatSummary(
				dst, capacity, n, phase == 0 ? "  baseline" : "  post",
				m_DisplayBoundaryErrors[ arm ][ phase ],
				m_DisplayBoundaryFirst[ arm ][ phase ],
				m_DisplayBoundaryLast[ arm ][ phase ],
				m_DisplayBoundaryAND[ arm ][ phase ],
				m_DisplayBoundaryOR[ arm ][ phase ],
				m_DisplayBoundaryRuns[ arm ][ phase ],
				m_DisplayBoundaryMaxRun[ arm ][ phase ] );
			busDiagText( dst, capacity, n,
			             "  first expected/actual xor-and/or capture-us: " );
			busDiagHex( dst, capacity, n,
			            m_DisplayBoundaryFirstExpected[ arm ][ phase ] );
			busDiagChar( dst, capacity, n, '/' );
			busDiagHex( dst, capacity, n,
			            m_DisplayBoundaryFirstActual[ arm ][ phase ] );
			busDiagChar( dst, capacity, n, '/' );
			busDiagHex( dst, capacity, n,
			            m_DisplayBoundaryXorAND[ arm ][ phase ] );
			busDiagChar( dst, capacity, n, '/' );
			busDiagHex( dst, capacity, n,
			            m_DisplayBoundaryXorOR[ arm ][ phase ] );
			busDiagChar( dst, capacity, n, '/' );
			busDiagDecimal( dst, capacity, n,
			                m_DisplayBoundaryCaptureUsec[ arm ][ phase ] );
			busDiagEndLine( dst, capacity, n );
			if ( m_DisplayBoundaryErrors[ arm ][ phase ] )
			{
				busDiagText( dst, capacity, n, "  map-bits:" );
				displayRowFormatMapBits(
					dst, capacity, n,
					displayBoundaryMismatch[ arm ][ phase ] );
			}
		}
	}
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
	busDiagText( dst, capacity, n,
	             "runtime transfers/IEC-line-transfers/idle-primes: " );
	busDiagDecimal( dst, capacity, n, (u32)m_Transfers );
	busDiagChar( dst, capacity, n, '/' );
	busDiagDecimal( dst, capacity, n, (u32)m_SerialTransfers );
	busDiagChar( dst, capacity, n, '/' );
	busDiagDecimal( dst, capacity, n, (u32)m_ReadPrimes );
	busDiagEndLine( dst, capacity, n );
	if ( m_LoadedReadTimingRan )
	{
		busDiagText( dst, capacity, n,
		             "read-eye core1 A/B quiet configured/selected/start/end/best/errors: " );
		busDiagDecimal( dst, capacity, n, m_QuietReadTimingConfigured );
		busDiagChar( dst, capacity, n, '/' );
		busDiagDecimal( dst, capacity, n, m_QuietReadTimingSelected );
		busDiagChar( dst, capacity, n, '/' );
		busDiagDecimal( dst, capacity, n, m_QuietReadTimingStart );
		busDiagChar( dst, capacity, n, '/' );
		busDiagDecimal( dst, capacity, n, m_QuietReadTimingEnd );
		busDiagChar( dst, capacity, n, '/' );
		busDiagDecimal( dst, capacity, n, m_QuietReadTimingBestErrorSample );
		busDiagChar( dst, capacity, n, '/' );
		busDiagDecimal( dst, capacity, n, m_QuietReadTimingBestError );
		busDiagText( dst, capacity, n,
		             " loaded configured/selected/start/end/best/errors: " );
		busDiagDecimal( dst, capacity, n, m_ReadTimingConfigured );
		busDiagChar( dst, capacity, n, '/' );
		busDiagDecimal( dst, capacity, n, m_ReadTimingSelected );
		busDiagChar( dst, capacity, n, '/' );
		busDiagDecimal( dst, capacity, n, m_ReadTimingStart );
		busDiagChar( dst, capacity, n, '/' );
		busDiagDecimal( dst, capacity, n, m_ReadTimingEnd );
		busDiagChar( dst, capacity, n, '/' );
		busDiagDecimal( dst, capacity, n, m_ReadTimingBestErrorSample );
		busDiagChar( dst, capacity, n, '/' );
		busDiagDecimal( dst, capacity, n, m_ReadTimingBestError );
		busDiagEndLine( dst, capacity, n );
		busDiagText( dst, capacity, n, "read-eye quiet sample/errors:" );
		for ( u32 i = 0; i < READ_TIMING_SCORE_COUNT; i++ )
		{
			busDiagChar( dst, capacity, n, ' ' );
			busDiagDecimal( dst, capacity, n, 300u + i * 5u );
			busDiagChar( dst, capacity, n, '/' );
			busDiagDecimal( dst, capacity, n, m_QuietReadTimingErrors[ i ] );
		}
		busDiagEndLine( dst, capacity, n );
	}
	busDiagText( dst, capacity, n,
	             "read-eye configured/selected/stable-start/stable-end/best-sample/best-errors: " );
	busDiagDecimal( dst, capacity, n, m_ReadTimingConfigured );
	busDiagChar( dst, capacity, n, '/' );
	busDiagDecimal( dst, capacity, n, m_ReadTimingSelected );
	busDiagChar( dst, capacity, n, '/' );
	busDiagDecimal( dst, capacity, n, m_ReadTimingStart );
	busDiagChar( dst, capacity, n, '/' );
	busDiagDecimal( dst, capacity, n, m_ReadTimingEnd );
	busDiagChar( dst, capacity, n, '/' );
	busDiagDecimal( dst, capacity, n, m_ReadTimingBestErrorSample );
	busDiagChar( dst, capacity, n, '/' );
	busDiagDecimal( dst, capacity, n, m_ReadTimingBestError );
	busDiagEndLine( dst, capacity, n );
	busDiagText( dst, capacity, n, "read-eye sample/errors:" );
	for ( u32 i = 0; i < READ_TIMING_SCORE_COUNT; i++ )
	{
		busDiagChar( dst, capacity, n, ' ' );
		busDiagDecimal( dst, capacity, n, 300u + i * 5u );
		busDiagChar( dst, capacity, n, '/' );
		busDiagDecimal( dst, capacity, n, m_ReadTimingErrors[ i ] );
	}
	busDiagEndLine( dst, capacity, n );
	busDiagText( dst, capacity, n,
	             "read-eye split sample/mixed-ram-errors/ram-only-errors/mixed-vic-errors/isolated-vic-errors/mixed-distinct/mixed-dominant-value/mixed-dominant-count/isolated-distinct/isolated-dominant-value/isolated-dominant-count:" );
	for ( u32 i = 0; i < READ_TIMING_SCORE_COUNT; i++ )
	{
		busDiagChar( dst, capacity, n, ' ' );
		busDiagDecimal( dst, capacity, n, 300u + i * 5u );
		busDiagChar( dst, capacity, n, '/' );
		busDiagDecimal( dst, capacity, n, m_ReadTimingRAMErrors[ i ] );
		busDiagChar( dst, capacity, n, '/' );
		busDiagDecimal( dst, capacity, n, m_ReadTimingRAMOnlyErrors[ i ] );
		busDiagChar( dst, capacity, n, '/' );
		busDiagDecimal( dst, capacity, n, m_ReadTimingMixedVICErrors[ i ] );
		busDiagChar( dst, capacity, n, '/' );
		busDiagDecimal( dst, capacity, n, m_ReadTimingIsolatedVICErrors[ i ] );
		busDiagChar( dst, capacity, n, '/' );
		busDiagDecimal( dst, capacity, n, m_ReadTimingMixedVICDistinct[ i ] );
		busDiagChar( dst, capacity, n, '/' );
		busDiagHex( dst, capacity, n, m_ReadTimingMixedVICDominantValue[ i ] );
		busDiagChar( dst, capacity, n, '/' );
		busDiagDecimal( dst, capacity, n, m_ReadTimingMixedVICDominantCount[ i ] );
		busDiagChar( dst, capacity, n, '/' );
		busDiagDecimal( dst, capacity, n, m_ReadTimingIsolatedVICDistinct[ i ] );
		busDiagChar( dst, capacity, n, '/' );
		busDiagHex( dst, capacity, n, m_ReadTimingIsolatedVICDominantValue[ i ] );
		busDiagChar( dst, capacity, n, '/' );
		busDiagDecimal( dst, capacity, n, m_ReadTimingIsolatedVICDominantCount[ i ] );
	}
	busDiagEndLine( dst, capacity, n );
	const u32 ramBestIndex = m_ReadTimingRAMBestSample >= 300u
	                       && m_ReadTimingRAMBestSample <= 620u
	                       ? ( m_ReadTimingRAMBestSample - 300u ) / 5u : 0u;
	busDiagText( dst, capacity, n, "read-eye RAM-best sample/errors: " );
	busDiagDecimal( dst, capacity, n, m_ReadTimingRAMBestSample );
	busDiagChar( dst, capacity, n, '/' );
	busDiagDecimal( dst, capacity, n, m_ReadTimingRAMBestError );
	busDiagEndLine( dst, capacity, n );
	busDiagText( dst, capacity, n, "read-eye RAM-only-best sample/errors: " );
	busDiagDecimal( dst, capacity, n, m_ReadTimingRAMOnlyBestSample );
	busDiagChar( dst, capacity, n, '/' );
	busDiagDecimal( dst, capacity, n, m_ReadTimingRAMOnlyBestError );
	busDiagEndLine( dst, capacity, n );
	busDiagText( dst, capacity, n,
	             "read-eye RAM-best rotated position-errors (positions 0..5):" );
	for ( u32 p = 0; p < 6; p++ )
	{
		busDiagChar( dst, capacity, n, ' ' );
		busDiagDecimal( dst, capacity, n,
		                m_ReadTimingMixedRAMPositionErrors[ ramBestIndex ][ p ] );
	}
	busDiagEndLine( dst, capacity, n );
	busDiagText( dst, capacity, n,
	             "read-eye RAM-best rotated address-errors ($0334..$0339):" );
	for ( u32 p = 0; p < 6; p++ )
	{
		busDiagChar( dst, capacity, n, ' ' );
		busDiagDecimal( dst, capacity, n,
		                m_ReadTimingMixedRAMAddressErrors[ ramBestIndex ][ p ] );
	}
	busDiagEndLine( dst, capacity, n );
	busDiagText( dst, capacity, n, "read-eye RAM-best mixed-vic actual:" );
	for ( u32 r = 0; r < READ_TIMING_REPETITIONS; r++ )
	{
		busDiagChar( dst, capacity, n, ' ' );
		busDiagHex( dst, capacity, n,
		            m_ReadTimingMixedVICActual[ ramBestIndex ][ r ] );
	}
	busDiagEndLine( dst, capacity, n );
	busDiagText( dst, capacity, n, "read-eye RAM-best isolated-vic actual:" );
	for ( u32 r = 0; r < READ_TIMING_REPETITIONS; r++ )
	{
		busDiagChar( dst, capacity, n, ' ' );
		busDiagHex( dst, capacity, n,
		            m_ReadTimingIsolatedVICActual[ ramBestIndex ][ r ] );
	}
	busDiagEndLine( dst, capacity, n );
	busDiagText( dst, capacity, n,
	             "snapshot byte0 observed kernal/basic: " );
	if ( m_SnapshotKernalObserved )
	{
		busDiagHex( dst, capacity, n, m_SnapshotKernalFirst );
		busDiagChar( dst, capacity, n, '/' );
		busDiagHex( dst, capacity, n, m_SnapshotKernalReread );
		busDiagChar( dst, capacity, n, '/' );
		busDiagHex( dst, capacity, n, m_SnapshotKernalCopied );
	}
	else busDiagText( dst, capacity, n, "not-read" );
	busDiagText( dst, capacity, n, " " );
	if ( m_SnapshotBasicObserved )
	{
		busDiagHex( dst, capacity, n, m_SnapshotBasicFirst );
		busDiagChar( dst, capacity, n, '/' );
		busDiagHex( dst, capacity, n, m_SnapshotBasicReread );
		busDiagChar( dst, capacity, n, '/' );
		busDiagHex( dst, capacity, n, m_SnapshotBasicCopied );
	}
	else busDiagText( dst, capacity, n, "not-read" );
	busDiagText( dst, capacity, n, " (first/reread/copied)\r\n" );
	if ( m_AccessSentinelRan )
	{
		busDiagText( dst, capacity, n,
		             "protocol: C64-ACCESS-SENTINEL-K221 ranges=0800-9FFF,C000-CFFF bytes=43008 traffic-rate-per-sec=16000 traffic-count=" );
		busDiagDecimal( dst, capacity, n, m_AccessSentinelTrafficCount );
		busDiagText( dst, capacity, n,
		             " arms=control,read-D020,write-same-D020 baseline-passes=2 exposure-passes=2 diagnostic_only=1\r\n" );
		static const char *armNames[ 3 ] =
			{ "control", "read-D020", "write-D020" };
		for ( u32 arm = 0; arm < 3; arm++ )
		{
			busDiagText( dst, capacity, n, "access-sentinel " );
			busDiagText( dst, capacity, n, armNames[ arm ] );
			busDiagText( dst, capacity, n,
			             " base0/base1/exposure0/exposure1/retained/arm-added/arm-removed/verify-same/verify-new/verify-cleared/elapsed-us: " );
			busDiagDecimal( dst, capacity, n,
			                m_AccessSentinelBaselineErrors[ arm ][ 0 ] );
			busDiagChar( dst, capacity, n, '/' );
			busDiagDecimal( dst, capacity, n,
			                m_AccessSentinelBaselineErrors[ arm ][ 1 ] );
			busDiagChar( dst, capacity, n, '/' );
			busDiagDecimal( dst, capacity, n,
			                m_AccessSentinelExposureErrors[ arm ][ 0 ] );
			busDiagChar( dst, capacity, n, '/' );
			busDiagDecimal( dst, capacity, n,
			                m_AccessSentinelExposureErrors[ arm ][ 1 ] );
			busDiagChar( dst, capacity, n, '/' );
			busDiagDecimal( dst, capacity, n,
			                m_AccessSentinelBaselineRetainedErrors[ arm ] );
			busDiagChar( dst, capacity, n, '/' );
			busDiagDecimal( dst, capacity, n,
			                m_AccessSentinelArmAddedErrors[ arm ] );
			busDiagChar( dst, capacity, n, '/' );
			busDiagDecimal( dst, capacity, n,
			                m_AccessSentinelArmRemovedErrors[ arm ] );
			busDiagChar( dst, capacity, n, '/' );
			busDiagDecimal( dst, capacity, n,
			                m_AccessSentinelSameErrors[ arm ] );
			busDiagChar( dst, capacity, n, '/' );
			busDiagDecimal( dst, capacity, n,
			                m_AccessSentinelNewErrors[ arm ] );
			busDiagChar( dst, capacity, n, '/' );
			busDiagDecimal( dst, capacity, n,
			                m_AccessSentinelClearedErrors[ arm ] );
			busDiagChar( dst, capacity, n, '/' );
			busDiagDecimal( dst, capacity, n,
			                m_AccessSentinelElapsedUsec[ arm ] );
			busDiagEndLine( dst, capacity, n );
			busDiagText( dst, capacity, n,
			             "  first0 addr/expected/actual: " );
			busDiagHexWord( dst, capacity, n,
			                m_AccessSentinelFirstAddr[ arm ][ 0 ] );
			busDiagChar( dst, capacity, n, '/' );
			busDiagHex( dst, capacity, n,
			            m_AccessSentinelFirstExpected[ arm ][ 0 ] );
			busDiagChar( dst, capacity, n, '/' );
			busDiagHex( dst, capacity, n,
			            m_AccessSentinelFirstActual[ arm ][ 0 ] );
			busDiagText( dst, capacity, n,
			             " first1 addr/expected/actual: " );
			busDiagHexWord( dst, capacity, n,
			                m_AccessSentinelFirstAddr[ arm ][ 1 ] );
			busDiagChar( dst, capacity, n, '/' );
			busDiagHex( dst, capacity, n,
			            m_AccessSentinelFirstExpected[ arm ][ 1 ] );
			busDiagChar( dst, capacity, n, '/' );
			busDiagHex( dst, capacity, n,
			            m_AccessSentinelFirstActual[ arm ][ 1 ] );
			busDiagEndLine( dst, capacity, n );
		}
	}
	if ( m_DisplaySentinelRan )
	{
		busDiagText( dst, capacity, n,
		             "protocol: C64-DISPLAY-SENTINEL-K222 matrix=0400-07FF bitmap=2000-3FFF bytes=9216 display-oracle=hires-AA55 readback-DEN0=1 readback-passes=3 exposure-seconds=8 arms=on-none,on-read4k,on-read16k,on-read64k,on-write4k,on-write16k,on-write64k,off-read64k,off-write64k diagnostic_only=1\r\n" );
		static const char *armNames[ 9 ] =
		{
			"on-none", "on-read-4k", "on-read-16k", "on-read-64k",
			"on-write-4k", "on-write-16k", "on-write-64k",
			"off-read-64k", "off-write-64k"
		};
		for ( u32 arm = 0; arm < 9; arm++ )
		{
			busDiagText( dst, capacity, n, "display-sentinel " );
			busDiagText( dst, capacity, n, armNames[ arm ] );
			busDiagText( dst, capacity, n,
			             " rate/count/base0/base1/exposure0/exposure1/exposure2/retained/arm-added/arm-removed/verify01-same/new/cleared/verify12-same/new/cleared/elapsed-us: " );
			busDiagDecimal( dst, capacity, n, m_DisplaySentinelRate[ arm ] );
			busDiagChar( dst, capacity, n, '/' );
			busDiagDecimal( dst, capacity, n,
			                m_DisplaySentinelTrafficCount[ arm ] );
			for ( u32 pass = 0; pass < 2; pass++ )
			{
				busDiagChar( dst, capacity, n, '/' );
				busDiagDecimal( dst, capacity, n,
				                m_DisplaySentinelBaselineErrors[ arm ][ pass ] );
			}
			for ( u32 pass = 0; pass < 3; pass++ )
			{
				busDiagChar( dst, capacity, n, '/' );
				busDiagDecimal( dst, capacity, n,
				                m_DisplaySentinelExposureErrors[ arm ][ pass ] );
			}
			const u32 summary[ 9 ] =
			{
				m_DisplaySentinelBaselineRetainedErrors[ arm ],
				m_DisplaySentinelArmAddedErrors[ arm ],
				m_DisplaySentinelArmRemovedErrors[ arm ],
				m_DisplaySentinelVerifySameErrors[ arm ][ 0 ],
				m_DisplaySentinelVerifyNewErrors[ arm ][ 0 ],
				m_DisplaySentinelVerifyClearedErrors[ arm ][ 0 ],
				m_DisplaySentinelVerifySameErrors[ arm ][ 1 ],
				m_DisplaySentinelVerifyNewErrors[ arm ][ 1 ],
				m_DisplaySentinelVerifyClearedErrors[ arm ][ 1 ]
			};
			for ( u32 i = 0; i < 9; i++ )
			{
				busDiagChar( dst, capacity, n, '/' );
				busDiagDecimal( dst, capacity, n, summary[ i ] );
			}
			busDiagChar( dst, capacity, n, '/' );
			busDiagDecimal( dst, capacity, n,
			                m_DisplaySentinelElapsedUsec[ arm ] );
			busDiagEndLine( dst, capacity, n );
			for ( u32 pass = 0; pass < 3; pass++ )
			{
				busDiagText( dst, capacity, n, "  first" );
				busDiagChar( dst, capacity, n, (char)( '0' + pass ) );
				busDiagText( dst, capacity, n, " addr/expected/actual: " );
				busDiagHexWord( dst, capacity, n,
				                m_DisplaySentinelFirstAddr[ arm ][ pass ] );
				busDiagChar( dst, capacity, n, '/' );
				busDiagHex( dst, capacity, n,
				            m_DisplaySentinelFirstExpected[ arm ][ pass ] );
				busDiagChar( dst, capacity, n, '/' );
				busDiagHex( dst, capacity, n,
				            m_DisplaySentinelFirstActual[ arm ][ pass ] );
				busDiagEndLine( dst, capacity, n );
			}
		}
	}
	if ( m_DisplayAddressRan )
	{
		busDiagText( dst, capacity, n,
		             "protocol: C64-DISPLAY-ADDRESS-K223 target=2078 immediate-trials=256 modes=text-DEN0,bitmap-DEN0 seed-styles=single,double maps=0400-07FF+2000-3FFF map-bytes=9216 map-order=matrix-then-bitmap delay-us=0,100,1000,4000,16000,64000,256000,1000000,4000000 diagnostic_only=1\r\n" );
		static const char *modeNames[ 2 ] = { "text", "bitmap" };
		static const char *styleNames[ 2 ] = { "single", "double" };
		for ( u32 mode = 0; mode < 2; mode++ )
		{
			for ( u32 style = 0; style < 2; style++ )
			{
				const u16 fixedMask = m_DisplayAddressMapErrors[ mode ][ style ]
					? (u16)~( m_DisplayAddressMapAND[ mode ][ style ]
					          ^ m_DisplayAddressMapOR[ mode ][ style ] )
					: 0;
				busDiagText( dst, capacity, n, "display-address " );
				busDiagText( dst, capacity, n, modeNames[ mode ] );
				busDiagChar( dst, capacity, n, '-' );
				busDiagText( dst, capacity, n, styleNames[ style ] );
				busDiagText( dst, capacity, n,
				             " immediate-errors-r0/r1/r2/first-trial/expected/actual0/actual1/actual2: " );
				for ( u32 read = 0; read < 3; read++ )
				{
					if ( read ) busDiagChar( dst, capacity, n, '/' );
					busDiagDecimal( dst, capacity, n,
					                m_DisplayAddressImmediateErrors[ mode ][ style ][ read ] );
				}
				busDiagChar( dst, capacity, n, '/' );
				busDiagDecimal( dst, capacity, n,
				                m_DisplayAddressImmediateFirstTrial[ mode ][ style ] );
				busDiagChar( dst, capacity, n, '/' );
				busDiagHex( dst, capacity, n,
				            m_DisplayAddressImmediateFirstExpected[ mode ][ style ] );
				for ( u32 read = 0; read < 3; read++ )
				{
					busDiagChar( dst, capacity, n, '/' );
					busDiagHex( dst, capacity, n,
					            m_DisplayAddressImmediateFirstActual[ mode ][ style ][ read ] );
				}
				busDiagEndLine( dst, capacity, n );
				busDiagText( dst, capacity, n,
				             "  map errors/first/last/and/or/fixed-mask/fixed-value/runs/max-run: " );
				busDiagDecimal( dst, capacity, n,
				                m_DisplayAddressMapErrors[ mode ][ style ] );
				const u16 mapSummary[ 7 ] =
				{
					m_DisplayAddressMapFirst[ mode ][ style ],
					m_DisplayAddressMapLast[ mode ][ style ],
					m_DisplayAddressMapAND[ mode ][ style ],
					m_DisplayAddressMapOR[ mode ][ style ],
					fixedMask,
					(u16)( m_DisplayAddressMapAND[ mode ][ style ] & fixedMask ),
					m_DisplayAddressMapRuns[ mode ][ style ]
				};
				for ( u32 i = 0; i < 7; i++ )
				{
					busDiagChar( dst, capacity, n, '/' );
					busDiagHexWord( dst, capacity, n, mapSummary[ i ] );
				}
				busDiagChar( dst, capacity, n, '/' );
				busDiagHexWord( dst, capacity, n,
				                 m_DisplayAddressMapMaxRun[ mode ][ style ] );
				busDiagEndLine( dst, capacity, n );

				// A fixed-size raw bitmap makes every address recoverable from the
				// SD log. Index 0 is $0400; index 1024 is $2000.
				busDiagText( dst, capacity, n, "  map-bits " );
				busDiagText( dst, capacity, n, modeNames[ mode ] );
				busDiagChar( dst, capacity, n, '-' );
				busDiagText( dst, capacity, n, styleNames[ style ] );
				busDiagText( dst, capacity, n, ":" );
				for ( u32 byte = 0; byte < DISPLAY_SENTINEL_BITMAP_BYTES; byte++ )
				{
					if ( ( byte & 31u ) == 0 ) busDiagEndLine( dst, capacity, n );
					busDiagHex( dst, capacity, n,
					            displayAddressMismatch[ mode ][ style ][ byte ] );
				}
				busDiagEndLine( dst, capacity, n );
			}
		}
		busDiagText( dst, capacity, n, "display-address ladder-ran: " );
		busDiagDecimal( dst, capacity, n, m_DisplayAddressLadderRan );
		busDiagEndLine( dst, capacity, n );
		if ( m_DisplayAddressLadderRan )
		{
			for ( u32 mode = 0; mode < 2; mode++ )
			{
				for ( u32 rung = 0; rung < 9; rung++ )
				{
					busDiagText( dst, capacity, n, "delay " );
					busDiagText( dst, capacity, n, modeNames[ mode ] );
					busDiagText( dst, capacity, n,
					             " requested-us/elapsed-us/expected/initial0/initial1/initial2/delayed0/delayed1/delayed2: " );
					busDiagDecimal( dst, capacity, n,
					                m_DisplayAddressDelayUsec[ rung ] );
					busDiagChar( dst, capacity, n, '/' );
					busDiagDecimal( dst, capacity, n,
					                m_DisplayAddressElapsedUsec[ mode ][ rung ] );
					busDiagChar( dst, capacity, n, '/' );
					busDiagHex( dst, capacity, n,
					            m_DisplayAddressLadderExpected[ mode ][ rung ] );
					for ( u32 read = 0; read < 3; read++ )
					{
						busDiagChar( dst, capacity, n, '/' );
						busDiagHex( dst, capacity, n,
						            m_DisplayAddressLadderInitial[ mode ][ rung ][ read ] );
					}
					for ( u32 read = 0; read < 3; read++ )
					{
						busDiagChar( dst, capacity, n, '/' );
						busDiagHex( dst, capacity, n,
						            m_DisplayAddressLadderDelayed[ mode ][ rung ][ read ] );
					}
					busDiagEndLine( dst, capacity, n );
				}
			}
		}
	}
	if ( m_DisplayRowRan )
		formatDisplayRowResults( dst, capacity, n );
	if ( m_DisplayFetchRan )
		formatDisplayFetchResults( dst, capacity, n );
	if ( m_DisplayPersistenceRan )
		formatDisplayPersistenceResults( dst, capacity, n );
	if ( m_DisplayTimingRan )
		formatDisplayTimingResults( dst, capacity, n );
	if ( m_DisplayBoundaryRan )
		formatDisplayBoundaryResults( dst, capacity, n );
	if ( m_DisplayRefreshRan )
		formatDisplayRefreshResults( dst, capacity, n );
	if ( m_DisplayBAGuardRan )
		formatDisplayBAGuardResults( dst, capacity, n );
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

	busDiagText( dst, capacity, n, "SID timing: configured=" );
	busDiagDecimal( dst, capacity, n, m_SIDTimingConfigured );
	if ( m_SIDTimingEdge )
	{
		busDiagText( dst, capacity, n, " selected=LAST-HIGH d/d/r=" );
		busDiagDecimal( dst, capacity, n, m_SIDTimingDistinct );
		busDiagChar( dst, capacity, n, '/' );
		busDiagDecimal( dst, capacity, n, m_SIDTimingDominant );
		busDiagChar( dst, capacity, n, '/' );
		busDiagDecimal( dst, capacity, n, m_SIDTimingRamp );
	}
	else if ( m_SIDTimingStart )
	{
		busDiagText( dst, capacity, n, " saw=" );
		busDiagDecimal( dst, capacity, n, m_SIDTimingStart );
		busDiagText( dst, capacity, n, ".." );
		busDiagDecimal( dst, capacity, n, m_SIDTimingEnd );
		busDiagText( dst, capacity, n, " selected=" );
		busDiagDecimal( dst, capacity, n, m_SIDTimingSelected );
		busDiagText( dst, capacity, n, " d/d/r=" );
		busDiagDecimal( dst, capacity, n, m_SIDTimingDistinct );
		busDiagChar( dst, capacity, n, '/' );
		busDiagDecimal( dst, capacity, n, m_SIDTimingDominant );
		busDiagChar( dst, capacity, n, '/' );
		busDiagDecimal( dst, capacity, n, m_SIDTimingRamp );
	}
	else
	{
		busDiagText( dst, capacity, n, " NO-SAW-WINDOW best=" );
		busDiagDecimal( dst, capacity, n, m_SIDTimingBestSample );
		busDiagText( dst, capacity, n, " d/d/r=" );
		busDiagDecimal( dst, capacity, n, m_SIDTimingDistinct );
		busDiagChar( dst, capacity, n, '/' );
		busDiagDecimal( dst, capacity, n, m_SIDTimingDominant );
		busDiagChar( dst, capacity, n, '/' );
		busDiagDecimal( dst, capacity, n, m_SIDTimingRamp );
	}
	busDiagEndLine( dst, capacity, n );
	if ( !m_SIDPhysicalReliable )
	{
		busDiagText( dst, capacity, n,
		             "SID fallback: OSC3=model POT=17-read-filter" );
		busDiagEndLine( dst, capacity, n );
	}

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

// The first transfer of a run is not trustworthy. Every diagnostic in this
// file already assumes it -- sacrificial writes before a marker, discarded
// first reads before a target -- but CRADBus::read() never needed the same
// care, because the write buffer kept the bus continuously busy and a read was
// effectively never first in its run.
//
// VIDEO_MODE 1 removed that traffic. With no DRAM mirroring the bus can sit
// idle for milliseconds between one keyboard scan and the next, so every CIA
// read became a cold first transfer, and a garbage keyboard matrix reads as
// keys being held down -- symbols typing themselves across the screen.
//
// So a read that follows an idle gap is primed with a throwaway transfer first.
// $02FE is the tail of the cassette buffer: RAM on every machine and unused
// while we hold the bus. It is the same address the calibration path primes
// with, for the same reason.
//
// The gap threshold matters, and 10us -- the first value shipped -- was a bug:
// it sits exactly on JiffyDOS's own rhythm. A JiffyDOS byte is timed reads
// spaced roughly 10-15us apart, so at a 10us threshold the prime fired randomly
// between bit reads, inserting a bus transaction that shifted the next sample
// late. 150us is above anything inside an active transfer and far below the
// interval between keyboard scans.
void CRADBus::primeAfterIdle()
{
	const u64 now  = hostCycles();
	const u64 rate = hostCyclesPerSec();
	const u64 idle = rate ? ( rate * 150 / 1000000 ) : 0;	// 150us

	if ( idle && ( now - m_LastTransferCycles ) >= idle )
	{
		u8 discard;
		RAD_SPEEK( 0x02FE, discard );
		(void)discard;
		m_ReadPrimes++;
		m_Transfers++;
	}
	m_LastTransferCycles = now;
}

u8 CRADBus::read( u16 addr )
{
	if ( m_TrafficHalted ) return 0xFF;
	u8 v = 0xFF;

	if ( !m_SIDPhysicalReliable && addr == 0xD41B )
		v = sidReadOSC3Model();
	else if ( !m_SIDPhysicalReliable && ( addr == 0xD419 || addr == 0xD41A ) )
		v = sidReadPOTFiltered( addr );
	else
	{
		// Do not reread the target itself: $DC0D and $DD0D clear interrupt
		// flags when read.  Prime through harmless RAM instead.
		primeAfterIdle();
		RAD_SPEEK( addr, v );
		m_Transfers++;
		if ( addr == 0xDD00 || addr == 0xDD02 ) m_SerialTransfers++;
	}
	m_Reads++;
	return v;
}

u8 CRADBus::readRAM( u16 addr )
{
	if ( m_TrafficHalted ) return 0xFF;

	// Keep the same cold-transfer protection and accounting as a normal read,
	// but bypass the per-access /GAME selector. This is used only to verify the
	// physical DRAM image (notably a bitmap crossing $D000) and must never touch
	// the side-effectful I/O chip at the same address.
	primeAfterIdle();
	const u8 v = radDirectRead( addr, true );
	m_LastTransferCycles = hostCycles();
	m_Transfers++;
	m_Reads++;
	return v;
}

void CRADBus::writeRAM( u16 addr, u8 value )
{
	if ( m_TrafficHalted ) return;

	// Same-polarity cold-write protection as write(), but deliberately leave
	// /GAME released so the prepared $01=$34 host stores into physical DRAM.
	const u64 nowW  = hostCycles();
	const u64 rateW = hostCyclesPerSec();
	const u64 idleW = rateW ? ( rateW * 150 / 1000000 ) : 0;
	if ( idleW && ( nowW - m_LastTransferCycles ) >= idleW )
	{
		radDirectWrite( 0x02FE, 0xA6, true );
		m_Transfers++;
		m_ReadPrimes++;
	}
	radDirectWrite( addr, value, true );
	m_LastTransferCycles = hostCycles();
	m_Transfers++;
	m_Writes++;
}

void CRADBus::write( u16 addr, u8 value )
{
	if ( m_TrafficHalted ) return;

	sidObserveWrite( addr, value );
	// Writes require a same-polarity prime.  A read prime here creates the
	// read-to-write turnaround which previously lost interrupt acknowledges
	// ($DC0D/$D019) and IEC handshakes ($DD00).
	{
		const u64 nowW  = hostCycles();
		const u64 rateW = hostCyclesPerSec();
		const u64 idleW = rateW ? ( rateW * 150 / 1000000 ) : 0;
		if ( idleW && ( nowW - m_LastTransferCycles ) >= idleW )
		{
			RAD_SPOKE( 0x02FE, 0xA6 );
			m_Transfers++;
			m_ReadPrimes++;
		}
	}
	RAD_SPOKE( addr, value );
	m_LastTransferCycles = hostCycles();
	m_Transfers++;
	if ( addr == 0xDD00 || addr == 0xDD02 ) m_SerialTransfers++;
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
		m_LastTransferCycles = hostCycles();
		m_Transfers += count;
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
	m_LastTransferCycles = hostCycles();
	m_Transfers += count;
}

void CRADBus::readBlock( u16 addr, u8 *dst, u32 length )
{
	if ( m_TrafficHalted )
	{
		for ( u32 i = 0; i < length; i++ ) dst[ i ] = 0xFF;
		return;
	}
	const bool observeKernal = addr == 0xE000 && length != 0;
	const bool observeBasic  = addr == 0xA000 && length != 0;
	for ( u32 i = 0; i < length; i++ )
	{
		u8 v = 0xFF;
		RAD_SPEEK( (u16)( addr + i ), v );
		dst[ i ] = v;
		if ( i == 0 && ( observeKernal || observeBasic ) )
		{
			// Preserve the actual unprimed byte in the snapshot, then reread
			// the same side-effect-free ROM location immediately. A mismatch
			// exposes byte-zero turnaround residue without silently fixing it.
			u8 reread = 0xFF;
			RAD_SPEEK( addr, reread );
			m_Reads++;
			if ( observeKernal )
			{
				m_SnapshotKernalFirst = v;
				m_SnapshotKernalReread = reread;
				m_SnapshotKernalCopied = dst[ 0 ];
				m_SnapshotKernalObserved = 1;
			}
			else
			{
				m_SnapshotBasicFirst = v;
				m_SnapshotBasicReread = reread;
				m_SnapshotBasicCopied = dst[ 0 ];
				m_SnapshotBasicObserved = 1;
			}
		}
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
	// Raster polling is real bus traffic, so it must both receive the cold-read
	// protection and warm the idle-transfer clock for the next caller.
	primeAfterIdle();
	for ( u32 attempt = 0; attempt < 4; attempt++ )
	{
		u8 hiBefore = 0, lo = 0, hiAfter = 0;

		RAD_SPEEK( 0xD011, hiBefore );
		RAD_SPEEK( 0xD012, lo );
		RAD_SPEEK( 0xD011, hiAfter );
		m_Reads += 3;
		m_Transfers += 3;
		m_LastTransferCycles = hostCycles();

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
