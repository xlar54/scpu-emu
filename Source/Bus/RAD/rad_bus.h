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
#ifndef _rad_bus_h
#define _rad_bus_h

#include "../../C64/c64_bus.h"

// Must match arm_freq in config.txt. The bus timings are counted in these
// cycles too, so this is the same clock the whole design is pinned to.
#define SCPU_ARM_CLOCK_HZ 1400000000u

class CLogger;
class CWriteBuffer;

class CRADBus : public IC64Bus
{
public:
	CRADBus();

	// Optional progress logging. acquire() is a sequence of steps that can each
	// stall or fail, and without this a failure anywhere in it looks identical
	// from the outside: a C64 that just sits there. Only phase boundaries are
	// logged, never anything inside a timed loop.
	void setLogger( CLogger *logger ) { m_Logger = logger; }

	// --- IC64Bus ----------------------------------------------------------
	bool acquire() override;
	void release() override;
	const C64Signals &signals() const override { return m_Signals; }

	u8   read( u16 addr ) override;
	void write( u16 addr, u8 value ) override;
	bool verifyC64CIA2DDRA( u8 expected ) override;
	void writeBurst( const C64BusWrite *writes, u32 count ) override;
	void readBlock( u16 addr, u8 *dst, u32 length ) override;

	bool irqAsserted() override;
	bool nmiAsserted() override;
	void sampleInterrupts( bool &irq, bool &nmi ) override;
	u16  rasterLine() override;

	u64  hostCycles() override;
	u32  hostCyclesPerSec() override { return SCPU_ARM_CLOCK_HZ; }

	const char *name() const override { return "RAD expansion unit"; }

	// Flash the border a few times, as visible proof that we hold the bus and
	// can drive it. Costs a fraction of a second and happens before the CPU
	// core starts, so it is unmistakably ours rather than anything the C64 did.
	// Without it a failed takeover and a successful one both just look like a
	// C64 sitting at a BASIC prompt.
	void signalAlive();

	// Exercise each bus operation against known values and report what works.
	// The border flash only proves single writes reach the VIC; this separates
	// single reads, single writes and burst writes from each other, which is
	// the difference between "the bus is broken" and "one code path is".
	// Returns true if everything passed.
	bool selfTest();
	u8 selfTestFailure() const { return m_SelfTestFailure; }
	bool c128KnownFirstTransferOnly() const;
	void logSelfTestResults( CLogger *logger ) const;
	u32 formatSelfTestResults( char *dst, u32 capacity ) const;
	// C64-only diagnostic: seed a broad physical-RAM sentinel and compare
	// equal-duration control, VIC-read and same-value VIC-write workloads.
	// Two readback passes distinguish stable modifications from an invasive
	// verifier. This deliberately destroys the test RAM and must be followed by
	// a halt, not a normal SuperCPU boot.
	bool runAccessSentinelDiagnostic();
	void logAccessSentinelResults( CLogger *logger ) const;
	// K222: make the oracle itself the active VIC hires bitmap during each
	// exposure, but blank DEN for seed, baseline and all three readback passes.
	bool runDisplaySentinelDiagnostic();
	void logDisplaySentinelResults( CLogger *logger ) const;
	// K223: distinguish an immediate write/address-placement failure from
	// delayed retention at K222's invariant first mismatch ($2078). It also
	// preserves complete mismatch maps for text/bitmap and single/double seeds.
	bool runDisplayAddressDiagnostic();
	void logDisplayAddressResults( CLogger *logger ) const;
	// K225: high-entropy, whole-map follow-up. It makes every matrix/bitmap
	// cell observable, compares single writes with true per-address repeats,
	// and runs independent map-wide retention intervals.
	bool runDisplayRowDiagnostic();
	void logDisplayRowResults( CLogger *logger ) const;
	// K226: repeat K222's active-display/traffic matrix with K225's
	// high-entropy oracle and verified complement setup. It also closes K225's
	// retention gap by delaying maps written exactly once.
	bool runDisplayFetchDiagnostic();
	void logDisplayFetchResults( CLogger *logger ) const;
	// K227: repeat every active text/bitmap traffic combination twice and take
	// three consecutive post-exposure maps. This separates persistent DRAM
	// modification from a transient/invasive first verifier pass.
	bool runDisplayPersistenceDiagnostic();
	void logDisplayPersistenceResults( CLogger *logger ) const;
	// K228: read the same exposed map at multiple sample points, restore the
	// configured point, then repair only its mismatches and verify again.
	bool runDisplayTimingDiagnostic();
	void logDisplayTimingResults( CLogger *logger ) const;
	// K229: distinguish corruption caused by the D011 display transitions from
	// corruption accumulated while the VIC actively fetches display data.
	bool runDisplayBoundaryDiagnostic();
	void logDisplayBoundaryResults( CLogger *logger ) const;
	// K230: A/B the same active-display dwell with /DMA held continuously and
	// with the core-3 VIC-half release primitive running continuously.
	bool runDisplayRefreshDiagnostic();
	// K231: repeat K230 with the refresh pulses emitted synchronously on core 0.
	// K230 proved that the core-3 helper stalls on this C64 host after 0-1 slots.
	bool runDisplayCore0RefreshDiagnostic();
	// K232: compare the normal high-impedance R/W idle state with R/W actively
	// held high during an active display dwell with no RAD transactions.
	bool runDisplayRWDiagnostic();
	// K233: compare long active-display controls against the production
	// A1=0/A3=1 shadow scrub, using the same CWriteBuffer implementation.
	bool runDisplayScrubDiagnostic( CWriteBuffer &scrubBuffer );
	// K239: identical composite bitmap oracle across repeated idle/read/write
	// arms. Write arms bias only the address-OE point after BA arbitration;
	// three separately primed captures distinguish stored changes from an
	// unstable physical read. Diagnostic-only and destructive to C64 RAM.
	bool runDisplayBAGuardDiagnostic();
	void logDisplayBAGuardResults( CLogger *logger ) const;
	void logDisplayRefreshResults( CLogger *logger ) const;
	// Experimental C128 DRAM-refresh fix: seed every byte in page $04, then
	// release /DMA only during each VIC-owned half-cycle and capture a complete
	// 256-cell coverage map without issuing synthetic RAM reads.
	void runRefreshFiniteTests();
	// Measure how long refresh may be suspended before physical C128 DRAM loses
	// the page-$04 oracle. Rungs are 0.25, 0.5, 1, 2, 4 and 8 ms, each after a
	// clean reseed and ten seconds of normal pulsing.
	void runRefreshGapLadder();
	// E18 diagnostic family: map the C128's physical VIC refresh requirement in
	// the production line-anchor coordinate system.
	bool runRefreshPhaseScan();
	bool refreshPhaseScanTrusted() const
		{ return m_PhaseScanStage1Agree && m_PhaseScanStage2Ran; }
	void renderRefreshPhaseScanSummary();
	// Display the captured 16x16 coverage map and keep VIC-half /DMA release
	// running forever. The caller must save formatSelfTestResults() first.
	void runRefreshDiagnosticHold();

	// Hold /DMA but drive nothing for roughly the given number of seconds, so
	// the C64's display can be observed with zero bus traffic from us.
	void idleHold( u32 seconds );

	// Same, but returns early if the button is pressed. Never use this for the
	// post-reset settle -- a short wait there breaks the handover.
	void idleHoldInterruptible( u32 seconds );

	// True while the RAD's button is held. Reading it is free -- no bus cycle,
	// just a GPIO level -- so it is safe to poll from the run loop.
	bool buttonPressed();

	// True while the C64's /RESET line is being pulled low by something other
	// than us -- i.e. the RAD's hardware reset button. That button is wired
	// straight to the reset line and never reaches the Pi as a button, but we
	// can see its effect because /RESET is an input to us between accesses.
	bool hardwareResetPressed();

	// Statistics, useful for judging how much bus bandwidth a workload burns.
	u64 m_Reads, m_Writes, m_BurstWrites;

	bool acquired() const { return m_Acquired; }
	void setTrafficHalted( bool halted );
	bool trafficHalted() const { return m_TrafficHalted; }
	// With every RAD driver disabled and /DMA still asserted, sample CPU-half
	// cycles for evidence that another host CPU is driving the expansion bus.
	void probeIdleMaster( u32 &eligiblePhases, u32 &addrChanges,
	                      u32 &distinctAddrs, u32 &sequentialAddrs,
	                      u32 &writePhases, u8 &firstAddr, u8 &lastAddr );

private:
	void formatDisplayRowResults( char *dst, u32 capacity, u32 &length ) const;
	void formatDisplayFetchResults( char *dst, u32 capacity, u32 &length ) const;
	void formatDisplayPersistenceResults( char *dst, u32 capacity,
	                                      u32 &length ) const;
	void formatDisplayTimingResults( char *dst, u32 capacity,
	                                 u32 &length ) const;
	void formatDisplayBoundaryResults( char *dst, u32 capacity,
	                                   u32 &length ) const;
	void formatDisplayRefreshResults( char *dst, u32 capacity,
	                                  u32 &length ) const;
	void formatDisplayBAGuardResults( char *dst, u32 capacity,
	                                u32 &length ) const;
	bool runDisplayRefreshDiagnosticVariant( u8 variant,
	                                         CWriteBuffer *scrubBuffer = 0 );
	// Find the stable ordinary-read sampling window immediately after /DMA
	// takeover, before machine detection and ROM snapshots use that path.
	// SID reads retain their separate, later configured sample point.
	void calibrateReadTiming();
	// Use calibrated RAM reads as the oracle for the two independent write
	// timing points. Both single and burst paths must pass at a candidate.
	void calibrateWriteTiming();
	// SID reads need a later sample than RAM/VIC reads and cannot use a
	// write/read-back oracle. Run voice 3's saw oscillator and identify the
	// contiguous sample window where OSC3 has a plausible ramp instead of
	// open-bus or bus-transition data.
	void calibrateSIDReadTiming();
	// A validated physical SID window remains authoritative. If calibration
	// cannot find one, OSC3 is advanced from host time and the slow-changing
	// paddle registers are recovered by repeated physical sampling.
	void sidModelAdvance();
	void sidObserveWrite( u16 addr, u8 value );
	u8 sidReadOSC3Model();
	u8 sidReadPOTFiltered( u16 addr );
	// Acquire a real raster transition while core 3 holds a bounded exclusive
	// traffic interval, then hand that phase to the guarded per-line refresh
	// scheduler. The coarse E13 scheduler remains active if acquisition fails.
	bool startC128LineRefresh();
	void runFirstTransferDiscriminator();
	void seedRefreshMap( u8 border );
	void seedRefreshOracle( u8 salt );
	void captureRefreshMap();
	u32 core0RefreshRasterLine();
	u32 core0RefreshFullFrame();
	bool runVICRefreshSlot();
	void renderRefreshMap();
	bool prepareRefreshControl( u32 rung, u8 salt );
	void recordRefreshControl( u32 rung );
	bool runRefreshWindowControl( u32 rung, u32 k, u32 w,
	                              u32 exposureLines );
	bool runRefreshContinuousControl( u32 rung, u32 exposureSeconds, u8 salt );
	bool runRefreshCore0ContinuousControl( u32 rung, u32 exposureSeconds,
	                                      u8 salt );
	bool runRefreshBlackoutLongControl( u32 rung, u8 salt );
	bool runRefreshContinuousWarmup( u32 exposureSeconds );
	bool refreshControlStrict( u32 rung ) const;
	bool refreshWindowAccountingExact( u32 rung, u32 expectedPulses,
	                                   u32 expectedLines ) const;
	bool refreshExposureControlAcceptable( u32 rung, u32 expectedPulses,
	                                       u32 expectedLines ) const;

	enum RefreshControlSlot
	{
		REFRESH_PRE_FLOOR0 = 0,
		REFRESH_PRE_FLOOR1,
		REFRESH_PRE_BLACKOUT_LONG,
		REFRESH_PRE_CONT2,
		REFRESH_PRE_CORE0_CONT2,
		REFRESH_PRE_CONT8_A,
		REFRESH_PRE_KWFULL2,
		REFRESH_PRE_CONT8_B,
		REFRESH_TOPOLOGY_SCRATCH,
		REFRESH_INTERLEAVE_16,
		REFRESH_INTERLEAVE_32,
		REFRESH_INTERLEAVE_48,
		REFRESH_INTERLEAVE_64,
		REFRESH_POST_FLOOR0,
		REFRESH_POST_FLOOR1,
		REFRESH_POST_CONT2,
		REFRESH_POST_KWFULL2,
		REFRESH_CONTROL_COUNT
	};

	C64Signals m_Signals;
	bool       m_Acquired;
	bool       m_TrafficHalted;
	u8         m_SelfTestFailure; // bit 0: single R/W, bit 1: burst, bit 2: first-transfer series
	u16        m_ReadTimingConfigured, m_ReadTimingStart, m_ReadTimingEnd,
	           m_ReadTimingSelected;
	static const u32 READ_TIMING_SCORE_COUNT = 65;
	static const u32 READ_TIMING_REPETITIONS = 12;
	u16        m_ReadTimingErrors[ READ_TIMING_SCORE_COUNT ];
	u16        m_ReadTimingBestError, m_ReadTimingBestErrorSample;
	// Observation-only split of the mixed RAM/VIC calibration. Production
	// selection continues to use m_ReadTimingErrors until this identifies why
	// $D020 contributes one failure per repetition on physical C64s.
	u16        m_ReadTimingRAMErrors[ READ_TIMING_SCORE_COUNT ];
	u16        m_ReadTimingRAMOnlyErrors[ READ_TIMING_SCORE_COUNT ];
	u16        m_ReadTimingMixedVICErrors[ READ_TIMING_SCORE_COUNT ];
	u16        m_ReadTimingIsolatedVICErrors[ READ_TIMING_SCORE_COUNT ];
	u8         m_ReadTimingMixedVICActual
	              [ READ_TIMING_SCORE_COUNT ][ READ_TIMING_REPETITIONS ];
	u8         m_ReadTimingIsolatedVICActual
	              [ READ_TIMING_SCORE_COUNT ][ READ_TIMING_REPETITIONS ];
	u8         m_ReadTimingMixedVICDistinct[ READ_TIMING_SCORE_COUNT ];
	u8         m_ReadTimingMixedVICDominantValue[ READ_TIMING_SCORE_COUNT ];
	u8         m_ReadTimingMixedVICDominantCount[ READ_TIMING_SCORE_COUNT ];
	u8         m_ReadTimingIsolatedVICDistinct[ READ_TIMING_SCORE_COUNT ];
	u8         m_ReadTimingIsolatedVICDominantValue[ READ_TIMING_SCORE_COUNT ];
	u8         m_ReadTimingIsolatedVICDominantCount[ READ_TIMING_SCORE_COUNT ];
	u16        m_ReadTimingRAMBestError, m_ReadTimingRAMBestSample;
	u16        m_ReadTimingRAMOnlyBestError, m_ReadTimingRAMOnlyBestSample;
	// Rotate the six mixed-chain RAM addresses through all six sequence
	// positions. If failures follow position zero the cause is device-to-RAM
	// turnaround; if they follow one address the cause is address-specific.
	u8         m_ReadTimingMixedRAMPositionErrors
	              [ READ_TIMING_SCORE_COUNT ][ 6 ];
	u8         m_ReadTimingMixedRAMAddressErrors
	              [ READ_TIMING_SCORE_COUNT ][ 6 ];
	// The ROM snapshot keeps its original first byte. A side-effect-free
	// immediate reread records whether that unprimed byte was residue.
	u8         m_SnapshotKernalFirst, m_SnapshotKernalReread,
	           m_SnapshotKernalCopied;
	u8         m_SnapshotBasicFirst, m_SnapshotBasicReread,
	           m_SnapshotBasicCopied;
	u8         m_SnapshotKernalObserved, m_SnapshotBasicObserved;
	u16        m_WriteTimingConfiguredAddr, m_WriteTimingConfiguredData,
	           m_WriteTimingSelectedAddr, m_WriteTimingSelectedData,
	           m_WriteTimingPassingPoints;
	u16        m_SIDTimingConfigured, m_SIDTimingStart, m_SIDTimingEnd,
	           m_SIDTimingSelected, m_SIDTimingBestSample;
	u8         m_SIDTimingDistinct, m_SIDTimingDominant, m_SIDTimingRamp,
	           m_SIDTimingEdge;
	bool       m_SIDPhysicalReliable;
	u16        m_SIDModelFreq;
	u8         m_SIDModelControl;
	u32        m_SIDModelPhase;
	u64        m_SIDModelLastHost, m_SIDModelRemainder;
	u8         m_SIDPotFiltered[ 2 ];
	u8         m_TestRaster[ 6 ];
	u8         m_TestAddrLines[ 6 ];
	u8         m_TestSingle[ 6 ];
	u8         m_TestBurst[ 6 ];
	u8         m_TestAddrFirstReread, m_TestSingleFirstReread,
	           m_TestBurstFirstReread;
	u8         m_PosRawBeforeSingle[ 2 ];
	u8         m_TurnBaseTarget[ 3 ][ 16 ];
	u8         m_TurnSingleTarget[ 3 ][ 16 ];
	u8         m_TurnDoubleTarget[ 3 ][ 16 ];
	u8         m_TurnSinglePostFlush[ 3 ][ 3 ];
	u8         m_TurnDoublePostFlush[ 3 ][ 3 ];
	u8         m_TurnBaseNeighbor[ 3 ][ 10 ];
	u8         m_TurnSingleNeighbor[ 3 ][ 10 ];
	u8         m_TurnDoubleNeighbor[ 3 ][ 10 ];
	u8         m_TurnFlags[ 3 ][ 14 ];
	u8         m_TurnSingleFlush[ 3 ], m_TurnDoubleFlush[ 3 ];
	u8         m_TurnDetectedMachine;
	u8         m_RefreshMapExpected[ 256 ];
	u8         m_RefreshMapReadback[ 3 ][ 256 ];
	u8         m_RefreshControlExpected[ 8 ];
	u8         m_RefreshControlReadback[ 3 ][ 8 ];
	u8         m_RefreshFailureBitmap[ 32 ];
	u8         m_RefreshSweepChecksum;
	u32        m_RefreshSweepCount;
	u32        m_RefreshElapsedMs;
	u32        m_RefreshPassUsec;
	u32        m_RefreshFailureCount;
	u32        m_RefreshUnstableCount;
	u32        m_RefreshHostWritePhases;
	u16        m_RefreshGapFailures[ 6 ];
	u16        m_RefreshGapUnstable[ 6 ];
	u16        m_PhaseScanStage1Failures[ 2 ][ 8 ];
	u16        m_PhaseScanStage1Unstable[ 2 ][ 8 ];
	u32        m_PhaseScanStage1AnchorArm[ 2 ][ 8 ];
	u8         m_PhaseScanStage1AnchorPhi[ 2 ][ 8 ];
	u16        m_PhaseScanStage2Failures[ 65 ];
	u16        m_PhaseScanStage2BaselineFailures[ 65 ];
	u16        m_PhaseScanStage2BaselineUnstable[ 65 ];
	u8         m_PhaseScanStage2BaselineAttempts[ 65 ];
	u8         m_PhaseScanStage2ExecutionIndex[ 65 ];
	u8         m_PhaseScanExecutionK[ 65 ];
	u32        m_PhaseScanStage2BaselineSeedUsec[ 65 ];
	u32        m_PhaseScanStage2BaselineCaptureUsec[ 65 ];
	u32        m_PhaseScanStage2CaptureUsec[ 65 ];
	u16        m_PhaseScanStage2PassFailures[ 65 ][ 3 ];
	u16        m_PhaseScanStage2Unstable[ 65 ];
	u8         m_PhaseScanStage2MismatchMask[ 65 ][ 3 ][ 33 ];
	u8         m_PhaseScanStage2Class[ 65 ];
	u32        m_PhaseScanStage2AnchorArm[ 65 ];
	u32        m_PhaseScanStage2Pulses[ 65 ];
	u32        m_PhaseScanStage2Lines[ 65 ];
	u8         m_PhaseScanStage2AnchorPhi[ 65 ];
	u8         m_PhaseScanRan, m_PhaseScanStage1Agree,
	           m_PhaseScanStage2Ran, m_PhaseScanStage2Contiguous,
	           m_PhaseScanStage2PassCount, m_PhaseScanStage2FailureCount,
	           m_PhaseScanStage2FlagCount, m_PhaseScanStage2RungsCompleted,
	           m_PhaseScanStage2BandStart, m_PhaseScanLineCycles;
	u16        m_PhaseScanBaselineFailures, m_PhaseScanBaselineUnstable;
	u8         m_PhaseScanBaselineK, m_PhaseScanBaselineW,
	           m_PhaseScanBaselineAttempts;
	// E18M retains every pre-, interleaved-, and post-scan control. The
	// topology scratch slot is copied losslessly into the dedicated 65-rung
	// arrays before it is reused.
	u8         m_RefreshControlValid[ REFRESH_CONTROL_COUNT ];
	u8         m_RefreshControlBaselineAttempts[ REFRESH_CONTROL_COUNT ];
	u16        m_RefreshControlBaselineFailures[ REFRESH_CONTROL_COUNT ];
	u16        m_RefreshControlBaselineUnstable[ REFRESH_CONTROL_COUNT ];
	u16        m_RefreshControlBaselinePassFailures
	             [ REFRESH_CONTROL_COUNT ][ 2 ][ 3 ];
	u16        m_RefreshControlBaselinePassUnstable
	             [ REFRESH_CONTROL_COUNT ][ 2 ][ 2 ];
	u32        m_RefreshControlBaselineFramePulses
	             [ REFRESH_CONTROL_COUNT ][ 2 ];
	u32        m_RefreshControlBaselineFrameExpected
	             [ REFRESH_CONTROL_COUNT ][ 2 ];
	u16        m_RefreshControlBaselineFrameSweeps
	             [ REFRESH_CONTROL_COUNT ][ 2 ];
	u32        m_RefreshControlBaselineSeedUsec
	             [ REFRESH_CONTROL_COUNT ][ 2 ];
	u32        m_RefreshControlBaselineCaptureUsec
	             [ REFRESH_CONTROL_COUNT ][ 2 ];
	u32        m_RefreshControlBaselinePassUsec
	             [ REFRESH_CONTROL_COUNT ][ 2 ][ 3 ];
	u16        m_RefreshControlPassFailures[ REFRESH_CONTROL_COUNT ][ 3 ];
	u16        m_RefreshControlUnstable01[ REFRESH_CONTROL_COUNT ];
	u16        m_RefreshControlUnstable12[ REFRESH_CONTROL_COUNT ];
	u8         m_RefreshControlMismatchMask[ REFRESH_CONTROL_COUNT ][ 3 ][ 33 ];
	u32        m_RefreshControlPulses[ REFRESH_CONTROL_COUNT ];
	u32        m_RefreshControlExpectedPulses[ REFRESH_CONTROL_COUNT ];
	u32        m_RefreshControlExposureLines[ REFRESH_CONTROL_COUNT ];
	u32        m_RefreshControlExposureUsec[ REFRESH_CONTROL_COUNT ];
	u32        m_RefreshControlCaptureUsec[ REFRESH_CONTROL_COUNT ];
	u32        m_RefreshControlCapturePassUsec
	             [ REFRESH_CONTROL_COUNT ][ 3 ];
	u32        m_RefreshControlAnchorArm[ REFRESH_CONTROL_COUNT ];
	u8         m_RefreshControlAnchorPhi[ REFRESH_CONTROL_COUNT ];
	u32        m_RefreshWarmupPulses;
	u32        m_LastRefreshSeedUsec, m_LastRefreshCaptureUsec,
	           m_LastRefreshCapturePassUsec[ 3 ];
	u32        m_BlackoutLongDelayUsec;
	u32        m_Core0FrameSweeps, m_Core0FramePulses,
	           m_Core0FrameExpectedPulses, m_Core0FrameAccountingErrors,
	           m_Core0FrameMinUsec, m_Core0FrameMaxUsec;
	static const u32 REFRESH_PULSE_SCAN_COUNT = 7;
	u8         m_PulseScanRan, m_PulseScanCompleted;
	u16        m_PulseScanWidth[ REFRESH_PULSE_SCAN_COUNT ];
	u8         m_TestBankA, m_TestBankE;
	u8         m_TestRasterChanged, m_TestAddrErrors, m_TestSingleErrors,
	           m_TestBurstErrors, m_TestRepeatedFailures;
	bool       m_AccessSentinelRan;
	u32        m_AccessSentinelTrafficCount;
	u32        m_AccessSentinelBaselineErrors[ 3 ][ 2 ];
	u32        m_AccessSentinelExposureErrors[ 3 ][ 2 ];
	u32        m_AccessSentinelBaselineRetainedErrors[ 3 ];
	u32        m_AccessSentinelArmAddedErrors[ 3 ];
	u32        m_AccessSentinelArmRemovedErrors[ 3 ];
	u32        m_AccessSentinelSameErrors[ 3 ];
	u32        m_AccessSentinelNewErrors[ 3 ];
	u32        m_AccessSentinelClearedErrors[ 3 ];
	u32        m_AccessSentinelElapsedUsec[ 3 ];
	u16        m_AccessSentinelFirstAddr[ 3 ][ 2 ];
	u8         m_AccessSentinelFirstExpected[ 3 ][ 2 ];
	u8         m_AccessSentinelFirstActual[ 3 ][ 2 ];
	bool       m_DisplaySentinelRan;
	u8         m_DisplaySentinelDisplayOn[ 9 ];
	u8         m_DisplaySentinelOperation[ 9 ]; // 0 none, 1 read, 2 write
	u32        m_DisplaySentinelRate[ 9 ];
	u32        m_DisplaySentinelTrafficCount[ 9 ];
	u32        m_DisplaySentinelElapsedUsec[ 9 ];
	u32        m_DisplaySentinelBaselineErrors[ 9 ][ 2 ];
	u32        m_DisplaySentinelExposureErrors[ 9 ][ 3 ];
	u32        m_DisplaySentinelBaselineRetainedErrors[ 9 ];
	u32        m_DisplaySentinelArmAddedErrors[ 9 ];
	u32        m_DisplaySentinelArmRemovedErrors[ 9 ];
	u32        m_DisplaySentinelVerifySameErrors[ 9 ][ 2 ];
	u32        m_DisplaySentinelVerifyNewErrors[ 9 ][ 2 ];
	u32        m_DisplaySentinelVerifyClearedErrors[ 9 ][ 2 ];
	u16        m_DisplaySentinelFirstAddr[ 9 ][ 3 ];
	u8         m_DisplaySentinelFirstExpected[ 9 ][ 3 ];
	u8         m_DisplaySentinelFirstActual[ 9 ][ 3 ];
	bool       m_DisplayAddressRan;
	u32        m_DisplayAddressImmediateErrors[ 2 ][ 2 ][ 3 ];
	u16        m_DisplayAddressImmediateFirstTrial[ 2 ][ 2 ];
	u8         m_DisplayAddressImmediateFirstExpected[ 2 ][ 2 ];
	u8         m_DisplayAddressImmediateFirstActual[ 2 ][ 2 ][ 3 ];
	u32        m_DisplayAddressMapErrors[ 2 ][ 2 ];
	u16        m_DisplayAddressMapFirst[ 2 ][ 2 ];
	u16        m_DisplayAddressMapLast[ 2 ][ 2 ];
	u16        m_DisplayAddressMapAND[ 2 ][ 2 ];
	u16        m_DisplayAddressMapOR[ 2 ][ 2 ];
	u16        m_DisplayAddressMapRuns[ 2 ][ 2 ];
	u16        m_DisplayAddressMapMaxRun[ 2 ][ 2 ];
	u8         m_DisplayAddressLadderRan;
	u32        m_DisplayAddressDelayUsec[ 9 ];
	u32        m_DisplayAddressElapsedUsec[ 2 ][ 9 ];
	u8         m_DisplayAddressLadderExpected[ 2 ][ 9 ];
	u8         m_DisplayAddressLadderInitial[ 2 ][ 9 ][ 3 ];
	u8         m_DisplayAddressLadderDelayed[ 2 ][ 9 ][ 3 ];
	bool       m_DisplayRowRan;
	u8         m_DisplayRowMode[ 10 ];       // 0 text, 1 bitmap
	u8         m_DisplayRowKind[ 10 ];       // 0 single, 1 adjacent repeat
	u8         m_DisplayRowSalt[ 10 ];
	u8         m_DisplayRowPrefillAttempts[ 10 ];
	u32        m_DisplayRowSeedUsec[ 10 ][ 2 ];
	u32        m_DisplayRowCaptureUsec[ 10 ][ 2 ];
	u32        m_DisplayRowErrors[ 10 ][ 2 ]; // prefill, tested map
	u16        m_DisplayRowFirst[ 10 ][ 2 ];
	u16        m_DisplayRowLast[ 10 ][ 2 ];
	u16        m_DisplayRowAND[ 10 ][ 2 ];
	u16        m_DisplayRowOR[ 10 ][ 2 ];
	u16        m_DisplayRowRuns[ 10 ][ 2 ];
	u16        m_DisplayRowMaxRun[ 10 ][ 2 ];
	u8         m_DisplayRowRetentionSalt[ 6 ];
	u8         m_DisplayRowRetentionAttempts[ 6 ];
	u32        m_DisplayRowRetentionDelayUsec[ 6 ];
	u32        m_DisplayRowRetentionElapsedUsec[ 6 ];
	u32        m_DisplayRowRetentionSeedUsec[ 6 ];
	u32        m_DisplayRowRetentionCaptureUsec[ 6 ][ 2 ];
	u32        m_DisplayRowRetentionErrors[ 6 ][ 2 ];
	u16        m_DisplayRowRetentionFirst[ 6 ][ 2 ];
	u16        m_DisplayRowRetentionLast[ 6 ][ 2 ];
	u16        m_DisplayRowRetentionAND[ 6 ][ 2 ];
	u16        m_DisplayRowRetentionOR[ 6 ][ 2 ];
	u16        m_DisplayRowRetentionRuns[ 6 ][ 2 ];
	u16        m_DisplayRowRetentionMaxRun[ 6 ][ 2 ];
	bool       m_DisplayFetchRan;
	u8         m_DisplayFetchState[ 9 ];       // 0 DEN-off, 1 text, 2 bitmap
	u8         m_DisplayFetchOperation[ 9 ];   // 0 none, 1 read, 2 write
	u8         m_DisplayFetchSalt[ 9 ];
	u8         m_DisplayFetchPrefillAttempts[ 9 ];
	u32        m_DisplayFetchRate[ 9 ];
	u32        m_DisplayFetchTrafficCount[ 9 ];
	u32        m_DisplayFetchElapsedUsec[ 9 ];
	u32        m_DisplayFetchSeedUsec[ 9 ][ 2 ]; // complement, single target
	u32        m_DisplayFetchCaptureUsec[ 9 ][ 3 ]; // prefill, baseline, post
	u32        m_DisplayFetchErrors[ 9 ][ 3 ];
	u16        m_DisplayFetchFirst[ 9 ][ 3 ];
	u16        m_DisplayFetchLast[ 9 ][ 3 ];
	u16        m_DisplayFetchAND[ 9 ][ 3 ];
	u16        m_DisplayFetchOR[ 9 ][ 3 ];
	u16        m_DisplayFetchRuns[ 9 ][ 3 ];
	u16        m_DisplayFetchMaxRun[ 9 ][ 3 ];
	u8         m_DisplayFetchRetentionSalt[ 6 ];
	u8         m_DisplayFetchRetentionPrefillAttempts[ 6 ];
	u32        m_DisplayFetchRetentionDelayUsec[ 6 ];
	u32        m_DisplayFetchRetentionElapsedUsec[ 6 ];
	u32        m_DisplayFetchRetentionSeedUsec[ 6 ][ 2 ];
	u32        m_DisplayFetchRetentionCaptureUsec[ 6 ][ 3 ];
	u32        m_DisplayFetchRetentionErrors[ 6 ][ 3 ];
	u16        m_DisplayFetchRetentionFirst[ 6 ][ 3 ];
	u16        m_DisplayFetchRetentionLast[ 6 ][ 3 ];
	u16        m_DisplayFetchRetentionAND[ 6 ][ 3 ];
	u16        m_DisplayFetchRetentionOR[ 6 ][ 3 ];
	u16        m_DisplayFetchRetentionRuns[ 6 ][ 3 ];
	u16        m_DisplayFetchRetentionMaxRun[ 6 ][ 3 ];
	bool       m_DisplayPersistenceRan;
	u8         m_DisplayPersistenceState[ 12 ];
	u8         m_DisplayPersistenceOperation[ 12 ];
	u8         m_DisplayPersistenceSalt[ 12 ];
	u8         m_DisplayPersistencePrefillAttempts[ 12 ];
	u32        m_DisplayPersistenceTrafficCount[ 12 ];
	u32        m_DisplayPersistenceElapsedUsec[ 12 ];
	u32        m_DisplayPersistenceSeedUsec[ 12 ][ 2 ];
	u32        m_DisplayPersistenceCaptureUsec[ 12 ][ 5 ];
	u32        m_DisplayPersistenceErrors[ 12 ][ 5 ];
	u16        m_DisplayPersistenceFirst[ 12 ][ 5 ];
	u16        m_DisplayPersistenceLast[ 12 ][ 5 ];
	u16        m_DisplayPersistenceAND[ 12 ][ 5 ];
	u16        m_DisplayPersistenceOR[ 12 ][ 5 ];
	u16        m_DisplayPersistenceRuns[ 12 ][ 5 ];
	u16        m_DisplayPersistenceMaxRun[ 12 ][ 5 ];
	u8         m_DisplayPersistenceFirstExpected[ 12 ][ 5 ];
	u8         m_DisplayPersistenceFirstActual[ 12 ][ 5 ];
	u8         m_DisplayPersistenceXorAND[ 12 ][ 5 ];
	u8         m_DisplayPersistenceXorOR[ 12 ][ 5 ];
	u32        m_DisplayPersistenceSame[ 12 ][ 2 ];
	u32        m_DisplayPersistenceAdded[ 12 ][ 2 ];
	u32        m_DisplayPersistenceRemoved[ 12 ][ 2 ];
	bool       m_DisplayTimingRan;
	u8         m_DisplayTimingState[ 6 ];
	u8         m_DisplayTimingOperation[ 6 ];
	u8         m_DisplayTimingSalt[ 6 ];
	u8         m_DisplayTimingPrefillAttempts[ 6 ];
	u32        m_DisplayTimingPrefillErrors[ 6 ];
	u32        m_DisplayTimingBaselineErrors[ 6 ];
	u32        m_DisplayTimingTrafficCount[ 6 ];
	u32        m_DisplayTimingElapsedUsec[ 6 ];
	u16        m_DisplayTimingSample[ 10 ];
	u32        m_DisplayTimingCaptureUsec[ 6 ][ 10 ];
	u32        m_DisplayTimingErrors[ 6 ][ 10 ];
	u16        m_DisplayTimingFirst[ 6 ][ 10 ];
	u16        m_DisplayTimingLast[ 6 ][ 10 ];
	u16        m_DisplayTimingAND[ 6 ][ 10 ];
	u16        m_DisplayTimingOR[ 6 ][ 10 ];
	u16        m_DisplayTimingRuns[ 6 ][ 10 ];
	u16        m_DisplayTimingMaxRun[ 6 ][ 10 ];
	u8         m_DisplayTimingFirstExpected[ 6 ][ 10 ];
	u8         m_DisplayTimingFirstActual[ 6 ][ 10 ];
	u8         m_DisplayTimingXorAND[ 6 ][ 10 ];
	u8         m_DisplayTimingXorOR[ 6 ][ 10 ];
	u32        m_DisplayTimingSame[ 6 ][ 10 ];
	u32        m_DisplayTimingAdded[ 6 ][ 10 ];
	u32        m_DisplayTimingRemoved[ 6 ][ 10 ];
	u32        m_DisplayTimingRepairWrites[ 6 ];
	bool       m_DisplayBoundaryRan;
	u8         m_DisplayBoundaryState[ 16 ];
	u8         m_DisplayBoundarySafe[ 16 ];
	u8         m_DisplayBoundaryDwell[ 16 ];
	u8         m_DisplayBoundarySalt[ 16 ];
	u8         m_DisplayBoundaryPrefillAttempts[ 16 ];
	u32        m_DisplayBoundaryPrefillErrors[ 16 ];
	u32        m_DisplayBoundaryElapsedUsec[ 16 ];
	u32        m_DisplayBoundaryWaitReads[ 16 ][ 2 ];
	u32        m_DisplayBoundaryCaptureUsec[ 16 ][ 2 ];
	u32        m_DisplayBoundaryErrors[ 16 ][ 2 ];
	u16        m_DisplayBoundaryFirst[ 16 ][ 2 ];
	u16        m_DisplayBoundaryLast[ 16 ][ 2 ];
	u16        m_DisplayBoundaryAND[ 16 ][ 2 ];
	u16        m_DisplayBoundaryOR[ 16 ][ 2 ];
	u16        m_DisplayBoundaryRuns[ 16 ][ 2 ];
	u16        m_DisplayBoundaryMaxRun[ 16 ][ 2 ];
	u8         m_DisplayBoundaryFirstExpected[ 16 ][ 2 ];
	u8         m_DisplayBoundaryFirstActual[ 16 ][ 2 ];
	u8         m_DisplayBoundaryXorAND[ 16 ][ 2 ];
	u8         m_DisplayBoundaryXorOR[ 16 ][ 2 ];
	u32        m_DisplayBoundarySame[ 16 ];
	u32        m_DisplayBoundaryAdded[ 16 ];
	u32        m_DisplayBoundaryRemoved[ 16 ];
	bool       m_DisplayRefreshRan;
	u8         m_DisplayDiagnosticVariant;
	u8         m_DisplayRefreshEnabled[ 8 ];
	u8         m_DisplayRefreshStartOK[ 8 ];
	u64        m_DisplayRefreshSlots[ 8 ];
	u32        m_DisplayScrubOpportunities[ 8 ];
	u32        m_DisplayScrubMaxChunkCycles[ 8 ];
	bool       m_DisplayBAGuardRan;
	static const u32 DISPLAY_BA_ARM_COUNT = 12;
	static const u32 DISPLAY_BA_FAMILY_COUNT = 5; // matrix, 00, FF, alternating, entropy
	u8         m_DisplayBAOperation[ DISPLAY_BA_ARM_COUNT ]; // 0 idle, 1 read, 2 write
	u8         m_DisplayBABias[ DISPLAY_BA_ARM_COUNT ];
	u8         m_DisplayBARotation[ DISPLAY_BA_ARM_COUNT ];
	u8         m_DisplayBAPrefillAttempts[ DISPLAY_BA_ARM_COUNT ];
	u8         m_DisplayBATargetAttempts[ DISPLAY_BA_ARM_COUNT ];
	u32        m_DisplayBATrafficCount[ DISPLAY_BA_ARM_COUNT ];
	u32        m_DisplayBAElapsedUsec[ DISPLAY_BA_ARM_COUNT ];
	u32        m_DisplayBACaptureUsec[ DISPLAY_BA_ARM_COUNT ][ 6 ];
	u32        m_DisplayBABaselinePersistent[ DISPLAY_BA_ARM_COUNT ];
	u32        m_DisplayBABaselineUnion[ DISPLAY_BA_ARM_COUNT ];
	u32        m_DisplayBAPostPersistent[ DISPLAY_BA_ARM_COUNT ];
	u32        m_DisplayBAPostUnion[ DISPLAY_BA_ARM_COUNT ];
	u32        m_DisplayBAAddedPersistent[ DISPLAY_BA_ARM_COUNT ];
	u32        m_DisplayBAUnstableOnly[ DISPLAY_BA_ARM_COUNT ];
	u32        m_DisplayBAFamilyAdded[ DISPLAY_BA_ARM_COUNT ][ DISPLAY_BA_FAMILY_COUNT ];
	// Raw first-transfer classifier, three targets. Sample columns are:
	// old,new,pre-lag,pre,immediate,control,final,settled. Count columns are:
	// success,write-lost,first-read-only,false-immediate-pass,
	// one-read-behind-control,other.
	u8         m_FirstDiscSample[ 3 ][ 8 ];
	u16        m_FirstDiscCount[ 3 ][ 6 ];
	CLogger   *m_Logger;
};

#endif
