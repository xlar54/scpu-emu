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

class CRADBus : public IC64Bus
{
public:
	CRADBus();

	// Optional progress logging. acquire() is a sequence of steps that can each
	// stall or fail, and without this a failure anywhere in it looks identical
	// from the outside: a C64 that just sits there. Only phase boundaries are
	// logged, never anything inside a timed loop.
	void setLogger( CLogger *logger ) { m_Logger = logger; }
	// FPGA C64U compatibility: use RAD's original reset, three-second KERNAL
	// settle and badline takeover instead of the C128-safe Ultimax sequence.
	void setC64ULegacyTakeover( bool enabled )
		{ m_C64ULegacyTakeover = enabled; }

	// --- IC64Bus ----------------------------------------------------------
	bool acquire() override;
	void release() override;
	const C64Signals &signals() const override { return m_Signals; }

	u8   read( u16 addr ) override;
	void write( u16 addr, u8 value ) override;
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
	// Find the stable ordinary-read sampling window immediately after /DMA
	// takeover, before machine detection and ROM snapshots use that path.
	// SID reads retain their separate, later configured sample point.
	void calibrateReadTiming();
	// Use calibrated RAM reads as the oracle for the two independent write
	// timing points. Both single and burst paths must pass at a candidate.
	void calibrateWriteTiming();
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
	bool       m_C64ULegacyTakeover;
	bool       m_TrafficHalted;
	u8         m_SelfTestFailure; // bit 0: single R/W, bit 1: burst, bit 2: first-transfer series
	u16        m_ReadTimingConfigured, m_ReadTimingStart, m_ReadTimingEnd,
	           m_ReadTimingSelected;
	u16        m_WriteTimingConfiguredAddr, m_WriteTimingConfiguredData,
	           m_WriteTimingSelectedAddr, m_WriteTimingSelectedData,
	           m_WriteTimingPassingPoints;
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
	// Raw first-transfer classifier, three targets. Sample columns are:
	// old,new,pre-lag,pre,immediate,control,final,settled. Count columns are:
	// success,write-lost,first-read-only,false-immediate-pass,
	// one-read-behind-control,other.
	u8         m_FirstDiscSample[ 3 ][ 8 ];
	u16        m_FirstDiscCount[ 3 ][ 6 ];
	CLogger   *m_Logger;
};

#endif
