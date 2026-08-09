/*
   SCPU-EMU - physical C128 DRAM refresh service
*/
#include "c128_refresh.h"

#include <circle/bcm2835.h>
#include <circle/memio.h>
#include <circle/startup.h>
#include <circle/synchronize.h>

#include "gpio_defs.h"
#include "lowlevel_arm64.h"

static volatile u32 s_RefreshReady = 0;
static volatile u32 s_RefreshRequested = 0;
static volatile u32 s_RefreshActive = 0;
static volatile u32 s_ForceContinuousRefresh = 0;
// E18N measured the cleanest continuous-refresh result at 480 ARM cycles:
// one stable read-0 mismatch versus four at 400, while retaining roughly
// 145 ns before the measured C128 CPU half. This remains well below the
// diagnostic hard limit of 560.
static volatile u32 s_DMAReleaseCycles = 480;
static volatile u64 s_RefreshSlots = 0;
static volatile u64 s_RefreshSkippedSlots = 0;
static volatile u64 s_RefreshWindowRWLow = 0;
static volatile u64 s_Core0RefreshSlots = 0;
static volatile u64 s_TrafficEpochs = 0;
static volatile u64 s_RefreshEpochs = 0;
static volatile u64 s_MaxTrafficBusyCycles = 0;
static volatile u32 s_LineSyncRequested = 0;
static volatile u32 s_LineSyncReady = 0;
static volatile u32 s_LineSyncCommitted = 0;
static volatile u32 s_LineCycles = 65;
static volatile u32 s_LineRasterLines = 263;
static volatile u32 s_LineSyncRaster = 0;
static volatile u32 s_LineActive = 0;
static volatile u32 s_LineFallback = 0;
static volatile u32 s_LineFallbackReason = 0; // 1=PHI measure, 2=deadline, 3=forced
static volatile u32 s_LinePredictedRaster = 0;
static volatile u32 s_LineObservedRaster = 0;
static volatile u32 s_LineObservedPredicted = 0;
static volatile u32 s_LineObservedSerial = 0;
static volatile u32 s_LineMeasuredPhiCycles = 0;
static volatile u64 s_LineResyncs = 0;
static volatile u64 s_LinePhaseErrors = 0;
static volatile u64 s_LineDeadlineMisses = 0;
static volatile u64 s_LineDeadlineRecoveries = 0;
static volatile u64 s_LineRecoveryLines = 0;
static volatile u64 s_LineMaxLateCycles = 0;
static volatile u64 s_LineOverrunCount = 0;
static volatile u64 s_LineTotalOverrunCycles = 0;
static volatile u32 s_LinePermutationStep = 17;
static volatile u32 s_LinePhaseOpenCount[ 80 ] = {};
static volatile u32 s_LinePhaseOverrunCount[ 80 ] = {};
static volatile u32 s_LinePhaseMaxOverrunCycles[ 80 ] = {};
static volatile u32 s_WindowScanEnabled = 0;
static volatile u32 s_WindowScanK = 0;
static volatile u32 s_WindowScanW = 0;
static volatile u32 s_WindowScanExposureLines = 0;
static volatile u32 s_WindowScanComplete = 0;
static volatile u32 s_WindowScanAnchorArmCycles = 0;
static volatile u32 s_WindowScanAnchorPhiCycles = 0;
static volatile u64 s_WindowScanExposurePulses = 0;
static volatile u32 s_WindowScanExposureLinesCompleted = 0;

volatile u32 g_C128EpochEnabled = 0;
volatile u32 g_C128EpochState = 0;
volatile u32 g_C128TrafficBusy = 0;

static const u32 C128_REFRESH_CORE = 3;
// Kernel194 proved that even a four-half-cycle traffic island can lose sparse
// rows on the adapter-equipped C128. Per-line admission therefore cannot make
// every row safe: the VIC refresh counter advances past refreshes suppressed
// by /DMA and does not replay them later.
//
// Kernel195 changes topology. Keep refresh continuously available for about
// four milliseconds, long enough for the VIC's free-running counter to visit
// every DRAM row, then admit a short traffic epoch. Kernel196's 3.5-ms/750-us
// reallocation restored self-test and screen failures on the adapter-equipped
// C128. Keep kernel195's known-clean conservative schedule as the production
// baseline until physical transfers can preserve the intervening VIC halves.
static const u64 C128_TRAFFIC_BASE_CYCLES = 350000; // 250 us at 1.4 GHz
static const u64 C128_TRAFFIC_STEP_CYCLES = 0;
static const u64 C128_REFRESH_EPOCH_CYCLES = 5600000; // 4 ms at 1.4 GHz
static const u32 C128_REFRESH_START_LIMIT = 100000000;
// E14 begins conservatively: after an observed raster transition it releases
// /DMA for 24 consecutive VIC halves, then alternates 32 traffic cycles with
// the remainder of each line as one contiguous guarded refresh block. For an
// NTSC-R8 line that is 32 traffic / 33 refresh cycles. The documented 11..15
// refresh burst is far inside the block even with several cycles of phase
// handoff uncertainty. This first hardware round measures the safe phase;
// later revisions can narrow the guard toward 9..11 cycles.
static const u32 C128_LINE_INITIAL_REFRESH_PULSES = 24;
// Kernel197 removes the fixed-phase alias with a complete phase permutation.
// For the stock-C128 performance ladder, kernel198 doubles the window from
// four to eight cycles while retaining that permutation and all bus timings.
static const u32 C128_LINE_TRAFFIC_CYCLES = 8;

static u32 c128GCD( u32 a, u32 b )
{
	while ( b )
	{
		const u32 next = a % b;
		a = b;
		b = next;
	}
	return a;
}

static u32 c128PermutationStep( u32 lineCycles )
{
	if ( lineCycles < 2 ) return 1;
	// Seventeen is coprime to both real formats (65 NTSC, 63 PAL). Search
	// forward for unusual measured line lengths rather than silently creating
	// a shorter phase cycle.
	for ( u32 step = 17; step < lineCycles; step++ )
		if ( c128GCD( step, lineCycles ) == 1 ) return step;
	for ( u32 step = 1; step < 17 && step < lineCycles; step++ )
		if ( c128GCD( step, lineCycles ) == 1 ) return step;
	return 1;
}

static bool c128WaitFallingEdge()
{
	u32 gpio;
	do { gpio = read32( ARM_GPIO_GPLEV0 ); }
	while ( s_RefreshRequested && !( gpio & bPHI ) );
	if ( !s_RefreshRequested ) return false;
	do { gpio = read32( ARM_GPIO_GPLEV0 ); }
	while ( s_RefreshRequested && ( gpio & bPHI ) );
	return s_RefreshRequested != 0;
}

static void c128PulseDMA( u64 &now )
{
	u64 pulseStart;
	READ_CYCLE_COUNTER( pulseStart );
	write32( ARM_GPIO_GPSET0, bDMA_OUT );
	do { READ_CYCLE_COUNTER( now ); }
	while ( now - pulseStart < s_DMAReleaseCycles );
	write32( ARM_GPIO_GPCLR0, bDMA_OUT );
	s_RefreshSlots++;
}

static void c128CloseTraffic( u64 &now )
{
	__atomic_store_n( &g_C128EpochState, 2, __ATOMIC_RELEASE );
	DataMemBarrier();
	u64 waitStart;
	READ_CYCLE_COUNTER( waitStart );
	while ( s_RefreshRequested
	        && __atomic_load_n( &g_C128TrafficBusy,
	                            __ATOMIC_ACQUIRE ) != 0 )
		asm volatile( "yield" );
	READ_CYCLE_COUNTER( now );
	if ( now - waitStart > s_MaxTrafficBusyCycles )
		s_MaxTrafficBusyCycles = now - waitStart;
}

static u32 c128RasterDistance( u32 a, u32 b, u32 lines )
{
	if ( lines == 0 ) return 0;
	u32 d = a > b ? a - b : b - a;
	u32 wrap = lines - d;
	return d < wrap ? d : wrap;
}

static void c128RunLineScheduler()
{
	u64 now = 0;

	// Hold an exclusive traffic interval open until core 0 observes a real
	// $D012 transition. No refresh is needed during this bounded (< one line)
	// phase-acquisition interval.
	__atomic_store_n( &g_C128EpochState, 1, __ATOMIC_RELEASE );
	s_LineSyncReady = 1;
	DataSyncBarrier();
	asm volatile( "sev" );
	while ( s_RefreshRequested && s_LineSyncRequested
	        && !s_LineSyncCommitted )
		asm volatile( "yield" );
	u64 commitSeen = 0;
	READ_CYCLE_COUNTER( commitSeen );

	if ( !s_RefreshRequested || !s_LineSyncRequested )
	{
		s_LineSyncReady = 0;
		return;
	}

	c128CloseTraffic( now );
	if ( !s_RefreshRequested ) return;
	__atomic_store_n( &g_C128EpochState, 3, __ATOMIC_RELEASE );
	s_LinePredictedRaster = s_LineSyncRaster;
	s_LineFallback = 0;
	s_LineFallbackReason = 0;
	s_LineResyncs++;

	// Anchor on the next verified falling edge and measure the actual PHI2
	// period while applying a deliberately wide initial refresh guard.
	u64 firstEdge = 0, lastEdge = 0;
	for ( u32 i = 0; i < C128_LINE_INITIAL_REFRESH_PULSES; i++ )
	{
		if ( !c128WaitFallingEdge() ) return;
		READ_CYCLE_COUNTER( now );
		if ( i == 0 ) firstEdge = now;
		lastEdge = now;
		c128PulseDMA( now );
	}
	u32 phiCycles = C128_LINE_INITIAL_REFRESH_PULSES > 1
	              ? (u32)( ( lastEdge - firstEdge )
	                       / ( C128_LINE_INITIAL_REFRESH_PULSES - 1 ) )
	              : 0;
	if ( phiCycles < 1000 || phiCycles > 1800 )
	{
		// Bad phase measurement: preserve RAM and refuse traffic.
		s_LineFallback = 1;
		s_LineFallbackReason = 1;
		phiCycles = 1400;
	}
	s_LineMeasuredPhiCycles = phiCycles;
	const u64 anchorArmCycles = firstEdge >= commitSeen
	                          ? firstEdge - commitSeen : 0;
	s_WindowScanAnchorArmCycles =
		anchorArmCycles > 0xFFFFFFFFu ? 0xFFFFFFFFu : (u32)anchorArmCycles;
	s_WindowScanAnchorPhiCycles = phiCycles
		? (u32)( ( anchorArmCycles + phiCycles / 2 ) / phiCycles ) : 0;
	s_LineActive = 1;
	DataSyncBarrier();
	asm volatile( "sev" );

	u32 observedSerial = s_LineObservedSerial;
	const u32 lineCycles = s_LineCycles;
	if ( s_WindowScanEnabled )
	{
		// The first measured edge is coordinate zero. Measurement has already
		// pulsed coordinates 0..23; finish that partial line with continuous
		// refresh so it cannot affect the timed exposure, then begin at the next
		// coordinate-zero edge. K is the first protected/released edge and W is
		// the number of consecutive protected edges, both modulo lineCycles.
		__atomic_store_n( &g_C128EpochState, 3, __ATOMIC_RELEASE );
		for ( u32 phase = C128_LINE_INITIAL_REFRESH_PULSES;
		      phase < lineCycles && s_RefreshRequested && s_LineSyncRequested;
		      phase++ )
		{
			if ( !c128WaitFallingEdge() ) return;
			c128PulseDMA( now );
		}

		const u32 k = lineCycles ? s_WindowScanK % lineCycles : 0;
		const u32 w = s_WindowScanW < lineCycles
		              ? s_WindowScanW : lineCycles;
		const u32 exposureLines = s_WindowScanExposureLines;
		for ( u32 line = 0;
		      line < exposureLines && s_RefreshRequested && s_LineSyncRequested;
		      line++ )
		{
			for ( u32 phase = 0;
			      phase < lineCycles && s_RefreshRequested && s_LineSyncRequested;
			      phase++ )
			{
				const u32 fromK = ( phase + lineCycles - k ) % lineCycles;
				// Decide before the edge. Computing a runtime modulo after the
				// falling edge delayed /DMA release relative to the proven tight
				// continuous loop and made even W=linecycles lose cells. Entering
				// the protected branch first makes wait->pulse identical to that
				// continuous primitive.
				if ( fromK < w )
				{
					if ( !c128WaitFallingEdge() ) return;
					c128PulseDMA( now );
					s_WindowScanExposurePulses++;
				}
				else if ( !c128WaitFallingEdge() ) return;
			}
			s_WindowScanExposureLinesCompleted++;
		}

		__atomic_store_n( &s_WindowScanComplete, 1u, __ATOMIC_RELEASE );
		asm volatile( "sev" );
		// Preserve the measured RAM indefinitely while core 0 notices
		// completion. It performs no bus access until it cancels line sync.
		while ( s_RefreshRequested && s_LineSyncRequested
		        && s_WindowScanEnabled )
		{
			if ( !c128WaitFallingEdge() ) break;
			c128PulseDMA( now );
		}
		s_LineActive = 0;
		return;
	}
	// The first measured edge is phase zero. Open initially at the last guarded
	// phase, then advance the start by a coprime step in each physical raster
	// line. This visits every phase exactly once per lineCycles lines instead of
	// permanently starving the rows beneath one fixed per-line traffic hole.
	const u64 linePeriod = (u64)phiCycles * lineCycles;
	u64 lineOrigin = firstEdge;
	u64 openEdge = lastEdge;
	u32 permutationPhase = C128_LINE_INITIAL_REFRESH_PULSES - 1;
	while ( permutationPhase >= lineCycles ) permutationPhase -= lineCycles;
	const u32 permutationStep = c128PermutationStep( lineCycles );
	s_LinePermutationStep = permutationStep;

	while ( s_RefreshRequested && s_LineSyncRequested && !s_LineFallback )
	{
		// Traffic is deliberately timer-slept on core 3: polling GPLEV0 here
		// would contend for the GPIO peripheral with core 0's timed accesses.
		const u64 closeTarget = openEdge
		                        + (u64)phiCycles * C128_LINE_TRAFFIC_CYCLES;
		__atomic_store_n( &g_C128EpochState, 1, __ATOMIC_RELEASE );
		s_TrafficEpochs++;
		s_LinePhaseOpenCount[ permutationPhase ]++;
		asm volatile( "sev" );
		do
		{
			READ_CYCLE_COUNTER( now );
			asm volatile( "yield" );
		} while ( s_RefreshRequested && !s_ForceContinuousRefresh
		       && now < closeTarget );

		c128CloseTraffic( now );
		if ( !s_RefreshRequested ) break;
		const u64 overrun = now > closeTarget ? now - closeTarget : 0;
		if ( overrun )
		{
			s_LineOverrunCount++;
			s_LineTotalOverrunCycles += overrun;
			s_LinePhaseOverrunCount[ permutationPhase ]++;
			const u32 clipped = overrun > 0xFFFFFFFFu
			                  ? 0xFFFFFFFFu : (u32)overrun;
			if ( clipped > s_LinePhaseMaxOverrunCycles[ permutationPhase ] )
				s_LinePhaseMaxOverrunCycles[ permutationPhase ] = clipped;
			if ( overrun > s_LineMaxLateCycles )
				s_LineMaxLateCycles = overrun;
		}
		bool deadlineLate = overrun > (u64)phiCycles * 12u;
		if ( deadlineLate )
		{
			s_LineDeadlineMisses++;
			s_LineDeadlineRecoveries++;
		}
		if ( s_ForceContinuousRefresh )
		{
			s_LineFallback = 1;
			s_LineFallbackReason = 3;
			break;
		}

		__atomic_store_n( &g_C128EpochState, 3, __ATOMIC_RELEASE );
		s_RefreshEpochs++;

		// Advance the origin and permutation once for every physical line,
		// including recovery lines. Advancing only the deadline while retaining
		// the old phase would silently break the complete permutation.
		u32 linesAdvanced = 1;
		u64 nextLineOrigin = lineOrigin + linePeriod;
		u32 nextPhase = permutationPhase + permutationStep;
		if ( nextPhase >= lineCycles ) nextPhase -= lineCycles;
		u64 nextOpenTarget = nextLineOrigin + (u64)nextPhase * phiCycles;
		if ( deadlineLate )
		{
			// A transiently long physical transfer must not permanently strand
			// core 0 in c128TrafficBegin(). Keep traffic closed for at least one
			// additional complete line, advancing the permutation with it.
			nextLineOrigin += linePeriod;
			nextPhase += permutationStep;
			if ( nextPhase >= lineCycles ) nextPhase -= lineCycles;
			nextOpenTarget = nextLineOrigin + (u64)nextPhase * phiCycles;
			linesAdvanced++;
		}
		while ( nextOpenTarget <= now + phiCycles / 2 )
		{
			if ( !deadlineLate )
			{
				deadlineLate = true;
				s_LineDeadlineMisses++;
				s_LineDeadlineRecoveries++;
			}
			nextLineOrigin += linePeriod;
			nextPhase += permutationStep;
			if ( nextPhase >= lineCycles ) nextPhase -= lineCycles;
			nextOpenTarget = nextLineOrigin + (u64)nextPhase * phiCycles;
			linesAdvanced++;
		}
		if ( linesAdvanced > 1 ) s_LineRecoveryLines += linesAdvanced - 1;
		for ( ;; )
		{
			if ( !c128WaitFallingEdge() ) return;
			u64 edge;
			READ_CYCLE_COUNTER( edge );
			// The first edge at the next open phase belongs to traffic. The
			// half-period tolerance absorbs integer division in phiCycles.
			if ( edge + phiCycles / 2 >= nextOpenTarget )
			{
				openEdge = edge;
				permutationPhase = nextPhase;
				// Reconstruct this physical line's origin from the actual edge.
				// The integer PHI average can be fractionally low or high; carrying
				// a purely predicted origin would accumulate that error forever.
				const u64 phaseOffset = (u64)permutationPhase * phiCycles;
				lineOrigin = edge >= phaseOffset
				           ? edge - phaseOffset : nextLineOrigin;
				break;
			}
			c128PulseDMA( now );
		}

		u32 predicted = s_LinePredictedRaster + linesAdvanced;
		if ( s_LineRasterLines ) predicted %= s_LineRasterLines;
		s_LinePredictedRaster = predicted;

		const u32 serial = __atomic_load_n( &s_LineObservedSerial,
		                                    __ATOMIC_ACQUIRE );
		if ( serial != observedSerial )
		{
			observedSerial = serial;
			const u32 observed = __atomic_load_n( &s_LineObservedRaster,
			                                      __ATOMIC_RELAXED );
			const u32 observedPredicted =
				__atomic_load_n( &s_LineObservedPredicted,
				                 __ATOMIC_RELAXED );
			if ( observed >= s_LineRasterLines
			     || observedPredicted >= s_LineRasterLines
			     || c128RasterDistance( observed, observedPredicted,
			                             s_LineRasterLines ) > 2 )
			{
				s_LinePhaseErrors++;
			}
		}
	}

	// Fail safe: never reopen traffic with an untrusted phase. The E11 mode is
	// known to preserve DRAM indefinitely; the visible freeze is intentional
	// and diagnostics expose why it happened.
	__atomic_store_n( &g_C128EpochState, 3, __ATOMIC_RELEASE );
	while ( s_RefreshRequested && s_LineSyncRequested )
	{
		if ( !c128WaitFallingEdge() ) break;
		c128PulseDMA( now );
	}
	s_LineActive = 0;
}

CC128RefreshService::CC128RefreshService( CMemorySystem *memory )
	: CMultiCoreSupport( memory )
{
}

void CC128RefreshService::Run( unsigned core )
{
	// Cores 1 and 2 are intentionally unused. Keep them asleep rather than
	// returning through Circle's secondary-core logger during timed operation.
	if ( core != C128_REFRESH_CORE )
	{
		DisableIRQs();
		for ( ;; ) asm volatile( "wfe" );
	}

	DisableIRQs();
	initCycleCounter();
	s_RefreshReady = 1;
	DataSyncBarrier();
	asm volatile( "sev" );

	for ( ;; )
	{
		while ( !s_RefreshRequested ) asm volatile( "wfe" );
		s_RefreshActive = 1;
		DataSyncBarrier();

		while ( s_RefreshRequested )
		{
			if ( s_LineSyncRequested )
			{
				c128RunLineScheduler();
				continue;
			}
			// Close the traffic epoch before examining busy. A core-0 transfer
			// already committed may finish; a racing transfer rechecks state and
			// backs out before enabling any RAD driver.
			__atomic_store_n( &g_C128EpochState, 2, __ATOMIC_RELEASE );
			// Pair with core0's busy-store barrier. Without this DMB, AArch64 may
			// let both cores observe the other's pre-store value and proceed.
			DataMemBarrier();
			u64 waitStart, now;
			READ_CYCLE_COUNTER( waitStart );
			while ( s_RefreshRequested
			        && __atomic_load_n( &g_C128TrafficBusy,
			                            __ATOMIC_ACQUIRE ) != 0 )
				asm volatile( "yield" );
			READ_CYCLE_COUNTER( now );
			if ( now - waitStart > s_MaxTrafficBusyCycles )
				s_MaxTrafficBusyCycles = now - waitStart;
			if ( !s_RefreshRequested ) break;

			// With core 0 parked and every driver off, this is exactly the
			// kernel144 refresh primitive: one verified falling-edge pulse per
			// VIC half and no GPIO sample inside the release window.
			__atomic_store_n( &g_C128EpochState, 3, __ATOMIC_RELEASE );
			s_RefreshEpochs++;
			u64 refreshStart;
			READ_CYCLE_COUNTER( refreshStart );
			while ( s_RefreshRequested && !s_LineSyncRequested )
			{
				if ( !c128WaitFallingEdge() ) break;
				c128PulseDMA( now );
				READ_CYCLE_COUNTER( now );
				if ( !__atomic_load_n( &s_ForceContinuousRefresh,
				                       __ATOMIC_RELAXED )
				     && now - refreshStart >= C128_REFRESH_EPOCH_CYCLES ) break;
			}
			write32( ARM_GPIO_GPCLR0, bDMA_OUT );
			if ( !s_RefreshRequested ) break;
			if ( s_LineSyncRequested ) continue;

			__atomic_store_n( &g_C128EpochState, 1, __ATOMIC_RELEASE );
			s_TrafficEpochs++;
			asm volatile( "sev" );
			u64 trafficStart;
			// The production full-sweep schedule uses a fixed traffic duration.
			// Keep the step term here so diagnostic builds can still dither it.
			const u64 trafficLimit = C128_TRAFFIC_BASE_CYCLES
			                           + ( ( s_TrafficEpochs * 7u ) & 15u )
			                             * C128_TRAFFIC_STEP_CYCLES;
			READ_CYCLE_COUNTER( trafficStart );
			do
			{
				READ_CYCLE_COUNTER( now );
				asm volatile( "yield" );
			} while ( s_RefreshRequested
			       && now - trafficStart < trafficLimit );
		}

		// Acknowledge only after /DMA is definitely asserted again.
		write32( ARM_GPIO_GPCLR0, bDMA_OUT );
		__atomic_store_n( &g_C128EpochState, 0, __ATOMIC_RELEASE );
		DataSyncBarrier();
		s_RefreshActive = 0;
		DataSyncBarrier();
		asm volatile( "sev" );
	}
}

bool c128RefreshStart()
{
	for ( u32 spins = 0; !s_RefreshReady && spins < C128_REFRESH_START_LIMIT;
	      spins++ )
		asm volatile( "yield" );
	if ( !s_RefreshReady ) return false;

	__atomic_store_n( &g_C128TrafficBusy, 0, __ATOMIC_RELAXED );
	s_LineSyncRequested = 0;
	s_LineSyncReady = 0;
	s_LineSyncCommitted = 0;
	s_WindowScanEnabled = 0;
	s_WindowScanComplete = 0;
	s_LineActive = 0;
	s_LineFallback = 0;
	s_LineFallbackReason = 0;
	__atomic_store_n( &g_C128EpochState, 3, __ATOMIC_RELAXED );
	__atomic_store_n( &g_C128EpochEnabled, 1, __ATOMIC_RELEASE );
	s_RefreshRequested = 1;
	DataSyncBarrier();
	asm volatile( "sev" );
	for ( u32 spins = 0; !s_RefreshActive && spins < C128_REFRESH_START_LIMIT;
	      spins++ )
		asm volatile( "yield" );
	return s_RefreshActive != 0;
}

void c128RefreshStop()
{
	if ( !s_RefreshRequested && !s_RefreshActive ) return;
	s_LineSyncRequested = 0;
	s_LineSyncCommitted = 0;
	s_RefreshRequested = 0;
	DataSyncBarrier();
	asm volatile( "sev" );
	while ( s_RefreshActive ) asm volatile( "yield" );
	__atomic_store_n( &g_C128EpochEnabled, 0, __ATOMIC_RELEASE );
	__atomic_store_n( &g_C128EpochState, 0, __ATOMIC_RELEASE );
	__atomic_store_n( &g_C128TrafficBusy, 0, __ATOMIC_RELEASE );
	s_LineSyncReady = 0;
	s_LineActive = 0;
	DataSyncBarrier();
}

void c128RefreshForceContinuous( bool enable )
{
	__atomic_store_n( &s_ForceContinuousRefresh, enable ? 1u : 0u,
	                  __ATOMIC_RELEASE );
	asm volatile( "sev" );
}

void c128RefreshSetDMAReleaseCycles( u32 cycles )
{
	// At the measured ~1368 ARM cycles per complete PHI period, 560 leaves
	// roughly 124 cycles before the CPU half. Keep a hard diagnostic guard on
	// both ends even if a caller supplies a bad value.
	if ( cycles < 200 ) cycles = 200;
	if ( cycles > 560 ) cycles = 560;
	__atomic_store_n( &s_DMAReleaseCycles, cycles, __ATOMIC_RELEASE );
}

u32 c128RefreshDMAReleaseCycles()
{
	return __atomic_load_n( &s_DMAReleaseCycles, __ATOMIC_ACQUIRE );
}

bool c128RefreshBeginLineSync( u32 cyclesPerLine, u32 rasterLines )
{
	if ( !s_RefreshActive || cyclesPerLine < 48 || cyclesPerLine > 80
	     || rasterLines < 240 || rasterLines > 320 )
		return false;

	s_LineCycles = cyclesPerLine;
	s_LineRasterLines = rasterLines;
	s_LineSyncReady = 0;
	s_LineSyncCommitted = 0;
	s_LineActive = 0;
	s_LineFallback = 0;
	s_LineFallbackReason = 0;
	s_LineOverrunCount = 0;
	s_LineTotalOverrunCycles = 0;
	s_LineMaxLateCycles = 0;
	s_LinePermutationStep = c128PermutationStep( cyclesPerLine );
	for ( u32 phase = 0; phase < 80; phase++ )
	{
		s_LinePhaseOpenCount[ phase ] = 0;
		s_LinePhaseOverrunCount[ phase ] = 0;
		s_LinePhaseMaxOverrunCycles[ phase ] = 0;
	}
	s_LineSyncRequested = 1;
	DataSyncBarrier();
	asm volatile( "sev" );

	for ( u32 spins = 0; !s_LineSyncReady && spins < C128_REFRESH_START_LIMIT;
	      spins++ )
		asm volatile( "yield" );
	return s_LineSyncReady != 0;
}

bool c128RefreshCommitLineSync( u16 rasterLine )
{
	if ( !s_LineSyncReady || rasterLine >= s_LineRasterLines ) return false;
	s_LineSyncRaster = rasterLine;
	s_LineObservedRaster = rasterLine;
	s_LineObservedPredicted = rasterLine;
	__atomic_add_fetch( &s_LineObservedSerial, 1u, __ATOMIC_RELEASE );
	s_LineSyncCommitted = 1;
	DataSyncBarrier();
	asm volatile( "sev" );
	for ( u32 spins = 0; !s_LineActive && !s_LineFallback
	      && spins < C128_REFRESH_START_LIMIT; spins++ )
		asm volatile( "yield" );
	return s_LineActive != 0 && !s_LineFallback;
}

void c128RefreshCancelLineSync()
{
	s_LineSyncRequested = 0;
	s_LineSyncCommitted = 0;
	DataSyncBarrier();
	asm volatile( "sev" );
	for ( u32 spins = 0; s_LineActive && spins < C128_REFRESH_START_LIMIT;
	      spins++ )
		asm volatile( "yield" );
	s_WindowScanEnabled = 0;
	s_WindowScanComplete = 0;
}

void c128RefreshObserveRaster( u16 rasterLine )
{
	if ( !s_LineActive || rasterLine >= s_LineRasterLines ) return;
	// Capture the scheduler's line at the same instant as the physical sample.
	// Comparing against the later value on core 3 made a perfectly good sample
	// appear stale whenever a refresh boundary occurred before it was consumed.
	const u32 predicted = __atomic_load_n( &s_LinePredictedRaster,
	                                       __ATOMIC_ACQUIRE );
	__atomic_store_n( &s_LineObservedRaster, (u32)rasterLine,
	                  __ATOMIC_RELAXED );
	__atomic_store_n( &s_LineObservedPredicted, predicted,
	                  __ATOMIC_RELAXED );
	__atomic_add_fetch( &s_LineObservedSerial, 1u, __ATOMIC_RELEASE );
}

bool c128RefreshConfigureWindowScan( u32 k, u32 w, u32 exposureLines )
{
	if ( !s_RefreshActive || s_LineActive || s_LineSyncRequested
	     || s_LineCycles == 0 || w == 0 || w > s_LineCycles
	     )
		return false;
	s_WindowScanK = k % s_LineCycles;
	s_WindowScanW = w;
	s_WindowScanExposureLines = exposureLines;
	s_WindowScanAnchorArmCycles = 0;
	s_WindowScanAnchorPhiCycles = 0;
	s_WindowScanExposurePulses = 0;
	s_WindowScanExposureLinesCompleted = 0;
	__atomic_store_n( &s_WindowScanComplete, 0u, __ATOMIC_RELAXED );
	__atomic_store_n( &s_WindowScanEnabled, 1u, __ATOMIC_RELEASE );
	return true;
}

bool c128RefreshWindowScanComplete()
{
	return __atomic_load_n( &s_WindowScanComplete, __ATOMIC_ACQUIRE ) != 0;
}

u32 c128RefreshWindowScanAnchorArmCycles()
{
	return s_WindowScanAnchorArmCycles;
}

u32 c128RefreshWindowScanAnchorPhiCycles()
{
	return s_WindowScanAnchorPhiCycles;
}

u64 c128RefreshWindowScanExposurePulses()
{
	return s_WindowScanExposurePulses;
}

u32 c128RefreshWindowScanExposureLines()
{
	return s_WindowScanExposureLinesCompleted;
}

bool c128RefreshActive() { return s_RefreshActive != 0; }
bool c128RefreshLineActive() { return s_LineActive != 0; }
bool c128RefreshLineFallback() { return s_LineFallback != 0; }
u32 c128RefreshLineFallbackReason() { return s_LineFallbackReason; }
u64 c128RefreshSlots() { return s_RefreshSlots; }
u64 c128RefreshSkippedSlots() { return s_RefreshSkippedSlots; }
u64 c128RefreshWindowRWLow() { return s_RefreshWindowRWLow; }
u64 c128RefreshCore0Slots() { return s_Core0RefreshSlots; }
u64 c128TrafficEpochs() { return s_TrafficEpochs; }
u64 c128RefreshEpochs() { return s_RefreshEpochs; }
u64 c128MaxTrafficBusyCycles() { return s_MaxTrafficBusyCycles; }
u64 c128RefreshLineResyncs() { return s_LineResyncs; }
u64 c128RefreshLinePhaseErrors() { return s_LinePhaseErrors; }
u64 c128RefreshLineDeadlineMisses() { return s_LineDeadlineMisses; }
u64 c128RefreshLineDeadlineRecoveries() { return s_LineDeadlineRecoveries; }
u64 c128RefreshLineRecoveryLines() { return s_LineRecoveryLines; }
u64 c128RefreshLineMaxLateCycles() { return s_LineMaxLateCycles; }
u64 c128RefreshLineOverrunCount() { return s_LineOverrunCount; }
u64 c128RefreshLineTotalOverrunCycles() { return s_LineTotalOverrunCycles; }
u32 c128RefreshMeasuredPhiCycles() { return s_LineMeasuredPhiCycles; }
u32 c128RefreshLineCyclesUsed() { return s_LineCycles; }
u32 c128RefreshPermutationStep() { return s_LinePermutationStep; }
u32 c128RefreshPhasesVisited()
{
	u32 visited = 0;
	for ( u32 phase = 0; phase < s_LineCycles && phase < 80; phase++ )
		if ( s_LinePhaseOpenCount[ phase ] ) visited++;
	return visited;
}
u32 c128RefreshPhaseMinOpens()
{
	u32 minimum = 0xFFFFFFFFu;
	for ( u32 phase = 0; phase < s_LineCycles && phase < 80; phase++ )
		if ( s_LinePhaseOpenCount[ phase ] < minimum )
			minimum = s_LinePhaseOpenCount[ phase ];
	return minimum == 0xFFFFFFFFu ? 0 : minimum;
}
u32 c128RefreshPhaseMaxOpens()
{
	u32 maximum = 0;
	for ( u32 phase = 0; phase < s_LineCycles && phase < 80; phase++ )
		if ( s_LinePhaseOpenCount[ phase ] > maximum )
			maximum = s_LinePhaseOpenCount[ phase ];
	return maximum;
}
u32 c128RefreshWorstOverrunPhase()
{
	u32 worst = 0;
	for ( u32 phase = 1; phase < s_LineCycles && phase < 80; phase++ )
		if ( s_LinePhaseMaxOverrunCycles[ phase ]
		     > s_LinePhaseMaxOverrunCycles[ worst ] ) worst = phase;
	return worst;
}
u32 c128RefreshWorstPhaseOverrunCycles()
{
	return s_LinePhaseMaxOverrunCycles[ c128RefreshWorstOverrunPhase() ];
}
u32 c128RefreshPhaseOpenCount( u32 phase )
{
	return phase < 80 ? s_LinePhaseOpenCount[ phase ] : 0;
}
u32 c128RefreshPhaseOverrunCount( u32 phase )
{
	return phase < 80 ? s_LinePhaseOverrunCount[ phase ] : 0;
}
u32 c128RefreshPhaseMaxOverrunCycles( u32 phase )
{
	return phase < 80 ? s_LinePhaseMaxOverrunCycles[ phase ] : 0;
}
u32 c128RefreshPredictedRaster() { return s_LinePredictedRaster; }
