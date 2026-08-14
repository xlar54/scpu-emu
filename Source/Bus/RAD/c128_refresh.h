/*
   SCPU-EMU - physical C128 DRAM refresh service

   A physical C128 with the SuperCPU MMU adapter installed loses effective
   VIC-IIe DRAM refresh while RAD holds /DMA continuously.  Core 3 releases
   /DMA only inside each VIC-owned half-cycle, then reasserts it with margin
   before the 8502 can own the bus.  Core 0 remains dedicated to emulation.
*/
#ifndef _scpu_c128_refresh_h
#define _scpu_c128_refresh_h

#include <circle/memory.h>
#include <circle/multicore.h>
#include <circle/synchronize.h>
#include <circle/types.h>

// Epoch handoff shared by the emulation core and refresh core. Core 0 marks a
// physical transaction busy only after it observes a traffic epoch; core 3
// closes that epoch, then waits for busy to clear before touching /DMA.
extern volatile u32 g_C128EpochEnabled;
extern volatile u32 g_C128EpochState;
extern volatile u32 g_C128TrafficBusy;

// Non-blocking half of c128TrafficBegin(). Callers which must first align to
// PHI and inspect BA can do that without owning the refresh interlock, then
// claim only if the same traffic epoch is still open. No bus driver may be
// enabled before this returns true.
static __attribute__((always_inline)) inline bool c128TrafficTryBegin()
{
	if ( !__atomic_load_n( &g_C128EpochEnabled, __ATOMIC_RELAXED ) )
		return true;
	if ( __atomic_load_n( &g_C128EpochState, __ATOMIC_ACQUIRE ) != 1 )
		return false;
	__atomic_store_n( &g_C128TrafficBusy, 1, __ATOMIC_RELEASE );
	DataMemBarrier();
	if ( __atomic_load_n( &g_C128EpochState, __ATOMIC_ACQUIRE ) == 1 )
		return true;
	__atomic_store_n( &g_C128TrafficBusy, 0, __ATOMIC_RELEASE );
	asm volatile( "sev" );
	return false;
}

static __attribute__((always_inline)) inline void c128TrafficBegin()
{
	if ( !__atomic_load_n( &g_C128EpochEnabled, __ATOMIC_RELAXED ) )
		return;
	for ( ;; )
	{
		while ( __atomic_load_n( &g_C128EpochState, __ATOMIC_ACQUIRE ) != 1 )
			asm volatile( "yield" );
		if ( c128TrafficTryBegin() ) return;
	}
}

static __attribute__((always_inline)) inline void c128TrafficEnd()
{
	if ( __atomic_load_n( &g_C128EpochEnabled, __ATOMIC_RELAXED ) )
	{
		__atomic_store_n( &g_C128TrafficBusy, 0, __ATOMIC_RELEASE );
		asm volatile( "sev" );
	}
}

class CC128RefreshService : public CMultiCoreSupport
{
public:
	explicit CC128RefreshService( CMemorySystem *memory );
	void Run( unsigned core ) override;
};

// Core 1 is otherwise idle. A passive Pi-side task can claim it once. The task
// must never touch the Commodore bus; core 0 owns emulation and core 3 owns the
// C128 refresh service.
typedef void (*RADCoreTask)( void *context );
void radSetCore1Task( RADCoreTask task, void *context );

// Called only while RAD owns the host under /DMA. Start returns after core 3
// has acknowledged the request. Stop returns only after it has reasserted
// /DMA and stopped touching the pin.
bool c128RefreshStart();
void c128RefreshStop();
void c128RefreshForceContinuous( bool enable );
// Diagnostic override for the interval in which /DMA is released after the
// PHI falling edge. The production default is 480 ARM cycles. Call only while
// the service is stopped; values are clamped inside the VIC-owned half-cycle.
void c128RefreshSetDMAReleaseCycles( u32 cycles );
u32  c128RefreshDMAReleaseCycles();
// Convert the coarse startup epochs to a raster-line scheduler. Begin opens an
// exclusive traffic interval so core 0 can poll $D012 without refresh gaps;
// commit hands the observed transition to core 3. A failed sync can be
// cancelled back to the coarse scheduler. Once active, observations provide a
// non-timing-critical health check. Production advances the traffic phase by
// a complete coprime permutation across physical lines. A late close advances
// both the raster origin and permutation across every skipped recovery line.
bool c128RefreshBeginLineSync( u32 cyclesPerLine, u32 rasterLines );
bool c128RefreshCommitLineSync( u16 rasterLine );
void c128RefreshCancelLineSync();
void c128RefreshObserveRaster( u16 rasterLine );
// Diagnostic-only protected-window exposure. Configure this before beginning
// the ordinary line-sync path; core 3 then uses the same commit, close,
// first-edge acquisition and PHI measurement as production, but releases
// /DMA only for W consecutive falling edges beginning at K. No traffic is
// admitted during the exposure. Completion leaves continuous safe refresh
// running until line sync is cancelled.
bool c128RefreshConfigureWindowScan( u32 k, u32 w, u32 exposureLines );
bool c128RefreshWindowScanComplete();
u32  c128RefreshWindowScanAnchorArmCycles();
u32  c128RefreshWindowScanAnchorPhiCycles();
u64  c128RefreshWindowScanExposurePulses();
u32  c128RefreshWindowScanExposureLines();
bool c128RefreshActive();
bool c128RefreshLineActive();
bool c128RefreshLineFallback();
u32  c128RefreshLineFallbackReason();
u64  c128RefreshSlots();
u64  c128RefreshSkippedSlots();
u64  c128RefreshWindowRWLow();
u64  c128RefreshCore0Slots();
u64  c128TrafficEpochs();
u64  c128RefreshEpochs();
u64  c128MaxTrafficBusyCycles();
u64  c128RefreshLineResyncs();
u64  c128RefreshLinePhaseErrors();
u64  c128RefreshLineDeadlineMisses();
u64  c128RefreshLineDeadlineRecoveries();
u64  c128RefreshLineRecoveryLines();
u64  c128RefreshLineMaxLateCycles();
u64  c128RefreshLineOverrunCount();
u64  c128RefreshLineTotalOverrunCycles();
u32  c128RefreshMeasuredPhiCycles();
u32  c128RefreshLineCyclesUsed();
u32  c128RefreshPermutationStep();
u32  c128RefreshPhasesVisited();
u32  c128RefreshPhaseMinOpens();
u32  c128RefreshPhaseMaxOpens();
u32  c128RefreshWorstOverrunPhase();
u32  c128RefreshWorstPhaseOverrunCycles();
u32  c128RefreshPhaseOpenCount( u32 phase );
u32  c128RefreshPhaseOverrunCount( u32 phase );
u32  c128RefreshPhaseMaxOverrunCycles( u32 phase );
u32  c128RefreshPredictedRaster();

#endif
