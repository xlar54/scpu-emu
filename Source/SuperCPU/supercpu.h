/*
   SCPU-EMU - CMD SuperCPU emulation for the C64/C128 using a RAD Expansion Unit
   Copyright (c) 2026 SCPU-EMU contributors

   CSuperCPU - the accelerator itself.

   Owns and wires together the pieces that make up a SuperCPU:

       CPU core  ->  CC64Memory (bank 0, shadowed)  ->  IC64Bus
                          |                  |
                          |                  +-> CWriteBuffer  (VIC mirroring)
                          +-> CSuperCPURegisters   ($D07x, $D0Bx, $D2xx/$D3xx)

   It is deliberately platform-agnostic: hand it any IC64Bus and it runs, which
   is what lets the whole accelerator be exercised on a PC against CHostBus.

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
#ifndef _supercpu_h
#define _supercpu_h

#include "../Common/types.h"
#include "../C64/c64_bus.h"
#include "../C64/c64_memory.h"
#include "../CPU/cpu.h"
#include "../CPU/M6502/m6502.h"
#include "../CPU/W65C816/w65c816.h"
#include "write_buffer.h"
#include "registers.h"
#include "../REU/reu_wiring.h"
#include "fast_ram.h"
#include "memory_map.h"

// Which core drives the machine. The 65816 is the real thing; the 6502 stays
// available as milestone-1 scaffolding and as the oracle the 65816 is tested
// against in emulation mode (see Tests/CPU/test_w65c816_diff.cpp).
enum SCPUCoreType
{
	SCPU_CORE_6502 = 0,
	SCPU_CORE_65816
};

// Physical C128 and current operating-system mode are separate facts.  AUTO
// uses the post-handoff MMU/KERNAL signature; the explicit values are useful
// for bring-up and avoid pretending an internal 8502 port can be read by DMA.
enum SCPUC128Mode
{
	SCPU_C128_MODE_AUTO = 0,
	SCPU_C128_MODE_C64,
	SCPU_C128_MODE_NATIVE
};

// K381 diagnostic phases. A completed IEC transaction first holds only the
// unconditional background sweep, then performs a targeted active-display
// rewrite so the physical VIC picture can distinguish transient fetch noise
// from stored DRAM corruption in one run.
enum SCPUResyncDiagnosticPhase
{
	SCPU_RESYNC_DIAG_IDLE = 0,
	SCPU_RESYNC_DIAG_HOLD,
	SCPU_RESYNC_DIAG_REPAIRING,
	SCPU_RESYNC_DIAG_COMPLETE
};

// Mirrored bytes allowed per drain call while the beam is inside the picture.
// Effectively the mirror's write-through throughput; see
// CSuperCPU::setMirrorDisplayBudget for the reasoning.
#define SCPU_MIRROR_DISPLAY_BYTES_DEFAULT 1024

// Active-screen bytes per scheduler opportunity while the beam is outside the
// picture. This is deliberately separate from the general display ration: a
// 1024-byte VIC matrix ring then converges in four opportunities without
// turning every ordinary mirror drain into a 256-byte CPU pause.
#define SCPU_MIRROR_SCREEN_BYTES_PER_OPPORTUNITY 256

// Passive IEC/mirror diagnostic. The last few serial-active epochs are kept
// entirely in RAM and copied to the reset-button dump only after emulation has
// stopped. This is intentionally not a live logger: filesystem or console work
// during IEC would change the timing being measured.
#define SCPU_IEC_EPOCH_DIAG_CAPACITY 8
struct SCPUIECEpochDiagnostic
{
	u32 sequence;
	u8  valid;
	u8  quietSeen;
	u8  matrixDrained;
	u8  allDrained;
	u8  dd00Valid;
	u8  bitmapMode;
	u8  d011;
	u8  d016;
	u8  d018;
	u8  dd00;
	u32 screenBase;
	u32 graphicsBase;
	u32 pendingAtEntry;
	u32 matrixPendingAtEntry;
	u32 pendingAtQuiet;
	u32 matrixPendingAtQuiet;
	u64 matrixWritesAccepted;
	u64 matrixWritesCoalesced;
	u64 activeHostCycles;
	u64 matrixDrainHostCycles;
	u64 allDrainHostCycles;
	u32 matrixDrainOpportunities;
	u32 allDrainOpportunities;

	// Internal timestamps/counter shared with the passive scheduler observer.
	u64 activeStartedAt;
	u64 quietStartedAt;
	u32 opportunitiesSinceQuiet;
};

// Bounded, measurement-only comparisons of the active physical VIC matrix
// against the value the mirror believes it delivered. Storage is outside
// CSuperCPU so enabling the diagnostic cannot move the timing-sensitive RAD
// bus object in memory (see supercpu.cpp). The first three slots are clean
// gameplay samples; the fourth preserves the matrix exactly as found when the
// heartbeat declares a stall, including any outstanding mirror bytes.
#define SCPU_VSP_MATRIX_MISMATCH_SAMPLES 16
#define SCPU_VSP_MATRIX_CLEAN_CAPTURES 3
#define SCPU_VSP_MATRIX_CAPTURE_SLOTS 4
#define SCPU_VSP_MATRIX_STALL_SLOT 3
struct SCPUVSPMatrixMismatch
{
	u16 offset;
	u8 expected;
	u8 physical;
};

struct SCPUVSPMatrixCapture
{
	u8  state;                 // 0=waiting, 1=armed, 2=captured
	u8  kind;                  // 0=clean gameplay, 1=at-stall preservation
	u8  safeWindow;
	u8  mismatchSampleCount;
	u32 targetFrame;
	u32 qualifyingFrame;
	u32 traceTotal;
	u8  gamePhase;
	u8  reserved0;
	u16 reserved1;
	u32 captureAttempts;
	u32 unsafeWindowWaits;
	u32 pendingWaits;
	u32 logicalBase;
	u32 physicalBase;
	u16 rasterBefore;
	u16 rasterAfter;
	u32 pendingAtCapture;
	u32 directMatches;
	u32 mismatchCount;
	u16 firstMismatch;
	u16 lastMismatch;
	s32 bestShift;
	u32 bestShiftMatches;
	u32 bestShiftCompared;
	SCPUVSPMatrixMismatch mismatch[ SCPU_VSP_MATRIX_MISMATCH_SAMPLES ];
};

struct SCPUVSPMatrixDiagnostic
{
	u8  state;                 // 0=waiting, 1=gameplay qualified, 2=clean set complete
	u8  gameplaySeen;
	u8  phaseChanges;
	u8  capturedCount;
	u32 qualifyingFrames;
	u32 quietFrames;
	u32 traceTotal;
	SCPUVSPMatrixCapture capture[ SCPU_VSP_MATRIX_CAPTURE_SLOTS ];
};

class CSuperCPU
{
public:
	CSuperCPU();

	// Wire up a bus and bring the accelerator to its reset state. Returns
	// false if the bus could not be acquired or no usable ROMs were obtained.
	// simmMB: SuperRAM to fit, in megabytes. 0/1/4/8/16, as the real card
	// accepts. Reachable only by the 65816 core -- a 6502 has no way to name an
	// address above $FFFF, so with SCPU_CORE_6502 the SIMM is fitted but idle.
	// reuSize is a REUSIZE_* selector, not a size. Defaulted so every
	// existing caller compiles unchanged and gets a machine with no REU --
	// exactly the behaviour that existed before the unit did.
	bool init( IC64Bus *bus, SCPUCoreType core = SCPU_CORE_6502,
	           u32 simmMB = SCPU_SIMM_16MB,
	           u8 reuSize = REUSIZE_NONE );

	// Supply ROM images explicitly. Call before init() to override the images
	// that would otherwise be snapshotted off the live machine.
	void setBasicROM ( const u8 *data ) { m_Memory.setBasicROM( data ); }
	void setKernalROM( const u8 *data ) { m_Memory.setKernalROM( data ); }
	void setCharROM  ( const u8 *data ) { m_Memory.setCharROM( data ); }

	// Bootmap: map the SuperCPU's own ROM over bank 0 at reset, so CMD's boot
	// code runs before the C64's KERNAL -- which is where the SuperCPU banner
	// comes from, and what a real card does. Needs a ROM image of at least 64K;
	// without one this is ignored and the machine boots its own KERNAL.
	void setBootmapEnabled( bool on ) { m_BootmapEnabled = on; }
	bool bootmapEnabled() const       { return m_BootmapEnabled; }
	void setC128Mode( SCPUC128Mode mode ) { m_C128Mode = mode; }

	// How many mirrored bytes may be pushed per drain call while the beam is
	// INSIDE the visible display -- effectively the mirror's write-through
	// throughput. A real SuperCPU mirrors every write to DRAM immediately,
	// mid-display, always; games rely on their OWN vblank discipline for
	// tear-free updates, and it is delivery DELAY that breaks them: border-only
	// mirroring caps throughput near 3KB/frame, so an animating game's objects
	// arrive split across border windows and whole frames display half-updated
	// data. Bus safety is not the issue -- the burst path yields to the VIC on
	// BA, exactly as a real REU does.
	//
	// The scheduler spreads the historical eight-drain allowance across 128
	// points per frame. Thus the default 1024 becomes ~64 bytes per opportunity
	// and ~8KB/frame: similar throughput without millisecond CPU stalls. 0
	// restores border-only mirroring. Sprite pointers of the active screen
	// bypass this queue entirely -- see CC64Memory::writeFast.
	void setMirrorDisplayBudget( u32 bytes ) { m_MirrorDisplayBudget = bytes; }
	void setDisplayScrub( bool enabled, bool masked, u32 periodSeconds = 0 )
	{
		m_DisplayScrubEnabled = enabled;
		m_DisplayScrubMasked = masked;
		m_DisplayScrubPeriodSeconds = periodSeconds;
		m_DisplayScrubPhaseOn = enabled && periodSeconds == 0;
		m_DisplayScrubPhaseStart = 0;
		m_DisplayScrubPhaseGeneration = 0;
		m_DisplayScrubBitmapSeen = false;
	}
	bool displayScrubPhaseOn() const { return m_DisplayScrubPhaseOn; }
	u32 displayScrubPhaseGeneration() const { return m_DisplayScrubPhaseGeneration; }
	u32 displayScrubPeriodSeconds() const { return m_DisplayScrubPeriodSeconds; }
	SCPUResyncDiagnosticPhase resyncDiagnosticPhase() const
	{
		return m_ResyncDiagnosticPhase;
	}
	u32 resyncDiagnosticGeneration() const
	{
		return m_ResyncDiagnosticGeneration;
	}
	u32 resyncDiagnosticFramesRemaining() const
	{
		return m_ResyncDiagnosticFramesRemaining;
	}
	bool resyncDiagnosticShadowChanged() const
	{
		return m_ResyncDiagnosticShadowChanged;
	}
	u32 resyncDiagnosticRepairCallsRemaining() const
	{
		return m_ResyncDiagnosticRepairCallsRemaining;
	}

	// Diagnostic kill switch: when set, runFrame stops ALL mirror bus traffic
	// -- flushes and the resync sweep -- leaving real DRAM untouched so the
	// screen shows exactly what it holds. See MIRROR_HALT_AFTER_S.
	void setMirrorHalted( bool halted ) { m_MirrorHalted = halted; }
	bool mirrorHalted() const           { return m_MirrorHalted; }

	// VIDEO_MODE 1 owns presentation after its first completed HDMI frame.
	// Make that handoff one-way: stop accepting new VIC-visible RAM writes,
	// forget anything already queued, and prevent both ordinary drains and the
	// background resync sweep from touching physical DRAM. Immediate VIC/CIA
	// register accesses remain live because they are part of machine semantics,
	// not display mirroring. VIDEO_MODE 0 never calls this method.
	void disablePhysicalMirror();
	// Reverse the HDMI handoff. The physical VIC has continued to receive chip
	// register writes, but its DRAM may be stale, so queue the current visible
	// matrix, bitmap/charset and enabled sprite shapes for raster-aware repair.
	void enablePhysicalMirror();

	// Under-I/O sprite-shape relocation (MIRROR_D000_RELOCATE); see
	// CC64Memory's relocation section. On by default; the switch exists so a
	// game that goes wrong with it can prove it with one config line.
	void setUnderIORelocate( bool e ) { m_Memory.setUnderIORelocate( e ); }
	void setRAMUnderIOAccessible( bool accessible )
	{
		m_WriteBuffer.setRAMUnderIOAccessible( accessible );
		m_Memory.setRAMUnderIOAccessible( accessible );
		// Native delivery supersedes the old compatibility workaround. Leaving
		// relocation armed would redirect the very writes now able to reach RAM.
		// CC64Memory::setRAMUnderIOAccessible() also retires relocation when
		// native delivery is available.
	}

	// CIA2 timer NMI retiming (NMI_RETIME); see CC64Memory's retiming
	// section. On by default, same escape-hatch reasoning.
	void setNMIRetime( bool e )
	{
		m_Memory.m_NMIRetimeEnable = e;
		m_Memory.cia2RecomputeNMIState();
	}
	void setDeferNativeNMI( bool e ) { m_Core65816.m_DeferNativeNMI = e; }

	// Interrupt vector reroute into the accelerator EPROM (VECTOR_REROUTE);
	// see CC64Memory::interruptRerouteActive.
	void setVectorReroute( bool e ) { m_Memory.m_VectorRerouteEnable = e; }

	// Charge C64-bus accesses their real cost (IO_STRETCH); see
	// CC64Memory::tickFast.
	void setIOStretch( bool e ) { m_Memory.m_IOStretchEnable = e; }
	void setMirrorStretch( bool e ) { m_Memory.m_MirrorStretchEnable = e; }
	// Consecutive mirror opportunities skipped because the guest was inside a
	// cycle-counted raster window, and the lifetime total. The RUN is what
	// bounds the deferral -- see the guard in runFrame -- and the TOTAL is
	// the readout that says whether the guard ever fired, which is the first
	// thing to check if the screen does not change.
	u32 m_RasterDeferRun = 0;
	u64 m_RasterDeferred = 0;
	// How many consecutive opportunities may be skipped. Mario's window is
	// about 62 C64 cycles against a 134-cycle slice, so a single deferral
	// normally clears it; this is the runaway bound for a program that polls
	// $D012 continuously, not a tuning value.
	static constexpr u32 SCPU_RASTER_DEFER_LIMIT = 8;

	// Emulated BA stall; see CC64Memory::baStallSettle. mode: 0 off, 1 on
	// except while a serial transaction window is armed, 2 always. maxCycles
	// bounds ONE freeze, in C64 cycles: a badline is 43 and a badline that
	// runs into sprite DMA cannot fill a line, so the clamp below keeps every
	// configured value inside one line and out of the range where a stuck BA
	// would be mistaken for a long steal.
	void setBAStall( u32 mode, u32 maxCycles )
	{
		m_Memory.m_BAStallMode = (u8)( mode > 2 ? 2 : mode );
		// 80, not 128: RAD_BA_STALL_CEILING in rad_bus.cpp clamps the pin side
		// to 80 anyway, so a larger configured value would be silently
		// unreachable. One number, enforced in both places, so the config
		// documentation can state the real range.
		if ( maxCycles < 8 )  maxCycles = 8;
		if ( maxCycles > 80 ) maxCycles = 80;
		m_Memory.m_BAStallMaxCycles = (u16)maxCycles;
	}
	// Evidence required before a CIA2 access opens or extends the serial
	// transaction window. Clamped here as well as in readConfig, because the
	// tests reach the memory object through this setter and not through the
	// config file.
	void setIECWindow( u32 mode )
	{
		m_Memory.m_IECWindowMode = (u8)( mode > 2 ? 2 : mode );
	}

	// The hypothesis-free hang detector (CW65C816's W65_PROG_ block). On by
	// default: a diagnostic that needs a config edit to arm is a diagnostic
	// that costs a hardware run when the edit is forgotten, and its confirm
	// threshold is 8.4s of unbroken zero novelty specifically so that leaving
	// it on cannot cost a normal run anything.
	void setHangWatch( bool enable )
	{
		m_Core65816.m_ProgEnable = enable ? 1 : 0;
	}
	// Watched-RAM write forensics. Addresses above 0xFFFF disarm a slot; see
	// CC64Memory::WatchedWrite for why the hook is free on every unwatched
	// page and what the watched page itself pays.
	void setRAMWatch( u32 slot0, u32 slot1 )
	{
		m_Memory.setRAMWatch( slot0, slot1 );
	}
	u32  mirrorDisplayBudget() const         { return m_MirrorDisplayBudget; }

	void reset();

	// ARM cycles per emulated cycle for a pure interpreter workload, measured
	// at init() time by running a small loop entirely inside bank 1 -- no C64
	// bus involvement at all. This number isolates interpreter changes from
	// everything the frame scheduler, pacer and raster flushing add on top,
	// so build-to-build comparisons stay honest even when those change.
	// 0 when the 65816 core is not in use.
	u32 benchArmPerEmuCycle() const { return m_BenchArmPerEmuCycle; }
	// Representative paths the pure bank-1 loop cannot see. These use the same
	// ARM-cycles / base-65816-cycle unit, so regressions in bank-0 dispatch,
	// mirror bookkeeping, physical I/O, and SuperRAM batching are comparable.
	u32 m_BenchBank0ArmPerCycle = 0;
	u32 m_BenchScreenArmPerCycle = 0;
	u32 m_BenchIOArmPerCycle = 0;
	u32 m_BenchSuperRAMArmPerCycle = 0;
	u32 m_BenchSuperRAMStretchPer1k = 0;

	// PMU event rates measured over the same benchmark, each scaled to
	// per-1000-emulated-cycles so they read as densities. Zero on the host
	// build, which has no A53 PMU. These are what turn "128 arm/emucycle" into
	// a diagnosis: instruction-cache refills, data-cache refills, and branch
	// mispredicts each demand a completely different optimisation.
	u32 m_BenchInstrPer1k   = 0;	// instructions retired
	u32 m_BenchL1IRefillPer1k = 0;	// I-cache refills
	u32 m_BenchL1DRefillPer1k = 0;	// D-cache refills
	u32 m_BenchBranchMissPer1k = 0;	// mispredicted branches

	// Run the accelerator for approximately one C64 frame's worth of emulated
	// time, then flush pending mirrored writes. Returns cycles executed.
	u64 runFrame();

	// Run until stopped. Returns if stop() is called, otherwise never.
	void run();

	// Ask the run loop to return at the next frame boundary.
	void stop() { m_Running = false; }

	// Called once per frame while running. Returning true stops the loop --
	// used to make the RAD's button responsive without the bus layer having to
	// know anything about the accelerator.
	typedef bool (*FrameHook)( void *ctx );
	void setFrameHook( FrameHook hook, void *ctx ) { m_FrameHook = hook; m_FrameHookCtx = ctx; }

	CC64Memory         &memory()      { return m_Memory; }
	CFastRAM           &fastRAM()     { return m_FastRAM; }
	CSuperCPUMemoryMap &memoryMap()   { return m_MemoryMap; }
	CREU               &reu()         { return m_REU; }
	CWriteBuffer       &writeBuffer() { return m_WriteBuffer; }
	CSuperCPURegisters &registers()   { return m_Registers; }
	ICpu               *cpu()         { return m_CPU; }
	// Diagnostics sometimes need to paint a failure report directly into the
	// host's VIC-visible RAM after the emulated CPU has been stopped.
	IC64Bus            *hostBus()     { return m_Bus; }
	// Copy an IEC epoch in chronological order (oldest first). False means the
	// requested index is outside the retained fixed-size ring.
	u32 iecEpochDiagnosticCount() const;
	bool iecEpochDiagnostic( u32 index, SCPUIECEpochDiagnostic &out ) const;
	// Null unless the 65816 core is driving. For diagnostics only.
	CW65C816           *core65816()   { return ( m_CPU == &m_Core65816 ) ? &m_Core65816 : 0; }

	// Emulated 65816 cycles per second for each speed setting. The real
	// hardware is 20MHz in turbo and drops to the host's own 1MHz (2MHz on a
	// C128 in fast mode) when software asks for compatibility.
	u32 currentClockHz() const;

private:
	CC64Memory         m_Memory;
	CWriteBuffer       m_WriteBuffer;
	CSuperCPURegisters m_Registers;
	// The REU and the three pieces that connect it. The chain exists because
	// CC64Memory holds a single IIOInterceptor and m_Registers already has it.
	static void reuFF00Trampoline( void *context );
	CREU                m_REU;
	CREUInterceptor     m_REUInterceptor;
	CREUMemoryHost      m_REUHost;
	CIOInterceptorChain m_IOChain;
	CM6502             m_Core6502;
	CW65C816           m_Core65816;
	CFastRAM           m_FastRAM;
	CSuperCPUMemoryMap m_MemoryMap;

	ICpu    *m_CPU;
	IC64Bus *m_Bus;
	bool     m_Running;
	bool     m_BootmapEnabled;
	SCPUC128Mode m_C128Mode;
	u32      m_MirrorDisplayBudget;
	bool     m_MirrorHalted = false;
	bool     m_DisplayScrubEnabled = false;
	bool     m_DisplayScrubMasked = true;
	u32      m_DisplayScrubPeriodSeconds = 0;
	bool     m_DisplayScrubPhaseOn = false;
	u64      m_DisplayScrubPhaseStart = 0;
	u32      m_DisplayScrubPhaseGeneration = 0;
	bool     m_DisplayScrubBitmapSeen = false;
	bool     m_ResyncDiagnosticAwaitingDrain = false;
	bool     m_ResyncDiagnosticHolding = false;
	SCPUResyncDiagnosticPhase m_ResyncDiagnosticPhase = SCPU_RESYNC_DIAG_IDLE;
	u32      m_ResyncDiagnosticFramesRemaining = 0;
	u32      m_ResyncDiagnosticGeneration = 0;
	u32      m_ResyncDiagnosticStartHash = 0;
	bool     m_ResyncDiagnosticShadowChanged = false;
	u32      m_ResyncDiagnosticRepairCallsRemaining = 0;
	// runFrame() may advance the CPU to reach the physical VIC border before
	// draining mirrors. Those cycles belong to the following frame; carrying
	// them here prevents the border phase from becoming a variable turbo boost.
	u64      m_FrameTickDebt = 0;
	u32      m_BenchArmPerEmuCycle = 0;
	void benchmark65816();
	void updateIECEpochDiagnostic();
	void noteIECEpochDrainOpportunity();
	void updateDisplayScrubPhase( bool bitmapActive );
	void updateResyncDiagnostic( bool sawIEC, C64VideoStandard video );
	void runResyncDiagnosticRepair( const C64Signals &sig );
	u32 displayShadowHash() const;
	FrameHook m_FrameHook;
	void     *m_FrameHookCtx;
};

// Freeze the passive IEC ring outside the accelerator object before reset.
// Keeping both the live and frozen rings out of CSuperCPU preserves the exact
// hardware-validated data layout of the RAD bus object during acquisition.
void scpuFreezeIECEpochDiagnostic( const CSuperCPU *scpu, u32 hostHz );
u32 scpuFrozenIECEpochDiagnosticCount();
u32 scpuFrozenIECEpochHostHz();
bool scpuFrozenIECEpochDiagnostic( u32 index,
	                               SCPUIECEpochDiagnostic &out );

// Copy the live bounded VSP matrix comparisons. Safe after the emulated CPU
// has stopped; it only reads Pi-owned diagnostic storage.
void scpuVSPMatrixDiagnostic( SCPUVSPMatrixDiagnostic &out );

// Preserve one final physical-vs-shadow comparison when the heartbeat declares
// a Mario/VSP stall. This is diagnostic-only and waits for a hidden physical
// raster interval without advancing the guest or draining the mirror queue.
bool scpuCaptureVSPStallMatrixDiagnostic( CSuperCPU *scpu );

#define SCPU_TURBO_HZ   20000000u
#define SCPU_NORMAL_HZ   1000000u

#endif
