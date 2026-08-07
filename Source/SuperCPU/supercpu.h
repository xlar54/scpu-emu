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

// Mirrored bytes allowed per drain call while the beam is inside the picture.
// Effectively the mirror's write-through throughput; see
// CSuperCPU::setMirrorDisplayBudget for the reasoning.
#define SCPU_MIRROR_DISPLAY_BYTES_DEFAULT 1024

class CSuperCPU
{
public:
	CSuperCPU();

	// Wire up a bus and bring the accelerator to its reset state. Returns
	// false if the bus could not be acquired or no usable ROMs were obtained.
	// simmMB: SuperRAM to fit, in megabytes. 0/1/4/8/16, as the real card
	// accepts. Reachable only by the 65816 core -- a 6502 has no way to name an
	// address above $FFFF, so with SCPU_CORE_6502 the SIMM is fitted but idle.
	bool init( IC64Bus *bus, SCPUCoreType core = SCPU_CORE_6502,
	           u32 simmMB = SCPU_SIMM_16MB );

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

	// Diagnostic kill switch: when set, runFrame stops ALL mirror bus traffic
	// -- flushes and the resync sweep -- leaving real DRAM untouched so the
	// screen shows exactly what it holds. See MIRROR_HALT_AFTER_S.
	void setMirrorHalted( bool halted ) { m_MirrorHalted = halted; }
	bool mirrorHalted() const           { return m_MirrorHalted; }

	// Under-I/O sprite-shape relocation (MIRROR_D000_RELOCATE); see
	// CC64Memory's relocation section. On by default; the switch exists so a
	// game that goes wrong with it can prove it with one config line.
	void setUnderIORelocate( bool e ) { m_Memory.m_RelocEnable = e; }

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
	u32  mirrorDisplayBudget() const         { return m_MirrorDisplayBudget; }

	void reset();

	// ARM cycles per emulated cycle for a pure interpreter workload, measured
	// at init() time by running a small loop entirely inside bank 1 -- no C64
	// bus involvement at all. This number isolates interpreter changes from
	// everything the frame scheduler, pacer and raster flushing add on top,
	// so build-to-build comparisons stay honest even when those change.
	// 0 when the 65816 core is not in use.
	u32 benchArmPerEmuCycle() const { return m_BenchArmPerEmuCycle; }

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
	CWriteBuffer       &writeBuffer() { return m_WriteBuffer; }
	CSuperCPURegisters &registers()   { return m_Registers; }
	ICpu               *cpu()         { return m_CPU; }
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
	CM6502             m_Core6502;
	CW65C816           m_Core65816;
	CFastRAM           m_FastRAM;
	CSuperCPUMemoryMap m_MemoryMap;

	ICpu    *m_CPU;
	IC64Bus *m_Bus;
	bool     m_Running;
	bool     m_BootmapEnabled;
	u32      m_MirrorDisplayBudget;
	bool     m_MirrorHalted = false;
	// runFrame() may advance the CPU to reach the physical VIC border before
	// draining mirrors. Those cycles belong to the following frame; carrying
	// them here prevents the border phase from becoming a variable turbo boost.
	u64      m_FrameTickDebt = 0;
	u32      m_BenchArmPerEmuCycle = 0;

	void benchmark65816();
	FrameHook m_FrameHook;
	void     *m_FrameHookCtx;
};

#define SCPU_TURBO_HZ   20000000u
#define SCPU_NORMAL_HZ   1000000u

#endif
