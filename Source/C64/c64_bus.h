/*
   SCPU-EMU - CMD SuperCPU emulation for the C64/C128 using a RAD Expansion Unit
   Copyright (c) 2026 SCPU-EMU contributors

   IC64Bus - the abstraction of "the physical Commodore 64 on the other side of
   the expansion port".

   Two implementations exist:
     * Bus/RAD/rad_bus   - drives the real machine over the RAD cartridge, one
                           access per ~1us C64 cycle, while holding the 6510
                           off the bus via DMA.
     * Bus/Host/host_bus - a plain in-memory stand-in so the CPU cores, banking
                           model and memory map can be tested on a PC.

   Cost model, which the layers above must respect: every call here that is not
   a no-op costs a real C64 bus cycle (~1us on a PAL machine). Reads of shadowed
   RAM must never come through this interface -- they are served from the Pi's
   own copy. Only I/O accesses and VIC-visible write mirroring belong here.
   This mirrors how the real SuperCPU behaves: 20MHz out of its own SRAM,
   dropping to 1MHz whenever it must actually touch the C64.

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
#ifndef _c64_bus_h
#define _c64_bus_h

#include "../Common/types.h"
#include "c64_signals.h"

// One entry of a mirrored-write burst.
typedef struct
{
	u16 addr;
	u8  value;
} C64BusWrite;

class IC64Bus
{
public:
	virtual ~IC64Bus() {}

	// Bring the bus up: detect the machine, take it over. After a successful
	// call the host 6510 is held off the bus and we are bus master.
	// Returns false if no running machine was found.
	virtual bool acquire() = 0;

	// Hand the machine back (release DMA). Mainly for a clean shutdown.
	virtual void release() = 0;

	virtual const C64Signals &signals() const = 0;

	// --- single accesses, one C64 cycle each -----------------------------
	virtual u8   read( u16 addr ) = 0;
	virtual void write( u16 addr, u8 value ) = 0;

	// Read the physical DRAM cell regardless of whether $D000-$DFFF is exposed
	// as I/O to an ordinary guest access. The distinction exists on real
	// SuperCPU hardware: /GAME selects I/O for read()/write(), while mirror
	// verification leaves the specially prepared 6510 port in all-RAM mode.
	// Host backends have only one flat array, so their implementation is the
	// same operation with the same accounting.
	virtual u8 readRAM( u16 addr ) = 0;
	virtual void writeRAM( u16 addr, u8 value ) = 0;

	// Verify the C64-personality CIA2 direction-register seed after the caller
	// has written it. The RAD backend samples several points inside the
	// calibrated read eye and discards bus-turnaround reads before accepting
	// the value; a single expansion-bus read is not authoritative on every
	// physical machine. This hook must not write any register itself.
	virtual bool verifyC64CIA2DDRA( u8 expected ) = 0;

	// --- burst writes ----------------------------------------------------
	// Amortises the per-access setup by keeping the address latch and bus
	// transceiver configured across the whole run. This is the path used to
	// flush the VIC-visible write buffer, and is roughly one C64 cycle per
	// byte rather than the two or three a naive write() sequence costs.
	virtual void writeBurst( const C64BusWrite *writes, u32 count ) = 0;

	// --- bulk read -------------------------------------------------------
	// Copies 'length' bytes starting at 'addr' into 'dst'. Used at start-up to
	// snapshot the C64's ROMs into the Pi's shadow memory.
	virtual void readBlock( u16 addr, u8 *dst, u32 length ) = 0;

	// --- interrupt lines, as seen on the expansion port -------------------
	virtual bool irqAsserted() = 0;
	virtual bool nmiAsserted() = 0;

	// Both lines in one call. On the RAD backend the two are bits of the SAME
	// GPIO level register, and an MMIO read of that register costs on the order
	// of a hundred ARM cycles -- so one combined read instead of two separate
	// virtual calls halves the cost even before any caching above this layer.
	virtual void sampleInterrupts( bool &irq, bool &nmi )
	{
		irq = irqAsserted();
		nmi = nmiAsserted();
	}

	// Current VIC-II raster line, or 0xFFFF if the backend cannot tell.
	// A coherent sample costs three bus cycles on real hardware
	// ($D011/$D012/$D011), and up to twelve when a boundary forces retries.
	virtual u16 rasterLine() { return 0xFFFF; }

	// --- host time source, for pacing ------------------------------------
	// A free-running counter and its rate, used to hold the emulated CPU to
	// real time. Returning 0 from hostCyclesPerSec() means "no clock here",
	// which disables pacing -- what the host test backend does, so tests keep
	// running as fast as the machine allows.
	virtual u64 hostCycles() { return 0; }
	virtual u32 hostCyclesPerSec() { return 0; }

	// Timestamp (in hostCycles units) of a PHI2 rising edge observed JUST NOW
	// -- the start of a CPU half-cycle -- or 0 when the backend cannot see
	// PHI. The pacer uses this to phase-lock its cycle grid to the physical
	// bus: exact intra-opcode settlement is worthless if the settled moment
	// can land on either side of a PHI edge at random. Implementations only
	// observe the line; they must not drive anything. May busy-wait up to
	// about two C64 cycles.
	virtual u64 phiCpuHalfStartHostCycles() { return 0; }

	// True when the backend's NEXT single write pays the read->write
	// transceiver turnaround (K243: a real C64 measurably lost the write
	// without it). The RAD backend pays that idle pass INSIDE the write's own
	// resync -- the one cycle a read->write sequence always has between the
	// two, where a 6510 puts an RMW's dummy write -- so it no longer displaces
	// the drive and cycle-exact placement no longer aims a turnaround write a
	// cycle early. Diagnostic and test visibility only. State changes with
	// every physical access; query at the access, never cache.
	virtual bool writeTurnaroundPending() { return false; }

	// Host stamp (hostCycles units) taken by the most recent physical single
	// read just after it aligned to the PHI2 falling edge that began its data
	// cycle: the START of the cycle the read occupied, to within one edge poll.
	// 0 when the backend keeps no such stamp. The raster-window catch-up
	// anchors on this and not on the read's RETURN, which is one whole cycle
	// and a teardown later: a count target built on the return held every
	// placed write until the start of the cycle it belonged in, and the
	// primitive's resync then carried it one whole cycle past.
	virtual u64 lastReadCycleStartHostCycles() { return 0; }

	// RASTER-WINDOW TURNAROUND (K504). While true, a single write that follows
	// a read pays its K243 idle pass INSIDE its own resync -- one idle period,
	// the 6510's dummy-write cycle -- instead of as a second period after it.
	// The memory layer raises it only around a VIC write placed inside the
	// raster window (a $D012 anchor at 1MHz within 200 cycles), which is where
	// that placement was measured, and clears it right after. Every other
	// write keeps the resync-then-pass order validated for general traffic
	// (K243: two idle periods); K503 applied the short form to every write and
	// Metal Dust's loader showed half-drawn screens and wrong colour cells.
	virtual void setWriteTurnaroundMinimal( bool on ) { (void)on; }

	// How many whole C64 cycles the VIC is holding the bus for RIGHT NOW,
	// having waited them out. Observation only: no latch, no transceiver, no
	// address and no /GAME is driven, so this is safe from any core-0 context
	// between bus accesses -- the same contract as phiCpuHalfStartHostCycles().
	//
	// WHY THE EMULATED CPU NEEDS IT. On a badline the VIC pulls BA low and a
	// real 6510 is FROZEN: it cannot fetch, so a NOP sled stops advancing.
	// readFast() serves $0800-$0FFF out of the m_ReadBase shadow at every
	// speed, so Mario's 24-NOP IRQ stabiliser issues no physical transaction,
	// HANDLE_BUS_AVAILABLE_READ never runs, and the sled races through a steal
	// the real machine absorbs. Measured K451: the second IRQ then arrives at
	// $0849 inside the DEX loop instead of in the sled, TXS writes S = $0100 +
	// X = $0104, and the machine hangs spinning on $9F88.
	//
	// Returning 0 is the common answer and must cost no more than one level
	// read: the caller asks once per emulated instruction at 1MHz. A backend
	// that cannot see BA returns 0 and the stall does not exist there.
	// maxCycles bounds the wait, and a backend must never exceed it: these
	// spins run under runFrame(), so a BA that never came back would take the
	// frame loop and the button hook with it (see RAD_BA_WAIT_LIMIT).
	virtual u32 busStealCycles( u32 maxCycles ) { (void)maxCycles; return 0; }

	virtual const char *name() const = 0;
};

#endif
