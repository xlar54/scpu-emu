/*
   SCPU-EMU - CMD SuperCPU emulation for the C64/C128 using a RAD Expansion Unit
   Copyright (c) 2026 SCPU-EMU contributors

   VIC-visible write mirroring.

   The accelerator runs out of RAM local to the Raspberry Pi, so the C64's own
   DRAM goes stale the moment we write anything. That is invisible to the
   program -- it reads back our copy -- but very visible to the VIC-II, which
   fetches screen, colour and bitmap data straight out of DRAM. Every write the
   VIC could observe therefore has to be pushed back across the expansion port,
   and that costs a C64 cycle each: roughly 1us, against the ~50ns an emulated
   65816 cycle takes. Mirroring is the bottleneck of the entire design.

   Two things make it affordable, and both are what the real SuperCPU does:

     * Coalescing. Writes are accumulated against a dirty-address bitmap rather
       than a FIFO, so a routine that fills the same screen byte a hundred times
       costs one mirrored write, not a hundred. A full 1000-byte screen clear
       becomes 1000 cycles no matter how the program got there.

     * Optimization modes. The SuperCPU lets software declare which regions the
       VIC actually reads -- $D074..$D077 and friends -- and skips mirroring
       everything else. A BASIC program only needs $0400-$07FF kept coherent;
       mirroring its zero page and stack traffic would be pure waste.

   Ordering rule: ordinary I/O never drains the buffer synchronously. The frame
   scheduler alone delivers queued RAM during sampled border windows. A display
   pointer can therefore expose stale data for a frame, but its exact raster
   write is preserved and no visible-region burst is generated.

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
#ifndef _scpu_write_buffer_h
#define _scpu_write_buffer_h

#include "../Common/types.h"
#include "../C64/c64_memory.h"
#include "../C64/c64_bus.h"

// Mirroring policy. Names follow the CMD documentation; see
// Docs/research/supercpu-registers.md for how each maps to a register write.
enum SCPUOptMode
{
	SCPU_OPT_NONE = 0,	// $D077 - mirror everything. Maximum compatibility.
	SCPU_OPT_DEFAULT,	// mirror everything except zero page and stack
	SCPU_OPT_BASIC,		// $D076 - only the standard screen at $0400-$07FF
	SCPU_OPT_VICBANK0,	// $0000-$3FFF
	SCPU_OPT_VICBANK1,	// $D075 - $4000-$7FFF
	SCPU_OPT_VICBANK2,	// $D074 - $8000-$BFFF (this is the GEOS setting)
	SCPU_OPT_VICBANK3,	// $C000-$FFFF
	SCPU_OPT_FULL,		// mirror nothing (C128 80-column mode: VDC has its own RAM)
	SCPU_OPT_MODES
};

// The dirty set can hold every address in the 64K space. It used to be 4096,
// which forced an automatic flush roughly every 4096 distinct writes -- and the
// KERNAL's RAM test alone touches ~38000 addresses, so it fired repeatedly at
// arbitrary raster positions. Bulk bus traffic across the visible display
// corrupts the VIC-II's fetches, so flushes must be schedulable rather than
// happening whenever a fixed-size buffer happens to fill.
#define SCPU_WRITEBUF_CAPACITY 65536

// Writes emitted per burst call. Keeps the scratch array small and gives the
// raster-scheduled flush a granularity at which it can stop.
#define SCPU_WRITEBUF_CHUNK 512

class CWriteBuffer : public IMirrorSink
{
public:
	CWriteBuffer();

	void attach( IC64Bus *bus, const u8 *ram );

	void setOptMode( SCPUOptMode mode );
	SCPUOptMode optMode() const { return m_Mode; }

	// When true, zero page and stack ($0000-$01FF) are never mirrored. The
	// SuperCPU exposes this as the "Z flag" alongside the optimization mode:
	// almost nothing points the VIC at page 0 or 1, and CPU traffic there is
	// heavy, so excluding it is a large win for a small compatibility risk.
	void setExcludeZeroPageStack( bool exclude ) { m_ExcludeZPStack = exclude; }

	// --- IMirrorSink ------------------------------------------------------
	void onRamWrite( u16 addr, u8 value ) override;

	// Flush everything, regardless of where the raster is. Correct but not
	// polite: a large flush here will disturb the display.
	void flush() override;

	// Forget everything queued without touching the C64 bus. Reset uses this:
	// bytes staged by the program that was just reset are obsolete, and blasting
	// them into the machine at an arbitrary raster position only corrupts the
	// new display state.
	void discard();

	// Flush at most maxBytes and return how many writes are still outstanding.
	// This is what the raster-scheduled path uses, so a transfer can be sized
	// to the time actually left in the safe window.
	u32 flushUpTo( u32 maxBytes ) override;

	// Mark a range dirty wholesale, e.g. after a block move. Cheaper than
	// funnelling every byte through onRamWrite().
	void invalidateRange( u16 addr, u32 length );

	u32 pending() const { return m_Count; }
	u32 pendingBytes() override { return m_Count; }

	// True when there is nothing outstanding.
	bool empty() const { return m_Count == 0; }

	// --- statistics -------------------------------------------------------
	u64 m_WritesAccepted;	// writes that the policy said to mirror
	u64 m_WritesSkipped;	// writes the policy discarded
	u64 m_WritesCoalesced;	// accepted writes that hit an already-dirty address
	u64 m_BytesFlushed;		// bytes actually sent over the bus
	u64 m_Flushes;

	// Writes aimed at $D000-$DFFF that were refused. Non-zero means the
	// emulated banking had the I/O window mapped as RAM while the real machine
	// had it mapped as I/O -- see the note in shouldMirror().
	mutable u64 m_IOWindowSuppressed;

	void resetStats();

	// Exposed for tests.
	bool shouldMirror( u16 addr ) const;

private:
	IC64Bus  *m_Bus;
	const u8 *m_RAM;

	SCPUOptMode m_Mode;
	bool        m_ExcludeZPStack;

	// Region the current mode mirrors, as an inclusive address range.
	u16 m_RangeLo, m_RangeHi;

	// Dirty set: bitmap for O(1) membership, list for O(n) iteration.
	u64 m_Dirty[ 0x10000 / 64 ];
	// Value accepted under the policy in force at the time of the write. Keeping
	// it here lets an optimisation-mode change take effect immediately without
	// synchronously flushing the old queue. A later write excluded by the new
	// policy must not change the value of an already queued old-policy write.
	u8  m_PendingValue[ SCPU_WRITEBUF_CAPACITY ];
	// A RING, consumed oldest-first. It used to be a stack flushed from the
	// tail, which is one line simpler -- and starves. A program that
	// continuously re-dirties memory (a scrolling PRINT loop rewrites the
	// whole screen every line) keeps feeding the tail, the tail keeps getting
	// flushed first, and the head entries never drain: parts of the real
	// screen track the shadow while others stay frozen at whatever they held
	// minutes ago. FIFO makes every dirty byte reach the C64 in bounded time.
	// Capacity covers the whole address space and m_Head wraps with a mask.
	u16 m_List[ SCPU_WRITEBUF_CAPACITY ];
	u32 m_Head;
	u32 m_Count;

	// Scratch used to build the burst handed to the bus.
	C64BusWrite m_Burst[ SCPU_WRITEBUF_CHUNK ];

	inline bool isDirty( u16 a ) const { return ( m_Dirty[ a >> 6 ] >> ( a & 63 ) ) & 1; }
	inline void setDirty( u16 a )      { m_Dirty[ a >> 6 ] |= ( 1ULL << ( a & 63 ) ); }
	// A rewritten sprite pointer is a commit record for the sprite data it
	// selects. Move its existing ring entry behind everything currently queued
	// without creating a duplicate dirty entry.
	void moveDirtyToTail( u16 addr );
	void clearDirty();
};

#endif
