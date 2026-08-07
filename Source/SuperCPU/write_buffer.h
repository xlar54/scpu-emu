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
// Docs/SuperCPU64/supercpu-registers.md for how each maps to a register write.
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
	bool onRamWrite( u16 addr, u8 value ) override;

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

	// Background convergence: deliver up to maxBytes of CLEAN shadow bytes
	// from a rotating cursor over the mirrored range, marking them synced.
	// Real DRAM is write-only from the Pi, so shadow==DRAM can never be
	// verified -- only re-established. Divergence has real sources: CMD's
	// boot RAM test writes under a mirror-skipping policy, optimisation-mode
	// switches strand earlier writes, and a glitched burst write can corrupt
	// a byte the eliminator will then never resend. The sweep makes all of
	// them heal within seconds instead of persisting forever. Call only when
	// the raster is somewhere safe; the caller owns that judgement.
	void resyncSweep( u32 maxBytes );

	// Bitmap of 64-byte blocks currently selected by ACTIVE sprite pointers
	// (owned by CC64Memory). Bytes inside them are delivered only by border
	// drains, and a display drain stops when it meets one: a re-rendered
	// sprite shape must reach real DRAM whole, never as the render's
	// cleared/partial transient, because the VIC fetches shapes on the
	// sprite's own display lines.
	void attachHotShapeBlocks( const u64 *bits ) { m_HotBlocks = bits; }

	// flushUpTo with a policy: deferHot stops at the first hot-shape byte.
	// The IMirrorSink flushUpTo() delegates here with deferHot=false.
	u32 flushUpToPolicy( u32 maxBytes, bool deferHot );

	// Under-I/O shape relocation wiring; the tables are owned by CC64Memory
	// (see its relocation section for the why). ptrReloc maps an under-I/O
	// block (V-$40, 64 entries) to its relocated block; inUse maps a
	// relocated block (48 entries) back to its source V, $FF meaning free;
	// count is the live allocation total and the fast-out for every check
	// here.
	void attachRelocation( const u8 *ptrReloc, const u8 *inUse,
	                       const u8 *count )
	{
		m_PtrReloc   = ptrReloc;
		m_RelocInUse = inUse;
		m_RelocCount = count;
	}

	// Deliver-time translation for pointer-row bytes: a bank-3 row byte
	// selecting an under-I/O block goes out as its relocated block, the only
	// address the real VIC can be fed the shape at. Identity otherwise.
	//
	// "Row byte" is decided by address SHAPE -- the last eight bytes of any
	// 1K screen slot in bank 3 -- not by the currently active row. A
	// double-buffering game has TWO live rows and flips between them every
	// frame (3D Pool flips $DD00 in its raster IRQ); a queued row byte
	// regularly flushes while the OTHER row is active, and the background
	// sweep periodically re-delivers the inactive row. Both would put the
	// raw under-I/O value back on the real screen's next flip. A data byte
	// that merely LOOKS like a row entry gets translated too; that only
	// touches its dead DRAM copy -- visible solely if the VIC displays it as
	// a pointer row, in which case translating it was correct.
	inline u8 deliverValue( u16 a, u8 v ) const
	{
		// A relocated 64-byte block is shape data even when its final eight
		// addresses happen to look like a bank-3 sprite-pointer row. Translating
		// those data bytes corrupts blocks 15, 31 and 47 deterministically.
		if ( m_RelocCount && *m_RelocCount
		     && a >= 0xC000 && a < 0xCC00
		     && m_RelocInUse[ ( a - 0xC000 ) >> 6 ] != 0xFF )
			return v;
		if ( m_RelocCount && *m_RelocCount
		     && a >= 0xC000 && ( a & 0x3FF ) >= 0x3F8
		     && v >= 0x40 && v < 0x80 )
		{
			const u8 r = m_PtrReloc[ v - 0x40 ];
			if ( r != 0xFF )
				return r;
		}
		return v;
	}

	u32 pending() const { return m_Count; }
	u32 pendingBytes() override { return m_Count; }

	// True when there is nothing outstanding.
	bool empty() const { return m_Count == 0; }

	// --- statistics -------------------------------------------------------
	u64 m_WritesAccepted;	// writes that the policy said to mirror
	u64 m_WritesEliminated;	// writes dropped because DRAM already held the value
	u64 m_RelocForwarded;	// under-I/O writes redirected to a relocated block
	u64 m_RelocShielded;	// program writes into stolen blocks, suppressed
	u64 m_BytesResynced;	// background-sweep bytes re-delivered
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
	// Addresses this buffer has delivered at least once since attach/discard.
	// Only for those is "shadow == real DRAM when clean" a fact rather than a
	// hope, which is what makes same-value elimination sound: after a reset
	// the two diverge wholesale, and an eliminated write there would leave
	// stale DRAM on screen forever.
	u64 m_Synced[ 0x10000 / 64 ];
	u16 m_ResyncCursor = 0;
	const u64 *m_HotBlocks = 0;
	// Relocation wiring; see attachRelocation().
	const u8  *m_PtrReloc = 0;
	const u8  *m_RelocInUse = 0;
	const u8  *m_RelocCount = 0;
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
