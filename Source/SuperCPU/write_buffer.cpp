/*
   SCPU-EMU - CMD SuperCPU emulation for the C64/C128 using a RAD Expansion Unit
   Copyright (c) 2026 SCPU-EMU contributors

   VIC-visible write mirroring.

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
#include "write_buffer.h"

CWriteBuffer::CWriteBuffer()
	: m_WritesAccepted( 0 ), m_WritesEliminated( 0 ),
	  m_RelocForwarded( 0 ), m_RelocShielded( 0 ), m_BytesResynced( 0 ),
	  m_WritesSkipped( 0 ), m_WritesCoalesced( 0 ),
	  m_BytesFlushed( 0 ), m_Flushes( 0 ),
	  m_IOWindowSuppressed( 0 ),
	  m_Bus( 0 ), m_RAM( 0 ),
	  m_Mode( SCPU_OPT_DEFAULT ), m_ExcludeZPStack( true ),
	  m_RangeLo( 0x0000 ), m_RangeHi( 0xFFFF ),
	  m_Head( 0 ), m_Count( 0 )
{
	clearDirty();
	setOptMode( SCPU_OPT_DEFAULT );
}

void CWriteBuffer::attach( IC64Bus *bus, const u8 *ram )
{
	m_Bus = bus;
	m_RAM = ram;
	for ( u32 i = 0; i < 0x10000 / 64; i++ ) m_Synced[ i ] = 0;
}

void CWriteBuffer::clearDirty()
{
	for ( u32 i = 0; i < ( 0x10000 / 64 ); i++ )
		m_Dirty[ i ] = 0;
}

static inline bool isPotentialVICSpritePointer( u16 addr )
{
	// Every possible VIC screen is aligned to 1KB. Its last eight bytes are
	// the sprite-pointer table, regardless of which 16KB VIC bank contains it.
	return ( addr & 0x03FF ) >= 0x03F8;
}

void CWriteBuffer::moveDirtyToTail( u16 addr )
{
	if ( m_Count < 2 )
		return;

	const u32 mask = SCPU_WRITEBUF_CAPACITY - 1;
	u32 position = 0;
	while ( position < m_Count && m_List[ ( m_Head + position ) & mask ] != addr )
		position++;

	// isDirty() guarantees an entry should exist. Be defensive if the bitmap
	// and ring are ever damaged, and avoid work when it is already last.
	if ( position >= m_Count || position + 1 == m_Count )
		return;

	for ( u32 i = position; i + 1 < m_Count; i++ )
		m_List[ ( m_Head + i ) & mask ] = m_List[ ( m_Head + i + 1 ) & mask ];

	m_List[ ( m_Head + m_Count - 1 ) & mask ] = addr;
}

void CWriteBuffer::resetStats()
{
	m_WritesAccepted = m_WritesEliminated = m_WritesSkipped = m_WritesCoalesced = 0;
	m_RelocForwarded = m_RelocShielded = 0;
	m_BytesResynced = 0;
	m_BytesFlushed = m_Flushes = 0;
	m_IOWindowSuppressed = 0;
}

void CWriteBuffer::setOptMode( SCPUOptMode mode )
{
	m_Mode = mode;

	switch ( mode )
	{
	case SCPU_OPT_BASIC:     m_RangeLo = 0x0400; m_RangeHi = 0x07FF; break;
	case SCPU_OPT_VICBANK0:  m_RangeLo = 0x0000; m_RangeHi = 0x3FFF; break;
	case SCPU_OPT_VICBANK1:  m_RangeLo = 0x4000; m_RangeHi = 0x7FFF; break;
	case SCPU_OPT_VICBANK2:  m_RangeLo = 0x8000; m_RangeHi = 0xBFFF; break;
	case SCPU_OPT_VICBANK3:  m_RangeLo = 0xC000; m_RangeHi = 0xFFFF; break;
	case SCPU_OPT_FULL:      m_RangeLo = 0xFFFF; m_RangeHi = 0x0000; break;	// empty
	case SCPU_OPT_NONE:
	case SCPU_OPT_DEFAULT:
	default:                 m_RangeLo = 0x0000; m_RangeHi = 0xFFFF; break;
	}
}

bool CWriteBuffer::shouldMirror( u16 addr ) const
{
	if ( m_Mode == SCPU_OPT_FULL )
		return false;

	// SCPU_OPT_NONE means "no optimization": mirror everything, zero page and
	// stack included. This is derived from the mode rather than latched into
	// m_ExcludeZPStack, because clearing the flag on the way into NONE left it
	// clear on the way back out -- so a later switch to DEFAULT kept mirroring
	// page 0 and 1, contradicting what DEFAULT means.
	if ( m_Mode != SCPU_OPT_NONE && m_ExcludeZPStack && addr < 0x0200 )
		return false;

	// NEVER mirror $D000-$DFFF. On the real machine that window is chip
	// registers -- VIC-II, SID, CIAs, colour RAM -- not DRAM the VIC fetches
	// from, so there is nothing there to keep coherent.
	//
	// Mirroring it is actively destructive. Our emulated $01 can decide the
	// window is RAM (CHAREN clear, or LORAM and HIRAM both clear), in which
	// case CC64Memory::write8() routes the write to shadow RAM and queues it
	// here. But the halted 6510 leaves the *real* machine's $01 at $37, so I/O
	// is always banked in over there and the mirrored byte lands on the actual
	// register. A stray write to $D011 clears DEN and corrupts the raster
	// compare; one to $DD00 moves the VIC's 16K bank. Either produces a blanked
	// or rolling picture -- which is precisely the fault this was hunting.
	if ( addr >= 0xD000 && addr <= 0xDFFF )
	{
		m_IOWindowSuppressed++;
		return false;
	}

	return ( addr >= m_RangeLo && addr <= m_RangeHi );
}

bool CWriteBuffer::onRamWrite( u16 addr, u8 value )
{
	// Under-I/O shape relocation, when armed. A write into a relocated
	// $D000-$DFFF block is forwarded to the block's deliverable copy; a
	// program write into the copy's own underlying address is suppressed,
	// because that DRAM now belongs to the relocated shape and is only ever
	// fetched by the VIC through the translated pointer. Both directions
	// disarm same-value elimination: the shadow at the delivery address holds
	// the program's data, not what was last put on the bus.
	if ( m_RelocCount && *m_RelocCount )
	{
		if ( addr >= 0xD000 )
		{
			const u8 r = m_PtrReloc[ ( addr >> 6 ) & 0x3F ];
			if ( addr <= 0xDFFF && r != 0xFF )
			{
				addr = (u16)( 0xC000 + ( (u16)r << 6 ) + ( addr & 63 ) );
				m_RelocForwarded++;
				m_Synced[ addr >> 6 ] &= ~( 1ULL << ( addr & 63 ) );
			}
		}
		else if ( addr >= 0xC000 && addr < 0xCC00
		          && m_RelocInUse[ ( (u32)addr - 0xC000 ) >> 6 ] != 0xFF )
		{
			m_RelocShielded++;
			m_Synced[ addr >> 6 ] &= ~( 1ULL << ( addr & 63 ) );
			return false;
		}
	}

	if ( !shouldMirror( addr ) )
	{
		m_WritesSkipped++;
		// The shadow byte is about to change while real DRAM does not, so
		// "delivered once" stops implying "still equal": disarm elimination
		// for this address until something delivers it again. Without this,
		// a program that writes data under a restrictive optimisation mode
		// and then rewrites the SAME values under a permissive one gets
		// eliminated into permanent DRAM staleness -- 3D Pool stages sprite
		// shapes at $0200-$03FF during init exactly that way, and its balls
		// stayed garbage on the real screen while the shadow was perfect.
		m_Synced[ addr >> 6 ] &= ~( 1ULL << ( addr & 63 ) );
		return false;
	}

	// The caller stores AFTER this sink runs, so m_RAM still holds what real
	// DRAM holds whenever the address is clean and has been delivered before.
	// A write that changes nothing then queues nothing. Programs re-render
	// unconditionally every frame -- 3D Pool rewrites ~44K bytes per frame on
	// a completely static screen -- and echoing the resulting identical bytes
	// onto the bus mid-display was pure collision exposure for sprite DMA.
	{
		const u32 w  = addr >> 6;
		const u64 bit = 1ULL << ( addr & 63 );
		if ( m_RAM && value == m_RAM[ addr ]
		     && !( m_Dirty[ w ] & bit )
		     && ( m_Synced[ w ] & bit ) )
		{
			m_WritesEliminated++;
			return false;
		}
	}

	m_WritesAccepted++;
	m_PendingValue[ addr ] = value;

	if ( isDirty( addr ) )
	{
		// Already queued. m_PendingValue was updated above, so the latest write
		// accepted under the current policy replaces the old one without adding
		// another address to the ring. A sprite-pointer byte is different from
		// ordinary data: its newest value selects the data written before it, so
		// move that existing entry behind everything currently queued. Otherwise
		// coalescing can expose a new pointer before its sprite bytes arrive.
		if ( isPotentialVICSpritePointer( addr ) )
			moveDirtyToTail( addr );
		m_WritesCoalesced++;
		return true;
	}

	setDirty( addr );

	// Cannot overflow: the dirty bitmap admits each address at most once and
	// the ring is sized for the whole 64K space.
	if ( m_Count < SCPU_WRITEBUF_CAPACITY )
	{
		m_List[ ( m_Head + m_Count ) & ( SCPU_WRITEBUF_CAPACITY - 1 ) ] = addr;
		m_Count++;
	}
	return true;
}

void CWriteBuffer::invalidateRange( u16 addr, u32 length )
{
	for ( u32 i = 0; i < length; i++ )
	{
		const u16 a = (u16)( addr + i );
		onRamWrite( a, m_RAM ? m_RAM[ a ] : 0 );
	}
}

u32 CWriteBuffer::flushUpTo( u32 maxBytes )
{
	return flushUpToPolicy( maxBytes, false );
}

u32 CWriteBuffer::flushUpToPolicy( u32 maxBytes, bool deferHot )
{
	if ( m_Count == 0 || maxBytes == 0 )
		return m_Count;

	if ( !m_Bus )
	{
		// No bus attached: drop the queue rather than grow forever.
		clearDirty();
		m_Count = 0;
		m_Head = 0;
		return 0;
	}

	u32 sent = 0;

	while ( m_Count > 0 && sent < maxBytes )
	{
		u32 n = m_Count;
		if ( n > SCPU_WRITEBUF_CHUNK )     n = SCPU_WRITEBUF_CHUNK;
		if ( n > ( maxBytes - sent ) )     n = maxBytes - sent;

		// Consume from the HEAD: oldest dirty address first. See the note on
		// m_List -- flushing newest-first starves the head under continuous
		// re-dirtying, which shows up as regions of the real screen frozen at
		// stale contents while the rest updates.
		bool stoppedAtHot = false;
		for ( u32 i = 0; i < n; i++ )
		{
			u16 a = m_List[ ( m_Head + i ) & ( SCPU_WRITEBUF_CAPACITY - 1 ) ];
			if ( deferHot && m_HotBlocks )
			{
				const u32 blk = (u32)a >> 6;
				if ( ( m_HotBlocks[ blk >> 6 ] >> ( blk & 63 ) ) & 1 )
				{
					// An active sprite-shape byte: the rest of this drain
					// waits for the border, keeping the block's delivery
					// atomic from the VIC's point of view.
					n = i;
					stoppedAtHot = true;
					break;
				}
			}
			m_Burst[ i ].addr  = a;
			// Active-row pointer bytes translate at delivery; everything else
			// goes out as queued.
			m_Burst[ i ].value = deliverValue( a, m_PendingValue[ a ] );
		}
		if ( n == 0 )
			return m_Count;		// hot byte at the head; nothing to send

		m_Bus->writeBurst( m_Burst, n );

		for ( u32 i = 0; i < n; i++ )
		{
			u16 a = m_List[ ( m_Head + i ) & ( SCPU_WRITEBUF_CAPACITY - 1 ) ];
			m_Dirty[ a >> 6 ] &= ~( 1ULL << ( a & 63 ) );
			// Delivered: real DRAM now provably matches the queued value, so
			// same-value elimination becomes sound for this address.
			m_Synced[ a >> 6 ] |= 1ULL << ( a & 63 );
		}

		m_Head  = ( m_Head + n ) & ( SCPU_WRITEBUF_CAPACITY - 1 );
		m_Count -= n;
		sent    += n;

		if ( stoppedAtHot )
			break;
	}

	m_BytesFlushed += sent;
	m_Flushes++;

	return m_Count;
}

void CWriteBuffer::resyncSweep( u32 maxBytes )
{
	if ( !m_Bus || !m_RAM || maxBytes == 0 )
		return;

	u32 n = 0;
	u32 guard = 0x10000;		// at most one full lap per call
	while ( n < maxBytes && guard-- )
	{
		const u16 a = m_ResyncCursor;
		m_ResyncCursor = ( m_ResyncCursor == m_RangeHi )
		               ? m_RangeLo : (u16)( m_ResyncCursor + 1 );

		if ( !shouldMirror( a ) )
			continue;				// honours the I/O window and exclusions
		if ( isDirty( a ) )
			continue;				// pending value owns this address

		u8 v = m_RAM[ a ];
		if ( m_RelocCount && *m_RelocCount && a >= 0xC000 && a < 0xCC00 )
		{
			// A stolen block heals from its under-I/O SOURCE, not from the
			// shadow underneath it -- the shadow there is program data whose
			// DRAM copy is deliberately dead. This is also what repairs the
			// rare mis-elimination a value-collision can cause.
			const u8 srcV = m_RelocInUse[ ( (u32)a - 0xC000 ) >> 6 ];
			if ( srcV != 0xFF )
				v = m_RAM[ 0xC000 + ( (u32)srcV << 6 ) + ( a & 63 ) ];
		}
		m_Burst[ n ].addr  = a;
		m_Burst[ n ].value = deliverValue( a, v );
		m_Synced[ a >> 6 ] |= 1ULL << ( a & 63 );
		if ( ++n == SCPU_WRITEBUF_CHUNK )
			break;
	}
	if ( n )
	{
		m_Bus->writeBurst( m_Burst, n );
		m_BytesResynced += n;
	}
}

void CWriteBuffer::flush()
{
	// Unbounded: reserve this for callers which explicitly know that bus traffic
	// is safe. Policy changes retain their already-accepted values in the queue;
	// reset discards obsolete entries. Normal display work uses flushUpTo().
	while ( flushUpTo( SCPU_WRITEBUF_CHUNK * 8 ) > 0 )
		;
}

void CWriteBuffer::discard()
{
	clearDirty();
	// Reset clears the shadow wholesale, so "shadow == real DRAM" stops
	// being true anywhere; elimination re-arms address by address as the
	// new program's writes get delivered.
	for ( u32 i = 0; i < 0x10000 / 64; i++ ) m_Synced[ i ] = 0;
	m_Head = 0;
	m_Count = 0;
}
