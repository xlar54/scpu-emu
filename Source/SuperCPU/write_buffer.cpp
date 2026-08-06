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

void CWriteBuffer::onRamWrite( u16 addr, u8 value )
{
	if ( !shouldMirror( addr ) )
	{
		m_WritesSkipped++;
		return;
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
			return;
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
		return;
	}

	setDirty( addr );

	// Cannot overflow: the dirty bitmap admits each address at most once and
	// the ring is sized for the whole 64K space.
	if ( m_Count < SCPU_WRITEBUF_CAPACITY )
	{
		m_List[ ( m_Head + m_Count ) & ( SCPU_WRITEBUF_CAPACITY - 1 ) ] = addr;
		m_Count++;
	}
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
		for ( u32 i = 0; i < n; i++ )
		{
			u16 a = m_List[ ( m_Head + i ) & ( SCPU_WRITEBUF_CAPACITY - 1 ) ];
			m_Burst[ i ].addr  = a;
			m_Burst[ i ].value = m_PendingValue[ a ];
		}

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
	}

	m_BytesFlushed += sent;
	m_Flushes++;

	return m_Count;
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
