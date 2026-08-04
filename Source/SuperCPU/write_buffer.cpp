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
	: m_WritesAccepted( 0 ), m_WritesSkipped( 0 ), m_WritesCoalesced( 0 ),
	  m_BytesFlushed( 0 ), m_Flushes( 0 ), m_IOWindowSuppressed( 0 ),
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
}

void CWriteBuffer::clearDirty()
{
	for ( u32 i = 0; i < ( 0x10000 / 64 ); i++ )
		m_Dirty[ i ] = 0;
}

void CWriteBuffer::resetStats()
{
	m_WritesAccepted = m_WritesSkipped = m_WritesCoalesced = 0;
	m_BytesFlushed = m_Flushes = 0;
	m_IOWindowSuppressed = 0;
}

void CWriteBuffer::setOptMode( SCPUOptMode mode )
{
	// Anything already buffered was queued under the old policy; get it out
	// before the rules change.
	flush();

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
	(void)value;	// the current byte is read out of shadow RAM at flush time

	if ( !shouldMirror( addr ) )
	{
		m_WritesSkipped++;
		return;
	}

	m_WritesAccepted++;

	if ( isDirty( addr ) )
	{
		// Already queued. The flush reads the live value out of shadow RAM, so
		// there is nothing to update here -- this is where the coalescing win
		// comes from.
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
		onRamWrite( (u16)( addr + i ), 0 );
}

u32 CWriteBuffer::flushUpTo( u32 maxBytes )
{
	if ( m_Count == 0 || maxBytes == 0 )
		return m_Count;

	if ( !m_Bus || !m_RAM )
	{
		// No bus attached (host tests): drop the queue rather than grow forever.
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
			m_Burst[ i ].value = m_RAM[ a ];
		}

		m_Bus->writeBurst( m_Burst, n );

		for ( u32 i = 0; i < n; i++ )
		{
			u16 a = m_List[ ( m_Head + i ) & ( SCPU_WRITEBUF_CAPACITY - 1 ) ];
			m_Dirty[ a >> 6 ] &= ~( 1ULL << ( a & 63 ) );
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
	// Unbounded: used at points where correctness beats politeness, such as a
	// mode change that would otherwise strand writes queued under the old
	// policy. Prefer flushUpTo() from the raster-scheduled path.
	while ( flushUpTo( SCPU_WRITEBUF_CHUNK * 8 ) > 0 )
		;
}
