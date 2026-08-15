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
	  m_DisplayScrubBytes( 0 ), m_DisplayScrubMatrixBytes( 0 ),
	  m_DisplayScrubBitmapBytes( 0 ), m_DisplayScrubCharsetBytes( 0 ),
	  m_DisplayScrubSpriteBytes( 0 ),
	  m_WritesSkipped( 0 ), m_WritesCoalesced( 0 ),
	  m_BytesFlushed( 0 ), m_Flushes( 0 ),
	  m_PolicyEntriesExamined( 0 ), m_PolicyZeroProgressScans( 0 ),
	  m_PolicyNoProgressCacheHits( 0 ),
	  m_IOWindowSuppressed( 0 ),
	  m_Bus( 0 ), m_RAM( 0 ),
	  m_Mode( SCPU_OPT_DEFAULT ), m_ExcludeZPStack( true ),
	  m_DeliveryEnabled( true ), m_RAMUnderIOAccessible( false ),
	  m_RangeLo( 0x0000 ), m_RangeHi( 0xFFFF ),
	  m_Head( 0 ), m_Count( 0 ), m_QueueGeneration( 0 ),
	  m_NoProgressValid( false ), m_NoProgressDeferHot( false ),
	  m_NoProgressQueueGeneration( 0 ), m_NoProgressHotGeneration( 0 ),
	  m_NoProgressScreenBase( 0xFFFFFFFF ),
	  m_NoProgressBitmapBase( 0xFFFFFFFF )
{
	m_DisplayScrubSampled[ 0 ] = m_DisplayScrubSampled[ 1 ] = 0;
	m_DisplayScrubMismatches[ 0 ] = m_DisplayScrubMismatches[ 1 ] = 0;
	clearDirty();
	setOptMode( SCPU_OPT_DEFAULT );
}

void CWriteBuffer::attach( IC64Bus *bus, const u8 *ram )
{
	m_Bus = bus;
	m_RAM = ram;
	for ( u32 i = 0; i < 0x10000 / 64; i++ ) m_Synced[ i ] = 0;
	m_DisplayScrubScreenBase = m_DisplayScrubGraphicsBase = 0xFFFFFFFF;
	m_DisplayScrubGraphicsLength = 0;
	m_DisplayScrubScreenOffset = m_DisplayScrubGraphicsOffset = 0;
	m_DisplayScrubSpriteOffset = 0;
	m_DisplayScrubTurn = 0;
	m_QueueGeneration++;
	m_NoProgressValid = false;
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
	m_DisplayScrubBytes = m_DisplayScrubMatrixBytes = 0;
	m_DisplayScrubBitmapBytes = m_DisplayScrubCharsetBytes = 0;
	m_DisplayScrubSpriteBytes = 0;
	m_DisplayScrubSampled[ 0 ] = m_DisplayScrubSampled[ 1 ] = 0;
	m_DisplayScrubMismatches[ 0 ] = m_DisplayScrubMismatches[ 1 ] = 0;
	m_BytesFlushed = m_Flushes = 0;
	m_PolicyEntriesExamined = m_PolicyZeroProgressScans = 0;
	m_PolicyNoProgressCacheHits = 0;
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

	// Never mirror $D000-$DFFF until the physical bus has explicitly proved the
	// real-SuperCPU mapping used to reach RAM underneath I/O. On a conventional
	// $01=$37 host this window is chip registers -- VIC-II, SID, CIAs and colour
	// RAM -- and mirroring a shadow-RAM write would be actively destructive.
	//
	// Mirroring it is actively destructive. Our emulated $01 can decide the
	// window is RAM (CHAREN clear, or LORAM and HIRAM both clear), in which
	// case CC64Memory::write8() routes the write to shadow RAM and queues it
	// here. But the halted 6510 leaves the *real* machine's $01 at $37, so I/O
	// is always banked in over there and the mirrored byte lands on the actual
	// register. A stray write to $D011 clears DEN and corrupts the raster
	// compare; one to $DD00 moves the VIC's 16K bank. Either produces a blanked
	// or rolling picture -- which is precisely the fault this was hunting.
	if ( !m_RAMUnderIOAccessible && addr >= 0xD000 && addr <= 0xDFFF )
	{
		m_IOWindowSuppressed++;
		return false;
	}

	return ( addr >= m_RangeLo && addr <= m_RangeHi );
}

bool CWriteBuffer::onRamWrite( u16 addr, u8 value )
{
	// VIDEO_MODE 1 owns presentation from Pi shadow RAM. Keep only the real
	// card's optimisation decision for posted-write timing; do not relocate,
	// eliminate, dirty or enqueue a byte that can never be delivered. A real
	// SuperCPU cannot eliminate same-value writes here, so every policy-accepted
	// write consumes the slot.
	if ( !m_DeliveryEnabled )
	{
		if ( shouldMirror( addr ) )
		{
			m_WritesAccepted++;
			return true;
		}
		m_WritesSkipped++;
		return false;
	}

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
		m_QueueGeneration++;
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
	return flushUpToPolicy( maxBytes, false, 0xFFFFFFFF );
}

static inline bool addressInRange( u16 addr, u32 base, u32 len )
{
	return base < 0x10000 && len != 0
	    && (u32)addr >= base && (u32)addr - base < len;
}

bool CWriteBuffer::hasPendingInRange( u32 base, u32 len ) const
{
	if ( base >= 0x10000 || len == 0 )
		return false;
	u32 end = base + len;
	if ( end > 0x10000 ) end = 0x10000;

	const u32 first = base >> 6;
	const u32 last = ( end - 1 ) >> 6;
	for ( u32 w = first; w <= last; w++ )
	{
		u64 bits = m_Dirty[ w ];
		if ( w == first ) bits &= ~0ULL << ( base & 63 );
		if ( w == last && ( end & 63 ) )
			bits &= ( 1ULL << ( end & 63 ) ) - 1;
		if ( bits ) return true;
	}
	return false;
}

u32 CWriteBuffer::flushSelectedChunk( u32 maxBytes, bool deferHot,
	                                  u32 displayBase, u32 displayLen,
	                                  bool displayOnly,
	                                  u32 displayBase2, u32 displayLen2 )
{
	if ( m_Count == 0 || maxBytes == 0 || !m_Bus )
		return 0;
	if ( maxBytes > SCPU_WRITEBUF_CHUNK ) maxBytes = SCPU_WRITEBUF_CHUNK;

	// Examine each entry that existed on entry at most once. A deferred entry
	// rotates from the head to the tail; selected entries are consumed. This is
	// stable filtering without an O(queue-size) compaction after every 64-byte
	// installment: deferred entries retain their order, as do delivered ones.
	const u32 original = m_Count;
	const u32 mask = SCPU_WRITEBUF_CAPACITY - 1;
	u32 examined = 0;
	u32 selected = 0;
	while ( examined < original && selected < maxBytes && m_Count != 0 )
	{
		const u16 a = m_List[ m_Head ];
		const bool inDisplay = addressInRange( a, displayBase, displayLen )
		                    || addressInRange( a, displayBase2, displayLen2 );
		bool hot = false;
		if ( deferHot && m_HotBlocks )
		{
			const u32 blk = (u32)a >> 6;
			hot = ( ( m_HotBlocks[ blk >> 6 ] >> ( blk & 63 ) ) & 1 ) != 0;
		}
		const bool take = displayOnly ? inDisplay : ( !inDisplay && !hot );
		examined++;

		if ( !take )
		{
			// The slot one past the logical tail is free (or is the head when
			// the ring is full). Copy before advancing so a full ring rotates
			// correctly too.
			m_List[ ( m_Head + m_Count ) & mask ] = a;
			m_Head = ( m_Head + 1 ) & mask;
			continue;
		}

		m_Burst[ selected ].addr = a;
		m_Burst[ selected ].value = deliverValue( a, m_PendingValue[ a ] );
		selected++;
		m_Head = ( m_Head + 1 ) & mask;
		m_Count--;
	}
	m_PolicyEntriesExamined += examined;

	if ( selected == 0 )
		return 0;

	m_Bus->writeBurst( m_Burst, selected );
	for ( u32 i = 0; i < selected; i++ )
	{
		const u16 a = m_Burst[ i ].addr;
		m_Dirty[ a >> 6 ] &= ~( 1ULL << ( a & 63 ) );
		m_Synced[ a >> 6 ] |= 1ULL << ( a & 63 );
	}
	m_QueueGeneration++;
	m_NoProgressValid = false;
	return selected;
}

u32 CWriteBuffer::flushUpToPolicy( u32 maxBytes, bool deferHot,
	                               u32 screenBase, u32 bitmapBase )
{
	if ( m_Count == 0 || maxBytes == 0 )
		return m_Count;
	if ( noProgressCacheMatches( deferHot, screenBase, bitmapBase ) )
	{
		m_PolicyNoProgressCacheHits++;
		return m_Count;
	}

	if ( !m_Bus )
	{
		// No bus attached: drop the queue rather than grow forever.
		clearDirty();
		m_Count = 0;
		m_Head = 0;
		m_QueueGeneration++;
		m_NoProgressValid = false;
		return 0;
	}

	u32 sent = 0;
	while ( m_Count > 0 && sent < maxBytes )
	{
		const u32 n = flushSelectedChunk( maxBytes - sent, deferHot,
		                                  screenBase, 1000, false,
		                                  bitmapBase, 8000 );
		if ( n == 0 )
		{
			m_PolicyZeroProgressScans++;
			rememberNoProgress( deferHot, screenBase, bitmapBase );
			break;
		}
		sent += n;
	}

	m_BytesFlushed += sent;
	if ( sent ) m_Flushes++;

	return m_Count;
}

u32 CWriteBuffer::flushRangeUpTo( u32 base, u32 len, u32 maxBytes )
{
	if ( m_Count == 0 || maxBytes == 0 || !hasPendingInRange( base, len ) )
		return m_Count;

	u32 sent = 0;
	while ( m_Count > 0 && sent < maxBytes && hasPendingInRange( base, len ) )
	{
		const u32 n = flushSelectedChunk( maxBytes - sent, false,
		                                  base, len, true );
		if ( n == 0 ) break;
		sent += n;
	}
	m_BytesFlushed += sent;
	if ( sent ) m_Flushes++;

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

		// A stolen block heals from its under-I/O SOURCE, not from the
		// shadow underneath it -- the shadow there is program data whose
		// DRAM copy is deliberately dead.
		const u8 v = authoritativeShadowValue( a );
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

u32 CWriteBuffer::resyncDisplayed( u32 screenBase, u32 graphicsBase,
	                                u32 graphicsLength, u32 maxBytes,
	                                bool masked, bool repair, bool sample )
{
	if ( !m_Bus || !m_RAM || maxBytes == 0 )
		return 0;
	if ( maxBytes > SCPU_WRITEBUF_CHUNK )
		maxBytes = SCPU_WRITEBUF_CHUNK;
	if ( graphicsBase >= 0x10000 ) graphicsLength = 0;
	if ( graphicsLength != 2048 && graphicsLength != 8192 ) graphicsLength = 0;

	if ( screenBase != m_DisplayScrubScreenBase )
	{
		m_DisplayScrubScreenBase = screenBase;
		m_DisplayScrubScreenOffset = 0;
	}
	if ( graphicsBase != m_DisplayScrubGraphicsBase
	     || graphicsLength != m_DisplayScrubGraphicsLength )
	{
		m_DisplayScrubGraphicsBase = graphicsBase;
		m_DisplayScrubGraphicsLength = graphicsLength;
		m_DisplayScrubGraphicsOffset = 0;
	}

	const bool haveScreen = screenBase < 0x10000;
	const bool haveGraphics = graphicsLength != 0;
	u16 hotBlocks[ 8 ];
	u32 hotCount = 0;
	if ( m_HotBlocks )
	{
		for ( u32 block = 0; block < 1024 && hotCount < 8; block++ )
			if ( ( m_HotBlocks[ block >> 6 ] >> ( block & 63 ) ) & 1 )
				hotBlocks[ hotCount++ ] = (u16)block;
	}

	const u32 screenSlots = haveScreen ? 1u : 0u;
	const u32 graphicsSlots = haveGraphics ? graphicsLength / 1024u : 0u;
	const u32 spriteSlots = hotCount ? 1u : 0u;
	const u32 totalSlots = screenSlots + graphicsSlots + spriteSlots;
	if ( totalSlots == 0 )
		return 0;
	if ( m_DisplayScrubTurn >= totalSlots ) m_DisplayScrubTurn = 0;

	enum { REGION_MATRIX, REGION_GRAPHICS, REGION_SPRITES };
	u32 region;
	u32 base = 0, length = 0;
	u16 *offset = 0;
	if ( m_DisplayScrubTurn < screenSlots )
	{
		region = REGION_MATRIX;
		base = screenBase;
		length = 1024;
		offset = &m_DisplayScrubScreenOffset;
	}
	else if ( m_DisplayScrubTurn < screenSlots + graphicsSlots )
	{
		region = REGION_GRAPHICS;
		base = graphicsBase;
		length = graphicsLength;
		offset = &m_DisplayScrubGraphicsOffset;
	}
	else
	{
		region = REGION_SPRITES;
		length = hotCount * 64u;
		offset = &m_DisplayScrubSpriteOffset;
		if ( *offset >= length ) *offset = 0;
	}

	u32 candidates = 0;
	u32 written = 0;
	u32 inspected = 0;
	const u32 mode = graphicsLength == 8192 ? 1u : 0u;
	while ( candidates < maxBytes && inspected < length )
	{
		u16 a;
		if ( region == REGION_SPRITES )
		{
			const u32 virtualOffset = *offset;
			a = (u16)( ( (u32)hotBlocks[ virtualOffset >> 6 ] << 6 )
			          + ( virtualOffset & 63 ) );
		}
		else
			a = (u16)( base + *offset );
		*offset = (u16)( ( *offset + 1u ) % length );
		inspected++;

		if ( masked && ( a & 0x000A ) != 0x0008 )
			continue;
		if ( !shouldMirror( a ) || isDirty( a ) )
			continue;
		const u8 expected = deliverValue( a, authoritativeShadowValue( a ) );
		candidates++;
		if ( sample )
		{
			// The region may be physical DRAM underneath the I/O window. Sampling
			// through an ordinary read would select the VIC/SID/CIA overlay and can
			// even clear a CIA ICR. Ask the bus explicitly for RAM instead.
			const u8 actual = m_Bus->readRAM( a );
			m_DisplayScrubSampled[ mode ]++;
			if ( actual != expected ) m_DisplayScrubMismatches[ mode ]++;
		}
		if ( !repair )
			continue;
		// Selection and writeBurst run synchronously with the guest CPU stopped,
		// but sampling above may take many physical cycles. Re-check immediately
		// before staging the write so a future asynchronous producer cannot let a
		// stale scrub value overtake a newer dirty value.
		if ( isDirty( a ) )
			continue;
		m_Burst[ written ].addr = a;
		m_Burst[ written ].value = expected;
		m_Synced[ a >> 6 ] |= 1ULL << ( a & 63 );
		written++;
	}

	m_DisplayScrubTurn = (u8)( ( m_DisplayScrubTurn + 1u ) % totalSlots );

	if ( written )
	{
		m_Bus->writeBurst( m_Burst, written );
		m_DisplayScrubBytes += written;
		if ( region == REGION_MATRIX ) m_DisplayScrubMatrixBytes += written;
		else if ( region == REGION_SPRITES ) m_DisplayScrubSpriteBytes += written;
		else if ( graphicsLength == 8192 ) m_DisplayScrubBitmapBytes += written;
		else m_DisplayScrubCharsetBytes += written;
	}
	return written;
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
	m_QueueGeneration++;
	m_NoProgressValid = false;
}
