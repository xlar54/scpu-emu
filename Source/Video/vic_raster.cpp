/*
   SCPU-EMU - completed-frame selection and VIC-II raster-band planning
*/
#include "vic_raster.h"

#include <string.h>

static inline const VICRegWrite &rasterEvent( const VICRegWrite *log, u32 index )
{
	return log[ index & ( SCPU_VIC_LOG_SIZE - 1 ) ];
}

s32 CVICRasterTimeline::lineOfCycle( u32 cycle, u64 anchor,
	                                 const VICRasterTiming &timing )
{
	if ( !timing.cyclesPerLine || !timing.rasterLines || !( anchor >> 63 ) )
		return -1;
	const u32 anchorCycle = (u32)( ( anchor >> 16 ) & 0xFFFFFFFFu );
	const s32 anchorLine = (s32)( anchor & 0x1FF );
	const s32 delta = (s32)( cycle - anchorCycle );
	s32 lines = delta / (s32)timing.cyclesPerLine;
	// C++ truncates negative division toward zero. Raster mapping needs floor
	// division or an event one cycle before an anchor lands one line too low.
	if ( delta < 0 && delta % (s32)timing.cyclesPerLine ) lines--;
	s32 line = anchorLine + lines;
	line %= (s32)timing.rasterLines;
	if ( line < 0 ) line += (s32)timing.rasterLines;
	return line;
}

s32 CVICRasterTimeline::lineOfEvent( const VICRegWrite &event, u64 anchor,
	                                 const VICRasterTiming &timing )
{
	if ( vicLogHasRasterLine( event ) )
		return timing.rasterLines
		     ? (s32)( vicLogRasterLine( event ) % timing.rasterLines ) : -1;
	return lineOfCycle( event.cycle, anchor, timing );
}

u32 CVICRasterTimeline::rowOfEvent( const VICRegWrite &event, u64 anchor,
	                                const VICRasterTiming &timing )
{
	s32 line = lineOfEvent( event, anchor, timing );
	if ( line < 0 ) return timing.frameRows;

	// A sprite pointer written during line N cannot affect line N. The VIC
	// fetched it in the p-access at the END of line N-1, before the write
	// happened, so the change first appears on line N+1. Registers are not like
	// this -- they are sampled as the beam passes -- which is why only the
	// pointers are shifted. Measured against VICE: without this, every pointer
	// change lands exactly one row early.
	//
	// A write in the last few cycles of a line would miss the next line's fetch
	// too and take effect a line later still. That is sub-line precision this
	// model does not carry, and handlers write pointers early in the line.
	const u16 reg = vicLogRegister( event );
	if ( reg >= VIC_LOG_REG_SPRPTR && reg < VIC_LOG_REG_SPRPTR + 8 ) line++;

	s32 row = line - timing.topRaster;
	if ( row < 0 ) row += (s32)timing.rasterLines;
	if ( row < 0 ) return 0;
	if ( row > (s32)timing.frameRows ) return timing.frameRows;
	return (u32)row;
}

VICRasterFrameResult CVICRasterTimeline::build( const VICRegWrite *log,
	                                             u32 head, u32 now, u64 anchor,
	                                             const VICRasterTiming &timing,
	                                             VICRasterFrame &frame )
{
	memset( &frame, 0, sizeof frame );
	frame.result = VIC_RASTER_SNAPSHOT;
	frame.tailAfter = head;
	if ( !log || !timing.cyclesPerLine || !timing.rasterLines
	     || !timing.frameRows )
	{
		prime( head );
		return frame.result;
	}

	const u32 tail = m_Tail;
	if ( !m_Primed || head - tail > SCPU_VIC_LOG_SIZE )
	{
		prime( head );
		frame.bandCount = 1;
		frame.bands[ 0 ] = { 0, (u16)timing.frameRows, head, head };
		return frame.result;
	}

	// No state event does not mean no picture change: screen/bitmap RAM writes
	// are snapshotted separately. Re-render one static band from persistent
	// register/colour state.
	if ( head == tail )
	{
		frame.result = VIC_RASTER_REPLAY;
		frame.drawStart = frame.drawEnd = tail;
		frame.tailAfter = tail;
		frame.bandCount = 1;
		frame.bands[ 0 ] = { 0, (u16)timing.frameRows, tail, tail };
		frame.finishBegin = frame.finishEnd = tail;
		return frame.result;
	}

	u32 frameStart = tail;
	u32 previousStart = tail;
	s32 previousLine = -1;
	u32 previousCycle = 0;
	const u32 frameCycles = timing.cyclesPerLine * timing.rasterLines;
	for ( u32 i = tail; i != head; i++ )
	{
		const VICRegWrite &event = rasterEvent( log, i );
		const s32 line = lineOfEvent( event, anchor, timing );
		if ( line < 0 )
		{
			prime( head );
			frame.bandCount = 1;
			frame.bands[ 0 ] = { 0, (u16)timing.frameRows, head, head };
			return frame.result;
		}
		if ( i != tail
		     && ( line < previousLine
		          || (u32)( event.cycle - previousCycle )
		             >= frameCycles - timing.cyclesPerLine ) )
		{
			previousStart = frameStart;
			frameStart = i;
		}
		previousLine = line;
		previousCycle = event.cycle;
	}

	const u32 newestCycle = rasterEvent( log, head - 1 ).cycle;
	const bool tailStale = (u32)( now - newestCycle )
	                     > ( frameCycles * 3 ) / 2;
	u32 drawStart;
	u32 drawEnd;
	if ( tailStale )
	{
		drawStart = frameStart;
		drawEnd = head;
	}
	else if ( frameStart != tail )
	{
		drawStart = previousStart;
		drawEnd = frameStart;
	}
	else
	{
		frame.result = VIC_RASTER_HOLD;
		frame.tailAfter = tail;
		return frame.result;
	}

	frame.result = VIC_RASTER_REPLAY;
	frame.drawStart = drawStart;
	frame.drawEnd = drawEnd;
	frame.tailAfter = drawEnd;

	u32 bandRow = 0;
	u32 applyBegin = tail;
	for ( u32 i = drawStart; i != drawEnd; i++ )
	{
		const u32 row = rowOfEvent( rasterEvent( log, i ), anchor, timing );
		if ( row > bandRow )
		{
			VICRasterBand &band = frame.bands[ frame.bandCount++ ];
			band.firstRow = (u16)bandRow;
			band.rowCount = (u16)( row - bandRow );
			band.applyBegin = applyBegin;
			band.applyEnd = i;
			applyBegin = i;
			bandRow = row;
		}
	}
	if ( bandRow < timing.frameRows )
	{
		VICRasterBand &band = frame.bands[ frame.bandCount++ ];
		band.firstRow = (u16)bandRow;
		band.rowCount = (u16)( timing.frameRows - bandRow );
		band.applyBegin = applyBegin;
		band.applyEnd = drawEnd;
		applyBegin = drawEnd;
	}
	frame.finishBegin = applyBegin;
	frame.finishEnd = drawEnd;
	m_Tail = drawEnd;
	return frame.result;
}

bool CVICRasterTimeline::yScrollVariesInDisplay(
	const VICRegWrite *log, const VICRasterFrame &frame, u64 anchor,
	const VICRasterTiming &timing, u8 initialD011 )
{
	if ( !log || frame.result != VIC_RASTER_REPLAY ) return false;
	u8 yScroll = (u8)( initialD011 & 7 );
	const u32 displayEnd = timing.displayFirstRow + timing.displayRows;
	for ( u32 i = frame.drawStart; i != frame.drawEnd; i++ )
	{
		const VICRegWrite &event = rasterEvent( log, i );
		if ( vicLogRegister( event ) != 0x11 ) continue;
		const u8 next = (u8)( event.value & 7 );
		const u32 row = rowOfEvent( event, anchor, timing );
		if ( row >= timing.displayFirstRow && row < displayEnd
		     && next != yScroll )
			return true;
		yScroll = next;
	}
	return false;
}

// --- persistent replay and bounded source-page planning ------------------

CVICRasterReplay::CVICRasterReplay()
	: m_Bank( 0 ), m_SpritePtrValid( false ), m_Generation( 0 ),
	  m_Primed( false ), m_Resyncs( 0 ),
	  m_LostEventResyncs( 0 ), m_ResetResyncs( 0 ),
	  m_SnapshotFallbacks( 0 )
{
	memset( m_VIC, 0, sizeof m_VIC );
	memset( m_SpritePtr, 0, sizeof m_SpritePtr );
}

void CVICRasterReplay::invalidate()
{
	m_Primed = false;
	m_Timeline.invalidate();
}

u32 CVICRasterReplay::spritePtrAddr( const u8 vic[ 0x40 ], u8 bank )
{
	return ( (u32)( bank & 3 ) << 14 )
	     + ( (u32)( vic[ 0x18 ] >> 4 ) << 10 ) + 0x3F8;
}

void CVICRasterReplay::rePrime( u32 head, u32 generation,
	                             const u8 liveVIC[ 0x40 ], u8 liveBank,
	                             const u8 *liveRAM )
{
	if ( liveVIC ) memcpy( m_VIC, liveVIC, sizeof m_VIC );
	else           memset( m_VIC, 0, sizeof m_VIC );
	m_Bank = liveBank & 3;

	// Seed the pointers from the live matrix. Timestamped writes only carry
	// CHANGES, so without a starting value the first frame after a resync would
	// show whatever was left in the array.
	if ( liveRAM )
	{
		const u32 base = spritePtrAddr( m_VIC, m_Bank );
		for ( u32 n = 0; n < 8; n++ )
			m_SpritePtr[ n ] = liveRAM[ (u16)( base + n ) ];
		m_SpritePtrValid = true;
	}
	else
	{
		memset( m_SpritePtr, 0, sizeof m_SpritePtr );
		m_SpritePtrValid = false;
	}

	m_Generation = generation;
	m_Primed = true;
	m_Timeline.prime( head );
	m_Resyncs++;
}

void CVICRasterReplay::applyEvent( u8 vic[ 0x40 ], u8 &bank, u8 spritePtr[ 8 ],
	                                bool &spritePtrValid,
	                                const VICRegWrite &event )
{
	const u16 reg = vicLogRegister( event );
	if ( reg < 0x40 )
	{
		// Moving the screen matrix moves the pointer bytes with it, so what we
		// have been tracking no longer describes the address the VIC will read.
		// Give up rather than guess: the snapshot is then used, which is the
		// behaviour that existed before pointers were timestamped at all.
		if ( reg == 0x18 && ( ( vic[ 0x18 ] ^ event.value ) & 0xF0 ) )
			spritePtrValid = false;
		vic[ reg ] = event.value;
	}
	else if ( reg == 0x40 )
	{
		if ( ( bank ^ ( event.value & 3 ) ) != 0 ) spritePtrValid = false;
		bank = event.value & 3;
	}
	else if ( reg >= VIC_LOG_REG_SPRPTR && reg < VIC_LOG_REG_SPRPTR + 8 )
		spritePtr[ reg - VIC_LOG_REG_SPRPTR ] = event.value;
}

static void rasterSelectPage( VICRasterReplayPlan &plan, u32 page )
{
	page &= 0xFF;
	u8 &slot = plan.pageMask[ page >> 3 ];
	const u8 bit = (u8)( 1u << ( page & 7 ) );
	if ( !( slot & bit ) )
	{
		slot |= bit;
		plan.pageCount++;
	}
}

static void rasterSelectRange( VICRasterReplayPlan &plan, u32 address,
	                            u32 count )
{
	if ( !count ) return;
	const u32 first = ( address & 0xFFFF ) >> 8;
	const u32 last = ( ( address + count - 1 ) & 0xFFFF ) >> 8;
	if ( last >= first )
		for ( u32 page = first; page <= last; page++ )
			rasterSelectPage( plan, page );
	else
	{
		for ( u32 page = first; page < 256; page++ )
			rasterSelectPage( plan, page );
		for ( u32 page = 0; page <= last; page++ )
			rasterSelectPage( plan, page );
	}
}

bool CVICRasterReplay::collectPages( VICRasterReplayPlan &plan,
	                                  const u8 *liveRAM )
{
	memset( plan.pageMask, 0, sizeof plan.pageMask );
	plan.pageCount = 0;
	for ( u32 b = 0; b < plan.bandCount; b++ )
	{
		const VICRasterReplayBand &band = plan.bands[ b ];
		const u8 d011 = band.vic[ 0x11 ];
		if ( !( d011 & 0x10 ) ) continue;

		const u32 bank = (u32)( band.bank & 3 ) << 14;
		const u8 d018 = band.vic[ 0x18 ];
		const u32 screen = bank + ( (u32)( d018 >> 4 ) << 10 );
		rasterSelectRange( plan, screen, 1024 );

		const bool ecm = ( d011 & 0x40 ) != 0;
		const bool bmm = ( d011 & 0x20 ) != 0;
		const bool mcm = ( band.vic[ 0x16 ] & 0x10 ) != 0;
		const bool invalid = ecm && ( bmm || mcm );
		if ( !invalid && bmm )
		{
			const u32 bitmap = bank + ( ( d018 & 0x08 ) ? 0x2000 : 0 );
			rasterSelectRange( plan, bitmap, 8192 );
		}
		else if ( !invalid )
		{
			const u32 charset = (u32)( d018 & 0x0E ) << 10;
			const bool charROM = ( ( band.bank & 1 ) == 0 )
			                  && charset >= 0x1000 && charset < 0x2000;
			if ( !charROM ) rasterSelectRange( plan, bank + charset, 2048 );
		}

		// Sprite pointers live at the tail of this band's screen matrix. The
		// once-per-frame RAM snapshot deliberately uses their final-frame values;
		// mid-frame pointer-memory rewrites share the documented RAM fidelity limit.
		if ( liveRAM )
		{
			const u8 enabled = band.vic[ 0x15 ];
			for ( u32 sprite = 0; sprite < 8; sprite++ )
			{
				if ( !( enabled & ( 1u << sprite ) ) ) continue;
				const u8 pointer = liveRAM[ (u16)( screen + 0x3F8 + sprite ) ];
				rasterSelectRange( plan, bank + (u32)pointer * 64, 64 );
			}
		}
	}
	return plan.pageCount <= VIC_RASTER_MAX_SNAPSHOT_PAGES;
}

void CVICRasterReplay::makeStaticPlan( VICRasterFrameResult result,
	                                    const VICRasterTiming &timing,
	                                    const u8 liveVIC[ 0x40 ], u8 liveBank,
	                                    const u8 *liveRAM, bool fallback,
	                                    VICRasterReplayPlan &plan )
{
	memset( &plan, 0, sizeof plan );
	plan.result = result;
	plan.fallback = fallback;
	plan.bandCount = 1;
	plan.bands[ 0 ].firstRow = 0;
	plan.bands[ 0 ].rowCount = (u16)timing.frameRows;
	if ( liveVIC ) memcpy( plan.bands[ 0 ].vic, liveVIC, 0x40 );
	else           memset( plan.bands[ 0 ].vic, 0, 0x40 );
	plan.bands[ 0 ].bank = liveBank & 3;
	collectPages( plan, liveRAM );
}

VICRasterFrameResult CVICRasterReplay::build(
	const VICRegWrite *log, u32 head, u32 now, u64 anchor, u32 generation,
	const VICRasterTiming &timing, const u8 liveVIC[ 0x40 ], u8 liveBank,
	const u8 *liveRAM, VICRasterReplayPlan &plan )
{
	memset( &plan, 0, sizeof plan );
	plan.result = VIC_RASTER_HOLD;

	const bool generationChanged = m_Primed && generation != m_Generation;
	const u32 oldTail = m_Timeline.tail();
	const bool lost = m_Primed && !generationChanged
	               && head - oldTail > SCPU_VIC_LOG_SIZE;
	if ( !m_Primed || generationChanged || lost )
	{
		if ( generationChanged ) m_ResetResyncs++;
		if ( lost ) m_LostEventResyncs++;
		rePrime( head, generation, liveVIC, liveBank, liveRAM );
		makeStaticPlan( VIC_RASTER_SNAPSHOT, timing, liveVIC, liveBank,
		                liveRAM, false, plan );
		return plan.result;
	}

	VICRasterFrame frame;
	const VICRasterFrameResult result = m_Timeline.build(
		log, head, now, anchor, timing, frame );
	if ( result == VIC_RASTER_SNAPSHOT )
	{
		rePrime( head, generation, liveVIC, liveBank, liveRAM );
		makeStaticPlan( result, timing, liveVIC, liveBank,
		                liveRAM, false, plan );
		return plan.result;
	}
	if ( result == VIC_RASTER_HOLD )
		return plan.result;

	// Establish the D011 value at the completed frame's start before asking
	// whether visible writes vary it. Events before drawStart belong to older
	// completed frames but still advance the persistent state.
	u8 startVIC[ 0x40 ];
	memcpy( startVIC, m_VIC, sizeof startVIC );
	u8 startBank = m_Bank;
	u8 startPtr[ 8 ];
	memcpy( startPtr, m_SpritePtr, sizeof startPtr );
	bool startPtrValid = m_SpritePtrValid;
	for ( u32 i = oldTail; i != frame.drawStart; i++ )
		applyEvent( startVIC, startBank, startPtr, startPtrValid,
		            rasterEvent( log, i ) );
	plan.yScrollVaries = CVICRasterTimeline::yScrollVariesInDisplay(
		log, frame, anchor, timing, startVIC[ 0x11 ] );

	plan.result = VIC_RASTER_REPLAY;
	plan.bandCount = frame.bandCount;
	for ( u32 b = 0; b < frame.bandCount; b++ )
	{
		const VICRasterBand &source = frame.bands[ b ];
		for ( u32 i = source.applyBegin; i != source.applyEnd; i++ )
			applyEvent( m_VIC, m_Bank, m_SpritePtr, m_SpritePtrValid,
			            rasterEvent( log, i ) );
		VICRasterReplayBand &band = plan.bands[ b ];
		band.firstRow = source.firstRow;
		band.rowCount = source.rowCount;
		memcpy( band.vic, m_VIC, sizeof band.vic );
		band.bank = m_Bank;
		memcpy( band.spritePtr, m_SpritePtr, sizeof band.spritePtr );
		band.spritePtrValid = m_SpritePtrValid;
	}
	for ( u32 i = frame.finishBegin; i != frame.finishEnd; i++ )
		applyEvent( m_VIC, m_Bank, m_SpritePtr, m_SpritePtrValid,
		            rasterEvent( log, i ) );

	// The matrix may have moved during the frame. Re-seeding at the end means
	// the NEXT frame starts from truth instead of inheriting the doubt.
	if ( !m_SpritePtrValid && liveRAM )
	{
		const u32 base = spritePtrAddr( m_VIC, m_Bank );
		for ( u32 n = 0; n < 8; n++ )
			m_SpritePtr[ n ] = liveRAM[ (u16)( base + n ) ];
		m_SpritePtrValid = true;
	}

	if ( !collectPages( plan, liveRAM ) )
	{
		m_SnapshotFallbacks++;
		// A pathological multi-bank/multi-mode frame must degrade atomically:
		// one clean static frame, never a partial dependency snapshot.
		makeStaticPlan( result, timing, liveVIC, liveBank,
		                liveRAM, true, plan );
	}
	return plan.result;
}
