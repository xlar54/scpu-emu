/*
   SCPU-EMU - VIC-II raster timeline tests
*/
#include "../test_framework.h"
#include "../../Source/Video/vic_raster.h"
#include "../../Source/Video/vic_renderer.h"

#include <string.h>

static VICRasterTiming ntscTiming()
{
	VICRasterTiming timing = { 63, 263, 272, 15, 36, 200 };
	return timing;
}

static VICRegWrite rasterWrite( u32 cycle, u16 reg, u8 value )
{
	VICRegWrite event = { cycle, reg, value, 0 };
	return event;
}

static VICRegWrite exactRasterWrite( u32 cycle, u16 reg, u8 value, u16 line )
{
	VICRegWrite event = { cycle, (u16)( reg | VIC_LOG_LINE_VALID ), value,
	                      (u8)line };
	if ( line & 0x100 ) event.reg |= VIC_LOG_LINE_HIGH;
	return event;
}

static void defaultVIC( u8 vic[ 0x40 ] )
{
	memset( vic, 0, 0x40 );
	vic[ 0x11 ] = 0x1B;
	vic[ 0x16 ] = 0x08;
	vic[ 0x18 ] = 0x18; // screen +$0400, RAM character set +$2000
	vic[ 0x20 ] = 6;
	vic[ 0x21 ] = 0;
}

static bool replayPageSelected( const VICRasterReplayPlan &plan, u32 page )
{
	return ( plan.pageMask[ ( page & 0xFF ) >> 3 ]
	       & ( 1u << ( page & 7 ) ) ) != 0;
}

static void renderReplay( const VICRasterReplayPlan &plan, const u8 *ram,
	                       const u8 *chars, const u8 *colours, u8 *pixels )
{
	memset( pixels, 0xFF, VIC_RENDER_WIDTH * VIC_RENDER_HEIGHT );
	CVICRenderer renderer;
	VICRenderSpriteSequencer sprites = {};
	for ( u32 b = 0; b < plan.bandCount; b++ )
	{
		const VICRasterReplayBand &band = plan.bands[ b ];
		VICRenderState state;
		memset( &state, 0, sizeof state );
		state.ram = ram;
		state.charROM = chars;
		state.colourRAM = colours;
		state.bankBase = (u32)( band.bank & 3 ) << 14;
		state.screenBase = state.bankBase
		                 + ( (u32)( band.vic[ 0x18 ] >> 4 ) << 10 );
		state.bitmapBase = state.bankBase
		                 + ( ( band.vic[ 0x18 ] & 8 ) ? 0x2000 : 0 );
		const u32 charset = (u32)( band.vic[ 0x18 ] & 0x0E ) << 10;
		const bool charROM = ( ( band.bank & 1 ) == 0 )
		                  && charset >= 0x1000 && charset < 0x2000;
		state.charsetBase = charROM ? 0xFFFFFFFF
		                            : state.bankBase + charset;
		state.yScrollVaries = plan.yScrollVaries;
		memcpy( state.vic, band.vic, sizeof state.vic );
		renderer.renderRows( state,
		                     pixels + band.firstRow * VIC_RENDER_WIDTH,
		                     VIC_RENDER_WIDTH, band.firstRow, band.rowCount,
		                     0, &sprites );
	}
}

TEST( vic_raster_clock_mapping_floors_negative_deltas )
{
	const VICRasterTiming timing = ntscTiming();
	const u64 anchor = ( 1ULL << 63 ) | ( (u64)1000 << 16 ) | 100;
	CHECK_EQ( CVICRasterTimeline::lineOfCycle( 1000, anchor, timing ), 100 );
	CHECK_EQ( CVICRasterTimeline::lineOfCycle( 999, anchor, timing ), 99 );
	CHECK_EQ( CVICRasterTimeline::lineOfCycle( 937, anchor, timing ), 99 );
	CHECK_EQ( CVICRasterTimeline::lineOfCycle( 936, anchor, timing ), 98 );
}

TEST( vic_raster_replays_only_a_completed_frame )
{
	const VICRasterTiming timing = ntscTiming();
	VICRegWrite log[ SCPU_VIC_LOG_SIZE ] = {};
	log[ 0 ] = rasterWrite( 60 * 63, 0x20, 2 );
	log[ 1 ] = rasterWrite( 200 * 63, 0x20, 6 );
	log[ 2 ] = rasterWrite( ( 263 + 60 ) * 63, 0x20, 5 );
	log[ 3 ] = rasterWrite( ( 263 + 200 ) * 63, 0x20, 7 );
	const u64 anchor = 1ULL << 63;

	CVICRasterTimeline timeline;
	timeline.prime( 0 );
	VICRasterFrame frame;
	CHECK_EQ( timeline.build( log, 4, ( 263 + 210 ) * 63, anchor,
	                          timing, frame ), VIC_RASTER_REPLAY );
	CHECK_EQ( frame.drawStart, 0u );
	CHECK_EQ( frame.drawEnd, 2u );
	CHECK_EQ( frame.tailAfter, 2u );
	CHECK_EQ( frame.bandCount, 3u );
	CHECK_EQ( frame.bands[ 0 ].firstRow, 0 );
	CHECK_EQ( frame.bands[ 0 ].rowCount, 45 );
	CHECK_EQ( frame.bands[ 1 ].firstRow, 45 );
	CHECK_EQ( frame.bands[ 1 ].rowCount, 140 );
	CHECK_EQ( frame.bands[ 2 ].firstRow, 185 );
	CHECK_EQ( frame.bands[ 2 ].rowCount, 87 );
	CHECK_EQ( frame.bands[ 1 ].applyBegin, 0u );
	CHECK_EQ( frame.bands[ 1 ].applyEnd, 1u );
	CHECK_EQ( frame.bands[ 2 ].applyBegin, 1u );
	CHECK_EQ( frame.bands[ 2 ].applyEnd, 2u );

	// The second frame remains held until a successor begins or it becomes old.
	CHECK_EQ( timeline.build( log, 4, ( 263 + 210 ) * 63, anchor,
	                          timing, frame ), VIC_RASTER_HOLD );
}

TEST( vic_raster_fixed_line_writes_form_frames_from_the_timestamp_gap )
{
	const VICRasterTiming timing = ntscTiming();
	VICRegWrite log[ SCPU_VIC_LOG_SIZE ] = {};
	log[ 0 ] = rasterWrite( 100 * 63, 0x18, 0x14 );
	log[ 1 ] = rasterWrite( ( 263 + 100 ) * 63, 0x18, 0x24 );
	CVICRasterTimeline timeline;
	timeline.prime( 0 );
	VICRasterFrame frame;
	CHECK_EQ( timeline.build( log, 2, ( 263 + 120 ) * 63, 1ULL << 63,
	                          timing, frame ), VIC_RASTER_REPLAY );
	CHECK_EQ( frame.drawStart, 0u );
	CHECK_EQ( frame.drawEnd, 1u );
}

TEST( vic_raster_exact_irq_line_overrides_a_coarse_clock_anchor )
{
	const VICRasterTiming timing = ntscTiming();
	VICRegWrite log[ SCPU_VIC_LOG_SIZE ] = {};
	log[ 0 ] = exactRasterWrite( 10, 0x20, 2, 200 );
	CVICRasterTimeline timeline;
	timeline.prime( 0 );
	VICRasterFrame frame;
	const u64 deliberatelyWrongAnchor = ( 1ULL << 63 ) | ( (u64)10 << 16 ) | 5;
	CHECK_EQ( timeline.build( log, 1, 100000, deliberatelyWrongAnchor,
	                          timing, frame ), VIC_RASTER_REPLAY );
	CHECK_EQ( frame.bandCount, 2u );
	CHECK_EQ( frame.bands[ 1 ].firstRow, 185 );
}

TEST( vic_raster_treats_visible_yscroll_changes_as_fli_not_band_shifts )
{
	const VICRasterTiming timing = ntscTiming();
	VICRegWrite log[ SCPU_VIC_LOG_SIZE ] = {};
	log[ 0 ] = exactRasterWrite( 0, 0x11, 0x1A, 40 );
	log[ 1 ] = exactRasterWrite( 1, 0x11, 0x1B, 80 );
	CVICRasterTimeline timeline;
	timeline.prime( 0 );
	VICRasterFrame frame;
	CHECK_EQ( timeline.build( log, 2, 100000, 1ULL << 63,
	                          timing, frame ), VIC_RASTER_REPLAY );
	CHECK( CVICRasterTimeline::yScrollVariesInDisplay(
	       log, frame, 1ULL << 63, timing, 0x1B ) );

	// A once-per-frame setup write in the top border defines the frame's one
	// scroll value and must not disable smooth scrolling.
	log[ 0 ] = exactRasterWrite( 0, 0x11, 0x1A, 20 );
	timeline.prime( 0 );
	CHECK_EQ( timeline.build( log, 1, 100000, 1ULL << 63,
	                          timing, frame ), VIC_RASTER_REPLAY );
	CHECK( !CVICRasterTimeline::yScrollVariesInDisplay(
	       log, frame, 1ULL << 63, timing, 0x1B ) );
}

TEST( vic_raster_replay_renders_border_splits_at_the_planned_rows )
{
	const VICRasterTiming timing = ntscTiming();
	u8 liveVIC[ 0x40 ];
	defaultVIC( liveVIC );
	u8 ram[ 65536 ] = {};
	u8 chars[ 4096 ] = {};
	u8 colours[ 1024 ];
	memset( colours, 2, sizeof colours );
	u8 pixels[ VIC_RENDER_WIDTH * VIC_RENDER_HEIGHT ];
	VICRegWrite log[ SCPU_VIC_LOG_SIZE ] = {};
	log[ 0 ] = exactRasterWrite( 60 * 63, 0x20, 2, 60 );
	log[ 1 ] = exactRasterWrite( 200 * 63, 0x20, 6, 200 );
	log[ 2 ] = exactRasterWrite( ( 263 + 60 ) * 63, 0x20, 5, 60 );

	CVICRasterReplay replay;
	VICRasterReplayPlan plan;
	CHECK_EQ( replay.build( log, 0, 0, 1ULL << 63, 1, timing,
	                        liveVIC, 0, ram, plan ), VIC_RASTER_SNAPSHOT );
	CHECK_EQ( replay.build( log, 3, ( 263 + 80 ) * 63, 1ULL << 63, 1,
	                        timing, liveVIC, 0, ram, plan ), VIC_RASTER_REPLAY );
	CHECK_EQ( plan.bandCount, 3u );
	renderReplay( plan, ram, chars, colours, pixels );
	CHECK_EQ( pixels[ 44 * VIC_RENDER_WIDTH ], 6 );
	CHECK_EQ( pixels[ 45 * VIC_RENDER_WIDTH ], 2 );
	CHECK_EQ( pixels[ 184 * VIC_RENDER_WIDTH ], 2 );
	CHECK_EQ( pixels[ 185 * VIC_RENDER_WIDTH ], 6 );
}

TEST( vic_raster_replay_switches_vic_banks_and_source_pages_mid_frame )
{
	const VICRasterTiming timing = ntscTiming();
	u8 liveVIC[ 0x40 ];
	defaultVIC( liveVIC );
	u8 ram[ 65536 ] = {};
	u8 chars[ 4096 ] = {};
	u8 colours[ 1024 ];
	u8 pixels[ VIC_RENDER_WIDTH * VIC_RENDER_HEIGHT ];
	memset( colours, 2, sizeof colours );
	for ( u32 row = 0; row < 25; row++ )
	{
		ram[ 0x0400 + row * 40 ] = 1;
		ram[ 0x4400 + row * 40 ] = 1;
	}
	for ( u32 line = 0; line < 8; line++ )
	{
		ram[ 0x2000 + 8 + line ] = 0x80;
		ram[ 0x6000 + 8 + line ] = 0x01;
	}
	VICRegWrite log[ SCPU_VIC_LOG_SIZE ] = {};
	log[ 0 ] = exactRasterWrite( 100 * 63, 0x40, 1, 100 );
	log[ 1 ] = exactRasterWrite( ( 263 + 20 ) * 63, 0x20, 5, 20 );
	CVICRasterReplay replay;
	VICRasterReplayPlan plan;
	replay.build( log, 0, 0, 1ULL << 63, 1, timing,
	              liveVIC, 0, ram, plan );
	CHECK_EQ( replay.build( log, 2, ( 263 + 40 ) * 63, 1ULL << 63, 1,
	                        timing, liveVIC, 1, ram, plan ), VIC_RASTER_REPLAY );
	CHECK_EQ( plan.bands[ 0 ].bank, 0 );
	CHECK_EQ( plan.bands[ 1 ].bank, 1 );
	CHECK( replayPageSelected( plan, 0x04 ) );
	CHECK( replayPageSelected( plan, 0x20 ) );
	CHECK( replayPageSelected( plan, 0x44 ) );
	CHECK( replayPageSelected( plan, 0x60 ) );
	renderReplay( plan, ram, chars, colours, pixels );
	CHECK_EQ( pixels[ 50 * VIC_RENDER_WIDTH + 32 ], 2 );
	CHECK_EQ( pixels[ 100 * VIC_RENDER_WIDTH + 32 ], 0 );
	CHECK_EQ( pixels[ 100 * VIC_RENDER_WIDTH + 39 ], 2 );
}

TEST( vic_raster_replay_changes_sprite_state_inside_a_sprite )
{
	const VICRasterTiming timing = ntscTiming();
	u8 liveVIC[ 0x40 ];
	defaultVIC( liveVIC );
	liveVIC[ 0x00 ] = 32;
	liveVIC[ 0x01 ] = 54;
	liveVIC[ 0x15 ] = 1;
	liveVIC[ 0x27 ] = 2;
	u8 ram[ 65536 ] = {};
	u8 chars[ 4096 ] = {};
	u8 colours[ 1024 ] = {};
	u8 pixels[ VIC_RENDER_WIDTH * VIC_RENDER_HEIGHT ];
	ram[ 0x07F8 ] = 0x20;
	for ( u32 line = 0; line < 21; line++ ) ram[ 0x0800 + line * 3 ] = 0x80;
	VICRegWrite log[ SCPU_VIC_LOG_SIZE ] = {};
	// topRaster 15 makes raster line 65 output row 50.
	log[ 0 ] = exactRasterWrite( 65 * 63, 0x27, 5, 65 );
	log[ 1 ] = exactRasterWrite( ( 263 + 20 ) * 63, 0x20, 6, 20 );
	CVICRasterReplay replay;
	VICRasterReplayPlan plan;
	replay.build( log, 0, 0, 1ULL << 63, 1, timing,
	              liveVIC, 0, ram, plan );
	CHECK_EQ( replay.build( log, 2, ( 263 + 40 ) * 63, 1ULL << 63, 1,
	                        timing, liveVIC, 0, ram, plan ), VIC_RASTER_REPLAY );
	CHECK( replayPageSelected( plan, 0x08 ) );
	renderReplay( plan, ram, chars, colours, pixels );
	CHECK_EQ( pixels[ 45 * VIC_RENDER_WIDTH + 40 ], 2 );
	CHECK_EQ( pixels[ 55 * VIC_RENDER_WIDTH + 40 ], 5 );
}

TEST( vic_raster_replay_latches_sprite_y_and_y_expand_at_trigger )
{
	const VICRasterTiming timing = ntscTiming();
	u8 liveVIC[ 0x40 ];
	defaultVIC( liveVIC );
	liveVIC[ 0x00 ] = 32;
	liveVIC[ 0x01 ] = 54;              // output rows 40..60
	liveVIC[ 0x15 ] = 1;
	liveVIC[ 0x27 ] = 2;
	u8 ram[ 65536 ] = {};
	u8 chars[ 4096 ] = {};
	u8 colours[ 1024 ] = {};
	u8 pixels[ VIC_RENDER_WIDTH * VIC_RENDER_HEIGHT ];
	ram[ 0x07F8 ] = 0x20;
	for ( u32 line = 0; line < 21; line++ )
		ram[ 0x0800 + line * 3 ] = 0x80;
	ram[ 0x0800 + 10 * 3 ] = 0x40;

	VICRegWrite log[ SCPU_VIC_LOG_SIZE ] = {};
	// At output row 50, move the Y trigger to that very row and enable Y
	// expansion. A register-per-band renderer restarts and stretches the
	// sprite here. The VIC's already-active sequencer must instead continue
	// original line 10 and finish at row 60.
	log[ 0 ] = exactRasterWrite( 65 * 63, 0x01, 64, 65 );
	log[ 1 ] = exactRasterWrite( 65 * 63 + 1, 0x17, 1, 65 );
	log[ 2 ] = exactRasterWrite( ( 263 + 20 ) * 63, 0x20, 6, 20 );
	CVICRasterReplay replay;
	VICRasterReplayPlan plan;
	replay.build( log, 0, 0, 1ULL << 63, 1, timing,
	              liveVIC, 0, ram, plan );
	CHECK_EQ( replay.build( log, 3, ( 263 + 40 ) * 63, 1ULL << 63, 1,
	                        timing, liveVIC, 0, ram, plan ), VIC_RASTER_REPLAY );
	renderReplay( plan, ram, chars, colours, pixels );
	CHECK_EQ( pixels[ 40 * VIC_RENDER_WIDTH + 40 ], 2 );
	CHECK_EQ( pixels[ 50 * VIC_RENDER_WIDTH + 40 ], 0 );
	CHECK_EQ( pixels[ 50 * VIC_RENDER_WIDTH + 41 ], 2 );
	CHECK_EQ( pixels[ 60 * VIC_RENDER_WIDTH + 40 ], 2 );
	CHECK_EQ( pixels[ 61 * VIC_RENDER_WIDTH + 40 ], 0 );
}

TEST( vic_raster_replay_detects_fli_and_uses_a_neutral_vertical_origin )
{
	const VICRasterTiming timing = ntscTiming();
	u8 liveVIC[ 0x40 ];
	defaultVIC( liveVIC );
	u8 ram[ 65536 ] = {};
	u8 chars[ 4096 ] = {};
	u8 colours[ 1024 ] = {};
	u8 pixels[ VIC_RENDER_WIDTH * VIC_RENDER_HEIGHT ];
	ram[ 0x0400 ] = 1;
	ram[ 0x2008 ] = 0x80;
	colours[ 0 ] = 2;
	VICRegWrite log[ SCPU_VIC_LOG_SIZE ] = {};
	log[ 0 ] = exactRasterWrite( 40 * 63, 0x11, 0x1A, 40 );
	log[ 1 ] = exactRasterWrite( 80 * 63, 0x11, 0x1B, 80 );
	log[ 2 ] = exactRasterWrite( ( 263 + 20 ) * 63, 0x20, 6, 20 );
	CVICRasterReplay replay;
	VICRasterReplayPlan plan;
	replay.build( log, 0, 0, 1ULL << 63, 1, timing,
	              liveVIC, 0, ram, plan );
	CHECK_EQ( replay.build( log, 3, ( 263 + 40 ) * 63, 1ULL << 63, 1,
	                        timing, liveVIC, 0, ram, plan ), VIC_RASTER_REPLAY );
	CHECK( plan.yScrollVaries );
	renderReplay( plan, ram, chars, colours, pixels );
	CHECK_EQ( pixels[ VIC_RENDER_DISPLAY_Y * VIC_RENDER_WIDTH
	                 + VIC_RENDER_DISPLAY_X ], 2 );
}

TEST( vic_raster_replay_resyncs_on_reset_and_ring_overwrite )
{
	const VICRasterTiming timing = ntscTiming();
	u8 liveVIC[ 0x40 ];
	defaultVIC( liveVIC );
	u8 ram[ 65536 ] = {};
	VICRegWrite log[ SCPU_VIC_LOG_SIZE ] = {};
	CVICRasterReplay replay;
	VICRasterReplayPlan plan;
	replay.build( log, 0, 0, 1ULL << 63, 10, timing,
	              liveVIC, 0, ram, plan );
	CHECK_EQ( replay.build( log, 0, 0, 1ULL << 63, 11, timing,
	                        liveVIC, 0, ram, plan ), VIC_RASTER_SNAPSHOT );
	CHECK_EQ( replay.resetResyncs(), 1u );
	CHECK_EQ( replay.build( log, SCPU_VIC_LOG_SIZE + 1, 0, 1ULL << 63, 11,
	                        timing, liveVIC, 0, ram, plan ), VIC_RASTER_SNAPSHOT );
	CHECK_EQ( replay.lostEventResyncs(), 1u );
	CHECK_EQ( replay.tail(), SCPU_VIC_LOG_SIZE + 1u );
}

TEST( vic_raster_replay_falls_back_atomically_when_page_union_is_too_large )
{
	const VICRasterTiming timing = ntscTiming();
	u8 liveVIC[ 0x40 ];
	defaultVIC( liveVIC );
	liveVIC[ 0x11 ] = 0x3B;
	u8 ram[ 65536 ] = {};
	VICRegWrite log[ SCPU_VIC_LOG_SIZE ] = {};
	log[ 0 ] = exactRasterWrite( 50 * 63, 0x40, 1, 50 );
	log[ 1 ] = exactRasterWrite( 100 * 63, 0x40, 2, 100 );
	log[ 2 ] = exactRasterWrite( 150 * 63, 0x40, 3, 150 );
	log[ 3 ] = exactRasterWrite( ( 263 + 20 ) * 63, 0x40, 0, 20 );
	CVICRasterReplay replay;
	VICRasterReplayPlan plan;
	replay.build( log, 0, 0, 1ULL << 63, 1, timing,
	              liveVIC, 0, ram, plan );
	CHECK_EQ( replay.build( log, 4, ( 263 + 40 ) * 63, 1ULL << 63, 1,
	                        timing, liveVIC, 3, ram, plan ), VIC_RASTER_REPLAY );
	CHECK( plan.fallback );
	CHECK_EQ( plan.bandCount, 1u );
	CHECK( plan.pageCount <= VIC_RASTER_MAX_SNAPSHOT_PAGES );
	CHECK_EQ( replay.snapshotFallbacks(), 1u );
}
