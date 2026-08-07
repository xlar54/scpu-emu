/*
   SCPU-EMU - write buffer / VIC mirroring tests.

   Bus bandwidth is the scarce resource in this design, so these tests assert on
   the exact number of bus cycles a workload costs, not just on correctness.
*/
#include "../test_framework.h"
#include "../../Source/SuperCPU/write_buffer.h"
#include "../../Source/Bus/Host/host_bus.h"

struct WBFixture
{
	CHostBus     bus;
	CWriteBuffer wb;
	u8           ram[ 0x10000 ];

	WBFixture()
	{
		std::memset( ram, 0, sizeof( ram ) );
		wb.attach( &bus, ram );
		bus.resetStats();
	}

	void poke( u16 addr, u8 v ) { ram[ addr ] = v; wb.onRamWrite( addr, v ); }
};

TEST( writebuf_flush_pushes_current_values )
{
	WBFixture f;
	f.wb.setOptMode( SCPU_OPT_NONE );

	f.poke( 0x0400, 0x41 );
	f.poke( 0x0401, 0x42 );

	CHECK_EQ( f.wb.pending(), 2 );
	CHECK_EQ( f.bus.m_Cycles, 0 );		// nothing on the bus yet

	f.wb.flush();

	CHECK_EQ( f.bus.m_Memory[ 0x0400 ], 0x41 );
	CHECK_EQ( f.bus.m_Memory[ 0x0401 ], 0x42 );
	CHECK_EQ( f.bus.m_Cycles, 2 );
	CHECK_EQ( f.wb.pending(), 0 );
}

TEST( writebuf_coalesces_repeated_writes_to_one_address )
{
	WBFixture f;
	f.wb.setOptMode( SCPU_OPT_NONE );

	// A loop hammering one address -- the pathological case for a naive FIFO.
	for ( int i = 0; i < 1000; i++ )
		f.poke( 0x0400, (u8)i );

	CHECK_EQ( f.wb.pending(), 1 );
	f.wb.flush();

	CHECK_EQ( f.bus.m_Cycles, 1 );					// 1000 writes, one bus cycle
	CHECK_EQ( f.bus.m_Memory[ 0x0400 ], (u8)999 );	// and the final value wins
	CHECK_EQ( f.wb.m_WritesCoalesced, 999 );
}

TEST( writebuf_ordinary_coalescing_keeps_its_fifo_position )
{
	WBFixture f;
	f.wb.setOptMode( SCPU_OPT_NONE );
	f.bus.m_Memory[ 0x0400 ] = 0xA0;
	f.bus.m_Memory[ 0x0401 ] = 0xA1;

	f.poke( 0x0400, 0x10 );
	f.poke( 0x0401, 0x20 );
	f.poke( 0x0400, 0x30 );	// ordinary data coalesces in place

	CHECK_EQ( f.wb.pending(), 2 );
	CHECK_EQ( f.wb.m_WritesCoalesced, 1 );
	f.wb.flushUpTo( 1 );

	CHECK_EQ( f.bus.m_Memory[ 0x0400 ], 0x30 );
	CHECK_EQ( f.bus.m_Memory[ 0x0401 ], 0xA1 );
	CHECK_EQ( f.wb.pending(), 1 );
}

TEST( writebuf_rewritten_sprite_pointer_moves_behind_queued_sprite_data )
{
	WBFixture f;
	f.wb.setOptMode( SCPU_OPT_NONE );
	static const u16 pointer = 0x07F8;	// final eight bytes of screen $0400
	static const u16 sprite  = 0x2000;

	// The pointer was dirtied before this update began and therefore starts at
	// the queue head. Its physical value represents the sprite currently shown.
	f.bus.m_Memory[ pointer ] = 0x44;
	f.poke( pointer, 0x80 );

	for ( u32 i = 0; i < 63; i++ )
		f.poke( (u16)( sprite + i ), (u8)( 0x40 + i ) );

	// Coalescing the commit byte in place would put $81 ahead of all 63 data
	// bytes. It must retain one dirty entry but move that entry to the tail.
	f.poke( pointer, 0x81 );
	CHECK_EQ( f.wb.pending(), 64 );
	CHECK_EQ( f.wb.m_WritesCoalesced, 1 );

	f.wb.flushUpTo( 63 );
	for ( u32 i = 0; i < 63; i++ )
		CHECK_EQ( f.bus.m_Memory[ sprite + i ], (u8)( 0x40 + i ) );
	CHECK_EQ( f.bus.m_Memory[ pointer ], 0x44 );	// commit has not landed yet
	CHECK_EQ( f.wb.pending(), 1 );

	f.wb.flushUpTo( 1 );
	CHECK_EQ( f.bus.m_Memory[ pointer ], 0x81 );
	CHECK_EQ( f.wb.pending(), 0 );
}

TEST( writebuf_screen_clear_costs_one_cycle_per_byte )
{
	WBFixture f;
	f.wb.setOptMode( SCPU_OPT_NONE );

	for ( u16 a = 0x0400; a < 0x07E8; a++ )
		f.poke( a, 0x20 );

	f.wb.flush();
	CHECK_EQ( f.bus.m_Cycles, 1000 );
}

TEST( writebuf_default_mode_skips_zero_page_and_stack )
{
	WBFixture f;
	f.wb.setOptMode( SCPU_OPT_DEFAULT );

	CHECK( !f.wb.shouldMirror( 0x0000 ) );
	CHECK( !f.wb.shouldMirror( 0x00FF ) );
	CHECK( !f.wb.shouldMirror( 0x01FF ) );	// stack
	CHECK(  f.wb.shouldMirror( 0x0200 ) );
	CHECK(  f.wb.shouldMirror( 0x0400 ) );

	f.poke( 0x00FE, 0xAA );		// heavy zero-page traffic, never mirrored
	f.poke( 0x0400, 0xBB );
	f.wb.flush();

	CHECK_EQ( f.bus.m_Cycles, 1 );
	CHECK_EQ( f.wb.m_WritesSkipped, 1 );
}

TEST( writebuf_basic_mode_mirrors_only_the_screen )
{
	WBFixture f;
	f.wb.setOptMode( SCPU_OPT_BASIC );

	CHECK(  f.wb.shouldMirror( 0x0400 ) );
	CHECK(  f.wb.shouldMirror( 0x07FF ) );
	CHECK( !f.wb.shouldMirror( 0x0800 ) );	// BASIC program text
	CHECK( !f.wb.shouldMirror( 0x2000 ) );	// a bitmap the VIC is not pointed at
}

TEST( writebuf_vic_bank_modes_cover_the_right_quarter )
{
	WBFixture f;

	f.wb.setOptMode( SCPU_OPT_VICBANK0 );
	CHECK(  f.wb.shouldMirror( 0x3FFF ) );
	CHECK( !f.wb.shouldMirror( 0x4000 ) );

	f.wb.setOptMode( SCPU_OPT_VICBANK1 );
	CHECK( !f.wb.shouldMirror( 0x3FFF ) );
	CHECK(  f.wb.shouldMirror( 0x4000 ) );
	CHECK(  f.wb.shouldMirror( 0x7FFF ) );

	// Bank 2 is the GEOS setting.
	f.wb.setOptMode( SCPU_OPT_VICBANK2 );
	CHECK(  f.wb.shouldMirror( 0x8000 ) );
	CHECK(  f.wb.shouldMirror( 0xBFFF ) );
	CHECK( !f.wb.shouldMirror( 0xC000 ) );

	f.wb.setOptMode( SCPU_OPT_VICBANK3 );
	CHECK(  f.wb.shouldMirror( 0xC000 ) );
	CHECK(  f.wb.shouldMirror( 0xFFFF ) );
}

TEST( writebuf_full_optimization_mirrors_nothing )
{
	WBFixture f;
	f.wb.setOptMode( SCPU_OPT_FULL );

	for ( u16 a = 0x0400; a < 0x0800; a++ )
		f.poke( a, 0x20 );

	f.wb.flush();
	CHECK_EQ( f.bus.m_Cycles, 0 );
	CHECK_EQ( f.wb.pending(), 0 );
}

TEST( writebuf_none_mode_includes_zero_page )
{
	WBFixture f;
	f.wb.setOptMode( SCPU_OPT_NONE );

	CHECK( f.wb.shouldMirror( 0x0000 ) );
	CHECK( f.wb.shouldMirror( 0x01FF ) );
}

TEST( writebuf_mode_change_defers_old_policy_writes_without_changing_their_value )
{
	WBFixture f;
	f.wb.setOptMode( SCPU_OPT_NONE );

	f.poke( 0x8000, 0x11 );
	CHECK_EQ( f.wb.pending(), 1 );

	// A policy change must not emit an arbitrary-raster burst. The old-policy
	// write stays queued for the scheduler instead.
	f.wb.setOptMode( SCPU_OPT_BASIC );
	CHECK_EQ( f.wb.pending(), 1 );
	CHECK_EQ( f.bus.m_Cycles, 0 );

	// This later write is excluded by BASIC mode. It must not replace the value
	// accepted while NONE was active merely because both share shadow RAM.
	f.poke( 0x8000, 0x22 );
	f.wb.flush();
	CHECK_EQ( f.bus.m_Memory[ 0x8000 ], 0x11 );
}

TEST( writebuf_discard_drops_pending_data_without_bus_traffic )
{
	WBFixture f;
	f.wb.setOptMode( SCPU_OPT_NONE );
	f.poke( 0x0400, 0x41 );
	f.poke( 0x0401, 0x42 );

	f.wb.discard();

	CHECK_EQ( f.wb.pending(), 0 );
	CHECK_EQ( f.bus.m_Cycles, 0 );
}

TEST( writebuf_holds_the_whole_address_space_without_auto_flushing )
{
	// The buffer used to auto-flush every 4096 distinct writes. The KERNAL's
	// RAM test alone touches ~38000 addresses, so that fired repeatedly at
	// arbitrary raster positions -- precisely the unscheduled bulk traffic that
	// corrupts VIC-II fetches. The dirty set now spans all 64K and nothing is
	// sent until someone asks.
	WBFixture f;
	f.wb.setOptMode( SCPU_OPT_NONE );

	for ( u32 a = 0x0200; a <= 0xFFFF; a++ )
		f.poke( (u16)a, (u8)a );

	// Everything from $0200 up, less the 4096 bytes of $D000-$DFFF, which are
	// chip registers on the real machine and are never mirrored.
	CHECK_EQ( f.wb.pending(), ( 0xFFFF - 0x0200 + 1 ) - 0x1000 );
	CHECK_EQ( f.bus.m_Cycles, 0 );		// nothing went out on its own
	CHECK_EQ( f.wb.m_Flushes, 0 );
}

TEST( writebuf_flush_up_to_bounds_the_transfer )
{
	// The raster-scheduled path only has the border and vertical blank to work
	// in, so it must be able to send part of the queue and stop.
	WBFixture f;
	f.wb.setOptMode( SCPU_OPT_NONE );

	for ( u32 i = 0; i < 3000; i++ )
		f.poke( (u16)( 0x0400 + i ), (u8)i );

	CHECK_EQ( f.wb.pending(), 3000 );

	u32 left = f.wb.flushUpTo( 1000 );
	CHECK_EQ( left, 2000 );
	CHECK_EQ( f.bus.m_Cycles, 1000 );

	left = f.wb.flushUpTo( 1000 );
	CHECK_EQ( left, 1000 );
	CHECK_EQ( f.bus.m_Cycles, 2000 );

	// Asking for more than remains drains it and reports empty.
	left = f.wb.flushUpTo( 99999 );
	CHECK_EQ( left, 0 );
	CHECK( f.wb.empty() );
	CHECK_EQ( f.bus.m_Cycles, 3000 );

	// Every byte arrived, and with the right value.
	for ( u32 i = 0; i < 3000; i++ )
		CHECK_EQ( f.bus.m_Memory[ 0x0400 + i ], (u8)i );
}

TEST( writebuf_flush_up_to_zero_sends_nothing )
{
	WBFixture f;
	f.wb.setOptMode( SCPU_OPT_NONE );
	f.poke( 0x0400, 0x41 );

	CHECK_EQ( f.wb.flushUpTo( 0 ), 1 );
	CHECK_EQ( f.bus.m_Cycles, 0 );
	CHECK_EQ( f.wb.pending(), 1 );
}

TEST( writebuf_none_then_default_restores_zp_exclusion )
{
	// Regression: NONE used to latch m_ExcludeZPStack = false, so switching
	// back to DEFAULT kept mirroring page 0 and 1 -- which is precisely what
	// DEFAULT is defined not to do.
	WBFixture f;

	f.wb.setOptMode( SCPU_OPT_DEFAULT );
	CHECK( !f.wb.shouldMirror( 0x00FE ) );

	f.wb.setOptMode( SCPU_OPT_NONE );
	CHECK( f.wb.shouldMirror( 0x00FE ) );

	f.wb.setOptMode( SCPU_OPT_DEFAULT );
	CHECK( !f.wb.shouldMirror( 0x00FE ) );		// must be excluded again
	CHECK( !f.wb.shouldMirror( 0x01FF ) );
	CHECK(  f.wb.shouldMirror( 0x0400 ) );
}

TEST( writebuf_never_mirrors_the_io_window )
{
	// $D000-$DFFF is VIC-II, SID, CIAs and colour RAM on the real machine, not
	// DRAM. Our emulated $01 may map it as RAM (CHAREN clear, or LORAM and
	// HIRAM both clear) and route writes here -- but the halted 6510 leaves the
	// real machine's $01 at $37, so a mirrored byte would land on the actual
	// register. A stray $D011 clears DEN; a stray $DD00 moves the VIC bank.
	WBFixture f;
	f.wb.setOptMode( SCPU_OPT_NONE );		// the most permissive policy

	CHECK( !f.wb.shouldMirror( 0xD000 ) );
	CHECK( !f.wb.shouldMirror( 0xD011 ) );	// display enable / raster compare
	CHECK( !f.wb.shouldMirror( 0xD018 ) );	// screen + charset pointers
	CHECK( !f.wb.shouldMirror( 0xD400 ) );	// SID
	CHECK( !f.wb.shouldMirror( 0xD800 ) );	// colour RAM
	CHECK( !f.wb.shouldMirror( 0xDD00 ) );	// CIA2 port A - the VIC bank select
	CHECK( !f.wb.shouldMirror( 0xDFFF ) );

	// Either side of the window is still mirrored.
	CHECK( f.wb.shouldMirror( 0xCFFF ) );
	CHECK( f.wb.shouldMirror( 0xE000 ) );

	// And nothing gets through even if written.
	f.poke( 0xD011, 0x00 );
	f.poke( 0xDD00, 0x00 );
	f.wb.flush();
	CHECK_EQ( f.bus.m_Cycles, 0 );
	CHECK( f.wb.m_IOWindowSuppressed >= 2 );
}

TEST( writebuf_partial_flush_cannot_starve_old_entries )
{
	// The regression that showed as a half-frozen screen on real hardware. A
	// scrolling PRINT loop re-dirties the whole screen faster than the border
	// windows drain it; with the queue flushed newest-first, the OLDEST entries
	// never went out at all, and their screen cells stayed at whatever the C64
	// held minutes earlier. Oldest-first delivery is the property under test:
	// every dirty byte must reach the machine in bounded time.
	WBFixture f;
	f.wb.setOptMode( SCPU_OPT_NONE );

	// Dirty $0400-$04FF once, then keep re-dirtying $0500-$05FF while draining
	// in small budgets -- the re-dirtied range plays the scrolling program.
	for ( u32 a = 0x0400; a < 0x0500; a++ ) f.poke( (u16)a, 0x11 );

	for ( int round = 0; round < 32; round++ )
	{
		for ( u32 a = 0x0500; a < 0x0600; a++ ) f.poke( (u16)a, (u8)round );
		f.wb.flushUpTo( 64 );	// far less than the dirty set per round
	}

	// The original block was queued first, so it must have drained long ago.
	for ( u32 a = 0x0400; a < 0x0500; a++ )
		CHECK_EQ( f.bus.m_Memory[ a ], 0x11 );
}

TEST( writebuf_same_value_writes_are_eliminated_only_once_synced )
{
	// Programs re-render unconditionally: 3D Pool rewrites ~44K bytes per
	// frame on a completely static screen, and echoing the identical bytes
	// onto the bus mid-display was pure collision exposure for sprite DMA.
	// A write whose value real DRAM already holds queues nothing -- but ONLY
	// once this buffer has delivered that address, because after attach or
	// discard the shadow and real DRAM diverge wholesale and an eliminated
	// write would leave stale DRAM on screen forever.
	CHostBus bus;
	u8 ram[ 0x10000 ] = { 0 };
	CWriteBuffer wb;
	wb.attach( &bus, ram );
	wb.setOptMode( SCPU_OPT_NONE );

	// Not yet delivered: even a same-value write must queue.
	CHECK_EQ( ram[ 0x2000 ], 0 );
	wb.onRamWrite( 0x2000, 0 );			// value == shadow, but unsynced
	ram[ 0x2000 ] = 0;
	CHECK_EQ( wb.pending(), 1u );
	wb.flush();
	CHECK_EQ( wb.pending(), 0u );

	// Delivered once: an identical rewrite now queues nothing...
	wb.onRamWrite( 0x2000, 0 );
	CHECK_EQ( wb.pending(), 0u );
	CHECK_EQ( wb.m_WritesEliminated, 1u );

	// ...while a changed value still queues normally.
	wb.onRamWrite( 0x2000, 7 );
	ram[ 0x2000 ] = 7;
	CHECK_EQ( wb.pending(), 1u );

	// Pending address: rewriting the OLD value must still coalesce into the
	// queue (DRAM will not match shadow until the flush).
	wb.onRamWrite( 0x2000, 7 );
	CHECK_EQ( wb.pending(), 1u );
	wb.flush();

	// discard() (reset) forgets sync: elimination is off again until the
	// address is delivered under the new program.
	wb.discard();
	wb.onRamWrite( 0x2000, 7 );			// same value as shadow AND real DRAM
	ram[ 0x2000 ] = 7;
	CHECK_EQ( wb.pending(), 1u );
	wb.flush();

	// A policy-skipped write diverges shadow from DRAM, so it must disarm
	// elimination for that address. 3D Pool: shapes staged at $0200-$03FF
	// under a restrictive optimisation mode, then rewritten with the same
	// values under a permissive one -- without the disarm those rewrites were
	// eliminated and the balls stayed garbage in real DRAM forever.
	wb.setOptMode( SCPU_OPT_VICBANK1 );	// $4000-$7FFF only: $2000 excluded
	wb.onRamWrite( 0x2000, 9 );			// skipped; shadow moves on without DRAM
	ram[ 0x2000 ] = 9;
	CHECK_EQ( wb.pending(), 0u );
	wb.setOptMode( SCPU_OPT_NONE );		// back to everything
	wb.onRamWrite( 0x2000, 9 );			// same value as shadow -- MUST queue
	CHECK_EQ( wb.pending(), 1u );
	wb.flush();
	CHECK_EQ( bus.m_Memory[ 0x2000 ], 9 );
}

TEST( writebuf_resync_sweep_converges_dram_to_shadow )
{
	// Real DRAM is write-only from the Pi, so shadow==DRAM can only be
	// re-established, never verified. The sweep walks the mirrored range
	// delivering clean shadow bytes; divergence from any source -- boot-era
	// policy skips, optimisation-mode switches, a glitched burst -- heals
	// within a bounded number of calls instead of persisting forever.
	CHostBus bus;
	u8 ram[ 0x10000 ];
	for ( u32 i = 0; i < 0x10000; i++ ) ram[ i ] = (u8)( i * 7 );
	CWriteBuffer wb;
	wb.attach( &bus, ram );
	wb.setOptMode( SCPU_OPT_NONE );		// full range, no exclusions

	// DRAM starts divergent everywhere. Sweep in chunks until one full lap.
	for ( u32 i = 0; i < 0x10000 / 512; i++ )
		wb.resyncSweep( 512 );

	u32 diffs = 0;
	for ( u32 a = 0; a < 0x10000; a++ )
	{
		if ( ( a & 0xF000 ) == 0xD000 ) continue;	// I/O window never swept
		if ( ram[ a ] != bus.m_Memory[ a ] ) diffs++;
	}
	CHECK_EQ( diffs, 0u );

	// A dirty address is owned by its pending value: the sweep must not
	// deliver the shadow byte underneath it.
	bus.resetStats();
	wb.onRamWrite( 0x8000, 0x11 );
	ram[ 0x8000 ] = 0x11;
	const u8 before = bus.m_Memory[ 0x8000 ];
	for ( u32 i = 0; i < 0x10000 / 512; i++ )
		wb.resyncSweep( 512 );
	CHECK_EQ( bus.m_Memory[ 0x8000 ], before );	// untouched by the sweep
	wb.flush();
	CHECK_EQ( bus.m_Memory[ 0x8000 ], 0x11 );	// delivered by the queue
}

TEST( writebuf_hot_shape_blocks_wait_for_the_border )
{
	// A re-rendered sprite shape must reach real DRAM whole. The render
	// passes through a cleared/partial transient, and at queue latency that
	// transient would sit in DRAM for milliseconds -- the VIC fetches shapes
	// on the sprite's own display lines and shows it as a torn, flickering
	// sprite. So bytes in ACTIVE shape blocks are delivered only by border
	// drains (deferHot=false). It must skip the shape and continue with cold
	// traffic, or a hot byte at the head would starve the entire mirror queue.
	CHostBus bus;
	u8 ram[ 0x10000 ] = { 0 };
	u64 hot[ 1024 / 64 ] = { 0 };
	CWriteBuffer wb;
	wb.attach( &bus, ram );
	wb.setOptMode( SCPU_OPT_NONE );
	wb.attachHotShapeBlocks( hot );

	// Block 64 ($1000-$103F) is an active shape block.
	hot[ 64 >> 6 ] |= 1ULL << ( 64 & 63 );

	// Queue: two cold bytes, one hot byte, one more cold byte -- FIFO order.
	wb.onRamWrite( 0x2000, 0x11 ); ram[ 0x2000 ] = 0x11;
	wb.onRamWrite( 0x2001, 0x22 ); ram[ 0x2001 ] = 0x22;
	wb.onRamWrite( 0x1010, 0x33 ); ram[ 0x1010 ] = 0x33;	// hot
	wb.onRamWrite( 0x3000, 0x44 ); ram[ 0x3000 ] = 0x44;
	CHECK_EQ( wb.pending(), 4u );

	// Display drain: skips the hot byte but still delivers both cold regions.
	wb.flushUpToPolicy( 64, true );
	CHECK_EQ( bus.m_Memory[ 0x2000 ], 0x11 );
	CHECK_EQ( bus.m_Memory[ 0x2001 ], 0x22 );
	CHECK( bus.m_Memory[ 0x1010 ] != 0x33 );
	CHECK_EQ( bus.m_Memory[ 0x3000 ], 0x44 );
	CHECK_EQ( wb.pending(), 1u );

	// Border drain delivers the deferred hot block.
	wb.flushUpToPolicy( 64, false );
	CHECK_EQ( bus.m_Memory[ 0x1010 ], 0x33 );
	CHECK_EQ( wb.pending(), 0u );
}

TEST( writebuf_visible_drain_skips_screen_and_keeps_making_progress )
{
	WBFixture f;
	f.wb.setOptMode( SCPU_OPT_NONE );

	// The whole active matrix can remain dirty in a scroller. A matrix byte at
	// the FIFO head must not prevent later character/sprite/general RAM traffic
	// from using the display-time ration.
	for ( u32 i = 0; i < 1000; i++ )
		f.poke( (u16)( 0x7400 + i ), (u8)( i ^ 0x5A ) );
	f.poke( 0x2000, 0x11 );
	f.poke( 0x2001, 0x22 );

	f.wb.flushUpToPolicy( 64, false, 0x7400 );
	CHECK_EQ( f.bus.m_Memory[ 0x2000 ], 0x11 );
	CHECK_EQ( f.bus.m_Memory[ 0x2001 ], 0x22 );
	CHECK( f.bus.m_Memory[ 0x7400 ] != (u8)( 0 ^ 0x5A ) );
	CHECK_EQ( f.wb.pending(), 1000u );
	CHECK( f.wb.hasPendingInRange( 0x7400, 1000 ) );
}

TEST( writebuf_hidden_drain_prioritizes_screen_and_keeps_latest_values )
{
	WBFixture f;
	f.wb.setOptMode( SCPU_OPT_NONE );

	// General traffic is older and therefore ahead in the FIFO. The hidden
	// screen path must select the matrix anyway, in stable address order, and a
	// coalesced rewrite must still deliver its newest value.
	f.poke( 0x2000, 0x11 );
	f.poke( 0x2001, 0x22 );
	for ( u32 i = 0; i < 80; i++ )
		f.poke( (u16)( 0x7400 + i ), (u8)i );
	f.poke( 0x7400, 0xA5 );

	f.wb.flushRangeUpTo( 0x7400, 1000, 64 );
	CHECK_EQ( f.bus.m_Memory[ 0x7400 ], 0xA5 );
	for ( u32 i = 1; i < 64; i++ )
		CHECK_EQ( f.bus.m_Memory[ 0x7400 + i ], (u8)i );
	CHECK( f.bus.m_Memory[ 0x2000 ] != 0x11 );
	CHECK_EQ( f.wb.pending(), 18u );

	// Finish the matrix, then ordinary FIFO order resumes for the cold bytes.
	f.wb.flushRangeUpTo( 0x7400, 1000, 64 );
	CHECK( !f.wb.hasPendingInRange( 0x7400, 1000 ) );
	CHECK_EQ( f.wb.pending(), 2u );
	f.wb.flush();
	CHECK_EQ( f.bus.m_Memory[ 0x2000 ], 0x11 );
	CHECK_EQ( f.bus.m_Memory[ 0x2001 ], 0x22 );
}

TEST( relocation_shape_tail_is_not_mistaken_for_a_pointer_row )
{
	CWriteBuffer wb;
	u8 ptrReloc[ 64 ];
	u8 inUse[ 48 ];
	for ( u32 i = 0; i < 64; i++ ) ptrReloc[ i ] = 0xFF;
	for ( u32 i = 0; i < 48; i++ ) inUse[ i ] = 0xFF;
	u8 count = 1;
	ptrReloc[ 1 ] = 7;
	inUse[ 15 ] = 0x40;	// block 15 is $C3C0-$C3FF
	wb.attachRelocation( ptrReloc, inUse, &count );

	CHECK_EQ( wb.deliverValue( 0xC3F8, 0x41 ), 0x41 );	// shape byte: identity
	CHECK_EQ( wb.deliverValue( 0xC7F8, 0x41 ), 0x07 );	// real pointer row
}

// ---------------------------------------------------------------------------
// Under-I/O sprite-shape relocation
//
// In VIC bank 3 a sprite pointer of $40-$7F selects DRAM at $D000-$DFFF --
// RAM to the VIC, chip registers to our bus writes, so the mirror can never
// put the shape there. 3D Pool /SCPU double-buffers across banks 1 and 3
// with its ball shapes under I/O; the fix delivers TRANSLATED pointers into
// relocated copies at $C000-$CBFF. These tests wire CC64Memory to the buffer
// exactly as CSuperCPU::init does and replay the game's configuration:
// bank 3 via $DD00, screen $CC00 / bitmap $E000 via $D018=$38, shapes staged
// under $01=$34 banking.
// ---------------------------------------------------------------------------

struct RelocFixture
{
	CHostBus     bus;
	CC64Memory   mem;
	CWriteBuffer wb;

	RelocFixture()
	{
		mem.attachBus( &bus );
		mem.setMirrorSink( &wb );
		wb.attach( &bus, mem.m_RAM );
		wb.setOptMode( SCPU_OPT_NONE );
		wb.attachRelocation( mem.m_PtrReloc, mem.m_RelocInUse,
		                     &mem.m_RelocCount );
		mem.reset();
		bus.resetStats();

		// The 3D Pool configuration: VIC bank 3, bitmap $E000, screen $CC00,
		// so the pointer row is $CFF8.
		mem.write8( 0xDD00, 0xC4 );
		mem.write8( 0xD011, 0x3B );
		mem.write8( 0xD018, 0x38 );
	}

	// Bank I/O out, store into RAM under it, bank I/O back in -- the game's
	// own staging pattern ($4521: SEI / LDA #$34 / STA $01).
	void pokeUnderIO( u16 addr, u8 v )
	{
		mem.write8( 0x0001, 0x34 );
		mem.writeFast( addr, v );
		mem.write8( 0x0001, 0x35 );
	}
};

TEST( reloc_pointer_into_under_io_block_delivers_translated_copy )
{
	RelocFixture f;

	// Shapes staged first, pointer after -- the common order. Block $43 is
	// $D0C0-$D0FF, the snapshot's actual cue-ball block.
	for ( u32 i = 0; i < 63; i++ )
		f.pokeUnderIO( (u16)( 0xD0C0 + i ), (u8)( 0x80 + i ) );

	// Sentinel: block 0 is also the value zero, which a zeroed bus array
	// would fake. Prove the delivery actually happened.
	f.bus.m_Memory[ 0xCFF8 ] = 0xEE;
	f.mem.writeFast( 0xCFF8, 0x43 );

	// The immediate pointer delivery carried the RELOCATED block -- the first
	// free one, $C000/64 = 0 -- not the raw $43 the shadow holds.
	CHECK_EQ( f.bus.m_Memory[ 0xCFF8 ], 0x00 );
	CHECK_EQ( f.mem.m_RAM[ 0xCFF8 ], 0x43 );		// shadow stays faithful
	CHECK_EQ( f.mem.m_RelocAllocs, 1u );
	CHECK_EQ( f.mem.m_PtrReloc[ 0x43 - 0x40 ], 0x00 );
	CHECK_EQ( f.mem.m_RelocInUse[ 0 ], 0x43 );

	// The hot-shape tracking follows the relocated block: bank 3, block 0.
	const u32 blk = ( 3u << 8 ) + 0;
	CHECK( ( f.mem.m_HotShapeBlocks[ blk >> 6 ] >> ( blk & 63 ) ) & 1 );
	const u32 raw = ( 3u << 8 ) + 0x43;
	CHECK( !( ( f.mem.m_HotShapeBlocks[ raw >> 6 ] >> ( raw & 63 ) ) & 1 ) );

	// Flushing delivers the shape bytes at the relocated address, and the
	// QUEUED copy of the pointer byte goes out translated too.
	f.wb.flush();
	for ( u32 i = 0; i < 63; i++ )
		CHECK_EQ( f.bus.m_Memory[ 0xC000 + i ], (u8)( 0x80 + i ) );
	CHECK_EQ( f.bus.m_Memory[ 0xCFF8 ], 0x00 );
	// Real DRAM under I/O was never touched ($D0C0 on the host bus).
	CHECK_EQ( f.bus.m_Memory[ 0xD0C0 ], 0x00 );
}

TEST( reloc_shapes_written_after_the_pointer_forward_per_write )
{
	RelocFixture f;

	// Pointer first: allocation replays the (still empty) block, then each
	// later shape write is forwarded as it happens.
	f.bus.m_Memory[ 0xCFF8 ] = 0xEE;
	f.mem.writeFast( 0xCFF8, 0x52 );			// block $52 = $D480
	CHECK_EQ( f.bus.m_Memory[ 0xCFF8 ], 0x00 );	// relocated to block 0

	f.pokeUnderIO( 0xD481, 0xAB );
	f.wb.flush();
	CHECK_EQ( f.bus.m_Memory[ 0xC001 ], 0xAB );
	CHECK( f.wb.m_RelocForwarded >= 1u );
}

TEST( reloc_sweep_heals_stolen_blocks_from_the_under_io_source )
{
	RelocFixture f;

	for ( u32 i = 0; i < 63; i++ )
		f.pokeUnderIO( (u16)( 0xD0C0 + i ), (u8)( 0x80 + i ) );
	f.mem.writeFast( 0xCFF8, 0x43 );
	f.wb.flush();
	CHECK_EQ( f.bus.m_Memory[ 0xC005 ], 0x85 );

	// A glitched burst corrupts the relocated copy in DRAM. The sweep must
	// heal it from the under-I/O SOURCE bytes -- the shadow underneath the
	// stolen block holds program data that must NOT be delivered.
	f.bus.m_Memory[ 0xC005 ] = 0x99;
	f.mem.m_RAM[ 0xC005 ] = 0x77;				// program data in the shadow
	for ( u32 i = 0; i < 0x10000 / 512; i++ )
		f.wb.resyncSweep( 512 );
	CHECK_EQ( f.bus.m_Memory[ 0xC005 ], 0x85 );	// healed, from $D0C5
}

TEST( reloc_program_writes_into_stolen_blocks_are_shielded )
{
	RelocFixture f;

	for ( u32 i = 0; i < 63; i++ )
		f.pokeUnderIO( (u16)( 0xD0C0 + i ), (u8)( 0x80 + i ) );
	f.mem.writeFast( 0xCFF8, 0x43 );
	f.wb.flush();

	// The program updates its own data at $C000 -- an address whose DRAM now
	// belongs to the relocated shape. The write reaches the shadow (programs
	// read it back) but never the bus.
	f.mem.writeFast( 0xC000, 0x77 );
	CHECK_EQ( f.mem.m_RAM[ 0xC000 ], 0x77 );
	CHECK_EQ( f.wb.m_RelocShielded, 1u );
	f.wb.flush();
	CHECK_EQ( f.bus.m_Memory[ 0xC000 ], 0x80 );	// still the shape byte
}

TEST( reloc_bank1_pointers_pass_through_untranslated )
{
	RelocFixture f;
	f.mem.write8( 0xDD00, 0xC6 );				// bank bits 10 -> bank 1
	// Pointer row is now $4FF8; the same value $43 selects plain RAM $50C0
	// and must go out untouched.
	f.mem.writeFast( 0x4FF8, 0x43 );
	CHECK_EQ( f.bus.m_Memory[ 0x4FF8 ], 0x43 );
	CHECK_EQ( f.mem.m_RelocCount, 0 );
}

TEST( reloc_disabled_delivers_raw_pointers )
{
	RelocFixture f;
	f.mem.m_RelocEnable = false;
	f.mem.writeFast( 0xCFF8, 0x43 );
	CHECK_EQ( f.bus.m_Memory[ 0xCFF8 ], 0x43 );
	CHECK_EQ( f.mem.m_RelocAllocs, 0u );
	CHECK_EQ( f.mem.m_RelocCount, 0 );
}

TEST( reloc_survives_the_double_buffer_bank_flip )
{
	// 3D Pool's raster IRQ flips $DD00 between banks 3 and 1 every frame. A
	// queued $CFF8 row byte regularly FLUSHES while bank 1 is active, and the
	// sweep re-delivers the inactive row wholesale. Translation is decided by
	// address shape, not by the currently active row -- otherwise both paths
	// put the raw under-I/O value back and the flicker returns at flip rate.
	RelocFixture f;

	for ( u32 i = 0; i < 63; i++ )
		f.pokeUnderIO( (u16)( 0xD0C0 + i ), (u8)( 0x80 + i ) );
	f.mem.writeFast( 0xCFF8, 0x43 );		// queues raw $43, delivers translated

	// The IRQ flips to bank 1 BEFORE the queue drains -- the raced order.
	f.mem.write8( 0xDD00, 0xC6 );
	f.wb.flush();
	CHECK_EQ( f.bus.m_Memory[ 0xCFF8 ], 0x00 );	// still the relocated block

	// And the sweep, crossing the now-INACTIVE bank-3 row, must not undo it.
	f.bus.m_Memory[ 0xCFF8 ] = 0xEE;
	for ( u32 i = 0; i < 0x10000 / 512; i++ )
		f.wb.resyncSweep( 512 );
	CHECK_EQ( f.bus.m_Memory[ 0xCFF8 ], 0x00 );
}

TEST( reloc_low_bitmap_leaves_no_pool_and_falls_back_to_raw )
{
	RelocFixture f;
	// Bitmap in the LOW half of bank 3 ($D018 bit 3 clear): the 8K bitmap
	// spans $C000-$DF3F, covering the whole relocation window. Allocation
	// must refuse and the pointer goes out raw -- stale, but honest, and
	// counted.
	f.mem.write8( 0xD018, 0x30 );
	f.mem.writeFast( 0xCFF8, 0x43 );
	CHECK_EQ( f.bus.m_Memory[ 0xCFF8 ], 0x43 );
	CHECK_EQ( f.mem.m_RelocExhausted, 1u );
	CHECK_EQ( f.mem.m_RelocCount, 0 );
}
