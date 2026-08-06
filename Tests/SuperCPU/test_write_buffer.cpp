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
}
