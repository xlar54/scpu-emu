/*
   SCPU-EMU - REU register file and transfer engine.

   The unit is pure logic with a two-method host interface, so every transfer
   mode, the trigger paths and the register end-states can be checked here with
   no hardware and no C64. What CANNOT be checked here is whether a transfer
   into screen memory actually reaches the physical machine -- that depends on
   the host implementation mirroring, and is called out in reu_wiring.h.
*/
#include "../test_framework.h"
#include "../../Source/REU/reu.h"

#include <cstring>

namespace {

// A plain 64K array standing in for C64 memory. Records access counts so a
// test can prove a fixed-address transfer really did hammer one location.
struct FakeHost : public IREUHost
{
	u8  mem[ 0x10000 ];
	u32 reads;
	u32 writes;

	FakeHost() : reads( 0 ), writes( 0 ) { std::memset( mem, 0, sizeof mem ); }

	u8 reuHostRead( u16 addr ) override { reads++; return mem[ addr ]; }
	void reuHostWrite( u16 addr, u8 value ) override
	{
		writes++;
		mem[ addr ] = value;
	}
};

// Set up a transfer without executing it. Command is written last by the
// caller, because writing it with bit 7 set is what starts things.
void program( CREU &reu, u16 c64, u32 reuAddr, u16 length )
{
	reu.write( 0xDF02, (u8)( c64 & 0xFF ) );
	reu.write( 0xDF03, (u8)( c64 >> 8 ) );
	reu.write( 0xDF04, (u8)( reuAddr & 0xFF ) );
	reu.write( 0xDF05, (u8)( ( reuAddr >> 8 ) & 0xFF ) );
	reu.write( 0xDF06, (u8)( ( reuAddr >> 16 ) & 0x07 ) );
	reu.write( 0xDF07, (u8)( length & 0xFF ) );
	reu.write( 0xDF08, (u8)( length >> 8 ) );
}

// Execute now: type in the low bits, EXECUTE set, $FF00 decode disabled.
u8 runNow( u8 type ) { return (u8)( 0x80 | 0x10 | type ); }

} // namespace

TEST( reu_size_selectors_map_to_the_documented_sizes )
{
	struct { u8 sel; u32 bytes; } cases[] = {
		{ REUSIZE_NONE, 0 },
		{ REUSIZE_128K,   128u * 1024u },
		{ REUSIZE_256K,   256u * 1024u },
		{ REUSIZE_512K,   512u * 1024u },
		{ REUSIZE_2MB,   2048u * 1024u },
		{ REUSIZE_4MB,   4096u * 1024u },
		{ REUSIZE_16MB, 16384u * 1024u },
	};
	for ( u32 i = 0; i < sizeof cases / sizeof cases[ 0 ]; i++ )
	{
		CREU reu;
		CHECK( reu.init( cases[ i ].sel ) );
		CHECK_EQ( reu.sizeBytes(), cases[ i ].bytes );
		CHECK_EQ( reu.present(), cases[ i ].bytes != 0 );
	}
}

TEST( reu_unknown_selector_is_absent_not_rounded )
{
	// A typo must produce no unit, never a different one: a machine that
	// silently has the wrong REU is harder to diagnose than one with none.
	CREU reu;
	CHECK( reu.init( 99 ) );
	CHECK( !reu.present() );
	CHECK_EQ( reu.sizeBytes(), 0u );
}

TEST( reu_absent_unit_declines_every_access )
{
	// Declining rather than answering matters: software probes for an REU by
	// writing and reading back, and a device that answers but never transfers
	// is worse than no device at all.
	CREU reu;
	reu.init( REUSIZE_NONE );
	u8 v = 0x11;
	CHECK( !reu.read( 0xDF00, v ) );
	CHECK( !reu.write( 0xDF01, 0x90 ) );
	CHECK_EQ( v, 0x11 );
}

TEST( reu_ignores_addresses_that_are_not_its_own )
{
	CREU reu;
	reu.init( REUSIZE_512K );
	u8 v = 0;
	CHECK( !reu.read( 0xDEFF, v ) );
	CHECK( !reu.read( 0xDF0B, v ) );
	CHECK( !reu.write( 0xDF7E, 0x00 ) );	// RAMLink strobe, not ours
}

TEST( reu_stash_moves_c64_memory_into_expansion_ram )
{
	CREU reu; FakeHost host;
	reu.init( REUSIZE_512K );
	reu.attachHost( &host );

	for ( u32 i = 0; i < 256; i++ ) host.mem[ 0x1000 + i ] = (u8)( i ^ 0x5A );

	program( reu, 0x1000, 0x020000, 256 );
	reu.write( 0xDF01, runNow( 0 ) );

	for ( u32 i = 0; i < 256; i++ )
		CHECK_EQ( reu.peek( 0x020000 + i ), (u8)( i ^ 0x5A ) );
	CHECK_EQ( reu.bytesMoved(), 256u );
	CHECK_EQ( reu.lastTransferCycles(), 256u );
}

TEST( reu_fetch_moves_expansion_ram_into_c64_memory )
{
	CREU reu; FakeHost host;
	reu.init( REUSIZE_512K );
	reu.attachHost( &host );

	for ( u32 i = 0; i < 256; i++ ) reu.poke( 0x030000 + i, (u8)( i + 3 ) );

	program( reu, 0x2000, 0x030000, 256 );
	reu.write( 0xDF01, runNow( 1 ) );

	for ( u32 i = 0; i < 256; i++ )
		CHECK_EQ( host.mem[ 0x2000 + i ], (u8)( i + 3 ) );
}

TEST( reu_swap_exchanges_both_sides )
{
	CREU reu; FakeHost host;
	reu.init( REUSIZE_512K );
	reu.attachHost( &host );

	for ( u32 i = 0; i < 64; i++ )
	{
		host.mem[ 0x0400 + i ] = (u8)( 0xA0 + i );
		reu.poke( 0x000100 + i, (u8)( 0x10 + i ) );
	}

	program( reu, 0x0400, 0x000100, 64 );
	reu.write( 0xDF01, runNow( 2 ) );

	for ( u32 i = 0; i < 64; i++ )
	{
		CHECK_EQ( host.mem[ 0x0400 + i ], (u8)( 0x10 + i ) );
		CHECK_EQ( reu.peek( 0x000100 + i ), (u8)( 0xA0 + i ) );
	}
}

TEST( reu_verify_reports_a_match_without_fault )
{
	CREU reu; FakeHost host;
	reu.init( REUSIZE_512K );
	reu.attachHost( &host );

	for ( u32 i = 0; i < 128; i++ )
	{
		host.mem[ 0x0800 + i ] = (u8)i;
		reu.poke( 0x001000 + i, (u8)i );
	}

	program( reu, 0x0800, 0x001000, 128 );
	reu.write( 0xDF01, runNow( 3 ) );

	u8 status = 0;
	CHECK( reu.read( 0xDF00, status ) );
	CHECK( ( status & 0x40 ) != 0 );	// end of block
	CHECK( ( status & 0x20 ) == 0 );	// no fault
	CHECK_EQ( reu.verifyFaults(), 0u );
}

TEST( reu_verify_stops_at_the_first_mismatch )
{
	// Stopping early is the behaviour, not an optimisation: software reads the
	// address registers afterwards to report WHERE the comparison failed.
	CREU reu; FakeHost host;
	reu.init( REUSIZE_512K );
	reu.attachHost( &host );

	for ( u32 i = 0; i < 128; i++ )
	{
		host.mem[ 0x0800 + i ] = (u8)i;
		reu.poke( 0x001000 + i, (u8)i );
	}
	reu.poke( 0x001000 + 10, 0xFF );	// break byte 10

	program( reu, 0x0800, 0x001000, 128 );
	reu.write( 0xDF01, runNow( 3 ) );

	u8 status = 0;
	CHECK( reu.read( 0xDF00, status ) );
	CHECK( ( status & 0x20 ) != 0 );	// fault
	CHECK_EQ( reu.verifyFaults(), 1u );
	CHECK_EQ( reu.bytesMoved(), 11u );	// stopped on the offending byte
}

TEST( reu_status_bits_clear_on_read )
{
	// This is the entire acknowledge mechanism -- there is no separate write
	// to dismiss them -- so the side effect is load-bearing.
	CREU reu; FakeHost host;
	reu.init( REUSIZE_512K );
	reu.attachHost( &host );

	program( reu, 0x1000, 0x000000, 16 );
	reu.write( 0xDF01, runNow( 0 ) );

	u8 first = 0, second = 0;
	CHECK( reu.read( 0xDF00, first ) );
	CHECK( ( first & 0x40 ) != 0 );
	CHECK( reu.read( 0xDF00, second ) );
	CHECK( ( second & 0x40 ) == 0 );
}

TEST( reu_size_bit_distinguishes_the_large_units )
{
	CREU small, large;
	small.init( REUSIZE_256K );
	large.init( REUSIZE_512K );

	u8 s = 0, l = 0;
	small.read( 0xDF00, s );
	large.read( 0xDF00, l );
	CHECK( ( s & 0x10 ) == 0 );
	CHECK( ( l & 0x10 ) != 0 );
}

TEST( reu_length_zero_means_a_full_64k )
{
	CREU reu; FakeHost host;
	reu.init( REUSIZE_512K );
	reu.attachHost( &host );

	program( reu, 0x0000, 0x000000, 0 );
	reu.write( 0xDF01, runNow( 0 ) );
	CHECK_EQ( reu.bytesMoved(), 0x10000u );
}

TEST( reu_autoload_restores_the_programmed_registers )
{
	// Autoload is why a repeated block move costs one register write instead
	// of five, so the shadow must hold what the PROGRAM asked for rather than
	// where the last transfer finished.
	CREU reu; FakeHost host;
	reu.init( REUSIZE_512K );
	reu.attachHost( &host );

	program( reu, 0x1000, 0x000200, 64 );
	reu.write( 0xDF01, (u8)( 0x80 | 0x20 | 0x10 | 0 ) );	// autoload + run now

	u8 lo = 0, hi = 0;
	reu.read( 0xDF02, lo ); reu.read( 0xDF03, hi );
	CHECK_EQ( (u16)( lo | ( hi << 8 ) ), (u16)0x1000 );

	u8 rl = 0, rh = 0;
	reu.read( 0xDF04, rl ); reu.read( 0xDF05, rh );
	CHECK_EQ( (u16)( rl | ( rh << 8 ) ), (u16)0x0200 );
}

TEST( reu_without_autoload_the_addresses_advance )
{
	CREU reu; FakeHost host;
	reu.init( REUSIZE_512K );
	reu.attachHost( &host );

	program( reu, 0x1000, 0x000200, 64 );
	reu.write( 0xDF01, runNow( 0 ) );

	u8 lo = 0, hi = 0;
	reu.read( 0xDF02, lo ); reu.read( 0xDF03, hi );
	CHECK_EQ( (u16)( lo | ( hi << 8 ) ), (u16)( 0x1000 + 64 ) );
}

TEST( reu_fixed_address_bits_hold_one_side_still )
{
	CREU reu; FakeHost host;
	reu.init( REUSIZE_512K );
	reu.attachHost( &host );

	for ( u32 i = 0; i < 32; i++ ) reu.poke( 0x000000 + i, (u8)( 0x80 + i ) );

	// Fix the C64 address: every byte lands on the same location, so only the
	// last one survives -- which is how a fill works.
	reu.write( 0xDF0A, 0x80 );
	program( reu, 0x3000, 0x000000, 32 );
	reu.write( 0xDF01, runNow( 1 ) );

	CHECK_EQ( host.mem[ 0x3000 ], (u8)( 0x80 + 31 ) );
	CHECK_EQ( host.mem[ 0x3001 ], (u8)0 );
	CHECK_EQ( host.writes, 32u );	// it really did write 32 times
}

TEST( reu_ff00_trigger_waits_then_fires )
{
	// Programs use this to line a transfer up with a known point in their own
	// instruction stream, so arming must NOT transfer.
	CREU reu; FakeHost host;
	reu.init( REUSIZE_512K );
	reu.attachHost( &host );

	for ( u32 i = 0; i < 16; i++ ) host.mem[ 0x0500 + i ] = (u8)( i + 1 );

	program( reu, 0x0500, 0x000400, 16 );
	reu.write( 0xDF01, (u8)( 0x80 | 0 ) );		// EXECUTE, $FF00 decode ENABLED

	CHECK_EQ( reu.transfers(), 0u );			// armed only
	CHECK_EQ( reu.peek( 0x000400 ), (u8)0 );

	reu.noteFF00Write();

	CHECK_EQ( reu.transfers(), 1u );
	CHECK_EQ( reu.peek( 0x000400 ), (u8)1 );
}

TEST( reu_a_stray_ff00_write_does_nothing_when_nothing_is_armed )
{
	CREU reu; FakeHost host;
	reu.init( REUSIZE_512K );
	reu.attachHost( &host );
	reu.noteFF00Write();
	CHECK_EQ( reu.transfers(), 0u );
}

TEST( reu_expansion_address_aliases_within_the_fitted_size )
{
	// A real unit decodes a fixed number of address lines, so an address past
	// the fitted size wraps onto lower memory. Size-probing software depends
	// on exactly that, which is why this is a mask and not a bounds check.
	CREU reu; FakeHost host;
	reu.init( REUSIZE_128K );
	reu.attachHost( &host );

	reu.poke( 0x000010, 0x77 );
	CHECK_EQ( reu.peek( 0x020010 ), (u8)0x77 );	// 128K above wraps to the same cell
}

TEST( reu_interrupt_needs_both_the_enable_and_the_cause )
{
	CREU reu; FakeHost host;
	reu.init( REUSIZE_512K );
	reu.attachHost( &host );

	// Enable the line but mask the end-of-block cause: no interrupt.
	reu.write( 0xDF09, 0x80 );
	program( reu, 0x1000, 0x000000, 8 );
	reu.write( 0xDF01, runNow( 0 ) );
	u8 status = 0;
	reu.read( 0xDF00, status );
	CHECK( ( status & 0x80 ) == 0 );

	// Enable both: interrupt pending.
	reu.write( 0xDF09, (u8)( 0x80 | 0x40 ) );
	program( reu, 0x1000, 0x000000, 8 );
	reu.write( 0xDF01, runNow( 0 ) );
	reu.read( 0xDF00, status );
	CHECK( ( status & 0x80 ) != 0 );
}

TEST( reu_execute_bit_clears_when_the_transfer_completes )
{
	// A program polling the command register learns the unit is idle this way.
	CREU reu; FakeHost host;
	reu.init( REUSIZE_512K );
	reu.attachHost( &host );

	program( reu, 0x1000, 0x000000, 8 );
	reu.write( 0xDF01, runNow( 0 ) );

	u8 cmd = 0;
	CHECK( reu.read( 0xDF01, cmd ) );
	CHECK( ( cmd & 0x80 ) == 0 );
}

TEST( reu_transfer_without_a_host_completes_rather_than_hanging )
{
	// Nothing to transfer against must still report end-of-block: a program
	// polling for completion would otherwise spin forever.
	CREU reu;
	reu.init( REUSIZE_512K );
	program( reu, 0x1000, 0x000000, 16 );
	reu.write( 0xDF01, runNow( 0 ) );

	u8 status = 0;
	reu.read( 0xDF00, status );
	CHECK( ( status & 0x40 ) != 0 );
}

TEST( reu_reset_clears_registers_but_keeps_expansion_contents )
{
	// The hardware does not clear expansion RAM, and software has kept data
	// across a warm reset.
	CREU reu;
	reu.init( REUSIZE_512K );
	reu.poke( 0x001234, 0xC7 );
	program( reu, 0x4321, 0x001234, 99 );

	reu.reset();

	CHECK_EQ( reu.peek( 0x001234 ), (u8)0xC7 );
	u8 lo = 0, hi = 0;
	reu.read( 0xDF02, lo ); reu.read( 0xDF03, hi );
	CHECK_EQ( (u16)( lo | ( hi << 8 ) ), (u16)0 );
}
