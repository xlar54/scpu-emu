/*
   SCPU-EMU - C64 PLA banking tests.

   The accelerator decides whether an access can be served from local RAM or has
   to reach the machine purely from this table, so a wrong entry means either a
   silent wrong-memory read or a pointless bus cycle.
*/
#include "../test_framework.h"
#include "../../Source/C64/banking.h"

static u8 modeFor( u8 port01, u8 game = 1, u8 exrom = 1 )
{
	C64BankConfig cfg;
	cfg.port01 = port01;
	cfg.game   = game;
	cfg.exrom  = exrom;
	return c64BankMode( &cfg );
}

TEST( banking_default_37_maps_basic_kernal_and_io )
{
	c64BankingInit();
	u8 m = modeFor( 0x37 );	// LORAM=1 HIRAM=1 CHAREN=1 -- the KERNAL default

	CHECK_EQ( c64MapRead( 0x0400, m ), REG_RAM );
	CHECK_EQ( c64MapRead( 0xA000, m ), REG_BASIC );
	CHECK_EQ( c64MapRead( 0xBFFF, m ), REG_BASIC );
	CHECK_EQ( c64MapRead( 0xC000, m ), REG_RAM );
	CHECK_EQ( c64MapRead( 0xD000, m ), REG_IO );
	CHECK_EQ( c64MapRead( 0xD800, m ), REG_IO );
	CHECK_EQ( c64MapRead( 0xE000, m ), REG_KERNAL );
	CHECK_EQ( c64MapRead( 0xFFFC, m ), REG_KERNAL );
}

TEST( banking_charen_low_exposes_character_rom )
{
	c64BankingInit();
	u8 m = modeFor( 0x33 );	// LORAM=1 HIRAM=1 CHAREN=0

	CHECK_EQ( c64MapRead( 0xD000, m ), REG_CHARROM );
	CHECK_EQ( c64MapRead( 0xA000, m ), REG_BASIC );
	CHECK_EQ( c64MapRead( 0xE000, m ), REG_KERNAL );
}

TEST( banking_all_ram_when_loram_and_hiram_low )
{
	c64BankingInit();
	u8 m = modeFor( 0x30 );	// LORAM=0 HIRAM=0

	CHECK_EQ( c64MapRead( 0xA000, m ), REG_RAM );
	CHECK_EQ( c64MapRead( 0xD000, m ), REG_RAM );	// not even I/O
	CHECK_EQ( c64MapRead( 0xE000, m ), REG_RAM );
}

TEST( banking_kernal_without_basic )
{
	c64BankingInit();
	u8 m = modeFor( 0x36 );	// LORAM=0 HIRAM=1 CHAREN=1

	CHECK_EQ( c64MapRead( 0xA000, m ), REG_RAM );
	CHECK_EQ( c64MapRead( 0xE000, m ), REG_KERNAL );
	CHECK_EQ( c64MapRead( 0xD000, m ), REG_IO );
}

TEST( banking_io_visible_with_only_loram )
{
	c64BankingInit();
	u8 m = modeFor( 0x35 );	// LORAM=1 HIRAM=0 CHAREN=1

	CHECK_EQ( c64MapRead( 0xD000, m ), REG_IO );
	CHECK_EQ( c64MapRead( 0xE000, m ), REG_RAM );
	CHECK_EQ( c64MapRead( 0xA000, m ), REG_RAM );
}

TEST( banking_ultimax_replaces_the_map )
{
	c64BankingInit();
	// /GAME low, /EXROM high. This is exactly the configuration that makes a
	// freezer cartridge and a SuperCPU mutually exclusive.
	u8 m = modeFor( 0x37, /*game*/ 0, /*exrom*/ 1 );

	CHECK_EQ( c64MapRead( 0x0000, m ), REG_RAM );
	CHECK_EQ( c64MapRead( 0x1000, m ), REG_OPEN );
	CHECK_EQ( c64MapRead( 0x8000, m ), REG_CART_ROML );
	CHECK_EQ( c64MapRead( 0xA000, m ), REG_OPEN );
	CHECK_EQ( c64MapRead( 0xD000, m ), REG_IO );
	CHECK_EQ( c64MapRead( 0xE000, m ), REG_CART_ROMH );
}

TEST( banking_port_effective_honours_data_direction )
{
	// Lines configured as inputs float high because of the external pull-ups,
	// which is how a C64 still sees $37 after a program writes $00 to $01 with
	// the DDR left at 0.
	CHECK_EQ( c64PortEffective( 0x00, 0x00 ), 0x07 );
	CHECK_EQ( c64PortEffective( 0x2F, 0x37 ), 0x07 );
	CHECK_EQ( c64PortEffective( 0x2F, 0x35 ), 0x05 );
	CHECK_EQ( c64PortEffective( 0x2F, 0x30 ), 0x00 );
}

TEST( banking_writes_only_divert_to_io )
{
	c64BankingInit();
	u8 m = modeFor( 0x37 );

	CHECK( c64WriteIsIO( 0xD020, m ) );
	CHECK( !c64WriteIsIO( 0xA000, m ) );	// falls through to the RAM under BASIC
	CHECK( !c64WriteIsIO( 0xE000, m ) );	// and under the KERNAL
	CHECK( !c64WriteIsIO( 0x0400, m ) );
}

TEST( banking_port_read_returns_the_whole_port )
{
	// $0001 carries the cassette lines as well as the banking bits. Returning
	// only bits 0-2 loses motor and sense state that software does read.
	// Bits 6 and 7 do not exist on a 6510 and read as 0.

	// Everything an output, latch $37: the latch shows through.
	CHECK_EQ( c64PortRead( 0x3F, 0x37 ), 0x37 );

	// The usual C64 configuration: DDR $2F leaves bit 4 (cassette sense) an
	// input, which reads high with no button pressed.
	CHECK_EQ( c64PortRead( 0x2F, 0x37 ), 0x37 );

	// Cassette motor (bit 5) driven low.
	CHECK_EQ( c64PortRead( 0x2F, 0x17 ), 0x17 );

	// All inputs: pull-ups on 0-2, cassette sense high, rest low.
	CHECK_EQ( c64PortRead( 0x00, 0x00 ), 0x17 );

	// Bits 6/7 never appear, whatever the latch says.
	CHECK_EQ( c64PortRead( 0xFF, 0xFF ) & 0xC0, 0x00 );

	// Banking still only ever looks at bits 0-2.
	CHECK_EQ( c64PortEffective( 0x2F, 0x37 ), 0x07 );
}
