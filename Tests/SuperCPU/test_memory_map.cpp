/*
   SCPU-EMU - 24-bit address space and SuperRAM tests.

   Layout verified against VICE's SCPU64 emulation; see Source/SuperCPU/memory_map.h.
*/
#include "../test_framework.h"
#include "../../Source/SuperCPU/memory_map.h"
#include "../../Source/Bus/Host/host_bus.h"

struct MapFixture
{
	CHostBus            bus;
	CC64Memory          bank0;
	CFastRAM            simm;
	CSuperCPUMemoryMap  map;

	MapFixture( u32 simmMB = SCPU_SIMM_16MB )
	{
		bank0.attachBus( &bus );
		simm.init( simmMB );
		map.attachBank0( &bank0 );
		map.attachFastRAM( &simm );
		bank0.reset();
		map.reset();
	}
};

TEST( map_bank0_is_the_c64 )
{
	MapFixture f;

	// Bank 0 goes through CC64Memory, so it sees C64 banking. $0400 is RAM.
	f.map.write8( 0x000400, 0x41 );
	CHECK_EQ( f.map.read8( 0x000400 ), 0x41 );
	CHECK_EQ( f.bank0.m_RAM[ 0x0400 ], 0x41 );
	CHECK( f.map.m_Bank0Accesses > 0 );
	CHECK_EQ( f.map.m_FastAccesses, 0 );
}

TEST( map_bank1_is_private_sram )
{
	MapFixture f;

	f.map.write8( 0x010000, 0x5A );
	f.map.write8( 0x01FFFF, 0xA5 );

	CHECK_EQ( f.map.read8( 0x010000 ), 0x5A );
	CHECK_EQ( f.map.read8( 0x01FFFF ), 0xA5 );

	// Bank 1 is the accelerator's own memory: nothing reaches the C64.
	CHECK_EQ( f.bus.m_Cycles, 0 );
	CHECK_EQ( f.map.m_Bank0Accesses, 0 );
}

TEST( map_superram_is_addressable_across_16mb )
{
	MapFixture f( SCPU_SIMM_16MB );
	CHECK_EQ( f.simm.sizeMB(), 16 );

	// A byte in a high bank, well beyond anything a 6502 could name.
	f.map.write8( 0x123456, 0x77 );
	CHECK_EQ( f.map.read8( 0x123456 ), 0x77 );

	// Distinct banks are distinct memory.
	f.map.write8( 0x020000, 0x11 );
	f.map.write8( 0x030000, 0x22 );
	CHECK_EQ( f.map.read8( 0x020000 ), 0x11 );
	CHECK_EQ( f.map.read8( 0x030000 ), 0x22 );

	// None of it touches the expansion port, which is the point: SuperRAM runs
	// at full speed because it never leaves the Pi.
	CHECK_EQ( f.bus.m_Cycles, 0 );
}

TEST( map_superram_aliases_beyond_the_fitted_size )
{
	// A real SIMM decodes a fixed number of address lines, so an address past
	// the fitted size wraps onto lower memory. Software sizing the card relies
	// on exactly that.
	MapFixture f( SCPU_SIMM_1MB );
	CHECK_EQ( f.simm.sizeMB(), 1 );

	f.map.write8( 0x020000, 0x99 );

	// Past 1MB the map stops decoding SuperRAM and returns open bus, which on a
	// 65816 is the bank byte.
	CHECK_EQ( f.map.read8( 0x800000 ), 0x80 );
}

TEST( map_unmapped_reads_return_the_bank_byte )
{
	MapFixture f( SCPU_SIMM_NONE );
	CHECK( !f.simm.present() );

	// With no SIMM fitted everything above bank 1 is open bus.
	CHECK_EQ( f.map.read8( 0x020000 ), 0x02 );
	CHECK_EQ( f.map.read8( 0xAB1234 ), 0xAB );
}

TEST( map_rom_region_reads_the_image_when_supplied )
{
	MapFixture f;

	static u8 rom[ 0x1000 ];
	for ( u32 i = 0; i < sizeof( rom ); i++ ) rom[ i ] = (u8)( i ^ 0x5A );
	f.map.setROM( rom, sizeof( rom ) );
	CHECK( f.map.hasROM() );

	CHECK_EQ( f.map.read8( SCPU_ROM_BASE ), rom[ 0 ] );
	CHECK_EQ( f.map.read8( SCPU_ROM_BASE + 0x123 ), rom[ 0x123 ] );

	// Writes to ROM are discarded, not passed anywhere.
	f.map.write8( SCPU_ROM_BASE, 0xFF );
	CHECK_EQ( f.map.read8( SCPU_ROM_BASE ), rom[ 0 ] );
}

TEST( map_128k_rom_mirrors_across_the_whole_region )
{
	// The real SuperCPU DOS images are 128K and the ROM region is 512K, so an
	// image mirrors four times. The property that matters is the alignment: the
	// TOP of the image has to land at the TOP of the address space, because
	// that is where a 65816 vector table sits. SuperCPU DOS 2.04 has a valid one
	// there -- RESET=$FF3D, NMI=$FF05, IRQ=$FF17 -- and a mapping that put the
	// image's top anywhere else would decode it as ordinary data.
	MapFixture f;

	static u8 rom[ 0x20000 ];					// 128K, as shipped
	for ( u32 i = 0; i < sizeof( rom ); i++ ) rom[ i ] = (u8)( i * 7 + ( i >> 8 ) );
	f.map.setROM( rom, sizeof( rom ) );

	// Base of the region is the base of the image.
	CHECK_EQ( f.map.read8( SCPU_ROM_BASE ), rom[ 0 ] );

	// Top of the address space is the top of the image.
	CHECK_EQ( f.map.read8( 0xFFFFFF ), rom[ 0x1FFFF ] );
	CHECK_EQ( f.map.read8( 0xFFFFFC ), rom[ 0x1FFFC ] );

	// And it repeats every 128K in between, four times over the 512K region.
	for ( u32 copy = 0; copy < 4; copy++ )
	{
		const u32 base = SCPU_ROM_BASE + copy * 0x20000;
		CHECK_EQ( f.map.read8( base ),          rom[ 0 ] );
		CHECK_EQ( f.map.read8( base + 0x1234 ), rom[ 0x1234 ] );
		CHECK_EQ( f.map.read8( base + 0x1FFFF), rom[ 0x1FFFF ] );
	}
}

TEST( map_rom_region_is_open_bus_without_an_image )
{
	MapFixture f;
	CHECK( !f.map.hasROM() );

	// SCPU-EMU boots the machine's own KERNAL out of bank 0, so the
	// accelerator's ROM being absent is a supported configuration.
	CHECK_EQ( f.map.read8( 0xF80000 ), 0xF8 );
}

TEST( map_simm_window_reaches_the_card )
{
	MapFixture f;

	f.map.write8( SCPU_SIMM_WINDOW, 0xC3 );
	CHECK_EQ( f.map.read8( SCPU_SIMM_WINDOW ), 0xC3 );

	// The window is a view onto the start of the SIMM, so the same byte is
	// visible through the linear mapping at bank $02... which is offset 0.
	CHECK_EQ( f.simm.read( 0 ), 0xC3 );
}

TEST( map_simm_sizes_round_down_to_what_hardware_accepts )
{
	CFastRAM r;
	r.init( 3 );  CHECK_EQ( r.sizeMB(), 1 );	// 3 -> 1
	r.init( 7 );  CHECK_EQ( r.sizeMB(), 4 );	// 7 -> 4
	r.init( 12 ); CHECK_EQ( r.sizeMB(), 8 );	// 12 -> 8
	r.init( 64 ); CHECK_EQ( r.sizeMB(), 16 );	// clamped to the maximum
	r.init( 0 );  CHECK( !r.present() );
}

// ---------------------------------------------------------------------------
// Bootmap
// ---------------------------------------------------------------------------
// A real SuperCPU comes out of reset with its own ROM mapped over bank 0, so
// CMD's boot code runs before the C64's KERNAL. The layout below is VICE's
// scpu64meminit.c at the reset configuration (mem_config $87), and the ROM is
// indexed 1:1 by the 16-bit address -- $FFFC reads image offset $FFFC.

static void fillBootROM( u8 *rom )
{
	// Distinctive per-address content, plus a plausible reset vector.
	for ( u32 i = 0; i < 0x10000; i++ ) rom[ i ] = (u8)( ( i >> 8 ) ^ 0xA5 );
	rom[ 0xFFFC ] = 0x00;
	rom[ 0xFFFD ] = 0xFE;		// SuperCPU DOS 2.04 resets to $FE00
}

TEST( bootmap_maps_the_scpu_rom_over_bank_zero )
{
	static u8 rom[ 0x10000 ];
	fillBootROM( rom );

	CC64Memory mem;
	mem.setBootmapROM( rom );
	mem.m_BootmapActive = true;

	// RAM below $8000 is untouched -- the boot code needs somewhere to work.
	mem.m_RAM[ 0x1234 ] = 0x77;
	CHECK_EQ( mem.read8( 0x1234 ), 0x77 );

	// $8000-$CFFF and $E000-$FFFF come from the EPROM, indexed 1:1.
	CHECK_EQ( mem.read8( 0x8000 ), rom[ 0x8000 ] );
	CHECK_EQ( mem.read8( 0xA000 ), rom[ 0xA000 ] );
	CHECK_EQ( mem.read8( 0xCFFF ), rom[ 0xCFFF ] );
	CHECK_EQ( mem.read8( 0xE000 ), rom[ 0xE000 ] );
	CHECK_EQ( mem.read8( 0xFFFC ), 0x00 );
	CHECK_EQ( mem.read8( 0xFFFD ), 0xFE );

	// ...and the reset vector a 65816 would fetch is CMD's, not the KERNAL's.
	const u16 resetVector = (u16)( mem.read8( 0xFFFC ) | ( mem.read8( 0xFFFD ) << 8 ) );
	CHECK_EQ( resetVector, 0xFE00 );
}

TEST( bootmap_leaves_io_and_low_ram_alone )
{
	static u8 rom[ 0x10000 ];
	fillBootROM( rom );

	CC64Memory mem;
	mem.setBootmapROM( rom );
	mem.m_BootmapActive = true;

	// $D000-$DFFF stays I/O at every configuration this emulator can reach.
	// The boot code needs the VIC-II and the CIAs, so mapping ROM over them
	// would leave it with no screen and no keyboard.
	CHECK_EQ( c64MapRead( 0xD020, mem.m_BankMode ), REG_IO );

	// And the whole of $0000-$7FFF is still RAM.
	mem.m_RAM[ 0x0400 ] = 0x11;
	mem.m_RAM[ 0x7FFF ] = 0x22;
	CHECK_EQ( mem.read8( 0x0400 ), 0x11 );
	CHECK_EQ( mem.read8( 0x7FFF ), 0x22 );
}

TEST( bootmap_off_restores_the_normal_c64_map )
{
	static u8 rom[ 0x10000 ];
	fillBootROM( rom );

	static u8 kernal[ C64_KERNAL_SIZE ];
	for ( u32 i = 0; i < C64_KERNAL_SIZE; i++ ) kernal[ i ] = 0x5A;

	CC64Memory mem;
	mem.setKernalROM( kernal );
	mem.setBootmapROM( rom );
	mem.m_BootmapActive = true;

	CHECK_EQ( mem.read8( 0xE000 ), rom[ 0xE000 ] );

	// Writing $D0B6 maps it out. The very next fetch must see the KERNAL --
	// the boot code disables bootmap and continues straight on, so anything
	// lazier than immediate would keep executing ROM that is no longer there.
	mem.m_BootmapActive = false;
	CHECK_EQ( mem.read8( 0xE000 ), 0x5A );
	CHECK_EQ( mem.read8( 0xFFFC ), 0x5A );
}
