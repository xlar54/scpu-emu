/*
   SCPU-EMU - 24-bit address space and SuperRAM tests.

   Layout verified against VICE's SCPU64 emulation; see Source/SuperCPU/memory_map.h.
*/
#include "../test_framework.h"
#include "../../Source/SuperCPU/memory_map.h"
#include "../../Source/Bus/Host/host_bus.h"
#include "../../Source/SuperCPU/registers.h"
#include "../../Source/SuperCPU/write_buffer.h"

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

TEST( map_bank1_port_bytes_alias_bank0 )
{
	MapFixture f;
	f.map.write8( 0x010000, 0x2F );
	f.map.write8( 0x010001, 0x35 );
	CHECK_EQ( f.bank0.m_Port00, 0x2F );
	CHECK_EQ( f.bank0.m_Port01, 0x35 );
	CHECK_EQ( f.map.read8( 0x010000 ), f.map.read8( 0x000000 ) );
	CHECK_EQ( f.map.read8( 0x010001 ), f.map.read8( 0x000001 ) );
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

TEST( map_eprom_accesses_charge_three_extra_turbo_cycles )
{
	MapFixture f;
	static u8 rom[ 0x10000 ] = { 0 };
	f.map.setROM( rom, sizeof rom );
	f.bank0.setPacing( 0, 20000000 );
	const u64 start = f.bank0.m_EmuCycles;
	f.map.read8( SCPU_ROM_BASE );
	f.bank0.tickFast( 1 );
	CHECK_EQ( f.bank0.m_EmuCycles - start, 4u );
	CHECK_EQ( f.bank0.m_RegionStretchCycles, 3u );
}

TEST( map_simm_configuration_changes_decode_and_page_timing )
{
	MapFixture f( SCPU_SIMM_16MB );
	CSuperCPURegisters regs;
	regs.attachFastRAM( &f.simm );
	regs.reset();
	regs.ioWrite( SCPU_REG_HWREGS_ENABLE, 0 );
	regs.ioWrite( SCPU_REG_SIMM_CONFIG, 0 );	// 1MB, 2KB rows
	CHECK_EQ( f.simm.config() & 7, 0 );

	f.simm.write( 0x000123, 0x5A );
	CHECK_EQ( f.simm.read( 0x100123 ), 0x5A );	// configured decode aliases
	CHECK_EQ( f.simm.accessPenaltyHalfCycles( 0x002000, false ), 5u );
	CHECK_EQ( f.simm.accessPenaltyHalfCycles( 0x002004, false ), 0u );
	CHECK_EQ( f.simm.accessPenaltyHalfCycles( 0x002100, false ), 2u );
	CHECK_EQ( f.simm.accessPenaltyHalfCycles( 0x004000, true ), 4u );
}

TEST( map_simm_window_writes_require_the_hardware_register_gate )
{
	MapFixture f( SCPU_SIMM_16MB );
	CSuperCPURegisters regs;
	f.bank0.setIOInterceptor( &regs );
	regs.reset();
	f.map.write8( SCPU_SIMM_WINDOW + 0x123, 0x11 );
	CHECK_EQ( f.simm.read( 0x123 ), 0x00 );
	regs.ioWrite( SCPU_REG_HWREGS_ENABLE, 0 );
	f.map.write8( SCPU_SIMM_WINDOW + 0x123, 0x22 );
	CHECK_EQ( f.simm.read( 0x123 ), 0x22 );
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

TEST( scpu128_bootmap_enters_the_upper_dos204_extension_half )
{
	static u8 rom[ 0x20000 ];
	for ( u32 i = 0; i < sizeof( rom ); i++ ) rom[ i ] = 0;
	rom[ 0x05E45 ] = 0x00;       // low-half target is deliberately empty
	rom[ 0x15E45 ] = 0xB5;       // actual DOS 2.04 C128 entry byte
	rom[ 0x0FE00 ] = 0x78;       // common reset stub remains in the low half

	CC64Memory mem;
	mem.setBootmapROM( rom, sizeof( rom ) );
	mem.setSCPU128Mode( true );
	mem.m_BootmapActive = true;

	CHECK_EQ( mem.read8( 0xFE00 ), 0x78 );
	CHECK_EQ( mem.read8( 0x5E45 ), 0xB5 );
	mem.m_RAM[ 0x6000 ] = 0x66;
	CHECK_EQ( mem.read8( 0x6000 ), 0x66 );
}

TEST( scpu128_always_visible_mmu_registers_bypass_c64_pla_and_bootmap )
{
	static u8 rom[ 0x20000 ];
	for ( u32 i = 0; i < sizeof( rom ); i++ ) rom[ i ] = 0xA5;

	CHostBus bus;
	CC64Memory mem;
	mem.attachBus( &bus );
	mem.setBootmapROM( rom, sizeof( rom ) );
	mem.setSCPU128Mode( true );
	mem.m_BootmapActive = true;
	mem.m_Port01 = 0xE3;         // would hide I/O in the C64 PLA model
	mem.updateBankMode();

	bus.m_Memory[ 0xD505 ] = 0xB7;
	bus.m_Memory[ 0xD600 ] = 0x5A;
	bus.m_Memory[ 0xFF00 ] = 0x3E;
	CHECK_EQ( mem.read8( 0xD505 ), 0xB7 );
	CHECK_EQ( mem.read8( 0xD600 ), 0x5A );
	CHECK_EQ( mem.read8( 0xFF00 ), 0x3E );

	mem.write8( 0xD505, 0xF7 );
	mem.write8( 0xFF00, 0x00 );
	CHECK_EQ( bus.m_Memory[ 0xD505 ], 0xF7 );
	CHECK_EQ( bus.m_Memory[ 0xFF00 ], 0x00 );
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

// ---------------------------------------------------------------------------
// The KERNAL window moves when the register bank opens
// ---------------------------------------------------------------------------

TEST( kernal_window_follows_the_hardware_register_bank )
{
	// VICE's scpu64meminit.c, at ordinary C64 banking with bootmap off:
	//
	//   hwregs off  KT  mem_sram[0x10000 + addr]  ->  bank 1 $E000-$FFFF
	//   hwregs on   KS  mem_sram[0x8000  + addr]  ->  bank 1 $6000-$7FFF
	//
	// This matters more than it looks. While the registers are open, bank 1
	// $E000-$FFFF is NOT the KERNAL -- it is free SRAM, and the SuperCPU's boot
	// code is entitled to use it as scratch. Serving the KERNAL from there
	// anyway means watching it get overwritten, which presents as a machine
	// that runs the whole boot animation correctly and then comes up blank.
	static u8 bank1[ 0x10000 ];
	for ( u32 i = 0; i < 0x10000; i++ ) bank1[ i ] = 0;

	// Two distinguishable KERNALs, one in each window.
	for ( u32 i = 0; i < C64_KERNAL_SIZE; i++ )
	{
		bank1[ 0xE000 + i ] = 0xAA;		// KT
		bank1[ 0x6000 + i ] = 0x55;		// KS
	}

	CC64Memory mem;
	CSuperCPURegisters regs;
	mem.setIOInterceptor( &regs );
	mem.setROMShadow( bank1 );
	regs.trackKernalShadow( &mem.m_KernalShadowBase );
	regs.reset();

	// Registers closed: the KERNAL comes from bank 1 $E000.
	CHECK( !regs.hardwareRegsEnabled() );
	CHECK_EQ( mem.read8( 0xE000 ), 0xAA );
	CHECK_EQ( mem.read8( 0xFFFF ), 0xAA );

	// Opening the bank moves the window immediately -- the very next fetch has
	// to see it, because that is how the boot code uses it.
	mem.write8( SCPU_REG_HWREGS_ENABLE, 0 );
	CHECK( regs.hardwareRegsEnabled() );
	CHECK_EQ( mem.read8( 0xE000 ), 0x55 );
	CHECK_EQ( mem.read8( 0xFFFF ), 0x55 );

	// And closing it moves the window back.
	mem.write8( SCPU_REG_HWREGS_DISABLE, 0 );
	CHECK( !regs.hardwareRegsEnabled() );
	CHECK_EQ( mem.read8( 0xE000 ), 0xAA );

	// BASIC does not move: it is R1 in both cases.
	for ( u32 i = 0; i < C64_BASIC_SIZE; i++ ) bank1[ 0xA000 + i ] = 0x77;
	CHECK_EQ( mem.read8( 0xA000 ), 0x77 );
	mem.write8( SCPU_REG_HWREGS_ENABLE, 0 );
	CHECK_EQ( mem.read8( 0xA000 ), 0x77 );
}

TEST( kernal_window_follows_a_d0b2_bank_close )
{
	// CMD's serial receive ends with a cross-image trampoline: from the KS
	// image, STZ $D0B2 (bit 7 clear = close the register bank), and the very
	// next fetch must come from the KT image -- the two kernal variants differ
	// at that address by design, and the KT side is the routine's epilogue.
	// Missing the window move here left the machine looping in the KS image
	// forever: the disk-access freeze, identical on two different drives.
	// VICE scpu64mem.c:753 is the reference for the write semantics.
	static u8 bank1[ 0x10000 ];
	for ( u32 i = 0; i < 0x10000; i++ ) bank1[ i ] = 0;
	for ( u32 i = 0; i < C64_KERNAL_SIZE; i++ )
	{
		bank1[ 0xE000 + i ] = 0xAA;		// KT
		bank1[ 0x6000 + i ] = 0x55;		// KS
	}

	CC64Memory mem;
	CSuperCPURegisters regs;
	mem.setIOInterceptor( &regs );
	mem.setROMShadow( bank1 );
	regs.trackKernalShadow( &mem.m_KernalShadowBase );
	regs.reset();

	// Open the bank the way software does, then close it via $D0B2 bit 7.
	mem.write8( SCPU_REG_HWREGS_ENABLE, 0 );
	CHECK( regs.hardwareRegsEnabled() );
	CHECK_EQ( mem.read8( 0xEE82 ), 0x55 );		// fetching from KS

	mem.write8( SCPU_REG_STATUS, 0x00 );		// STZ $D0B2

	CHECK( !regs.hardwareRegsEnabled() );
	CHECK_EQ( mem.read8( 0xEE82 ), 0xAA );		// the VERY NEXT fetch is KT

	// And bit 7 SET must leave the bank open and the window in place.
	mem.write8( SCPU_REG_HWREGS_ENABLE, 0 );
	mem.write8( SCPU_REG_STATUS, 0x80 );
	CHECK( regs.hardwareRegsEnabled() );
	CHECK_EQ( mem.read8( 0xEE82 ), 0x55 );
}

TEST( scpu_private_ram_writes_need_the_register_bank_open )
{
	// The SuperCPU's private RAM at $D200-$D3FF is READABLE at all times but
	// only WRITABLE while the hardware register bank is open. $D27E is the one
	// exception in the $D2xx page; $D3xx has none. Matches VICE's
	// scpu64_d200_store / scpu64_d300_store.
	//
	// Writing both pages unconditionally is a fidelity bug a private-RAM test
	// catches: it passes against hardware that gates the writes and fails
	// against an emulator that does not, without the test ever opening the
	// bank.
	CSuperCPURegisters regs;
	regs.reset();

	// Bank closed after reset: writes are discarded, reads still answer.
	u8 v = 0xAA;
	CHECK( regs.ioWrite( 0xD20C, 0x5A ) );
	CHECK( regs.ioRead( 0xD20C, v ) );
	CHECK_EQ( v, 0x00 );

	CHECK( regs.ioWrite( 0xD34F, 0x5A ) );
	CHECK( regs.ioRead( 0xD34F, v ) );
	CHECK_EQ( v, 0x00 );

	// $D27E takes a write even with the bank closed.
	CHECK( regs.ioWrite( 0xD27E, 0x5A ) );
	CHECK( regs.ioRead( 0xD27E, v ) );
	CHECK_EQ( v, 0x5A );

	// Open the bank: both pages become writable.
	regs.ioWrite( SCPU_REG_HWREGS_ENABLE, 0x00 );
	CHECK( regs.ioWrite( 0xD20C, 0x5A ) );
	CHECK( regs.ioRead( 0xD20C, v ) );
	CHECK_EQ( v, 0x5A );

	CHECK( regs.ioWrite( 0xD34F, 0x5A ) );
	CHECK( regs.ioRead( 0xD34F, v ) );
	CHECK_EQ( v, 0x5A );

	// Close it again: the stored values stay readable, new writes do not land.
	regs.ioWrite( SCPU_REG_HWREGS_DISABLE, 0x00 );
	CHECK( regs.ioWrite( 0xD20C, 0x11 ) );
	CHECK( regs.ioRead( 0xD20C, v ) );
	CHECK_EQ( v, 0x5A );
}

TEST( scpu_v2_private_ram_aliases_bank1_sram )
{
	CSuperCPUMemoryMap map;
	CSuperCPURegisters regs;
	regs.attachBank1( map.m_Bank1 );
	regs.reset();

	regs.ioWrite( SCPU_REG_HWREGS_ENABLE, 0 );
	CHECK( regs.ioWrite( 0xD234, 0x5A ) );
	CHECK_EQ( map.read8( 0x01D234 ), 0x5A );

	map.write8( 0x01D345, 0xA5 );
	u8 v = 0;
	CHECK( regs.ioRead( 0xD345, v ) );
	CHECK_EQ( v, 0xA5 );
}

TEST( scpu128_native_reset_selects_the_dos204_c128_branch )
{
	CSuperCPURegisters regs;
	regs.setSCPU128Mode( true );
	regs.reset();

	u8 v = 0;
	CHECK( regs.ioRead( SCPU_REG_VERSION, v ) );
	CHECK_EQ( v, 0x01 );
	CHECK( regs.ioRead( SCPU_REG_STATUS, v ) );
	CHECK_EQ( v, 0x81 );
	CHECK( regs.ioRead( SCPU_REG_OPTIM_V2, v ) );
	CHECK_EQ( v, 0xC1 );
	CHECK( regs.hardwareRegsEnabled() );
	CHECK_EQ( regs.optimRegister(), 0xC1 );

	// Switching the same model back to its C64 personality restores the v2
	// SCPU64-compatible reset values used by DOS 2.04's other branch.
	regs.setSCPU128Mode( false );
	regs.reset();
	CHECK( regs.ioRead( SCPU_REG_VERSION, v ) );
	CHECK_EQ( v, 0x47 );
	CHECK( regs.ioRead( SCPU_REG_STATUS, v ) );
	CHECK_EQ( v, 0x07 );
	CHECK( !regs.hardwareRegsEnabled() );
	CHECK_EQ( regs.optimRegister(), 0xC7 );
}

TEST( scpu_v2_colour_ram_is_a_full_byte_bank1_cache )
{
	CHostBus bus;
	CC64Memory mem;
	CSuperCPUMemoryMap map;
	CSuperCPURegisters regs;
	mem.attachBus( &bus );
	mem.setIOInterceptor( &regs );
	map.attachBank0( &mem );
	regs.attachBank1( map.m_Bank1 );
	regs.setHardwareVersion( SCPU_V2 );
	regs.reset();
	mem.reset();
	mem.setPacing( 0, 20000000 );

	mem.write8( 0xD800, 0xFA );
	CHECK_EQ( bus.m_Memory[ 0xD800 ], 0xFA );	// physical chip still written
	CHECK_EQ( mem.read8( 0xD800 ), 0xFA );		// full cached byte, no open bus
	CHECK_EQ( map.read8( 0x01D800 ), 0xFA );	// same byte through bank 1

	// The colour write uses the posted buffer class and the cached read is
	// internal SRAM, so neither queues an ordinary I/O stretch.
	mem.tickFast( 4 );
	CHECK_EQ( mem.m_IOStretchCycles, 0u );
}

TEST( scpu_ramlink_strobes_update_status_without_swallowing_the_bus_write )
{
	CSuperCPURegisters regs;
	regs.reset();
	u8 v = 0;
	CHECK( !regs.ioWrite( 0xDF7E, 0 ) );
	CHECK( regs.interruptRerouteRequested() );
	CHECK( regs.ioRead( SCPU_REG_DOSEXT, v ) );
	CHECK( ( v & SCPU_DOS_RAMLINK ) != 0 );
	CHECK( !regs.ioWrite( 0xDF7F, 0 ) );
	CHECK( regs.ioRead( SCPU_REG_DOSEXT, v ) );
	CHECK( ( v & SCPU_DOS_RAMLINK ) == 0 );
}

TEST( scpu_dos_extension_maps_its_bank1_work_areas )
{
	CHostBus bus;
	CC64Memory bank0;
	CSuperCPUMemoryMap map;
	CSuperCPURegisters regs;
	bank0.attachBus( &bus );
	bank0.setIOInterceptor( &regs );
	map.attachBank0( &bank0 );
	bank0.setROMShadow( map.m_Bank1 );
	regs.attachBank1( map.m_Bank1 );
	regs.reset();
	bank0.reset();

	regs.ioWrite( SCPU_REG_HWREGS_ENABLE, 0 );
	regs.ioWrite( SCPU_REG_DOSEXT_ON, 0 );
	CHECK( regs.dosExtensionEnabled() );

	bank0.m_RAM[ 0x1234 ] = 0x11;
	bank0.m_RAM[ 0x8123 ] = 0x22;
	map.m_Bank1[ 0x1234 ] = 0xA1;
	map.m_Bank1[ 0x8123 ] = 0xB2;
	CHECK_EQ( bank0.readFast( 0x1234 ), 0xA1 );
	CHECK_EQ( bank0.readFast( 0x8123 ), 0xB2 );

	bank0.writeFast( 0x1234, 0xC3 );
	bank0.writeFast( 0x8123, 0xD4 );
	CHECK_EQ( map.m_Bank1[ 0x1234 ], 0xC3 );
	CHECK_EQ( map.m_Bank1[ 0x8123 ], 0xD4 );
	CHECK_EQ( bank0.m_RAM[ 0x1234 ], 0x11 );
	CHECK_EQ( bank0.m_RAM[ 0x8123 ], 0x22 );

	regs.ioWrite( SCPU_REG_DOSEXT_OFF, 0 );
	CHECK_EQ( bank0.readFast( 0x1234 ), 0x11 );
	CHECK_EQ( bank0.readFast( 0x8123 ), 0x22 );
}

TEST( bank0_readfast_matches_read8_in_every_banking_state )
{
	// readFast() answers RAM, BASIC and KERNAL inline instead of calling the
	// 2372-byte read8(). That is only legitimate if the byte is identical in
	// every state read8() can be in, so enumerate them: both ROM-shadow
	// states, both KERNAL shadow bases ($E000 KT and $6000 KS), SCPU128 mode
	// on and off, and all 32 PLA configurations against every 4K page.
	// I/O is excluded because read8() has read-to-clear side effects there
	// and readFast() declines to it anyway.
	CHostBus bus;
	CC64Memory bank0;
	CSuperCPUMemoryMap map;
	bank0.attachBus( &bus );
	map.attachBank0( &bank0 );
	bank0.reset();

	static u8 basic[ 0x2000 ], kernal[ 0x2000 ], chars[ 0x1000 ];
	for ( u32 i = 0; i < 0x2000; i++ ) basic[ i ]  = (u8)( 0x40 + ( i & 0x0F ) );
	for ( u32 i = 0; i < 0x2000; i++ ) kernal[ i ] = (u8)( 0x50 + ( i & 0x0F ) );
	for ( u32 i = 0; i < 0x1000; i++ ) chars[ i ]  = (u8)( 0x60 + ( i & 0x0F ) );
	bank0.setBasicROM( basic );
	bank0.setKernalROM( kernal );
	bank0.setCharROM( chars );
	for ( u32 a = 0; a < 0x10000; a++ ) bank0.m_RAM[ a ] = (u8)( a ^ 0x5A );
	// The bank-1 fill must not repeat every 256 bytes. The two KERNAL shadow
	// bases are exactly $8000 apart, so a pattern derived from the low byte
	// alone answers KT and KS with the SAME value and the test goes blind to
	// the one mistake that matters most here -- the boot animation runs and
	// the screen then goes blank.
	for ( u32 a = 0; a < SCPU_BANK1_SIZE; a++ )
		map.m_Bank1[ a ] = (u8)( a ^ ( a >> 8 ) ^ 0xA5 );

	for ( u32 scpu128 = 0; scpu128 < 2; scpu128++ )
	for ( u32 shadow = 0; shadow < 2; shadow++ )
	for ( u32 ks = 0; ks < 2; ks++ )
	{
		bank0.setSCPU128Mode( scpu128 != 0 );
		bank0.setROMShadow( shadow ? map.m_Bank1 : 0 );
		bank0.m_KernalShadowBase = ks ? 0x6000 : 0xE000;
		for ( u32 mode = 0; mode < 32; mode++ )
		{
			bank0.m_BankMode = (u8)mode;
			// This loop writes m_BankMode and m_KernalShadowBase behind their
			// setters, so it owes the read-decode table what updateBankMode()
			// and the register layer's map hook do for the live machine. Delete
			// this line to watch the table go stale: 6572 of these checks fail,
			// which is the guard that makes a cached decode safe to ship.
			bank0.rebuildReadDecode();
			for ( u32 page = 0; page < 16; page++ )
			{
				const C64Region r = c64MapRead( (u16)( page << 12 ), (u8)mode );
				if ( r == REG_IO )
					continue;
				// All three SCPU128 windows have to be sampled or the
				// m_SCPU128Mode decline goes untested: $FF00-$FF04 sits inside
				// the KERNAL page, $D500 and $D600 inside page $D.
				const u16 lows[ 7 ] = { 0x0002, 0x0345, 0x0500, 0x0600,
				                        0x0F00, 0x0F04, 0x0FFF };
				for ( u32 i = 0; i < 7; i++ )
				{
					const u16 a = (u16)( ( page << 12 ) | lows[ i ] );
					const bool scpu128Window =
					       ( a >= 0xD500 && a <= 0xD50B )
					    || ( a >= 0xD600 && a <= 0xD601 )
					    || ( a >= 0xFF00 && a <= 0xFF04 );
					// PRE-EXISTING divergence, and not this fast path's: when the
					// PLA resolves one of those windows to RAM, readFast()'s RAM
					// arm has always served it out of bank-0 DRAM while read8()
					// routes it to the 8722. Drop the r == REG_RAM term to
					// reproduce; it fails with no fast ROM arm at all.
					if ( scpu128 && scpu128Window && r == REG_RAM )
						continue;
					CHECK_EQ( bank0.readFast( a ), bank0.read8( a ) );
				}
			}
		}
	}
}

// Two machines in identical state, one driven through writeFast and one
// through write8. Heap-allocated: CC64Memory is ~140KB and this test needs two
// of them plus their buses, which is more than a test stack should carry.
struct WriteDiffPair
{
	CHostBus    busA, busB;
	CC64Memory  a, b;

	WriteDiffPair()
	{
		a.attachBus( &busA );
		b.attachBus( &busB );
		a.reset();
		b.reset();
	}
};

TEST( bank0_writefast_matches_write8_in_every_banking_state )
{
	// The write-side twin of bank0_readfast_matches_read8_in_every_banking_state
	// above, and the reason the WRITE decode cache is safe to ship.
	//
	// writeFast() stores through m_WriteBase instead of calling write8(), so
	// every byte it stores must land where write8() would have put it, in every
	// state write8() can be in. A stale entry here is strictly worse than on
	// the read side: it does not return one wrong byte, it STORES a byte into
	// bank-0 DRAM that belonged to a VIC/SID/CIA/colour-RAM register, so the
	// DRAM byte is permanently wrong AND the register write never happens.
	//
	// Three observables per write, which between them pin both failure
	// directions:
	//   m_RAM[ a ]   did the byte land in DRAM, or not?
	//   m_IOWrites   did the access reach the I/O path at all?
	//   m_RamWrites  was it accounted as a DRAM write?
	// I/O pages are deliberately NOT skipped the way the read test skips them:
	// "the table still says RAM for a page that is now I/O" is the catastrophic
	// direction, and skipping I/O pages is exactly what would hide it.
	WriteDiffPair *p = new WriteDiffPair();
	CC64Memory &a = p->a;
	CC64Memory &b = p->b;

	static u8 shadowA[ SCPU_BANK1_SIZE ], shadowB[ SCPU_BANK1_SIZE ];
	for ( u32 i = 0; i < SCPU_BANK1_SIZE; i++ )
		shadowA[ i ] = shadowB[ i ] = (u8)( i ^ ( i >> 8 ) ^ 0xA5 );

	u8 value = 0x11;

	for ( u32 scpu128 = 0; scpu128 < 2; scpu128++ )
	for ( u32 shadow = 0; shadow < 2; shadow++ )
	for ( u32 ks = 0; ks < 2; ks++ )
	{
		a.setSCPU128Mode( scpu128 != 0 );
		b.setSCPU128Mode( scpu128 != 0 );
		a.setROMShadow( shadow ? shadowA : 0 );
		b.setROMShadow( shadow ? shadowB : 0 );
		a.m_KernalShadowBase = b.m_KernalShadowBase = ks ? 0x6000 : 0xE000;
		for ( u32 mode = 0; mode < 32; mode++ )
		{
			a.m_BankMode = b.m_BankMode = (u8)mode;
			// This loop writes m_BankMode behind its setter, so it owes the
			// decode tables what updateBankMode() does for the live machine.
			// DELETE THE rebuildWriteDecode() LINE TO WATCH THE TABLE GO
			// STALE: thousands of these checks fail, and they fail as wrong
			// BYTES in DRAM, which is the guard that makes a cached write
			// decode safe to ship at all.
			a.rebuildReadDecode();
			b.rebuildReadDecode();
			a.rebuildWriteDecode();
			b.rebuildWriteDecode();
			for ( u32 page = 0; page < 16; page++ )
			{
				const C64Region r = c64MapRead( (u16)( page << 12 ), (u8)mode );
				// The table must be CURRENT, not merely present. A table that is
				// null everywhere is writeFast's deliberate bisect
				// (WRITE_DECODE_CACHE = false): every write then goes through
				// write8(), which is bit-identical and would satisfy every byte
				// comparison below while buying nothing. So assert the shape of
				// the table itself as well as the bytes it produces -- dropping an
				// invalidation can leave it EMPTY as easily as STALE, and only one
				// of those two shows up in the bytes. Skipped when the cache is
				// bisected off, because then null everywhere IS the correct shape
				// and the byte comparisons below are the whole point of the run.
				if ( CC64Memory::WRITE_DECODE_CACHE )
				{
					CHECK_EQ( a.m_WriteBase[ page ] != 0, r != REG_IO );
					CHECK_EQ( a.m_WriteBase[ page ] != 0,
					          !c64WriteIsIO( (u16)( page << 12 ),
					                         a.m_BankMode ) );
				}
				// $0000/$0001 are the 6510 port: writeFast declines to write8()
				// unconditionally and write8() then calls updateBankMode(),
				// which would overwrite this loop's hand-set m_BankMode. There
				// is nothing to compare -- both paths are the same call.
				const u16 lows[ 8 ] = { 0x0002, 0x0345, 0x03F8, 0x0500,
				                        0x0600, 0x0F00, 0x0F04, 0x0FFF };
				for ( u32 i = 0; i < 8; i++ )
				{
					const u16 addr = (u16)( ( page << 12 ) | lows[ i ] );
					const bool scpu128Window =
					       ( addr >= 0xD500 && addr <= 0xD50B )
					    || ( addr >= 0xD600 && addr <= 0xD601 )
					    || ( addr >= 0xFF00 && addr <= 0xFF04 );
					// PRE-EXISTING divergence, inherited from writeFast and
					// shared with readFast -- see the same exemption in the read
					// test. write8() routes these three windows to the 8722
					// BEFORE it consults the PLA, so whenever the PLA resolves
					// the page to anything but I/O, writeFast has always stored
					// them into bank-0 DRAM instead. Drop the `r != REG_IO` term
					// to reproduce; it fails with no write table at all, i.e. it
					// is not this cache's doing. The one-line fix is to null
					// m_WriteBase[ 0xD ] and [ 0xF ] while m_SCPU128Mode is set,
					// which also means deleting this exemption.
					if ( scpu128 && scpu128Window && r != REG_IO )
						continue;

					// Both machines start from the same sentinel so "stored" and
					// "left alone" are distinguishable, and the value changes on
					// every iteration so a stale DRAM byte cannot coincidentally
					// match the one being written.
					value = (u8)( value + 0x37 );
					a.m_RAM[ addr ] = b.m_RAM[ addr ] = 0x5A;
					if ( shadow )
						shadowA[ addr ] = shadowB[ addr ] = 0xA5;

					a.writeFast( addr, value );
					b.write8( addr, value );

					CHECK_EQ( a.m_RAM[ addr ], b.m_RAM[ addr ] );
					CHECK_EQ( a.m_IOWrites, b.m_IOWrites );
					CHECK_EQ( a.m_RamWrites, b.m_RamWrites );
					if ( shadow )
						CHECK_EQ( shadowA[ addr ], shadowB[ addr ] );
				}
			}
		}
	}

	delete p;
}

TEST( bank0_write_decode_follows_a_live_port_write )
{
	// THE INVALIDATION SITE THAT MATTERS, and the one the exhaustive test above
	// cannot reach: m_BankMode changes because the GUEST wrote $01, so
	// updateBankMode() has to re-resolve m_WriteBase in the same breath. That
	// test hand-sets m_BankMode and rebuilds the table itself, so it proves the
	// table's CONTENT, not its invalidation. This one drives the real path.
	// DELETE rebuildWriteDecode() FROM updateBankMode() TO WATCH IT FAIL.
	//
	// The dangerous direction is a stale NON-null entry -- a page that was RAM
	// or character ROM and is now I/O -- because then writeFast stores into
	// bank-0 DRAM and the register write never happens. $D020 is the test
	// point: with CHAREN=0 the page is REG_CHARROM and the write falls through
	// to the DRAM underneath (writes never land in ROM), with CHAREN=1 it is
	// the VIC border register. Alternating the two is exactly the transition a
	// stale table gets wrong, and every page is swept because a stale entry is
	// not confined to the page whose mapping moved.
	WriteDiffPair *p = new WriteDiffPair();
	CC64Memory &a = p->a;
	CC64Memory &b = p->b;

	// Prime the table once, the way updateBankMode() would, so that what the
	// loop below measures is INVALIDATION and not initial construction. Without
	// a prime, dropping the rebuild from updateBankMode() leaves the table NULL
	// rather than stale, and null is the SAFE direction -- writeFast simply
	// declines to write8() and every byte comparison passes.
	//
	// Prime it with CHAREN=0, where page $D is character ROM and therefore
	// WRITABLE DRAM, so the primed entry is non-null. The loop then walks into
	// the three port values where page $D is the VIC, and an entry carried over
	// from here stores $D020 into bank-0 DRAM instead -- a WRONG BYTE, which is
	// the failure this test exists to produce. (The port write below goes
	// through write8() on both machines, so it is the shipped path even here.)
	a.writeFast( 0x0000, 0x07 );
	b.write8   ( 0x0000, 0x07 );
	a.writeFast( 0x0001, 0x03 );
	b.write8   ( 0x0001, 0x03 );
	a.rebuildWriteDecode();
	b.rebuildWriteDecode();

	u8 value = 0x40;
	// Four rounds, so every ordered pair of port values is visited and the test
	// sees each transition in both directions. Within one round the ascending
	// sweep already contains the transition that kills a stale table: port 4
	// (LORAM=0 HIRAM=0 CHAREN=1) resolves page $D to RAM, port 5 resolves it to
	// I/O, so an entry carried over from port 4 stores $D020 into DRAM and
	// loses the VIC register write.
	for ( u32 round = 0; round < 4; round++ )
	for ( u32 port = 0; port < 8; port++ )
	{
		// Through the guest path on both machines: writeFast delegates $00/$01
		// to write8(), which is where updateBankMode() lives, so this is the
		// shipped sequence and not a test-only poke.
		a.writeFast( 0x0000, 0x07 );
		b.write8   ( 0x0000, 0x07 );
		a.writeFast( 0x0001, (u8)port );
		b.write8   ( 0x0001, (u8)port );

		// Current after a GUEST port write, not just after a hand rebuild.
		// Skipped when the cache is bisected off -- see the twin above.
		if ( CC64Memory::WRITE_DECODE_CACHE )
			for ( u32 page = 0; page < 16; page++ )
				CHECK_EQ( a.m_WriteBase[ page ] != 0,
				          !c64WriteIsIO( (u16)( page << 12 ), a.m_BankMode ) );

		for ( u32 page = 0; page < 16; page++ )
		{
			const u16 addr = (u16)( ( page << 12 ) | 0x0020 );
			value = (u8)( value + 0x37 );
			a.m_RAM[ addr ] = b.m_RAM[ addr ] = 0x5A;
			a.writeFast( addr, value );
			b.write8   ( addr, value );
			CHECK_EQ( a.m_RAM[ addr ], b.m_RAM[ addr ] );
			CHECK_EQ( a.m_IOWrites,    b.m_IOWrites );
			CHECK_EQ( a.m_RamWrites,   b.m_RamWrites );
		}
	}

	delete p;
}

TEST( bank0_write_notify_pages_cover_every_sprite_pointer_row )
{
	// m_WriteNotifyPages is writeFast's gate for noteRAMWriteSideEffects, so a
	// stale bit is a sprite-pointer write that never reaches the real bus --
	// "every sprite wears its last shape", which this codebase has shipped once
	// already when writeFast and write8 drifted apart over this exact check.
	//
	// Enumerate every row the VIC can be pointed at: 4 banks x 16 screen
	// positions, driven through the registers that move it, and require the
	// bitmap to name the row's page every time. DELETE rebuildWriteNotify()
	// from updateSpritePtrBase() to watch this fail.
	MapFixture f;
	f.bank0.setRAMUnderIOAccessible( true );	// keep every row reachable

	for ( u32 bank = 0; bank < 4; bank++ )
	{
		// $DD02 makes PA0/PA1 outputs, $DD00 selects the bank through the
		// inverters, so the latch is the complement of the bank number.
		f.bank0.write8( 0xDD02, 0x3F );
		f.bank0.write8( 0xDD00, (u8)( ( ~bank ) & 0x03 ) );
		for ( u32 sel = 0; sel < 16; sel++ )
		{
			f.bank0.write8( 0xD018, (u8)( sel << 4 ) );
			const u32 base = ( bank << 14 ) + ( sel << 10 ) + 0x3F8;
			if ( !CC64Memory::WRITE_NOTIFY_CACHE )
			{
				// Bisected off: every page notifies, which is the K451
				// behaviour and the correct shape for that build.
				CHECK_EQ( f.bank0.m_WriteNotifyPages, 0xFFFFu );
				continue;
			}
			// EXACT, not just "the bit is set": the fail-safe default is
			// 0xFFFF, which would satisfy a bit test while saying nothing.
			// Bit 15 is in the expected value because it is unconditional,
			// which is what makes the REU hook a non-input.
			CHECK_EQ( f.bank0.m_WriteNotifyPages,
			          0x8000u | ( 1u << ( base >> 12 ) ) );
		}
	}
}

TEST( integration_hardware_interrupt_does_not_change_the_register_bank )
{
	// VICE's only SuperCPU-specific interrupt hook is the EPROM vector reroute.
	// Interrupt entry and RTI do not mutate the hardware-register gate. Doing
	// so before choosing the vector could disable the reroute for the very
	// interrupt being dispatched.
	CSuperCPURegisters regs;
	u32 kernalShadowBase = 0xE000;
	regs.reset();
	regs.trackKernalShadow( &kernalShadowBase );

	regs.ioWrite( SCPU_REG_HWREGS_ENABLE, 0 );
	CHECK( regs.hardwareRegsEnabled() );
	CHECK_EQ( kernalShadowBase, 0x6000u );		// KS while open

	CHECK( regs.hardwareRegsEnabled() );
	CHECK_EQ( kernalShadowBase, 0x6000u );
}

TEST( scpu_boot_animation_deviation_is_one_byte_wide )
{
	// BOOT_ANIMATION answers exactly one read -- $D20C with the register bank
	// CLOSED -- with the zeroed VIC register underneath, which is the single
	// branch input 2.04 uses to decide whether to run its C64 startup
	// animation. Everything else must be untouched: the open-bank read still
	// sees the accelerator's RAM, neighbours are unaffected, and with the
	// flag off behaviour is the faithful path.
	CSuperCPURegisters regs;
	regs.reset();
	u8 v = 0xAA;

	// Park $FF in $D20C the way the ROM does: bank open, write, close.
	regs.ioWrite( SCPU_REG_HWREGS_ENABLE, 0 );
	regs.ioWrite( 0xD20C, 0xFF );
	regs.ioWrite( 0xD20D, 0x55 );
	regs.ioWrite( SCPU_REG_HWREGS_DISABLE, 0 );

	// Faithful path: reads answer from the accelerator's RAM regardless.
	CHECK( regs.ioRead( 0xD20C, v ) ); CHECK_EQ( v, 0xFF );

	regs.setBootAnimationHack( true );

	// Closed bank: the one deviating byte reads zero...
	CHECK( regs.ioRead( 0xD20C, v ) ); CHECK_EQ( v, 0x00 );
	// ...its neighbour does not...
	CHECK( regs.ioRead( 0xD20D, v ) ); CHECK_EQ( v, 0x55 );
	// ...and with the bank OPEN the accelerator's RAM answers as ever.
	regs.ioWrite( SCPU_REG_HWREGS_ENABLE, 0 );
	CHECK( regs.ioRead( 0xD20C, v ) ); CHECK_EQ( v, 0xFF );
}

// ---------------------------------------------------------------------------
// CIA2 timer NMI retiming
//
// The physical NMI line is sampled with up to a microsecond of slack; code
// that arms a short CIA2 timer NMI to land at an exact instruction cannot
// survive that jitter. Winter Games /SCPU fires a 20-cycle one-shot timer
// NMI per transfer chunk while keeping native-mode windows whose only
// protection is the NMI's timing. These tests drive the model through
// CC64Memory's real write8/read8/tickFast paths with a host bus standing in
// for the physical line. Pacing is off, so one emulated cycle equals one C64
// cycle and the $14 latch means a deadline 21 cycles out.
// ---------------------------------------------------------------------------

struct RetimeFixture
{
	CHostBus   bus;
	CC64Memory mem;

	RetimeFixture()
	{
		mem.attachBus( &bus );
		mem.reset();
	}

	// The Winter Games arm sequence, exactly: latch $0014, one-shot START,
	// then enable the timer-A NMI mask.
	void armWinterGames()
	{
		mem.write8( 0xDD04, 0x14 );
		mem.write8( 0xDD05, 0x00 );
		mem.write8( 0xDD0E, 0x09 );
		mem.tickFast( 4 );		// the STA $DD0E instruction completes
		mem.write8( 0xDD0D, 0x81 );
	}

	// Advance time and ask for the NMI the way the core does.
	bool nmiAfter( u32 cycles )
	{
		mem.tickFast( cycles );
		return mem.nmiFast();
	}
};

TEST( nmi_retime_delivers_at_the_emulated_deadline_not_the_sampled_line )
{
	RetimeFixture f;
	f.armWinterGames();

	// The physical line asserts EARLY -- the jitter this exists to erase.
	f.bus.m_NMI = true;

	// Ten cycles in: before the deadline, and the early line is ignored.
	CHECK( !f.nmiAfter( 10 ) );
	// Twenty-one cycles total: the timer underflows on the emulated clock.
	CHECK( f.nmiAfter( 11 ) );
}

TEST( nmi_retime_uses_emulated_cycles_even_with_a_hardware_clock )
{
	// RAD's ARM clock measures how quickly the interpreter happens to run,
	// not the 20MHz clock being emulated. C64Tests/16-NMITIMING caught the old
	// host deadline expiring after about half VICE's instruction count whenever
	// an armed NMI forced the core onto its slower fine-tick path.
	CHostBus bus;
	CC64Memory mem;
	mem.attachBus( &bus );
	mem.reset();
	mem.setPacing( 1400000000ULL, 20000000 );
	mem.m_IOStretchEnable = false;

	mem.write8( 0xDD04, 0x20 );
	mem.write8( 0xDD05, 0x00 );
	mem.write8( 0xDD0E, 0x09 );
	// Advance/finish directly: CHostBus's test clock advances only on bus
	// traffic, so enabling its pacer and calling tickFast would wait forever.
	mem.m_EmuCycles += 4;
	mem.cia2FinishArm();		// anchor after the STA completes
	mem.write8( 0xDD0D, 0x81 );

	CHECK_EQ( mem.m_CIA2TADeadline - mem.m_EmuCycles, 33u * 20u );
	mem.m_EmuCycles += 659;
	CHECK( !mem.nmiFast() );
	mem.m_EmuCycles += 1;
	CHECK( mem.nmiFast() );
}

TEST( nmi_retime_ack_ends_the_assertion_and_the_next_shot_is_a_new_edge )
{
	RetimeFixture f;
	f.armWinterGames();
	f.bus.m_NMI = true;
	CHECK( f.nmiAfter( 21 ) );

	// The handler acknowledges; the line (still sampled high -- the real
	// chip has not been serviced in this fake) no longer reaches the core:
	// the timers own it while the mask holds.
	f.mem.read8( 0xDD0D );
	f.bus.m_NMI = false;
	CHECK( !f.nmiAfter( 1 ) );

	// Re-arm for the next chunk; a fresh deadline, a fresh edge.
	f.mem.write8( 0xDD0E, 0x09 );
	f.mem.tickFast( 4 );		// the STA $DD0E instruction completes
	CHECK( !f.nmiAfter( 10 ) );
	CHECK( f.nmiAfter( 11 ) );
}

TEST( nmi_retime_unarmed_line_passes_through )
{
	RetimeFixture f;
	// No timer, no mask: RESTORE-style NMIs behave exactly as before.
	CHECK( !f.nmiAfter( 5 ) );
	f.bus.m_NMI = true;
	CHECK( f.nmiAfter( 1 ) );
	f.bus.m_NMI = false;
	CHECK( !f.nmiAfter( 1 ) );
}

TEST( nmi_retime_mixed_sources_fall_back_to_the_line )
{
	RetimeFixture f;
	f.mem.write8( 0xDD04, 0x14 );
	f.mem.write8( 0xDD05, 0x00 );
	f.mem.write8( 0xDD0E, 0x09 );
	f.mem.tickFast( 4 );		// the STA $DD0E instruction completes
	// Timer A AND FLAG enabled: FLAG events are invisible to the model, so
	// the timers cannot own the line; sampled delivery for everything.
	f.mem.write8( 0xDD0D, 0x91 );
	f.bus.m_NMI = true;
	CHECK( f.nmiAfter( 1 ) );		// immediate, jitter and all
}

TEST( nmi_retime_stop_and_mask_clear_disarm )
{
	RetimeFixture f;
	f.armWinterGames();
	f.mem.write8( 0xDD0E, 0x08 );	// STOP before underflow
	CHECK( !f.nmiAfter( 64 ) );		// never fires

	f.armWinterGames();
	f.mem.write8( 0xDD0D, 0x7F );	// mask cleared: source disabled
	CHECK( !f.nmiAfter( 64 ) );
}

TEST( nmi_retime_continuous_mode_fires_every_period )
{
	RetimeFixture f;
	f.mem.write8( 0xDD04, 0x14 );
	f.mem.write8( 0xDD05, 0x00 );
	f.mem.write8( 0xDD0E, 0x01 );	// continuous
	f.mem.tickFast( 4 );			// the STA $DD0E instruction completes
	f.mem.write8( 0xDD0D, 0x81 );

	CHECK( !f.nmiAfter( 10 ) );
	CHECK( f.nmiAfter( 11 ) );		// first underflow
	f.mem.read8( 0xDD0D );			// ack
	CHECK( !f.nmiAfter( 1 ) );
	CHECK( f.nmiAfter( 21 ) );		// second period, second edge
}

TEST( nmi_retime_disabled_restores_line_delivery )
{
	RetimeFixture f;
	f.mem.m_NMIRetimeEnable = false;
	f.armWinterGames();
	f.mem.cia2RecomputeNMIState();

	f.bus.m_NMI = true;
	CHECK( f.nmiAfter( 1 ) );		// the sampled line, as before the feature
	CHECK_EQ( f.mem.m_SynthNMIsDelivered, 0u );
}

#include "../../Source/CPU/W65C816/w65c816.h"
#include <initializer_list>

TEST( nmi_retime_keeps_the_nmi_out_of_a_native_mode_window )
{
	// The Winter Games shape, end to end on the real core: arm the 20-cycle
	// one-shot timer NMI, then pass through a short CLC/XCE native window
	// (its only protection being that the NMI lands outside it), with the
	// physical line asserting IN the window the way jittered sampling saw
	// it. Retimed delivery must hold the NMI to the deadline, which lies
	// beyond the SEC/XCE -- so the core takes it in emulation mode, where
	// the vectors are sane. This is the exact contract whose violation
	// BRK-stormed the real machine through KT's accidental $6C03 vector.
	CHostBus bus;
	CC64Memory mem;
	mem.attachBus( &bus );
	mem.reset();

	// All-RAM banking so $FFFA comes from RAM, like the game's loader phase.
	mem.write8( 0x0001, 0x35 );

	u16 a = 0x1000;
	auto emit = [&]( std::initializer_list<int> bytes )
	{
		for ( int b : bytes ) mem.write8( a++, (u8)b );
	};
	emit( { 0x78 } );							// SEI
	emit( { 0xA9, 0x14, 0x8D, 0x04, 0xDD } );	// LDA #$14  STA $DD04
	emit( { 0xA9, 0x00, 0x8D, 0x05, 0xDD } );	// LDA #$00  STA $DD05
	emit( { 0xA9, 0x09, 0x8D, 0x0E, 0xDD } );	// LDA #$09  STA $DD0E  (one-shot START)
	emit( { 0xA9, 0x81, 0x8D, 0x0D, 0xDD } );	// LDA #$81  STA $DD0D  (NMI enable)
	// The window must COMPLETE before the deadline -- that is the contract
	// the game's engineering establishes and the retimer preserves. Arm at
	// ~cycle 16, deadline 21 later at 37; this window spans cycles 26-34.
	const u16 windowEntry = a;
	emit( { 0x18, 0xFB } );						// CLC XCE      -> native
	emit( { 0x38, 0xFB } );						// SEC XCE      -> emulation
	const u16 windowExit = a;
	for ( u32 i = 0; i < 40; i++ ) emit( { 0xEA } );	// post-window run
	emit( { 0x4C, (u8)( ( a + 3 ) & 0xFF ), (u8)( ( a + 3 ) >> 8 ) } );	// JMP next
	// NMI handler: just RTI; taking it at all is what the test observes.
	mem.write8( 0x2000, 0x40 );
	mem.write8( 0xFFFA, 0x00 );
	mem.write8( 0xFFFB, 0x20 );

	CW65C816 cpu;
	cpu.attachBus( &mem );
	cpu.m_PC = 0x1000; cpu.m_PBR = 0; cpu.m_E = true;
	cpu.m_S = 0x01F0;

	u64 lastNMIs = cpu.m_NMIsTaken;
	bool tookInNative = false, tookInEmulation = false;
	for ( u32 step = 0; step < 400 && !( tookInNative || tookInEmulation ); step++ )
	{
		// The moment the core enters the native window, the "physical" line
		// asserts -- the early, jittered arrival.
		if ( cpu.m_PC >= windowEntry && cpu.m_PC < windowExit && !cpu.m_E )
			bus.m_NMI = true;

		cpu.step();

		if ( cpu.m_NMIsTaken != lastNMIs )
		{
			lastNMIs = cpu.m_NMIsTaken;
			if ( cpu.m_E ) tookInEmulation = true;
			else           tookInNative    = true;
		}
	}

	CHECK( tookInEmulation );
	CHECK( !tookInNative );
	CHECK_EQ( mem.m_SynthNMIsDelivered, 1u );
}

TEST( nmi_retime_serial_polls_sustain_the_1mhz_hold_for_the_arm )
{
	// Winter Games arms its timer NMI in the middle of a $DD00 poll phase.
	// A real SuperCPU's auto-slow re-arms on EVERY serial-port access, polls
	// included, so the arm and its aftermath run at 1MHz and the NMI lands a
	// handful of instructions later, exactly where the handler expects. Our
	// edge-armed hold had expired there; polls must sustain the SPEED hold.
	CHostBus bus;
	CC64Memory mem;
	mem.attachBus( &bus );
	mem.reset();
	mem.setPacing( 0, 20000000 );

	// Serial lines idle-high: these are MERE polls, no receive evidence.
	bus.m_Memory[ 0xDD00 ] = 0xFF;
	// An old edge, long expired; the KERNAL keeps polling while the drive
	// steps between sectors.
	mem.write8( 0xDD00, 0x17 );
	mem.tickFast( 110000 );				// the edge-armed hold fully decays
	CHECK( !mem.iecThrottleActive() );
	for ( u32 i = 0; i < 300; i++ )
	{
		mem.tickFast( 700 );			// polls tens of microseconds apart
		mem.read8( 0xDD00 );
	}
	CHECK( mem.iecThrottleActive() );	// sustained by polls alone

	mem.write8( 0xDD04, 0x14 );
	mem.write8( 0xDD05, 0x00 );
	mem.write8( 0xDD0E, 0x09 );
	mem.tickFast( 4 );		// the STA $DD0E instruction completes
	mem.write8( 0xDD0D, 0x81 );

	// Deadline in HELD units: 21 emulated cycles, the 1MHz landing zone.
	mem.tickFast( 10 );
	CHECK( !mem.nmiFast() );
	mem.tickFast( 12 );
	CHECK( mem.nmiFast() );
	CHECK_EQ( mem.m_SynthNMIsDelivered, 1u );
}

TEST( nmi_retime_poll_hold_decays_quickly_without_polls )
{
	// The top-up is short by design: a game reading $DD00 once per frame
	// pays about a millisecond, not the 100ms edge-hold -- no latch-up.
	CHostBus bus;
	CC64Memory mem;
	mem.attachBus( &bus );
	mem.reset();
	mem.setPacing( 0, 20000000 );
	bus.m_Memory[ 0xDD00 ] = 0xFF;		// idle lines: a poll, not a receive

	mem.read8( 0xDD00 );				// one isolated poll from cold
	CHECK( mem.iecThrottleActive() );
	mem.tickFast( 1100 );
	CHECK( !mem.iecThrottleActive() );	// gone within ~a millisecond
}

TEST( explicit_1mhz_poll_remains_visible_as_iec_activity_evidence )
{
	// Test Drive explicitly selects 1MHz before its bit-timed loader. That
	// makes an automatic speed hold redundant, but a tight BIT $DD00 loop is
	// still positive evidence that a deferred 9K VIC-map replay must wait.
	CHostBus bus;
	CC64Memory mem;
	mem.attachBus( &bus );
	mem.reset();
	mem.setPacing( 0, 1000000 );
	bus.m_Memory[ 0xDD00 ] = 0xFF;

	mem.read8( 0xDD00 );
	CHECK( mem.iecPollActive() );
	CHECK( !mem.iecThrottleActive() );	// already slow; no fallback active

	mem.tickFast( 1100 );
	CHECK( !mem.iecPollActive() );
	CHECK( !mem.iecThrottleActive() );
}

TEST( cia2_diagnostic_snapshot_does_not_rearm_the_1mhz_poll_hold )
{
	CHostBus bus;
	CC64Memory mem;
	mem.attachBus( &bus );
	mem.reset();
	mem.setPacing( 0, 20000000 );

	u8 portA = 0, ddra = 0;
	CHECK( !mem.trackedCIA2Snapshot( portA, ddra ) );

	// Establish the three pieces of tracked state: DDRA, the hidden output
	// latch, and one physical input sample. With outputs $17 and PA6/PA7 high,
	// the reconstructed port read is $D7.
	mem.write8( 0xDD02, 0x3F );
	mem.write8( 0xDD00, 0x17 );
	bus.m_Memory[ 0xDD00 ] = 0xFF;
	mem.read8( 0xDD00 );
	mem.tickFast( 110000 );
	CHECK( !mem.iecThrottleActive() );

	const u64 ioReads = mem.m_IOReads;
	CHECK( mem.trackedCIA2Snapshot( portA, ddra ) );
	CHECK_EQ( portA, (u8)0xD7 );
	CHECK_EQ( ddra, (u8)0x3F );
	CHECK_EQ( mem.m_IOReads, ioReads );
	CHECK( !mem.iecThrottleActive() );
}

TEST( c64_clock_preserves_elapsed_time_across_explicit_speed_changes )
{
	CHostBus bus;
	CC64Memory mem;
	mem.attachBus( &bus );
	mem.reset();

	mem.setPacing( 0, 20000000 );
	mem.tickFast( 2000 );
	CHECK_EQ( mem.c64Now(), 100u );

	mem.setPacing( 0, 1000000 );
	CHECK_EQ( mem.c64Now(), 100u );
	mem.tickFast( 100 );
	CHECK_EQ( mem.c64Now(), 200u );

	mem.setPacing( 0, 20000000 );
	CHECK_EQ( mem.c64Now(), 200u );
	mem.tickFast( 200 );
	CHECK_EQ( mem.c64Now(), 210u );
}

TEST( c64_clock_does_not_drop_fractional_time_without_a_rate_change )
{
	CC64Memory mem;
	mem.reset();
	mem.setPacing( 0, 20000000 );

	mem.tickFast( 10 );
	CHECK_EQ( mem.c64Now(), 0u );
	mem.cia2OnRateChange();
	mem.tickFast( 10 );
	CHECK_EQ( mem.c64Now(), 1u );
}

TEST( c64_clock_preserves_elapsed_time_across_iec_poll_hold )
{
	CHostBus bus;
	CC64Memory mem;
	mem.attachBus( &bus );
	mem.reset();
	mem.setPacing( 0, 20000000 );
	bus.m_Memory[ 0xDD00 ] = 0xFF;

	mem.tickFast( 2000 );
	CHECK_EQ( mem.c64Now(), 100u );
	mem.read8( 0xDD00 );
	CHECK( mem.iecThrottleActive() );
	CHECK_EQ( mem.c64Now(), 100u );

	mem.tickFast( 100 );
	CHECK_EQ( mem.c64Now(), 200u );
	mem.tickFast( 900 );
	CHECK( !mem.iecThrottleActive() );
	CHECK_EQ( mem.c64Now(), 1100u );

	mem.tickFast( 200 );
	CHECK_EQ( mem.c64Now(), 1110u );
}

TEST( vic_log_is_mode1_gated_and_captures_mirrors_and_bank )
{
	CHostBus bus;
	CC64Memory mem;
	mem.attachBus( &bus );
	mem.reset();
	mem.setPacing( 0, 20000000 );

	mem.write8( 0xD020, 2 );
	CHECK_EQ( mem.vicLogHead(), 0u );
	CHECK_EQ( mem.vicRegister( 0x20 ), 2 );

	mem.enableVICLog( true );
	mem.write8( 0xD060, 6 );		// mirror of $D020
	mem.write8( 0xD800, 0xFA );
	mem.write8( 0xDD02, 0x03 );
	mem.write8( 0xDD00, 0xFE );		// inverted bank select -> bank 1

	CHECK_EQ( mem.vicRegister( 0x20 ), 6 );
	CHECK_EQ( bus.m_Memory[ 0xD060 ], 6 );
	// Colour RAM is copied once per completed frame. Logging every colour-cell
	// write would let an ordinary 1000-cell clear overwrite the 2048-event
	// raster register ring before core 1 could consume it.
	CHECK_EQ( mem.vicLogHead(), 3u );
	const VICRegWrite *events = mem.vicLog();
	CHECK_EQ( vicLogRegister( events[ 0 ] ), 0x20 );
	CHECK_EQ( events[ 0 ].value, 6 );
	CHECK_EQ( vicLogRegister( events[ 1 ] ), 0x40 );
	CHECK_EQ( events[ 1 ].value, 0 );
	CHECK_EQ( vicLogRegister( events[ 2 ] ), 0x40 );
	CHECK_EQ( events[ 2 ].value, 1 );

	mem.enableVICLog( false );
	mem.write8( 0xD021, 3 );
	CHECK_EQ( mem.vicLogHead(), 3u );
}

TEST( vic_log_generation_changes_when_capture_or_machine_resets )
{
	CHostBus bus;
	CC64Memory mem;
	mem.attachBus( &bus );
	mem.reset();
	const u32 afterReset = mem.vicLogGeneration();

	mem.enableVICLog( true );
	const u32 afterEnable = mem.vicLogGeneration();
	CHECK( afterEnable != afterReset );
	mem.write8( 0xD020, 2 );
	CHECK_EQ( mem.vicLogHead(), 1u );

	mem.reset();
	CHECK( mem.vicLogGeneration() != afterEnable );
	CHECK_EQ( mem.vicLogHead(), 0u );
}

TEST( vic_log_precise_irq_anchor_keeps_the_full_compare_line )
{
	CHostBus bus;
	CC64Memory mem;
	mem.attachBus( &bus );
	mem.reset();
	mem.setPacing( 0, 20000000 );

	// Establish a nine-bit compare before capture starts. The compare shadow
	// must already be authoritative when VIDEO_MODE 1 is enabled.
	mem.write8( 0xD011, 0x9B );
	mem.write8( 0xD012, 0x34 );
	mem.enableVICLog( true );

	mem.write8( 0xD019, 0x01 );		// IRQ acknowledge at line $134
	mem.tickFast( 20 );
	mem.write8( 0xD020, 2 );
	CHECK_EQ( mem.vicLogHead(), 2u );
	const VICRegWrite *events = mem.vicLog();
	CHECK( vicLogHasRasterLine( events[ 0 ] ) );
	CHECK( vicLogHasRasterLine( events[ 1 ] ) );
	CHECK_EQ( vicLogRasterLine( events[ 0 ] ), 0x134 );
	CHECK_EQ( vicLogRasterLine( events[ 1 ] ), 0x134 );

	const u64 precise = mem.rasterAnchor();
	CHECK( ( precise >> 63 ) != 0 );
	CHECK_EQ( precise & 0x1FF, 0x134u );
	mem.noteRasterCoarse( 7 );
	CHECK_EQ( mem.rasterAnchor(), precise );
}

TEST( vic_log_anchors_only_the_first_compare_write_in_a_handler )
{
	CHostBus bus;
	CC64Memory mem;
	mem.attachBus( &bus );
	mem.reset();
	mem.setPacing( 0, 20000000 );
	mem.write8( 0xD011, 0x9B );
	mem.write8( 0xD012, 0x34 );		// current compare $134
	mem.enableVICLog( true );

	// Low and high halves of the next compare are written together. Both
	// writes belong to the handler for $134, not to the newly assembled $050.
	mem.write8( 0xD012, 0x50 );
	mem.tickFast( 20 );
	mem.write8( 0xD011, 0x1B );
	const VICRegWrite *events = mem.vicLog();
	CHECK_EQ( mem.vicLogHead(), 2u );
	CHECK_EQ( vicLogRasterLine( events[ 0 ] ), 0x134 );
	CHECK_EQ( vicLogRasterLine( events[ 1 ] ), 0x134 );

	// A later handler is outside the 256-C64-cycle guard and anchors to the
	// compare assembled above.
	mem.tickFast( 20 * 300 );
	mem.write8( 0xD012, 0x60 );
	CHECK_EQ( mem.vicLogHead(), 3u );
	CHECK_EQ( vicLogRasterLine( events[ 2 ] ), 0x050 );
}

TEST( nmi_retime_deadline_survives_an_explicit_speed_change_mid_count )
{
	// Winter Games' arm sequence ends with a $D07B write: timer armed in
	// turbo, machine dropped to 1MHz ONE INSTRUCTION LATER. The deadline was
	// computed as 21 C64 cycles = 420 turbo emu-cycles; after the switch
	// those same C64 cycles are 21 emu-cycles. Without re-denomination the
	// NMI fires 20x late in C64 time -- landing, deterministically, inside
	// the native window it was engineered to miss.
	CHostBus bus;
	CC64Memory mem;
	mem.attachBus( &bus );
	mem.reset();
	mem.setPacing( 0, 20000000 );	// turbo: 20 emu cycles per C64 cycle

	mem.write8( 0xDD04, 0x14 );
	mem.write8( 0xDD05, 0x00 );
	mem.write8( 0xDD0E, 0x09 );
	mem.tickFast( 4 );		// the STA $DD0E instruction completes
	mem.write8( 0xDD0D, 0x81 );

	mem.setPacing( 0, 1000000 );	// the $D07B: explicit 1MHz
	CHECK( mem.m_CIA2Rescales >= 1u );

	// 21 C64 cycles at the new rate: fire at ~21 emulated cycles, not 420.
	mem.tickFast( 10 );
	CHECK( !mem.nmiFast() );
	mem.tickFast( 12 );
	CHECK( mem.nmiFast() );
	CHECK_EQ( mem.m_SynthNMIsDelivered, 1u );
}

TEST( nmi_retime_deadline_survives_a_speed_up_mid_count )
{
	// The mirror image: armed at 1MHz, then turbo selected. 21 C64 cycles
	// become 420 emu-cycles of code; firing at emu-cycle 21 would be 20x
	// EARLY, interrupting code that has barely begun.
	CHostBus bus;
	CC64Memory mem;
	mem.attachBus( &bus );
	mem.reset();
	mem.setPacing( 0, 1000000 );

	mem.write8( 0xDD04, 0x14 );
	mem.write8( 0xDD05, 0x00 );
	mem.write8( 0xDD0E, 0x09 );
	mem.tickFast( 4 );		// the STA $DD0E instruction completes
	mem.write8( 0xDD0D, 0x81 );

	mem.setPacing( 0, 20000000 );

	mem.tickFast( 100 );
	CHECK( !mem.nmiFast() );
	mem.tickFast( 250 );
	CHECK( !mem.nmiFast() );
	mem.tickFast( 100 );
	CHECK( mem.nmiFast() );
}

TEST( nmi_retime_explicit_speed_select_clears_the_poll_hold )
{
	// Winter Games' OTHER arm site: arm under the poll-sustained 1MHz, then
	// write $D07B to run the timed section at turbo. Turbo may already be
	// the nominal selection -- no speed change, no speed hook -- yet the
	// declaration must still end the advisory poll-slow and re-denominate
	// the armed deadline, or the code runs 20x dilated into a native window
	// while the NMI arrives dead on time.
	CHostBus bus;
	CC64Memory mem;
	CSuperCPURegisters regs;
	mem.attachBus( &bus );
	mem.setIOInterceptor( &regs );
	regs.reset();
	mem.reset();
	mem.setPacing( 0, 20000000 );
	mem.m_IOStretchEnable = false;		// isolate deadline rescaling from bus cost
	bus.m_Memory[ 0xDD00 ] = 0xFF;		// idle lines: polls, not receives

	mem.read8( 0xDD00 );				// the KERNAL's poll loop
	CHECK( mem.iecThrottleActive() );

	mem.write8( 0xDD04, 0x14 );			// arm under the poll hold: factor 1
	mem.write8( 0xDD05, 0x00 );
	mem.write8( 0xDD0E, 0x09 );
	mem.tickFast( 4 );		// the STA $DD0E instruction completes
	mem.write8( 0xDD0D, 0x81 );

	mem.write8( 0xD07B, 0x01 );			// the game declares its speed

	CHECK( !mem.iecThrottleActive() );	// advisory hold gone
	CHECK( mem.m_CIA2Rescales >= 1u );	// deadline moved into turbo units

	// ~21 C64 cycles at full speed: ~420 emulated cycles, not 21.
	mem.tickFast( 30 );
	CHECK( !mem.nmiFast() );
	mem.tickFast( 350 );
	CHECK( !mem.nmiFast() );
	mem.tickFast( 60 );
	CHECK( mem.nmiFast() );
	CHECK_EQ( mem.m_SynthNMIsDelivered, 1u );
}

#include "../../Source/SuperCPU/supercpu.h"

TEST( scpu_c128_host_uses_stable_ff_presets_or_an_explicit_mode )
{
	static u8 scpuROM[ 0x20000 ];
	static u8 basic[ C64_BASIC_SIZE ];
	static u8 kernal[ C64_KERNAL_SIZE ];
	static u8 chargen[ 4096 ];
	std::memset( scpuROM, 0xEA, sizeof scpuROM );
	std::memset( basic, 0xEA, sizeof basic );
	std::memset( kernal, 0xEA, sizeof kernal );
	std::memset( chargen, 0, sizeof chargen );

	{
		static CHostBus nativeBus;
		static CSuperCPU nativeSCPU;
		nativeBus.m_Signals.machine = MACHINE_C128;
		nativeBus.m_Memory[ 0xFF01 ] = 0x3F;
		nativeBus.m_Memory[ 0xFF02 ] = 0x7F;
		nativeBus.m_Memory[ 0xFF03 ] = 0x01;
		nativeBus.m_Memory[ 0xFF04 ] = 0x41;
		nativeSCPU.memoryMap().setROM( scpuROM, sizeof scpuROM );
		nativeSCPU.setBasicROM( basic );
		nativeSCPU.setKernalROM( kernal );
		nativeSCPU.setCharROM( chargen );
		CHECK( nativeSCPU.init( &nativeBus, SCPU_CORE_65816, SCPU_SIMM_NONE ) );
		CHECK( nativeSCPU.registers().scpu128Mode() );
		CHECK_EQ( nativeBus.m_Memory[ 0xDD02 ], (u8)0x00 );
	}

	{
		static CHostBus c64Bus;
		static CSuperCPU c64SCPU;
		c64Bus.m_Signals.machine = MACHINE_C128;
		// In C64 mode this window is ordinary KERNAL ROM, not the MMU presets.
		c64Bus.m_Memory[ 0xFF01 ] = 0x8D;
		c64Bus.m_Memory[ 0xFF02 ] = 0x07;
		c64Bus.m_Memory[ 0xFF03 ] = 0xDD;
		c64Bus.m_Memory[ 0xFF04 ] = 0x4C;
		c64SCPU.memoryMap().setROM( scpuROM, sizeof scpuROM );
		c64SCPU.setBasicROM( basic );
		c64SCPU.setKernalROM( kernal );
		c64SCPU.setCharROM( chargen );
		CHECK( c64SCPU.init( &c64Bus, SCPU_CORE_65816, SCPU_SIMM_NONE ) );
		CHECK( !c64SCPU.registers().scpu128Mode() );
		CHECK_EQ( c64Bus.m_Memory[ 0xDD02 ], (u8)0x3F );
	}

	{
		static CHostBus forcedBus;
		static CSuperCPU forcedSCPU;
		forcedBus.m_Signals.machine = MACHINE_C128;
		// Even a native-looking bus signature must not override the explicit
		// bring-up selection used by the real-hardware C64-mode test.
		forcedBus.m_Memory[ 0xFF01 ] = 0x3F;
		forcedBus.m_Memory[ 0xFF02 ] = 0x7F;
		forcedBus.m_Memory[ 0xFF03 ] = 0x01;
		forcedBus.m_Memory[ 0xFF04 ] = 0x41;
		forcedSCPU.setC128Mode( SCPU_C128_MODE_C64 );
		forcedSCPU.memoryMap().setROM( scpuROM, sizeof scpuROM );
		forcedSCPU.setBasicROM( basic );
		forcedSCPU.setKernalROM( kernal );
		forcedSCPU.setCharROM( chargen );
		CHECK( forcedSCPU.init( &forcedBus, SCPU_CORE_65816, SCPU_SIMM_NONE ) );
		CHECK( !forcedSCPU.registers().scpu128Mode() );
		CHECK_EQ( forcedBus.m_Memory[ 0xDD02 ], (u8)0x3F );
	}
}

TEST( nmi_retime_delivery_is_punctual_through_the_run_loop )
{
	// The full delivery chain through the REAL run loop, whose batching and
	// per-slice interrupt sampling are what actually delayed Winter Games'
	// NMI on hardware. The game's contract: a 20-cycle one-shot NMI with a
	// throwaway RTI handler, armed a handful of 1MHz instructions before a
	// native-mode section containing a LONG interruptible MVN -- the NMI
	// must land in the plain code right after the arm (emulation mode,
	// where its vectors are sane), never inside the native section.
	//
	// Two distinguishable handlers make the landing mode observable: the
	// emulation vector counts at $0FFD, the native vector counts at $0FFE.
	static CHostBus bus;
	static CSuperCPU scpu;
	static u8 kernal[ 8192 ], basic[ 8192 ], chargen[ 4096 ];
	std::memset( kernal, 0xAA, sizeof kernal );
	std::memset( basic, 0xAA, sizeof basic );
	std::memset( chargen, 0, sizeof chargen );
	scpu.setKernalROM( kernal );
	scpu.setBasicROM( basic );
	scpu.setCharROM( chargen );
	CHECK( scpu.init( &bus, SCPU_CORE_65816, SCPU_SIMM_NONE ) );

	CC64Memory &mem = scpu.memory();
	mem.write8( 0x0001, 0x35 );			// all-RAM banking, like the loader
	bus.m_Memory[ 0xDD00 ] = 0xFF;

	u16 a = 0x1000;
	auto emit = [&]( int b ) { mem.write8( a++, (u8)b ); };
	auto emit3 = [&]( int x, int y, int z ) { emit( x ); emit( y ); emit( z ); };
	emit( 0x78 );						// SEI
	emit( 0xA9 ); emit( 0x14 ); emit3( 0x8D, 0x04, 0xDD );
	emit( 0xA9 ); emit( 0x00 ); emit3( 0x8D, 0x05, 0xDD );
	emit( 0xA9 ); emit( 0x09 ); emit3( 0x8D, 0x0E, 0xDD );
	emit( 0xA9 ); emit( 0x81 ); emit3( 0x8D, 0x0D, 0xDD );
	for ( u32 i = 0; i < 12; i++ ) emit( 0xEA );	// the designed landing zone
	emit( 0x18 ); emit( 0xFB );			// CLC XCE: native
	for ( u32 i = 0; i < 300; i++ ) emit( 0xEA );	// long native section (the MVN)
	emit( 0x38 ); emit( 0xFB );			// SEC XCE: back
	const u16 loop = a;
	emit3( 0x4C, loop & 0xFF, loop >> 8 );

	mem.write8( 0x2000, 0xEE ); mem.write8( 0x2001, 0xFD ); mem.write8( 0x2002, 0x0F );
	mem.write8( 0x2003, 0x40 );			// INC $0FFD / RTI      (emulation NMI)
	mem.write8( 0x2100, 0xEE ); mem.write8( 0x2101, 0xFE ); mem.write8( 0x2102, 0x0F );
	mem.write8( 0x2103, 0xDB );			// INC $0FFE / STP      (native NMI)
	mem.write8( 0xFFFA, 0x00 ); mem.write8( 0xFFFB, 0x20 );
	mem.write8( 0xFFEA, 0x00 ); mem.write8( 0xFFEB, 0x21 );
	mem.write8( 0x0FFD, 0x00 ); mem.write8( 0x0FFE, 0x00 );

	CW65C816 *c = scpu.core65816();
	c->m_PC = 0x1000; c->m_PBR = 0; c->m_E = true;
	c->m_S = 0x01F0;

	// The device state at the crash: the KERNAL's poll loop has the machine
	// under the 1MHz poll hold, so the arm computes a 21-cycle deadline.
	mem.read8( 0xDD00 );

	// Large budgets exercise the batch path; re-entry after a run break is
	// how the exposed edge reaches the core, exactly as on the device.
	for ( u32 i = 0; i < 40 && c->m_NMIsTaken == 0; i++ )
		c->run( 2000 );
	for ( u32 i = 0; i < 10; i++ )
		c->run( 2000 );					// let the RTI path settle

	CHECK_EQ( c->m_NMIsTaken, 1u );
	CHECK_EQ( mem.m_RAM[ 0x0FFD ], 1 );	// landed in emulation mode
	CHECK_EQ( mem.m_RAM[ 0x0FFE ], 0 );	// never inside the native section
	CHECK( !c->m_Stopped );
}

TEST( nmi_in_native_mode_dispatches_natively )
{
	// Native NMIs are never delayed. On a SuperCPU the memory map reroutes the
	// vector fetch into EPROM; this plain-bus test checks the underlying 65816
	// contract directly through $00FFEA.
	CHostBus bus;
	CC64Memory mem;
	mem.attachBus( &bus );
	mem.reset();
	mem.write8( 0x0001, 0x35 );

	u16 a = 0x1000;
	auto emit = [&]( int b ) { mem.write8( a++, (u8)b ); };
	emit( 0x18 ); emit( 0xFB );
	for ( u32 i = 0; i < 10; i++ ) emit( 0xEA );

	mem.write8( 0x2100, 0xEE ); mem.write8( 0x2101, 0xFE ); mem.write8( 0x2102, 0x0F );
	mem.write8( 0x2103, 0x40 );
	mem.write8( 0xFFEA, 0x00 ); mem.write8( 0xFFEB, 0x21 );

	CW65C816 cpu;
	cpu.attachBus( &mem );
	cpu.m_PC = 0x1000; cpu.m_PBR = 0; cpu.m_E = true; cpu.m_S = 0x01F0;

	cpu.step(); cpu.step();				// into native mode
	bus.m_NMI = true;
	for ( u32 i = 0; i < 6 && cpu.m_NMIsTaken == 0; i++ )
		cpu.step();
	for ( u32 i = 0; i < 4; i++ ) cpu.step();		// let the handler run
	CHECK_EQ( cpu.m_NMIsTaken, 1u );
	CHECK_EQ( mem.m_RAM[ 0x0FFE ], 1 );	// dispatched through the native vector
}

TEST( nmi_native_compatibility_hold_releases_in_emulation_mode )
{
	CHostBus bus;
	CC64Memory mem;
	mem.attachBus( &bus );
	mem.reset();
	mem.write8( 0x0001, 0x35 );
	mem.write8( 0x1000, 0x18 ); mem.write8( 0x1001, 0xFB ); // CLC/XCE native
	mem.write8( 0x1002, 0xEA );
	mem.write8( 0x1003, 0x38 ); mem.write8( 0x1004, 0xFB ); // SEC/XCE emu
	mem.write8( 0x2000, 0x40 );
	mem.write8( 0xFFFA, 0x00 ); mem.write8( 0xFFFB, 0x20 );

	CW65C816 cpu;
	cpu.attachBus( &mem );
	cpu.m_DeferNativeNMI = true;
	cpu.m_PC = 0x1000; cpu.m_PBR = 0; cpu.m_E = true; cpu.m_S = 0x01F0;
	cpu.step(); cpu.step();
	bus.m_NMI = true;
	cpu.step();
	CHECK_EQ( cpu.m_NMIsTaken, 0u );
	CHECK( cpu.m_NMIsDeferred > 0 );
	cpu.step(); cpu.step();
	CHECK( cpu.m_E );
	for ( u32 i = 0; i < 4 && cpu.m_NMIsTaken == 0; i++ ) cpu.step();
	CHECK_EQ( cpu.m_NMIsTaken, 1u );
}

// ---------------------------------------------------------------------------
// Interrupt vector reroute
//
// A real SuperCPU does not fetch interrupt vectors from the C64 map while it
// is acting as an accelerator: it redirects the fetch into its own EPROM at
// $F80000+vector, which carries a genuine 65816 vector table. VICE models
// this as scpu64_interrupt_reroute() hooked into LOAD_INT_ADDR. It is why the
// KERNAL images can hold the ordinary C64 jump table across $FFE4-$FFEF and
// why a 65816 program may keep its own code at those addresses.
// ---------------------------------------------------------------------------

struct RerouteFixture
{
	CHostBus            bus;
	CC64Memory          bank0;
	CFastRAM            simm;
	CSuperCPUMemoryMap  map;
	CSuperCPURegisters  regs;
	u8                  bank1[ 0x10000 ];
	u8                  rom[ 0x20000 ];

	RerouteFixture()
	{
		std::memset( bank1, 0, sizeof bank1 );
		std::memset( rom, 0, sizeof rom );
		// EPROM vectors, deliberately distinct from the C64 map's.
		rom[ 0xFFEA ] = 0x34; rom[ 0xFFEB ] = 0x12;		// native NMI -> $1234
		rom[ 0xFFFE ] = 0x78; rom[ 0xFFFF ] = 0x56;		// emu IRQ/BRK -> $5678
		// What the accelerator's KERNAL window holds at the same addresses.
		bank1[ 0xFFEA ] = 0xAD; bank1[ 0xFFEB ] = 0xDE;	// -> $DEAD
		bank1[ 0xFFFE ] = 0xEF; bank1[ 0xFFFF ] = 0xBE;	// -> $BEEF

		bank0.attachBus( &bus );
		bank0.setIOInterceptor( &regs );
		bank0.setROMShadow( bank1 );
		regs.trackKernalShadow( &bank0.m_KernalShadowBase );
		// trackKernalShadow() on its own is only half of what the live machine
		// does: CSuperCPU::initialize() also installs the map hook, because a
		// raw-u32* write to m_KernalShadowBase is invisible to CC64Memory's
		// read-decode table. This is the only hand-wired fixture that drives a
		// real CW65C816 through readFast(), so without this a $D0B2/$D0Bx
		// toggle in a future test here would fetch from the wrong 8K with
		// nothing to catch it.
		regs.setMapHook( []( void *ctx )
			{ ( (CC64Memory *)ctx )->rebuildReadDecode(); }, &bank0 );
		regs.reset();
		simm.init( SCPU_SIMM_NONE );
		map.attachBank0( &bank0 );
		map.attachFastRAM( &simm );
		bank0.reset();
		map.reset();
		map.setROM( rom, sizeof rom );
	}
};

TEST( vector_reroute_applies_in_native_mode )
{
	RerouteFixture f;
	// Default banking has the KERNAL window at page $FF.
	CHECK( f.map.interruptRerouteActive( false ) );		// native: always
	CHECK_EQ( f.map.rerouteVector( 0xFFEA ), 0x1234 );	// from the EPROM
	CHECK_EQ( f.bank0.read8( 0xFFEA ), 0xAD );			// map still reads KT
}

TEST( vector_reroute_stays_out_of_ordinary_emulation_mode )
{
	RerouteFixture f;
	// Emulation mode with the card idle is an ordinary C64: the KERNAL's own
	// vectors are used, exactly as before this existed.
	CHECK( !f.regs.hardwareRegsEnabled() );
	CHECK( !f.map.interruptRerouteActive( true ) );

	// ...but with the hardware registers open, the card has taken over.
	f.bank0.write8( SCPU_REG_HWREGS_ENABLE, 0 );
	CHECK( f.regs.hardwareRegsEnabled() );
	CHECK( f.map.interruptRerouteActive( true ) );
	CHECK_EQ( f.map.rerouteVector( 0xFFFE ), 0x5678 );
}

TEST( vector_reroute_needs_the_accelerator_kernal_window )
{
	RerouteFixture f;
	// With RAM banked in at $E000-$FFFF the vectors are the program's own and
	// page $FF is C64 DRAM, so nothing is rerouted -- in either mode.
	f.bank0.write8( 0x0001, 0x35 );
	CHECK( !f.map.interruptRerouteActive( false ) );
	CHECK( !f.map.interruptRerouteActive( true ) );
}

TEST( vector_reroute_disabled_by_config )
{
	RerouteFixture f;
	f.bank0.m_VectorRerouteEnable = false;
	CHECK( !f.map.interruptRerouteActive( false ) );
}

TEST( vector_reroute_reaches_the_core )
{
	// End to end: a native-mode interrupt taken by the real core must land on
	// the EPROM's vector, not the KERNAL window's.
	RerouteFixture f;
	CW65C816 cpu;
	cpu.attachFastBus( &f.map );
	cpu.m_PC = 0x1000; cpu.m_PBR = 0; cpu.m_E = true; cpu.m_S = 0x01F0;

	// Into native mode: CLC XCE.
	f.bank0.write8( 0x1000, 0x18 );
	f.bank0.write8( 0x1001, 0xFB );
	cpu.step(); cpu.step();
	CHECK( !cpu.m_E );

	f.bus.m_NMI = true;
	for ( u32 i = 0; i < 8 && cpu.m_NMIsTaken == 0; i++ )
		cpu.step();
	CHECK_EQ( cpu.m_NMIsTaken, 1u );
	CHECK_EQ( cpu.m_PC, 0x1234 );				// the EPROM's vector
	CHECK( f.bank0.m_VectorReroutes > 0 );
}

TEST( io_stretch_quantises_a_poll_loop_to_c64_cycles )
{
	// The property that matters, and the one we had wrong: on a real
	// SuperCPU a C64-bus access costs a WHOLE C64 cycle, so a poll loop
	//     LDA $D012 / CMP #x / BNE
	// takes exactly one C64 cycle per iteration -- the processor's own nine
	// cycles disappear inside the stall. 1000 iterations are 1000
	// microseconds, not 450. VICE charges this in
	// scpu64_clock_read_stretch_io(); we charged nothing, and a CIA timer
	// armed for 21us landed at a completely different instruction.
	CHostBus bus;
	CC64Memory mem;
	mem.attachBus( &bus );
	mem.reset();
	mem.setPacing( 0, 20000000 );			// turbo: 20 emulated cycles per C64 cycle

	u16 loop = 0x1000;
	mem.write8( loop + 0, 0xAD ); mem.write8( loop + 1, 0x12 ); mem.write8( loop + 2, 0xD0 );
	mem.write8( loop + 3, 0xC9 ); mem.write8( loop + 4, 0xFF );	// never equal
	mem.write8( loop + 5, 0xD0 ); mem.write8( loop + 6, 0xF9 );

	CW65C816 cpu;
	cpu.attachBus( &mem );
	cpu.m_PC = loop; cpu.m_PBR = 0; cpu.m_E = true; cpu.m_S = 0x01F0;

	const u64 start = mem.m_EmuCycles;
	for ( u32 i = 0; i < 100; i++ )
	{
		cpu.step();							// LDA $D012
		cpu.step();							// CMP #$00
		cpu.step();							// BNE
	}
	const u64 elapsed = mem.m_EmuCycles - start;

	// One C64 cycle per iteration -- 100 accesses stretched to 20 emulated
	// cycles each -- plus the last iteration's CMP and BNE, which have no
	// following access to be absorbed into.
	CHECK_EQ( elapsed, 2005u );
	CHECK( mem.m_IOStretchCycles > 0 );

	// Without the stretch the same loop would be nine cycles an iteration --
	// the error that made I/O-bound code run several times too fast.
	CHostBus bus2;
	CC64Memory mem2;
	mem2.attachBus( &bus2 );
	mem2.reset();
	mem2.setPacing( 0, 20000000 );
	mem2.m_IOStretchEnable = false;
	for ( u16 i = 0; i < 7; i++ ) mem2.write8( loop + i, mem.m_RAM[ loop + i ] );
	CW65C816 cpu2;
	cpu2.attachBus( &mem2 );
	cpu2.m_PC = loop; cpu2.m_PBR = 0; cpu2.m_E = true; cpu2.m_S = 0x01F0;
	const u64 start2 = mem2.m_EmuCycles;
	for ( u32 i = 0; i < 100; i++ ) { cpu2.step(); cpu2.step(); cpu2.step(); }
	CHECK_EQ( mem2.m_EmuCycles - start2, 900u );
}

TEST( io_stretch_does_not_apply_at_1mhz )
{
	// At 1MHz the processor already runs at the bus rate: nothing to stall
	// for, and the serial paths keep their per-instruction granularity.
	CHostBus bus;
	CC64Memory mem;
	mem.attachBus( &bus );
	mem.reset();
	mem.setPacing( 0, 1000000 );

	mem.write8( 0x1000, 0xAD ); mem.write8( 0x1001, 0x12 ); mem.write8( 0x1002, 0xD0 );
	CW65C816 cpu;
	cpu.attachBus( &mem );
	cpu.m_PC = 0x1000; cpu.m_PBR = 0; cpu.m_E = true; cpu.m_S = 0x01F0;

	const u64 start = mem.m_EmuCycles;
	cpu.step();
	CHECK_EQ( mem.m_EmuCycles - start, 4u );	// LDA abs, and nothing added
	CHECK_EQ( mem.m_IOStretchCycles, 0u );
}

TEST( io_stretch_preserves_every_rmw_bus_access )
{
	// INC abs performs one read and, in emulation mode, the NMOS-compatible
	// dummy write plus the final write. None of those three bus events may be
	// collapsed into another.
	CHostBus bus;
	CC64Memory mem;
	mem.attachBus( &bus );
	mem.reset();
	mem.setPacing( 0, 20000000 );
	mem.write8( 0x1000, 0xEE ); mem.write8( 0x1001, 0x20 ); mem.write8( 0x1002, 0xD0 );

	CW65C816 cpu;
	cpu.attachBus( &mem );
	cpu.m_PC = 0x1000; cpu.m_PBR = 0; cpu.m_E = true; cpu.m_S = 0x01F0;
	const u64 start = mem.m_EmuCycles;
	cpu.step();
	CHECK_EQ( mem.m_EmuCycles - start, 60u );
}

TEST( io_stretch_extra_cycle_applies_only_to_cia_writes )
{
	CHostBus bus;
	CC64Memory mem;
	mem.attachBus( &bus );
	mem.reset();
	mem.setPacing( 0, 20000000 );
	mem.write8( 0x1000, 0xAD ); mem.write8( 0x1001, 0x00 ); mem.write8( 0x1002, 0xDC );
	mem.write8( 0x1003, 0x8D ); mem.write8( 0x1004, 0x00 ); mem.write8( 0x1005, 0xDC );

	CW65C816 cpu;
	cpu.attachBus( &mem );
	cpu.m_PC = 0x1000; cpu.m_PBR = 0; cpu.m_E = true; cpu.m_S = 0x01F0;
	u64 start = mem.m_EmuCycles;
	cpu.step();
	CHECK_EQ( mem.m_EmuCycles - start, 20u );	// CIA read: ordinary stretch
	start = mem.m_EmuCycles;
	cpu.step();
	CHECK_EQ( mem.m_EmuCycles - start, 40u );	// CIA write: two C64 cycles
}

TEST( mirrored_writes_use_a_one_deep_posted_buffer )
{
	CHostBus bus;
	CC64Memory mem;
	CWriteBuffer wb;
	mem.attachBus( &bus );
	mem.reset();
	mem.setPacing( 0, 20000000 );
	mem.m_MirrorStretchEnable = true;
	mem.write8( 0x1000, 0x8D ); mem.write8( 0x1001, 0x00 ); mem.write8( 0x1002, 0x04 );
	mem.write8( 0x1003, 0x8D ); mem.write8( 0x1004, 0x01 ); mem.write8( 0x1005, 0x04 );
	mem.write8( 0x1006, 0xAD ); mem.write8( 0x1007, 0x12 ); mem.write8( 0x1008, 0xD0 );
	wb.attach( &bus, mem.m_RAM );
	wb.setOptMode( SCPU_OPT_NONE );
	mem.setMirrorSink( &wb );

	// Two consecutive STA abs instructions. The first posts without waiting;
	// the second must wait for its predecessor's next-PHI2 retirement.
	CW65C816 cpu;
	cpu.attachBus( &mem );
	cpu.m_PC = 0x1000; cpu.m_PBR = 0; cpu.m_E = true; cpu.m_S = 0x01F0;

	u64 start = mem.m_EmuCycles;
	cpu.step();
	CHECK_EQ( mem.m_EmuCycles - start, 4u );
	start = mem.m_EmuCycles;
	cpu.step();
	CHECK_EQ( mem.m_EmuCycles - start, 16u );
	CHECK_EQ( mem.m_MirrorStretchCycles, 12u );

	// An I/O read must drain the second posted write before touching the bus.
	start = mem.m_EmuCycles;
	cpu.step();
	CHECK_EQ( mem.m_EmuCycles - start, 40u );
}

TEST( mirrored_write_stretch_is_disabled_by_default_on_rad )
{
	CHostBus bus;
	CC64Memory mem;
	CWriteBuffer wb;
	mem.attachBus( &bus );
	mem.reset();
	mem.setPacing( 0, 20000000 );
	mem.write8( 0x1000, 0x8D ); mem.write8( 0x1001, 0x00 ); mem.write8( 0x1002, 0x04 );
	mem.write8( 0x1003, 0x8D ); mem.write8( 0x1004, 0x01 ); mem.write8( 0x1005, 0x04 );
	wb.attach( &bus, mem.m_RAM );
	wb.setOptMode( SCPU_OPT_NONE );
	mem.setMirrorSink( &wb );

	CW65C816 cpu;
	cpu.attachBus( &mem );
	cpu.m_PC = 0x1000; cpu.m_PBR = 0; cpu.m_E = true; cpu.m_S = 0x01F0;

	u64 start = mem.m_EmuCycles;
	cpu.step();
	CHECK_EQ( mem.m_EmuCycles - start, 4u );
	start = mem.m_EmuCycles;
	cpu.step();
	CHECK_EQ( mem.m_EmuCycles - start, 4u );
	CHECK_EQ( mem.m_MirrorStretchCycles, 0u );
}

TEST( pacing_period_is_phi2_and_not_a_round_megahertz )
{
	// The pacer's "1MHz" is the host machine's own PHI2, and no C64 runs at a
	// round megahertz. hostPerC64 is the trace's record of the divisor the
	// pacer and its phase-lock modulus actually use, so pinning it on both
	// standards is the host-side proof the physical period reached them:
	// 1400 on either would mean the /1000000 dilation is back. A measured
	// discriminator -- without the fix all three checks below fail.
	CHostBus   bus;
	CC64Memory mem;
	mem.attachBus( &bus );
	mem.reset();

	VSPTimingEvent events[ 4 ];
	u32 total = 0, overwritten = 0;

	bus.m_Signals.video = VIDEO_PAL;
	mem.setPacing( 1400000000ULL, 1000000u );
	mem.write8( 0xD011, 0x1B );
	CHECK_EQ( mem.copyVSPTrace( events, 4, total, overwritten ), 1u );
	CHECK_EQ( events[ 0 ].hostPerC64, 1420u );		// 1.4GHz / 985248Hz

	bus.m_Signals.video = VIDEO_NTSC_R8;
	mem.setPacing( 1400000000ULL, 1000000u );
	mem.write8( 0xD011, 0x1B );
	CHECK_EQ( mem.copyVSPTrace( events, 4, total, overwritten ), 2u );
	CHECK_EQ( events[ 1 ].hostPerC64, 1368u );		// 1.4GHz / 1022727Hz

	// Turbo leaves this period alone: it is the C64's cycle, not the selected
	// one, and the card's 20MHz comes from its own oscillator.
	mem.setPacing( 1400000000ULL, 20000000u );
	mem.write8( 0xD011, 0x1B );
	CHECK_EQ( mem.copyVSPTrace( events, 4, total, overwritten ), 3u );
	CHECK_EQ( events[ 2 ].hostPerC64, 1368u );
}

// A host clock the test moves by hand. CHostBus returns 0 from hostCycles(),
// and TimedHostBus's now++ cannot express "the guest was stopped for exactly
// this long" because every read of it advances time. These claims are about a
// specific duration, so the duration has to be the test's to set.
struct HeldHostBus : CHostBus
{
	u64 now = 0;
	u64 hostCycles() override { return now; }
	u32 hostCyclesPerSec() override { return 1400000000u; }
};

TEST( guest_stop_bracket_keeps_the_longest_pause_and_the_path_that_took_it )
{
	// K478's displacement is 201 NTSC raster lines -- about 13,000 C64 cycles,
	// 17.8M ARM cycles at 1368 per cycle -- and nothing in the emulator
	// measures how long the guest is stopped while the physical raster keeps
	// moving. This is that measurement: the interval between a return from
	// CW65C816::run() and the next entry.
	HeldHostBus bus;
	bus.m_Signals.video = VIDEO_NTSC_R8;
	CC64Memory mem;
	mem.attachBus( &bus );
	mem.reset();
	mem.setPacing( 1400000000ULL, 1000000u );

	// An ordinary 64-byte mirror installment first, then the displacement.
	// The long one must win AND must carry ITS OWN cause bits: a union of the
	// two would name the border drain for work the flush did, which is the one
	// way this instrument could send the next investigation to the wrong path.
	mem.noteGuestStopped();
	mem.noteGuestStopCause( SCPU_GUEST_STOP_FLUSH );
	bus.now += 90000;						// ~64 physical bytes
	mem.noteGuestResumed();

	mem.noteGuestStopped();
	mem.noteGuestStopCause( SCPU_GUEST_STOP_BORDER );
	mem.noteGuestStopCause( SCPU_GUEST_STOP_SWEEP );
	bus.now += 17800000;					// the K478 displacement
	mem.noteGuestResumed();

	mem.noteGuestStopped();
	mem.noteGuestStopCause( SCPU_GUEST_STOP_RASTER );
	bus.now += 4100;						// three $D012 transactions
	mem.noteGuestResumed();

	CHECK_EQ( mem.guestStopMaxHost(), 17800000u );
	CHECK_EQ( mem.guestStopMaxCauses(),
	          (u32)( SCPU_GUEST_STOP_BORDER | SCPU_GUEST_STOP_SWEEP ) );

	// A stop that is never closed cannot be counted: noteGuestStopped() with
	// no matching resume would otherwise let the NEXT resume measure from an
	// origin that belongs to a different interval.
	mem.noteGuestStopped();
	bus.now += 99000000;
	CHECK_EQ( mem.guestStopMaxHost(), 17800000u );
	mem.noteGuestResumed();
	CHECK_EQ( mem.guestStopMaxHost(), 99000000u );
	CHECK_EQ( mem.guestStopMaxCauses(), 0u );
	// And it closes exactly once, so a second resume cannot re-bill it.
	bus.now += 500000000;
	mem.noteGuestResumed();
	CHECK_EQ( mem.guestStopMaxHost(), 99000000u );
}

TEST( vsp_event_carries_the_stall_the_poll_shape_and_the_irq_distance )
{
	// The three fields are read together, so they are proved together: the
	// event has to drain all of them and then zero them, because each window
	// means "since the previous retained event" like bastall= and not "since
	// the previous write to this register" like dprev=.
	HeldHostBus bus;
	bus.m_Signals.video = VIDEO_NTSC_R8;
	CC64Memory mem;
	mem.attachBus( &bus );
	mem.reset();
	mem.setPacing( 1400000000ULL, 1000000u );

	VSPTimingEvent events[ 4 ];
	u32 total = 0, overwritten = 0;

	// The physical /IRQ rising edge, observed where the interrupt sample is
	// already paid for. Mario's chain is driven by the real VIC through the
	// real pin, so this is the only stamp that can separate "we noticed the
	// assertion late" from "it was asserted late".
	// m_IntCredit starts "due" so the first query samples; m_CachedIRQ starts
	// false, so this one call is the rising edge.
	bus.now = 1000;
	bus.m_IRQ = true;
	CHECK( mem.irqFast() );

	mem.noteGuestStopped();
	mem.noteGuestStopCause( SCPU_GUEST_STOP_MATRIXDIAG );
	bus.now += 1400000;
	mem.noteGuestResumed();

	// Mario's stabiliser polls $D012 from the bottom border down to its target
	// line. CHostBus answers $D011/$D012 from a free-running counter that
	// advances on bus accesses, so these reads really do return rising values
	// and d012First is the one the FIRST of them saw.
	const u8 first = mem.read8( 0xD012 );
	for ( u32 i = 0; i < 6; i++ ) (void)mem.read8( 0xD012 );

	bus.now += 7000;
	mem.write8( 0xD011, 0x1B );
	CHECK_EQ( mem.copyVSPTrace( events, 4, total, overwritten ), 1u );
	CHECK_EQ( events[ 0 ].stallHost, 1400000u );
	CHECK_EQ( events[ 0 ].stallCause, (u8)SCPU_GUEST_STOP_MATRIXDIAG );
	CHECK_EQ( events[ 0 ].d012Reads, 7u );
	CHECK_EQ( events[ 0 ].d012First, first );
	CHECK_EQ( events[ 0 ].irqToWriteHost, (u32)( bus.now - 1000 ) );

	// Drained: the next event reports its own window, not a running total.
	mem.write8( 0xD011, 0x1B );
	CHECK_EQ( mem.copyVSPTrace( events, 4, total, overwritten ), 2u );
	CHECK_EQ( events[ 1 ].stallHost, 0u );
	CHECK_EQ( events[ 1 ].stallCause, 0u );
	CHECK_EQ( events[ 1 ].d012Reads, 0u );
	CHECK_EQ( events[ 1 ].d012First, 0u );

	// With no assertion ever observed the distance has to read as "no
	// reference", exactly like sinceReleaseHost, and never as a small
	// plausible number.
	CC64Memory fresh;
	HeldHostBus freshBus;
	freshBus.m_Signals.video = VIDEO_NTSC_R8;
	fresh.attachBus( &freshBus );
	fresh.reset();
	fresh.setPacing( 1400000000ULL, 1000000u );
	freshBus.now = 5000;
	fresh.write8( 0xD011, 0x1B );
	CHECK_EQ( fresh.copyVSPTrace( events, 4, total, overwritten ), 1u );
	CHECK_EQ( events[ 0 ].irqToWriteHost, 0xFFFFFFFFu );
}

// --- the interrupt-control write ring ---------------------------------------
// WHAT THESE ARE FOR. The 2026-10-12 Mario capture could establish an earlier,
// NESTED interrupt entry and nothing more: no instrument recorded what armed
// the compare, what the acknowledgement did, or what the enable mask was. A
// capture that cannot discriminate between its candidates is not worth a card,
// so each test below fixes one of the discriminations the field capture will
// be read against.
//
// 1368 is host cycles per C64 cycle at 1.4GHz on an NTSC-R8 machine
// (1400000000 / 1022727), which is what setPacing computes from the detected
// standard. Ages in this ring are WHOLE C64 CYCLES, deliberately unlike the
// VSP trace's dprev=/stall=, which print hundredths.
static const u64 SCPU_TEST_HOST_PER_C64 = 1368;

TEST( intctl_ring_keeps_compare_programming_and_skips_marios_vsp_pair )
{
	HeldHostBus bus;
	bus.m_Signals.video = VIDEO_NTSC_R8;
	CC64Memory mem;
	mem.attachBus( &bus );
	mem.reset();
	mem.setPacing( 1400000000ULL, 1000000u );
	bus.now = 1000000;

	// Mario's VSP DEC/INC pair at $0871/$0874 changes YSCROLL only. It must
	// NOT enter this ring: the hold between those two writes is what the VSP
	// instrument measures, and an entry is real time inside it.
	mem.write8( 0xD011, 0x1A );
	mem.write8( 0xD011, 0x1B );
	CHECK_EQ( mem.m_ICtlHead, 0u );

	// A $D012 arm is kept, with the OTHER compare half beside it so the
	// composed compare can be checked rather than believed.
	mem.write8( 0xD012, 0x35 );
	CHECK_EQ( mem.m_ICtlHead, 1u );
	CHECK_EQ( mem.m_ICtlRing[ 0 ].reg, 0x12 );
	CHECK_EQ( mem.m_ICtlRing[ 0 ].value, 0x35 );
	CHECK_EQ( mem.m_ICtlRing[ 0 ].prev, 0x00 );
	CHECK_EQ( mem.m_ICtlRing[ 0 ].other, 0x1B );
	CHECK_EQ( mem.m_ICtlRing[ 0 ].compare, 0x35 );
	// No guest $D012 read yet and no physical bit-8 sample yet, so both raster
	// observations must read as ABSENT. A missing reference presented as a
	// small plausible number is the failure mode this guards.
	CHECK_EQ( mem.m_ICtlRing[ 0 ].grasterAge, 0xFFFFu );
	CHECK_EQ( mem.m_ICtlRing[ 0 ].prasterAge, 0xFFFFu );
	CHECK( !( mem.m_ICtlRing[ 0 ].flags & SCPU_ICTL_GRASTER_VALID ) );

	// The mask, the acknowledgement, and a $D011 that carries raster bit 8 --
	// the $96/$97 shape K478 caught coming out of a read-modify-write.
	mem.write8( 0xD01A, 0x01 );
	mem.write8( 0xD019, 0x01 );
	mem.write8( 0xD011, 0x9B );
	CHECK_EQ( mem.m_ICtlHead, 4u );
	CHECK_EQ( mem.m_ICtlRing[ 1 ].reg, 0x1A );
	CHECK_EQ( mem.m_ICtlRing[ 2 ].reg, 0x19 );
	CHECK_EQ( mem.m_ICtlRing[ 3 ].reg, 0x11 );
	CHECK_EQ( mem.m_ICtlRing[ 3 ].compare, 0x135 );
	// The mask as it stood AT the write, not as it stands at capture time.
	CHECK_EQ( mem.m_ICtlRing[ 3 ].d01a, 0x01 );
	// Clearing bit 8 again is compare programming too, in the other direction.
	mem.write8( 0xD011, 0x1B );
	CHECK_EQ( mem.m_ICtlHead, 5u );
	CHECK_EQ( mem.m_ICtlRing[ 4 ].compare, 0x35 );
}

TEST( intctl_ring_dates_the_raster_observations_it_did_not_pay_for )
{
	HeldHostBus bus;
	bus.m_Signals.video = VIDEO_NTSC_R8;
	CC64Memory mem;
	mem.attachBus( &bus );
	mem.reset();
	mem.setPacing( 1400000000ULL, 1000000u );
	bus.now = 1000000;

	// The guest's own $D012 read is the physical raster observation this ring
	// quotes. 91 C64 cycles is the staleness K478 measured on the bad event,
	// and the point of the field is that it reads as 91 and not as "now".
	const u8 seen = mem.read8( 0xD012 );
	bus.now += 91 * SCPU_TEST_HOST_PER_C64;
	mem.write8( 0xD012, 0x36 );
	CHECK( mem.m_ICtlRing[ 0 ].flags & SCPU_ICTL_GRASTER_VALID );
	CHECK_EQ( mem.m_ICtlRing[ 0 ].graster, seen );
	CHECK_EQ( mem.m_ICtlRing[ 0 ].grasterAge, 91u );
}

TEST( intctl_ack_records_whether_the_plain_store_released_the_irq_line )
{
	HeldHostBus bus;
	bus.m_Signals.video = VIDEO_NTSC_R8;
	CC64Memory mem;
	mem.attachBus( &bus );
	mem.reset();
	mem.setPacing( 1400000000ULL, 1000000u );
	bus.now = 1000000;

	// THE LEADING STRUCTURAL SUSPECT, made testable. Mario acks with a plain
	// STA $D019. If the source is still asserting immediately afterwards, it
	// re-enters through whatever vector is current -- which reproduces both the
	// short entry gap and the nesting with every compare in the ring correct.
	// Here the host bus holds /IRQ from a source the $D019 write cannot clear.
	bus.m_IRQ = true;
	mem.write8( 0xD019, 0xFF );
	CHECK_EQ( mem.m_D019Writes, 1u );
	CHECK( mem.m_ICtlRing[ 0 ].flags & SCPU_ICTL_IRQ_AFTER_VALID );
	CHECK( mem.m_ICtlRing[ 0 ].flags & SCPU_ICTL_IRQ_AFTER );
	CHECK_EQ( mem.m_D019AckHeldIRQ, 1u );

	// The follow-up sample, which is what separates a latch that did not clear
	// from pull-up propagation on a shared open-collector line. It comes from
	// the cached level -- no second GPIO read -- so the cache has to be
	// refreshed after the write for it to count.
	mem.tickFast( 2 );
	(void)mem.irqAsserted();
	bus.now += 3 * SCPU_TEST_HOST_PER_C64;
	mem.write8( 0xD012, 0x35 );
	CHECK( mem.m_ICtlRing[ 0 ].flags & SCPU_ICTL_IRQ_LATE_VALID );
	CHECK( mem.m_ICtlRing[ 0 ].flags & SCPU_ICTL_IRQ_LATE );
	CHECK_EQ( mem.m_ICtlRing[ 0 ].lateHost, 3 * SCPU_TEST_HOST_PER_C64 );
	CHECK_EQ( mem.m_D019AckHeldIRQLate, 1u );

	// And the healthy shape: the line releases, so this acknowledgement is
	// ruled out as the cause of anything.
	bus.m_IRQ = false;
	mem.tickFast( 2 );
	(void)mem.irqAsserted();
	mem.write8( 0xD019, 0xFF );
	CHECK_EQ( mem.m_D019Writes, 2u );
	CHECK( mem.m_ICtlRing[ 2 ].flags & SCPU_ICTL_IRQ_AFTER_VALID );
	CHECK( !( mem.m_ICtlRing[ 2 ].flags & SCPU_ICTL_IRQ_AFTER ) );
	CHECK_EQ( mem.m_D019AckHeldIRQ, 1u );
}

TEST( intctl_late_verdict_refuses_a_cache_that_predates_the_acknowledgement )
{
	HeldHostBus bus;
	bus.m_Signals.video = VIDEO_NTSC_R8;
	CC64Memory mem;
	mem.attachBus( &bus );
	mem.reset();
	mem.setPacing( 1400000000ULL, 1000000u );
	bus.now = 1000000;

	// THE BATCH, REPRODUCED, and this is the test the old credit gate could not
	// pass. CW65C816::run executes up to eight instructions and ticks the bus
	// ONCE, at the boundary, so m_IntCredit does not move while the guest
	// acknowledges. Sample /IRQ low, let the EMULATED clock move on with no new
	// sample, then acknowledge and release the line as the chip would. The
	// follow-up must REFUSE to quote the pre-ack level: with m_IntCredit at 0
	// the old test ( 0 * perEmu > d ) was false and passed it as valid, which is
	// how K487's irq=101 came to be read as a re-assertion and sent the hunt
	// after a VIC re-latch that the beam position rules out.
	// EVERY tickFast HERE IS PRECEDED BY A HOST-CLOCK ADVANCE, and it is not
	// decoration: HeldHostBus only moves when the test moves it, and tickSettle
	// SPINS until hostCycles() reaches m_PacingAnchor + owed. A tick whose owed
	// time has not been granted hangs the suite rather than failing it. Eight
	// C64 cycles covers two emulated cycles plus any I/O stretch the pending
	// posted write adds.
	bus.m_IRQ = true;
	bus.now += 8 * SCPU_TEST_HOST_PER_C64;
	mem.tickFast( 2 );
	(void)mem.irqAsserted();			// the cache is sampled BEFORE the ack
	// The emulated clock moves on with NO new sample, so the stamp this cache
	// carries is provably older than the acknowledgement below. Two cycles, and
	// the amount is load-bearing in the OTHER direction too: the old credit gate
	// computed 2 * 1368 = 2736 host cycles of claimed cache age against the 4104
	// gap below, so it still ACCEPTED here and this test still fails against it.
	// tickFast( 4 ) would have made the old gate reject for the wrong reason and
	// the test would have proved nothing.
	bus.now += 8 * SCPU_TEST_HOST_PER_C64;
	mem.tickFast( 2 );
	bus.m_IRQ = false;					// the acknowledgement releases the line
	mem.write8( 0xD019, 0xFF );
	// MEASURED on this sequence: the entry records irq=1 before, 0 after -- the
	// plain store DID release the line -- and the ack arms the re-sample
	// one-shot, which is what ends the batch so the core cannot act on the stale
	// level above.
	CHECK( mem.m_ICtlRing[ 0 ].flags & SCPU_ICTL_IRQ_BEFORE );
	CHECK( !( mem.m_ICtlRing[ 0 ].flags & SCPU_ICTL_IRQ_AFTER ) );
	CHECK( mem.m_IRQResampleDue );
	bus.now += 3 * SCPU_TEST_HOST_PER_C64;
	mem.write8( 0xD012, 0x35 );			// resolves the follow-up
	CHECK( !( mem.m_ICtlRing[ 0 ].flags & SCPU_ICTL_IRQ_LATE_VALID ) );
	CHECK( !( mem.m_ICtlRing[ 0 ].flags & SCPU_ICTL_IRQ_LATE ) );
	CHECK_EQ( mem.m_D019AckHeldIRQLate, 0u );
	// The GAP still reports, so a refused verdict stays visible as refused
	// rather than as silence -- the same discipline as "--" for a missing age.
	CHECK_EQ( mem.m_ICtlRing[ 0 ].lateHost, 3 * SCPU_TEST_HOST_PER_C64 );

	// The forced sample disarms the one-shot, so a $D019 write cannot leave
	// turbo parked on fine ticks for longer than one instruction.
	bus.now += 8 * SCPU_TEST_HOST_PER_C64;
	mem.tickFast( 2 );
	(void)mem.irqAsserted();
	CHECK( !mem.m_IRQResampleDue );

	// AND THE POSITIVE CONTROL, without which the test above would also pass a
	// gate that simply refused everything: that same sample provably FOLLOWS
	// the $D012 write it is being quoted against, so this verdict is valid --
	// and it reads 0, the healthy shape, because the line really did release.
	bus.now += 3 * SCPU_TEST_HOST_PER_C64;
	mem.write8( 0xD012, 0x36 );
	CHECK( mem.m_ICtlRing[ 1 ].flags & SCPU_ICTL_IRQ_LATE_VALID );
	CHECK( !( mem.m_ICtlRing[ 1 ].flags & SCPU_ICTL_IRQ_LATE ) );
}

TEST( intctl_ring_latches_at_the_first_impossible_compare_and_then_locks )
{
	HeldHostBus bus;
	bus.m_Signals.video = VIDEO_NTSC_R8;
	CC64Memory mem;
	mem.attachBus( &bus );
	mem.reset();
	mem.setPacing( 1400000000ULL, 1000000u );
	bus.now = 1000000;

	// 272 on a 263-line machine, the same shape as K471's 318. The latch has to
	// include the write that COMPOSED it -- the detection happens before the
	// ring entry exists, which is why the freeze is deferred by one append.
	// The raster interrupt must be ENABLED for an impossible compare to matter.
	// K486 proved why: it latched during boot on CMD's ROM composing 311 with
	// $D01A = $00, inert and overwritten microseconds later, 600M cycles before
	// the hang -- and that false positive cost an entire hardware capture.
	mem.write8( 0xD01A, 0x01 );
	mem.write8( 0xD012, 0x10 );
	mem.write8( 0xD011, 0x9B );
	CHECK_EQ( mem.m_RasterCompareImpossible, 1u );
	CHECK_EQ( mem.m_ICtlFrozenReason, 1u );
	CHECK_EQ( mem.m_ICtlFrozenHead, 3u );
	CHECK_EQ( mem.m_ICtlFrozen[ 2 ].reg, 0x11 );
	CHECK_EQ( mem.m_ICtlFrozen[ 2 ].value, 0x9B );
	CHECK_EQ( mem.m_ICtlFrozen[ 2 ].compare, 272 );
	CHECK( mem.m_ICtlFrozen[ 2 ].flags & SCPU_ICTL_COMPARE_IMPOSSIBLE );

	// Seconds of post-hang traffic and any number of progress-detector
	// episodes must not disturb it: that traffic is exactly what overwrites the
	// live ring before the button is pressed.
	mem.write8( 0xD012, 0x20 );
	mem.noteProgressSuspects( 1 );
	mem.noteProgressSuspects( 2 );
	CHECK_EQ( mem.m_ICtlFrozenReason, 1u );
	CHECK_EQ( mem.m_ICtlFrozenHead, 3u );
	CHECK_EQ( mem.m_ICtlFrozen[ 2 ].reg, 0x11 );
}

TEST( intctl_ring_ignores_an_impossible_compare_while_the_raster_irq_is_off )
{
	// The exact K486 false positive, pinned so it cannot return: CMD's boot ROM
	// composes 311 on a 263-line machine with the raster interrupt disabled.
	// It is inert -- nothing can fire on it -- so it must not consume the latch.
	HeldHostBus bus;
	bus.m_Signals.video = VIDEO_NTSC_R8;
	CC64Memory mem;
	mem.attachBus( &bus );
	mem.reset();
	mem.setPacing( 1400000000ULL, 1000000u );
	bus.now = 1000000;

	mem.write8( 0xD012, 0x37 );
	mem.write8( 0xD011, 0x9B );
	CHECK_EQ( mem.m_RasterCompareImpossible, 1u );	// still COUNTED
	CHECK_EQ( mem.m_ICtlFrozenReason, 0u );			// but NOT latched

	// Arming it afterwards and composing another one does latch, so the guard
	// defers the latch rather than disabling it.
	mem.write8( 0xD01A, 0x01 );
	mem.write8( 0xD011, 0x1B );
	mem.write8( 0xD011, 0x9B );
	CHECK_EQ( mem.m_ICtlFrozenReason, 1u );
}

TEST( intctl_ring_latches_on_a_progress_suspect_when_nothing_else_has )
{
	HeldHostBus bus;
	bus.m_Signals.video = VIDEO_NTSC_R8;
	CC64Memory mem;
	mem.attachBus( &bus );
	mem.reset();
	mem.setPacing( 1400000000ULL, 1000000u );
	bus.now = 1000000;

	// The hypothesis-free backstop, with the same re-copy rule as
	// CW65C816::m_IntRingSuspect: a disk load reaches SUSPECT harmlessly and
	// must not be able to silence the instrument for the rest of the run.
	mem.write8( 0xD012, 0x35 );
	mem.noteProgressSuspects( 1 );
	CHECK_EQ( mem.m_ICtlFrozenReason, 2u );
	CHECK_EQ( mem.m_ICtlFrozenHead, 1u );
	mem.write8( 0xD012, 0x36 );
	mem.noteProgressSuspects( 2 );
	CHECK_EQ( mem.m_ICtlFrozenHead, 2u );
	// The same episode count is not a new episode.
	mem.write8( 0xD012, 0x37 );
	mem.noteProgressSuspects( 2 );
	CHECK_EQ( mem.m_ICtlFrozenHead, 2u );
}

TEST( interrupt_context_reports_the_armed_compare_the_mask_and_the_anchor_age )
{
	HeldHostBus bus;
	bus.m_Signals.video = VIDEO_NTSC_R8;
	CC64Memory mem;
	mem.attachBus( &bus );
	mem.reset();
	mem.setPacing( 1400000000ULL, 1000000u );
	bus.now = 1000000;

	// What the interrupt ring cannot see for itself, and the reason the entry
	// gap and the stack pointer alone could not identify a source: an entry
	// whose observed raster is nowhere near its armed compare was not let in by
	// that compare. No bus cycle, no GPIO read and no CIA access here -- $DC0D
	// and $DD0D clear on read, so they are never sampled for observation.
	mem.write8( 0xD01A, 0x01 );
	mem.write8( 0xD012, 0x35 );
	const u8 seen = mem.read8( 0xD012 );
	bus.now += 88 * SCPU_TEST_HOST_PER_C64;
	const u64 ctx = mem.interruptContext();
	CHECK_EQ( (u16)ctx, 0x35 );
	CHECK_EQ( (u8)( ctx >> 16 ), 0x01 );
	CHECK_EQ( (u8)( ctx >> 24 ), seen );
	CHECK_EQ( (u8)( ctx >> 32 ), 22 );			// 88 C64 cycles, reported / 4
	CHECK( (u8)( ctx >> 40 ) & SCPU_IRQCTX_RASTER_VALID );
	CHECK( !( (u8)( ctx >> 40 ) & SCPU_IRQCTX_COMPARE_IMPOSSIBLE ) );

	// An acknowledgement that left the line asserted is reported ONCE, on the
	// next interrupt entry, and then cleared: the bit describes the interval
	// before that entry, which is the interval a nested re-entry happened in.
	bus.m_IRQ = true;
	mem.write8( 0xD019, 0xFF );
	CHECK( (u8)( mem.interruptContext() >> 40 ) & SCPU_IRQCTX_ACK_HELD );
	CHECK( !( (u8)( mem.interruptContext() >> 40 ) & SCPU_IRQCTX_ACK_HELD ) );
}
