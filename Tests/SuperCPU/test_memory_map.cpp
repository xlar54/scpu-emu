/*
   SCPU-EMU - 24-bit address space and SuperRAM tests.

   Layout verified against VICE's SCPU64 emulation; see Source/SuperCPU/memory_map.h.
*/
#include "../test_framework.h"
#include "../../Source/SuperCPU/memory_map.h"
#include "../../Source/Bus/Host/host_bus.h"
#include "../../Source/SuperCPU/registers.h"

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

TEST( integration_hardware_interrupt_closes_the_register_bank )
{
	// The "@$" freeze, pinned. With the register bank OPEN, $E000-$FFFF is the
	// KS image -- and CMD's KS image is not a whole KERNAL. ROM $9F2D-$9F74,
	// which is $FF2D-$FF74 through that window, is zeros in BOTH the 1.4 and
	// 2.04 images, and $FF48 (the KERNAL IRQ entry) sits inside it.
	//
	// Yet the KS vector table's own IRQ vector reads $FF48. A vector pointing
	// into a hole in the image that supplied it is only coherent if the handler
	// runs from the OTHER image, where $FF48 is the real KERNAL entry. So a
	// hardware interrupt must close the bank as it is taken. Without that the
	// machine executes $00 = BRK at $FF48, which vectors back to $FF48: an
	// infinite loop with the stack filling, seen on hardware as a frozen C64.
	//
	// BRK and COP are instructions, not interrupts, and must NOT close it.
	CSuperCPURegisters regs;
	u32 kernalShadowBase = 0xE000;
	regs.reset();
	regs.trackKernalShadow( &kernalShadowBase );

	regs.ioWrite( SCPU_REG_HWREGS_ENABLE, 0 );
	CHECK( regs.hardwareRegsEnabled() );
	CHECK_EQ( kernalShadowBase, 0x6000u );		// KS while open

	regs.onInterruptAcknowledged();
	CHECK( !regs.hardwareRegsEnabled() );
	CHECK_EQ( kernalShadowBase, 0xE000u );		// KT for the handler
	CHECK_EQ( regs.m_InterruptBankCloses, 1u );

	// RTI hands the interrupted code back the world it left: bank open again.
	// Without this, DOS code interrupted mid-command resumed with its $D2xx
	// scratch write-protected and its state silently corrupting -- seen on
	// hardware as sporadic warm resets on wedge commands.
	regs.onInterruptReturned();
	CHECK( regs.hardwareRegsEnabled() );
	CHECK_EQ( kernalShadowBase, 0x6000u );

	// Nested NMI-in-IRQ: the inner acknowledge sees the bank already closed,
	// so the inner return must NOT reopen it -- only the outer one does.
	regs.onInterruptAcknowledged();				// outer, bank open -> closes
	regs.onInterruptAcknowledged();				// inner, already closed
	CHECK_EQ( regs.m_InterruptBankCloses, 2u );
	regs.onInterruptReturned();					// inner: stays closed
	CHECK( !regs.hardwareRegsEnabled() );
	regs.onInterruptReturned();					// outer: reopens
	CHECK( regs.hardwareRegsEnabled() );

	// A stray RTI with no interrupt in flight -- RTI used as a jump -- must
	// not touch the bank.
	regs.ioWrite( SCPU_REG_HWREGS_DISABLE, 0 );
	regs.onInterruptReturned();
	CHECK( !regs.hardwareRegsEnabled() );
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

TEST( nmi_native_defer_holds_until_emulation_mode )
{
	// A native-mode NMI vectors through $00FFEA. On a SuperCPU nothing valid
	// is there, so the deferral holds the dispatch until the processor is
	// back in emulation mode, where the vector is the machine's real one.
	CHostBus bus;
	CC64Memory mem;
	mem.attachBus( &bus );
	mem.reset();
	mem.write8( 0x0001, 0x35 );			// all-RAM: vectors come from RAM

	u16 a = 0x1000;
	auto emit = [&]( int b ) { mem.write8( a++, (u8)b ); };
	emit( 0x18 ); emit( 0xFB );			// CLC XCE -> native
	for ( u32 i = 0; i < 20; i++ ) emit( 0xEA );
	const u16 backToEmu = a;
	emit( 0x38 ); emit( 0xFB );			// SEC XCE -> emulation
	for ( u32 i = 0; i < 10; i++ ) emit( 0xEA );

	// Distinguishable handlers: emulation counts $0FFD, native counts $0FFE.
	mem.write8( 0x2000, 0xEE ); mem.write8( 0x2001, 0xFD ); mem.write8( 0x2002, 0x0F );
	mem.write8( 0x2003, 0x40 );
	mem.write8( 0x2100, 0xEE ); mem.write8( 0x2101, 0xFE ); mem.write8( 0x2102, 0x0F );
	mem.write8( 0x2103, 0x40 );
	mem.write8( 0xFFFA, 0x00 ); mem.write8( 0xFFFB, 0x20 );
	mem.write8( 0xFFEA, 0x00 ); mem.write8( 0xFFEB, 0x21 );

	CW65C816 cpu;
	cpu.attachBus( &mem );
	cpu.m_DeferNativeNMI = true;
	cpu.m_PC = 0x1000; cpu.m_PBR = 0; cpu.m_E = true; cpu.m_S = 0x01F0;

	cpu.step();							// CLC
	cpu.step();							// XCE -> native
	CHECK( !cpu.m_E );

	bus.m_NMI = true;					// the line asserts INSIDE native mode
	for ( u32 i = 0; i < 12 && cpu.m_PC < backToEmu; i++ )
		cpu.step();
	CHECK_EQ( cpu.m_NMIsTaken, 0u );	// held
	CHECK( cpu.m_NMIsDeferred > 0u );
	CHECK_EQ( mem.m_RAM[ 0x0FFE ], 0 );	// never dispatched natively

	// Back to emulation mode: the held NMI is delivered at once.
	for ( u32 i = 0; i < 40 && cpu.m_NMIsTaken == 0; i++ )
		cpu.step();
	for ( u32 i = 0; i < 4; i++ ) cpu.step();		// let the handler run
	CHECK_EQ( cpu.m_NMIsTaken, 1u );
	CHECK_EQ( mem.m_RAM[ 0x0FFD ], 1 );	// through the emulation vector
	CHECK_EQ( mem.m_RAM[ 0x0FFE ], 0 );
}

TEST( nmi_native_defer_off_dispatches_natively )
{
	// With the escape hatch clear, the processor behaves as itself.
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
	cpu.m_DeferNativeNMI = false;
	cpu.m_PC = 0x1000; cpu.m_PBR = 0; cpu.m_E = true; cpu.m_S = 0x01F0;

	cpu.step(); cpu.step();				// into native mode
	bus.m_NMI = true;
	for ( u32 i = 0; i < 6 && cpu.m_NMIsTaken == 0; i++ )
		cpu.step();
	for ( u32 i = 0; i < 4; i++ ) cpu.step();		// let the handler run
	CHECK_EQ( cpu.m_NMIsTaken, 1u );
	CHECK_EQ( mem.m_RAM[ 0x0FFE ], 1 );	// dispatched through the native vector
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
