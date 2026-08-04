/*
   SCPU-EMU - end-to-end tests across CPU core, banking, shadow memory, write
   buffer and bus backend.

   The point of these is to check the seams: that a read of ROM costs nothing on
   the bus, that a read of $D012 does, that RAM writes reach the C64 only via
   the mirror path, and that the whole stack survives a synthetic KERNAL.
*/
#include "../test_framework.h"
#include "../../Source/C64/c64_memory.h"
#include "../../Source/CPU/M6502/m6502.h"
#include "../../Source/SuperCPU/write_buffer.h"
#include "../../Source/SuperCPU/registers.h"
#include "../../Source/SuperCPU/supercpu.h"
#include "../../Source/Bus/Host/host_bus.h"

struct SystemFixture
{
	CHostBus           bus;
	CC64Memory         mem;
	CWriteBuffer       wb;
	CSuperCPURegisters regs;
	CM6502             cpu;

	SystemFixture()
	{
		mem.attachBus( &bus );
		mem.setMirrorSink( &wb );
		mem.setIOInterceptor( &regs );
		wb.attach( &bus, mem.m_RAM );
		regs.attach( &wb );
		cpu.attachBus( &mem );
	}

	// Install a KERNAL image whose reset vector points at 'entry'.
	void installKernal( u16 entry, const u8 *code, u32 len, u16 org )
	{
		u8 kernal[ C64_KERNAL_SIZE ];
		std::memset( kernal, 0xEA, sizeof( kernal ) );	// NOP fill

		if ( org >= 0xE000 )
			std::memcpy( &kernal[ org - 0xE000 ], code, len );

		kernal[ 0x1FFC ] = (u8)( entry & 0xFF );
		kernal[ 0x1FFD ] = (u8)( entry >> 8 );
		mem.setKernalROM( kernal );
	}

	void start()
	{
		mem.reset();
		wb.resetStats();
		bus.resetStats();
		cpu.reset();
	}
};

TEST( integration_reset_vector_comes_from_shadowed_kernal )
{
	SystemFixture f;
	const u8 code[] = { 0xEA };
	f.installKernal( 0xFCE2, code, sizeof( code ), 0xFCE2 );
	f.start();

	CHECK_EQ( f.cpu.m_PC, 0xFCE2 );
	// Fetching the vector out of shadow RAM must not touch the machine at all.
	CHECK_EQ( f.bus.m_Cycles, 0 );
}

TEST( integration_rom_execution_is_free_on_the_bus )
{
	SystemFixture f;
	// LDA #$41 / STA $0400 ... run entirely from ROM.
	const u8 code[] = { 0xA9, 0x41, 0x8D, 0x00, 0x04, 0x4C, 0x00, 0xE0 };
	f.installKernal( 0xE000, code, sizeof( code ), 0xE000 );
	f.start();
	f.wb.setOptMode( SCPU_OPT_NONE );

	f.cpu.step();	// LDA #
	f.cpu.step();	// STA $0400

	// Instruction fetches and the RAM write are all local. The write is queued
	// for mirroring but has not been flushed.
	CHECK_EQ( f.bus.m_Cycles, 0 );
	CHECK_EQ( f.mem.m_RAM[ 0x0400 ], 0x41 );
	CHECK_EQ( f.wb.pending(), 1 );

	f.wb.flush();
	CHECK_EQ( f.bus.m_Cycles, 1 );
	CHECK_EQ( f.bus.m_Memory[ 0x0400 ], 0x41 );
}

TEST( integration_io_read_goes_to_the_machine )
{
	SystemFixture f;
	// $D020 rather than $D012: the host bus synthesises a raster counter for
	// $D011/$D012 so a real KERNAL can boot, so those two are not plain storage.
	const u8 code[] = { 0xAD, 0x20, 0xD0 };	// LDA $D020
	f.installKernal( 0xE000, code, sizeof( code ), 0xE000 );
	f.start();

	f.bus.m_Memory[ 0xD020 ] = 0x37;

	f.cpu.step();
	CHECK_EQ( f.cpu.m_A, 0x37 );
	CHECK_EQ( f.bus.m_Reads, 1 );
	CHECK_EQ( f.mem.m_IOReads, 1 );
}

TEST( integration_ordinary_io_does_not_flush_the_mirror_buffer )
{
	// Flushing before EVERY I/O access meant a booting KERNAL -- which touches
	// I/O constantly -- drove a continuous stream of unscheduled bursts across
	// the visible display, corrupting the VIC-II's fetches. So ordinary I/O
	// does not flush; mirroring is scheduled against the raster in
	// CSuperCPU::runFrame() instead.
	//
	// $D020 is the border colour: it changes what the VIC-II draws, but not
	// which memory it reads, so nothing has to arrive first.
	SystemFixture f;
	const u8 code[] = { 0xA9, 0x41, 0x8D, 0x00, 0x04,	// LDA #$41 / STA $0400
	                    0xA9, 0x0E, 0x8D, 0x20, 0xD0 };	// LDA #$0E / STA $D020
	f.installKernal( 0xE000, code, sizeof( code ), 0xE000 );
	f.start();
	f.wb.setOptMode( SCPU_OPT_NONE );
	f.bus.m_LogEnabled = true;

	for ( int i = 0; i < 4; i++ ) f.cpu.step();

	// The screen byte is still queued; only the register write went out.
	CHECK_EQ( f.wb.pending(), 1 );
	CHECK_EQ( f.bus.m_LogCount, 1 );
	CHECK_EQ( f.bus.m_Log[ 0 ].addr, 0xD020 );
	CHECK_EQ( f.bus.m_Log[ 0 ].op, HOSTOP_WRITE );

	// It reaches the machine when the mirror is flushed, not before.
	f.wb.flush();
	CHECK_EQ( f.bus.m_Memory[ 0x0400 ], 0x41 );
}

TEST( integration_display_state_registers_flush_first )
{
	// The four registers that change what the VIC-II LOOKS AT are the exception,
	// and they have to be, because the reordering is visible.
	//
	// Staging a screen and then pointing the chip at it is the standard way to
	// switch display: draw into memory, then write $D011/$D018. If the register
	// write overtakes the drawing, the VIC-II starts displaying the new location
	// before our bytes have arrived, so the machine shows whatever was there
	// before and the intended picture appears a frame or more later. On the CMD
	// SuperCPU's own boot animation that reads as the screen staying in text
	// mode throughout, then flipping to graphics once it is over.
	static const u16 displayRegs[] = { 0xD011, 0xD016, 0xD018, 0xDD00 };

	for ( u32 i = 0; i < 4; i++ )
	{
		const u16 reg = displayRegs[ i ];

		SystemFixture f;
		const u8 code[] = { 0xA9, 0x41, 0x8D, 0x00, 0x04,				// STA $0400
		                    0xA9, 0x3B, 0x8D, (u8)( reg & 0xFF ),
		                                      (u8)( reg >> 8 ) };		// STA reg
		f.installKernal( 0xE000, code, sizeof( code ), 0xE000 );
		f.start();
		f.wb.setOptMode( SCPU_OPT_NONE );
		f.bus.m_LogEnabled = true;

		for ( int i2 = 0; i2 < 4; i2++ ) f.cpu.step();

		// The staged byte must already be on the machine, and must have got
		// there BEFORE the register write.
		CHECK_EQ( f.wb.pending(), 0 );
		CHECK_EQ( f.bus.m_Memory[ 0x0400 ], 0x41 );

		bool sawScreenWrite = false, orderedCorrectly = false;
		for ( u32 j = 0; j < f.bus.m_LogCount; j++ )
		{
			if ( f.bus.m_Log[ j ].addr == 0x0400 ) sawScreenWrite = true;
			if ( f.bus.m_Log[ j ].addr == reg )    orderedCorrectly = sawScreenWrite;
		}
		CHECK( sawScreenWrite );
		CHECK( orderedCorrectly );
	}
}

TEST( integration_banking_switch_changes_what_executes )
{
	SystemFixture f;
	const u8 code[] = { 0xAD, 0x00, 0xE0 };	// LDA $E000
	f.installKernal( 0xE000, code, sizeof( code ), 0xE000 );
	f.start();

	// With HIRAM set, $E000 reads the KERNAL (first byte of our LDA).
	f.cpu.step();
	CHECK_EQ( f.cpu.m_A, 0xAD );

	// Drop HIRAM and LORAM: $E000 must now read the RAM underneath.
	f.mem.m_RAM[ 0xE000 ] = 0x5A;
	f.mem.write8( 0x0001, 0x30 );
	f.cpu.m_PC = 0xE000;	// re-run the same instruction, now out of RAM

	CHECK_EQ( f.mem.read8( 0xE000 ), 0x5A );
}

TEST( integration_supercpu_registers_never_reach_the_bus )
{
	SystemFixture f;
	// Open the register bank, then select BASIC optimization.
	const u8 code[] = { 0x8D, 0x7E, 0xD0,	// STA $D07E - enable hw registers
	                    0x8D, 0x76, 0xD0,	// STA $D076 - BASIC optimization
	                    0x8D, 0x7A, 0xD0 };	// STA $D07A - normal speed
	f.installKernal( 0xE000, code, sizeof( code ), 0xE000 );
	f.start();

	CHECK( f.regs.turboEnabled() );

	for ( int i = 0; i < 3; i++ ) f.cpu.step();

	CHECK( f.regs.hardwareRegsEnabled() );
	CHECK_EQ( f.wb.optMode(), SCPU_OPT_BASIC );
	CHECK( !f.regs.turboEnabled() );

	// None of it should have generated a single C64 bus cycle.
	CHECK_EQ( f.bus.m_Cycles, 0 );
	CHECK_EQ( f.mem.m_IOWrites, 0 );
}

TEST( integration_optimization_registers_ignored_while_bank_closed )
{
	SystemFixture f;
	const u8 code[] = { 0x8D, 0x76, 0xD0 };	// STA $D076 without enabling first
	f.installKernal( 0xE000, code, sizeof( code ), 0xE000 );
	f.start();

	SCPUOptMode before = f.wb.optMode();
	f.cpu.step();
	CHECK_EQ( f.wb.optMode(), before );
}

TEST( integration_supercpu_detection_idiom )
{
	SystemFixture f;
	// $D0BC = decimal 53436, the documented detection register. (An earlier
	// source quoted PEEK(53433) = $D0B9; the SuperCPU 128 register list and the
	// compatibility notes both say $D0BC, so 53433 looks like a transposition.)
	const u8 code[] = { 0xAD, 0xBC, 0xD0 };	// LDA $D0BC
	f.installKernal( 0xE000, code, sizeof( code ), 0xE000 );
	f.start();

	f.cpu.step();
	// Software checks (PEEK(53436) AND 128) = 0 to detect an active accelerator.
	CHECK_EQ( f.cpu.m_A & 0x80, 0x00 );
}

TEST( integration_snapshot_roms_from_the_machine )
{
	SystemFixture f;

	// Pretend the live C64 has a KERNAL whose reset vector is the usual $FCE2.
	for ( u32 i = 0; i < 0x2000; i++ )
		f.bus.m_Memory[ 0xE000 + i ] = (u8)( i & 0xFF );
	f.bus.m_Memory[ 0xFFFC ] = 0xE2;
	f.bus.m_Memory[ 0xFFFD ] = 0xFC;
	for ( u32 i = 0; i < 0x2000; i++ )
		f.bus.m_Memory[ 0xA000 + i ] = (u8)( ~i & 0xFF );

	CHECK( f.mem.snapshotROMsFromBus() );
	CHECK( f.mem.hasKernalROM() );
	CHECK( f.mem.hasBasicROM() );

	f.mem.reset();
	f.cpu.reset();
	CHECK_EQ( f.cpu.m_PC, 0xFCE2 );

	// 8K of BASIC plus 8K of KERNAL, plus the two-byte reset-vector probe that
	// confirms the machine had its ROMs banked in.
	CHECK_EQ( f.bus.m_Reads, 0x4000 + 2 );
}

TEST( integration_snapshot_fills_in_only_the_missing_rom )
{
	// A caller may supply one image and want the other taken off the machine.
	// Overwriting the supplied one would silently discard it.
	SystemFixture f;

	u8 myKernal[ C64_KERNAL_SIZE ];
	std::memset( myKernal, 0x5A, sizeof( myKernal ) );
	myKernal[ 0x1FFC ] = 0x00;
	myKernal[ 0x1FFD ] = 0xE5;
	f.mem.setKernalROM( myKernal );

	// The machine holds a different KERNAL and a BASIC we do want.
	for ( u32 i = 0; i < 0x2000; i++ ) f.bus.m_Memory[ 0xE000 + i ] = 0xCC;
	f.bus.m_Memory[ 0xFFFC ] = 0xE2;
	f.bus.m_Memory[ 0xFFFD ] = 0xFC;
	for ( u32 i = 0; i < 0x2000; i++ ) f.bus.m_Memory[ 0xA000 + i ] = 0x77;

	CHECK( f.mem.snapshotROMsFromBus() );

	// Supplied KERNAL survived; BASIC came from the machine.
	CHECK_EQ( f.mem.m_Kernal[ 0x100 ], 0x5A );
	CHECK_EQ( f.mem.m_Basic[ 0x100 ], 0x77 );

	// Only BASIC was fetched: 8K plus the vector probe, not 16K.
	CHECK_EQ( f.bus.m_Reads, 0x2000 + 2 );

	f.mem.reset();
	f.cpu.reset();
	CHECK_EQ( f.cpu.m_PC, 0xE500 );		// our KERNAL's vector, not the machine's
}

TEST( integration_snapshot_is_a_no_op_when_both_roms_supplied )
{
	SystemFixture f;

	u8 rom[ C64_KERNAL_SIZE ];
	std::memset( rom, 0x11, sizeof( rom ) );
	f.mem.setKernalROM( rom );
	f.mem.setBasicROM( rom );

	CHECK( f.mem.snapshotROMsFromBus() );
	CHECK_EQ( f.bus.m_Reads, 0 );		// not even the vector probe
}

TEST( integration_snapshot_rejects_a_machine_that_was_not_banked_in )
{
	SystemFixture f;
	// All zeroes at $FFFC/$FFFD: the reset vector does not point into the
	// KERNAL, so what we read was DRAM, not ROM.
	CHECK( !f.mem.snapshotROMsFromBus() );
}

TEST( integration_synthetic_kernal_fills_the_screen )
{
	SystemFixture f;

	// LDX #$00
	// loop: TXA / STA $0400,X / INX / BNE loop / JMP self
	const u8 code[] = {
		0xA2, 0x00,				// LDX #$00
		0x8A,					// TXA
		0x9D, 0x00, 0x04,		// STA $0400,X
		0xE8,					// INX
		0xD0, 0xF9,				// BNE loop
		0x4C, 0x09, 0xE0		// JMP *
	};
	f.installKernal( 0xE000, code, sizeof( code ), 0xE000 );
	f.start();
	f.wb.setOptMode( SCPU_OPT_NONE );

	f.cpu.run( 5000 );
	f.wb.flush();

	for ( u32 i = 0; i < 256; i++ )
	{
		CHECK_EQ( f.mem.m_RAM[ 0x0400 + i ], (u8)i );
		CHECK_EQ( f.bus.m_Memory[ 0x0400 + i ], (u8)i );
	}

	// 256 distinct addresses written once each, coalesced into one burst.
	CHECK_EQ( f.bus.m_BurstWrites, 256 );
}

TEST( integration_detection_answers_on_both_documented_addresses )
{
	// $D0BC is the detection register. The rest of the block is decoded too, so
	// a program probing a neighbouring address still sees bit 7 clear rather
	// than the open-bus 1 a stock machine returns.
	SystemFixture f;
	const u8 code[] = { 0xEA };
	f.installKernal( 0xE000, code, sizeof( code ), 0xE000 );
	f.start();

	u8 v;
	CHECK( f.regs.ioRead( 0xD0B9, v ) );  CHECK_EQ( v & 0x80, 0x00 );	// 53433
	CHECK( f.regs.ioRead( 0xD0BC, v ) );  CHECK_EQ( v & 0x80, 0x00 );	// 53436
	CHECK( f.regs.ioRead( 0xD0BF, v ) );  CHECK_EQ( v & 0x80, 0x00 );

	// $D0B0 is the version/mode register, not the presence flag.
	CHECK( f.regs.ioRead( 0xD0B0, v ) );
	CHECK_EQ( v, SCPU_VERSION_V2 );

	// None of it reaches the machine.
	CHECK_EQ( f.bus.m_Cycles, 0 );
}

TEST( integration_speed_switch_gates_turbo_rather_than_forcing_it )
{
	// The physical switch is "force 1MHz, or merely ALLOW 20MHz" -- it never
	// accelerates the machine by itself.
	//
	// VICE adds a wrinkle worth knowing: the switch is honoured only while the
	// hardware registers are DISABLED. Software that opens the register bank
	// takes the switch out of circuit entirely.
	SystemFixture f;
	const u8 code[] = { 0xEA };
	f.installKernal( 0xE000, code, sizeof( code ), 0xE000 );

	f.regs.setSpeedSwitchAllowsTurbo( false );		// switch selects 1MHz
	f.start();
	f.regs.setSpeedSwitchAllowsTurbo( false );

	CHECK( !f.regs.fastMode() );					// held at 1MHz by the switch

	// $D0B5 bit 6 reports the switch position.
	u8 v;
	CHECK( f.regs.ioRead( 0xD0B5, v ) );
	CHECK_EQ( v & SCPU_SWITCH_1MHZ, SCPU_SWITCH_1MHZ );

	// Opening the register bank takes the switch out of circuit.
	f.regs.ioWrite( SCPU_REG_HWREGS_ENABLE, 0 );
	CHECK( f.regs.fastMode() );

	// ...and closing it puts the switch back in charge.
	f.regs.ioWrite( SCPU_REG_HWREGS_DISABLE, 0 );
	CHECK( !f.regs.fastMode() );

	// Flipping the switch to turbo permits speed, but software still decides.
	f.regs.setSpeedSwitchAllowsTurbo( true );
	CHECK( f.regs.fastMode() );
	CHECK( f.regs.ioRead( 0xD0B5, v ) );
	CHECK_EQ( v & SCPU_SWITCH_1MHZ, 0x00 );
}

TEST( integration_moving_switch_to_normal_drops_speed_immediately )
{
	SystemFixture f;
	const u8 code[] = { 0xEA };
	f.installKernal( 0xE000, code, sizeof( code ), 0xE000 );
	f.start();

	CHECK( f.regs.turboEnabled() );
	f.regs.setSpeedSwitchAllowsTurbo( false );
	CHECK( !f.regs.turboEnabled() );
}

TEST( integration_status_block_reports_distinct_flags )
{
	// $D0B0-$D0BF is not a mirrored block: each address carries its own flags.
	// Layout verified against VICE's SCPU64 emulation.
	SystemFixture f;
	const u8 code[] = { 0xEA };
	f.installKernal( 0xE000, code, sizeof( code ), 0xE000 );
	f.start();

	u8 v;

	// $D0B0: hardware version.
	CHECK( f.regs.ioRead( 0xD0B0, v ) );
	CHECK_EQ( v & 0xC0, SCPU_VERSION_V2 );

	// $D0B2: bit7 register bank open, bit6 system 1MHz. Both clear after reset.
	CHECK( f.regs.ioRead( 0xD0B2, v ) );
	CHECK_EQ( v & SCPU_STATUS_HWREGS,   0x00 );
	CHECK_EQ( v & SCPU_STATUS_SYS_1MHZ, 0x00 );

	f.regs.ioWrite( SCPU_REG_HWREGS_ENABLE, 0 );
	CHECK( f.regs.ioRead( 0xD0B2, v ) );
	CHECK_EQ( v & SCPU_STATUS_HWREGS, SCPU_STATUS_HWREGS );

	// System 1MHz is a separate request from software 1MHz.
	f.regs.ioWrite( SCPU_REG_SYS_1MHZ_ON, 0 );
	CHECK( f.regs.ioRead( 0xD0B2, v ) );
	CHECK_EQ( v & SCPU_STATUS_SYS_1MHZ, SCPU_STATUS_SYS_1MHZ );
	CHECK( !f.regs.fastMode() );
	f.regs.ioWrite( SCPU_REG_SYS_1MHZ_OFF, 0 );
	CHECK( f.regs.fastMode() );

	// $D0B4: current optimization mode in bits 7-6.
	f.regs.ioWrite( SCPU_REG_OPT_BASIC, 0 );
	CHECK( f.regs.ioRead( 0xD0B4, v ) );
	CHECK_EQ( v & 0xC0, SCPU_OPTIM_BASIC );
	CHECK_EQ( f.wb.optMode(), SCPU_OPT_BASIC );

	f.regs.ioWrite( SCPU_REG_OPT_VICBANK2, 0 );		// the GEOS setting
	CHECK( f.regs.ioRead( 0xD0B4, v ) );
	CHECK_EQ( v & 0xC0, SCPU_OPTIM_VICBANK2 );

	f.regs.ioWrite( SCPU_REG_OPT_NONE, 0 );
	CHECK( f.regs.ioRead( 0xD0B4, v ) );
	CHECK_EQ( v & 0xC0, SCPU_OPTIM_NONE );

	// $D0B5: JiffyDOS switch in bit 7.
	f.regs.setJiffyDOSSwitch( true );
	CHECK( f.regs.ioRead( 0xD0B5, v ) );
	CHECK_EQ( v & SCPU_SWITCH_JIFFYDOS, SCPU_SWITCH_JIFFYDOS );

	// $D0B6: the 6502 core is always in emulation mode.
	CHECK( f.regs.ioRead( 0xD0B6, v ) );
	CHECK_EQ( v & SCPU_PROC_EMULATION, SCPU_PROC_EMULATION );

	// $D0B8 and its mirror $D0B9 (decimal 53433) report the software request in
	// bit 7 and the combined "anything asking for 1MHz" in bit 6.
	f.regs.ioWrite( SCPU_REG_SOFT_1MHZ_ON, 0 );
	CHECK( f.regs.ioRead( 0xD0B8, v ) );
	CHECK_EQ( v & SCPU_SPEED_SOFT_1MHZ, SCPU_SPEED_SOFT_1MHZ );
	CHECK_EQ( v & SCPU_SPEED_ANY_1MHZ,  SCPU_SPEED_ANY_1MHZ );

	u8 mirrored;
	CHECK( f.regs.ioRead( 0xD0B9, mirrored ) );
	CHECK_EQ( mirrored, v );						// $D0B9 mirrors $D0B8

	f.regs.ioWrite( SCPU_REG_SOFT_1MHZ_OFF, 0 );
	CHECK( f.regs.ioRead( 0xD0B8, v ) );
	CHECK_EQ( v & SCPU_SPEED_SOFT_1MHZ, 0x00 );

	// $D0B3 is writable on v2 while the bank is open.
	CHECK( f.regs.ioWrite( 0xD0B3, 0x40 ) );
	CHECK( f.regs.ioRead( 0xD0B3, v ) );
	CHECK_EQ( v & 0xC0, 0x40 );
}

TEST( integration_low_optim_bits_appear_in_every_status_read )
{
	// VICE ORs the low three bits of the optimization register into every read
	// of the $D0Bx block, whichever register was addressed.
	SystemFixture f;
	const u8 code[] = { 0xEA };
	f.installKernal( 0xE000, code, sizeof( code ), 0xE000 );
	f.start();

	f.regs.ioWrite( SCPU_REG_HWREGS_ENABLE, 0 );
	f.regs.ioWrite( 0xD0B3, 0x05 );			// sets low bits 0 and 2

	u8 a, b, c;
	CHECK( f.regs.ioRead( 0xD0B0, a ) );
	CHECK( f.regs.ioRead( 0xD0B5, b ) );
	CHECK( f.regs.ioRead( 0xD0BC, c ) );

	CHECK_EQ( a & 0x07, 0x05 );
	CHECK_EQ( b & 0x07, 0x05 );
	CHECK_EQ( c & 0x07, 0x05 );
}

TEST( integration_hardware_registers_gate_the_optimisation_selects )
{
	SystemFixture f;
	const u8 code[] = { 0xEA };
	f.installKernal( 0xE000, code, sizeof( code ), 0xE000 );
	f.start();

	// Bank closed: the optimisation select is decoded but does nothing.
	SCPUOptMode before = f.wb.optMode();
	f.regs.ioWrite( SCPU_REG_OPT_BASIC, 0 );
	CHECK_EQ( f.wb.optMode(), before );

	// Bank open: it takes effect.
	f.regs.ioWrite( SCPU_REG_HWREGS_ENABLE, 0 );
	f.regs.ioWrite( SCPU_REG_OPT_BASIC, 0 );
	CHECK_EQ( f.wb.optMode(), SCPU_OPT_BASIC );

	// Speed selection is NOT gated -- that is what makes POKE 53370,0 work
	// straight from BASIC without opening the bank first.
	f.regs.ioWrite( SCPU_REG_HWREGS_DISABLE, 0 );
	f.regs.ioWrite( SCPU_REG_SOFT_1MHZ_ON, 0 );
	CHECK( !f.regs.fastMode() );
	f.regs.ioWrite( SCPU_REG_SOFT_1MHZ_OFF, 0 );
	CHECK( f.regs.fastMode() );
}

TEST( integration_cia2_port_a_write_arms_the_iec_throttle )
{
	// The KERNAL times its serial-bus bits by counting cycles, and those counts
	// assume a 1MHz CPU. At 20MHz every pulse is twenty times too short and no
	// drive answers -- which presents as DEVICE NOT PRESENT. A real SuperCPU
	// has the same problem and CMD document that it always drops to 1MHz for
	// disk access; this reproduces that.
	SystemFixture f;
	const u8 code[] = { 0xA9, 0x07, 0x8D, 0x00, 0xDD,
	                    0xA9, 0x17, 0x8D, 0x00, 0xDD };	// then change IEC CLK
	f.installKernal( 0xE000, code, sizeof( code ), 0xE000 );
	f.start();

	CHECK( !f.mem.iecThrottleActive() );

	f.cpu.step();		// LDA
	f.cpu.step();		// first STA establishes the CIA2 output baseline
	CHECK( !f.mem.iecThrottleActive() );
	f.cpu.step();
	f.cpu.step();		// IEC bit change arms the fallback

	CHECK( f.mem.iecThrottleActive() );
	CHECK_EQ( f.mem.m_IECThrottleEvents, 1 );

	// The write still reaches the real chip; throttling is a side effect, not a
	// substitution.
	CHECK_EQ( f.bus.m_Memory[ 0xDD00 ], 0x17 );
}

TEST( integration_iec_throttle_lapses_after_the_bus_goes_quiet )
{
	SystemFixture f;
	const u8 code[] = { 0xA9, 0x07, 0x8D, 0x00, 0xDD,
	                    0xA9, 0x17, 0x8D, 0x00, 0xDD, 0xEA };
	f.installKernal( 0xE000, code, sizeof( code ), 0xE000 );
	f.start();

	f.cpu.step();
	f.cpu.step();
	f.cpu.step();
	f.cpu.step();
	CHECK( f.mem.iecThrottleActive() );

	// It is a countdown in emulated cycles, so quiet time lets it expire and
	// the machine returns to full speed rather than being stuck slow.
	f.cpu.run( 200000 );
	CHECK( !f.mem.iecThrottleActive() );
}

TEST( integration_iec_throttle_can_be_disabled )
{
	SystemFixture f;
	const u8 code[] = { 0xA9, 0x07, 0x8D, 0x00, 0xDD,
	                    0xA9, 0x17, 0x8D, 0x00, 0xDD };
	f.installKernal( 0xE000, code, sizeof( code ), 0xE000 );
	f.start();
	f.mem.setIECThrottle( false );

	f.cpu.step();
	f.cpu.step();
	f.cpu.step();
	f.cpu.step();
	CHECK( !f.mem.iecThrottleActive() );
}

TEST( integration_vic_bank_changes_do_not_arm_iec_throttle )
{
	SystemFixture f;
	const u8 code[] = { 0xA9, 0x37, 0x8D, 0x00, 0xDD,
	                    0xA9, 0x36, 0x8D, 0x00, 0xDD };
	f.installKernal( 0xE000, code, sizeof( code ), 0xE000 );
	f.start();
	f.cpu.run( 12 );

	CHECK( !f.mem.iecThrottleActive() );
	CHECK_EQ( f.mem.m_IECThrottleEvents, 0 );
}

TEST( integration_cia2_read_provides_iec_throttle_baseline )
{
	SystemFixture f;
	f.start();
	f.bus.m_Memory[ 0xDD00 ] = 0x17;
	CHECK_EQ( f.mem.read8( 0xDD00 ), 0x17 );
	f.mem.write8( 0xDD00, 0x07 );

	CHECK( f.mem.iecThrottleActive() );
}

TEST( integration_explicit_slow_mode_supersedes_iec_fallback )
{
	SystemFixture f;
	f.start();
	f.mem.setPacing( 0, SCPU_NORMAL_HZ );
	f.mem.write8( 0xDD00, 0x07 );
	f.mem.write8( 0xDD00, 0x17 );

	CHECK( !f.mem.iecThrottleActive() );
}

TEST( integration_iec_throttle_uses_effective_interrupt_sample_rate )
{
	struct CountingBus : CHostBus
	{
		u32 samples = 0;
		void sampleInterrupts( bool &irq, bool &nmi ) override
		{
			samples++;
			irq = m_IRQ;
			nmi = m_NMI;
		}
	} bus;

	CC64Memory mem;
	mem.attachBus( &bus );
	mem.setPacing( 0, SCPU_TURBO_HZ );
	mem.reset();
	mem.irqAsserted();
	CHECK_EQ( bus.samples, 1 );

	mem.tickFast( 1 );
	mem.irqAsserted();
	CHECK_EQ( bus.samples, 1 );		// nominal turbo cache interval is 20 cycles

	mem.write8( 0xDD00, 0x07 );
	mem.write8( 0xDD00, 0x17 );
	CHECK( mem.iecThrottleActive() );
	mem.irqAsserted();
	CHECK_EQ( bus.samples, 2 );		// effective 1MHz interval is one cycle
}

TEST( integration_reset_expires_iec_and_refreshes_interrupt_cache )
{
	struct CountingBus : CHostBus
	{
		u32 samples = 0;
		void sampleInterrupts( bool &irq, bool &nmi ) override
		{
			samples++;
			irq = m_IRQ;
			nmi = m_NMI;
		}
	} bus;

	CC64Memory mem;
	mem.attachBus( &bus );
	bus.m_NMI = true;
	CHECK( mem.nmiAsserted() );
	mem.write8( 0xDD00, 0x07 );
	mem.write8( 0xDD00, 0x17 );
	CHECK( mem.iecThrottleActive() );

	bus.m_NMI = false;
	mem.reset();
	CHECK( !mem.iecThrottleActive() );
	CHECK( !mem.nmiAsserted() );
	CHECK_EQ( bus.samples, 2 );
}

TEST( integration_frame_budget_recalculates_after_speed_change )
{
	CHostBus bus;
	CSuperCPU scpu;
	u8 basic[ C64_BASIC_SIZE ];
	u8 kernal[ C64_KERNAL_SIZE ];
	std::memset( basic, 0xEA, sizeof( basic ) );
	std::memset( kernal, 0xEA, sizeof( kernal ) );

	const u8 code[] = { 0xA9, 0x00,			// LDA #$00
	                    0x8D, 0x7A, 0xD0,	// STA $D07A -- switch to 1MHz
	                    0x4C, 0x05, 0xE0 };	// JMP $E005
	std::memcpy( kernal, code, sizeof( code ) );
	kernal[ 0x1FFC ] = 0x00;
	kernal[ 0x1FFD ] = 0xE0;
	scpu.setBasicROM( basic );
	scpu.setKernalROM( kernal );
	CHECK( scpu.init( &bus, SCPU_CORE_65816, 0 ) );

	const u64 ran = scpu.runFrame();
	CHECK( !scpu.registers().fastMode() );
	CHECK( ran > 19000 );
	CHECK( ran < 21000 );	// old stale turbo budget ran about 393,000 cycles
}

TEST( integration_frame_budget_counts_automatic_iec_throttle_as_slow )
{
	CHostBus bus;
	CSuperCPU scpu;
	u8 basic[ C64_BASIC_SIZE ];
	u8 kernal[ C64_KERNAL_SIZE ];
	std::memset( basic, 0xEA, sizeof( basic ) );
	std::memset( kernal, 0xEA, sizeof( kernal ) );

	const u8 code[] = { 0xA9, 0x07, 0x8D, 0x00, 0xDD,
	                    0xA9, 0x17, 0x8D, 0x00, 0xDD,
	                    0x4C, 0x0A, 0xE0 };
	std::memcpy( kernal, code, sizeof( code ) );
	kernal[ 0x1FFC ] = 0x00;
	kernal[ 0x1FFD ] = 0xE0;
	scpu.setBasicROM( basic );
	scpu.setKernalROM( kernal );
	CHECK( scpu.init( &bus, SCPU_CORE_6502, 0 ) );

	const u64 ran = scpu.runFrame();
	CHECK( scpu.registers().fastMode() );
	CHECK( scpu.memory().iecThrottleActive() );
	CHECK( ran > 19000 );
	CHECK( ran < 21000 );
}
