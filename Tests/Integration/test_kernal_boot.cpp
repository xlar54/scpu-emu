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

TEST( integration_io_access_does_not_flush_the_mirror_buffer )
{
	// Contract change, deliberate. Flushing before every I/O access meant a
	// booting KERNAL -- which touches I/O constantly -- drove a continuous
	// stream of unscheduled bursts across the visible display, corrupting the
	// VIC-II's fetches. Mirroring is now scheduled against the raster in
	// CSuperCPU::runFrame() instead.
	//
	// The accepted cost: a program that stages a screen and immediately points
	// the VIC at it can show stale data for one frame. The real SuperCPU
	// mirrors asynchronously and has the same property.
	SystemFixture f;
	const u8 code[] = { 0xA9, 0x41, 0x8D, 0x00, 0x04,	// LDA #$41 / STA $0400
	                    0xA9, 0x3B, 0x8D, 0x11, 0xD0 };	// LDA #$3B / STA $D011
	f.installKernal( 0xE000, code, sizeof( code ), 0xE000 );
	f.start();
	f.wb.setOptMode( SCPU_OPT_NONE );
	f.bus.m_LogEnabled = true;

	for ( int i = 0; i < 4; i++ ) f.cpu.step();

	// The screen byte is still queued; only the register write went out.
	CHECK_EQ( f.wb.pending(), 1 );
	CHECK_EQ( f.bus.m_LogCount, 1 );
	CHECK_EQ( f.bus.m_Log[ 0 ].addr, 0xD011 );
	CHECK_EQ( f.bus.m_Log[ 0 ].op, HOSTOP_WRITE );

	// It reaches the machine when the mirror is flushed, not before.
	f.wb.flush();
	CHECK_EQ( f.bus.m_Memory[ 0x0400 ], 0x41 );
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
	CHECK_EQ( v, SCPU_MODE_V2_64 );

	// None of it reaches the machine.
	CHECK_EQ( f.bus.m_Cycles, 0 );
}

TEST( integration_speed_switch_gates_turbo_rather_than_forcing_it )
{
	// Per the 1541 Ultimate report, the physical switch is "force 1MHz, or
	// merely ALLOW 20MHz" -- it does not accelerate the machine by itself, and
	// in the NORMAL position a request for turbo is simply discarded.
	SystemFixture f;
	const u8 code[] = { 0x8D, 0x7B, 0xD0 };	// STA $D07B -- ask for turbo
	f.installKernal( 0xE000, code, sizeof( code ), 0xE000 );

	f.regs.setSpeedSwitchAllowsTurbo( false );
	f.start();

	CHECK( !f.regs.turboEnabled() );		// locked at 1MHz after reset
	f.cpu.step();
	CHECK( !f.regs.turboEnabled() );		// and the request was ignored

	// $D0B5 bit 6 reports the switch, bit 7 still says "accelerator present".
	u8 v;
	CHECK( f.regs.ioRead( 0xD0B5, v ) );
	CHECK_EQ( v & 0x40, SCPU_SWITCH_SPEED_NORMAL );
	CHECK_EQ( v & 0x80, 0x00 );

	// Flip the switch to TURBO: permitted, but not automatic.
	f.regs.setSpeedSwitchAllowsTurbo( true );
	CHECK( f.regs.ioRead( 0xD0B5, v ) );
	CHECK_EQ( v & 0x40, 0x00 );

	f.cpu.m_PC = 0xE000;
	f.cpu.step();
	CHECK( f.regs.turboEnabled() );
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
	SystemFixture f;
	const u8 code[] = { 0xEA };
	f.installKernal( 0xE000, code, sizeof( code ), 0xE000 );
	f.start();

	u8 v;

	// $D0B2: bit7 hardware registers enabled, bit6 system running at 1MHz.
	CHECK( f.regs.ioRead( 0xD0B2, v ) );
	CHECK_EQ( v & SCPU_STATUS_HWREGS, 0x00 );		// bank closed after reset
	CHECK_EQ( v & SCPU_STATUS_1MHZ,   0x00 );		// and in turbo

	f.regs.ioWrite( SCPU_REG_HWREGS_ENABLE, 0 );
	f.regs.ioWrite( SCPU_REG_SPEED_NORMAL, 0 );
	CHECK( f.regs.ioRead( 0xD0B2, v ) );
	CHECK_EQ( v & SCPU_STATUS_HWREGS, SCPU_STATUS_HWREGS );
	CHECK_EQ( v & SCPU_STATUS_1MHZ,   SCPU_STATUS_1MHZ );

	// $D0B4: current optimization mode in bits 7-6.
	f.regs.ioWrite( SCPU_REG_OPT_BASIC, 0 );
	CHECK( f.regs.ioRead( 0xD0B4, v ) );
	CHECK_EQ( v & 0xC0, SCPU_OPTFLAG_BASIC );

	f.regs.ioWrite( SCPU_REG_OPT_VICBANK2, 0 );		// the GEOS setting
	CHECK( f.regs.ioRead( 0xD0B4, v ) );
	CHECK_EQ( v & 0xC0, SCPU_OPTFLAG_VICBANK2 );

	f.regs.ioWrite( SCPU_REG_OPT_NONE, 0 );
	CHECK( f.regs.ioRead( 0xD0B4, v ) );
	CHECK_EQ( v & 0xC0, SCPU_OPTFLAG_NONE );

	// $D0B5: JiffyDOS switch in bit 7.
	f.regs.setJiffyDOSSwitch( true );
	CHECK( f.regs.ioRead( 0xD0B5, v ) );
	CHECK_EQ( v & SCPU_SWITCH_JIFFYDOS, SCPU_SWITCH_JIFFYDOS );

	// $D0B6: the 6502 core is always in emulation mode.
	CHECK( f.regs.ioRead( 0xD0B6, v ) );
	CHECK_EQ( v & SCPU_PROC_EMULATION, SCPU_PROC_EMULATION );

	// $D0B3 is the one writable register, and only while the bank is open.
	CHECK( f.regs.ioWrite( 0xD0B3, 0x5A ) );
	CHECK( f.regs.ioRead( 0xD0B3, v ) );
	CHECK_EQ( v, 0x5A );

	f.regs.ioWrite( SCPU_REG_HWREGS_DISABLE, 0 );
	CHECK( !f.regs.ioWrite( 0xD0B3, 0x11 ) );		// refused with the bank closed
	CHECK( f.regs.ioRead( 0xD0B3, v ) );
	CHECK_EQ( v, 0x5A );							// still readable, unchanged
}
