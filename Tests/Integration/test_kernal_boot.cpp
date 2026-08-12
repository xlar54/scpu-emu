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

struct CIA2SeedBus : CHostBus
{
	bool allowVerification = true;
	u32 verificationCalls = 0;

	bool verifyC64CIA2DDRA( u8 expected ) override
	{
		verificationCalls++;
		return allowVerification && m_Memory[ 0xDD02 ] == expected;
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

TEST( integration_display_state_writes_never_flush_synchronously )
{
	// A synchronous flush emits a burst, and bursts are never display-safe at
	// arbitrary raster positions. Nor may the write wait for a later border:
	// the CMD splash deliberately toggles $D011 mode bits mid-display, while
	// sprite multiplexers need every $D015 transition. Preserve immediate I/O
	// semantics and leave mirror delivery to the frame scheduler.
	static const u16 displayRegs[] = { 0xD011, 0xD015, 0xD016, 0xD018, 0xDD00 };

	for ( u32 i = 0; i < sizeof displayRegs / sizeof displayRegs[ 0 ]; i++ )
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

		// The staged byte stays queued: only the register write itself went to
		// the machine. The border-scheduled flusher delivers the data within a
		// frame, which is the same transient a real SuperCPU shows.
		CHECK_EQ( f.wb.pending(), 1 );

		bool sawRegWrite = false, sawScreenWrite = false;
		for ( u32 j = 0; j < f.bus.m_LogCount; j++ )
		{
			if ( f.bus.m_Log[ j ].addr == reg && f.bus.m_Log[ j ].op == HOSTOP_WRITE ) sawRegWrite = true;
			if ( f.bus.m_Log[ j ].addr == 0x0400 ) sawScreenWrite = true;
		}
		CHECK( sawRegWrite );
		CHECK( !sawScreenWrite );

		// And the border flush delivers it afterwards.
		f.wb.flush();
		CHECK_EQ( f.bus.m_Memory[ 0x0400 ], 0x41 );
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
	// VICE ORs reset's low optimization flags ($07) into every status read.
	CHECK_EQ( v & 0xC0, SCPU_VERSION_V2 );
	CHECK_EQ( v & 0x07, 0x07 );

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

	// $D0B3: the complete optimisation register. Low flags are significant;
	// $80 maps to the BASIC-only $0400-$07FF range.
	f.regs.ioWrite( SCPU_REG_OPTIM_V2, 0x80 );
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

TEST( integration_d0b5_defaults_on_and_bit7_is_a_writable_virtual_jiffy_switch )
{
	SystemFixture f;
	f.start();
	f.regs.setSpeedSwitchAllowsTurbo( false );

	u8 v = 0;
	CHECK( f.regs.ioRead( SCPU_REG_SWITCHES, v ) );
	CHECK_EQ( v & SCPU_SWITCH_JIFFYDOS, SCPU_SWITCH_JIFFYDOS );
	CHECK_EQ( v & SCPU_SWITCH_1MHZ, SCPU_SWITCH_1MHZ );

	// This emulator-only switch is deliberately usable as a direct BASIC POKE.
	// $D07E must not be required: opening that bank also swaps KERNAL windows
	// immediately and can strand BASIC or an IRQ in mismatched KERNAL code.
	CHECK( !f.regs.hardwareRegsEnabled() );
	f.mem.write8( SCPU_REG_SWITCHES, 0 );
	CHECK( !f.regs.hardwareRegsEnabled() );
	CHECK( f.regs.ioRead( SCPU_REG_SWITCHES, v ) );
	CHECK_EQ( v & SCPU_SWITCH_JIFFYDOS, 0x00 );
	CHECK_EQ( v & SCPU_SWITCH_1MHZ, SCPU_SWITCH_1MHZ );
	CHECK_EQ( f.bus.m_Cycles, 0 );

	// Reset changes register state, not a physical/virtual switch position.
	f.regs.reset();
	CHECK( f.regs.ioRead( SCPU_REG_SWITCHES, v ) );
	CHECK_EQ( v & SCPU_SWITCH_JIFFYDOS, 0x00 );

	f.mem.write8( SCPU_REG_SWITCHES, SCPU_SWITCH_JIFFYDOS );
	CHECK( f.regs.ioRead( SCPU_REG_SWITCHES, v ) );
	CHECK_EQ( v & SCPU_SWITCH_JIFFYDOS, SCPU_SWITCH_JIFFYDOS );
	CHECK_EQ( v & SCPU_SWITCH_1MHZ, SCPU_SWITCH_1MHZ );
	CHECK( !f.regs.hardwareRegsEnabled() );
}

TEST( integration_configured_jiffy_switch_survives_supercpu_init_and_reset )
{
	CHostBus bus;
	CSuperCPU scpu;
	u8 basic[ C64_BASIC_SIZE ];
	u8 kernal[ C64_KERNAL_SIZE ];
	std::memset( basic, 0xEA, sizeof basic );
	std::memset( kernal, 0xEA, sizeof kernal );
	kernal[ 0x1FFC ] = 0x00;
	kernal[ 0x1FFD ] = 0xE0;
	scpu.setBasicROM( basic );
	scpu.setKernalROM( kernal );

	// boot.cpp applies JIFFYDOS before init(); init() performs a reset, which
	// must preserve the configured virtual switch just like physical hardware.
	scpu.registers().setJiffyDOSSwitch( false );
	CHECK( scpu.init( &bus, SCPU_CORE_6502, 0 ) );

	u8 v = 0;
	CHECK( scpu.registers().ioRead( SCPU_REG_SWITCHES, v ) );
	CHECK_EQ( v & SCPU_SWITCH_JIFFYDOS, 0x00 );
	scpu.reset();
	CHECK( scpu.registers().ioRead( SCPU_REG_SWITCHES, v ) );
	CHECK_EQ( v & SCPU_SWITCH_JIFFYDOS, 0x00 );
}

TEST( integration_enhanced_optimisation_bits_select_the_vice_mirror_ranges )
{
	SystemFixture f;
	const u8 code[] = { 0xEA };
	f.installKernal( 0xE000, code, sizeof( code ), 0xE000 );
	f.start();

	// Hardware reset is $C7, which VICE maps to $0200-$FFFF.
	CHECK_EQ( f.regs.optimRegister(), 0xC7 );
	CHECK_EQ( f.wb.optMode(), SCPU_OPT_DEFAULT );
	CHECK( !f.wb.shouldMirror( 0x01FF ) );
	CHECK(  f.wb.shouldMirror( 0x0200 ) );

	f.regs.ioWrite( SCPU_REG_HWREGS_ENABLE, 0 );

	f.regs.ioWrite( SCPU_REG_OPTIM_V2, 0x84 );	// VICE mirror index 10: empty
	CHECK_EQ( f.wb.optMode(), SCPU_OPT_FULL );
	CHECK( !f.wb.shouldMirror( 0x0400 ) );

	f.regs.ioWrite( SCPU_REG_OPTIM_V2, 0xC1 );	// index 13: $0200-$FFFF
	CHECK_EQ( f.wb.optMode(), SCPU_OPT_DEFAULT );
	CHECK( !f.wb.shouldMirror( 0x01FF ) );
	CHECK(  f.wb.shouldMirror( 0x0200 ) );

	f.regs.ioWrite( SCPU_REG_OPTIM_V2, 0x04 );	// index 2: $0000-$3FFF
	CHECK_EQ( f.wb.optMode(), SCPU_OPT_VICBANK0 );
	CHECK(  f.wb.shouldMirror( 0x0000 ) );
	CHECK(  f.wb.shouldMirror( 0x3FFF ) );
	CHECK( !f.wb.shouldMirror( 0x4000 ) );

	f.regs.ioWrite( SCPU_REG_OPTIM_V2, 0x44 );	// index 6: $C000-$FFFF
	CHECK_EQ( f.wb.optMode(), SCPU_OPT_VICBANK3 );
	CHECK( !f.wb.shouldMirror( 0xBFFF ) );
	CHECK(  f.wb.shouldMirror( 0xC000 ) );
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
	CHECK_EQ( f.regs.optimRegister() & 0x07, 0x00 );

	// Speed selection is NOT gated -- that is what makes POKE 53370,0 work
	// straight from BASIC without opening the bank first.
	f.regs.ioWrite( SCPU_REG_HWREGS_DISABLE, 0 );
	f.regs.ioWrite( SCPU_REG_SOFT_1MHZ_ON, 0 );
	CHECK( !f.regs.fastMode() );
	f.regs.ioWrite( SCPU_REG_SOFT_1MHZ_OFF, 0 );
	CHECK( f.regs.fastMode() );
}

TEST( integration_register_attach_applies_the_power_on_mirror_policy )
{
	CWriteBuffer wb;
	wb.setOptMode( SCPU_OPT_FULL );
	CSuperCPURegisters regs;
	regs.attach( &wb );

	CHECK_EQ( regs.optimRegister(), 0xC7 );
	CHECK_EQ( wb.optMode(), SCPU_OPT_DEFAULT );
	CHECK( !wb.shouldMirror( 0x0100 ) );
	CHECK( wb.shouldMirror( 0x0200 ) );
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
	f.cpu.step();		// first STA: unknown pin state is conservatively active
	CHECK( f.mem.iecThrottleActive() );
	f.cpu.step();
	f.cpu.step();		// IEC bit change refreshes the fallback

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
	f.start();

	// Establish both hidden CIA baselines, then let the conservative first-write
	// window expire. With PA3-PA5 configured as outputs, changing only PA0/PA1
	// (the VIC bank) must not look like serial traffic.
	f.mem.write8( 0xDD02, 0x3F );
	f.mem.write8( 0xDD00, 0x37 );
	for ( u32 i = 0; i < 110000; i++ ) f.mem.tickFast( 1 );
	CHECK( !f.mem.iecThrottleActive() );
	const u64 events = f.mem.m_IECThrottleEvents;

	f.mem.write8( 0xDD00, 0x36 );

	CHECK( !f.mem.iecThrottleActive() );
	CHECK_EQ( f.mem.m_IECThrottleEvents, events );
}

TEST( integration_cia2_receive_inputs_are_active_low )
{
	SystemFixture f;
	f.start();

	// PA6/PA7 are inputs. Both high is the idle IEC bus and must not start a
	// transaction merely because software sampled it.
	f.bus.m_Memory[ 0xDD02 ] = 0x00;
	CHECK_EQ( f.mem.read8( 0xDD02 ), 0x00 );
	f.bus.m_Memory[ 0xDD00 ] = 0xFF;
	CHECK_EQ( f.mem.read8( 0xDD00 ), 0xFF );
	CHECK( !f.mem.iecBusActive() );

	// DATA-IN low means the drive asserted the line. This is the state seen in
	// the hardware freeze as $DD00=$3F (both receive inputs low).
	f.bus.m_Memory[ 0xDD00 ] = 0x7F;
	CHECK_EQ( f.mem.read8( 0xDD00 ), 0x7F );
	CHECK( f.mem.iecBusActive() );
}

TEST( integration_cia2_held_receive_line_refreshes_activity )
{
	SystemFixture f;
	f.start();
	f.bus.m_Memory[ 0xDD02 ] = 0x00;
	f.mem.read8( 0xDD02 );
	f.bus.m_Memory[ 0xDD00 ] = 0x7F;
	f.mem.read8( 0xDD00 );

	// Let the original 2ms activity window expire almost completely, then a
	// repeated poll of the still-low line must extend the live transaction.
	for ( u32 i = 0; i < 1500; i++ ) f.mem.tickFast( 1 );
	f.mem.read8( 0xDD00 );
	for ( u32 i = 0; i < 1500; i++ ) f.mem.tickFast( 1 );
	CHECK( f.mem.iecBusActive() );
}

TEST( integration_dd02_direction_change_can_start_iec_activity )
{
	SystemFixture f;
	f.start();

	// Store a high ATN latch while PA3 is an input: no physical drive yet.
	f.mem.write8( 0xDD02, 0x00 );
	f.mem.write8( 0xDD00, 0x08 );
	for ( u32 i = 0; i < 110000; i++ ) f.mem.tickFast( 1 );
	CHECK( !f.mem.iecBusActive() );

	// Making PA3 an output now asserts the inverted open-collector line.
	f.mem.write8( 0xDD02, 0x08 );
	CHECK( f.mem.iecBusActive() );
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
	mem.tickFast( 7 );
	CHECK( mem.m_EmuCycles > 0 );
	CHECK( mem.m_MaxTickChunk > 0 );
	CHECK( mem.m_IOLogPos > 0 );
	CHECK( mem.m_CIALogPos > 0 );

	bus.m_NMI = false;
	mem.reset();
	CHECK( !mem.iecThrottleActive() );
	CHECK( !mem.iecBusActive() );
	CHECK_EQ( mem.m_IECThrottleEvents, 0 );
	CHECK_EQ( mem.m_EmuCycles, 0 );
	CHECK_EQ( mem.m_MaxTickChunk, 0 );
	CHECK_EQ( mem.m_IOLogPos, 0 );
	CHECK_EQ( mem.m_CIALogPos, 0 );
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

TEST( integration_serial_toggles_on_dd00_do_not_flush )
{
	// The IEC-killing regression, pinned. $DD00 is both the VIC bank select
	// AND the serial port: ATN, CLK and DATA live in bits 3-5 and an IEC
	// transfer toggles them thousands of times. Flushing the dirty buffer on
	// every toggle inserted milliseconds of stall between protocol edges; the
	// drive timed out mid-byte and the KERNAL hung forever in its no-timeout
	// receive wait. Only bits 0-1 -- the VIC bank -- may trigger the flush.
	SystemFixture f;
	const u8 code[] = {
		0xA9, 0x41, 0x8D, 0x00, 0x04,	// LDA #$41 / STA $0400  (dirty a byte)
		0xAD, 0x00, 0xDD,				// LDA $DD00             (seed baseline $97)
		0xA9, 0x87, 0x8D, 0x00, 0xDD,	// STA $DD00  CLK toggled  (bit 4: $97->$87)
		0xA9, 0x97, 0x8D, 0x00, 0xDD,	// STA $DD00  CLK back
		0xA9, 0xB7, 0x8D, 0x00, 0xDD,	// STA $DD00  DATA toggled (bit 5)
		0xA9, 0x96, 0x8D, 0x00, 0xDD	// STA $DD00  VIC BANK changed (bits 0-1)
	};
	f.installKernal( 0xE000, code, sizeof( code ), 0xE000 );
	f.start();
	f.wb.setOptMode( SCPU_OPT_NONE );
	f.bus.m_Memory[ 0xDD00 ] = 0x97;

	// Through the three serial-bit toggles the dirty byte must stay queued.
	// (9 instructions: the two setup, the baseline read, then three LDA/STA
	// pairs ending with the DATA toggle.)
	for ( int i = 0; i < 9; i++ ) f.cpu.step();
	CHECK_EQ( f.wb.pending(), 1 );

	// The VIC-bank change no longer flushes synchronously either -- nothing
	// does; see integration_display_state_writes_never_flush_synchronously.
	f.cpu.step(); f.cpu.step();
	CHECK_EQ( f.wb.pending(), 1 );

	f.wb.flush();
	CHECK_EQ( f.bus.m_Memory[ 0x0400 ], 0x41 );
}

TEST( integration_iec_activity_window_expires )
{
	// The black-screen regression, pinned. The serial-activity window
	// suppresses mirror flushing during a transaction; if it never counts
	// down, one boot-time serial probe suppresses mirroring FOREVER and the
	// machine runs perfectly behind a blank display. Arm it, run emulated
	// time past the window, and require it to expire.
	SystemFixture f;
	const u8 code[] = { 0xEA, 0x4C, 0x00, 0xE0 };	// NOP / JMP loop
	f.installKernal( 0xE000, code, sizeof( code ), 0xE000 );
	f.start();

	// Seed the baseline, then a serial-line transition arms the window.
	f.mem.read8( 0xDD00 );
	u8 v = f.bus.m_Memory[ 0xDD00 ];
	f.mem.write8( 0xDD00, (u8)( v ^ 0x08 ) );
	CHECK( f.mem.iecBusActive() );

	// The transaction gate spans IEC's EOI pause and remains active until the
	// full 2000-cycle quiet tail has elapsed.
	for ( u32 i = 0; i < 1999; i++ ) f.mem.tickFast( 1 );
	CHECK( f.mem.iecBusActive() );
	f.mem.tickFast( 1 );
	CHECK( !f.mem.iecBusActive() );

	// Mirror suppression and speed selection are intentionally independent:
	// the automatic 100000-cycle 1MHz fallback remains armed after the serial
	// transaction itself is quiet, so shortening the display blackout cannot
	// make the remainder of disk handling sprint at turbo speed.
	CHECK( f.mem.iecThrottleActive() );
	for ( u32 i = 2000; i < 100000; i++ ) f.mem.tickFast( 1 );
	CHECK( !f.mem.iecThrottleActive() );
}

TEST( integration_bulk_display_flip_never_waits_or_flushes_synchronously )
{
	// Raster IRQ code changes these registers at exact scanlines. Delaying the
	// I/O write while polling toward a border changes program semantics and the
	// polling itself consumes the physical bus through visible VIC fetches.
	// Large queues therefore follow the same immediate-write rule as small ones.
	SystemFixture f;
	const u8 code[] = { 0xA9, 0x3B, 0x8D, 0x18, 0xD0 };	// LDA #$3B / STA $D018
	f.installKernal( 0xE000, code, sizeof( code ), 0xE000 );
	f.start();
	f.wb.setOptMode( SCPU_OPT_NONE );

	// Stage a bitmap's worth of data -- well past the trivial threshold.
	for ( u32 i = 0; i < 100; i++ ) f.mem.write8( (u16)( 0x2000 + i ), (u8)i );
	CHECK( f.wb.pending() >= 100 );
	f.bus.resetStats();

	f.cpu.step(); f.cpu.step();		// LDA, then the flip

	CHECK_EQ( f.wb.pending(), 100 );
	CHECK_EQ( f.bus.m_Reads, 0 );		// no raster polling
	CHECK_EQ( f.bus.m_Writes, 1 );		// only the direct $D018 write
	CHECK_EQ( f.bus.m_Memory[ 0xD018 ], 0x3B );

	// The normal raster scheduler (represented here by an explicit test drain)
	// owns delivery of the queued RAM bytes.
	f.wb.flush();
	CHECK_EQ( f.bus.m_Memory[ 0x2063 ], 0x63 );
}

TEST( integration_unknown_raster_never_authorizes_a_flush )
{
	struct UnknownRasterBus : CHostBus
	{
		u32 samples = 0;
		u16 rasterLine() override { samples++; return 0xFFFF; }
	} bus;

	CSuperCPU scpu;
	u8 basic[ C64_BASIC_SIZE ];
	u8 kernal[ C64_KERNAL_SIZE ];
	std::memset( basic, 0xEA, sizeof basic );
	std::memset( kernal, 0xEA, sizeof kernal );
	kernal[ 0 ] = 0x4C;		// JMP $E000
	kernal[ 1 ] = 0x00;
	kernal[ 2 ] = 0xE0;
	kernal[ 0x1FFC ] = 0x00;
	kernal[ 0x1FFD ] = 0xE0;
	scpu.setBasicROM( basic );
	scpu.setKernalROM( kernal );
	CHECK( scpu.init( &bus, SCPU_CORE_6502, 0 ) );
	scpu.writeBuffer().setOptMode( SCPU_OPT_NONE );

	for ( u32 i = 0; i < 100; i++ )
		scpu.memory().write8( (u16)( 0x2000 + i ), (u8)i );
	bus.resetStats();

	scpu.runFrame();
	CHECK( bus.samples > 0 );
	CHECK_EQ( bus.m_BurstWrites, 0 );
	CHECK_EQ( scpu.writeBuffer().pending(), 100 );

	CHECK( !c64RasterIsSafeForBulkTransfer( VIDEO_PAL, 0xFFFF ) );
	CHECK( !c64RasterIsSafeForBulkTransfer( VIDEO_PAL, 312 ) );
	CHECK( !c64RasterIsSafeForBulkTransfer( VIDEO_NTSC_R56A, 511 ) );
}

TEST( integration_mirror_drain_rechecks_raster_before_each_small_chunk )
{
	struct OneSafeSampleBus : CHostBus
	{
		u32 samples = 0;
		u16 rasterLine() override
		{
			// The first sample is in the top border. Every later sample is in
			// the visible display, so exactly one guarded chunk may leave.
			return samples++ == 0 ? 0 : 100;
		}
	} bus;

	CSuperCPU scpu;
	u8 basic[ C64_BASIC_SIZE ];
	u8 kernal[ C64_KERNAL_SIZE ];
	std::memset( basic, 0xEA, sizeof basic );
	std::memset( kernal, 0xEA, sizeof kernal );
	kernal[ 0 ] = 0x4C;              // JMP $E000
	kernal[ 1 ] = 0x00;
	kernal[ 2 ] = 0xE0;
	kernal[ 0x1FFC ] = 0x00;
	kernal[ 0x1FFD ] = 0xE0;
	scpu.setBasicROM( basic );
	scpu.setKernalROM( kernal );
	CHECK( scpu.init( &bus, SCPU_CORE_6502, 0 ) );
	scpu.writeBuffer().setOptMode( SCPU_OPT_NONE );
	scpu.setMirrorDisplayBudget( 0 );		// strict border-only

	for ( u32 i = 0; i < 200; i++ )
		scpu.memory().write8( (u16)( 0x2000 + i ), (u8)i );
	CHECK_EQ( scpu.writeBuffer().pending(), 200 );
	bus.resetStats();
	bus.samples = 0;

	scpu.runFrame();

	CHECK( bus.samples > 1 );
	CHECK_EQ( bus.m_BurstWrites, 64 );
	CHECK_EQ( scpu.writeBuffer().pending(), 136 );
}

TEST( integration_mirror_drains_inside_the_display_within_its_ration )
{
	// Border-only mirroring bounds delivery at roughly 3KB per frame, and only
	// on frames where a raster sample catches the border at all. A game that
	// redraws its moving objects every frame outruns that, so the VIC fetches a
	// mixture of several frames' bytes for exactly the parts that move while
	// static scenery -- delivered long ago -- stays perfect.
	//
	// Writing inside the picture is safe: the burst path re-samples BA and
	// hands the bus back whenever the VIC claims it, which is what a real REU
	// does. So a display ration must drain, and must stop at the ration.
	struct AlwaysDisplayBus : CHostBus
	{
		u16 rasterLine() override { return 100; }	// never in the border
	} bus;

	CSuperCPU scpu;
	u8 basic[ C64_BASIC_SIZE ];
	u8 kernal[ C64_KERNAL_SIZE ];
	std::memset( basic, 0xEA, sizeof basic );
	std::memset( kernal, 0xEA, sizeof kernal );
	kernal[ 0 ] = 0x4C;              // JMP $E000
	kernal[ 1 ] = 0x00;
	kernal[ 2 ] = 0xE0;
	kernal[ 0x1FFC ] = 0x00;
	kernal[ 0x1FFD ] = 0xE0;
	scpu.setBasicROM( basic );
	scpu.setKernalROM( kernal );
	CHECK( scpu.init( &bus, SCPU_CORE_6502, 0 ) );
	scpu.writeBuffer().setOptMode( SCPU_OPT_NONE );
	scpu.setMirrorDisplayBudget( 128 );

	for ( u32 i = 0; i < 4000; i++ )
		scpu.memory().write8( (u16)( 0x2000 + i ), (u8)i );
	const u32 staged = scpu.writeBuffer().pending();
	CHECK( staged >= 4000 );
	bus.resetStats();

	scpu.runFrame();

	// The legacy 128-byte allowance is divided across 128 opportunities. That
	// preserves roughly the former per-frame ceiling while turning each long
	// pause into a small installment.
	const u32 sent = staged - scpu.writeBuffer().pending();
	CHECK( sent >= 128 );
	CHECK( sent <= 9 * 128 );
}

TEST( integration_border_only_mode_sends_nothing_inside_the_display )
{
	// The escape hatch: MIRROR_DISPLAY_BYTES 0 restores the old rule exactly,
	// so a card can back this out without a rebuild.
	struct AlwaysDisplayBus : CHostBus
	{
		u16 rasterLine() override { return 100; }
	} bus;

	CSuperCPU scpu;
	u8 basic[ C64_BASIC_SIZE ];
	u8 kernal[ C64_KERNAL_SIZE ];
	std::memset( basic, 0xEA, sizeof basic );
	std::memset( kernal, 0xEA, sizeof kernal );
	kernal[ 0 ] = 0x4C;
	kernal[ 1 ] = 0x00;
	kernal[ 2 ] = 0xE0;
	kernal[ 0x1FFC ] = 0x00;
	kernal[ 0x1FFD ] = 0xE0;
	scpu.setBasicROM( basic );
	scpu.setKernalROM( kernal );
	CHECK( scpu.init( &bus, SCPU_CORE_6502, 0 ) );
	scpu.writeBuffer().setOptMode( SCPU_OPT_NONE );
	scpu.setMirrorDisplayBudget( 0 );

	for ( u32 i = 0; i < 200; i++ )
		scpu.memory().write8( (u16)( 0x2000 + i ), (u8)i );
	bus.resetStats();

	scpu.runFrame();
	CHECK_EQ( bus.m_BurstWrites, 0 );
	CHECK_EQ( scpu.writeBuffer().pending(), 200 );
}

TEST( integration_polling_dd00_with_idle_lines_never_latches_iec_activity )
{
	// The mirror-blackout latch-up, pinned. Re-arming the activity window on
	// any $DD00 read once it happened to be open made it self-sustaining: that
	// window is only ~2.5ms of real time at 20MHz, so ordinary code reading
	// $DD00 a few times a frame held it open forever. iecBusActive() gates ALL
	// mirroring, so the physical screen froze while the machine ran on.
	SystemFixture f;
	f.start();
	f.mem.write8( 0xDD02, 0x3F );		// PA6/PA7 inputs, as the KERNAL sets

	// A real transaction opens the window.
	f.bus.m_Memory[ 0xDD00 ] = 0x7F;		// DATA in low: a drive is talking
	f.mem.read8( 0xDD00 );
	CHECK( f.mem.iecBusActive() );

	// The drive releases. From here on the lines are idle high and the only
	// traffic is a game polling the register -- which must not hold the
	// window open. Poll far more often than the window is long.
	f.bus.m_Memory[ 0xDD00 ] = 0xFF;
	for ( u32 i = 0; i < 200000; i++ )
	{
		if ( ( i % 1000 ) == 0 ) f.mem.read8( 0xDD00 );
		f.mem.tickFast( 1 );
	}
	CHECK( !f.mem.iecBusActive() );
}

TEST( integration_frame_end_runs_the_cpu_toward_the_border_instead_of_missing_it )
{
	// Every other drain is opportunistic: sample the raster, transfer only if
	// the beam happens to be outside the picture. The border is under a
	// quarter of the frame, so a meaningful share of frames delivered NOTHING
	// -- and a frame that delivers nothing shows the previous frame's objects,
	// which is a flicker no budget size can fix.
	//
	// At frame end the machine now approaches the border deliberately: it runs
	// the emulated CPU for exactly the time the beam needs to get there. The
	// wait is useful work rather than a bus poll, and arrival is by
	// construction rather than by luck.
	struct FixedRasterBus : CHostBus
	{
		u16 line = 250;
		u16 rasterLine() override { return line; }
	};

	auto runOne = []( u16 rasterAt ) -> u64
	{
		static FixedRasterBus bus;
		bus.line = rasterAt;
		static CSuperCPU scpu;
		u8 basic[ C64_BASIC_SIZE ];
		u8 kernal[ C64_KERNAL_SIZE ];
		std::memset( basic, 0xEA, sizeof basic );
		std::memset( kernal, 0xEA, sizeof kernal );
		kernal[ 0 ] = 0x4C;              // JMP $E000
		kernal[ 1 ] = 0x00;
		kernal[ 2 ] = 0xE0;
		kernal[ 0x1FFC ] = 0x00;
		kernal[ 0x1FFD ] = 0xE0;
		scpu.setBasicROM( basic );
		scpu.setKernalROM( kernal );
		CHECK( scpu.init( &bus, SCPU_CORE_6502, 0 ) );
		scpu.writeBuffer().setOptMode( SCPU_OPT_NONE );
		scpu.setMirrorDisplayBudget( 0 );

		for ( u32 i = 0; i < 100; i++ )
			scpu.memory().write8( (u16)( 0x2000 + i ), (u8)i );
		return scpu.runFrame();
	};

	// Beam already in the bottom border: nothing to approach.
	const u64 atBorder = runOne( 250 );

	// Beam mid-picture: the frame runs on until the border is reached.
	const u64 inDisplay = runOne( 100 );

	CHECK( inDisplay > atBorder );
}

TEST( integration_frame_end_border_approach_is_repaid_next_frame )
{
	// Advancing to the physical border is useful work, but it must not be free
	// work. A fixed mid-display sample makes the first call borrow a known
	// amount; the next call must execute less ordinary frame work while repaying
	// it. Otherwise the sampled raster phase becomes a variable CPU overclock.
	struct FixedRasterBus : CHostBus
	{
		u16 rasterLine() override { return 100; }
	} bus;

	CSuperCPU scpu;
	u8 basic[ C64_BASIC_SIZE ];
	u8 kernal[ C64_KERNAL_SIZE ];
	std::memset( basic, 0xEA, sizeof basic );
	std::memset( kernal, 0xEA, sizeof kernal );
	kernal[ 0 ] = 0x4C;
	kernal[ 1 ] = 0x00;
	kernal[ 2 ] = 0xE0;
	kernal[ 0x1FFC ] = 0x00;
	kernal[ 0x1FFD ] = 0xE0;
	scpu.setBasicROM( basic );
	scpu.setKernalROM( kernal );
	CHECK( scpu.init( &bus, SCPU_CORE_6502, 0 ) );
	scpu.writeBuffer().setOptMode( SCPU_OPT_NONE );
	scpu.setMirrorDisplayBudget( 0 );

	for ( u32 i = 0; i < 100; i++ )
		scpu.memory().write8( (u16)( 0x2000 + i ), (u8)i );
	const u64 borrowingFrame = scpu.runFrame();

	// Keep a mirror outstanding so this call takes the same border-approach
	// path. Its total must nevertheless be one normal frame, not frame+approach.
	for ( u32 i = 0; i < 100; i++ )
		scpu.memory().write8( (u16)( 0x2100 + i ), (u8)i );
	const u64 repayingFrame = scpu.runFrame();

	CHECK( repayingFrame < borrowingFrame );
}

TEST( integration_active_screen_sprite_pointers_reach_the_bus_immediately )
{
	// Double-buffered sprite animation flips a pointer in vblank and starts
	// redrawing the retired block at once. A pointer flip that arrives even a
	// fraction of a frame late leaves the physical VIC displaying the block
	// the game is now overwriting -- sprites flicker between correct and
	// garbage at animation rate. So the ACTIVE screen's eight pointer bytes
	// bypass the queue and land on the real bus like a VIC register write;
	// everything else keeps queueing.
	SystemFixture f;
	f.start();
	f.wb.setOptMode( SCPU_OPT_NONE );

	// Default machine: bank 0, $D018=$14 -> screen $0400, pointers $07F8.
	f.mem.write8( 0x07F8, 0x55 );
	CHECK_EQ( f.bus.m_Memory[ 0x07F8 ], 0x55 );		// immediate, no flush

	// A neighbouring screen byte still queues.
	f.mem.write8( 0x07F0, 0x66 );
	CHECK( f.bus.m_Memory[ 0x07F0 ] != 0x66 );

	// Move the screen: $D018 top nibble selects matrix $2000, pointers $23F8.
	f.mem.write8( 0xD018, 0x80 );
	f.mem.write8( 0x23F8, 0x77 );
	CHECK_EQ( f.bus.m_Memory[ 0x23F8 ], 0x77 );
	f.mem.write8( 0x07F8, 0x99 );					// old row no longer special
	CHECK( f.bus.m_Memory[ 0x07F8 ] != 0x99 );

	// And the queued copies still flush the same values later.
	f.wb.flush();
	CHECK_EQ( f.bus.m_Memory[ 0x07F0 ], 0x66 );
	CHECK_EQ( f.bus.m_Memory[ 0x07F8 ], 0x99 );
}

TEST( integration_active_screen_base_tracks_d018_and_vic_bank )
{
	SystemFixture f;
	f.start();

	CHECK_EQ( f.mem.activeScreenBase(), 0x0400u );
	f.mem.write8( 0xD018, 0xDF );			// matrix offset $3400
	CHECK_EQ( f.mem.activeScreenBase(), 0x3400u );
	f.mem.write8( 0xDD00, 0xC6 );			// VIC bank 1 ($4000)
	CHECK_EQ( f.mem.activeScreenBase(), 0x7400u );	// Metal Dust snapshot
	f.mem.write8( 0xDD02, 0x00 );			// PA0/PA1 inputs float high
	CHECK_EQ( f.mem.activeScreenBase(), 0x3400u );	// back to VIC bank 0
	f.mem.write8( 0xDD02, 0x03 );			// drive both bank-select pins
	CHECK_EQ( f.mem.activeScreenBase(), 0x7400u );

	// A bank-3 matrix at $D000 is behind the real machine's I/O window and
	// therefore cannot participate in queued DRAM mirroring.
	f.mem.write8( 0xDD00, 0xC4 );			// VIC bank 3 ($C000)
	f.mem.write8( 0xD018, 0x40 );			// matrix offset $1000
	CHECK_EQ( f.mem.activeScreenBase(), 0xFFFFFFFFu );
}

TEST( integration_active_bitmap_base_tracks_mode_d018_and_vic_bank )
{
	SystemFixture f;
	f.start();

	CHECK_EQ( f.mem.activeBitmapBase(), 0xFFFFFFFFu );
	f.mem.write8( 0xD011, 0x3B );			// bitmap mode on
	CHECK_EQ( f.mem.activeBitmapBase(), 0x0000u );
	f.mem.write8( 0xD018, 0x1C );			// bitmap high half
	CHECK_EQ( f.mem.activeBitmapBase(), 0x2000u );
	f.mem.write8( 0xDD00, 0xC6 );			// VIC bank 1
	CHECK_EQ( f.mem.activeBitmapBase(), 0x6000u );

	// Bitmap selection remains known even when the screen matrix is under I/O.
	f.mem.write8( 0xDD00, 0xC4 );			// VIC bank 3
	f.mem.write8( 0xD018, 0x40 );			// screen $D000, bitmap $C000
	CHECK_EQ( f.mem.activeScreenBase(), 0xFFFFFFFFu );
	CHECK_EQ( f.mem.activeBitmapBase(), 0xC000u );
	f.mem.write8( 0xD011, 0x1B );			// bitmap mode off
	CHECK_EQ( f.mem.activeBitmapBase(), 0xFFFFFFFFu );
}

TEST( integration_active_charset_base_tracks_d018_bank_and_character_rom )
{
	SystemFixture f;
	f.start();

	// Default D018=$14 selects the VIC's internal character ROM at $1000.
	CHECK_EQ( f.mem.activeCharsetBase(), 0xFFFFFFFFu );
	f.mem.write8( 0xD018, 0x02 );			// bank-0 DRAM charset $0800
	CHECK_EQ( f.mem.activeCharsetBase(), 0x0800u );
	f.mem.write8( 0xDD00, 0xC6 );			// VIC bank 1
	CHECK_EQ( f.mem.activeCharsetBase(), 0x4800u );
	f.mem.write8( 0xD018, 0x04 );			// bank-1 DRAM charset $5000
	CHECK_EQ( f.mem.activeCharsetBase(), 0x5000u );
	f.mem.write8( 0xDD00, 0xC5 );			// VIC bank 2
	f.mem.write8( 0xD018, 0x02 );			// offset $0800 is DRAM
	CHECK_EQ( f.mem.activeCharsetBase(), 0x8800u );
	f.mem.write8( 0xD018, 0x06 );			// offset $1800 is character ROM
	CHECK_EQ( f.mem.activeCharsetBase(), 0xFFFFFFFFu );
	f.mem.write8( 0xD011, 0x3B );			// bitmap mode suppresses charset
	CHECK_EQ( f.mem.activeCharsetBase(), 0xFFFFFFFFu );
}

TEST( integration_d011_read_uses_shadowed_control_and_live_raster_high_bit )
{
	SystemFixture f;
	f.start();

	// Establish a valid hires-bitmap control latch through the same write path
	// used by the emulated CPU, then corrupt only the physical readback. GEOS
	// reads, masks bit 7 and writes this value back; the low seven bits must not
	// be allowed to inherit expansion-bus residue.
	f.mem.write8( 0xD011, 0x3B );
	f.bus.m_Memory[ 0xD011 ] = 0x7F;
	CHECK_EQ( f.mem.read8( 0xD011 ), (u8)0x3B );

	// Bit 7 remains live and comes from the physical raster counter rather than
	// the shadowed compare value.
	f.bus.m_Cycles = (u64)300 * c64CyclesPerLine( VIDEO_PAL );
	CHECK_EQ( f.mem.read8( 0xD011 ), (u8)0xBB );
}

TEST( integration_c64_cia2_seed_repeats_only_the_verified_ddra_write )
{
	CIA2SeedBus bus;
	CC64Memory mem;
	mem.attachBus( &bus );
	mem.reset();
	bus.m_LogEnabled = true;

	CHECK( mem.initializeC64CIA2() );
	CHECK_EQ( bus.verificationCalls, 1u );
	CHECK_EQ( bus.m_LogCount, 2u );
	CHECK_EQ( bus.m_Log[ 0 ].op, HOSTOP_WRITE );
	CHECK_EQ( bus.m_Log[ 0 ].addr, 0xDD02 );
	CHECK_EQ( bus.m_Log[ 0 ].value, 0x3F );
	CHECK_EQ( bus.m_Log[ 1 ].op, HOSTOP_WRITE );
	CHECK_EQ( bus.m_Log[ 1 ].addr, 0xDD02 );
	CHECK_EQ( bus.m_Log[ 1 ].value, 0x3F );

	// Ordinary guest writes remain single accesses; the repeat is confined to
	// the dedicated pure-latch initialization above.
	mem.write8( 0xDD02, 0x37 );
	CHECK_EQ( bus.m_LogCount, 3u );
	CHECK_EQ( bus.m_Log[ 2 ].value, 0x37 );
}

TEST( integration_failed_cia2_seed_verification_leaves_ddra_unknown )
{
	CIA2SeedBus bus;
	CC64Memory mem;
	mem.attachBus( &bus );
	mem.reset();
	bus.allowVerification = false;

	CHECK( !mem.initializeC64CIA2() );
	bus.m_Memory[ 0xDD02 ] = 0xA5;
	bus.resetStats();
	CHECK_EQ( mem.read8( 0xDD02 ), (u8)0xA5 );
	CHECK_EQ( bus.m_Reads, 1u );
}

TEST( integration_verified_cia2_ddra_shadow_rejects_poisoned_physical_reads )
{
	CIA2SeedBus bus;
	CC64Memory mem;
	mem.attachBus( &bus );
	mem.reset();
	CHECK( mem.initializeC64CIA2() );

	bus.m_Memory[ 0xDD02 ] = 0x00;
	bus.resetStats();
	const u8 rmw = mem.read8( 0xDD02 );
	CHECK_EQ( rmw, (u8)0x3F );
	CHECK_EQ( bus.m_Reads, 0u );
	mem.write8( 0xDD02, rmw );
	CHECK_EQ( bus.m_Memory[ 0xDD02 ], (u8)0x3F );
}

TEST( integration_cia2_pra_read_combines_latch_outputs_with_physical_inputs )
{
	CIA2SeedBus bus;
	CC64Memory mem;
	mem.attachBus( &bus );
	mem.reset();
	CHECK( mem.initializeC64CIA2() );
	mem.write8( 0xDD00, 0x35 );

	// DDRA=$3F: PA0-PA5 are outputs from the hidden latch; PA6-PA7 are live
	// physical inputs. Neither half may overwrite the other.
	bus.m_Memory[ 0xDD00 ] = 0xC0;
	CHECK_EQ( mem.read8( 0xDD00 ), (u8)0xF5 );
	bus.m_Memory[ 0xDD00 ] = 0x00;
	CHECK_EQ( mem.read8( 0xDD00 ), (u8)0x35 );
}

TEST( integration_cia2_pra_rmw_residue_cannot_move_the_vic_bank )
{
	CIA2SeedBus bus;
	CC64Memory mem;
	mem.attachBus( &bus );
	mem.reset();
	CHECK( mem.initializeC64CIA2() );
	mem.write8( 0xDD00, 0xC6 );		// VIC bank 1
	CHECK_EQ( mem.activeScreenBase(), 0x4400u );

	// Physical residue claims PA0/PA1 are both low. A read-modify-write must
	// preserve the output latch's %10 bank selection rather than changing it.
	bus.m_Memory[ 0xDD00 ] = 0x00;
	const u8 rmw = mem.read8( 0xDD00 );
	mem.write8( 0xDD00, rmw );
	CHECK_EQ( rmw & 3, (u8)2 );
	CHECK_EQ( mem.activeScreenBase(), 0x4400u );
}

TEST( integration_visible_drain_defers_screen_but_not_cold_traffic )
{
	struct FixedRasterBus : CHostBus
	{
		u16 line = 100;
		u32 maxBurst = 0;
		u16 rasterLine() override { return line; }
		void writeBurst( const C64BusWrite *writes, u32 count ) override
		{
			if ( count > maxBurst ) maxBurst = count;
			CHostBus::writeBurst( writes, count );
		}
	} bus;

	CSuperCPU scpu;
	u8 basic[ C64_BASIC_SIZE ];
	u8 kernal[ C64_KERNAL_SIZE ];
	std::memset( basic, 0xEA, sizeof basic );
	std::memset( kernal, 0xEA, sizeof kernal );
	kernal[ 0 ] = 0x4C;              // JMP $E000
	kernal[ 1 ] = 0x00;
	kernal[ 2 ] = 0xE0;
	kernal[ 0x1FFC ] = 0x00;
	kernal[ 0x1FFD ] = 0xE0;
	scpu.setBasicROM( basic );
	scpu.setKernalROM( kernal );
	CHECK( scpu.init( &bus, SCPU_CORE_6502, 0 ) );
	scpu.writeBuffer().setOptMode( SCPU_OPT_NONE );
	scpu.setMirrorDisplayBudget( 256 );

	// Put the active matrix at the FIFO head, as a scrolling screen does, then
	// queue unrelated VIC-visible RAM behind it.
	for ( u32 i = 0; i < 1000; i++ )
		scpu.memory().write8( (u16)( 0x0400 + i ), (u8)( i ^ 0xA5 ) );
	scpu.memory().write8( 0x2000, 0x5A );

	// With the beam held in the picture, cold traffic progresses past all 1000
	// deferred matrix entries but no matrix byte reaches physical DRAM.
	scpu.runFrame();
	CHECK_EQ( bus.m_Memory[ 0x2000 ], 0x5A );
	CHECK( bus.m_Memory[ 0x0400 ] != (u8)0xA5 );
	CHECK_EQ( scpu.writeBuffer().pending(), 1000u );

	// Once outside the picture, the same small installments prioritize and
	// finish the screen before ordinary border work. The dedicated allowance is
	// 256 bytes per opportunity, but the physical writes remain raster-checked
	// 64-byte bursts.
	bus.line = 251;						// first PAL line outside the picture
	bus.resetStats();
	bus.maxBurst = 0;
	scpu.runFrame();
	CHECK_EQ( bus.m_BurstWrites, 1000u );
	CHECK_EQ( bus.maxBurst, 64u );
	CHECK_EQ( bus.m_Memory[ 0x0400 ], (u8)0xA5 );
	CHECK_EQ( bus.m_Memory[ 0x07E7 ], (u8)( 999 ^ 0xA5 ) );
	CHECK( !scpu.writeBuffer().hasPendingInRange( 0x0400, 1000 ) );
}

TEST( integration_bitmap_drain_is_hidden_and_prioritized )
{
	struct FixedRasterBus : CHostBus
	{
		u16 line = 100;
		u32 maxBurst = 0;
		u16 rasterLine() override { return line; }
		void writeBurst( const C64BusWrite *writes, u32 count ) override
		{
			if ( count > maxBurst ) maxBurst = count;
			CHostBus::writeBurst( writes, count );
		}
	} bus;

	CSuperCPU scpu;
	u8 basic[ C64_BASIC_SIZE ];
	u8 kernal[ C64_KERNAL_SIZE ];
	std::memset( basic, 0xEA, sizeof basic );
	std::memset( kernal, 0xEA, sizeof kernal );
	kernal[ 0 ] = 0x4C;
	kernal[ 1 ] = 0x00;
	kernal[ 2 ] = 0xE0;
	kernal[ 0x1FFC ] = 0x00;
	kernal[ 0x1FFD ] = 0xE0;
	scpu.setBasicROM( basic );
	scpu.setKernalROM( kernal );
	CHECK( scpu.init( &bus, SCPU_CORE_6502, 0 ) );
	scpu.writeBuffer().setOptMode( SCPU_OPT_NONE );
	scpu.setMirrorDisplayBudget( 256 );

	// Bank 2, bitmap at $A000. Queue bitmap bytes first and unrelated cold RAM
	// behind them to prove the visible drain skips the entire 8K display span.
	scpu.memory().write8( 0xDD00, 0xC5 );
	scpu.memory().write8( 0xD011, 0x3B );
	scpu.memory().write8( 0xD018, 0x18 );
	CHECK_EQ( scpu.memory().activeBitmapBase(), 0xA000u );
	for ( u32 i = 0; i < 8000; i++ )
		scpu.memory().write8( (u16)( 0xA000 + i ), (u8)( i ^ 0x5A ) );
	scpu.memory().write8( 0x2000, 0xA5 );

	scpu.runFrame();
	CHECK_EQ( bus.m_Memory[ 0x2000 ], 0xA5 );
	CHECK( bus.m_Memory[ 0xA000 ] != (u8)0x5A );
	CHECK( scpu.writeBuffer().hasPendingInRange( 0xA000, 8000 ) );

	// In the hidden window, the bitmap is selected ahead of cold FIFO traffic
	// and completed through individually raster-checked 64-byte bursts.
	bus.line = 251;
	bus.resetStats();
	bus.maxBurst = 0;
	scpu.runFrame();
	CHECK_EQ( bus.m_BurstWrites, 8000u );
	CHECK_EQ( bus.maxBurst, 64u );
	CHECK_EQ( bus.m_Memory[ 0xA000 ], (u8)0x5A );
	CHECK_EQ( bus.m_Memory[ 0xBF3F ], (u8)( 7999 ^ 0x5A ) );
	CHECK( !scpu.writeBuffer().hasPendingInRange( 0xA000, 8000 ) );
}

TEST( integration_display_drain_avoids_sprite_fetch_lines )
{
	// Mid-display delivery must step around enabled sprites' fetch spans:
	// their BA windows are narrow and abrupt -- unlike a badline's long low --
	// and a burst write straddling one is how 3D Pool's static balls got hit
	// by its own re-render traffic. Lines clear of sprites keep delivering.
	struct FixedRasterBus : CHostBus
	{
		u16 line = 100;
		u16 rasterLine() override { return line; }
	} bus;

	CSuperCPU scpu;
	u8 basic[ C64_BASIC_SIZE ];
	u8 kernal[ C64_KERNAL_SIZE ];
	std::memset( basic, 0xEA, sizeof basic );
	std::memset( kernal, 0xEA, sizeof kernal );
	kernal[ 0 ] = 0x4C;              // JMP $E000
	kernal[ 1 ] = 0x00;
	kernal[ 2 ] = 0xE0;
	kernal[ 0x1FFC ] = 0x00;
	kernal[ 0x1FFD ] = 0xE0;
	scpu.setBasicROM( basic );
	scpu.setKernalROM( kernal );
	CHECK( scpu.init( &bus, SCPU_CORE_6502, 0 ) );
	scpu.writeBuffer().setOptMode( SCPU_OPT_NONE );
	scpu.setMirrorDisplayBudget( 256 );

	// Sprite 3 enabled with its span covering raster line 100.
	scpu.memory().write8( 0xD015, 0x08 );
	scpu.memory().write8( 0xD007, 95 );		// sprite 3 Y

	for ( u32 i = 0; i < 600; i++ )
		scpu.memory().write8( (u16)( 0x2000 + i ), (u8)( i ^ 0xA5 ) );
	const u32 staged = scpu.writeBuffer().pending();
	CHECK( staged >= 600 );
	bus.resetStats();

	// Beam parked inside the sprite's fetch span: nothing may burst.
	scpu.runFrame();
	CHECK_EQ( bus.m_BurstWrites, 0 );

	// Beam on a clear line: delivery resumes.
	bus.line = 180;
	scpu.runFrame();
	CHECK( bus.m_BurstWrites > 0 );
}
