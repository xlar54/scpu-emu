/*
   SCPU-EMU - boot a real C64 KERNAL with the 65816 core driving.

   The 6502 version of this (test_real_kernal.cpp) proved the banking model and
   shadow RAM. This one proves the actual product: the same genuine Commodore
   ROMs, reaching the same BASIC READY prompt, with a CW65C816 in emulation mode
   as the CPU and the full 24-bit memory map underneath it.

   That is thousands of instructions of unmodified 1982 code -- the RAM test,
   the vector setup, the screen editor, the whole BASIC cold start -- across
   every addressing mode, decimal arithmetic and both banking configurations. It
   is the closest thing to the real machine that runs without hardware.

   Then, having booted, the tests switch the running machine into native mode
   and reach 16MB of SuperRAM, which is the thing a 6502 fundamentally cannot do
   and the reason this core exists.

   The ROM images are not in the repository (see ROMs/README.md). If they are
   absent these tests report as skipped rather than failing, so a fresh clone
   still goes green.

   Copyright (c) 2026 SCPU-EMU contributors, GPLv3 -- see w65c816.h.
*/
#include "../test_framework.h"
#include "../../Source/C64/c64_memory.h"
#include "../../Source/CPU/W65C816/w65c816.h"
#include "../../Source/SuperCPU/write_buffer.h"
#include "../../Source/SuperCPU/fast_ram.h"
#include "../../Source/SuperCPU/memory_map.h"
#include "../../Source/Bus/Host/host_bus.h"

#include <cstdio>

static bool load816ROM( const char *path, u8 *dst, u32 expected )
{
	std::FILE *f = std::fopen( path, "rb" );
	if ( !f ) return false;
	size_t got = std::fread( dst, 1, expected, f );
	std::fclose( f );
	return got == expected;
}

struct Machine816
{
	CHostBus           bus;
	CC64Memory         mem;
	CWriteBuffer       wb;
	CFastRAM           simm;
	CSuperCPUMemoryMap map;
	CW65C816           cpu;
	bool               haveROMs;

	Machine816()
	{
		u8 basic[ C64_BASIC_SIZE ], kernal[ C64_KERNAL_SIZE ], chargen[ C64_CHARROM_SIZE ];

		haveROMs = load816ROM( "ROMs/kernal.rom", kernal, C64_KERNAL_SIZE )
		        && load816ROM( "ROMs/basic.rom",  basic,  C64_BASIC_SIZE );
		if ( !haveROMs ) return;

		mem.setKernalROM( kernal );
		mem.setBasicROM( basic );
		if ( load816ROM( "ROMs/chargen.rom", chargen, C64_CHARROM_SIZE ) )
			mem.setCharROM( chargen );

		mem.attachBus( &bus );
		mem.setMirrorSink( &wb );
		wb.attach( &bus, mem.m_RAM );

		// The whole point: the CPU sees a 24-bit space whose bank 0 is the C64.
		simm.init( SCPU_SIMM_16MB );
		map.attachBank0( &mem );
		map.attachFastRAM( &simm );
		cpu.attachBus( &map );

		// Unconnected CIA port lines read high, so no key appears pressed.
		for ( u32 a = 0xDC00; a <= 0xDCFF; a++ ) bus.m_Memory[ a ] = 0xFF;
		for ( u32 a = 0xDD00; a <= 0xDDFF; a++ ) bus.m_Memory[ a ] = 0xFF;

		mem.reset();
		wb.setOptMode( SCPU_OPT_NONE );
		cpu.reset();
	}

	static char toAscii( u8 c )
	{
		if ( c == 0x20 ) return ' ';
		if ( c >= 0x01 && c <= 0x1A ) return (char)( 'A' + c - 1 );
		if ( c >= 0x30 && c <= 0x39 ) return (char)( '0' + c - 0x30 );
		if ( c == 0x2E ) return '.';
		if ( c == 0x2A ) return '*';
		if ( c == 0x2C ) return ',';
		if ( c == 0x2F ) return '/';
		if ( c == 0x28 ) return '(';
		if ( c == 0x29 ) return ')';
		return ' ';
	}

	bool screenContains( const char *needle ) const
	{
		char screen[ 1001 ];
		for ( u32 i = 0; i < 1000; i++ ) screen[ i ] = toAscii( mem.m_RAM[ 0x0400 + i ] );
		screen[ 1000 ] = 0;
		return std::strstr( screen, needle ) != 0;
	}

	void dumpScreen() const
	{
		std::printf( "\n    screen:\n" );
		for ( u32 row = 0; row < 25; row++ )
		{
			char line[ 41 ];
			for ( u32 col = 0; col < 40; col++ )
				line[ col ] = toAscii( mem.m_RAM[ 0x0400 + row * 40 + col ] );
			line[ 40 ] = 0;
			bool blank = true;
			for ( u32 i = 0; i < 40; i++ ) if ( line[ i ] != ' ' ) blank = false;
			if ( !blank ) std::printf( "    |%s|\n", line );
		}
	}

	// Runs until BASIC prints its prompt, or gives up.
	bool boot()
	{
		for ( u64 i = 0; i < 200; i++ )
		{
			cpu.run( 50000 );
			if ( screenContains( "READY" ) ) return true;
			if ( cpu.m_Stopped ) break;
		}
		return false;
	}
};

TEST( kernal_65816_reaches_basic_ready_prompt )
{
	Machine816 m;
	if ( !m.haveROMs ) { std::printf( "(skipped: no ROMs) " ); return; }

	CHECK_EQ( m.cpu.m_PC, 0xFCE2 );		// documented C64 reset entry point
	CHECK( m.cpu.m_E );					// and it comes up in emulation mode

	const bool ready = m.boot();

	CHECK( !m.cpu.m_Stopped );
	CHECK( ready );
	if ( !ready ) m.dumpScreen();

	// A stock KERNAL never leaves emulation mode, and the invariants must have
	// held for every one of those instructions.
	CHECK( m.cpu.m_E );
	CHECK_EQ( m.cpu.m_NativeInstructions, 0 );
	CHECK_EQ( m.cpu.m_S & 0xFF00, 0x0100 );
	CHECK( m.cpu.m_X <= 0xFF );
	CHECK( m.cpu.m_Y <= 0xFF );
	CHECK_EQ( m.cpu.m_D, 0 );
	CHECK_EQ( m.cpu.m_PBR, 0 );
}

TEST( kernal_65816_prints_the_boot_banner )
{
	Machine816 m;
	if ( !m.haveROMs ) { std::printf( "(skipped: no ROMs) " ); return; }
	m.boot();

	CHECK( m.screenContains( "COMMODORE 64 BASIC" ) );
	CHECK( m.screenContains( "BYTES FREE" ) );

	// The RAM test walks upward until a write fails to read back, and must stop
	// at $A000 where BASIC ROM appears -- which is what yields the canonical
	// figure. Getting this right needs the banking model AND the 65816's
	// addressing to agree with a 6510's for the whole test loop.
	CHECK( m.screenContains( "38911" ) );

	const u16 memTop = (u16)( m.mem.m_RAM[ 0x0037 ] | ( m.mem.m_RAM[ 0x0038 ] << 8 ) );
	CHECK_EQ( memTop, 0xA000 );
}

TEST( kernal_65816_boot_costs_the_same_bus_traffic_as_the_6502 )
{
	Machine816 m;
	if ( !m.haveROMs ) { std::printf( "(skipped: no ROMs) " ); return; }
	m.boot();
	m.wb.flush();

	// Bus bandwidth is the scarce resource -- roughly one byte per microsecond
	// -- so a whole cold start including a 38K RAM test must cost only a few
	// thousand bus cycles. Everything else is served from shadow RAM at no cost.
	// If this climbs into the hundreds of thousands, reads are leaking onto the
	// bus and the acceleration is gone.
	std::printf( "\n    bus cycles: %llu (io r=%llu w=%llu, mirrored=%llu of %llu writes)\n    ",
	             (unsigned long long)m.bus.m_Cycles,
	             (unsigned long long)m.mem.m_IOReads,
	             (unsigned long long)m.mem.m_IOWrites,
	             (unsigned long long)m.wb.m_BytesFlushed,
	             (unsigned long long)m.mem.m_RamWrites );

	CHECK( m.bus.m_Cycles < 100000 );
}

// ---------------------------------------------------------------------------
// Native mode and SuperRAM, on a machine that has actually booted
// ---------------------------------------------------------------------------

TEST( kernal_65816_native_mode_reaches_superram )
{
	Machine816 m;
	if ( !m.haveROMs ) { std::printf( "(skipped: no ROMs) " ); return; }
	if ( !m.boot() ) { CHECK( false ); return; }

	// Drop a native-mode routine into the cassette buffer, the traditional home
	// for machine code on a C64, and run it. It switches to native mode, widens
	// everything, and writes a 16-bit value 8MB up -- an address a 6510 cannot
	// even express.
	static const u8 prog[] = {
		0x18, 0xFB,					// CLC XCE           -> native
		0xC2, 0x30,					// REP #$30          -> 16-bit A, X, Y
		0xA9, 0xCD, 0xAB,			// LDA #$ABCD
		0x8F, 0x00, 0x00, 0x80,		// STA $800000       -> SuperRAM, 8MB up
		0xA9, 0x00, 0x00,			// LDA #$0000
		0xAF, 0x00, 0x00, 0x80,		// LDA $800000       -> read it back
		0xDB						// STP
	};
	std::memcpy( &m.mem.m_RAM[ 0x0334 ], prog, sizeof( prog ) );

	m.cpu.m_PBR = 0x00;
	m.cpu.m_PC  = 0x0334;
	m.cpu.run( 200 );

	CHECK( m.cpu.m_Stopped );			// reached the STP
	CHECK( !m.cpu.m_E );				// and stayed in native mode
	CHECK( !m.cpu.memory8() );
	CHECK_EQ( m.cpu.m_C, 0xABCD );		// the value survived the round trip

	// It really landed in the SIMM, not in bank 0.
	CHECK_EQ( m.simm.read( 0x800000 ), 0xCD );
	CHECK_EQ( m.simm.read( 0x800001 ), 0xAB );

	// And SuperRAM costs nothing on the expansion port: those accesses never
	// touched the C64. This is why the accelerator is fast.
	const u64 busBefore = m.bus.m_Cycles;
	m.cpu.reset();
	m.cpu.m_PBR = 0x00;
	m.cpu.m_PC  = 0x0334;
	m.cpu.run( 200 );
	CHECK_EQ( m.bus.m_Cycles - busBefore, 0 );
}

TEST( kernal_65816_native_mode_survives_a_return_to_emulation )
{
	Machine816 m;
	if ( !m.haveROMs ) { std::printf( "(skipped: no ROMs) " ); return; }
	if ( !m.boot() ) { CHECK( false ); return; }

	// The pattern real SuperCPU software uses: go native, do 16-bit work, come
	// back, and hand control to the KERNAL as if nothing happened. Coming back
	// destroys XH and YH, so anything worth keeping has to be stored first --
	// which is exactly what this does.
	static const u8 prog[] = {
		0x18, 0xFB,					// CLC XCE
		0xC2, 0x30,					// REP #$30
		0xA9, 0x34, 0x12,			// LDA #$1234
		0xA2, 0x78, 0x56,			// LDX #$5678
		0x8E, 0x00, 0x03,			// STX $0300     save X while it is still 16-bit
		0x38, 0xFB,					// SEC XCE       -> back to emulation
		0xDB						// STP
	};
	std::memcpy( &m.mem.m_RAM[ 0x0334 ], prog, sizeof( prog ) );

	m.cpu.m_PBR = 0x00;
	m.cpu.m_PC  = 0x0334;
	m.cpu.run( 200 );

	CHECK( m.cpu.m_Stopped );
	CHECK( m.cpu.m_E );

	// The 16-bit X was saved before the mode change, so both halves are in RAM.
	CHECK_EQ( m.mem.m_RAM[ 0x0300 ], 0x78 );
	CHECK_EQ( m.mem.m_RAM[ 0x0301 ], 0x56 );

	// The live register kept only its low byte -- narrowing X is destructive.
	CHECK_EQ( m.cpu.m_X, 0x0078 );

	// The accumulator lost nothing: B still holds $12 and XBA would bring it
	// back. This asymmetry is the single most-missed detail on the chip.
	CHECK_EQ( m.cpu.m_C, 0x1234 );

	// And the emulation-mode invariants are back in force.
	CHECK_EQ( m.cpu.m_S & 0xFF00, 0x0100 );
	CHECK( ( m.cpu.m_P & W65_M ) != 0 );
	CHECK( ( m.cpu.m_P & W65_X ) != 0 );
}

TEST( integration_run_ticks_per_instruction_while_iec_active )
{
	// The intermittent drive-freeze regression, pinned. run() batches bus
	// ticks for speed, which lets consecutive serial-line edges leave the
	// Pi nanoseconds apart instead of paced microseconds apart -- drives
	// sometimes miss edges that close together. While the serial bus is
	// active the loop must tick after EVERY instruction; when it is quiet
	// the batching must come back, or the speed win silently dies.
	Machine816 m;
	if ( !m.haveROMs ) { std::printf( "(skipped: no ROMs) " ); return; }
	m.cpu.attachFastBus( &m.map );

	// A NOP sled looping in RAM, IRQs masked so the KERNAL never runs.
	for ( u32 i = 0; i < 16; i++ ) m.mem.m_RAM[ 0x1000 + i ] = 0xEA;
	m.mem.m_RAM[ 0x1010 ] = 0x4C;	// JMP $1000
	m.mem.m_RAM[ 0x1011 ] = 0x00;
	m.mem.m_RAM[ 0x1012 ] = 0x10;
	m.cpu.m_PBR = 0x00;
	m.cpu.m_PC  = 0x1000;
	m.cpu.m_P  |= W65_I;

	// Quiet bus: chunks must span several instructions (8 NOPs = 16 cycles).
	// I/O, active IEC traffic and an armed CIA2 timer still break the batch at
	// the current instruction; pure code keeps the throughput needed for 20MHz.
	m.mem.m_MaxTickChunk = 0;
	m.cpu.run( 400 );
	CHECK( m.mem.m_MaxTickChunk > 8 );

	// Queued RAM writes are equally quiet when RAD mirror stretching is off.
	// A 095 regression still recorded a discarded timing event for every STA;
	// fineTicksRequired() therefore broke the batch on every screen update and
	// made write-heavy animation such as Metal Dust visibly stutter.
	m.mem.m_RAM[ 0x1020 ] = 0x8D; // STA $0400
	m.mem.m_RAM[ 0x1021 ] = 0x00;
	m.mem.m_RAM[ 0x1022 ] = 0x04;
	m.mem.m_RAM[ 0x1023 ] = 0x4C; // JMP $1020
	m.mem.m_RAM[ 0x1024 ] = 0x20;
	m.mem.m_RAM[ 0x1025 ] = 0x10;
	m.cpu.m_PC = 0x1020;
	m.mem.m_MaxTickChunk = 0;
	m.cpu.run( 400 );
	CHECK_EQ( m.mem.m_MirrorStretchCycles, 0u );
	CHECK( m.mem.m_MaxTickChunk > 8 );
	m.cpu.m_PC = 0x1000;

	// A serial-line edge arms the activity window (and the speed hold).
	m.mem.read8( 0xDD00 );
	const u8 v = m.bus.m_Memory[ 0xDD00 ];
	m.mem.write8( 0xDD00, (u8)( v ^ 0x08 ) );
	CHECK( m.mem.iecBusActive() );

	// Active bus: no chunk may exceed one instruction -- NOP is 2 cycles,
	// JMP is 3, so anything above 3 means a batch slipped through.
	m.mem.m_MaxTickChunk = 0;
	m.cpu.run( 400 );
	CHECK( m.mem.iecBusActive() );
	CHECK( m.mem.m_MaxTickChunk <= 3 );
}

TEST( integration_run_ticks_per_instruction_while_speed_is_explicitly_1mhz )
{
	Machine816 m;
	if ( !m.haveROMs ) { std::printf( "(skipped: no ROMs) " ); return; }
	m.cpu.attachFastBus( &m.map );

	for ( u32 i = 0; i < 16; i++ ) m.mem.m_RAM[ 0x1000 + i ] = 0xEA;
	m.mem.m_RAM[ 0x1010 ] = 0x4C;
	m.mem.m_RAM[ 0x1011 ] = 0x00;
	m.mem.m_RAM[ 0x1012 ] = 0x10;
	m.cpu.m_PBR = 0x00;
	m.cpu.m_PC  = 0x1000;
	m.cpu.m_P  |= W65_I;

	// CMD selects 1MHz itself before some serial waits, so the automatic IEC
	// timer can legitimately be quiet while instruction-level pacing is still
	// required.
	m.mem.setPacing( 0, 1000000u );
	m.mem.m_MaxTickChunk = 0;
	m.cpu.run( 400 );
	CHECK( m.mem.m_MaxTickChunk <= 3 );

	m.mem.setPacing( 0, 20000000u );
	m.mem.m_MaxTickChunk = 0;
	m.cpu.run( 400 );
	CHECK( m.mem.m_MaxTickChunk > 8 );
}
