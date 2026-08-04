/*
   SCPU-EMU - boot a real C64 KERNAL through the CPU core.

   This is the strongest test in the suite that does not need hardware: it runs
   the genuine Commodore KERNAL and BASIC ROMs through CM6502, CC64Memory and
   the banking model, and checks the machine reaches the BASIC READY prompt.
   Getting there exercises the RAM test, the vector setup, the screen editor
   init and the whole BASIC cold start -- thousands of instructions across every
   addressing mode, decimal arithmetic and both banking configurations.

   The ROM images are not in the repository (see ROMs/README.md). If they are
   absent these tests report as skipped rather than failing, so a fresh clone
   still goes green.
*/
#include "../test_framework.h"
#include "../../Source/C64/c64_memory.h"
#include "../../Source/CPU/M6502/m6502.h"
#include "../../Source/SuperCPU/write_buffer.h"
#include "../../Source/Bus/Host/host_bus.h"

#include <cstdio>

static bool loadROM( const char *path, u8 *dst, u32 expected )
{
	std::FILE *f = std::fopen( path, "rb" );
	if ( !f )
		return false;

	size_t got = std::fread( dst, 1, expected, f );
	std::fclose( f );
	return got == expected;
}

struct RealMachine
{
	CHostBus     bus;
	CC64Memory   mem;
	CWriteBuffer wb;
	CM6502       cpu;
	bool         haveROMs;

	RealMachine()
	{
		u8 basic[ C64_BASIC_SIZE ], kernal[ C64_KERNAL_SIZE ], chargen[ C64_CHARROM_SIZE ];

		haveROMs = loadROM( "ROMs/kernal.rom", kernal, C64_KERNAL_SIZE )
		        && loadROM( "ROMs/basic.rom",  basic,  C64_BASIC_SIZE );

		if ( !haveROMs )
			return;

		mem.setKernalROM( kernal );
		mem.setBasicROM( basic );
		if ( loadROM( "ROMs/chargen.rom", chargen, C64_CHARROM_SIZE ) )
			mem.setCharROM( chargen );

		mem.attachBus( &bus );
		mem.setMirrorSink( &wb );
		wb.attach( &bus, mem.m_RAM );
		cpu.attachBus( &mem );

		// Unconnected CIA port lines read high, so no key appears pressed and
		// no joystick appears pushed. Leaving these at 0 makes the KERNAL think
		// every key is down.
		for ( u32 a = 0xDC00; a <= 0xDCFF; a++ ) bus.m_Memory[ a ] = 0xFF;
		for ( u32 a = 0xDD00; a <= 0xDDFF; a++ ) bus.m_Memory[ a ] = 0xFF;

		mem.reset();
		wb.setOptMode( SCPU_OPT_NONE );
		cpu.reset();
	}

	// Screen codes -> ASCII, enough to read the text the KERNAL puts up.
	static char screenCodeToAscii( u8 c )
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
		for ( u32 i = 0; i < 1000; i++ )
			screen[ i ] = screenCodeToAscii( mem.m_RAM[ 0x0400 + i ] );
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
				line[ col ] = screenCodeToAscii( mem.m_RAM[ 0x0400 + row * 40 + col ] );
			line[ 40 ] = 0;

			bool blank = true;
			for ( u32 i = 0; i < 40; i++ ) if ( line[ i ] != ' ' ) blank = false;
			if ( !blank ) std::printf( "    |%s|\n", line );
		}
	}
};

TEST( real_kernal_reaches_basic_ready_prompt )
{
	RealMachine m;
	if ( !m.haveROMs )
	{
		std::printf( "(skipped: no ROMs) " );
		return;
	}

	CHECK_EQ( m.cpu.m_PC, 0xFCE2 );		// documented C64 reset entry point

	// A cold start is a few hundred thousand cycles, dominated by the RAM test.
	// Stop early once BASIC has printed its prompt.
	bool reachedReady = false;
	for ( u64 i = 0; i < 200; i++ )
	{
		m.cpu.run( 50000 );
		if ( m.screenContains( "READY" ) ) { reachedReady = true; break; }
		if ( m.cpu.m_Jammed ) break;
	}

	CHECK( !m.cpu.m_Jammed );
	CHECK( reachedReady );

	if ( !reachedReady )
		m.dumpScreen();
}

TEST( real_kernal_prints_the_boot_banner )
{
	RealMachine m;
	if ( !m.haveROMs ) { std::printf( "(skipped: no ROMs) " ); return; }

	for ( u64 i = 0; i < 200; i++ )
	{
		m.cpu.run( 50000 );
		if ( m.screenContains( "READY" ) ) break;
	}

	CHECK( m.screenContains( "COMMODORE 64 BASIC" ) );
	CHECK( m.screenContains( "BYTES FREE" ) );
}

TEST( real_kernal_ram_test_finds_38911_bytes_free )
{
	RealMachine m;
	if ( !m.haveROMs ) { std::printf( "(skipped: no ROMs) " ); return; }

	for ( u64 i = 0; i < 200; i++ )
	{
		m.cpu.run( 50000 );
		if ( m.screenContains( "READY" ) ) break;
	}

	// The KERNAL's RAM test walks upward until a write fails to read back. Our
	// banking model must make that stop at $A000, where BASIC ROM appears --
	// which is what yields the canonical 38911 bytes free.
	CHECK( m.screenContains( "38911" ) );

	// Top of BASIC memory, as recorded by RAMTAS.
	u16 memTop = (u16)( m.mem.m_RAM[ 0x0037 ] | ( m.mem.m_RAM[ 0x0038 ] << 8 ) );
	CHECK_EQ( memTop, 0xA000 );
}

TEST( real_kernal_boot_bus_traffic_is_only_io_and_mirroring )
{
	RealMachine m;
	if ( !m.haveROMs ) { std::printf( "(skipped: no ROMs) " ); return; }

	for ( u64 i = 0; i < 200; i++ )
	{
		m.cpu.run( 50000 );
		if ( m.screenContains( "READY" ) ) break;
	}
	m.wb.flush();

	// A whole cold start, including a 38K RAM test, must cost only a few
	// thousand bus cycles: everything else is served from shadow RAM. If this
	// number climbs into the hundreds of thousands, reads are leaking onto the
	// bus and the acceleration is gone.
	std::printf( "\n    bus cycles: %llu (io r=%llu w=%llu, mirrored=%llu of %llu writes)\n    ",
	             (unsigned long long)m.bus.m_Cycles,
	             (unsigned long long)m.mem.m_IOReads,
	             (unsigned long long)m.mem.m_IOWrites,
	             (unsigned long long)m.wb.m_BytesFlushed,
	             (unsigned long long)m.mem.m_RamWrites );

	CHECK( m.bus.m_Cycles < 100000 );
	CHECK( m.wb.m_BytesFlushed < m.mem.m_RamWrites );	// coalescing did something
}
