/*
   SCPU-EMU - SID write delivery conformance

   The audio faults in VIDEO_MODE 1 have never been bisected: nobody knows
   whether the wrong sound is a wrong write, a missing write, or a correct write
   that reached the chip at the wrong time. This supplies the upstream half of
   that split, on host, with no hardware.

   25-sidtrace records every SID write it intends to make into RAM as it goes.
   Running it here gives two independent comparisons:

     recorded buffer vs a VICE state dump   did our CPU and memory map execute
                                            the program the way VICE did?
     recorded buffer vs the host bus log    did every intended write actually
                                            reach the chip, once, in order?

   If both pass, everything upstream of the bus is exonerated and the remaining
   suspect is delivery -- timing, the eye, the mirror path -- which is a hardware
   question. If either fails, the failing one names the layer, which is the whole
   point of building it.

   The second comparison is the sharp one. CWriteBuffer's mirror sink drops
   writes that change nothing, which is right for DRAM and fatal for a SID: a
   re-write of a control register is how a gate is retriggered, and merging it
   silently removes the note. 25-sidtrace writes the same value three times in a
   row specifically to catch that.

     sid_trace <prg> [--vice-ram <ram.bin>] [--verbose]
*/
#include "../../Source/Bus/Host/host_bus.h"
#include "../../Source/C64/c64_memory.h"
#include "../../Source/SuperCPU/write_buffer.h"
#include "../../Source/SuperCPU/registers.h"
#include "../../Source/CPU/M6502/m6502.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static bool loadFile( const char *path, u8 *dst, u32 expected )
{
	FILE *f = fopen( path, "rb" );
	if ( !f ) { fprintf( stderr, "cannot open %s\n", path ); return false; }
	const size_t got = fread( dst, 1, expected, f );
	fclose( f );
	return got == expected;
}

struct Machine
{
	CHostBus           bus;
	CC64Memory         mem;
	CSuperCPURegisters regs;
	CWriteBuffer       wb;
	CM6502             cpu;
};

int main( int argc, char **argv )
{
	if ( argc < 2 )
	{
		fprintf( stderr, "usage: sid_trace <prg> [--vice-ram <ram.bin>] "
		                 "[--verbose]\n" );
		return 2;
	}
	const char *prgPath = argv[ 1 ];
	const char *viceRAMPath = 0;
	bool verbose = false;
	// The audio faults are VIDEO_MODE 1 specific, so a clean result in the
	// default configuration proves less than it looks. --viclog turns on
	// timestamp capture, which is what VIDEO_MODE 1 actually changes in this
	// path, so the check can be run in the configuration that misbehaves.
	bool vicLog = false;
	for ( int i = 2; i < argc; i++ )
	{
		if ( !strcmp( argv[ i ], "--verbose" ) ) verbose = true;
		else if ( !strcmp( argv[ i ], "--viclog" ) ) vicLog = true;
		else if ( !strcmp( argv[ i ], "--vice-ram" ) && i + 1 < argc )
			viceRAMPath = argv[ ++i ];
	}

	static Machine m;
	static u8 basic[ C64_BASIC_SIZE ], kernal[ C64_KERNAL_SIZE ];
	if ( !loadFile( "ROMs/kernal.rom", kernal, C64_KERNAL_SIZE )
	  || !loadFile( "ROMs/basic.rom", basic, C64_BASIC_SIZE ) )
		return 1;

	m.mem.setKernalROM( kernal );
	m.mem.setBasicROM( basic );

	static u8 bank1[ 0x10000 ];
	for ( u32 i = 0; i < C64_BASIC_SIZE; i++ )
		bank1[ 0xA000 + i ] = basic[ i ];
	for ( u32 i = 0; i < C64_KERNAL_SIZE; i++ )
	{
		bank1[ 0xE000 + i ] = kernal[ i ];
		bank1[ 0x6000 + i ] = kernal[ i ];
	}
	m.regs.attachBank1( bank1 );
	m.regs.setHardwareVersion( SCPU_V2 );
	m.mem.setIOInterceptor( &m.regs );
	m.mem.setROMShadow( bank1 );
	m.regs.trackKernalShadow( &m.mem.m_KernalShadowBase );
	m.regs.reset();

	m.mem.attachBus( &m.bus );
	m.mem.setMirrorSink( &m.wb );
	m.wb.attach( &m.bus, m.mem.m_RAM );
	m.cpu.attachBus( &m.mem );

	for ( u32 a = 0xDC00; a <= 0xDDFF; a++ ) m.bus.m_Memory[ a ] = 0xFF;

	m.mem.reset();
	m.wb.setOptMode( SCPU_OPT_NONE );
	m.cpu.reset();

	bool ready = false;
	for ( u32 i = 0; i < 400 && !ready; i++ )
	{
		m.cpu.run( 50000 );
		if ( m.cpu.m_Jammed ) break;
		for ( u32 p = 0; p + 5 < 1000 && !ready; p++ )
			ready = m.mem.m_RAM[ 0x0400 + p + 0 ] == 0x12
			     && m.mem.m_RAM[ 0x0400 + p + 1 ] == 0x05
			     && m.mem.m_RAM[ 0x0400 + p + 2 ] == 0x01
			     && m.mem.m_RAM[ 0x0400 + p + 3 ] == 0x04
			     && m.mem.m_RAM[ 0x0400 + p + 4 ] == 0x19;
	}
	if ( !ready ) { fprintf( stderr, "machine never reached READY\n" ); return 1; }

	FILE *pf = fopen( prgPath, "rb" );
	if ( !pf ) { fprintf( stderr, "cannot open %s\n", prgPath ); return 1; }
	u8 header[ 2 ];
	if ( fread( header, 1, 2, pf ) != 2 ) { fclose( pf ); return 1; }
	const u32 loadAddr = (u32)header[ 0 ] | ( (u32)header[ 1 ] << 8 );
	u32 length = 0;
	int byte;
	while ( ( byte = fgetc( pf ) ) != EOF && loadAddr + length < 0x10000 )
		m.mem.m_RAM[ loadAddr + length++ ] = (u8)byte;
	fclose( pf );

	const u32 end = loadAddr + length;
	m.mem.m_RAM[ 0x2D ] = (u8)( end & 0xFF );
	m.mem.m_RAM[ 0x2E ] = (u8)( end >> 8 );
	for ( u32 p = 0x2F; p <= 0x32; p += 2 )
	{
		m.mem.m_RAM[ p ] = m.mem.m_RAM[ 0x2D ];
		m.mem.m_RAM[ p + 1 ] = m.mem.m_RAM[ 0x2E ];
	}

	if ( vicLog ) m.mem.enableVICLog( true );

	// Only the program's own traffic is interesting. Everything the KERNAL did
	// getting here would bury it.
	m.bus.resetStats();
	m.bus.m_LogEnabled = true;
	m.bus.clearLog();

	m.mem.m_RAM[ 0x0277 ] = 0x52;	// R
	m.mem.m_RAM[ 0x0278 ] = 0x55;	// U
	m.mem.m_RAM[ 0x0279 ] = 0x4E;	// N
	m.mem.m_RAM[ 0x027A ] = 0x0D;
	m.mem.m_RAM[ 0x00C6 ] = 4;

	// Run until the program stamps its signature, then a little longer so any
	// buffered write has every chance to be flushed. A write still missing at
	// that point is missing, not merely late.
	bool done = false;
	for ( u32 i = 0; i < 4000 && !done; i++ )
	{
		m.cpu.run( 2000 );
		if ( m.cpu.m_Jammed ) { fprintf( stderr, "CPU jammed\n" ); return 1; }
		done = m.mem.m_RAM[ 0xC002 ] == 0x5A && m.mem.m_RAM[ 0xC003 ] == 0xA5;
	}
	if ( !done )
	{
		fprintf( stderr, "sid trace         program never stamped its "
		                 "signature at $C002/$C003\n" );
		return 1;
	}
	m.wb.flush();
	m.cpu.run( 20000 );

	const u32 count = (u32)m.mem.m_RAM[ 0xC000 ]
	                | ( (u32)m.mem.m_RAM[ 0xC001 ] << 8 );
	if ( count == 0 || count > 0x400 )
	{
		fprintf( stderr, "sid trace         implausible write count %u\n", count );
		return 1;
	}

	// ---- comparison 1: our execution against VICE's --------------------
	int rc = 0;
	if ( viceRAMPath )
	{
		static u8 viceRAM[ 0x10002 ];
		FILE *vf = fopen( viceRAMPath, "rb" );
		if ( !vf ) { fprintf( stderr, "cannot open %s\n", viceRAMPath ); return 1; }
		long total = fseek( vf, 0, SEEK_END ) == 0 ? ftell( vf ) : 0;
		fseek( vf, ( total == 0x10002 ) ? 2 : 0, SEEK_SET );
		size_t got = fread( viceRAM, 1, 0x10000, vf );
		fclose( vf );
		if ( got < 0x10000 ) { fprintf( stderr, "short VICE dump\n" ); return 1; }

		const u32 viceCount = (u32)viceRAM[ 0xC000 ]
		                    | ( (u32)viceRAM[ 0xC001 ] << 8 );
		u32 firstDiff = 0xFFFFFFFF;
		for ( u32 i = 0; i < count * 2 && firstDiff == 0xFFFFFFFF; i++ )
			if ( m.mem.m_RAM[ 0xC100 + i ] != viceRAM[ 0xC100 + i ] )
				firstDiff = i;

		const bool ok = viceCount == count && firstDiff == 0xFFFFFFFF;
		fprintf( stderr, "vs VICE           %u writes recorded, reference %u  %s\n",
		         count, viceCount, ok ? "ok" : "MISMATCH" );
		if ( !ok )
		{
			if ( firstDiff != 0xFFFFFFFF )
				fprintf( stderr, "                  first difference at write %u: "
				                 "ours $%02X, VICE $%02X\n",
				         firstDiff / 2, m.mem.m_RAM[ 0xC100 + firstDiff ],
				         viceRAM[ 0xC100 + firstDiff ] );
			rc = 1;
		}
	}

	// ---- comparison 2: intent against what reached the bus --------------
	// Walk the two sequences together. The bus log holds every access the
	// program made; the SID writes among them must be exactly the recorded
	// sequence, in order, with nothing merged, reordered, added or dropped.
	u32 wanted = 0, seen = 0, mismatches = 0;
	for ( u32 i = 0; i < m.bus.m_LogCount; i++ )
	{
		const HostBusLogEntry &e = m.bus.m_Log[ i ];
		if ( e.op != HOSTOP_WRITE && e.op != HOSTOP_BURST_WRITE ) continue;
		if ( e.addr < 0xD400 || e.addr > 0xD41F ) continue;
		seen++;
		if ( wanted >= count ) continue;

		const u8 wantReg = m.mem.m_RAM[ 0xC100 + wanted * 2 ];
		const u8 wantVal = m.mem.m_RAM[ 0xC100 + wanted * 2 + 1 ];
		const u8 gotReg = (u8)( e.addr - 0xD400 );
		if ( gotReg != wantReg || e.value != wantVal )
		{
			if ( mismatches < 8 )
				fprintf( stderr, "                  write %u: wanted $D4%02X=$%02X, "
				                 "bus had $D4%02X=$%02X\n",
				         wanted, wantReg, wantVal, gotReg, e.value );
			mismatches++;
		}
		wanted++;
		if ( verbose && wanted <= 12 )
			fprintf( stderr, "  [%2u] $D4%02X = $%02X\n", wanted - 1, gotReg,
			         e.value );
	}

	const bool delivered = ( seen == count ) && ( mismatches == 0 );
	fprintf( stderr, "bus delivery      %u intended, %u reached the chip, "
	                 "%u wrong  %s  (VIC log %s)\n",
	         count, seen, mismatches, delivered ? "ok" : "FAILED",
	         vicLog ? "on, as VIDEO_MODE 1" : "off" );
	if ( seen < count )
		fprintf( stderr, "                  %u write(s) never reached the bus -- "
		                 "a merged or swallowed SID write is a lost note\n",
		         count - seen );
	if ( !delivered ) rc = 1;
	return rc;
}
