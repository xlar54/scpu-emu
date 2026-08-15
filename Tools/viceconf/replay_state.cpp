/*
   SCPU-EMU - end-to-end band replay conformance against VICE

   render_state.cpp proves the renderer draws a single machine state the way
   VICE does. That leaves the entire path a raster effect actually travels
   untested: the CPU makes timestamped writes, CC64Memory logs them,
   CVICRasterTimeline turns cycle stamps into output rows, CVICRasterReplay
   turns those into bands, and only THEN does the renderer run -- once per band.
   Everything between the write and the renderer had been checked only against
   logs we wrote ourselves, which is the same internally-consistent trap that
   hid DISPLAY_Y and the sprite offset for weeks: every test agreed with every
   other test and all of them were a row out.

   So run the real thing. This boots the genuine KERNAL on CHostBus + CC64Memory
   + CM6502, autostarts the .prg, lets a raster program install its interrupt
   and generate a GENUINE write log with genuine cycle stamps, then drives the
   exact sequence CHDMIDisplay::renderFrame() drives on core 1. The output is a
   full 384x272 frame built from many bands, which is directly comparable to a
   VICE screenshot of the same program -- because a VICE screenshot IS the
   correctly rendered multi-state frame.

   Nothing here is a stand-in. The CPU, memory map, log, anchor estimation,
   timeline, band planner and renderer are the shipping objects.

     replay_state <prg> <chargen.rom> <out.raw> [--frames N] [--verbose]
*/
#include "../../Source/Bus/Host/host_bus.h"
#include "../../Source/C64/c64_memory.h"
#include "../../Source/SuperCPU/write_buffer.h"
#include "../../Source/SuperCPU/registers.h"
#include "../../Source/CPU/M6502/m6502.h"
#include "../../Source/Video/vic_raster.h"
#include "../../Source/Video/vic_renderer.h"

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

// The raster model needs emulated C64 time, which only the memory map knows.
static u64 c64CycleSource( void *context )
{
	return ( (CC64Memory *)context )->emuNow();
}

int main( int argc, char **argv )
{
	if ( argc < 4 )
	{
		fprintf( stderr, "usage: replay_state <prg> <chargen.rom> <out.raw> "
		                 "[--frames N] [--verbose]\n" );
		return 2;
	}
	const char *prgPath = argv[ 1 ];
	const char *charPath = argv[ 2 ];
	const char *outPath = argv[ 3 ];
	u32 settleFrames = 8;
	bool verbose = false;
	for ( int i = 4; i < argc; i++ )
	{
		if ( !strcmp( argv[ i ], "--verbose" ) ) verbose = true;
		else if ( !strcmp( argv[ i ], "--frames" ) && i + 1 < argc )
			settleFrames = (u32)atoi( argv[ ++i ] );
	}

	static Machine m;
	static u8 basic[ C64_BASIC_SIZE ], kernal[ C64_KERNAL_SIZE ];
	static u8 chargen[ C64_CHARROM_SIZE ];
	if ( !loadFile( "ROMs/kernal.rom", kernal, C64_KERNAL_SIZE )
	  || !loadFile( "ROMs/basic.rom", basic, C64_BASIC_SIZE )
	  || !loadFile( charPath, chargen, C64_CHARROM_SIZE ) )
		return 1;

	m.mem.setKernalROM( kernal );
	m.mem.setBasicROM( basic );
	m.mem.setCharROM( chargen );

	// Colour RAM is not a separate array. It lives at $D800 inside the
	// SuperCPU's bank-1 SRAM, and the thing that PUTS it there is the v2 I/O
	// interceptor -- CSuperCPURegisters::ioWrite() captures $D800-$DDFF into
	// bank 1 while the physical chip still gets its low nibble. Attach it, or
	// colourRAMShadow() reads an array nobody ever wrote and the whole display
	// window renders in one colour: a convincing renderer fault that is really
	// a missing wire. Seed the ROM windows exactly as CSuperCPU::reset() does.
	static u8 bank1[ 0x10000 ];
	for ( u32 i = 0; i < C64_BASIC_SIZE; i++ )
		bank1[ 0xA000 + i ] = basic[ i ];
	for ( u32 i = 0; i < C64_KERNAL_SIZE; i++ )
	{
		bank1[ 0xE000 + i ] = kernal[ i ];	// KT
		bank1[ 0x6000 + i ] = kernal[ i ];	// KS
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

	// Unconnected CIA port lines read high: no key down, no joystick pushed.
	for ( u32 a = 0xDC00; a <= 0xDDFF; a++ ) m.bus.m_Memory[ a ] = 0xFF;

	m.mem.reset();
	m.wb.setOptMode( SCPU_OPT_NONE );
	m.cpu.reset();

	// Boot to READY before injecting anything: a program loaded into a machine
	// that has not finished its RAM test is loaded into memory the KERNAL is
	// about to clear.
	bool ready = false;
	for ( u32 i = 0; i < 400 && !ready; i++ )
	{
		m.cpu.run( 50000 );
		if ( m.cpu.m_Jammed ) break;
		// "READY." lands at the start of a line; scan for the screen codes.
		for ( u32 p = 0; p + 5 < 1000 && !ready; p++ )
			ready = m.mem.m_RAM[ 0x0400 + p + 0 ] == 0x12	// R
			     && m.mem.m_RAM[ 0x0400 + p + 1 ] == 0x05	// E
			     && m.mem.m_RAM[ 0x0400 + p + 2 ] == 0x01	// A
			     && m.mem.m_RAM[ 0x0400 + p + 3 ] == 0x04	// D
			     && m.mem.m_RAM[ 0x0400 + p + 4 ] == 0x19;	// Y
	}
	if ( !ready )
	{
		fprintf( stderr, "machine never reached READY (jammed=%d)\n",
		         (int)m.cpu.m_Jammed );
		return 1;
	}

	// Load the .prg the way a loader would: honour its load address, then jump
	// through the BASIC stub rather than guessing an entry point.
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

	// BASIC's end-of-program pointers, so RUN finds the stub.
	const u32 end = loadAddr + length;
	m.mem.m_RAM[ 0x2D ] = (u8)( end & 0xFF );
	m.mem.m_RAM[ 0x2E ] = (u8)( end >> 8 );
	m.mem.m_RAM[ 0x2F ] = m.mem.m_RAM[ 0x2D ];
	m.mem.m_RAM[ 0x30 ] = m.mem.m_RAM[ 0x2E ];
	m.mem.m_RAM[ 0x31 ] = m.mem.m_RAM[ 0x2D ];
	m.mem.m_RAM[ 0x32 ] = m.mem.m_RAM[ 0x2E ];

	// Everything above this point is set-up. From here the machine is a real
	// one running a real program, and the log it produces is the thing under
	// test -- so enable capture and the raster clock now, not before.
	m.mem.enableVICLog( true );
	m.bus.setRasterClock( c64CycleSource, &m.mem );

	// RUN. $A7AE is BASIC's interpreter loop; entering at the RUN token's
	// handler is fragile, so type it instead: push "RUN\r" into the keyboard
	// buffer and let the KERNAL's own path take it.
	m.mem.m_RAM[ 0x0277 ] = 0x52;	// R
	m.mem.m_RAM[ 0x0278 ] = 0x55;	// U
	m.mem.m_RAM[ 0x0279 ] = 0x4E;	// N
	m.mem.m_RAM[ 0x027A ] = 0x0D;	// RETURN
	m.mem.m_RAM[ 0x00C6 ] = 4;		// characters in buffer

	const u32 cyclesPerLine = c64CyclesPerLine( m.bus.signals().video );
	const u32 rasterLines = c64RasterLines( m.bus.signals().video );
	const u64 cyclesPerFrame = (u64)cyclesPerLine * rasterLines;

	// Let the program install itself and settle. A raster program needs several
	// whole frames before its interrupt chain is in steady state.
	const u64 startCycle = m.mem.emuNow();
	while ( m.mem.emuNow() - startCycle < cyclesPerFrame * settleFrames )
	{
		m.cpu.run( 2000 );
		if ( m.cpu.m_Jammed )
		{
			fprintf( stderr, "CPU jammed at $%04X\n", m.cpu.m_PC );
			return 1;
		}
	}

	if ( verbose )
		fprintf( stderr, "log head=%u generation=%u rasterIRQs=%u anchor=%s\n",
		         m.mem.vicLogHead(), m.mem.vicLogGeneration(),
		         m.bus.rasterIRQsRaised(),
		         ( m.mem.rasterAnchor() >> 63 ) ? "valid" : "INVALID" );

	// ---- from here this is CHDMIDisplay::renderFrame(), verbatim in spirit --
	VICRasterTiming timing;
	timing.cyclesPerLine = cyclesPerLine;
	timing.rasterLines = rasterLines;
	timing.frameRows = VIC_RENDER_HEIGHT;
	timing.topRaster = (s32)c64DisplayFirstLine( m.bus.signals().video )
	                 - (s32)VIC_RENDER_DISPLAY_Y;
	timing.displayFirstRow = VIC_RENDER_DISPLAY_Y;
	timing.displayRows = VIC_RENDER_DISPLAY_H;

	// Where the timeline thinks each logged write lands. The band planner sits
	// downstream of this, so if the rows here are wrong nothing after it can be
	// right -- and if they are right, a wrong band is the planner's doing. That
	// split is the whole diagnostic value of dumping it.
	if ( verbose )
	{
		const u64 anchor = m.mem.rasterAnchor();
		const u32 head = m.mem.vicLogHead();
		const u32 first = head > 24 ? head - 24 : 0;
		fprintf( stderr, "  last %u log events (anchor line %u @ cycle %u):\n",
		         head - first, (u32)( anchor & 0x1FF ),
		         (u32)( ( anchor >> 16 ) & 0xFFFFFFFF ) );
		for ( u32 i = first; i < head; i++ )
		{
			const VICRegWrite &e = m.mem.vicLog()[ i & ( SCPU_VIC_LOG_SIZE - 1 ) ];
			const s32 line = CVICRasterTimeline::lineOfEvent( e, anchor, timing );
			fprintf( stderr, "    [%3u] cycle=%-10u reg=$%03X val=$%02X "
			                 "line=%-4d row=%-4d %s\n",
			         i, e.cycle, vicLogRegister( e ), e.value, line,
			         line - timing.topRaster,
			         vicLogHasRasterLine( e ) ? "exact" : "" );
		}
	}

	static CVICRasterReplay replay;
	static VICRasterReplayPlan plan;
	static u8 liveVIC[ 0x40 ];
	static u8 vicRAM[ 0x10000 ];
	static u8 colours[ 1024 ];

	// Run the pipeline to STEADY STATE rather than stopping at the first
	// multi-band frame. The first replay after priming necessarily starts from
	// a mid-frame tail, so its opening band inherits whatever state the machine
	// happened to be in -- an artefact of starting up, not of band planning. On
	// the card this settles within a frame or two because renderFrame() is
	// called continuously; here it has to be driven deliberately.
	VICRasterFrameResult result = VIC_RASTER_SNAPSHOT;
	u32 replays = 0;
	for ( u32 pass = 0; pass < 24; pass++ )
	{
		for ( u32 i = 0; i < 0x40; i++ ) liveVIC[ i ] = m.mem.vicRegister( (u8)i );
		const u8 liveBank = (u8)( m.mem.activeVICBankBase() >> 14 );

		result = replay.build( m.mem.vicLog(), m.mem.vicLogHead(),
		                       (u32)m.mem.emuNow(), m.mem.rasterAnchor(),
		                       m.mem.vicLogGeneration(), timing,
		                       liveVIC, liveBank, m.mem.m_RAM, plan );
		if ( result == VIC_RASTER_REPLAY && plan.bandCount > 1 ) replays++;
		if ( verbose )
			fprintf( stderr, "pass %2u: result=%d bands=%2u fallback=%d pages=%u\n",
			         pass, (int)result, plan.bandCount, (int)plan.fallback,
			         plan.pageCount );
		// Four good frames in a row is settled: the plan is being rebuilt from
		// a full frame of log every time rather than from a start-up remnant.
		if ( replays >= 4 ) break;
		// Advance a whole frame so the next pass has a closed frame to replay.
		const u64 mark = m.mem.emuNow();
		while ( m.mem.emuNow() - mark < cyclesPerFrame ) m.cpu.run( 2000 );
	}
	const bool replayed = replays > 0;

	memcpy( vicRAM, m.mem.m_RAM, sizeof vicRAM );
	const u8 *liveColours = m.mem.colourRAMShadow();
	if ( liveColours ) memcpy( colours, liveColours, sizeof colours );
	else               memset( colours, 14, sizeof colours );

	static u8 pixels[ VIC_RENDER_WIDTH * VIC_RENDER_HEIGHT ];
	memset( pixels, 0, sizeof pixels );
	CVICRenderer renderer;
	VICRenderCollisions collisions = {};

	u32 rowsDrawn = 0;
	for ( u32 b = 0; b < plan.bandCount; b++ )
	{
		const VICRasterReplayBand &band = plan.bands[ b ];
		VICRenderState state;
		memset( &state, 0, sizeof state );
		state.ram = vicRAM;
		state.charROM = chargen;
		state.colourRAM = colours;
		state.bankBase = (u32)( band.bank & 3 ) << 14;
		state.screenBase = state.bankBase
		                 + ( (u32)( band.vic[ 0x18 ] >> 4 ) << 10 );
		state.bitmapBase = state.bankBase
		                 + ( ( band.vic[ 0x18 ] & 0x08 ) ? 0x2000 : 0 );
		const u32 charset = (u32)( band.vic[ 0x18 ] & 0x0E ) << 10;
		const bool charROM = ( ( band.bank & 1 ) == 0 )
		                  && charset >= 0x1000 && charset < 0x2000;
		state.charsetBase = charROM ? 0xFFFFFFFF : state.bankBase + charset;
		state.yScrollVaries = plan.yScrollVaries;
		state.spritePointers = band.spritePtrValid ? band.spritePtr : 0;
		memcpy( state.vic, band.vic, sizeof state.vic );

		if ( band.firstRow + band.rowCount > VIC_RENDER_HEIGHT ) continue;
		renderer.renderRows( state, pixels + band.firstRow * VIC_RENDER_WIDTH,
		                     VIC_RENDER_WIDTH, band.firstRow, band.rowCount,
		                     &collisions, 0 );
		rowsDrawn += band.rowCount;
		if ( verbose && b < 12 )
			fprintf( stderr, "  band %2u rows %3u..%-3u  $D020=%X $D021=%X\n",
			         b, band.firstRow, band.firstRow + band.rowCount - 1,
			         band.vic[ 0x20 ] & 15, band.vic[ 0x21 ] & 15 );
	}

	FILE *out = fopen( outPath, "wb" );
	if ( !out ) { fprintf( stderr, "cannot write %s\n", outPath ); return 1; }
	fwrite( pixels, 1, sizeof pixels, out );
	fclose( out );

	fprintf( stderr, "replay            result=%s bands=%u rows=%u/%u "
	                 "fallback=%d resyncs=%u\n",
	         result == VIC_RASTER_REPLAY ? "REPLAY"
	         : result == VIC_RASTER_HOLD ? "HOLD" : "SNAPSHOT",
	         plan.bandCount, rowsDrawn, VIC_RENDER_HEIGHT,
	         (int)plan.fallback, replay.resyncs() );

	// A single-band plan means the whole point of the test did not happen: the
	// frame was painted from one state, which cannot distinguish a working band
	// planner from a broken one. Fail loudly rather than compare a frame that
	// proves nothing.
	if ( !replayed )
	{
		fprintf( stderr, "replay            FAILED: never produced a "
		                 "multi-band replay, so band placement is untested\n" );
		return 1;
	}
	if ( rowsDrawn != VIC_RENDER_HEIGHT )
	{
		fprintf( stderr, "replay            FAILED: bands cover %u of %u rows\n",
		         rowsDrawn, VIC_RENDER_HEIGHT );
		return 1;
	}
	return 0;
}
