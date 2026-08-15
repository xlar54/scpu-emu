/*
   SCPU-EMU - what does CMD's own ROM ask of the hardware, and do we answer?

   The question "is there anything we are missing in our SuperCPU emulation"
   has been answered so far by reading VICE's implementation -- registers.cpp
   cites it fourteen times. That is a good oracle for semantics, but it says
   nothing about COVERAGE: which registers CMD's firmware actually exercises,
   and whether every one of those accesses is answered by something we modelled
   rather than falling through to a default.

   This uses the ROM as a 128KB test suite without reading a line of it. The
   accelerator's own I/O interceptor already reports, per access, whether it
   handled the access -- ioRead/ioWrite return bool for exactly that. So wrap
   it, run CMD's firmware, and record the verdict for every access.

   The output is a coverage table:

     addr    reads  writes  claimed  values seen
     $D0B3      41       2      yes  C7 47

   An UNCLAIMED access is the interesting one. It means CMD's own code touched
   something we do not model, and whatever it read came from the bus or from a
   default -- which is precisely "something we are missing", named and counted
   rather than guessed at.

   Deliberately NOT a diff against VICE. VICE's monitor tracepoints print to a
   console that a headless run cannot capture, so no reference sequence exists
   to diff against; that was checked before building this. Coverage is what can
   be established honestly here, and it is the half that answers the question.

     scpu_trace [--rom <path>] [--frames N] [--all]
*/
#include "../../Source/Bus/Host/host_bus.h"
#include "../../Source/C64/c64_memory.h"
#include "../../Source/SuperCPU/supercpu.h"
#include "../../Source/SuperCPU/registers.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static bool loadFile( const char *path, u8 *dst, u32 expected )
{
	FILE *f = fopen( path, "rb" );
	if ( !f ) return false;
	const size_t got = fread( dst, 1, expected, f );
	fclose( f );
	return got == expected;
}

// One row of the coverage table. Values are kept as a small set rather than a
// count so a register that only ever reads back one thing is distinguishable
// from one the firmware is actually driving.
struct Coverage
{
	u32 reads, writes;
	u32 claimedReads, claimedWrites;
	u8  values[ 8 ];
	u32 valueCount;

	void noteValue( u8 v )
	{
		for ( u32 i = 0; i < valueCount; i++ ) if ( values[ i ] == v ) return;
		if ( valueCount < 8 ) values[ valueCount++ ] = v;
	}
};

// Records the verdict the real interceptor gives, and forwards everything
// unchanged. It must not alter behaviour: the point is to observe CMD's
// firmware running against the shipping register layer, not against a variant.
class CRecordingInterceptor : public IIOInterceptor
{
public:
	IIOInterceptor *inner = 0;
	Coverage cov[ 0x100 ];			// $D000-$D0FF, the accelerator's window
	Coverage wide[ 4 ];				// $D200-$D3FF and $DF00 buckets
	u64 totalReads = 0, totalWrites = 0;

	void reset()
	{
		memset( cov, 0, sizeof cov );
		memset( wide, 0, sizeof wide );
		totalReads = totalWrites = 0;
	}

	bool ioRead( u16 addr, u8 &value ) override
	{
		const bool claimed = inner ? inner->ioRead( addr, value ) : false;
		Coverage *c = slot( addr );
		if ( c )
		{
			c->reads++;
			if ( claimed ) { c->claimedReads++; c->noteValue( value ); }
			totalReads++;
		}
		return claimed;
	}

	bool ioWrite( u16 addr, u8 value ) override
	{
		const bool claimed = inner ? inner->ioWrite( addr, value ) : false;
		Coverage *c = slot( addr );
		if ( c )
		{
			c->writes++;
			c->noteValue( value );
			if ( claimed ) c->claimedWrites++;
			totalWrites++;
		}
		return claimed;
	}

	// Everything below is pure forwarding. Changing any of it would change the
	// machine being observed.
	bool ioAccessNeedsStretch( u16 a, bool w ) const override
		{ return inner ? inner->ioAccessNeedsStretch( a, w ) : w; }
	bool ioAccessUsesWriteBuffer( u16 a, bool w ) const override
		{ return inner ? inner->ioAccessUsesWriteBuffer( a, w ) : false; }
	bool dosExtensionEnabled() const override
		{ return inner ? inner->dosExtensionEnabled() : false; }

	// Is this address one the ACCELERATOR is supposed to answer?
	//
	// This distinction is the whole report. $D000-$D03F are the VIC's own
	// registers and $D400-$D41F the SID's: declining those is correct, because
	// they belong to the C64 and must reach it. Flagging them as "not modelled"
	// buries the real signal under sixty-four false positives -- which is
	// exactly what the first run of this tool did.
	//
	// The accelerator's own windows are $D071-$D07F (control strobes),
	// $D0B0-$D0BF (status/optimisation/DOS-extension) and $D200-$D2FF (DOS
	// scratch RAM). $D300-$D3FF and $DF00-$DFFF are watched too: the first is
	// the sysram mirror region, the second carries RAMLink's strobes.
	static bool isAcceleratorWindow( u16 addr )
	{
		return ( addr >= 0xD071 && addr <= 0xD07F )
		    || ( addr >= SCPU_REG_STATUS_FIRST && addr <= SCPU_REG_STATUS_LAST )
		    || ( addr >= SCPU_SYSRAM_BASE
		         && addr < SCPU_SYSRAM_BASE + SCPU_SYSRAM_SIZE );
	}

private:
	Coverage *slot( u16 addr )
	{
		if ( addr >= 0xD000 && addr <= 0xD0FF ) return &cov[ addr - 0xD000 ];
		if ( addr >= 0xD200 && addr <= 0xD2FF ) return &wide[ 0 ];
		if ( addr >= 0xD300 && addr <= 0xD3FF ) return &wide[ 1 ];
		if ( addr >= 0xDF00 && addr <= 0xDFFF ) return &wide[ 2 ];
		return 0;
	}
};

static void printRow( const char *label, const Coverage &c )
{
	if ( !c.reads && !c.writes ) return;
	const bool anyClaimed = c.claimedReads || c.claimedWrites;
	const bool allClaimed = c.claimedReads == c.reads
	                     && c.claimedWrites == c.writes;
	printf( "  %-10s %6u %7u   %-9s ", label, c.reads, c.writes,
	        allClaimed ? "yes" : ( anyClaimed ? "PARTIAL" : "NO" ) );
	for ( u32 i = 0; i < c.valueCount; i++ ) printf( "%02X ", c.values[ i ] );
	if ( !allClaimed )
		printf( "  <-- %u read(s), %u write(s) NOT modelled",
		        c.reads - c.claimedReads, c.writes - c.claimedWrites );
	printf( "\n" );
}

int main( int argc, char **argv )
{
	const char *romPath = 0;
	u32 frames = 400;
	bool showAll = false;
	for ( int i = 1; i < argc; i++ )
	{
		if ( !strcmp( argv[ i ], "--all" ) ) showAll = true;
		else if ( !strcmp( argv[ i ], "--rom" ) && i + 1 < argc ) romPath = argv[ ++i ];
		else if ( !strcmp( argv[ i ], "--frames" ) && i + 1 < argc )
			frames = (u32)atoi( argv[ ++i ] );
	}

	static u8 kernal[ 8192 ], basic[ 8192 ], chargen[ 4096 ], rom[ 131072 ];
	if ( !loadFile( "ROMs/kernal.rom", kernal, sizeof kernal )
	  || !loadFile( "ROMs/basic.rom", basic, sizeof basic ) )
	{
		printf( "need ROMs/kernal.rom and ROMs/basic.rom\n" );
		return 1;
	}
	loadFile( "ROMs/chargen.rom", chargen, sizeof chargen );

	// 2.04 is the preferred image; 1.4 is the documented fallback.
	const char *tried = romPath ? romPath : "ROMs/scpu-dos-2.04.bin";
	if ( !loadFile( tried, rom, sizeof rom ) )
	{
		tried = "ROMs/scpu-dos-1.4.bin";
		if ( !loadFile( tried, rom, sizeof rom ) )
		{
			printf( "no SuperCPU ROM found -- supply ROMs/scpu-dos-2.04.bin\n" );
			return 1;
		}
	}
	printf( "SuperCPU ROM     : %s\n", tried );

	static CHostBus bus;
	static CSuperCPU scpu;
	static CRecordingInterceptor rec;

	scpu.setKernalROM( kernal );
	scpu.setBasicROM( basic );
	scpu.setCharROM( chargen );
	for ( u32 a = 0xDC00; a <= 0xDDFF; a++ ) bus.m_Memory[ a ] = 0xFF;
	bus.m_Memory[ 0xDD00 ] = 0x3F;
	bus.m_Memory[ 0xDD02 ] = 0x3F;

	scpu.memoryMap().setROM( rom, sizeof rom );
	scpu.setBootmapEnabled( true );
	if ( !scpu.init( &bus, SCPU_CORE_65816, SCPU_SIMM_16MB ) )
	{
		printf( "init failed\n" );
		return 1;
	}

	// Interpose AFTER init, so the machine is fully built and the object being
	// wrapped is the one it actually installed.
	rec.inner = &scpu.registers();
	scpu.memory().setIOInterceptor( &rec );
	rec.reset();

	// The splash waits on raster interrupts, so give the bus a real raster
	// clock -- the one added for the band replay harness serves this too.
	struct Src { static u64 now( void *ctx ) { return ( (CC64Memory *)ctx )->emuNow(); } };
	bus.setRasterClock( Src::now, &scpu.memory() );

	for ( u32 f = 0; f < frames; f++ ) scpu.runFrame();

	printf( "frames run       : %u\n", frames );
	printf( "PC after boot    : $%06X\n", (unsigned)scpu.cpu()->pc() );
	printf( "intercepted      : %llu reads, %llu writes\n",
	        (unsigned long long)rec.totalReads,
	        (unsigned long long)rec.totalWrites );
	printf( "raster IRQs      : %u\n", bus.rasterIRQsRaised() );
	printf( "\n  register     reads  writes   modelled  values seen\n" );
	printf( "  ---------------------------------------------------\n" );

	u32 shown = 0, unmodelled = 0, passthrough = 0;
	for ( u32 i = 0; i < 0x100; i++ )
	{
		const Coverage &c = rec.cov[ i ];
		if ( !c.reads && !c.writes ) continue;
		const u16 addr = (u16)( 0xD000 + i );
		shown++;
		if ( !CRecordingInterceptor::isAcceleratorWindow( addr ) )
		{
			// The C64's own chips. Declining is correct; count and move on.
			passthrough++;
			if ( showAll )
			{
				char label[ 12 ];
				snprintf( label, sizeof label, "$%04X", addr );
				printRow( label, c );
			}
			continue;
		}
		const bool allClaimed = c.claimedReads == c.reads
		                     && c.claimedWrites == c.writes;
		if ( !allClaimed ) unmodelled++;
		char label[ 12 ];
		snprintf( label, sizeof label, "$%04X", addr );
		printRow( label, c );
	}
	static const char *wideNames[ 3 ] = { "$D2xx", "$D3xx", "$DFxx" };
	// $D2xx/$D3xx are the accelerator's own scratch RAM and its mirror. $DFxx is
	// NOT: it is cartridge I/O that the SuperCPU merely OBSERVES -- $DF7E/$DF7F
	// set and clear the RAMLink flag and then deliberately return false so the
	// write still reaches the cartridge bus. Judging it as unclaimed reports a
	// design decision as a defect.
	static const bool wideIsAccelerator[ 3 ] = { true, true, false };
	for ( u32 i = 0; i < 3; i++ )
	{
		const Coverage &c = rec.wide[ i ];
		if ( !c.reads && !c.writes ) continue;
		shown++;
		if ( !wideIsAccelerator[ i ] )
		{
			passthrough++;
			if ( showAll ) printRow( wideNames[ i ], c );
			continue;
		}
		const bool allClaimed = c.claimedReads == c.reads
		                     && c.claimedWrites == c.writes;
		if ( !allClaimed ) unmodelled++;
		printRow( wideNames[ i ], c );
	}

	printf( "\n  %u locations touched by CMD's firmware: %u in the "
	        "accelerator's own\n  windows (listed above), %u belonging to the "
	        "C64's chips and correctly passed\n  through.\n",
	        shown, shown - passthrough, passthrough );
	printf( "\n  unmodelled accesses inside the accelerator's windows: %u\n",
	        unmodelled );
	if ( !unmodelled )
		printf( "  every access CMD's own ROM made was answered by something "
		        "we model\n" );
	else
		printf( "  the rows above are what CMD's firmware asks for and we do "
		        "not answer\n" );
	if ( !showAll && !unmodelled )
		printf( "  (--all lists the fully-modelled ones too)\n" );
	return unmodelled ? 1 : 0;
}
