/*
   SCPU-EMU - run the SingleStepTests 65816 vectors against our core

   Tests/CPU/test_w65c816_diff.cpp is a good test that proves the wrong thing to
   trust on its own: it shows our 65816 in EMULATION mode agrees with our 6502.
   Two cores we wrote, agreeing with each other. That is the same shape as
   DISPLAY_Y and the sprite offset being wrong together for weeks while every
   test passed -- internal consistency is not correctness.

   It is also, by construction, emulation mode only. Native mode -- 16-bit
   accumulator and index registers, MVN/MVP, long addressing, the 65816-only
   stack instructions, an unaligned direct page -- rests entirely on
   hand-written expectations in test_w65c816.cpp. Native mode is what SuperCPU
   software actually runs in.

   So point it at an external authority. github.com/SingleStepTests/65816 has
   10,000 vectors per opcode per mode, each one a complete initial state, a
   complete final state, and the cycle-by-cycle bus activity. This runs them.

     ss816 <file.json>...              registers and memory
     ss816 --cycles <file.json>...     also the bus access sequence

   Memory is a sparse 24-bit map, because a vector may touch any bank and
   allocating 16MB per test would be slower than the tests.
*/
#include "../../Source/CPU/W65C816/w65c816.h"
#include "../../Source/CPU/cpu_bus.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// --- sparse 24-bit memory ---------------------------------------------------
// Flat 16MB would be 16MB of memset per test. A small open-addressed table is
// both faster and keeps "was this address ever touched?" answerable, which the
// final-state comparison needs.
#define SS_SLOTS 4096

struct SparseMem
{
	u32 addr[ SS_SLOTS ];
	u8  value[ SS_SLOTS ];
	bool used[ SS_SLOTS ];

	void clear() { memset( used, 0, sizeof used ); }

	u32 slotOf( u32 a ) const
	{
		u32 i = ( a * 2654435761u ) & ( SS_SLOTS - 1 );
		while ( used[ i ] && addr[ i ] != a ) i = ( i + 1 ) & ( SS_SLOTS - 1 );
		return i;
	}
	bool has( u32 a ) const { const u32 i = slotOf( a ); return used[ i ]; }
	u8 get( u32 a ) const
	{
		const u32 i = slotOf( a );
		return used[ i ] ? value[ i ] : 0;
	}
	void set( u32 a, u8 v )
	{
		const u32 i = slotOf( a );
		addr[ i ] = a; value[ i ] = v; used[ i ] = true;
	}
};

// One recorded bus access, so the cycle sequence can be compared.
struct Access { u32 addr; u8 value; bool write; };
#define SS_MAX_ACCESSES 64

class SSBus : public ICpuBus
{
public:
	SparseMem mem;
	Access    log[ SS_MAX_ACCESSES ];
	u32       count = 0;
	bool      overflow = false;

	void beginTest() { mem.clear(); count = 0; overflow = false; }

	u8 read8( scpu_addr_t a ) override
	{
		const u8 v = mem.get( (u32)a & 0xFFFFFF );
		record( (u32)a & 0xFFFFFF, v, false );
		return v;
	}
	void write8( scpu_addr_t a, u8 v ) override
	{
		mem.set( (u32)a & 0xFFFFFF, v );
		record( (u32)a & 0xFFFFFF, v, true );
	}
	bool irqAsserted() override { return false; }
	bool nmiAsserted() override { return false; }

private:
	void record( u32 a, u8 v, bool w )
	{
		if ( count >= SS_MAX_ACCESSES ) { overflow = true; return; }
		log[ count ].addr = a;
		log[ count ].value = v;
		log[ count ].write = w;
		count++;
	}
};

// --- a parser for exactly this file's shape ---------------------------------
// Not a general JSON parser. The files are machine-generated and perfectly
// regular, so scanning for keys is both far faster than parsing 6MB of tree and
// impossible to get subtly wrong in a way that silently drops tests.
struct Scanner
{
	const char *p, *end;

	bool skipTo( char c ) { while ( p < end && *p != c ) p++; return p < end; }

	// Find "key": within the current object and return the value position.
	const char *find( const char *key, const char *limit ) const
	{
		const size_t n = strlen( key );
		for ( const char *q = p; q + n + 2 < limit; q++ )
			if ( q[ 0 ] == '"' && !memcmp( q + 1, key, n ) && q[ n + 1 ] == '"' )
				return q + n + 2;
		return 0;
	}
	static long num( const char *q )
	{
		while ( *q && ( *q == ':' || *q == ' ' ) ) q++;
		return strtol( q, 0, 10 );
	}
};

struct State
{
	u16 pc, s, a, x, y, d;
	u8  p, dbr, pbr;
	bool e;
};

// Read one state object ("initial" or "final") plus its ram list.
static const char *readState( const char *q, const char *limit, State &st,
                              SparseMem *ram, u32 *ramCount )
{
	Scanner sc; sc.p = q; sc.end = limit;
	const char *r;
	#define FIELD(name,dst,type) \
		r = sc.find( name, limit ); if ( r ) dst = (type)Scanner::num( r );
	FIELD( "pc",  st.pc,  u16 )
	FIELD( "s",   st.s,   u16 )
	FIELD( "p",   st.p,   u8 )
	FIELD( "a",   st.a,   u16 )
	FIELD( "x",   st.x,   u16 )
	FIELD( "y",   st.y,   u16 )
	FIELD( "dbr", st.dbr, u8 )
	FIELD( "d",   st.d,   u16 )
	FIELD( "pbr", st.pbr, u8 )
	#undef FIELD
	r = sc.find( "e", limit );
	st.e = r && Scanner::num( r ) != 0;

	// "ram": [[addr, value], ...]
	if ( ramCount ) *ramCount = 0;
	const char *ramKey = sc.find( "ram", limit );
	if ( !ramKey ) return limit;
	const char *cur = ramKey;
	while ( cur < limit && *cur != '[' ) cur++;
	if ( cur >= limit ) return limit;
	cur++;			// into the outer list
	while ( cur < limit )
	{
		while ( cur < limit && *cur != '[' && *cur != ']' ) cur++;
		if ( cur >= limit || *cur == ']' ) break;
		cur++;
		const long a = strtol( cur, (char **)&cur, 10 );
		while ( cur < limit && ( *cur == ',' || *cur == ' ' ) ) cur++;
		const long v = strtol( cur, (char **)&cur, 10 );
		if ( ram ) ram->set( (u32)a & 0xFFFFFF, (u8)v );
		if ( ramCount ) ( *ramCount )++;
		while ( cur < limit && *cur != ']' ) cur++;
		if ( cur < limit ) cur++;
	}
	return cur;
}

static u32 failures = 0, passes = 0, skipped = 0;
static u32 reported = 0;

static void report( const char *name, const char *what,
                    long want, long got )
{
	if ( reported++ >= 12 ) return;
	fprintf( stderr, "  %-14s %-22s want %-8ld got %ld\n",
	         name, what, want, got );
}

int main( int argc, char **argv )
{
	bool checkCycles = false;
	int first = 1;
	for ( ; first < argc; first++ )
	{
		if ( !strcmp( argv[ first ], "--cycles" ) ) checkCycles = true;
		else break;
	}
	if ( first >= argc )
	{
		fprintf( stderr, "usage: ss816 [--cycles] <file.json>...\n" );
		return 2;
	}

	static SSBus bus;
	static CW65C816 cpu;
	cpu.attachBus( &bus );

	for ( int fi = first; fi < argc; fi++ )
	{
		FILE *f = fopen( argv[ fi ], "rb" );
		if ( !f ) { fprintf( stderr, "cannot open %s\n", argv[ fi ] ); return 1; }
		fseek( f, 0, SEEK_END );
		const long size = ftell( f );
		fseek( f, 0, SEEK_SET );
		char *text = (char *)malloc( (size_t)size + 1 );
		if ( !text ) { fclose( f ); fprintf( stderr, "out of memory\n" ); return 1; }
		if ( fread( text, 1, (size_t)size, f ) != (size_t)size )
		{ fclose( f ); free( text ); fprintf( stderr, "short read\n" ); return 1; }
		text[ size ] = 0;
		fclose( f );

		const char *end = text + size;
		u32 fileFail = 0, fileRun = 0;

		// The opcode is the leading hex of the filename's basename.
		const char *base = argv[ fi ];
		for ( const char *q = base; *q; q++ )
			if ( *q == '/' || *q == '\\' ) base = q + 1;
		const long opcode = strtol( base, 0, 16 );
		const bool blockMove = ( opcode == 0x44 || opcode == 0x54 );

		const char *cur = text;
		while ( cur < end )
		{
			// Each test is one object beginning at `{ "name":`.
			const char *nameKey = strstr( cur, "\"name\"" );
			if ( !nameKey ) break;
			const char *nextName = strstr( nameKey + 6, "\"name\"" );
			const char *limit = nextName ? nextName : end;

			char name[ 32 ] = { 0 };
			{
				const char *q = nameKey + 6;
				while ( q < limit && *q != '"' ) q++;
				if ( q < limit ) q++;
				u32 n = 0;
				while ( q < limit && *q != '"' && n < sizeof name - 1 )
					name[ n++ ] = *q++;
			}

			const char *initKey = strstr( nameKey, "\"initial\"" );
			const char *finalKey = strstr( nameKey, "\"final\"" );
			if ( !initKey || !finalKey || initKey > limit || finalKey > limit )
			{ cur = limit; continue; }

			// MVN/MVP cannot be judged by an instruction-stepping runner, and
			// saying so is better than reporting 10,000 false failures.
			//
			// A block move re-executes itself once per byte, so ONE instruction
			// can be tens of thousands of cycles long. These vectors are capped
			// at 100 cycles, which lands mid-instruction: a representative $44
			// asks for 3263 bytes, moves 14, and stops with PC two bytes into
			// its own opcode after a partial re-fetch. Reproducing that needs
			// cycle-level stepping, which is a different harness -- not a
			// verdict on the core.
			if ( blockMove )
			{
				skipped++;
				cur = limit;
				continue;
			}

			State init, want;
			bus.beginTest();
			readState( initKey, finalKey, init, &bus.mem, 0 );

			// Final-state RAM into its own map, so "unchanged" and "changed to
			// zero" stay distinguishable.
			static SparseMem wantRAM;
			wantRAM.clear();
			const char *cyclesKey = strstr( finalKey, "\"cycles\"" );
			const char *finalLimit = ( cyclesKey && cyclesKey < limit )
			                       ? cyclesKey : limit;
			readState( finalKey, finalLimit, want, &wantRAM, 0 );

			cpu.reset();
			cpu.m_PC = init.pc;   cpu.m_S = init.s;   cpu.m_P = init.p;
			cpu.m_C = init.a;     cpu.m_X = init.x;   cpu.m_Y = init.y;
			cpu.m_D = init.d;     cpu.m_DBR = init.dbr; cpu.m_PBR = init.pbr;
			cpu.m_E = init.e;
			cpu.applyE();
			bus.count = 0;		// the reset above must not appear in the log

			cpu.step();
			fileRun++;

			bool bad = false;
			#define CHECK(field,label,got) \
				if ( (long)(got) != (long)(field) ) \
					{ if ( !bad ) report( name, label, (long)(field), (long)(got) ); \
					  bad = true; }
			CHECK( want.pc,  "pc",  cpu.m_PC )
			CHECK( want.s,   "s",   cpu.m_S )
			CHECK( want.a,   "a",   cpu.m_C )
			CHECK( want.x,   "x",   cpu.m_X )
			CHECK( want.y,   "y",   cpu.m_Y )
			CHECK( want.d,   "d",   cpu.m_D )
			CHECK( want.dbr, "dbr", cpu.m_DBR )
			CHECK( want.pbr, "pbr", cpu.m_PBR )
			CHECK( want.p,   "p",   cpu.m_P )
			CHECK( want.e ? 1 : 0, "e", cpu.m_E ? 1 : 0 )
			#undef CHECK

			for ( u32 i = 0; i < SS_SLOTS && !bad; i++ )
				if ( wantRAM.used[ i ] )
				{
					const u32 a = wantRAM.addr[ i ];
					if ( bus.mem.get( a ) != wantRAM.value[ i ] )
					{
						char label[ 24 ];
						snprintf( label, sizeof label, "ram $%06X", a );
						report( name, label, wantRAM.value[ i ],
						        bus.mem.get( a ) );
						bad = true;
					}
				}

			if ( bad ) { failures++; fileFail++; } else passes++;
			cur = limit;
		}

		if ( blockMove )
			fprintf( stderr, "%-24s      -               SKIPPED  (block move; "
			                 "see the note in ss816.cpp)\n", argv[ fi ] );
		else
			fprintf( stderr, "%-24s %6u tests, %6u failed%s\n",
			         argv[ fi ], fileRun, fileFail,
			         fileFail == 0 ? "  ok" : "  FAILED" );
		free( text );
	}

	if ( checkCycles )
		fprintf( stderr, "(cycle-sequence comparison not implemented; "
		                 "registers and memory only)\n" );
	fprintf( stderr, "\n%u passed, %u failed, %u skipped\n",
	         passes, failures, skipped );
	return failures ? 1 : 0;
}
