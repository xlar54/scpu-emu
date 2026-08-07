/*
   SCPU-EMU - differential test: CW65C816 in emulation mode vs CM6502.

   In emulation mode the 65816 IS a 6502, so the two cores must agree
   instruction for instruction -- registers, flags, cycle counts and every byte
   of bus traffic. That is a far stronger check than any hand-written
   expectation, because it exercises paths nobody thought to write a test for.

   The agreement is not total, and the differences are real hardware behaviour
   rather than tolerance. Docs/SuperCPU64/65816-reference.md section 10 lists them;
   this file encodes that list as explicit exemptions. Anything not exempted
   that differs is a bug in one of the cores.

     1  the 105 undocumented encodings   avoided by construction -- the generator
                                         emits documented opcodes only, and
                                         test_w65c816.cpp covers what the 65816
                                         does with the rest
     2  decimal ADC/SBC N and Z          the 65816 uses the 65C02 rule; A, C and
                                         V must still match
     3  interrupts clear D               separate targeted test below
     4  abs,X carrying past $FFFF         avoided by construction: base < $FF00
     5  JMP ($xxFF)                      separate targeted test below
     6  P bit 4                          normalised -- CM6502 holds it clear in
                                         the live P, the 65816 holds it set
                                         because x is forced to 1. Both PUSH the
                                         same byte, so the mask applies to the
                                         live register only
     8  the RMW dummy write              NOT exempt. Both cores emit it, and the
                                         write log is compared byte for byte,
                                         which is the point of restoring
                                         m_RMWDummyWrite

   Copyright (c) 2026 SCPU-EMU contributors, GPLv3 -- see w65c816.h.
*/
#include "../test_framework.h"
#include "../../Source/CPU/M6502/m6502.h"
#include "../../Source/CPU/M6502/m6502_opcodes.h"
#include "../../Source/CPU/W65C816/w65c816.h"
#include "../../Source/CPU/W65C816/w65c816_opcodes.h"

#define DIFF_LOG_SIZE 4096

// A flat 64K with a write log, so the comparison covers bus traffic and not
// just the final memory image. An RMW that writes the right value by the wrong
// route -- one write instead of two -- has to fail.
class CDiffBus : public ICpuBus
{
public:
	u8   m_Mem[ 0x10000 ];
	bool m_IRQ, m_NMI;
	u64  m_Ticks;

	struct Entry { u16 addr; u8 value; };
	Entry m_Writes[ DIFF_LOG_SIZE ];
	u32   m_WriteCount;

	CDiffBus() : m_IRQ( false ), m_NMI( false ), m_Ticks( 0 ), m_WriteCount( 0 )
	{
		std::memset( m_Mem, 0, sizeof( m_Mem ) );
	}

	u8 read8( scpu_addr_t a ) override { return m_Mem[ a & 0xFFFF ]; }

	void write8( scpu_addr_t a, u8 v ) override
	{
		if ( m_WriteCount < DIFF_LOG_SIZE )
		{
			m_Writes[ m_WriteCount ].addr  = (u16)( a & 0xFFFF );
			m_Writes[ m_WriteCount ].value = v;
			m_WriteCount++;
		}
		m_Mem[ a & 0xFFFF ] = v;
	}

	bool irqAsserted() override { return m_IRQ; }
	bool nmiAsserted() override { return m_NMI; }
	void tick( u32 n ) override { m_Ticks += n; }
};

// A tiny deterministic PRNG. std::rand() varies between libraries, and a test
// that generates different programs on different machines is not a test.
struct CRandom
{
	u32 s;
	explicit CRandom( u32 seed ) : s( seed ? seed : 1 ) {}
	u32 next()
	{
		s ^= s << 13; s ^= s >> 17; s ^= s << 5;
		return s;
	}
	u32 below( u32 n ) { return next() % n; }
};

// ---------------------------------------------------------------------------
// Program generation
// ---------------------------------------------------------------------------

// Documented opcodes only, minus the handful whose divergence is real hardware
// behaviour and is covered by a targeted test instead.
static bool generatable( u8 op )
{
	if ( m6502Opcodes[ op ].undocumented ) return false;

	switch ( op )
	{
	case 0x00:	// BRK  -- interrupt entry clears D on the 65816 (exemption 3)
	case 0x40:	// RTI  -- needs a matching stack frame to be meaningful
	case 0x6C:	// JMP (abs) -- the $xxFF page-wrap bug is fixed (exemption 5)
	case 0x4C:	// JMP abs   -- would jump out of the generated block
	case 0x20:	// JSR
	case 0x60:	// RTS
		return false;

	case 0xF8:	// SED
		// Exemption 2 is not confined to the ADC or SBC that causes it: once a
		// decimal ADC has left N set where the 6502 leaves it clear, the two
		// cores carry different N until something recomputes it, and every
		// subsequent branch on N can diverge too. Masking N and Z only at the
		// arithmetic instruction would therefore be wrong rather than lenient.
		// Decimal mode gets its own test below, which compares what actually
		// has to match: A, C and V.
		return false;

	default:
		return true;
	}
}

// Fills a block with random but well-formed instructions. Operands are chosen
// so the two cores cannot legitimately diverge:
//
//   * absolute bases stay below $FF00, so abs,X and abs,Y can never carry past
//     $FFFF into what the 65816 would treat as bank 1 (exemption 4)
//   * absolute bases stay above $0200, so nothing writes over the program
//   * branch displacements are small and forward, so execution stays inside the
//     block and every instruction is reached from a known state
static u32 generateProgram( u8 *out, u32 size, CRandom &rng )
{
	u32 n = 0;

	while ( n + 3 < size )
	{
		u8 op;
		do { op = (u8)rng.below( 256 ); } while ( !generatable( op ) );

		const u8 mode = m6502Opcodes[ op ].mode;
		const u8 len  = m6502ModeLength[ mode ];

		if ( n + len > size ) break;

		out[ n++ ] = op;

		if ( mode == AM_ABS || mode == AM_ABX || mode == AM_ABY || mode == AM_IND )
		{
			const u16 base = (u16)( 0x0400 + rng.below( 0xFA00 ) );
			out[ n++ ] = (u8)( base & 0xFF );
			out[ n++ ] = (u8)( base >> 8 );
		} else if ( mode == AM_REL )
		{
			out[ n++ ] = (u8)rng.below( 8 );		// short forward branch only
		} else if ( len == 2 )
		{
			out[ n++ ] = (u8)rng.below( 256 );
		}
	}

	return n;
}

// ---------------------------------------------------------------------------
// Comparison
// ---------------------------------------------------------------------------

// P bit 4 is exemption 6: CM6502 keeps it clear in the live register while the
// 65816 keeps it set, because x is held at 1 in emulation mode. Bit 5 is the
// unused bit on one core and M on the other, held set on both. Neither
// difference is observable to a program: the byte PUSHED by PHP and by an
// interrupt is identical, and that byte is compared through the write log.
#define P_COMPARE_MASK ( (u8)~( 0x10 | 0x20 ) )

struct Divergence
{
	const char *what;
	u32  atInstruction;
	u16  pc;
	u8   opcode;
	long long a, b;
};

// Runs both cores over an identical image and reports the first divergence.
// Returns true if one was found.
static bool runDifferential( const u8 *program, u32 length, u32 instructions,
                             Divergence &d )
{
	CDiffBus busA, busB;
	CM6502   c6502;
	CW65C816 c816;

	std::memcpy( &busA.m_Mem[ 0x0200 ], program, length );
	std::memcpy( &busB.m_Mem[ 0x0200 ], program, length );

	// Some junk in zero page and the stack area so indirect modes address
	// something other than $0000, and so the compare is not trivially satisfied
	// by everything being zero.
	CRandom fill( 0xC0FFEE );
	for ( u32 i = 0; i < 0x0200; i++ )
	{
		u8 v = (u8)fill.below( 256 );
		busA.m_Mem[ i ] = v;
		busB.m_Mem[ i ] = v;
	}

	busA.m_Mem[ 0xFFFC ] = busB.m_Mem[ 0xFFFC ] = 0x00;
	busA.m_Mem[ 0xFFFD ] = busB.m_Mem[ 0xFFFD ] = 0x02;

	c6502.attachBus( &busA );
	c816.attachBus( &busB );
	c6502.reset();
	c816.reset();

	#define DIVERGE( name, av, bv )                                     \
		{ d.what = name; d.atInstruction = i; d.pc = pcBefore;          \
		  d.opcode = opcode; d.a = (long long)(av); d.b = (long long)(bv); \
		  return true; }

	for ( u32 i = 0; i < instructions; i++ )
	{
		const u16 pcBefore = c6502.m_PC;
		const u8  opcode   = busA.m_Mem[ pcBefore ];

		// Stop before running off the end of the generated block: past it lies
		// zeroed memory, which decodes as BRK.
		if ( pcBefore < 0x0200 || pcBefore >= 0x0200 + length ) break;

		// Exemption 1. The generator only emits documented opcodes, but a short
		// branch can land mid-instruction and execute an operand byte, which
		// may be one of the 105 encodings the 65816 reinterprets. From there the
		// two instruction streams legitimately diverge -- 50 of those encodings
		// have a different length -- so there is nothing left to compare.
		if ( m6502Opcodes[ opcode ].undocumented ) break;

		// Exemption 2. SED is not generated, but PLP can pull a random byte
		// with D set, so decimal mode is still reachable. Once a decimal ADC or
		// SBC runs, the two cores carry different N -- permanently, until
		// something recomputes it -- and every later branch on N can diverge
		// too. Everything up to this point has been compared; the arithmetic
		// itself is covered by w65c816_diff_decimal_arithmetic.
		if ( ( c6502.m_P & M6502_D )
		     && ( m6502Opcodes[ opcode ].mnemonic[ 0 ] == 'A'
		          || m6502Opcodes[ opcode ].mnemonic[ 0 ] == 'S' )
		     && ( std::strcmp( m6502Opcodes[ opcode ].mnemonic, "ADC" ) == 0
		          || std::strcmp( m6502Opcodes[ opcode ].mnemonic, "SBC" ) == 0 ) )
			break;

		const u32 cyc6502 = c6502.step();
		const u32 cyc816  = c816.step();

		if ( cyc6502 != cyc816 )       DIVERGE( "cycles", cyc6502, cyc816 )
		if ( c6502.m_PC != c816.m_PC ) DIVERGE( "PC", c6502.m_PC, c816.m_PC )
		if ( c6502.m_A != c816.getA() )DIVERGE( "A", c6502.m_A, c816.getA() )
		if ( c6502.m_X != ( c816.m_X & 0xFF ) ) DIVERGE( "X", c6502.m_X, c816.m_X )
		if ( c6502.m_Y != ( c816.m_Y & 0xFF ) ) DIVERGE( "Y", c6502.m_Y, c816.m_Y )
		if ( c6502.m_S != ( c816.m_S & 0xFF ) ) DIVERGE( "S", c6502.m_S, c816.m_S )

		// The 65816 must never leave emulation mode here, and its stack must
		// stay in page 1 and its index registers 8-bit. If any of those slip,
		// applyE() has a hole.
		if ( !c816.m_E )                DIVERGE( "E left emulation mode", 1, 0 )
		if ( ( c816.m_S & 0xFF00 ) != 0x0100 ) DIVERGE( "S left page 1", 0x0100, c816.m_S )
		if ( c816.m_X > 0xFF )          DIVERGE( "XH not held at 0", 0, c816.m_X )
		if ( c816.m_Y > 0xFF )          DIVERGE( "YH not held at 0", 0, c816.m_Y )

		const u8 pA = (u8)( c6502.m_P & P_COMPARE_MASK );
		const u8 pB = (u8)( c816.m_P  & P_COMPARE_MASK );

		if ( pA != pB ) DIVERGE( "P", pA, pB )

		if ( busA.m_WriteCount != busB.m_WriteCount )
			DIVERGE( "write count", busA.m_WriteCount, busB.m_WriteCount )

		for ( u32 w = 0; w < busA.m_WriteCount; w++ )
		{
			if ( busA.m_Writes[ w ].addr != busB.m_Writes[ w ].addr )
				DIVERGE( "write address", busA.m_Writes[ w ].addr, busB.m_Writes[ w ].addr )
			if ( busA.m_Writes[ w ].value != busB.m_Writes[ w ].value )
				DIVERGE( "write value", busA.m_Writes[ w ].value, busB.m_Writes[ w ].value )
		}
		busA.m_WriteCount = busB.m_WriteCount = 0;

		if ( std::memcmp( busA.m_Mem, busB.m_Mem, 0x10000 ) != 0 )
			DIVERGE( "memory", 0, 1 )
	}

	#undef DIVERGE

	return false;
}

static void report( const Divergence &d )
{
	std::printf( "\n  diverged on %s after %u instructions, at $%04X (opcode $%02X %s)\n"
	             "    6502 = $%llX   65816 = $%llX\n",
	             d.what, d.atInstruction, d.pc, d.opcode,
	             m6502Opcodes[ d.opcode ].mnemonic,
	             (unsigned long long)d.a, (unsigned long long)d.b );
}

// ---------------------------------------------------------------------------

TEST( w65c816_diff_random_programs )
{
	// 64 independent programs of ~1000 bytes each. Enough to hit every
	// documented opcode many times over with varied operands and flag states.
	for ( u32 seed = 1; seed <= 64; seed++ )
	{
		u8 program[ 1024 ];
		CRandom rng( seed * 2654435761u );
		const u32 len = generateProgram( program, sizeof( program ), rng );

		Divergence d;
		if ( runDifferential( program, len, 4000, d ) )
		{
			std::printf( "\n  seed %u:", seed );
			report( d );
			CHECK( false );
			return;			// one report is enough; more would be noise
		}
	}

	CHECK( true );
}

// Every documented opcode, exercised deliberately rather than by chance, from a
// set of starting flag states. The random generator will reach these too, but
// this makes a regression name the opcode it broke.
TEST( w65c816_diff_every_documented_opcode )
{
	for ( u32 op = 0; op < 256; op++ )
	{
		if ( !generatable( (u8)op ) ) continue;

		const u8 mode = m6502Opcodes[ op ].mode;
		const u8 len  = m6502ModeLength[ mode ];

		// A flag setup, the opcode under test three times with different
		// operands, then a couple of NOPs. Decimal mode is deliberately absent:
		// it is exempt (exemption 2) and has its own test.
		static const u8 setups[][ 2 ] = {
			{ 0x18, 0xB8 },		// CLC CLV
			{ 0x38, 0xB8 },		// SEC CLV
			{ 0x18, 0x78 },		// CLC SEI
			{ 0x38, 0x58 },		// SEC CLI
		};

		for ( u32 s = 0; s < 4; s++ )
		{
			u8 program[ 64 ];
			u32 n = 0;

			program[ n++ ] = 0xA9; program[ n++ ] = 0x99;	// LDA #$99
			program[ n++ ] = 0xA2; program[ n++ ] = 0x37;	// LDX #$37
			program[ n++ ] = 0xA0; program[ n++ ] = 0x5B;	// LDY #$5B
			program[ n++ ] = setups[ s ][ 0 ];
			program[ n++ ] = setups[ s ][ 1 ];

			for ( u32 rep = 0; rep < 3; rep++ )
			{
				program[ n++ ] = (u8)op;
				if ( mode == AM_ABS || mode == AM_ABX || mode == AM_ABY || mode == AM_IND )
				{
					const u16 base = (u16)( 0x0800 + rep * 0x1000 );
					program[ n++ ] = (u8)( base & 0xFF );
					program[ n++ ] = (u8)( base >> 8 );
				} else if ( mode == AM_REL )
				{
					program[ n++ ] = 0x00;		// branch to the next instruction either way
				} else if ( len == 2 )
				{
					program[ n++ ] = (u8)( 0x40 + rep * 0x33 );
				}
			}

			program[ n++ ] = 0xEA;
			program[ n++ ] = 0xEA;

			Divergence d;
			if ( runDifferential( program, n, 32, d ) )
			{
				std::printf( "\n  opcode $%02X %s, setup %u:",
				             (unsigned)op, m6502Opcodes[ op ].mnemonic, s );
				report( d );
				CHECK( false );
				return;
			}
		}
	}

	CHECK( true );
}

// ---------------------------------------------------------------------------
// The exemptions, asserted rather than assumed. Each of these SHOULD differ; a
// test that only checked agreement would silently pass if a core stopped doing
// the 65816-specific thing.
// ---------------------------------------------------------------------------

// Exemption 2, stated precisely. Across every 8-bit operand pair and both carry
// values, decimal ADC and SBC must produce the same A, C and V on both cores.
// N and Z must NOT match everywhere: the 65816 computes them from the final BCD
// result (the 65C02 rule) while the NMOS 6502 takes N from the pre-fixup high
// nibble and Z from the binary result. If they ever agreed everywhere, this
// core would have copied CM6502's decimal path by mistake.
TEST( w65c816_diff_decimal_arithmetic )
{
	int nDiffered = 0, zDiffered = 0, cases = 0;

	for ( int isSBC = 0; isSBC <= 1; isSBC++ )
	for ( int carry = 0; carry <= 1; carry++ )
	for ( u32 a = 0; a < 256; a++ )
	for ( u32 b = 0; b < 256; b++ )
	{
		// SED, CLC/SEC, LDA #a, ADC/SBC #b
		const u8 program[] = {
			0xF8,
			(u8)( carry ? 0x38 : 0x18 ),
			0xA9, (u8)a,
			(u8)( isSBC ? 0xE9 : 0x69 ), (u8)b
		};

		CDiffBus busA, busB;
		CM6502   c6502;
		CW65C816 c816;

		std::memcpy( &busA.m_Mem[ 0x0200 ], program, sizeof( program ) );
		std::memcpy( &busB.m_Mem[ 0x0200 ], program, sizeof( program ) );
		busA.m_Mem[ 0xFFFC ] = busB.m_Mem[ 0xFFFC ] = 0x00;
		busA.m_Mem[ 0xFFFD ] = busB.m_Mem[ 0xFFFD ] = 0x02;

		c6502.attachBus( &busA ); c6502.reset();
		c816.attachBus( &busB );  c816.reset();

		for ( int i = 0; i < 4; i++ ) { c6502.step(); c816.step(); }

		cases++;

		if ( c6502.m_A != c816.getA() )
		{
			std::printf( "\n  %s #$%02X %s #$%02X carry=%d: A $%02X vs $%02X",
			             "LDA", (unsigned)a, isSBC ? "SBC" : "ADC", (unsigned)b,
			             carry, c6502.m_A, c816.getA() );
			CHECK( false );
			return;
		}

		if ( ( c6502.m_P & M6502_C ) != 0 ) { if ( ( c816.m_P & W65_C ) == 0 ) { CHECK( false ); return; } }
		else                                { if ( ( c816.m_P & W65_C ) != 0 ) { CHECK( false ); return; } }

		if ( ( ( c6502.m_P & M6502_V ) != 0 ) != ( ( c816.m_P & W65_V ) != 0 ) )
		{
			std::printf( "\n  %s #$%02X carry=%d: V differs", isSBC ? "SBC" : "ADC",
			             (unsigned)b, carry );
			CHECK( false );
			return;
		}

		if ( ( ( c6502.m_P & M6502_N ) != 0 ) != ( ( c816.m_P & W65_N ) != 0 ) ) nDiffered++;
		if ( ( ( c6502.m_P & M6502_Z ) != 0 ) != ( ( c816.m_P & W65_Z ) != 0 ) ) zDiffered++;

		// The 65C02 rule, asserted directly rather than by comparison: on the
		// 65816, N and Z always describe the accumulator that came out.
		const u8 result = c816.getA();
		CHECK_EQ( ( c816.m_P & W65_N ) != 0, ( result & 0x80 ) != 0 );
		CHECK_EQ( ( c816.m_P & W65_Z ) != 0, result == 0 );
	}

	CHECK_EQ( cases, 2 * 2 * 256 * 256 );

	// The reference measured roughly half of decimal ADC results disagreeing on
	// N and a much smaller number on Z. The exact counts depend on the operand
	// set; what matters is that both are substantial and non-zero, because zero
	// would mean this core is running the NMOS rule.
	CHECK( nDiffered > 1000 );
	CHECK( zDiffered > 0 );
}

TEST( w65c816_diff_exemption_jmp_indirect_page_wrap )
{
	// JMP ($10FF). The NMOS 6502 reads the high byte from $1000; the 65816
	// fixed the bug and reads $1100.
	const u8 program[] = { 0x6C, 0xFF, 0x10 };

	CDiffBus busA, busB;
	CM6502   c6502;
	CW65C816 c816;

	for ( int i = 0; i < 2; i++ )
	{
		CDiffBus &b = i ? busB : busA;
		std::memcpy( &b.m_Mem[ 0x0200 ], program, sizeof( program ) );
		b.m_Mem[ 0x10FF ] = 0x34;
		b.m_Mem[ 0x1000 ] = 0xAA;		// what the 6502 will pick up
		b.m_Mem[ 0x1100 ] = 0xBB;		// what the 65816 will pick up
		b.m_Mem[ 0xFFFC ] = 0x00;
		b.m_Mem[ 0xFFFD ] = 0x02;
	}

	c6502.attachBus( &busA ); c6502.reset();
	c816.attachBus( &busB );  c816.reset();

	CHECK_EQ( c6502.step(), 5 );
	CHECK_EQ( c816.step(), 5 );			// same cost, different answer

	CHECK_EQ( c6502.m_PC, 0xAA34 );
	CHECK_EQ( c816.m_PC, 0xBB34 );
}

TEST( w65c816_diff_exemption_interrupt_clears_decimal )
{
	// SED then BRK. The NMOS 6502 leaves D set on entry to the handler; the
	// 65816 clears it, in both modes.
	const u8 program[] = { 0xF8, 0x00, 0xEA };

	CDiffBus busA, busB;
	CM6502   c6502;
	CW65C816 c816;

	for ( int i = 0; i < 2; i++ )
	{
		CDiffBus &b = i ? busB : busA;
		std::memcpy( &b.m_Mem[ 0x0200 ], program, sizeof( program ) );
		b.m_Mem[ 0xFFFC ] = 0x00; b.m_Mem[ 0xFFFD ] = 0x02;
		b.m_Mem[ 0xFFFE ] = 0x00; b.m_Mem[ 0xFFFF ] = 0x30;	// handler at $3000
	}

	c6502.attachBus( &busA ); c6502.reset();
	c816.attachBus( &busB );  c816.reset();

	c6502.step(); c6502.step();		// SED, BRK
	c816.step();  c816.step();

	CHECK_EQ( c6502.m_PC, 0x3000 );
	CHECK_EQ( c816.m_PC, 0x3000 );

	CHECK( ( c6502.m_P & M6502_D ) != 0 );		// 6502 keeps it
	CHECK( ( c816.m_P & W65_D ) == 0 );			// 65816 clears it

	// The byte PUSHED must still be identical -- both push P with D set and the
	// B bit set, which is what a handler actually inspects.
	CHECK_EQ( busA.m_Mem[ 0x01FB ], busB.m_Mem[ 0x01FB ] );
	CHECK( ( busA.m_Mem[ 0x01FB ] & 0x08 ) != 0 );
	CHECK( ( busA.m_Mem[ 0x01FB ] & 0x10 ) != 0 );
}

TEST( w65c816_diff_exemption_absolute_indexed_crosses_bank )
{
	// LDA $FFFF,X with X=1. The 6502 wraps to $0000; the 65816 carries into
	// bank 1. With DBR=$00 that is the one addressing difference between
	// emulation mode and a real 6502.
	const u8 program[] = { 0xA2, 0x01, 0xBD, 0xFF, 0xFF };

	CDiffBus busA;
	CM6502   c6502;
	c6502.attachBus( &busA );
	std::memcpy( &busA.m_Mem[ 0x0200 ], program, sizeof( program ) );
	busA.m_Mem[ 0xFFFC ] = 0x00; busA.m_Mem[ 0xFFFD ] = 0x02;
	busA.m_Mem[ 0x0000 ] = 0x5A;
	c6502.reset();
	c6502.step(); c6502.step();
	CHECK_EQ( c6502.m_A, 0x5A );		// wrapped to $0000

	// The 65816 needs a 24-bit bus to show the difference, so it gets one.
	class CBankBus : public ICpuBus
	{
	public:
		u8 m_Mem[ 0x20000 ];
		u32 m_LastRead;
		CBankBus() : m_LastRead( 0 ) { std::memset( m_Mem, 0, sizeof( m_Mem ) ); }
		u8   read8( scpu_addr_t a ) override        { m_LastRead = a; return m_Mem[ a & 0x1FFFF ]; }
		void write8( scpu_addr_t a, u8 v ) override { m_Mem[ a & 0x1FFFF ] = v; }
		bool irqAsserted() override { return false; }
		bool nmiAsserted() override { return false; }
		void tick( u32 ) override {}
	} busB;

	CW65C816 c816;
	c816.attachBus( &busB );
	std::memcpy( &busB.m_Mem[ 0x0200 ], program, sizeof( program ) );
	busB.m_Mem[ 0xFFFC ] = 0x00; busB.m_Mem[ 0xFFFD ] = 0x02;
	busB.m_Mem[ 0x00000 ] = 0x5A;
	busB.m_Mem[ 0x10000 ] = 0xC3;		// bank 1
	c816.reset();
	c816.step(); c816.step();

	CHECK_EQ( busB.m_LastRead, 0x10000 );
	CHECK_EQ( c816.getA(), 0xC3 );
}

// ---------------------------------------------------------------------------
// The claim that makes the differential test fair: evaluated at e=1, m=1, x=1
// and an aligned direct page, the 65816 table must reproduce the 6502 table
// exactly for every documented opcode.
// ---------------------------------------------------------------------------

TEST( w65c816_table_matches_m6502_in_emulation_mode )
{
	int lengthMismatch = 0, cycleMismatch = 0;

	for ( u32 op = 0; op < 256; op++ )
	{
		if ( m6502Opcodes[ op ].undocumented ) continue;

		const u8 want = m6502ModeLength[ m6502Opcodes[ op ].mode ];
		const u8 got  = w65c816Length( (u8)op, true, true );

		if ( want != got )
		{
			std::printf( "\n  $%02X %s: length %u, 65816 says %u",
			             (unsigned)op, m6502Opcodes[ op ].mnemonic, want, got );
			lengthMismatch++;
		}

		const u8 wantC = m6502Opcodes[ op ].cycles;
		const u8 gotC  = w65c816Cycles( (u8)op, true, true, true, false, false, false );

		if ( wantC != gotC )
		{
			std::printf( "\n  $%02X %s: %u cycles, 65816 says %u",
			             (unsigned)op, m6502Opcodes[ op ].mnemonic, wantC, gotC );
			cycleMismatch++;
		}
	}

	CHECK_EQ( lengthMismatch, 0 );
	CHECK_EQ( cycleMismatch, 0 );
}
