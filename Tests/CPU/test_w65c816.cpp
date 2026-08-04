/*
   SCPU-EMU - WDC 65C816 core tests: everything the differential test cannot
   reach, which is everything that makes a 65816 not a 6502.

   Where a case below cites a specific address or register value, it is a worked
   example from Docs/research/65816-reference.md, most of which were measured
   against SingleStepTests, VICE and the WDC datasheet rather than reasoned out.
   Keeping the numbers is the point: they are evidence, not illustration.

   Copyright (c) 2026 SCPU-EMU contributors, GPLv3 -- see w65c816.h.
*/
#include "../test_framework.h"
#include "../../Source/CPU/W65C816/w65c816.h"
#include "../../Source/CPU/W65C816/w65c816_opcodes.h"

#define W65T_LOG 512

// A full 24-bit address space, logging every access so tests can assert on the
// exact addresses an instruction touches -- which is where the wrapping rules
// live. 16MB is allocated once and shared; each fixture clears what it needs.
static u8 s_Mem[ 1 << 24 ];

class CW65Bus : public ICpuBus
{
public:
	struct Access { u32 addr; u8 value; bool write; };
	Access m_Log[ W65T_LOG ];
	u32    m_LogCount;
	bool   m_IRQ, m_NMI;
	u64    m_Ticks;

	CW65Bus() : m_LogCount( 0 ), m_IRQ( false ), m_NMI( false ), m_Ticks( 0 ) {}

	u8 read8( scpu_addr_t a ) override
	{
		u8 v = s_Mem[ a & 0xFFFFFF ];
		log( a, v, false );
		return v;
	}

	void write8( scpu_addr_t a, u8 v ) override
	{
		s_Mem[ a & 0xFFFFFF ] = v;
		log( a, v, true );
	}

	bool irqAsserted() override { return m_IRQ; }
	bool nmiAsserted() override { return m_NMI; }
	void tick( u32 n ) override { m_Ticks += n; }

	void clearLog() { m_LogCount = 0; }

	// Reads and writes are logged together, but tests usually care about one or
	// the other. These pick out the nth of each kind.
	bool nthWrite( u32 n, u32 &addr, u8 &value ) const
	{
		u32 seen = 0;
		for ( u32 i = 0; i < m_LogCount; i++ )
			if ( m_Log[ i ].write && seen++ == n )
			{
				addr = m_Log[ i ].addr; value = m_Log[ i ].value; return true;
			}
		return false;
	}

	u32 writeCount() const
	{
		u32 n = 0;
		for ( u32 i = 0; i < m_LogCount; i++ ) if ( m_Log[ i ].write ) n++;
		return n;
	}

	// True if any read touched exactly this address.
	bool readAddress( u32 addr ) const
	{
		for ( u32 i = 0; i < m_LogCount; i++ )
			if ( !m_Log[ i ].write && m_Log[ i ].addr == addr ) return true;
		return false;
	}

private:
	void log( u32 a, u8 v, bool w )
	{
		if ( m_LogCount < W65T_LOG )
		{
			m_Log[ m_LogCount ].addr  = a & 0xFFFFFF;
			m_Log[ m_LogCount ].value = v;
			m_Log[ m_LogCount ].write = w;
			m_LogCount++;
		}
	}
};

struct W65Fixture
{
	CW65Bus  bus;
	CW65C816 cpu;

	W65Fixture( const u8 *code, u32 len, u32 org = 0x000200 )
	{
		std::memset( s_Mem, 0, 1 << 24 );
		std::memcpy( &s_Mem[ org ], code, len );
		s_Mem[ 0xFFFC ] = (u8)( org & 0xFF );
		s_Mem[ 0xFFFD ] = (u8)( ( org >> 8 ) & 0xFF );
		cpu.attachBus( &bus );
		cpu.reset();
		bus.clearLog();
	}

	void run( u32 instructions )
	{
		for ( u32 i = 0; i < instructions; i++ ) cpu.step();
	}
};

// CLC : XCE -- the standard way into native mode. Two instructions.
#define GO_NATIVE 0x18, 0xFB

// REP #$30 -- 16-bit accumulator and index registers.
#define WIDE_ALL 0xC2, 0x30

// ---------------------------------------------------------------------------
// Mode switching
// ---------------------------------------------------------------------------

TEST( w65c816_reset_enters_emulation_mode )
{
	const u8 code[] = { 0xEA };
	W65Fixture f( code, sizeof( code ), 0x1234 );

	CHECK( f.cpu.m_E );
	CHECK_EQ( f.cpu.m_PC, 0x1234 );
	CHECK_EQ( f.cpu.m_S, 0x01FD );
	CHECK_EQ( f.cpu.m_D, 0 );
	CHECK_EQ( f.cpu.m_DBR, 0 );
	CHECK_EQ( f.cpu.m_PBR, 0 );
	CHECK( ( f.cpu.m_P & W65_I ) != 0 );
	CHECK( ( f.cpu.m_P & W65_D ) == 0 );
	CHECK( f.cpu.memory8() );
	CHECK( f.cpu.index8() );
}

TEST( w65c816_xce_enters_and_leaves_native_mode )
{
	// CLC XCE  -> native, then SEC XCE -> back to emulation.
	const u8 code[] = { GO_NATIVE, 0x38, 0xFB };
	W65Fixture f( code, sizeof( code ) );

	f.run( 2 );
	CHECK( !f.cpu.m_E );
	CHECK( ( f.cpu.m_P & W65_C ) != 0 );	// the old E came back in C

	// m and x were being HELD at 1, not set; they simply become writable.
	CHECK( ( f.cpu.m_P & W65_M ) != 0 );
	CHECK( ( f.cpu.m_P & W65_X ) != 0 );

	f.run( 2 );
	CHECK( f.cpu.m_E );
	CHECK( ( f.cpu.m_P & W65_C ) == 0 );
}

TEST( w65c816_entering_emulation_destroys_index_high_bytes_but_keeps_b )
{
	// Native, 16-bit everything: load A=$1234, X=$5678, Y=$9ABC, S=$0234, then
	// SEC XCE back to emulation.
	//
	// The datasheet is explicit and this is the most-missed detail on the chip:
	// narrowing the ACCUMULATOR never loses data -- B survives and is still
	// reachable via XBA -- while narrowing the INDEX registers always does.
	const u8 code[] = {
		GO_NATIVE,
		WIDE_ALL,
		0xA9, 0x34, 0x12,		// LDA #$1234
		0xA2, 0x78, 0x56,		// LDX #$5678
		0xA0, 0xBC, 0x9A,		// LDY #$9ABC
		0x48,					// PHA          (so TCS has something safe to do)
		0x68,					// PLA
		0xA9, 0x34, 0x02,		// LDA #$0234
		0x1B,					// TCS
		0xA9, 0x34, 0x12,		// LDA #$1234
		0x38, 0xFB				// SEC XCE
	};
	W65Fixture f( code, sizeof( code ) );
	f.run( 13 );

	CHECK( f.cpu.m_E );
	CHECK_EQ( f.cpu.m_S, 0x0134 );		// SH forced to $01, SL preserved
	CHECK_EQ( f.cpu.m_X, 0x0078 );		// XH destroyed
	CHECK_EQ( f.cpu.m_Y, 0x00BC );		// YH destroyed
	CHECK_EQ( f.cpu.m_C, 0x1234 );		// A AND B both preserved

	// And B really is still reachable.
	const u8 xba[] = { 0xEB };
	std::memcpy( &s_Mem[ f.cpu.m_PC ], xba, sizeof( xba ) );
	f.cpu.step();
	CHECK_EQ( f.cpu.getA(), 0x12 );
}

TEST( w65c816_narrowing_x_destroys_the_high_bytes_immediately )
{
	// SEP #$10 : REP #$10 : the high bytes must already be gone. Not deferred,
	// not lazily masked.
	const u8 code[] = {
		GO_NATIVE, WIDE_ALL,
		0xA2, 0x78, 0x56,		// LDX #$5678
		0xE2, 0x10,				// SEP #$10   x=1
		0xC2, 0x10				// REP #$10   x=0 again
	};
	W65Fixture f( code, sizeof( code ) );
	f.run( 4 );
	CHECK_EQ( f.cpu.m_X, 0x5678 );

	f.run( 1 );					// SEP #$10
	CHECK_EQ( f.cpu.m_X, 0x0078 );

	f.run( 1 );					// REP #$10 -- widening cannot bring it back
	CHECK_EQ( f.cpu.m_X, 0x0078 );
}

TEST( w65c816_emulation_mode_holds_m_and_x_against_rep )
{
	// REP #$3F in emulation clears C, Z, I and D but leaves m and x at 1.
	const u8 code[] = { 0x38, 0xF8, 0x78, 0xC2, 0x3F };	// SEC SED SEI REP #$3F
	W65Fixture f( code, sizeof( code ) );
	f.run( 4 );

	CHECK( ( f.cpu.m_P & W65_C ) == 0 );
	CHECK( ( f.cpu.m_P & W65_D ) == 0 );
	CHECK( ( f.cpu.m_P & W65_I ) == 0 );
	CHECK( ( f.cpu.m_P & W65_M ) != 0 );
	CHECK( ( f.cpu.m_P & W65_X ) != 0 );
}

// ---------------------------------------------------------------------------
// Address wrapping -- the worked examples
// ---------------------------------------------------------------------------

TEST( w65c816_direct_page_indexed_stays_in_bank_zero )
{
	// Native, D=$6524, X=$9A85, LDA $D1,X. The raw sum is $1_007A; truncating
	// to 16 bits gives EA $00007A, NOT $01007A. 2553 measured cases, all in
	// bank 0.
	const u8 code[] = {
		GO_NATIVE, WIDE_ALL,
		0xA9, 0x24, 0x65,		// LDA #$6524
		0x5B,					// TCD
		0xA2, 0x85, 0x9A,		// LDX #$9A85
		0xE2, 0x20,				// SEP #$20   8-bit accumulator, so one read
		0xB5, 0xD1				// LDA $D1,X
	};
	W65Fixture f( code, sizeof( code ) );
	f.run( 7 );

	s_Mem[ 0x00007A ] = 0x5A;
	s_Mem[ 0x01007A ] = 0xA5;
	f.bus.clearLog();
	f.cpu.step();

	CHECK_EQ( f.cpu.getA(), 0x5A );
	CHECK( f.bus.readAddress( 0x00007A ) );
	CHECK( !f.bus.readAddress( 0x01007A ) );
}

TEST( w65c816_direct_page_16bit_operand_wraps_within_bank_zero )
{
	// D=$FF00, m=0, LDA $FF: low byte from $00FFFF, HIGH byte from $000000 --
	// the direct page is confined to bank 0, so the operand cannot run on into
	// bank 1.
	const u8 code[] = {
		GO_NATIVE, WIDE_ALL,
		0xA9, 0x00, 0xFF,		// LDA #$FF00
		0x5B,					// TCD
		0xA5, 0xFF				// LDA $FF
	};
	W65Fixture f( code, sizeof( code ) );
	f.run( 5 );

	s_Mem[ 0x00FFFF ] = 0x34;
	s_Mem[ 0x000000 ] = 0x12;
	s_Mem[ 0x010000 ] = 0xEE;		// what a 24-bit continuation would pick up
	f.bus.clearLog();
	f.cpu.step();

	CHECK_EQ( f.cpu.m_C, 0x1234 );
	CHECK( f.bus.readAddress( 0x000000 ) );
	CHECK( !f.bus.readAddress( 0x010000 ) );
}

TEST( w65c816_emulation_direct_page_indexed_wraps_at_ff_when_aligned )
{
	// Emulation, D=$C700 (DL=$00), X=$B2, LDA $EA,X: ($EA+$B2)&$FF = $9C, so
	// EA $00C79C. A plain 16-bit add would give $00C89C. 21/21 measured.
	const u8 code[] = {
		GO_NATIVE, WIDE_ALL,
		0xA9, 0x00, 0xC7,		// LDA #$C700
		0x5B,					// TCD
		0x38, 0xFB,				// SEC XCE  -- back to emulation, D survives
		0xA2, 0xB2,				// LDX #$B2
		0xB5, 0xEA				// LDA $EA,X
	};
	W65Fixture f( code, sizeof( code ) );
	f.run( 8 );

	CHECK( f.cpu.m_E );
	CHECK_EQ( f.cpu.m_D, 0xC700 );		// D survives a mode change

	s_Mem[ 0x00C79C ] = 0x5A;
	s_Mem[ 0x00C89C ] = 0xA5;
	f.bus.clearLog();
	f.cpu.step();

	CHECK_EQ( f.cpu.getA(), 0x5A );
}

TEST( w65c816_emulation_direct_page_indexed_does_not_wrap_when_unaligned )
{
	// The SAME instruction with DL != 0 uses the full 16-bit add, even in
	// emulation mode. The datasheet's own example: D=$3401, X=1, LDA $FF,X
	// reads $3501, not $3401. All three conditions have to hold for the 8-bit
	// wrap -- E=1 AND DL=$00 AND a mode a 65C02 had.
	const u8 code[] = {
		GO_NATIVE,
		0xF4, 0x01, 0x34,		// PEA #$3401
		0x2B,					// PLD
		0x38, 0xFB,				// SEC XCE
		0xA2, 0x01,				// LDX #$01
		0xB5, 0xFF				// LDA $FF,X
	};
	W65Fixture f( code, sizeof( code ) );
	f.run( 7 );

	CHECK( f.cpu.m_E );
	CHECK_EQ( f.cpu.m_D, 0x3401 );

	s_Mem[ 0x003501 ] = 0x5A;
	s_Mem[ 0x003401 ] = 0xA5;
	f.bus.clearLog();
	f.cpu.step();

	CHECK_EQ( f.cpu.getA(), 0x5A );
}

TEST( w65c816_indirect_long_runs_off_the_end_of_the_direct_page )
{
	// [d], [d],Y and PEI are the three modes the datasheet EXCLUDES from the
	// direct-page wrap. With E=1 and D=$2F00, LDA [$FE] reads its pointer from
	// $002FFE, $002FFF and $003000 -- past the end of the page.
	const u8 code[] = {
		GO_NATIVE,
		0xF4, 0x00, 0x2F,		// PEA #$2F00
		0x2B,					// PLD
		0x38, 0xFB,				// SEC XCE
		0xA7, 0xFE				// LDA [$FE]
	};
	W65Fixture f( code, sizeof( code ) );
	f.run( 6 );

	CHECK( f.cpu.m_E );
	CHECK_EQ( f.cpu.m_D, 0x2F00 );

	s_Mem[ 0x002FFE ] = 0x11;
	s_Mem[ 0x002FFF ] = 0x22;
	s_Mem[ 0x003000 ] = 0x33;		// bank byte, from OUTSIDE the direct page
	s_Mem[ 0x332211 ] = 0x5A;
	f.bus.clearLog();
	f.cpu.step();

	CHECK( f.bus.readAddress( 0x003000 ) );
	CHECK_EQ( f.cpu.getA(), 0x5A );
}

TEST( w65c816_dp_pointer_high_byte_wraps_in_page )
{
	// U1, pinned. E=1, D=$0000, LDA ($FF),Y: does the pointer's high byte come
	// from $0000 (wrap) or $0100 (no wrap)?
	//
	// The datasheet lists only [d], [d],Y and PEI as exceptions; Bruce Clark
	// and VICE both wrap; SingleStepTests does not. We wrap. If a real SuperCPU
	// ever meets a logic analyser and disagrees, flip
	// W65C816_DP_POINTER_WRAP -- and this test is where the decision is
	// recorded, so it should fail loudly rather than quietly change meaning.
	const u8 code[] = { 0xA0, 0x00, 0xB1, 0xFF };		// LDY #$00 : LDA ($FF),Y
	W65Fixture f( code, sizeof( code ) );

	s_Mem[ 0x0000FF ] = 0x34;		// pointer low
	s_Mem[ 0x000000 ] = 0x12;		// pointer high, WRAPPED
	s_Mem[ 0x000100 ] = 0xEE;		// pointer high, unwrapped
	s_Mem[ 0x001234 ] = 0x5A;
	s_Mem[ 0x00EE34 ] = 0xA5;

	f.run( 2 );

#if W65C816_DP_POINTER_WRAP
	CHECK_EQ( f.cpu.getA(), 0x5A );
#else
	CHECK_EQ( f.cpu.getA(), 0xA5 );
#endif
}

TEST( w65c816_absolute_indexed_carries_into_the_next_bank )
{
	// Emulation, DBR=$7C, X=$93, LDA $FFA9,X -> $7D003C. A true 24-bit add, in
	// BOTH modes. 17/17 measured bank-crossing cases carried; none wrapped.
	const u8 code[] = {
		0xA9, 0x7C,				// LDA #$7C
		0x48,					// PHA
		0xAB,					// PLB      DBR = $7C
		0xA2, 0x93,				// LDX #$93
		0xBD, 0xA9, 0xFF		// LDA $FFA9,X
	};
	W65Fixture f( code, sizeof( code ) );
	f.run( 4 );

	CHECK_EQ( f.cpu.m_DBR, 0x7C );

	s_Mem[ 0x7D003C ] = 0x5A;
	s_Mem[ 0x7C003C ] = 0xA5;
	f.bus.clearLog();
	f.cpu.step();

	CHECK_EQ( f.cpu.getA(), 0x5A );
	CHECK( f.bus.readAddress( 0x7D003C ) );
}

TEST( w65c816_long_indexed_is_a_24bit_add )
{
	// X=$000A, m=0, LDA $12FFFE,X -> low byte from $130008, high from $130009.
	const u8 code[] = {
		GO_NATIVE, WIDE_ALL,
		0xA2, 0x0A, 0x00,			// LDX #$000A
		0xBF, 0xFE, 0xFF, 0x12		// LDA $12FFFE,X
	};
	W65Fixture f( code, sizeof( code ) );
	f.run( 4 );

	s_Mem[ 0x130008 ] = 0x34;
	s_Mem[ 0x130009 ] = 0x12;
	f.bus.clearLog();
	f.cpu.step();

	CHECK_EQ( f.cpu.m_C, 0x1234 );
	CHECK( f.bus.readAddress( 0x130008 ) );
	CHECK( f.bus.readAddress( 0x130009 ) );
}

TEST( w65c816_program_counter_wraps_inside_its_bank )
{
	// An instruction at the very top of a bank is followed by the one at
	// $xx0000, not by the next bank. PBR does not increment.
	const u8 code[] = { 0xEA };
	W65Fixture f( code, sizeof( code ), 0x12FFFF );

	f.cpu.m_PBR = 0x12;
	f.cpu.m_PC  = 0xFFFF;
	s_Mem[ 0x12FFFF ] = 0xE8;		// INX
	s_Mem[ 0x120000 ] = 0xE8;		// INX

	f.cpu.step();
	CHECK_EQ( f.cpu.m_PBR, 0x12 );
	CHECK_EQ( f.cpu.m_PC, 0x0000 );
	CHECK_EQ( f.cpu.m_X, 1 );
}

// ---------------------------------------------------------------------------
// The stack, and the old/new split
// ---------------------------------------------------------------------------

TEST( w65c816_native_stack_is_16bit_and_wraps_in_bank_zero )
{
	// Native, m=0, PHA with S=$0001: writes AH at $000001, AL at $000000, and
	// leaves S=$FFFF. 0 of 40000 measured native stack accesses left bank 0.
	const u8 code[] = {
		GO_NATIVE, WIDE_ALL,
		0xA9, 0x01, 0x00,		// LDA #$0001
		0x1B,					// TCS
		0xA9, 0x34, 0x12,		// LDA #$1234
		0x48					// PHA
	};
	W65Fixture f( code, sizeof( code ) );
	f.run( 6 );
	CHECK_EQ( f.cpu.m_S, 0x0001 );

	f.bus.clearLog();
	f.cpu.step();

	u32 addr = 0; u8 value = 0;
	CHECK_EQ( f.bus.writeCount(), 2 );
	CHECK( f.bus.nthWrite( 0, addr, value ) );
	CHECK_EQ( addr, 0x000001 );		// high byte first
	CHECK_EQ( value, 0x12 );
	CHECK( f.bus.nthWrite( 1, addr, value ) );
	CHECK_EQ( addr, 0x000000 );
	CHECK_EQ( value, 0x34 );
	CHECK_EQ( f.cpu.m_S, 0xFFFF );
}

TEST( w65c816_emulation_old_stack_wraps_inside_page_one )
{
	// JSR abs with S=$0100: writes $000100 then $0001FF -- it wraps INSIDE the
	// page. This is the "old" class, everything a 65C02 had.
	const u8 code[] = {
		0xA2, 0x00,				// LDX #$00
		0x9A,					// TXS      S = $0100
		0x20, 0x00, 0x30		// JSR $3000
	};
	W65Fixture f( code, sizeof( code ) );
	f.run( 2 );
	CHECK_EQ( f.cpu.m_S, 0x0100 );

	f.bus.clearLog();
	f.cpu.step();

	u32 addr = 0; u8 value = 0;
	CHECK_EQ( f.bus.writeCount(), 2 );
	CHECK( f.bus.nthWrite( 0, addr, value ) );
	CHECK_EQ( addr, 0x000100 );
	CHECK( f.bus.nthWrite( 1, addr, value ) );
	CHECK_EQ( addr, 0x0001FF );		// wrapped, NOT $0000FF
	CHECK_EQ( f.cpu.m_S, 0x01FE );
}

TEST( w65c816_emulation_new_stack_runs_off_the_bottom_of_page_one )
{
	// PEA with S=$0100: writes $000100 then $0000FF -- it does NOT wrap. PEA is
	// on the datasheet's "new" list along with JSL, JSR (a,X), PEI, PER, PHD,
	// PLD and RTL. Note the final S is back inside page 1 either way, which is
	// what makes this so easy to get wrong.
	const u8 code[] = {
		0xA2, 0x00,				// LDX #$00
		0x9A,					// TXS      S = $0100
		0xF4, 0x34, 0x12		// PEA #$1234
	};
	W65Fixture f( code, sizeof( code ) );
	f.run( 2 );
	CHECK_EQ( f.cpu.m_S, 0x0100 );

	f.bus.clearLog();
	f.cpu.step();

	u32 addr = 0; u8 value = 0;
	CHECK_EQ( f.bus.writeCount(), 2 );
	CHECK( f.bus.nthWrite( 0, addr, value ) );
	CHECK_EQ( addr, 0x000100 );
	CHECK_EQ( value, 0x12 );
	CHECK( f.bus.nthWrite( 1, addr, value ) );
	CHECK_EQ( addr, 0x0000FF );		// off the bottom of the page
	CHECK_EQ( value, 0x34 );
	CHECK_EQ( f.cpu.m_S, 0x01FE );	// but S still lands inside page 1
}

TEST( w65c816_emulation_jsl_pushes_three_bytes_off_the_page )
{
	// JSL with S=$0100: $000100, $0000FF, $0000FE, leaving S=$01FD.
	const u8 code[] = {
		0xA2, 0x00,					// LDX #$00
		0x9A,						// TXS
		0x22, 0x00, 0x40, 0x05		// JSL $054000
	};
	W65Fixture f( code, sizeof( code ) );
	f.run( 2 );

	f.bus.clearLog();
	f.cpu.step();

	u32 addr = 0; u8 value = 0;
	CHECK_EQ( f.bus.writeCount(), 3 );
	CHECK( f.bus.nthWrite( 0, addr, value ) ); CHECK_EQ( addr, 0x000100 );
	CHECK( f.bus.nthWrite( 1, addr, value ) ); CHECK_EQ( addr, 0x0000FF );
	CHECK( f.bus.nthWrite( 2, addr, value ) ); CHECK_EQ( addr, 0x0000FE );
	CHECK_EQ( f.cpu.m_S, 0x01FD );
	CHECK_EQ( f.cpu.m_PBR, 0x05 );
	CHECK_EQ( f.cpu.m_PC, 0x4000 );
}

TEST( w65c816_jsl_rtl_roundtrip_across_banks )
{
	const u8 code[] = { 0x22, 0x00, 0x40, 0x05, 0xE8 };		// JSL $054000 : INX
	W65Fixture f( code, sizeof( code ) );

	s_Mem[ 0x054000 ] = 0x6B;		// RTL

	f.cpu.step();
	CHECK_EQ( f.cpu.m_PBR, 0x05 );
	CHECK_EQ( f.cpu.m_PC, 0x4000 );

	f.cpu.step();
	CHECK_EQ( f.cpu.m_PBR, 0x00 );
	CHECK_EQ( f.cpu.m_PC, 0x0204 );	// the instruction after the JSL

	f.cpu.step();
	CHECK_EQ( f.cpu.m_X, 1 );
}

TEST( w65c816_stack_relative_is_never_confined_to_page_one )
{
	// d,S is a "new" mode: with S=$0180 and an offset of $F0, the address is
	// $000270, well outside page 1.
	const u8 code[] = {
		0xA2, 0x80,				// LDX #$80
		0x9A,					// TXS      S = $0180
		0xA3, 0xF0				// LDA $F0,S
	};
	W65Fixture f( code, sizeof( code ) );
	f.run( 2 );

	s_Mem[ 0x000270 ] = 0x5A;
	f.bus.clearLog();
	f.cpu.step();

	CHECK_EQ( f.cpu.getA(), 0x5A );
	CHECK( f.bus.readAddress( 0x000270 ) );
}

// ---------------------------------------------------------------------------
// Read-modify-write
// ---------------------------------------------------------------------------

TEST( w65c816_emulation_rmw_emits_the_dummy_write )
{
	// The behaviour a C64 raster handler depends on. INC $D019 must write the
	// ORIGINAL byte first: bit 0 of $81 is set and clears the VIC-II latch,
	// while bit 0 of the incremented $82 is not.
	const u8 code[] = { 0xEE, 0x19, 0xD0 };		// INC $D019
	W65Fixture f( code, sizeof( code ) );

	s_Mem[ 0x00D019 ] = 0x81;
	f.cpu.step();

	u32 addr = 0; u8 value = 0;
	CHECK_EQ( f.bus.writeCount(), 2 );
	CHECK( f.bus.nthWrite( 0, addr, value ) );
	CHECK_EQ( addr, 0x00D019 );
	CHECK_EQ( value, 0x81 );		// the original, which acknowledges
	CHECK( f.bus.nthWrite( 1, addr, value ) );
	CHECK_EQ( value, 0x82 );
}

TEST( w65c816_native_rmw_has_no_dummy_write )
{
	// In native mode the chip runs an internal cycle instead. One write.
	const u8 code[] = { GO_NATIVE, 0xEE, 0x19, 0xD0 };
	W65Fixture f( code, sizeof( code ) );
	f.run( 2 );

	s_Mem[ 0x00D019 ] = 0x81;
	f.bus.clearLog();
	f.cpu.step();

	CHECK_EQ( f.bus.writeCount(), 1 );

	u32 addr = 0; u8 value = 0;
	CHECK( f.bus.nthWrite( 0, addr, value ) );
	CHECK_EQ( value, 0x82 );
}

TEST( w65c816_native_16bit_rmw_writes_the_high_byte_first )
{
	// Reads run low->high, writes run high->low. Only observable against I/O,
	// but that is exactly where it matters. 4971/4971 measured.
	const u8 code[] = { GO_NATIVE, WIDE_ALL, 0xEE, 0x00, 0x40 };	// INC $4000
	W65Fixture f( code, sizeof( code ) );
	f.run( 3 );

	s_Mem[ 0x004000 ] = 0xFF;
	s_Mem[ 0x004001 ] = 0x00;
	f.bus.clearLog();
	f.cpu.step();

	u32 addr = 0; u8 value = 0;
	CHECK_EQ( f.bus.writeCount(), 2 );
	CHECK( f.bus.nthWrite( 0, addr, value ) );
	CHECK_EQ( addr, 0x004001 );		// high byte first
	CHECK_EQ( value, 0x01 );
	CHECK( f.bus.nthWrite( 1, addr, value ) );
	CHECK_EQ( addr, 0x004000 );
	CHECK_EQ( value, 0x00 );
}

// ---------------------------------------------------------------------------
// Block moves
// ---------------------------------------------------------------------------

TEST( w65c816_mvn_operand_order_is_destination_then_source )
{
	// Object code $54 dd ss. The assembler mnemonic reads source-first, which
	// is the reverse -- so decoding by mnemonic order copies the wrong way.
	//
	// Move 4 bytes from bank $6D to bank $3D.
	const u8 code[] = {
		GO_NATIVE, WIDE_ALL,
		0xA2, 0x00, 0x10,		// LDX #$1000   source offset
		0xA0, 0x00, 0x20,		// LDY #$2000   destination offset
		0xA9, 0x03, 0x00,		// LDA #$0003   count-1, so four bytes
		0x54, 0x3D, 0x6D		// MVN  dd=$3D ss=$6D
	};
	W65Fixture f( code, sizeof( code ) );
	f.run( 6 );

	for ( int i = 0; i < 4; i++ ) s_Mem[ 0x6D1000 + i ] = (u8)( 0xA0 + i );

	// Four bytes means four executions -- the PC stays parked on the opcode.
	const u16 opcodePC = f.cpu.m_PC;
	for ( int i = 0; i < 3; i++ )
	{
		f.cpu.step();
		CHECK_EQ( f.cpu.m_PC, opcodePC );	// still parked: interruptible
	}
	f.cpu.step();
	CHECK_EQ( f.cpu.m_PC, opcodePC + 3 );	// and only now does it advance

	for ( int i = 0; i < 4; i++ )
		CHECK_EQ( s_Mem[ 0x3D2000 + i ], (u8)( 0xA0 + i ) );

	CHECK_EQ( f.cpu.m_DBR, 0x3D );		// DBR is left at the DESTINATION bank
	CHECK_EQ( f.cpu.m_C, 0xFFFF );
	CHECK_EQ( f.cpu.m_X, 0x1004 );		// one past the end
	CHECK_EQ( f.cpu.m_Y, 0x2004 );
}

TEST( w65c816_mvp_decrements )
{
	// $44 decrements, despite WDC calling it "Block Move Positive". Copy four
	// bytes ending at $6D1003 down to a block ending at $3D2003.
	const u8 code[] = {
		GO_NATIVE, WIDE_ALL,
		0xA2, 0x03, 0x10,		// LDX #$1003
		0xA0, 0x03, 0x20,		// LDY #$2003
		0xA9, 0x03, 0x00,		// LDA #$0003
		0x44, 0x3D, 0x6D		// MVP
	};
	W65Fixture f( code, sizeof( code ) );
	f.run( 6 );

	for ( int i = 0; i < 4; i++ ) s_Mem[ 0x6D1000 + i ] = (u8)( 0xA0 + i );

	for ( int i = 0; i < 4; i++ ) f.cpu.step();

	for ( int i = 0; i < 4; i++ )
		CHECK_EQ( s_Mem[ 0x3D2000 + i ], (u8)( 0xA0 + i ) );

	CHECK_EQ( f.cpu.m_X, 0x0FFF );		// one before the start
	CHECK_EQ( f.cpu.m_Y, 0x1FFF );
}

TEST( w65c816_mvn_moves_one_byte_when_c_is_zero )
{
	// C is the count MINUS ONE, so C=0 moves exactly one byte.
	const u8 code[] = {
		GO_NATIVE, WIDE_ALL,
		0xA2, 0x00, 0x10,
		0xA0, 0x00, 0x20,
		0xA9, 0x00, 0x00,		// LDA #$0000
		0x54, 0x3D, 0x6D
	};
	W65Fixture f( code, sizeof( code ) );
	f.run( 6 );

	s_Mem[ 0x6D1000 ] = 0x5A;
	s_Mem[ 0x6D1001 ] = 0xA5;

	const u16 opcodePC = f.cpu.m_PC;
	f.cpu.step();

	CHECK_EQ( f.cpu.m_PC, opcodePC + 3 );	// done after one byte
	CHECK_EQ( s_Mem[ 0x3D2000 ], 0x5A );
	CHECK_EQ( s_Mem[ 0x3D2001 ], 0x00 );
	CHECK_EQ( f.cpu.m_C, 0xFFFF );
}

// ---------------------------------------------------------------------------
// Interrupts
// ---------------------------------------------------------------------------

TEST( w65c816_native_interrupt_pushes_four_bytes_with_the_program_bank )
{
	const u8 code[] = { GO_NATIVE, 0x00, 0x00 };	// CLC XCE BRK
	W65Fixture f( code, sizeof( code ) );

	s_Mem[ 0xFFE6 ] = 0x00;		// native BRK vector -> $3000
	s_Mem[ 0xFFE7 ] = 0x30;

	f.run( 2 );
	f.cpu.m_PBR = 0x00;
	const u16 sBefore = f.cpu.m_S;
	f.bus.clearLog();
	f.cpu.step();

	CHECK_EQ( f.cpu.m_PC, 0x3000 );
	CHECK_EQ( f.cpu.m_PBR, 0x00 );
	CHECK_EQ( f.bus.writeCount(), 4 );		// PBR, PCH, PCL, P
	CHECK_EQ( sBefore - f.cpu.m_S, 4 );		// four bytes, not three

	// Push order: program bank first.
	u32 addr = 0; u8 value = 0;
	CHECK( f.bus.nthWrite( 0, addr, value ) );
	CHECK_EQ( addr, ( 0x000000 | sBefore ) );
	CHECK_EQ( value, 0x00 );				// the PBR
}

TEST( w65c816_native_brk_pushes_x_flag_not_a_break_bit )
{
	// In native mode bit 4 of the pushed P is the X flag, verbatim. There is no
	// B bit at all; BRK is distinguished from IRQ only by its vector, so a
	// native handler must not test bit 4.
	const u8 code[] = {
		GO_NATIVE,
		0xC2, 0x10,				// REP #$10   x = 0
		0x00, 0x00				// BRK
	};
	W65Fixture f( code, sizeof( code ) );

	s_Mem[ 0xFFE6 ] = 0x00;
	s_Mem[ 0xFFE7 ] = 0x30;

	f.run( 3 );
	const u16 sBefore = f.cpu.m_S;
	f.cpu.step();

	// The P byte is the last of the four pushed, so it sits at S+1.
	const u8 pushed = s_Mem[ ( sBefore - 3 ) & 0xFFFF ];
	CHECK( ( pushed & W65_X ) == 0 );		// x was 0 and is pushed as 0
}

TEST( w65c816_emulation_brk_pushes_the_break_bit )
{
	const u8 code[] = { 0x00, 0x00 };		// BRK
	W65Fixture f( code, sizeof( code ) );

	s_Mem[ 0xFFFE ] = 0x00;
	s_Mem[ 0xFFFF ] = 0x30;

	const u16 sBefore = f.cpu.m_S;
	f.cpu.step();

	CHECK_EQ( f.cpu.m_PC, 0x3000 );
	CHECK_EQ( sBefore - f.cpu.m_S, 3 );		// three bytes in emulation mode

	const u8 pushed = s_Mem[ ( sBefore - 2 ) & 0xFFFF ];
	CHECK( ( pushed & W65_B ) != 0 );

	// And the pushed PC is the address after the signature byte.
	const u16 pushedPC = (u16)( s_Mem[ ( sBefore - 1 ) & 0xFFFF ]
	                  | ( s_Mem[ sBefore & 0xFFFF ] << 8 ) );
	CHECK_EQ( pushedPC, 0x0202 );
}

TEST( w65c816_wai_resumes_without_the_handler_when_irq_is_masked )
{
	// The second exit, and the entire point of WAI: with I=1 and IRQ asserted,
	// execution continues at the instruction AFTER the WAI without going near
	// the handler.
	const u8 code[] = { 0x78, 0xCB, 0xE8 };		// SEI : WAI : INX
	W65Fixture f( code, sizeof( code ) );

	f.cpu.step();				// SEI
	f.cpu.step();				// WAI -- parks

	f.cpu.step();				// no IRQ yet: burns a cycle
	CHECK_EQ( f.cpu.m_X, 0 );

	f.bus.m_IRQ = true;
	f.cpu.step();				// wakes and runs the INX
	CHECK_EQ( f.cpu.m_X, 1 );
	CHECK_EQ( f.cpu.m_PC, 0x0203 );
}

TEST( w65c816_stp_stops_until_reset )
{
	const u8 code[] = { 0xDB, 0xE8 };		// STP : INX
	W65Fixture f( code, sizeof( code ) );

	f.cpu.step();
	CHECK( f.cpu.m_Stopped );

	// Neither IRQ nor NMI restarts it, and the bus keeps ticking so the C64
	// around us carries on.
	f.bus.m_IRQ = true;
	f.bus.m_NMI = true;
	const u64 ticksBefore = f.bus.m_Ticks;
	for ( int i = 0; i < 10; i++ ) f.cpu.step();
	CHECK_EQ( f.cpu.m_X, 0 );
	CHECK( f.bus.m_Ticks > ticksBefore );

	f.cpu.reset();
	CHECK( !f.cpu.m_Stopped );
}

// ---------------------------------------------------------------------------
// Odds and ends that are easy to get wrong
// ---------------------------------------------------------------------------

TEST( w65c816_transfer_group_flag_behaviour )
{
	// TCS sets NO flags. TSC, TCD and TDC set N and Z from all 16 bits, even in
	// emulation mode. This group is the most commonly mis-implemented on the
	// chip.
	const u8 code[] = {
		0xA9, 0x00,		// LDA #$00     A = 0
		0x1B,			// TCS          must not set Z
		0x3B,			// TSC          C = $01xx, so Z clear and N clear
		0xEB			// XBA          B should now hold $01
	};
	W65Fixture f( code, sizeof( code ) );

	f.run( 2 );

	// LDA #$00 set Z, and TCS sets no flags at all -- so Z must still be set.
	// A TCS that "helpfully" updated the flags would clear it, since S is now
	// $0100 rather than zero.
	CHECK( ( f.cpu.m_P & W65_Z ) != 0 );
	CHECK_EQ( f.cpu.m_S, 0x0100 );

	f.cpu.step();							// TSC
	CHECK_EQ( f.cpu.m_C, 0x0100 );
	CHECK( ( f.cpu.m_P & W65_Z ) == 0 );	// $0100 is not zero, over 16 bits
	CHECK( ( f.cpu.m_P & W65_N ) == 0 );

	f.cpu.step();							// XBA
	CHECK_EQ( f.cpu.getA(), 0x01 );			// B held $01 and is now A
	CHECK( ( f.cpu.m_P & W65_Z ) == 0 );
}

TEST( w65c816_xba_sets_flags_from_the_new_low_byte )
{
	// XBA ignores M, X and E entirely, and its flags always describe the new
	// 8-bit low byte -- even with a 16-bit accumulator.
	const u8 code[] = {
		GO_NATIVE, WIDE_ALL,
		0xA9, 0x80, 0x00,		// LDA #$0080   B=$00, A=$80
		0xEB					// XBA          -> B=$80, A=$00
	};
	W65Fixture f( code, sizeof( code ) );
	f.run( 5 );

	CHECK_EQ( f.cpu.m_C, 0x8000 );
	CHECK( ( f.cpu.m_P & W65_Z ) != 0 );	// from the new low byte, $00
	CHECK( ( f.cpu.m_P & W65_N ) == 0 );	// NOT from bit 15
}

TEST( w65c816_bit_immediate_only_sets_zero )
{
	// BIT #imm is the odd one out: N and V are left alone.
	const u8 code[] = {
		0xA9, 0x0F,		// LDA #$0F
		0x89, 0xC0,		// BIT #$C0     top bits set, but immediate
		0x24, 0x10		// BIT $10      same operand from memory
	};
	W65Fixture f( code, sizeof( code ) );
	s_Mem[ 0x000010 ] = 0xC0;

	f.run( 2 );
	CHECK( ( f.cpu.m_P & W65_Z ) != 0 );	// $0F & $C0 == 0
	CHECK( ( f.cpu.m_P & W65_N ) == 0 );	// untouched
	CHECK( ( f.cpu.m_P & W65_V ) == 0 );	// untouched

	f.cpu.step();
	CHECK( ( f.cpu.m_P & W65_N ) != 0 );	// now copied from the operand
	CHECK( ( f.cpu.m_P & W65_V ) != 0 );
}

TEST( w65c816_jmp_indirect_x_takes_its_pointer_from_the_program_bank )
{
	// JMP (a) reads its pointer from bank 0; JMP (a,X) reads its from PBR. The
	// two sit next to each other in the opcode map and go opposite ways.
	const u8 code[] = { 0xA2, 0x02, 0x7C, 0x00, 0x30 };	// LDX #$02 : JMP ($3000,X)
	W65Fixture f( code, sizeof( code ), 0x050200 );

	f.cpu.m_PBR = 0x05;
	f.cpu.m_PC  = 0x0200;

	s_Mem[ 0x053002 ] = 0x00;		// pointer in bank 5, where PBR points
	s_Mem[ 0x053003 ] = 0x40;
	s_Mem[ 0x003002 ] = 0xEE;		// a decoy in bank 0
	s_Mem[ 0x003003 ] = 0xEE;

	f.run( 2 );
	CHECK_EQ( f.cpu.m_PC, 0x4000 );
	CHECK_EQ( f.cpu.m_PBR, 0x05 );	// JMP (a,X) does not change the bank
}

TEST( w65c816_16bit_decimal_subtract )
{
	// Bruce Clark's worked example: A=$0001, m=0, d=1, c=1, SBC #$2003 gives
	// A=$7998 with n=0, z=0, c=0. Four-digit BCD, and the carry reports the
	// result leaving the range 0..9999.
	const u8 code[] = {
		GO_NATIVE, WIDE_ALL,
		0xF8,					// SED
		0x38,					// SEC
		0xA9, 0x01, 0x00,		// LDA #$0001
		0xE9, 0x03, 0x20		// SBC #$2003
	};
	W65Fixture f( code, sizeof( code ) );
	f.run( 7 );

	CHECK_EQ( f.cpu.m_C, 0x7998 );
	CHECK( ( f.cpu.m_P & W65_N ) == 0 );
	CHECK( ( f.cpu.m_P & W65_Z ) == 0 );
	CHECK( ( f.cpu.m_P & W65_C ) == 0 );
}

TEST( w65c816_16bit_decimal_add )
{
	// $9999 + $0001 in four-digit BCD wraps to $0000 with carry out.
	const u8 code[] = {
		GO_NATIVE, WIDE_ALL,
		0xF8, 0x18,				// SED CLC
		0xA9, 0x99, 0x99,		// LDA #$9999
		0x69, 0x01, 0x00		// ADC #$0001
	};
	W65Fixture f( code, sizeof( code ) );
	f.run( 7 );

	CHECK_EQ( f.cpu.m_C, 0x0000 );
	CHECK( ( f.cpu.m_P & W65_C ) != 0 );
	CHECK( ( f.cpu.m_P & W65_Z ) != 0 );
	CHECK( ( f.cpu.m_P & W65_N ) == 0 );
}

TEST( w65c816_wdm_is_two_bytes )
{
	// Treating $42 as one byte desynchronises the instruction stream.
	const u8 code[] = { 0x42, 0xFF, 0xE8 };		// WDM $FF : INX
	W65Fixture f( code, sizeof( code ) );

	f.cpu.step();
	CHECK_EQ( f.cpu.m_PC, 0x0202 );

	f.cpu.step();
	CHECK_EQ( f.cpu.m_X, 1 );
}

TEST( w65c816_immediate_width_follows_m_and_x )
{
	// After REP #$20 an LDA # takes a 16-bit operand and is three bytes long.
	// This is why REP and SEP change the length of already-assembled code.
	CHECK_EQ( w65c816Length( 0xA9, true,  true  ), 2 );
	CHECK_EQ( w65c816Length( 0xA9, false, true  ), 3 );
	CHECK_EQ( w65c816Length( 0xA2, true,  true  ), 2 );
	CHECK_EQ( w65c816Length( 0xA2, true,  false ), 3 );

	// ...but REP, SEP, BRK, COP and WDM are always two bytes.
	CHECK_EQ( w65c816Length( 0xC2, false, false ), 2 );
	CHECK_EQ( w65c816Length( 0xE2, false, false ), 2 );
	CHECK_EQ( w65c816Length( 0x00, false, false ), 2 );
	CHECK_EQ( w65c816Length( 0x42, false, false ), 2 );

	// And JSL is four bytes where the 6502 table calls $22 a one-byte JAM.
	CHECK_EQ( w65c816Length( 0x22, true, true ), 4 );
	CHECK_EQ( w65c816Length( 0x0B, true, true ), 1 );	// PHD, one byte
}

TEST( w65c816_every_opcode_is_defined )
{
	// All 256 encodings are real instructions. A blank row would mean the table
	// is incomplete rather than that the program did something odd.
	for ( u32 op = 0; op < 256; op++ )
	{
		CHECK( w65c816Opcodes[ op ].mnemonic != 0 );
		CHECK( w65c816Opcodes[ op ].mnemonic[ 0 ] != '\0' );
		CHECK( w65c816Opcodes[ op ].baseCycles >= 1 );
		CHECK( w65c816Opcodes[ op ].baseCycles <= 8 );
	}
}

TEST( w65c816_dispatch_consumes_exactly_the_table_length )
{
	// Every opcode must advance the PC by exactly what the table says. This is
	// the check that catches an instruction-stream desync at its source rather
	// than as corruption somewhere else entirely.
	//
	// Control-flow instructions move the PC for their own reasons and are
	// checked elsewhere; block moves park it deliberately.
	for ( u32 op = 0; op < 256; op++ )
	{
		const char *m = w65c816Opcodes[ op ].mnemonic;

		// Anything that legitimately redirects execution.
		if ( std::strcmp( m, "JMP" ) == 0 || std::strcmp( m, "JML" ) == 0
		  || std::strcmp( m, "JSR" ) == 0 || std::strcmp( m, "JSL" ) == 0
		  || std::strcmp( m, "RTS" ) == 0 || std::strcmp( m, "RTL" ) == 0
		  || std::strcmp( m, "RTI" ) == 0 || std::strcmp( m, "BRK" ) == 0
		  || std::strcmp( m, "COP" ) == 0 || std::strcmp( m, "MVN" ) == 0
		  || std::strcmp( m, "MVP" ) == 0 || std::strcmp( m, "STP" ) == 0
		  || std::strcmp( m, "WAI" ) == 0 || std::strcmp( m, "BRL" ) == 0
		  || m[ 0 ] == 'B' )		// every branch
			continue;

		// XCE and the width instructions change the widths mid-instruction, but
		// their own length is fixed, so they are fine to include.
		const u8 code[] = { (u8)op, 0x00, 0x00, 0x00, 0xEA };
		W65Fixture f( code, sizeof( code ) );

		const u16 before = f.cpu.m_PC;
		f.cpu.step();
		const u16 advanced = (u16)( f.cpu.m_PC - before );
		const u8  expected = w65c816Length( (u8)op, true, true );

		if ( advanced != expected )
		{
			std::printf( "\n  $%02X %s advanced %u, table says %u",
			             (unsigned)op, m, advanced, expected );
			CHECK( false );
			return;
		}
	}

	CHECK( true );
}
