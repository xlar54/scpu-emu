/*
   SCPU-EMU - MOS 6502 core tests.

   These pin down the behaviours the C64 KERNAL actually depends on and that are
   easy to get subtly wrong: NMOS decimal arithmetic, the indirect-JMP page-wrap
   bug, page-cross cycle penalties, and the exact stack layout of BRK vs IRQ.
*/
#include "../test_framework.h"
#include "../../Source/CPU/M6502/m6502.h"
#include "../../Source/CPU/M6502/m6502_opcodes.h"

// Flat 64K of RAM, nothing else.
class CTestBus : public ICpuBus
{
public:
	u8   m_Mem[ 0x10000 ];
	bool m_IRQ, m_NMI;
	u64  m_Ticks;

	CTestBus() : m_IRQ( false ), m_NMI( false ), m_Ticks( 0 )
	{
		std::memset( m_Mem, 0, sizeof( m_Mem ) );
	}

	u8   read8( scpu_addr_t a ) override            { return m_Mem[ a & 0xFFFF ]; }
	void write8( scpu_addr_t a, u8 v ) override     { m_Mem[ a & 0xFFFF ] = v; }
	bool irqAsserted() override                     { return m_IRQ; }
	bool nmiAsserted() override                     { return m_NMI; }
	void tick( u32 n ) override                     { m_Ticks += n; }

	void load( u16 addr, const u8 *code, u32 len )
	{
		std::memcpy( &m_Mem[ addr ], code, len );
	}
	void setResetVector( u16 v )
	{
		m_Mem[ 0xFFFC ] = (u8)( v & 0xFF );
		m_Mem[ 0xFFFD ] = (u8)( v >> 8 );
	}
};

struct Fixture
{
	CTestBus bus;
	CM6502   cpu;

	Fixture( const u8 *code, u32 len, u16 org = 0x0200 )
	{
		bus.load( org, code, len );
		bus.setResetVector( org );
		cpu.attachBus( &bus );
		cpu.reset();
	}
};

// ---------------------------------------------------------------------------

TEST( m6502_reset_reads_vector )
{
	const u8 code[] = { 0xEA };
	Fixture f( code, sizeof( code ), 0x1234 );
	CHECK_EQ( f.cpu.m_PC, 0x1234 );
	CHECK_EQ( f.cpu.m_S, 0xFD );
	CHECK( ( f.cpu.m_P & M6502_I ) != 0 );
}

TEST( m6502_lda_sets_zero_and_negative )
{
	const u8 code[] = { 0xA9, 0x00,		// LDA #$00
	                    0xA9, 0x80,		// LDA #$80
	                    0xA9, 0x01 };	// LDA #$01
	Fixture f( code, sizeof( code ) );

	f.cpu.step();
	CHECK( ( f.cpu.m_P & M6502_Z ) != 0 );
	CHECK( ( f.cpu.m_P & M6502_N ) == 0 );

	f.cpu.step();
	CHECK( ( f.cpu.m_P & M6502_Z ) == 0 );
	CHECK( ( f.cpu.m_P & M6502_N ) != 0 );

	f.cpu.step();
	CHECK( ( f.cpu.m_P & M6502_Z ) == 0 );
	CHECK( ( f.cpu.m_P & M6502_N ) == 0 );
	CHECK_EQ( f.cpu.m_A, 0x01 );
}

TEST( m6502_adc_binary_overflow )
{
	// $50 + $50 = $A0: no carry out, but signed overflow.
	const u8 code[] = { 0x18, 0xA9, 0x50, 0x69, 0x50 };	// CLC / LDA #$50 / ADC #$50
	Fixture f( code, sizeof( code ) );

	f.cpu.step(); f.cpu.step(); f.cpu.step();

	CHECK_EQ( f.cpu.m_A, 0xA0 );
	CHECK( ( f.cpu.m_P & M6502_V ) != 0 );
	CHECK( ( f.cpu.m_P & M6502_C ) == 0 );
	CHECK( ( f.cpu.m_P & M6502_N ) != 0 );
}

TEST( m6502_adc_decimal )
{
	// BCD 09 + 01 = 10.
	const u8 code[] = { 0xF8, 0x18, 0xA9, 0x09, 0x69, 0x01 };	// SED/CLC/LDA #$09/ADC #$01
	Fixture f( code, sizeof( code ) );

	for ( int i = 0; i < 4; i++ ) f.cpu.step();

	CHECK_EQ( f.cpu.m_A, 0x10 );
	CHECK( ( f.cpu.m_P & M6502_C ) == 0 );
}

TEST( m6502_adc_decimal_carry_out )
{
	// BCD 99 + 01 = 00 with carry.
	const u8 code[] = { 0xF8, 0x18, 0xA9, 0x99, 0x69, 0x01 };
	Fixture f( code, sizeof( code ) );

	for ( int i = 0; i < 4; i++ ) f.cpu.step();

	CHECK_EQ( f.cpu.m_A, 0x00 );
	CHECK( ( f.cpu.m_P & M6502_C ) != 0 );
}

TEST( m6502_sbc_decimal )
{
	// BCD 10 - 01 = 09.
	const u8 code[] = { 0xF8, 0x38, 0xA9, 0x10, 0xE9, 0x01 };	// SED/SEC/LDA #$10/SBC #$01
	Fixture f( code, sizeof( code ) );

	for ( int i = 0; i < 4; i++ ) f.cpu.step();

	CHECK_EQ( f.cpu.m_A, 0x09 );
	CHECK( ( f.cpu.m_P & M6502_C ) != 0 );	// no borrow
}

TEST( m6502_jmp_indirect_page_wrap_bug )
{
	// The vector at $30FF must take its high byte from $3000, not $3100.
	const u8 code[] = { 0x6C, 0xFF, 0x30 };	// JMP ($30FF)
	Fixture f( code, sizeof( code ) );

	f.bus.m_Mem[ 0x30FF ] = 0x34;
	f.bus.m_Mem[ 0x3100 ] = 0xAA;	// what a fixed CPU would use
	f.bus.m_Mem[ 0x3000 ] = 0x12;	// what the NMOS 6502 actually uses

	f.cpu.step();
	CHECK_EQ( f.cpu.m_PC, 0x1234 );
}

TEST( m6502_branch_cycle_penalties )
{
	// BNE taken within the same page: 3 cycles.
	{
		const u8 code[] = { 0xA9, 0x01, 0xD0, 0x02 };	// LDA #$01 / BNE +2
		Fixture f( code, sizeof( code ), 0x0200 );
		f.cpu.step();
		CHECK_EQ( f.cpu.step(), 3 );
	}

	// BNE taken across a page boundary: 4 cycles.
	{
		const u8 code[] = { 0xA9, 0x01, 0xD0, 0x7F };	// LDA #$01 / BNE +127
		Fixture f( code, sizeof( code ), 0x02F0 );
		f.cpu.step();
		CHECK_EQ( f.cpu.step(), 4 );
	}

	// Not taken: 2 cycles.
	{
		const u8 code[] = { 0xA9, 0x00, 0xD0, 0x02 };	// LDA #$00 / BNE +2
		Fixture f( code, sizeof( code ), 0x0200 );
		f.cpu.step();
		CHECK_EQ( f.cpu.step(), 2 );
	}
}

TEST( m6502_indexed_read_page_cross_penalty )
{
	// LDA $12FF,X with X=1 crosses into $1300: 5 cycles instead of 4.
	const u8 code[] = { 0xA2, 0x01, 0xBD, 0xFF, 0x12 };	// LDX #$01 / LDA $12FF,X
	Fixture f( code, sizeof( code ) );
	f.cpu.step();
	CHECK_EQ( f.cpu.step(), 5 );

	// STA never gets the penalty: always 5.
	const u8 code2[] = { 0xA2, 0x01, 0x9D, 0xFF, 0x12 };	// LDX #$01 / STA $12FF,X
	Fixture g( code2, sizeof( code2 ) );
	g.cpu.step();
	CHECK_EQ( g.cpu.step(), 5 );
}

TEST( m6502_jsr_rts_roundtrip )
{
	const u8 code[] = { 0x20, 0x10, 0x02 };	// $0200: JSR $0210
	const u8 sub[]  = { 0x60 };				// $0210: RTS
	Fixture f( code, sizeof( code ) );
	f.bus.load( 0x0210, sub, sizeof( sub ) );

	f.cpu.step();
	CHECK_EQ( f.cpu.m_PC, 0x0210 );
	CHECK_EQ( f.cpu.m_S, 0xFB );
	// JSR pushes the address of its own last byte.
	CHECK_EQ( f.bus.m_Mem[ 0x01FD ], 0x02 );
	CHECK_EQ( f.bus.m_Mem[ 0x01FC ], 0x02 );

	f.cpu.step();
	CHECK_EQ( f.cpu.m_PC, 0x0203 );
	CHECK_EQ( f.cpu.m_S, 0xFD );
}

TEST( m6502_brk_sets_b_on_stack_irq_does_not )
{
	// BRK: pushes PC+2 and a status byte with B set.
	{
		const u8 code[] = { 0x00, 0xEA };	// BRK / padding
		Fixture f( code, sizeof( code ) );
		f.bus.m_Mem[ 0xFFFE ] = 0x00;
		f.bus.m_Mem[ 0xFFFF ] = 0x30;

		f.cpu.step();
		CHECK_EQ( f.cpu.m_PC, 0x3000 );
		CHECK_EQ( f.bus.m_Mem[ 0x01FD ], 0x02 );	// PC high
		CHECK_EQ( f.bus.m_Mem[ 0x01FC ], 0x02 );	// PC low = $0202
		CHECK( ( f.bus.m_Mem[ 0x01FB ] & M6502_B ) != 0 );
		CHECK( ( f.cpu.m_P & M6502_I ) != 0 );
	}

	// A hardware IRQ pushes the same layout but with B clear.
	{
		const u8 code[] = { 0xEA };
		Fixture f( code, sizeof( code ) );
		f.bus.m_Mem[ 0xFFFE ] = 0x00;
		f.bus.m_Mem[ 0xFFFF ] = 0x40;

		f.cpu.m_P &= (u8)~M6502_I;	// allow interrupts
		f.bus.m_IRQ = true;

		u32 c = f.cpu.step();
		CHECK_EQ( c, 7 );
		CHECK_EQ( f.cpu.m_PC, 0x4000 );
		CHECK( ( f.bus.m_Mem[ 0x01FB ] & M6502_B ) == 0 );
	}
}

TEST( m6502_irq_masked_by_i_flag )
{
	const u8 code[] = { 0xEA, 0xEA };
	Fixture f( code, sizeof( code ) );
	f.bus.m_IRQ = true;			// I is set after reset

	f.cpu.step();
	CHECK_EQ( f.cpu.m_PC, 0x0201 );	// executed the NOP, no vector taken
}

TEST( m6502_nmi_is_edge_triggered )
{
	const u8 code[] = { 0xEA, 0xEA, 0xEA, 0xEA };
	Fixture f( code, sizeof( code ) );
	f.bus.m_Mem[ 0xFFFA ] = 0x00;
	f.bus.m_Mem[ 0xFFFB ] = 0x50;

	f.bus.m_NMI = true;
	f.cpu.step();
	CHECK_EQ( f.cpu.m_PC, 0x5000 );

	// Holding the line low must not re-trigger.
	f.bus.m_Mem[ 0x5000 ] = 0xEA;
	f.cpu.step();
	CHECK_EQ( f.cpu.m_PC, 0x5001 );
}

TEST( m6502_undocumented_opcodes_are_nops_of_correct_length )
{
	// $80 is a 2-byte undocumented NOP; the operand must be skipped, not run.
	const u8 code[] = { 0x80, 0xA9, 0xA9, 0x42 };	// NOP* #$A9 / LDA #$42
	Fixture f( code, sizeof( code ) );

	f.cpu.step();
	CHECK_EQ( f.cpu.m_PC, 0x0202 );

	f.cpu.step();
	CHECK_EQ( f.cpu.m_A, 0x42 );
}

TEST( m6502_jam_halts_the_core )
{
	const u8 code[] = { 0x02 };
	Fixture f( code, sizeof( code ) );

	f.cpu.step();
	CHECK( f.cpu.m_Jammed );
	CHECK_EQ( f.cpu.m_PC, 0x0200 );	// parked on the offending opcode
}

TEST( m6502_opcode_table_lengths_agree_with_decoder )
{
	// Every entry must have a length the disassembler can render, and the
	// documented set must cover exactly 151 encodings.
	int documented = 0;
	for ( int i = 0; i < 256; i++ )
	{
		const M6502OpcodeInfo *info = &m6502Opcodes[ i ];
		CHECK( info->mode < AM_MODES );
		CHECK( m6502ModeLength[ info->mode ] >= 1 && m6502ModeLength[ info->mode ] <= 3 );
		if ( !info->undocumented )
			documented++;
	}
	CHECK_EQ( documented, 151 );
}

TEST( m6502_rmw_performs_the_nmos_dummy_write )
{
	// An NMOS 6502 writes the original byte back before the modified one.
	// Programs depend on it against registers with write side effects; the
	// canonical case is INC $D019 acknowledging a VIC-II interrupt, where the
	// first write is what performs the acknowledge.
	const u8 code[] = { 0xEE, 0x00, 0x30 };		// INC $3000
	Fixture f( code, sizeof( code ) );
	f.bus.m_Mem[ 0x3000 ] = 0x41;

	// Record the write sequence by replaying it through a tiny shim.
	struct LogBus : ICpuBus
	{
		u8 mem[ 0x10000 ];
		u8 writes[ 8 ]; u32 n;
		LogBus() : n( 0 ) { std::memset( mem, 0, sizeof( mem ) ); }
		u8   read8( scpu_addr_t a ) override        { return mem[ a & 0xFFFF ]; }
		void write8( scpu_addr_t a, u8 v ) override { mem[ a & 0xFFFF ] = v; if ( n < 8 ) writes[ n++ ] = v; }
		bool irqAsserted() override { return false; }
		bool nmiAsserted() override { return false; }
	};

	LogBus lb;
	std::memcpy( &lb.mem[ 0x0200 ], code, sizeof( code ) );
	lb.mem[ 0xFFFC ] = 0x00; lb.mem[ 0xFFFD ] = 0x02;
	lb.mem[ 0x3000 ] = 0x41;

	CM6502 cpu;
	cpu.attachBus( &lb );
	cpu.reset();
	cpu.step();

	CHECK_EQ( lb.n, 2 );
	CHECK_EQ( lb.writes[ 0 ], 0x41 );		// original value written back first
	CHECK_EQ( lb.writes[ 1 ], 0x42 );		// then the incremented value
	CHECK_EQ( lb.mem[ 0x3000 ], 0x42 );

	// The dummy write must not change the cycle count -- it happens inside the
	// six cycles an absolute RMW already takes.
	Fixture g( code, sizeof( code ) );
	g.bus.m_Mem[ 0x3000 ] = 0x41;
	CHECK_EQ( g.cpu.step(), 6 );
}

TEST( m6502_rmw_dummy_write_can_be_suppressed_for_65816 )
{
	// A 65C816 runs an internal cycle where the NMOS part emits the dummy
	// write. CSuperCPU turns this off, because the accelerator is a 65816 and
	// the extra write would cost a real bus cycle against I/O.
	struct LogBus : ICpuBus
	{
		u8 mem[ 0x10000 ];
		u32 n;
		LogBus() : n( 0 ) { std::memset( mem, 0, sizeof( mem ) ); }
		u8   read8( scpu_addr_t a ) override        { return mem[ a & 0xFFFF ]; }
		void write8( scpu_addr_t a, u8 v ) override { mem[ a & 0xFFFF ] = v; n++; }
		bool irqAsserted() override { return false; }
		bool nmiAsserted() override { return false; }
	};

	const u8 code[] = { 0xEE, 0x00, 0x30 };		// INC $3000
	LogBus lb;
	std::memcpy( &lb.mem[ 0x0200 ], code, sizeof( code ) );
	lb.mem[ 0xFFFC ] = 0x00; lb.mem[ 0xFFFD ] = 0x02;
	lb.mem[ 0x3000 ] = 0x41;

	CM6502 cpu;
	cpu.attachBus( &lb );
	cpu.reset();
	cpu.m_RMWDummyWrite = false;
	cpu.step();

	CHECK_EQ( lb.n, 1 );					// one write, not two
	CHECK_EQ( lb.mem[ 0x3000 ], 0x42 );
}
