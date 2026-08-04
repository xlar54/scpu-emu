/*
   SCPU-EMU - CMD SuperCPU emulation for the C64/C128 using a RAD Expansion Unit
   Copyright (c) 2026 SCPU-EMU contributors

   MOS 6502 / 6510 interpreter.

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
#include "m6502.h"
#include "m6502_opcodes.h"

CM6502::CM6502()
	: m_A( 0 ), m_X( 0 ), m_Y( 0 ), m_S( 0xFD ), m_P( M6502_U | M6502_I ),
	  m_PC( 0 ), m_Jammed( false ), m_RMWDummyWrite( true ),
	  m_Bus( 0 ), m_Cycles( 0 ), m_RunBreak( false ),
	  m_NMIPrevLevel( false ), m_NMIPending( false )
{
}

void CM6502::reset()
{
	m_A = m_X = m_Y = 0;
	m_S = 0xFD;
	m_P = M6502_U | M6502_I;
	m_Jammed = false;
	m_NMIPrevLevel = false;
	m_NMIPending = false;
	m_RunBreak = false;
	m_Cycles = 0;
	m_PC = rd16( M6502_VEC_RESET );
}

// ---------------------------------------------------------------------------
// ALU
// ---------------------------------------------------------------------------

// NMOS decimal-mode semantics: Z comes from the binary result, N and V are
// sampled from the high nibble *before* the final decimal fixup. This is the
// behaviour the C64 KERNAL and BASIC float routines rely on.
void CM6502::opADC( u8 v )
{
	u16 c = ( m_P & M6502_C ) ? 1 : 0;

	if ( m_P & M6502_D )
	{
		u16 lo = (u16)( m_A & 0x0F ) + (u16)( v & 0x0F ) + c;
		u16 hi = (u16)( m_A & 0xF0 ) + (u16)( v & 0xF0 );

		setFlag( M6502_Z, (u8)( m_A + v + c ) == 0 );

		if ( lo > 0x09 ) { hi += 0x10; lo += 0x06; }

		setFlag( M6502_N, ( hi & 0x80 ) != 0 );
		setFlag( M6502_V, ( ( ~( m_A ^ v ) & ( m_A ^ (u8)hi ) ) & 0x80 ) != 0 );

		if ( hi > 0x90 ) hi += 0x60;

		setFlag( M6502_C, ( hi & 0xFF00 ) != 0 );
		m_A = (u8)( ( lo & 0x0F ) | ( hi & 0xF0 ) );
	} else
	{
		u16 r = (u16)m_A + (u16)v + c;
		setFlag( M6502_C, r > 0xFF );
		setFlag( M6502_V, ( ( ~( m_A ^ v ) & ( m_A ^ (u8)r ) ) & 0x80 ) != 0 );
		m_A = (u8)r;
		setZN( m_A );
	}
}

void CM6502::opSBC( u8 v )
{
	u16 c = ( m_P & M6502_C ) ? 1 : 0;
	u16 r = (u16)m_A - (u16)v - ( 1 - c );

	setFlag( M6502_C, r < 0x100 );
	setFlag( M6502_V, ( ( m_A ^ v ) & ( m_A ^ (u8)r ) & 0x80 ) != 0 );

	if ( m_P & M6502_D )
	{
		u16 lo = (u16)( m_A & 0x0F ) - (u16)( v & 0x0F ) - ( 1 - c );
		u16 hi = (u16)( m_A & 0xF0 ) - (u16)( v & 0xF0 );

		if ( lo & 0x10 ) { lo -= 0x06; hi -= 0x10; }
		if ( hi & 0x0100 ) hi -= 0x60;

		m_A = (u8)( ( lo & 0x0F ) | ( hi & 0xF0 ) );
	} else
	{
		m_A = (u8)r;
	}

	setZN( (u8)r );
}

void CM6502::opCMP( u8 reg, u8 v )
{
	u16 r = (u16)reg - (u16)v;
	setFlag( M6502_C, reg >= v );
	setZN( (u8)r );
}

// ---------------------------------------------------------------------------
// Interrupts
// ---------------------------------------------------------------------------

u32 CM6502::serviceNMI()
{
	push16( m_PC );
	push( (u8)( ( m_P | M6502_U ) & ~M6502_B ) );
	m_P |= M6502_I;
	m_PC = rd16( M6502_VEC_NMI );
	return 7;
}

u32 CM6502::serviceIRQ()
{
	push16( m_PC );
	push( (u8)( ( m_P | M6502_U ) & ~M6502_B ) );
	m_P |= M6502_I;
	m_PC = rd16( M6502_VEC_IRQ );
	return 7;
}

// ---------------------------------------------------------------------------
// Interpreter
// ---------------------------------------------------------------------------

// Effective-address helpers. The _R variants add the extra cycle taken when an
// indexed read crosses a page; the _W variants never do, because writes and
// read-modify-writes always perform the dummy read and so cost a fixed count.
#define EA_ZP()    ea = fetch()
#define EA_ZPX()   ea = (u8)( fetch() + m_X )
#define EA_ZPY()   ea = (u8)( fetch() + m_Y )
#define EA_ABS()   ea = fetch16()
#define EA_ABX_R() { u16 b = fetch16(); ea = (u16)( b + m_X ); cyc += pageCrossed( b, ea ); }
#define EA_ABX_W() { ea = (u16)( fetch16() + m_X ); }
#define EA_ABY_R() { u16 b = fetch16(); ea = (u16)( b + m_Y ); cyc += pageCrossed( b, ea ); }
#define EA_ABY_W() { ea = (u16)( fetch16() + m_Y ); }
#define EA_IZX()   { u8 z = (u8)( fetch() + m_X ); ea = rd( z ) | ( (u16)rd( (u8)( z + 1 ) ) << 8 ); }
#define EA_IZY_R() { u8 z = fetch(); u16 b = rd( z ) | ( (u16)rd( (u8)( z + 1 ) ) << 8 ); \
                     ea = (u16)( b + m_Y ); cyc += pageCrossed( b, ea ); }
#define EA_IZY_W() { u8 z = fetch(); u16 b = rd( z ) | ( (u16)rd( (u8)( z + 1 ) ) << 8 ); \
                     ea = (u16)( b + m_Y ); }

// NMOS read-modify-write: the original byte is written back before the
// modified one. See the note on m_RMWDummyWrite in m6502.h.
#define RMW_DUMMY_WRITE( ea, v ) { if ( m_RMWDummyWrite ) wr( ea, v ); }

#define BRANCH( cond )                                            \
	{                                                             \
		s8 disp = (s8)fetch();                                    \
		if ( cond )                                               \
		{                                                         \
			u16 target = (u16)( m_PC + disp );                    \
			cyc += 1 + pageCrossed( m_PC, target );               \
			m_PC = target;                                        \
		}                                                         \
	}

u32 CM6502::step()
{
	if ( m_Jammed )
	{
		// Account for the cycle we report, so cycles() and the value returned
		// from step() cannot disagree.
		m_Cycles += 1;
		m_Bus->tick( 1 );
		return 1;
	}

	// NMI is edge triggered, IRQ is level triggered.
	bool nmiLevel = m_Bus->nmiAsserted();
	if ( nmiLevel && !m_NMIPrevLevel )
		m_NMIPending = true;
	m_NMIPrevLevel = nmiLevel;

	if ( m_NMIPending )
	{
		m_NMIPending = false;
		u32 c = serviceNMI();
		m_Cycles += c;
		m_Bus->tick( c );
		return c;
	}

	if ( !( m_P & M6502_I ) && m_Bus->irqAsserted() )
	{
		u32 c = serviceIRQ();
		m_Cycles += c;
		m_Bus->tick( c );
		return c;
	}

	u8  opcode = fetch();
	u32 cyc    = m6502Opcodes[ opcode ].cycles;
	u16 ea     = 0;
	u8  v      = 0;

	switch ( opcode )
	{
	// --- load ------------------------------------------------------------
	case 0xA9: m_A = fetch();          setZN( m_A ); break;					// LDA #
	case 0xA5: EA_ZP();    m_A = rd( ea ); setZN( m_A ); break;				// LDA zp
	case 0xB5: EA_ZPX();   m_A = rd( ea ); setZN( m_A ); break;				// LDA zp,X
	case 0xAD: EA_ABS();   m_A = rd( ea ); setZN( m_A ); break;				// LDA abs
	case 0xBD: EA_ABX_R(); m_A = rd( ea ); setZN( m_A ); break;				// LDA abs,X
	case 0xB9: EA_ABY_R(); m_A = rd( ea ); setZN( m_A ); break;				// LDA abs,Y
	case 0xA1: EA_IZX();   m_A = rd( ea ); setZN( m_A ); break;				// LDA (zp,X)
	case 0xB1: EA_IZY_R(); m_A = rd( ea ); setZN( m_A ); break;				// LDA (zp),Y

	case 0xA2: m_X = fetch();          setZN( m_X ); break;					// LDX #
	case 0xA6: EA_ZP();    m_X = rd( ea ); setZN( m_X ); break;				// LDX zp
	case 0xB6: EA_ZPY();   m_X = rd( ea ); setZN( m_X ); break;				// LDX zp,Y
	case 0xAE: EA_ABS();   m_X = rd( ea ); setZN( m_X ); break;				// LDX abs
	case 0xBE: EA_ABY_R(); m_X = rd( ea ); setZN( m_X ); break;				// LDX abs,Y

	case 0xA0: m_Y = fetch();          setZN( m_Y ); break;					// LDY #
	case 0xA4: EA_ZP();    m_Y = rd( ea ); setZN( m_Y ); break;				// LDY zp
	case 0xB4: EA_ZPX();   m_Y = rd( ea ); setZN( m_Y ); break;				// LDY zp,X
	case 0xAC: EA_ABS();   m_Y = rd( ea ); setZN( m_Y ); break;				// LDY abs
	case 0xBC: EA_ABX_R(); m_Y = rd( ea ); setZN( m_Y ); break;				// LDY abs,X

	// --- store -----------------------------------------------------------
	case 0x85: EA_ZP();    wr( ea, m_A ); break;							// STA zp
	case 0x95: EA_ZPX();   wr( ea, m_A ); break;							// STA zp,X
	case 0x8D: EA_ABS();   wr( ea, m_A ); break;							// STA abs
	case 0x9D: EA_ABX_W(); wr( ea, m_A ); break;							// STA abs,X
	case 0x99: EA_ABY_W(); wr( ea, m_A ); break;							// STA abs,Y
	case 0x81: EA_IZX();   wr( ea, m_A ); break;							// STA (zp,X)
	case 0x91: EA_IZY_W(); wr( ea, m_A ); break;							// STA (zp),Y

	case 0x86: EA_ZP();    wr( ea, m_X ); break;							// STX zp
	case 0x96: EA_ZPY();   wr( ea, m_X ); break;							// STX zp,Y
	case 0x8E: EA_ABS();   wr( ea, m_X ); break;							// STX abs

	case 0x84: EA_ZP();    wr( ea, m_Y ); break;							// STY zp
	case 0x94: EA_ZPX();   wr( ea, m_Y ); break;							// STY zp,X
	case 0x8C: EA_ABS();   wr( ea, m_Y ); break;							// STY abs

	// --- transfer --------------------------------------------------------
	case 0xAA: m_X = m_A; setZN( m_X ); break;								// TAX
	case 0xA8: m_Y = m_A; setZN( m_Y ); break;								// TAY
	case 0xBA: m_X = m_S; setZN( m_X ); break;								// TSX
	case 0x8A: m_A = m_X; setZN( m_A ); break;								// TXA
	case 0x9A: m_S = m_X;               break;								// TXS (no flags)
	case 0x98: m_A = m_Y; setZN( m_A ); break;								// TYA

	// --- stack -----------------------------------------------------------
	case 0x48: push( m_A ); break;											// PHA
	case 0x08: push( (u8)( m_P | M6502_B | M6502_U ) ); break;				// PHP
	case 0x68: m_A = pop(); setZN( m_A ); break;							// PLA
	case 0x28: m_P = (u8)( ( pop() & ~M6502_B ) | M6502_U ); break;			// PLP

	// --- logic -----------------------------------------------------------
	case 0x29: m_A &= fetch();          setZN( m_A ); break;				// AND #
	case 0x25: EA_ZP();    m_A &= rd( ea ); setZN( m_A ); break;
	case 0x35: EA_ZPX();   m_A &= rd( ea ); setZN( m_A ); break;
	case 0x2D: EA_ABS();   m_A &= rd( ea ); setZN( m_A ); break;
	case 0x3D: EA_ABX_R(); m_A &= rd( ea ); setZN( m_A ); break;
	case 0x39: EA_ABY_R(); m_A &= rd( ea ); setZN( m_A ); break;
	case 0x21: EA_IZX();   m_A &= rd( ea ); setZN( m_A ); break;
	case 0x31: EA_IZY_R(); m_A &= rd( ea ); setZN( m_A ); break;

	case 0x49: m_A ^= fetch();          setZN( m_A ); break;				// EOR #
	case 0x45: EA_ZP();    m_A ^= rd( ea ); setZN( m_A ); break;
	case 0x55: EA_ZPX();   m_A ^= rd( ea ); setZN( m_A ); break;
	case 0x4D: EA_ABS();   m_A ^= rd( ea ); setZN( m_A ); break;
	case 0x5D: EA_ABX_R(); m_A ^= rd( ea ); setZN( m_A ); break;
	case 0x59: EA_ABY_R(); m_A ^= rd( ea ); setZN( m_A ); break;
	case 0x41: EA_IZX();   m_A ^= rd( ea ); setZN( m_A ); break;
	case 0x51: EA_IZY_R(); m_A ^= rd( ea ); setZN( m_A ); break;

	case 0x09: m_A |= fetch();          setZN( m_A ); break;				// ORA #
	case 0x05: EA_ZP();    m_A |= rd( ea ); setZN( m_A ); break;
	case 0x15: EA_ZPX();   m_A |= rd( ea ); setZN( m_A ); break;
	case 0x0D: EA_ABS();   m_A |= rd( ea ); setZN( m_A ); break;
	case 0x1D: EA_ABX_R(); m_A |= rd( ea ); setZN( m_A ); break;
	case 0x19: EA_ABY_R(); m_A |= rd( ea ); setZN( m_A ); break;
	case 0x01: EA_IZX();   m_A |= rd( ea ); setZN( m_A ); break;
	case 0x11: EA_IZY_R(); m_A |= rd( ea ); setZN( m_A ); break;

	// BIT: N and V come straight from the operand's bits 7 and 6.
	case 0x24: EA_ZP();  v = rd( ea ); goto doBIT;							// BIT zp
	case 0x2C: EA_ABS(); v = rd( ea );
	doBIT:
		setFlag( M6502_Z, ( m_A & v ) == 0 );
		setFlag( M6502_N, ( v & 0x80 ) != 0 );
		setFlag( M6502_V, ( v & 0x40 ) != 0 );
		break;

	// --- arithmetic ------------------------------------------------------
	case 0x69: opADC( fetch() );          break;							// ADC #
	case 0x65: EA_ZP();    opADC( rd( ea ) ); break;
	case 0x75: EA_ZPX();   opADC( rd( ea ) ); break;
	case 0x6D: EA_ABS();   opADC( rd( ea ) ); break;
	case 0x7D: EA_ABX_R(); opADC( rd( ea ) ); break;
	case 0x79: EA_ABY_R(); opADC( rd( ea ) ); break;
	case 0x61: EA_IZX();   opADC( rd( ea ) ); break;
	case 0x71: EA_IZY_R(); opADC( rd( ea ) ); break;

	case 0xE9: opSBC( fetch() );          break;							// SBC #
	case 0xE5: EA_ZP();    opSBC( rd( ea ) ); break;
	case 0xF5: EA_ZPX();   opSBC( rd( ea ) ); break;
	case 0xED: EA_ABS();   opSBC( rd( ea ) ); break;
	case 0xFD: EA_ABX_R(); opSBC( rd( ea ) ); break;
	case 0xF9: EA_ABY_R(); opSBC( rd( ea ) ); break;
	case 0xE1: EA_IZX();   opSBC( rd( ea ) ); break;
	case 0xF1: EA_IZY_R(); opSBC( rd( ea ) ); break;

	case 0xC9: opCMP( m_A, fetch() );          break;						// CMP #
	case 0xC5: EA_ZP();    opCMP( m_A, rd( ea ) ); break;
	case 0xD5: EA_ZPX();   opCMP( m_A, rd( ea ) ); break;
	case 0xCD: EA_ABS();   opCMP( m_A, rd( ea ) ); break;
	case 0xDD: EA_ABX_R(); opCMP( m_A, rd( ea ) ); break;
	case 0xD9: EA_ABY_R(); opCMP( m_A, rd( ea ) ); break;
	case 0xC1: EA_IZX();   opCMP( m_A, rd( ea ) ); break;
	case 0xD1: EA_IZY_R(); opCMP( m_A, rd( ea ) ); break;

	case 0xE0: opCMP( m_X, fetch() );        break;							// CPX #
	case 0xE4: EA_ZP();  opCMP( m_X, rd( ea ) ); break;
	case 0xEC: EA_ABS(); opCMP( m_X, rd( ea ) ); break;

	case 0xC0: opCMP( m_Y, fetch() );        break;							// CPY #
	case 0xC4: EA_ZP();  opCMP( m_Y, rd( ea ) ); break;
	case 0xCC: EA_ABS(); opCMP( m_Y, rd( ea ) ); break;

	// --- increment / decrement -------------------------------------------
	case 0xE6: EA_ZP();    goto doINC;										// INC zp
	case 0xF6: EA_ZPX();   goto doINC;
	case 0xEE: EA_ABS();   goto doINC;
	case 0xFE: EA_ABX_W();
	doINC:
		v = rd( ea );
		RMW_DUMMY_WRITE( ea, v );
		v = (u8)( v + 1 ); wr( ea, v ); setZN( v );
		break;

	case 0xC6: EA_ZP();    goto doDEC;										// DEC zp
	case 0xD6: EA_ZPX();   goto doDEC;
	case 0xCE: EA_ABS();   goto doDEC;
	case 0xDE: EA_ABX_W();
	doDEC:
		v = rd( ea );
		RMW_DUMMY_WRITE( ea, v );
		v = (u8)( v - 1 ); wr( ea, v ); setZN( v );
		break;

	case 0xE8: m_X++; setZN( m_X ); break;									// INX
	case 0xC8: m_Y++; setZN( m_Y ); break;									// INY
	case 0xCA: m_X--; setZN( m_X ); break;									// DEX
	case 0x88: m_Y--; setZN( m_Y ); break;									// DEY

	// --- shift / rotate --------------------------------------------------
	case 0x0A:																// ASL A
		setFlag( M6502_C, ( m_A & 0x80 ) != 0 );
		m_A = (u8)( m_A << 1 ); setZN( m_A );
		break;
	case 0x06: EA_ZP();    goto doASL;
	case 0x16: EA_ZPX();   goto doASL;
	case 0x0E: EA_ABS();   goto doASL;
	case 0x1E: EA_ABX_W();
	doASL:
		v = rd( ea );
		RMW_DUMMY_WRITE( ea, v );
		setFlag( M6502_C, ( v & 0x80 ) != 0 );
		v = (u8)( v << 1 ); wr( ea, v ); setZN( v );
		break;

	case 0x4A:																// LSR A
		setFlag( M6502_C, ( m_A & 0x01 ) != 0 );
		m_A = (u8)( m_A >> 1 ); setZN( m_A );
		break;
	case 0x46: EA_ZP();    goto doLSR;
	case 0x56: EA_ZPX();   goto doLSR;
	case 0x4E: EA_ABS();   goto doLSR;
	case 0x5E: EA_ABX_W();
	doLSR:
		v = rd( ea );
		RMW_DUMMY_WRITE( ea, v );
		setFlag( M6502_C, ( v & 0x01 ) != 0 );
		v = (u8)( v >> 1 ); wr( ea, v ); setZN( v );
		break;

	case 0x2A:																// ROL A
		{
			u8 c = ( m_P & M6502_C ) ? 1 : 0;
			setFlag( M6502_C, ( m_A & 0x80 ) != 0 );
			m_A = (u8)( ( m_A << 1 ) | c ); setZN( m_A );
		}
		break;
	case 0x26: EA_ZP();    goto doROL;
	case 0x36: EA_ZPX();   goto doROL;
	case 0x2E: EA_ABS();   goto doROL;
	case 0x3E: EA_ABX_W();
	doROL:
		{
			u8 c = ( m_P & M6502_C ) ? 1 : 0;
			v = rd( ea );
			RMW_DUMMY_WRITE( ea, v );
			setFlag( M6502_C, ( v & 0x80 ) != 0 );
			v = (u8)( ( v << 1 ) | c ); wr( ea, v ); setZN( v );
		}
		break;

	case 0x6A:																// ROR A
		{
			u8 c = ( m_P & M6502_C ) ? 0x80 : 0;
			setFlag( M6502_C, ( m_A & 0x01 ) != 0 );
			m_A = (u8)( ( m_A >> 1 ) | c ); setZN( m_A );
		}
		break;
	case 0x66: EA_ZP();    goto doROR;
	case 0x76: EA_ZPX();   goto doROR;
	case 0x6E: EA_ABS();   goto doROR;
	case 0x7E: EA_ABX_W();
	doROR:
		{
			u8 c = ( m_P & M6502_C ) ? 0x80 : 0;
			v = rd( ea );
			RMW_DUMMY_WRITE( ea, v );
			setFlag( M6502_C, ( v & 0x01 ) != 0 );
			v = (u8)( ( v >> 1 ) | c ); wr( ea, v ); setZN( v );
		}
		break;

	// --- flow ------------------------------------------------------------
	case 0x4C: m_PC = fetch16(); break;										// JMP abs
	case 0x6C:																// JMP (abs) -- page-wrap bug
		ea = fetch16();
		m_PC = rd16WrapPage( ea );
		break;

	case 0x20:																// JSR abs
		{
			u16 target = fetch16();
			push16( (u16)( m_PC - 1 ) );
			m_PC = target;
		}
		break;

	case 0x60: m_PC = (u16)( pop16() + 1 ); break;							// RTS
	case 0x40:																// RTI
		m_P  = (u8)( ( pop() & ~M6502_B ) | M6502_U );
		m_PC = pop16();
		break;

	case 0x00:																// BRK
		m_PC++;								// BRK is a two-byte instruction
		push16( m_PC );
		push( (u8)( m_P | M6502_B | M6502_U ) );
		m_P |= M6502_I;
		m_PC = rd16( M6502_VEC_IRQ );
		break;

	// --- branches --------------------------------------------------------
	case 0x10: BRANCH( !( m_P & M6502_N ) ); break;							// BPL
	case 0x30: BRANCH(  ( m_P & M6502_N ) ); break;							// BMI
	case 0x50: BRANCH( !( m_P & M6502_V ) ); break;							// BVC
	case 0x70: BRANCH(  ( m_P & M6502_V ) ); break;							// BVS
	case 0x90: BRANCH( !( m_P & M6502_C ) ); break;							// BCC
	case 0xB0: BRANCH(  ( m_P & M6502_C ) ); break;							// BCS
	case 0xD0: BRANCH( !( m_P & M6502_Z ) ); break;							// BNE
	case 0xF0: BRANCH(  ( m_P & M6502_Z ) ); break;							// BEQ

	// --- flags -----------------------------------------------------------
	case 0x18: m_P = (u8)( m_P & ~M6502_C ); break;							// CLC
	case 0x38: m_P |= M6502_C;               break;							// SEC
	case 0x58: m_P = (u8)( m_P & ~M6502_I ); break;							// CLI
	case 0x78: m_P |= M6502_I;               break;							// SEI
	case 0xB8: m_P = (u8)( m_P & ~M6502_V ); break;							// CLV
	case 0xD8: m_P = (u8)( m_P & ~M6502_D ); break;							// CLD
	case 0xF8: m_P |= M6502_D;               break;							// SED

	case 0xEA: break;														// NOP

	default:
		{
			// Undocumented encoding. SCPU-EMU executes it as a NOP of the
			// correct length -- see the note in m6502.h. A JAM encoding (base
			// cycle count 0 in the table) halts the core, as on real silicon.
			const M6502OpcodeInfo *info = &m6502Opcodes[ opcode ];
			if ( info->cycles == 0 )
			{
				m_Jammed = true;
				m_PC--;						// stay on the jamming opcode
				cyc = 1;
			} else
			{
				m_PC = (u16)( m_PC + m6502ModeLength[ info->mode ] - 1 );
			}
		}
		break;
	}

	m_Cycles += cyc;
	m_Bus->tick( cyc );
	return cyc;
}

#undef EA_ZP
#undef EA_ZPX
#undef EA_ZPY
#undef EA_ABS
#undef EA_ABX_R
#undef EA_ABX_W
#undef EA_ABY_R
#undef EA_ABY_W
#undef EA_IZX
#undef EA_IZY_R
#undef EA_IZY_W
#undef BRANCH
#undef RMW_DUMMY_WRITE

u64 CM6502::run( u64 nCycles )
{
	u64 start = m_Cycles;
	m_RunBreak = false;
	while ( m_Cycles - start < nCycles && !m_RunBreak )
	{
		step();
		if ( m_Jammed )
			break;
	}
	return m_Cycles - start;
}
