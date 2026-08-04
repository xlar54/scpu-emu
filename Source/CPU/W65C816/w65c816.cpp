/*
   SCPU-EMU - CMD SuperCPU emulation for the C64/C128 using a RAD Expansion Unit
   Copyright (c) 2026 SCPU-EMU contributors

   WDC 65C816 interpreter. See w65c816.h for the design, w65c816_addressing.h
   for the address wrapping and stack rules, and w65c816_opcodes.cpp for the
   lengths and cycle counts -- which are TABLE DRIVEN here on purpose. The
   dispatch below computes addresses and results; it never counts cycles, so a
   cycle count and the opcode table cannot drift apart.

   The full derivation of everything here, including what is measured and what
   is still open, is in Docs/research/65816-reference.md.

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
#include "w65c816.h"
#include "w65c816_addressing.h"
#include "w65c816_opcodes.h"

CW65C816::CW65C816()
	: m_C( 0 ), m_X( 0 ), m_Y( 0 ), m_S( 0x01FD ), m_D( 0 ),
	  m_PC( 0 ), m_PBR( 0 ), m_DBR( 0 ),
	  m_P( W65_M | W65_X | W65_I ), m_E( true ),
	  m_Stopped( false ), m_Waiting( false ),
	  m_NativeInstructions( 0 ),
	  m_Bus( 0 ), m_Cycles( 0 ),
	  m_NMIPrevLevel( false ), m_NMIPending( false ),
	  m_PageCross( false ), m_BranchTaken( false ), m_EABank0( false )
{
}

void CW65C816::reset()
{
	// RESET always lands in emulation mode with 8-bit registers, which is why a
	// stock C64 KERNAL runs unmodified on a SuperCPU. The datasheet leaves SL
	// indeterminate; $01FD is CM6502's convention, chosen so the differential
	// test starts both cores in the same place.
	m_E   = true;
	m_P   = (u8)( W65_M | W65_X | W65_I );
	m_C   = 0;
	m_X   = 0;
	m_Y   = 0;
	m_S   = 0x01FD;
	m_D   = 0;
	m_DBR = 0;
	m_PBR = 0;

	m_Stopped = false;
	m_Waiting = false;
	m_NMIPrevLevel = false;
	m_NMIPending = false;
	m_Cycles = 0;
	m_NativeInstructions = 0;
	m_PageCross = false;
	m_BranchTaken = false;
	m_EABank0 = false;

	m_PC = read16Bank0( (u16)W65_VEC_EMU_RESET );
}

// ---------------------------------------------------------------------------
// ALU
// ---------------------------------------------------------------------------

void CW65C816::opADC( u16 v )
{
	const bool m8 = memory8();
	const u16  a  = getAcc();
	const u32  c  = ( m_P & W65_C ) ? 1 : 0;
	const u16  sb = m8 ? 0x0080 : 0x8000;

	if ( m_P & W65_D )
	{
		// The 65816's decimal mode is the 65C02's, not the NMOS 6502's: N and Z
		// are computed from the final BCD result, so they are valid. V is still
		// computed the NMOS way -- from the top digit BEFORE its +6 fixup --
		// which makes it meaningless for BCD, but it is what the chip produces
		// and the differential test will compare it.
		//
		// Unlike the 65C02 there is NO extra cycle in decimal mode.
		const int nd = m8 ? 2 : 4;
		u32 result = 0, pre = 0, carry = c;

		for ( int i = 0; i < nd; i++ )
		{
			const int sh = i * 4;
			u32 dg = ( ( a >> sh ) & 0xF ) + ( ( v >> sh ) & 0xF ) + carry;
			carry = 0;

			if ( i == nd - 1 )
				pre = result | ( dg << sh );		// raw top digit, for V

			if ( dg > 9 ) { dg += 6; carry = 1; }
			result |= ( dg & 0xF ) << sh;
		}

		setFlag( W65_C, carry != 0 );
		setFlag( W65_V, ( ( ~( a ^ v ) & ( a ^ pre ) ) & sb ) != 0 );
		setAcc( (u16)result );
		if ( m8 ) setZN8( (u8)result ); else setZN16( (u16)result );
		return;
	}

	const u32 r = (u32)a + (u32)v + c;
	setFlag( W65_V, ( ( ~( a ^ v ) & ( a ^ r ) ) & sb ) != 0 );
	setFlag( W65_C, r > ( m8 ? 0xFFu : 0xFFFFu ) );
	setAcc( (u16)r );
	if ( m8 ) setZN8( (u8)r ); else setZN16( (u16)r );
}

void CW65C816::opSBC( u16 v )
{
	const bool m8   = memory8();
	const u16  a    = getAcc();
	const u32  c    = ( m_P & W65_C ) ? 1 : 0;
	const u16  sb   = m8 ? 0x0080 : 0x8000;
	const u32  mask = m8 ? 0xFFu : 0xFFFFu;

	// Subtraction is addition of the complement, and C and V come out of that
	// binary result in BOTH modes -- only N and Z differ in decimal.
	const u16 vc  = (u16)( ~v & mask );
	const u32 bin = (u32)a + (u32)vc + c;

	setFlag( W65_C, bin > mask );
	setFlag( W65_V, ( ( ( a ^ v ) & ( a ^ ( bin & mask ) ) ) & sb ) != 0 );

	if ( m_P & W65_D )
	{
		const int nd = m8 ? 2 : 4;
		u32 result = 0;
		s32 borrow = (s32)( 1 - c );

		for ( int i = 0; i < nd; i++ )
		{
			const int sh = i * 4;
			s32 dg = (s32)( ( a >> sh ) & 0xF ) - (s32)( ( v >> sh ) & 0xF ) - borrow;
			borrow = 0;
			if ( dg < 0 ) { dg -= 6; borrow = 1; }
			result |= ( (u32)dg & 0xF ) << sh;
		}

		setAcc( (u16)result );
		if ( m8 ) setZN8( (u8)result ); else setZN16( (u16)result );
		return;
	}

	setAcc( (u16)bin );
	if ( m8 ) setZN8( (u8)bin ); else setZN16( (u16)bin );
}

void CW65C816::opCMP( u16 reg, u16 v, bool eightBit )
{
	const u32 r = (u32)reg - (u32)v;
	setFlag( W65_C, reg >= v );
	if ( eightBit ) setZN8( (u8)r ); else setZN16( (u16)r );
}

void CW65C816::opBIT( u16 v, bool immediate )
{
	// BIT #imm is the odd one out: it sets only Z, leaving N and V alone. Every
	// other form copies the operand's top two bits into N and V.
	setFlag( W65_Z, ( getAcc() & v ) == 0 );

	if ( !immediate )
	{
		const u16 nb = memory8() ? 0x0080 : 0x8000;
		const u16 vb = memory8() ? 0x0040 : 0x4000;
		setFlag( W65_N, ( v & nb ) != 0 );
		setFlag( W65_V, ( v & vb ) != 0 );
	}
}

// ---------------------------------------------------------------------------
// Interrupts
// ---------------------------------------------------------------------------

void CW65C816::serviceInterrupt( u32 vectorNative, u32 vectorEmu, bool isBRKorCOP )
{
	// Interrupts and COP use the "old" stack class, so they stay inside page 1
	// in emulation mode.
	if ( m_E )
	{
		// Emulation mode pushes exactly what a 6502 does: three bytes, no
		// program bank, and bit 4 of the pushed P distinguishes BRK and COP
		// from a hardware interrupt. There is no settable B flag -- it is an
		// internal signal that only selects that bit. Because x is held at 1 in
		// emulation mode the byte comes out identical to an NMOS 6502's, which
		// is what lets CM6502 stand in for this core.
		push16( m_PC );
		push8( isBRKorCOP ? (u8)( m_P | W65_B ) : (u8)( m_P & ~W65_B ) );
	} else
	{
		// Native mode pushes four bytes, program bank first. There is no B bit
		// at all here: bit 4 is X and is pushed verbatim, BRK included. A
		// native-mode handler must not test it -- BRK and IRQ are distinguished
		// only by having separate vectors.
		push8( m_PBR );
		push16( m_PC );
		push8( m_P );
	}

	// Push P first, THEN set I and clear D. Clearing D on interrupt entry is a
	// real divergence from the NMOS 6502, which leaves it alone: a program that
	// takes an interrupt while in decimal mode behaves differently on a
	// SuperCPU than on a stock C64. That is hardware behaviour, not our bug.
	m_P |= W65_I;
	m_P = (u8)( m_P & ~W65_D );

	// PBR is cleared in both modes -- but in emulation it is not saved, so an
	// interrupt taken while executing outside bank 0 cannot be returned from.
	m_PBR = 0;

	m_PC = read16Bank0( (u16)( m_E ? vectorEmu : vectorNative ) );
	applyE();
}

// ---------------------------------------------------------------------------
// Interpreter
// ---------------------------------------------------------------------------

u32 CW65C816::step()
{
	// STP stops the clock, and only RESET restarts it -- not IRQ, not NMI. Burn
	// a cycle and keep ticking the bus so the C64 around us carries on.
	if ( m_Stopped )
	{
		m_Cycles += 1;
		m_Bus->tick( 1 );
		return 1;
	}

	// NMI is edge triggered, IRQ level triggered.
	const bool nmiLevel = m_Bus->nmiAsserted();
	if ( nmiLevel && !m_NMIPrevLevel ) m_NMIPending = true;
	m_NMIPrevLevel = nmiLevel;

	const bool irq = m_Bus->irqAsserted();

	if ( m_NMIPending )
	{
		m_NMIPending = false;
		m_Waiting = false;
		const u32 c = m_E ? 7 : 8;
		serviceInterrupt( W65_VEC_NATIVE_NMI, W65_VEC_EMU_NMI, false );
		m_Cycles += c;
		m_Bus->tick( c );
		return c;
	}

	// WAI parks the processor until an interrupt appears. It resumes even when
	// the interrupt is masked -- and that second exit is the whole point of the
	// instruction: with I=1 and IRQ asserted, execution simply continues at the
	// instruction after the WAI without going near the handler.
	if ( m_Waiting )
	{
		if ( !irq )
		{
			m_Cycles += 1;
			m_Bus->tick( 1 );
			return 1;
		}
		m_Waiting = false;
	}

	if ( irq && !( m_P & W65_I ) )
	{
		const u32 c = m_E ? 7 : 8;
		serviceInterrupt( W65_VEC_NATIVE_IRQ, W65_VEC_EMU_IRQ, false );
		m_Cycles += c;
		m_Bus->tick( c );
		return c;
	}

	// Captured before dispatch: REP, SEP, PLP, RTI and XCE change the register
	// widths mid-instruction, but their own cycle counts are the ones for the
	// state they started in.
	const bool m8Entry = memory8();
	const bool x8Entry = index8();
	const bool eEntry  = m_E;
	const bool dpUnaligned = ( m_D & 0xFF ) != 0;

	m_PageCross = false;
	m_BranchTaken = false;
	m_EABank0 = false;

	if ( !m_E ) m_NativeInstructions++;

	const u8 opcode = fetch8();
	u32 ea = 0;
	u16 v  = 0;
	u16 t  = 0;

	// Every branch shares this. The target wraps inside the program bank -- a
	// branch cannot leave it.
	#define W65_BRANCH( cond )                                          \
		{                                                               \
			s8 disp = (s8)fetch8();                                     \
			u16 target = (u16)( m_PC + disp );                          \
			if ( ( m_PC ^ target ) & 0xFF00 ) m_PageCross = true;       \
			if ( cond ) { m_BranchTaken = true; m_PC = target; }        \
		}

	switch ( opcode )
	{
	// --- LDA ---------------------------------------------------------------
	case 0xA9: v = immediateM();                    setAcc( v ); setZNM( v ); break;
	case 0xA5: ea = eaDirect();                     v = readM( ea ); setAcc( v ); setZNM( v ); break;
	case 0xB5: ea = eaDirectIndexed( getX() );      v = readM( ea ); setAcc( v ); setZNM( v ); break;
	case 0xAD: ea = eaAbsolute();                   v = readM( ea ); setAcc( v ); setZNM( v ); break;
	case 0xBD: ea = eaAbsoluteIndexed( getX() );    v = readM( ea ); setAcc( v ); setZNM( v ); break;
	case 0xB9: ea = eaAbsoluteIndexed( getY() );    v = readM( ea ); setAcc( v ); setZNM( v ); break;
	case 0xAF: ea = eaLong();                       v = readM( ea ); setAcc( v ); setZNM( v ); break;
	case 0xBF: ea = eaLongIndexed( getX() );        v = readM( ea ); setAcc( v ); setZNM( v ); break;
	case 0xA1: ea = eaIndirectX();                  v = readM( ea ); setAcc( v ); setZNM( v ); break;
	case 0xB1: ea = eaIndirectY();                  v = readM( ea ); setAcc( v ); setZNM( v ); break;
	case 0xB2: ea = eaIndirect();                   v = readM( ea ); setAcc( v ); setZNM( v ); break;
	case 0xA7: ea = eaIndirectLong();               v = readM( ea ); setAcc( v ); setZNM( v ); break;
	case 0xB7: ea = eaIndirectLongY();              v = readM( ea ); setAcc( v ); setZNM( v ); break;
	case 0xA3: ea = eaStackRelative();              v = readM( ea ); setAcc( v ); setZNM( v ); break;
	case 0xB3: ea = eaStackRelativeIndirectY();     v = readM( ea ); setAcc( v ); setZNM( v ); break;

	// --- LDX / LDY ---------------------------------------------------------
	case 0xA2: m_X = immediateX();                                 setZNX( m_X ); break;
	case 0xA6: ea = eaDirect();                  m_X = readX( ea ); setZNX( m_X ); break;
	case 0xB6: ea = eaDirectIndexed( getY() );   m_X = readX( ea ); setZNX( m_X ); break;
	case 0xAE: ea = eaAbsolute();                m_X = readX( ea ); setZNX( m_X ); break;
	case 0xBE: ea = eaAbsoluteIndexed( getY() ); m_X = readX( ea ); setZNX( m_X ); break;

	case 0xA0: m_Y = immediateX();                                 setZNX( m_Y ); break;
	case 0xA4: ea = eaDirect();                  m_Y = readX( ea ); setZNX( m_Y ); break;
	case 0xB4: ea = eaDirectIndexed( getX() );   m_Y = readX( ea ); setZNX( m_Y ); break;
	case 0xAC: ea = eaAbsolute();                m_Y = readX( ea ); setZNX( m_Y ); break;
	case 0xBC: ea = eaAbsoluteIndexed( getX() ); m_Y = readX( ea ); setZNX( m_Y ); break;

	// --- STA ---------------------------------------------------------------
	case 0x85: ea = eaDirect();                  writeM( ea, getAcc() ); break;
	case 0x95: ea = eaDirectIndexed( getX() );   writeM( ea, getAcc() ); break;
	case 0x8D: ea = eaAbsolute();                writeM( ea, getAcc() ); break;
	case 0x9D: ea = eaAbsoluteIndexed( getX() ); writeM( ea, getAcc() ); break;
	case 0x99: ea = eaAbsoluteIndexed( getY() ); writeM( ea, getAcc() ); break;
	case 0x8F: ea = eaLong();                    writeM( ea, getAcc() ); break;
	case 0x9F: ea = eaLongIndexed( getX() );     writeM( ea, getAcc() ); break;
	case 0x81: ea = eaIndirectX();               writeM( ea, getAcc() ); break;
	case 0x91: ea = eaIndirectY();               writeM( ea, getAcc() ); break;
	case 0x92: ea = eaIndirect();                writeM( ea, getAcc() ); break;
	case 0x87: ea = eaIndirectLong();            writeM( ea, getAcc() ); break;
	case 0x97: ea = eaIndirectLongY();           writeM( ea, getAcc() ); break;
	case 0x83: ea = eaStackRelative();           writeM( ea, getAcc() ); break;
	case 0x93: ea = eaStackRelativeIndirectY();  writeM( ea, getAcc() ); break;

	// --- STX / STY / STZ ---------------------------------------------------
	case 0x86: ea = eaDirect();                writeX( ea, m_X ); break;
	case 0x96: ea = eaDirectIndexed( getY() ); writeX( ea, m_X ); break;
	case 0x8E: ea = eaAbsolute();              writeX( ea, m_X ); break;

	case 0x84: ea = eaDirect();                writeX( ea, m_Y ); break;
	case 0x94: ea = eaDirectIndexed( getX() ); writeX( ea, m_Y ); break;
	case 0x8C: ea = eaAbsolute();              writeX( ea, m_Y ); break;

	case 0x64: ea = eaDirect();                  writeM( ea, 0 ); break;
	case 0x74: ea = eaDirectIndexed( getX() );   writeM( ea, 0 ); break;
	case 0x9C: ea = eaAbsolute();                writeM( ea, 0 ); break;
	case 0x9E: ea = eaAbsoluteIndexed( getX() ); writeM( ea, 0 ); break;

	// --- transfers ---------------------------------------------------------
	// Width follows the DESTINATION register. The TCS/TSC/TCD/TDC group is the
	// most commonly mis-implemented on the chip: TCS sets no flags at all, and
	// the other three set N and Z from all 16 bits even when m=1 or E=1.
	case 0xAA: m_X = index8() ? (u16)( m_C & 0xFF ) : m_C; setZNX( m_X ); break;	// TAX
	case 0xA8: m_Y = index8() ? (u16)( m_C & 0xFF ) : m_C; setZNX( m_Y ); break;	// TAY
	case 0x8A: setAcc( getX() ); setZNM( getAcc() ); break;						// TXA
	case 0x98: setAcc( getY() ); setZNM( getAcc() ); break;						// TYA
	case 0xBA: m_X = index8() ? (u16)( m_S & 0xFF ) : m_S; setZNX( m_X ); break;	// TSX
	case 0x9A: m_S = m_E ? (u16)( 0x0100 | ( m_X & 0xFF ) ) : getX(); break;		// TXS, no flags
	case 0x9B: m_Y = index8() ? (u16)( m_X & 0xFF ) : m_X; setZNX( m_Y ); break;	// TXY
	case 0xBB: m_X = index8() ? (u16)( m_Y & 0xFF ) : m_Y; setZNX( m_X ); break;	// TYX
	case 0x5B: m_D = m_C; setZN16( m_D ); break;								// TCD
	case 0x7B: m_C = m_D; setZN16( m_C ); break;								// TDC
	case 0x1B: m_S = m_E ? (u16)( 0x0100 | ( m_C & 0xFF ) ) : m_C; break;		// TCS, no flags, low byte only in emulation
	case 0x3B: m_C = m_S; setZN16( m_C ); break;								// TSC, returns the full $01xx in emulation
	case 0xEB:																	// XBA
		// Ignores M, X and E entirely, and sets N and Z from the new 8-bit low
		// byte even when the accumulator is 16-bit.
		m_C = (u16)( ( m_C >> 8 ) | ( m_C << 8 ) );
		setZN8( (u8)( m_C & 0xFF ) );
		break;

	// --- stack -------------------------------------------------------------
	// "old" class: confined to page 1 in emulation mode.
	case 0x48: if ( memory8() ) push8( getA() ); else push16( m_C ); break;	// PHA
	case 0xDA: if ( index8() ) push8( (u8)m_X ); else push16( m_X ); break;	// PHX
	case 0x5A: if ( index8() ) push8( (u8)m_Y ); else push16( m_Y ); break;	// PHY
	case 0x08: push8( m_E ? (u8)( m_P | W65_B ) : m_P ); break;				// PHP
	case 0x8B: push8( m_DBR ); break;										// PHB
	case 0x4B: push8( m_PBR ); break;										// PHK

	case 0x68:																// PLA
		if ( memory8() ) { u8 b = pull8(); setAcc( b ); setZN8( b ); }
		else             { m_C = pull16(); setZN16( m_C ); }
		break;
	case 0xFA:																// PLX
		if ( index8() ) { m_X = pull8(); setZN8( (u8)m_X ); }
		else            { m_X = pull16(); setZN16( m_X ); }
		break;
	case 0x7A:																// PLY
		if ( index8() ) { m_Y = pull8(); setZN8( (u8)m_Y ); }
		else            { m_Y = pull16(); setZN16( m_Y ); }
		break;
	case 0x28:																// PLP
		// In emulation mode this cannot touch m or x; applyE() re-asserts them.
		m_P = pull8();
		applyE();
		break;
	case 0xAB: m_DBR = pull8(); setZN8( m_DBR ); break;						// PLB

	// "new" class: a full 16-bit stack pointer even in emulation mode, so these
	// run off the bottom of page 1 rather than wrapping inside it.
	case 0x0B: push16New( m_D ); applyE(); break;										// PHD
	case 0x2B: m_D = pull16New(); setZN16( m_D ); applyE(); break;					// PLD
	case 0xF4: push16New( fetch16() ); applyE(); break;								// PEA -- pushes its operand literally, reads no memory
	case 0xD4:																// PEI -- reads, and is exempt from the direct-page wrap
		ea = eaDirect();
		push16New( read16Bank0( (u16)ea ) );
		applyE();
		break;
	case 0x62:																// PER
		{
			s16 disp = (s16)fetch16();
			push16New( (u16)( m_PC + disp ) );
			applyE();
		}
		break;

	// --- logic -------------------------------------------------------------
	#define W65_SETACC( expr ) { setAcc( (u16)( expr ) ); setZNM( getAcc() ); }

	case 0x29: v = immediateM();                 W65_SETACC( getAcc() & v ) break;
	case 0x25: ea = eaDirect();                  v = readM( ea ); W65_SETACC( getAcc() & v ) break;
	case 0x35: ea = eaDirectIndexed( getX() );   v = readM( ea ); W65_SETACC( getAcc() & v ) break;
	case 0x2D: ea = eaAbsolute();                v = readM( ea ); W65_SETACC( getAcc() & v ) break;
	case 0x3D: ea = eaAbsoluteIndexed( getX() ); v = readM( ea ); W65_SETACC( getAcc() & v ) break;
	case 0x39: ea = eaAbsoluteIndexed( getY() ); v = readM( ea ); W65_SETACC( getAcc() & v ) break;
	case 0x2F: ea = eaLong();                    v = readM( ea ); W65_SETACC( getAcc() & v ) break;
	case 0x3F: ea = eaLongIndexed( getX() );     v = readM( ea ); W65_SETACC( getAcc() & v ) break;
	case 0x21: ea = eaIndirectX();               v = readM( ea ); W65_SETACC( getAcc() & v ) break;
	case 0x31: ea = eaIndirectY();               v = readM( ea ); W65_SETACC( getAcc() & v ) break;
	case 0x32: ea = eaIndirect();                v = readM( ea ); W65_SETACC( getAcc() & v ) break;
	case 0x27: ea = eaIndirectLong();            v = readM( ea ); W65_SETACC( getAcc() & v ) break;
	case 0x37: ea = eaIndirectLongY();           v = readM( ea ); W65_SETACC( getAcc() & v ) break;
	case 0x23: ea = eaStackRelative();           v = readM( ea ); W65_SETACC( getAcc() & v ) break;
	case 0x33: ea = eaStackRelativeIndirectY();  v = readM( ea ); W65_SETACC( getAcc() & v ) break;

	case 0x09: v = immediateM();                 W65_SETACC( getAcc() | v ) break;
	case 0x05: ea = eaDirect();                  v = readM( ea ); W65_SETACC( getAcc() | v ) break;
	case 0x15: ea = eaDirectIndexed( getX() );   v = readM( ea ); W65_SETACC( getAcc() | v ) break;
	case 0x0D: ea = eaAbsolute();                v = readM( ea ); W65_SETACC( getAcc() | v ) break;
	case 0x1D: ea = eaAbsoluteIndexed( getX() ); v = readM( ea ); W65_SETACC( getAcc() | v ) break;
	case 0x19: ea = eaAbsoluteIndexed( getY() ); v = readM( ea ); W65_SETACC( getAcc() | v ) break;
	case 0x0F: ea = eaLong();                    v = readM( ea ); W65_SETACC( getAcc() | v ) break;
	case 0x1F: ea = eaLongIndexed( getX() );     v = readM( ea ); W65_SETACC( getAcc() | v ) break;
	case 0x01: ea = eaIndirectX();               v = readM( ea ); W65_SETACC( getAcc() | v ) break;
	case 0x11: ea = eaIndirectY();               v = readM( ea ); W65_SETACC( getAcc() | v ) break;
	case 0x12: ea = eaIndirect();                v = readM( ea ); W65_SETACC( getAcc() | v ) break;
	case 0x07: ea = eaIndirectLong();            v = readM( ea ); W65_SETACC( getAcc() | v ) break;
	case 0x17: ea = eaIndirectLongY();           v = readM( ea ); W65_SETACC( getAcc() | v ) break;
	case 0x03: ea = eaStackRelative();           v = readM( ea ); W65_SETACC( getAcc() | v ) break;
	case 0x13: ea = eaStackRelativeIndirectY();  v = readM( ea ); W65_SETACC( getAcc() | v ) break;

	case 0x49: v = immediateM();                 W65_SETACC( getAcc() ^ v ) break;
	case 0x45: ea = eaDirect();                  v = readM( ea ); W65_SETACC( getAcc() ^ v ) break;
	case 0x55: ea = eaDirectIndexed( getX() );   v = readM( ea ); W65_SETACC( getAcc() ^ v ) break;
	case 0x4D: ea = eaAbsolute();                v = readM( ea ); W65_SETACC( getAcc() ^ v ) break;
	case 0x5D: ea = eaAbsoluteIndexed( getX() ); v = readM( ea ); W65_SETACC( getAcc() ^ v ) break;
	case 0x59: ea = eaAbsoluteIndexed( getY() ); v = readM( ea ); W65_SETACC( getAcc() ^ v ) break;
	case 0x4F: ea = eaLong();                    v = readM( ea ); W65_SETACC( getAcc() ^ v ) break;
	case 0x5F: ea = eaLongIndexed( getX() );     v = readM( ea ); W65_SETACC( getAcc() ^ v ) break;
	case 0x41: ea = eaIndirectX();               v = readM( ea ); W65_SETACC( getAcc() ^ v ) break;
	case 0x51: ea = eaIndirectY();               v = readM( ea ); W65_SETACC( getAcc() ^ v ) break;
	case 0x52: ea = eaIndirect();                v = readM( ea ); W65_SETACC( getAcc() ^ v ) break;
	case 0x47: ea = eaIndirectLong();            v = readM( ea ); W65_SETACC( getAcc() ^ v ) break;
	case 0x57: ea = eaIndirectLongY();           v = readM( ea ); W65_SETACC( getAcc() ^ v ) break;
	case 0x43: ea = eaStackRelative();           v = readM( ea ); W65_SETACC( getAcc() ^ v ) break;
	case 0x53: ea = eaStackRelativeIndirectY();  v = readM( ea ); W65_SETACC( getAcc() ^ v ) break;

	#undef W65_SETACC

	// --- BIT ---------------------------------------------------------------
	case 0x89: opBIT( immediateM(), true );  break;
	case 0x24: ea = eaDirect();                  opBIT( readM( ea ), false ); break;
	case 0x34: ea = eaDirectIndexed( getX() );   opBIT( readM( ea ), false ); break;
	case 0x2C: ea = eaAbsolute();                opBIT( readM( ea ), false ); break;
	case 0x3C: ea = eaAbsoluteIndexed( getX() ); opBIT( readM( ea ), false ); break;

	// --- TRB / TSB ---------------------------------------------------------
	case 0x14: ea = eaDirect();   v = readM( ea ); setFlag( W65_Z, ( getAcc() & v ) == 0 ); rmwWrite( ea, v, (u16)( v & ~getAcc() ) ); break;
	case 0x1C: ea = eaAbsolute(); v = readM( ea ); setFlag( W65_Z, ( getAcc() & v ) == 0 ); rmwWrite( ea, v, (u16)( v & ~getAcc() ) ); break;
	case 0x04: ea = eaDirect();   v = readM( ea ); setFlag( W65_Z, ( getAcc() & v ) == 0 ); rmwWrite( ea, v, (u16)( v |  getAcc() ) ); break;
	case 0x0C: ea = eaAbsolute(); v = readM( ea ); setFlag( W65_Z, ( getAcc() & v ) == 0 ); rmwWrite( ea, v, (u16)( v |  getAcc() ) ); break;

	// --- ADC / SBC ---------------------------------------------------------
	case 0x69: opADC( immediateM() ); break;
	case 0x65: ea = eaDirect();                  opADC( readM( ea ) ); break;
	case 0x75: ea = eaDirectIndexed( getX() );   opADC( readM( ea ) ); break;
	case 0x6D: ea = eaAbsolute();                opADC( readM( ea ) ); break;
	case 0x7D: ea = eaAbsoluteIndexed( getX() ); opADC( readM( ea ) ); break;
	case 0x79: ea = eaAbsoluteIndexed( getY() ); opADC( readM( ea ) ); break;
	case 0x6F: ea = eaLong();                    opADC( readM( ea ) ); break;
	case 0x7F: ea = eaLongIndexed( getX() );     opADC( readM( ea ) ); break;
	case 0x61: ea = eaIndirectX();               opADC( readM( ea ) ); break;
	case 0x71: ea = eaIndirectY();               opADC( readM( ea ) ); break;
	case 0x72: ea = eaIndirect();                opADC( readM( ea ) ); break;
	case 0x67: ea = eaIndirectLong();            opADC( readM( ea ) ); break;
	case 0x77: ea = eaIndirectLongY();           opADC( readM( ea ) ); break;
	case 0x63: ea = eaStackRelative();           opADC( readM( ea ) ); break;
	case 0x73: ea = eaStackRelativeIndirectY();  opADC( readM( ea ) ); break;

	case 0xE9: opSBC( immediateM() ); break;
	case 0xE5: ea = eaDirect();                  opSBC( readM( ea ) ); break;
	case 0xF5: ea = eaDirectIndexed( getX() );   opSBC( readM( ea ) ); break;
	case 0xED: ea = eaAbsolute();                opSBC( readM( ea ) ); break;
	case 0xFD: ea = eaAbsoluteIndexed( getX() ); opSBC( readM( ea ) ); break;
	case 0xF9: ea = eaAbsoluteIndexed( getY() ); opSBC( readM( ea ) ); break;
	case 0xEF: ea = eaLong();                    opSBC( readM( ea ) ); break;
	case 0xFF: ea = eaLongIndexed( getX() );     opSBC( readM( ea ) ); break;
	case 0xE1: ea = eaIndirectX();               opSBC( readM( ea ) ); break;
	case 0xF1: ea = eaIndirectY();               opSBC( readM( ea ) ); break;
	case 0xF2: ea = eaIndirect();                opSBC( readM( ea ) ); break;
	case 0xE7: ea = eaIndirectLong();            opSBC( readM( ea ) ); break;
	case 0xF7: ea = eaIndirectLongY();           opSBC( readM( ea ) ); break;
	case 0xE3: ea = eaStackRelative();           opSBC( readM( ea ) ); break;
	case 0xF3: ea = eaStackRelativeIndirectY();  opSBC( readM( ea ) ); break;

	// --- CMP / CPX / CPY ---------------------------------------------------
	#define W65_CMPA( src ) opCMP( getAcc(), src, m8Entry )

	case 0xC9: W65_CMPA( immediateM() ); break;
	case 0xC5: ea = eaDirect();                  W65_CMPA( readM( ea ) ); break;
	case 0xD5: ea = eaDirectIndexed( getX() );   W65_CMPA( readM( ea ) ); break;
	case 0xCD: ea = eaAbsolute();                W65_CMPA( readM( ea ) ); break;
	case 0xDD: ea = eaAbsoluteIndexed( getX() ); W65_CMPA( readM( ea ) ); break;
	case 0xD9: ea = eaAbsoluteIndexed( getY() ); W65_CMPA( readM( ea ) ); break;
	case 0xCF: ea = eaLong();                    W65_CMPA( readM( ea ) ); break;
	case 0xDF: ea = eaLongIndexed( getX() );     W65_CMPA( readM( ea ) ); break;
	case 0xC1: ea = eaIndirectX();               W65_CMPA( readM( ea ) ); break;
	case 0xD1: ea = eaIndirectY();               W65_CMPA( readM( ea ) ); break;
	case 0xD2: ea = eaIndirect();                W65_CMPA( readM( ea ) ); break;
	case 0xC7: ea = eaIndirectLong();            W65_CMPA( readM( ea ) ); break;
	case 0xD7: ea = eaIndirectLongY();           W65_CMPA( readM( ea ) ); break;
	case 0xC3: ea = eaStackRelative();           W65_CMPA( readM( ea ) ); break;
	case 0xD3: ea = eaStackRelativeIndirectY();  W65_CMPA( readM( ea ) ); break;

	#undef W65_CMPA

	case 0xE0: opCMP( getX(), immediateX(), x8Entry ); break;
	case 0xE4: ea = eaDirect();   opCMP( getX(), readX( ea ), x8Entry ); break;
	case 0xEC: ea = eaAbsolute(); opCMP( getX(), readX( ea ), x8Entry ); break;

	case 0xC0: opCMP( getY(), immediateX(), x8Entry ); break;
	case 0xC4: ea = eaDirect();   opCMP( getY(), readX( ea ), x8Entry ); break;
	case 0xCC: ea = eaAbsolute(); opCMP( getY(), readX( ea ), x8Entry ); break;

	// --- INC / DEC ---------------------------------------------------------
	case 0x1A: setAcc( (u16)( getAcc() + 1 ) ); setZNM( getAcc() ); break;	// INC A
	case 0x3A: setAcc( (u16)( getAcc() - 1 ) ); setZNM( getAcc() ); break;	// DEC A
	case 0xE8: m_X = index8() ? (u16)( (u8)( m_X + 1 ) ) : (u16)( m_X + 1 ); setZNX( m_X ); break;
	case 0xC8: m_Y = index8() ? (u16)( (u8)( m_Y + 1 ) ) : (u16)( m_Y + 1 ); setZNX( m_Y ); break;
	case 0xCA: m_X = index8() ? (u16)( (u8)( m_X - 1 ) ) : (u16)( m_X - 1 ); setZNX( m_X ); break;
	case 0x88: m_Y = index8() ? (u16)( (u8)( m_Y - 1 ) ) : (u16)( m_Y - 1 ); setZNX( m_Y ); break;

	#define W65_INCDEC( addrExpr, delta )                                       \
		{ ea = addrExpr; v = readM( ea ); t = (u16)( v + (delta) );              \
		  rmwWrite( ea, v, t ); setZNM( t ); }

	case 0xE6: W65_INCDEC( eaDirect(),                  1 ) break;
	case 0xF6: W65_INCDEC( eaDirectIndexed( getX() ),   1 ) break;
	case 0xEE: W65_INCDEC( eaAbsolute(),                1 ) break;
	case 0xFE: W65_INCDEC( eaAbsoluteIndexed( getX() ), 1 ) break;

	case 0xC6: W65_INCDEC( eaDirect(),                  -1 ) break;
	case 0xD6: W65_INCDEC( eaDirectIndexed( getX() ),   -1 ) break;
	case 0xCE: W65_INCDEC( eaAbsolute(),                -1 ) break;
	case 0xDE: W65_INCDEC( eaAbsoluteIndexed( getX() ), -1 ) break;

	#undef W65_INCDEC

	// --- shifts and rotates ------------------------------------------------
	#define W65_ASL_BODY  { const u16 top = memory8() ? 0x80 : 0x8000;            \
	                        setFlag( W65_C, ( t & top ) != 0 ); t = (u16)( t << 1 ); }
	#define W65_LSR_BODY  { setFlag( W65_C, ( t & 1 ) != 0 ); t = (u16)( t >> 1 );  \
	                        if ( memory8() ) t = (u16)( t & 0x7F ); }
	#define W65_ROL_BODY  { const u16 top = memory8() ? 0x80 : 0x8000;            \
	                        const u16 cin = ( m_P & W65_C ) ? 1 : 0;              \
	                        setFlag( W65_C, ( t & top ) != 0 );                   \
	                        t = (u16)( ( t << 1 ) | cin ); }
	#define W65_ROR_BODY  { const u16 top = memory8() ? 0x80 : 0x8000;            \
	                        const u16 cin = ( m_P & W65_C ) ? top : 0;            \
	                        setFlag( W65_C, ( t & 1 ) != 0 );                     \
	                        t = (u16)( ( t >> 1 ) | cin );                        \
	                        if ( memory8() ) t = (u16)( t & 0xFF ); }

	#define W65_SHIFT_ACC( body ) { t = getAcc(); body; setAcc( t ); setZNM( getAcc() ); }
	#define W65_SHIFT_MEM( addrExpr, body )                                     \
		{ ea = addrExpr; v = readM( ea ); t = v; body;                          \
		  rmwWrite( ea, v, t ); setZNM( t ); }

	case 0x0A: W65_SHIFT_ACC( W65_ASL_BODY ) break;
	case 0x4A: W65_SHIFT_ACC( W65_LSR_BODY ) break;
	case 0x2A: W65_SHIFT_ACC( W65_ROL_BODY ) break;
	case 0x6A: W65_SHIFT_ACC( W65_ROR_BODY ) break;

	case 0x06: W65_SHIFT_MEM( eaDirect(),                  W65_ASL_BODY ) break;
	case 0x16: W65_SHIFT_MEM( eaDirectIndexed( getX() ),   W65_ASL_BODY ) break;
	case 0x0E: W65_SHIFT_MEM( eaAbsolute(),                W65_ASL_BODY ) break;
	case 0x1E: W65_SHIFT_MEM( eaAbsoluteIndexed( getX() ), W65_ASL_BODY ) break;

	case 0x46: W65_SHIFT_MEM( eaDirect(),                  W65_LSR_BODY ) break;
	case 0x56: W65_SHIFT_MEM( eaDirectIndexed( getX() ),   W65_LSR_BODY ) break;
	case 0x4E: W65_SHIFT_MEM( eaAbsolute(),                W65_LSR_BODY ) break;
	case 0x5E: W65_SHIFT_MEM( eaAbsoluteIndexed( getX() ), W65_LSR_BODY ) break;

	case 0x26: W65_SHIFT_MEM( eaDirect(),                  W65_ROL_BODY ) break;
	case 0x36: W65_SHIFT_MEM( eaDirectIndexed( getX() ),   W65_ROL_BODY ) break;
	case 0x2E: W65_SHIFT_MEM( eaAbsolute(),                W65_ROL_BODY ) break;
	case 0x3E: W65_SHIFT_MEM( eaAbsoluteIndexed( getX() ), W65_ROL_BODY ) break;

	case 0x66: W65_SHIFT_MEM( eaDirect(),                  W65_ROR_BODY ) break;
	case 0x76: W65_SHIFT_MEM( eaDirectIndexed( getX() ),   W65_ROR_BODY ) break;
	case 0x6E: W65_SHIFT_MEM( eaAbsolute(),                W65_ROR_BODY ) break;
	case 0x7E: W65_SHIFT_MEM( eaAbsoluteIndexed( getX() ), W65_ROR_BODY ) break;

	#undef W65_SHIFT_ACC
	#undef W65_SHIFT_MEM
	#undef W65_ASL_BODY
	#undef W65_LSR_BODY
	#undef W65_ROL_BODY
	#undef W65_ROR_BODY

	// --- jumps and subroutines ---------------------------------------------
	case 0x4C: m_PC = fetch16(); break;									// JMP abs
	case 0x6C:															// JMP (abs)
		// Pointer from BANK 0, and the 6502's $xxFF page-wrap bug is fixed --
		// the increment is a proper 16-bit one. There must be no equivalent of
		// CM6502::rd16WrapPage() anywhere in this core.
		ea = fetch16();
		m_PC = read16Bank0( (u16)ea );
		break;
	case 0x7C:															// JMP (abs,X)
		// Pointer from the PROGRAM bank, not bank 0 -- the opposite of JMP (abs)
		// directly above, and easy to read past in the datasheet.
		{
			u16 ptr = (u16)( fetch16() + getX() );
			m_PC = (u16)( read8( ( (u32)m_PBR << 16 ) | ptr )
			     | ( (u16)read8( ( (u32)m_PBR << 16 ) | (u16)( ptr + 1 ) ) << 8 ) );
		}
		break;
	case 0x5C:															// JML long
		{
			u32 target = fetch24();
			m_PC  = (u16)( target & 0xFFFF );
			m_PBR = (u8)( target >> 16 );
		}
		break;
	case 0xDC:															// JML [abs] -- pointer from bank 0
		ea = fetch16();
		m_PC  = read16Bank0( (u16)ea );
		m_PBR = read8( (u16)( ea + 2 ) );
		break;

	case 0x20:															// JSR abs -- "old" stack
		{
			u16 target = fetch16();
			push16( (u16)( m_PC - 1 ) );	// the address of the last operand byte, 6502 style
			m_PC = target;
		}
		break;
	case 0xFC:															// JSR (abs,X) -- "new" stack, pointer from PBR
		{
			u16 ptr = (u16)( fetch16() + getX() );
			push16New( (u16)( m_PC - 1 ) );	// pushed BEFORE the pointer is read
			m_PC = (u16)( read8( ( (u32)m_PBR << 16 ) | ptr )
			     | ( (u16)read8( ( (u32)m_PBR << 16 ) | (u16)( ptr + 1 ) ) << 8 ) );
			applyE();
		}
		break;
	case 0x22:															// JSL long -- "new" stack, 3 bytes
		{
			u32 target = fetch24();
			push8New( m_PBR );				// pushed before the new bank is taken up
			push16New( (u16)( m_PC - 1 ) );
			m_PC  = (u16)( target & 0xFFFF );
			m_PBR = (u8)( target >> 16 );
			applyE();
		}
		break;

	case 0x60: m_PC = (u16)( pull16() + 1 ); break;						// RTS
	case 0x6B:															// RTL -- "new" stack, 3 bytes
		// The +1 is a 16-bit increment with NO carry into the bank: a stacked
		// $xx:FFFF resumes at $xx:0000. Mismatching JSL/RTL with JSR/RTS is
		// unrecoverable, which is why they are kept next to each other here.
		m_PC  = (u16)( pull16New() + 1 );
		m_PBR = pull8New();
		applyE();
		break;
	case 0x40:															// RTI -- "old" stack
		m_P = pull8();
		m_PC = pull16();
		if ( !m_E ) m_PBR = pull8();
		// RTI restores m and x from the stacked P -- and can therefore zero XH
		// and YH -- but it never restores E.
		applyE();
		break;

	// --- branches ----------------------------------------------------------
	case 0x10: W65_BRANCH( !( m_P & W65_N ) ) break;					// BPL
	case 0x30: W65_BRANCH(  ( m_P & W65_N ) ) break;					// BMI
	case 0x50: W65_BRANCH( !( m_P & W65_V ) ) break;					// BVC
	case 0x70: W65_BRANCH(  ( m_P & W65_V ) ) break;					// BVS
	case 0x90: W65_BRANCH( !( m_P & W65_C ) ) break;					// BCC
	case 0xB0: W65_BRANCH(  ( m_P & W65_C ) ) break;					// BCS
	case 0xD0: W65_BRANCH( !( m_P & W65_Z ) ) break;					// BNE
	case 0xF0: W65_BRANCH(  ( m_P & W65_Z ) ) break;					// BEQ
	case 0x80:															// BRA
		// Always taken, so its 3-cycle base already includes what the other
		// branches pay conditionally; only the emulation-mode page cross is
		// extra. That is why the table gives it W65C_PE rather than W65C_BR.
		W65_BRANCH( true )
		break;
	case 0x82:															// BRL
		{
			s16 disp = (s16)fetch16();
			m_PC = (u16)( m_PC + disp );	// wraps inside the program bank
		}
		break;

	// --- flags -------------------------------------------------------------
	case 0x18: m_P = (u8)( m_P & ~W65_C ); break;						// CLC
	case 0x38: m_P |= W65_C; break;										// SEC
	case 0x58: m_P = (u8)( m_P & ~W65_I ); break;						// CLI
	case 0x78: m_P |= W65_I; break;										// SEI
	case 0xB8: m_P = (u8)( m_P & ~W65_V ); break;						// CLV
	case 0xD8: m_P = (u8)( m_P & ~W65_D ); break;						// CLD
	case 0xF8: m_P |= W65_D; break;										// SED

	// REP and SEP can modify ANY status bit -- SEP #$04 is a legal, unidiomatic
	// SEI. In emulation mode they cannot touch m or x; applyE() enforces that at
	// the end of the instruction, and every other named bit still changes.
	case 0xC2: m_P = (u8)( m_P & ~fetch8() ); applyE(); break;			// REP #imm
	case 0xE2: m_P = (u8)( m_P |  fetch8() ); applyE(); break;			// SEP #imm

	case 0xFB:															// XCE
		{
			// The only way in or out of native mode -- not PLP, not RTI, not
			// REP/SEP. Entering emulation forces SH=$01 and DESTROYS XH and YH,
			// while B survives: narrowing the accumulator never loses data,
			// narrowing the index registers always does. applyE() does both.
			// D, DBR, PBR and PC are untouched by a mode change.
			const bool oldE = m_E;
			m_E = ( m_P & W65_C ) != 0;
			setFlag( W65_C, oldE );
			applyE();
		}
		break;

	// --- block moves -------------------------------------------------------
	// One byte per execution, with the PC parked on the opcode until the count
	// runs out. That is what makes a 64KB move interruptible: an interrupt taken
	// between bytes pushes the address of the MVN/MVP itself, and RTI re-enters
	// the instruction, picking up from X, Y and C. A handler must preserve all
	// three.
	//
	// THE TRAP: the operands appear in the object code as DESTINATION bank then
	// SOURCE bank, the reverse of how the assembler mnemonic reads. The
	// direction is also the opposite of WDC's names -- $54 "Block Move
	// Negative" increments. Go by increment/decrement and ignore both names.
	case 0x54:															// MVN
	case 0x44:															// MVP
		{
			const u8 dstBank = fetch8();
			const u8 srcBank = fetch8();
			const bool up = ( opcode == 0x54 );

			write8( ( (u32)dstBank << 16 ) | getY(),
			        read8( ( (u32)srcBank << 16 ) | getX() ) );

			// DBR is not consulted for addressing, but IS set to the
			// destination bank, destroying whatever it held.
			m_DBR = dstBank;

			if ( index8() )
			{
				// With 8-bit index registers the move is confined to page 0 of
				// each bank, because XH and YH are architecturally $00.
				// Practically useless, but this is what it does.
				m_X = (u16)( ( up ? ( m_X + 1 ) : ( m_X - 1 ) ) & 0xFF );
				m_Y = (u16)( ( up ? ( m_Y + 1 ) : ( m_Y - 1 ) ) & 0xFF );
			} else
			{
				m_X = (u16)( up ? ( m_X + 1 ) : ( m_X - 1 ) );
				m_Y = (u16)( up ? ( m_Y + 1 ) : ( m_Y - 1 ) );
			}

			// C is the byte count MINUS ONE, always the full 16 bits whatever M
			// says. C=0 moves one byte; C=$FFFF moves 65536.
			m_C = (u16)( m_C - 1 );

			if ( m_C != 0xFFFF )
				m_PC = (u16)( m_PC - 3 );		// park on the opcode and re-execute
		}
		break;

	// --- interrupts and control -------------------------------------------
	// BRK and COP skip their signature byte, so the pushed PC is the opcode
	// address plus two. Both execute regardless of I.
	case 0x00: fetch8(); serviceInterrupt( W65_VEC_NATIVE_BRK, W65_VEC_EMU_BRK, true ); break;
	case 0x02: fetch8(); serviceInterrupt( W65_VEC_NATIVE_COP, W65_VEC_EMU_COP, true ); break;

	case 0xDB: m_Stopped = true; break;									// STP
	case 0xCB: m_Waiting = true; break;									// WAI
	case 0xEA: break;													// NOP

	case 0x42:															// WDM
		// Reserved for future expansion and a plain TWO-byte no-op on every
		// chip WDC has shipped. Treating it as one byte desynchronises the
		// instruction stream. A WDM in a C64 program means the CPU is executing
		// data, which is worth knowing but is not ours to trap.
		fetch8();
		break;
	}

	#undef W65_BRANCH

	const u32 used = w65c816Cycles( opcode, m8Entry, x8Entry, eEntry,
	                                dpUnaligned, m_PageCross, m_BranchTaken );
	m_Cycles += used;
	m_Bus->tick( used );
	return used;
}

u64 CW65C816::run( u64 nCycles )
{
	const u64 start = m_Cycles;
	while ( m_Cycles - start < nCycles )
	{
		step();
		if ( m_Stopped )
			break;
	}
	return m_Cycles - start;
}
