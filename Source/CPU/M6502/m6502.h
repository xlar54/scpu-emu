/*
   SCPU-EMU - CMD SuperCPU emulation for the C64/C128 using a RAD Expansion Unit
   Copyright (c) 2026 SCPU-EMU contributors

   MOS 6502 / 6510 core.

   Scope: all 151 documented opcodes, including decimal-mode ADC/SBC (the C64
   KERNAL uses it in its time-of-day and BASIC float routines) and the JMP
   ($xxFF) page-wrap bug. Undocumented opcodes are treated as documented NOPs
   of the correct length -- which also matches the real SuperCPU, whose 65816
   implements "documented 6510 opcodes only" and reuses those encodings for
   65816 instructions.

   This core deliberately does not model the 6510's on-chip I/O port at $00/$01.
   Banking lives in Source/C64/banking, because on real hardware the SuperCPU's
   CPU is not the 6510 and the port has to be emulated outside the core.

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
#ifndef _m6502_h
#define _m6502_h

#include "../cpu.h"

// Processor status flags
#define M6502_C 0x01	// carry
#define M6502_Z 0x02	// zero
#define M6502_I 0x04	// IRQ disable
#define M6502_D 0x08	// decimal
#define M6502_B 0x10	// break (only ever set in the stacked copy)
#define M6502_U 0x20	// unused, reads as 1
#define M6502_V 0x40	// overflow
#define M6502_N 0x80	// negative

// Hardware vectors
#define M6502_VEC_NMI   0xFFFA
#define M6502_VEC_RESET 0xFFFC
#define M6502_VEC_IRQ   0xFFFE

class CM6502 : public ICpu
{
public:
	CM6502();

	// --- ICpu -------------------------------------------------------------
	void attachBus( ICpuBus *bus ) override { m_Bus = bus; }
	void reset() override;
	u32  step() override;
	u64  run( u64 nCycles ) override;
	void requestRunBreak() override { m_RunBreak = true; }
	u64  cycles() const override { return m_Cycles; }
	scpu_addr_t pc() const override { return m_PC; }
	const char *name() const override { return "MOS 6502"; }

	// --- register file, public for tracing and unit tests -----------------
	u8  m_A, m_X, m_Y, m_S, m_P;
	u16 m_PC;

	// True while the core is halted on a JAM/KIL encoding.
	bool m_Jammed;

	// NMOS read-modify-write behaviour: INC/DEC/ASL/LSR/ROL/ROR write the
	// *original* byte back before writing the modified one. Programs rely on
	// this against registers with write side effects -- the classic case is
	// INC $D019 to acknowledge a VIC-II interrupt, where the first write is
	// what actually performs the acknowledge.
	//
	// Default true, because this class is a faithful 6502/6510. Set false to
	// match a 65C816, which performs an internal cycle instead of the dummy
	// write; CSuperCPU does exactly that, since the accelerator's CPU is a
	// 65816 and emitting the extra write would both misrepresent it and cost a
	// real ~1us bus cycle on every RMW that targets I/O.
	bool m_RMWDummyWrite;

private:
	ICpuBus *m_Bus;
	u64      m_Cycles;
	bool     m_RunBreak;

	// NMI is edge triggered: remember the previous line level so that a level
	// held low does not re-trigger every instruction.
	bool m_NMIPrevLevel;
	bool m_NMIPending;

	// --- bus helpers ------------------------------------------------------
	inline u8   rd( u16 a )            { return m_Bus->read8( a ); }
	inline void wr( u16 a, u8 v )      { m_Bus->write8( a, v ); }
	inline u8   fetch()                { return rd( m_PC++ ); }
	inline u16  fetch16()              { u16 lo = fetch(); return lo | ( (u16)fetch() << 8 ); }
	inline u16  rd16( u16 a )          { return rd( a ) | ( (u16)rd( a + 1 ) << 8 ); }

	// Reads a vector honouring the 6502 page-wrap bug for indirect JMP.
	inline u16 rd16WrapPage( u16 a )
	{
		u16 hiAddr = ( a & 0xFF00 ) | ( ( a + 1 ) & 0x00FF );
		return rd( a ) | ( (u16)rd( hiAddr ) << 8 );
	}

	inline void push( u8 v )   { wr( 0x0100 | m_S, v ); m_S--; }
	inline u8   pop()          { m_S++; return rd( 0x0100 | m_S ); }
	inline void push16( u16 v ) { push( (u8)( v >> 8 ) ); push( (u8)( v & 0xFF ) ); }
	inline u16  pop16()        { u16 lo = pop(); return lo | ( (u16)pop() << 8 ); }

	// --- flag helpers -----------------------------------------------------
	inline void setZN( u8 v )
	{
		m_P = (u8)( ( m_P & ~( M6502_Z | M6502_N ) ) | ( v ? 0 : M6502_Z ) | ( v & M6502_N ) );
	}
	inline void setFlag( u8 mask, bool on )
	{
		if ( on ) m_P |= mask; else m_P = (u8)( m_P & ~mask );
	}

	// --- interrupt sequences ---------------------------------------------
	u32 serviceNMI();
	u32 serviceIRQ();

	// --- ALU --------------------------------------------------------------
	void opADC( u8 v );
	void opSBC( u8 v );
	void opCMP( u8 reg, u8 v );

	// Returns 1 when the effective address crossed a page relative to base.
	static inline u32 pageCrossed( u16 base, u16 eff )
	{
		return ( ( base ^ eff ) & 0xFF00 ) ? 1u : 0u;
	}
};

#endif
