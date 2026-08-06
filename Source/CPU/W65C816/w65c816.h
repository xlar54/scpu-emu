/*
   SCPU-EMU - CMD SuperCPU emulation for the C64/C128 using a RAD Expansion Unit
   Copyright (c) 2026 SCPU-EMU contributors

   WDC 65C816 core -- the processor a CMD SuperCPU actually contains.

   Two modes, and almost everything difficult follows from the difference:

     EMULATION (E=1)  What the machine comes up in, and what a C64 KERNAL runs
                      in. Behaves as a 6502: 8-bit registers, stack confined to
                      page 1, and status bits 4/5 acting as B and unused. In
                      this mode the core must agree with CM6502 instruction for
                      instruction, which is what Tests/CPU/test_w65c816_*.cpp
                      checks by running both over the same programs.

     NATIVE (E=0)     Entered with CLC/XCE. The M and X flags now independently
                      select 8- or 16-bit accumulator and index registers, the
                      stack is a full 16-bit pointer, and 24-bit addressing
                      reaches the SuperCPU's SuperRAM.

   The accumulator is held as one 16-bit register, m_C. In 8-bit mode only its
   low half (A) participates; the high half (B) is preserved and can be swapped
   in with XBA. Getting that preservation wrong is a classic source of bugs, so
   the 8/16-bit accessors below are the only supported way to touch it.

   Undocumented 6502 opcodes do NOT exist here. Those encodings are real 65816
   instructions -- which is exactly why a real SuperCPU cannot run software that
   depends on the 6502's illegal opcodes, and why CM6502 treats them as NOPs
   rather than emulating NMOS behaviour.

   Address wrapping lives in w65c816_addressing.h, with the rules documented.
   That file is where the subtlety is; read it before changing anything here.

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
#ifndef _w65c816_h
#define _w65c816_h

#include "../cpu.h"

class CSuperCPUMemoryMap;

// Status register. In emulation mode bits 4 and 5 behave as the 6502's B and
// unused bits; in native mode they select index and accumulator width.
#define W65_C 0x01	// carry
#define W65_Z 0x02	// zero
#define W65_I 0x04	// IRQ disable
#define W65_D 0x08	// decimal
#define W65_X 0x10	// native: index registers 8-bit.  emulation: break
#define W65_M 0x20	// native: accumulator 8-bit.      emulation: unused, reads 1
#define W65_V 0x40	// overflow
#define W65_N 0x80	// negative

#define W65_B W65_X	// the emulation-mode name for bit 4

// Native-mode vectors
#define W65_VEC_NATIVE_COP    0x00FFE4
#define W65_VEC_NATIVE_BRK    0x00FFE6
#define W65_VEC_NATIVE_ABORT  0x00FFE8
#define W65_VEC_NATIVE_NMI    0x00FFEA
#define W65_VEC_NATIVE_IRQ    0x00FFEE

// Emulation-mode vectors. Note BRK and IRQ share $FFFE, as on a 6502.
#define W65_VEC_EMU_COP       0x00FFF4
#define W65_VEC_EMU_ABORT     0x00FFF8
#define W65_VEC_EMU_NMI       0x00FFFA
#define W65_VEC_EMU_RESET     0x00FFFC
#define W65_VEC_EMU_IRQ       0x00FFFE
#define W65_VEC_EMU_BRK       0x00FFFE

// Bits of CW65C816::m_Pending. See the member note.
#define W65_PEND_STOPPED 1
#define W65_PEND_WAITING 2
#define W65_PEND_NMI     4
#define W65_PEND_SAMPLE  8
#define W65_PEND_IRQLINE 16

class CW65C816 : public ICpu
{
public:
	CW65C816();

	// --- ICpu -------------------------------------------------------------
	void attachBus( ICpuBus *bus ) override { m_Bus = bus; m_FastBus = 0; }

	// Attach the concrete memory map instead of the abstract bus. The static
	// type is what lets the compiler devirtualise and INLINE every memory
	// access, interrupt sample and tick into the interpreter -- through
	// ICpuBus* each of those is an indirect call, several per instruction,
	// and on an in-order A53 that indirection was one of the two dominant
	// costs in the whole emulator. Tests keep using attachBus() with their own
	// bus classes; the firmware uses this. Defined in w65c816_addressing.h,
	// where the map's full type is visible.
	void attachFastBus( CSuperCPUMemoryMap *map );
	void reset() override;
	u32  step() override;
	u32  stepNoTick();
	u64  run( u64 nCycles ) override;
	void requestRunBreak() override { m_RunBreak = true; }
	u64  cycles() const override { return m_Cycles; }
	scpu_addr_t pc() const override { return ( (u32)m_PBR << 16 ) | m_PC; }
	u16  stackPointer() const override { return m_S; }
	const char *name() const override { return "WDC 65C816"; }

	// --- register file, public for tracing and tests ----------------------
	u16 m_C;		// accumulator; A is the low byte, B the high byte
	u16 m_X, m_Y;	// index registers
	u16 m_S;		// stack pointer
	u16 m_D;		// direct page register
	u16 m_PC;
	u8  m_PBR;		// program bank
	u8  m_DBR;		// data bank
	u8  m_P;		// status
	bool m_E;		// emulation mode

	bool m_Stopped;	// STP executed; only RESET recovers
	bool m_Waiting;	// WAI executed; an interrupt resumes

	// --- register width ---------------------------------------------------
	// Emulation mode forces both to 8 bits regardless of the flag bits, so
	// these are the only correct way to ask.
	// always_inline on exactly these five: at -Os the compiler outlines them
	// into comdat functions that every OTHER outlined helper then calls --
	// setZNM became a 16-instruction body plus a call to memory8 plus a tail
	// call to setZN8. Forcing just these five makes the mid-level helpers
	// self-contained leaves, for +400 bytes of step(). Do NOT widen the set:
	// adding the accumulator/index accessors as well measured +13.2KB, which
	// blows the 32K L1I budget and reintroduces the refills -Os eliminated.
	__attribute__((always_inline)) inline bool memory8() const { return m_E || ( m_P & W65_M ) != 0; }
	__attribute__((always_inline)) inline bool index8()  const { return m_E || ( m_P & W65_X ) != 0; }

	// While E=1 the hardware CONTINUOUSLY forces SH=$01, m=1, x=1 and the high
	// bytes of X and Y to $00 -- writes to them simply do not take.
	//
	// This used to run after EVERY instruction, which was the safe way to write
	// it and cost 21% of the whole interpreter -- four read-modify-writes per
	// instruction to re-assert something almost nothing disturbs.
	//
	// It is now called only from the instructions that can actually break an
	// invariant, which is a much smaller set than it first appears:
	//
	//   X and Y   already cannot exceed $FF while index8() is true. Every site
	//             that writes them -- LDX/LDY, TAX/TAY, TSX, TXY/TYX, PLX/PLY,
	//             INX/DEX, MVN/MVP -- masks to 8 bits itself. Nothing to fix.
	//   S         push8/pull8 keep the "old" stack inside page 1 on their own,
	//             and TXS/TCS special-case emulation mode. Only the "new" stack
	//             class (PEA, PEI, PER, PHD, PLD, JSL, RTL, JSR (a,X)) leaves S
	//             outside page 1, by design, and needs putting back.
	//   P         only PLP, RTI, REP, SEP and XCE can clear m or x, or change E.
	//
	// So those thirteen opcodes call it, plus serviceInterrupt(). If one is ever
	// missed, the differential test catches it immediately: it asserts after
	// every single instruction that E still holds, S is inside page 1, and
	// neither index register exceeds $FF.
	inline void applyE()
	{
		if ( m_E )
		{
			m_P |= (u8)( W65_M | W65_X );
			m_S  = (u16)( 0x0100 | ( m_S & 0x00FF ) );
			m_X &= 0x00FF;
			m_Y &= 0x00FF;
		} else if ( m_P & W65_X )
		{
			// Narrowing the index registers DESTROYS their high bytes,
			// immediately and irrecoverably. Contrast the accumulator, where
			// narrowing preserves B -- getting these the same way round is the
			// classic 65816 bug.
			m_X &= 0x00FF;
			m_Y &= 0x00FF;
		}
	}

	// Accumulator halves. In 8-bit mode B must survive untouched.
	inline u8   getA() const     { return (u8)( m_C & 0xFF ); }
	inline void setA( u8 v )     { m_C = (u16)( ( m_C & 0xFF00 ) | v ); }
	inline u16  getAcc() const   { return memory8() ? (u16)( m_C & 0xFF ) : m_C; }
	inline void setAcc( u16 v )
	{
		if ( memory8() ) m_C = (u16)( ( m_C & 0xFF00 ) | ( v & 0xFF ) );
		else             m_C = v;
	}

	inline u16 getX() const { return index8() ? (u16)( m_X & 0xFF ) : m_X; }
	inline u16 getY() const { return index8() ? (u16)( m_Y & 0xFF ) : m_Y; }

	// Statistics.
	u64 m_NativeInstructions;
	u64 m_IRQsTaken = 0;
	u64 m_NMIsTaken = 0;

private:
	ICpuBus            *m_Bus;
	CSuperCPUMemoryMap *m_FastBus;

	// Every bus touch in the core goes through these. One perfectly-predicted
	// branch selects the inline fast path; the ICpuBus path stays for tests.
	inline u8   busRead8( u32 addr );
	inline void busWrite8( u32 addr, u8 v );
	inline bool busIRQ();
	inline bool busNMI();
	inline void busInterrupts( bool &irq, bool &nmi );
	inline bool busFineTicks();
	inline void busTick( u32 n );
	u64      m_Cycles;
	bool     m_RunBreak;
	bool     m_NMIPrevLevel;
	bool     m_NMIPending;

	// Pending-work byte: nonzero when the NEXT instruction needs the cold
	// prologue -- interrupt sample, NMI/IRQ dispatch, WAI, STP. Zero is the
	// common case inside a run() batch, where one byte load and branch
	// replaces ~47 instructions of per-instruction interrupt bookkeeping.
	// That bookkeeping is provably redundant there: the interrupt cache can
	// only refresh in tickFast, and run() ticks once per batch, so the
	// sampled lines cannot change between the batch's instructions. The
	// underlying members (m_Stopped, m_Waiting, m_NMIPending) stay
	// authoritative; this byte is derived state, rebuilt by rebuildPending().
	u8       m_Pending;

	void rebuildPending()
	{
		m_Pending = (u8)( ( m_Stopped    ? W65_PEND_STOPPED : 0 )
		                | ( m_Waiting    ? W65_PEND_WAITING : 0 )
		                | ( m_NMIPending ? W65_PEND_NMI     : 0 )
		                | W65_PEND_SAMPLE );
	}
	u32 stepInner();

	// Set during address calculation and branching, consumed by w65c816Cycles().
	bool m_PageCross;
	bool m_BranchTaken;

	// Set by the ea* helpers: true when the effective address is one of the
	// bank-0-confined modes (dp, dp,X, dp,Y, d,S), in which case a 16-bit
	// operand's second byte wraps at $FFFF rather than continuing into bank 1.
	// With D=$FF00 and m=0, LDA $FF takes its low byte from $00FFFF and its
	// HIGH byte from $000000.
	bool m_EABank0;

	// --- instruction stream, data access, stack (w65c816_addressing.h) -----
	inline u8   fetch8();
	inline u16  fetch16();
	inline u32  fetch24();

	inline u8   read8( u32 addr );
	inline void write8( u32 addr, u8 v );
	inline u16  read16( u32 addr );
	inline void write16( u32 addr, u16 v );
	inline u16  read16Bank0( u16 addr );
	inline u16  readDPPointer( u32 ptr );
	inline void rmwWrite( u32 addr, u16 original, u16 v );

	// "old" stack: confined to page 1 in emulation mode
	inline void push8( u8 v );
	inline u8   pull8();
	inline void push16( u16 v );
	inline u16  pull16();

	// "new" stack: 16-bit even in emulation mode. JSL, JSR (a,X), PEA, PEI,
	// PER, PHD, PLD and RTL only.
	inline void push8New( u8 v );
	inline u8   pull8New();
	inline void push16New( u16 v );
	inline u16  pull16New();

	// --- effective addresses (w65c816_addressing.h) -----------------------
	inline u32 eaDirect();
	inline u32 eaDirectIndexed( u16 index );
	inline u32 eaAbsolute();
	inline u32 eaAbsoluteIndexed( u16 index );
	inline u32 eaLong();
	inline u32 eaLongIndexed( u16 index );
	inline u32 eaIndirect();
	inline u32 eaIndirectX();
	inline u32 eaIndirectY();
	inline u32 eaIndirectLong();
	inline u32 eaIndirectLongY();
	inline u32 eaStackRelative();
	inline u32 eaStackRelativeIndirectY();

	// --- width-aware operand access ---------------------------------------
	// The second byte of a 16-bit operand follows m_EABank0, which the ea*
	// helper that produced the address has already set. Doing it that way rather
	// than passing a flag at every call site means a new addressing mode cannot
	// silently get the wrong wrap.
	inline u32 operandNext( u32 addr ) const
	{
		return m_EABank0 ? (u32)( (u16)( addr + 1 ) ) : (u32)( addr + 1 );
	}

	inline u16 read16Operand( u32 addr )
	{
		u16 lo = read8( addr );
		return (u16)( lo | ( (u16)read8( operandNext( addr ) ) << 8 ) );
	}

	inline void write16Operand( u32 addr, u16 v )
	{
		write8( addr, (u8)( v & 0xFF ) );
		write8( operandNext( addr ), (u8)( v >> 8 ) );
	}

	inline u16  readM( u32 addr ) { return memory8() ? (u16)read8( addr ) : read16Operand( addr ); }
	inline void writeM( u32 addr, u16 v )
	{
		if ( memory8() ) write8( addr, (u8)v ); else write16Operand( addr, v );
	}
	inline u16  readX( u32 addr ) { return index8() ? (u16)read8( addr ) : read16Operand( addr ); }
	inline void writeX( u32 addr, u16 v )
	{
		if ( index8() ) write8( addr, (u8)v ); else write16Operand( addr, v );
	}
	inline u16 immediateM() { return memory8() ? (u16)fetch8() : fetch16(); }
	inline u16 immediateX() { return index8()  ? (u16)fetch8() : fetch16(); }

	// --- flags ------------------------------------------------------------
	__attribute__((always_inline)) inline void setFlag( u8 mask, bool on ) { if ( on ) m_P |= mask; else m_P = (u8)( m_P & ~mask ); }
	__attribute__((always_inline)) inline void setZN8( u8 v )  { setFlag( W65_Z, v == 0 ); setFlag( W65_N, ( v & 0x80 ) != 0 ); }
	__attribute__((always_inline)) inline void setZN16( u16 v ){ setFlag( W65_Z, v == 0 ); setFlag( W65_N, ( v & 0x8000 ) != 0 ); }
	inline void setZNM( u16 v ) { if ( memory8() ) setZN8( (u8)v ); else setZN16( v ); }
	inline void setZNX( u16 v ) { if ( index8() )  setZN8( (u8)v ); else setZN16( v ); }

	// --- ALU --------------------------------------------------------------
	void opADC( u16 v );
	void opSBC( u16 v );
	void opCMP( u16 reg, u16 v, bool eightBit );
	void opBIT( u16 v, bool immediate );

	// --- interrupts -------------------------------------------------------
	void serviceInterrupt( u32 vectorNative, u32 vectorEmu, bool isBRKorCOP );
};

// Pulled in at the END, once CW65C816 is complete, so that including this
// header alone gives a fully defined class. The addressing header's own include
// of this one is then a no-op via the guard above.
#include "w65c816_addressing.h"

#endif
