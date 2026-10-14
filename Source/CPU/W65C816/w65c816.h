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
	// Cycles, signed, clamped by the config reader to -6..+6. Cached rather
	// than fetched per access: it is set once from scpu.cfg and never moves,
	// so the aim point pays one register read instead of a bus call.
	void setPreciseAim( int readCycles, int writeCycles ) override
	{
		m_PreciseAimRead  = (s8)readCycles;
		m_PreciseAimWrite = (s8)writeCycles;
	}
	// What this core is ACTUALLY using. Reported at startup so the log cannot
	// claim a calibration value that was never applied -- see the note at the
	// old call site in boot.cpp.
	int preciseAimRead() const override  { return m_PreciseAimRead; }
	int preciseAimWrite() const override { return m_PreciseAimWrite; }
	void setWriteLead( int quarterCycles ) override
	{
		m_WriteLeadQuarters = (s8)quarterCycles;
	}
	int writeLeadQuarters() const override { return m_WriteLeadQuarters; }

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

	// Per-opcode form of stepInner's precise-access-timeline predicate,
	// indexed by ( m_E << 8 ) | opcode. m_E is a bool, so the index cannot
	// leave the array.
	//
	// The element type is bool, NOT u8, and that is load-bearing: the
	// destination m_PreciseAccessTimeline is a bool, so a u8 table forces GCC
	// to emit a redundant `cmp w2,#0 / cset w3,ne` to normalise the byte
	// before storing it. MEASURED both ways in the shipped object with the
	// project toolchain: bool makes the predicate 4 instructions, u8 makes it
	// 6, and bool is 8 bytes smaller in stepInner. Do not "tidy" this to u8.
	//
	// Public ONLY so Tests/CPU/test_w65c816.cpp can enumerate it against the
	// expression it replaces; nothing else may read it. Defined in
	// w65c816.cpp, where the measured justification lives.
	static const bool s_PreciseForm[ 512 ];

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
	// Cached: recomputed by applyE() from E and P. Any direct write to m_P,
	// m_E or m_D outside the audited thirteen-opcode set must be followed by
	// applyE(), or the widths and the cycle key go stale -- the differential
	// suite asserts the widths after every instruction and catches a miss.
	__attribute__((always_inline)) inline bool memory8() const { return m_M8; }
	__attribute__((always_inline)) inline bool index8()  const { return m_X8; }

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

		// Refresh the cached widths and the cycle-table key. Every legitimate
		// writer of E, M, X or (via PLD) D already calls applyE; TCD updates
		// the key's dp bit itself.
		m_M8 = m_E || ( m_P & W65_M ) != 0;
		m_X8 = m_E || ( m_P & W65_X ) != 0;
		m_CycKey = (u8)( ( m_M8 ? 1 : 0 ) | ( m_X8 ? 2 : 0 )
		               | ( m_E ? 4 : 0 ) | ( ( ( m_D & 0xFF ) != 0 ) ? 8 : 0 ) );
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
	u64 m_NMIsDeferred = 0;
	// Compatibility option for software whose short native-mode windows are
	// timed around a CIA2 NMI. Hardware testing shows Winter Games needs the
	// pending edge released on return to emulation mode even with EPROM vector
	// rerouting enabled. This remains an approximation and is config-gated.
	bool m_DeferNativeNMI = false;

	// Emu-cycle stamp of the most recent NMI dispatch, for the stall
	// dump's delivery forensics.
	u64      m_LastNMITakeCycles = 0;

private:
	ICpuBus            *m_Bus;
	// Both zero on every normal build, which is what keeps the aim point's
	// arithmetic byte-identical to the pre-calibration expression.
	s8                  m_PreciseAimRead = 0;
	s8                  m_PreciseAimWrite = 0;
	s8                  m_WriteLeadQuarters = 0;	// K502, quarter cycles, see ICpu::setWriteLead
	CSuperCPUMemoryMap *m_FastBus;

	// Every bus touch in the core goes through these. One perfectly-predicted
	// branch selects the inline fast path; the ICpuBus path stays for tests.
	// always_inline on BOTH the declaration and the definition, matching
	// preciseTimelineBeforeAccess below -- GCC warns at every include site if
	// one declaration disagrees with another about inline-ness.
	//
	// MEASURED in the shipped object: these two have only THREE call sites in
	// the whole translation unit (fetch8, read8, write8 -- the relocation
	// census proves it), so inlining them duplicates the body twice and ZERO
	// times inside stepInner. At -Os GCC left busRead8 (0x144 bytes) and
	// busWrite8 (0x1dc) out of line with their own frames, while read8 and
	// write8 were mere 8-byte thunks that tail-branched to them. After this
	// edit both symbols disappear from the object and read8 absorbs the body,
	// losing a branch and a 48-byte frame with six stack memory ops from
	// every out-of-line access. A host census of the TI benchmark measured
	// 0.823 such accesses per emulated instruction, i.e. 29% of all bank-0
	// accesses still took this chain after only the OPCODE fetch was inlined.
	//
	// Do NOT also put always_inline on fetch8: that would inline this body at
	// fetch8's stepInner call sites and grow the hot function by ~2.3KB. This
	// edit gets most of the same win for ~200 bytes OUTSIDE stepInner.
	__attribute__((always_inline)) inline u8   busRead8( u32 addr );
	__attribute__((always_inline)) inline void busWrite8( u32 addr, u8 v );
	inline bool busIRQ();
	inline bool busNMI();
	inline void busInterrupts( bool &irq, bool &nmi );
	inline bool busFineTicks();
	inline u32  busPreciseCycleAccess( u32 addr, bool write );
	inline void busTick( u32 n );
	inline void settleBusThrough( u64 logicalCycle );
	inline void busPhaseTrim( s32 c64Cycles );
	// Split so the gate inlines and the body does not: busRead8 called this
	// out of line, so the opcode fetch paid the call setup plus the callee's
	// own ldrb/tbz/ret to read one almost-always-false bool. Measured in the
	// K449-era object: preciseTimelineBeforeAccess is a 0x74-byte weak symbol
	// whose first two instructions are that gate, and busRead8+0x1c is a `bl`
	// to it. Same shape as CC64Memory::dosExtensionMapsBank1 and its noinline
	// state helper, for the same measured reason. The predicate itself still
	// runs first and in the same place; only the cost of reaching it changes.
	__attribute__((always_inline)) inline void preciseTimelineBeforeAccess( u32 addr, bool write );
	void preciseTimelineBeforeAccessArmed( u32 addr, bool write );
	u64      m_Cycles;
	// Logical cycle through which ICpuBus::tick has already been paid. A
	// physical VIC access can settle part of the CURRENT instruction; the
	// ordinary instruction/batch epilogue then pays only the remainder.
	u64      m_BusSettledCycles;
	bool     m_RunBreak;
	bool     m_NMIPrevLevel;
	bool     m_NMIPending;
	bool     m_M8, m_X8;	// cached widths; see memory8()/index8()
	u8       m_CycKey;		// cached index into w65c816CycleLUT

	// Exact access sequence for the bus-only absolute forms used by VSP code.
	// Cycle 1 is the opcode fetch; fetch8/read8/write8 advance the remaining
	// accesses. Other forms stay on end-of-instruction timing until their
	// internal cycles have an explicit schedule.
	u8       m_PreciseAccessCycle;
	bool     m_PreciseAccessTimeline;

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

	// Emulation-mode fast-path helpers -- always-inline twins of the generic
	// accessors, defined in w65c816.cpp above stepInner(). See the fast
	// switch there for why they exist.
	inline u8   f8E();
	inline u8   rd8E( u32 addr );
	inline void wr8E( u32 addr, u8 v );
	inline void pushE8( u8 v );
	inline u8   pullE8();
	inline void znE( u8 v );
	inline u32  emuDoneE( u32 used );
	void opBIT( u16 v, bool immediate );

	// --- interrupts -------------------------------------------------------
	void serviceInterrupt( u32 vectorNative, u32 vectorEmu, bool isBRKorCOP );

public:
	// Interrupt-entry forensics, for Mario's double-IRQ stabiliser at $0813
	// (disassembly: /c/tmp/mario-irq.txt).  The chain re-points $FFFE to $0846
	// and falls into a 24-NOP sled at $082E-$0845 where the SECOND raster IRQ
	// is meant to land; $0846 TXS restores the S that $082C TSX saved, and X is
	// destroyed two instructions later by $0847 LDX #$07 / $0849 DEX / $084A
	// BNE.  So an entry at $0846 or $0847 is harmless (X still holds the TSX
	// value) but an entry anywhere in $0849-$0870 sets S = $0100 + X with X in
	// 0..7, and the RTI at $0897 then pops from $0101.. and returns to garbage.
	// Nothing we record today can tell those two cases apart: the entry PC (the
	// sled index) and the pre-push S are the whole question.
	//
	// Deliberately placed LAST in the class so that no existing member's offset
	// moves.  The hot members (m_Cycles, m_P, m_S, the register file) keep the
	// byte offsets the -Ofast AArch64 code was generated against, which is what
	// makes the "throughput must not move" check on this change meaningful.
	// MEASURED with this placement: stepInner goes 0x6468 -> 0x6484 and every
	// one of the 25 differing instruction lines out of 6,428 lands between the
	// busInterrupts call and the first preciseTimelineBeforeAccessArmed call --
	// the cold `if ( m_Pending )` prologue.  The dispatch path is identical.
	//
	// Pure sidecar: it reads emulated state and writes only its own storage.
	struct W65InterruptEntry
	{
		u32 pc;		// ( PBR << 16 ) | PC at entry, before any push
		u16 s;		// S BEFORE the interrupt's pushes
		u8  p;
		u8  nmi;	// 1 = NMI, 0 = IRQ
		// Low 32 bits of m_Cycles.  These are CPU-rate cycles, so they are C64
		// cycles only while the guest is at 1MHz -- true inside Mario's
		// stabiliser, which $0819 STA $D07A puts in software 1MHz, and false
		// for the turbo-mode entries either side of it.
		u32 cycles;
		// The handler this interrupt actually ENTERED, filled after the vector
		// fetch.  The 2026-10-12 capture could not say which rung of Mario's
		// chain was running: the ring records the PC we LEFT, and the main loop
		// looks identical whether $FFFE held $0813, $0898 or $0991.  Mario
		// re-points $FFFE three times per frame, so without this field a dead
		// rung and a live one produce the same 64 lines.  $9F88 is written at
		// $09B0, inside the $0991 rung; "that rung never ran" and "it ran and
		// wrote the same value" are different bugs, and this is the field that
		// separates them.
		u16 target;
		// --- WHY THIS INTERRUPT HAPPENED, as far as it can be seen from
		// here, and no further.
		//
		// The 2026-10-12 capture established an EARLIER, NESTED entry (gap
		// 1228 cycles against a healthy 6547, S=$01F9 against $01FF) and
		// nothing beyond that: four target addresses prove vector rotation,
		// not interrupt-source identity. These four fields are the state the
		// ring could not see for itself, filled by one cold call into the
		// memory layer per interrupt -- no bus cycle, no GPIO read, and no CIA
		// access, because $DC0D and $DD0D clear on read.
		//
		// READ THEM AS A SET:
		//   raster within a line or two of compare, rasterAge4 small
		//                        the beam really was at the armed compare, so
		//                        the VIC's raster compare is the source and it
		//                        was armed where we think.
		//   raster far from compare, d01a = $01
		//                        a raster-only mask cannot explain this entry.
		//                        Either the latch from the PREVIOUS match was
		//                        never cleared, or something that is not the
		//                        VIC is on the shared /IRQ line; iflags'
		//                        ACK_HELD bits say which of those the $D019
		//                        acknowledgement test saw.
		//   FRESH_SAMPLE clear   the level that let this interrupt in was the
		//                        cached one, not a sample taken at this
		//                        instruction boundary. That is the stale-level
		//                        candidate, tested rather than bounded.
		u16 compare;		// the compare armed at this entry, OUR decode
		u8  d01a;			// the VIC interrupt-enable mask then
		u8  raster;			// guest-observed $D012 low byte
		u8  rasterAge4;		// its age in C64 cycles / 4; $FF = no anchor
		u8  iflags;			// SCPU_IRQCTX_* from CC64Memory, plus the core's
							// own FRESH_SAMPLE bit
	};
	// 64, not 16: Mario runs a multi-rung raster chain ($0813, $0846, $0898 and
	// the $09B0 rung), so 16 slots is two or three frames -- less than one pass
	// of the eight-phase YSCROLL table at $A365 that the scroll gate has to be
	// read against, and too short to hold any history before a death.
	static const u32 W65_INT_RING = 64;
	W65InterruptEntry m_IntRing[ W65_INT_RING ] = {};
	u32 m_IntRingHead = 0;
	u32 m_IntRingFrozen = 0;
	u32 m_StackAnomalies = 0;
	u32 m_IntRingFrozenCycles = 0;

	// The freeze is one-shot: once the stack pointer is destroyed the machine
	// keeps taking interrupts through $FFFE, so without it the entries LEADING
	// UP TO the death are recycled long before anyone can press the RAD button.
	//
	// $0110, not $0120: the trip is irreversible for the rest of the session,
	// so a false positive during loading would silence the instrument for the
	// run that matters.  $0110 still sits eight bytes clear of the whole
	// predicted signature, $0100..$0107 -- and $0100 is IN that set, it is the
	// value after the DEX loop has run to zero, which is the widest window of
	// the three.  m_IntRingFrozenCycles is what makes a spurious trip
	// self-identifying: compared against the live cycle count in the
	// post-mortem it says whether the freeze happened just now or minutes ago
	// during the load, in which case the run is void rather than a finding.
	//
	// MEASURED: GCC 13 at -Ofast emits this OUT OF LINE anyway (two
	// R_AARCH64_CALL26 sites in stepInner's prologue). noinline makes that
	// deterministic, so no future optimiser decision can inline 1KB of ring
	// addressing into stepInner and move the hot path's register allocation.
	__attribute__((noinline)) void noteInterruptEntry( bool nmi )
	{
		if ( m_IntRingFrozen ) return;
		W65InterruptEntry &e = m_IntRing[ m_IntRingHead++ % W65_INT_RING ];
		e.pc = ( (u32)m_PBR << 16 ) | m_PC;
		e.s = m_S;
		e.p = m_P;
		e.nmi = nmi ? 1 : 0;
		e.cycles = (u32)m_Cycles;
		m_IntRingTargetDue = 1;
		if ( m_E && m_S < 0x0110 )
		{
			m_StackAnomalies++;
			m_IntRingFrozen = 1;
			m_IntRingFrozenCycles = (u32)m_Cycles;
		}
	}

	// Called immediately after serviceInterrupt(), which is where m_PC first
	// holds the vector's destination.  Separate from the entry hook because the
	// vector fetch happens after the pushes, and gated on m_IntRingTargetDue
	// rather than on !m_IntRingFrozen so that the entry which TRIPPED the
	// freeze still gets its target: that entry is the most interesting one in
	// the ring.
	// Fills the four context fields above on the entry noteInterruptEntry
	// just wrote. Out of line in w65c816.cpp, not here, for two reasons: it
	// needs the complete CSuperCPUMemoryMap type, and keeping it out of this
	// header keeps stepInner's register allocation -- which is the throughput
	// claim this file's other notes are all about -- untouched. Gated on
	// m_IntRingTargetDue, so a frozen ring is not written to, exactly like
	// noteInterruptTarget below.
	__attribute__((noinline)) void noteInterruptContext( bool freshSample,
	                                                     bool carriedLine );

	__attribute__((noinline)) void noteInterruptTarget()
	{
		if ( !m_IntRingTargetDue ) return;
		m_IntRingTargetDue = 0;
		m_IntRing[ ( m_IntRingHead - 1 ) % W65_INT_RING ].target = m_PC;
	}

	// --- hypothesis-free hang detector ------------------------------------
	//
	// WHY THIS EXISTS, and it is a lesson about instruments and not about
	// Mario.  The stack-anomaly trip above was derived FROM the hypothesis it
	// was built to test: S < $0110, the signature of $0849 overrunning the
	// sled.  It catches that hang perfectly.  On 2026-10-12, with that hang
	// fixed, the machine hung again in front of a detector that reported
	// stack-anomalies=0: S was $01FA, the stack held two ordinary return
	// addresses, I was 0 and 5,658 interrupts had been taken.  A detector
	// derived from a hypothesis can only ever confirm that hypothesis.  So this
	// one is derived from the DEFINITION of a hang instead: forward progress
	// stopped.  It knows nothing about stacks, vectors, rasters or $9F88.
	//
	// WHY "THE PC SET STOPPED GROWING" AND NOT THE OTHER THREE CANDIDATES.
	// Each of the obvious alternatives is already falsified by the 2026-10-12
	// capture, which is the only honest way to choose between them:
	//   * PC confined to a small window -- FALSE on this hang.  The raster
	//     chain keeps running, so the PCs span $0813..$A768, and any window
	//     test loose enough to fire here also fires on healthy code.
	//   * no VIC register written for a frame -- FALSE.  The chain writes
	//     $D011, $D016, $D018 and $D019 every frame throughout the hang.
	//   * a frame-rate watchdog -- FALSE.  Frames keep happening; the raster
	//     chain is the part that still works.
	// What is true of both hangs, and of every hang, is that the set of code
	// addresses the machine executes stops acquiring new members.  A healthy
	// game reaches new code constantly (new enemies, new states, new
	// branches); a hung one re-executes a fixed cycle forever.  That is the one
	// property this detector measures.
	//
	// ZERO THRESHOLD, which is the point.  Each epoch counts the code cells
	// executed that were NOT executed in the previous epoch.  The trip is
	// novel == 0 -- not "novel is small", which would need a calibrated number
	// and would smuggle a hypothesis straight back in.
	//
	// 16-BYTE CELLS over bank 0's 64K: 4096 bits, 512 bytes per generation, two
	// generations.  The bank is deliberately ignored.  Every PC in this
	// investigation is in bank 0, and a bank-$F8 PC aliases onto a bank-0 cell,
	// which MERGES cells and so can only ever reduce the novelty count.  That
	// direction is a false negative (the detector stays quiet), never a false
	// positive, so the aliasing cannot manufacture a finding.
	//
	// SAMPLED PER BATCH, NOT PER INSTRUCTION, and that is a cost decision with
	// numbers behind it.  The bitmap update is ~6 ARM instructions; per
	// instruction, against the ~49 ARM per emulated instruction this core
	// currently costs, that is an 8-12% throughput loss AND it recompiles
	// stepInner, which is the byte-identity proof the whole "turbo is
	// untouched" argument rests on.  Sampled from run()'s batch loop it costs
	// ~0.1 ARM per instruction and stepInner is not recompiled at all.  The
	// price is that the footprint is a SUBSAMPLE of the true PC set -- which is
	// exactly why the novelty test compares against the previous epoch's
	// accumulated bitmap instead of demanding an identical footprint: sampling
	// drift adds cells early and then stops, where an exact-match test would be
	// broken by it forever and the detector would never fire.
	//
	// TWO STAGES, because the two things we want are in tension.  Freezing
	// early catches the TRANSITION into the hang; freezing late is the only way
	// to stop a false positive silencing the instrument for the whole run, and
	// a disk load does sit in a tight loop for seconds.  So: SUSPECT at 2
	// epochs (~0.35s) copies the ring aside, which a loader can do harmlessly
	// as often as it likes because every episode overwrites the same scratch
	// copy; CONFIRM at 48 epochs (~8.4s of unbroken zero novelty) freezes the
	// live ring and latches that copy.  A hang is permanent, so waiting 8s to
	// confirm costs nothing; a load is not, so it never reaches the confirm.
	// The post-mortem then carries both the transition and the steady state.
	static const u32 W65_PROG_CELLS = 4096;			// 16-byte cells over 64K
	static const u32 W65_PROG_WORDS = W65_PROG_CELLS / 64;
	// 65536 batch TICKS, of which one in eight is actually sampled: ~0.5M
	// emulated instructions, ~0.17s at the 3.0M instructions/s the current
	// benchmark measures, and 8192 PC samples per epoch.
	static const u32 W65_PROG_EPOCH = 1u << 16;
	static const u32 W65_PROG_SUSPECT_EPOCHS = 2;
	static const u32 W65_PROG_CONFIRM_EPOCHS = 48;
	static const u32 W65_PROG_CELL_REPORT = 64;

	u64 m_ProgCur[ W65_PROG_WORDS ] = {};
	u64 m_ProgPrev[ W65_PROG_WORDS ] = {};
	u32 m_ProgSamples = 0;
	u32 m_ProgNovel = 0;
	u32 m_ProgEpochs = 0;
	u32 m_ProgStallEpochs = 0;
	u32 m_ProgStallEpochsMax = 0;
	u32 m_ProgSuspects = 0;			// episodes that reached SUSPECT
	u32 m_ProgConfirmed = 0;
	u32 m_ProgSuspectCycles = 0;
	u32 m_ProgConfirmCycles = 0;
	u32 m_ProgCellCount = 0;		// cells in the confirming epoch
	u16 m_ProgCells[ W65_PROG_CELL_REPORT ] = {};
	u16 m_ProgPC = 0;				// PC at the confirming sample
	W65InterruptEntry m_IntRingSuspect[ W65_INT_RING ] = {};
	u32 m_IntRingSuspectHead = 0;
	u8  m_ProgEnable = 1;
	u8  m_IntRingTargetDue = 0;

	// Called from run()'s batch loop -- NOT from step() or stepNoTick(), which
	// are the test and single-step entries, so the host differential suite is
	// unaffected by construction.
	//
	// THE SUBSAMPLE IS A MEASURED TRADE, not caution.  Taking the bitmap on
	// every batch costs 15 ARM instructions in the shipped run() (disassembled:
	// enable load, cbz, the PC load, two ubfx, add, lsl, ldr, tst, branch, then
	// the counter's ldr/add/str/cmp/branch) -- 1.9 ARM per emulated
	// instruction against the ~49 this core costs, so ~3.8%, enough to move a
	// reported TI by a whole point and pollute a throughput series that is
	// load-bearing in CHT.  Gating the bitmap on the counter's low three bits
	// leaves 7 instructions on the common path, ~0.9 ARM per emulated
	// instruction, and still takes 8192 PC samples per epoch.  A hang's cycle
	// is a handful of cells and gets covered many times over; what a coarser
	// sample risks is MISSING a rarely-executed cell in a healthy epoch, and
	// that error has to repeat for 48 consecutive epochs to reach a confirm.
	// With HANG_WATCH 0 the whole thing is two instructions (ldrb, cbz), which
	// is the clean A/B for any TI measurement.
	inline void noteProgressSample()
	{
		if ( !m_ProgEnable ) return;
		if ( ++m_ProgSamples & 7 ) return;
		const u32 cell = (u32)m_PC >> 4;
		const u32 word = cell >> 6;
		const u64 bit = 1ULL << ( cell & 63 );
		if ( !( m_ProgCur[ word ] & bit ) )
		{
			m_ProgCur[ word ] |= bit;
			if ( !( m_ProgPrev[ word ] & bit ) ) m_ProgNovel++;
		}
		if ( m_ProgSamples >= W65_PROG_EPOCH ) rollProgressEpoch();
	}

	__attribute__((noinline)) void rollProgressEpoch()
	{
		m_ProgSamples = 0;
		m_ProgEpochs++;
		if ( m_ProgNovel )
			m_ProgStallEpochs = 0;
		else
		{
			m_ProgStallEpochs++;
			if ( m_ProgStallEpochs > m_ProgStallEpochsMax )
				m_ProgStallEpochsMax = m_ProgStallEpochs;
			if ( m_ProgStallEpochs == W65_PROG_SUSPECT_EPOCHS
			     && !m_ProgConfirmed )
			{
				m_ProgSuspects++;
				m_ProgSuspectCycles = (u32)m_Cycles;
				for ( u32 i = 0; i < W65_INT_RING; i++ )
					m_IntRingSuspect[ i ] = m_IntRing[ i ];
				m_IntRingSuspectHead = m_IntRingHead;
			}
			// Report the footprint BEFORE the generation flip below, because
			// m_ProgCur is the stalled epoch's own cell set -- the list of code
			// the machine is still executing.  That list is the most
			// informative thing a hang post-mortem can carry and reading it
			// needs no hypothesis at all.
			if ( m_ProgStallEpochs == W65_PROG_CONFIRM_EPOCHS )
			{
				m_ProgConfirmed = 1;
				m_ProgConfirmCycles = (u32)m_Cycles;
				m_ProgPC = m_PC;
				m_ProgCellCount = 0;
				for ( u32 i = 0; i < W65_PROG_WORDS; i++ )
				{
					u64 w = m_ProgCur[ i ];
					while ( w )
					{
						const u32 b = (u32)__builtin_ctzll( w );
						w &= w - 1;
						if ( m_ProgCellCount < W65_PROG_CELL_REPORT )
							m_ProgCells[ m_ProgCellCount ] =
								(u16)( ( i << 6 ) | b );
						m_ProgCellCount++;
					}
				}
				m_IntRingFrozen = 1;
				if ( !m_IntRingFrozenCycles )
					m_IntRingFrozenCycles = (u32)m_Cycles;
			}
		}
		for ( u32 i = 0; i < W65_PROG_WORDS; i++ )
		{
			m_ProgPrev[ i ] = m_ProgCur[ i ];
			m_ProgCur[ i ] = 0;
		}
		m_ProgNovel = 0;
	}
};

// Pulled in at the END, once CW65C816 is complete, so that including this
// header alone gives a fully defined class. The addressing header's own include
// of this one is then a no-op via the guard above.
#include "w65c816_addressing.h"

#endif
