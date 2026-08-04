/*
   SCPU-EMU - CMD SuperCPU emulation for the C64/C128 using a RAD Expansion Unit
   Copyright (c) 2026 SCPU-EMU contributors

   WDC 65C816 effective-address calculation and stack access.

   This is where 65816 emulators go wrong, so it lives in its own file with the
   reasoning attached. The wrapping rules are NOT uniform across addressing
   modes -- each one wraps differently, several change behaviour between
   emulation and native mode, and a 6502 exercises none of it. Getting one wrong
   produces software that works for months and then corrupts memory in one
   specific program.

   Verified against VICE's 65816core.c, the WDC datasheet, Bruce Clark's
   "65C816 Opcodes" and SingleStepTests/65816. The full derivation, including
   what is measured and what is still open, is in
   Docs/research/65816-reference.md section 2.

   THE MASTER RULE -- three address classes, and every mode belongs to one:

     Bank 0, 16-bit wrap   the direct page; the stack; the POINTER of every
                           indirect mode; the pointer of JMP (a) and JMP [a]
     Bank K, 16-bit wrap   instruction fetches; branch targets; PER; the
                           POINTER of JMP/JSR (a,X)
     24-bit, no wrap       the DATA of every mode -- absolute, absolute indexed,
                           long, and the resolved target of every indirect

   PBR supplies the bank for instruction fetches only. DBR supplies the bank for
   data. Direct page, stack and pointer fetches ignore both. Both bank registers
   are live in emulation mode -- E=1 does not force them to zero.

   THE RULES, each with the trap it sets:

   Direct page, dp
       ea = (operand + D) & 0xFFFF, always in bank 0.
       Trap: it wraps within bank 0, not within a page, and never leaves bank 0
       however large D is.

   Direct page indexed, dp,X and dp,Y
       In EMULATION mode with D page-aligned (D & 0xFF == 0):
           ea = ((operand + index) & 0xFF) + D
       i.e. it wraps at $FF exactly like a 6502. Otherwise:
           ea = (operand + index + D) & 0xFFFF
       Trap: the same instruction wraps two different ways depending on mode and
       on the low byte of D. This is the single most commonly botched rule. All
       three conditions matter -- E=1 AND DL=$00 AND a mode a 65C02 also had.

   Indirect pointers, (dp), (dp,X), (dp),Y
       The pointer is read from bank 0, and its HIGH BYTE also wraps inside the
       page under the same E=1, DL=$00 condition -- see W65C816_DP_POINTER_WRAP
       below, which is the one genuinely unresolved question in this file.

   Indirect long, [dp] and [dp],Y, and PEI
       Explicitly EXCLUDED from the page wrap by the datasheet: these run off the
       end of the direct page into whatever follows. The bank comes from the
       pointer, not DBR.

   Absolute, abs
       ea = operand | (DBR << 16). The DATA bank, not the program bank.

   Absolute indexed, abs,X and abs,Y
       ea = (operand + index + (DBR << 16)) & 0xFFFFFF
       Trap: this CARRIES INTO THE NEXT BANK, in BOTH modes. Indexing off the top
       of a bank silently continues into the one above, which no 6502 can do.
       This is the only addressing difference between emulation mode and a real
       6502, so it is exempt from the differential test rather than "fixed".

   Absolute long, long and long,X
       ea = full 24-bit operand (+ index), no wrap. The bank comes from the
       instruction, so DBR is ignored. There is no long,Y.

   Stack relative, sr and (sr),Y
       ea = (operand + S) & 0xFFFF, always bank 0. These are "new" modes and are
       NEVER confined to page 1, even in emulation. The indirect form then
       applies DBR and may cross a bank.

   Program counter
       Execution wraps WITHIN the program bank: the PC is 16-bit and PBR does
       not increment when it rolls over, mid-instruction included. Branches, BRL
       and PER all wrap in bank K too.

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
#ifndef _w65c816_addressing_h
#define _w65c816_addressing_h

#include "w65c816.h"
#include "../../SuperCPU/memory_map.h"

// The fast-bus plumbing. See attachFastBus() in w65c816.h for why this exists.
inline void CW65C816::attachFastBus( CSuperCPUMemoryMap *map )
{
	m_FastBus = map;
	m_Bus     = map;
}

inline u8 CW65C816::busRead8( u32 addr )
{
	return m_FastBus ? m_FastBus->read8( addr ) : m_Bus->read8( addr );
}

inline void CW65C816::busWrite8( u32 addr, u8 v )
{
	if ( m_FastBus ) m_FastBus->write8( addr, v ); else m_Bus->write8( addr, v );
}

inline bool CW65C816::busIRQ()
{
	return m_FastBus ? m_FastBus->irqAsserted() : m_Bus->irqAsserted();
}

inline bool CW65C816::busNMI()
{
	return m_FastBus ? m_FastBus->nmiAsserted() : m_Bus->nmiAsserted();
}

inline void CW65C816::busTick( u32 n )
{
	if ( m_FastBus ) m_FastBus->tick( n ); else m_Bus->tick( n );
}

// UNRESOLVED (U1 in Docs/research/65816-reference.md). With E=1 and DL=$00,
// does the pointer high byte of (dp), (dp,X) and (dp),Y wrap inside the direct
// page? LDA ($FF),Y -- is the high byte at $0000 or $0100?
//
// Wrap:    WDC datasheet 8.2.1 (which lists [d], [d],Y and PEI as the ONLY
//          exceptions), Bruce Clark 5.11, and VICE, which special-cases it
//          deliberately in INDIRECT_X_FUNC and INDIRECT_Y_FUNC.
// No wrap: SingleStepTests -- but that suite is emulator-generated rather than
//          a hardware capture, and it produced exactly one distinguishing case.
//
// Three sources including WDC's own exception list beat one generator, and the
// wrap is also what keeps EA_IZX/EA_IZY in CM6502 agreeing with us for the
// differential test. It is a one-line difference nobody would notice, so it is
// named here: if a SuperCPU ever meets a logic analyser, this is the flag to
// flip, and Tests/CPU/test_w65c816_addressing.cpp pins the current choice.
#define W65C816_DP_POINTER_WRAP 1

// --- instruction stream ----------------------------------------------------
// Fetches come from the program bank and the PC wraps within it.

inline u8 CW65C816::fetch8()
{
	u8 v = busRead8( ( (u32)m_PBR << 16 ) | m_PC );
	m_PC = (u16)( m_PC + 1 );		// wraps within the bank; PBR unchanged
	return v;
}

inline u16 CW65C816::fetch16()
{
	u16 lo = fetch8();
	return (u16)( lo | ( (u16)fetch8() << 8 ) );
}

inline u32 CW65C816::fetch24()
{
	u32 lo = fetch16();
	return lo | ( (u32)fetch8() << 16 );
}

// --- data access -----------------------------------------------------------

inline u8 CW65C816::read8( u32 addr )
{
	return busRead8( addr & SCPU_ADDR_MASK );
}

inline void CW65C816::write8( u32 addr, u8 v )
{
	busWrite8( addr & SCPU_ADDR_MASK, v );
}

// 16-bit data accesses carry into the next byte with 24-bit wrap, so a word at
// $12FFFF continues at $130000. Data is never bank-confined.
inline u16 CW65C816::read16( u32 addr )
{
	u16 lo = read8( addr );
	return (u16)( lo | ( (u16)read8( addr + 1 ) << 8 ) );
}

inline void CW65C816::write16( u32 addr, u16 v )
{
	write8( addr, (u8)( v & 0xFF ) );
	write8( addr + 1, (u8)( v >> 8 ) );
}

// Bank 0 accessor, for pointers and vectors, which never leave it.
inline u16 CW65C816::read16Bank0( u16 addr )
{
	u16 lo = read8( addr );
	return (u16)( lo | ( (u16)read8( (u16)( addr + 1 ) ) << 8 ) );
}

// The direct-page pointer read used by (dp), (dp,X) and (dp),Y. Differs from
// read16Bank0 only in the high byte's wrap -- see W65C816_DP_POINTER_WRAP.
inline u16 CW65C816::readDPPointer( u32 ptr )
{
	u8 lo = read8( (u16)ptr );

	u16 hiAddr;
#if W65C816_DP_POINTER_WRAP
	if ( m_E && !( m_D & 0xFF ) )
		hiAddr = (u16)( ( m_D & 0xFF00 ) | ( ( ptr + 1 ) & 0xFF ) );
	else
#endif
		hiAddr = (u16)( ptr + 1 );

	return (u16)( lo | ( (u16)read8( hiAddr ) << 8 ) );
}

// The read-modify-write tail, which is NOT one behaviour but two:
//
//   E=1        read, write, write -- the original byte goes back first. WDC kept
//              the NMOS dummy write in emulation mode deliberately, because a
//              drop-in 6502 replacement has to do it. This is what makes
//              INC $D019 and LSR $D019 acknowledge VIC-II interrupts: the write
//              of the ORIGINAL value has bit 0 set and clears the latch. Without
//              it a raster handler re-enters forever -- a hang, not a cosmetic
//              difference.
//   E=0, m=1   read, read, write -- an internal cycle instead.
//   E=0, m=0   the two writes go HIGH BYTE FIRST, the reverse of the reads.
//              Only observable against I/O, but that is where it matters.
inline void CW65C816::rmwWrite( u32 addr, u16 original, u16 v )
{
	if ( m_E )
	{
		write8( addr, (u8)original );
		write8( addr, (u8)v );
		return;
	}

	if ( memory8() )
	{
		write8( addr, (u8)v );
		return;
	}

	write8( operandNext( addr ), (u8)( v >> 8 ) );
	write8( addr, (u8)( v & 0xFF ) );
}

// --- stack -----------------------------------------------------------------
// Emulation mode has TWO stack behaviours, and which one applies is a per
// instruction property, not a per-access one. The datasheet names the "new"
// set explicitly: JSL, JSR (a,X), PEA, PEI, PER, PHD, PLD, RTL. Everything a
// 65C02 had -- every PH*/PL*, JSR abs, RTS, RTI, BRK -- plus COP and all
// hardware interrupts is "old".
//
//   old   confined to page 1: only SL moves, so PHA at S=$0100 writes $000100
//         and leaves S=$01FF.
//   new   a full 16-bit pointer for the duration: PEA at S=$0100 writes $000100
//         and then $0000FF, running off the bottom of the page.
//
// Either way applyE() puts S back inside page 1 at the end of the instruction,
// which is why the final S is identical for both classes.
//
// Native mode has one form: a full 16-bit pointer, always in bank 0.

inline void CW65C816::push8( u8 v )
{
	write8( m_S, v );
	m_S = m_E ? (u16)( 0x0100 | ( ( m_S - 1 ) & 0xFF ) ) : (u16)( m_S - 1 );
}

inline u8 CW65C816::pull8()
{
	m_S = m_E ? (u16)( 0x0100 | ( ( m_S + 1 ) & 0xFF ) ) : (u16)( m_S + 1 );
	return read8( m_S );
}

inline void CW65C816::push16( u16 v )
{
	push8( (u8)( v >> 8 ) );
	push8( (u8)( v & 0xFF ) );
}

inline u16 CW65C816::pull16()
{
	u16 lo = pull8();
	return (u16)( lo | ( (u16)pull8() << 8 ) );
}

// The "new" forms: 16-bit S even in emulation mode.
inline void CW65C816::push8New( u8 v )
{
	write8( m_S, v );
	m_S = (u16)( m_S - 1 );
}

inline u8 CW65C816::pull8New()
{
	m_S = (u16)( m_S + 1 );
	return read8( m_S );
}

inline void CW65C816::push16New( u16 v )
{
	push8New( (u8)( v >> 8 ) );
	push8New( (u8)( v & 0xFF ) );
}

inline u16 CW65C816::pull16New()
{
	u16 lo = pull8New();
	return (u16)( lo | ( (u16)pull8New() << 8 ) );
}

// --- effective addresses ---------------------------------------------------
// These compute addresses only. Cycle counts come from w65c816Cycles() using
// the table, so the one thing these do report is m_PageCross.

// dp
inline u32 CW65C816::eaDirect()
{
	u8 off = fetch8();
	m_EABank0 = true;
	return (u32)( (u16)( off + m_D ) );		// bank 0, 16-bit wrap
}

// dp,X and dp,Y -- see the header comment; this is the rule most often wrong.
inline u32 CW65C816::eaDirectIndexed( u16 index )
{
	u8 off = fetch8();
	m_EABank0 = true;

	if ( m_E && !( m_D & 0xFF ) )
	{
		// Emulation mode with an aligned direct page: wrap at $FF, 6502 style.
		return (u32)( (u8)( off + index ) + m_D );
	}

	return (u32)( (u16)( off + index + m_D ) );
}

// abs -- relative to the DATA bank
inline u32 CW65C816::eaAbsolute()
{
	u16 a = fetch16();
	m_EABank0 = false;
	return ( (u32)m_DBR << 16 ) | a;
}

// abs,X and abs,Y -- a true 24-bit add, so it carries into the next bank
inline u32 CW65C816::eaAbsoluteIndexed( u16 index )
{
	u16 base = fetch16();
	m_EABank0 = false;

	if ( ( base ^ (u16)( base + index ) ) & 0xFF00 )
		m_PageCross = true;

	return ( ( ( (u32)m_DBR << 16 ) | base ) + index ) & SCPU_ADDR_MASK;
}

// long and long,X -- the bank comes from the instruction, so DBR is ignored
inline u32 CW65C816::eaLong()
{
	m_EABank0 = false;
	return fetch24() & SCPU_ADDR_MASK;
}

inline u32 CW65C816::eaLongIndexed( u16 index )
{
	m_EABank0 = false;
	return ( fetch24() + index ) & SCPU_ADDR_MASK;
}

// (dp) -- pointer read from bank 0, result relative to DBR
inline u32 CW65C816::eaIndirect()
{
	u32 ptr = eaDirect();
	u16 target = readDPPointer( ptr );
	m_EABank0 = false;			// the pointer was bank 0; the data is not
	return ( (u32)m_DBR << 16 ) | target;
}

// (dp,X) -- index applied to the pointer, before the indirection
inline u32 CW65C816::eaIndirectX()
{
	u32 ptr = eaDirectIndexed( getX() );
	u16 target = readDPPointer( ptr );
	m_EABank0 = false;
	return ( (u32)m_DBR << 16 ) | target;
}

// (dp),Y -- index applied after, and may cross a bank
inline u32 CW65C816::eaIndirectY()
{
	u32 ptr = eaDirect();
	u16 target = readDPPointer( ptr );
	m_EABank0 = false;
	const u16 y = getY();

	if ( ( target ^ (u16)( target + y ) ) & 0xFF00 )
		m_PageCross = true;

	return ( ( ( (u32)m_DBR << 16 ) | target ) + y ) & SCPU_ADDR_MASK;
}

// [dp] -- 24-bit pointer, so the bank comes from the pointer itself. Excluded
// from the direct-page wrap by the datasheet: this runs off the end of the page.
inline u32 CW65C816::eaIndirectLong()
{
	u32 ptr = eaDirect();
	u32 target = read8( (u16)ptr )
	           | ( (u32)read8( (u16)( ptr + 1 ) ) << 8 )
	           | ( (u32)read8( (u16)( ptr + 2 ) ) << 16 );
	m_EABank0 = false;
	return target & SCPU_ADDR_MASK;
}

// [dp],Y
inline u32 CW65C816::eaIndirectLongY()
{
	return ( eaIndirectLong() + getY() ) & SCPU_ADDR_MASK;
}

// sr -- stack relative, always bank 0, never confined to page 1
inline u32 CW65C816::eaStackRelative()
{
	u8 off = fetch8();
	m_EABank0 = true;
	return (u32)( (u16)( off + m_S ) );
}

// (sr),Y -- pointer from bank 0, then DBR and Y, and may cross a bank
inline u32 CW65C816::eaStackRelativeIndirectY()
{
	u32 ptr = eaStackRelative();
	u16 target = read16Bank0( (u16)ptr );
	m_EABank0 = false;
	return ( ( ( (u32)m_DBR << 16 ) | target ) + getY() ) & SCPU_ADDR_MASK;
}

#endif
