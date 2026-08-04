/*
   SCPU-EMU - CMD SuperCPU emulation for the C64/C128 using a RAD Expansion Unit
   Copyright (c) 2026 SCPU-EMU contributors

   WDC 65C816 opcode table: mnemonic, addressing mode, length and cycle count for
   all 256 encodings.

   This table exists SEPARATELY from the 6502's on purpose. All 256 encodings are
   real instructions on a 65816, and 50 of the 105 the NMOS 6502 leaves
   undocumented have a DIFFERENT LENGTH here -- $22 is a 4-byte JSL where the 6502
   table says 1 byte, $0B is a 1-byte PHD where it says 2. Deriving 65816 lengths
   from m6502ModeLength[] desynchronises the instruction stream, which shows up as
   memory corruption a long way from its cause. Never do it.

   Lengths and cycles are not constants: they depend on the M and X flags, on
   whether the direct page is aligned, and on emulation mode. w65c816Length() and
   w65c816Cycles() apply those rules; the flags below name them.

   Derived from Bruce Clark's "65C816 Opcodes" cycle formulas and cross-checked
   against the WDC datasheet, VICE's 65816core.c and SingleStepTests/65816.
   Evaluated at e=1, m=1, x=1, w=0 the table reproduces m6502Opcodes[] exactly for
   all 151 documented opcodes -- lengths, base cycles, page-cross and branch
   penalties -- which is what makes the differential test against CM6502 fair.
   See Docs/research/65816-reference.md, sections 9 and 10.

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
#ifndef _w65c816_opcodes_h
#define _w65c816_opcodes_h

#include "../cpu.h"

enum EW65Mode
{
	W65M_IMP,		// implied
	W65M_ACC,		// accumulator
	W65M_IMM8,		// immediate, always one byte: BRK COP WDM REP SEP
	W65M_IMMM,		// immediate, width follows M
	W65M_IMMX,		// immediate, width follows X
	W65M_ZP,		// dp
	W65M_ZPX,		// dp,X
	W65M_ZPY,		// dp,Y
	W65M_IZX,		// (dp,X)
	W65M_IZY,		// (dp),Y
	W65M_IZP,		// (dp)
	W65M_IZL,		// [dp]
	W65M_IZLY,		// [dp],Y
	W65M_ABS,		// abs
	W65M_ABX,		// abs,X
	W65M_ABY,		// abs,Y
	W65M_ABL,		// long
	W65M_ABLX,		// long,X
	W65M_IND,		// (abs)      -- pointer from bank 0
	W65M_IAX,		// (abs,X)    -- pointer from the program bank
	W65M_INDL,		// [abs]      -- pointer from bank 0
	W65M_SR,		// sr,S
	W65M_SRY,		// (sr,S),Y
	W65M_REL,		// 8-bit relative
	W65M_RELL,		// 16-bit relative
	W65M_BM			// block move: dstbank, srcbank
};

// Cycle adjustments, in Bruce Clark's notation.
#define W65C_M1 0x01	// +1 when the accumulator is 16-bit
#define W65C_M2 0x02	// +2 when the accumulator is 16-bit (read-modify-write)
#define W65C_X1 0x04	// +1 when the index registers are 16-bit
#define W65C_W  0x08	// +1 when the direct page is unaligned (DL != 0)
#define W65C_P  0x10	// +1 on a page cross, or unconditionally with 16-bit index
#define W65C_E0 0x20	// +1 in native mode (the extra byte pushed)
#define W65C_BR 0x40	// +1 when taken, +1 more if emulation mode and page crossed
#define W65C_PE 0x80	// +1 if emulation mode and page crossed (BRA)

struct SW65Opcode
{
	const char *mnemonic;
	u8          mode;			// EW65Mode
	u8          baseCycles;		// at e=1, m=1, x=1, w=0, no penalties
	u8          cycleFlags;
};

extern const SW65Opcode w65c816Opcodes[ 256 ];

// Instruction length in bytes. m8/x8 are the EFFECTIVE widths, i.e. what
// CW65C816::memory8() and index8() return -- emulation mode forces both true.
u8 w65c816Length( u8 opcode, bool m8, bool x8 );

// Cycle count with every adjustment applied. pageCross is meaningful only for
// indexed and relative modes; branchTaken only for W65M_REL.
u8 w65c816Cycles( u8 opcode, bool m8, bool x8, bool emulation,
                  bool dpUnaligned, bool pageCross, bool branchTaken );

#endif
