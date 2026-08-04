/*
   SCPU-EMU - CMD SuperCPU emulation for the C64/C128 using a RAD Expansion Unit
   Copyright (c) 2026 SCPU-EMU contributors

   6502 opcode metadata table.

   Used by the disassembler, the trace comparison tool and the unit tests. The
   interpreter in m6502.cpp does not read this table on its hot path -- it
   dispatches through a switch so the compiler can build a jump table -- but the
   table is the single source of truth for instruction length and base cycle
   count, and the tests assert the two agree.

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
#ifndef _m6502_opcodes_h
#define _m6502_opcodes_h

#include "../../Common/types.h"

enum M6502AddrMode
{
	AM_IMP = 0,	// implied
	AM_ACC,		// accumulator
	AM_IMM,		// #$nn
	AM_ZP,		// $nn
	AM_ZPX,		// $nn,X
	AM_ZPY,		// $nn,Y
	AM_ABS,		// $nnnn
	AM_ABX,		// $nnnn,X
	AM_ABY,		// $nnnn,Y
	AM_IND,		// ($nnnn)
	AM_IZX,		// ($nn,X)
	AM_IZY,		// ($nn),Y
	AM_REL,		// branch displacement
	AM_MODES
};

// Instruction length in bytes for each addressing mode, including the opcode.
extern const u8 m6502ModeLength[ AM_MODES ];

typedef struct
{
	const char *mnemonic;
	u8 mode;		// M6502AddrMode
	u8 cycles;		// base cycles, before page-cross / branch-taken penalties
	u8 undocumented;	// 1 for encodings SCPU-EMU executes as a NOP (see m6502.h)
} M6502OpcodeInfo;

extern const M6502OpcodeInfo m6502Opcodes[ 256 ];

// Disassemble one instruction at 'addr'. 'readByte' supplies memory. The text
// is written to 'out' (needs >= 32 bytes); returns the instruction length.
typedef u8 (*M6502ReadFn)( void *ctx, u16 addr );
u8 m6502Disassemble( char *out, u16 addr, M6502ReadFn readByte, void *ctx );

#endif
