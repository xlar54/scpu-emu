/*
   SCPU-EMU - CMD SuperCPU emulation for the C64/C128 using a RAD Expansion Unit
   Copyright (c) 2026 SCPU-EMU contributors

   WDC 65C816 opcode table. See w65c816_opcodes.h for why this is a separate
   table from the 6502's and must stay that way.

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
#include "w65c816_opcodes.h"

const SW65Opcode w65c816Opcodes[ 256 ] =
{
	{ "BRK", W65M_IMM8,  7, W65C_E0                   },	// $00
	{ "ORA", W65M_IZX,   6, W65C_M1 | W65C_W          },	// $01
	{ "COP", W65M_IMM8,  7, W65C_E0                   },	// $02
	{ "ORA", W65M_SR,    4, W65C_M1                   },	// $03
	{ "TSB", W65M_ZP,    5, W65C_M2 | W65C_W          },	// $04
	{ "ORA", W65M_ZP,    3, W65C_M1 | W65C_W          },	// $05
	{ "ASL", W65M_ZP,    5, W65C_M2 | W65C_W          },	// $06
	{ "ORA", W65M_IZL,   6, W65C_M1 | W65C_W          },	// $07
	{ "PHP", W65M_IMP,   3, 0                         },	// $08
	{ "ORA", W65M_IMMM,  2, W65C_M1                   },	// $09
	{ "ASL", W65M_ACC,   2, 0                         },	// $0A
	{ "PHD", W65M_IMP,   4, 0                         },	// $0B
	{ "TSB", W65M_ABS,   6, W65C_M2                   },	// $0C
	{ "ORA", W65M_ABS,   4, W65C_M1                   },	// $0D
	{ "ASL", W65M_ABS,   6, W65C_M2                   },	// $0E
	{ "ORA", W65M_ABL,   5, W65C_M1                   },	// $0F
	{ "BPL", W65M_REL,   2, W65C_BR                   },	// $10
	{ "ORA", W65M_IZY,   5, W65C_M1 | W65C_W | W65C_P },	// $11
	{ "ORA", W65M_IZP,   5, W65C_M1 | W65C_W          },	// $12
	{ "ORA", W65M_SRY,   7, W65C_M1                   },	// $13
	{ "TRB", W65M_ZP,    5, W65C_M2 | W65C_W          },	// $14
	{ "ORA", W65M_ZPX,   4, W65C_M1 | W65C_W          },	// $15
	{ "ASL", W65M_ZPX,   6, W65C_M2 | W65C_W          },	// $16
	{ "ORA", W65M_IZLY,  6, W65C_M1 | W65C_W          },	// $17
	{ "CLC", W65M_IMP,   2, 0                         },	// $18
	{ "ORA", W65M_ABY,   4, W65C_M1 | W65C_P          },	// $19
	{ "INC", W65M_ACC,   2, 0                         },	// $1A
	{ "TCS", W65M_IMP,   2, 0                         },	// $1B
	{ "TRB", W65M_ABS,   6, W65C_M2                   },	// $1C
	{ "ORA", W65M_ABX,   4, W65C_M1 | W65C_P          },	// $1D
	{ "ASL", W65M_ABX,   7, W65C_M2                   },	// $1E
	{ "ORA", W65M_ABLX,  5, W65C_M1                   },	// $1F
	{ "JSR", W65M_ABS,   6, 0                         },	// $20
	{ "AND", W65M_IZX,   6, W65C_M1 | W65C_W          },	// $21
	{ "JSL", W65M_ABL,   8, 0                         },	// $22
	{ "AND", W65M_SR,    4, W65C_M1                   },	// $23
	{ "BIT", W65M_ZP,    3, W65C_M1 | W65C_W          },	// $24
	{ "AND", W65M_ZP,    3, W65C_M1 | W65C_W          },	// $25
	{ "ROL", W65M_ZP,    5, W65C_M2 | W65C_W          },	// $26
	{ "AND", W65M_IZL,   6, W65C_M1 | W65C_W          },	// $27
	{ "PLP", W65M_IMP,   4, 0                         },	// $28
	{ "AND", W65M_IMMM,  2, W65C_M1                   },	// $29
	{ "ROL", W65M_ACC,   2, 0                         },	// $2A
	{ "PLD", W65M_IMP,   5, 0                         },	// $2B
	{ "BIT", W65M_ABS,   4, W65C_M1                   },	// $2C
	{ "AND", W65M_ABS,   4, W65C_M1                   },	// $2D
	{ "ROL", W65M_ABS,   6, W65C_M2                   },	// $2E
	{ "AND", W65M_ABL,   5, W65C_M1                   },	// $2F
	{ "BMI", W65M_REL,   2, W65C_BR                   },	// $30
	{ "AND", W65M_IZY,   5, W65C_M1 | W65C_W | W65C_P },	// $31
	{ "AND", W65M_IZP,   5, W65C_M1 | W65C_W          },	// $32
	{ "AND", W65M_SRY,   7, W65C_M1                   },	// $33
	{ "BIT", W65M_ZPX,   4, W65C_M1 | W65C_W          },	// $34
	{ "AND", W65M_ZPX,   4, W65C_M1 | W65C_W          },	// $35
	{ "ROL", W65M_ZPX,   6, W65C_M2 | W65C_W          },	// $36
	{ "AND", W65M_IZLY,  6, W65C_M1 | W65C_W          },	// $37
	{ "SEC", W65M_IMP,   2, 0                         },	// $38
	{ "AND", W65M_ABY,   4, W65C_M1 | W65C_P          },	// $39
	{ "DEC", W65M_ACC,   2, 0                         },	// $3A
	{ "TSC", W65M_IMP,   2, 0                         },	// $3B
	{ "BIT", W65M_ABX,   4, W65C_M1 | W65C_P          },	// $3C
	{ "AND", W65M_ABX,   4, W65C_M1 | W65C_P          },	// $3D
	{ "ROL", W65M_ABX,   7, W65C_M2                   },	// $3E
	{ "AND", W65M_ABLX,  5, W65C_M1                   },	// $3F
	{ "RTI", W65M_IMP,   6, W65C_E0                   },	// $40
	{ "EOR", W65M_IZX,   6, W65C_M1 | W65C_W          },	// $41
	{ "WDM", W65M_IMM8,  2, 0                         },	// $42
	{ "EOR", W65M_SR,    4, W65C_M1                   },	// $43
	{ "MVP", W65M_BM,    7, 0                         },	// $44
	{ "EOR", W65M_ZP,    3, W65C_M1 | W65C_W          },	// $45
	{ "LSR", W65M_ZP,    5, W65C_M2 | W65C_W          },	// $46
	{ "EOR", W65M_IZL,   6, W65C_M1 | W65C_W          },	// $47
	{ "PHA", W65M_IMP,   3, W65C_M1                   },	// $48
	{ "EOR", W65M_IMMM,  2, W65C_M1                   },	// $49
	{ "LSR", W65M_ACC,   2, 0                         },	// $4A
	{ "PHK", W65M_IMP,   3, 0                         },	// $4B
	{ "JMP", W65M_ABS,   3, 0                         },	// $4C
	{ "EOR", W65M_ABS,   4, W65C_M1                   },	// $4D
	{ "LSR", W65M_ABS,   6, W65C_M2                   },	// $4E
	{ "EOR", W65M_ABL,   5, W65C_M1                   },	// $4F
	{ "BVC", W65M_REL,   2, W65C_BR                   },	// $50
	{ "EOR", W65M_IZY,   5, W65C_M1 | W65C_W | W65C_P },	// $51
	{ "EOR", W65M_IZP,   5, W65C_M1 | W65C_W          },	// $52
	{ "EOR", W65M_SRY,   7, W65C_M1                   },	// $53
	{ "MVN", W65M_BM,    7, 0                         },	// $54
	{ "EOR", W65M_ZPX,   4, W65C_M1 | W65C_W          },	// $55
	{ "LSR", W65M_ZPX,   6, W65C_M2 | W65C_W          },	// $56
	{ "EOR", W65M_IZLY,  6, W65C_M1 | W65C_W          },	// $57
	{ "CLI", W65M_IMP,   2, 0                         },	// $58
	{ "EOR", W65M_ABY,   4, W65C_M1 | W65C_P          },	// $59
	{ "PHY", W65M_IMP,   3, W65C_X1                   },	// $5A
	{ "TCD", W65M_IMP,   2, 0                         },	// $5B
	{ "JML", W65M_ABL,   4, 0                         },	// $5C
	{ "EOR", W65M_ABX,   4, W65C_M1 | W65C_P          },	// $5D
	{ "LSR", W65M_ABX,   7, W65C_M2                   },	// $5E
	{ "EOR", W65M_ABLX,  5, W65C_M1                   },	// $5F
	{ "RTS", W65M_IMP,   6, 0                         },	// $60
	{ "ADC", W65M_IZX,   6, W65C_M1 | W65C_W          },	// $61
	{ "PER", W65M_RELL,  6, 0                         },	// $62
	{ "ADC", W65M_SR,    4, W65C_M1                   },	// $63
	{ "STZ", W65M_ZP,    3, W65C_M1 | W65C_W          },	// $64
	{ "ADC", W65M_ZP,    3, W65C_M1 | W65C_W          },	// $65
	{ "ROR", W65M_ZP,    5, W65C_M2 | W65C_W          },	// $66
	{ "ADC", W65M_IZL,   6, W65C_M1 | W65C_W          },	// $67
	{ "PLA", W65M_IMP,   4, W65C_M1                   },	// $68
	{ "ADC", W65M_IMMM,  2, W65C_M1                   },	// $69
	{ "ROR", W65M_ACC,   2, 0                         },	// $6A
	{ "RTL", W65M_IMP,   6, 0                         },	// $6B
	{ "JMP", W65M_IND,   5, 0                         },	// $6C
	{ "ADC", W65M_ABS,   4, W65C_M1                   },	// $6D
	{ "ROR", W65M_ABS,   6, W65C_M2                   },	// $6E
	{ "ADC", W65M_ABL,   5, W65C_M1                   },	// $6F
	{ "BVS", W65M_REL,   2, W65C_BR                   },	// $70
	{ "ADC", W65M_IZY,   5, W65C_M1 | W65C_W | W65C_P },	// $71
	{ "ADC", W65M_IZP,   5, W65C_M1 | W65C_W          },	// $72
	{ "ADC", W65M_SRY,   7, W65C_M1                   },	// $73
	{ "STZ", W65M_ZPX,   4, W65C_M1 | W65C_W          },	// $74
	{ "ADC", W65M_ZPX,   4, W65C_M1 | W65C_W          },	// $75
	{ "ROR", W65M_ZPX,   6, W65C_M2 | W65C_W          },	// $76
	{ "ADC", W65M_IZLY,  6, W65C_M1 | W65C_W          },	// $77
	{ "SEI", W65M_IMP,   2, 0                         },	// $78
	{ "ADC", W65M_ABY,   4, W65C_M1 | W65C_P          },	// $79
	{ "PLY", W65M_IMP,   4, W65C_X1                   },	// $7A
	{ "TDC", W65M_IMP,   2, 0                         },	// $7B
	{ "JMP", W65M_IAX,   6, 0                         },	// $7C
	{ "ADC", W65M_ABX,   4, W65C_M1 | W65C_P          },	// $7D
	{ "ROR", W65M_ABX,   7, W65C_M2                   },	// $7E
	{ "ADC", W65M_ABLX,  5, W65C_M1                   },	// $7F
	{ "BRA", W65M_REL,   3, W65C_PE                   },	// $80
	{ "STA", W65M_IZX,   6, W65C_M1 | W65C_W          },	// $81
	{ "BRL", W65M_RELL,  4, 0                         },	// $82
	{ "STA", W65M_SR,    4, W65C_M1                   },	// $83
	{ "STY", W65M_ZP,    3, W65C_X1 | W65C_W          },	// $84
	{ "STA", W65M_ZP,    3, W65C_M1 | W65C_W          },	// $85
	{ "STX", W65M_ZP,    3, W65C_X1 | W65C_W          },	// $86
	{ "STA", W65M_IZL,   6, W65C_M1 | W65C_W          },	// $87
	{ "DEY", W65M_IMP,   2, 0                         },	// $88
	{ "BIT", W65M_IMMM,  2, W65C_M1                   },	// $89
	{ "TXA", W65M_IMP,   2, 0                         },	// $8A
	{ "PHB", W65M_IMP,   3, 0                         },	// $8B
	{ "STY", W65M_ABS,   4, W65C_X1                   },	// $8C
	{ "STA", W65M_ABS,   4, W65C_M1                   },	// $8D
	{ "STX", W65M_ABS,   4, W65C_X1                   },	// $8E
	{ "STA", W65M_ABL,   5, W65C_M1                   },	// $8F
	{ "BCC", W65M_REL,   2, W65C_BR                   },	// $90
	{ "STA", W65M_IZY,   6, W65C_M1 | W65C_W          },	// $91
	{ "STA", W65M_IZP,   5, W65C_M1 | W65C_W          },	// $92
	{ "STA", W65M_SRY,   7, W65C_M1                   },	// $93
	{ "STY", W65M_ZPX,   4, W65C_X1 | W65C_W          },	// $94
	{ "STA", W65M_ZPX,   4, W65C_M1 | W65C_W          },	// $95
	{ "STX", W65M_ZPY,   4, W65C_X1 | W65C_W          },	// $96
	{ "STA", W65M_IZLY,  6, W65C_M1 | W65C_W          },	// $97
	{ "TYA", W65M_IMP,   2, 0                         },	// $98
	{ "STA", W65M_ABY,   5, W65C_M1                   },	// $99
	{ "TXS", W65M_IMP,   2, 0                         },	// $9A
	{ "TXY", W65M_IMP,   2, 0                         },	// $9B
	{ "STZ", W65M_ABS,   4, W65C_M1                   },	// $9C
	{ "STA", W65M_ABX,   5, W65C_M1                   },	// $9D
	{ "STZ", W65M_ABX,   5, W65C_M1                   },	// $9E
	{ "STA", W65M_ABLX,  5, W65C_M1                   },	// $9F
	{ "LDY", W65M_IMMX,  2, W65C_X1                   },	// $A0
	{ "LDA", W65M_IZX,   6, W65C_M1 | W65C_W          },	// $A1
	{ "LDX", W65M_IMMX,  2, W65C_X1                   },	// $A2
	{ "LDA", W65M_SR,    4, W65C_M1                   },	// $A3
	{ "LDY", W65M_ZP,    3, W65C_X1 | W65C_W          },	// $A4
	{ "LDA", W65M_ZP,    3, W65C_M1 | W65C_W          },	// $A5
	{ "LDX", W65M_ZP,    3, W65C_X1 | W65C_W          },	// $A6
	{ "LDA", W65M_IZL,   6, W65C_M1 | W65C_W          },	// $A7
	{ "TAY", W65M_IMP,   2, 0                         },	// $A8
	{ "LDA", W65M_IMMM,  2, W65C_M1                   },	// $A9
	{ "TAX", W65M_IMP,   2, 0                         },	// $AA
	{ "PLB", W65M_IMP,   4, 0                         },	// $AB
	{ "LDY", W65M_ABS,   4, W65C_X1                   },	// $AC
	{ "LDA", W65M_ABS,   4, W65C_M1                   },	// $AD
	{ "LDX", W65M_ABS,   4, W65C_X1                   },	// $AE
	{ "LDA", W65M_ABL,   5, W65C_M1                   },	// $AF
	{ "BCS", W65M_REL,   2, W65C_BR                   },	// $B0
	{ "LDA", W65M_IZY,   5, W65C_M1 | W65C_W | W65C_P },	// $B1
	{ "LDA", W65M_IZP,   5, W65C_M1 | W65C_W          },	// $B2
	{ "LDA", W65M_SRY,   7, W65C_M1                   },	// $B3
	{ "LDY", W65M_ZPX,   4, W65C_X1 | W65C_W          },	// $B4
	{ "LDA", W65M_ZPX,   4, W65C_M1 | W65C_W          },	// $B5
	{ "LDX", W65M_ZPY,   4, W65C_X1 | W65C_W          },	// $B6
	{ "LDA", W65M_IZLY,  6, W65C_M1 | W65C_W          },	// $B7
	{ "CLV", W65M_IMP,   2, 0                         },	// $B8
	{ "LDA", W65M_ABY,   4, W65C_M1 | W65C_P          },	// $B9
	{ "TSX", W65M_IMP,   2, 0                         },	// $BA
	{ "TYX", W65M_IMP,   2, 0                         },	// $BB
	{ "LDY", W65M_ABX,   4, W65C_X1 | W65C_P          },	// $BC
	{ "LDA", W65M_ABX,   4, W65C_M1 | W65C_P          },	// $BD
	{ "LDX", W65M_ABY,   4, W65C_X1 | W65C_P          },	// $BE
	{ "LDA", W65M_ABLX,  5, W65C_M1                   },	// $BF
	{ "CPY", W65M_IMMX,  2, W65C_X1                   },	// $C0
	{ "CMP", W65M_IZX,   6, W65C_M1 | W65C_W          },	// $C1
	{ "REP", W65M_IMM8,  3, 0                         },	// $C2
	{ "CMP", W65M_SR,    4, W65C_M1                   },	// $C3
	{ "CPY", W65M_ZP,    3, W65C_X1 | W65C_W          },	// $C4
	{ "CMP", W65M_ZP,    3, W65C_M1 | W65C_W          },	// $C5
	{ "DEC", W65M_ZP,    5, W65C_M2 | W65C_W          },	// $C6
	{ "CMP", W65M_IZL,   6, W65C_M1 | W65C_W          },	// $C7
	{ "INY", W65M_IMP,   2, 0                         },	// $C8
	{ "CMP", W65M_IMMM,  2, W65C_M1                   },	// $C9
	{ "DEX", W65M_IMP,   2, 0                         },	// $CA
	{ "WAI", W65M_IMP,   3, 0                         },	// $CB
	{ "CPY", W65M_ABS,   4, W65C_X1                   },	// $CC
	{ "CMP", W65M_ABS,   4, W65C_M1                   },	// $CD
	{ "DEC", W65M_ABS,   6, W65C_M2                   },	// $CE
	{ "CMP", W65M_ABL,   5, W65C_M1                   },	// $CF
	{ "BNE", W65M_REL,   2, W65C_BR                   },	// $D0
	{ "CMP", W65M_IZY,   5, W65C_M1 | W65C_W | W65C_P },	// $D1
	{ "CMP", W65M_IZP,   5, W65C_M1 | W65C_W          },	// $D2
	{ "CMP", W65M_SRY,   7, W65C_M1                   },	// $D3
	{ "PEI", W65M_ZP,    6, W65C_W                    },	// $D4
	{ "CMP", W65M_ZPX,   4, W65C_M1 | W65C_W          },	// $D5
	{ "DEC", W65M_ZPX,   6, W65C_M2 | W65C_W          },	// $D6
	{ "CMP", W65M_IZLY,  6, W65C_M1 | W65C_W          },	// $D7
	{ "CLD", W65M_IMP,   2, 0                         },	// $D8
	{ "CMP", W65M_ABY,   4, W65C_M1 | W65C_P          },	// $D9
	{ "PHX", W65M_IMP,   3, W65C_X1                   },	// $DA
	{ "STP", W65M_IMP,   3, 0                         },	// $DB
	{ "JML", W65M_INDL,  6, 0                         },	// $DC
	{ "CMP", W65M_ABX,   4, W65C_M1 | W65C_P          },	// $DD
	{ "DEC", W65M_ABX,   7, W65C_M2                   },	// $DE
	{ "CMP", W65M_ABLX,  5, W65C_M1                   },	// $DF
	{ "CPX", W65M_IMMX,  2, W65C_X1                   },	// $E0
	{ "SBC", W65M_IZX,   6, W65C_M1 | W65C_W          },	// $E1
	{ "SEP", W65M_IMM8,  3, 0                         },	// $E2
	{ "SBC", W65M_SR,    4, W65C_M1                   },	// $E3
	{ "CPX", W65M_ZP,    3, W65C_X1 | W65C_W          },	// $E4
	{ "SBC", W65M_ZP,    3, W65C_M1 | W65C_W          },	// $E5
	{ "INC", W65M_ZP,    5, W65C_M2 | W65C_W          },	// $E6
	{ "SBC", W65M_IZL,   6, W65C_M1 | W65C_W          },	// $E7
	{ "INX", W65M_IMP,   2, 0                         },	// $E8
	{ "SBC", W65M_IMMM,  2, W65C_M1                   },	// $E9
	{ "NOP", W65M_IMP,   2, 0                         },	// $EA
	{ "XBA", W65M_IMP,   3, 0                         },	// $EB
	{ "CPX", W65M_ABS,   4, W65C_X1                   },	// $EC
	{ "SBC", W65M_ABS,   4, W65C_M1                   },	// $ED
	{ "INC", W65M_ABS,   6, W65C_M2                   },	// $EE
	{ "SBC", W65M_ABL,   5, W65C_M1                   },	// $EF
	{ "BEQ", W65M_REL,   2, W65C_BR                   },	// $F0
	{ "SBC", W65M_IZY,   5, W65C_M1 | W65C_W | W65C_P },	// $F1
	{ "SBC", W65M_IZP,   5, W65C_M1 | W65C_W          },	// $F2
	{ "SBC", W65M_SRY,   7, W65C_M1                   },	// $F3
	{ "PEA", W65M_ABS,   5, 0                         },	// $F4
	{ "SBC", W65M_ZPX,   4, W65C_M1 | W65C_W          },	// $F5
	{ "INC", W65M_ZPX,   6, W65C_M2 | W65C_W          },	// $F6
	{ "SBC", W65M_IZLY,  6, W65C_M1 | W65C_W          },	// $F7
	{ "SED", W65M_IMP,   2, 0                         },	// $F8
	{ "SBC", W65M_ABY,   4, W65C_M1 | W65C_P          },	// $F9
	{ "PLX", W65M_IMP,   4, W65C_X1                   },	// $FA
	{ "XCE", W65M_IMP,   2, 0                         },	// $FB
	{ "JSR", W65M_IAX,   8, 0                         },	// $FC
	{ "SBC", W65M_ABX,   4, W65C_M1 | W65C_P          },	// $FD
	{ "INC", W65M_ABX,   7, W65C_M2                   },	// $FE
	{ "SBC", W65M_ABLX,  5, W65C_M1                   },	// $FF
};

// Operand bytes per addressing mode, before the M/X width adjustment.
static const u8 s_ModeLength[] =
{
	1,	// W65M_IMP
	1,	// W65M_ACC
	2,	// W65M_IMM8
	2,	// W65M_IMMM   -- 3 when the accumulator is 16-bit
	2,	// W65M_IMMX   -- 3 when the index registers are 16-bit
	2,	// W65M_ZP
	2,	// W65M_ZPX
	2,	// W65M_ZPY
	2,	// W65M_IZX
	2,	// W65M_IZY
	2,	// W65M_IZP
	2,	// W65M_IZL
	2,	// W65M_IZLY
	3,	// W65M_ABS
	3,	// W65M_ABX
	3,	// W65M_ABY
	4,	// W65M_ABL
	4,	// W65M_ABLX
	3,	// W65M_IND
	3,	// W65M_IAX
	3,	// W65M_INDL
	2,	// W65M_SR
	2,	// W65M_SRY
	2,	// W65M_REL
	3,	// W65M_RELL
	3	// W65M_BM
};

u8 w65c816Length( u8 opcode, bool m8, bool x8 )
{
	const u8 mode = w65c816Opcodes[ opcode ].mode;
	u8 len = s_ModeLength[ mode ];

	// The only variable-length modes: an immediate operand is as wide as the
	// register it feeds. This is why REP/SEP change the length of instructions
	// that have already been assembled.
	if ( mode == W65M_IMMM && !m8 ) len++;
	if ( mode == W65M_IMMX && !x8 ) len++;

	return len;
}

u8 w65c816Cycles( u8 opcode, bool m8, bool x8, bool emulation,
                  bool dpUnaligned, bool pageCross, bool branchTaken )
{
	const SW65Opcode &op = w65c816Opcodes[ opcode ];
	u8 c = op.baseCycles;
	const u8 f = op.cycleFlags;

	if ( ( f & W65C_M1 ) && !m8 ) c += 1;
	if ( ( f & W65C_M2 ) && !m8 ) c += 2;
	if ( ( f & W65C_X1 ) && !x8 ) c += 1;
	if ( ( f & W65C_W  ) && dpUnaligned ) c++;
	if ( ( f & W65C_E0 ) && !emulation ) c++;

	// Indexed reads pay the penalty on a page cross with 8-bit index registers,
	// and unconditionally with 16-bit ones -- the address adder always needs the
	// extra cycle when the index can carry.
	if ( f & W65C_P )
	{
		if ( !x8 || pageCross ) c++;
	}

	if ( f & W65C_BR )
	{
		if ( branchTaken )
		{
			c++;
			// The page-cross penalty on a taken branch is emulation-mode only.
			if ( emulation && pageCross ) c++;
		}
	}

	if ( ( f & W65C_PE ) && emulation && pageCross ) c++;

	return c;
}
