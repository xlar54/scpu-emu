/*
   SCPU-EMU - CMD SuperCPU emulation for the C64/C128 using a RAD Expansion Unit
   Copyright (c) 2026 SCPU-EMU contributors

   6502 opcode metadata table and disassembler.

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
#include "m6502_opcodes.h"

const u8 m6502ModeLength[ AM_MODES ] =
{
	1,	// AM_IMP
	1,	// AM_ACC
	2,	// AM_IMM
	2,	// AM_ZP
	2,	// AM_ZPX
	2,	// AM_ZPY
	3,	// AM_ABS
	3,	// AM_ABX
	3,	// AM_ABY
	3,	// AM_IND
	2,	// AM_IZX
	2,	// AM_IZY
	2	// AM_REL
};

// D = documented, U = undocumented (executed as a NOP of the listed length).
#define D( m, mode, cyc ) { m, mode, cyc, 0 }
#define U( m, mode, cyc ) { m, mode, cyc, 1 }

const M6502OpcodeInfo m6502Opcodes[ 256 ] =
{
	/* 00 */ D("BRK",AM_IMP,7), D("ORA",AM_IZX,6), U("JAM",AM_IMP,0), U("SLO",AM_IZX,8),
	/* 04 */ U("NOP",AM_ZP ,3), D("ORA",AM_ZP ,3), D("ASL",AM_ZP ,5), U("SLO",AM_ZP ,5),
	/* 08 */ D("PHP",AM_IMP,3), D("ORA",AM_IMM,2), D("ASL",AM_ACC,2), U("ANC",AM_IMM,2),
	/* 0C */ U("NOP",AM_ABS,4), D("ORA",AM_ABS,4), D("ASL",AM_ABS,6), U("SLO",AM_ABS,6),

	/* 10 */ D("BPL",AM_REL,2), D("ORA",AM_IZY,5), U("JAM",AM_IMP,0), U("SLO",AM_IZY,8),
	/* 14 */ U("NOP",AM_ZPX,4), D("ORA",AM_ZPX,4), D("ASL",AM_ZPX,6), U("SLO",AM_ZPX,6),
	/* 18 */ D("CLC",AM_IMP,2), D("ORA",AM_ABY,4), U("NOP",AM_IMP,2), U("SLO",AM_ABY,7),
	/* 1C */ U("NOP",AM_ABX,4), D("ORA",AM_ABX,4), D("ASL",AM_ABX,7), U("SLO",AM_ABX,7),

	/* 20 */ D("JSR",AM_ABS,6), D("AND",AM_IZX,6), U("JAM",AM_IMP,0), U("RLA",AM_IZX,8),
	/* 24 */ D("BIT",AM_ZP ,3), D("AND",AM_ZP ,3), D("ROL",AM_ZP ,5), U("RLA",AM_ZP ,5),
	/* 28 */ D("PLP",AM_IMP,4), D("AND",AM_IMM,2), D("ROL",AM_ACC,2), U("ANC",AM_IMM,2),
	/* 2C */ D("BIT",AM_ABS,4), D("AND",AM_ABS,4), D("ROL",AM_ABS,6), U("RLA",AM_ABS,6),

	/* 30 */ D("BMI",AM_REL,2), D("AND",AM_IZY,5), U("JAM",AM_IMP,0), U("RLA",AM_IZY,8),
	/* 34 */ U("NOP",AM_ZPX,4), D("AND",AM_ZPX,4), D("ROL",AM_ZPX,6), U("RLA",AM_ZPX,6),
	/* 38 */ D("SEC",AM_IMP,2), D("AND",AM_ABY,4), U("NOP",AM_IMP,2), U("RLA",AM_ABY,7),
	/* 3C */ U("NOP",AM_ABX,4), D("AND",AM_ABX,4), D("ROL",AM_ABX,7), U("RLA",AM_ABX,7),

	/* 40 */ D("RTI",AM_IMP,6), D("EOR",AM_IZX,6), U("JAM",AM_IMP,0), U("SRE",AM_IZX,8),
	/* 44 */ U("NOP",AM_ZP ,3), D("EOR",AM_ZP ,3), D("LSR",AM_ZP ,5), U("SRE",AM_ZP ,5),
	/* 48 */ D("PHA",AM_IMP,3), D("EOR",AM_IMM,2), D("LSR",AM_ACC,2), U("ALR",AM_IMM,2),
	/* 4C */ D("JMP",AM_ABS,3), D("EOR",AM_ABS,4), D("LSR",AM_ABS,6), U("SRE",AM_ABS,6),

	/* 50 */ D("BVC",AM_REL,2), D("EOR",AM_IZY,5), U("JAM",AM_IMP,0), U("SRE",AM_IZY,8),
	/* 54 */ U("NOP",AM_ZPX,4), D("EOR",AM_ZPX,4), D("LSR",AM_ZPX,6), U("SRE",AM_ZPX,6),
	/* 58 */ D("CLI",AM_IMP,2), D("EOR",AM_ABY,4), U("NOP",AM_IMP,2), U("SRE",AM_ABY,7),
	/* 5C */ U("NOP",AM_ABX,4), D("EOR",AM_ABX,4), D("LSR",AM_ABX,7), U("SRE",AM_ABX,7),

	/* 60 */ D("RTS",AM_IMP,6), D("ADC",AM_IZX,6), U("JAM",AM_IMP,0), U("RRA",AM_IZX,8),
	/* 64 */ U("NOP",AM_ZP ,3), D("ADC",AM_ZP ,3), D("ROR",AM_ZP ,5), U("RRA",AM_ZP ,5),
	/* 68 */ D("PLA",AM_IMP,4), D("ADC",AM_IMM,2), D("ROR",AM_ACC,2), U("ARR",AM_IMM,2),
	/* 6C */ D("JMP",AM_IND,5), D("ADC",AM_ABS,4), D("ROR",AM_ABS,6), U("RRA",AM_ABS,6),

	/* 70 */ D("BVS",AM_REL,2), D("ADC",AM_IZY,5), U("JAM",AM_IMP,0), U("RRA",AM_IZY,8),
	/* 74 */ U("NOP",AM_ZPX,4), D("ADC",AM_ZPX,4), D("ROR",AM_ZPX,6), U("RRA",AM_ZPX,6),
	/* 78 */ D("SEI",AM_IMP,2), D("ADC",AM_ABY,4), U("NOP",AM_IMP,2), U("RRA",AM_ABY,7),
	/* 7C */ U("NOP",AM_ABX,4), D("ADC",AM_ABX,4), D("ROR",AM_ABX,7), U("RRA",AM_ABX,7),

	/* 80 */ U("NOP",AM_IMM,2), D("STA",AM_IZX,6), U("NOP",AM_IMM,2), U("SAX",AM_IZX,6),
	/* 84 */ D("STY",AM_ZP ,3), D("STA",AM_ZP ,3), D("STX",AM_ZP ,3), U("SAX",AM_ZP ,3),
	/* 88 */ D("DEY",AM_IMP,2), U("NOP",AM_IMM,2), D("TXA",AM_IMP,2), U("ANE",AM_IMM,2),
	/* 8C */ D("STY",AM_ABS,4), D("STA",AM_ABS,4), D("STX",AM_ABS,4), U("SAX",AM_ABS,4),

	/* 90 */ D("BCC",AM_REL,2), D("STA",AM_IZY,6), U("JAM",AM_IMP,0), U("SHA",AM_IZY,6),
	/* 94 */ D("STY",AM_ZPX,4), D("STA",AM_ZPX,4), D("STX",AM_ZPY,4), U("SAX",AM_ZPY,4),
	/* 98 */ D("TYA",AM_IMP,2), D("STA",AM_ABY,5), D("TXS",AM_IMP,2), U("TAS",AM_ABY,5),
	/* 9C */ U("SHY",AM_ABX,5), D("STA",AM_ABX,5), U("SHX",AM_ABY,5), U("SHA",AM_ABY,5),

	/* A0 */ D("LDY",AM_IMM,2), D("LDA",AM_IZX,6), D("LDX",AM_IMM,2), U("LAX",AM_IZX,6),
	/* A4 */ D("LDY",AM_ZP ,3), D("LDA",AM_ZP ,3), D("LDX",AM_ZP ,3), U("LAX",AM_ZP ,3),
	/* A8 */ D("TAY",AM_IMP,2), D("LDA",AM_IMM,2), D("TAX",AM_IMP,2), U("LXA",AM_IMM,2),
	/* AC */ D("LDY",AM_ABS,4), D("LDA",AM_ABS,4), D("LDX",AM_ABS,4), U("LAX",AM_ABS,4),

	/* B0 */ D("BCS",AM_REL,2), D("LDA",AM_IZY,5), U("JAM",AM_IMP,0), U("LAX",AM_IZY,5),
	/* B4 */ D("LDY",AM_ZPX,4), D("LDA",AM_ZPX,4), D("LDX",AM_ZPY,4), U("LAX",AM_ZPY,4),
	/* B8 */ D("CLV",AM_IMP,2), D("LDA",AM_ABY,4), D("TSX",AM_IMP,2), U("LAS",AM_ABY,4),
	/* BC */ D("LDY",AM_ABX,4), D("LDA",AM_ABX,4), D("LDX",AM_ABY,4), U("LAX",AM_ABY,4),

	/* C0 */ D("CPY",AM_IMM,2), D("CMP",AM_IZX,6), U("NOP",AM_IMM,2), U("DCP",AM_IZX,8),
	/* C4 */ D("CPY",AM_ZP ,3), D("CMP",AM_ZP ,3), D("DEC",AM_ZP ,5), U("DCP",AM_ZP ,5),
	/* C8 */ D("INY",AM_IMP,2), D("CMP",AM_IMM,2), D("DEX",AM_IMP,2), U("SBX",AM_IMM,2),
	/* CC */ D("CPY",AM_ABS,4), D("CMP",AM_ABS,4), D("DEC",AM_ABS,6), U("DCP",AM_ABS,6),

	/* D0 */ D("BNE",AM_REL,2), D("CMP",AM_IZY,5), U("JAM",AM_IMP,0), U("DCP",AM_IZY,8),
	/* D4 */ U("NOP",AM_ZPX,4), D("CMP",AM_ZPX,4), D("DEC",AM_ZPX,6), U("DCP",AM_ZPX,6),
	/* D8 */ D("CLD",AM_IMP,2), D("CMP",AM_ABY,4), U("NOP",AM_IMP,2), U("DCP",AM_ABY,7),
	/* DC */ U("NOP",AM_ABX,4), D("CMP",AM_ABX,4), D("DEC",AM_ABX,7), U("DCP",AM_ABX,7),

	/* E0 */ D("CPX",AM_IMM,2), D("SBC",AM_IZX,6), U("NOP",AM_IMM,2), U("ISC",AM_IZX,8),
	/* E4 */ D("CPX",AM_ZP ,3), D("SBC",AM_ZP ,3), D("INC",AM_ZP ,5), U("ISC",AM_ZP ,5),
	/* E8 */ D("INX",AM_IMP,2), D("SBC",AM_IMM,2), D("NOP",AM_IMP,2), U("SBC",AM_IMM,2),
	/* EC */ D("CPX",AM_ABS,4), D("SBC",AM_ABS,4), D("INC",AM_ABS,6), U("ISC",AM_ABS,6),

	/* F0 */ D("BEQ",AM_REL,2), D("SBC",AM_IZY,5), U("JAM",AM_IMP,0), U("ISC",AM_IZY,8),
	/* F4 */ U("NOP",AM_ZPX,4), D("SBC",AM_ZPX,4), D("INC",AM_ZPX,6), U("ISC",AM_ZPX,6),
	/* F8 */ D("SED",AM_IMP,2), D("SBC",AM_ABY,4), U("NOP",AM_IMP,2), U("ISC",AM_ABY,7),
	/* FC */ U("NOP",AM_ABX,4), D("SBC",AM_ABX,4), D("INC",AM_ABX,7), U("ISC",AM_ABX,7)
};

#undef D
#undef U

// ---------------------------------------------------------------------------

static void appendStr( char **p, const char *s )
{
	while ( *s ) *(*p)++ = *s++;
}

static void appendHex( char **p, u32 value, int digits )
{
	const char *hex = "0123456789ABCDEF";
	for ( int i = digits - 1; i >= 0; i-- )
		*(*p)++ = hex[ ( value >> ( i * 4 ) ) & 0xF ];
}

u8 m6502Disassemble( char *out, u16 addr, M6502ReadFn readByte, void *ctx )
{
	u8 opcode = readByte( ctx, addr );
	const M6502OpcodeInfo *info = &m6502Opcodes[ opcode ];
	u8 len = m6502ModeLength[ info->mode ];

	char *p = out;
	appendStr( &p, info->mnemonic );
	if ( info->undocumented ) *p++ = '*';

	u8 lo = ( len > 1 ) ? readByte( ctx, (u16)( addr + 1 ) ) : 0;
	u8 hi = ( len > 2 ) ? readByte( ctx, (u16)( addr + 2 ) ) : 0;
	u16 abs = (u16)( lo | ( hi << 8 ) );

	switch ( info->mode )
	{
	case AM_IMP:                                                        break;
	case AM_ACC: appendStr( &p, " A" );                                 break;
	case AM_IMM: appendStr( &p, " #$" ); appendHex( &p, lo, 2 );        break;
	case AM_ZP:  appendStr( &p, " $" );  appendHex( &p, lo, 2 );        break;
	case AM_ZPX: appendStr( &p, " $" );  appendHex( &p, lo, 2 ); appendStr( &p, ",X" ); break;
	case AM_ZPY: appendStr( &p, " $" );  appendHex( &p, lo, 2 ); appendStr( &p, ",Y" ); break;
	case AM_ABS: appendStr( &p, " $" );  appendHex( &p, abs, 4 );       break;
	case AM_ABX: appendStr( &p, " $" );  appendHex( &p, abs, 4 ); appendStr( &p, ",X" ); break;
	case AM_ABY: appendStr( &p, " $" );  appendHex( &p, abs, 4 ); appendStr( &p, ",Y" ); break;
	case AM_IND: appendStr( &p, " ($" ); appendHex( &p, abs, 4 ); appendStr( &p, ")" );  break;
	case AM_IZX: appendStr( &p, " ($" ); appendHex( &p, lo, 2 );  appendStr( &p, ",X)" ); break;
	case AM_IZY: appendStr( &p, " ($" ); appendHex( &p, lo, 2 );  appendStr( &p, "),Y" ); break;
	case AM_REL:
		{
			u16 target = (u16)( addr + 2 + (s8)lo );
			appendStr( &p, " $" ); appendHex( &p, target, 4 );
		}
		break;
	default: break;
	}

	*p = 0;
	return len;
}
