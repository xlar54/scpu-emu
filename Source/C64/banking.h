/*
   SCPU-EMU - CMD SuperCPU emulation for the C64/C128 using a RAD Expansion Unit
   Copyright (c) 2026 SCPU-EMU contributors

   C64 PLA banking model.

   Why this exists at all: on a real SuperCPU the 6510 is held off the bus, so
   its on-chip I/O port at $00/$01 -- the thing that actually drives LORAM,
   HIRAM and CHAREN -- is no longer the authority on banking. The accelerator
   has to emulate the port itself and resolve the memory map in its own logic
   before deciding whether an access can be served from fast local RAM or has
   to go out to the C64. That resolution is what this file does.

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
#ifndef _c64_banking_h
#define _c64_banking_h

#include "../Common/types.h"

// What is visible in a given 4K page for a given configuration.
enum C64Region
{
	REG_RAM = 0,		// C64 DRAM
	REG_BASIC,			// BASIC ROM        $A000-$BFFF
	REG_KERNAL,			// KERNAL ROM       $E000-$FFFF
	REG_CHARROM,		// character ROM    $D000-$DFFF
	REG_IO,				// VIC/SID/CIA/colour RAM $D000-$DFFF
	REG_CART_ROML,		// cartridge ROML   $8000-$9FFF
	REG_CART_ROMH,		// cartridge ROMH   $A000-$BFFF or $E000-$FFFF
	REG_OPEN			// unmapped (Ultimax); reads float
};

// Banking inputs. 'port01' holds the effective LORAM/HIRAM/CHAREN levels in
// bits 0..2: a line whose DDR bit says "input" floats high and must be passed
// in here as 1, which is what c64PortEffective() below computes.
// 'game'/'exrom' are pin levels: 1 = high = inactive, 0 = pulled low.
typedef struct
{
	u8 port01;
	u8 game;
	u8 exrom;
} C64BankConfig;

// Build the index used by the lookup table: LORAM|HIRAM|CHAREN|GAME|EXROM.
static inline u8 c64BankMode( const C64BankConfig *cfg )
{
	return (u8)( ( cfg->port01 & 0x07 )
	           | ( ( cfg->game  & 1 ) << 3 )
	           | ( ( cfg->exrom & 1 ) << 4 ) );
}

// Resolve the effective port lines from the 6510's DDR ($00) and port ($01)
// registers. Bits configured as inputs read back as 1 because of the external
// pull-ups on LORAM/HIRAM/CHAREN.
static inline u8 c64PortEffective( u8 ddr00, u8 port01 )
{
	// Driven lines take the latch value, floating lines read high. Only bits
	// 0..2 are meaningful; the rest of the port drives the cassette and is of
	// no interest to banking. For the value software actually reads back from
	// $0001, use c64PortRead() instead -- this one deliberately throws the
	// other bits away.
	return (u8)( ( ( port01 & ddr00 ) | ~ddr00 ) & 0x07 );
}

// Level seen on each port pin when the 6510 has it configured as an input:
//   bits 0-2  LORAM/HIRAM/CHAREN, pulled high
//   bit  3    cassette write, an output in practice
//   bit  4    cassette sense, high with no button pressed
//   bit  5    cassette motor, an output in practice
#define C64_PORT_INPUT_LEVELS 0x17

// The byte software reads back from $0001. Output bits return the latch,
// input bits return the pin level. Bits 6 and 7 do not exist on the 6510 and
// read as 0.
static inline u8 c64PortRead( u8 ddr00, u8 port01 )
{
	return (u8)( ( ( port01 & ddr00 ) | ( C64_PORT_INPUT_LEVELS & ~ddr00 ) ) & 0x3F );
}

// [mode][page] -> C64Region, where page is addr >> 12. Built by c64BankingInit().
extern u8 c64RegionTable[ 32 ][ 16 ];

void c64BankingInit();

__attribute__((always_inline)) static inline C64Region c64MapRead( u16 addr, u8 mode )
{
	return (C64Region)c64RegionTable[ mode & 31 ][ addr >> 12 ];
}

// Writes never land in ROM: they fall through to the DRAM underneath. The only
// exception is the I/O window, where a write goes to the chips instead.
static inline bool c64WriteIsIO( u16 addr, u8 mode )
{
	return c64RegionTable[ mode & 31 ][ addr >> 12 ] == REG_IO;
}

// True when the VIC-II can fetch from this address, i.e. a write to it must be
// mirrored into real C64 DRAM or the display will not follow. The VIC sees the
// whole 64K through its four 16K banks, so from a correctness standpoint every
// RAM write qualifies; the SuperCPU's optimization modes exist precisely to
// narrow this down and buy back bus bandwidth. See SuperCPU/write_buffer.
static inline bool c64AddrIsVicVisible( u16 addr )
{
	(void)addr;
	return true;
}

#endif
