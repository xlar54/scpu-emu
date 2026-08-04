/*
   SCPU-EMU - CMD SuperCPU emulation for the C64/C128 using a RAD Expansion Unit
   Copyright (c) 2026 SCPU-EMU contributors

   C64 PLA banking model.

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
#include "banking.h"

u8 c64RegionTable[ 32 ][ 16 ];

void c64BankingInit()
{
	for ( u32 mode = 0; mode < 32; mode++ )
	{
		const bool loram  = ( mode & 0x01 ) != 0;
		const bool hiram  = ( mode & 0x02 ) != 0;
		const bool charen = ( mode & 0x04 ) != 0;
		const bool game   = ( mode & 0x08 ) != 0;	// pin level, 1 = inactive
		const bool exrom  = ( mode & 0x10 ) != 0;

		// Ultimax: /GAME low while /EXROM stays high. The cartridge replaces
		// almost the entire map, which is why Action Replay-style freezers and
		// a SuperCPU cannot coexist.
		const bool ultimax = ( !game && exrom );

		for ( u32 page = 0; page < 16; page++ )
		{
			C64Region r = REG_RAM;

			if ( ultimax )
			{
				if ( page == 0x0 )                 r = REG_RAM;
				else if ( page >= 0x8 && page <= 0x9 ) r = REG_CART_ROML;
				else if ( page == 0xD )            r = REG_IO;
				else if ( page >= 0xE )            r = REG_CART_ROMH;
				else                               r = REG_OPEN;
			} else
			{
				switch ( page )
				{
				case 0x8:
				case 0x9:
					// ROML appears only in the 8K/16K cartridge configurations.
					if ( !exrom && loram && hiram ) r = REG_CART_ROML;
					break;

				case 0xA:
				case 0xB:
					if ( !game && !exrom && loram && hiram )
						r = REG_CART_ROMH;			// 16K cartridge
					else if ( game && loram && hiram )
						r = REG_BASIC;
					break;

				case 0xD:
					// RAM only when both LORAM and HIRAM are low; otherwise
					// CHAREN picks between the character ROM and the I/O chips.
					if ( !loram && !hiram )  r = REG_RAM;
					else if ( !charen )      r = REG_CHARROM;
					else                     r = REG_IO;
					break;

				case 0xE:
				case 0xF:
					if ( hiram ) r = REG_KERNAL;
					break;

				default:
					r = REG_RAM;
					break;
				}
			}

			c64RegionTable[ mode ][ page ] = (u8)r;
		}
	}
}
