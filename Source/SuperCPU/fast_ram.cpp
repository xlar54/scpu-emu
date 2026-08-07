/*
   SCPU-EMU - CMD SuperCPU emulation for the C64/C128 using a RAD Expansion Unit
   Copyright (c) 2026 SCPU-EMU contributors

   SuperRAM: the SIMM the SuperCPU carries on its expansion card.

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
#include "fast_ram.h"

#ifdef SCPU_HOST_BUILD
	#include <stdlib.h>
	#include <string.h>
#else
	#include <circle/util.h>
	#include <stdlib.h>
#endif

CFastRAM::CFastRAM()
	: m_RAM( 0 ), m_Size( 0 ), m_Mask( 0 ), m_DecodeMask( 0 ),
	  m_RowMask( ~( 8192u - 1 ) ), m_LastCell( 0 ), m_Config( 4 )
{
}

CFastRAM::~CFastRAM()
{
	if ( m_RAM )
		free( m_RAM );
}

bool CFastRAM::init( u32 megabytes )
{
	if ( m_RAM )
	{
		free( m_RAM );
		m_RAM = 0;
	}
	m_Size = 0;
	m_Mask = 0;
	m_DecodeMask = 0;

	// Round down to a size the hardware would actually accept.
	u32 mb;
	if      ( megabytes >= SCPU_SIMM_16MB ) mb = SCPU_SIMM_16MB;
	else if ( megabytes >= SCPU_SIMM_8MB )  mb = SCPU_SIMM_8MB;
	else if ( megabytes >= SCPU_SIMM_4MB )  mb = SCPU_SIMM_4MB;
	else if ( megabytes >= SCPU_SIMM_1MB )  mb = SCPU_SIMM_1MB;
	else                                    return true;	// no card fitted

	const u32 bytes = mb << 20;

	m_RAM = (u8 *)malloc( bytes );
	if ( !m_RAM )
		return false;

	// Power-on state. Real DRAM comes up with whatever it comes up with, but
	// zeroing is reproducible, and software that cares does its own RAM test.
	memset( m_RAM, 0, bytes );

	m_Size = bytes;
	m_Mask = bytes - 1;		// every accepted size is a power of two
	setConfig( 4 );
	return true;
}

void CFastRAM::setConfig( u8 value )
{
	m_Config = value;
	u32 configuredBytes;
	u32 rowBytes;
	switch ( value & 7 )
	{
	case 0: configuredBytes =  1u << 20; rowBytes = 2048; break;
	case 1: configuredBytes =  4u << 20; rowBytes = 4096; break;
	case 2: configuredBytes =  8u << 20; rowBytes = 4096; break;
	case 3: configuredBytes = 16u << 20; rowBytes = 4096; break;
	default: configuredBytes = 16u << 20; rowBytes = 8192; break;
	}
	const u32 decoded = ( m_Size && m_Size < configuredBytes ) ? m_Size : configuredBytes;
	m_DecodeMask = decoded ? decoded - 1 : 0;
	m_RowMask = ~( rowBytes - 1 );
	m_LastCell = 0;
}

u32 CFastRAM::accessPenaltyHalfCycles( u32 offset, bool write )
{
	if ( !m_Size ) return 0;
	const u32 addr = offset & m_DecodeMask;
	if ( write )
	{
		const u32 cost = ( ( m_LastCell ^ addr ) & m_RowMask ) ? 4 : 2;
		m_LastCell = addr;
		return cost;
	}
	if ( !( ( m_LastCell ^ addr ) & ~3u ) )
		return 0;
	if ( ( m_LastCell ^ addr ) & m_RowMask )
	{
		m_LastCell = addr;
		return 5;
	}
	if ( !( ( ( m_LastCell + 4 ) ^ addr ) & ~3u ) )
	{
		m_LastCell = addr;
		return 0;
	}
	m_LastCell = addr;
	return 2;
}
