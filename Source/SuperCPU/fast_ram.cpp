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
	: m_RAM( 0 ), m_Size( 0 ), m_Mask( 0 )
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
	return true;
}
