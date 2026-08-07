/*
   SCPU-EMU - CMD SuperCPU emulation for the C64/C128 using a RAD Expansion Unit
   Copyright (c) 2026 SCPU-EMU contributors

   The SuperCPU's 24-bit address space. Layout per VICE; see memory_map.h.

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
#include "memory_map.h"

CSuperCPUMemoryMap::CSuperCPUMemoryMap()
	: m_Bank0Accesses( 0 ), m_FastAccesses( 0 ),
	  m_Bank0( 0 ), m_SIMM( 0 ), m_ROM( 0 ), m_ROMLength( 0 )
{
	for ( u32 i = 0; i < SCPU_BANK1_SIZE; i++ )
		m_Bank1[ i ] = 0;
}

void CSuperCPUMemoryMap::setROM( const u8 *image, u32 length )
{
	m_ROM = image;
	m_ROMLength = ( length > SCPU_ROM_MAXSIZE ) ? SCPU_ROM_MAXSIZE : length;
}

void CSuperCPUMemoryMap::reset()
{
	m_Bank0Accesses = m_FastAccesses = 0;

	for ( u32 i = 0; i < SCPU_BANK1_SIZE; i++ )
		m_Bank1[ i ] = 0;
}

u8 CSuperCPUMemoryMap::readBank1Port( u16 offset )
{
	return m_Bank0 ? m_Bank0->read8( offset ) : 0xFF;
}

void CSuperCPUMemoryMap::writeBank1Port( u16 offset, u8 value )
{
	if ( m_Bank0 ) m_Bank0->write8( offset, value );
}

// ---------------------------------------------------------------------------

// Everything ABOVE bank 0. Bank 0 itself is inlined in the header -- see the
// note there.
u8 CSuperCPUMemoryMap::readAbove( scpu_addr_t addr )
{
	m_FastAccesses++;

	if ( addr < 0x020000 )
		return m_Bank1[ addr - 0x010000 ];

	if ( addr >= SCPU_ROM_BASE )
	{
		if ( m_Bank0 ) m_Bank0->noteFastHalfCycles( 6 );
		if ( !m_ROMLength )
			return (u8)( addr >> 16 );			// no image fitted: open bus
		return m_ROM[ ( addr - SCPU_ROM_BASE ) % m_ROMLength ];
	}

	if ( addr >= SCPU_SIMM_WINDOW && addr < SCPU_SIMM_WINDOW + SCPU_SIMM_WINDOW_SZ )
	{
		if ( !m_SIMM || !m_SIMM->present() )
			return (u8)( addr >> 16 );
		const u32 offset = addr - SCPU_SIMM_WINDOW;
		if ( m_Bank0 ) m_Bank0->noteFastHalfCycles(
			m_SIMM->accessPenaltyHalfCycles( offset, false ) );
		return m_SIMM->read( offset );
	}

	// SuperRAM proper. Banks $02 upward map linearly onto the SIMM, and an
	// address beyond the fitted size aliases down -- which is how software
	// discovers how much is installed.
	if ( m_SIMM && m_SIMM->present() && addr < m_SIMM->sizeBytes() )
	{
		if ( m_Bank0 ) m_Bank0->noteFastHalfCycles(
			m_SIMM->accessPenaltyHalfCycles( addr, false ) );
		return m_SIMM->read( addr );
	}

	// Nothing decodes here. A 65816 leaves the bank byte on the bus.
	return (u8)( addr >> 16 );
}

// Everything ABOVE bank 0; bank 0 is inlined in the header.
void CSuperCPUMemoryMap::writeAbove( scpu_addr_t addr, u8 value )
{
	m_FastAccesses++;

	if ( addr < 0x020000 )
	{
		m_Bank1[ addr - 0x010000 ] = value;
		return;
	}

	if ( addr >= SCPU_ROM_BASE )
	{
		if ( m_Bank0 ) m_Bank0->noteFastHalfCycles( 6 );
		return;									// ROM: writes are discarded
	}

	if ( addr >= SCPU_SIMM_WINDOW && addr < SCPU_SIMM_WINDOW + SCPU_SIMM_WINDOW_SZ )
	{
		if ( m_Bank0 && !m_Bank0->simmWindowWritesEnabled() )
			return;
		if ( m_SIMM )
		{
			const u32 offset = addr - SCPU_SIMM_WINDOW;
			if ( m_Bank0 ) m_Bank0->noteFastHalfCycles(
				m_SIMM->accessPenaltyHalfCycles( offset, true ) );
			m_SIMM->write( offset, value );
		}
		return;
	}

	if ( m_SIMM && m_SIMM->present() && addr < m_SIMM->sizeBytes() )
	{
		if ( m_Bank0 ) m_Bank0->noteFastHalfCycles(
			m_SIMM->accessPenaltyHalfCycles( addr, true ) );
		m_SIMM->write( addr, value );
	}
}
