/*
   SCPU-EMU - CMD SuperCPU emulation for the C64/C128 using a RAD Expansion Unit
   Copyright (c) 2026 SCPU-EMU contributors

   SuperRAM: the SIMM the SuperCPU carries on its expansion card.

   On real hardware this is a single 72-pin fast-page-mode SIMM of 1 to 16MB,
   70ns or faster. Here it is just memory belonging to the Raspberry Pi, which
   is the one part of a SuperCPU that is genuinely easier to emulate than to
   build.

   It is only reachable through 24-bit addressing, so nothing can touch it until
   the 65816 core lands -- a 6502 has no way to name an address above $FFFF.

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
#ifndef _scpu_fast_ram_h
#define _scpu_fast_ram_h

#include "../Common/types.h"

// The sizes a real SuperCPU accepts, in megabytes. VICE offers the same set.
#define SCPU_SIMM_NONE   0
#define SCPU_SIMM_1MB    1
#define SCPU_SIMM_4MB    4
#define SCPU_SIMM_8MB    8
#define SCPU_SIMM_16MB  16

class CFastRAM
{
public:
	CFastRAM();
	~CFastRAM();

	// Allocate the SIMM. Sizes other than 0/1/4/8/16 are rounded down to the
	// next valid one, matching what the hardware would accept. Returns false if
	// the allocation failed, in which case the card is simply absent.
	bool init( u32 megabytes );

	u32  sizeBytes() const { return m_Size; }
	u32  sizeMB() const    { return m_Size >> 20; }
	bool present() const   { return m_Size != 0; }
	void setConfig( u8 value );
	u8   config() const { return m_Config; }
	u32  accessPenaltyHalfCycles( u32 offset, bool write );

	// Reads and writes are masked into range rather than bounds-checked and
	// rejected. A real SIMM decodes a fixed number of address lines, so an
	// address beyond the fitted size aliases onto lower memory; software that
	// probes for size relies on exactly that behaviour.
	inline u8 read( u32 offset ) const
	{
		return m_Size ? m_RAM[ offset & m_DecodeMask ] : 0x00;
	}

	inline void write( u32 offset, u8 value )
	{
		if ( m_Size ) m_RAM[ offset & m_DecodeMask ] = value;
	}

	u8 *raw() { return m_RAM; }

private:
	u8 *m_RAM;
	u32 m_Size;
	u32 m_Mask;
	u32 m_DecodeMask;
	u32 m_RowMask;
	u32 m_LastCell;
	u8  m_Config;
};

#endif
