/*
   SCPU-EMU - CMD SuperCPU emulation for the C64/C128 using a RAD Expansion Unit
   Copyright (c) 2026 SCPU-EMU contributors

   The two small pieces that connect CREU to a machine.

   CIOInterceptorChain exists because CC64Memory holds exactly ONE
   IIOInterceptor and CSuperCPURegisters already occupies it. Rather than widen
   that interface -- which is on a hot path and owned by the memory layer -- the
   chain presents itself as a single interceptor and forwards to two.

   CREUMemoryHost is the other half: it hands the REU a way to move bytes to and
   from C64 memory without the REU knowing what a CC64Memory is.

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
#ifndef _scpu_reu_wiring_h
#define _scpu_reu_wiring_h

#include "reu.h"
#include "../C64/c64_memory.h"

// Presents two interceptors as one.
//
// ORDER MATTERS AND IS NOT ARBITRARY. The primary is asked first and the
// secondary only sees what the primary declined. CSuperCPURegisters must be
// primary: it observes $DF7E/$DF7F for RAMLink and deliberately returns false
// so the write still continues to the cartridge bus. If the REU were asked
// first it would decline those too (they are outside $DF00-$DF0A), so the order
// happens not to matter for that pair today -- but it will the moment either
// device claims a range the other also watches, and a chain whose behaviour
// depends on nobody ever overlapping is a trap.
class CIOInterceptorChain : public IIOInterceptor
{
public:
	CIOInterceptorChain() : m_Primary( 0 ), m_Secondary( 0 ) {}

	void attach( IIOInterceptor *primary, IIOInterceptor *secondary )
	{
		m_Primary = primary;
		m_Secondary = secondary;
	}

	bool ioRead( u16 addr, u8 &value ) override
	{
		if ( m_Primary && m_Primary->ioRead( addr, value ) ) return true;
		if ( m_Secondary && m_Secondary->ioRead( addr, value ) ) return true;
		return false;
	}

	bool ioWrite( u16 addr, u8 value ) override
	{
		if ( m_Primary && m_Primary->ioWrite( addr, value ) ) return true;
		if ( m_Secondary && m_Secondary->ioWrite( addr, value ) ) return true;
		return false;
	}

	// The timing and policy queries below are asked ABOUT an address rather
	// than performed on it, so there is no "handled" answer to chain on. The
	// primary owns them: it is the accelerator's own register file, and its
	// answers describe the machine. The REU adds no stretch policy of its own.
	bool ioAccessNeedsStretch( u16 a, bool w ) const override
		{ return m_Primary ? m_Primary->ioAccessNeedsStretch( a, w ) : w; }
	bool ioAccessUsesWriteBuffer( u16 a, bool w ) const override
		{ return m_Primary ? m_Primary->ioAccessUsesWriteBuffer( a, w ) : false; }
	bool dosExtensionEnabled() const override
		{ return m_Primary ? m_Primary->dosExtensionEnabled() : false; }

private:
	IIOInterceptor *m_Primary;
	IIOInterceptor *m_Secondary;
};

// Presents a CREU as an IIOInterceptor.
//
// CREU deliberately does NOT derive from IIOInterceptor: that interface lives
// in the memory layer, and inheriting it would drag c64_memory.h into a class
// whose whole value is being pure logic testable without a C64. The adapter
// costs one virtual call on an access that was already going to a device, and
// keeps that separation intact.
class CREUInterceptor : public IIOInterceptor
{
public:
	CREUInterceptor() : m_REU( 0 ) {}
	void attach( CREU *reu ) { m_REU = reu; }

	bool ioRead( u16 addr, u8 &value ) override
	{
		return m_REU ? m_REU->read( addr, value ) : false;
	}
	bool ioWrite( u16 addr, u8 value ) override
	{
		return m_REU ? m_REU->write( addr, value ) : false;
	}

private:
	CREU *m_REU;
};

// Moves REU transfer bytes through the ordinary memory path.
//
// write8() is used rather than a direct array store ON PURPOSE. A real REU
// writes into the DRAM the VIC-II fetches from, so a transfer into screen or
// bitmap memory has to reach the physical machine exactly as a CPU store would.
// Writing only the Pi's shadow would produce a machine that believes the
// transfer happened while the picture never changes -- the same failure the
// $D000-$DFFF suppression produced, and just as hard to recognise.
//
// It also means an REU transfer costs mirror bus traffic. That is correct: on
// real hardware the transfer really does occupy the bus, which is why the unit
// holds the CPU off it for the duration.
class CREUMemoryHost : public IREUHost
{
public:
	CREUMemoryHost() : m_Memory( 0 ) {}
	void attach( CC64Memory *memory ) { m_Memory = memory; }

	u8 reuHostRead( u16 addr ) override
	{
		return m_Memory ? m_Memory->read8( addr ) : 0xFF;
	}

	void reuHostWrite( u16 addr, u8 value ) override
	{
		if ( m_Memory ) m_Memory->write8( addr, value );
	}

private:
	CC64Memory *m_Memory;
};

#endif
