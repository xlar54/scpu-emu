/*
   SCPU-EMU - CMD SuperCPU emulation for the C64/C128 using a RAD Expansion Unit
   Copyright (c) 2026 SCPU-EMU contributors

   CHostBus - an in-memory stand-in for the physical C64.

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
#include "host_bus.h"

CHostBus::CHostBus()
	: m_IRQ( false ), m_NMI( false ),
	  m_Cycles( 0 ), m_Reads( 0 ), m_Writes( 0 ), m_BurstWrites( 0 ),
	  m_LogCount( 0 ), m_LogEnabled( false )
{
	for ( u32 i = 0; i < 0x10000; i++ )
		m_Memory[ i ] = 0;

	m_Signals.machine       = MACHINE_C64;
	m_Signals.video         = VIDEO_PAL;
	m_Signals.gameAsserted  = false;
	m_Signals.exromAsserted = false;
}

bool CHostBus::acquire()
{
	return true;
}

void CHostBus::release()
{
}

void CHostBus::resetStats()
{
	m_Cycles = m_Reads = m_Writes = m_BurstWrites = 0;
	m_LogCount = 0;
}

void CHostBus::logAccess( u8 op, u16 addr, u8 value )
{
	if ( !m_LogEnabled )
		return;

	if ( m_LogCount < HOSTBUS_LOG_SIZE )
	{
		m_Log[ m_LogCount ].op    = op;
		m_Log[ m_LogCount ].addr  = addr;
		m_Log[ m_LogCount ].value = value;
		m_LogCount++;
	}
}

u8 CHostBus::read( u16 addr )
{
	m_Cycles++;
	m_Reads++;

	u8 v;

	// Just enough VIC-II to let a real KERNAL boot. Its start-up sits in
	// "LDA $D012 / BNE" at $FF5E waiting for rasterline 0, so a $D012 that never
	// changes hangs the machine. Everything else in the I/O window is still a
	// plain array -- this is not a VIC-II emulation, only a free-running raster
	// counter so that code which polls the beam makes progress.
	if ( addr == 0xD012 || addr == 0xD011 )
	{
		u16 raster = rasterLineInternal();

		if ( addr == 0xD012 )
			v = (u8)( raster & 0xFF );
		else
			// $D011 bit 7 is raster bit 8; the low bits belong to whatever the
			// program last wrote to the control register.
			v = (u8)( ( m_Memory[ 0xD011 ] & 0x7F ) | ( ( raster & 0x100 ) ? 0x80 : 0 ) );
	} else
	{
		v = m_Memory[ addr ];
	}

	logAccess( HOSTOP_READ, addr, v );
	return v;
}

void CHostBus::write( u16 addr, u8 value )
{
	m_Cycles++;
	m_Writes++;
	m_Memory[ addr ] = value;
	logAccess( HOSTOP_WRITE, addr, value );
}

void CHostBus::writeBurst( const C64BusWrite *writes, u32 count )
{
	for ( u32 i = 0; i < count; i++ )
	{
		m_Cycles++;
		m_BurstWrites++;
		m_Memory[ writes[ i ].addr ] = writes[ i ].value;
		logAccess( HOSTOP_BURST_WRITE, writes[ i ].addr, writes[ i ].value );
	}
}

void CHostBus::readBlock( u16 addr, u8 *dst, u32 length )
{
	for ( u32 i = 0; i < length; i++ )
	{
		m_Cycles++;
		m_Reads++;
		dst[ i ] = m_Memory[ (u16)( addr + i ) ];
	}
}

u16 CHostBus::rasterLineInternal() const
{
	// Derived from the bus cycle count so that code polling the beam makes
	// progress instead of spinning forever. It advances more slowly than a real
	// VIC-II, because only accesses that reach the bus tick it -- good enough
	// for "wait until the beam reaches line N", not for raster-exact timing.
	u32 perLine = c64CyclesPerLine( m_Signals.video );
	u32 lines   = c64RasterLines( m_Signals.video );
	return (u16)( ( m_Cycles / perLine ) % lines );
}

u16 CHostBus::rasterLine()
{
	m_Cycles++;
	m_Reads++;
	return rasterLineInternal();
}
