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
	} else if ( addr == 0xD019 && m_CycleSource )
	{
		// Unused bits of the interrupt register read as 1 on a real VIC, and a
		// handler that tests them cares. Bit 7 is the summary bit.
		advanceRaster();
		v = (u8)( 0x70 | ( m_RasterLatch ? 0x01 : 0 ) );
		if ( m_RasterLatch && ( m_Memory[ 0xD01A ] & 0x01 ) ) v |= 0x80;
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
	// The VIC clears exactly the latch bits that are SET in the written byte.
	// Modelled strictly, because the strictness is the interesting part: the
	// classic ASL $D019 idiom only acknowledges by way of the NMOS
	// read-modify-write dummy write, which carries the original value with bit
	// 0 still set. The shifted value that follows it has bit 0 clear. A 65816
	// omits that dummy write, so the same instruction does NOT acknowledge --
	// which is a real SuperCPU compatibility hazard, and one this bus will now
	// reproduce rather than paper over.
	if ( addr == 0xD019 && m_CycleSource )
	{
		advanceRaster();
		if ( value & 0x01 ) m_RasterLatch = false;
	}
	m_Memory[ addr ] = value;
	logAccess( HOSTOP_WRITE, addr, value );
}

u8 CHostBus::readRAM( u16 addr )
{
	// The host bus has no PLA or chip overlay; preserve ordinary read accounting
	// and logging so tests can still assert on the exact physical operation.
	return read( addr );
}

void CHostBus::writeRAM( u16 addr, u8 value )
{
	// The host bus has no overlay; preserve normal write accounting and logging.
	write( addr, value );
}

bool CHostBus::verifyC64CIA2DDRA( u8 expected )
{
	// The host backend has no marginal expansion bus, but use the ordinary
	// read path so tests retain exact access accounting and can override this
	// hook to exercise a failed hardware verification.
	return read( 0xDD02 ) == expected;
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
	u32 perLine = c64CyclesPerLine( m_Signals.video );
	u32 lines   = c64RasterLines( m_Signals.video );

	// With a cycle source the beam runs on emulated C64 time, which is the only
	// thing a raster split can be measured against.
	if ( m_CycleSource )
		return (u16)( ( m_CycleSource( m_CycleContext ) / perLine ) % lines );

	// Otherwise derive it from the bus cycle count so that code polling the
	// beam makes progress instead of spinning forever. It advances more slowly
	// than a real VIC-II, because only accesses that reach the bus tick it --
	// good enough for "wait until the beam reaches line N", not for
	// raster-exact timing.
	return (u16)( ( m_Cycles / perLine ) % lines );
}

void CHostBus::setRasterClock( CycleSource source, void *context )
{
	m_CycleSource = source;
	m_CycleContext = context;
	m_RasterLatch = false;
	m_RasterIRQsRaised = 0;
	m_RasterSeenLine = source
	                 ? source( context ) / c64CyclesPerLine( m_Signals.video )
	                 : 0;
}

void CHostBus::advanceRaster()
{
	if ( !m_CycleSource ) return;

	const u32 perLine = c64CyclesPerLine( m_Signals.video );
	const u32 lines   = c64RasterLines( m_Signals.video );
	const u64 nowLine = m_CycleSource( m_CycleContext ) / perLine;
	if ( nowLine <= m_RasterSeenLine ) return;

	// Absolute line numbers are monotonic, so "was the compare crossed?" is
	// exact no matter how coarsely this is sampled -- no tick loop, and no way
	// for a split to be missed because nobody looked during its line.
	const u16 compare = (u16)( m_Memory[ 0xD012 ]
	                  | ( ( m_Memory[ 0xD011 ] & 0x80 ) ? 0x100 : 0 ) );
	if ( compare < lines )
	{
		// First absolute line strictly after the last one accounted for that is
		// congruent to the compare line.
		const u64 first = m_RasterSeenLine + 1;
		u64 delta = ( (u64)compare + lines - ( first % lines ) ) % lines;
		if ( first + delta <= nowLine )
		{
			if ( !m_RasterLatch ) m_RasterIRQsRaised++;
			m_RasterLatch = true;
		}
	}

	m_RasterSeenLine = nowLine;
}

bool CHostBus::irqAsserted()
{
	if ( m_CycleSource )
	{
		advanceRaster();
		if ( m_RasterLatch && ( m_Memory[ 0xD01A ] & 0x01 ) ) return true;
	}
	return m_IRQ;
}

// THE BADLINE RULE, written down once, in the stand-in that has no BA pin.
// The firmware never runs this: CRADBus::busStealCycles reads the real VIC's
// BA instead, because a model that disagrees with the VIC is worse than no
// model. This exists so the rule can be TESTED against the published rule, and
// so the stall can be exercised without a cartridge.
//
// SOURCE: Christian Bauer, "The MOS 6567/6569 video controller (VIC-II) and
// its application in the Commodore 64", section 3.5 -- a Bad Line Condition
// holds when RASTER >= $30 and RASTER <= $F7, (RASTER & 7) == YSCROLL
// ($D011 bits 0-2), and DEN ($D011 bit 4) was set at some point in raster line
// $30. Sections 3.6.3 and 3.14.6 give the placement: the VIC performs 40
// c-accesses in cycles 15-54 and pulls BA low three cycles ahead of the first
// one, so the CPU is held from cycle 12 through 54 and resumes at 55 -- about
// 43 cycles. A VSP-created badline (the guest writing $D011 mid-line so the
// condition becomes true late) starts the steal at that write, which is why
// Mario's INC resumes at 54 and completes at 59.
//
// THREE deliberate simplifications, all only reachable here, and the first two
// are what make this a BOUND rather than an identity -- in OPPOSITE directions,
// which is exactly why the firmware reads the pin instead of this.
//   AEC   Bauer 3.6.3: AEC stays high for three cycles after BA falls, so a
//         real 6510 can still finish up to three WRITE cycles in 12..14. This
//         model freezes it at 12, OVER-stating a steal entered in those three
//         cycles by up to three cycles. A stall that is too long is as bad as
//         none, so this is named rather than left implicit.
//   SPRITES  not modelled at all, which UNDER-states any line carrying sprite
//         DMA. This is the larger error and the one the sibling project's
//         negative result cares about (stall more, never less).
//   DEN   read live rather than latched at line $30, because this stand-in
//         keeps no per-frame VIC state; it matters only to a program that
//         toggles DEN mid-frame.
u32 CHostBus::busStealCycles( u32 maxCycles )
{
	// Without an emulated-cycle source the raster counter advances on bus
	// accesses, which cannot place a cycle inside a line; refuse rather than
	// invent a steal.
	if ( !m_CycleSource || !maxCycles ) return 0;

	const u32 perLine = c64CyclesPerLine( m_Signals.video );
	const u32 lines   = c64RasterLines( m_Signals.video );
	if ( !perLine || !lines ) return 0;
	const u64 now  = m_CycleSource( m_CycleContext );
	const u32 line = (u32)( ( now / perLine ) % lines );
	// Bauer numbers the cycles of a line from 1, and the VSP trace's
	// cycle-in-line figures are quoted in that same frame, so keep it here
	// instead of converting at every comparison.
	const u32 cycle = (u32)( now % perLine ) + 1;

	const u8 d011 = m_Memory[ 0xD011 ];
	if ( line < 0x30 || line > 0xF7 ) return 0;
	if ( !( d011 & 0x10 ) ) return 0;
	if ( ( line & 7 ) != (u32)( d011 & 7 ) ) return 0;
	if ( cycle < HOSTBUS_BADLINE_BA_LOW || cycle > HOSTBUS_BADLINE_BA_LAST )
		return 0;

	u32 stolen = HOSTBUS_BADLINE_BA_LAST + 1 - cycle;
	if ( stolen > maxCycles ) stolen = maxCycles;
	return stolen;
}

u16 CHostBus::rasterLine()
{
	m_Cycles++;
	m_Reads++;
	return rasterLineInternal();
}
