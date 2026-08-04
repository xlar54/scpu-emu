/*
   SCPU-EMU - CMD SuperCPU emulation for the C64/C128 using a RAD Expansion Unit
   Copyright (c) 2026 SCPU-EMU contributors

   CSuperCPU - the accelerator itself.

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
#include "supercpu.h"

CSuperCPU::CSuperCPU()
	: m_CPU( 0 ), m_Bus( 0 ), m_Running( false ),
	  m_FrameHook( 0 ), m_FrameHookCtx( 0 )
{
}

bool CSuperCPU::init( IC64Bus *bus, SCPUCoreType core )
{
	m_Bus = bus;

	if ( !m_Bus->acquire() )
		return false;

	m_Memory.attachBus( m_Bus );
	m_Memory.setMirrorSink( &m_WriteBuffer );
	m_Memory.setIOInterceptor( &m_Registers );

	m_WriteBuffer.attach( m_Bus, m_Memory.m_RAM );
	m_Registers.attach( &m_WriteBuffer );
	m_Registers.setC128Mode( m_Bus->signals().machine == MACHINE_C128 );

	// Fill in whichever ROM images the caller did not supply by reading them off
	// the live machine. This is the zero-configuration path: no files needed,
	// and the KERNAL you end up running is the one actually fitted to the
	// computer. Supplying one image and not the other is fine.
	if ( !m_Memory.snapshotROMsFromBus() )
	{
		// Never leave the machine halted under /DMA because we failed to start.
		// The 6510 is off the bus until somebody releases it, and if we simply
		// return the C64 sits frozen with no way back short of a power cycle.
		m_Bus->release();
		return false;
	}

	switch ( core )
	{
	case SCPU_CORE_65816:
		// Not yet implemented -- fall back rather than run with no core.
		// See Docs/roadmap.md.
		m_CPU = &m_Core6502;
		break;

	case SCPU_CORE_6502:
	default:
		m_CPU = &m_Core6502;
		break;
	}

	// The accelerator's CPU is a 65816, which runs an internal cycle where an
	// NMOS 6510 emits a dummy write of the original byte. Suppress the dummy
	// write so the milestone-1 core behaves like the chip we are standing in
	// for -- and so an RMW against I/O does not cost a second ~1us bus cycle.
	m_Core6502.m_RMWDummyWrite = false;

	m_CPU->attachBus( &m_Memory );

	// Hold the emulated CPU to real time. Without this a cycle count is not a
	// duration, and everything that measures time by counting -- IEC transfers,
	// CIA-timed loops, raster-chasing code -- behaves wrongly.
	m_Memory.setPacing( m_Bus->hostCyclesPerSec(), currentClockHz() );

	reset();
	return true;
}

void CSuperCPU::reset()
{
	m_WriteBuffer.flush();
	m_Memory.reset();
	m_Registers.reset();

	// Put the mirroring policy back to its power-on state too, or a reset would
	// inherit whatever optimization mode the previous program happened to leave
	// selected.
	m_WriteBuffer.setExcludeZeroPageStack( true );
	m_WriteBuffer.setOptMode( SCPU_OPT_DEFAULT );
	m_WriteBuffer.resetStats();

	m_Memory.resyncPacing();

	if ( m_CPU )
		m_CPU->reset();
}

u32 CSuperCPU::currentClockHz() const
{
	return m_Registers.turboEnabled() ? SCPU_TURBO_HZ : SCPU_NORMAL_HZ;
}

u64 CSuperCPU::runFrame()
{
	if ( !m_CPU || !m_Bus )
		return 0;

	// One frame of C64 time, scaled by the accelerator's current speed: in
	// turbo we execute twenty times as many instructions per frame as the host
	// CPU would have.
	const C64Signals &sig = m_Bus->signals();
	const u64 c64CyclesPerFrame =
		(u64)c64CyclesPerLine( sig.video ) * (u64)c64RasterLines( sig.video );

	const u32 clockHz = currentClockHz();
	const u64 budget  = c64CyclesPerFrame * ( clockHz / SCPU_NORMAL_HZ );

	// A write to $D07A/$D07B changes how long an emulated cycle is worth in
	// real time, so the pacer has to be told. This is what finally makes the
	// SuperCPU's speed selection mean something.
	if ( m_Registers.consumeSpeedChanged() )
		m_Memory.setPacing( m_Bus->hostCyclesPerSec(), clockHz );

	u64 executed = m_CPU->run( budget );

	// --- raster-scheduled mirroring ----------------------------------------
	// Mirrored writes must not be pushed across the visible display. While the
	// VIC-II is fetching character, colour and bitmap data, bulk traffic on the
	// bus corrupts those fetches and the picture will not hold -- which is
	// exactly the failure this replaces. RAD-Doom, which moves ~10KB a frame
	// without disturbing the display, does the same thing: it works out how
	// many scanlines its transfer needs and waits so it lands in the border and
	// vertical blank.
	//
	// So: wait for the beam to leave the display window, then send only as much
	// as fits in the time remaining before it comes back.
	if ( !m_WriteBuffer.empty() )
	{
		const u32 perLine = c64CyclesPerLine( sig.video );
		const u32 lines   = c64RasterLines( sig.video );

		// Bounded wait. Each poll costs two bus cycles, so we cap it rather
		// than spin for a whole frame if something is wrong with the raster.
		u16 line = 0;
		bool safe = false;
		for ( u32 poll = 0; poll < 400; poll++ )
		{
			line = m_Bus->rasterLine();
			if ( line == 0xFFFF ) break;			// backend cannot tell
			if ( c64RasterIsSafeForBulkTransfer( sig.video, line ) ) { safe = true; break; }
		}

		if ( safe )
		{
			// Lines left before the display window resumes. Below the window we
			// have the bottom border plus vertical blank plus the top border;
			// above it, just the remaining top border.
			u32 linesLeft = ( line > c64DisplayLastLine( sig.video ) )
			              ? ( lines - line ) + c64DisplayFirstLine( sig.video )
			              : c64DisplayFirstLine( sig.video ) - line;

			// One byte per C64 cycle, less a margin so we finish before the
			// display resumes rather than running into it.
			u32 budgetBytes = ( linesLeft * perLine * 3 ) / 4;

			m_WriteBuffer.flushUpTo( budgetBytes );
		} else if ( line == 0xFFFF )
		{
			m_WriteBuffer.flush();				// host backend: no raster to respect
		}
	}

	// Pacing is no longer done here. It happens after every instruction in
	// CC64Memory::tick(), because frame granularity (~20ms) is far too coarse
	// for anything that measures time by counting cycles -- the KERNAL's IEC
	// routines bit-bang the serial lines to microsecond tolerances.

	return executed;
}

void CSuperCPU::run()
{
	m_Running = true;
	while ( m_Running )
	{
		runFrame();

		if ( m_FrameHook && m_FrameHook( m_FrameHookCtx ) )
			m_Running = false;
	}
}
