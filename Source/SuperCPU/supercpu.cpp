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
	: m_CPU( 0 ), m_Bus( 0 ), m_Running( false ), m_BootmapEnabled( false ),
	  m_FrameHook( 0 ), m_FrameHookCtx( 0 )
{
}

bool CSuperCPU::init( IC64Bus *bus, SCPUCoreType core, u32 simmMB )
{
	m_Bus = bus;

	if ( !m_Bus->acquire() )
		return false;

	m_Memory.attachBus( m_Bus );
	m_Memory.setMirrorSink( &m_WriteBuffer );
	m_Memory.setIOInterceptor( &m_Registers );

	m_WriteBuffer.attach( m_Bus, m_Memory.m_RAM );

	// SuperRAM, and the 24-bit space it lives in. Bank 0 of that space is the
	// C64 itself; everything above is the accelerator's own memory and never
	// touches the expansion port, which is why it runs at full speed.
	m_FastRAM.init( simmMB );
	m_MemoryMap.attachBank0( &m_Memory );
	m_MemoryMap.attachFastRAM( &m_FastRAM );
	m_Registers.attach( &m_WriteBuffer );
	// $D0B0 on an SCPU64 reports only the hardware revision; the 64-vs-128
	// distinction belongs to the SuperCPU 128's own register set, which we do
	// not emulate yet. The detected machine type is still used elsewhere for
	// video timing.
	m_Registers.setHardwareVersion( SCPU_V2 );

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

	// Which core, and -- just as importantly -- what it is attached to.
	//
	// The 65816 goes on the 24-bit memory map, so bank 0 is the C64 and
	// everything above it is SuperRAM. The 6502 goes straight to CC64Memory
	// instead: it has no way to name an address above $FFFF, so the map would be
	// a dispatch on the hot path that could never take its second branch.
	switch ( core )
	{
	case SCPU_CORE_65816:
		m_CPU = &m_Core65816;
		m_Core65816.attachFastBus( &m_MemoryMap );
		m_Registers.trackEmulationMode( &m_Core65816.m_E );
		break;

	case SCPU_CORE_6502:
	default:
		m_CPU = &m_Core6502;
		m_CPU->attachBus( &m_Memory );
		m_Registers.trackEmulationMode( 0 );
		m_Registers.setEmulationMode( true );
		break;
	}

	// Leave m_RMWDummyWrite alone -- i.e. ON, the NMOS 6510 behaviour.
	//
	// It is tempting to switch it off on the grounds that a 65816 runs an
	// internal cycle where a 6510 emits a dummy write of the original byte. But
	// the 65816 does that only in NATIVE mode. In EMULATION mode -- the mode a
	// C64 boots in, and the mode CM6502 is standing in for -- it emits the dummy
	// write exactly like a 6510, deliberately, because that is what a drop-in
	// 6502 replacement has to do. Measured 10000/10000 on INC abs, and VICE
	// gates the same behaviour on reg_emul.
	//
	// It matters: INC $D019 and LSR $D019 are the two standard VIC-II interrupt
	// acknowledgements. INC $D019 reads $81 and writes $82, and bit 0 of $82 is
	// clear -- so without the dummy write of the ORIGINAL $81 the raster latch
	// is never cleared and the handler re-enters forever. That is a hang, worth
	// far more than the ~1us the extra bus cycle costs.
	//
	// The flag stays as a knob for one open hardware question: whether the
	// SuperCPU's gate array forwards a cycle with VDA and VPA both low. If
	// raster interrupts ever misbehave in a way that points here, this is the
	// first thing to flip. See Docs/research/65816-reference.md section 6 and U2.

	// Hold the emulated CPU to real time. Without this a cycle count is not a
	// duration, and everything that measures time by counting -- IEC transfers,
	// CIA-timed loops, raster-chasing code -- behaves wrongly.
	m_Memory.setPacing( m_Bus->hostCyclesPerSec(), currentClockHz() );

	// Re-arm the pacer the moment a speed register is written, not at the next
	// frame boundary. CMD's KERNAL drops to 1MHz via $D07A immediately before
	// an IEC transaction; a frame-boundary catch-up leaves that transaction's
	// first 20ms running at turbo pacing.
	m_Registers.setSpeedHook( []( void *ctx )
	{
		CSuperCPU *self = (CSuperCPU *)ctx;
		self->m_Memory.setPacing( self->m_Bus->hostCyclesPerSec(),
		                          self->currentClockHz() );
		if ( self->m_CPU ) self->m_CPU->requestRunBreak();
	}, this );
	m_Memory.setTimingHook( []( void *ctx )
	{
		CSuperCPU *self = (CSuperCPU *)ctx;
		if ( self->m_CPU ) self->m_CPU->requestRunBreak();
	}, this );

	reset();
	return true;
}

void CSuperCPU::reset()
{
	m_WriteBuffer.flush();
	m_Memory.reset();

	// Bootmap, before the registers are reset -- CSuperCPURegisters::reset()
	// drives the live flag through trackBootmap(), so the ROM pointer has to be
	// in place first or the flag would come up set with nothing behind it.
	//
	// Requires at least 64K of image: bootmap indexes the EPROM 1:1 with the
	// 16-bit address, so anything smaller would read past the end. Without a
	// usable image the machine simply boots its own KERNAL, which is the
	// configuration everything up to now has been running in.
	const bool bootmapUsable = m_BootmapEnabled
	                        && m_MemoryMap.romImage() != 0
	                        && m_MemoryMap.romLength() >= 0x10000;

	m_Memory.setBootmapROM( bootmapUsable ? m_MemoryMap.romImage() : 0 );
	m_Registers.trackBootmap( bootmapUsable ? &m_Memory.m_BootmapActive : 0 );
	if ( !bootmapUsable )
		m_Memory.m_BootmapActive = false;

	// The BASIC and KERNAL windows are served from bank-1 SRAM, not from the
	// snapshotted ROMs -- see CC64Memory::setROMShadow(). Seed that SRAM with
	// the snapshots so a machine that never runs the accelerator's boot code
	// behaves exactly as it did before this existed: same ROMs, same banner.
	//
	// If the SuperCPU's own boot ROM does run, it overwrites these with its
	// patched copies, which is the entire point. Re-seeding on every reset is
	// deliberate: bootmap comes back on at reset too, so the boot code gets to
	// install its patches again, and anything left over from a previous run
	// cannot persist into a fresh boot.
	// Both KERNAL windows get seeded, because which one is live depends on
	// whether the register bank happens to be open: $E000 for KT, $6000 for KS.
	// Seeding only one leaves the machine reading uninitialised SRAM the moment
	// software opens the registers.
	for ( u32 i = 0; i < C64_BASIC_SIZE; i++ )
		m_MemoryMap.m_Bank1[ 0xA000 + i ] = m_Memory.m_Basic[ i ];
	for ( u32 i = 0; i < C64_KERNAL_SIZE; i++ )
	{
		m_MemoryMap.m_Bank1[ 0xE000 + i ] = m_Memory.m_Kernal[ i ];	// KT
		m_MemoryMap.m_Bank1[ 0x6000 + i ] = m_Memory.m_Kernal[ i ];	// KS
	}

	m_Memory.setROMShadow( m_MemoryMap.m_Bank1 );
	m_Registers.trackKernalShadow( &m_Memory.m_KernalShadowBase );

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

	// Account in 20MHz ticks: a turbo CPU cycle consumes one, while a 1MHz
	// cycle consumes twenty. A speed-register write breaks run() after its
	// current instruction, allowing the remainder to be recalculated instead
	// of finishing a turbo-sized frame budget at 1MHz.
	const u64 frameTicks = c64CyclesPerFrame * ( SCPU_TURBO_HZ / SCPU_NORMAL_HZ );
	u64 ticksUsed = 0;
	u64 executed = 0;

	// The frame is run in slices rather than one call, for two reasons. A
	// speed change breaks run() so the remainder can be re-budgeted at the new
	// rate. And the mirror buffer gets a flush OPPORTUNITY between slices: when
	// the machine falls behind real time, frame boundaries drift out of phase
	// with the real raster, and a flush attempted only at the boundary lands in
	// the safe border window by luck -- roughly a third of the time. Screen
	// writes then reach the C64 in visible ~quarter-second bursts. Eight
	// chances per frame instead of one makes hitting the border a near
	// certainty, at the cost of one raster read per slice.
	const u64 sliceTicks = frameTicks / 8;
	const u32 perLine = c64CyclesPerLine( sig.video );

	while ( ticksUsed < frameTicks )
	{
		const u32 selectedClockHz = currentClockHz();
		if ( m_Registers.consumeSpeedChanged() )
			m_Memory.setPacing( m_Bus->hostCyclesPerSec(), selectedClockHz );

		const u32 clockHz = m_Memory.iecThrottleActive()
		                  ? SCPU_NORMAL_HZ : selectedClockHz;
		const u32 ticksPerCycle = SCPU_TURBO_HZ / clockHz;
		const u64 ticksLeft = frameTicks - ticksUsed;
		u64 budget = ( ticksLeft + ticksPerCycle - 1 ) / ticksPerCycle;

		const u64 sliceBudget = ( sliceTicks + ticksPerCycle - 1 ) / ticksPerCycle;
		if ( budget > sliceBudget ) budget = sliceBudget;

		const u64 ran = m_CPU->run( budget );
		executed += ran;
		ticksUsed += ran * ticksPerCycle;

		// Opportunistic flush: one raster read, and only drain what fits in
		// the border time remaining. Never during the visible display.
		if ( !m_WriteBuffer.empty() && ticksUsed < frameTicks )
		{
			const u16 line = m_Bus->rasterLine();
			if ( line != 0xFFFF
			     && c64RasterIsSafeForBulkTransfer( sig.video, line ) )
			{
				const u32 lines = c64RasterLines( sig.video );
				u32 linesLeft = ( line > c64DisplayLastLine( sig.video ) )
				              ? ( lines - line ) + c64DisplayFirstLine( sig.video )
				              : c64DisplayFirstLine( sig.video ) - line;
				m_WriteBuffer.flushUpTo( ( linesLeft * perLine * 3 ) / 4 );
			}
		}
	}

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

		// Look once, briefly. Each poll costs two bus cycles, and the previous
		// version was willing to spend 400 of them -- up to 800us per frame with
		// the CPU stopped dead, which showed up as visible stuttering.
		//
		// If the beam is not somewhere useful within a short look, give up and
		// try next frame. Nothing is lost: the buffer holds its contents, the
		// safe window is over a third of every frame, and the CPU keeps running
		// in the meantime, which matters more than flushing promptly.
		u16 line = 0;
		bool safe = false;
		for ( u32 poll = 0; poll < 24; poll++ )
		{
			line = m_Bus->rasterLine();
			if ( line == 0xFFFF ) break;			// backend cannot tell
			if ( c64RasterIsSafeForBulkTransfer( sig.video, line ) ) { safe = true; break; }
		}

		if ( safe )
		{
			// Lines left before the display window resumes. Below the window we
			// have the bottom border, the vertical blank and the top border;
			// above it, only the remaining top border.
			u32 linesLeft = ( line > c64DisplayLastLine( sig.video ) )
			              ? ( lines - line ) + c64DisplayFirstLine( sig.video )
			              : c64DisplayFirstLine( sig.video ) - line;

			// One byte per C64 cycle, less a margin so the transfer finishes
			// before the display resumes rather than running into it.
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
