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
	  m_C128Mode( SCPU_C128_MODE_AUTO ),
	  m_MirrorDisplayBudget( SCPU_MIRROR_DISPLAY_BYTES_DEFAULT ),
	  m_FrameHook( 0 ), m_FrameHookCtx( 0 )
{
}

void CSuperCPU::disablePhysicalMirror()
{
	// Core 1 presents directly from shadow RAM, so no later write may enter the
	// physical delivery queue. Keep the buffer attached as a timing-only policy
	// sink: the real SuperCPU still has a one-deep posted C64-RAM write slot even
	// when this implementation deliberately sends no corresponding VIC byte.
	m_WriteBuffer.discard();
	m_WriteBuffer.setDeliveryEnabled( false );
	m_Memory.setMirrorSink( &m_WriteBuffer );
	m_Memory.enablePostedWriteTiming( true );
	m_MirrorHalted = true;
}

void CSuperCPU::enablePhysicalMirror()
{
	// Forget any timing-only history, restore the ordinary delivery policy, and
	// rebuild just the VIC-visible working set. runFrame() performs the actual
	// writes in its established raster-safe installments; this call itself does
	// not touch the physical bus.
	// Collision bits accumulated by the physical VIC while it displayed stale
	// memory must not leak into the newly-selected mode's first guest read.
	m_Memory.clearPhysicalVICCollisionLatches();
	m_WriteBuffer.discard();
	m_WriteBuffer.setDeliveryEnabled( true );
	m_Memory.setMirrorSink( &m_WriteBuffer );
	m_Memory.enablePostedWriteTiming( false );
	m_MirrorHalted = false;

	const u32 screen = m_Memory.activeScreenBase();
	if ( screen < 0x10000 )
		m_WriteBuffer.invalidateRange( (u16)screen, 1024 );

	const u32 bitmap = m_Memory.activeBitmapBase();
	if ( bitmap < 0x10000 )
		m_WriteBuffer.invalidateRange( (u16)bitmap, 8192 );
	else
	{
		const u32 charset = m_Memory.activeCharsetBase();
		if ( charset < 0x10000 )
			m_WriteBuffer.invalidateRange( (u16)charset, 2048 );
	}

	// Sprite data is independent of text/bitmap mode. Read pointers from the
	// authoritative Pi shadow even when the matrix itself is under I/O; the
	// existing relocation policy handles any shape selected from $D000-$DFFF.
	const u8 enabled = m_Memory.vicRegister( 0x15 );
	const u32 logicalScreen = m_Memory.activeVICScreenBase();
	const u32 bank = m_Memory.activeVICBankBase();
	const u8 *ram = m_Memory.ramShadow();
	for ( u32 sprite = 0; sprite < 8; sprite++ )
	{
		if ( !( enabled & ( 1u << sprite ) ) ) continue;
		const u8 pointer = ram[ ( logicalScreen + 0x3F8 + sprite ) & 0xFFFF ];
		const u32 shape = bank + (u32)pointer * 64;
		if ( shape < 0x10000 )
			m_WriteBuffer.invalidateRange( (u16)shape, 64 );
	}
}

bool CSuperCPU::init( IC64Bus *bus, SCPUCoreType core, u32 simmMB )
{
	m_Bus = bus;

	if ( !m_Bus->acquire() )
		return false;

	m_Memory.attachBus( m_Bus );
	m_Memory.enablePostedWriteTiming( false );
	m_WriteBuffer.setDeliveryEnabled( true );
	m_Memory.setMirrorSink( &m_WriteBuffer );
	m_Memory.setIOInterceptor( &m_Registers );

	m_WriteBuffer.attach( m_Bus, m_Memory.m_RAM );
	m_WriteBuffer.attachHotShapeBlocks( m_Memory.m_HotShapeBlocks );
	m_WriteBuffer.attachRelocation( m_Memory.m_PtrReloc, m_Memory.m_RelocInUse,
	                                &m_Memory.m_RelocCount );

	// SuperRAM, and the 24-bit space it lives in. Bank 0 of that space is the
	// C64 itself; everything above is the accelerator's own memory and never
	// touches the expansion port, which is why it runs at full speed.
	m_FastRAM.init( simmMB );
	m_MemoryMap.attachBank0( &m_Memory );
	m_MemoryMap.attachFastRAM( &m_FastRAM );
	m_Registers.attach( &m_WriteBuffer );
	m_Registers.attachBank1( m_MemoryMap.m_Bank1 );
	m_Registers.attachFastRAM( &m_FastRAM );
	// $D0B0 on an SCPU64 reports only the hardware revision.  Native SCPU128
	// mode has its own identity below; C64 mode on a physical C128 deliberately
	// keeps using the proven SCPU64-compatible path for now.
	m_Registers.setHardwareVersion( SCPU_V2 );
	// A physical C128 remains MACHINE_C128 after the Commodore key has selected
	// C64 mode, so machine type alone is not a mode discriminator.  Nor can a
	// DMA master read the 8502's internal $0001 port -- that was the source of
	// nondeterministic C64/native selection in kernel117.  AUTO instead checks
	// the stable $FF01-$FF04 view after the Z80 handoff: native mode exposes the
	// 8722 presets captured on real hardware (3F 7F 01 41), while C64 mode
	// exposes ordinary KERNAL bytes.  Bring-up can override this explicitly.
	bool scpu128Mode = false;
	if ( m_Bus->signals().machine == MACHINE_C128
	     && m_MemoryMap.romLength() >= 0x20000 )
	{
		if ( m_C128Mode == SCPU_C128_MODE_NATIVE )
			scpu128Mode = true;
		else if ( m_C128Mode == SCPU_C128_MODE_AUTO )
		{
			const u8 p1 = m_Bus->read( 0xFF01 );
			const u8 p2 = m_Bus->read( 0xFF02 );
			const u8 p3 = m_Bus->read( 0xFF03 );
			const u8 p4 = m_Bus->read( 0xFF04 );
			scpu128Mode = p1 == 0x3F && p2 == 0x7F
			           && p3 == 0x01 && p4 == 0x41;
		}
	}
	m_Registers.setSCPU128Mode( scpu128Mode );
	m_Memory.setSCPU128Mode( scpu128Mode );

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
	// first thing to flip. See Docs/SuperCPU64/65816-reference.md section 6 and U2.

	// Measure the interpreter before pacing is armed at a real rate --
	// reset() below re-seeds bank 1 over the top of the benchmark loop.
	if ( core == SCPU_CORE_65816 )
		benchmark65816();

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
	// reset() deliberately invalidates every CIA shadow, so establish the
	// deterministic C64 KERNAL direction state only afterwards. A physical C128
	// running the C64 personality belongs on this path too; future native C128
	// mode does not. acquire() still owns /DMA here, therefore no host CPU can
	// change the latch behind the shadow before guest execution begins.
	if ( !scpu128Mode )
		m_Memory.initializeC64CIA2();
	return true;
}

void CSuperCPU::reset()
{
	// A reset invalidates the old program's staged display writes. Sending them
	// now would be an unbounded burst at an arbitrary raster position, followed
	// immediately by clearing the shadow anyway.
	m_WriteBuffer.discard();
	m_FrameTickDebt = 0;
	m_DisplayScrubPhaseStart = 0;
	m_DisplayScrubPhaseOn = m_DisplayScrubEnabled
	                      && m_DisplayScrubPeriodSeconds == 0;
	m_DisplayScrubBitmapSeen = false;
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

	m_Memory.setBootmapROM( bootmapUsable ? m_MemoryMap.romImage() : 0,
	                       bootmapUsable ? m_MemoryMap.romLength() : 0 );
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

	// CSuperCPURegisters::reset() applied the hardware's $C7 power-on mirror
	// policy. Do not override it out of band: the value reported at $D0B3/$D0B4
	// and the policy actually used by the buffer must remain the same state.
	m_WriteBuffer.resetStats();

	m_Memory.resyncPacing();

	if ( m_CPU )
		m_CPU->reset();
}

void CSuperCPU::updateDisplayScrubPhase( bool bitmapActive )
{
	bool next = m_DisplayScrubEnabled;
	if ( m_DisplayScrubEnabled && m_DisplayScrubPeriodSeconds )
	{
		const u32 hz = m_Bus ? m_Bus->hostCyclesPerSec() : 0;
		const u64 now = m_Bus ? m_Bus->hostCycles() : 0;
		if ( bitmapActive && !m_DisplayScrubBitmapSeen )
		{
			m_DisplayScrubBitmapSeen = true;
			m_DisplayScrubPhaseStart = now;
		}
		if ( hz && m_DisplayScrubBitmapSeen )
		{
			const u64 interval = (u64)m_DisplayScrubPeriodSeconds * hz;
			next = interval && ( ( ( now - m_DisplayScrubPhaseStart ) / interval ) & 1 );
		}
		else
			next = false;
	}
	if ( next != m_DisplayScrubPhaseOn )
	{
		m_DisplayScrubPhaseOn = next;
		m_DisplayScrubPhaseGeneration++;
	}
}

void CSuperCPU::benchmark65816()
{
	// A representative emulation-mode loop, placed in bank 1 so neither the
	// fetches nor the data ever leave the Pi: immediate, ALU, absolute store
	// and load (DBR=1 keeps them in bank 1), index, taken branch.
	static const u8 loop[] = {
		0xA2, 0x00,				// LDX #$00
		0xA9, 0x55,				// LDA #$55
		0x49, 0xFF,				// EOR #$FF      <- loop
		0x8D, 0x00, 0xF1,		// STA $F100     (DBR=1 -> $01F100)
		0xAD, 0x00, 0xF1,		// LDA $F100
		0xE8,					// INX
		0xD0, 0xF5,				// BNE loop
		0x4C, 0x02, 0xF0		// JMP $F002
	};
	for ( u32 i = 0; i < sizeof( loop ); i++ )
		m_MemoryMap.m_Bank1[ 0xF000 + i ] = loop[ i ];

	// Pacing at an unreachable rate: the pacer never binds, and the interrupt
	// cache samples the GPIO only once per 200 emulated cycles, so neither
	// contaminates the measurement. reset() rearms the real rate afterwards.
	m_Memory.setPacing( m_Bus->hostCyclesPerSec(), 200000000u );

	m_Core65816.m_E   = true;
	m_Core65816.m_P   = (u8)( W65_M | W65_X | W65_I );	// I set: IRQs masked
	m_Core65816.m_PBR = 0x01;
	m_Core65816.m_DBR = 0x01;
	m_Core65816.m_PC  = 0xF000;
	m_Core65816.m_S   = 0x01FD;
	m_Core65816.applyE();		// the pokes above bypass the audited writers

#if !defined( SCPU_HOST_BUILD ) && defined( __aarch64__ )
	// Program four PMU event counters alongside the cycle counter the bus
	// timing already owns (PMCR is enabled in lowlevel_arm64). Event numbers
	// from the ARMv8 architectural set: 0x08 instructions retired, 0x01 L1I
	// refill, 0x03 L1D refill, 0x10 branch mispredicted.
	static const u32 events[ 4 ] = { 0x08, 0x01, 0x03, 0x10 };
	for ( u32 i = 0; i < 4; i++ )
	{
		asm volatile( "msr PMSELR_EL0, %0" : : "r"( (u64)i ) );
		asm volatile( "isb" );
		asm volatile( "msr PMXEVTYPER_EL0, %0" : : "r"( (u64)events[ i ] ) );
		asm volatile( "msr PMXEVCNTR_EL0, %0" : : "r"( (u64)0 ) );
	}
	u64 enset;
	asm volatile( "mrs %0, PMCNTENSET_EL0" : "=r"( enset ) );
	enset |= 0x0F;
	asm volatile( "msr PMCNTENSET_EL0, %0" : : "r"( enset ) );
	asm volatile( "isb" );
#endif

	const u64 c0 = m_Core65816.cycles();
	const u64 h0 = m_Bus->hostCycles();
	m_Core65816.run( 2000000 );
	const u64 dh = m_Bus->hostCycles() - h0;
	const u64 dc = m_Core65816.cycles() - c0;

	m_BenchArmPerEmuCycle = dc ? (u32)( dh / dc ) : 0;

#if !defined( SCPU_HOST_BUILD ) && defined( __aarch64__ )
	if ( dc )
	{
		u64 ev[ 4 ];
		for ( u32 i = 0; i < 4; i++ )
		{
			asm volatile( "msr PMSELR_EL0, %0" : : "r"( (u64)i ) );
			asm volatile( "isb" );
			asm volatile( "mrs %0, PMXEVCNTR_EL0" : "=r"( ev[ i ] ) );
		}
		m_BenchInstrPer1k      = (u32)( ev[ 0 ] * 1000 / dc );
		m_BenchL1IRefillPer1k  = (u32)( ev[ 1 ] * 1000 / dc );
		m_BenchL1DRefillPer1k  = (u32)( ev[ 2 ] * 1000 / dc );
		m_BenchBranchMissPer1k = (u32)( ev[ 3 ] * 1000 / dc );
	}
#endif

	// The pure loop above intentionally isolates the interpreter, which also
	// made it blind to the two paths that dominate real accelerated software:
	// bank-0 dispatch and SuperRAM. Measure representative loops before reset
	// discards their scratch state. Keeping these separate tells us whether a
	// change improved decoding, mirroring, physical I/O, or private-memory
	// execution instead of hiding all four behind one flattering number.
	static const u8 bank0Loop[] = {
		0xA9, 0x55,             // LDA #$55
		0x49, 0xFF,             // EOR #$FF
		0x8D, 0x00, 0x70,       // STA $7000
		0xAD, 0x00, 0x70,       // LDA $7000
		0x4C, 0x02, 0x20        // JMP $2002
	};
	static const u8 screenLoop[] = {
		0xA9, 0x00,             // LDA #$00
		0x49, 0x01,             // EOR #$01
		0x8D, 0x00, 0x04,       // STA $0400
		0x4C, 0x02, 0xF2        // JMP $F202
	};
	static const u8 ioLoop[] = {
		0xA9, 0x00,             // LDA #$00
		0x49, 0x01,             // EOR #$01
		0x8D, 0x20, 0xD0,       // STA $D020
		0x4C, 0x02, 0xF3        // JMP $F302
	};
	static const u8 superLoop[] = {
		0xA9, 0x55,             // LDA #$55
		0x49, 0xFF,             // EOR #$FF
		0x8D, 0x00, 0x10,       // STA $1000 in DBR $02
		0xAD, 0x00, 0x10,       // LDA $1000 in DBR $02
		0x4C, 0x02, 0x00        // JMP $0002 in PBR $02
	};
	for ( u32 i = 0; i < sizeof( bank0Loop ); i++ ) m_Memory.m_RAM[ 0x2000 + i ] = bank0Loop[ i ];
	for ( u32 i = 0; i < sizeof( screenLoop ); i++ ) m_MemoryMap.m_Bank1[ 0xF200 + i ] = screenLoop[ i ];
	for ( u32 i = 0; i < sizeof( ioLoop ); i++ ) m_MemoryMap.m_Bank1[ 0xF300 + i ] = ioLoop[ i ];
	for ( u32 i = 0; i < sizeof( superLoop ); i++ ) m_FastRAM.write( 0x020000 + i, superLoop[ i ] );

	auto prepare = [this]( u8 pbr, u8 dbr, u16 pc, bool emulation )
	{
		m_Core65816.m_E   = emulation;
		m_Core65816.m_P   = (u8)( W65_M | W65_X | W65_I );
		m_Core65816.m_PBR = pbr;
		m_Core65816.m_DBR = dbr;
		m_Core65816.m_PC  = pc;
		m_Core65816.m_S   = 0x01FD;
		m_Core65816.applyE();
		m_Memory.m_BusEventCount = 0;
		m_Memory.m_PendingFastHalfCycles = 0;
		m_Memory.m_FastHalfCarry = 0;
		m_Memory.resyncPacing();
	};
	auto measure = [this]() -> u32
	{
		const u64 c0 = m_Core65816.cycles();
		const u64 h0 = m_Bus->hostCycles();
		m_Core65816.run( 500000 );
		const u64 dc = m_Core65816.cycles() - c0;
		return dc ? (u32)( ( m_Bus->hostCycles() - h0 ) / dc ) : 0;
	};

	prepare( 0x00, 0x00, 0x2000, true );
	m_BenchBank0ArmPerCycle = measure();
	prepare( 0x01, 0x00, 0xF200, true );
	m_BenchScreenArmPerCycle = measure();
	prepare( 0x01, 0x00, 0xF300, true );
	m_BenchIOArmPerCycle = measure();
	prepare( 0x02, 0x02, 0x0000, false );
	{
		const u64 stretch0 = m_Memory.m_RegionStretchCycles;
		const u64 cycle0 = m_Core65816.cycles();
		m_BenchSuperRAMArmPerCycle = measure();
		const u64 dc = m_Core65816.cycles() - cycle0;
		m_BenchSuperRAMStretchPer1k = dc
			? (u32)( ( m_Memory.m_RegionStretchCycles - stretch0 ) * 1000 / dc ) : 0;
	}
}

u32 CSuperCPU::currentClockHz() const
{
	return m_Registers.turboEnabled() ? SCPU_TURBO_HZ : SCPU_NORMAL_HZ;
}

// Drain mirrored RAM in small, independently raster-guarded bursts. A single
// sample cannot safely authorize thousands of writes: sprite DMA, badlines and
// ordinary beam progress can consume the window while the burst is still
// running. Re-sampling before every chunk lets the physical VIC steer us.
//
// Border bytes are effectively free -- nothing else wants the bus -- so the
// border budget is the whole remaining border. Display bytes are RATIONED but
// not forbidden, and that distinction is the point of this function.
//
// Mirroring only in the border bounds delivery at roughly 3KB per frame, and
// only on frames where a raster sample happens to catch the border at all. A
// game that redraws moving objects every frame queues more than that, so the
// queue ages: the VIC then fetches a MIXTURE of several frames' bytes for the
// objects that changed, while static scenery -- written once, delivered long
// ago -- stays perfect. That is precisely the observed artifact, moving pool
// balls rendered as blocks of mixed-frame garbage on a clean table.
//
// Writing non-screen data inside the picture is what a real REU does, and is
// safe for the same reason: busWriteByteBurst_p1 re-samples BA at the configured
// in-cycle point and hands the bus back whenever the VIC claims it, so a VIC
// fetch is never disturbed -- only our own write is delayed. The active screen
// matrix is the exception: even individually safe writes expose a mixture of
// the old and one-character-shifted matrices. It is skipped here and completed
// in short installments outside the picture.
// True when the given raster line is inside any enabled sprite's fetch span.
// The VIC steals its s-accesses on every line a sprite is visible and fetches
// the pointer and first data on the line BEFORE, so the span starts early.
// Those BA windows are narrow and abrupt -- unlike a badline's long low --
// and they are where a mid-display burst write can still collide: 3D Pool's
// flicker was exactly its static balls being hit by bursts of its own
// unchanging re-render traffic.
static bool c64LineInSpriteFetchSpan( const u8 *vic, u16 line )
{
	const u8 enabled = vic[ 0x15 ];
	if ( !enabled )
		return false;
	for ( u32 n = 0; n < 8; n++ )
	{
		if ( !( ( enabled >> n ) & 1 ) )
			continue;
		const u16 y = vic[ 1 + 2 * n ];
		// [y-3, y+21): fetch lead-in plus the 21 visible lines.
		if ( line + 3 >= y && line < (u16)( y + 21 ) )
			return true;
	}
	return false;
}

static void flushMirrorsRasterAware( CWriteBuffer &buffer, IC64Bus &bus,
                                     const C64Signals &sig, u32 displayBudget,
                                     u32 transferBudget, u32 screenBudget,
	                                 const u8 *vicShadow, u32 screenBase,
	                                 u32 bitmapBase )
{
	const u32 perLine  = c64CyclesPerLine( sig.video );
	const u32 lines    = c64RasterLines( sig.video );
	const u32 maxChunk = 64;
	u32 displayLeft = displayBudget;
	u32 transferLeft = transferBudget;
	u32 screenLeft = screenBudget;

	while ( !buffer.empty() && ( transferLeft != 0 || screenLeft != 0 ) )
	{
		const u16 line = bus.rasterLine();
		u32 budgetBytes;
		bool inDisplay = false;
		bool displayPriority = false;
		u32 priorityBase = 0xFFFFFFFF, priorityLen = 0;

		if ( c64RasterIsSafeForBulkTransfer( sig.video, line ) )
		{
			// Below the display the window includes bottom border, vertical
			// blank and top border; above it, only the remaining top border.
			const u32 linesLeft = ( line > c64DisplayLastLine( sig.video ) )
			                    ? ( lines - line ) + c64DisplayFirstLine( sig.video )
			                    : c64DisplayFirstLine( sig.video ) - line;
			budgetBytes = ( linesLeft * perLine * 3 ) / 4;
			const bool screenPending = screenBase < 0x10000
			                        && buffer.hasPendingInRange( screenBase, 1000 );
			const bool bitmapPending = bitmapBase < 0x10000
			                        && buffer.hasPendingInRange( bitmapBase, 8000 );
			if ( screenPending || bitmapPending )
			{
				// Do not spend this opportunity on general FIFO traffic while a
				// matrix transaction is still incomplete. Its dedicated allowance
				// is larger, but every physical burst below remains 64 bytes and
				// the raster is sampled again before the next one.
				if ( screenLeft == 0 ) break;
				displayPriority = true;
				// Matrix first: it is only 1000 bytes and includes bitmap colour
				// selection. The bitmap consumes the remaining hidden opportunities
				// and converges over subsequent frames without visible-picture tears.
				priorityBase = screenPending ? screenBase : bitmapBase;
				priorityLen = screenPending ? 1000 : 8000;
				if ( budgetBytes > screenLeft ) budgetBytes = screenLeft;
			}
			else
			{
				if ( transferLeft == 0 ) break;
				if ( budgetBytes > transferLeft ) budgetBytes = transferLeft;
			}
		}
		else if ( line < lines && displayLeft != 0 && transferLeft != 0
		          && !c64LineInSpriteFetchSpan( vicShadow, line ) )
		{
			// A real line, inside the picture, with ration left, and no
			// sprite fetching nearby. Sprite fetch lines defer to the next
			// slice rather than risk the burst/BA race; the border branch
			// keeps delivering regardless, because parked multiplexer
			// sprites would otherwise starve it permanently.
			inDisplay = true;
			budgetBytes = displayLeft < transferLeft ? displayLeft : transferLeft;
		}
		else
		{
			// Torn or unknown raster, or the display ration is spent. An
			// unusable sample must never authorize a transfer.
			break;
		}

		const u32 chunk = budgetBytes < maxChunk ? budgetBytes : maxChunk;
		if ( chunk == 0 )
			break;

		const u32 before = buffer.pending();
		if ( displayPriority )
		{
			// A smooth character scroller rewrites the matrix at every
			// eight-pixel boundary. Deliver those bytes first, but still in
			// the same short installments: while the beam is outside the
			// picture a partially updated matrix is invisible, and completing
			// it before general traffic prevents an old/new one-character
			// composite from appearing on the next frame.
			buffer.flushRangeUpTo( priorityBase, priorityLen, chunk );
		}
		else
		{
			// Inside the picture, skip both active sprite shapes and the active
			// screen matrix while continuing past them to cold traffic. Outside
			// it, any leftover installment is ordinary FIFO delivery.
			buffer.flushUpToPolicy( chunk, inDisplay,
			                        inDisplay ? screenBase : 0xFFFFFFFF,
			                        inDisplay ? bitmapBase : 0xFFFFFFFF );
		}
		const u32 after = buffer.pending();

		// A detached or failing sink must not turn this loop into a spin.
		if ( after >= before )
			break;

		const u32 sent = before - after;
		if ( displayPriority )
			screenLeft = ( screenLeft > sent ) ? ( screenLeft - sent ) : 0;
		else
		{
			transferLeft = ( transferLeft > sent ) ? ( transferLeft - sent ) : 0;
			if ( inDisplay )
				displayLeft = ( displayLeft > sent ) ? ( displayLeft - sent ) : 0;
		}
	}
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
	// The previous call may have run ahead to meet the physical VIC's border.
	// Repay that work before granting this frame any new CPU time. Without this
	// accounting the amount of extra execution depended on the sampled raster,
	// making games randomly speed up and slow down as the two frame phases
	// drifted past one another.
	const u64 repaid = m_FrameTickDebt < frameTicks
	                 ? m_FrameTickDebt : frameTicks;
	u64 ticksUsed = repaid;
	m_FrameTickDebt -= repaid;
	u64 executed = 0;

	// The frame is run in slices rather than one call, for two reasons. A
	// speed change breaks run() so the remainder can be re-budgeted at the new
	// rate. And the mirror buffer gets a flush OPPORTUNITY between slices: when
	// the machine falls behind real time, frame boundaries drift out of phase
	// with the real raster, and a flush attempted only at the boundary lands in
	// the safe border window by luck -- roughly a third of the time. Screen
	// writes then reach the C64 in visible ~quarter-second bursts. Frequent,
	// short opportunities also matter on real hardware: a 1024-byte drain
	// stalls the emulated CPU for about 1ms while the physical raster continues,
	// followed by a catch-up sprint. That stop/go cadence jerks scrolling and
	// can make a raster split restore its display mode late. Spread the same
	// historical eight-drain allowance over 128 points instead: the default
	// 1024 setting becomes 64 bytes (~64us) per opportunity and still delivers
	// roughly 8KB/frame.
	static const u32 frameSlices = 128;
	const u64 sliceTicks = frameTicks / frameSlices;
	const u32 displayPerOpportunity = ( m_MirrorDisplayBudget + 15 ) / 16;
	// Border-only mode still needs a finite installment. Unlimited border
	// drains merely move the same millisecond pause outside the picture while
	// raster-timed CPU code remains stopped.
	const u32 transferPerOpportunity = displayPerOpportunity
	                                 ? displayPerOpportunity : 64;

	while ( ticksUsed < frameTicks )
	{
		const u32 scrubBitmapBase = m_Memory.activeBitmapBase();
		updateDisplayScrubPhase( scrubBitmapBase < 0x10000 );
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

		// Opportunistic flush: one raster read. Cold traffic may use its small
		// display ration; the active screen matrix and hot sprite shapes wait
		// for the hidden window.
		//
		// And NEVER while the serial bus is active. A flush stalls the
		// emulated CPU for up to several hundred microseconds, and the slow
		// serial protocol assigns MEANING to pauses: a talker holding its
		// ready state longer than 200us is signalling EOI. A flush landing
		// between a host's ready and its first bit made the drive see EOI on
		// ordinary bytes, desyncing the transfer -- the drive then sat holding
		// CLK for an acknowledgment phase the host never entered, which is
		// precisely the $DD00=$87 deadlock captured on hardware. Mirroring can
		// always wait 100ms; the serial bus cannot.
		if ( !m_MirrorHalted && !m_Memory.iecBusActive()
		     && !m_WriteBuffer.empty() && ticksUsed < frameTicks )
			flushMirrorsRasterAware( m_WriteBuffer, *m_Bus, sig,
			                         displayPerOpportunity,
			                         transferPerOpportunity,
			                         SCPU_MIRROR_SCREEN_BYTES_PER_OPPORTUNITY,
			                         m_Memory.m_VICRegShadow,
			                         m_Memory.activeScreenBase(),
			                         m_Memory.activeBitmapBase() );

		// Physical-C64 display DRAM can acquire sparse stored single-bit errors
		// while the VIC is actively fetching. Repair the measured vulnerable
		// address class from authoritative shadow, but only after ordinary dirty
		// traffic has drained and only in the hidden raster window. One 32-byte
		// chunk per scheduler opportunity keeps the measured worst pause below
		// 60us. A timed trial alternates repair off/on, making the visual result
		// self-controlled without adding physical reads on machines whose bus has
		// no calibrated stable read window.
		if ( m_DisplayScrubEnabled && !m_MirrorHalted
		     && sig.machine == MACHINE_C64
		     && scrubBitmapBase < 0x10000
		     && ( m_Memory.m_VICRegShadow[ 0x11 ] & 0x10 )
		     && !m_Memory.iecBusActive() && m_WriteBuffer.empty()
		     && ticksUsed < frameTicks )
		{
			const u16 scrubLine = m_Bus->rasterLine();
			if ( c64RasterIsSafeForBulkTransfer( sig.video, scrubLine ) )
			{
				m_WriteBuffer.resyncDisplayed(
					m_Memory.activeScreenBase(), scrubBitmapBase, 8192,
					32, m_DisplayScrubMasked, m_DisplayScrubPhaseOn, false );
			}
		}
	}

	// --- raster-scheduled mirroring ----------------------------------------
	// Land in the border ON PURPOSE, once per frame.
	//
	// Every drain above is opportunistic: sample the raster, and transfer only
	// if the beam happens to be outside the picture. The border is under a
	// quarter of the frame, so with nine samples about one frame in eleven
	// drains nothing at all -- and a frame that delivers nothing shows the
	// previous frame's objects. That is a flicker with no cure in the size of
	// the budget, because the problem is WHEN the budget is spent.
	//
	// Waiting for the border by polling was the old answer and it cost real
	// time (the CPU stopped dead for up to 800us a frame, which showed as
	// stutter). Instead, work out how far the beam is from the border and run
	// the emulated CPU for exactly that long. The wait becomes useful work, the
	// machine arrives at the border by construction, and the drain then has the
	// whole border window to itself.
	//
	// The cycles spent here belong to the next frame, which is precisely the
	// point: it re-phases the emulated frame against the real raster instead of
	// letting the two drift.
	if ( !m_MirrorHalted && !m_WriteBuffer.empty() && !m_Memory.iecBusActive() )
	{
		const u16 line = m_Bus->rasterLine();
		const u32 lines = c64RasterLines( sig.video );

		if ( line < lines && !c64RasterIsSafeForBulkTransfer( sig.video, line ) )
		{
			const u32 linesToGo = ( c64DisplayLastLine( sig.video ) + 1 ) - line;
			const u64 approachTicks = (u64)linesToGo
			                        * (u64)c64CyclesPerLine( sig.video )
			                        * ( SCPU_TURBO_HZ / SCPU_NORMAL_HZ );

			const u32 clockHz = m_Memory.iecThrottleActive()
			                  ? SCPU_NORMAL_HZ : currentClockHz();
			const u32 ticksPerCycle = SCPU_TURBO_HZ / clockHz;
			const u64 ran = m_CPU->run( approachTicks / ticksPerCycle );
			executed += ran;
			m_FrameTickDebt += ran * ticksPerCycle;
		}

		// Re-check the serial bus: the approach run may have started one.
		if ( !m_MirrorHalted && !m_Memory.iecBusActive() )
		{
			flushMirrorsRasterAware( m_WriteBuffer, *m_Bus, sig,
			                         displayPerOpportunity,
			                         transferPerOpportunity,
			                         SCPU_MIRROR_SCREEN_BYTES_PER_OPPORTUNITY,
			                         m_Memory.m_VICRegShadow,
			                         m_Memory.activeScreenBase(),
			                         m_Memory.activeBitmapBase() );

			// Spare border time goes to convergence: re-deliver a slice of
			// clean shadow so real DRAM provably approaches shadow no matter
			// what made them differ (boot-era policy skips, mode switches,
			// a glitched burst). Keep this to one normal installment too: the old
			// 512-byte sweep was an otherwise unexplained half-millisecond CPU
			// pause whenever the ordinary queue happened to empty.
			if ( m_WriteBuffer.empty()
			     && m_Bus->signals().machine != MACHINE_C128 )
			{
				// On a physical C128 the background sweep is not benign: once
				// the ordinary queue drains it adds otherwise unnecessary DRAM
				// writes. The K198 no-repair test left
				// the same static A-Z screen unchanged for ten minutes after
				// MIRROR_HALT stopped this traffic. Suppress only the sweep on a
				// C128; dirty writes still use the normal queue, while C64 sprite
				// relocation healing keeps its established behaviour.
				const u16 line = m_Bus->rasterLine();
				if ( c64RasterIsSafeForBulkTransfer( sig.video, line ) )
				{
					const u32 lines = c64RasterLines( sig.video );
					const u32 linesLeft = ( line > c64DisplayLastLine( sig.video ) )
					                    ? ( lines - line ) + c64DisplayFirstLine( sig.video )
					                    : c64DisplayFirstLine( sig.video ) - line;
					if ( linesLeft >= 12 )
						m_WriteBuffer.resyncSweep( transferPerOpportunity );
				}
			}
		}
	}

	// Pacing is no longer done here. It happens after every instruction in
	// CC64Memory::tick(), because frame granularity (~20ms) is far too coarse
	// for anything that measures time by counting cycles -- the KERNAL's IEC
	// routines bit-bang the serial lines to microsecond tolerances.

	// One coarse clock/raster pairing per frame gives non-IRQ programs a
	// reference without perturbing VIDEO_MODE 0. Precise anchors taken inside
	// raster handlers outrank this sample for roughly a frame.
	if ( m_Memory.vicLogEnabled() && !m_Memory.iecBusActive() )
	{
		const u16 line = m_Bus->rasterLine();
		if ( line != 0xFFFF ) m_Memory.noteRasterCoarse( line );
	}

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
