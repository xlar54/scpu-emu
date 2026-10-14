/*
   SCPU-EMU - CMD SuperCPU emulation for the C64/C128 using a RAD Expansion Unit
   Copyright (c) 2026 SCPU-EMU contributors

   Bank 0: the C64-visible 64K address space, served from the Raspberry Pi.

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
#include "c64_memory.h"

// Published by the bus primitives, one store per access. Declared here rather
// than pulled in through lowlevel_dma.h: this file is the C64 layer and has no
// other reason to see the GPIO headers.
#ifdef SCPU_HOST_BUILD
// The host test harness links no GPIO layer at all -- gpio_defs.cpp is Pi-only
// -- so the definitions live here for that build and stay at zero. The trace
// fields then read 0r/0w under the harness, which is the truth: there is no
// physical BA to wait out, so no access can spin on one. Defining rather than
// stubbing out the reads keeps ONE code path through appendVSPTiming, so the
// host tests exercise the same field arithmetic the Pi does.
volatile u32 radBASpinsLastRead = 0;
volatile u32 radBASpinsLastWrite = 0;
#else
extern volatile u32 radBASpinsLastRead;
extern volatile u32 radBASpinsLastWrite;
#endif

// CIA2 PA3-PA5 feed the inverting open-collector IEC drivers, so only a high
// latch bit configured as an output actively pulls a bus line.
static inline u8 cia2EffectiveIECDrive( u8 latch, u8 ddra )
{
	return (u8)( latch & ddra & 0x38 );
}

// The initialiser list follows the DECLARATION order in the header. GCC warns
// otherwise (-Wreorder), and rightly: the compiler initialises in declaration
// order regardless of what is written here, so a list in a different order
// reads as a lie about what happens.
CC64Memory::CC64Memory()
	: m_KernalShadowBase( 0xE000 ), m_BootmapActive( false ),
	  m_BankMode( 0 ), m_SCPU128Mode( false ), m_ROMShadow( 0 ),
	  m_Mirror( 0 ), m_RamWrites( 0 ),
	  m_Port00( 0x2F ), m_Port01( 0x37 ),
	  m_IOReads( 0 ), m_IOWrites( 0 ),
	  m_IECThrottleEvents( 0 ),
	  m_PacingDebtCycles( 0 ), m_PacingCheckCycles( 1 ),
	  m_PacerWaitHostCycles( 0 ), m_SlowPacedCycles( 0 ), m_IECThrottledCycles( 0 ),
	  m_C64( 0 ), m_IO( 0 ), m_BootmapROM( 0 ),
	  m_BootmapROMLength( 0 ),
	  m_HasBasic( false ), m_HasKernal( false ), m_HasChar( false ),
	  m_HostPerEmuQ16( 0 ), m_HostPerEmuQ16Slow( 0 ), m_PacingAnchor( 0 ),
	  m_LastPhiLockHost( 0 ), m_PhiRelockHostCycles( 0 ),
	  m_SelectedEmulatedHz( 0 ),
	  m_IECThrottleEnabled( true ), m_IECHoldCycles( 0 ), m_IECActivityCycles( 0 ),
	  m_MaxTickChunk( 0 ),
	  m_CIA2PortALatch( 0 ), m_LastCIA2PortARead( 0 ), m_CIA2DDRA( 0 ),
	  m_HaveCIA2PortALatch( false ), m_HaveCIA2PortARead( false ),
	  m_HaveCIA2DDRA( false ),
	  m_IOLog{ 0 }, m_IOLogPos( 0 ), m_CIALog{ 0 }, m_CIALogCyc{ 0 },
	  m_CIALastRead( 0xFFFFFFFF ), m_CIALogPos( 0 ), m_EmuCycles( 0 ),
	  m_TimingHook( 0 ), m_TimingHookCtx( 0 )
{
	c64BankingInit();

	for ( u32 i = 0; i < C64_RAM_SIZE; i++ )      m_RAM[ i ] = 0;
	for ( u32 i = 0; i < C64_BASIC_SIZE; i++ )    m_Basic[ i ] = 0xFF;
	for ( u32 i = 0; i < C64_KERNAL_SIZE; i++ )   m_Kernal[ i ] = 0xFF;
	for ( u32 i = 0; i < C64_CHARROM_SIZE; i++ )  m_CharROM[ i ] = 0;
	for ( u32 i = 0; i < 0x40; i++ ) m_PtrReloc[ i ] = 0xFF;
	for ( u32 i = 0; i < 0x30; i++ ) m_RelocInUse[ i ] = 0xFF;

	updateBankMode();
}

// --- CIA2 timer NMI retiming -----------------------------------------------
// See the header note. All cold paths: register writes and expired deadlines.

void CC64Memory::notePollHold()
{
	// A read of the serial port registers -- a mere poll included -- keeps
	// the real SuperCPU at 1MHz for a short window: its auto-slow re-arms on
	// every serial-port ACCESS. Winter Games' loader arms a 20-cycle timer
	// NMI in the middle of a $DD00 poll phase and its handler expects to
	// interrupt the very next instructions at 1MHz; with only edge-armed
	// holds, our machine had drifted back to turbo there and the NMI landed
	// 400 cycles deep, inside a native-mode window.
	//
	// The top-up is SHORT, CMD-scale: a poll LOOP (iterations tens of µs
	// apart) sustains it seamlessly, while a game that reads $DD00 once a
	// frame pays about a millisecond and returns to turbo -- no latch-up.
	// Deliberately does NOT touch m_IECActivityCycles: mirror suppression
	// still requires real evidence of a transaction (see the $DD00 read
	// handler), which is the lesson the mirror-blackout latch-up taught.
	const bool explicitlySlow = m_SelectedEmulatedHz != 0
	                         && m_SelectedEmulatedHz <= 1000000u;
	if ( !m_IECThrottleEnabled )
		return;
	if ( m_IECPollHoldCycles < 1000 )
	{
		const bool wasInactive = ( m_IECPollHoldCycles == 0
		                        && m_IECHoldCycles == 0 );
		m_IECPollHoldCycles = 1000;
		m_IECPollArms++;
		// At an explicitly selected 1MHz the SPEED effect is redundant, but
		// the sustained-poll evidence is not: a loader can be in a tight
		// BIT $DD00 wait while the edge-only transaction window expires. Keep
		// recording the short poll top-up, without announcing a rate change
		// that did not actually occur.
		if ( wasInactive && !explicitlySlow )
		{
			cia2OnRateChange();		// effective speed just changed
			if ( m_TimingHook )
				m_TimingHook( m_TimingHookCtx );
		}
	}
}

void CC64Memory::cia2OnRateChange()
{
	const u64 f = effectiveCIAFactor();
	// Bank the elapsed C64-clock segment with the OLD stored factor, then
	// remember the effective factor selected by the state change which brought
	// us here. Do this only for a real transition: timer-register writes also
	// call this function, and rebasing an unchanged turbo segment would discard
	// its sub-C64-cycle remainder every time.
	if ( f != m_C64FactorInUse )
		c64ClockRebase();

	if ( f == m_CIA2FactorInUse )
		return;
	// Re-denominate the REMAINING count of every armed deadline. A deadline
	// already reached stays reached (remaining clamps at zero: fires on the
	// next check).
	if ( m_CIA2TADeadline )
	{
		const u64 rem = ( m_CIA2TADeadline > m_EmuCycles )
		              ? ( m_CIA2TADeadline - m_EmuCycles ) : 0;
		m_CIA2TADeadline = m_EmuCycles + rem * f / m_CIA2FactorInUse;
		if ( m_CIA2TADeadline == m_EmuCycles ) m_CIA2TADeadline = m_EmuCycles + 1;
		m_CIA2Rescales++;
	}
	if ( m_CIA2TBDeadline )
	{
		const u64 rem = ( m_CIA2TBDeadline > m_EmuCycles )
		              ? ( m_CIA2TBDeadline - m_EmuCycles ) : 0;
		m_CIA2TBDeadline = m_EmuCycles + rem * f / m_CIA2FactorInUse;
		if ( m_CIA2TBDeadline == m_EmuCycles ) m_CIA2TBDeadline = m_EmuCycles + 1;
		m_CIA2Rescales++;
	}
	m_CIA2FactorInUse = f;
	cia2RecomputeNMIState();
}

void CC64Memory::cia2TimerWrite( u16 a, u8 v )
{
	// Keep the stored denomination in sync with the current speed BEFORE any
	// arithmetic: deadlines armed here are in effectiveCIAFactor() units, and
	// cia2OnRateChange() re-denominates them at every later speed change.
	cia2OnRateChange();

	switch ( a )
	{
	case 0xDD04: m_CIA2TALatch = (u16)( ( m_CIA2TALatch & 0xFF00 ) | v ); break;
	case 0xDD05: m_CIA2TALatch = (u16)( ( m_CIA2TALatch & 0x00FF ) | ( (u16)v << 8 ) ); break;
	case 0xDD06: m_CIA2TBLatch = (u16)( ( m_CIA2TBLatch & 0xFF00 ) | v ); break;
	case 0xDD07: m_CIA2TBLatch = (u16)( ( m_CIA2TBLatch & 0x00FF ) | ( (u16)v << 8 ) ); break;

	case 0xDD0D:
		// Set/clear semantics via bit 7, like the chip.
		if ( v & 0x80 ) m_CIA2ICRMask |= (u8)( v & 0x1F );
		else            m_CIA2ICRMask &= (u8)~( v & 0x1F );
		// Disabling both timer sources ends any synthetic assertion: on the
		// chip that mask change is exactly what releases the line.
		if ( !( m_CIA2ICRMask & 0x03 ) )
			m_SynthNMIActive = false;
		else
		{
			// Enabling with an underflow already pending asserts the line
			// RIGHT NOW -- the chip evaluates (pending & mask) continuously.
			// Winter Games' arm sequence starts its 20-cycle timer two
			// instructions before enabling the mask; a base computed a few
			// cycles stale can put the underflow in that gap.
			cia2RecomputeNMIState();
			cia2MaybeDeliverSynthNMI();
		}
		break;

	case 0xDD0E:
		// Timer A control. START on a stopped timer -- or a force-load strobe
		// -- (re)computes the underflow deadline; STOP disarms it. A write
		// that leaves a running timer running does not restart the count,
		// exactly like the chip. Counting CNT pulses (bit 5) is external
		// input we cannot model: disarm and let the sampled line handle it.
		if ( v & 0x20 )
			m_CIA2TADeadline = 0;
		else if ( ( v & 0x01 ) && ( !( m_CIA2TACR & 0x01 ) || ( v & 0x10 ) ) )
			m_CIA2ArmPending |= 0x01;		// anchored in cia2FinishArm()
		else if ( !( v & 0x01 ) )
			m_CIA2TADeadline = 0;
		m_CIA2TACR = v;
		break;

	case 0xDD0F:
		// Timer B: as A, plus bits 5-6 select CNT or timer-A-underflow
		// counting -- both unmodelable here.
		if ( v & 0x60 )
			m_CIA2TBDeadline = 0;
		else if ( ( v & 0x01 ) && ( !( m_CIA2TBCR & 0x01 ) || ( v & 0x10 ) ) )
			m_CIA2ArmPending |= 0x02;		// anchored in cia2FinishArm()
		else if ( !( v & 0x01 ) )
			m_CIA2TBDeadline = 0;
		m_CIA2TBCR = v;
		break;

	default:
		return;						// $DD08-$DD0C: TOD and SDR, no model
	}
	const bool wasArmed = ( m_CIA2NMIDeadline != 0 );
	cia2RecomputeNMIState();
	// A deadline coming live must take effect on the very NEXT instruction:
	// the run loop batches bus ticks, and a 20-cycle deadline checked
	// against a clock that only advances per batch slips past the designed
	// landing zone -- Winter Games' throwaway RTI-handler NMI then falls
	// into the long MVN inside its native-mode window instead of the five
	// plain instructions after the arm. The armed deadline switches
	// fineTicksRequired() on; the hook breaks the current batch.
	if ( !wasArmed && m_CIA2NMIDeadline && m_TimingHook )
		m_TimingHook( m_TimingHookCtx );
}

void CC64Memory::cia2RecomputeNMIState()
{
	// Nearest armed deadline, for the hot path's single compare.
	u64 next = m_CIA2TADeadline;
	if ( m_CIA2TBDeadline && ( !next || m_CIA2TBDeadline < next ) )
		next = m_CIA2TBDeadline;
	m_CIA2NMIDeadline = m_NMIRetimeEnable ? next : 0;

	// The timers own the NMI line only while EVERY enabled NMI source is a
	// modeled timer. A program mixing FLAG or SDR interrupts with timer
	// interrupts gets the sampled line for everything -- jittery, but never
	// wrong about which sources exist.
	m_CIA2NMIOwned = m_NMIRetimeEnable
	              && ( m_CIA2ICRMask & 0x03 ) != 0
	              && ( m_CIA2ICRMask & 0x1C ) == 0;
}

void CC64Memory::cia2FinishArm()
{
	cia2OnRateChange();				// units first, then the anchor
	const u64 now = ciaNow();
	if ( m_CIA2ArmPending & 0x01 )
	{
		m_CIA2TADeadline = now + ciaSpan( (u64)m_CIA2TALatch + 1 );
		m_NMITraceArmAt = now;
		m_NMITraceGen++;
	}
	if ( m_CIA2ArmPending & 0x02 )
		m_CIA2TBDeadline = now + ciaSpan( (u64)m_CIA2TBLatch + 1 );
	m_CIA2ArmPending = 0;
	cia2RecomputeNMIState();
}

void CC64Memory::cia2MaybeDeliverSynthNMI()
{
	// The line asserts when an enabled source is pending -- evaluated
	// continuously by the chip, so both "underflow while enabled" and
	// "enable while pending" produce the edge. A new edge only if the
	// previous assertion was acknowledged: an unread ICR holds the real
	// line low too, and a held line has no edges.
	if ( m_CIA2NMIOwned && !m_SynthNMIActive
	     && ( m_CIA2ICRPending & m_CIA2ICRMask & 0x03 ) )
	{
		m_SynthNMIActive = true;
		m_CachedNMI = true;				// exposed NOW; the next sample sees it
		m_SynthNMIsDelivered++;
		m_NMITraceExposeAt = ciaNow();
		m_NMITraceExposeGen = m_NMITraceGen;
		// Break the run loop: interrupt sampling happens at slice entry, so
		// without this the exposed edge waits out the rest of the slice.
		if ( m_TimingHook )
			m_TimingHook( m_TimingHookCtx );
	}
}

void CC64Memory::ciaTimerNMIDue()
{
	cia2OnRateChange();				// continuous re-arm below uses fresh units
	const u64 now = ciaNow();

	if ( m_CIA2TADeadline && now >= m_CIA2TADeadline )
	{
		m_CIA2ICRPending |= 0x01;		// latches regardless of the mask
		m_NMITraceLatchAt = now;
		m_NMITraceLatchGen = m_NMITraceGen;
		if ( m_CIA2TACR & 0x08 )
		{
			m_CIA2TADeadline = 0;			// one-shot: the chip clears START
			m_CIA2TACR &= (u8)~0x01;
		}
		else
			m_CIA2TADeadline += ciaSpan( (u64)m_CIA2TALatch + 1 );
	}
	if ( m_CIA2TBDeadline && now >= m_CIA2TBDeadline )
	{
		m_CIA2ICRPending |= 0x02;
		if ( m_CIA2TBCR & 0x08 )
		{
			m_CIA2TBDeadline = 0;
			m_CIA2TBCR &= (u8)~0x01;
		}
		else
			m_CIA2TBDeadline += ciaSpan( (u64)m_CIA2TBLatch + 1 );
	}

	cia2MaybeDeliverSynthNMI();
	cia2RecomputeNMIState();
}

// Claim a free deliverable block in $C000-$CBFF as the relocated home of
// under-I/O shape block v ($40-$7F), and queue the block's current shadow
// content at its new address. Returns the relocated block value, or $FF when
// nothing usable is free.
//
// A block is usable when the VIC cannot be displaying it as anything else:
// not inside the active screen matrix (whose pointer row the translation
// serves) and not inside an active bank-3 bitmap. The blocks may well hold
// program data -- 3D Pool keeps tables in all of $C000-$CBFF -- but that data
// is only ever read back through the shadow; its DRAM copy is dead weight
// unless displayed, and allocation just ruled displaying out.
u8 CC64Memory::allocRelocBlock( u8 v )
{
	// Callers must rebuild the hot-shape set immediately after a successful
	// allocation: changing m_PtrReloc can make a formerly deferred source block
	// deliverable. noteSpritePointerWrite() currently supplies that ordering.
	// Forbidden span 1: the active and most-recent screen matrices, as blocks.
	// Double-buffered programs can queue pointer-row bytes for one while the
	// other is active; stealing either matrix makes the next flip destructive.
	const u32 screenBase = ( m_SpritePtrBase - 0x3F8 ) & 0xFFFF;
	const u32 previousScreenBase = ( m_PreviousSpritePtrBase - 0x3F8 ) & 0xFFFF;
	// Forbidden span 2: a bitmap in the low half of bank 3. $D011 bit 5 turns
	// bitmap mode on; $D018 bit 3 picks the half. 8K starting at $C000 covers
	// the whole window -- in that configuration there is nothing to allocate.
	const bool lowBitmap = ( m_VICRegShadow[ 0x11 ] & 0x20 )
	                    && !( m_LastD018 & 0x08 )
	                    && !m_DisplayRelocActive;

	for ( u8 r = 0; r < 0x30; r++ )
	{
		if ( m_RelocInUse[ r ] != 0xFF )
			continue;
		const u32 addr = 0xC000 + ( (u32)r << 6 );
		if ( addr >= screenBase && addr < screenBase + 0x400 )
			continue;
		if ( m_PreviousSpritePtrBase != 0xFFFFFFFF
		     && addr >= previousScreenBase && addr < previousScreenBase + 0x400 )
			continue;
		if ( lowBitmap )
			continue;

		m_PtrReloc[ v - 0x40 ] = r;
		m_RelocInUse[ r ] = v;
		m_RelocCount++;
		m_RelocAllocs++;
		m_DisplayRelocGeneration++;

		// Re-issue the shape's current shadow bytes at their under-I/O source
		// addresses, with the tables already armed: the write buffer's
		// translation queues them at the relocated block, bypassing same-value
		// elimination the way every translated write does. Shapes written
		// before the pointer arrive through this replay; shapes written after
		// are translated per-write as they happen.
		if ( m_Mirror )
		{
			const u32 src = 0xC000 + ( (u32)v << 6 );
			for ( u32 i = 0; i < 64; i++ )
				m_Mirror->onRamWrite( (u16)( src + i ), m_RAM[ src + i ] );
		}
		return r;
	}
	return 0xFF;
}

void CC64Memory::recomputeVICPhysicalMap()
{
	const bool oldActive = m_DisplayRelocActive;
	const u16 oldScreen = m_DisplayRelocScreenSource;
	const u16 oldGraphics = m_DisplayRelocGraphicsSource;
	const u16 oldLength = m_DisplayRelocGraphicsLength;

	const u32 bank = activeVICBankBase();
	const u32 screen = activeVICScreenBase();
	u32 graphics = activeBitmapBase();
	u32 graphicsLength = 8192;
	if ( graphics >= 0x10000 )
	{
		graphics = activeCharsetBase();
		graphicsLength = graphics < 0x10000 ? 2048 : 0;
	}

	const bool screenUnderIO = screen < 0xE000 && screen + 1024 > 0xD000;
	const bool graphicsUnderIO = graphics < 0xE000
	                          && graphics + graphicsLength > 0xD000;
	m_DisplayRelocActive = m_RelocEnable && !m_RAMUnderIOAccessible
	                    && bank == 0xC000
	                    && ( screenUnderIO || graphicsUnderIO );
	m_DisplayRelocScreenSource = (u16)screen;
	m_DisplayRelocGraphicsSource = graphics < 0x10000 ? (u16)graphics : 0;
	m_DisplayRelocGraphicsLength = (u16)graphicsLength;

	const bool physicalMapChanged = oldActive != m_DisplayRelocActive
	                             || ( m_DisplayRelocActive
	                                  && ( oldScreen != m_DisplayRelocScreenSource
	                                    || oldGraphics != m_DisplayRelocGraphicsSource
	                                    || oldLength != m_DisplayRelocGraphicsLength ) );
	if ( !physicalMapChanged )
		return;

	m_DisplayRelocGeneration++;
	if ( m_Mirror )
		m_Mirror->onVICMapChanged( screen, graphics, graphicsLength );
}

void CC64Memory::reset()
{
	m_Port00 = 0x2F;
	m_Port01 = 0x37;
	m_IOReads = m_IOWrites = m_RamWrites = 0;
	m_IECThrottleEvents = 0;
	m_PacingDebtCycles = 0;
	m_PacerWaitHostCycles = m_SlowPacedCycles = m_IECThrottledCycles = 0;
	m_PacerLateEntries = m_PacerLateProbes = 0;
	m_PacerLateSnapHostCycles = 0;
	m_PacerCreditEntries = m_PacerClampEntries = 0;
	m_PacerCreditHostCycles = 0;
	m_IECHoldCycles = 0;
	m_IECActivityCycles = 0;
	m_IECPollHoldCycles = 0;
	// The armed/arm counters are a per-run population like the BA stall's, so
	// they belong to the same reset as the windows they describe.
	m_IECHoldArmedCycles = 0;
	m_IECPollArmedCycles = 0;
	m_IECActivityArmedCycles = 0;
	m_IECArmReadEdge = 0;
	m_IECArmReadLevel = 0;
	m_IECArmReadRefused = 0;
	m_IECArmWrite = 0;
	m_IECArmWriteRefused = 0;
	m_IECPollArms = 0;
	m_MaxTickChunk = 0;
	m_CIA2PortALatch = 0;
	m_LastCIA2PortARead = 0;
	m_LastD018 = 0x14;			// the KERNAL default: screen $0400, charset $1000
	m_SpritePtrBase = m_PreviousSpritePtrBase = 0xFFFFFFFF;
	for ( u32 i = 0; i < 0x40; i++ ) m_VICRegShadow[ i ] = 0;
	m_CIA2DDRA = 0;
	m_HaveCIA2PortALatch = false;
	m_HaveCIA2PortARead = false;
	m_HaveCIA2DDRA = false;
	// Relocation tables before updateHotShapeBlocks, which consults them.
	// Cleared on reset: the game that allocated them is gone, and stale
	// mappings would translate a NEW program's pointers to blocks holding the
	// OLD program's shapes.
	for ( u32 i = 0; i < 0x40; i++ ) m_PtrReloc[ i ] = 0xFF;
	for ( u32 i = 0; i < 0x30; i++ ) m_RelocInUse[ i ] = 0xFF;
	m_RelocCount = 0;
	recomputeVICPhysicalMap();
	updateSpritePtrBase();
	updateHotShapeBlocks();
	m_IOLogPos = 0;
	// The forensics rings reset with the machine: a post-mortem must not blend
	// pre-reset history into a fresh run's capture.
	m_WatchPos = m_WatchWrites = m_WatchChanges = 0;
	m_D012ReadPos = m_D012ReadCount = 0;
	m_ICtlHead = m_ICtlFrozenHead = 0;
	m_ICtlFrozenReason = m_ICtlFrozenCycles = 0;
	m_D019Writes = m_D019AckHeldIRQ = m_D019AckHeldIRQLate = 0;
	m_ICtlSuspectsSeen = 0;
	m_ICtlFollowDue = 0;
	m_ICtlFollowHost = 0;
	// AND THE EMULATED-CLOCK STAMPS, because reset() zeroes m_EmuCycles a few
	// lines below: a surviving pre-reset m_IRQSampleEmu would be LARGER than
	// every post-reset m_ICtlFollowEmu and the new gate would accept a sample
	// from the previous run -- manufacturing the very unsoundness it removes.
	m_ICtlFollowEmu = 0;
	m_IRQSampleEmu = 0;
	m_IRQResampleDue = false;
	m_ICtlFreezeDue = 0;
	m_ICtlAckHeldPending = 0;
	m_D011RST8LastHost = 0;
	m_CIALogPos = 0;
	m_CIALastRead = 0xFFFFFFFF;
	m_VSPTraceHead = 0;
	m_VSPHaveRasterAnchor = false;
	m_VSPLastRaster = 0;
	m_VSPLastRasterHost = 0;
	for ( u32 i = 0; i < SCPU_VSP_TRACE_SIZE; i++ )
		m_VSPTrace[ i ] = VSPTimingEvent{};
	m_EmuCycles = 0;
	m_C64ClockBase = 0;
	m_EmuAtRateChange = 0;
	m_C64FactorInUse = effectiveCIAFactor();
	m_RasterCompare = 0;
	m_LastCompareWrite = 0x80000000u;
	m_LastPreciseAnchor = 0x80000000u;
	m_LastPreciseLine = 0xFFFF;
	__atomic_store_n( &m_RasterAnchor, 0ULL, __ATOMIC_RELAXED );
	__atomic_store_n( &m_VICLogHead, 0u, __ATOMIC_RELEASE );
	if ( m_HDMISpriteSpriteCollision )
		__atomic_store_n( m_HDMISpriteSpriteCollision, 0u, __ATOMIC_RELEASE );
	if ( m_HDMISpriteBackgroundCollision )
		__atomic_store_n( m_HDMISpriteBackgroundCollision, 0u,
		                  __ATOMIC_RELEASE );
	// The keepalive deadline is on the host clock. Carrying it across an
	// emulated reset can leave it arbitrarily far in the future relative to a
	// restarted session, allowing the physical bus to go cold permanently.
	m_NextKeepalive = 0;
	for ( u32 i = 0; i < 64; i++ )
	{
		m_IOLog[ i ] = 0;
		m_CIALog[ i ] = 0;
		m_CIALogCyc[ i ] = 0;
	}
	m_IntCredit = 0xFFFFF;
	m_CachedIRQ = m_CachedNMI = false;
	// CIA2 timer NMI retiming model; m_NMIRetimeEnable itself is config-owned.
	m_CIA2NMIOwned = false;
	m_SynthNMIActive = false;
	m_CIA2ICRMask = 0;
	m_CIA2TACR = m_CIA2TBCR = 0;
	m_CIA2TALatch = m_CIA2TBLatch = 0xFFFF;
	m_CIA2ICRPending = 0;
	m_CIA2ArmPending = 0;
	m_BusEventCount = 0;
	m_PostedWritePending = false;
	m_MirrorBufferFreeAt = 0;
	m_PendingFastHalfCycles = 0;
	m_FastHalfCarry = 0;
	m_CIA2TADeadline = m_CIA2TBDeadline = m_CIA2NMIDeadline = 0;
	m_CIA2FactorInUse = effectiveCIAFactor();
	resyncPacing();

	for ( u32 i = 0; i < C64_RAM_SIZE; i++ )
		m_RAM[ i ] = 0;

	updateBankMode();
	// Publish reset only after every live shadow above is back in its reset
	// state. Core 1 treats a generation change as an unconditional re-prime;
	// the numeric log head alone is ambiguous after it returns to zero.
	__atomic_add_fetch( &m_VICLogGeneration, 1u, __ATOMIC_RELEASE );
}

void CC64Memory::noteVSPRasterAnchor( u8 raster )
{
	// Bookkeeping around a read the guest already requested.  Do not ask the
	// VIC, GPIO or raster backend for anything else here: the sidecar must not
	// create a bus cycle of its own or move the timing it is observing.
	m_VSPLastRaster = raster;
	m_VSPLastRasterHost = m_C64 ? m_C64->hostCycles() : m_EmuCycles;
	// The grid instant of this read's ledger position -- see the member. From
	// the primitive's start-of-cycle stamp when the backend has one (the RAD bus
	// and the VSP fixture do); otherwise from the return stamp by the
	// primitive's fixed shape: the read occupies the cycle after the one its
	// settle exited in and returns a teardown after the following falling edge,
	// so the grid instant is 7/4 cycles before the return, to within that
	// teardown. Two loads and a few integer ops on a path that just paid for a
	// physical read; no bus cycle of its own.
	{
		const u64 perC64 = m_HostPerEmuQ16Slow >> 16;
		const u64 start = m_C64 ? m_C64->lastReadCycleStartHostCycles() : 0;
		m_VSPLastRasterGridHost = start ? start - 3 * perC64 / 4
		                                : m_VSPLastRasterHost - 7 * perC64 / 4;
	}
	m_VSPHaveRasterAnchor = true;
	m_VSPAimFloorPending = true;	// K498: the next placed write carries the aim's entry floor
	m_VSPHistPending = true;		// K503: the next $D011 write is this anchor's histogram sample
	// The same instant on the other clock. Two loads and a store on a path
	// that just paid for a physical bus read, and it creates no bus cycle of
	// its own -- this function's own standing rule. See m_VSPLastRasterC64.
	m_VSPLastRasterC64 = c64Now();
	// Still this read's own value: the primitive wrote it before returning
	// and nothing between there and here touches the bus. One load, one
	// store, on a path that just paid for a physical read.
	m_VSPAnchorBASpins = (u16)radBASpinsLastRead;
	// WHERE THE POLL STARTED, and how long it ran. m_VSPLastRaster above is
	// where it ENDED, which is all the trace has ever carried -- and on the
	// K478 bad event that was 254 with no way to tell whether Mario's wait loop
	// walked there from the border or was handed 254 on its first look. The
	// first value plus the count says which, and both are free here: this
	// function is already three stores on a path that just paid for a physical
	// bus read.
	if ( !m_VSPD012Reads ) m_VSPD012First = raster;
	if ( m_VSPD012Reads != 0xFFFF ) m_VSPD012Reads++;
}

// WHAT $D011 BIT 7 MUST BE, when the physical sample says it is set.
//
// THE GAP THIS DOES NOT CLOSE, measured off the bus primitives rather than
// assumed, because it is the reason this is a COHERENCE check and not a
// resample.  A guest $D011 read is one radDirectRead(): BUS_RESYNC aligns to
// PHI (under one C64 cycle) and then HANDLE_BUS_AVAILABLE_READ waits out
// whatever the VIC is holding (lowlevel_dma.h) before the data sample.  A
// badline is 43 cycles and a sprite steal 2-3, so the separation between the
// emulated access instant and the physical sample is normally under 0.7 of a
// raster line, and its hard ceiling is RAD_BA_WAIT_LIMIT = 4000 cycles, 61.5
// lines on NTSC.  Sampling bit 7 EARLIER therefore cannot be the fix: K471 saw
// the bit set while Mario's chain was on line 54, and 202 lines is not
// reachable from 0.7 or even from 61.5.  Nor can a model replace the sample:
// $D012 is the low eight bits of the SAME counter and has no shadow, so a bit 8
// derived from c64Now() -- which is never phase-locked to the physical frame --
// would hand a raster-polling guest a 9-bit line assembled from two unrelated
// clocks, and a PAL program waiting for line 300 would wait for ever.
//
// So: ask the one question that needs no model and no second clock.  A set bit
// 8 CLAIMS the beam is in 256..rasterLines-1, which on NTSC means $D012 reads
// 0..6 and on PAL 0..55.  Read the low byte and test the composed line against
// the machine's line count.  318 is not a raster line on a 263-line machine, so
// a sample that composes to 318 is proof of its own invalidity and bit 8 must
// be 0 -- it is the only value left.  This is the range check
// CRADBus::rasterLine() has always applied to the firmware's OWN raster probe
// (`if ( line < rasterLines )`), finally applied to the value a GUEST sees.
//
// COST, and why it is not charged.  One extra transfer on the bit-7-set arm
// only.  It is deliberately not passed to noteBusAccess(): a genuine bit 8 can
// only be set in lines 256..rasterLines-1, which is past c64DisplayLastLine()
// on both standards, so the extra cycle provably cannot land inside the display
// window where VSP timing lives -- and charging it WOULD move the emulated
// write-back placement of the very DEC $D011 whose hold = 59 - DECcycle
// invariant is under measurement.  An invalid sample can land anywhere, but it
// is already a defect and one uncharged cycle is the cheapest price for not
// corrupting RST8.
//
// The guest's own $D012 forensics ring and m_VSPLastRaster are NOT touched
// here.  They mean "what the GUEST saw", and the VSP instrument currently
// measuring the BA stall reads m_VSPLastRaster as a guest-observed anchor.
// The accepted arm of mirrorRamWrite, kept out of line so the floor test can
// inline without it. See the note on mirrorRamWrite.
void CC64Memory::mirrorRamWriteAccepted( u16 a, u8 v )
{
	if ( m_Mirror->onRamWrite( a, v ) )
		noteMirrorWrite();
}

// WHEN A $D011 READ MAY SKIP THE BUS (K500). Inside the raster window the
// guest's last $D012 read is at most 200 C64 cycles old -- 3.1 lines on NTSC,
// 3.2 on PAL -- so its low byte bounds the line the beam is on now. The low
// bytes 0..L-257 (L = the machine's line count: NTSC 0..6, PAL 0..55) also
// name lines 256..L-1, so an anchor there cannot prove bit 8; an anchor above
// 252 could cross into 256 before the read. Everything else proves RST8 == 0
// for the whole window without a model and without a second clock -- the same
// range argument d011RasterHighBit applies to a set bit, applied to a clear
// one. Outside the window, in turbo, or inside a serial hold the physical read
// runs as before.
bool CC64Memory::d011ShadowReadSafe()
{
	if ( !rasterCriticalWindowOpen() )
		return false;
	const u32 lines = m_C64 ? c64RasterLines( m_C64->signals().video ) : 263;
	const u32 ambiguous = lines > 256 ? lines - 256 : 0;
	if ( m_VSPLastRaster < ambiguous || m_VSPLastRaster > 252 )
	{
		m_VSPD011ShadowDeclined++;
		return false;
	}
	return true;
}

u8 CC64Memory::d011RasterHighBit()
{
	m_D011RST8Samples++;
	const u32 lines = m_C64 ? c64RasterLines( m_C64->signals().video )
	                        : 263;
	const u8 lo = m_C64 ? m_C64->read( 0xD012 ) : 0;
	m_IOReads++;
	m_D011RST8LastLow = lo;
	// STAMPED, so the interrupt-control ring can report this sample's AGE
	// rather than presenting an arbitrarily old line as a measurement. Two
	// stores on a path that just paid for a physical bus read, and the only
	// physical raster this firmware observes that the guest did not ask for.
	m_D011RST8LastHost = m_C64 ? m_C64->hostCycles() : m_EmuCycles;
	if ( ( 0x100u | (u32)lo ) < lines )
		return 0x80;
	m_D011RST8Rejected++;
	m_D011RST8LastRejLow = lo;
	return 0;
}

void CC64Memory::appendVSPTiming( u16 address, u8 value, u8 physicalValue,
                                  bool inWindow )
{
	// K506: with the filter on, a write outside a raster window is not
	// retained (its displacement counters then drain into the next retained
	// event, which the ring's own doctrine already tolerates).
	if ( m_VSPTraceWindowOnly && !inWindow )
		return;
	const u32 sequence = m_VSPTraceHead++;
	VSPTimingEvent &event =
		m_VSPTrace[ sequence & ( SCPU_VSP_TRACE_SIZE - 1 ) ];
	const u64 host = m_C64 ? m_C64->hostCycles() : m_EmuCycles;
	const u64 perC64 = m_HostPerEmuQ16Slow >> 16;

	event.hostCycle = host;
	event.emuCycle = m_EmuCycles;
	event.c64Cycle = c64Now();
	event.pacingAnchor = m_PacingAnchor;
	event.pacingDebt = m_PacingDebtCycles;
	event.rasterHostCycle = m_VSPLastRasterHost;
	event.rasterC64Cycle = m_VSPLastRasterC64;
	event.anchorBASpins = m_VSPAnchorBASpins;
	// This write's own, for the same reason: appendVSPTiming is called
	// immediately after m_C64->write() returns.
	event.writeBASpins = (u16)radBASpinsLastWrite;
	event.sequence = sequence;
	event.hostPerC64 = perC64 > 0xFFFFFFFFULL ? 0xFFFFFFFFu : (u32)perC64;
	event.address = address;
	event.value = value;
	event.physicalValue = physicalValue;
	event.raster = m_VSPLastRaster;
	const u64 factor = effectiveCIAFactor();
	event.factor = factor > 255 ? 255 : (u8)factor;
	event.gamePhase = m_RAM[ 0x9F88 ];
	event.d016 = m_VICRegShadow[ 0x16 ];
	event.d018 = m_LastD018;
	event.flags = m_VSPHaveRasterAnchor ? SCPU_VSP_RASTER_VALID : 0;
	event.catchDeficit = 0;
	event.catchWait = 0;
	if ( m_VSPLastCatchValid )
	{
		const s64 d = m_VSPLastCatchDeficitHost;
		event.catchDeficit = d > 0x7FFFFFFF ? 0x7FFFFFFF
		                   : d < -0x7FFFFFFF ? -0x7FFFFFFF : (s32)d;
		event.catchWait = m_VSPLastCatchWaitHost > 0xFFFFFFFFULL
		                ? 0xFFFFFFFFu : (u32)m_VSPLastCatchWaitHost;
		event.flags |= SCPU_VSP_CATCH_VALID;
	}
	// Stolen cycles since the previous retained event. Across Mario's DEC/INC
	// $D011 pair this is the badline steal the real VIC imposed BETWEEN them:
	// the INC should resume at cycle 54 and complete at 59, so dprev= and this
	// field have to move together or the hold was not produced by the stall.
	{
		const u64 stalled = m_BAStallCycles - m_VSPStallAtLastEvent;
		event.baStallCycles = stalled > 0xFFFF ? 0xFFFF : (u16)stalled;
		m_VSPStallAtLastEvent = m_BAStallCycles;
		// How many freezes made up those cycles. One badline and ten sprite
		// steals are the same cycle total and a different mechanism, and the
		// per-event anchor residual only matters when this reads large.
		const u64 events = m_BAStallEvents - m_VSPStallEventsAtLast;
		event.baStallEvents = events > 0xFFFF ? 0xFFFF : (u16)events;
		m_VSPStallEventsAtLast = m_BAStallEvents;
		// Distance from the measured release to this write. Saturated rather
		// than wrapped: a stale or absent stamp has to read as "no reference",
		// never as a small plausible number.
		const u64 since = host - m_BAStallLastReleaseHost;
		event.sinceReleaseHost =
			( m_BAStallLastReleaseHost && host >= m_BAStallLastReleaseHost
			  && since < 0xFFFF ) ? (u16)since : 0xFFFF;
		// And WHOSE release it was. Without this the field has a silent failure
		// mode that the rest of the instrument cannot see: a sprite steal
		// releasing between the badline release and this write replaces the
		// reference with one about 40 cycles later, and postrel= then reads a
		// small plausible number for the wrong event.
		if ( m_BAStallLastReleaseLen >= 20 )
			event.flags |= SCPU_VSP_RELEASE_BADLINE;
		// The badline-class reference, on its own clock and its own width.
		// Saturated rather than wrapped for the same reason sinceReleaseHost
		// is: no reference at all has to read as no reference, never as a
		// small plausible number. u32 reaches 3.1 million cycles, so a frame
		// and a half -- it cannot run out inside the window being measured.
		{
			const u64 sinceBad = host - m_BAStallLastBadlineHost;
			event.sinceBadlineHost =
				( m_BAStallLastBadlineHost && host >= m_BAStallLastBadlineHost
				  && sinceBad < 0xFFFFFFFFULL ) ? (u32)sinceBad : 0xFFFFFFFFu;
			event.badlineReleaseLen = m_BAStallLastBadlineLen;
		}
	}
	// The displacement fields. Drained and zeroed here so each event reports
	// the window since the PREVIOUS retained event, matching bastall= rather
	// than dprev=: a ring overwrite therefore loses a window instead of
	// silently merging two, which is the failure code= already prints "seq" for.
	event.stallHost = m_GuestStopMaxHost;
	event.stallCause = (u8)m_GuestStopMaxCauses;
	event.d012Reads = m_VSPD012Reads;
	event.d012First = m_VSPD012First;
	// Saturated rather than wrapped, like sinceReleaseHost: no observed
	// assertion at all has to read as "no reference", never as a small
	// plausible number.
	event.irqToWriteHost =
		( m_IRQAssertHost && host >= m_IRQAssertHost
		  && host - m_IRQAssertHost < 0xFFFFFFFFULL )
		? (u32)( host - m_IRQAssertHost ) : 0xFFFFFFFFu;
	// K503 histograms: the first $D011 write after an anchor read, whole
	// cycles of IRQ->write and the anchor's value, both vs the first sample.
	if ( address == 0xD011 && m_VSPHistPending && m_VSPHaveRasterAnchor
	     && event.irqToWriteHost != 0xFFFFFFFFu && perC64 )
	{
		m_VSPHistPending = false;
		const u64 cyc = ( (u64)event.irqToWriteHost + perC64 / 2 ) / perC64;
		if ( cyc < 128 ) m_VSPHistIrq[ cyc ]++;
		else             m_VSPHistIrqOther++;
		m_VSPHistAnchor[ m_VSPLastRaster ]++;
	}
	m_GuestStopMaxHost = 0;
	m_GuestStopMaxCauses = 0;
	m_VSPD012Reads = 0;
	m_VSPD012First = 0;
	if ( address == 0xD011 )
	{
		// These predicates describe the nearest guest-observed raster anchor.
		// The raw raster/host stamps remain authoritative: offline analysis can
		// reconstruct the write line with the measured physical C64 clock rather
		// than mistaking this low-byte anchor for an exact horizontal position.
		if ( m_VSPHaveRasterAnchor
		     && ( m_VSPLastRaster & 7 ) == ( value & 7 ) )
			event.flags |= SCPU_VSP_ANCHOR_BADLINE_MOD;
		if ( m_VSPHaveRasterAnchor && m_VSPLastRaster >= 0x30
		     && m_VSPLastRaster <= 0xF7 )
			event.flags |= SCPU_VSP_ANCHOR_BADLINE_RANGE;
		if ( value & 0x10 )
			event.flags |= SCPU_VSP_D011_DEN;
	}
}

// Age of an already-paid physical observation, in WHOLE C64 cycles. Saturated
// rather than wrapped, and 0xFFFF for "no observation at all": a missing
// reference has to read as missing, never as a small plausible number. This is
// the same discipline as sinceReleaseHost above, and the units are deliberately
// NOT the hundredths that dprev= and stall= print -- a capture that mixes the
// two silently has already cost this project one wrong conclusion.
static inline u16 ictlAge( u64 now, u64 then, u32 perC64, bool valid )
{
	if ( !valid || !perC64 || now < then ) return 0xFFFF;
	const u64 cycles = ( now - then ) / perC64;
	return cycles < 0xFFFF ? (u16)cycles : 0xFFFE;
}

// The follow-up half of the acknowledgement test. Taken from the CACHED /IRQ
// level, which costs no GPIO read at all and is at most one sampling quantum
// old -- one C64 cycle at the 1MHz Mario's stabiliser runs at, because
// m_PacingCheckCycles is emulated cycles per microsecond. lateHost says how far
// after the write it was taken, so the reader can tell a meaningful gap from a
// useless one instead of trusting this note.
//
// WHY A FOLLOW-UP EXISTS AT ALL: /IRQ is open collector with a pull-up, and our
// immediate post-write sample is taken within a cycle or two of the write
// landing. "Still low" there could be propagation rather than a latch that did
// not clear. Two samples separate those; one cannot.
void CC64Memory::noteIntCtlLate( u64 host )
{
	if ( !m_ICtlFollowDue ) return;
	VICIntCtlWrite &e =
		m_ICtlRing[ ( m_ICtlFollowDue - 1 ) & ( SCPU_ICTL_RING - 1 ) ];
	m_ICtlFollowDue = 0;
	if ( host < m_ICtlFollowHost ) return;
	const u64 d = host - m_ICtlFollowHost;
	e.lateHost = d < 0xFFFF ? (u16)d : 0xFFFF;
	// AND THE CACHE HAS TO BE NEWER THAN THE WRITE, or this reports the level
	// from BEFORE the acknowledgement as the level after it -- which would
	// manufacture exactly the finding the field exists to test. m_IntCredit is
	// emulated cycles since the last sample: free, exact, and already
	// maintained by tickFast. Converted through the CURRENT rate, so the test
	// holds in turbo as well as at the 1MHz the stabiliser runs at. A rejected
	// follow-up leaves IRQ_LATE_VALID clear and still reports lateh=, so the
	// gap stays visible and the verdict does not.
	// REWRITTEN FROM A CREDIT TO A STAMP, because a credit cannot answer this
	// question inside a batch. CW65C816::run executes up to EIGHT instructions
	// and calls busTick ONCE, at the batch boundary (w65c816.cpp:1327-1341), so
	// m_IntCredit -- which only tickFast advances -- is frozen at whatever that
	// boundary left it while the guest acknowledges and runs on. The boundary
	// leaves it at 0, because run() sets W65_PEND_SAMPLE and the batch's first
	// instruction samples and zeroes it. The old test ( 0 * perEmu > d ) was
	// therefore false, so a level sampled BEFORE the acknowledgement was stamped
	// IRQ_LATE_VALID and reported as the level after it -- exactly the failure
	// the paragraph above says must not happen, and the whole of K487's irq=101.
	//
	// m_IRQSampleEmu is the emulated cycle of the last real pin read and
	// m_ICtlFollowEmu the emulated cycle of this write: same clock domain, exact,
	// and no host-cycle read on a path that must stay cheap. Only the ORDER of
	// the two is asked, so this is rate-independent -- no conversion, and none of
	// the m_EmuCycles clock-domain trap. <= and not < is load-bearing: a sample
	// taken in the SAME emulated cycle as the write cannot be PROVEN to follow
	// it, so it is refused rather than guessed.
	if ( m_IRQSampleEmu <= m_ICtlFollowEmu ) return;
	e.flags |= SCPU_ICTL_IRQ_LATE_VALID;
	if ( !m_CachedIRQ ) return;
	e.flags |= SCPU_ICTL_IRQ_LATE;
	if ( e.reg != 0x19 ) return;
	m_D019AckHeldIRQLate++;
	m_ICtlAckHeldPending |= 2;
}

void CC64Memory::appendIntCtlWrite( u8 reg, u8 value, u8 prev )
{
	const u64 host = m_C64 ? m_C64->hostCycles() : m_EmuCycles;
	// The PREVIOUS entry's follow-up sample, resolved before this entry is
	// written so that the two can never be confused in the ring.
	noteIntCtlLate( host );
	// The SLOW period, deliberately, and not the accelerator's current one:
	// the ages below are C64 cycles, a property of the machine's clock, and
	// dividing host time by a turbo period would report them 20x too large.
	// This is the same clock-domain trap that cost a day over m_EmuCycles.
	const u32 perC64 = (u32)( m_HostPerEmuQ16Slow >> 16 );
	// WHO WROTE IT. The probe is the same one the watched-write and $D012-read
	// hooks use, called here on four registers out of 47 -- a few hundred times
	// a second -- and its cycle half is the LOW 32 BITS OF THE CORE's m_Cycles,
	// the same field and the same clock domain the interrupt ring stamps. That
	// is what makes "which rung armed this compare" a join and not a guess.
	const u64 probe = m_CPUProbe ? m_CPUProbe( m_CPUProbeCtx ) : 0;
	VICIntCtlWrite &e = m_ICtlRing[ m_ICtlHead & ( SCPU_ICTL_RING - 1 ) ];
	e.cycles = (u32)( probe >> 32 );
	e.pc = (u32)probe;
	e.compare = m_RasterCompare;
	e.reg = reg;
	e.value = value;
	e.prev = prev;
	// The OTHER half of the compare, as the shadow holds it, so compare= can be
	// recomposed and checked instead of believed.
	e.other = ( reg == 0x12 ) ? m_VICRegShadow[ 0x11 ]
	                          : m_VICRegShadow[ 0x12 ];
	e.d01a = m_VICRegShadow[ 0x1A ];
	e.graster = m_VSPLastRaster;
	e.praster = m_D011RST8LastLow;
	e.grasterAge = ictlAge( host, m_VSPLastRasterHost, perC64,
	                        m_VSPHaveRasterAnchor );
	e.prasterAge = ictlAge( host, m_D011RST8LastHost, perC64,
	                        m_D011RST8LastHost != 0 );
	e.lateHost = 0xFFFF;
	u32 flags = 0;
	if ( m_VSPHaveRasterAnchor ) flags |= SCPU_ICTL_GRASTER_VALID;
	if ( m_D011RST8LastHost ) flags |= SCPU_ICTL_PRASTER_VALID;
	if ( m_CachedIRQ ) flags |= SCPU_ICTL_IRQ_BEFORE;
	if ( m_C64 && m_RasterCompare
	     >= (u16)c64RasterLines( m_C64->signals().video ) )
		flags |= SCPU_ICTL_COMPARE_IMPOSSIBLE;
	if ( reg == 0x19 )
	{
		m_D019Writes++;
		// THE ONE NEW PHYSICAL OBSERVATION IN THIS INSTRUMENT, and it is a GPIO
		// PIN LEVEL -- one GPLEV0 read, about four times a frame -- not a bus
		// cycle and not a register read. $FF to $D019 clears EVERY VIC latch,
		// so a line still low immediately after the ack is either a VIC latch
		// our write path did not clear or a non-VIC source holding the shared
		// open-collector /IRQ. It cannot separate those two by itself. What it
		// CAN do is rule both out: a line that RELEASES here says the VIC was
		// the only asserting source at that instant and the ack worked, which
		// is the half of the question firmware can answer at all.
		//
		// NOT a CIA read, and never will be from here: $DC0D and $DD0D clear on
		// read, so sampling them for observation would destroy the very state
		// the guest's own handler is about to act on.
		if ( m_C64 )
		{
			bool irq = false, nmi = false;
			m_C64->sampleInterrupts( irq, nmi );
			(void)nmi;
			flags |= SCPU_ICTL_IRQ_AFTER_VALID;
			if ( irq )
			{
				flags |= SCPU_ICTL_IRQ_AFTER;
				m_D019AckHeldIRQ++;
				m_ICtlAckHeldPending |= 1;
			}
		}
	}
	e.flags = (u8)flags;
	// AFTER AN ACKNOWLEDGEMENT, RE-READ THE PIN BEFORE DECIDING ANYTHING.
	// The core caches the IRQ LEVEL as well as this layer does:
	// CW65C816::stepInner reads it from the sticky W65_PEND_IRQLINE
	// (w65c816.cpp:451) and only re-samples when W65_PEND_SAMPLE is set, which
	// run() sets once per BATCH of up to eight instructions (w65c816.cpp:1344).
	// So without this, a level sampled before the guest's $D019 ack can still be
	// acted on up to eight instructions after it -- including across the RTI that
	// clears I, which enters the next vector with /IRQ already released. In Mario
	// that window straddles STA $D019 / LDA # / STA $D012 / PLA.../RTI, and
	// whether the batch boundary falls inside it is pure phase luck: the shape of
	// a hang that only appears under scrolling. A C64-bus access is charged a
	// whole C64 cycle AT the batch boundary, so for an I/O-heavy batch the stale
	// window is ~8 C64 cycles -- the order K487 saw.
	//
	// THE ACK IS NOT TOUCHED, AND THIS IS WHY IT IS HERE RATHER THAN AT THE
	// SHADOW ASSIGNMENT: the physical store m_C64->write( a, physicalValue )
	// runs in the VIC write path immediately BEFORE this function is called, so
	// at this point the byte has provably reached the chip. This only forces the
	// next interrupt decision to be made on a pin sample taken after it.
	// fineTicksRequired() ends the batch on the very next check (the IEC
	// mechanism, reused, not a new one), tickFast then advances the credit and
	// run() sets W65_PEND_SAMPLE, so the following instruction samples the pin
	// fresh. $D01A joins $D019 because unmasking is just as capable of changing
	// the line. Nothing is added to the per-VIC-write path: this function is
	// reached on four registers out of 47, a few hundred times a second.
	// COST: two stores here, and one extra GPLEV0 read per acknowledgement --
	// about eight a frame against the one-per-emulated-microsecond the sampling
	// quantum already pays. Deliberately no CIA read: $DC0D/$DD0D clear on read.
	if ( ( reg == 0x19 && ( value & 0x0F ) ) || reg == 0x1A )
	{
		m_IRQResampleDue = true;
		m_IntCredit = 0xFFFFF;
	}
	m_ICtlFollowDue = (u16)( ( m_ICtlHead & ( SCPU_ICTL_RING - 1 ) ) + 1 );
	m_ICtlFollowHost = host;
	// The same instant on the EMULATED clock, for the follow-up's validity test.
	// lateHost stays in host cycles because that is what the reader wants to see;
	// the VERDICT is decided in emulated cycles because that is the domain the
	// sample stamp lives in. Mixing the two is the trap that cost this project a
	// day over m_EmuCycles and a capture over the gras= ages.
	m_ICtlFollowEmu = m_EmuCycles;
	m_ICtlHead++;
	// Latch AFTER the entry, not at the compose that set the flag: the write
	// that composed the impossible compare is the most interesting entry in the
	// ring and the detection happens before this append exists.
	if ( !m_ICtlFreezeDue ) return;
	m_ICtlFreezeDue = 0;
	freezeIntCtlRing( 1 );
}

void CC64Memory::freezeIntCtlRing( u32 reason )
{
	if ( m_ICtlFrozenReason == 1 ) return;		// the precise latch wins
	for ( u32 i = 0; i < SCPU_ICTL_RING; i++ )
		m_ICtlFrozen[ i ] = m_ICtlRing[ i ];
	m_ICtlFrozenHead = m_ICtlHead;
	m_ICtlFrozenReason = reason;
	const u64 probe = m_CPUProbe ? m_CPUProbe( m_CPUProbeCtx ) : 0;
	m_ICtlFrozenCycles = (u32)( probe >> 32 );
}

// The hypothesis-free backstop, called once per frame from CSuperCPU::runFrame
// with the progress detector's own episode count. A load sits in a tight loop
// for seconds and reaches SUSPECT harmlessly, so this re-copies on every
// episode exactly as CW65C816's own suspect copy does; the impossible-compare
// latch outranks it and cannot be overwritten.
void CC64Memory::noteProgressSuspects( u32 suspects )
{
	if ( suspects == m_ICtlSuspectsSeen ) return;
	m_ICtlSuspectsSeen = suspects;
	freezeIntCtlRing( 2 );
}

u64 CC64Memory::interruptContext()
{
	const u64 host = m_C64 ? m_C64->hostCycles() : m_EmuCycles;
	// Resolve a pending follow-up first: it belongs to the interval BEFORE this
	// interrupt, and the ack-held bits below report that interval.
	noteIntCtlLate( host );
	const u32 perC64 = (u32)( m_HostPerEmuQ16Slow >> 16 );
	// Quarter-cycles of a C64 cycle would be the wrong trade here: this is
	// "how stale is the line I am about to quote", and 4-cycle granularity over
	// a 0..1016 cycle range (16 raster lines) covers every staleness that could
	// still leave the quote usable. 0xFF means no anchor at all.
	u32 age = 0xFF;
	if ( m_VSPHaveRasterAnchor && perC64 && host >= m_VSPLastRasterHost )
	{
		const u64 quarters = ( host - m_VSPLastRasterHost ) / perC64 / 4;
		age = quarters > 0xFE ? 0xFE : (u32)quarters;
	}
	u32 flags = 0;
	if ( m_CachedIRQ ) flags |= SCPU_IRQCTX_IRQ_LEVEL;
	if ( m_VSPHaveRasterAnchor ) flags |= SCPU_IRQCTX_RASTER_VALID;
	if ( m_ICtlAckHeldPending & 1 ) flags |= SCPU_IRQCTX_ACK_HELD;
	if ( m_ICtlAckHeldPending & 2 ) flags |= SCPU_IRQCTX_ACK_HELD_LATE;
	m_ICtlAckHeldPending = 0;
	if ( m_C64 && m_RasterCompare
	     >= (u16)c64RasterLines( m_C64->signals().video ) )
		flags |= SCPU_IRQCTX_COMPARE_IMPOSSIBLE;
	return (u64)m_RasterCompare
	     | ( (u64)m_VICRegShadow[ 0x1A ] << 16 )
	     | ( (u64)m_VSPLastRaster << 24 )
	     | ( (u64)age << 32 )
	     | ( (u64)flags << 40 );
}

u32 CC64Memory::copyVSPTrace( VSPTimingEvent *out, u32 capacity,
	                           u32 &total, u32 &overwritten ) const
{
	total = m_VSPTraceHead;
	const u32 retained = total < SCPU_VSP_TRACE_SIZE
	                   ? total : SCPU_VSP_TRACE_SIZE;
	overwritten = total - retained;
	const u32 count = capacity < retained ? capacity : retained;
	const u32 first = total - retained;
	for ( u32 i = 0; i < count; i++ )
		out[ i ] = m_VSPTrace[ ( first + i ) & ( SCPU_VSP_TRACE_SIZE - 1 ) ];
	return count;
}

bool CC64Memory::initializeC64CIA2()
{
	if ( !m_C64 )
		return false;

	// This is intentionally the ONLY repeated CIA write in the emulator. The
	// physical C128 expansion port can lose the first standalone write after a
	// direction change; repeating arbitrary CIA writes would be unsafe because
	// many registers have edge, acknowledge, or timer-load semantics. DDRA is a
	// pure latch, and $3F is the C64 KERNAL's established state: VIC bank and IEC
	// outputs on PA0-PA5, receive lines on PA6-PA7.
	static const u8 c64DDRA = 0x3F;
	m_C64->write( 0xDD02, c64DDRA );
	m_C64->write( 0xDD02, c64DDRA );

	// Never promote one physical read to authoritative state. On marginal
	// machines it may be data-bus residue, and a later RMW would then persist
	// that residue back into the real CIA. The backend verifies at several
	// calibrated sample points before this shadow becomes live.
	if ( !m_C64->verifyC64CIA2DDRA( c64DDRA ) )
		return false;

	m_CIA2DDRA = c64DDRA;
	m_HaveCIA2DDRA = true;
	updateSpritePtrBase();
	updateHotShapeBlocks();
	return true;
}

// Mode 2's rule, and the whole argument for why it is safe: every real serial
// conversation -- KERNAL and JiffyDOS alike -- OPENS with the host pulling ATN
// low to send TALK or LISTEN. There is no transaction without that, so an ATN
// assertion is the one host-side act a VIC-bank write cannot imitate. CLK and
// DATA changes are the bit-banging INSIDE a conversation, so they re-arm an
// open window and do not open one. What protects a load in either mode is that
// this governs only the 2ms mirror window and the 100ms speed hold:
// notePollHold() still tops the 1MHz speed up on EVERY $DD00/$DD02 access,
// which is what the KERNAL's bit timing actually rests on, and that is
// untouched here.
bool CC64Memory::iecWriteEvidence( u8 driveBefore, u8 driveAfter ) const
{
	if ( m_IECWindowMode < 2 )
		return true;
	if ( ( driveAfter & 0x08 ) && !( driveBefore & 0x08 ) )
		return true;
	return m_IECHoldCycles != 0 || m_IECActivityCycles != 0;
}

void CC64Memory::noteIECWriteArm( bool driveKnown, u8 driveBefore, u8 driveAfter )
{
	// Unknown CIA state keeps arming unconditionally, in every mode: the
	// existing note on cia2IECChanged is right that a harmless initial 1MHz
	// interval beats delaying the first protocol edge of a cold machine.
	if ( !driveKnown || iecWriteEvidence( driveBefore, driveAfter ) )
	{
		m_IECArmWrite++;
		noteIECActivity();
	}
	else
		m_IECArmWriteRefused++;
}

void CC64Memory::noteIECActivity()
{
	const bool wasQuiet = ( m_IECActivityCycles == 0 );
	// Serial edges continually re-arm this transaction window.  Keep it long
	// enough to span IEC's >200us EOI pause, but short enough that screen
	// mirroring resumes promptly after the final edge.  The separate 100ms
	// m_IECHoldCycles timer remains responsible for automatic 1MHz pacing; it
	// is deliberately not evidence that the bus is still active.
	m_IECActivityCycles = 2000;		// 2ms at the serial path's 1MHz rate
	if ( wasQuiet && m_TimingHook )
		m_TimingHook( m_TimingHookCtx );

	// CMD normally selects 1MHz explicitly. The automatic hold is the fallback
	// for serial code which reaches CIA2 while still nominally in turbo mode.
	const bool explicitlySlow = m_SelectedEmulatedHz != 0
	                         && m_SelectedEmulatedHz <= 1000000u;
	if ( m_IECThrottleEnabled && !explicitlySlow )
	{
		const bool wasInactive = ( m_IECHoldCycles == 0 );
		if ( wasInactive )
			m_IECThrottleEvents++;
		m_IECHoldCycles = 100000;
		if ( wasInactive )
		{
			cia2OnRateChange();		// hold arming changes the effective speed
			if ( m_TimingHook )
				m_TimingHook( m_TimingHookCtx );
		}
	}
}

void CC64Memory::updateBankMode()
{
	C64BankConfig cfg;
	cfg.port01 = c64PortEffective( m_Port00, m_Port01 );

	if ( m_C64 )
	{
		const C64Signals &s = m_C64->signals();
		cfg.game  = s.gameAsserted  ? 0 : 1;
		cfg.exrom = s.exromAsserted ? 0 : 1;
	} else
	{
		cfg.game = cfg.exrom = 1;	// nothing plugged in
	}

	m_BankMode = c64BankMode( &cfg );

	// m_BankMode is a read-decode input and this is its only assignment in
	// Source/, so this one call covers $00/$01 writes, reset() and the
	// constructor. See the note on CC64Memory::m_ReadBase.
	rebuildReadDecode();
	// ... and a WRITE-decode input, for the same reason and with a worse
	// failure mode. See the note on CC64Memory::m_WriteBase.
	rebuildWriteDecode();
}

// The resolved form of readFast()'s own arms, for the bootmap-off map.
// Nothing else may appear here: an arm this has and readFast does not is
// only slower, but an arm readFast has and this does not is a wrong byte.
void CC64Memory::rebuildReadDecode()
{
	for ( u32 p = 0; p < 16; p++ )
	{
		const u8 *base = 0;
		const u16 first = (u16)( p << 12 );

		if ( m_ReadDecodeEnable )
			switch ( c64MapRead( first, m_BankMode ) )
			{
			case REG_RAM:
				base = m_RAM + first;
				break;

			case REG_BASIC:
				// Bank-1 SRAM at the same offset, not the C64's ROM -- see
				// setROMShadow(). Only ever pages $A-$B (c64BankingInit).
				if ( m_ROMShadow && !m_SCPU128Mode )
					base = m_ROMShadow + first;
				break;

			case REG_KERNAL:
				// Only ever pages $E-$F (c64BankingInit), so first - 0xE000 is
				// 0 or 0x1000 and the base lands at bank 1 $E000/$F000 for KT
				// or $6000/$7000 for KS -- inside the 64K shadow either way.
				if ( m_ROMShadow && !m_SCPU128Mode )
					base = m_ROMShadow + m_KernalShadowBase + ( first - 0xE000 );
				break;

			default:
				// I/O, character ROM, cartridge and open bus either have read
				// side effects or no linear backing store. read8() answers
				// those, exactly as readFast has always declined to it.
				break;
			}

		m_ReadBase[ p ] = base;
	}
}

// The resolved form of writeFast()'s own arm, for the bootmap-off map. Same
// contract as rebuildReadDecode above it: an arm this has and writeFast does
// not is only slower, an arm writeFast has and this does not is a wrong byte
// -- and on this side a wrong byte is a PERMANENT one, because the store
// lands in DRAM instead of in a register. See the note on m_WriteBase.
void CC64Memory::rebuildWriteDecode()
{
	for ( u32 p = 0; p < 16; p++ )
	{
		u8 *base = 0;
		const u16 first = (u16)( p << 12 );

		// ONE arm, where the read table has three: writes never land in ROM,
		// they fall through to the DRAM underneath (c64WriteIsIO). REG_IO is
		// the only region with write side effects and no linear backing
		// store, and write8() keeps it -- together with the SCPU128 $D5xx /
		// $D6xx / $FF0x window and the DOS-extension window, both of which
		// writeFast still reaches write8() for exactly as before.
		// The predicate is the WRITE-side helper on purpose. It is the same
		// c64RegionTable byte as c64MapRead() != REG_IO, and writeFast used to
		// spell it the read way, but this table's whole job is to answer
		// "would write8() have sent this page to the I/O path?" and the test
		// (bank0_writefast_matches_write8_in_every_banking_state) asserts the
		// shape against c64WriteIsIO. One name, one authority, no drift.
		if ( m_WriteDecodeEnable && !c64WriteIsIO( first, m_BankMode ) )
			base = m_RAM + first;

		m_WriteBase[ p ] = base;
	}
}

void CC64Memory::setBasicROM( const u8 *data )
{
	for ( u32 i = 0; i < C64_BASIC_SIZE; i++ ) m_Basic[ i ] = data[ i ];
	m_HasBasic = true;
}

void CC64Memory::setKernalROM( const u8 *data )
{
	for ( u32 i = 0; i < C64_KERNAL_SIZE; i++ ) m_Kernal[ i ] = data[ i ];
	m_HasKernal = true;
}

void CC64Memory::setCharROM( const u8 *data )
{
	for ( u32 i = 0; i < C64_CHARROM_SIZE; i++ ) m_CharROM[ i ] = data[ i ];
	m_HasChar = true;
}

bool CC64Memory::snapshotROMsFromBus()
{
	if ( !m_C64 )
		return false;

	// Only fill in what the caller did not already supply. Overwriting an image
	// that was deliberately loaded from a file would silently discard it -- and
	// the two ROMs are chosen independently, so "KERNAL from a file, BASIC from
	// the machine" has to work.
	if ( m_HasBasic && m_HasKernal )
		return true;

	// Sanity check: the machine must currently have its ROMs banked in. The
	// reset vector lives in the KERNAL and must point back into it; if it does
	// not, the halted 6510 left $01 somewhere unhelpful and everything we read
	// would be DRAM rather than ROM.
	u16 resetVec = (u16)( m_C64->read( 0xFFFC ) | ( (u16)m_C64->read( 0xFFFD ) << 8 ) );
	m_IOReads += 2;
	if ( resetVec < 0xE000 )
		return false;

	if ( !m_HasKernal )
	{
		m_C64->readBlock( 0xE000, m_Kernal, C64_KERNAL_SIZE );
		m_HasKernal = true;
	}

	if ( !m_HasBasic )
	{
		m_C64->readBlock( 0xA000, m_Basic, C64_BASIC_SIZE );
		m_HasBasic = true;
	}

	return true;
}

// ---------------------------------------------------------------------------

u8 CC64Memory::read8( scpu_addr_t addr )
{
	const u16 a = (u16)( addr & 0xFFFF );

	// The 6510's on-chip port shadows the first two bytes of address space.
	// $0001 must return the whole port, not just the three banking bits --
	// software reads the cassette sense and motor lines through here too.
	if ( a <= 1 )
		return ( a == 0 ) ? m_Port00 : c64PortRead( m_Port00, m_Port01 );
	if ( dosExtensionMapsBank1( a ) )
		return m_ROMShadow[ a ];

	// In native C128 mode these are decoded by the host independently of the
	// 6510-style $0001 PLA state. In particular $FF00 is the 8722's
	// always-visible configuration register, even while the SCPU boot ROM is
	// mapped. Let the real MMU/VDC answer until their shadow model exists.
	if ( m_SCPU128Mode
	     && ( ( a >= 0xD500 && a <= 0xD50B )
	       || ( a >= 0xD600 && a <= 0xD601 )
	       || ( a >= 0xFF00 && a <= 0xFF04 ) ) )
	{
		m_IOReads++;
		noteBusAccess( a, false );
		return m_C64 ? m_C64->read( a ) : 0xFF;
	}

	// DOS 2.04's native-C128 reset stub lives in the low 64K half, but its
	// first BRL transfers from $FE40 to $5E45. The bytes at low-half $5E45
	// are zero; the executable target is image offset $15E45 in the upper
	// half. This is the SCPU128 boot extension window, not ordinary C128 RAM.
	if ( m_BootmapActive && m_SCPU128Mode && m_BootmapROMLength >= 0x20000
	     && a >= 0x1000 && a <= 0x5FFF )
	{
		noteFastHalfCycles( 6 );
		return m_BootmapROM[ 0x10000u + a ];
	}

	// Bootmap sits ABOVE the PLA: while it is active the $01 banking bits do
	// not matter for these ranges. I/O is deliberately left alone -- the boot
	// code needs the VIC-II and the CIAs, and $D000-$DFFF stays I/O in VICE's
	// table at every configuration this emulator can reach.
	if ( m_BootmapActive
	     && ( ( a >= 0x8000 && a <= 0xCFFF ) || a >= 0xE000 ) )
	{
		noteFastHalfCycles( 6 );
		return m_BootmapROM[ a ];
	}

	switch ( c64MapRead( a, m_BankMode ) )
	{
	case REG_RAM:
		return m_RAM[ a ];

	case REG_BASIC:
		// Bank-1 SRAM at the same offset, not the C64's ROM -- see
		// setROMShadow(). The boot ROM's patched copy lives here.
		return m_ROMShadow ? m_ROMShadow[ a ] : m_Basic[ a - 0xA000 ];

	case REG_KERNAL:
		return m_ROMShadow ? m_ROMShadow[ m_KernalShadowBase + ( a - 0xE000 ) ]
		                   : m_Kernal[ a - 0xE000 ];

	case REG_CHARROM:
		noteBusAccess( a, false );
		return m_CharROM[ a - 0xD000 ];

	case REG_IO:
		{
			// Registers that live inside the cartridge never reach the C64.
			u8 v;
			if ( m_IO && m_IO->ioRead( a, v ) )
			{
				if ( m_IO->ioAccessNeedsStretch( a, false ) )
					noteBusAccess( a, false );
				return v;
			}
			// Deliberately no flush here. Flushing before every I/O access
			// meant a booting KERNAL -- which touches I/O constantly -- drove
			// a continuous stream of unscheduled bursts across the visible
			// display, corrupting the VIC-II's fetches. Mirroring is now
			// scheduled against the raster in CSuperCPU::runFrame().
			//
			// The cost is that a program which stages a screen and immediately
			// points the VIC at it may show stale data for one frame. The real
			// SuperCPU mirrors asynchronously and has the same property.
			m_IOReads++;
			noteBusAccess( a, false );
			// Once verified or written by the guest, CIA2 DDRA is a pure output
			// latch. While /DMA is held, the physical 6510/8502 cannot alter it
			// behind us, so a marginal physical read must never poison the shadow.
			// Still charge the ordinary I/O stretch above: shadowing correctness
			// must not turn a 1MHz-class access into a fast private-register access.
			const u8 vicReadRegister = (u8)( a & 0x3F );
			const bool hdmiCollisionRead = a >= 0xD000 && a < 0xD400
			                            && ( vicReadRegister == 0x1E
			                              || vicReadRegister == 0x1F )
			                            && m_HDMIActive && *m_HDMIActive;
			if ( hdmiCollisionRead )
			{
				// VIC-II registers repeat every $40 bytes through $D3FF.
				// Treat collision-register mirrors exactly like $D01E/$D01F,
				// including their read-to-clear side effect.
				volatile u32 *latch = vicReadRegister == 0x1E
				                    ? m_HDMISpriteSpriteCollision
				                    : m_HDMISpriteBackgroundCollision;
				v = latch ? (u8)__atomic_exchange_n( latch, 0u,
				                                      __ATOMIC_ACQ_REL ) : 0;
			}
			else if ( a == 0xDD02 && m_HaveCIA2DDRA )
				v = m_CIA2DDRA;
			else if ( a >= 0xD000 && a < 0xD200 && vicReadRegister == 0x11
			          && m_VSPD011ShadowRead && d011ShadowReadSafe() )
			{
				// VSP_D011_SHADOW_READ (K500). No transfer: bits 6..0 come from
				// the shadow exactly as the reconstruction below would take
				// them, and bit 7 is 0 because the anchor proved it (see
				// d011ShadowReadSafe). What this removes from a DEC/INC $D011
				// pair is the read's 2.3 cycles and the turnaround the write
				// after it would pay, so the write reaches its count target
				// early and the catch-up places it; a frame that used to arrive
				// late and go out uncorrected no longer can.
				v = (u8)( m_VICRegShadow[ 0x11 ] & 0x7F );
				m_VSPD011ShadowReads++;
			}
			else
				v = m_C64 ? m_C64->read( a ) : 0xFF;
			if ( a >= 0xD000 && a < 0xD400 && vicReadRegister == 0x12 )
				noteVSPRasterAnchor( v );
			// $D011 is two different registers on read: bit 7 is the live
			// raster counter's ninth bit, while bits 6..0 are the writable VIC
			// control latch. Only the live bit needs the physical bus. Returning
			// all eight physical bits made a late/marginal expansion-bus sample
			// destructive: GEOS does LDA $D011 / AND #$7F / STA $D011, so one
			// residue value ($7F on the tested real C64) was written back and
			// enabled ECM+BMM, an invalid mode with a black bitmap but live
			// sprites. The halted host CPU means every writable VIC change passes
			// through write8(), so the shadow is authoritative for the low bits.
			// Bit 7 is now validated rather than trusted; see
			// d011RasterHighBit().  The MIRROR RANGE matches the WRITE path's
			// own ( a >= 0xD000 && a < 0xD200 ) exactly, so every address whose
			// write updates m_VICRegShadow[ 0x11 ] also reads back through this
			// reconstruction.  The old exact-address test left $D051, $D091,
			// $D0D1, $D111, $D151, $D191 and $D1D1 returning raw physical bits
			// 6..0 -- the same hole the GEOS residue paragraph above
			// describes, reachable through a mirror.
			if ( a >= 0xD000 && a < 0xD200 && vicReadRegister == 0x11 )
				v = (u8)( ( ( v & 0x80 ) ? d011RasterHighBit() : 0 )
				        | ( m_VICRegShadow[ 0x11 ] & 0x7F ) );
			// In fallback relocation mode the physical VIC receives a private
			// backing-store selector (for example $39 while Test Drive's guest
			// value remains $81).  That translated value is implementation state,
			// not a guest-visible register change.  VIC registers mirror every $40
			// bytes through $D3FF, so preserve the guest latch on every $D018 mirror.
			if ( a >= 0xD000 && a < 0xD400 && vicReadRegister == 0x18 )
				v = m_LastD018;
			// PRA reads combine live input pins with the hidden output latch. Once
			// both pieces of state are known, reconstruct that split explicitly:
			// output bits come from our authoritative latch, input bits remain the
			// physical sample. This prevents residue on PA0/PA1 from moving the VIC
			// bank through an otherwise innocent read-modify-write sequence.
			if ( a == 0xDD00 && m_HaveCIA2DDRA && m_HaveCIA2PortALatch )
				v = (u8)( ( v & (u8)~m_CIA2DDRA )
				        | ( m_CIA2PortALatch & m_CIA2DDRA ) );
			m_IOLog[ m_IOLogPos++ & 63 ] = (u32)a | ( (u32)v << 16 );
			// What raster the GUEST saw, with a stamp in the CORE's clock
			// domain so it can be read against the interrupt ring. One compare
			// on the I/O read path, which Mario takes a handful of times a
			// frame; the probe call is on the taken branch only.
			if ( vicReadRegister == 0x12 )
			{
				const u32 slot = m_D012ReadPos++ % SCPU_D012_RING;
				m_D012Reads[ slot ] = v;
				m_D012ReadCyc[ slot ] = m_CPUProbe
					? (u32)( m_CPUProbe( m_CPUProbeCtx ) >> 32 ) : 0;
				m_D012ReadCount++;
			}
			if ( ( a & 0xFE00 ) == 0xDC00 )
			{
				// Edge compression: a polling loop reads the same value tens of
				// thousands of times, and logging every repeat wipes the ring's
				// history in the gap between a freeze and the button press.
				// Log a read only when it DIFFERS from the last logged read.
				const u32 composite = (u32)a | ( (u32)v << 16 );
				if ( composite != m_CIALastRead )
				{
					m_CIALastRead = composite;
					m_CIALogCyc[ m_CIALogPos & 63 ] =
						m_C64 ? (u32)m_C64->hostCycles() : (u32)m_EmuCycles;
					m_CIALog[ m_CIALogPos++ & 63 ] = composite;
				}
			}
			if ( a == 0xDD00 || a == 0xDD02 )
			{
				// ANY serial-port read keeps the SPEED at 1MHz for a short
				// window, polls included -- that is the real SuperCPU's
				// auto-slow rule, and cycle-timed code depends on it. The
				// MIRROR window below stays evidence-gated.
				notePollHold();
			}
			if ( a == 0xDD00 )
			{
				// PA6/PA7 are the drive-to-computer CLK/DATA inputs. A changed
				// input, or an asserted input held across repeated polls, proves
				// that the serial conversation is still live. Keep this receive
				// sample separate from the output latch: otherwise the input bits
				// in a read destroy the baseline for the next PA3-PA5 write.
				const u8 inputMask = (u8)( 0xC0
				                   & ( m_HaveCIA2DDRA ? (u8)~m_CIA2DDRA : 0xFF ) );
				const bool inputChanged = m_HaveCIA2PortARead
				                       && ( ( m_LastCIA2PortARead ^ v ) & inputMask ) != 0;
				// PA6/PA7 are direct inputs: an IEC line is idle high and asserted
				// low. A PRA read reports pin levels, not the CIA's hidden output
				// latch, so never use it as a PA3-PA5 latch baseline.
				//
				// The level test runs over the lines WE ARE NOT PULLING DOWN. A
				// line the guest holds low reads back low through the wired-AND
				// bus (iecSelfDrivenInputMask), so counting it as "a drive is
				// answering" armed the 100ms hold on every $DD00 read for any
				// guest that left CLK or DATA set -- 98% armed during Mario
				// gameplay with no drive in the conversation at all. The EDGE test
				// above is deliberately left over the full input mask: an edge on a
				// line we drive is at worst a duplicate of the arm that line's own
				// write already made, never a false one.
				const u8 levelMask = m_IECWindowMode
				                   ? (u8)( inputMask & (u8)~iecSelfDrivenInputMask() )
				                   : inputMask;
				const bool inputAsserted = levelMask != 0
				                        && ( v & levelMask ) != levelMask;
				m_LastCIA2PortARead = v;
				m_HaveCIA2PortARead = true;

				// A changed input, or one a drive is actively holding low, is
				// evidence of a live conversation. The mere ACT of polling is
				// not, and treating it as such was a mirror-blackout latch-up:
				// re-arming the 50000-cycle window on any read once the window
				// happened to be open made it self-sustaining. That window is
				// only ~2.5ms of real time at 20MHz, so ordinary code reading
				// $DD00 a few times a frame -- VIC bank manipulation, raster
				// splits -- held it open forever. iecBusActive() gates ALL
				// mirroring, so the physical screen froze at whatever had last
				// been delivered while the emulated machine ran on perfectly:
				// static scenery intact, everything that moved stuck as a
				// half-delivered smear.
				//
				// Mode 2 goes one step further and lets a held level only RE-arm a
				// window that positive evidence already opened. That keeps the
				// behaviour the level test was ADDED for -- Test Drive's K411
				// deadlock, where a drive holds DATA low across an EOI pause with
				// no further edge -- while refusing to open one from a static
				// level alone.
				const bool levelArms = inputAsserted
				                    && ( m_IECWindowMode < 2
				                      || m_IECHoldCycles != 0
				                      || m_IECActivityCycles != 0 );
				if ( inputChanged || levelArms )
				{
					if ( inputChanged ) m_IECArmReadEdge++;
					else                m_IECArmReadLevel++;
					noteIECActivity();
				}
				else if ( ( v & inputMask ) != inputMask )
					// What mode 0 would have armed on and this mode did not. A
					// capture with this large and hold-armed near zero is the
					// direct measurement that the level arm was the over-arm.
					m_IECArmReadRefused++;
			}
			else if ( a == 0xDD0D )
			{
				// ICR read: the program acknowledged its NMI. The real read
				// (already performed above) cleared the chip; drop the
				// synthetic level and pending bits so the next underflow is
				// a fresh edge.
				m_SynthNMIActive = false;
				m_CIA2ICRPending = 0;
			}
			return v;
		}

	case REG_OPEN:
	default:
		// Open bus. Approximated as the high byte of the address, which is
		// what the last VIC fetch usually leaves floating.
		return (u8)( a >> 8 );
	}
}

void CC64Memory::write8( scpu_addr_t addr, u8 value )
{
	const u16 a = (u16)( addr & 0xFFFF );

	if ( a <= 1 )
	{
		if ( a == 0 ) m_Port00 = value; else m_Port01 = value;
		updateBankMode();

		// The C64 stores the port registers' shadow in DRAM too, and code does
		// read $00/$01 back through the VIC's eyes in a few places. Sink
		// before store -- see IMirrorSink::onRamWrite.
		mirrorRamWrite( a, value );
		m_RAM[ a ] = value;
		return;
	}

	if ( dosExtensionMapsBank1( a ) )
	{
		m_ROMShadow[ a ] = value;
		return;
	}

	if ( m_SCPU128Mode
	     && ( ( a >= 0xD500 && a <= 0xD50B )
	       || ( a >= 0xD600 && a <= 0xD601 )
	       || ( a >= 0xFF00 && a <= 0xFF04 ) ) )
	{
		m_IOWrites++;
		noteBusAccess( a, true );
		if ( m_C64 ) m_C64->write( a, value );
		return;
	}

	if ( c64WriteIsIO( a, m_BankMode ) )
	{
		const u32 physicalMapBefore = m_DisplayRelocGeneration;
		if ( m_IO && m_IO->ioWrite( a, value ) )
		{
			if ( m_IO->ioAccessUsesWriteBuffer( a, true ) )
				noteMirrorWrite();
			else if ( m_IO->ioAccessNeedsStretch( a, true ) )
				noteBusAccess( a, true );
			// An explicit software speed selection ends the ADVISORY
			// poll-slow immediately, even when the selection does not change
			// the nominal speed (then no speed hook fires, so it must happen
			// here): a program declaring its speed means the poll heuristic
			// must yield. The EDGE-armed hold is left alone -- it protects
			// live transactions and real edges re-arm it anyway.
			if ( ( a == 0xD07A || a == 0xD07B ) && m_IECPollHoldCycles )
			{
				m_IECPollHoldCycles = 0;
				cia2OnRateChange();
				if ( m_TimingHook )
					m_TimingHook( m_TimingHookCtx );
			}
			// Intercepted SuperCPU-register writes are logged too, tagged with
			// bit 25 -- the freeze forensics need to see the WHOLE conversation,
			// and CMD's idle loops are made almost entirely of these.
			m_IOLog[ m_IOLogPos++ & 63 ] = (u32)a | ( (u32)value << 16 )
			                             | ( 1u << 24 ) | ( 1u << 25 );
			return;
		}
		// Work out whether this can affect a physical IEC output. Unknown CIA
		// state is deliberately treated as a change: a harmless initial 1MHz
		// interval is preferable to delaying the first protocol edge.
		bool cia2IECChanged = false;
		// Kept so the arm site can apply the window mode's evidence rule: WHICH
		// line moved decides whether this is a transaction opening or a VIC-bank
		// write carrying stray bits in PA3-PA5.
		bool cia2DriveKnown = false;
		u8 cia2DriveBefore = 0, cia2DriveAfter = 0;
		const bool vicWrite = a >= 0xD000 && a < 0xD200;
		const u8 vicReg = (u8)( a & 0x3F );
		// Decided below, where the shadow byte this write replaces is still
		// in hand, and acted on after the PHYSICAL write has completed: a
		// $D019 entry's own /IRQ sample is only worth taking once the chip
		// has actually seen the acknowledgement.
		bool ictlWrite = false;
		u8 ictlPrev = 0;
		if ( a == 0xDD00 )
		{
			if ( m_HaveCIA2DDRA && m_HaveCIA2PortALatch )
			{
				cia2DriveKnown = true;
				cia2DriveBefore = cia2EffectiveIECDrive( m_CIA2PortALatch, m_CIA2DDRA );
				cia2DriveAfter  = cia2EffectiveIECDrive( value, m_CIA2DDRA );
				cia2IECChanged = cia2DriveBefore != cia2DriveAfter;
			}
			else
				cia2IECChanged = true;
		}
		else if ( a == 0xDD02 )
		{
			if ( m_HaveCIA2DDRA && m_HaveCIA2PortALatch )
			{
				cia2DriveKnown = true;
				cia2DriveBefore = cia2EffectiveIECDrive( m_CIA2PortALatch, m_CIA2DDRA );
				cia2DriveAfter  = cia2EffectiveIECDrive( m_CIA2PortALatch, value );
				cia2IECChanged = cia2DriveBefore != cia2DriveAfter;
			}
			else
				cia2IECChanged = true;
		}

		// Raster programs deliberately change VIC registers at exact scanlines,
		// and IEC line edges are timing-critical, so VIC/CIA writes are never
		// delayed behind mirror traffic. In particular, $D015 must stay immediate:
		// coalescing its transitions breaks sprite multiplexers and delaying a
		// falling bit leaves an unwanted sprite enabled.

		// $DD00 is the CIA2 output latch and $DD02 its direction register.
		// Keep both: a latch change only reaches the VIC/IEC pins when the
		// corresponding DDRA bit makes that pin an output.
		if ( a == 0xDD00 )
		{
			const u32 vicBankBefore = activeVICBankBase();
			const bool vicBankWasKnown = m_HaveCIA2PortALatch && m_HaveCIA2DDRA;
			m_CIA2PortALatch = value;
			m_HaveCIA2PortALatch = true;
			// Bits 3-5 are the IEC outputs and toggle for every serial bit.
			// They cannot alter the VIC map, sprite pointers or hot blocks.
			// Keep that timing-critical path out of display bookkeeping entirely.
			if ( !vicBankWasKnown || activeVICBankBase() != vicBankBefore )
			{
				recomputeVICPhysicalMap();
				updateSpritePtrBase();		// VIC bank lives in bits 0-1
				updateHotShapeBlocks();
				logVICBank();
			}
			if ( cia2IECChanged )
				noteIECWriteArm( cia2DriveKnown, cia2DriveBefore, cia2DriveAfter );
		}
		else if ( vicWrite && vicReg == 0x18 )
		{
			const u8 oldD018 = m_LastD018;
			m_LastD018 = value;
			// Bit 0 is unused by the VIC memory selectors. Preserve it in the
			// physical write below, but avoid rebuilding an unchanged map.
			if ( ( oldD018 ^ value ) & 0xFE )
			{
				recomputeVICPhysicalMap();
				updateSpritePtrBase();
				updateHotShapeBlocks();
			}
		}
		else if ( a == 0xDD02 )
		{
			const u32 vicBankBefore = activeVICBankBase();
			const bool vicBankWasKnown = m_HaveCIA2PortALatch && m_HaveCIA2DDRA;
			m_CIA2DDRA = value;
			m_HaveCIA2DDRA = true;
			if ( !vicBankWasKnown || activeVICBankBase() != vicBankBefore )
			{
				recomputeVICPhysicalMap();
				updateSpritePtrBase();		// input pins float high; direction can move the VIC bank
				updateHotShapeBlocks();
				logVICBank();
			}
			if ( cia2IECChanged )
				noteIECWriteArm( cia2DriveKnown, cia2DriveBefore, cia2DriveAfter );
		}
		else if ( a >= 0xDD04 && a <= 0xDD0F )
		{
			// CIA2 timer and interrupt-mask writes feed the NMI retiming
			// model on their way to the real chip.
			cia2TimerWrite( a, value );
		}

		m_IOWrites++;
		if ( vicWrite )
		{
			// The beam is at the OLD compare when an IRQ handler acknowledges or
			// re-aims it. Anchor only the first compare write in a handler; a
			// follow-up high/low-half write is already programming the next split.
			if ( m_VICLogEnabled )
			{
				const bool compareWrite = vicReg == 0x12
				   || ( vicReg == 0x11
				        && ( ( value ^ m_VICRegShadow[ 0x11 ] ) & 0x80 ) );
				if ( compareWrite )
				{
					const u32 now = (u32)c64Now();
					if ( (u32)( now - m_LastCompareWrite ) >= 256 )
						noteRasterPrecise( m_RasterCompare );
					m_LastCompareWrite = now;
				}
				else if ( vicReg == 0x19 && ( value & 0x01 )
				          && (u32)( (u32)c64Now() - m_LastCompareWrite ) >= 256 )
					noteRasterPrecise( m_RasterCompare );
			}

			const u8 oldVICValue = m_VICRegShadow[ vicReg ];
			// WHICH WRITES THE INTERRUPT-CONTROL RING KEEPS; see
			// VICIntCtlWrite for why the $D011 arm is ( value | prev ) & $80
			// and so excludes Mario's VSP DEC/INC pair by construction.
			ictlPrev = oldVICValue;
			ictlWrite = vicReg == 0x12 || vicReg == 0x19 || vicReg == 0x1A
			         || ( vicReg == 0x11
			              && ( ( value | oldVICValue ) & 0x80 ) );
			m_VICRegShadow[ vicReg ] = value;
			if ( vicReg == 0x11 )
			{
				// THE DIRECT TEST for the RST8 sampling bug, free on a branch
				// the map check already takes. Mario's own code is careful --
				// $09DA does AND #$7F before STA $D011 -- so across a gameplay
				// capture this must read 0. It reading non-zero is the proof
				// that a read-modify-write put raster bit 8 into the latch,
				// which is what composed the unreachable compare of 318. No
				// counter has to be inferred from an absent symptom any more.
				if ( value & 0x80 )
					m_D011RST8Writes++;
				// Only BMM selects between charset and bitmap backing. DEN,
				// scrolling and raster-MSB changes remain immediate but map-neutral.
				if ( ( oldVICValue ^ value ) & 0x20 )
				{
					recomputeVICPhysicalMap();
					updateSpritePtrBase();
					updateHotShapeBlocks();
				}
			}
			// Maintain both halves even while capture is disabled, so enabling
			// VIDEO_MODE 1 never begins with a stale compare line.
			if ( vicReg == 0x11 || vicReg == 0x12 )
			{
				m_RasterCompare = (u16)( m_VICRegShadow[ 0x12 ]
				                | ( ( m_VICRegShadow[ 0x11 ] & 0x80 )
				                    ? 0x100 : 0 ) );
				// NOT clamped. m_RasterCompare is a faithful decode of what the
				// guest programmed, and a program that really does arm a line
				// its machine does not have has to be visible as that, not as
				// a plausible neighbour. Counted instead, because "the chain is
				// parked on a compare that can never match" is the single most
				// expensive reading to reach by inference: K471 spent a capture
				// getting to 318. Four $D011/$D012 writes a frame in Mario, so
				// the signals() load is paid about 240 times a second.
				if ( m_C64 && m_RasterCompare
				     >= (u16)c64RasterLines( m_C64->signals().video ) )
				{
					m_RasterCompareImpossible++;
					// LATCH THE CONTROL RING HERE. This is the measured
					// signature of the Mario hang -- K471 reached 318 on a
					// 263-line machine -- and the button is pressed seconds
					// later, by which time the chain's own $D019/$D012 traffic
					// has overwritten the arming writes we need. Deferred one
					// append so the composing write is IN the frozen copy.
					// ...but ONLY when the compare can actually FIRE. K486
					// latched during boot at cycle 1531822 on CMD's own ROM
					// ($F8E5B0) writing $D011=$9B against $D012=$37, composing
					// 311 on a 263-line machine with $D01A = $00 -- raster IRQ
					// disabled, so the impossible value was inert and the ROM
					// overwrote it microseconds later. That false positive
					// locked the latch 600M cycles before the hang and cost a
					// whole capture. An impossible compare only matters if the
					// guest has armed the raster interrupt on it.
					if ( m_ICtlFrozenReason != 1
					     && ( m_VICRegShadow[ 0x1A ] & 0x01 ) )
						m_ICtlFreezeDue = 1;
				}
			}
			appendVICLog( vicReg, value );
			if ( vicReg == 0x15 )
				updateHotShapeBlocks();
		}
		// Colour RAM is deliberately not timestamped. A normal 1000-cell fill
		// would consume half the ring and make low-rate raster state lossy. HDMI
		// snapshots the colour array once per completed frame instead; mid-frame
		// colour-RAM rewrites are outside the replay fidelity contract.
		m_IOLog[ m_IOLogPos++ & 63 ] = (u32)a | ( (u32)value << 16 ) | ( 1u << 24 );
		if ( ( a & 0xFE00 ) == 0xDC00 )
		{
			m_CIALastRead = 0xFFFFFFFF;		// a write resets read compression
			m_CIALogCyc[ m_CIALogPos & 63 ] =
				m_C64 ? (u32)m_C64->hostCycles() : (u32)m_EmuCycles;
			m_CIALog[ m_CIALogPos++ & 63 ] = (u32)a | ( (u32)value << 16 ) | ( 1u << 24 );
		}
		if ( m_IO && m_IO->ioAccessUsesWriteBuffer( a, true ) )
			noteMirrorWrite();
		else
			noteBusAccess( a, true );
		if ( m_C64 )
		{
			const u8 physicalValue = vicWrite && vicReg == 0x18
			                       ? vicPhysicalD018( value ) : value;
			m_VSPLastCatchValid = false;	// K499: set again only if the gate below runs
			bool placedInWindow = false;	// K504: tells the bus to pay the pass inside the resync
			// RASTER-WINDOW CATCH-UP, the placement half of the VSP defect.
			//
			// The guest's last $D012 read is a physical time reference it then
			// counts cycles from. For the VIC to perform the number of
			// c-accesses the guest intended, this write has to reach the pin
			// that many C64 cycles after the read ACTUALLY happened. Nothing
			// upstream guarantees it: the credit arm in tickSettle is licensed
			// to run carried lag off with the spin skipped, at interpreter
			// speed, and when that burst falls between the read and this write
			// the window runs SHORT and the write goes out EARLY. One cycle
			// early is one c-access, which is one character column.
			//
			// MEASURED, .tmp/traces/vsptrace-k448-mario.txt, 21 consecutive
			// frames of ONE fixed code path, anchor-to-DEC in C64 cycles:
			// 59.08, 60.05..60.08, 61.03..61.07, 62.03..62.07, 63.03. Four
			// WHOLE cycles of spread while the sub-cycle residue stays inside
			// 0.05 of a cycle. Whole-cycle spread on a fixed instruction
			// sequence can only be unpaced guest time.
			//
			// Properties, so this cannot become another knob:
			//   it can only ever WAIT, like settleBusThrough;
			//   it is SCHEDULING, not charging -- m_EmuCycles is untouched and
			//     only the grid origin moves, the form baStallSettle uses at a
			//     BA release, so the carried lag survives and the credit arm's
			//     RATE behaviour is unchanged. Only the PHASE of this one
			//     access moves;
			//   past 8 cycles -- the same bound as PACER_CREDIT_CYCLES, all the
			//     unpaced sprinting the pacer can legitimately have done -- it
			//     DECLINES and counts rather than spinning;
			//   a stale anchor is excluded by the 200-cycle window guard, so a
			//     handler that has not polled $D012 pays nothing.
			// BA cycles cancel: the stall charges the guest and real time
			// passes, so stolen cycles sit inside BOTH sides of the compare.
			// An under-charged stall can only shrink the deficit, never invent
			// one, which is the safe direction.
			if ( vicWrite && m_C64 && m_VSPHaveRasterAnchor
			     && m_HostPerEmuQ16Slow && effectiveCIAFactor() == 1
			     // NOT inside a serial window. effectiveCIAFactor() answers 1
			     // for an IEC hold with turbo still selected as well as for a
			     // genuine 1MHz selection, so the gate above alone would let
			     // this fire mid-transaction. baStallSettle already refuses to
			     // freeze there and says why: our IEC edges reach a REAL drive
			     // through the physical CIA, the drive has no model of our
			     // freeze, and a JiffyDOS byte is timed reads 10-15us apart.
			     // This spins up to 8us -- smaller than a badline's 43us, not
			     // small against that handshake. Same doctrine, same guard.
			     && !m_IECActivityCycles
			     && ( vicReg == 0x11 || vicReg == 0x16 || vicReg == 0x18 ) )
			{
				const u64 catchPerC64 = m_HostPerEmuQ16Slow >> 16;
				const u64 counted = c64Now() - m_VSPLastRasterC64;
				const u64 entry = m_C64->hostCycles();
				// GRID INSTANT TO GRID INSTANT. The anchor is where the read's own
				// settle exited on the pacer's grid, so `counted` cycles later is
				// where THIS write's settle exits on the same grid: inside the cycle
				// before the write's, from which the primitive's resync lands the
				// drive in the write's own cycle. Anchored on the read's RETURN, as
				// this stood before, the target sat at the START of the write's own
				// cycle and the resync carried every placed write one whole cycle
				// past it -- two, with the K243 pass that then followed the resync.
				// That was the constant skew, one oracle cell per cycle.
				const u64 target = ( m_VSPPlacementK501 ? m_VSPLastRasterGridHost
				                                        : m_VSPLastRasterHost )
				                 + ( ( counted * m_HostPerEmuQ16Slow ) >> 16 );
				const s64 deficit = (s64)( target - entry );
				if ( counted <= 200 )
				{
					placedInWindow = m_VSPPlacementK501;
					// The launch instant with no aim: the count target when the
					// write is early by up to the 8-cycle licence, otherwise now
					// (past the licence it declines and counts, as before).
					s64 launch = (s64)entry;
					if ( deficit > 0 )
					{
						if ( deficit <= (s64)( 8 * catchPerC64 ) ) launch = (s64)target;
						else                                         m_VSPCatchupDeclined++;
					}
					const s64 launchUnaimed = launch;
					// VSP_AIM_WRITE (K498), applied HERE and nowhere else: this is
					// the last gate before the primitive and the only one that
					// knows the count target. Whole cycles added to the count
					// target of every placed write after the anchor and, for the
					// FIRST of them, to the launch instant itself, so a write that
					// arrived after its target (entry-bound) still moves by
					// exactly the aim. Every later write of the same handler
					// already carries that displacement physically -- the grid
					// origin moves with the wait below -- and meets a target moved
					// by the same amount, so the sequence shifts uniformly and the
					// DEC->INC interval is untouched. The pacer-level trim
					// (tickPhaseTrim) is not used for writes: in the host model it
					// landed before this catch-up and the catch-up slack swallowed
					// it (+0.98 of a +2 aim) while the INC received it twice.
					// A negative aim can only give back catch-up slack; the launch
					// never precedes entry. Negative readings that stop changing
					// are the floor, not a broken knob.
					if ( m_VSPAimWriteCycles )
					{
						const s32 aim = m_VSPAimWriteCycles;
						const u64 mag = ( (u64)( aim < 0 ? -aim : aim )
						                  * m_HostPerEmuQ16Slow ) >> 16;
						const s64 aimHost = aim < 0 ? -(s64)mag : (s64)mag;
						s64 aimed = (s64)target + aimHost;
						if ( aimHost > 0 )
						{
							if ( m_VSPAimFloorPending && aimed < (s64)entry + aimHost )
								aimed = (s64)entry + aimHost;
							if ( aimed > launch
							     && aimed - (s64)entry <= (s64)( 8 * catchPerC64 ) + aimHost )
								launch = aimed;
						}
						else if ( launch > (s64)entry )
							launch = aimed > (s64)entry ? aimed : (s64)entry;
						m_VSPAimFloorPending = false;
					}
					if ( launch > (s64)entry )
					{
						u64 now = entry;
						do { now = m_C64->hostCycles(); }
						while ( (s64)( (u64)launch - now ) > 0 );
						m_PacingAnchor += now - entry;
						const u64 unaimed = launchUnaimed > (s64)entry
						                  ? (u64)( launchUnaimed - (s64)entry ) : 0;
						m_VSPCatchupHost += unaimed;
						if ( unaimed ) m_VSPCatchupEvents++;
						if ( launch != launchUnaimed )
						{
							m_VSPAimWaitHost += ( now - entry ) - unaimed;
							m_VSPAimEvents++;
						}
					}
					// For the trace record (K499): what this write looked like
					// at entry and what was done about it.
					m_VSPLastCatchValid = true;
					m_VSPLastCatchDeficitHost = deficit;
					m_VSPLastCatchWaitHost = launch > (s64)entry
					                       ? (u64)( launch - (s64)entry ) : 0;
				}
			}
			if ( placedInWindow ) m_C64->setWriteTurnaroundMinimal( true );
			m_C64->write( a, physicalValue );
			if ( placedInWindow ) m_C64->setWriteTurnaroundMinimal( false );
			if ( vicWrite && ( vicReg == 0x11 || vicReg == 0x16
			                   || vicReg == 0x18 ) )
				appendVSPTiming( (u16)( 0xD000 | vicReg ), value,
				                 physicalValue, placedInWindow );
			if ( ictlWrite )
				appendIntCtlWrite( vicReg, value, ictlPrev );
			// $D011, $DD00 and $DD02 can activate/deactivate relocation
			// without a guest $D018 write. Keep the physical selector paired
			// with the new backing map while guest-visible state stays intact.
			if ( physicalMapBefore != m_DisplayRelocGeneration
			     && !( vicWrite && vicReg == 0x18 ) )
				m_C64->write( 0xD018, vicPhysicalD018( m_LastD018 ) );
		}
		return;
	}

	// Everything else falls through to DRAM, including writes "into" ROM.
	// Sink before store -- see IMirrorSink::onRamWrite.
	{
		const u8 old = m_RAM[ a ];
		SCPU_COUNT_WRITE( m_RamWrites );
		mirrorRamWrite( a, value );
		m_RAM[ a ] = value;

		// Same side effects as writeFast, because it is literally the same
		// function. This path used to carry its own copy and had already
		// fallen behind once.
		noteRAMWriteSideEffects( a, old, value );
	}
}

bool CC64Memory::irqAsserted()
{
	return irqFast();
}

bool CC64Memory::nmiAsserted()
{
	return nmiFast();
}

void CC64Memory::setPacing( u64 hostCyclesPerSecond, u32 emulatedHz )
{
	m_SelectedEmulatedHz = emulatedHz;
	m_PacingCheckCycles = emulatedHz / 1000000u;
	if ( m_PacingCheckCycles < 1 ) m_PacingCheckCycles = 1;

	// An explicit slow request supersedes any heuristic hold, so a later $D07B
	// can restore turbo immediately rather than inheriting a 100ms tail.
	if ( emulatedHz != 0 && emulatedHz <= 1000000u )
		m_IECHoldCycles = 0;

	// Speed selection is the canonical rate transition: Winter Games arms
	// its transition timer and writes $D07B (1MHz) one instruction later.
	// Armed CIA2 deadlines must follow the machine into the new units.
	cia2OnRateChange();

	if ( hostCyclesPerSecond == 0 || emulatedHz == 0 )
	{
		m_HostPerEmuQ16 = 0;			// pacing off
		return;
	}

	// A selected "1MHz" is the HOST MACHINE's own cycle, and no C64 runs at a
	// round megahertz. Dividing by 1000000 made a paced cycle 1400 ARM cycles
	// where the true NTSC period is 1368.9: a constant +2.27% dilation of
	// precisely the code that asks for 1MHz, which is where Mario's raster
	// handler runs because it writes $D07A. Both acquire paths detect the
	// standard before init() reaches here (rad_bus.cpp:1243 legacy, :1319
	// Ultimax/C128), so signals() is already populated; a future caller that
	// armed pacing before attachBus would silently get PAL.
	//
	// Derived from the DETECTED standard rather than radMeasureMachineRate():
	// that measurement agreed with the nominal figure to 19ppm, but
	// radWaitForMachineRunning accepts a 33% window, so the accepted sample
	// varies boot to boot and a pacing grid that moves cannot be bisected.
	// Boot logs both so a mis-detection is visible.
	const u64 c64Hz = c64CpuHz( m_C64 ? m_C64->signals().video : VIDEO_PAL );

	// The rate to fall back to while the serial bus is busy -- and tickSettle's
	// phase-lock MODULUS, so this one constant fixes how fast we run AND where
	// PHI2 is predicted to be. At 1.4GHz host: 1368 NTSC, 1420 PAL. The lock a
	// few lines below renews every ~2ms to absorb ~100ppm of crystal drift; a
	// 22700ppm modulus error slid the grid a quarter cycle every 11 C64 cycles,
	// so the lock could not hold a phase at all until this was right.
	m_HostPerEmuQ16Slow = ( hostCyclesPerSecond << 16 ) / c64Hz;

	// Host cycles per emulated cycle, 16.16 fixed point. A selected 1MHz IS the
	// host machine's cycle, so it shares the period above. Turbo keeps its
	// nominal divisor: the card's 20MHz is its own oscillator, not PHI2, and
	// the interpreter never meets that 70.0 budget anyway -- measured ~156 ARM
	// cycles per emulated cycle -- so the pacer never binds there.
	m_HostPerEmuQ16 = ( emulatedHz <= 1000000u )
	                ? m_HostPerEmuQ16Slow
	                : ( ( hostCyclesPerSecond << 16 ) / (u64)emulatedHz );

	// PHI relock cadence, about 2ms. The conversion rate above is NOMINAL --
	// hostCyclesPerSec() is a compile-time constant, not a measurement -- so
	// the Pi and C64 crystals drift apart by up to ~100ppm. At 100ppm the
	// quarter-cycle phase margin erodes in roughly 2.5ms, hence this cadence:
	// the lock is renewed before drift can carry a settled moment across a
	// PHI edge.
	m_PhiRelockHostCycles = hostCyclesPerSecond / 512;

	resyncPacing();
}

void CC64Memory::resyncPacing()
{
	m_PacingDebtCycles = 0;
	m_PacingAnchor = m_C64 ? m_C64->hostCycles() : 0;
	m_LastPhiLockHost = 0;	// stale on purpose: first settle relocks at once
	m_PacerPhaseTrimHost = 0;	// a trim never survives a rate change
}

void CC64Memory::tick( u32 nCycles )
{
	// Virtual entry, kept for callers that only have an ICpuBus. The real work
	// is the inline tickFast() in the header plus this settle path.
	tickFast( nCycles );
}

void CC64Memory::tickPhaseTrim( s32 c64Cycles )
{
	// Armed only at explicit 1MHz -- tickSettle's own lockable condition --
	// where every instruction ends in a settle (m_PacingCheckCycles == 1) and
	// so the very next tickSettle is the one that places this access. In
	// turbo the trim would wait for a settle up to twenty cycles away and
	// displace an unrelated instruction by whole C64 cycles, a throughput
	// cost with no placement meaning, so it is refused there. QUARTER C64
	// cycles at the Q16 slow period (K502: the write lead is a quarter-cycle
	// quantity; VSP_AIM_READ arrives multiplied by four). The grid's own phase
	// class is untouched either way -- the trim never enters the anchor
	// advance, only this one exit moves; magnitude then sign, so no negative
	// value is ever shifted.
	if ( m_SelectedEmulatedHz == 0 || m_SelectedEmulatedHz > 1000000u
	     || !m_C64 || !c64Cycles )
	{
		m_PacerPhaseTrimHost = 0;
		return;
	}
	// K504: a LEAD (negative trim) is honoured only inside the raster window,
	// where the catch-up that follows the settle holds an early entry to its
	// count target. Outside it there is nothing to hold the entry, and a write
	// with a short path could enter the primitive before the falling edge that
	// precedes its cycle and drive a cycle early. K503 applied the lead to every
	// VIC write and a timed animation showed colour cells off.
	if ( c64Cycles < 0 && !rasterCriticalWindowOpen() )
	{
		m_PacerPhaseTrimHost = 0;
		return;
	}
	const u64 mag = ( (u64)( c64Cycles < 0 ? -c64Cycles : c64Cycles )
	                  * m_HostPerEmuQ16Slow ) >> 18;
	m_PacerPhaseTrimHost = c64Cycles < 0 ? -(s64)mag : (s64)mag;
}

// THE EMULATED BA STALL. While the VIC holds BA a real 6510 cannot fetch, so a
// badline FREEZES it for about 43 cycles. Our fetches come out of the
// m_ReadBase shadow and touch no bus, so nothing above the bus layer had any
// BA concept at all: a grep finds BA only in the bus and boot layers, with zero
// hits in w65c816.cpp and this file beyond the VSP event flags -- and those
// CLASSIFY a badline-relevant $D011 write and then charge nothing for it.
//
// WHAT ACTUALLY RESTORES THE DISPLACEMENT, stated precisely so a later reader
// does not look for it in the wrong line. busStealCycles() SPINS on the pin for
// the real duration of the steal. That spin is the fix: real time passes and no
// emulated instruction retires, which is what BA does to a 6510. The two
// assignments below keep the rest of the emulator consistent with that spin --
// they do not create the stall.
//
// WHY THE PIN AND NOT A MODEL. We could derive the stall from our own badline
// bookkeeping; SCPU_VSP_ANCHOR_BADLINE_MOD already computes the raster-&-7
// test in appendVSPTiming(). We do not, for three reasons that are measured
// rather than assumed. The raster anchor is only as good as the last guest
// $D012 read and carries no cycle-in-line phase at all -- noteRasterPrecise()
// ESTIMATES the line start from an earlier logged write. Sprite DMA also pulls
// BA low, three cycles ahead of each s-access, and Mario is full of sprites,
// so a badline-only model would under-stall exactly where the sled lives. And
// a model that disagrees with the real VIC is worse than no model, whereas the
// pin IS the VIC. The published rule is implemented once, in
// CHostBus::busStealCycles, where there is no pin; the firmware never runs it.
// That is how the two are kept in agreement -- the model exists to be tested
// against the rule, never to time the machine.
//
// WHY THE RESUME POINT IS EXACT THOUGH THE ENTRY IS NOT. We sample at an
// instruction boundary, so we can be up to one instruction late noticing that
// BA fell. The spin exits when the VIC RELEASES the bus, which pins the resume
// to the real cycle 55 however late we entered, and the error cannot
// accumulate. That is what makes Mario's invariant fall out of this:
// hold = 59 - DECcycle, because the INC's remaining cycles start at the
// release. The cost is that the exit phase is the RELEASE phase and not the
// phase tickSettle's lock aims for; that is correct (the VIC sets it) but it is
// where a systematic overshoot of 59 - DECcycle would come from if one appears.
//
// 1MHz ONLY, and that is not a performance compromise but the behaviour: in
// turbo the accelerator runs from its own SRAM, and the C64 accesses it does
// make already wait for BA inside the bus primitives. The sibling project's
// 60821c88 is the independent confirmation for the other half -- they hold
// BA/RDY against the accelerator's own LOCAL opcode and shadow reads at 1MHz,
// because Metal Dust's $D012 raster search needs VIC steals to walk the phase
// of its retries, and they recorded relaxing BA for reads as immediate data
// and DOS-output corruption. So: stall more, never less.
//
// THE GATE IMPLIES factor == 1, which is why charging m_EmuCycles is the same
// as charging C64 cycles here, and it is the property the whole charge rests
// on. Callers only reach this when ( lockable || iecActive ): lockable needs
// m_SelectedEmulatedHz <= 1000000, which setPacing() turns into
// m_PacingCheckCycles == 1, and iecActive needs m_IECHoldCycles or
// m_IECPollHoldCycles, which effectiveCIAFactor() answers with a hard 1. There
// is no path into this function with a factor above 1, so c64Now()'s divisor is
// 1 and the freeze is one-for-one. Without that, mode 2 under an IEC throttle
// with turbo still selected would charge 43 emulated cycles that c64Now() read
// as two.
void CC64Memory::baStallSettle( bool iecActive )
{
	// A serial transaction is the one place where a faithful stall is also a
	// hazard: our IEC edges reach a REAL drive through the physical CIA, and
	// the drive has no model of our freeze. m_IECActivityCycles is the flag
	// behind iecBusActive(), whose own comment already says "the CPU must not
	// be stalled mid-transaction whatever the pacing mode", and a JiffyDOS byte
	// is timed reads 10-15us apart, so one 43us freeze would destroy it. Mode 1
	// therefore leaves the K451-identical path in place whenever that window is
	// armed. Mode 2 is the faithful setting -- a real C64 does take badlines
	// during IEC -- and is what an A/B promotes this to once a load has been
	// proved to survive it.
	if ( m_BAStallMode < 2 && ( iecActive || m_IECActivityCycles ) )
	{
		m_BAStallSuppressedIEC++;
		return;
	}

	// m_C64 is non-null: tickFast returns early otherwise, and tickSettle
	// dereferenced it on entry.
	const u32 stolen = m_C64->busStealCycles( m_BAStallMaxCycles );
	if ( !stolen )
		return;

	// FIRST, ahead of the bookkeeping below, because what is wanted is the
	// instant of the RELEASE and everything between here and the read adds to
	// the offset. CRADBus::busStealCycles returns from the sample that read BA
	// high, taken TIMING_BA_SIGNAL_AVAIL (280 ARM of 1368) into the first free
	// cycle, so this stamp is that sample plus one return path.
	//
	// COSTS, because this sits between the release and the guest's next access,
	// which is the very placement under investigation: CRADBus::hostCycles() is
	// one MRS of PMCCNTR_EL0 behind an indirect call, order 10-40 ARM, about
	// 0.02 of a C64 cycle, against the 350-ARM separation the experiment turns
	// on. Once per FREEZE, not per settle -- the zero case returned above -- and
	// this path has just spent tens of microseconds inside the spin. The
	// m_CIA2NMIDeadline compare below is justified the same way.
	const u64 baStallRelease = m_C64->hostCycles();
	// Kept because the assignment below destroys it and the re-entry test at the
	// end of this block needs the PREVIOUS release. A register copy, so nothing
	// lands between the spin's return and the stamp above.
	const u64 baStallPrevRelease = m_BAStallLastReleaseHost;
	m_BAStallLastReleaseHost = baStallRelease;
	m_BAStallLastReleaseLen = stolen > 0xFFFF ? 0xFFFF : (u16)stolen;
	// Badline-class only, by the same >= 20 test the event flag uses: a
	// pure sprite steal cannot exceed 3 + 2*8 = 19 and a badline is 40+,
	// so the classes cannot overlap and this reference cannot be stolen by
	// the sprite steal that follows it a few cycles later.
	if ( stolen >= 20 )
		{
			m_BAStallLastBadlineHost = baStallRelease;
			m_BAStallLastBadlineLen = stolen > 0xFFFF ? 0xFFFF : (u16)stolen;
		}

	// RE-ENTRY SPACING. See m_BAStallB2B for why this is the only test that can
	// decide whether this function's arithmetic is capable of the +3..+4 hold
	// error at all, and for why there are two windows rather than one.
	//
	// The entry is RECONSTRUCTED, not timestamped, and busStealCycles()'s own
	// increment-order note is what licenses it: `stolen` is whole C64 cycles
	// counted from OUR entry to the release, so entry = release - stolen
	// periods. No second PMCCNTR read goes on the one path whose separation from
	// the guest's next access is under measurement. The reconstruction puts the
	// entry up to one cycle EARLY, because we entered partway through the first
	// stolen cycle and counted it whole -- hence the -1 floor and not 0.
	//
	// One signed divide rather than three multiplies, so the thresholds read in
	// C64 cycles and cannot drift from the comment. SDIV is about 20 ARM on an
	// A53, on a path that has just spent tens of microseconds inside the spin.
	{
		const u64 perC64 = m_HostPerEmuQ16Slow >> 16;
		if ( baStallPrevRelease && perC64 )
		{
			const s64 gap = (s64)( baStallRelease - (u64)stolen * perC64 )
			              - (s64)baStallPrevRelease;
			const s64 gapCycles = gap / (s64)perC64;
			if ( gapCycles >= -1 && gapCycles <= 8 )
			{
				m_BAStallB2B++;
				m_BAStallB2BCycles += stolen;
				if ( gapCycles <= 2 )
					m_BAStallB2BTight++;
				if ( stolen <= 5 )
					m_BAStallB2BShort++;
			}
		}
	}
	// The distribution, because the mean of this population describes no event
	// that ever happened -- see m_BAStallHist. Index arithmetic rather than a
	// loop: 5-cycle bins put a sprite steal in bin 0, eight sprites' 3+2N=19 in
	// bin 3 and a badline's 43 in bin 8.
	m_BAStallHist[ stolen < 75 ? stolen / 5 : 15 ]++;

	m_BAStallCycles += stolen;
	m_BAStallEvents++;

	// The guest's clock, in C64 cycles -- which is the emulated cycle here, see
	// the factor argument above. Everything that measures time by counting
	// follows from this one add: c64Now(), the VIC log stamps, ciaNow() and the
	// CIA2 deadlines. NOTE for whoever reads the pacer diagnostic: slow-cycles
	// is incremented in tickFast from the instruction's own cycles and does NOT
	// include these, so the two populations differ by m_BAStallCycles.
	m_EmuCycles += stolen;

	// The real time those cycles took has ALREADY been spent, inside the spin,
	// so the pacer must not bill it again: advance the grid origin, not the
	// debt. Without this the next settle finds itself `stolen` cycles behind
	// and spends a probe or a credit on time that was never owed -- which is
	// what bastall_hands_the_pacer_the_time_the_spin_already_spent pins. The
	// amount uses the same ( cycles * rate ) >> 16 form as owed, and the 1MHz
	// rate EXPLICITLY -- m_HostPerEmuQ16 is still the turbo period when it was
	// the IEC throttle that made us slow. Whole C64 cycles at the Q16 period
	// preserve the phase class the lock installed, which is the property the
	// clamp arm's note insists on. m_PacingDebtCycles is deliberately untouched.
	// Residual: we entered the steal mid-cycle, so this can be up to one cycle
	// out either way, which the signed compare and the next spin absorb.
	m_PacingAnchor += ( (u64)stolen * m_HostPerEmuQ16Slow ) >> 16;

	// A timer deadline can mature inside the freeze. refreshInterruptsIfDue()
	// re-checks this before the next instruction anyway, so this is belt and
	// braces rather than the mechanism; it costs one compare against an
	// almost-always-zero u64 on a path that just spent microseconds, and
	// ciaTimerNMIDue() is guarded by the same deadline so a double call is a
	// no-op.
	if ( m_CIA2NMIDeadline && ciaNow() >= m_CIA2NMIDeadline )
		ciaTimerNMIDue();

	// Force the post-release interrupt sample. HONEST SCOPE, because the first
	// draft of this comment claimed this line fixed the hang and it does not:
	// at an effective 1MHz refreshInterruptsIfDue()'s threshold is already 1
	// and tickFast has just credited this instruction's cycles, so the next
	// instruction re-samples anyway. This line changes NOTHING today. What puts
	// Mario's second IRQ back into the $082E-$0845 sled is the SPIN above. It
	// is kept so the stall stays correct if that sampling quantum ever stops
	// being one cycle at 1MHz, which is a coupling no comment can enforce.
	m_IntCredit = 0xFFFFF;
}

void CC64Memory::tickSettle( bool iecActive )
{
	u64 now = m_C64->hostCycles();
	// VIDEO_MODE 1 keepalive. Mode 0's physical DRAM mirroring naturally keeps
	// the bus busy; exclusive HDMI removes that traffic and exposes unreliable
	// cold first transfers. Keep the bus warm with one harmless same-polarity
	// write every 90 REAL microseconds, comfortably below CRADBus's 150us
	// per-access backstop. Host time is load-independent; the former emulated-
	// cycle deadline ran late precisely when HDMI reduced the achieved CPU rate.
	//
	// Suspend only for evidence of a real IEC transaction. The poll-only speed
	// hold is deliberately excluded: a poll without line activity supplies no
	// physical bus warmth, so suppressing here would recreate the cold gap.
	const bool realIEC = m_IECHoldCycles != 0 || m_IECActivityCycles != 0;
	if ( m_BusKeepalive && !realIEC && m_C64
	     && ( m_NextKeepalive == 0 || (s64)( now - m_NextKeepalive ) >= 0 ) )
	{
		m_C64->write( 0x02FE, 0xA6 );
		now = m_C64->hostCycles();
		const u64 hostRate = m_C64->hostCyclesPerSec();
		m_NextKeepalive = now + ( hostRate ? hostRate * 90 / 1000000 : 1 );
	}

	// While the serial bus is active, run at 1MHz whatever speed is selected --
	// the KERNAL's bit timing depends on it. See setIECThrottle().
	const u64 rate = iecActive ? m_HostPerEmuQ16Slow : m_HostPerEmuQ16;

	// How much real time those emulated cycles were supposed to take.
	const u64 owed = ( m_PacingDebtCycles * rate ) >> 16;

	// THE PHYSICAL PHASE TRIM, one-shot. A placed VIC access may have asked for
	// this settle's deadline to move by whole C64 cycles (VSP_AIM_READ /
	// VSP_AIM_WRITE, via tickPhaseTrim). It moves ONLY the instant this settle
	// exits: owed, the debt, m_EmuCycles and the anchor advance never see it,
	// so the guest's ledger stays exactly its own cycles and the grid keeps its
	// origin. The next settle meets the displacement as ordinary lag and the
	// credit arm carries it like any other bus time.
	//
	// EXACT IN BOTH ARMS (K498): one unit is one C64 cycle, measured from the
	// exit this settle would otherwise have had. On time, the spin at the
	// bottom exits at anchor + owed + trim. Late, the late arm re-anchors
	// exactly as it always has and THEN waits the trim out from that instant.
	// Before K498 the late arm had no spin at all, so an access whose settle
	// arrived late by L lost the first L cycles of its trim. Only VSP_AIM_READ
	// arrives here now: the write aim is applied at the catch-up launch in
	// write8 (a settle-level trim on a write landed before that catch-up and
	// its slack swallowed it). A negative trim only shortens the spin and never
	// widens the late arm, so the credit arithmetic cannot underflow.
	// Consumed or dropped on EVERY entry, so it can never attach to a later,
	// unrelated settle; under an IEC hold (unlocked pacing) it is dropped.
	s64 due = (s64)owed;		// the instant this settle may exit
	const s64 late = (s64)owed;	// past this there is nothing to wait for
	s64 trimHost = 0;
	if ( m_PacerPhaseTrimHost )
	{
		if ( !iecActive ) trimHost = m_PacerPhaseTrimHost;
		m_PacerPhaseTrimHost = 0;
	}
	due += trimHost;

	// --- PHI phase maintenance, explicit 1MHz only -------------------------
	// Cycle-exact guest code (VSP, FLI, any stabilised raster handler) needs
	// two properties from this pacer that plain rate-keeping does not give:
	//
	//   EXACTNESS   the spin below must exit on an absolute grid, so the
	//               overshoot of one settle is absorbed by the next instead of
	//               accumulating -- hence "anchor += owed", never "= now";
	//   PHASE       that grid must sit at a KNOWN offset inside the physical
	//               PHI cycle. The bus primitive lands an access in cycle N
	//               exactly when it is entered during the CPU half of N-1, a
	//               fixed half-microsecond window. An unlocked anchor puts the
	//               grid at a random phase, re-rolled at every re-anchor, and
	//               when it straddles an edge the same instruction lands in
	//               adjacent cycles on different frames. That is precisely the
	//               VSP failure mode.
	//
	// The bus primitive's half-cycle waits capture an arrival anywhere inside
	// one full PHI cycle into the same target slot; the discontinuity sits at
	// the PHI FALLING edge. The lock therefore aims settle exits a quarter
	// cycle BEFORE a CPU-half start: after the few hundred nanoseconds of
	// call-path between the settle and the primitive's first level read, the
	// read lands centred in that window with maximum margin both ways. IEC
	// keeps its long-validated unlocked behaviour untouched.
	const bool lockable = !iecActive
	                   && m_SelectedEmulatedHz != 0
	                   && m_SelectedEmulatedHz <= 1000000u;
	const u64 perC64 = m_HostPerEmuQ16Slow >> 16;
	if ( lockable && perC64
	     && ( now - m_LastPhiLockHost ) > m_PhiRelockHostCycles
	     && (s64)( ( m_PacingAnchor + owed ) - now ) > (s64)( 3 * perC64 ) )
	{
		// Renew only when the pending spin leaves comfortable slack: the edge
		// measurement below can busy-wait up to two C64 cycles, and a lock
		// must never be the thing that makes us late.
		const u64 edge = m_C64->phiCpuHalfStartHostCycles();
		now = m_C64->hostCycles();
		m_LastPhiLockHost = now;
		if ( edge )
		{
			// Nudge the anchor's phase class onto (edge - quarter cycle) by
			// the smallest signed amount. Sub-cycle by construction, so the
			// guest sees at most one cycle stretched by half a microsecond --
			// invisible at 1MHz, and it happens between instructions.
			const u64 want = ( edge + 3 * perC64 / 4 ) % perC64;
			const u64 have = m_PacingAnchor % perC64;
			s64 adj = (s64)want - (s64)have;
			if ( adj > (s64)( perC64 / 2 ) )       adj -= (s64)perC64;
			else if ( adj < -(s64)( perC64 / 2 ) ) adj += (s64)perC64;
			m_PacingAnchor = (u64)( (s64)m_PacingAnchor + adj );
		}
	}

	// One reading in the common case. hostCycles() is a virtual call wrapping a
	// system-register read, and this runs after every instruction -- the
	// previous version called it four times, which at 20MHz was a serious
	// fraction of the whole per-instruction budget.
	//
	// Signed comparison: a freshly phase-snapped anchor may sit slightly in
	// the future, which an unsigned difference would misread as "behind by
	// half an epoch".
	if ( (s64)( now - m_PacingAnchor ) >= late )
	{
		// Behind, or exactly on time. Settle up and carry on rather than
		// accumulating a debt that can never be repaid: a burst of I/O easily
		// overruns its own budget, and clawing that back later would just make
		// the following stretch run too slowly.
		//
		// The phase the lock installed lives in the OLD anchor, and the cheap
		// recovery below carries it across this re-anchor without touching the
		// bus, so keep it before it is overwritten. Free: the compiler shares
		// it with the difference the branch predicate above already computed.
		const u64 wasAnchor = m_PacingAnchor;
		m_PacingAnchor = now;
		m_PacingDebtCycles = 0;
		if ( lockable && perC64 )
		{
			// K448 measured this branch instead of assuming it: 388991 entries
			// over 15865599 slow cycles adding 657379314 host cycles = 1690 ARM
			// = 1.23 C64 cycles EACH, which is precisely the bimodal ~12.0 mode
			// that appeared beside the ~11.0 $D011 hold. The trace's two modes
			// are 1350 ARM apart -- one whole PHI cycle, not 1690 -- because the
			// bus primitive quantises placement to an edge, so pushing the
			// anchor past a cycle boundary displaces the access by exactly one.
			//
			// That 1690 separates into two parts and the larger one is the snap:
			// the edge probe waits out the rest of the current PHI half and then
			// a whole one, averaging half a cycle (684 ARM), while the snap that
			// follows costs about 1026 ARM EVERY time rather than on average --
			// the probe returns AT the edge and the aim is edge + 3/4 cycle, so
			// "snap forward onto the wanted class" is always three quarters of a
			// cycle. 684 + 1026 = 1710 predicted against 1690 measured.
			//
			// A late arrival is not evidence that the phase moved; it is evidence
			// that the work did not fit. So the probe runs on a CADENCE and the
			// phase is carried arithmetically in between.
			m_PacerLateEntries++;
			const u64 lateArrival = now;
			// A quarter of the lock cadence, not the whole of it. The anchor
			// walks off physical PHI by about 1 ARM per settle -- 0.5 from owed's
			// truncation to whole host cycles and 0.48 from the <=100ppm crystal
			// error in the nominal rate -- and settles are PER INSTRUCTION at
			// explicit 1MHz (fineTicksRequired), about 292 of them per
			// millisecond, so the quarter-cycle aim margin of 342 ARM is spent in
			// roughly 1.2ms. 0.49ms keeps 2.4x headroom on that and still spends
			// a probe once per ~500 C64 cycles instead of once per late arrival.
			// INFERRED, not measured: there is no instrument for phase error.
			if ( ( now - m_LastPhiLockHost ) > m_PhiRelockHostCycles / 4 )
			{
				// A re-anchor is exactly the moment the phase lottery used to
				// be re-rolled; roll it deliberately instead. Snapping FORWARD
				// onto the wanted phase class keeps the anchor <= one cycle
				// ahead -- a backward snap after a PROBE would need a second
				// probe to find out where it had landed.
				m_PacerLateProbes++;
				const u64 edge = m_C64->phiCpuHalfStartHostCycles();
				const u64 after = m_C64->hostCycles();
				m_LastPhiLockHost = after;
				m_PacingAnchor = after;
				if ( edge )
				{
					const u64 want = ( edge + 3 * perC64 / 4 ) % perC64;
					const u64 have = after % perC64;
					m_PacingAnchor = after + ( ( want >= have )
					                 ? ( want - have )
					                 : ( perC64 - ( have - want ) ) );
				}
				// Billed on probing entries only, so this stays "what a probe
				// costs": divide it by late-probes, never by late-entries.
				m_PacerLateSnapHostCycles += m_PacingAnchor - lateArrival;
			}
			else if ( ( now - wasAnchor ) - owed
			          <= PACER_CREDIT_CYCLES * perC64 )
			{
				// THE CREDIT, and the whole of the VSP fix. The lag below is
				// elapsed REAL time the guest was never charged for, and it is
				// almost entirely the physical access, because nothing else inside
				// a settle-to-settle window costs microseconds. It needs no new
				// instrument: the PMCCNTR the pacer has already read measured it.
				// Advancing the anchor by exactly owed hands that lag to the next
				// deadline, which is then already due, so the spin at the bottom of
				// this function has nothing left to wait out and the $D011 interval
				// collapses from 6 + bus toward max( 6, bus ).
				//
				// SCHEDULING, not charging -- CODEX's binding constraint.
				// m_EmuCycles and m_PacingDebtCycles are untouched: the guest is
				// charged exactly the cycles the 65816 took. Only the real-time grid
				// origin moves.
				//
				// WHAT THE 1MHz GUARANTEE NOW IS, stated exactly, because the loose
				// version of it is false. Every arm advances the anchor by at least
				// owed, and every arm exits with now >= anchor except the probe arm,
				// which may leave it up to one C64 cycle ahead by construction. So
				// elapsed real time since resyncPacing() is >= sum( owed ) - perC64
				// at every settle boundary: CUMULATIVELY the guest can never get
				// more than one cycle ahead of the 1MHz grid. It is NOT
				// instantaneously 1MHz, and this arm is why -- until the carried lag
				// is worked off, up to PACER_CREDIT_CYCLES of guest time runs with
				// the spin skipped, at interpreter speed. That burst is the price of
				// putting the guest back where a real 1MHz machine would already be,
				// and the cap is what bounds it.
				//
				// AND THE HAZARD, the first thing to look at if VSP gets WORSE
				// rather than better: a credited settle's deadline is already past,
				// so the spin does not run and the exit is NOT on the locked phase.
				// The grid's phase class survives in the anchor -- wasAnchor + owed
				// adds only owed's sub-ARM truncation, the same error the on-time
				// path at the bottom of this function already accepts -- but the
				// EXIT lands wherever the interpreter happened to finish. Through
				// back-to-back I/O that is every instruction, which is the "random
				// phase, re-rolled at every re-anchor" this function's own header
				// calls the VSP failure mode. The hold should shrink; read the
				// trace's grid= column and the SPREAD, not just the mean, to see
				// what it cost.
				m_PacerCreditEntries++;
				m_PacerCreditHostCycles += ( now - wasAnchor ) - owed;
				m_PacingAnchor = wasAnchor + owed;
			}
			else
			{
				// Past the credit cap: a0215f5's anti-windup clamp doing its
				// original job, forgiving a lag that a genuinely slower-than-1MHz
				// stretch could never repay.
				m_PacerClampEntries++;
				// No bus observation and no rounding up: move the anchor to the
				// nearest host cycle that still holds the phase the last probe
				// installed. The modulus is the Q16 period and NOT perC64:
				// truncating 1368.889 to 1368 would walk the carried phase off
				// real PHI by 0.889 ARM for every cycle it is carried across,
				// which is the one error this path could introduce that the
				// probe it replaces could not. CODEX raised exactly that in CHT
				// ("do not modulo absolute PMU stamps by an integer-truncated
				// bus period and assume phase lock"); the probe arms above keep
				// the truncated modulus safely only because each one installs a
				// freshly measured absolute edge, so its error cannot compound.
				//
				// The branch predicate guarantees now - wasAnchor >= owed >= 0,
				// and resyncPacing() leaves m_LastPhiLockHost at 0 so the first
				// entry after any rate change takes the probe arm -- this path
				// can never carry a phase no probe installed. The shift cannot
				// overflow: the anchor is re-set on every settle in each arm,
				// so it would need a 2^48-cycle gap, 56 hours. The error
				// accepted is at most half a cycle and symmetric about zero,
				// where rounding forward was +0.75 of a cycle on every one of
				// those 388991 entries; the signed compare above already
				// tolerates an anchor that sits slightly in the future.
				const u64 slip = ( ( now - wasAnchor ) << 16 )
				               % m_HostPerEmuQ16Slow;
				m_PacingAnchor = ( 2 * slip <= m_HostPerEmuQ16Slow )
				               ? ( now - ( slip >> 16 ) )
				               : ( now + ( ( m_HostPerEmuQ16Slow - slip ) >> 16 ) );
			}
		}
		// THE TRIM ON A LATE ARRIVAL (K498). The arms above have chosen the
		// anchor; the exit they would have had is now. A positive trim is
		// owed from exactly that instant, so it is waited out here, after the
		// re-anchor and before the stall sample, and the next settle meets it
		// as ordinary lag like any other bus time. Only a placed VIC access
		// with a non-zero VSP_AIM_* ever reaches this: one extra PMCCNTR read
		// on that path and nothing on any other.
		if ( trimHost > 0 )
		{
			const u64 from = m_C64->hostCycles();
			do {
				now = m_C64->hostCycles();
			} while ( (s64)( now - from ) < trimHost );
			m_PacerWaitHostCycles += now - from;
		}
		// The stall is sampled on BOTH exits of this function and nowhere
		// else. This arm has no pacing spin -- real time is already past the
		// emulated boundary -- so sampling here is sampling now, which is
		// what the pin answers anyway. Two one-line call sites rather than
		// re-bracing the pacer: nothing about WHEN the arms above run may
		// change. lockable || iecActive is exactly "effectively 1MHz", and
		// GCC tail-merges this site with the one at the end of the function
		// into a single `b` to baStallSettle.
		if ( ( lockable || iecActive ) && m_BAStallMode )
			baStallSettle( iecActive );
		return;
	}

	// Ahead of schedule: wait out the difference. This is what makes a cycle
	// count mean a duration.
	const u64 waitStart = now;
	do {
		now = m_C64->hostCycles();
	} while ( (s64)( now - m_PacingAnchor ) < due );
	m_PacerWaitHostCycles += now - waitStart;

	// Advance by exactly what was owed, NOT to "now": the spin exits a few
	// tens of nanoseconds late, and re-anchoring to the exit time would let
	// that overshoot accumulate one settle at a time -- a slow forward slide
	// of the whole grid that is invisible everywhere except cycle-exact
	// raster code, where it is fatal. On the absolute grid the next spin
	// simply absorbs this one's overshoot.
	m_PacingAnchor += owed;
	m_PacingDebtCycles = 0;

	// AFTER the spin, deliberately. The spin is what makes the emulated
	// boundary and real time coincide, so this is the only point in a settle
	// where "is the VIC holding the bus right now" is a question about the
	// guest's actual position. Asking before it would read the pin up to one
	// instruction early AND would leave this instruction's own owed time to be
	// waited out after the release, pushing the first post-release access 2-6
	// cycles late -- straight into the hold this is meant to reproduce.
	if ( ( lockable || iecActive ) && m_BAStallMode )
		baStallSettle( iecActive );
}
