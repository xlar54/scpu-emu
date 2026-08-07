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
	  m_Port00( 0x2F ), m_Port01( 0x37 ), m_BankMode( 0 ),
	  m_IOReads( 0 ), m_IOWrites( 0 ), m_RamWrites( 0 ),
	  m_IECThrottleEvents( 0 ),
	  m_PacingDebtCycles( 0 ), m_PacingCheckCycles( 1 ),
	  m_PacerWaitHostCycles( 0 ), m_SlowPacedCycles( 0 ), m_IECThrottledCycles( 0 ),
	  m_C64( 0 ), m_Mirror( 0 ), m_IO( 0 ), m_BootmapROM( 0 ), m_ROMShadow( 0 ),
	  m_HasBasic( false ), m_HasKernal( false ), m_HasChar( false ),
	  m_HostPerEmuQ16( 0 ), m_HostPerEmuQ16Slow( 0 ), m_PacingAnchor( 0 ),
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
	if ( !m_IECThrottleEnabled || explicitlySlow )
		return;
	if ( m_IECPollHoldCycles < 1000 )
	{
		const bool wasInactive = ( m_IECPollHoldCycles == 0
		                        && m_IECHoldCycles == 0 );
		m_IECPollHoldCycles = 1000;
		if ( wasInactive )
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
	// Forbidden span 1: the active and most-recent screen matrices, as blocks.
	// Double-buffered programs can queue pointer-row bytes for one while the
	// other is active; stealing either matrix makes the next flip destructive.
	const u32 screenBase = ( m_SpritePtrBase - 0x3F8 ) & 0xFFFF;
	const u32 previousScreenBase = ( m_PreviousSpritePtrBase - 0x3F8 ) & 0xFFFF;
	// Forbidden span 2: a bitmap in the low half of bank 3. $D011 bit 5 turns
	// bitmap mode on; $D018 bit 3 picks the half. 8K starting at $C000 covers
	// the whole window -- in that configuration there is nothing to allocate.
	const bool lowBitmap = ( m_VICRegShadow[ 0x11 ] & 0x20 )
	                    && !( m_LastD018 & 0x08 );

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

void CC64Memory::reset()
{
	m_Port00 = 0x2F;
	m_Port01 = 0x37;
	m_IOReads = m_IOWrites = m_RamWrites = 0;
	m_IECThrottleEvents = 0;
	m_PacingDebtCycles = 0;
	m_PacerWaitHostCycles = m_SlowPacedCycles = m_IECThrottledCycles = 0;
	m_IECHoldCycles = 0;
	m_IECActivityCycles = 0;
	m_IECPollHoldCycles = 0;
	m_MaxTickChunk = 0;
	m_CIA2PortALatch = 0;
	m_LastCIA2PortARead = 0;
	m_LastD018 = 0x14;			// the KERNAL default: screen $0400, charset $1000
	m_SpritePtrBase = m_PreviousSpritePtrBase = 0xFFFFFFFF;
	// Relocation tables before updateHotShapeBlocks, which consults them.
	// Cleared on reset: the game that allocated them is gone, and stale
	// mappings would translate a NEW program's pointers to blocks holding the
	// OLD program's shapes.
	for ( u32 i = 0; i < 0x40; i++ ) m_PtrReloc[ i ] = 0xFF;
	for ( u32 i = 0; i < 0x30; i++ ) m_RelocInUse[ i ] = 0xFF;
	m_RelocCount = 0;
	updateSpritePtrBase();
	updateHotShapeBlocks();
	for ( u32 i = 0; i < 0x40; i++ ) m_VICRegShadow[ i ] = 0;
	m_CIA2DDRA = 0;
	m_HaveCIA2PortALatch = false;
	m_HaveCIA2PortARead = false;
	m_HaveCIA2DDRA = false;
	m_IOLogPos = 0;
	m_CIALogPos = 0;
	m_CIALastRead = 0xFFFFFFFF;
	m_EmuCycles = 0;
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
	m_MirrorBufferFreeAt = 0;
	m_PendingFastHalfCycles = 0;
	m_FastHalfCarry = 0;
	m_CIA2TADeadline = m_CIA2TBDeadline = m_CIA2NMIDeadline = 0;
	m_CIA2FactorInUse = effectiveCIAFactor();
	resyncPacing();

	for ( u32 i = 0; i < C64_RAM_SIZE; i++ )
		m_RAM[ i ] = 0;

	updateBankMode();
}

void CC64Memory::noteIECActivity()
{
	const bool wasQuiet = ( m_IECActivityCycles == 0 );
	m_IECActivityCycles = 50000;		// 50ms while the serial path runs at 1MHz
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
			v = m_C64 ? m_C64->read( a ) : 0xFF;
			m_IOLog[ m_IOLogPos++ & 63 ] = (u32)a | ( (u32)v << 16 );
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
				const bool inputAsserted = ( v & inputMask ) != inputMask;
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
				if ( inputChanged || inputAsserted )
					noteIECActivity();
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
			else if ( a == 0xDD02 )
			{
				// A read establishes the real direction baseline; it does not
				// itself represent a direction change on the pins.
				m_CIA2DDRA = v;
				m_HaveCIA2DDRA = true;
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
		if ( m_Mirror && m_Mirror->onRamWrite( a, value ) )
			noteMirrorWrite();
		m_RAM[ a ] = value;
		return;
	}

	if ( dosExtensionMapsBank1( a ) )
	{
		m_ROMShadow[ a ] = value;
		return;
	}

	if ( c64WriteIsIO( a, m_BankMode ) )
	{
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
		if ( a == 0xDD00 )
		{
			if ( m_HaveCIA2DDRA && m_HaveCIA2PortALatch )
				cia2IECChanged = cia2EffectiveIECDrive( m_CIA2PortALatch, m_CIA2DDRA )
				                 != cia2EffectiveIECDrive( value, m_CIA2DDRA );
			else
				cia2IECChanged = true;
		}
		else if ( a == 0xDD02 )
		{
			if ( m_HaveCIA2DDRA && m_HaveCIA2PortALatch )
				cia2IECChanged = cia2EffectiveIECDrive( m_CIA2PortALatch, m_CIA2DDRA )
				                 != cia2EffectiveIECDrive( m_CIA2PortALatch, value );
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
			m_CIA2PortALatch = value;
			m_HaveCIA2PortALatch = true;
			updateSpritePtrBase();		// VIC bank lives in bits 0-1
			updateHotShapeBlocks();
			if ( cia2IECChanged )
				noteIECActivity();
		}
		else if ( a == 0xD018 )
		{
			m_LastD018 = value;
			updateSpritePtrBase();
			updateHotShapeBlocks();
		}
		else if ( a == 0xDD02 )
		{
			m_CIA2DDRA = value;
			m_HaveCIA2DDRA = true;
			if ( cia2IECChanged )
				noteIECActivity();
		}
		else if ( a >= 0xDD04 && a <= 0xDD0F )
		{
			// CIA2 timer and interrupt-mask writes feed the NMI retiming
			// model on their way to the real chip.
			cia2TimerWrite( a, value );
		}

		m_IOWrites++;
		if ( a < 0xD040 )
			m_VICRegShadow[ a & 0x3F ] = value;
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
		if ( m_C64 ) m_C64->write( a, value );
		return;
	}

	// Everything else falls through to DRAM, including writes "into" ROM.
	// Sink before store -- see IMirrorSink::onRamWrite.
	{
		const u8 old = m_RAM[ a ];
		m_RamWrites++;
		if ( m_Mirror && m_Mirror->onRamWrite( a, value ) )
			noteMirrorWrite();
		m_RAM[ a ] = value;

		// Active-screen sprite pointers go to the real bus immediately; see
		// the note in writeFast, which is the path that usually takes them.
		// Delivered through the same under-I/O relocation as writeFast.
		if ( old != value && ( (u32)a & ~7u ) == m_SpritePtrBase )
		{
			if ( m_C64 ) m_C64->write( a, relocPointerValue( value ) );
			updateHotShapeBlocks();
		}
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

	// Host cycles per emulated cycle, 16.16 fixed point. At 1.4GHz host and
	// 1MHz emulated that is 1400.0; at 20MHz it is 70.0.
	m_HostPerEmuQ16 = ( hostCyclesPerSecond << 16 ) / (u64)emulatedHz;

	// The rate to fall back to while the serial bus is busy.
	m_HostPerEmuQ16Slow = ( hostCyclesPerSecond << 16 ) / 1000000ULL;

	resyncPacing();
}

void CC64Memory::resyncPacing()
{
	m_PacingDebtCycles = 0;
	m_PacingAnchor = m_C64 ? m_C64->hostCycles() : 0;
}

void CC64Memory::tick( u32 nCycles )
{
	// Virtual entry, kept for callers that only have an ICpuBus. The real work
	// is the inline tickFast() in the header plus this settle path.
	tickFast( nCycles );
}

void CC64Memory::tickSettle( bool iecActive )
{
	// While the serial bus is active, run at 1MHz whatever speed is selected --
	// the KERNAL's bit timing depends on it. See setIECThrottle().
	const u64 rate = iecActive ? m_HostPerEmuQ16Slow : m_HostPerEmuQ16;

	// How much real time those emulated cycles were supposed to take.
	const u64 owed = ( m_PacingDebtCycles * rate ) >> 16;

	// One reading in the common case. hostCycles() is a virtual call wrapping a
	// system-register read, and this runs after every instruction -- the
	// previous version called it four times, which at 20MHz was a serious
	// fraction of the whole per-instruction budget.
	u64 now = m_C64->hostCycles();

	if ( ( now - m_PacingAnchor ) >= owed )
	{
		// Behind, or exactly on time. Settle up and carry on rather than
		// accumulating a debt that can never be repaid: a burst of I/O easily
		// overruns its own budget, and clawing that back later would just make
		// the following stretch run too slowly.
		m_PacingAnchor = now;
		m_PacingDebtCycles = 0;
		return;
	}

	// Ahead of schedule: wait out the difference. This is what makes a cycle
	// count mean a duration.
	const u64 waitStart = now;
	do {
		now = m_C64->hostCycles();
	} while ( ( now - m_PacingAnchor ) < owed );
	m_PacerWaitHostCycles += now - waitStart;

	m_PacingAnchor = now;
	m_PacingDebtCycles = 0;
}
