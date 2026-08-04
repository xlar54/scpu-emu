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
	  m_IECThrottleEnabled( true ), m_IECHoldCycles( 0 ),
	  m_LastCIA2PortA( 0 ), m_HaveCIA2PortA( false ),
	  m_TimingHook( 0 ), m_TimingHookCtx( 0 )
{
	c64BankingInit();

	for ( u32 i = 0; i < C64_RAM_SIZE; i++ )      m_RAM[ i ] = 0;
	for ( u32 i = 0; i < C64_BASIC_SIZE; i++ )    m_Basic[ i ] = 0xFF;
	for ( u32 i = 0; i < C64_KERNAL_SIZE; i++ )   m_Kernal[ i ] = 0xFF;
	for ( u32 i = 0; i < C64_CHARROM_SIZE; i++ )  m_CharROM[ i ] = 0;

	updateBankMode();
}

void CC64Memory::reset()
{
	m_Port00 = 0x2F;
	m_Port01 = 0x37;
	m_IOReads = m_IOWrites = m_RamWrites = 0;
	m_PacerWaitHostCycles = m_SlowPacedCycles = m_IECThrottledCycles = 0;
	m_IECHoldCycles = 0;
	m_HaveCIA2PortA = false;
	m_IntCredit = 0xFFFFF;
	m_CachedIRQ = m_CachedNMI = false;

	for ( u32 i = 0; i < C64_RAM_SIZE; i++ )
		m_RAM[ i ] = 0;

	updateBankMode();
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

	// Bootmap sits ABOVE the PLA: while it is active the $01 banking bits do
	// not matter for these ranges. I/O is deliberately left alone -- the boot
	// code needs the VIC-II and the CIAs, and $D000-$DFFF stays I/O in VICE's
	// table at every configuration this emulator can reach.
	if ( m_BootmapActive
	     && ( ( a >= 0x8000 && a <= 0xCFFF ) || a >= 0xE000 ) )
		return m_BootmapROM[ a ];

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
		return m_CharROM[ a - 0xD000 ];

	case REG_IO:
		{
			// Registers that live inside the cartridge never reach the C64.
			u8 v;
			if ( m_IO && m_IO->ioRead( a, v ) )
				return v;

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
			v = m_C64 ? m_C64->read( a ) : 0xFF;
			if ( a == 0xDD00 )
			{
				m_LastCIA2PortA = v;
				m_HaveCIA2PortA = true;
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
		// read $00/$01 back through the VIC's eyes in a few places.
		m_RAM[ a ] = value;
		if ( m_Mirror ) m_Mirror->onRamWrite( a, value );
		return;
	}

	if ( c64WriteIsIO( a, m_BankMode ) )
	{
		if ( m_IO && m_IO->ioWrite( a, value ) )
			return;

		// $DD00 is CIA2 port A, which carries ATN, CLK and DATA for the serial
		// bus as well as the VIC bank select. Only IEC-line changes arm the
		// fallback; an ordinary VIC bank change must remain at turbo speed.
		if ( a == 0xDD00 )
		{
			// Bits 0-1 select the VIC bank; bits 3-5 drive the IEC lines. A VIC
			// bank change must not impose a 100ms serial-bus slowdown. A preceding
			// read supplies the usual baseline; otherwise the first write does.
			const bool iecChanged = m_HaveCIA2PortA
			                     && ( ( m_LastCIA2PortA ^ value ) & 0x38 ) != 0;
			m_LastCIA2PortA = value;
			m_HaveCIA2PortA = true;

			// CMD's KERNAL explicitly uses $D07A/$D07B. The automatic hold is a
			// fallback for code touching IEC while still nominally in fast mode;
			// it must not leave a slow tail after explicit control restores turbo.
			const bool explicitlySlow = m_SelectedEmulatedHz != 0
			                         && m_SelectedEmulatedHz <= 1000000u;
			if ( m_IECThrottleEnabled && iecChanged && !explicitlySlow )
			{
				const bool wasInactive = m_IECHoldCycles == 0;
				if ( wasInactive )
					m_IECThrottleEvents++;
				m_IECHoldCycles = 100000;
				if ( wasInactive && m_TimingHook )
					m_TimingHook( m_TimingHookCtx );
			}
		}

		// Mirroring is normally asynchronous -- see the note in read8() -- which
		// means a buffered RAM write and an I/O write can reach the C64 out of
		// program order. That is fine almost everywhere, because the VIC-II
		// cannot tell when a byte of screen memory arrived.
		//
		// It is NOT fine for the four registers that change what the VIC-II
		// LOOKS AT. Enabling bitmap mode, or repointing the screen or character
		// base, makes the chip start displaying memory that our pending writes
		// have not reached yet -- so the machine shows the previous contents of
		// the new location, and the intended picture appears only later, when
		// the buffer happens to drain. A program that draws a screen and then
		// switches to it sees the two steps in the wrong order.
		//
		//   $D011  control 1  -- bitmap mode, display enable, Y scroll
		//   $D016  control 2  -- multicolour, X scroll
		//   $D018  memory pointers -- screen base, character/bitmap base
		//   $DD00  CIA2 port A -- VIC bank select
		//
		// Flushing before just these restores program order where it is
		// visible. They are rare -- a handful per mode change -- unlike the
		// constant $D012/$DC01 traffic that made flushing before EVERY I/O
		// write corrupt the display.
		if ( m_Mirror && ( a == 0xD011 || a == 0xD016 || a == 0xD018 || a == 0xDD00 ) )
			m_Mirror->flush();

		m_IOWrites++;
		if ( m_C64 ) m_C64->write( a, value );
		return;
	}

	// Everything else falls through to DRAM, including writes "into" ROM.
	m_RAM[ a ] = value;
	m_RamWrites++;

	if ( m_Mirror )
		m_Mirror->onRamWrite( a, value );
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
	if ( m_C64 )
		m_PacingAnchor = m_C64->hostCycles();
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
