/*
   SCPU-EMU - CMD SuperCPU emulation for the C64/C128 using a RAD Expansion Unit
   Copyright (c) 2026 SCPU-EMU contributors

   CREU - the 8726 expansion controller. See reu.h for what this is and what it
   deliberately does not do.

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
#include "reu.h"

#include <stdlib.h>
#include <string.h>

// --- command register bits --------------------------------------------------
#define REU_CMD_EXECUTE     0x80
#define REU_CMD_AUTOLOAD    0x20
#define REU_CMD_NO_FF00     0x10	// set: run now. clear: wait for $FF00.
#define REU_CMD_TYPE_MASK   0x03

#define REU_XFER_STASH      0		// C64 -> REU
#define REU_XFER_FETCH      1		// REU -> C64
#define REU_XFER_SWAP       2
#define REU_XFER_VERIFY     3

// --- status register bits ---------------------------------------------------
#define REU_ST_IRQ          0x80
#define REU_ST_END_OF_BLOCK 0x40
#define REU_ST_FAULT        0x20
#define REU_ST_SIZE         0x10	// 1 = 512K or larger

// --- interrupt mask bits ----------------------------------------------------
#define REU_IRQ_ENABLE      0x80
#define REU_IRQ_ON_END      0x40
#define REU_IRQ_ON_FAULT    0x20

// --- address control bits ---------------------------------------------------
#define REU_ADDR_FIX_C64    0x80
#define REU_ADDR_FIX_REU    0x40

static u32 reuSizeFromSelector( u8 selector )
{
	switch ( selector )
	{
	case REUSIZE_128K: return   128u * 1024u;
	case REUSIZE_256K: return   256u * 1024u;
	case REUSIZE_512K: return   512u * 1024u;
	case REUSIZE_2MB:  return  2048u * 1024u;
	case REUSIZE_4MB:  return  4096u * 1024u;
	case REUSIZE_16MB: return 16384u * 1024u;
	// REUSIZE_NONE and anything unrecognised. An unknown selector is treated
	// as absent rather than rounded to a neighbour: a typo in the config
	// should produce a machine with no REU, not silently a different one.
	default:           return 0;
	}
}

CREU::CREU()
	: m_Host( 0 ), m_RAM( 0 ), m_Size( 0 ), m_AddressMask( 0 ),
	  m_C64Base( 0 ), m_REUBase( 0 ), m_Length( 0 ),
	  m_Command( 0 ), m_IRQMask( 0 ), m_AddressControl( 0 ),
	  m_C64Shadow( 0 ), m_REUShadow( 0 ), m_LengthShadow( 0 ),
	  m_IRQPending( false ), m_EndOfBlock( false ), m_Fault( false ),
	  m_PendingFF00( false ),
	  m_Transfers( 0 ), m_BytesMoved( 0 ), m_VerifyFaults( 0 ),
	  m_LastTransferCycles( 0 )
{
}

CREU::~CREU()
{
	if ( m_RAM ) free( m_RAM );
	m_RAM = 0;
}

bool CREU::init( u8 reuSizeSelector )
{
	if ( m_RAM ) { free( m_RAM ); m_RAM = 0; }
	m_Size = 0;
	m_AddressMask = 0;

	const u32 want = reuSizeFromSelector( reuSizeSelector );
	if ( !want )
	{
		reset();
		return true;			// "no REU" is a success, not a failure
	}

	m_RAM = (u8 *)malloc( want );
	if ( !m_RAM )
	{
		reset();
		return false;			// asked for a unit and could not have one
	}

	// A real unit powers up with indeterminate contents. Zero is chosen over
	// a pattern so that a program reading uninitialised expansion RAM behaves
	// the same on every boot -- an intermittent difference here would be
	// mistaken for a transfer bug.
	memset( m_RAM, 0, want );

	m_Size = want;
	m_AddressMask = want - 1;	// every supported size is a power of two
	reset();
	return true;
}

void CREU::reset()
{
	// Expansion RAM contents deliberately survive: the hardware does not clear
	// them, and software has been known to keep data across a warm reset.
	m_C64Base = 0;
	m_REUBase = 0;
	m_Length = 0;
	m_Command = 0;
	m_IRQMask = 0;
	m_AddressControl = 0;
	m_C64Shadow = 0;
	m_REUShadow = 0;
	m_LengthShadow = 0;
	m_IRQPending = false;
	m_EndOfBlock = false;
	m_Fault = false;
	m_PendingFF00 = false;
	m_LastTransferCycles = 0;
}

bool CREU::read( u16 addr, u8 &value )
{
	if ( addr < REU_REG_FIRST || addr > REU_REG_LAST ) return false;
	if ( !m_Size )
	{
		// No unit fitted: leave the access alone entirely rather than
		// answering with a plausible-looking register file. Software probes
		// for an REU by writing and reading back, and a device that answers
		// but does not transfer is worse than no device at all.
		return false;
	}

	switch ( addr )
	{
	case 0xDF00:
	{
		value = 0;
		if ( m_IRQPending ) value |= REU_ST_IRQ;
		if ( m_EndOfBlock ) value |= REU_ST_END_OF_BLOCK;
		if ( m_Fault )      value |= REU_ST_FAULT;
		// The size bit is how software tells a 1750-class unit from the
		// smaller ones, and everything at or above 512K reports the large
		// value. Version bits stay zero.
		if ( m_Size >= 512u * 1024u ) value |= REU_ST_SIZE;

		// Reading the status register CLEARS the three event bits. That is
		// the whole acknowledge mechanism -- there is no separate write to
		// dismiss them -- so this side effect is load-bearing, not tidiness.
		m_IRQPending = false;
		m_EndOfBlock = false;
		m_Fault = false;
		return true;
	}

	// Unused bits read as 1 across the register file. Software has used that
	// to distinguish a real controller from empty I/O space.
	case 0xDF01: value = (u8)( m_Command | 0x4C ); return true;
	case 0xDF02: value = (u8)( m_C64Base & 0xFF ); return true;
	case 0xDF03: value = (u8)( m_C64Base >> 8 ); return true;
	case 0xDF04: value = (u8)( m_REUBase & 0xFF ); return true;
	case 0xDF05: value = (u8)( ( m_REUBase >> 8 ) & 0xFF ); return true;
	case 0xDF06: value = (u8)( ( ( m_REUBase >> 16 ) & 0x07 ) | 0xF8 ); return true;
	case 0xDF07: value = (u8)( m_Length & 0xFF ); return true;
	case 0xDF08: value = (u8)( m_Length >> 8 ); return true;
	case 0xDF09: value = (u8)( m_IRQMask | 0x1F ); return true;
	case 0xDF0A: value = (u8)( m_AddressControl | 0x3F ); return true;
	default:     value = 0xFF; return true;
	}
}

bool CREU::write( u16 addr, u8 value )
{
	if ( addr < REU_REG_FIRST || addr > REU_REG_LAST ) return false;
	if ( !m_Size ) return false;

	switch ( addr )
	{
	case 0xDF00:
		// Status is read-only. Accept and discard, which is what the chip
		// does; refusing would send the access to the bus.
		return true;

	case 0xDF01:
		m_Command = value;
		if ( value & REU_CMD_EXECUTE )
		{
			if ( value & REU_CMD_NO_FF00 )
				executeTransfer();
			else
				// Armed, but the transfer does not begin until something
				// writes $FF00. Programs use this to line a transfer up with
				// a known point in their own instruction stream.
				m_PendingFF00 = true;
		}
		return true;

	// Every address/length write updates BOTH the live register and the
	// shadow. Autoload restores from the shadow at the end of a transfer, so
	// the shadow has to be whatever the program last asked for rather than
	// whatever the previous transfer left behind.
	case 0xDF02:
		m_C64Base = (u16)( ( m_C64Base & 0xFF00 ) | value );
		m_C64Shadow = m_C64Base;
		return true;
	case 0xDF03:
		m_C64Base = (u16)( ( m_C64Base & 0x00FF ) | ( (u16)value << 8 ) );
		m_C64Shadow = m_C64Base;
		return true;

	case 0xDF04:
		m_REUBase = ( m_REUBase & 0xFFFF00u ) | value;
		m_REUShadow = m_REUBase;
		return true;
	case 0xDF05:
		m_REUBase = ( m_REUBase & 0xFF00FFu ) | ( (u32)value << 8 );
		m_REUShadow = m_REUBase;
		return true;
	case 0xDF06:
		m_REUBase = ( m_REUBase & 0x00FFFFu )
		          | ( ( (u32)value & 0x07u ) << 16 );
		m_REUShadow = m_REUBase;
		return true;

	case 0xDF07:
		m_Length = (u16)( ( m_Length & 0xFF00 ) | value );
		m_LengthShadow = m_Length;
		return true;
	case 0xDF08:
		m_Length = (u16)( ( m_Length & 0x00FF ) | ( (u16)value << 8 ) );
		m_LengthShadow = m_Length;
		return true;

	case 0xDF09:
		m_IRQMask = value;
		return true;

	case 0xDF0A:
		m_AddressControl = value;
		return true;

	default:
		return true;
	}
}

void CREU::noteFF00Write()
{
	if ( !m_Size || !m_PendingFF00 ) return;
	m_PendingFF00 = false;
	executeTransfer();
}

void CREU::executeTransfer()
{
	m_PendingFF00 = false;
	if ( !m_Host || !m_Size )
	{
		// Nothing to transfer against. Report completion rather than hanging
		// a program that polls for end-of-block.
		finishTransfer( false );
		return;
	}

	// A length of zero means the full 64KB. Zero bytes would be useless and
	// the hardware has no way to express it, so the encoding is reused.
	u32 remaining = m_Length ? (u32)m_Length : 0x10000u;
	const u8 type = (u8)( m_Command & REU_CMD_TYPE_MASK );
	const bool fixC64 = ( m_AddressControl & REU_ADDR_FIX_C64 ) != 0;
	const bool fixREU = ( m_AddressControl & REU_ADDR_FIX_REU ) != 0;

	u16 c64 = m_C64Base;
	u32 reu = m_REUBase;
	bool fault = false;
	u32 moved = 0;

	while ( remaining-- )
	{
		const u32 reuOffset = reu & m_AddressMask;

		switch ( type )
		{
		case REU_XFER_STASH:
			m_RAM[ reuOffset ] = m_Host->reuHostRead( c64 );
			break;

		case REU_XFER_FETCH:
			m_Host->reuHostWrite( c64, m_RAM[ reuOffset ] );
			break;

		case REU_XFER_SWAP:
		{
			const u8 fromC64 = m_Host->reuHostRead( c64 );
			m_Host->reuHostWrite( c64, m_RAM[ reuOffset ] );
			m_RAM[ reuOffset ] = fromC64;
			break;
		}

		case REU_XFER_VERIFY:
		default:
			if ( m_Host->reuHostRead( c64 ) != m_RAM[ reuOffset ] )
				fault = true;
			break;
		}

		moved++;

		// A mismatch stops a verify where it found it, leaving the address
		// registers pointing at the offending byte. Software uses that to
		// report WHERE a comparison failed, so stopping early is the
		// behaviour rather than an optimisation.
		if ( fault ) break;

		// The C64 side wraps within its 64K; the REU side wraps within the
		// fitted size, which the mask already does on every access.
		if ( !fixC64 ) c64 = (u16)( c64 + 1 );
		if ( !fixREU ) reu = reu + 1;
	}

	m_BytesMoved += moved;
	m_LastTransferCycles = moved;	// one byte per cycle while it holds the bus

	if ( m_Command & REU_CMD_AUTOLOAD )
	{
		// Reload from the shadow, so an identical transfer needs only another
		// command write. This is why the shadow exists.
		m_C64Base = m_C64Shadow;
		m_REUBase = m_REUShadow;
		m_Length = m_LengthShadow;
	}
	else
	{
		// Without autoload the registers keep where the transfer finished.
		m_C64Base = c64;
		m_REUBase = reu & 0x07FFFFFu;
		// A completed transfer leaves the length reading 1 rather than 0 on
		// real hardware. Worth confirming against VICE's reu.c before anything
		// is allowed to depend on it -- it is the sort of detail that differs
		// between implementations and that a loader might notice.
		m_Length = fault ? (u16)( remaining + 1 ) : 1;
	}

	if ( fault ) m_VerifyFaults++;
	m_Transfers++;
	finishTransfer( fault );
}

void CREU::finishTransfer( bool fault )
{
	m_EndOfBlock = !fault;
	m_Fault = fault;

	// The interrupt is raised only if enabled AND the matching cause is
	// unmasked. Both bits are required; enabling the line alone does nothing.
	if ( m_IRQMask & REU_IRQ_ENABLE )
	{
		if ( ( m_EndOfBlock && ( m_IRQMask & REU_IRQ_ON_END ) )
		  || ( fault && ( m_IRQMask & REU_IRQ_ON_FAULT ) ) )
			m_IRQPending = true;
	}

	// The execute bit clears itself when the transfer completes, which is how
	// a program polling the command register learns the unit is idle.
	m_Command = (u8)( m_Command & ~REU_CMD_EXECUTE );
}
