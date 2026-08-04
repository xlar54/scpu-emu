/*
   SCPU-EMU - CMD SuperCPU emulation for the C64/C128 using a RAD Expansion Unit
   Copyright (c) 2026 SCPU-EMU contributors

   SuperCPU hardware registers.

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
#include "registers.h"

CSuperCPURegisters::CSuperCPURegisters()
	: m_RegWrites( 0 ), m_WriteBuffer( 0 ),
	  m_Version( SCPU_V2 ), m_C128Mode( false ),
	  m_Turbo( true ), m_HWRegsEnabled( false ), m_SpeedChanged( false ),
	  m_SwitchAllowsTurbo( true ), m_JiffyDOSSwitch( false ), m_EnhancedOpt( 0x80 )
{
	reset();
}

void CSuperCPURegisters::setSpeedSwitchAllowsTurbo( bool allow )
{
	m_SwitchAllowsTurbo = allow;

	// Moving the switch to NORMAL drops the machine to 1MHz immediately; moving
	// it to TURBO only permits software to ask, it does not accelerate on its own.
	if ( !allow && m_Turbo )
	{
		m_Turbo = false;
		m_SpeedChanged = true;
	}
}

void CSuperCPURegisters::reset()
{
	// A SuperCPU powers up in turbo with its register bank closed, so that a
	// program which happens to touch $D074 does not silently change the
	// mirroring policy.
	m_Turbo         = m_SwitchAllowsTurbo;
	m_HWRegsEnabled = false;
	m_SpeedChanged  = true;
	m_RegWrites     = 0;
	m_EnhancedOpt   = 0x80;	// documented default: Z = 1, B = 0

	for ( u32 i = 0; i < SCPU_SYSRAM_SIZE; i++ )  m_SysRAM[ i ]  = 0;
	for ( u32 i = 0; i < SCPU_USERRAM_SIZE; i++ ) m_UserRAM[ i ] = 0;
}

bool CSuperCPURegisters::consumeSpeedChanged()
{
	bool c = m_SpeedChanged;
	m_SpeedChanged = false;
	return c;
}

void CSuperCPURegisters::selectOptMode( SCPUOptMode mode )
{
	if ( m_WriteBuffer )
		m_WriteBuffer->setOptMode( mode );
}

bool CSuperCPURegisters::ioRead( u16 addr, u8 &value )
{
	if ( addr >= SCPU_SYSRAM_BASE && addr < SCPU_SYSRAM_BASE + SCPU_SYSRAM_SIZE )
	{
		value = m_SysRAM[ addr - SCPU_SYSRAM_BASE ];
		return true;
	}

	if ( addr >= SCPU_USERRAM_BASE && addr < SCPU_USERRAM_BASE + SCPU_USERRAM_SIZE )
	{
		value = m_UserRAM[ addr - SCPU_USERRAM_BASE ];
		return true;
	}

	// The $D0B0-$D0BF status block. Each address carries distinct flags -- it is
	// not a mirrored register. Layout from the SuperCPU 128 register list; see
	// Docs/research/supercpu-registers.md for the sources and what is inferred.
	switch ( addr )
	{
	case SCPU_REG_MODE:
		value = ( m_Version == SCPU_V1 )
		      ? SCPU_MODE_V1_OR_OFF
		      : ( m_C128Mode ? SCPU_MODE_V2_128 : SCPU_MODE_V2_64 );
		return true;

	case SCPU_REG_STATUS:
		value = (u8)( ( m_HWRegsEnabled ? SCPU_STATUS_HWREGS : 0 )
		            | ( m_Turbo ? 0 : SCPU_STATUS_1MHZ ) );
		return true;

	case SCPU_REG_ENHANCED_OPT:
		// v2 only. Readable always, writable once the register bank is open.
		// The bit encoding is only partly documented -- we keep the byte and
		// report the documented power-on default (Z=1, B=0) rather than
		// pretending to decode it.
		value = ( m_Version == SCPU_V2 ) ? m_EnhancedOpt : 0xFF;
		return true;

	case SCPU_REG_OPTFLAGS:
		switch ( m_WriteBuffer ? m_WriteBuffer->optMode() : SCPU_OPT_NONE )
		{
		case SCPU_OPT_VICBANK2: value = SCPU_OPTFLAG_VICBANK2; break;
		case SCPU_OPT_VICBANK1: value = SCPU_OPTFLAG_VICBANK1; break;
		case SCPU_OPT_BASIC:    value = SCPU_OPTFLAG_BASIC;    break;
		default:
			// DEFAULT, NONE and the v2-only bank 0/3 and FULL modes have no
			// encoding in these two bits; report "no optimization".
			value = SCPU_OPTFLAG_NONE;
			break;
		}
		return true;

	case SCPU_REG_SWITCHES:
		value = (u8)( ( m_JiffyDOSSwitch ? SCPU_SWITCH_JIFFYDOS : 0 )
		            | ( m_SwitchAllowsTurbo ? 0 : SCPU_SWITCH_SPEED_NORMAL ) );
		return true;

	case SCPU_REG_PROCFLAGS:
		// Milestone 1 runs a 6502, so the 65816 is effectively always in
		// emulation mode. The 65816 core will drive this from its E flag.
		value = SCPU_PROC_EMULATION;
		return true;

	case SCPU_REG_SPEEDFLAGS:
		value = (u8)( ( m_Turbo ? 0 : SCPU_SPEEDFLAG_SW_NORMAL )
		            | ( ( m_Turbo && m_SwitchAllowsTurbo ) ? 0 : SCPU_SPEEDFLAG_MASTER_NORMAL ) );
		return true;

	case SCPU_REG_DOSFLAGS:
		// This is the detection register: bit 7 reads 1 on a stock machine
		// because nothing decodes the address, and 0 here because we do.
		// Both remaining flags (DOS extension mode, RAMLink registers) are off.
		value = 0x00;
		return true;

	default:
		break;
	}

	// Any other address in the block is decoded but carries no documented
	// flags. Answering keeps bit 7 clear so detection still succeeds if a
	// program probes a neighbouring address.
	if ( addr > SCPU_REG_MODE && addr <= SCPU_REG_STATUS_LAST )
	{
		value = 0x00;
		return true;
	}

	// The optimization and speed selects are write-only. Reading them on real
	// hardware returns open bus; let the access fall through to the C64 so the
	// value matches whatever the machine would have produced.
	return false;
}

bool CSuperCPURegisters::ioWrite( u16 addr, u8 value )
{
	if ( addr >= SCPU_SYSRAM_BASE && addr < SCPU_SYSRAM_BASE + SCPU_SYSRAM_SIZE )
	{
		m_SysRAM[ addr - SCPU_SYSRAM_BASE ] = value;
		return true;
	}

	if ( addr >= SCPU_USERRAM_BASE && addr < SCPU_USERRAM_BASE + SCPU_USERRAM_SIZE )
	{
		m_UserRAM[ addr - SCPU_USERRAM_BASE ] = value;
		return true;
	}

	// $D0B3 is the one genuinely writable register, and only while the bank is
	// open. Everything else below is write-sensitive: the value is irrelevant.
	if ( addr == SCPU_REG_ENHANCED_OPT && m_HWRegsEnabled && m_Version == SCPU_V2 )
	{
		m_EnhancedOpt = value;
		m_RegWrites++;
		return true;
	}

	switch ( addr )
	{
	case SCPU_REG_HWREGS_ENABLE:
		m_HWRegsEnabled = true;
		m_RegWrites++;
		return true;

	case SCPU_REG_HWREGS_DISABLE:
		m_HWRegsEnabled = false;
		m_RegWrites++;
		return true;

	// Speed selection works whether or not the register bank is open -- this
	// is what makes POKE 53370,0 usable straight from BASIC.
	case SCPU_REG_SPEED_NORMAL:
		if ( m_Turbo ) { m_Turbo = false; m_SpeedChanged = true; }
		m_RegWrites++;
		return true;

	case SCPU_REG_SPEED_TURBO:
		// The switch gates this. In the NORMAL position the request is accepted
		// and discarded -- the machine stays at 1MHz -- which is what lets a
		// user force compatibility for software that asks for turbo regardless.
		if ( m_SwitchAllowsTurbo && !m_Turbo ) { m_Turbo = true; m_SpeedChanged = true; }
		m_RegWrites++;
		return true;

	default:
		break;
	}

	// The optimization selects require the register bank to be open.
	if ( m_HWRegsEnabled )
	{
		switch ( addr )
		{
		case SCPU_REG_OPT_VICBANK2: selectOptMode( SCPU_OPT_VICBANK2 ); m_RegWrites++; return true;
		case SCPU_REG_OPT_VICBANK1: selectOptMode( SCPU_OPT_VICBANK1 ); m_RegWrites++; return true;
		case SCPU_REG_OPT_BASIC:    selectOptMode( SCPU_OPT_BASIC );    m_RegWrites++; return true;
		case SCPU_REG_OPT_NONE:     selectOptMode( SCPU_OPT_NONE );     m_RegWrites++; return true;

		// V2-only. Inferred assignment -- see the note in registers.h.
		case SCPU_REG_OPT_VICBANK0:
			if ( m_Version == SCPU_V2 ) { selectOptMode( SCPU_OPT_VICBANK0 ); m_RegWrites++; return true; }
			break;
		case SCPU_REG_OPT_VICBANK3:
			if ( m_Version == SCPU_V2 ) { selectOptMode( SCPU_OPT_VICBANK3 ); m_RegWrites++; return true; }
			break;

		default:
			break;
		}
	}

	return false;
}
