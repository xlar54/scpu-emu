/*
   SCPU-EMU - CMD SuperCPU emulation for the C64/C128 using a RAD Expansion Unit
   Copyright (c) 2026 SCPU-EMU contributors

   SuperCPU hardware registers. Written against VICE's SCPU64 emulation; see the
   note at the top of registers.h.

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
	: m_RegWrites( 0 ), m_WriteBuffer( 0 ), m_Version( SCPU_V2 ),
	  m_Sys1MHz( false ), m_Soft1MHz( false ),
	  m_SwitchSlow( false ), m_SwitchJiffy( true ),
	  m_HWRegsEnabled( false ), m_Bootmap( true ), m_BootmapFlag( 0 ),
	  m_KernalShadowBase( 0 ),
	  m_DOSExt( false ), m_RAMLink( false ),
	  m_EmulationMode( true ), m_EmulationFlag( 0 ), m_SpeedChanged( true ),
	  m_Optim( 0xC7 ), m_SIMMConfig( 4 )
{
	reset();
}

void CSuperCPURegisters::reset()
{
	// Nothing requesting 1MHz, register bank closed. A program that happens to
	// touch $D074 must not silently change the mirroring policy.
	m_Sys1MHz       = false;
	m_Soft1MHz      = false;
	m_HWRegsEnabled = false;
	// A real SuperCPU comes out of reset with its own ROM mapped in --
	// VICE's scpu64_hardware_reset() sets mem_reg_bootmap = 1. Whether it
	// takes effect depends on an image being loaded; see CSuperCPU::reset().
	m_Bootmap       = true;
	m_DOSExt        = false;
	m_RAMLink       = false;
	m_SpeedChanged  = true;
	m_RegWrites     = 0;
	// VICE and the v2 hardware reset to $C7: no-optimisation mode with the
	// B/Z flags selecting $0200-$FFFF (zero page and stack excluded).
	m_Optim         = 0xC7;
	m_SIMMConfig    = 4;

	for ( u32 i = 0; i < SCPU_SYSRAM_SIZE; i++ )  m_SysRAM[ i ]  = 0;
	for ( u32 i = 0; i < SCPU_USERRAM_SIZE; i++ ) m_UserRAM[ i ] = 0;

	applyOptimisation();
	applyBootmap();
	applyKernalShadow();
}

void CSuperCPURegisters::noteSpeedChange( bool wasFast )
{
	if ( fastMode() != wasFast )
	{
		m_SpeedChanged = true;
		if ( m_SpeedHook ) m_SpeedHook( m_SpeedHookCtx );
	}
}

bool CSuperCPURegisters::consumeSpeedChanged()
{
	bool c = m_SpeedChanged;
	m_SpeedChanged = false;
	return c;
}

void CSuperCPURegisters::setSpeedSwitchAllowsTurbo( bool allow )
{
	bool wasFast = fastMode();
	m_SwitchSlow = !allow;
	noteSpeedChange( wasFast );
}

void CSuperCPURegisters::applyOptimisation()
{
	if ( !m_WriteBuffer )
		return;

	// The hardware reduces bits 7, 6, 2 and 0 to a four-bit mirror selector.
	// The bundled VICE reference's mem_mirrors[] table is the authoritative
	// mapping. The low B/Z flags are not merely status bits: ignoring them can
	// select the opposite VIC bank or turn "no mirroring" into heavy mirroring.
	const u8 mirror = (u8)( ( ( m_Optim & 0x01 ) ? 0x01 : 0 )
	                      | ( ( m_Optim & 0x04 ) ? 0x02 : 0 )
	                      | ( ( m_Optim & 0x40 ) ? 0x04 : 0 )
	                      | ( ( m_Optim & 0x80 ) ? 0x08 : 0 ) );

	m_WriteBuffer->setExcludeZeroPageStack( false );
	switch ( mirror )
	{
	case 0:
	case 1:  m_WriteBuffer->setOptMode( SCPU_OPT_VICBANK2 ); break; // $8000-$BFFF
	case 2:  m_WriteBuffer->setOptMode( SCPU_OPT_VICBANK0 ); break; // $0000-$3FFF
	case 3:  m_WriteBuffer->setExcludeZeroPageStack( true );
	         m_WriteBuffer->setOptMode( SCPU_OPT_VICBANK0 ); break; // $0200-$3FFF
	case 4:
	case 5:  m_WriteBuffer->setOptMode( SCPU_OPT_VICBANK1 ); break; // $4000-$7FFF
	case 6:
	case 7:  m_WriteBuffer->setOptMode( SCPU_OPT_VICBANK3 ); break; // $C000-$FFFF
	case 8:
	case 9:  m_WriteBuffer->setOptMode( SCPU_OPT_BASIC );    break; // $0400-$07FF
	case 10:
	case 11: m_WriteBuffer->setOptMode( SCPU_OPT_FULL );     break; // nothing
	case 12:
	case 14: m_WriteBuffer->setOptMode( SCPU_OPT_NONE );     break; // $0000-$FFFF
	case 13:
	case 15: m_WriteBuffer->setExcludeZeroPageStack( true );
	         m_WriteBuffer->setOptMode( SCPU_OPT_DEFAULT );  break; // $0200-$FFFF
	}
}

// ---------------------------------------------------------------------------

bool CSuperCPURegisters::ioRead( u16 addr, u8 &value )
{
	// $D200-$D3FF READS are always answered from the accelerator's RAM,
	// whether or not the register bank is open. Only writes are gated -- see
	// ioWrite, and VICE's scpu64_d200_read, which has no such condition.
	if ( addr >= SCPU_SYSRAM_BASE && addr < SCPU_SYSRAM_BASE + SCPU_SYSRAM_SIZE )
	{
		// BOOT_ANIMATION, a deliberate one-byte deviation. 2.04's startup
		// writes $FF to $D20C with the bank open, closes the bank one
		// instruction before reading it back, and runs its C64 startup
		// animation only when the read returns zero -- so on the faithful
		// path (reads always answered from this RAM, as VICE does) the
		// animation is dead code. Answering that ONE closed-bank read with
		// what sits underneath the accelerator -- $D20C is the VIC mirror's
		// sprite 6 X position, freshly zeroed by the ROM's own JSR $E5A0 --
		// takes the animation branch instead. Scoped to exactly this byte
		// and bank state; DOS's own open-bank scratch traffic is untouched.
		if ( m_BootAnimHack && addr == 0xD20C && !m_HWRegsEnabled )
		{
			value = 0x00;
			return true;
		}
		value = m_SysRAM[ addr - SCPU_SYSRAM_BASE ];
		return true;
	}

	if ( addr >= SCPU_USERRAM_BASE && addr < SCPU_USERRAM_BASE + SCPU_USERRAM_SIZE )
	{
		value = m_UserRAM[ addr - SCPU_USERRAM_BASE ];
		return true;
	}

	if ( addr < SCPU_REG_STATUS_FIRST || addr > SCPU_REG_STATUS_LAST )
		return false;			// $D07x is write-only; let it fall through

	u8 v = 0x00;

	switch ( addr )
	{
	case SCPU_REG_VERSION:
		v = ( m_Version == SCPU_V2 ) ? SCPU_VERSION_V2 : SCPU_VERSION_V1;
		break;

	case SCPU_REG_STATUS:
		v = (u8)( ( m_HWRegsEnabled ? SCPU_STATUS_HWREGS : 0 )
		        | ( m_Sys1MHz ? SCPU_STATUS_SYS_1MHZ : 0 ) );
		break;

	case SCPU_REG_OPTIM_V2:
		if ( m_Version == SCPU_V2 )
			v = (u8)( m_Optim & 0xC0 );
		break;

	case SCPU_REG_OPTIM:
		v = (u8)( m_Optim & 0xC0 );
		break;

	case SCPU_REG_SWITCHES:
		v = (u8)( ( m_SwitchJiffy ? SCPU_SWITCH_JIFFYDOS : 0 )
		        | ( m_SwitchSlow ? SCPU_SWITCH_1MHZ : 0 ) );
		break;

	case SCPU_REG_PROCMODE:
		{
			const bool emu = m_EmulationFlag ? *m_EmulationFlag : m_EmulationMode;
			v = emu ? SCPU_PROC_EMULATION : 0x00;
		}
		break;

	case SCPU_REG_SPEEDFLAGS:
	case SCPU_REG_SPEEDFLAGS_ALT:
		// bit 6 is the combined answer: is anything holding us at 1MHz? Note
		// the switch only counts while the register bank is closed, matching
		// fastMode().
		v = (u8)( ( m_Soft1MHz ? SCPU_SPEED_SOFT_1MHZ : 0 )
		        | ( ( m_Soft1MHz || ( m_SwitchSlow && !m_HWRegsEnabled ) || m_Sys1MHz )
		            ? SCPU_SPEED_ANY_1MHZ : 0 ) );
		break;

	case SCPU_REG_DOSEXT:
	case SCPU_REG_DOSEXT_ALT:
	case SCPU_REG_DOSEXT_ON:
	case SCPU_REG_DOSEXT_OFF:
		// This is also the detection register: bit 7 reads as 1 on a stock
		// machine because nothing decodes the address, and as the DOS extension
		// flag (normally 0) here because we do.
		v = (u8)( ( m_DOSExt ? SCPU_DOS_EXTENSION : 0 )
		        | ( m_RAMLink ? SCPU_DOS_RAMLINK : 0 ) );
		break;

	default:
		// $D0B1, $D0B7, $D0BA, $D0BB: decoded, no flags of their own.
		break;
	}

	// The low three bits of the optimization register appear in every read of
	// this block, whichever register was addressed.
	value = (u8)( v | ( m_Optim & 0x07 ) );
	return true;
}

bool CSuperCPURegisters::ioWrite( u16 addr, u8 value )
{
	// $D200-$D3FF is WRITE-PROTECTED while the register bank is closed, though
	// it always reads back. VICE's scpu64_d200_store/scpu64_d300_store gate on
	// mem_reg_hwenable, with $D27E the single exception.
	//
	// This is not a detail. CMD's 2.04 startup writes $FF to $D20C with the
	// bank closed -- so the write is discarded and the location stays zero --
	// and later branches on reading it back. Honouring that write leaves $D20C
	// non-zero, which sends the boot down the path that skips the C64 startup
	// animation entirely. $D20C is written once and read once in the whole
	// 128K image, and the discarded write IS the mechanism.
	if ( addr >= SCPU_SYSRAM_BASE && addr < SCPU_SYSRAM_BASE + SCPU_SYSRAM_SIZE )
	{
		if ( m_HWRegsEnabled || addr == SCPU_SYSRAM_ALWAYS_WRITABLE )
			m_SysRAM[ addr - SCPU_SYSRAM_BASE ] = value;
		return true;
	}

	if ( addr >= SCPU_USERRAM_BASE && addr < SCPU_USERRAM_BASE + SCPU_USERRAM_SIZE )
	{
		if ( m_HWRegsEnabled )
			m_UserRAM[ addr - SCPU_USERRAM_BASE ] = value;
		return true;
	}

	const bool inD07x = ( addr >= SCPU_REG_D071 && addr <= SCPU_REG_HWREGS_DISABLE );
	const bool inD0Bx = ( addr >= SCPU_REG_STATUS_FIRST && addr <= SCPU_REG_STATUS_LAST );

	if ( !inD07x && !inD0Bx )
		return false;

	const bool wasFast = fastMode();
	m_RegWrites++;

	switch ( addr )
	{
	// --- speed, always available whether or not the bank is open -----------
	case SCPU_REG_SYS_1MHZ_ON:   m_Sys1MHz  = true;  break;
	case SCPU_REG_SYS_1MHZ_OFF:  m_Sys1MHz  = false; break;
	case SCPU_REG_SOFT_1MHZ_ON:  m_Soft1MHz = true;  break;
	case SCPU_REG_SOFT_1MHZ_ALT:
	case SCPU_REG_SOFT_1MHZ_OFF: m_Soft1MHz = false; break;

	// --- register bank -----------------------------------------------------
	case SCPU_REG_HWREGS_ENABLE:  m_HWRegsEnabled = true;  applyKernalShadow(); break;
	case SCPU_REG_HWREGS_ALT:
	case SCPU_REG_HWREGS_DISABLE: m_HWRegsEnabled = false; applyKernalShadow(); break;

	// --- optimization: needs the bank open ---------------------------------
	case SCPU_REG_OPT_VICBANK2:
	case SCPU_REG_OPT_VICBANK1:
	case SCPU_REG_OPT_BASIC:
	case SCPU_REG_OPT_NONE:
		if ( m_HWRegsEnabled )
		{
			// VICE models these legacy switches as a complete assignment, not a
			// read/modify/write: the address supplies bits 7-6 and the enhanced
			// B/Z flags are cleared. Preserving reset's low $07 here would make
			// $D076 select mirror-table entry 11 (empty) instead of BASIC.
			m_Optim = (u8)( ( addr & 3 ) << 6 );
			applyOptimisation();
		}
		break;

	case SCPU_REG_SIMM_CONFIG:
		if ( m_HWRegsEnabled )
			m_SIMMConfig = value;
		break;

	case SCPU_REG_D071:
	case SCPU_REG_D07C:
		break;					// decoded, no effect

	// --- $D0Bx writes ------------------------------------------------------
	case SCPU_REG_STATUS:
		// Sets the system 1MHz request, and clearing bit 7 closes the bank.
		if ( m_HWRegsEnabled )
		{
			m_Sys1MHz = ( value & SCPU_STATUS_SYS_1MHZ ) != 0;
			if ( !( value & SCPU_STATUS_HWREGS ) )
			{
				m_HWRegsEnabled = false;

				// Closing the bank MOVES THE KERNAL WINDOW, and it must move
				// before the next instruction fetch. CMD's serial code ends
				// its receive routine with a cross-image trampoline:
				//
				//   KS $EE7F: STZ $D0B2      bit 7 clear -> bank closes
				//   KS $EE82: (falls through)
				//
				// and on real hardware the fetch at $EE82 now comes from the
				// KT image, where it is CLI / CLC / RTS -- the clean end of
				// the byte receive. With the flag cleared but the window left
				// in place, execution stays in the KS image, whose bytes at
				// $EE8A are JMP $EE7F: an unconditional loop, a frozen
				// machine, and an IEC bus stuck mid-transfer. That was the
				// disk freeze, verified against VICE scpu64mem.c:753.
				applyKernalShadow();
			}
		}
		break;

	case SCPU_REG_OPTIM_V2:
		if ( m_HWRegsEnabled && m_Version == SCPU_V2 )
		{
			m_Optim = (u8)( ( m_Optim & 0x38 ) | ( value & 0xC7 ) );
			applyOptimisation();
		}
		break;

	case SCPU_REG_OPTIM:
		if ( m_HWRegsEnabled )
		{
			m_Optim = (u8)( ( m_Optim & 0x3F ) | ( value & 0xC0 ) );
			applyOptimisation();
		}
		break;

	case SCPU_REG_SWITCHES:
		// Emulator extension: real hardware reads two physical switches here,
		// but this build has no JiffyDOS switch input. Make bit 7 directly
		// writable as the virtual switch; unlike the real $D0Bx controls this is
		// intentionally NOT gated by $D07E. Opening the hardware-register bank
		// immediately swaps the active KERNAL window and is unsafe as a BASIC
		// configuration sequence. Bit 6 remains the separately configured,
		// read-only speed-switch state.
		m_SwitchJiffy = ( value & SCPU_SWITCH_JIFFYDOS ) != 0;
		break;

	case SCPU_REG_PROCMODE:		// write side: disable bootmap
		if ( m_HWRegsEnabled )
		{
			m_Bootmap = false;
			applyBootmap();
		}
		break;

	case SCPU_REG_BOOTMAP_ON:
		if ( m_HWRegsEnabled )
		{
			m_Bootmap = true;
			applyBootmap();
		}
		break;

	case SCPU_REG_SPEEDFLAGS:
		if ( m_HWRegsEnabled )
			m_Soft1MHz = ( value & SCPU_SPEED_SOFT_1MHZ ) != 0;
		break;

	case SCPU_REG_DOSEXT:
		if ( m_HWRegsEnabled )
			m_DOSExt = ( value & SCPU_DOS_EXTENSION ) != 0;
		break;

	case SCPU_REG_DOSEXT_ON:
		if ( m_HWRegsEnabled )
			m_DOSExt = true;
		break;

	case SCPU_REG_DOSEXT_ALT:
	case SCPU_REG_DOSEXT_OFF:
		// Note: no hwenable check here, matching VICE.
		m_DOSExt = false;
		break;

	default:
		break;				// $D0B0, $D0B1, $D0B9, $D0BA, $D0BB: no write effect
	}

	noteSpeedChange( wasFast );
	return true;
}
