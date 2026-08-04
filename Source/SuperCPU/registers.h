/*
   SCPU-EMU - CMD SuperCPU emulation for the C64/C128 using a RAD Expansion Unit
   Copyright (c) 2026 SCPU-EMU contributors

   SuperCPU hardware registers.

   These live inside the cartridge, so they must be claimed before an access
   reaches the C64 -- nothing on the machine decodes $D074 and a stray write
   would simply be lost.

   SOURCE OF TRUTH: written against VICE's SCPU64 emulation
   (vice/src/scpu64/scpu64mem.c, scpu64_hardware_read / scpu64_hardware_store),
   which is exercised by real SuperCPU software. An earlier version of this file
   was derived from manuals and community documentation and got several things
   wrong: $D078 is SIMM configuration rather than an optimization mode, $D079 is
   a mirror of $D07B rather than an optimization mode, $D072/$D073 were missing
   entirely, and most of the $D0Bx block turned out to be readable AND writable
   with distinct meanings rather than a single status byte. Where documentation
   and VICE disagree, VICE wins.

   The $D07x registers are "write-sensitive switches": storing any value
   triggers the action and the value is ignored. That is why BASIC toggles speed
   with POKE 53370,0 -- the 0 carries no meaning. The $D0Bx block is different:
   genuinely readable, and several entries take meaningful bit values on write.

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
#ifndef _scpu_registers_h
#define _scpu_registers_h

#include "../Common/types.h"
#include "../C64/c64_memory.h"
#include "write_buffer.h"

// --- $D07x, write-sensitive ------------------------------------------------
#define SCPU_REG_D071           0xD071	// decoded, no effect
#define SCPU_REG_SYS_1MHZ_ON    0xD072	// system 1MHz enable
#define SCPU_REG_SYS_1MHZ_OFF   0xD073	// system 1MHz disable
#define SCPU_REG_OPT_VICBANK2   0xD074	// "GEOS optimization" -> optim bits 7-6 = 00
#define SCPU_REG_OPT_VICBANK1   0xD075	//                     -> 01
#define SCPU_REG_OPT_BASIC      0xD076	//                     -> 10
#define SCPU_REG_OPT_NONE       0xD077	//                     -> 11
#define SCPU_REG_SIMM_CONFIG    0xD078	// SIMM configuration; takes a value
#define SCPU_REG_SOFT_1MHZ_ALT  0xD079	// mirror of $D07B
#define SCPU_REG_SOFT_1MHZ_ON   0xD07A	// 53370: software 1MHz enable  (slow)
#define SCPU_REG_SOFT_1MHZ_OFF  0xD07B	// 53371: software 1MHz disable (fast)
#define SCPU_REG_D07C           0xD07C	// decoded, no effect
#define SCPU_REG_HWREGS_ALT     0xD07D	// mirror of $D07F
#define SCPU_REG_HWREGS_ENABLE  0xD07E	// 53374
#define SCPU_REG_HWREGS_DISABLE 0xD07F	// 53375

// --- $D0Bx, readable and partly writable -----------------------------------
#define SCPU_REG_VERSION        0xD0B0	// bits 7-6: hardware version
#define SCPU_REG_STATUS         0xD0B2	// b7 hwregs enabled, b6 system 1MHz
#define SCPU_REG_OPTIM_V2       0xD0B3	// v2 only: optimization, read/write
#define SCPU_REG_OPTIM          0xD0B4	// optimization mode, read/write
#define SCPU_REG_SWITCHES       0xD0B5	// b7 JiffyDOS switch, b6 speed switch
#define SCPU_REG_PROCMODE       0xD0B6	// read: b7 emulation mode. write: bootmap off
#define SCPU_REG_BOOTMAP_ON     0xD0B7	// write: bootmap on
#define SCPU_REG_SPEEDFLAGS     0xD0B8	// b7 software 1MHz, b6 1MHz from any source
#define SCPU_REG_SPEEDFLAGS_ALT 0xD0B9	// 53433: mirror of $D0B8
#define SCPU_REG_DOSEXT         0xD0BC	// b7 DOS extension, b6 RAMLink
#define SCPU_REG_DOSEXT_ALT     0xD0BD	// mirror of $D0BF
#define SCPU_REG_DOSEXT_ON      0xD0BE
#define SCPU_REG_DOSEXT_OFF     0xD0BF
#define SCPU_REG_STATUS_FIRST   0xD0B0
#define SCPU_REG_STATUS_LAST    0xD0BF

// $D0B0 bits 7-6. The SCPU64 reports only "v2" or "not v2"; the 64-vs-128
// distinction belongs to the SuperCPU 128's own register set.
#define SCPU_VERSION_V2         0x40
#define SCPU_VERSION_V1         0xC0

#define SCPU_STATUS_HWREGS      0x80	// $D0B2
#define SCPU_STATUS_SYS_1MHZ    0x40

// $D0B5 -- note the polarity: the bit is SET when the switch selects 1MHz.
#define SCPU_SWITCH_JIFFYDOS    0x80
#define SCPU_SWITCH_1MHZ        0x40

#define SCPU_PROC_EMULATION     0x80	// $D0B6

#define SCPU_SPEED_SOFT_1MHZ    0x80	// $D0B8
#define SCPU_SPEED_ANY_1MHZ     0x40

#define SCPU_DOS_EXTENSION      0x80	// $D0BC
#define SCPU_DOS_RAMLINK        0x40

// Optimization mode, bits 7-6 of the optimization register.
#define SCPU_OPTIM_VICBANK2     0x00	// GEOS
#define SCPU_OPTIM_VICBANK1     0x40
#define SCPU_OPTIM_BASIC        0x80
#define SCPU_OPTIM_NONE         0xC0

// --- private RAM windows ---------------------------------------------------
#define SCPU_SYSRAM_BASE        0xD200	// $D200-$D2FF, SuperCPU DOS scratch
#define SCPU_SYSRAM_SIZE        0x0100
#define SCPU_USERRAM_BASE       0xD300	// $D300-$D3FF, free for user programs
#define SCPU_USERRAM_SIZE       0x0100

enum SCPUHardwareVersion { SCPU_V1 = 1, SCPU_V2 = 2 };

class CSuperCPURegisters : public IIOInterceptor
{
public:
	CSuperCPURegisters();

	void attach( CWriteBuffer *writeBuffer ) { m_WriteBuffer = writeBuffer; }

	void setHardwareVersion( SCPUHardwareVersion v ) { m_Version = v; }

	// Physical switch positions on the cartridge.
	void setSpeedSwitchAllowsTurbo( bool allow );	// false = switch selects 1MHz
	void setJiffyDOSSwitch( bool on ) { m_SwitchJiffy = on; }

	bool speedSwitchAllowsTurbo() const { return !m_SwitchSlow; }

	// Emulation vs native mode, reported at $D0B6. The 6502 core is always in
	// emulation mode, so setEmulationMode() is enough for it.
	void setEmulationMode( bool emulation ) { m_EmulationMode = emulation; }

	// The 65816 changes mode with a single XCE, and software can read $D0B6 in
	// the very next instruction -- so a per-frame snapshot would be stale. Point
	// this at the core's E flag instead and the register reports the live state.
	// Null with the 6502 core, which falls back to setEmulationMode().
	void trackEmulationMode( const bool *emulationFlag ) { m_EmulationFlag = emulationFlag; }

	void reset();

	// --- IIOInterceptor ---------------------------------------------------
	bool ioRead( u16 addr, u8 &value ) override;
	bool ioWrite( u16 addr, u8 value ) override;

	// --- state ------------------------------------------------------------
	// The machine runs fast only when nothing is asking for 1MHz. Three
	// independent sources, and this is VICE's rule verbatim -- note that the
	// physical switch is honoured only while the hardware registers are
	// DISABLED, so software that opens the register bank takes the switch out
	// of circuit.
	bool fastMode() const
	{
		return !( m_Sys1MHz || m_Soft1MHz || ( m_SwitchSlow && !m_HWRegsEnabled ) );
	}

	bool turboEnabled() const        { return fastMode(); }
	bool hardwareRegsEnabled() const { return m_HWRegsEnabled; }
	bool bootmapEnabled() const      { return m_Bootmap; }

	// Point this at CC64Memory::m_BootmapActive. A $D0B6 write then takes
	// effect on the very next instruction fetch, which is exactly what the
	// SuperCPU's own boot code depends on: it maps itself out and continues
	// straight into the machine's KERNAL. Anything lazier -- updating at a
	// frame boundary, say -- would keep executing ROM that is no longer there.
	void trackBootmap( bool *flag ) { m_BootmapFlag = flag; applyBootmap(); }

	// Point this at CC64Memory::m_KernalShadowBase. Opening or closing the
	// register bank moves the KERNAL window between two different parts of
	// bank 1 -- see the note on that member.
	void trackKernalShadow( u32 *base ) { m_KernalShadowBase = base; applyKernalShadow(); }
	bool dosExtensionEnabled() const { return m_DOSExt; }
	u8   simmConfig() const          { return m_SIMMConfig; }
	u8   optimRegister() const       { return m_Optim; }

	// Set when anything changed the speed, so the pacer can be re-armed.
	bool consumeSpeedChanged();

	u64 m_RegWrites;

private:
	CWriteBuffer *m_WriteBuffer;

	SCPUHardwareVersion m_Version;

	// The three independent 1MHz requests, plus the physical switches.
	bool m_Sys1MHz;			// $D072 / $D073 / $D0B2
	bool m_Soft1MHz;		// $D07A / $D07B / $D0B8
	bool m_SwitchSlow;		// physical speed switch in the 1MHz position
	bool m_SwitchJiffy;		// physical JiffyDOS switch

	bool m_HWRegsEnabled;
	bool  m_Bootmap;
	bool *m_BootmapFlag;

	void applyBootmap() { if ( m_BootmapFlag ) *m_BootmapFlag = m_Bootmap; }

	u32 *m_KernalShadowBase;
	void applyKernalShadow()
	{
		// KS while the registers are open, KT while they are not.
		if ( m_KernalShadowBase ) *m_KernalShadowBase = m_HWRegsEnabled ? 0x6000 : 0xE000;
	}
	bool m_DOSExt;
	bool m_RAMLink;
	bool m_EmulationMode;
	const bool *m_EmulationFlag;
	bool m_SpeedChanged;

	u8   m_Optim;			// bits 7-6 mode; bits 2-0 OR'd into every $D0Bx read
	u8   m_SIMMConfig;

	u8   m_SysRAM[ SCPU_SYSRAM_SIZE ];
	u8   m_UserRAM[ SCPU_USERRAM_SIZE ];

	void applyOptimisation();

	// Call around anything that can change speed; records a change if the
	// effective fast/slow state actually moved.
	void noteSpeedChange( bool wasFast );
};

#endif
