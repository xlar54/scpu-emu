/*
   SCPU-EMU - CMD SuperCPU emulation for the C64/C128 using a RAD Expansion Unit
   Copyright (c) 2026 SCPU-EMU contributors

   SuperCPU hardware registers.

   These live inside the cartridge, so they must be claimed before an access
   reaches the C64 -- nothing on the machine decodes $D074 and a stray write
   would simply be lost.

   Almost all of them are "write-sensitive switches": storing *any* value
   triggers the action, the value itself is ignored, and the location cannot be
   read back. That is why BASIC code toggles speed with POKE 53370,0 /
   POKE 53371,0 -- the 0 is meaningless.

   Confidence note. The $D07x switches and the whole $D0Bx status block are
   corroborated by the CMD SuperCPU 128 V2 user's guide, the SuperCPU 128
   register list and the compatibility notes. $D078/$D079 (VIC banks 0 and 3 on
   V2 hardware) are documented as existing but their exact assignment is
   inferred, and the bit encoding of $D0B3 is only partly known. Both are
   flagged in Docs/research/supercpu-registers.md.

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

// --- optimization mode selects (write-sensitive) ---------------------------
#define SCPU_REG_OPT_VICBANK2   0xD074	// "GEOS optimization"
#define SCPU_REG_OPT_VICBANK1   0xD075
#define SCPU_REG_OPT_BASIC      0xD076
#define SCPU_REG_OPT_NONE       0xD077
#define SCPU_REG_OPT_VICBANK0   0xD078	// V2 only, assignment inferred
#define SCPU_REG_OPT_VICBANK3   0xD079	// V2 only, assignment inferred

// --- speed select (write-sensitive) ----------------------------------------
#define SCPU_REG_SPEED_NORMAL   0xD07A	// 53370: 1MHz (2MHz in C128 fast mode)
#define SCPU_REG_SPEED_TURBO    0xD07B	// 53371: 20MHz

// --- register bank enable (write-sensitive) --------------------------------
#define SCPU_REG_HWREGS_ENABLE  0xD07E
#define SCPU_REG_HWREGS_DISABLE 0xD07F

// --- status block, $D0B0-$D0BF (read) --------------------------------------
// Each address carries distinct flags; this is not a mirrored block.
#define SCPU_REG_MODE           0xD0B0	// version / 64-vs-128 mode
#define SCPU_REG_STATUS         0xD0B2	// bit7 hw-regs enabled, bit6 system 1MHz
#define SCPU_REG_ENHANCED_OPT   0xD0B3	// v2 enhanced optimization (read/write)
#define SCPU_REG_OPTFLAGS       0xD0B4	// bits 7-6: current optimization mode
#define SCPU_REG_SWITCHES       0xD0B5	// bit7 JiffyDOS switch, bit6 speed switch
#define SCPU_REG_PROCFLAGS      0xD0B6	// bit7 emulation mode, bit6 reset switch (v1)
#define SCPU_REG_SPEEDFLAGS     0xD0B8	// bit7 software speed, bit6 master speed
#define SCPU_REG_DOSFLAGS       0xD0BC	// bit7 DOS extension, bit6 RAMLink regs
#define SCPU_REG_STATUS_LAST    0xD0BF

// Detection: bit 7 of $D0BC reads 1 on a stock machine and 0 on a SuperCPU.
#define SCPU_DETECT_BIT         0x80

// $D0B5
#define SCPU_SWITCH_JIFFYDOS    0x80	// 1 = JiffyDOS switch enabled
#define SCPU_SWITCH_SPEED_NORMAL 0x40	// 1 = switch in NORMAL, locked to 1MHz

// $D0B2
#define SCPU_STATUS_HWREGS      0x80
#define SCPU_STATUS_1MHZ        0x40

// $D0B4 optimization encoding, bits 7-6
#define SCPU_OPTFLAG_VICBANK2   0x00
#define SCPU_OPTFLAG_VICBANK1   0x40
#define SCPU_OPTFLAG_BASIC      0x80
#define SCPU_OPTFLAG_NONE       0xC0

// $D0B6
#define SCPU_PROC_EMULATION     0x80

// $D0B8
#define SCPU_SPEEDFLAG_SW_NORMAL     0x80
#define SCPU_SPEEDFLAG_MASTER_NORMAL 0x40

// $D0B0 bit 7/6 encoding
#define SCPU_MODE_V2_128        0x00	// 00xxxxxx
#define SCPU_MODE_V2_64         0x40	// 01xxxxxx
#define SCPU_MODE_V1_OR_OFF     0xC0	// 11xxxxxx

// --- private RAM windows ---------------------------------------------------
#define SCPU_SYSRAM_BASE        0xD200	// $D200-$D2FF, reserved for the SCPU DOS
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
	void setC128Mode( bool c128 ) { m_C128Mode = c128; }

	// Position of the physical speed switch. True (the default) is the TURBO
	// position, which permits software to select 20MHz; false is NORMAL, which
	// locks the machine to 1MHz and makes writes to $D07B a no-op.
	void setSpeedSwitchAllowsTurbo( bool allow );
	bool speedSwitchAllowsTurbo() const { return m_SwitchAllowsTurbo; }

	// Position of the JiffyDOS switch, reported in $D0B5 bit 7.
	void setJiffyDOSSwitch( bool on ) { m_JiffyDOSSwitch = on; }

	void reset();

	// --- IIOInterceptor ---------------------------------------------------
	bool ioRead( u16 addr, u8 &value ) override;
	bool ioWrite( u16 addr, u8 value ) override;

	// --- state ------------------------------------------------------------
	bool turboEnabled() const      { return m_Turbo; }
	bool hardwareRegsEnabled() const { return m_HWRegsEnabled; }

	// Set when software last switched speed, so the timing layer can pick up
	// the change without polling.
	bool consumeSpeedChanged();

	u64 m_RegWrites;	// statistics

private:
	CWriteBuffer *m_WriteBuffer;

	SCPUHardwareVersion m_Version;
	bool m_C128Mode;

	bool m_Turbo;
	bool m_HWRegsEnabled;
	bool m_SpeedChanged;
	bool m_SwitchAllowsTurbo;
	bool m_JiffyDOSSwitch;
	u8   m_EnhancedOpt;

	u8 m_SysRAM[ SCPU_SYSRAM_SIZE ];
	u8 m_UserRAM[ SCPU_USERRAM_SIZE ];

	void selectOptMode( SCPUOptMode mode );
};

#endif
