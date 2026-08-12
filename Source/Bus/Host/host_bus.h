/*
   SCPU-EMU - CMD SuperCPU emulation for the C64/C128 using a RAD Expansion Unit
   Copyright (c) 2026 SCPU-EMU contributors

   CHostBus - an in-memory stand-in for the physical C64.

   Lets the CPU cores, the banking model, the memory map and the write buffer be
   built and exercised on a PC with no cartridge, no Raspberry Pi and no C64.
   It is not a C64 emulator: there is no VIC-II, SID or CIA behind it. I/O
   reads return whatever the test wrote there, and every access is logged so
   tests can assert on the exact bus traffic a change produces -- which is the
   property we actually care about, since bus traffic is the scarce resource.

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
#ifndef _host_bus_h
#define _host_bus_h

#include "../../C64/c64_bus.h"

#define HOSTBUS_LOG_SIZE 65536

enum HostBusOp { HOSTOP_READ = 0, HOSTOP_WRITE, HOSTOP_BURST_WRITE };

typedef struct
{
	u8  op;
	u16 addr;
	u8  value;
} HostBusLogEntry;

class CHostBus : public IC64Bus
{
public:
	CHostBus();

	// --- IC64Bus ----------------------------------------------------------
	bool acquire() override;
	void release() override;
	const C64Signals &signals() const override { return m_Signals; }

	u8   read( u16 addr ) override;
	void write( u16 addr, u8 value ) override;
	bool verifyC64CIA2DDRA( u8 expected ) override;
	void writeBurst( const C64BusWrite *writes, u32 count ) override;
	void readBlock( u16 addr, u8 *dst, u32 length ) override;

	bool irqAsserted() override { return m_IRQ; }
	bool nmiAsserted() override { return m_NMI; }
	u16  rasterLine() override;

	// Free-running raster position, without charging a bus cycle. Used to
	// answer reads of $D011/$D012 -- see the note in read().
	u16  rasterLineInternal() const;

	const char *name() const override { return "host (simulated)"; }

	// --- test control -----------------------------------------------------
	// Backing store for the whole 64K, so tests can preload ROM images at
	// $A000/$E000 before calling CC64Memory::snapshotROMsFromBus().
	u8   m_Memory[ 0x10000 ];

	bool m_IRQ, m_NMI;
	C64Signals m_Signals;

	// Bus-cycle accounting. One cycle per single access; a burst costs one
	// cycle per byte, matching what the RAD backend achieves.
	u64  m_Cycles;
	u64  m_Reads, m_Writes, m_BurstWrites;

	// Access log, oldest first. Recording stops once full rather than wrapping:
	// tests assert on the *first* accesses a sequence produces, so keeping the
	// head is what matters and a wrap would destroy it.
	HostBusLogEntry m_Log[ HOSTBUS_LOG_SIZE ];
	u32  m_LogCount;
	bool m_LogEnabled;

	void resetStats();
	void clearLog() { m_LogCount = 0; }

private:
	void logAccess( u8 op, u16 addr, u8 value );
};

#endif
