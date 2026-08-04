/*
   SCPU-EMU - CMD SuperCPU emulation for the C64/C128 using a RAD Expansion Unit
   Copyright (c) 2026 SCPU-EMU contributors

   IC64Bus - the abstraction of "the physical Commodore 64 on the other side of
   the expansion port".

   Two implementations exist:
     * Bus/RAD/rad_bus   - drives the real machine over the RAD cartridge, one
                           access per ~1us C64 cycle, while holding the 6510
                           off the bus via DMA.
     * Bus/Host/host_bus - a plain in-memory stand-in so the CPU cores, banking
                           model and memory map can be tested on a PC.

   Cost model, which the layers above must respect: every call here that is not
   a no-op costs a real C64 bus cycle (~1us on a PAL machine). Reads of shadowed
   RAM must never come through this interface -- they are served from the Pi's
   own copy. Only I/O accesses and VIC-visible write mirroring belong here.
   This mirrors how the real SuperCPU behaves: 20MHz out of its own SRAM,
   dropping to 1MHz whenever it must actually touch the C64.

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
#ifndef _c64_bus_h
#define _c64_bus_h

#include "../Common/types.h"
#include "c64_signals.h"

// One entry of a mirrored-write burst.
typedef struct
{
	u16 addr;
	u8  value;
} C64BusWrite;

class IC64Bus
{
public:
	virtual ~IC64Bus() {}

	// Bring the bus up: detect the machine, take it over. After a successful
	// call the host 6510 is held off the bus and we are bus master.
	// Returns false if no running machine was found.
	virtual bool acquire() = 0;

	// Hand the machine back (release DMA). Mainly for a clean shutdown.
	virtual void release() = 0;

	virtual const C64Signals &signals() const = 0;

	// --- single accesses, one C64 cycle each -----------------------------
	virtual u8   read( u16 addr ) = 0;
	virtual void write( u16 addr, u8 value ) = 0;

	// --- burst writes ----------------------------------------------------
	// Amortises the per-access setup by keeping the address latch and bus
	// transceiver configured across the whole run. This is the path used to
	// flush the VIC-visible write buffer, and is roughly one C64 cycle per
	// byte rather than the two or three a naive write() sequence costs.
	virtual void writeBurst( const C64BusWrite *writes, u32 count ) = 0;

	// --- bulk read -------------------------------------------------------
	// Copies 'length' bytes starting at 'addr' into 'dst'. Used at start-up to
	// snapshot the C64's ROMs into the Pi's shadow memory.
	virtual void readBlock( u16 addr, u8 *dst, u32 length ) = 0;

	// --- interrupt lines, as seen on the expansion port -------------------
	virtual bool irqAsserted() = 0;
	virtual bool nmiAsserted() = 0;

	// Current VIC-II raster line, or 0xFFFF if the backend cannot tell.
	// Costs two bus cycles on real hardware ($D012 plus $D011 bit 7).
	virtual u16 rasterLine() { return 0xFFFF; }

	// --- host time source, for pacing ------------------------------------
	// A free-running counter and its rate, used to hold the emulated CPU to
	// real time. Returning 0 from hostCyclesPerSec() means "no clock here",
	// which disables pacing -- what the host test backend does, so tests keep
	// running as fast as the machine allows.
	virtual u64 hostCycles() { return 0; }
	virtual u32 hostCyclesPerSec() { return 0; }

	virtual const char *name() const = 0;
};

#endif
