/*
   SCPU-EMU - CMD SuperCPU emulation for the C64/C128 using a RAD Expansion Unit
   Copyright (c) 2026 SCPU-EMU contributors

   ICpuBus - the only thing a CPU core is allowed to know about the outside world.

   Deliberately narrow: a CPU core reads and writes bytes in a 24-bit address
   space and observes two interrupt lines. It knows nothing about GPIO, the RAD
   cartridge, C64 banking, or the SuperCPU memory map. That keeps the cores
   testable on a PC against a plain array of memory.

   Layering note: the SuperCPU memory map (Source/SuperCPU/memory_map) implements
   this interface. Its fast path -- reads and writes that hit RAM local to the
   Raspberry Pi -- is inlined there, so the virtual call here is only paid on
   accesses that genuinely leave the Pi and drive the real C64 bus (I/O reads,
   mirrored writes). Those cost ~1us of bus time anyway, which dwarfs a vtable
   dispatch by three orders of magnitude.

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
#ifndef _cpu_bus_h
#define _cpu_bus_h

#include "../Common/types.h"

class ICpuBus
{
public:
	virtual ~ICpuBus() {}

	// Data bus. addr is a full 24-bit address; 8-bit cores pass bank 0.
	virtual u8   read8( scpu_addr_t addr ) = 0;
	virtual void write8( scpu_addr_t addr, u8 value ) = 0;

	// Interrupt lines, sampled by the core at instruction boundaries.
	// Both are active-high here (i.e. "an interrupt is being requested"),
	// unlike the active-low physical /IRQ and /NMI signals.
	virtual bool irqAsserted() = 0;
	virtual bool nmiAsserted() = 0;

	// Called once per emulated CPU cycle group so the implementation can
	// account for elapsed time (VIC-II raster position, CIA timers, and the
	// SuperCPU's own 1MHz/20MHz speed switching). nCycles is in 65816 cycles.
	virtual void tick( u32 nCycles ) { (void)nCycles; }
};

#endif
