/*
   SCPU-EMU - CMD SuperCPU emulation for the C64/C128 using a RAD Expansion Unit
   Copyright (c) 2026 SCPU-EMU contributors

   ICpu - common interface implemented by both the MOS 6502/6510 core and the
   WDC 65C816 core.

   Two cores exist on purpose:

     * M6502   - milestone-1 scaffolding. Small enough to be obviously correct,
                 it is used to prove that the DMA hijack, C64 banking model and
                 shadow RAM work by booting a stock KERNAL. It also serves as a
                 test oracle: in emulation mode the 65816 must agree with it
                 instruction for instruction.
     * W65C816 - the actual product. Supersedes M6502 for normal operation.

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
#ifndef _cpu_h
#define _cpu_h

#include "../Common/types.h"
#include "cpu_bus.h"

class ICpu
{
public:
	virtual ~ICpu() {}

	virtual void attachBus( ICpuBus *bus ) = 0;

	// Assert RESET: reload PC from the reset vector and put the core into its
	// documented post-reset state.
	virtual void reset() = 0;

	// Execute exactly one instruction (including any interrupt sequence taken
	// at its boundary). Returns the number of cycles consumed.
	virtual u32 step() = 0;

	// Execute until at least nCycles have elapsed. Returns cycles actually run,
	// which may overshoot by up to one instruction.
	virtual u64 run( u64 nCycles ) = 0;

	// End the current run() slice after the instruction in progress. This lets
	// hardware events change timing without interrupting an instruction.
	virtual void requestRunBreak() = 0;

	// Total cycles executed since reset.
	virtual u64 cycles() const = 0;

	// Current program counter, for tracing and test assertions.
	virtual scpu_addr_t pc() const = 0;

	virtual const char *name() const = 0;
};

#endif
