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

	// Stack pointer as a bank-0 address, for post-mortem stack dumps.
	virtual u16 stackPointer() const { return 0; }

	// DIAGNOSTIC PLACEMENT CALIBRATION, in C64 cycles, positive landing the
	// physical access LATER. Defaulted to a no-op because only a core with a
	// cycle-exact external timeline can honour it; CM6502 has no such aim
	// point and silently ignoring the request is correct for it.
	//
	// WHY THIS EXISTS AT ALL. On a VSP screen the horizontal offset is a
	// DIRECT readout of the machine's end-to-end placement constant at one
	// column per cycle, so sweeping this and watching the screen measures
	// what no internal instrument here has managed to: every software anchor
	// in this project reconstructs placement from a stamp whose own in-line
	// position is unknown. The VIC's own c-access counter has no such
	// ambiguity. The value that zeroes the offset IS the constant.
	virtual void setPreciseAim( int readCycles, int writeCycles )
	{
		(void)readCycles; (void)writeCycles;
	}

	// READBACK of what the RUNNING core is actually using, so a startup log can
	// report applied state instead of parsed state. This exists because the
	// difference was not academic: the setter above was being called before the
	// core existed, the call was silently skipped, and the log printed the
	// config file's values anyway -- so three hardware calibration sweeps were
	// run against a control that was never connected. A core that ignores the
	// setter correctly reports 0 here, which is also the truth.
	virtual int preciseAimRead() const { return 0; }
	virtual int preciseAimWrite() const { return 0; }

	// THE WRITE LEAD (K502): the settle that places a VIC write exits this many
	// QUARTER C64 cycles before the pacer's lock phase, so the write's entry
	// into the bus primitive sits in the middle of the cycle before the one it
	// must drive in, instead of on top of that cycle's falling edge. The CPU
	// path from a settle exit to the primitive is ~630 ARM heavier for a write
	// than for the read the lock phase was tuned on (measured: catch= -0.46
	// cycle on every Mario DEC with the shadow read on, letters flickering
	// 0/+1). Physical only: ledger, grid origin and cycle count never see it.
	virtual void setWriteLead( int quarterCycles ) { (void)quarterCycles; }
	virtual int  writeLeadQuarters() const { return 0; }

	virtual const char *name() const = 0;
};

#endif
