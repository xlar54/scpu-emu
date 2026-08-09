/*
   SCPU-EMU - CMD SuperCPU emulation for the C64/C128 using a RAD Expansion Unit

   Taking the 6510 off the bus.

   This is the manoeuvre the whole project rests on, and it is also what the
   real CMD SuperCPU does: assert /DMA on the expansion port so the host CPU
   tri-states its address, data and R/W pins, then drive the bus ourselves. The
   VIC-II keeps running -- it still refreshes DRAM and generates the display and
   raster interrupts -- so from the C64's point of view nothing has been removed,
   it has merely acquired a much faster processor.

   The delicate part is *when* to assert /DMA. It has to happen while the VIC-II
   has the bus anyway (a badline, signalled by BA going low), otherwise the
   6510 is mid-cycle and the transition is not clean. The wait-for-badline
   sequence below is taken from RAD-Doom, where it is proven on real hardware.

   Derived from the RAD Expansion Unit framework
   Copyright (c) 2022, 2023 Carsten Dachsbacher <frenetic@dachsbacher.de>
   Copyright (c) 2026 SCPU-EMU contributors

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
#ifndef _cpu_hijack_h
#define _cpu_hijack_h

#include "../../Common/types.h"
#include "../../C64/c64_signals.h"

// Block until a C64 is detected running at the expected PHI2 rate. Returns
// once the machine is up; used to avoid hijacking a computer that is still
// powering on.
bool radWaitForMachineRunning();

// Same measurement, but reports what it saw: the number of ARM cycles observed
// for 1000 C64 cycles, or 0 if PHI2 never transitioned at all. A PAL C64 with
// the Pi at 1400MHz should read about 1421000. Anything wildly different is the
// single most useful number when a takeover will not start.
u64 radMeasureMachineRate();

// Pulse /RESET and let the machine boot its KERNAL. Needed so that $01 ends up
// at $37 (BASIC, KERNAL and I/O all banked in) before we snapshot the ROMs.
void radResetMachine();

// Wait for a badline, then assert /DMA and keep it asserted. On success we are
// bus master and the host CPU is halted.
//
// Returns false if no badline appeared within the timeout -- which happens when
// the screen is blanked, so the VIC-II never takes the bus. In that case /DMA
// is NOT asserted: taking the bus at an arbitrary phase, with the 6510 possibly
// mid-cycle and still driving, risks contention between two drivers.
bool radHijackCPU();

// Reset through an emulated Ultimax cartridge and take the bus while the host
// CPU is executing a known stream of read-only NOPs.  Unlike the C64-only
// badline handoff above, this also gives the C128's reset-time Z80 cartridge
// probe the GAME assertion it needs to select C64 mode before starting the
// 8502.  This is the takeover used by current upstream RAD on both machines.
bool radHijackCPUWithUltimax();

// Release /DMA and hand the machine back to its own CPU.
void radReleaseCPU();

// Probe the attached machine. Valid only once radHijackCPU() has returned.
C64MachineType   radDetectMachine();
C64VideoStandard radDetectVideoStandard();

#endif
