/*
   SCPU-EMU - CMD SuperCPU emulation for the C64/C128 using a RAD Expansion Unit

   Bus timing state.

   The low-level DMA primitives need every timing constant available in a single
   cache line: they run inside hard-real-time windows where an L2 miss on a timing
   value would blow the cycle budget. RAD kept these constants inside its REUSTATE
   struct; SCPU-EMU has no REU, so they live here instead.

   Derived from the RAD Expansion Unit framework
   Copyright (c) 2022 Carsten Dachsbacher <frenetic@dachsbacher.de>
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
#ifndef _bus_timing_h
#define _bus_timing_h

#include <circle/types.h>
#include "lowlevel_arm64.h"

#pragma pack(push)
#pragma pack(1)

// Local copy of every timing value, kept in one aligned block so that the
// DMA inner loops never stall on a cache miss while driving the C64 bus.
typedef struct
{
	u16 WAIT_FOR_SIGNALS,
		WAIT_CYCLE_MULTIPLEXER,
		WAIT_CYCLE_READ,
		WAIT_CYCLE_WRITEDATA,
		WAIT_CYCLE_READ2,
		WAIT_CYCLE_READ_VIC2,
		WAIT_CYCLE_WRITEDATA_VIC2,
		WAIT_CYCLE_MULTIPLEXER_VIC2,
		WAIT_TRIGGER_DMA,
		WAIT_RELEASE_DMA,
		TIMING_OFFSET_CBTD,
		TIMING_DATA_HOLD,
		TIMING_TRIGGER_DMA,
		TIMING_ENABLE_ADDRLATCH,
		TIMING_READ_BA_WRITING,
		TIMING_ENABLE_RWOUT_ADDR_LATCH_WRITING,
		TIMING_ENABLE_DATA_WRITING,
		TIMING_BA_SIGNAL_AVAIL;
} __attribute__((packed)) BUSTIMING;

#pragma pack(pop)

extern BUSTIMING busTiming AAA;

// Snapshot the global timing values (set by setDefaultTimings()/readConfig())
// into busTiming. Call once after the configuration file has been parsed.
extern void initBusTiming();

#endif
