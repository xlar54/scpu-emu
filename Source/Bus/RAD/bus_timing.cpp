/*
   SCPU-EMU - CMD SuperCPU emulation for the C64/C128 using a RAD Expansion Unit

   Bus timing state.

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
#include "bus_timing.h"

BUSTIMING busTiming AAA;

void initBusTiming()
{
	busTiming.WAIT_FOR_SIGNALS            = WAIT_FOR_SIGNALS;
	busTiming.WAIT_CYCLE_MULTIPLEXER      = WAIT_CYCLE_MULTIPLEXER;
	busTiming.WAIT_CYCLE_READ             = WAIT_CYCLE_READ;
	busTiming.WAIT_CYCLE_WRITEDATA        = WAIT_CYCLE_WRITEDATA;
	busTiming.WAIT_CYCLE_READ2            = WAIT_CYCLE_READ + 20;
	busTiming.WAIT_CYCLE_READ_VIC2        = WAIT_CYCLE_READ_VIC2;
	busTiming.WAIT_CYCLE_WRITEDATA_VIC2   = WAIT_CYCLE_WRITEDATA_VIC2;
	busTiming.WAIT_CYCLE_MULTIPLEXER_VIC2 = WAIT_CYCLE_MULTIPLEXER_VIC2;
	busTiming.WAIT_TRIGGER_DMA            = WAIT_TRIGGER_DMA;
	busTiming.WAIT_RELEASE_DMA            = WAIT_RELEASE_DMA;

	busTiming.TIMING_OFFSET_CBTD          = TIMING_OFFSET_CBTD;
	busTiming.TIMING_DATA_HOLD            = TIMING_DATA_HOLD;
	busTiming.TIMING_TRIGGER_DMA          = TIMING_TRIGGER_DMA;
	busTiming.TIMING_ENABLE_ADDRLATCH     = TIMING_ENABLE_ADDRLATCH;
	busTiming.TIMING_READ_BA_WRITING      = TIMING_READ_BA_WRITING;
	busTiming.TIMING_ENABLE_RWOUT_ADDR_LATCH_WRITING
	                                      = TIMING_ENABLE_RWOUT_ADDR_LATCH_WRITING;
	busTiming.TIMING_ENABLE_DATA_WRITING  = TIMING_ENABLE_DATA_WRITING;
	busTiming.TIMING_BA_SIGNAL_AVAIL      = TIMING_BA_SIGNAL_AVAIL;
}
