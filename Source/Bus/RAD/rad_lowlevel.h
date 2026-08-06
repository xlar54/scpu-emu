/*
   SCPU-EMU - CMD SuperCPU emulation for the C64/C128 using a RAD Expansion Unit

   Shared single-access macros over the RAD DMA primitives.

   Every macro here assumes two things are in scope at the call site:
     * 'register u32 g2' -- the cached GPIO level word;
     * 'armCycleCounter' -- the file-local cycle-counter anchor the
       WAIT_UP_TO_CYCLE macros compare against.
   This is the same contract RAD uses; it is what lets these expand to a handful
   of instructions with no memory traffic in the timing-critical window.

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
#ifndef _rad_lowlevel_h
#define _rad_lowlevel_h

#include "lowlevel_dma.h"

// Align to the start of a fresh C64 cycle: wait out the CPU half-cycle, then
// the VIC half-cycle, then re-anchor the ARM cycle counter.
#define BUS_RESYNC {			\
	WAIT_FOR_CPU_HALFCYCLE		\
	WAIT_FOR_VIC_HALFCYCLE		\
	RESTART_CYCLE_COUNTER }

// Bounded half-cycle waits, for use before we know a machine is actually there.
// The plain WAIT_FOR_*_HALFCYCLE macros spin on PHI2 forever, which is correct
// once the C64 is known to be running but hangs the firmware outright if it is
// powered off, unplugged or held in reset -- there are simply no edges to see.
//
// A PAL half-cycle is ~500ns and each iteration is a single GPIO read, so a
// normal wait completes in well under a thousand iterations. The cap below is
// several orders of magnitude beyond that and only trips when the clock is dead.
#define RAD_HALFCYCLE_TIMEOUT_ITERS 2000000u

#define WAIT_FOR_CPU_HALFCYCLE_BOUNDED( okFlag ) {			\
	u32 _i = 0; okFlag = true;								\
	do {													\
		g2 = read32( ARM_GPIO_GPLEV0 );						\
		if ( ++_i > RAD_HALFCYCLE_TIMEOUT_ITERS )			\
			{ okFlag = false; break; }						\
	} while ( VIC_HALF_CYCLE ); }

#define WAIT_FOR_VIC_HALFCYCLE_BOUNDED( okFlag ) {			\
	u32 _i = 0; okFlag = true;								\
	do {													\
		g2 = read32( ARM_GPIO_GPLEV0 );						\
		if ( ++_i > RAD_HALFCYCLE_TIMEOUT_ITERS )			\
			{ okFlag = false; break; }						\
	} while ( CPU_HALF_CYCLE ); }

// Unsynchronised access: only valid when the caller has already resynced and
// knows it is at a cycle boundary.
#define RAD_POKE( a, v ) { busWriteByte_p1( g2, a, v ); busWriteByte_p2( g2, false ); }
// The SID ($D400-$D7FF) drives the data bus later than every other chip, so
// its reads sample at their own configured point. See busReadByte_p3.
#define RAD_READ_SAMPLE_AT( a ) 	( ( ( (a) & 0xFC00 ) == 0xD400 ) ? (u32)busTiming.WAIT_CYCLE_READ_SID 	                                 : (u32)( WAIT_CYCLE_WRITEDATA + 20 ) )
#define RAD_PEEK( a, v ) { busReadByte_p1( g2, a ); busReadByte_p2( g2 ); busReadByte_p3( g2, v, false, RAD_READ_SAMPLE_AT( a ) ); }

// Burst access, valid only between busBeginBurstWrites()/busEndBurstWrites().
#define RAD_BURST_POKE( a, v ) { busWriteByteBurst_p1( g2, a, v ); busWriteByteBurst_p2( g2, false ); }

// Self-synchronising access. Costs an extra cycle of alignment; use the
// unsynchronised forms inside tight loops that already track the phase.
#define RAD_SPOKE( a, v ) { BUS_RESYNC; RAD_POKE( a, v ); }
#define RAD_SPEEK( a, v ) { BUS_RESYNC; RAD_PEEK( a, v ); }

#endif
