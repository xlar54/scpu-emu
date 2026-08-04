/*
   SCPU-EMU - CMD SuperCPU emulation for the C64/C128 using a RAD Expansion Unit
   Copyright (c) 2026 SCPU-EMU contributors

   C64 expansion-port signal levels, expressed in logical rather than electrical
   terms. On the connector /GAME, /EXROM, /IRQ, /NMI and /RESET are active-low;
   everything above this file works with "asserted = true" instead, so that no
   higher layer has to remember which pins are inverted.

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
#ifndef _c64_signals_h
#define _c64_signals_h

#include "../Common/types.h"

// Machine variants SCPU-EMU can find itself attached to.
enum C64MachineType
{
	MACHINE_UNKNOWN = 0,
	MACHINE_C64,
	MACHINE_C128
};

// Video standard, which fixes the cycles-per-rasterline budget used when
// scheduling bus traffic against the VIC-II.
enum C64VideoStandard
{
	VIDEO_PAL = 0,		// 312 rasterlines, 63 cycles/line
	VIDEO_NTSC_R56A,	// 262 rasterlines, 64 cycles/line
	VIDEO_NTSC_R8		// 263 rasterlines, 65 cycles/line
};

typedef struct
{
	C64MachineType   machine;
	C64VideoStandard video;

	// Cartridge control lines as *driven by us*. With no pass-through
	// cartridge both are inactive, which is the normal SuperCPU configuration.
	bool gameAsserted;		// true => /GAME pulled low
	bool exromAsserted;		// true => /EXROM pulled low
} C64Signals;

// Cycles per rasterline for each video standard.
static inline u32 c64CyclesPerLine( C64VideoStandard v )
{
	switch ( v )
	{
	case VIDEO_NTSC_R56A: return 64;
	case VIDEO_NTSC_R8:   return 65;
	case VIDEO_PAL:
	default:              return 63;
	}
}

static inline u32 c64RasterLines( C64VideoStandard v )
{
	switch ( v )
	{
	case VIDEO_NTSC_R56A: return 262;
	case VIDEO_NTSC_R8:   return 263;
	case VIDEO_PAL:
	default:              return 312;
	}
}

// First and last rasterline of the visible display window -- the lines during
// which the VIC-II is fetching character, colour and bitmap data. Driving bulk
// traffic on the bus across these lines corrupts those fetches and the picture
// will not hold. Outside them (bottom border, vertical blank, top border) the
// bus is ours to use freely, which is the window RAD-Doom blits in.
static inline u16 c64DisplayFirstLine( C64VideoStandard v )
{
	return ( v == VIDEO_PAL ) ? 51 : 41;
}

static inline u16 c64DisplayLastLine( C64VideoStandard v )
{
	return ( v == VIDEO_PAL ) ? 250 : 240;
}

static inline bool c64RasterIsSafeForBulkTransfer( C64VideoStandard v, u16 line )
{
	return line < c64DisplayFirstLine( v ) || line > c64DisplayLastLine( v );
}

#endif
