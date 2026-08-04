/*
   SCPU-EMU - CMD SuperCPU emulation for the C64/C128 using a RAD Expansion Unit
   Copyright (c) 2026 SCPU-EMU contributors

   Portable fixed-width types.

   Everything under Source/CPU/, Source/C64/ and Source/SuperCPU/ must compile
   both bare-metal against Circle (Raspberry Pi) and hosted against a normal
   libc (PC unit tests). Circle already defines u8/u16/u32/u64 in
   <circle/types.h>; on the host we provide the same names from <stdint.h> so
   that no core source file needs a platform #ifdef.

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
#ifndef _scpu_types_h
#define _scpu_types_h

#ifdef SCPU_HOST_BUILD

	#include <stdint.h>
	#include <stddef.h>

	typedef uint8_t   u8;
	typedef uint16_t  u16;
	typedef uint32_t  u32;
	typedef uint64_t  u64;
	typedef int8_t    s8;
	typedef int16_t   s16;
	typedef int32_t   s32;
	typedef int64_t   s64;

	#ifndef TRUE
	#define TRUE  true
	#define FALSE false
	#endif

#else

	#include <circle/types.h>

#endif

// 24-bit 65816 address. Kept as u32; callers are responsible for masking to
// 0x000000..0xFFFFFF where bank wrap-around semantics matter.
typedef u32 scpu_addr_t;

#define SCPU_ADDR_MASK 0x00FFFFFFu

#endif
