/*
   SCPU-EMU - CMD SuperCPU emulation for the C64/C128 using a RAD Expansion Unit
   Copyright (c) 2026 SCPU-EMU contributors

   Start-up sequence.

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
#ifndef _scpu_boot_h
#define _scpu_boot_h

#include <circle/logger.h>
#include "../Common/types.h"

#define SCPU_DRIVE          "SD:"
#define SCPU_CONFIG_FILE    "SD:SCPU/scpu.cfg"
#define SCPU_ROM_DIR        "SD:SCPU/"

// Read the RAD timing configuration and snapshot it into the bus timing block.
// Must run before any bus access.
bool scpuBootLoadConfig( CLogger *logger );

// Bring up the accelerator and hand control to it. Does not return while the
// machine is running.
void scpuBootRun( CLogger *logger );

#endif
