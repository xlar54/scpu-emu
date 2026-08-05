/*

  {_______            {_          {______
        {__          {_ __               {__
        {__         {_  {__               {__
     {__           {__   {__               {__
 {______          {__     {__              {__
       {__       {__       {__            {__   
         {_________         {______________		Expansion Unit
                
 RADExp - A framework for DMA interfacing with Commodore C64/C128 computers using a Raspberry Pi Zero 2 or 3A+/3B+
 Copyright (c) 2022 Carsten Dachsbacher <frenetic@dachsbacher.de>

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
#ifndef _config_h
#define _config_h


#define TIMING_NAMES 25
const char timingNames[TIMING_NAMES][32] = {
	"WAIT_FOR_SIGNALS", 
	"WAIT_CYCLE_READ", 
	"WAIT_CYCLE_WRITEDATA", 
	"WAIT_CYCLE_READ_BADLINE", 
	"WAIT_CYCLE_READ_VIC2", 
	"WAIT_CYCLE_WRITEDATA_VIC2", 
	"WAIT_CYCLE_MULTIPLEXER", 
	"WAIT_CYCLE_MULTIPLEXER_VIC2", 
	"WAIT_TRIGGER_DMA", 
	"WAIT_RELEASE_DMA",
	"WAIT_OFFSET_CBTD",
	"WAIT_DATA_HOLD",
	// Was a second "WAIT_TRIGGER_DMA" upstream. The parser stops at the first
	// name that matches, so this entry could never be reached and
	// TIMING_TRIGGER_DMA was unsettable from the config file. Renamed so it
	// works; index 8 remains WAIT_TRIGGER_DMA.
	"TIMING_TRIGGER_DMA",
	"WAIT_ENABLE_ADDRLATCH",
	"WAIT_READ_BA_WRITING",
	"WAIT_ENABLE_RW_ADDRLATCH",
	"WAIT_ENABLE_DATA_WRITING", 
	"WAIT_BA_SIGNAL_AVAIL",
	"CACHING_L1_WINDOW_KB",
	"CACHING_L2_OFFSET_KB",
	"CACHING_L2_PRELOADS_PER_CYCLE",
	// Not a timing: 0 selects the 6502 core, 1 the 65816. It rides in the same
	// table because that is the whole config mechanism, and having it on the SD
	// card means switching cores -- or falling back after a bad run -- needs no
	// rebuild and no toolchain.
	"CPU_CORE",
	// Also not a timing: 1 maps the SuperCPU's own ROM over bank 0 at reset.
	"BOOTMAP",
	// Virtual replacement for the cartridge's physical JiffyDOS switch.
	"JIFFYDOS",
	// Also not a timing: mirrored bytes allowed per drain while the VIC-II is
	// drawing the picture. 0 restores strict border-only mirroring.
	"MIRROR_DISPLAY_BYTES"
};

// Which CPU core boot.cpp should install. Set from CPU_CORE in the config file;
// SCPU_CFG_CORE_DEFAULT applies when the key is absent.
extern int cfgCPUCore;

#define SCPU_CFG_CORE_6502      0
#define SCPU_CFG_CORE_65816     1
#define SCPU_CFG_CORE_DEFAULT   SCPU_CFG_CORE_65816

// Whether to let the SuperCPU's own ROM run at reset. Set from BOOTMAP in the
// config file. Ignored unless SCPU/scpu.rom is present and at least 64K.
extern int cfgBootmap;

#define SCPU_CFG_BOOTMAP_DEFAULT 1

// Initial position of the virtual JiffyDOS switch. The config file is read
// before the SuperCPU ROM runs, so CMD's boot code sees this value when it
// chooses which KERNAL to install.
extern int cfgJiffyDOS;

#define SCPU_CFG_JIFFYDOS_DEFAULT 1

// Mirrored bytes per drain call while the beam is in the visible display.
// Border-only mirroring (0) bounds delivery at roughly 3KB per frame, which a
// game redrawing moving objects outruns -- the VIC then fetches a mixture of
// several frames for exactly the things that move. Writing inside the picture
// is safe because the burst path yields the bus to the VIC on BA.
extern int cfgMirrorDisplayBytes;

#define SCPU_CFG_MIRROR_DISPLAY_DEFAULT 224

extern int readConfig( CLogger *logger, const char *DRIVE, const char *FILENAME );

#endif
