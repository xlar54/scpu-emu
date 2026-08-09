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
#include <SDCard/emmc.h>
#include <fatfs/ff.h>
#include <circle/util.h>
#include "../Bus/RAD/lowlevel_arm64.h"
#include "config.h"
#include "helpers.h"

int atoi( char* str )
{
	int res = 0;
	for ( int i = 0; str[ i ] != '\0' && str[ i ] != 10 && str[ i ] != 13; i ++ )
		if ( str[ i ] >= '0' && str[ i ] <= '9' )
			res = res * 10 + str[ i ] - '0';
	return res;
}

char *cfgPos;
char curLine[ 2048 ];

int getNextLine()
{
	memset( curLine, 0, 2048 );

	int sp = 0, ep = 0;
	while ( cfgPos[ ep ] != 0 && cfgPos[ ep ] != '\n' ) ep++;

	while ( sp < ep && ( cfgPos[ sp ] == ' ' || cfgPos[ sp ] == 9 ) ) sp++;

	// Strip a trailing CR explicitly rather than assuming one is present. The
	// original unconditionally dropped the last character, which is correct for
	// a CRLF file but silently truncates every line of an LF-only one -- an
	// awkward failure, because the file parses and the timings are just wrong.
	int len = ep - sp;
	if ( len > 0 && cfgPos[ sp + len - 1 ] == '\r' ) len--;

	// curLine is a fixed 2048-byte buffer, so a pathologically long line (or a
	// binary file handed to us by mistake, which has no newlines at all) must be
	// truncated rather than written past the end. One byte is reserved for the
	// terminator, which the memset above has already placed.
	if ( len > (int)sizeof( curLine ) - 1 )
		len = (int)sizeof( curLine ) - 1;

	strncpy( curLine, &cfgPos[ sp ], len );

	// Step over the newline only if we actually stopped on one. A file whose
	// last line runs right up to the terminating NUL would otherwise leave
	// cfgPos one byte past the end of the buffer, and the caller's
	// "while ( *cfgPos != 0 )" would read it.
	cfgPos = &cfgPos[ ep + ( cfgPos[ ep ] == '\n' ? 1 : 0 ) ];

	return ep - sp;
}

int timingValues[ TIMING_NAMES ] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

int cfgCPUCore = SCPU_CFG_CORE_DEFAULT;
int cfgBootmap = SCPU_CFG_BOOTMAP_DEFAULT;
int cfgJiffyDOS = SCPU_CFG_JIFFYDOS_DEFAULT;
int cfgMirrorDisplayBytes = SCPU_CFG_MIRROR_DISPLAY_DEFAULT;
int cfgBootAnimation = 1;
int cfgMirrorHaltAfterS = 0;
int cfgMirrorD000Relocate = 1;
int cfgHeartbeat = 0;
int cfgNMIRetime = 1;
int cfgVectorReroute = 1;
int cfgIOStretch = 0;
int cfgNMINativeDefer = 1;
int cfgMirrorStretch = 0;
int cfgC128Mode = 0;
int cfgBusHaltAfterS = 0;

char cfg[ 65536 ];

int readConfig( CLogger *logger, const char *DRIVE, const char *FILENAME )
{
	u32 cfgBytes;
	memset( cfg, 0, 65536 );
	memset( timingValues, 0, sizeof timingValues );

	// A missing file or missing key means built-in defaults, even if this
	// function is called again after an earlier configuration. In particular,
	// JIFFYDOS 0 must not leak into a later parse with no JIFFYDOS line.
	cfgCPUCore = SCPU_CFG_CORE_DEFAULT;
	cfgBootmap = SCPU_CFG_BOOTMAP_DEFAULT;
	cfgJiffyDOS = SCPU_CFG_JIFFYDOS_DEFAULT;
	cfgMirrorDisplayBytes = SCPU_CFG_MIRROR_DISPLAY_DEFAULT;
	cfgBootAnimation = 1;
	cfgMirrorHaltAfterS = 0;
	cfgMirrorD000Relocate = 1;
	cfgHeartbeat = 0;
	cfgNMIRetime = 1;
	cfgVectorReroute = 1;
	cfgIOStretch = 0;
	cfgNMINativeDefer = 1;
	cfgMirrorStretch = 0;
	cfgC128Mode = 0;
	cfgBusHaltAfterS = 0;

	// Leave a byte spare so cfg is always NUL-terminated for the line scanner.
	if ( !readFile( logger, DRIVE, FILENAME, (u8*)cfg, &cfgBytes, sizeof( cfg ) - 1 ) )
		return 0;

	cfgPos = cfg;

	bool cpuCoreSeen = false;
	bool bootmapSeen = false;
	bool jiffyDOSSeen = false;
	bool mirrorDisplaySeen = false;
	bool bootAnimSeen = false;
	bool relocSeen = false;
	bool heartbeatSeen = false;
	bool nmiRetimeSeen = false;
	bool rerouteSeen = false;
	bool ioStretchSeen = false;
	bool nmiDeferSeen = false;
	bool mirrorStretchSeen = false;
	bool c128ModeSeen = false;
	bool busHaltSeen = false;

	while ( *cfgPos != 0 )
	{
		if ( getNextLine() && curLine[ 0 ] )
		{
			char *rest = 0;
			char *ptr = strtok_r( curLine, " \t", &rest );

			if ( ptr )
			{
					// RAD also parsed a STARTUP key here to select an REU or GeoRAM
					// size. SCPU-EMU emulates neither, so that block is gone -- it also
					// used the result of strtok_r without a null check, which a
					// truncated STARTUP line would have crashed on.

				for ( int i = 0; i < TIMING_NAMES; i++ )
					if ( strcmp( ptr, timingNames[ i ] ) == 0 && ( ptr = strtok_r( 0, "\"", &rest ) ) )
					{
						timingValues[ i ] = atoi( ptr );
						if ( i == 21 ) cpuCoreSeen = true;
						if ( i == 22 ) bootmapSeen = true;
						if ( i == 23 ) jiffyDOSSeen = true;
						if ( i == 24 ) mirrorDisplaySeen = true;
						if ( i == 26 ) bootAnimSeen = true;
						if ( i == 28 ) relocSeen = true;
						if ( i == 29 ) heartbeatSeen = true;
						if ( i == 30 ) nmiRetimeSeen = true;
						if ( i == 31 ) rerouteSeen = true;
						if ( i == 32 ) ioStretchSeen = true;
						if ( i == 33 ) nmiDeferSeen = true;
						if ( i == 34 ) mirrorStretchSeen = true;
						if ( i == 35 ) c128ModeSeen = true;
						if ( i == 36 ) busHaltSeen = true;
						while ( *ptr == '\t' || *ptr == ' ' ) ptr++;
					#ifdef DEBUG_OUT
						logger->Write( "RaspiMenu", LogNotice, "  %s >%d< (%s)", timingNames[ i ], timingValues[ i ], ptr );
					#endif
						break;
					}
			}
		}
	}

	if ( timingValues[ 0 ] ) WAIT_FOR_SIGNALS = timingValues[ 0 ];
	if ( timingValues[ 1 ] ) WAIT_CYCLE_READ = timingValues[ 1 ];
	if ( timingValues[ 2 ] ) WAIT_CYCLE_WRITEDATA = timingValues[ 2 ];
	if ( timingValues[ 3 ] ) WAIT_CYCLE_READ_BADLINE = timingValues[ 3 ];
	if ( timingValues[ 4 ] ) WAIT_CYCLE_READ_VIC2 = timingValues[ 4 ];
	if ( timingValues[ 5 ] ) WAIT_CYCLE_WRITEDATA_VIC2 = timingValues[ 5 ];
	if ( timingValues[ 6 ] ) WAIT_CYCLE_MULTIPLEXER = timingValues[ 6 ];
	if ( timingValues[ 7 ] ) WAIT_CYCLE_MULTIPLEXER_VIC2 = timingValues[ 7 ];
	if ( timingValues[ 8 ] ) WAIT_TRIGGER_DMA = timingValues[ 8 ];
	if ( timingValues[ 9 ] ) WAIT_RELEASE_DMA = timingValues[ 9 ];

	if ( timingValues[ 10 ] ) TIMING_OFFSET_CBTD = timingValues[ 10 ];
	if ( timingValues[ 11 ] ) TIMING_DATA_HOLD = timingValues[ 11 ];
	if ( timingValues[ 12 ] ) TIMING_TRIGGER_DMA = timingValues[ 12 ];
	if ( timingValues[ 13 ] ) TIMING_ENABLE_ADDRLATCH = timingValues[ 13 ];
	if ( timingValues[ 14 ] ) TIMING_READ_BA_WRITING = timingValues[ 14 ];
	if ( timingValues[ 15 ] ) TIMING_ENABLE_RWOUT_ADDR_LATCH_WRITING = timingValues[ 15 ];
	if ( timingValues[ 16 ] ) TIMING_ENABLE_DATA_WRITING = timingValues[ 16 ];
	if ( timingValues[ 17 ] ) TIMING_BA_SIGNAL_AVAIL = timingValues[ 17 ];

	if ( timingValues[ 18 ] ) CACHING_L1_WINDOW_KB = timingValues[ 18 ];
	if ( timingValues[ 19 ] ) CACHING_L2_OFFSET_KB = timingValues[ 19 ];
	if ( timingValues[ 20 ] ) CACHING_L2_PRELOADS_PER_CYCLE = timingValues[ 20 ];

	// CPU_CORE is the one key where 0 is a meaningful value rather than
	// "unset", so it cannot use the "if non-zero" idiom above. A line reading
	// CPU_CORE 0 has to be able to select the 6502.
	if ( cpuCoreSeen )
		cfgCPUCore = ( timingValues[ 21 ] != 0 ) ? SCPU_CFG_CORE_65816 : SCPU_CFG_CORE_6502;

	// Same reasoning: BOOTMAP 0 has to be able to turn it off.
	if ( bootmapSeen )
		cfgBootmap = ( timingValues[ 22 ] != 0 ) ? 1 : 0;

	// JIFFYDOS is a physical-switch replacement, so 0 and 1 are both explicit
	// values. It defaults on when the key is absent.
	if ( jiffyDOSSeen )
		cfgJiffyDOS = ( timingValues[ 23 ] != 0 ) ? 1 : 0;

	// A byte count, not a flag: 0 means border-only mirroring.
	if ( mirrorDisplaySeen && timingValues[ 24 ] >= 0 )
		cfgMirrorDisplayBytes = timingValues[ 24 ];

	if ( timingValues[ 25 ] ) WAIT_CYCLE_READ_SID = timingValues[ 25 ];

	// A flag, not a timing: explicit 0 disables, anything else enables.
	if ( bootAnimSeen )
		cfgBootAnimation = ( timingValues[ 26 ] != 0 ) ? 1 : 0;

	if ( timingValues[ 27 ] > 0 ) cfgMirrorHaltAfterS = timingValues[ 27 ];

	// A flag: MIRROR_D000_RELOCATE 0 must be able to switch relocation off.
	if ( relocSeen )
		cfgMirrorD000Relocate = ( timingValues[ 28 ] != 0 ) ? 1 : 0;

	// A flag, default off: HEARTBEAT 1 turns the per-second line on.
	if ( heartbeatSeen )
		cfgHeartbeat = ( timingValues[ 29 ] != 0 ) ? 1 : 0;

	// A flag, default on: NMI_RETIME 0 restores sampled-line delivery.
	if ( nmiRetimeSeen )
		cfgNMIRetime = ( timingValues[ 30 ] != 0 ) ? 1 : 0;

	// A flag, default on: VECTOR_REROUTE 0 fetches vectors from the C64 map.
	if ( rerouteSeen )
		cfgVectorReroute = ( timingValues[ 31 ] != 0 ) ? 1 : 0;

	// A flag, default off on RAD: the physical access already consumes its bus
	// time in wall-clock cycles, so charging the full delay again double-bills
	// I/O-heavy software. IO_STRETCH 1 remains available for model comparison.
	if ( ioStretchSeen )
		cfgIOStretch = ( timingValues[ 32 ] != 0 ) ? 1 : 0;

	if ( nmiDeferSeen )
		cfgNMINativeDefer = ( timingValues[ 33 ] != 0 ) ? 1 : 0;

	if ( mirrorStretchSeen )
		cfgMirrorStretch = ( timingValues[ 34 ] != 0 ) ? 1 : 0;

	if ( c128ModeSeen )
	{
		cfgC128Mode = timingValues[ 35 ];
		if ( cfgC128Mode < 0 || cfgC128Mode > 2 ) cfgC128Mode = 0;
	}

	if ( busHaltSeen && timingValues[ 36 ] > 0 )
		cfgBusHaltAfterS = timingValues[ 36 ];

	return 1;
}
