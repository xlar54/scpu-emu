/*
   SCPU-EMU - CMD SuperCPU emulation for the C64/C128 using a RAD Expansion Unit

   Taking the 6510 off the bus.

   Derived from the RAD Expansion Unit framework (rad_doom_hijack.cpp)
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
#include <circle/memio.h>
#include <circle/bcm2835.h>
#include <circle/types.h>

#include "gpio_defs.h"
#include "lowlevel_arm64.h"
#include "bus_timing.h"

static u64 armCycleCounter;

#include "rad_lowlevel.h"
#include "cpu_hijack.h"

u64 radMeasureMachineRate()
{
	register u32 g2;
	u64 start, duration;
	bool ok;

	RESET_CPU_CYCLE_COUNTER

	WAIT_FOR_CPU_HALFCYCLE_BOUNDED( ok ); if ( !ok ) return 0;
	WAIT_FOR_VIC_HALFCYCLE_BOUNDED( ok ); if ( !ok ) return 0;
	READ_CYCLE_COUNTER( start );

	for ( u32 i = 0; i < 1000; i++ )
	{
		WAIT_FOR_CPU_HALFCYCLE_BOUNDED( ok ); if ( !ok ) return 0;
		WAIT_FOR_VIC_HALFCYCLE_BOUNDED( ok ); if ( !ok ) return 0;
	}

	READ_CYCLE_COUNTER( duration );
	return duration - start;
}

bool radWaitForMachineRunning()
{
	// Time 1000 C64 cycles against the ARM cycle counter. A PAL C64 runs at
	// 985248Hz and the Pi at 1400MHz, so 1000 cycles should measure about
	// 1421000 ARM cycles; NTSC gives about 1369000. The window covers both plus
	// the spread across Pi models.
	//
	// Retried generously: the machine may still be completing its reset, and a
	// single marginal measurement should not condemn it. Every wait inside is
	// bounded, so a powered-off machine still fails rather than hanging.
	for ( u32 attempt = 0; attempt < 2000; attempt++ )
	{
		u64 duration = radMeasureMachineRate();

		if ( duration > 1200000 && duration < 1600000 )
			return true;
	}

	return false;
}

void radResetMachine()
{
	OUT_GPIO( RESET_OUT );
	CLR_GPIO( bRESET_OUT );
	DELAY( 1 << 25 );
	SET_GPIO( bRESET_OUT );
	INP_GPIO( RESET_OUT );
}

static u8 takeoverROM[ 256 ];

static const u8 takeoverROMPrefix[] = {
	0x78,                         // SEI
	0xA9, 0x00,                   // LDA #$00
	0x8D, 0x11, 0xD0,             // STA $D011 (blank display)
	0x8D, 0x20, 0xD0,             // STA $D020
	0x8D, 0x21, 0xD0,             // STA $D021
	0xA9, 0x35, 0x85, 0x01,       // LDA #$35 / STA $01
	0xA9, 0x2F, 0x85, 0x00        // LDA #$2F / STA $00
};

void radPrepareUltimaxTakeover( u8 physicalPort )
{
	// Materialise the exact 256-byte image generated from upstream RAD's
	// C64Side/ultimax_memcfg.a.  Keeping a complete table also keeps the timed
	// response loop identical to startWithUltimax(): one indexed byte load.
	for ( u32 i = 0; i < sizeof( takeoverROM ); i++ ) takeoverROM[ i ] = 0xEA;
	for ( u32 i = 0; i < sizeof( takeoverROMPrefix ); i++ )
		takeoverROM[ i ] = takeoverROMPrefix[ i ];
	// Immediate operand of LDA #$35 / STA $01 in takeoverROMPrefix. Keep the
	// source image readable and patch only the value selected by the caller.
	takeoverROM[ 13 ] = physicalPort;
	takeoverROM[ 0xFC ] = 0x00;
	takeoverROM[ 0xFD ] = 0xFF;
	takeoverROM[ 0xFE ] = 0x00;
	takeoverROM[ 0xFF ] = 0x00;

	CACHE_PRELOAD_INSTRUCTION_CACHE( (void *)radHijackCPUWithUltimax, 4096 );
	CACHE_PRELOAD_DATA_CACHE( &takeoverROM[ 0 ], 256, CACHE_PRELOADL2KEEP )
	FORCE_READ_LINEARa( (void *)radHijackCPUWithUltimax, 4096, 65536 );
	FORCE_READ_LINEAR32a( &takeoverROM, 256, 256 * 32 );
}

bool radHijackCPUWithUltimax()
{
	register u32 g2, g3;

	for ( u32 attempt = 0; attempt < 8; attempt++ )
	{
		u32 nCycles = 0;
		u32 nNOPs = 0;

		// Hold RESET, GAME and DMA low while the cartridge-facing pins are made
		// outputs.  Releasing RESET and DMA together below lets the C128's Z80
		// observe GAME, choose C64 mode and hand a known reset stream to the 8502.
		SET_GPIO( bLATCH_A_OE | bIRQ_OUT | bOE_Dx | bRW_OUT );
		INP_GPIO_RW();
		INP_GPIO_IRQ();
		OUT_GPIO( RESET_OUT );
		OUT_GPIO( GAME_OUT );
		OUT_GPIO( DMA_OUT );
		CLR_GPIO( bRESET_OUT | bGAME_OUT | bDMA_OUT );
		CLR_GPIO( bMPLEX_SEL );

		DELAY( 1 << 20 );

		WAIT_FOR_CPU_HALFCYCLE
		BEGIN_CYCLE_COUNTER
		WAIT_FOR_VIC_HALFCYCLE
		SET_GPIO( bRESET_OUT | bDMA_OUT );
		INP_GPIO( RESET_OUT );

		while ( nCycles++ < 1000000 )
		{
			WAIT_FOR_CPU_HALFCYCLE
			RESTART_CYCLE_COUNTER
			WAIT_UP_TO_CYCLE( WAIT_FOR_SIGNALS + TIMING_OFFSET_CBTD );
			g2 = read32( ARM_GPIO_GPLEV0 );

			SET_GPIO( bMPLEX_SEL );
			WAIT_UP_TO_CYCLE( WAIT_CYCLE_MULTIPLEXER );
			g3 = read32( ARM_GPIO_GPLEV0 );
			CLR_GPIO( bMPLEX_SEL );

			// Use RAD's dedicated high-byte comparator, exactly as upstream's
			// C128-safe startWithUltimax().  /ROMH is a PLA output and is not a
			// reliable substitute during the C128's Z80-to-8502 cartridge handoff.
			if ( ADDRESS_FFxx && CPU_READS_FROM_BUS )
			{
				const u8 data = takeoverROM[ ADDRESS0to7 ];
				register u32 dd = ( (u32)data ) << D0;
				write32( ARM_GPIO_GPCLR0,
				         ( D_FLAG & ( ~dd ) ) | bOE_Dx | bDIR_Dx );
				write32( ARM_GPIO_GPSET0, dd );
				SET_BANK2_OUTPUT
				WAIT_UP_TO_CYCLE( WAIT_CYCLE_READ );
				SET_GPIO( bOE_Dx | bDIR_Dx );

				if ( data == 0xEA )
					nNOPs++;
			}

			WAIT_FOR_VIC_HALFCYCLE

			if ( nNOPs > 12 )
			{
				// Every instruction in this region is a read-only NOP.  /DMA
				// therefore cannot catch the 8502 in the write-cycle hazard
				// documented in the C128 service manual.
				CLR_GPIO( bDMA_OUT );
				WAIT_FOR_CPU_HALFCYCLE
				WAIT_FOR_VIC_HALFCYCLE
				WAIT_FOR_CPU_HALFCYCLE
				WAIT_FOR_VIC_HALFCYCLE
				WAIT_FOR_CPU_HALFCYCLE
				WAIT_FOR_VIC_HALFCYCLE
				RESTART_CYCLE_COUNTER
				return true;
			}
		}
	}

	// Give the machine back a conventional cartridge-free reset if none of
	// the attempts reached the NOP stream.
	SET_GPIO( bLATCH_A_OE | bOE_Dx | bRW_OUT | bGAME_OUT | bDMA_OUT );
	INP_GPIO_RW();
	SET_BANK2_OUTPUT
	INP_GPIO( GAME_OUT );
	INP_GPIO( DMA_OUT );
	radResetMachine();
	return false;
}

bool radHijackCPU()
{
	register u32 g2;

	CLR_GPIO( bMPLEX_SEL );
	WAIT_FOR_CPU_HALFCYCLE
	BEGIN_CYCLE_COUNTER
	WAIT_FOR_VIC_HALFCYCLE

	// Spin until BA goes low, i.e. until the VIC-II announces a badline and
	// takes the bus away from the 6510 anyway. Asserting /DMA at that moment is
	// the clean handover point.
	//
	// The cycle cap stops us hanging forever when the screen is blanked and no
	// badlines ever occur. Reaching it means we never found a safe handover
	// point, so we report failure rather than asserting /DMA at an arbitrary
	// bus phase -- doing that with the 6510 mid-cycle risks bus contention
	// between the Pi and a CPU that has not yet tri-stated.
	// Several passes, so a machine that is still blanking its screen during
	// early KERNAL init gets another chance rather than failing outright.
	bool foundBadline = false;

	for ( u32 attempt = 0; attempt < 8 && !foundBadline; attempt++ )
	{
		u32 cycles = 0;
		do
		{
			WAIT_FOR_VIC_HALFCYCLE_EDGE
			RESTART_CYCLE_COUNTER
			WAIT_UP_TO_CYCLE( TIMING_BA_SIGNAL_AVAIL );
			g2 = read32( ARM_GPIO_GPLEV0 );
			cycles++;

			if ( !( g2 & bBA ) )
			{
				foundBadline = true;
				break;
			}
		} while ( cycles < 250000 );

		if ( !foundBadline )
			DELAY( 1 << 26 );		// ~0.5s, then look again
	}

	if ( !foundBadline )
		return false;

	WAIT_FOR_VIC_HALFCYCLE_EDGE
	RESTART_CYCLE_COUNTER

	// 80ns after the falling edge of PHI2.
	WAIT_UP_TO_CYCLE( TIMING_TRIGGER_DMA );
	OUT_GPIO( DMA_OUT );
	CLR_GPIO( bDMA_OUT );

	WAIT_FOR_CPU_HALFCYCLE
	WAIT_FOR_VIC_HALFCYCLE
	RESTART_CYCLE_COUNTER

	return true;
}

void radReleaseCPU()
{
	register u32 g2;

	WAIT_FOR_CPU_HALFCYCLE
	WAIT_FOR_VIC_HALFCYCLE
	RESTART_CYCLE_COUNTER

	SET_GPIO( bLATCH_A_OE | bOE_Dx | bRW_OUT | bDMA_OUT | bGAME_OUT );
	INP_GPIO_RW();
	SET_BANK2_OUTPUT
	INP_GPIO( GAME_OUT );
	INP_GPIO( DMA_OUT );
}

C64MachineType radDetectMachine()
{
	register u32 g2;
	u32 c128Votes = 0;

	// Authoritative current RAD probe.  After the Ultimax takeover the VIC-IIe
	// test register reads $FE or $FC; a C64 does not produce either value. This
	// probe now runs before timing calibration so C128 refresh can start before
	// the long write grid. Repeat it to tolerate the already-observed first
	// transfer loss and occasional buffered-port residue at the configured
	// baseline sample point. Requiring four votes prevents a single floating
	// C64 open-bus value from selecting C128 policy.
	for ( u32 i = 0; i < 16; i++ )
	{
		u8 y;
		RAD_SPEEK( 0xD030, y );
		if ( y == 0xFE || y == 0xFC ) c128Votes++;
	}

	WAIT_FOR_CPU_HALFCYCLE
	WAIT_FOR_VIC_HALFCYCLE
	RESTART_CYCLE_COUNTER

	return c128Votes >= 4 ? MACHINE_C128 : MACHINE_C64;
}

C64VideoStandard radDetectVideoStandard()
{
	register u32 g2;
	u16 curRasterLine  = 0;
	u16 maxRasterLine  = 0;
	u16 lastRasterLine = 0xFFFF;
	u32 wraps           = 0;

	BUS_RESYNC

	// Locate two frame wraps. Everything between them is one complete frame,
	// regardless of the raster line at which probing began. PAL reaches 311,
	// NTSC only 261 (6567R56A) or 262 (6567R8).
	//
	// Each sample uses high/low/high rather than the former low/high pair. A
	// pair which straddled line 255 could manufacture line 511 and falsely
	// identify an NTSC machine as PAL. Reads are still self-synchronising, as
	// they are everywhere else in this low-level probe. The cap is generous
	// enough for several frames and bounds a frozen raster counter. As with all
	// post-acquire bus operations, this still assumes the PHI2 liveness check
	// completed immediately beforehand.
	for ( u32 attempt = 0; attempt < 32768 && wraps < 2; attempt++ )
	{
		u8 hiBefore = 0, lo = 0, hiAfter = 0;

		RAD_SPEEK( 0xD011, hiBefore );
		RAD_SPEEK( 0xD012, lo );
		RAD_SPEEK( 0xD011, hiAfter );

		if ( ( ( hiBefore ^ hiAfter ) & 0x80 ) != 0 )
			continue;

		curRasterLine = (u16)( lo | ( ( hiAfter & 0x80 ) ? 0x100 : 0 ) );
		if ( curRasterLine >= 312 || curRasterLine == lastRasterLine )
			continue;

		if ( lastRasterLine != 0xFFFF && curRasterLine < lastRasterLine )
		{
			wraps++;
			if ( wraps == 1 )
				maxRasterLine = 0;
		}

		if ( curRasterLine > maxRasterLine )
			maxRasterLine = curRasterLine;

		lastRasterLine = curRasterLine;
	}

	// If the bounded scan did not encompass a complete frame, retain the old
	// best-effort behaviour using whatever coherent samples were obtained.
	// A normally advancing VIC always reaches two wraps well inside the cap.
	if ( maxRasterLine >= 300 )
		return VIDEO_PAL;

	return maxRasterLine >= 262 ? VIDEO_NTSC_R8 : VIDEO_NTSC_R56A;
}
