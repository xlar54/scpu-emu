/*
   SCPU-EMU - CMD SuperCPU emulation for the C64/C128 using a RAD Expansion Unit

   CRADBus - IC64Bus over the real expansion port.

   Copyright (c) 2026 SCPU-EMU contributors
   Built on the RAD Expansion Unit framework
   Copyright (c) 2022, 2023 Carsten Dachsbacher <frenetic@dachsbacher.de>

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
#include <circle/logger.h>

#include "gpio_defs.h"
#include "lowlevel_arm64.h"
#include "bus_timing.h"

static u64 armCycleCounter;

#include "rad_lowlevel.h"
#include "rad_bus.h"
#include "cpu_hijack.h"

CRADBus::CRADBus()
	: m_Reads( 0 ), m_Writes( 0 ), m_BurstWrites( 0 ), m_Acquired( false ), m_Logger( 0 )
{
	m_Signals.machine       = MACHINE_UNKNOWN;
	m_Signals.video         = VIDEO_PAL;
	m_Signals.gameAsserted  = false;
	m_Signals.exromAsserted = false;
}

#define RADLOG( ... ) do { if ( m_Logger ) m_Logger->Write( "RADbus", LogNotice, __VA_ARGS__ ); } while ( 0 )

bool CRADBus::acquire()
{
	// Order matters, and it used to be wrong. We reset the machine first and
	// only then waited for a valid clock -- so on a cold power-on, where the
	// Pi and the C64 come up together, the reset landed while the C64 was still
	// powering up. On real hardware that showed as an eight second struggle to
	// get a stable PHI2 measurement, and a takeover that happened before the
	// KERNAL had settled $01 to $37. A warm restart worked first time because
	// the machine was already running.
	//
	// So: confirm it is alive, THEN reset it, THEN confirm it came back, THEN
	// give the KERNAL room to finish before taking the bus.

	RADLOG( "1/6 waiting for the C64 to power up..." );
	if ( !radWaitForMachineRunning() )
	{
		u64 measured = radMeasureMachineRate();
		if ( measured == 0 )
			RADLOG( "    FAILED: PHI2 never changed state. Check the cartridge is"
			        " seated and the C64 is powered on." );
		else
			RADLOG( "    FAILED: 1000 C64 cycles measured %u ARM cycles; expected"
			        " 1200000-1600000. If this is a clean multiple or fraction of"
			        " the expected value, arm_freq does not match the timings.",
			        (unsigned)measured );
		return false;
	}

	RADLOG( "2/6 machine is alive; resetting it for a clean start..." );
	radResetMachine();

	RADLOG( "3/6 waiting for it to come back after reset..." );
	if ( !radWaitForMachineRunning() )
	{
		RADLOG( "    FAILED: no clock after reset." );
		return false;
	}

	// The KERNAL has to get far enough to set $01 = $37, which is what puts
	// BASIC, KERNAL and I/O where our emulated banking expects them. A cold
	// boot including the RAM test takes a good couple of seconds.
	RADLOG( "4/6 letting the KERNAL boot (3s)..." );
	idleHold( 3 );

	RADLOG( "5/6 waiting for a badline to assert /DMA..." );
	if ( !radHijackCPU() )
	{
		RADLOG( "    FAILED: no badline seen. Is the screen blanked? /DMA was NOT"
		        " asserted; the C64 still has its own CPU." );
		return false;
	}

	RADLOG( "6/6 /DMA asserted -- we are bus master. Probing the machine..." );
	m_Signals.machine = radDetectMachine();
	m_Signals.video   = radDetectVideoStandard();
	m_Acquired        = true;

	RADLOG( "    %s, %s -- bus acquired",
	        m_Signals.machine == MACHINE_C128 ? "C128" : "C64",
	        m_Signals.video == VIDEO_PAL ? "PAL" : "NTSC" );

	return true;
}

void CRADBus::release()
{
	if ( !m_Acquired )
		return;

	radReleaseCPU();
	m_Acquired = false;
}

bool CRADBus::selfTest()
{
	register u32 g2;
	u8 v;
	u32 romErr = 0, wrErr = 0, burstErr = 0;

	// --- is the read path alive at all? ------------------------------------
	// $D012 is the VIC-II raster counter. On a running machine it MUST change
	// between samples taken a few hundred microseconds apart -- the beam does
	// not stop. If every sample is identical we are not reading the bus at all,
	// which is a completely different fault from reading the wrong address.
	{
		u8 r[ 6 ];
		for ( u32 i = 0; i < 6; i++ )
		{
			RAD_SPEEK( 0xD012, r[ i ] );
			DELAY( 1 << 14 );
		}
		m_Reads += 6;

		bool changed = false;
		for ( u32 i = 1; i < 6; i++ )
			if ( r[ i ] != r[ 0 ] ) changed = true;

		RADLOG( "  self-test: $D012 raster %02X %02X %02X %02X %02X %02X -- %s",
		        r[0], r[1], r[2], r[3], r[4], r[5],
		        changed ? "advancing, reads are live"
		                : "FROZEN: the read path is returning nothing" );
	}

	// --- reads, against values we already know -----------------------------
	// The machine's own ROMs are banked in ($01 = $37 after a KERNAL start), so
	// these addresses have documented contents on every C64 ever made. If these
	// come back wrong, reads are broken and nothing else can be trusted.
	RAD_SPEEK( 0xFFFC, v ); if ( v != 0xE2 ) romErr++;	// reset vector lo
	RAD_SPEEK( 0xFFFD, v ); if ( v != 0xFC ) romErr++;	// reset vector hi -> $FCE2
	RAD_SPEEK( 0xA000, v ); if ( v != 0x94 ) romErr++;	// BASIC cold start lo
	RAD_SPEEK( 0xA001, v ); if ( v != 0xE3 ) romErr++;	// BASIC cold start hi -> $E394
	m_Reads += 4;

	// Informational only. The C64's PLA does not select BASIC/KERNAL while the
	// 6510 is off the bus, so these ranges return open bus to a DMA device --
	// which is exactly why an REU only ever transfers RAM. Nothing depends on
	// it: KERNAL and BASIC come from the SD card.
	RADLOG( "  self-test: ROM reads  %s (%u/4) -- expected to fail; the PLA"
	        " deselects ROM during DMA, so ROM is unreadable this way",
	        romErr ? "unreadable" : "readable!", romErr );

	if ( romErr )
	{
		u8 a, b, c, d;
		RAD_SPEEK( 0xFFFC, a ); RAD_SPEEK( 0xFFFD, b );
		RAD_SPEEK( 0xA000, c ); RAD_SPEEK( 0xA001, d );
		RADLOG( "    got $FFFC=%02X $FFFD=%02X (expect E2 FC), "
		        "$A000=%02X $A001=%02X (expect 94 E3)",
		        a, b, c, d );
	}

	// --- address line integrity --------------------------------------------
	// Every address that has worked so far has A13 low ($0334, $D012); every
	// one that failed has A13 high ($A000, $FFFC). That is suspicious enough to
	// rule out before blaming banking.
	//
	// These six addresses are RAM under any non-Ultimax configuration, so
	// banking cannot influence the result. Each differs from the others in
	// exactly one address line (A10..A15). If a line is stuck, two of them
	// collide and the earlier value is overwritten by the later one.
	{
		static const u16 aline[ 6 ] = { 0x0400, 0x0800, 0x1000, 0x2000, 0x4000, 0x8000 };
		u32 aErr = 0;

		for ( u32 i = 0; i < 6; i++ )
			{ RAD_SPOKE( aline[ i ], (u8)( 0xE0 | i ) ); }

		u8 got[ 6 ];
		for ( u32 i = 0; i < 6; i++ )
			{ RAD_SPEEK( aline[ i ], got[ i ] ); if ( got[ i ] != (u8)( 0xE0 | i ) ) aErr++; }

		m_Writes += 6; m_Reads += 6;

		RADLOG( "  self-test: addr lines %s -- 0400=%02X 0800=%02X 1000=%02X "
		        "2000=%02X 4000=%02X 8000=%02X (want E0 E1 E2 E3 E4 E5)",
		        aErr ? "FAIL" : "ok",
		        got[0], got[1], got[2], got[3], got[4], got[5] );

		if ( aErr )
			RADLOG( "    an address line is not reaching the C64; two of these"
			        " addresses are aliasing onto each other" );
	}

	// --- is anything actually banked in? -----------------------------------
	// Decisive test for the ROM failures. Writes always fall through to the RAM
	// underneath, whatever is banked in for reads. So:
	//   read back the ROM byte  -> ROM is banked in, reads are fine
	//   read back what we wrote -> ROM is NOT banked in; $01 is not $37 and we
	//                              have been reading blank RAM all along
	{
		u8 a, b;
		RAD_SPOKE( 0xA000, 0xA5 );
		RAD_SPEEK( 0xA000, a );
		RAD_SPOKE( 0xE000, 0x5A );
		RAD_SPEEK( 0xE000, b );
		m_Writes += 2; m_Reads += 2;

		RADLOG( "  self-test: banking   $A000 reads %02X after writing A5,"
		        " $E000 reads %02X after writing 5A", a, b );

		if ( a == 0xA5 || b == 0x5A )
			RADLOG( "    -> RAM is visible where ROM should be: the halted 6510"
			        " did not leave $01 at $37" );
		else if ( a == 0x00 && b == 0x00 )
			RADLOG( "    -> neither ROM nor the RAM we just wrote: this address"
			        " range is not responding at all" );
		else
			RADLOG( "    -> ROM is banked in and readable" );
	}

	// --- single writes, read back ------------------------------------------
	// $0334-$033F is the tail of the cassette buffer: RAM on every machine and
	// not used while we hold the bus.
	static const u8 patterns[] = { 0x00, 0xFF, 0x55, 0xAA, 0x01, 0x80 };

	for ( u32 i = 0; i < 6; i++ )
	{
		RAD_SPOKE( (u16)( 0x0334 + i ), patterns[ i ] );
	}
	u8 rb[ 6 ];
	for ( u32 i = 0; i < 6; i++ )
	{
		RAD_SPEEK( (u16)( 0x0334 + i ), rb[ i ] );
		if ( rb[ i ] != patterns[ i ] ) wrErr++;
	}
	m_Writes += 6; m_Reads += 6;

	// Report the bytes, not just a count. A bit that is always wrong points at
	// a data line; a byte that differs run to run points at timing margin.
	RADLOG( "  self-test: single r/w  %s (%u/6) got %02X %02X %02X %02X %02X %02X"
	        " want 00 FF 55 AA 01 80",
	        wrErr ? "FAIL" : "ok", wrErr,
	        rb[0], rb[1], rb[2], rb[3], rb[4], rb[5] );

	// Repeat it a few times: an intermittent fault is a timing problem and a
	// consistent one is not, and that distinction decides what to change next.
	{
		u32 reruns = 0, runsFailed = 0;
		for ( reruns = 0; reruns < 20; reruns++ )
		{
			u32 e = 0;
			for ( u32 i = 0; i < 6; i++ ) { RAD_SPOKE( (u16)( 0x0334 + i ), patterns[ i ] ); }
			for ( u32 i = 0; i < 6; i++ )
			{
				RAD_SPEEK( (u16)( 0x0334 + i ), v );
				if ( v != patterns[ i ] ) e++;
			}
			if ( e ) runsFailed++;
		}
		m_Writes += 120; m_Reads += 120;

		RADLOG( "  self-test: single r/w repeated 20x -- %u runs had errors%s",
		        runsFailed,
		        runsFailed == 0 ? " (stable)"
		                        : ( runsFailed >= 18 ? " (consistent: not timing)"
		                                             : " (INTERMITTENT: timing margin)" ) );
	}

	// --- burst writes, read back -------------------------------------------
	// This is the path the VIC mirroring uses, and it is entirely separate from
	// single writes: the address latch and transceiver stay configured across
	// the whole run, with a mid-burst resync when a badline starts.
	C64BusWrite burst[ 6 ];
	for ( u32 i = 0; i < 6; i++ )
	{
		burst[ i ].addr  = (u16)( 0x033A + i );
		burst[ i ].value = (u8)( patterns[ 5 - i ] );
	}
	writeBurst( burst, 6 );

	for ( u32 i = 0; i < 6; i++ )
	{
		RAD_SPEEK( burst[ i ].addr, v );
		if ( v != burst[ i ].value ) burstErr++;
	}
	m_Reads += 6;

	RADLOG( "  self-test: burst write %s (%u/6 wrong)",
	        burstErr ? "FAIL" : "ok", burstErr );

	if ( burstErr && !wrErr )
		RADLOG( "    single writes work but bursts do not -- the mirroring path"
		        " is the problem, not the bus itself" );

	// romErr deliberately excluded: see the note above.
	return ( wrErr + burstErr ) == 0;
}

bool CRADBus::buttonPressed()
{
	// The button is GPIO 3, which the 74LVC257 shares with address bit A7. The
	// button is only the value present while MPLEX_SEL is LOW, so the select
	// line has to be driven before sampling -- reading it blind samples A7
	// instead and reports phantom presses.
	//
	// That is not academic: it used to cut the post-reset settle short, so the
	// bus was taken while the KERNAL still had the screen blanked, and with no
	// badlines the handover could never find a safe point.
	CLR_GPIO( bMPLEX_SEL );

	// Let the multiplexer settle, then require two agreeing samples.
	DELAY( 64 );
	u32 a = read32( ARM_GPIO_GPLEV0 );
	DELAY( 64 );
	u32 b = read32( ARM_GPIO_GPLEV0 );

	return ( !( a & bBUTTON ) && !( b & bBUTTON ) );
}

bool CRADBus::hardwareResetPressed()
{
	// RESET_OUT is left as an input after radResetMachine(), so a low level
	// here means someone else is asserting reset -- the RAD's own reset button.
	u32 g2 = read32( ARM_GPIO_GPLEV0 );
	return CPU_RESET ? true : false;
}

void CRADBus::idleHold( u32 seconds )
{
	// Uninterruptible. This is used for the post-reset settle, where cutting the
	// wait short is exactly the failure we are avoiding. DELAY( 1 << 27 ) is
	// roughly one second at 1.4GHz.
	for ( u32 i = 0; i < seconds; i++ )
		DELAY( 1 << 27 );
}

void CRADBus::idleHoldInterruptible( u32 seconds )
{
	// For observation windows, where responding to the button is worth more
	// than the exact duration.
	for ( u32 i = 0; i < seconds * 8; i++ )
	{
		DELAY( 1 << 24 );
		if ( buttonPressed() )
			return;
	}
}

void CRADBus::signalAlive()
{
	register u32 g2;

	// Eight colours, three passes, about a third of a second each -- roughly
	// eight seconds in total.
	//
	// The first version of this used DELAY( 1 << 22 ), which is about 36ms per
	// colour: the whole sequence finished in a fifth of a second and was easy to
	// miss entirely, especially on a display that is not stable. A diagnostic
	// nobody can see is worse than none, because "no flash" then gets read as
	// "writes are broken".
	static const u8 colours[] = { 2, 7, 5, 13, 14, 3, 1, 0 };

	for ( u32 pass = 0; pass < 3; pass++ )
	{
		for ( u32 i = 0; i < 8; i++ )
		{
			RAD_SPOKE( 0xD020, colours[ i ] );
			DELAY( 1 << 25 );
		}
	}

	m_Writes += 24;
}

u8 CRADBus::read( u16 addr )
{
	register u32 g2;
	u8 v = 0xFF;

	RAD_SPEEK( addr, v );
	m_Reads++;
	return v;
}

void CRADBus::write( u16 addr, u8 value )
{
	register u32 g2;

	RAD_SPOKE( addr, value );
	m_Writes++;
}

void CRADBus::writeBurst( const C64BusWrite *writes, u32 count )
{
	if ( count == 0 )
		return;

	register u32 g2;

	// Configuring the address latch and bus transceiver once for the whole run
	// is what makes this roughly one C64 cycle per byte instead of the two or
	// three a sequence of individual write() calls costs.
	busBeginBurstWrites( g2 );
	BUS_RESYNC

	for ( u32 i = 0; i < count; i++ )
		RAD_BURST_POKE( writes[ i ].addr, writes[ i ].value );

	busEndBurstWrites( g2 );

	m_BurstWrites += count;
}

void CRADBus::readBlock( u16 addr, u8 *dst, u32 length )
{
	register u32 g2;

	for ( u32 i = 0; i < length; i++ )
	{
		u8 v = 0xFF;
		RAD_SPEEK( (u16)( addr + i ), v );
		dst[ i ] = v;
	}

	m_Reads += length;
}

bool CRADBus::irqAsserted()
{
	// /IRQ is a direct pin on the level shifter, readable whenever we have it
	// configured as an input (which the hijack sequence leaves it as).
	u32 g2 = read32( ARM_GPIO_GPLEV0 );
	return CPU_IRQ_LOW ? true : false;
}

bool CRADBus::nmiAsserted()
{
	// /NMI shares its GPIO with address bit A4 through the multiplexer, and is
	// the value present while MPLEX_SEL is low -- which is where the bus is
	// left outside of an access.
	u32 g2 = read32( ARM_GPIO_GPLEV0 );
	return CPU_NMI_LOW ? true : false;
}

u64 CRADBus::hostCycles()
{
	u64 c;
	READ_CYCLE_COUNTER( c );
	return c;
}

u16 CRADBus::rasterLine()
{
	register u32 g2;
	u8 lo = 0, hi = 0;

	RAD_SPEEK( 0xD012, lo );
	RAD_SPEEK( 0xD011, hi );
	m_Reads += 2;

	return (u16)( lo | ( ( hi & 0x80 ) ? 0x100 : 0 ) );
}
