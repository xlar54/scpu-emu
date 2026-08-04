/*
   SCPU-EMU - CMD SuperCPU emulation for the C64/C128 using a RAD Expansion Unit
   Copyright (c) 2026 SCPU-EMU contributors

   Start-up sequence.

   ROM policy. Two paths, in order of preference:

     1. Files on the SD card under SCPU/. Use this to run a real SuperCPU ROM
        image (which brings JiffyDOS and the SuperCPU DOS with it), or to pin a
        specific KERNAL revision. No ROM images ship with this project.
     2. Snapshot the ROMs off the live machine over the bus. Needs no files at
        all and gives you the KERNAL actually fitted to the computer. The
        character ROM cannot be captured this way -- exposing it needs CHAREN
        low and with the 6510 held off the bus nothing can rewrite its port --
        so chargen stays blank unless supplied as a file. That only matters for
        programs that read the character set through the CPU; the VIC-II reads
        it directly on the C64 side and is unaffected.

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
#include "boot.h"
#include <circle/actled.h>
#include <circle/startup.h>

#include "../Common/config.h"
#include "../Common/helpers.h"
#include "../Bus/RAD/gpio_defs.h"
#include "../Bus/RAD/lowlevel_arm64.h"
#include "../Bus/RAD/bus_timing.h"
#include "../Bus/RAD/rad_bus.h"
#include "../SuperCPU/supercpu.h"

// Defined in main.cpp, which owns the FAT filesystem object.
extern "C" void radUnmountFileSystem();

static bool radBusButtonPressed();
static bool radBusHardwareResetPressed();
static void scpuWaitForButtonThenReboot();

// Frame hook: reboot the Pi when the RAD's button is pressed, so the card can
// be swapped and retried without power-cycling anything.
// Runs once per emulated frame.
//
//   LEFT button  (GPIO 3)      -> reboot the Pi, so a freshly written card is
//                                 picked up without a power cycle
//   RIGHT button (hardware)    -> wired straight to the C64's /RESET line and
//                                 invisible to the Pi as a button, but we can
//                                 see the line go low. Treat it as a reset of
//                                 the emulated machine, which is what it would
//                                 do on a real SuperCPU.
static bool scpuCheckButton( void *ctx )
{
	CSuperCPU *scpu = (CSuperCPU *)ctx;

	if ( radBusButtonPressed() )
		return true;					// stop the run loop; caller reboots

	if ( radBusHardwareResetPressed() )
	{
		// Wait for release so one press is one reset, then restart the
		// emulated machine from its reset vector. The Pi keeps running and the
		// bus stays ours -- no need to renegotiate the handover.
		while ( radBusHardwareResetPressed() )
			;
		if ( scpu ) scpu->reset();
	}

	return false;
}

static CRADBus   radBus;
static CActLED   actLED;
static CSuperCPU superCPU;

static u8 romBuffer[ C64_BASIC_SIZE ];

static bool radBusButtonPressed() { return radBus.buttonPressed(); }
static bool radBusHardwareResetPressed() { return radBus.hardwareResetPressed(); }

// Park here on any failure. Without this a failed start-up never reaches the
// run loop, so nothing ever polls the button and the only way out is a power
// cycle -- which is exactly what the button exists to avoid.
static void scpuWaitForButtonThenReboot()
{
	while ( !radBus.buttonPressed() )
		;
	reboot();
}

bool scpuBootLoadConfig( CLogger *logger )
{
	// The RAD's bus timings depend on the Raspberry Pi model and the machine.
	// Start from the built-in defaults, then let scpu.cfg override.
	setDefaultTimings( AUTO_TIMING_RPI3PLUS_C64C128 );

	bool haveConfig = readConfig( logger, SCPU_DRIVE, SCPU_CONFIG_FILE ) != 0;
	if ( !haveConfig )
		logger->Write( "SCPU", LogWarning,
		               "no %s found, using built-in timings", SCPU_CONFIG_FILE );

	// Snapshot into the cache-resident block the DMA primitives read from.
	initBusTiming();
	return haveConfig;
}

static bool loadROMFile( CLogger *logger, const char *name, u8 *dst, u32 expected )
{
	u32 size = 0;

	// 'expected' is also the buffer capacity, so readFile refuses anything
	// larger before reading a single byte. That matters because the SuperCPU
	// DOS images are 128K and dropping one in as kernal.rom by mistake would
	// otherwise overrun an 8K buffer.
	if ( !readFile( logger, SCPU_DRIVE, name, dst, &size, expected ) )
		return false;

	if ( size != expected )
	{
		logger->Write( "SCPU", LogWarning, "%s is %u bytes, expected %u -- ignored",
		               name, size, expected );
		return false;
	}

	logger->Write( "SCPU", LogNotice, "loaded %s", name );
	return true;
}


// Screen codes -> ASCII, enough to read what the emulated KERNAL has drawn.
static char scpuScreenChar( u8 c )
{
	if ( c == 0x20 || c == 0x00 ) return ' ';
	if ( c >= 0x01 && c <= 0x1A ) return (char)( 'A' + c - 1 );
	if ( c >= 0x30 && c <= 0x39 ) return (char)( '0' + c - 0x30 );
	if ( c == 0x2E ) return '.';
	if ( c == 0x2A ) return '*';
	if ( c == 0x2C ) return ',';
	if ( c == 0x2F ) return '/';
	if ( c == 0x28 ) return '(';
	if ( c == 0x29 ) return ')';
	return '?';
}

// Report what the emulator believes the screen contains, straight out of shadow
// RAM. This separates two very different faults that look identical on the C64:
// a CPU core that is running wrong, versus a core that is running correctly
// while the mirroring fails to get its output across the bus.
static void scpuDumpEmulatedScreen( CLogger *logger, CSuperCPU &scpu )
{
	const u8 *ram = scpu.memory().m_RAM;

	logger->Write( "SCPU", LogNotice, "--- emulated screen, from SHADOW RAM $0400 ---" );

	for ( u32 row = 0; row < 25; row++ )
	{
		char line[ 41 ];
		bool blank = true;
		for ( u32 col = 0; col < 40; col++ )
		{
			line[ col ] = scpuScreenChar( ram[ 0x0400 + row * 40 + col ] );
			if ( line[ col ] != ' ' ) blank = false;
		}
		line[ 40 ] = 0;
		if ( !blank )
			logger->Write( "SCPU", LogNotice, "|%s|", line );
	}

	logger->Write( "SCPU", LogNotice,
	               "PC=$%04X  cycles=%u  memtop($37/$38)=$%02X%02X",
	               (unsigned)scpu.cpu()->pc(),
	               (unsigned)scpu.cpu()->cycles(),
	               ram[ 0x38 ], ram[ 0x37 ] );

	logger->Write( "SCPU", LogNotice,
	               "bus: ioReads=%u ioWrites=%u ramWrites=%u mirrored=%u",
	               (unsigned)scpu.memory().m_IOReads,
	               (unsigned)scpu.memory().m_IOWrites,
	               (unsigned)scpu.memory().m_RamWrites,
	               (unsigned)scpu.writeBuffer().m_BytesFlushed );
}

void scpuBootRun( CLogger *logger )
{
	logger->Write( "SCPU", LogNotice, "SCPU-EMU starting" );
	logger->Write( "SCPU", LogNotice, "if you are reading this on HDMI, the Pi"
	               " firmware and our kernel both loaded correctly" );

	// One ACT-LED blink per milestone, so there is a signal even with no
	// monitor attached. The Pi's own error codes are all >= 3 flashes, so a
	// single blink cannot be mistaken for one.
	actLED.Blink( 1 );

	// Optional ROM images. Anything not supplied here is taken off the machine.
	if ( loadROMFile( logger, SCPU_ROM_DIR "basic.rom", romBuffer, C64_BASIC_SIZE ) )
		superCPU.setBasicROM( romBuffer );

	if ( loadROMFile( logger, SCPU_ROM_DIR "kernal.rom", romBuffer, C64_KERNAL_SIZE ) )
		superCPU.setKernalROM( romBuffer );

	if ( loadROMFile( logger, SCPU_ROM_DIR "chargen.rom", romBuffer, C64_CHARROM_SIZE ) )
		superCPU.setCharROM( romBuffer );

	// From here on the SD card is not touched again: the FAT driver takes
	// interrupts and allocates, neither of which is safe once we are holding
	// DMA and driving the bus to a cycle budget.
	radUnmountFileSystem();
	actLED.Blink( 2 );		// milestone: ROMs handled, SD released

	radBus.setLogger( logger );

	if ( !superCPU.init( &radBus, SCPU_CORE_6502 ) )
	{
		// init() releases /DMA on every failure path, so the C64 is running its
		// own CPU again and is usable -- it simply has no accelerator.
		logger->Write( "SCPU", LogError,
		               "startup failed: no machine detected, no safe DMA handover "
		               "point found, or no usable KERNAL. The C64 has been released "
		               "and continues to run normally." );
		logger->Write( "SCPU", LogNotice,
		               "press the RAD button to reboot and retry." );
		scpuWaitForButtonThenReboot();
	}

	actLED.Blink( 3 );		// milestone: bus acquired

	const C64Signals &sig = radBus.signals();
	logger->Write( "SCPU", LogNotice, "attached to %s, %s",
	               sig.machine == MACHINE_C128 ? "C128" : "C64",
	               sig.video == VIDEO_PAL ? "PAL" : "NTSC" );

	logger->Write( "SCPU", LogNotice, "running bus self-test..." );
	if ( !radBus.selfTest() )
	{
		// Do not start the core over a bus we know is unreliable. It produces a
		// dramatic display and no information, and it leaves the machine
		// unusable. Hand the C64 back instead: it returns to its own CPU and
		// the log above stays on screen to be read.
		logger->Write( "SCPU", LogError,
		               "BUS SELF-TEST FAILED -- not starting the CPU core." );
		logger->Write( "SCPU", LogError,
		               "Releasing /DMA; the C64 gets its own CPU back and should "
		               "behave normally. Fix the failing operation above first." );
		radBus.release();
		actLED.Blink( 5 );		// distinct from the 1-4 milestone blinks
		logger->Write( "SCPU", LogNotice,
		               "press the RAD button to reboot and retry." );
		scpuWaitForButtonThenReboot();
	}

	actLED.Blink( 4 );		// milestone: handing over to the CPU core
	logger->Write( "SCPU", LogNotice, "starting the CPU core" );
	logger->Write( "SCPU", LogNotice,
	               "buttons: LEFT = reboot the Pi, RIGHT = reset the emulated C64" );

	// Run about two seconds of emulated time, then report what the emulator
	// thinks it has drawn before handing over to the free-running loop.
	for ( u32 i = 0; i < 120; i++ )
		superCPU.runFrame();

	scpuDumpEmulatedScreen( logger, superCPU );

	superCPU.setFrameHook( scpuCheckButton, &superCPU );
	superCPU.run();

	// The button was pressed. Release the machine and restart the Pi so a fresh
	// kernel8.img on the card is picked up.
	logger->Write( "SCPU", LogNotice, "button pressed -- releasing /DMA and rebooting" );
	radBus.release();
	actLED.Blink( 2 );
	reboot();
}
