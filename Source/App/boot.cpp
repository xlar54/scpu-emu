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
static void radBusSampleInterrupts( bool &irq, bool &nmi );

// Bare metal: no snprintf. Enough formatter for the freeze dumps.
static int scpuHexByte( char *dst, u8 v )
{
	static const char *h = "0123456789ABCDEF";
	dst[ 0 ] = h[ v >> 4 ]; dst[ 1 ] = h[ v & 15 ]; dst[ 2 ] = ' ';
	return 3;
}
static void scpuWaitForButtonThenReboot();
static CLogger *s_Logger;

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
// Sampled every ~50 frames, so the freeze dump can report the CIA state as it
// was BEFORE the reset button was pressed. The button is wired to the C64's
// physical /RESET line, so by the time the dump reads the CIAs they have
// already been reset -- DDR reads $00 regardless of what it was, which
// contaminated several rounds of serial forensics before this existed.
static u8  s_PreResetDD00 = 0, s_PreResetDD02 = 0;
static u32 s_PreResetSampleFrame = 0;

static bool scpuCheckButton( void *ctx )
{
	CSuperCPU *scpu = (CSuperCPU *)ctx;

	if ( scpu && ( ++s_PreResetSampleFrame % 50 ) == 0 )
	{
		s_PreResetDD00 = scpu->memory().read8( 0xDD00 );
		s_PreResetDD02 = scpu->memory().read8( 0xDD02 );
	}

	if ( radBusButtonPressed() )
		return true;					// stop the run loop; caller reboots

	if ( radBusHardwareResetPressed() )
	{
		// Before resetting, say where the machine was -- and what it was
		// doing. Three freezes in a row sampled the identical PC, which no
		// scattered busy-wait produces; either the stack is corrupt (an RTS
		// storm cycles through one or two addresses) or the code itself is.
		// The stack bytes, the code around the PC as the CPU actually sees it,
		// and the last I/O accesses decide between those from one photo.
		if ( s_Logger && scpu && scpu->cpu() )
		{
			const u32 pc = (u32)scpu->cpu()->pc();
			const u16 sp = scpu->cpu()->stackPointer();

			s_Logger->Write( "SCPU", LogNotice,
			               "reset pressed: PC=$%06X S=$%04X cycles=%u",
			               (unsigned)pc, (unsigned)sp,
			               (unsigned)scpu->cpu()->cycles() );

			// The interrupt picture, which is what an idle-loop freeze is
			// about: P says whether IRQs are masked, the taken-counters say
			// whether delivery ever stopped, and the live line sample says
			// whether the hardware is even asking.
			if ( CW65C816 *core = scpu->core65816() )
			{
				bool liveIRQ = false, liveNMI = false;
				radBusSampleInterrupts( liveIRQ, liveNMI );
				s_Logger->Write( "SCPU", LogNotice,
				               "  P=$%02X (I=%u) E=%u  irqsTaken=%u nmisTaken=%u  line: IRQ=%u NMI=%u",
				               core->m_P, ( core->m_P & W65_I ) ? 1 : 0,
				               core->m_E ? 1 : 0,
				               (unsigned)core->m_IRQsTaken,
				               (unsigned)core->m_NMIsTaken,
				               liveIRQ ? 1 : 0, liveNMI ? 1 : 0 );
			}

			// 16 stack bytes upward from S (the return addresses).
			char line[ 128 ]; int n = 0;
			for ( u32 i = 1; i <= 16; i++ )
				n += scpuHexByte( line + n,
				               scpu->memory().m_RAM[ 0x0100 | ( ( sp + i ) & 0xFF ) ] );
			line[ n ] = 0;
			s_Logger->Write( "SCPU", LogNotice, "  stack: %s", line );

			// The code around the PC, read the way the CPU reads it -- through
			// the live map, so a corrupted shadow shows itself here.
			if ( ( pc & 0xF000 ) != 0xD000 )
			{
				n = 0;
				for ( int i = -4; i < 12; i++ )
					n += scpuHexByte( line + n,
					               scpu->memory().read8( (u16)( pc + i ) ) );
				line[ n ] = 0;
				s_Logger->Write( "SCPU", LogNotice, "  code@PC-4: %s", line );
			}

			// Last 16 I/O accesses, oldest first: rWADDR=VAL.
			// The CIA conversation, which the idle loop cannot flood: the DDR
			// and interrupt-mask setup, then every ATN/CLK/DATA toggle. This is
			// the serial transaction itself, however long ago it stalled.
			for ( u32 row = 0; row < 4; row++ )
			{
				n = 0;
				for ( u32 i = 0; i < 16; i++ )
				{
					const u32 e = scpu->memory().m_CIALog[ ( scpu->memory().m_CIALogPos + row * 16 + i ) & 63 ];
					line[ n++ ] = ( e >> 24 ) ? 'w' : 'r';
					n += scpuHexByte( line + n, (u8)( ( e >> 8 ) & 0xFF ) ) - 1;
					n += scpuHexByte( line + n, (u8)( e & 0xFF ) ) - 1;
					line[ n++ ] = '=';
					n += scpuHexByte( line + n, (u8)( ( e >> 16 ) & 0xFF ) );
				}
				line[ n ] = 0;
				s_Logger->Write( "SCPU", LogNotice, "  cia%u: %s", row, line );
			}

			// The same entries as INTERVALS, in REAL microseconds (ARM cycles
			// / 1400) -- the pulse widths and gaps the DRIVE actually
			// experienced, including every stall the emulated-cycle view
			// cannot see. An ACK shorter than ~20us or a ready-gap beyond
			// ~200us names the desync directly. The ring is edge-compressed,
			// so these entries are transitions, not poll spam.
			n = 0;
			u32 shown = 0;
			for ( u32 i = 1; i < 64 && shown < 16; i++ )
			{
				const u32 idx  = ( scpu->memory().m_CIALogPos + 64 - i ) & 63;
				const u32 prev = ( scpu->memory().m_CIALogPos + 64 - i - 1 ) & 63;
				const u32 e = scpu->memory().m_CIALog[ idx ];
				u32 dt = ( scpu->memory().m_CIALogCyc[ idx ] - scpu->memory().m_CIALogCyc[ prev ] )
				       / ( SCPU_ARM_CLOCK_HZ / 1000000u );
				if ( dt > 99999 ) dt = 99999;
				line[ n++ ] = ( e >> 24 ) ? 'w' : 'r';
				n += scpuHexByte( line + n, (u8)( ( e >> 8 ) & 0xFF ) ) - 1;
				n += scpuHexByte( line + n, (u8)( e & 0xFF ) ) - 1;
				line[ n++ ] = '=';
				n += scpuHexByte( line + n, (u8)( ( e >> 16 ) & 0xFF ) ) - 1;
				line[ n++ ] = '+';
				// decimal microseconds
				char tmp[ 8 ]; int tn = 0;
				do { tmp[ tn++ ] = (char)( '0' + dt % 10 ); dt /= 10; } while ( dt );
				while ( tn ) line[ n++ ] = tmp[ --tn ];
				line[ n++ ] = ' ';
				shown++;
			}
			line[ n ] = 0;
			s_Logger->Write( "SCPU", LogNotice, "  ciaT: %s", line );

			// Live serial line state, read once, non-destructively ($DD0D is
			// deliberately NOT read -- reading the ICR clears it).
			if ( scpu->memory().read8( 0x0001 ) != 0 )	// cheap always-true guard
			{
				const u8 dd00 = scpu->memory().read8( 0xDD00 );
				const u8 dd02 = scpu->memory().read8( 0xDD02 );
				s_Logger->Write( "SCPU", LogNotice,
				               "  live: $DD00=$%02X (DATAin=%u CLKin=%u) $DD02=$%02X"
				               "  [POST-RESET: CIAs already cleared by the button]",
				               dd00, ( dd00 >> 7 ) & 1, ( dd00 >> 6 ) & 1, dd02 );
				s_Logger->Write( "SCPU", LogNotice,
				               "  pre-reset (~1s before): $DD00=$%02X $DD02=$%02X",
				               s_PreResetDD00, s_PreResetDD02 );
			}

			// 64 accesses, oldest first, 16 per line. Lower-case = went to the
			// real machine; upper-case W = intercepted SuperCPU register.
			for ( u32 row = 0; row < 4; row++ )
			{
				n = 0;
				for ( u32 i = 0; i < 16; i++ )
				{
					const u32 e = scpu->memory().m_IOLog[ ( scpu->memory().m_IOLogPos + row * 16 + i ) & 63 ];
					line[ n++ ] = ( e & ( 1u << 25 ) ) ? 'W' : ( ( e >> 24 ) ? 'w' : 'r' );
					n += scpuHexByte( line + n, (u8)( ( e >> 8 ) & 0xFF ) ) - 1;
					n += scpuHexByte( line + n, (u8)( e & 0xFF ) ) - 1;
					line[ n++ ] = '=';
					n += scpuHexByte( line + n, (u8)( ( e >> 16 ) & 0xFF ) );
				}
				line[ n ] = 0;
				s_Logger->Write( "SCPU", LogNotice, "  io%u: %s", row, line );
			}
		}

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

// Shared scratch for the three C64 images, safe to reuse because CC64Memory
// COPIES what it is given.
static u8 romBuffer[ C64_BASIC_SIZE ];

// The SuperCPU's own ROM needs its own home for the life of the program:
// CSuperCPUMemoryMap::setROM() stores a POINTER rather than copying, so this
// cannot share romBuffer above. Sized for the largest image the address space
// can hold; the shipped SuperCPU DOS images are 128K.
static u8 scpuROMBuffer[ SCPU_ROM_MAXSIZE ];

static bool radBusButtonPressed() { return radBus.buttonPressed(); }
static bool radBusHardwareResetPressed() { return radBus.hardwareResetPressed(); }
static void radBusSampleInterrupts( bool &irq, bool &nmi ) { radBus.sampleInterrupts( irq, nmi ); }

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

// Same, but for an image whose size is not known in advance. The SuperCPU DOS
// images ship at 128K, but the ROM region is larger and CMD produced more than
// one build, so the size is reported rather than demanded -- an image that is
// not the size we expected is far more useful loaded than refused.
// Returns 0 if absent or unusable.
static u32 loadROMFileSized( CLogger *logger, const char *name, u8 *dst, u32 capacity )
{
	u32 size = 0;

	if ( !readFile( logger, SCPU_DRIVE, name, dst, &size, capacity ) )
		return 0;

	if ( size == 0 )
		return 0;

	// A ROM that is not a power of two cannot mirror cleanly into its region,
	// which is how the address decode fills the space above the image.
	if ( size & ( size - 1 ) )
	{
		logger->Write( "SCPU", LogWarning,
		               "%s is %u bytes, not a power of two -- ignored", name, size );
		return 0;
	}

	logger->Write( "SCPU", LogNotice, "loaded %s (%u KB)", name, size / 1024 );
	return size;
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
	               "PC=$%06X  cycles=%llu  memtop($37/$38)=$%02X%02X",
	               (unsigned)scpu.cpu()->pc(),
	               (unsigned long long)scpu.cpu()->cycles(),
	               ram[ 0x38 ], ram[ 0x37 ] );

	// Achieved speed. The selected speed is only a request: if the interpreter
	// plus its pacing overhead cannot retire instructions fast enough, the
	// machine silently runs slower than it claims and everything time-based
	// drifts. This is the number that says whether that is happening.
	{
		const u64 c0 = scpu.cpu()->cycles();
		const u64 h0 = radBus.hostCycles();
		const u64 io0 = scpu.memory().m_IOReads + scpu.memory().m_IOWrites;
		const u64 mir0 = scpu.writeBuffer().m_BytesFlushed;
		const u64 wait0 = scpu.memory().m_PacerWaitHostCycles;
		const u64 slow0 = scpu.memory().m_SlowPacedCycles;
		const u64 iec0 = scpu.memory().m_IECThrottledCycles;

		for ( u32 i = 0; i < 60; i++ ) scpu.runFrame();

		const u64 dc  = scpu.cpu()->cycles() - c0;
		const u64 dh  = radBus.hostCycles() - h0;
		const u64 dio = ( scpu.memory().m_IOReads + scpu.memory().m_IOWrites ) - io0;
		const u64 dmir = scpu.writeBuffer().m_BytesFlushed - mir0;
		const u64 dwait = scpu.memory().m_PacerWaitHostCycles - wait0;
		const u64 dslow = scpu.memory().m_SlowPacedCycles - slow0;
		const u64 diec = scpu.memory().m_IECThrottledCycles - iec0;
		const u64 dfast = dc > dslow ? dc - dslow : 0;

		const u32 achievedKHz = dh ? (u32)( ( dc * ( SCPU_ARM_CLOCK_HZ / 1000 ) ) / dh ) : 0;
		const u64 expectedHost = dfast * ( SCPU_ARM_CLOCK_HZ / SCPU_TURBO_HZ )
		                       + dslow * ( SCPU_ARM_CLOCK_HZ / SCPU_NORMAL_HZ );

		logger->Write( "SCPU", LogNotice,
		               "speed: aggregate %u kHz, schedule %llu%% (%s)",
		               (unsigned)achievedKHz,
		               (unsigned long long)( dh ? ( expectedHost * 100 ) / dh : 0 ),
		               ( expectedHost * 100 >= dh * 90 )
		                   ? "keeping up" : "FALLING BEHIND -- timing will drift" );
		logger->Write( "SCPU", LogNotice,
		               "  modes: %llu turbo + %llu slow cycles (%llu IEC-forced)",
		               (unsigned long long)dfast, (unsigned long long)dslow,
		               (unsigned long long)diec );

		// Where the time actually went. Without this the speed figure says only
		// THAT we are behind, never WHY, and the two candidate explanations --
		// a slow interpreter versus too much traffic on the expansion port --
		// call for completely different fixes.
		//
		// A bus access costs roughly one microsecond, i.e. about 1400 ARM
		// cycles at 1.4GHz, so it takes very few of them to dominate. Compare
		// "arm/emucycle" against "bus%" to tell the two apart: a high
		// arm/emucycle with a low bus% means the interpreter is the limit.
		const u64 busAccesses = dio + dmir;
		const u64 busArmCycles = busAccesses * ( SCPU_ARM_CLOCK_HZ / 1000000 );
		logger->Write( "SCPU", LogNotice,
		               "  %llu emu cycles in %llu arm cycles = %llu arm/emucycle",
		               (unsigned long long)dc, (unsigned long long)dh,
		               (unsigned long long)( dc ? dh / dc : 0 ) );
		logger->Write( "SCPU", LogNotice,
		               "  pacer waited %llu arm cycles; excluding waits: %llu arm/emucycle",
		               (unsigned long long)dwait,
		               (unsigned long long)( dc && dh > dwait ? ( dh - dwait ) / dc : 0 ) );
		if ( superCPU.benchArmPerEmuCycle() )
		{
			logger->Write( "SCPU", LogNotice,
			               "  interpreter pure: %u arm/emucycle (70 = 20MHz)",
			               (unsigned)superCPU.benchArmPerEmuCycle() );
			// Per 1000 emulated cycles: the stall diagnosis. A 32K two-way
			// L1I refilling constantly, a D-cache thrashing, or a mispredict
			// per dispatch each point at a different fix.
			logger->Write( "SCPU", LogNotice,
			               "  bench PMU/1k emu: instr=%u l1i=%u l1d=%u brmiss=%u",
			               (unsigned)superCPU.m_BenchInstrPer1k,
			               (unsigned)superCPU.m_BenchL1IRefillPer1k,
			               (unsigned)superCPU.m_BenchL1DRefillPer1k,
			               (unsigned)superCPU.m_BenchBranchMissPer1k );
		}
		logger->Write( "SCPU", LogNotice,
		               "  bus: %llu io + %llu mirrored = %llu accesses, ~%llu%% of the time",
		               (unsigned long long)dio, (unsigned long long)dmir,
		               (unsigned long long)busAccesses,
		               (unsigned long long)( dh ? ( busArmCycles * 100 ) / dh : 0 ) );
	}

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

	// The SuperCPU's own ROM -- SuperCPU DOS and its JiffyDOS support. Entirely
	// optional: SCPU-EMU boots the machine's own KERNAL out of bank 0, so
	// without this the accelerator still works and $F80000 simply reads open
	// bus. With it, software that knows about the accelerator can reach the
	// code CMD shipped.
	//
	// Note this only makes the ROM READABLE. It does not map it over bank 0 at
	// reset -- see the bootmap note in registers.h. Changing what runs at
	// power-on is a different decision, and one that cannot be backed out from
	// the config file, because the machine would never get far enough to read
	// it.
	{
		const u32 scpuROMSize = loadROMFileSized( logger, SCPU_ROM_DIR "scpu.rom",
		                                          scpuROMBuffer, SCPU_ROM_MAXSIZE );
		if ( scpuROMSize )
			superCPU.memoryMap().setROM( scpuROMBuffer, scpuROMSize );
	}

	// From here on the SD card is not touched again: the FAT driver takes
	// interrupts and allocates, neither of which is safe once we are holding
	// DMA and driving the bus to a cycle budget.
	radUnmountFileSystem();
	actLED.Blink( 2 );		// milestone: ROMs handled, SD released

	s_Logger = logger;
	radBus.setLogger( logger );

	// Which core: the 65816 is the real accelerator, the 6502 is milestone-1
	// scaffolding kept as a fallback. CPU_CORE in scpu.cfg decides, so a bad run
	// can be backed out by editing one line on the SD card.
	const SCPUCoreType core = ( cfgCPUCore == SCPU_CFG_CORE_6502 )
	                        ? SCPU_CORE_6502 : SCPU_CORE_65816;

	logger->Write( "SCPU", LogNotice, "CPU core: %s",
	               ( core == SCPU_CORE_6502 ) ? "MOS 6502 (fallback)" : "WDC 65C816" );

	// Bootmap runs CMD's own boot code before the C64's KERNAL, which is where
	// the SuperCPU banner comes from. It is read from the config here, on the
	// Pi, well before any emulation starts -- so BOOTMAP 0 on the SD card is
	// always effective, however badly the emulated machine behaves with it on.
	superCPU.setBootmapEnabled( cfgBootmap != 0 );

	if ( !superCPU.init( &radBus, core, SCPU_SIMM_16MB ) )
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

	// The pure-interpreter figure, no bus and no scheduler: 70 means 20MHz.
	if ( superCPU.benchArmPerEmuCycle() )
		logger->Write( "SCPU", LogNotice,
		               "interpreter: %u arm/emucycle pure (70 = 20MHz)",
		               (unsigned)superCPU.benchArmPerEmuCycle() );

	const C64Signals &sig = radBus.signals();
	logger->Write( "SCPU", LogNotice, "SuperRAM: %u MB%s",
	               (unsigned)superCPU.fastRAM().sizeMB(),
	               superCPU.fastRAM().present()
	                   ? " (unreachable until the 65816 core lands)" : " -- none fitted" );

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
