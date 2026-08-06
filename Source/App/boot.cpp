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
#include <circle/synchronize.h>

#include "../Common/config.h"
#include "../Common/helpers.h"
#include "../Bus/RAD/gpio_defs.h"
#include "../Bus/RAD/lowlevel_arm64.h"
#include "../Bus/RAD/bus_timing.h"
#include "../Bus/RAD/rad_bus.h"
#include "../SuperCPU/supercpu.h"

// Defined in main.cpp, which owns the FAT filesystem object.
extern "C" void radUnmountFileSystem();

// Diagnostics have to run with interrupts ENABLED to reach the screen: Circle
// cannot complete a logger write while they are masked. The frame hook runs
// between frames with no bus access in flight, so briefly unmasking there
// cannot disturb a timed bus window.
class CScopedLoggingIRQs
{
public:
	CScopedLoggingIRQs()  { EnableIRQs(); }
	~CScopedLoggingIRQs() { DisableIRQs(); }
};

// Bus waits that hit their ceiling instead of seeing the signal they wanted.
// Defined in Bus/RAD/gpio_defs.cpp; declared here rather than including
// lowlevel_dma.h, which needs the whole GPIO header stack ahead of it.
extern volatile u32 radBAWaitTimeouts;
extern volatile u32 radPHIWaitTimeouts;

// Heartbeat: proves the frame loop is still running, and reports what the
// emulated machine is doing while it does so.
static u32 s_HeartbeatFrames = 0;
static u32 s_MirrorHaltFrames = 0;
static u32 s_HeartbeatPCLow = 0xFFFFFFFF;
static u32 s_HeartbeatPCHigh = 0;
static u32 s_HeartbeatStalledRuns = 0;
static bool s_HeartbeatDumped = false;

static bool radBusButtonPressed();
static bool radBusHardwareResetPressed();
static void radBusSampleInterrupts( bool &irq, bool &nmi );

// Bare metal: no snprintf. Keep every freeze-dump append explicitly bounded:
// a CIA timing row can be about 240 characters, substantially more than the
// old 128-byte scratch buffer allowed.
static const u32 SCPU_DUMP_LINE_SIZE = 256;

static void scpuLineStart( char *dst, u32 capacity, u32 &length )
{
	length = 0;
	if ( capacity ) dst[ 0 ] = 0;
}

static bool scpuLineChar( char *dst, u32 capacity, u32 &length, char c )
{
	if ( !capacity || length + 1 >= capacity )
		return false;
	dst[ length++ ] = c;
	dst[ length ] = 0;
	return true;
}

static bool scpuLineHexByte( char *dst, u32 capacity, u32 &length, u8 v )
{
	static const char *h = "0123456789ABCDEF";
	return scpuLineChar( dst, capacity, length, h[ v >> 4 ] )
	    && scpuLineChar( dst, capacity, length, h[ v & 15 ] );
}

static bool scpuLineDecimal( char *dst, u32 capacity, u32 &length, u32 v )
{
	char reversed[ 10 ];
	u32 digits = 0;
	do
	{
		reversed[ digits++ ] = (char)( '0' + v % 10 );
		v /= 10;
	} while ( v && digits < sizeof reversed );

	while ( digits )
		if ( !scpuLineChar( dst, capacity, length, reversed[ --digits ] ) )
			return false;
	return true;
}
static void scpuWaitForButtonThenReboot();
static CLogger *s_Logger;
static bool s_RebootRequested = false;

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
static bool s_PreResetSampleValid = false;
static const u32 SCPU_RESET_RELEASE_STABLE_SAMPLES = 100000;

// A reset press also resets the physical VIC and CIAs.  Keep the time between
// noticing /RESET and waiting for its release as short as possible: emitting a
// multi-line logger dump here used to leave the physical machine released
// while the emulated CPU was still stopped if the user let go during logging.
// Everything needed by the dump is therefore copied into this fixed-size
// snapshot first, and the frame hook emits it later in an IEC-idle frame.
struct SCPUResetDiagnostic
{
	bool pending;
	bool haveCPU;
	bool haveCore;
	bool haveCode;
	bool preResetSampleValid;
	u32 pc;
	u16 sp;
	u64 cycles;
	u8  p;
	u8  e;
	u32 irqsTaken;
	u32 nmisTaken;
	bool liveIRQ;
	bool liveNMI;
	u8 stack[ 16 ];
	u8 code[ 16 ];
	u32 cia[ 64 ];
	u32 ciaIntervalEntry[ 16 ];
	u32 ciaIntervalUS[ 16 ];
	u32 io[ 64 ];
	u8 preResetDD00;
	u8 preResetDD02;
};

static SCPUResetDiagnostic s_ResetDiagnostic = {};
static u32 s_ResetDiagnosticQuietFrames = 0;
static u32 s_HardwareResetGeneration = 0;

static void scpuSnapshotResetDiagnostic( CSuperCPU *scpu )
{
	SCPUResetDiagnostic &d = s_ResetDiagnostic;
	d.pending = false;
	d.haveCPU = scpu && scpu->cpu();
	d.haveCore = false;
	d.haveCode = false;
	d.preResetSampleValid = s_PreResetSampleValid;
	d.preResetDD00 = s_PreResetDD00;
	d.preResetDD02 = s_PreResetDD02;

	if ( !d.haveCPU )
	{
		d.pending = true;
		s_ResetDiagnosticQuietFrames = 0;
		return;
	}

	d.pc = (u32)scpu->cpu()->pc();
	d.sp = scpu->cpu()->stackPointer();
	d.cycles = scpu->cpu()->cycles();

	if ( CW65C816 *core = scpu->core65816() )
	{
		d.haveCore = true;
		d.p = core->m_P;
		d.e = core->m_E ? 1 : 0;
		d.irqsTaken = core->m_IRQsTaken;
		d.nmisTaken = core->m_NMIsTaken;
		radBusSampleInterrupts( d.liveIRQ, d.liveNMI );
	}

	for ( u32 i = 0; i < 16; i++ )
	{
		// The 6502 and emulation-mode 65816 wrap S inside page $01. In
		// native mode S is a full 16-bit bank-0 address.
		const u16 stackAddr = ( d.haveCore && !d.e )
		                    ? (u16)( d.sp + i + 1 )
		                    : (u16)( 0x0100 | ( ( d.sp + i + 1 ) & 0xFF ) );
		d.stack[ i ] = scpu->memory().m_RAM[ stackAddr ];
	}

	// This deliberately excludes bank 0's I/O page. No diagnostic snapshot may
	// add a physical CIA access while /RESET is asserted. Check the whole window,
	// not just PC: a PC near $CFFF can make PC+11 cross into $D000. Code in the
	// 65816's private banks is safe and retains its full 24-bit address.
	bool codeWindowSafe = true;
	for ( int i = -4; i < 12; i++ )
	{
		const u32 addr = ( d.pc + i ) & SCPU_ADDR_MASK;
		if ( ( addr & 0xFF0000 ) == 0 && ( addr & 0xF000 ) == 0xD000 )
		{
			codeWindowSafe = false;
			break;
		}
	}
	if ( codeWindowSafe )
	{
		d.haveCode = true;
		for ( int i = -4; i < 12; i++ )
			d.code[ i + 4 ] = scpu->memoryMap().read8(
				( d.pc + i ) & SCPU_ADDR_MASK );
	}

	for ( u32 i = 0; i < 64; i++ )
	{
		d.cia[ i ] = scpu->memory().m_CIALog[
			( scpu->memory().m_CIALogPos + i ) & 63 ];
		d.io[ i ] = scpu->memory().m_IOLog[
			( scpu->memory().m_IOLogPos + i ) & 63 ];
	}

	for ( u32 i = 0; i < 16; i++ )
	{
		const u32 age = i + 1;
		const u32 idx  = ( scpu->memory().m_CIALogPos + 64 - age ) & 63;
		const u32 prev = ( scpu->memory().m_CIALogPos + 64 - age - 1 ) & 63;
		d.ciaIntervalEntry[ i ] = scpu->memory().m_CIALog[ idx ];
		u32 dt = ( scpu->memory().m_CIALogCyc[ idx ]
		         - scpu->memory().m_CIALogCyc[ prev ] )
		       / ( SCPU_ARM_CLOCK_HZ / 1000000u );
		if ( dt > 99999 ) dt = 99999;
		d.ciaIntervalUS[ i ] = dt;
	}

	// Publish the snapshot only after every field is complete.
	d.pending = true;
	s_ResetDiagnosticQuietFrames = 0;
}

static void scpuEmitResetDiagnostic()
{
	SCPUResetDiagnostic &d = s_ResetDiagnostic;
	if ( !d.pending || !s_Logger )
		return;

	// Clear first so this is one bounded emission even if logging itself is
	// interrupted. A later reset press will simply publish a fresh snapshot.
	d.pending = false;
	s_ResetDiagnosticQuietFrames = 0;

	if ( !d.haveCPU )
	{
		s_Logger->Write( "SCPU", LogNotice,
		                 "reset pressed: CPU diagnostic unavailable" );
		return;
	}

	s_Logger->Write( "SCPU", LogNotice,
	               "reset pressed: PC=$%06X S=$%04X cycles=%llu",
	               (unsigned)d.pc, (unsigned)d.sp,
	               (unsigned long long)d.cycles );

	if ( d.haveCore )
		s_Logger->Write( "SCPU", LogNotice,
		               "  P=$%02X (I=%u) E=%u  irqsTaken=%u nmisTaken=%u  line: IRQ=%u NMI=%u",
		               d.p, ( d.p & W65_I ) ? 1 : 0, d.e,
		               (unsigned)d.irqsTaken, (unsigned)d.nmisTaken,
		               d.liveIRQ ? 1 : 0, d.liveNMI ? 1 : 0 );

	char line[ SCPU_DUMP_LINE_SIZE ];
	u32 n = 0;
	scpuLineStart( line, sizeof line, n );
	for ( u32 i = 0; i < 16; i++ )
	{
		scpuLineHexByte( line, sizeof line, n, d.stack[ i ] );
		scpuLineChar( line, sizeof line, n, ' ' );
	}
	s_Logger->Write( "SCPU", LogNotice, "  stack: %s", line );

	if ( d.haveCode )
	{
		scpuLineStart( line, sizeof line, n );
		for ( u32 i = 0; i < 16; i++ )
		{
			scpuLineHexByte( line, sizeof line, n, d.code[ i ] );
			scpuLineChar( line, sizeof line, n, ' ' );
		}
		s_Logger->Write( "SCPU", LogNotice, "  code@PC-4: %s", line );
	}

	for ( u32 row = 0; row < 4; row++ )
	{
		scpuLineStart( line, sizeof line, n );
		for ( u32 i = 0; i < 16; i++ )
		{
			const u32 e = d.cia[ row * 16 + i ];
			if ( !scpuLineChar( line, sizeof line, n, ( e >> 24 ) ? 'w' : 'r' )
			     || !scpuLineHexByte( line, sizeof line, n, (u8)( ( e >> 8 ) & 0xFF ) )
			     || !scpuLineHexByte( line, sizeof line, n, (u8)( e & 0xFF ) )
			     || !scpuLineChar( line, sizeof line, n, '=' )
			     || !scpuLineHexByte( line, sizeof line, n, (u8)( ( e >> 16 ) & 0xFF ) )
			     || !scpuLineChar( line, sizeof line, n, ' ' ) )
				break;
		}
		s_Logger->Write( "SCPU", LogNotice, "  cia%u: %s", row, line );
	}

	scpuLineStart( line, sizeof line, n );
	for ( u32 i = 0; i < 16; i++ )
	{
		const u32 e = d.ciaIntervalEntry[ i ];
		if ( !scpuLineChar( line, sizeof line, n, ( e >> 24 ) ? 'w' : 'r' )
		     || !scpuLineHexByte( line, sizeof line, n, (u8)( ( e >> 8 ) & 0xFF ) )
		     || !scpuLineHexByte( line, sizeof line, n, (u8)( e & 0xFF ) )
		     || !scpuLineChar( line, sizeof line, n, '=' )
		     || !scpuLineHexByte( line, sizeof line, n, (u8)( ( e >> 16 ) & 0xFF ) )
		     || !scpuLineChar( line, sizeof line, n, '+' )
		     || !scpuLineDecimal( line, sizeof line, n, d.ciaIntervalUS[ i ] )
		     || !scpuLineChar( line, sizeof line, n, ' ' ) )
			break;
	}
	s_Logger->Write( "SCPU", LogNotice, "  ciaT: %s", line );

	if ( d.preResetSampleValid )
		s_Logger->Write( "SCPU", LogNotice,
		               "  pre-reset (last IEC-idle sample): $DD00=$%02X $DD02=$%02X",
		               d.preResetDD00, d.preResetDD02 );
	else
		s_Logger->Write( "SCPU", LogNotice,
		               "  pre-reset: no IEC-idle CIA sample available" );
	s_Logger->Write( "SCPU", LogNotice,
	               "  live CIA state not sampled while physical /RESET was asserted" );

	// Both should read zero. Either one non-zero means a bus wait hit its
	// ceiling instead of seeing the signal it wanted -- which before those
	// ceilings existed was an unrecoverable hang, not a counter.
	s_Logger->Write( "SCPU", LogNotice,
	               "  bus wait timeouts: BA=%u PHI=%u",
	               (unsigned)radBAWaitTimeouts, (unsigned)radPHIWaitTimeouts );

	for ( u32 row = 0; row < 4; row++ )
	{
		scpuLineStart( line, sizeof line, n );
		for ( u32 i = 0; i < 16; i++ )
		{
			const u32 e = d.io[ row * 16 + i ];
			if ( !scpuLineChar( line, sizeof line, n,
			                    ( e & ( 1u << 25 ) ) ? 'W' : ( ( e >> 24 ) ? 'w' : 'r' ) )
			     || !scpuLineHexByte( line, sizeof line, n, (u8)( ( e >> 8 ) & 0xFF ) )
			     || !scpuLineHexByte( line, sizeof line, n, (u8)( e & 0xFF ) )
			     || !scpuLineChar( line, sizeof line, n, '=' )
			     || !scpuLineHexByte( line, sizeof line, n, (u8)( ( e >> 16 ) & 0xFF ) )
			     || !scpuLineChar( line, sizeof line, n, ' ' ) )
				break;
		}
		s_Logger->Write( "SCPU", LogNotice, "  io%u: %s", row, line );
	}
}

static bool scpuCheckButton( void *ctx )
{
	CSuperCPU *scpu = (CSuperCPU *)ctx;
	const bool hardwareResetPressed = radBusHardwareResetPressed();

	// Say so the FIRST time a bus wait ever hits its ceiling. Before those
	// ceilings existed this was the freeze that took the whole firmware down
	// -- no frame hook, no button, power cycle only -- so it left no trace at
	// all. One line, once, is the difference between a mystery and a lead.
	if ( !scpu || !scpu->memory().iecBusActive() )
	{
		static u32 lastBA = 0, lastPHI = 0;
		if ( radBAWaitTimeouts != lastBA || radPHIWaitTimeouts != lastPHI )
		{
			lastBA = radBAWaitTimeouts;
			lastPHI = radPHIWaitTimeouts;
			CScopedLoggingIRQs irqs;
			s_Logger->Write( "SCPU", LogNotice,
			               "BUS WAIT TIMEOUT: BA=%u PHI=%u (bus stopped answering)",
			               (unsigned)lastBA, (unsigned)lastPHI );
		}
	}

	// These reads go to the physical CIA. Do not insert them into an active
	// serial transaction merely for diagnostics; take the next scheduled
	// sample once IEC is quiet instead.
	if ( scpu && !hardwareResetPressed
	     && ( ++s_PreResetSampleFrame % 50 ) == 0
	     && !scpu->memory().iecBusActive() )
	{
		s_PreResetDD00 = scpu->memory().read8( 0xDD00 );
		s_PreResetDD02 = scpu->memory().read8( 0xDD02 );
		s_PreResetSampleValid = true;
	}

	if ( radBusButtonPressed() )
	{
		s_RebootRequested = true;
		return true;					// stop the run loop; caller reboots
	}

	if ( hardwareResetPressed )
	{
		// Take a bounded snapshot first; do not log or sample a physical CIA
		// until the machine has been released and normal execution has resumed.
		scpuSnapshotResetDiagnostic( scpu );

		// The physical VIC and CIAs remain in reset while the line is low, so the
		// emulator must not resume bus traffic until release. Require a long run
		// of agreeing high samples; any bounce resets the count. This remains
		// responsive to however long the user holds the button while preventing a
		// single press from becoming several emulated resets.
		u32 released = 0;
		while ( released < SCPU_RESET_RELEASE_STABLE_SAMPLES )
		{
			if ( radBusHardwareResetPressed() ) released = 0;
			else                                  released++;
		}

		if ( scpu ) scpu->reset();
		s_HardwareResetGeneration++;
		s_PreResetSampleFrame = 0;
		s_PreResetSampleValid = false;
		return false;
	}

	// --- diagnostic mirror halt ---------------------------------------------
	// MIRROR_HALT_AFTER_S: after N seconds, stop all mirror bus traffic so
	// the display shows whatever real DRAM holds, undisturbed. If corruption
	// SNAPS CLEAN when traffic stops, our ongoing writes were disturbing the
	// VIC's fetches; if it PERSISTS with the bus quiet, the DRAM content
	// itself is wrong -- our writes must be landing somewhere they should not.
	if ( cfgMirrorHaltAfterS > 0 && scpu && !scpu->mirrorHalted() )
	{
		if ( ++s_MirrorHaltFrames >= (u32)cfgMirrorHaltAfterS * 60 )
		{
			scpu->setMirrorHalted( true );
			CScopedLoggingIRQs irqs;
			s_Logger->Write( "SCPU", LogNotice,
			               "MIRROR HALTED (diagnostic): all mirror bus traffic stopped" );
		}
	}

	// --- heartbeat and stall self-detection ---------------------------------
	// A freeze used to leave nothing behind. The bounded bus waits keep the
	// firmware alive through one, but "no output" is still ambiguous: a silent
	// log looks identical whether the Pi is wedged or simply has nothing to
	// report. So say something every second, and say what the emulated machine
	// is doing while saying it.
	//
	// The PC range is the useful part. A machine that is working sweeps a wide
	// range of addresses in a second; one parked in a KERNAL wait loop spins
	// over a handful of bytes. When that happens for several seconds running,
	// dump the full diagnostic unprompted -- the reset button is a poor way to
	// collect it, because pressing it resets the physical CIAs and destroys the
	// serial state that explains the stall.
	if ( scpu && scpu->cpu() )
	{
		const u32 pc = (u32)scpu->cpu()->pc();
		if ( pc < s_HeartbeatPCLow )  s_HeartbeatPCLow  = pc;
		if ( pc > s_HeartbeatPCHigh ) s_HeartbeatPCHigh = pc;

		if ( ++s_HeartbeatFrames >= 60 )
		{
			// Never log mid-transaction: the serial protocol reads a pause as
			// EOI, and one heartbeat write costs milliseconds. Hold the window
			// open until a quiet second comes around; the accumulated PC range
			// still tells the story. This is the rule the reset dump has
			// always followed (three quiet frames) -- the first heartbeat cut
			// ignored it and broke JiffyDOS loads once per second.
			if ( scpu->memory().iecBusActive() )
				return false;
			const u32 span = s_HeartbeatPCHigh - s_HeartbeatPCLow;

			// A machine sitting at READY legitimately spins in the KERNAL's
			// keyboard wait ($E5CD-$E5D4), which a span heuristic cannot tell
			// from a wedge. Exempt exactly that window; a machine stuck
			// anywhere else -- including the $FF48 BRK loop this detector was
			// built for -- still trips.
			const bool kernalIdle = ( s_HeartbeatPCLow  >= 0x00E5C0
			                       && s_HeartbeatPCHigh <= 0x00E5D8 );
			const bool stalled = ( span < 64 ) && !kernalIdle;

			CScopedLoggingIRQs irqs;
			s_Logger->Write( "SCPU", LogNotice,
			               "hb PC=$%06X..$%06X wb=%u iec=%u BA=%u PHI=%u irqbank=%u%s",
			               (unsigned)s_HeartbeatPCLow, (unsigned)s_HeartbeatPCHigh,
			               (unsigned)scpu->writeBuffer().pending(),
			               (unsigned)( scpu->memory().iecBusActive() ? 1 : 0 ),
			               (unsigned)radBAWaitTimeouts,
			               (unsigned)radPHIWaitTimeouts,
			               (unsigned)scpu->registers().m_InterruptBankCloses,
			               stalled ? "  <-- STALLED" : "" );

			if ( stalled )
			{
				// Three seconds in the same few bytes is not a busy loop doing
				// work; dump once and then stay quiet until it moves again.
				if ( ++s_HeartbeatStalledRuns == 3 && !s_HeartbeatDumped )
				{
					s_HeartbeatDumped = true;
					scpuSnapshotResetDiagnostic( scpu );
					CScopedLoggingIRQs dumpIRQs;
					s_Logger->Write( "SCPU", LogNotice,
					               "STALL DETECTED -- dumping state (no reset pressed)" );
					scpuEmitResetDiagnostic();
				}
			}
			else
			{
				s_HeartbeatStalledRuns = 0;
				s_HeartbeatDumped = false;
			}

			s_HeartbeatFrames = 0;
			s_HeartbeatPCLow  = 0xFFFFFFFF;
			s_HeartbeatPCHigh = 0;
		}
	}

	// Emit the saved pre-reset state on a later hook invocation, after several
	// complete IEC-idle frames. No values are read from the reset CIAs here.
	if ( s_ResetDiagnostic.pending && scpu )
	{
		if ( scpu->memory().iecBusActive() )
			s_ResetDiagnosticQuietFrames = 0;
		else if ( ++s_ResetDiagnosticQuietFrames >= 3 )
		{
			CScopedLoggingIRQs irqs;
			scpuEmitResetDiagnostic();
		}
	}

	return false;
}

// runFrame() does not invoke the normal frame hook. Startup and diagnostic
// loops use this wrapper so the physical reset button is still serviced before
// the emulator enters its permanent run() loop. A reset aborts the diagnostic
// currently in progress; normal execution can then resume immediately.
static bool scpuRunFrameAndCheckButtons( CSuperCPU &scpu )
{
	const u32 resetGeneration = s_HardwareResetGeneration;
	scpu.runFrame();
	const bool stop = scpuCheckButton( &scpu );
	return stop || resetGeneration != s_HardwareResetGeneration;
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
	// Logging stops the emulated CPU while the physical drive keeps running.
	// The caller seeks a quiet window, but keep this defensive check here so a
	// future caller cannot accidentally pause an active IEC transaction.
	if ( scpu.memory().iecBusActive() )
		return;

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

		// A transaction can begin during the benchmark. Abort before producing
		// any more synchronous log output and return to the free-running loop.
		for ( u32 i = 0; i < 60; i++ )
		{
			if ( scpu.memory().iecBusActive() )
				return;
			if ( scpuRunFrameAndCheckButtons( scpu ) )
				return;
			if ( scpu.memory().iecBusActive() )
				return;
		}

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

// Startup diagnostics are useful, but logger output and the snapshot benchmark
// both stop normal execution at frame boundaries. Wait only by running normal
// frames, require IEC to remain quiet across several of them, and give up after
// a bounded interval if the machine is still probing or loading from a drive.
static bool scpuWaitForDiagnosticWindow( CSuperCPU &scpu )
{
	static const u32 MAX_WAIT_FRAMES = 180;
	static const u32 QUIET_FRAMES_REQUIRED = 3;
	u32 quietFrames = 0;

	for ( u32 i = 0; i < MAX_WAIT_FRAMES; i++ )
	{
		if ( !scpu.memory().iecBusActive() )
		{
			if ( ++quietFrames >= QUIET_FRAMES_REQUIRED )
				return true;
		}
		else
			quietFrames = 0;

		if ( scpuRunFrameAndCheckButtons( scpu ) )
			return false;
	}

	return false;
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

	// This replaces the physical JiffyDOS switch on the cartridge. Set it before
	// init() resets and starts the SuperCPU ROM; reset deliberately preserves the
	// switch position, so CMD's boot code sees the configured value when it
	// chooses which KERNAL to install. A later $D0B5 POKE remains in effect until
	// the Pi is rebooted and this configuration is read again.
	superCPU.registers().setJiffyDOSSwitch( cfgJiffyDOS != 0 );
	logger->Write( "SCPU", LogNotice, "JiffyDOS: %s (scpu.cfg)",
	               cfgJiffyDOS ? "enabled" : "disabled" );

	// Bootmap runs CMD's own boot code before the C64's KERNAL, which is where
	// the SuperCPU banner comes from. It is read from the config here, on the
	// Pi, well before any emulation starts -- so BOOTMAP 0 on the SD card is
	// always effective, however badly the emulated machine behaves with it on.
	superCPU.setBootmapEnabled( cfgBootmap != 0 );
	superCPU.registers().setBootAnimationHack( cfgBootAnimation != 0 );
	if ( cfgBootAnimation )
		logger->Write( "SCPU", LogNotice,
		               "boot animation: enabled (one-byte $D20C deviation)" );

	// Mirroring inside the visible picture. Border-only delivery (0) cannot
	// keep up with a game that redraws moving objects every frame, which shows
	// as coherent scenery with mixed-frame garbage on exactly the moving parts.
	superCPU.setMirrorDisplayBudget( (u32)cfgMirrorDisplayBytes );
	logger->Write( "SCPU", LogNotice, "mirror: %u bytes/drain inside the display%s",
	               (unsigned)cfgMirrorDisplayBytes,
	               cfgMirrorDisplayBytes ? "" : " (border-only)" );

	// Shape blocks a bank-3 game keeps under $D000-$DFFF are unreachable to
	// bus writes (the halted 6510 keeps real I/O banked in), so their sprite
	// pointers are delivered translated into relocated copies at $C000-$CBFF.
	superCPU.setUnderIORelocate( cfgMirrorD000Relocate != 0 );
	logger->Write( "SCPU", LogNotice, "mirror: under-I/O sprite relocation %s",
	               cfgMirrorD000Relocate ? "on" : "off" );

	// From superCPU.init() onward the Pi owns and accesses the C64 bus in
	// sub-microsecond GPIO windows. Circle's timer/SD IRQs may pre-empt at any
	// instruction, so leaving them enabled can stretch a nominally valid bus
	// phase into the VIC-II's half-cycle. Match upstream RAD and keep ARM IRQs
	// masked for timed bus operation.
	//
	// This masking is LOAD-BEARING, established by bisection: moving it later
	// so that startup logging would work stopped games loading at all, because
	// init(), the bus self-test and the first frames then ran with interrupts
	// live. Logging is bought back the other way instead -- CScopedLoggingIRQs
	// unmasks around individual log writes, which happen between bus
	// operations, never inside one.
	DisableIRQs();

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

	// Interrupts stay masked from init() to the end of the run loop. Each
	// message BLOCK below unmasks just for its writes -- never across a bus
	// operation. The first cut of this used one scope over all of startup,
	// which quietly included the bus self-test's timed reads and writes:
	// exactly the exposure the bisect proved fatal.
	{
	CScopedLoggingIRQs startupIRQs;

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
	}	// remask: the self-test is real, timed bus traffic

	if ( !radBus.selfTest() )
	{
		// Do not start the core over a bus we know is unreliable. It produces a
		// dramatic display and no information, and it leaves the machine
		// unusable. Hand the C64 back instead: it returns to its own CPU and
		// the log above stays on screen to be read.
		CScopedLoggingIRQs failIRQs;
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
	{
	CScopedLoggingIRQs handoverIRQs;
	logger->Write( "SCPU", LogNotice, "starting the CPU core" );
	logger->Write( "SCPU", LogNotice,
	               "buttons: LEFT = reboot the Pi, RIGHT = reset the emulated C64" );

	}	// startup reporting done; masked again for the run

	// Run about two seconds of emulated time, then report what the emulator
	// thinks it has drawn only after a bounded wait for an IEC-quiet window.
	// If boot-time drive probing never settles, skip diagnostics rather than
	// stopping the emulated CPU while the physical drive continues.
	bool startupInterrupted = false;
	for ( u32 i = 0; i < 120; i++ )
	{
		if ( scpuRunFrameAndCheckButtons( superCPU ) )
		{
			startupInterrupted = true;
			break;
		}
	}

	if ( !startupInterrupted && scpuWaitForDiagnosticWindow( superCPU ) )
		scpuDumpEmulatedScreen( logger, superCPU );

	// A short LEFT press can happen inside one of the startup wrappers above.
	// Preserve that request instead of consuming it merely as "stop this
	// diagnostic" and then entering the permanent run loop.
	if ( !s_RebootRequested )
	{
		superCPU.setFrameHook( scpuCheckButton, &superCPU );
		superCPU.run();
	}

	// The button was pressed. Release the machine and restart the Pi so a fresh
	// kernel8.img on the card is picked up.
	logger->Write( "SCPU", LogNotice, "button pressed -- releasing /DMA and rebooting" );
	radBus.release();
	actLED.Blink( 2 );
	reboot();
}
