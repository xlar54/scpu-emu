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
#ifndef _rad_bus_h
#define _rad_bus_h

#include "../../C64/c64_bus.h"

// Must match arm_freq in config.txt. The bus timings are counted in these
// cycles too, so this is the same clock the whole design is pinned to.
#define SCPU_ARM_CLOCK_HZ 1400000000u

class CLogger;

class CRADBus : public IC64Bus
{
public:
	CRADBus();

	// Optional progress logging. acquire() is a sequence of steps that can each
	// stall or fail, and without this a failure anywhere in it looks identical
	// from the outside: a C64 that just sits there. Only phase boundaries are
	// logged, never anything inside a timed loop.
	void setLogger( CLogger *logger ) { m_Logger = logger; }

	// --- IC64Bus ----------------------------------------------------------
	bool acquire() override;
	void release() override;
	const C64Signals &signals() const override { return m_Signals; }

	u8   read( u16 addr ) override;
	void write( u16 addr, u8 value ) override;
	void writeBurst( const C64BusWrite *writes, u32 count ) override;
	void readBlock( u16 addr, u8 *dst, u32 length ) override;

	bool irqAsserted() override;
	bool nmiAsserted() override;
	void sampleInterrupts( bool &irq, bool &nmi ) override;
	u16  rasterLine() override;

	u64  hostCycles() override;
	u32  hostCyclesPerSec() override { return SCPU_ARM_CLOCK_HZ; }

	const char *name() const override { return "RAD expansion unit"; }

	// Flash the border a few times, as visible proof that we hold the bus and
	// can drive it. Costs a fraction of a second and happens before the CPU
	// core starts, so it is unmistakably ours rather than anything the C64 did.
	// Without it a failed takeover and a successful one both just look like a
	// C64 sitting at a BASIC prompt.
	void signalAlive();

	// Exercise each bus operation against known values and report what works.
	// The border flash only proves single writes reach the VIC; this separates
	// single reads, single writes and burst writes from each other, which is
	// the difference between "the bus is broken" and "one code path is".
	// Returns true if everything passed.
	bool selfTest();

	// Hold /DMA but drive nothing for roughly the given number of seconds, so
	// the C64's display can be observed with zero bus traffic from us.
	void idleHold( u32 seconds );

	// Same, but returns early if the button is pressed. Never use this for the
	// post-reset settle -- a short wait there breaks the handover.
	void idleHoldInterruptible( u32 seconds );

	// True while the RAD's button is held. Reading it is free -- no bus cycle,
	// just a GPIO level -- so it is safe to poll from the run loop.
	bool buttonPressed();

	// True while the C64's /RESET line is being pulled low by something other
	// than us -- i.e. the RAD's hardware reset button. That button is wired
	// straight to the reset line and never reaches the Pi as a button, but we
	// can see its effect because /RESET is an input to us between accesses.
	bool hardwareResetPressed();

	// Statistics, useful for judging how much bus bandwidth a workload burns.
	u64 m_Reads, m_Writes, m_BurstWrites;

	bool acquired() const { return m_Acquired; }

private:
	C64Signals m_Signals;
	bool       m_Acquired;
	CLogger   *m_Logger;
};

#endif
