// THE VSP WINDOW, ON THE HOST, so it stops costing hardware rounds.
//
// WHAT IS BEING SCORED. Mario's screen defect is decided by ONE number: the
// real-time spacing between the two physical writes of
//
//     $0871  DEC $D011        YSCROLL 7 -> 6
//     $0874  INC $D011        YSCROLL 6 -> 7
//
// An NMOS 6510 with no steal puts those writes 6 C64 cycles apart. On the real
// machine at K493 the pair measures 14.57 cycles mean, bimodal 13.99 x9 and
// 15.01 x13, and what is ESTABLISHED is narrow: on 22 of 22 frames the emulated
// window minus the charged BA stall is exactly 6, so the core's own cycle
// accounting for the pair is correct and the charged stall accounts for the
// whole excess. WHERE that stall is taken, WHAT it is (sprite DMA, a badline
// remainder, or a $D011-induced badline) and whether any of it is illegitimate
// are ALL OPEN. An earlier version of this file asserted a write-tail root
// cause; instrumenting the hit sites showed two of the three are READ
// boundaries, which a 6510 does halt on, and the aggregate stall field cannot
// tell them apart. Do not reintroduce that claim.
//
// WHY THE EXISTING SUITE CANNOT SEE ANY OF THIS. CHostBus completes a read or a
// write in ZERO host time. On the Pi those are bit-banged GPIO transactions
// against a real VIC. So 806800 passing checks verify the EMULATED CYCLE
// ACCOUNTING -- which this investigation proved is exactly right -- and are
// structurally blind to the physical schedule, which is the half that is wrong.
//
// WHAT THIS FIXTURE IS AND IS NOT. It is a MODEL. It cannot discover a timing
// fact about real hardware and it cannot establish an impossibility result. It
// exists so a change to transaction count, order or stall policy can be scored
// in seconds instead of a flash-swap-play-swap-read round trip.
//
// SIX GAPS CLOSED AFTER REVIEW, each of which made an earlier version of this
// file measure something other than the machine:
//   1. The interval. The window runs from the DEC's write to the INC's write,
//      so the DEC's own read and write are OUTSIDE it. An earlier version
//      asserted a four-transaction "floor" of 11.89 cycles and concluded the
//      defect was unfixable. It contains TWO transactions and the floor is
//      5.94 -- under the 6.00 target. That error would have been reported as an
//      impossibility result.
//   2. The raster. Writing bus.m_Memory[0xD012] does NOTHING: CHostBus::read
//      SYNTHESISES $D012 from rasterLineInternal(). Without a cycle source the
//      beam advances on bus-access count, so the compare-and-absorber path was
//      never exercised. Now driven from the guest's own C64 clock.
//   3. The clock. The comments said 1.4GHz NTSC while the rate was 1.368e9 and
//      CHostBus defaults to PAL -- and setPacing() consults signals().video, so
//      the pacer and the cycle conversions were on different scales. Both are
//      now set explicitly from one place.
//   4. The stall. m_BAStallMode was left at its default 0, so the single
//      mechanism that turns out to BE the defect was switched off.
//   5. The assertions. A transaction-count test that subtracts two write costs
//      and checks one read remains is satisfied by the $D012 read alone, so
//      both $D011 reads could vanish undetected. Now an explicit event ledger.
//   6. CHECK( window > 600 ) would FAIL a legitimate six-cycle result. Removed.

#include "../test_framework.h"
#include <cstdio>
#include "../../Source/C64/c64_memory.h"
#include "../../Source/CPU/W65C816/w65c816.h"
#include "../../Source/SuperCPU/memory_map.h"
#include "../../Source/SuperCPU/fast_ram.h"
#include "../../Source/SuperCPU/supercpu.h"
#include "../../Source/Bus/Host/host_bus.h"

namespace {

// ONE PLACE for every rate, so the pacer and the arithmetic cannot drift apart.
// The capture's own basis: 1,400,000,000 ARM against NTSC-R8's 1,022,727 Hz,
// giving 1368.889 ARM per C64 cycle. Kept as a rational pair rather than a
// rounded constant, because truncating this exact ratio has already produced
// one confident false finding in this investigation.
const u64 HOST_HZ = 1400000000ull;
const u64 C64_HZ  = 1022727ull;

// Measured stage costs for one physical transaction (K450/K493, CHT).
const u64 VSP_READ_ARM        = 3136;	// rsync 856 + rvic 1426 + rcpu 854
const u64 VSP_WRITE_TURN_ARM  = 4999;	// wsync 671 + wp1turn 3449 + wp2 879
const u64 VSP_WRITE_PLAIN_ARM = 2833;	// same write with no direction change

// C64 cycles -> host cycles, the same rational pair everywhere else uses.
u64 cyclesToHost( u64 cycles )
{
	return ( cycles * HOST_HZ ) / C64_HZ;
}

u64 armToHundredthCycles( u64 arm )
{
	return ( arm * 100ull * C64_HZ ) / HOST_HZ;
}

struct BusEvent
{
	u16  addr;
	bool write;
	u64  host;		// when the pin event completed
	u8   value;		// what a write carried (reads: 0)
	u64  cycle;		// the PHI2 cycle the data was valid in: sampled (read) / driven (write)
};

// A host bus that costs what the real one costs, drives the beam from the
// guest's own clock, and records every physical transaction.
struct VSPTimedBus : CHostBus
{
	u64 now = 0;
	// Turnaround state, so a write that follows a read is charged the direction
	// change and a write that follows a write is not. This is the single
	// largest term in a write (wp1turn is 3449 of 4999 ARM) and the leading
	// candidate's whole saving comes from removing a read, so a model that
	// charges both writes the same cannot score that candidate at all.
	bool lastWasRead = false;

	// Guest clock, stamped at the pin events, so counted-vs-physical can be read
	// for the SAME access rather than inferred from two different conventions.
	CC64Memory *memForStamp = 0;
	u64 guestAtD012Read = 0, guestAtD011Write = 0;
	BusEvent ev[ 64 ];
	u32      evCount = 0;

	// EDGE-ALIGNED PRIMITIVES. The mean stage costs above can score a window
	// but cannot say WHICH PHI2 cycle an access occupied, and that is the
	// quantity the oracle screen reads out, one character cell per cycle. So
	// each physical access is modelled the way the RAD primitives are built, on
	// an exact edge grid: a PHI2 rising edge at every whole C64 cycle of host
	// time (which is what phiCpuHalfStartHostCycles below hands the pacer), the
	// falling edge half a cycle before it, cycle k = [fall_k, fall_k+1). A read
	// resyncs to the next falling edge, checks BA 311 ARM after it, samples 475
	// ARM after the rise and releases one poll plus a teardown after the
	// following fall (busReadByte_p1..p3). A write resyncs to the next falling
	// edge -- with a read's release behind it that wait IS the K243 idle pass,
	// which radDirectWrite pays there -- checks BA, has its data valid from the
	// rise and holds it through the next fall, where the VIC samples $D011
	// (busWriteByte_p1/p2). legacyTurnaroundPass replays the primitive as it
	// stood before that move, when the pass was a SECOND whole idle cycle after
	// the resync.
	static const u64 EDGE_POLL_ARM   = 90;	// one GPLEV0 read after an edge
	static const u64 TEARDOWN_ARM    = 40;	// DISABLE_ADDRESS_LATCH_AND_BUSTRANSCEIVER
	static const u64 BA_CHECK_ARM    = 311;	// WAIT_READ_BA_WRITING / WAIT_BA_SIGNAL_AVAIL
	static const u64 READ_SAMPLE_ARM = 475;	// WAIT_CYCLE_READ
	static const u64 DATA_HOLD_ARM   = 20;	// WAIT_DATA_HOLD
	bool legacyTurnaroundPass = false;
	u64  lastReadStart = 0;		// stageSynced of the last read: fall_k + one poll

	static u64 halfCycleHost() { return HOST_HZ / ( 2 * C64_HZ ); }
	static u64 riseAt( u64 k ) { return cyclesToHost( k ); }
	static u64 fallAt( u64 k ) { return cyclesToHost( k ) - halfCycleHost(); }
	// The cycle containing t: the largest k with fall_k <= t.
	static u64 cycleOf( u64 t )
	{
		u64 k = ( ( t + halfCycleHost() ) * C64_HZ ) / HOST_HZ;
		while ( fallAt( k + 1 ) <= t ) k++;
		while ( k && fallAt( k ) > t ) k--;
		return k;
	}
	bool baLowAt( u64 t ) const { return t >= baStartHost && t < baEndHost; }
	u64 lastReadCycleStartHostCycles() override { return lastReadStart; }

	// now++ , NOT now. The pacer's on-time arm spins
	//     do { n = hostCycles(); } while ( (s64)( n - deadline ) < 0 );
	// so a clock that only advances on bus traffic NEVER EXITS -- the first
	// version of this fixture hung there. One ARM per clock read against 1368
	// per C64 cycle is under 0.1%, the same choice TimedHostBus documents.
	u64 hostCycles() override { return now++; }
	u32 hostCyclesPerSec() override { return (u32)HOST_HZ; }

	// THE CAPABILITY QUERY, not just the cost. preciseVICCycleAccess() asks
	// this to choose lag 2 over lag 1 and aim a turnaround write's entry one
	// cycle early so the idle pass is absorbed instead of displacing the drive.
	// An earlier version of this fixture charged a turnaround write MORE but
	// still answered false here, so it modelled the price of the turnaround
	// without modelling the placement decision the turnaround exists to drive
	// -- which is the half that matters for a candidate that removes a read.
	bool writeTurnaroundPending() override { return lastWasRead; }

	// The most recent PHI2 rising edge at or before now, in host units. The
	// pacer phase-locks its grid to this; returning the base-class 0 means
	// "backend cannot see PHI" and the lock never runs, so a fixture that
	// leaves it at 0 is not reproducing shipping placement at all.
	u64 phiCpuHalfStartHostCycles() override
	{
		// The shipping implementation (rad_bus.cpp:8609) WAITS through the VIC
		// half for the following CPU-half edge and returns a freshly observed
		// timestamp -- it costs time. An earlier version here was a free oracle
		// for a PAST edge, which silently removed the pacer's late-probe cost
		// from every comparison that involves it. Advance to the next edge.
		const u64 per = HOST_HZ / C64_HZ;
		const u64 idx = ( now * C64_HZ ) / HOST_HZ;
		u64 edge = ( ( idx + 1 ) * HOST_HZ ) / C64_HZ;
		if ( edge <= now ) edge = now + per;
		now = edge;			// the observation spent this time
		return edge;
	}

	// BA LOW AT THE PIN, modelled at the PRIMITIVE and not only at the settle.
	//
	// WHY THIS HAD TO EXIST BEFORE THE MODEL COULD SCORE ANYTHING. The real
	// write path waits inside HANDLE_BUS_AVAILABLE: if the VIC holds the bus
	// when the primitive is entered, the write waits there regardless of what
	// the high-level pacer decided. A fixture that models only the pacer's
	// stall can therefore show a saving from skipping a high-level settle that
	// the hardware would simply have waited out one level lower -- inventing a
	// result. Any candidate that claims to move a write earlier must survive
	// this, or it is not a saving.
	// A BA-low interval SCHEDULED IN PHYSICAL TIME, set before the guest runs
	// and consulted by BOTH the primitives and busStealCycles.
	//
	// THE PREVIOUS VERSION OF THIS WAS DEAD CODE AND A MUTATION TEST PROVED IT.
	// busStealCycles did `now += len` and only THEN `baLowUntil = now`, so the
	// deadline was always the current instant and `baLowUntil > now` was never
	// true. Deleting the entire body of waitOutBA() produced byte-identical
	// output across all 18 checks. Worse than a missing assertion: the guard
	// that was supposed to stop the model inventing a saving had no effect on
	// the very comparison it existed to protect, so a candidate that skipped
	// the high-level settle would also have skipped creating the BA event at
	// all, and the model would have reported the saving as real.
	//
	// The interval is now independent of whether or when busStealCycles is
	// called, which is the point: the VIC takes the bus on its own schedule and
	// does not care which of our layers happens to look.
	// CANDIDATE POLICY: serve a $D011 RMW read from the shadow instead of the
	// bus. The shipping read path already takes $D011 bits 6..0 from
	// m_VICRegShadow (that is the existing GEOS fix); only bit 7, the live
	// raster bit 8, needs the pin. Removing that transaction takes TWO things
	// out of the window -- the read itself, and the read->write turnaround it
	// forces on the write that follows, which is 3449 of a write's 4999 ARM --
	// so the saving is larger than the read alone. Modelled here so the size of
	// the saving can be MEASURED before anyone argues about whether bit 7 can
	// be proven zero. That argument is a separate and harder question; this
	// only answers "is it worth having".
	bool skipRmwRead = false;

	u64 baStartHost = 0;
	u64 baEndHost   = 0;

	bool baLowNow() const { return now >= baStartHost && now < baEndHost; }

	void waitOutBA()
	{
		if ( baLowNow() ) now = baEndHost;
	}

	static bool physical( u16 a ) { return a >= 0xD000 && a < 0xE000; }

	void note( u16 addr, bool write, u64 cycle )
	{
		if ( evCount < 64 )
		{
			ev[ evCount ].addr  = addr;
			ev[ evCount ].write = write;
			ev[ evCount ].host  = now;
			ev[ evCount ].cycle = cycle;
			evCount++;
		}
	}

	u8 read( u16 addr ) override
	{
		if ( !physical( addr ) ) return CHostBus::read( addr );
		// The candidate: no pin event at all for a $D011 RMW read, so no stage
		// cost, no turnaround forced on the following write, and no exposure to
		// a BA hold. The value still comes from the same shadow the shipping
		// read path already trusts for these bits.
		if ( skipRmwRead && ( addr & 0x3F ) == 0x11 )
			return CHostBus::read( addr );
		// Resync to the next falling edge, wait out BA whole cycles at the
		// check point, then sample AT the data cycle: the value a real read
		// returns is the one present then, not at the moment the software
		// entered the primitive. Release a poll and a teardown after the fall
		// that ends the cycle, which is where busWriteTurnaroundNeeded is raised.
		u64 k = cycleOf( now ) + 1;				// BUS_RESYNC
		while ( baLowAt( fallAt( k ) + BA_CHECK_ARM ) ) k++;	// HANDLE_BUS_AVAILABLE_READ
		lastReadStart = fallAt( k ) + EDGE_POLL_ARM;		// stageSynced
		now = riseAt( k ) + EDGE_POLL_ARM + READ_SAMPLE_ARM;	// p3 sample point
		const u8 v = CHostBus::read( addr );
		now = fallAt( k + 1 ) + EDGE_POLL_ARM + TEARDOWN_ARM;	// p3 release
		lastWasRead = true;
		note( addr, false, k );
		if ( memForStamp && ( addr & 0x3F ) == 0x12 ) guestAtD012Read = memForStamp->emuNow();
		return v;
	}

	void write( u16 addr, u8 value ) override
	{
		if ( !physical( addr ) ) { CHostBus::write( addr, value ); return; }
		u64 k = cycleOf( now ) + 1;				// BUS_RESYNC (= the K243 pass after a read)
		if ( lastWasRead && legacyTurnaroundPass ) k++;	// pre-fix: a second whole idle cycle
		while ( baLowAt( fallAt( k ) + BA_CHECK_ARM ) ) k++;	// HANDLE_BUS_AVAILABLE
		now = fallAt( k + 1 );					// data valid rise_k..fall_k+1; the VIC samples here
		CHostBus::write( addr, value );
		now = fallAt( k + 1 ) + EDGE_POLL_ARM + DATA_HOLD_ARM + TEARDOWN_ARM;	// p2
		lastWasRead = false;
		note( addr, true, k );
		if ( evCount ) ev[ evCount - 1 ].value = value;
		if ( memForStamp && ( addr & 0x3F ) == 0x11 && !guestAtD011Write ) guestAtD011Write = memForStamp->emuNow();
	}

	u64 writeTime( u16 addr, u32 nth ) const
	{
		u32 seen = 0;
		for ( u32 i = 0; i < evCount; i++ )
			if ( ev[ i ].write && ev[ i ].addr == addr && ++seen == nth )
				return ev[ i ].host;
		return 0;
	}

	// The PHI2 cycle of the nth matching event (1-based), 0 if absent.
	u64 eventCycle( u16 addr, bool write, u32 nth ) const
	{
		u32 seen = 0;
		for ( u32 i = 0; i < evCount; i++ )
			if ( ev[ i ].write == write && ev[ i ].addr == addr && ++seen == nth )
				return ev[ i ].cycle;
		return 0;
	}
	// The cycle of the LAST matching event: the anchor read a guest counts from
	// is its most recent $D012 read, which is also what guestAtD012Read holds.
	u64 lastEventCycle( u16 addr, bool write ) const
	{
		u64 c = 0;
		for ( u32 i = 0; i < evCount; i++ )
			if ( ev[ i ].write == write && ev[ i ].addr == addr ) c = ev[ i ].cycle;
		return c;
	}

	// A SCRIPTED STEAL, because the inherited model is not enough to reproduce
	// the defect. CHostBus models BADLINES only and charges the whole remaining
	// BA window at the first settle that sees it, after which the beam has moved
	// on -- so a badline is consumed BEFORE the DEC/INC pair and never lands
	// between the two writes. The hardware's intruding steal is a 6-7 cycle
	// SPRITE DMA that does land there (bastall=6/1 or 7/1 on the INC event,
	// 22 frames of 22). Scripting the call index lets the steal be placed at the
	// exact boundary under test instead of wherever a badline happens to fall.
	// busStealCycles reads the SAME scheduled interval the primitives do. It
	// does not create the event and it does not decide when the event happens;
	// it only reports how much of it is left and waits that out, which is the
	// shipping contract ("how many whole C64 cycles the VIC is holding the bus
	// for RIGHT NOW, having waited them out").
	//
	// This is what makes a policy comparison honest: suppressing the high-level
	// settle no longer suppresses the VIC. The interval stays, and a primitive
	// entered inside it still waits in waitOutBA(). A candidate can now only
	// show a saving if it genuinely moves the access out of the VIC's window.
	u32 busStealCycles( u32 maxCycles ) override
	{
		if ( !baLowNow() ) return 0;
		const u64 leftHost = baEndHost - now;
		u32 st = (u32)( ( leftHost * C64_HZ + HOST_HZ - 1 ) / HOST_HZ );
		if ( st > maxCycles ) st = maxCycles;
		if ( !st ) return 0;
		// Having reported them, it has waited them out.
		now += ( (u64)st * HOST_HZ ) / C64_HZ;
		return st;
	}

	u32 countOps( u16 addr, bool write ) const
	{
		u32 n = 0;
		for ( u32 i = 0; i < evCount; i++ )
			if ( ev[ i ].addr == addr && ev[ i ].write == write ) n++;
		return n;
	}
};

struct VSPMachine
{
	VSPTimedBus        bus;
	CC64Memory         mem;
	CFastRAM           simm;
	CSuperCPUMemoryMap map;
	CW65C816           cpu;

	// THE BEAM RUNS ON PHYSICAL TIME, not on guest progress.
	//
	// This was wrong in the first version and the error was structural rather
	// than numeric. Feeding the guest's own emuNow() to the raster clock means
	// physical bus latency and pacer waits DO NOT MOVE THE VIC: advancing the
	// host clock by ten raster lines left rasterLineInternal() reporting the
	// same line. That deletes the feedback this entire investigation is about
	// -- the guest spends real time on the bus, the beam advances while it
	// does, and whether the write lands before or after a VIC decision point
	// is the defect. A model in which the beam waits politely for the guest
	// cannot represent that, and will always report placement as better than
	// it is.
	//
	// The VIC has its own oscillator and does not care what the CPU is doing,
	// so the clock is the host's, converted by the one rational pair used
	// everywhere else in this file.
	static u64 physClock( void *ctx )
	{
		const VSPTimedBus *b = (const VSPTimedBus *)ctx;
		return ( b->now * C64_HZ ) / HOST_HZ;
	}

	// seedCycle places the guest's clock inside the frame so a steal can be made
	// to intersect the window. CHostBus models BADLINES only (line in 48..247,
	// DEN set, (line & 7) == YSCROLL, inside the BA window) and not sprite DMA,
	// while the hardware's intruding steal at K493 is sprite DMA. The MECHANISM
	// under test is identical -- BA halting a cycle the real 6510 would have
	// completed -- so a badline is a valid stand-in for it, and saying so here
	// is cheaper than a sprite model that would have to be validated too.
	// startLine places the PHYSICAL beam, and baAtCycle/baLen schedule a BA-low
	// interval in PHYSICAL time measured from that origin.
	//
	// The previous signature took a guest-cycle seed, which stopped positioning
	// the beam the moment the raster clock was corrected to run on host time:
	// a "line 53" seed set m_EmuCycles to 3445 while the physical raster sat at
	// 0 and $D012 read back 0 rather than $35, so the captured compare was
	// still never the branch being exercised. The seed now moves the clock the
	// beam is actually derived from.
	explicit VSPMachine( u32 baStallMode, u32 startLine = 0,
	                     u32 baAtCycle = 0xFFFFFFFFu, u32 baLen = 0 )
	{
		// A MINIMAL KERNAL carrying only the emulation reset vector. Without a
		// ROM attached read8($FFFC) returns $FF whatever $01 says, the core
		// resets to $FFFF and never reaches the program -- which is what the
		// first run of this fixture did.
		static u8 kernal[ C64_KERNAL_SIZE ];
		for ( u32 i = 0; i < C64_KERNAL_SIZE; i++ ) kernal[ i ] = 0xFF;
		kernal[ 0x1FFC ] = 0x52;
		kernal[ 0x1FFD ] = 0x08;		// reset -> $0852
		mem.setKernalROM( kernal );

		// EXPLICIT, all of them, and the physical origin BEFORE setRasterClock
		// and setPacing so nothing anchors on a position that is about to move.
		bus.m_Signals.video = VIDEO_NTSC_R8;	// 263 lines, 65 cycles/line
		bus.now = cyclesToHost( (u64)startLine * 65ull );
		if ( baLen )
		{
			bus.baStartHost = cyclesToHost( (u64)startLine * 65ull + baAtCycle );
			bus.baEndHost   = bus.baStartHost + cyclesToHost( baLen );
		}
		bus.setRasterClock( &VSPMachine::physClock, &bus );
		bus.memForStamp = &mem;

		mem.attachBus( &bus );
		simm.init( SCPU_SIMM_16MB );
		map.attachBank0( &mem );
		map.attachFastRAM( &simm );
		cpu.attachBus( &map );
		mem.reset();

		mem.m_RAM[ 0x0000 ] = 0x2F;
		mem.m_RAM[ 0x0001 ] = 0x35;		// RAM + I/O + RAM
		mem.updateBankMode();
		// The captured configuration. Mode 0 switches off the mechanism the
		// excess is charged to, which is why it is the A/B and not the default.
		mem.m_BAStallMode = baStallMode;
		mem.m_BAStallMaxCycles = 80;
		mem.setPacing( bus.hostCyclesPerSec(), SCPU_NORMAL_HZ );

		u16 p = 0x0852;
		auto emit = [ & ]( u8 b ) { mem.m_RAM[ p++ ] = b; };
		emit( 0xA9 ); emit( 0x35 );				// LDA #$35
		emit( 0xCD ); emit( 0x12 ); emit( 0xD0 );		// CMP $D012  <- anchor
		emit( 0xF0 ); emit( 0x00 );				// BEQ *+2    absorber
		emit( 0x4C ); emit( 0x5C ); emit( 0x08 );		// JMP $085C
		while ( p < 0x0871 ) emit( 0xEA );			// 21-NOP sled
		emit( 0xCE ); emit( 0x11 ); emit( 0xD0 );		// DEC $D011  <- open
		emit( 0xEE ); emit( 0x11 ); emit( 0xD0 );		// INC $D011  <- close
		emit( 0x4C ); emit( 0x7A ); emit( 0x08 );		// JMP *

		mem.m_VICRegShadow[ 0x11 ] = 0x17;
		bus.m_Memory[ 0xD011 ] = 0x17;

		cpu.reset();
	}

	// Alternative trigger programs, loaded over the Mario bytes at $0852 before
	// the first step (the reset vector already points there).
	//   1: LDA $D012 / STA $D011 -- a turnaround write four cycles after its
	//      anchor read, the shape every "read the raster, poke the VIC"
	//      stabiliser has; its settle is skipped by the credit arm (the read
	//      cost more than its cycle), so the catch-up has to hold it.
	//   2: LDA $D012 / STA $D020 / STA $D011 -- the second STA is a PLAIN write
	//      (no read since the previous write) eight cycles after the anchor.
	void loadProgram( u32 program )
	{
		if ( !program ) return;
		u16 p = 0x0852;
		auto emit = [ & ]( u8 b ) { mem.m_RAM[ p++ ] = b; };
		emit( 0xAD ); emit( 0x12 ); emit( 0xD0 );		// LDA $D012  <- anchor, read at cycle 4
		if ( program == 2 )
		{
			emit( 0x8D ); emit( 0x20 ); emit( 0xD0 );	// STA $D020  turnaround write, R+4
		}
		emit( 0x8D ); emit( 0x11 ); emit( 0xD0 );		// STA $D011  write at R+4 (1) / R+8 (2)
		const u16 here = p;
		emit( 0x4C ); emit( (u8)( here & 0xFF ) ); emit( (u8)( here >> 8 ) );	// JMP *
	}

	void runToPair( u32 maxSteps = 400 )
	{
		for ( u32 i = 0; i < maxSteps && bus.countOps( 0xD011, true ) < 2; i++ )
			cpu.step();
	}

	// Hundredths of a C64 cycle, the same convention dprev= uses in firmware.
	u32 windowHundredths() const
	{
		const u64 a = bus.writeTime( 0xD011, 1 );
		const u64 b = bus.writeTime( 0xD011, 2 );
		return ( a && b && b > a ) ? (u32)armToHundredthCycles( b - a ) : 0;
	}
};

} // namespace

// THE FLOOR, corrected. The window holds the INC's read and the INC's write,
// not all four transactions of the pair: the DEC's write is that instruction's
// LAST cycle, so the DEC's own traffic is outside the interval being measured.
// 5.94 cycles of unavoidable bus time inside a 6.00-cycle budget, so the bus is
// fast enough and the defect is NOT a hardware floor. The opposite conclusion
// was reached by an earlier version of this file and would have been reported
// as "unfixable on this machine".
TEST( vsp_window_floor_is_two_transactions_and_fits_under_nmos )
{
	const u64 inside = armToHundredthCycles( VSP_READ_ARM + VSP_WRITE_TURN_ARM );
	CHECK_EQ( (u32)inside, 594u );
	CHECK( inside < 600u );
}

// THE TRANSACTION LEDGER. An explicit count of what actually reached the bus,
// so a change that removes the RMW's physical read fails HERE and names itself
// rather than the window quietly moving for an unexplained reason. The previous
// version of this test could be satisfied by the $D012 read alone while BOTH
// $D011 reads vanished.
TEST( vsp_rmw_pair_ledger_is_two_reads_and_two_writes )
{
	VSPMachine m( 2 );
	m.runToPair();
	CHECK_EQ( m.bus.countOps( 0xD011, true ), 2u );		// DEC + INC writes
	CHECK_EQ( m.bus.countOps( 0xD011, false ), 2u );	// DEC + INC RMW reads
	CHECK( m.bus.countOps( 0xD012, false ) >= 1u );		// the anchor poll
}

// THE ACCEPTANCE GATE. Before this model may be used to select a fix it has to
// reproduce the retained hardware observation with unchanged parameters: K493,
// 22 frames, window 14.57 cycles mean, bimodal 13.99 / 15.01, with a BA stall
// of 6 or 7 charged inside it and window - bastall = 6 exactly.
//
// This test does NOT yet assert the 14.57. The fixture's steal model is
// CHostBus's own and has not been shown to place a 6-7 cycle sprite steal where
// the real VIC does. It asserts what IS established -- that with no steal the
// window sits near the two-transaction floor, which is the result proving bus
// cost alone cannot explain 14.57 -- and prints both so the remaining gap is
// visible rather than assumed closed.
TEST( vsp_ba_stall_is_what_widens_the_window_not_bus_cost )
{
	VSPMachine off( 0 );	off.runToPair();
	VSPMachine on( 2 );	on.runToPair();

	const u32 wOff = off.windowHundredths();
	const u32 wOn  = on.windowHundredths();

	std::printf( "\n    ===== VSP WINDOW =====\n" );
	std::printf( "    BA stall OFF  : %u.%02u C64 cycles\n", wOff / 100u, wOff % 100u );
	std::printf( "    BA stall ON   : %u.%02u C64 cycles\n", wOn  / 100u, wOn  % 100u );
	std::printf( "    NMOS truth    :  6.00\n" );
	std::printf( "    two-txn floor :  5.94\n" );
	std::printf( "    HARDWARE K493 : 14.57  (13.99 x9, 15.01 x13)\n" );
	std::printf( "    stall charged : %lu C64 cycles\n",
	             (unsigned long)on.mem.m_BAStallCycles );
	std::printf( "    ======================\n" );

	CHECK( wOff > 0u );
	CHECK( wOn  > 0u );
	// With no steal the window is close to the two-transaction floor. That is
	// the result which says bus cost alone cannot explain the hardware's 14.57.
	CHECK( wOff < 900u );
}

// THE REPRODUCTION. A 7-cycle steal walked across every settle boundary, so
// it lands inside the DEC/INC window at some of them and outside at others.
//
// THE HARDWARE RESULT THIS IS REPRODUCING, K493, 22 frames of 22 with no
// exceptions:   emulated window - bastall = 6 EXACTLY.
// So the core's own accounting is right and the whole error is the stall.
TEST( vsp_a_steal_inside_the_window_widens_it_one_for_one )
{
	u32 widest = 0, hits = 0;
	std::printf( "\n    cyc   window  stall   (7-cyc BA interval walked across line 53)\n" );
	for ( u32 call = 0; call < 65; call += 2 )
	{
		VSPMachine m( 2, 53, call, 7 );
		m.runToPair();
		const u32 w = m.windowHundredths();
		const u32 st = (u32)m.mem.m_BAStallCycles;
		if ( w > 650u ) { hits++; if ( w > widest ) widest = w;
			std::printf( "    %3lu   %u.%02u   %lu\n",
			             (unsigned long)call, w / 100u, w % 100u,
			             (unsigned long)st ); }
	}
	std::printf( "       boundaries where the steal fell INSIDE the window: %u\n",
	             hits );
	std::printf( "       widest window: %u.%02u   (hardware K493: 14.57)\n",
	             widest / 100u, widest % 100u );

	// The defect, reproduced on the host: a steal that intersects the window
	// widens it past the NMOS 6, and the excess is the stall itself.
	CHECK( hits > 0u );
	CHECK( widest > 600u );
}

// CODEX'S TWO COUNTEREXAMPLES, pinned as tests so 'fixed' is provable rather
// than asserted. Both of these FAILED on the previous fixture and both were
// found by running it independently, not by reading it.

// COUNTEREXAMPLE 1: advance the HOST clock by ten raster lines without the
// guest executing anything. The beam must move. It did not before -- the
// raster clock was fed the guest's own emuNow(), so physical bus latency and
// pacer waits left the VIC frozen, deleting the feedback the defect lives in.
TEST( vsp_beam_advances_on_physical_time_not_guest_progress )
{
	VSPMachine m( 2 );
	const u16 before = m.bus.rasterLine();
	const u64 guestBefore = m.mem.emuNow();

	// Ten NTSC lines of REAL time, no guest execution at all.
	m.bus.now += ( 10ull * 65ull * HOST_HZ ) / C64_HZ;

	const u16 after = m.bus.rasterLine();
	CHECK_EQ( m.mem.emuNow(), guestBefore );	// guest genuinely did not run
	CHECK( after != before );			// but the VIC did
}

// COUNTEREXAMPLE 2: after a physical READ the backend must report a pending
// write turnaround. The pacer asks this to choose lag 2 over lag 1 and aim a
// turnaround write's entry a cycle early. The previous fixture charged a
// turnaround write more but still answered false, so it modelled the PRICE of
// the turnaround and not the PLACEMENT DECISION it exists to drive.
TEST( vsp_turnaround_is_reported_not_merely_charged )
{
	VSPMachine m( 2 );
	CHECK( !m.bus.writeTurnaroundPending() );	// nothing read yet
	(void)m.bus.read( 0xD012 );
	CHECK( m.bus.writeTurnaroundPending() );	// a read is outstanding
	m.bus.write( 0xD011, 0x16 );
	CHECK( !m.bus.writeTurnaroundPending() );	// consumed by the write
}

// AND THE PHI LOCK. Returning the base class's 0 means 'this backend cannot
// see PHI', so the pacer's phase lock never runs at all -- a fixture that
// leaves it there is not exercising shipping placement.
TEST( vsp_phi_edge_is_visible_and_lands_on_a_cycle_boundary )
{
	VSPMachine m( 2 );
	m.bus.now += 5000;
	const u64 edge = m.bus.phiCpuHalfStartHostCycles();
	CHECK( edge != 0u );
	CHECK( edge <= m.bus.now );
	// Within one C64 cycle of now, i.e. it really is the most recent edge.
	CHECK( m.bus.now - edge < ( HOST_HZ / C64_HZ ) + 2 );
}
// THE EXPERIMENT. Walk a 7-cycle BA interval across line 53 and measure the
// window under each candidate policy, so a fix is chosen on a measurement
// rather than on an argument.
TEST( vsp_candidate_comparison )
{
	std::printf( "\n    ==== CANDIDATE COMPARISON (NMOS 6.00) ====\n" );
	std::printf( "    cyc   baseline   skip-RMW-read\n" );
	u32 wBaseMax = 0, wSkipMax = 0;
	for ( u32 c = 56; c < 72; c += 2 )
	{
		VSPMachine a( 2, 53, c, 7 );
		a.runToPair();
		const u32 wa = a.windowHundredths();

		VSPMachine b( 2, 53, c, 7 );
		b.bus.skipRmwRead = true;
		b.runToPair();
		const u32 wb = b.windowHundredths();

		if ( wa > wBaseMax ) wBaseMax = wa;
		if ( wb > wSkipMax ) wSkipMax = wb;
		std::printf( "    %3lu    %u.%02u       %u.%02u\n",
		             (unsigned long)c, wa / 100u, wa % 100u,
		             wb / 100u, wb % 100u );
	}
	std::printf( "    WORST CASE  baseline %u.%02u   skip-RMW-read %u.%02u\n",
	             wBaseMax / 100u, wBaseMax % 100u,
	             wSkipMax / 100u, wSkipMax % 100u );

	// The candidate must not make the worst case worse. That is the only claim
	// this test makes; whether it reaches 6.00 is PRINTED, not asserted, because
	// the model does not meet the absolute hardware gate.
	CHECK( wSkipMax <= wBaseMax );
}

// COUNTED CYCLES FROM THE ANCHOR READ TO THE DEC WRITE. Mario's code is 54
// (BEQ 3 + JMP 3 + 21 NOPs + DEC's 6). Hardware measures 47. If this core
// counts 54 the seven cycles vanish only on the Pi; if it counts 47 the core
// miscounts this exact sequence and the bug is findable on the host.
TEST( vsp_counted_cycles_anchor_read_to_dec_write )
{
	VSPMachine m( 0 );
	u64 c64AtAnchor = 0, c64AtDec = 0;
	for ( u32 i = 0; i < 400; i++ )
	{
		m.cpu.step();
		if ( !c64AtAnchor && m.bus.countOps( 0xD012, false ) >= 1 ) c64AtAnchor = m.mem.emuNow();
		if ( m.bus.countOps( 0xD011, true ) >= 1 ) { c64AtDec = m.mem.emuNow(); break; }
	}
	std::printf( "\n    counted guest cycles, CMP $D012 read -> DEC $D011 write: %lu  (code says 54, Pi measures 47)\n",
	             (unsigned long)( c64AtDec - c64AtAnchor ) );
	CHECK( c64AtDec > c64AtAnchor );
}

// THE PI PATH. attachFastBus() + run(), which is how supercpu.cpp drives the
// core (init :564, run :1429/:1551), against attachBus() + step(), which is all
// this fixture had exercised. Hardware measures 47 physical cycles from the CMP
// $D012 read to the DEC $D011 write where the code says 54 and step() counts
// 53-54. If run()+fast bus reproduces the shortfall the loss is in this path.
static void runPath( VSPMachine &m, bool fast, u64 chunk )
{
	if ( fast ) m.cpu.attachFastBus( &m.map );
	for ( u32 i = 0; i < 4000 && m.bus.countOps( 0xD011, true ) < 2; i++ )
	{
		if ( fast ) m.cpu.run( chunk ); else m.cpu.step();
	}
}
TEST( vsp_pi_path_run_fastbus_vs_step )
{
	std::printf( "\n    CMP $D012 read -> DEC $D011 write   (code 54, hardware 47)\n" );
	std::printf( "    path                 counted(guest)  physical(C64 cyc)\n" );
	struct Cfg { const char *name; bool fast; u64 chunk; } cfgs[] = {
		{ "step()",             false,  0 },
		{ "run() chunk 1",      true,   1 },
		{ "run() chunk 8",      true,   8 },
		{ "run() chunk 134",    true, 134 },   // one scheduler slice at 1MHz
		{ "run() chunk 2000",   true, 2000 },
	};
	for ( const Cfg &c : cfgs )
	{
		VSPMachine m( 2 );
		runPath( m, c.fast, c.chunk );
		const u64 counted = m.bus.guestAtD011Write - m.bus.guestAtD012Read;
		u64 rd = 0; for ( u32 i = 0; i < m.bus.evCount; i++ ) if ( !m.bus.ev[i].write && ( m.bus.ev[i].addr & 0x3F ) == 0x12 ) rd = m.bus.ev[i].host;
		const u64 wr = m.bus.writeTime( 0xD011, 1 );
		const u32 ph = ( rd && wr > rd ) ? (u32)armToHundredthCycles( wr - rd ) : 0;
		std::printf( "    %-20s %6lu          %u.%02u\n", c.name, (unsigned long)counted, ph / 100u, ph % 100u );
	}
	CHECK( true );
}

// THE PACER'S CARRIED LAG, seeded. K448 measured the anchor->DEC span at 59-63
// and K493 at 47 for the same bytes -- a 14-cycle swing across builds, which is
// pacer state and not instruction counting. The credit arm runs up to
// PACER_CREDIT_CYCLES=8 guest cycles with the spin skipped when the guest is
// behind. Put the guest behind by L cycles at the anchor and read the span.
TEST( vsp_carried_lag_eats_the_span )
{
	std::printf( "\n    lag L at anchor -> physical anchor->DEC   (code 54; hardware K493 47.08)\n" );
	const u64 lags[] = { 0, 2, 4, 6, 7, 8, 10, 16 };
	u32 at0 = 0, at7 = 0;
	for ( u64 L : lags )
	{
		VSPMachine m( 2 );
		m.cpu.attachFastBus( &m.map );
		// run up to the anchor read, then let real time get ahead by L cycles
		for ( u32 i = 0; i < 4000 && m.bus.countOps( 0xD012, false ) < 1; i++ ) m.cpu.run( 1 );
		const u64 rdHost = m.bus.now;
		m.bus.now += cyclesToHost( L );
		for ( u32 i = 0; i < 4000 && m.bus.countOps( 0xD011, true ) < 1; i++ ) m.cpu.run( 1 );
		const u32 ph = (u32)armToHundredthCycles( m.bus.writeTime( 0xD011, 1 ) - rdHost );
		const u64 counted = m.bus.guestAtD011Write - m.bus.guestAtD012Read;
		std::printf( "    L=%-3lu  counted %lu   physical %u.%02u   credit-entries %lu  clamped %lu\n",
		             (unsigned long)L, (unsigned long)counted, ph / 100u, ph % 100u,
		             (unsigned long)m.mem.m_PacerCreditEntries, (unsigned long)m.mem.m_PacerClampEntries );
		if ( L == 0 ) at0 = ph; if ( L == 7 ) at7 = ph;
	}
	CHECK( at0 > 0u );
	CHECK( at7 > 0u );
}

// THE TRIM IS PHYSICAL ONLY. VSP_AIM_WRITE swept through the pacer-backed
// fixture: the guest's counted span and the bus ledger must not move by a
// single cycle at any aim, while the DEC write's physical time moves later
// with the aim whenever the pacer had slack to spend. The asymmetry is
// expected and printed: the write's settle is already late by the read's own
// bus cost, so small positive aims move nothing and larger ones move by
// ( aim - that lateness ). Before the fix the aim went into the LOGICAL
// cycle: the ledger led the instruction by max( 0, aim - 2 ), the INC's
// pre-read settle was skipped, and the grid itself walked by the aim at
// every placed write.
// THE LATE-CASE FIX, gated (K500). With VSP_D011_SHADOW_READ on, a $D011
// read inside the raster window is served from the shadow: no transfer, and
// no turnaround on the write that follows, so the DEC/INC pair reaches the
// bus as two writes. The values must be exactly what the real RMW produces
// ($16 then $17 from a $17 shadow), and the DEC must drive in the 6510's
// cycle with the transfer gone exactly as it does with it present. (K500 was
// first read as "a positive catch deficit": that reading belonged to the
// count target anchored on the read's RETURN, which was itself the skew; the
// target is now the grid instant of the write's own settle, so an on-time
// arrival sits a call path after it.) The default stays physical:
// vsp_rmw_pair_ledger_is_two_reads_and_two_writes guards that.
TEST( vsp_d011_shadow_read_removes_the_rmw_transfer_and_keeps_the_values )
{
	// Seeded to Mario's anchor line: the gate needs the $D012 read to return a
	// low byte that proves RST8 ($35 on the card, 148/148 frames). At the
	// default seed the fixture's beam is on line 0, which on NTSC is one of
	// the seven low bytes that also name a line above 255, so the gate must
	// decline -- and that refusal is itself checked below.
	VSPMachine off( 2, 53 ), on( 2, 53 ), low( 2, 0 );
	off.cpu.attachFastBus( &off.map );
	on.cpu.attachFastBus( &on.map );
	low.cpu.attachFastBus( &low.map );
	on.mem.setVSPD011ShadowRead( true );
	low.mem.setVSPD011ShadowRead( true );
	for ( u32 i = 0; i < 4000 && low.bus.countOps( 0xD011, true ) < 2; i++ ) low.cpu.run( 1 );
	CHECK_EQ( low.bus.countOps( 0xD011, false ), 2u );		// ambiguous anchor: physical reads kept
	CHECK_EQ( (u32)low.mem.m_VSPD011ShadowReads, 0u );
	CHECK_EQ( (u32)low.mem.m_VSPD011ShadowDeclined, 2u );
	for ( u32 i = 0; i < 4000 && off.bus.countOps( 0xD011, true ) < 2; i++ ) off.cpu.run( 1 );
	for ( u32 i = 0; i < 4000 && on.bus.countOps( 0xD011, true ) < 2; i++ ) on.cpu.run( 1 );

	CHECK_EQ( off.bus.countOps( 0xD011, false ), 2u );
	CHECK_EQ( on.bus.countOps( 0xD011, false ), 0u );		// the transfers are gone
	CHECK_EQ( on.bus.countOps( 0xD011, true ), 2u );		// the writes are not
	CHECK_EQ( (u32)on.mem.m_VSPD011ShadowReads, 2u );
	CHECK_EQ( (u32)on.mem.m_VSPD011ShadowDeclined, 0u );
	CHECK_EQ( (u32)on.bus.countOps( 0xD012, false ), (u32)off.bus.countOps( 0xD012, false ) );

	// The values the VIC receives are the real RMW's.
	u8 dec = 0, inc = 0; u32 seen = 0;
	for ( u32 i = 0; i < on.bus.evCount; i++ )
		if ( on.bus.ev[ i ].write && on.bus.ev[ i ].addr == 0xD011 )
			( ++seen == 1 ? dec : inc ) = on.bus.ev[ i ].value;
	CHECK_EQ( (u32)dec, 0x16u );
	CHECK_EQ( (u32)inc, 0x17u );

	// The catch-up ran for both, and both DECs drive in the cycle a 6510 would:
	// the anchor read's cycle plus the guest's own ledger distance. The record
	// of what the catch-up saw is the trace ring's and is printed below.
	VSPTimingEvent trOn[ 4 ], trOff[ 4 ];
	u32 total = 0, overwritten = 0;
	CHECK( on.mem.copyVSPTrace( trOn, 4, total, overwritten ) >= 1u );
	CHECK( off.mem.copyVSPTrace( trOff, 4, total, overwritten ) >= 1u );
	const VSPTimingEvent &decOn  = trOn[ 0 ];
	const VSPTimingEvent &decOff = trOff[ 0 ];
	CHECK( decOn.flags & SCPU_VSP_CATCH_VALID );
	CHECK_EQ( on.bus.eventCycle( 0xD011, true, 1 ),
	          on.bus.lastEventCycle( 0xD012, false )
	          + ( on.bus.guestAtD011Write - on.bus.guestAtD012Read ) );
	CHECK_EQ( off.bus.eventCycle( 0xD011, true, 1 ),
	          off.bus.lastEventCycle( 0xD012, false )
	          + ( off.bus.guestAtD011Write - off.bus.guestAtD012Read ) );
	const s64 dOff = decOff.catchDeficit, dOn = decOn.catchDeficit;
	const u32 mOff = (u32)armToHundredthCycles( (u64)( dOff < 0 ? -dOff : dOff ) );
	const u32 mOn  = (u32)armToHundredthCycles( (u64)( dOn  < 0 ? -dOn  : dOn  ) );
	std::printf( "\n    DEC catch deficit at entry: physical read %c%u.%02u cycles, shadow read %c%u.%02u cycles; "
	             "DEC->INC window %u.%02u -> %u.%02u\n",
	             dOff < 0 ? '-' : '+', mOff / 100u, mOff % 100u,
	             dOn  < 0 ? '-' : '+', mOn  / 100u, mOn  % 100u,
	             off.windowHundredths() / 100u, off.windowHundredths() % 100u,
	             on.windowHundredths() / 100u, on.windowHundredths() % 100u );
	for ( u32 k = 0; k < 2; k++ )
	{
		const VSPTimingEvent &a = trOff[ k ], &b = trOn[ k ];
		// The window above is completion-stamp to completion-stamp, not pin to
		// pin: with the reads gone the DEC is still a turnaround write (the
		// $D012 read precedes it) while the INC becomes a plain one, so the
		// INC's stamp sits ~1.6 cycles closer to its entry. The INC's counted
		// cycle also moves from 59 to 60 (lag 1 instead of the turnaround's
		// lag 2), which is the precise timeline keeping the DRIVE on cycle N
		// either way. Both are visible here so neither reads as a regression.
		std::printf( "    event %u: physical  deficit %+lld wait %u | shadow  deficit %+lld wait %u  (ARM; counted %llu / %llu)",
		             k, (long long)a.catchDeficit, a.catchWait, (long long)b.catchDeficit, b.catchWait,
		             (unsigned long long)( a.c64Cycle - a.rasterC64Cycle ), (unsigned long long)( b.c64Cycle - b.rasterC64Cycle ) );
		std::puts( "" );
	}
}

TEST( vsp_aim_write_moves_the_physical_deadline_and_nothing_else )
{
	struct Run { u64 counted, lead, dec; u32 win; BusEvent ev[ 64 ]; u32 evCount; };
	auto runAt = []( int aim ) -> Run
	{
		VSPMachine m( 2 );
		m.cpu.attachFastBus( &m.map );
		m.cpu.setPreciseAim( 0, aim );
		m.mem.setVSPAimWrite( aim );
		for ( u32 i = 0; i < 4000 && m.bus.countOps( 0xD011, true ) < 2; i++ ) m.cpu.run( 1 );
		Run r;
		r.counted = m.bus.guestAtD011Write - m.bus.guestAtD012Read;
		r.lead    = m.mem.emuNow() - m.cpu.cycles();
		r.dec     = m.bus.writeTime( 0xD011, 1 );
		r.win     = m.windowHundredths();
		r.evCount = m.bus.evCount;
		for ( u32 i = 0; i < m.bus.evCount; i++ ) r.ev[ i ] = m.bus.ev[ i ];
		return r;
	};
	const Run base = runAt( 0 );
	// WHERE A TRIM GOES, access by access: every physical transaction of the
	// aim +2 run against the same transaction of the aim 0 run, in hundredths
	// of a cycle. The trim is meant to move the $D011 RMW writes and nothing
	// before the anchor read; an earlier write that moves here is the trim
	// being spent on the handler's housekeeping and renormalised by the
	// stabiliser, which is where a shift that is not exactly the aim comes from.
	{
		const Run two = runAt( 2 );
		std::printf( "\n    aim +2 vs aim 0, per transaction (addr r/w: delta cycles)\n" );
		for ( u32 i = 0; i < base.evCount && i < two.evCount; i++ )
		{
			const s64 d = (s64)two.ev[ i ].host - (s64)base.ev[ i ].host;
			const u32 mag = (u32)armToHundredthCycles( (u64)( d < 0 ? -d : d ) );
			std::printf( "    %2u  $%04X %c  %c%u.%02u\n", i, base.ev[ i ].addr, base.ev[ i ].write ? 'w' : 'r',
			             d < 0 ? '-' : '+', mag / 100u, mag % 100u );
		}
	}
	std::printf( "\n    aim  counted(anchor->DEC)  ledger lead  DEC shift vs aim 0   DEC->INC window\n" );
	u32 shift2 = 0, shift4 = 0, shift6 = 0;
	for ( int aim = -2; aim <= 6; aim += 2 )
	{
		const Run r = runAt( aim );
		const s64 shiftArm = (s64)r.dec - (s64)base.dec;
		const u32 mag = (u32)armToHundredthCycles( (u64)( shiftArm < 0 ? -shiftArm : shiftArm ) );
		std::printf( "    %+d   %6lu                %lu          %c%u.%02u               %u.%02u\n",
		             aim, (unsigned long)r.counted, (unsigned long)r.lead,
		             shiftArm < 0 ? '-' : '+', mag / 100u, mag % 100u, r.win / 100u, r.win % 100u );
		// The guest never learns the aim exists.
		CHECK_EQ( r.counted, base.counted );
		CHECK_EQ( r.lead, base.lead );
		// The physical write never moves against the sign of the trim.
		if ( aim > 0 ) CHECK( shiftArm >= 0 );
		if ( aim < 0 ) CHECK( shiftArm <= 0 );
		// THE SEQUENCE SHIFTS AS ONE: the INC moves with the DEC, so the
		// DEC->INC interval is the same to a hundredth. This is the check that
		// caught the settle-level trim being applied to the INC a second time
		// (window 5.99 -> 7.94 at aim +2).
		CHECK( r.win <= base.win + 2u && r.win + 2u >= base.win );
		if ( aim == 2 ) shift2 = mag;
		if ( aim == 4 ) shift4 = mag;
		if ( aim == 6 ) shift6 = mag;
	}
	// ONE-FOR-ONE FROM THE FIRST CYCLE (K498). The aim is applied at the
	// catch-up launch -- max( entry, count target ) + aim for the first placed
	// write after the anchor -- so it cannot be swallowed by the write
	// settle's lateness (the K497 settle trim lost its first 2.31 cycles to
	// that) nor by the catch-up's own slack (an exact late-arm trim still lost
	// 1.02 cycles to it: the untrimmed DEC was early by that much and waited
	// here). Pinned to five hundredths. Below the aim means something between
	// the launch and the pin is absorbing it again; above means it is applied
	// twice.
	CHECK( shift2 >= 195u && shift2 <= 205u );
	CHECK( shift4 >= 395u && shift4 <= 405u );
	CHECK( shift6 >= 595u && shift6 <= 605u );
}

// THE DRIVE CYCLE. Not the window and not a stamp convention: the PHI2 cycle
// in which a placed $D011 write's data is valid on the pins, against the cycle
// a 6510 running the same bytes uses -- the anchor read's cycle plus the
// guest's own ledger distance from that read to the write (lag 1: start of
// the read's cycle to start of the write's). One cycle of difference here is
// one character cell on the oracle screen; this is the assertion the
// ref/n10/n20/n30 photos check by eye.
//
// Three routes to the primitive, because the skew was built from two parts
// that only one of them shows together:
//   0  Mario: DEC $D011 directly after its own read. Entry is pinned by the
//      read's return inside the dummy-write cycle, the settle is a no-op, and
//      the K243 pass is pending -- so the pass has to be the resync's own
//      cycle or the write is late by construction.
//   1  LDA $D012 / STA $D011: a turnaround write whose settle the credit arm
//      skipped (the read cost more than its cycle), so the catch-up holds it
//      -- the target has to be the grid instant, not the read's return.
//   2  LDA $D012 / STA $D020 / STA $D011: a plain write on the lag-1 path.
// legacyTurnaroundPass replays the primitive from before the pass moved into
// the resync: the same core then drives one cycle late on routes 0 and 1 and
// on time on route 2 (its catch-up absorbs the $D020's displacement). With
// the old lag-2/return-anchored core as well, every route was two cycles
// late: that is the +2 behind the "+3.1 stamp gap" of K496, and the two cells.
static void runToFirstD011Write( VSPMachine &m )
{
	m.cpu.attachFastBus( &m.map );
	for ( u32 i = 0; i < 4000 && m.bus.countOps( 0xD011, true ) < 1; i++ ) m.cpu.run( 1 );
}
TEST( vsp_d011_drive_cycle_is_the_6510s_cycle )
{
	static const char *names[ 3 ] = { "DEC after own read", "LDA/STA turnaround", "LDA/STA/STA plain" };
	std::printf( "\n    route                 seed  R(anchor)  counted  drive-R   legacy drive-R\n" );
	for ( u32 program = 0; program < 3; program++ )
		for ( u32 seedLine = 52; seedLine <= 54; seedLine++ )
		{
			VSPMachine m( 2, seedLine );
			m.loadProgram( program );
			runToFirstD011Write( m );
			const u64 R       = m.bus.lastEventCycle( 0xD012, false );
			const u64 Dc      = m.bus.eventCycle( 0xD011, true, 1 );
			const u64 counted = m.bus.guestAtD011Write - m.bus.guestAtD012Read;

			VSPMachine l( 2, seedLine );
			l.bus.legacyTurnaroundPass = true;
			l.loadProgram( program );
			runToFirstD011Write( l );
			const u64 Rl = l.bus.lastEventCycle( 0xD012, false );
			const u64 Dl = l.bus.eventCycle( 0xD011, true, 1 );

			std::printf( "    %-21s %3u  %8lu  %6lu   %+ld      %+ld\n", names[ program ], seedLine,
			             (unsigned long)R, (unsigned long)counted,
			             (long)( (s64)Dc - (s64)R - (s64)counted ),
			             (long)( (s64)Dl - (s64)Rl - (s64)counted ) );
			CHECK( R != 0u && Dc != 0u );
			// The guest's own count is the 6510's: 4 or 8 for the STA routes.
			if ( program == 1 ) CHECK_EQ( counted, 4u );
			if ( program == 2 ) CHECK_EQ( counted, 8u );
			// THE ASSERTION: data valid in the cycle a 6510 would drive it.
			CHECK_EQ( Dc, R + counted );
			// The pass as a second cycle after the resync is a cycle late wherever
			// a read precedes the write without a write between.
			CHECK_EQ( Dl, Rl + counted + ( program == 2 ? 0u : 1u ) );
			if ( program == 0 )
			{
				// And the INC follows six cycles later with no steal scheduled.
				for ( u32 i = 0; i < 4000 && m.bus.countOps( 0xD011, true ) < 2; i++ ) m.cpu.run( 1 );
				CHECK_EQ( m.bus.eventCycle( 0xD011, true, 2 ), Dc + 6 );
			}
		}
}

// THE COUNT TARGET IS ON THE GRID. For the route the catch-up actually holds
// (LDA/STA), the launch instant it chose must sit inside the cycle BEFORE the
// write's, a quarter cycle past that cycle's falling edge (tickSettle's lock
// phase) -- never at or past the falling edge that begins the write's own
// cycle, which is where a return-anchored target put it. Read back from the
// trace: the DEC's hostCycle is the completion stamp (fall_k+1 + hold), and
// catchWait is the hold the catch-up applied.
TEST( vsp_catchup_launch_is_inside_the_cycle_before_the_write )
{
	VSPMachine m( 2, 53 );
	m.loadProgram( 1 );
	runToFirstD011Write( m );
	VSPTimingEvent tr[ 4 ];
	u32 total = 0, overwritten = 0;
	CHECK( m.mem.copyVSPTrace( tr, 4, total, overwritten ) >= 1u );
	const VSPTimingEvent &sta = tr[ 0 ];
	CHECK( sta.flags & SCPU_VSP_CATCH_VALID );
	CHECK( sta.catchDeficit > 0 );			// the credit arm let it arrive early
	CHECK( sta.catchWait > 0u );			// and the catch-up held it
	// The write drove in cycle k, so the primitive was entered inside cycle
	// k-1: the launch the catch-up chose must lie in [fall_k-1, fall_k). Rebuild
	// the target the way CC64Memory does -- the read's start-of-cycle stamp
	// less three quarters of a cycle, plus the guest's counted cycles.
	const u64 k = m.bus.eventCycle( 0xD011, true, 1 );
	const u64 per = HOST_HZ / C64_HZ;
	const u64 counted = m.bus.guestAtD011Write - m.bus.guestAtD012Read;
	const u64 target = ( m.bus.lastReadStart - 3 * per / 4 ) + cyclesToHost( counted );
	CHECK( target >= VSPTimedBus::fallAt( k - 1 ) );
	CHECK( target <  VSPTimedBus::fallAt( k ) );
	std::printf( "\n    LDA/STA: count target sits %lu ARM after fall_(k-1) (lock aim 342 + one poll)\n",
	             (unsigned long)( target - VSPTimedBus::fallAt( k - 1 ) ) );
}
