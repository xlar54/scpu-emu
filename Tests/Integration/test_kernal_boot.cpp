/*
   SCPU-EMU - end-to-end tests across CPU core, banking, shadow memory, write
   buffer and bus backend.

   The point of these is to check the seams: that a read of ROM costs nothing on
   the bus, that a read of $D012 does, that RAM writes reach the C64 only via
   the mirror path, and that the whole stack survives a synthetic KERNAL.
*/
#include "../test_framework.h"
#include "../../Source/C64/c64_memory.h"
#include "../../Source/CPU/M6502/m6502.h"
#include "../../Source/SuperCPU/write_buffer.h"
#include "../../Source/SuperCPU/registers.h"
#include "../../Source/SuperCPU/supercpu.h"
#include "../../Source/Bus/Host/host_bus.h"
#include "../../Source/Common/runtime_diagnostic.h"

TEST( integration_runtime_diagnostic_reports_post_handoff_deltas )
{
	SCPURuntimeSample start = {};
	start.handoff = true;
	start.hostCycles = 100;
	start.emuCycles = 200;
	start.ramWrites = 300;
	start.acceptedWrites = 350;
	start.skippedWrites = 375;
	start.postedWaitCycles = 380;
	start.slowCycles = 390;
	start.iecThrottledCycles = 395;
	start.transfers = 400;
	start.serialTransfers = 500;
	start.idlePrimes = 600;

	SCPURuntimeSample end = start;
	end.hostCycles += 1400000;
	end.emuCycles += 20000;
	end.ramWrites += 700;
	end.acceptedWrites += 710;
	end.skippedWrites += 720;
	end.postedWaitCycles += 730;
	end.slowCycles += 740;
	end.iecThrottledCycles += 750;
	end.transfers += 800;
	end.serialTransfers += 90;
	end.idlePrimes += 10;
	end.maxBandUS = 77;
	end.missedBandDeadlines = 3;

	const SCPURuntimeResult r = scpuRuntimeDelta( start, end, 1400000000 );
	CHECK( r.valid );
	CHECK_EQ( r.hostUS, 1000 );
	CHECK_EQ( r.emuCycles, 20000 );
	CHECK_EQ( r.achievedKHz, 20000 );
	CHECK_EQ( r.ramWrites, 700 );
	CHECK_EQ( r.acceptedWrites, 710 );
	CHECK_EQ( r.skippedWrites, 720 );
	CHECK_EQ( r.postedWaitCycles, 730 );
	CHECK_EQ( r.slowCycles, 740 );
	CHECK_EQ( r.iecThrottledCycles, 750 );
	CHECK_EQ( r.transfers, 800 );
	CHECK_EQ( r.serialTransfers, 90 );
	CHECK_EQ( r.idlePrimes, 10 );
	CHECK_EQ( r.maxBandUS, 77 );
	CHECK_EQ( r.missedBandDeadlines, 3 );
}

TEST( integration_runtime_diagnostic_requires_handoff_and_elapsed_time )
{
	SCPURuntimeSample start = {};
	SCPURuntimeSample end = {};
	CHECK( !scpuRuntimeDelta( start, end, 1400000000 ).valid );
	start.handoff = end.handoff = true;
	CHECK( !scpuRuntimeDelta( start, end, 1400000000 ).valid );
	end.hostCycles = 1;
	CHECK( !scpuRuntimeDelta( start, end, 0 ).valid );
}

struct SystemFixture
{
	CHostBus           bus;
	CC64Memory         mem;
	CWriteBuffer       wb;
	CSuperCPURegisters regs;
	CM6502             cpu;

	SystemFixture()
	{
		mem.attachBus( &bus );
		mem.setMirrorSink( &wb );
		mem.setIOInterceptor( &regs );
		wb.attach( &bus, mem.m_RAM );
		regs.attach( &wb );
		cpu.attachBus( &mem );
	}

	// Install a KERNAL image whose reset vector points at 'entry'.
	void installKernal( u16 entry, const u8 *code, u32 len, u16 org )
	{
		u8 kernal[ C64_KERNAL_SIZE ];
		std::memset( kernal, 0xEA, sizeof( kernal ) );	// NOP fill

		if ( org >= 0xE000 )
			std::memcpy( &kernal[ org - 0xE000 ], code, len );

		kernal[ 0x1FFC ] = (u8)( entry & 0xFF );
		kernal[ 0x1FFD ] = (u8)( entry >> 8 );
		mem.setKernalROM( kernal );
	}

	void start()
	{
		mem.reset();
		wb.resetStats();
		bus.resetStats();
		cpu.reset();
	}
};

struct CIA2SeedBus : CHostBus
{
	bool allowVerification = true;
	u32 verificationCalls = 0;

	bool verifyC64CIA2DDRA( u8 expected ) override
	{
		verificationCalls++;
		return allowVerification && m_Memory[ 0xDD02 ] == expected;
	}
};

struct TimedHostBus : CHostBus
{
	u64 now = 0;
	u32 rate = 1000000;

	u64 hostCycles() override { return now++; }
	u32 hostCyclesPerSec() override { return rate; }
};

// A scripted BA pin. The RAD backend waits the steal out and reports the
// cycles; here the test says how long the VIC held the bus and counts how
// often it was asked, which is the only way to prove the 1MHz gate from the
// host side -- "was the pin even sampled" is a different claim from "were
// cycles charged".
struct StallHostBus : TimedHostBus
{
	// THE HOST CLOCK'S RESOLUTION IS LOAD-BEARING HERE, not just its rate.
	// TimedHostBus::hostCycles() returns now++, so at the inherited 1:1 rate
	// every clock READ costs a whole emulated cycle -- and the pacer arms these
	// tests exist to tell apart are one cycle wide, so the fixture can return a
	// verdict on how often the code under test looks at the clock. MEASURED, on
	// this tree: adding one more hostCycles() read to baStallSettle, between the
	// spin's return and `m_BAStallCycles += stolen`, flips
	// bastall_hands_the_pacer_the_time_the_spin_already_spent to FAIL on
	// m_PacerCreditEntries with the arithmetic it checks completely unchanged.
	// That is how this was found: the release stamp added to baStallSettle in
	// this same change is exactly that one extra read, and without the line
	// below the suite goes 806626/1. The Pi runs 1368 ARM cycles per C64
	// cycle, so model that ratio: every claim below is unchanged and now has
	// 1368x the margin.
	//
	// REBUILD FROM SCRATCH BEFORE BELIEVING EITHER VERDICT HERE. A one-read
	// sensitivity is also a one-stale-object sensitivity: an incremental make
	// over a copied build tree reported this suite green with the old
	// c64_memory.o linked in, which is the opposite answer.
	enum { PER_C64 = 1368 };

	u32 steal = 0;
	u32 asked = 0;

	StallHostBus() { rate = 1000000u * PER_C64; }

	u32 busStealCycles( u32 maxCycles ) override
	{
		asked++;
		u32 s = steal;
		if ( s > maxCycles ) s = maxCycles;
		// A real stall also consumes the real time it reports.
		now += (u64)s * PER_C64;
		return s;
	}
};

TEST( bastall_charges_the_guest_at_1mhz_and_forces_a_fresh_interrupt_sample )
{
	StallHostBus bus;
	CC64Memory mem;
	mem.attachBus( &bus );
	mem.m_BAStallMode = 1;
	mem.m_BAStallMaxCycles = 80;
	mem.setPacing( bus.rate, SCPU_NORMAL_HZ );

	// Nothing held the bus: no charge, and the sample costs one pin read.
	const u64 before = mem.m_EmuCycles;
	mem.tickFast( 2 );
	CHECK_EQ( bus.asked, 1u );
	CHECK_EQ( mem.m_EmuCycles - before, (u64)2 );
	CHECK_EQ( mem.m_BAStallEvents, (u64)0 );

	// A badline-length steal is charged to the guest's clock in full, on top
	// of the instruction's own cycles. 43 is BA low at cycle 12 through the
	// last c-access at 54.
	bus.steal = 43;
	const u64 mid = mem.m_EmuCycles;
	const u64 c64Before = mem.emuNow();
	mem.m_IntCredit = 0;
	mem.tickFast( 2 );
	CHECK_EQ( mem.m_EmuCycles - mid, (u64)45 );
	CHECK_EQ( mem.m_BAStallEvents, (u64)1 );
	CHECK_EQ( mem.m_BAStallCycles, (u64)43 );
	// The C64-domain clock moves with it: the gate implies factor == 1, so the
	// freeze is visible one-for-one to everything that counts time.
	CHECK_EQ( mem.emuNow() - c64Before, (u64)45 );
	// And the first instruction after the release must re-sample the lines.
	CHECK( mem.m_IntCredit >= 0x100 );
}

// THE PACER MUST NOT BILL THE SPIN TWICE. busStealCycles() has already spent
// the real time, so the settle that follows must land on the on-time arm and
// not on the late/credit/clamp arm -- which is where the VSP hold is decided.
// Delete the m_PacingAnchor advance in baStallSettle to watch this fail.
TEST( bastall_hands_the_pacer_the_time_the_spin_already_spent )
{
	StallHostBus bus;
	CC64Memory mem;
	mem.attachBus( &bus );
	mem.m_BAStallMode = 2;
	mem.m_BAStallMaxCycles = 80;
	mem.setPacing( bus.rate, SCPU_NORMAL_HZ );
	bus.steal = 43;

	mem.tickFast( 2 );
	CHECK_EQ( mem.m_BAStallEvents, (u64)1 );
	const u64 creditBefore = mem.m_PacerCreditEntries;
	const u64 clampBefore  = mem.m_PacerClampEntries;
	bus.steal = 0;
	mem.tickFast( 2 );
	CHECK_EQ( mem.m_PacerCreditEntries, creditBefore );
	CHECK_EQ( mem.m_PacerClampEntries, clampBefore );
}

// THE INSTRUMENT, not the mechanism. K456 left a hold error of +3 to +4 cycles
// against hold = 59 - DECcycle, and UNIFORM across the sweep -- so the slope
// the invariant asks for is already right and what is left is one constant.
// The arithmetic cannot say where that constant lives: the error is
// (INC_true - 59) + the bias in a cycle-in-line that is reconstructed from the
// guest's last $D012 read. These three records are what separate them: the
// LENGTH distribution of the steals, the host instant of the RELEASE, and
// which class of steal that release came from.
TEST( bastall_records_the_steal_distribution_and_the_release_instant )
{
	StallHostBus bus;
	CC64Memory mem;
	mem.attachBus( &bus );
	mem.m_BAStallMode = 2;
	mem.m_BAStallMaxCycles = 80;
	mem.setPacing( bus.rate, SCPU_NORMAL_HZ );

	// A badline and a sprite steal are DIFFERENT populations; their mean is
	// 22.5 and describes neither, which is the error that has already wrecked
	// three conclusions in this project.
	bus.steal = 43;
	mem.tickFast( 2 );
	CHECK_EQ( (u32)mem.m_BAStallLastReleaseLen, 43u );
	bus.steal = 2;
	mem.tickFast( 2 );
	CHECK_EQ( mem.m_BAStallEvents, (u64)2 );
	CHECK_EQ( mem.m_BAStallHist[ 8 ], 1u );
	CHECK_EQ( mem.m_BAStallHist[ 0 ], 1u );
	// And the sprite steal has REPLACED the badline as the reference, which is
	// exactly the confound SCPU_VSP_RELEASE_BADLINE exists to make visible.
	CHECK_EQ( (u32)mem.m_BAStallLastReleaseLen, 2u );

	// The stamp has to be taken AFTER the spin consumed the time it reported,
	// or it is not the release: 45 cycles have been spent by now.
	CHECK( mem.m_BAStallLastReleaseHost >= (u64)45 * StallHostBus::PER_C64 );

	// The censored tail is its own bin, so capped= can be compared with it
	// instead of being averaged into everything else.
	bus.steal = 80;
	mem.tickFast( 2 );
	CHECK_EQ( mem.m_BAStallHist[ 15 ], 1u );
}

TEST( bastall_is_never_sampled_in_turbo )
{
	StallHostBus bus;
	CC64Memory mem;
	mem.attachBus( &bus );
	mem.m_BAStallMode = 2;
	bus.steal = 43;
	mem.setPacing( bus.rate, SCPU_TURBO_HZ );

	// Several settles' worth at 20MHz. The accelerator runs from its own SRAM
	// there and BA is irrelevant to it; the hot path must not even look.
	for ( u32 i = 0; i < 64; i++ )
		mem.tickFast( 8 );
	CHECK_EQ( bus.asked, 0u );
	CHECK_EQ( mem.m_BAStallEvents, (u64)0 );
	CHECK_EQ( mem.m_BAStallCycles, (u64)0 );
}

TEST( bastall_mode_0_is_the_k451_path_exactly )
{
	StallHostBus bus;
	CC64Memory mem;
	mem.attachBus( &bus );
	mem.m_BAStallMode = 0;
	bus.steal = 43;
	mem.setPacing( bus.rate, SCPU_NORMAL_HZ );

	for ( u32 i = 0; i < 16; i++ )
		mem.tickFast( 2 );
	CHECK_EQ( bus.asked, 0u );
	CHECK_EQ( mem.m_BAStallEvents, (u64)0 );
}

TEST( bastall_mode_1_declines_inside_a_serial_transaction )
{
	StallHostBus bus;
	CC64Memory mem;
	mem.attachBus( &bus );
	mem.m_BAStallMode = 1;
	bus.steal = 43;
	mem.setPacing( bus.rate, SCPU_NORMAL_HZ );

	// An IEC line change arms the transaction window.
	mem.write8( 0xDD00, 0x08 );
	CHECK( mem.iecBusActive() );
	const u32 askedBefore = bus.asked;
	mem.tickFast( 2 );
	CHECK_EQ( bus.asked, askedBefore );
	CHECK_EQ( mem.m_BAStallEvents, (u64)0 );
	CHECK( mem.m_BAStallSuppressedIEC > 0 );

	// Mode 2 is the faithful setting and stalls there too.
	mem.m_BAStallMode = 2;
	mem.tickFast( 2 );
	CHECK_EQ( mem.m_BAStallEvents, (u64)1 );
	CHECK_EQ( mem.m_BAStallCycles, (u64)43 );
}

TEST( bastall_charge_is_bounded_by_the_configured_ceiling )
{
	StallHostBus bus;
	CC64Memory mem;
	mem.attachBus( &bus );
	mem.m_BAStallMode = 2;
	mem.m_BAStallMaxCycles = 80;
	bus.steal = 4000;			// a BA that never came back
	mem.setPacing( bus.rate, SCPU_NORMAL_HZ );

	mem.tickFast( 2 );
	CHECK_EQ( mem.m_BAStallCycles, (u64)80 );
}

// The setter is where the bound is enforced for every caller, so test it there
// and not only through the config parser. 80 and not 128: anything above 80 is
// unreachable past RAD_BA_STALL_CEILING, so the setter must not promise it.
TEST( bastall_setter_clamps_mode_and_ceiling )
{
	CSuperCPU cpu;
	cpu.setBAStall( 9, 4000 );
	CHECK_EQ( (u32)cpu.memory().m_BAStallMode, 2u );
	CHECK_EQ( (u32)cpu.memory().m_BAStallMaxCycles, 80u );
	cpu.setBAStall( 0, 1 );
	CHECK_EQ( (u32)cpu.memory().m_BAStallMode, 0u );
	CHECK_EQ( (u32)cpu.memory().m_BAStallMaxCycles, 8u );
}

TEST( hostbus_badline_model_matches_the_published_rule )
{
	CHostBus bus;				// the MODEL, not the scripted pin above
	static u64 cycle;
	cycle = 0;
	bus.setRasterClock( []( void *ctx ) -> u64 { (void)ctx; return cycle; }, 0 );
	const u32 perLine = c64CyclesPerLine( bus.m_Signals.video );

	// DEN set, YSCROLL = 6. Line 54 is then a badline ( 54 & 7 == 6 ).
	bus.m_Memory[ 0xD011 ] = (u8)( 0x10 | 6 );

	// Bauer 3.6.3: BA falls at cycle 12 and the last stolen cycle is 54, so a
	// CPU frozen at cycle 14 resumes at 55 -- 41 cycles -- and Mario's INC,
	// five cycles later, completes at 59.
	cycle = (u64)54 * perLine + 13;			// cycle-in-line 14, 1-based
	CHECK_EQ( bus.busStealCycles( 80 ), 41u );
	cycle = (u64)54 * perLine + 29;			// cycle 30
	CHECK_EQ( bus.busStealCycles( 80 ), 25u );
	cycle = (u64)54 * perLine + 49;			// cycle 50
	CHECK_EQ( bus.busStealCycles( 80 ), 5u );
	cycle = (u64)54 * perLine + 52;			// cycle 53
	CHECK_EQ( bus.busStealCycles( 80 ), 2u );

	// Before BA falls and after the last c-access there is no steal at all.
	cycle = (u64)54 * perLine + 10;			// cycle 11
	CHECK_EQ( bus.busStealCycles( 80 ), 0u );
	cycle = (u64)54 * perLine + 54;			// cycle 55
	CHECK_EQ( bus.busStealCycles( 80 ), 0u );

	// The model obeys its own caller's bound.
	cycle = (u64)54 * perLine + 13;
	CHECK_EQ( bus.busStealCycles( 10 ), 10u );

	// Not a badline line: YSCROLL no longer matches.
	cycle = (u64)55 * perLine + 13;
	CHECK_EQ( bus.busStealCycles( 80 ), 0u );

	// Outside the display window, and with DEN clear, there are no badlines --
	// which is exactly why a loader that blanks the screen keeps full speed.
	cycle = (u64)0x2E * perLine + 13;
	CHECK_EQ( bus.busStealCycles( 80 ), 0u );
	bus.m_Memory[ 0xD011 ] = 6;
	cycle = (u64)54 * perLine + 13;
	CHECK_EQ( bus.busStealCycles( 80 ), 0u );

	// And a backend with no cycle source, which cannot place a cycle inside a
	// line, must answer 0 rather than invent a steal -- that is what keeps
	// every other IC64Bus unchanged.
	CHostBus noClock;
	noClock.m_Memory[ 0xD011 ] = (u8)( 0x10 | 6 );
	CHECK_EQ( noClock.busStealCycles( 80 ), 0u );
}

TEST( integration_reset_vector_comes_from_shadowed_kernal )
{
	SystemFixture f;
	const u8 code[] = { 0xEA };
	f.installKernal( 0xFCE2, code, sizeof( code ), 0xFCE2 );
	f.start();

	CHECK_EQ( f.cpu.m_PC, 0xFCE2 );
	// Fetching the vector out of shadow RAM must not touch the machine at all.
	CHECK_EQ( f.bus.m_Cycles, 0 );
}

TEST( integration_rom_execution_is_free_on_the_bus )
{
	SystemFixture f;
	// LDA #$41 / STA $0400 ... run entirely from ROM.
	const u8 code[] = { 0xA9, 0x41, 0x8D, 0x00, 0x04, 0x4C, 0x00, 0xE0 };
	f.installKernal( 0xE000, code, sizeof( code ), 0xE000 );
	f.start();
	f.wb.setOptMode( SCPU_OPT_NONE );

	f.cpu.step();	// LDA #
	f.cpu.step();	// STA $0400

	// Instruction fetches and the RAM write are all local. The write is queued
	// for mirroring but has not been flushed.
	CHECK_EQ( f.bus.m_Cycles, 0 );
	CHECK_EQ( f.mem.m_RAM[ 0x0400 ], 0x41 );
	CHECK_EQ( f.wb.pending(), 1 );

	f.wb.flush();
	CHECK_EQ( f.bus.m_Cycles, 1 );
	CHECK_EQ( f.bus.m_Memory[ 0x0400 ], 0x41 );
}

TEST( integration_io_read_goes_to_the_machine )
{
	SystemFixture f;
	// $D020 rather than $D012: the host bus synthesises a raster counter for
	// $D011/$D012 so a real KERNAL can boot, so those two are not plain storage.
	const u8 code[] = { 0xAD, 0x20, 0xD0 };	// LDA $D020
	f.installKernal( 0xE000, code, sizeof( code ), 0xE000 );
	f.start();

	f.bus.m_Memory[ 0xD020 ] = 0x37;

	f.cpu.step();
	CHECK_EQ( f.cpu.m_A, 0x37 );
	CHECK_EQ( f.bus.m_Reads, 1 );
	CHECK_EQ( f.mem.m_IOReads, 1 );
}

TEST( integration_ordinary_io_does_not_flush_the_mirror_buffer )
{
	// Flushing before EVERY I/O access meant a booting KERNAL -- which touches
	// I/O constantly -- drove a continuous stream of unscheduled bursts across
	// the visible display, corrupting the VIC-II's fetches. So ordinary I/O
	// does not flush; mirroring is scheduled against the raster in
	// CSuperCPU::runFrame() instead.
	//
	// $D020 is the border colour: it changes what the VIC-II draws, but not
	// which memory it reads, so nothing has to arrive first.
	SystemFixture f;
	const u8 code[] = { 0xA9, 0x41, 0x8D, 0x00, 0x04,	// LDA #$41 / STA $0400
	                    0xA9, 0x0E, 0x8D, 0x20, 0xD0 };	// LDA #$0E / STA $D020
	f.installKernal( 0xE000, code, sizeof( code ), 0xE000 );
	f.start();
	f.wb.setOptMode( SCPU_OPT_NONE );
	f.bus.m_LogEnabled = true;

	for ( int i = 0; i < 4; i++ ) f.cpu.step();

	// The screen byte is still queued; only the register write went out.
	CHECK_EQ( f.wb.pending(), 1 );
	CHECK_EQ( f.bus.m_LogCount, 1 );
	CHECK_EQ( f.bus.m_Log[ 0 ].addr, 0xD020 );
	CHECK_EQ( f.bus.m_Log[ 0 ].op, HOSTOP_WRITE );

	// It reaches the machine when the mirror is flushed, not before.
	f.wb.flush();
	CHECK_EQ( f.bus.m_Memory[ 0x0400 ], 0x41 );
}

TEST( integration_display_state_writes_never_flush_synchronously )
{
	// A synchronous flush emits a burst, and bursts are never display-safe at
	// arbitrary raster positions. Nor may the write wait for a later border:
	// the CMD splash deliberately toggles $D011 mode bits mid-display, while
	// sprite multiplexers need every $D015 transition. Preserve immediate I/O
	// semantics and leave mirror delivery to the frame scheduler.
	static const u16 displayRegs[] = { 0xD011, 0xD015, 0xD016, 0xD018, 0xDD00 };

	for ( u32 i = 0; i < sizeof displayRegs / sizeof displayRegs[ 0 ]; i++ )
	{
		const u16 reg = displayRegs[ i ];

		SystemFixture f;
		const u8 code[] = { 0xA9, 0x41, 0x8D, 0x00, 0x04,				// STA $0400
		                    0xA9, 0x3B, 0x8D, (u8)( reg & 0xFF ),
		                                      (u8)( reg >> 8 ) };		// STA reg
		f.installKernal( 0xE000, code, sizeof( code ), 0xE000 );
		f.start();
		f.wb.setOptMode( SCPU_OPT_NONE );
		f.bus.m_LogEnabled = true;

		for ( int i2 = 0; i2 < 4; i2++ ) f.cpu.step();

		// The staged byte stays queued: only the register write itself went to
		// the machine. The border-scheduled flusher delivers the data within a
		// frame, which is the same transient a real SuperCPU shows.
		CHECK_EQ( f.wb.pending(), 1 );

		bool sawRegWrite = false, sawScreenWrite = false;
		for ( u32 j = 0; j < f.bus.m_LogCount; j++ )
		{
			if ( f.bus.m_Log[ j ].addr == reg && f.bus.m_Log[ j ].op == HOSTOP_WRITE ) sawRegWrite = true;
			if ( f.bus.m_Log[ j ].addr == 0x0400 ) sawScreenWrite = true;
		}
		CHECK( sawRegWrite );
		CHECK( !sawScreenWrite );

		// And the border flush delivers it afterwards.
		f.wb.flush();
		CHECK_EQ( f.bus.m_Memory[ 0x0400 ], 0x41 );
	}
}

TEST( integration_banking_switch_changes_what_executes )
{
	SystemFixture f;
	const u8 code[] = { 0xAD, 0x00, 0xE0 };	// LDA $E000
	f.installKernal( 0xE000, code, sizeof( code ), 0xE000 );
	f.start();

	// With HIRAM set, $E000 reads the KERNAL (first byte of our LDA).
	f.cpu.step();
	CHECK_EQ( f.cpu.m_A, 0xAD );

	// Drop HIRAM and LORAM: $E000 must now read the RAM underneath.
	f.mem.m_RAM[ 0xE000 ] = 0x5A;
	f.mem.write8( 0x0001, 0x30 );
	f.cpu.m_PC = 0xE000;	// re-run the same instruction, now out of RAM

	CHECK_EQ( f.mem.read8( 0xE000 ), 0x5A );
}

TEST( integration_supercpu_registers_never_reach_the_bus )
{
	SystemFixture f;
	// Open the register bank, then select BASIC optimization.
	const u8 code[] = { 0x8D, 0x7E, 0xD0,	// STA $D07E - enable hw registers
	                    0x8D, 0x76, 0xD0,	// STA $D076 - BASIC optimization
	                    0x8D, 0x7A, 0xD0 };	// STA $D07A - normal speed
	f.installKernal( 0xE000, code, sizeof( code ), 0xE000 );
	f.start();

	CHECK( f.regs.turboEnabled() );

	for ( int i = 0; i < 3; i++ ) f.cpu.step();

	CHECK( f.regs.hardwareRegsEnabled() );
	CHECK_EQ( f.wb.optMode(), SCPU_OPT_BASIC );
	CHECK( !f.regs.turboEnabled() );

	// None of it should have generated a single C64 bus cycle.
	CHECK_EQ( f.bus.m_Cycles, 0 );
	CHECK_EQ( f.mem.m_IOWrites, 0 );
}

TEST( integration_optimization_registers_ignored_while_bank_closed )
{
	SystemFixture f;
	const u8 code[] = { 0x8D, 0x76, 0xD0 };	// STA $D076 without enabling first
	f.installKernal( 0xE000, code, sizeof( code ), 0xE000 );
	f.start();

	SCPUOptMode before = f.wb.optMode();
	f.cpu.step();
	CHECK_EQ( f.wb.optMode(), before );
}

TEST( integration_supercpu_detection_idiom )
{
	SystemFixture f;
	// $D0BC = decimal 53436, the documented detection register. (An earlier
	// source quoted PEEK(53433) = $D0B9; the SuperCPU 128 register list and the
	// compatibility notes both say $D0BC, so 53433 looks like a transposition.)
	const u8 code[] = { 0xAD, 0xBC, 0xD0 };	// LDA $D0BC
	f.installKernal( 0xE000, code, sizeof( code ), 0xE000 );
	f.start();

	f.cpu.step();
	// Software checks (PEEK(53436) AND 128) = 0 to detect an active accelerator.
	CHECK_EQ( f.cpu.m_A & 0x80, 0x00 );
}

TEST( integration_snapshot_roms_from_the_machine )
{
	SystemFixture f;

	// Pretend the live C64 has a KERNAL whose reset vector is the usual $FCE2.
	for ( u32 i = 0; i < 0x2000; i++ )
		f.bus.m_Memory[ 0xE000 + i ] = (u8)( i & 0xFF );
	f.bus.m_Memory[ 0xFFFC ] = 0xE2;
	f.bus.m_Memory[ 0xFFFD ] = 0xFC;
	for ( u32 i = 0; i < 0x2000; i++ )
		f.bus.m_Memory[ 0xA000 + i ] = (u8)( ~i & 0xFF );

	CHECK( f.mem.snapshotROMsFromBus() );
	CHECK( f.mem.hasKernalROM() );
	CHECK( f.mem.hasBasicROM() );

	f.mem.reset();
	f.cpu.reset();
	CHECK_EQ( f.cpu.m_PC, 0xFCE2 );

	// 8K of BASIC plus 8K of KERNAL, plus the two-byte reset-vector probe that
	// confirms the machine had its ROMs banked in.
	CHECK_EQ( f.bus.m_Reads, 0x4000 + 2 );
}

TEST( integration_snapshot_fills_in_only_the_missing_rom )
{
	// A caller may supply one image and want the other taken off the machine.
	// Overwriting the supplied one would silently discard it.
	SystemFixture f;

	u8 myKernal[ C64_KERNAL_SIZE ];
	std::memset( myKernal, 0x5A, sizeof( myKernal ) );
	myKernal[ 0x1FFC ] = 0x00;
	myKernal[ 0x1FFD ] = 0xE5;
	f.mem.setKernalROM( myKernal );

	// The machine holds a different KERNAL and a BASIC we do want.
	for ( u32 i = 0; i < 0x2000; i++ ) f.bus.m_Memory[ 0xE000 + i ] = 0xCC;
	f.bus.m_Memory[ 0xFFFC ] = 0xE2;
	f.bus.m_Memory[ 0xFFFD ] = 0xFC;
	for ( u32 i = 0; i < 0x2000; i++ ) f.bus.m_Memory[ 0xA000 + i ] = 0x77;

	CHECK( f.mem.snapshotROMsFromBus() );

	// Supplied KERNAL survived; BASIC came from the machine.
	CHECK_EQ( f.mem.m_Kernal[ 0x100 ], 0x5A );
	CHECK_EQ( f.mem.m_Basic[ 0x100 ], 0x77 );

	// Only BASIC was fetched: 8K plus the vector probe, not 16K.
	CHECK_EQ( f.bus.m_Reads, 0x2000 + 2 );

	f.mem.reset();
	f.cpu.reset();
	CHECK_EQ( f.cpu.m_PC, 0xE500 );		// our KERNAL's vector, not the machine's
}

TEST( integration_snapshot_is_a_no_op_when_both_roms_supplied )
{
	SystemFixture f;

	u8 rom[ C64_KERNAL_SIZE ];
	std::memset( rom, 0x11, sizeof( rom ) );
	f.mem.setKernalROM( rom );
	f.mem.setBasicROM( rom );

	CHECK( f.mem.snapshotROMsFromBus() );
	CHECK_EQ( f.bus.m_Reads, 0 );		// not even the vector probe
}

TEST( integration_snapshot_rejects_a_machine_that_was_not_banked_in )
{
	SystemFixture f;
	// All zeroes at $FFFC/$FFFD: the reset vector does not point into the
	// KERNAL, so what we read was DRAM, not ROM.
	CHECK( !f.mem.snapshotROMsFromBus() );
}

TEST( integration_synthetic_kernal_fills_the_screen )
{
	SystemFixture f;

	// LDX #$00
	// loop: TXA / STA $0400,X / INX / BNE loop / JMP self
	const u8 code[] = {
		0xA2, 0x00,				// LDX #$00
		0x8A,					// TXA
		0x9D, 0x00, 0x04,		// STA $0400,X
		0xE8,					// INX
		0xD0, 0xF9,				// BNE loop
		0x4C, 0x09, 0xE0		// JMP *
	};
	f.installKernal( 0xE000, code, sizeof( code ), 0xE000 );
	f.start();
	f.wb.setOptMode( SCPU_OPT_NONE );

	f.cpu.run( 5000 );
	f.wb.flush();

	for ( u32 i = 0; i < 256; i++ )
	{
		CHECK_EQ( f.mem.m_RAM[ 0x0400 + i ], (u8)i );
		CHECK_EQ( f.bus.m_Memory[ 0x0400 + i ], (u8)i );
	}

	// 256 distinct addresses written once each, coalesced into one burst.
	CHECK_EQ( f.bus.m_BurstWrites, 256 );
}

TEST( integration_detection_answers_on_both_documented_addresses )
{
	// $D0BC is the detection register. The rest of the block is decoded too, so
	// a program probing a neighbouring address still sees bit 7 clear rather
	// than the open-bus 1 a stock machine returns.
	SystemFixture f;
	const u8 code[] = { 0xEA };
	f.installKernal( 0xE000, code, sizeof( code ), 0xE000 );
	f.start();

	u8 v;
	CHECK( f.regs.ioRead( 0xD0B9, v ) );  CHECK_EQ( v & 0x80, 0x00 );	// 53433
	CHECK( f.regs.ioRead( 0xD0BC, v ) );  CHECK_EQ( v & 0x80, 0x00 );	// 53436
	CHECK( f.regs.ioRead( 0xD0BF, v ) );  CHECK_EQ( v & 0x80, 0x00 );

	// $D0B0 is the version/mode register, not the presence flag.
	CHECK( f.regs.ioRead( 0xD0B0, v ) );
	// VICE ORs reset's low optimization flags ($07) into every status read.
	CHECK_EQ( v & 0xC0, SCPU_VERSION_V2 );
	CHECK_EQ( v & 0x07, 0x07 );

	// None of it reaches the machine.
	CHECK_EQ( f.bus.m_Cycles, 0 );
}

TEST( integration_speed_switch_gates_turbo_rather_than_forcing_it )
{
	// The physical switch is "force 1MHz, or merely ALLOW 20MHz" -- it never
	// accelerates the machine by itself.
	//
	// VICE adds a wrinkle worth knowing: the switch is honoured only while the
	// hardware registers are DISABLED. Software that opens the register bank
	// takes the switch out of circuit entirely.
	SystemFixture f;
	const u8 code[] = { 0xEA };
	f.installKernal( 0xE000, code, sizeof( code ), 0xE000 );

	f.regs.setSpeedSwitchAllowsTurbo( false );		// switch selects 1MHz
	f.start();
	f.regs.setSpeedSwitchAllowsTurbo( false );

	CHECK( !f.regs.fastMode() );					// held at 1MHz by the switch

	// $D0B5 bit 6 reports the switch position.
	u8 v;
	CHECK( f.regs.ioRead( 0xD0B5, v ) );
	CHECK_EQ( v & SCPU_SWITCH_1MHZ, SCPU_SWITCH_1MHZ );

	// Opening the register bank takes the switch out of circuit.
	f.regs.ioWrite( SCPU_REG_HWREGS_ENABLE, 0 );
	CHECK( f.regs.fastMode() );

	// ...and closing it puts the switch back in charge.
	f.regs.ioWrite( SCPU_REG_HWREGS_DISABLE, 0 );
	CHECK( !f.regs.fastMode() );

	// Flipping the switch to turbo permits speed, but software still decides.
	f.regs.setSpeedSwitchAllowsTurbo( true );
	CHECK( f.regs.fastMode() );
	CHECK( f.regs.ioRead( 0xD0B5, v ) );
	CHECK_EQ( v & SCPU_SWITCH_1MHZ, 0x00 );
}

TEST( integration_moving_switch_to_normal_drops_speed_immediately )
{
	SystemFixture f;
	const u8 code[] = { 0xEA };
	f.installKernal( 0xE000, code, sizeof( code ), 0xE000 );
	f.start();

	CHECK( f.regs.turboEnabled() );
	f.regs.setSpeedSwitchAllowsTurbo( false );
	CHECK( !f.regs.turboEnabled() );
}

TEST( integration_status_block_reports_distinct_flags )
{
	// $D0B0-$D0BF is not a mirrored block: each address carries its own flags.
	// Layout verified against VICE's SCPU64 emulation.
	SystemFixture f;
	const u8 code[] = { 0xEA };
	f.installKernal( 0xE000, code, sizeof( code ), 0xE000 );
	f.start();

	u8 v;

	// $D0B0: hardware version.
	CHECK( f.regs.ioRead( 0xD0B0, v ) );
	CHECK_EQ( v & 0xC0, SCPU_VERSION_V2 );

	// $D0B2: bit7 register bank open, bit6 system 1MHz. Both clear after reset.
	CHECK( f.regs.ioRead( 0xD0B2, v ) );
	CHECK_EQ( v & SCPU_STATUS_HWREGS,   0x00 );
	CHECK_EQ( v & SCPU_STATUS_SYS_1MHZ, 0x00 );

	f.regs.ioWrite( SCPU_REG_HWREGS_ENABLE, 0 );
	CHECK( f.regs.ioRead( 0xD0B2, v ) );
	CHECK_EQ( v & SCPU_STATUS_HWREGS, SCPU_STATUS_HWREGS );

	// System 1MHz is a separate request from software 1MHz.
	f.regs.ioWrite( SCPU_REG_SYS_1MHZ_ON, 0 );
	CHECK( f.regs.ioRead( 0xD0B2, v ) );
	CHECK_EQ( v & SCPU_STATUS_SYS_1MHZ, SCPU_STATUS_SYS_1MHZ );
	CHECK( !f.regs.fastMode() );
	f.regs.ioWrite( SCPU_REG_SYS_1MHZ_OFF, 0 );
	CHECK( f.regs.fastMode() );

	// $D0B3: the complete optimisation register. Low flags are significant;
	// $80 maps to the BASIC-only $0400-$07FF range.
	f.regs.ioWrite( SCPU_REG_OPTIM_V2, 0x80 );
	CHECK( f.regs.ioRead( 0xD0B4, v ) );
	CHECK_EQ( v & 0xC0, SCPU_OPTIM_BASIC );
	CHECK_EQ( f.wb.optMode(), SCPU_OPT_BASIC );

	f.regs.ioWrite( SCPU_REG_OPT_VICBANK2, 0 );		// the GEOS setting
	CHECK( f.regs.ioRead( 0xD0B4, v ) );
	CHECK_EQ( v & 0xC0, SCPU_OPTIM_VICBANK2 );

	f.regs.ioWrite( SCPU_REG_OPT_NONE, 0 );
	CHECK( f.regs.ioRead( 0xD0B4, v ) );
	CHECK_EQ( v & 0xC0, SCPU_OPTIM_NONE );

	// $D0B5: JiffyDOS switch in bit 7.
	f.regs.setJiffyDOSSwitch( true );
	CHECK( f.regs.ioRead( 0xD0B5, v ) );
	CHECK_EQ( v & SCPU_SWITCH_JIFFYDOS, SCPU_SWITCH_JIFFYDOS );

	// $D0B6: the 6502 core is always in emulation mode.
	CHECK( f.regs.ioRead( 0xD0B6, v ) );
	CHECK_EQ( v & SCPU_PROC_EMULATION, SCPU_PROC_EMULATION );

	// $D0B8 and its mirror $D0B9 (decimal 53433) report the software request in
	// bit 7 and the combined "anything asking for 1MHz" in bit 6.
	f.regs.ioWrite( SCPU_REG_SOFT_1MHZ_ON, 0 );
	CHECK( f.regs.ioRead( 0xD0B8, v ) );
	CHECK_EQ( v & SCPU_SPEED_SOFT_1MHZ, SCPU_SPEED_SOFT_1MHZ );
	CHECK_EQ( v & SCPU_SPEED_ANY_1MHZ,  SCPU_SPEED_ANY_1MHZ );

	u8 mirrored;
	CHECK( f.regs.ioRead( 0xD0B9, mirrored ) );
	CHECK_EQ( mirrored, v );						// $D0B9 mirrors $D0B8

	f.regs.ioWrite( SCPU_REG_SOFT_1MHZ_OFF, 0 );
	CHECK( f.regs.ioRead( 0xD0B8, v ) );
	CHECK_EQ( v & SCPU_SPEED_SOFT_1MHZ, 0x00 );

	// $D0B3 is writable on v2 while the bank is open.
	CHECK( f.regs.ioWrite( 0xD0B3, 0x40 ) );
	CHECK( f.regs.ioRead( 0xD0B3, v ) );
	CHECK_EQ( v & 0xC0, 0x40 );
}

TEST( integration_d0b5_defaults_on_and_bit7_is_a_writable_virtual_jiffy_switch )
{
	SystemFixture f;
	f.start();
	f.regs.setSpeedSwitchAllowsTurbo( false );

	u8 v = 0;
	CHECK( f.regs.ioRead( SCPU_REG_SWITCHES, v ) );
	CHECK_EQ( v & SCPU_SWITCH_JIFFYDOS, SCPU_SWITCH_JIFFYDOS );
	CHECK_EQ( v & SCPU_SWITCH_1MHZ, SCPU_SWITCH_1MHZ );

	// This emulator-only switch is deliberately usable as a direct BASIC POKE.
	// $D07E must not be required: opening that bank also swaps KERNAL windows
	// immediately and can strand BASIC or an IRQ in mismatched KERNAL code.
	CHECK( !f.regs.hardwareRegsEnabled() );
	f.mem.write8( SCPU_REG_SWITCHES, 0 );
	CHECK( !f.regs.hardwareRegsEnabled() );
	CHECK( f.regs.ioRead( SCPU_REG_SWITCHES, v ) );
	CHECK_EQ( v & SCPU_SWITCH_JIFFYDOS, 0x00 );
	CHECK_EQ( v & SCPU_SWITCH_1MHZ, SCPU_SWITCH_1MHZ );
	CHECK_EQ( f.bus.m_Cycles, 0 );

	// Reset changes register state, not a physical/virtual switch position.
	f.regs.reset();
	CHECK( f.regs.ioRead( SCPU_REG_SWITCHES, v ) );
	CHECK_EQ( v & SCPU_SWITCH_JIFFYDOS, 0x00 );

	f.mem.write8( SCPU_REG_SWITCHES, SCPU_SWITCH_JIFFYDOS );
	CHECK( f.regs.ioRead( SCPU_REG_SWITCHES, v ) );
	CHECK_EQ( v & SCPU_SWITCH_JIFFYDOS, SCPU_SWITCH_JIFFYDOS );
	CHECK_EQ( v & SCPU_SWITCH_1MHZ, SCPU_SWITCH_1MHZ );
	CHECK( !f.regs.hardwareRegsEnabled() );
}

TEST( integration_runtime_video_control_requires_one_shot_knock )
{
	SystemFixture f;
	bool hdmiActive = true;
	f.regs.configureVideoSwitch( true, true );
	f.regs.trackVideoHDMIActive( &hdmiActive );
	f.start();

	// A bare access retains the decoded/no-effect CMD behavior.
	CHECK( f.regs.videoHDMIRequested() );
	CHECK( f.regs.ioWrite( SCPU_REG_EMU_VIDEO, SCPU_EMU_VIDEO_VIC ) );
	CHECK( f.regs.videoHDMIRequested() );

	// Only the access following $A5 is interpreted as an emulator command.
	CHECK( f.regs.ioWrite( SCPU_REG_EMU_VIDEO, SCPU_EMU_VIDEO_KNOCK ) );
	CHECK( f.regs.ioWrite( SCPU_REG_EMU_VIDEO, SCPU_EMU_VIDEO_VIC ) );
	CHECK( !f.regs.videoHDMIRequested() );

	// Query reports the completed mode, not the still-pending request.
	u8 value = 0xFF;
	CHECK( f.regs.ioWrite( SCPU_REG_EMU_VIDEO, SCPU_EMU_VIDEO_KNOCK ) );
	CHECK( f.regs.ioRead( SCPU_REG_EMU_VIDEO, value ) );
	CHECK_EQ( value, SCPU_EMU_VIDEO_HDMI );
	hdmiActive = false;
	CHECK( f.regs.ioWrite( SCPU_REG_EMU_VIDEO, SCPU_EMU_VIDEO_KNOCK ) );
	CHECK( f.regs.ioRead( SCPU_REG_EMU_VIDEO, value ) );
	CHECK_EQ( value, SCPU_EMU_VIDEO_VIC );

	// Invalid values consume the latch without changing the request. Reset also
	// closes an unfinished latch and restores the configured boot preference.
	CHECK( f.regs.ioWrite( SCPU_REG_EMU_VIDEO, SCPU_EMU_VIDEO_KNOCK ) );
	CHECK( f.regs.ioWrite( SCPU_REG_EMU_VIDEO, 2 ) );
	CHECK( !f.regs.videoHDMIRequested() );
	f.regs.reset();
	CHECK( f.regs.videoHDMIRequested() );
	CHECK( f.regs.ioWrite( SCPU_REG_EMU_VIDEO, SCPU_EMU_VIDEO_VIC ) );
	CHECK( f.regs.videoHDMIRequested() );
}

TEST( integration_runtime_video_control_is_unavailable_without_hdmi_renderer )
{
	CSuperCPURegisters regs;
	regs.configureVideoSwitch( false, false );
	CHECK( regs.ioWrite( SCPU_REG_EMU_VIDEO, SCPU_EMU_VIDEO_KNOCK ) );
	CHECK( regs.ioWrite( SCPU_REG_EMU_VIDEO, SCPU_EMU_VIDEO_HDMI ) );
	CHECK( !regs.videoHDMIRequested() );
}

TEST( integration_hdmi_collision_reads_use_and_clear_rendered_latches )
{
	SystemFixture f;
	f.start();
	volatile u32 spriteSprite = 0x85;
	volatile u32 spriteBackground = 0x42;
	bool hdmiActive = true;
	f.mem.attachHDMICollisionLatches( &spriteSprite, &spriteBackground,
	                                  &hdmiActive );
	f.bus.m_Memory[ 0xD01E ] = 0xAA;
	f.bus.m_Memory[ 0xD01F ] = 0xBB;
	const u64 physicalReads = f.bus.m_Reads;

	CHECK_EQ( f.mem.read8( 0xD01E ), 0x85 );
	CHECK_EQ( spriteSprite, 0u );
	CHECK_EQ( f.mem.read8( 0xD01E ), 0x00 );
	CHECK_EQ( f.mem.read8( 0xD01F ), 0x42 );
	CHECK_EQ( spriteBackground, 0u );
	CHECK_EQ( f.bus.m_Reads, physicalReads );

	// VIC registers mirror every $40 bytes through $D3FF. Their aliases have
	// the same read-to-clear behaviour and must not fall through to stale DRAM.
	spriteSprite = 0x24;
	spriteBackground = 0x18;
	CHECK_EQ( f.mem.read8( 0xD05E ), 0x24 );
	CHECK_EQ( f.mem.read8( 0xD09F ), 0x18 );
	CHECK_EQ( spriteSprite, 0u );
	CHECK_EQ( spriteBackground, 0u );
	CHECK_EQ( f.bus.m_Reads, physicalReads );

	// VIDEO_MODE 0 and the physical side of a runtime switch retain the exact
	// established VIC-register path.
	hdmiActive = false;
	CHECK_EQ( f.mem.read8( 0xD01E ), 0xAA );
	CHECK_EQ( f.mem.read8( 0xD01F ), 0xBB );
	CHECK_EQ( f.bus.m_Reads, physicalReads + 2 );

	spriteSprite = spriteBackground = 0xFF;
	f.mem.reset();
	CHECK_EQ( spriteSprite, 0u );
	CHECK_EQ( spriteBackground, 0u );
}

TEST( integration_configured_jiffy_switch_survives_supercpu_init_and_reset )
{
	CHostBus bus;
	CSuperCPU scpu;
	u8 basic[ C64_BASIC_SIZE ];
	u8 kernal[ C64_KERNAL_SIZE ];
	std::memset( basic, 0xEA, sizeof basic );
	std::memset( kernal, 0xEA, sizeof kernal );
	kernal[ 0x1FFC ] = 0x00;
	kernal[ 0x1FFD ] = 0xE0;
	scpu.setBasicROM( basic );
	scpu.setKernalROM( kernal );

	// boot.cpp applies JIFFYDOS before init(); init() performs a reset, which
	// must preserve the configured virtual switch just like physical hardware.
	scpu.registers().setJiffyDOSSwitch( false );
	CHECK( scpu.init( &bus, SCPU_CORE_6502, 0 ) );

	u8 v = 0;
	CHECK( scpu.registers().ioRead( SCPU_REG_SWITCHES, v ) );
	CHECK_EQ( v & SCPU_SWITCH_JIFFYDOS, 0x00 );
	scpu.reset();
	CHECK( scpu.registers().ioRead( SCPU_REG_SWITCHES, v ) );
	CHECK_EQ( v & SCPU_SWITCH_JIFFYDOS, 0x00 );
}

TEST( integration_hdmi_exclusive_mode_discards_delivery_but_keeps_posted_timing )
{
	CHostBus bus;
	CSuperCPU scpu;
	u8 basic[ C64_BASIC_SIZE ];
	u8 kernal[ C64_KERNAL_SIZE ];
	std::memset( basic, 0xEA, sizeof basic );
	std::memset( kernal, 0xEA, sizeof kernal );
	kernal[ 0x1FFC ] = 0x00;
	kernal[ 0x1FFD ] = 0xE0;
	scpu.setBasicROM( basic );
	scpu.setKernalROM( kernal );
	CHECK( scpu.init( &bus, SCPU_CORE_6502, 0 ) );
	scpu.writeBuffer().setOptMode( SCPU_OPT_NONE );

	// Before HDMI takes ownership, RAM writes still enter the physical mirror.
	scpu.memory().write8( 0x0400, 0x41 );
	CHECK_EQ( scpu.writeBuffer().pending(), 1u );
	CHECK( !scpu.mirrorHalted() );

	// The handoff drops stale work and prevents later RAM writes from creating
	// any physical bus traffic. VIC/CIA access itself is deliberately untouched.
	scpu.disablePhysicalMirror();
	CHECK( scpu.mirrorHalted() );
	CHECK( !scpu.writeBuffer().deliveryEnabled() );
	CHECK_EQ( scpu.writeBuffer().pending(), 0u );
	const u64 physicalWrites = bus.m_Writes;
	scpu.memory().setPacing( 0, 20000000 );
	u64 cycleStart = scpu.memory().m_EmuCycles;
	scpu.memory().write8( 0x0401, 0x42 );
	CHECK( scpu.memory().fineTicksRequired() );
	scpu.memory().tickFast( 4 );
	CHECK_EQ( scpu.memory().m_EmuCycles - cycleStart, 4u );
	CHECK_EQ( scpu.writeBuffer().pending(), 0u );
	CHECK_EQ( bus.m_Writes, physicalWrites );

	// The first write posts for free; the next accepted write waits for that
	// slot's next 1MHz retirement boundary even though no physical mirror byte
	// is transmitted.
	cycleStart = scpu.memory().m_EmuCycles;
	scpu.memory().write8( 0x0402, 0x43 );
	scpu.memory().tickFast( 4 );
	CHECK_EQ( scpu.memory().m_EmuCycles - cycleStart, 16u );
	CHECK_EQ( scpu.writeBuffer().pending(), 0u );
	CHECK_EQ( bus.m_Writes, physicalWrites );

	// An emulated reset must not silently re-enable the native display path.
	scpu.reset();
	CHECK( scpu.mirrorHalted() );
	scpu.memory().write8( 0x0402, 0x43 );
	CHECK_EQ( scpu.writeBuffer().pending(), 0u );
}

TEST( integration_runtime_vic_handoff_requeues_visible_shadow )
{
	CHostBus bus;
	CSuperCPU scpu;
	u8 basic[ C64_BASIC_SIZE ] = {};
	u8 kernal[ C64_KERNAL_SIZE ] = {};
	kernal[ 0x1FFC ] = 0x00;
	kernal[ 0x1FFD ] = 0xE0;
	scpu.setBasicROM( basic );
	scpu.setKernalROM( kernal );
	CHECK( scpu.init( &bus, SCPU_CORE_6502, 0 ) );
	scpu.writeBuffer().setOptMode( SCPU_OPT_NONE );
	scpu.disablePhysicalMirror();

	// HDMI-only writes update authoritative shadow without queuing DRAM work.
	scpu.memory().write8( 0x0400, 0x41 );
	CHECK_EQ( scpu.writeBuffer().pending(), 0u );
	const u64 physicalWrites = bus.m_Writes;

	scpu.enablePhysicalMirror();
	CHECK( !scpu.mirrorHalted() );
	CHECK( scpu.writeBuffer().deliveryEnabled() );
	CHECK( scpu.writeBuffer().pending() >= 1024u );
	CHECK_EQ( bus.m_Writes, physicalWrites );
}

TEST( integration_hdmi_posted_timing_respects_full_optimization )
{
	CHostBus bus;
	CSuperCPU scpu;
	u8 basic[ C64_BASIC_SIZE ] = {};
	u8 kernal[ C64_KERNAL_SIZE ] = {};
	kernal[ 0x1FFC ] = 0x00;
	kernal[ 0x1FFD ] = 0xE0;
	scpu.setBasicROM( basic );
	scpu.setKernalROM( kernal );
	CHECK( scpu.init( &bus, SCPU_CORE_6502, 0 ) );
	scpu.writeBuffer().setOptMode( SCPU_OPT_FULL );
	scpu.disablePhysicalMirror();
	const u64 physicalWrites = bus.m_Writes;
	scpu.memory().setPacing( 0, 20000000 );
	scpu.memory().write8( 0x0400, 0x41 );
	CHECK( !scpu.memory().fineTicksRequired() );
	scpu.memory().tickFast( 4 );
	CHECK_EQ( scpu.memory().m_MirrorStretchCycles, 0u );
	CHECK_EQ( scpu.writeBuffer().pending(), 0u );
	CHECK_EQ( bus.m_Writes, physicalWrites );
}

TEST( integration_hdmi_bus_keepalive_uses_host_time_and_is_opt_in )
{
	TimedHostBus bus;
	CC64Memory mem;
	mem.attachBus( &bus );
	mem.setPacing( bus.rate, SCPU_NORMAL_HZ );

	// VIDEO_MODE 0 never enables this policy. Advancing far beyond the cadence
	// must therefore produce no physical transfer.
	bus.now = 1000;
	mem.tickFast( 1 );
	CHECK_EQ( bus.m_Writes, (u64)0 );

	mem.enableBusKeepalive( true );
	mem.tickFast( 1 );
	CHECK_EQ( bus.m_Writes, (u64)1 );
	CHECK_EQ( bus.m_Memory[ 0x02FE ], (u8)0xA6 );

	// The deadline is 90 host microseconds, independent of emulated cycles.
	// Leave a little room for hostCycles() itself: this fake clock advances on
	// every sample, while the real ARM counter advances continuously.
	bus.now += 80;
	mem.tickFast( 1 );
	CHECK_EQ( bus.m_Writes, (u64)1 );
	bus.now += 20;
	mem.tickFast( 1 );
	CHECK_EQ( bus.m_Writes, (u64)2 );
}

TEST( integration_hdmi_bus_keepalive_ignores_poll_hold_but_resets_its_deadline )
{
	TimedHostBus bus;
	CC64Memory mem;
	mem.attachBus( &bus );
	mem.setPacing( bus.rate, SCPU_TURBO_HZ );
	mem.enableBusKeepalive( true );

	// An idle $DD00 poll selects slow pacing but is not a real IEC transaction.
	bus.m_Memory[ 0xDD00 ] = 0xFF;
	mem.read8( 0xDD00 );
	CHECK( mem.iecThrottleActive() );
	CHECK( !mem.iecBusActive() );
	mem.tickFast( 1 );
	CHECK_EQ( bus.m_Writes, (u64)1 );
	CHECK_EQ( bus.m_Memory[ 0x02FE ], (u8)0xA6 );

	// Reset preserves the enabled policy but invalidates the old host deadline,
	// so the new emulated session is warmed on its first paced settlement.
	mem.reset();
	bus.now += 1;
	mem.tickFast( 1024 );
	CHECK_EQ( bus.m_Writes, (u64)2 );
}

TEST( integration_enhanced_optimisation_bits_select_the_vice_mirror_ranges )
{
	SystemFixture f;
	const u8 code[] = { 0xEA };
	f.installKernal( 0xE000, code, sizeof( code ), 0xE000 );
	f.start();

	// Hardware reset is $C7, which VICE maps to $0200-$FFFF.
	CHECK_EQ( f.regs.optimRegister(), 0xC7 );
	CHECK_EQ( f.wb.optMode(), SCPU_OPT_DEFAULT );
	CHECK( !f.wb.shouldMirror( 0x01FF ) );
	CHECK(  f.wb.shouldMirror( 0x0200 ) );

	f.regs.ioWrite( SCPU_REG_HWREGS_ENABLE, 0 );

	f.regs.ioWrite( SCPU_REG_OPTIM_V2, 0x84 );	// VICE mirror index 10: empty
	CHECK_EQ( f.wb.optMode(), SCPU_OPT_FULL );
	CHECK( !f.wb.shouldMirror( 0x0400 ) );

	f.regs.ioWrite( SCPU_REG_OPTIM_V2, 0xC1 );	// index 13: $0200-$FFFF
	CHECK_EQ( f.wb.optMode(), SCPU_OPT_DEFAULT );
	CHECK( !f.wb.shouldMirror( 0x01FF ) );
	CHECK(  f.wb.shouldMirror( 0x0200 ) );

	f.regs.ioWrite( SCPU_REG_OPTIM_V2, 0x04 );	// index 2: $0000-$3FFF
	CHECK_EQ( f.wb.optMode(), SCPU_OPT_VICBANK0 );
	CHECK(  f.wb.shouldMirror( 0x0000 ) );
	CHECK(  f.wb.shouldMirror( 0x3FFF ) );
	CHECK( !f.wb.shouldMirror( 0x4000 ) );

	f.regs.ioWrite( SCPU_REG_OPTIM_V2, 0x44 );	// index 6: $C000-$FFFF
	CHECK_EQ( f.wb.optMode(), SCPU_OPT_VICBANK3 );
	CHECK( !f.wb.shouldMirror( 0xBFFF ) );
	CHECK(  f.wb.shouldMirror( 0xC000 ) );
}

TEST( integration_low_optim_bits_appear_in_every_status_read )
{
	// VICE ORs the low three bits of the optimization register into every read
	// of the $D0Bx block, whichever register was addressed.
	SystemFixture f;
	const u8 code[] = { 0xEA };
	f.installKernal( 0xE000, code, sizeof( code ), 0xE000 );
	f.start();

	f.regs.ioWrite( SCPU_REG_HWREGS_ENABLE, 0 );
	f.regs.ioWrite( 0xD0B3, 0x05 );			// sets low bits 0 and 2

	u8 a, b, c;
	CHECK( f.regs.ioRead( 0xD0B0, a ) );
	CHECK( f.regs.ioRead( 0xD0B5, b ) );
	CHECK( f.regs.ioRead( 0xD0BC, c ) );

	CHECK_EQ( a & 0x07, 0x05 );
	CHECK_EQ( b & 0x07, 0x05 );
	CHECK_EQ( c & 0x07, 0x05 );
}

TEST( integration_hardware_registers_gate_the_optimisation_selects )
{
	SystemFixture f;
	const u8 code[] = { 0xEA };
	f.installKernal( 0xE000, code, sizeof( code ), 0xE000 );
	f.start();

	// Bank closed: the optimisation select is decoded but does nothing.
	SCPUOptMode before = f.wb.optMode();
	f.regs.ioWrite( SCPU_REG_OPT_BASIC, 0 );
	CHECK_EQ( f.wb.optMode(), before );

	// Bank open: it takes effect.
	f.regs.ioWrite( SCPU_REG_HWREGS_ENABLE, 0 );
	f.regs.ioWrite( SCPU_REG_OPT_BASIC, 0 );
	CHECK_EQ( f.wb.optMode(), SCPU_OPT_BASIC );
	CHECK_EQ( f.regs.optimRegister() & 0x07, 0x00 );

	// Speed selection is NOT gated -- that is what makes POKE 53370,0 work
	// straight from BASIC without opening the bank first.
	f.regs.ioWrite( SCPU_REG_HWREGS_DISABLE, 0 );
	f.regs.ioWrite( SCPU_REG_SOFT_1MHZ_ON, 0 );
	CHECK( !f.regs.fastMode() );
	f.regs.ioWrite( SCPU_REG_SOFT_1MHZ_OFF, 0 );
	CHECK( f.regs.fastMode() );
}

TEST( integration_register_attach_applies_the_power_on_mirror_policy )
{
	CWriteBuffer wb;
	wb.setOptMode( SCPU_OPT_FULL );
	CSuperCPURegisters regs;
	regs.attach( &wb );

	CHECK_EQ( regs.optimRegister(), 0xC7 );
	CHECK_EQ( wb.optMode(), SCPU_OPT_DEFAULT );
	CHECK( !wb.shouldMirror( 0x0100 ) );
	CHECK( wb.shouldMirror( 0x0200 ) );
}

TEST( integration_cia2_port_a_write_arms_the_iec_throttle )
{
	// The KERNAL times its serial-bus bits by counting cycles, and those counts
	// assume a 1MHz CPU. At 20MHz every pulse is twenty times too short and no
	// drive answers -- which presents as DEVICE NOT PRESENT. A real SuperCPU
	// has the same problem and CMD document that it always drops to 1MHz for
	// disk access; this reproduces that.
	SystemFixture f;
	const u8 code[] = { 0xA9, 0x07, 0x8D, 0x00, 0xDD,
	                    0xA9, 0x17, 0x8D, 0x00, 0xDD };	// then change IEC CLK
	f.installKernal( 0xE000, code, sizeof( code ), 0xE000 );
	f.start();

	CHECK( !f.mem.iecThrottleActive() );

	f.cpu.step();		// LDA
	f.cpu.step();		// first STA: unknown pin state is conservatively active
	CHECK( f.mem.iecThrottleActive() );
	f.cpu.step();
	f.cpu.step();		// IEC bit change refreshes the fallback

	CHECK( f.mem.iecThrottleActive() );
	CHECK_EQ( f.mem.m_IECThrottleEvents, 1 );

	// The write still reaches the real chip; throttling is a side effect, not a
	// substitution.
	CHECK_EQ( f.bus.m_Memory[ 0xDD00 ], 0x17 );
}

TEST( integration_iec_throttle_lapses_after_the_bus_goes_quiet )
{
	SystemFixture f;
	const u8 code[] = { 0xA9, 0x07, 0x8D, 0x00, 0xDD,
	                    0xA9, 0x17, 0x8D, 0x00, 0xDD, 0xEA };
	f.installKernal( 0xE000, code, sizeof( code ), 0xE000 );
	f.start();

	f.cpu.step();
	f.cpu.step();
	f.cpu.step();
	f.cpu.step();
	CHECK( f.mem.iecThrottleActive() );

	// It is a countdown in emulated cycles, so quiet time lets it expire and
	// the machine returns to full speed rather than being stuck slow.
	f.cpu.run( 200000 );
	CHECK( !f.mem.iecThrottleActive() );
}

TEST( integration_iec_throttle_can_be_disabled )
{
	SystemFixture f;
	const u8 code[] = { 0xA9, 0x07, 0x8D, 0x00, 0xDD,
	                    0xA9, 0x17, 0x8D, 0x00, 0xDD };
	f.installKernal( 0xE000, code, sizeof( code ), 0xE000 );
	f.start();
	f.mem.setIECThrottle( false );

	f.cpu.step();
	f.cpu.step();
	f.cpu.step();
	f.cpu.step();
	CHECK( !f.mem.iecThrottleActive() );
}

TEST( integration_vic_bank_changes_do_not_arm_iec_throttle )
{
	SystemFixture f;
	f.start();

	// Establish both hidden CIA baselines, then let the conservative first-write
	// window expire. With PA3-PA5 configured as outputs, changing only PA0/PA1
	// (the VIC bank) must not look like serial traffic.
	f.mem.write8( 0xDD02, 0x3F );
	f.mem.write8( 0xDD00, 0x37 );
	for ( u32 i = 0; i < 110000; i++ ) f.mem.tickFast( 1 );
	CHECK( !f.mem.iecThrottleActive() );
	const u64 events = f.mem.m_IECThrottleEvents;

	f.mem.write8( 0xDD00, 0x36 );

	CHECK( !f.mem.iecThrottleActive() );
	CHECK_EQ( f.mem.m_IECThrottleEvents, events );
}

TEST( integration_cia2_receive_inputs_are_active_low )
{
	SystemFixture f;
	f.start();

	// PA6/PA7 are inputs. Both high is the idle IEC bus and must not start a
	// transaction merely because software sampled it.
	f.bus.m_Memory[ 0xDD02 ] = 0x00;
	CHECK_EQ( f.mem.read8( 0xDD02 ), 0x00 );
	f.bus.m_Memory[ 0xDD00 ] = 0xFF;
	CHECK_EQ( f.mem.read8( 0xDD00 ), 0xFF );
	CHECK( !f.mem.iecBusActive() );

	// DATA-IN low means the drive asserted the line. This is the state seen in
	// the hardware freeze as $DD00=$3F (both receive inputs low).
	f.bus.m_Memory[ 0xDD00 ] = 0x7F;
	CHECK_EQ( f.mem.read8( 0xDD00 ), 0x7F );
	CHECK( f.mem.iecBusActive() );
}

TEST( integration_cia2_held_receive_line_refreshes_activity )
{
	SystemFixture f;
	f.start();
	f.bus.m_Memory[ 0xDD02 ] = 0x00;
	f.mem.read8( 0xDD02 );
	f.bus.m_Memory[ 0xDD00 ] = 0x7F;
	f.mem.read8( 0xDD00 );

	// Let the original 2ms activity window expire almost completely, then a
	// repeated poll of the still-low line must extend the live transaction.
	for ( u32 i = 0; i < 1500; i++ ) f.mem.tickFast( 1 );
	f.mem.read8( 0xDD00 );
	for ( u32 i = 0; i < 1500; i++ ) f.mem.tickFast( 1 );
	CHECK( f.mem.iecBusActive() );
}

// The 98% over-arm, reproduced and then refused. CIA2 PA3-PA5 drive inverting
// open collectors onto a wired-AND bus, so a latch bit the guest leaves set
// pulls its line low and the direct input reads that same low back. Measured
// on hardware at K456: during Mario gameplay with no drive access at all, the
// old predicate kept the 100ms hold armed 98% of the time.
TEST( integration_a_line_the_guest_pulls_down_is_not_a_drive_answering )
{
	SystemFixture f;
	f.start();

	// PA0-PA5 outputs with CLK OUT (bit 4) set: the guest is pulling CLK low
	// itself. $17 is the value a real capture of this state showed.
	f.mem.write8( 0xDD02, 0x3F );
	f.mem.write8( 0xDD00, 0x17 );
	// CLK IN low -- our own pull-down looped back -- DATA IN released high.
	f.bus.m_Memory[ 0xDD00 ] = 0x97;
	CHECK_EQ( f.mem.read8( 0xDD00 ), 0x97 );
	for ( u32 i = 0; i < 110000; i++ ) f.mem.tickFast( 1 );
	CHECK( !f.mem.iecThrottleActive() );
	CHECK( !f.mem.iecBusActive() );

	// One $DD00 read per frame, which a VIC-bank switch alone would do, for
	// ten frames. Nothing may arm the transaction window or the 100ms hold.
	// Baselines taken AFTER the deliberately conservative first-write arm has
	// expired: a cold machine with no CIA2 state yet still arms once, by design.
	const u64 events = f.mem.m_IECThrottleEvents;
	const u64 refused0 = f.mem.m_IECArmReadRefused;
	const u64 txn0 = f.mem.m_IECActivityArmedCycles;
	for ( u32 frame = 0; frame < 10; frame++ )
	{
		CHECK_EQ( f.mem.read8( 0xDD00 ), 0x97 );
		for ( u32 i = 0; i < 20000; i++ ) f.mem.tickFast( 1 );
	}
	CHECK_EQ( f.mem.m_IECThrottleEvents, events );
	CHECK( !f.mem.iecThrottleActive() );
	CHECK( !f.mem.iecBusActive() );
	// Ten reads the legacy predicate would have armed on, and not one cycle of
	// transaction time charged for them.
	CHECK_EQ( f.mem.m_IECArmReadRefused - refused0, (u64)10 );
	CHECK_EQ( f.mem.m_IECActivityArmedCycles - txn0, (u64)0 );
}

// The same ten reads under IEC_WINDOW 0, which is the behaviour that was
// measured at 98% armed. This test exists so the narrowing is shown against
// the defect rather than against nothing.
TEST( integration_legacy_iec_window_arms_on_the_guests_own_pull_down )
{
	SystemFixture f;
	f.start();
	f.mem.m_IECWindowMode = 0;

	f.mem.write8( 0xDD02, 0x3F );
	f.mem.write8( 0xDD00, 0x17 );
	f.bus.m_Memory[ 0xDD00 ] = 0x97;
	CHECK_EQ( f.mem.read8( 0xDD00 ), 0x97 );
	for ( u32 i = 0; i < 110000; i++ ) f.mem.tickFast( 1 );
	const u64 events = f.mem.m_IECThrottleEvents;

	for ( u32 frame = 0; frame < 10; frame++ )
	{
		CHECK_EQ( f.mem.read8( 0xDD00 ), 0x97 );
		for ( u32 i = 0; i < 20000; i++ ) f.mem.tickFast( 1 );
	}
	// Every frame re-armed a 100ms hold, so the machine never came back out of
	// it: one arm event, and the hold still live 200000 cycles later.
	CHECK( f.mem.m_IECThrottleEvents > events );
	CHECK( f.mem.iecThrottleActive() );
	CHECK_EQ( f.mem.m_IECArmReadRefused, (u64)0 );
}

// What the level test exists for must survive the narrowing: a drive holding a
// line WE released is still evidence, with no edge of its own. This is the
// K411 Test Drive case -- DATA held low across an EOI pause.
TEST( integration_a_drive_holding_a_line_we_released_still_arms )
{
	SystemFixture f;
	f.start();

	// Outputs configured, nothing asserted: PA3-PA5 all low in the latch.
	f.mem.write8( 0xDD02, 0x3F );
	f.mem.write8( 0xDD00, 0x07 );
	for ( u32 i = 0; i < 110000; i++ ) f.mem.tickFast( 1 );
	CHECK( !f.mem.iecBusActive() );

	// DATA IN low with DATA OUT released is the drive, not us.
	f.bus.m_Memory[ 0xDD00 ] = 0x40;
	CHECK_EQ( f.mem.read8( 0xDD00 ), 0x47 );
	CHECK( f.mem.iecBusActive() );
	CHECK( f.mem.m_IECArmReadEdge + f.mem.m_IECArmReadLevel > 0 );

	// And it keeps the window alive across repeated polls with no further edge.
	for ( u32 i = 0; i < 1500; i++ ) f.mem.tickFast( 1 );
	CHECK_EQ( f.mem.read8( 0xDD00 ), 0x47 );
	for ( u32 i = 0; i < 1500; i++ ) f.mem.tickFast( 1 );
	CHECK( f.mem.iecBusActive() );
}

// IEC_WINDOW 2: only an ATN assertion -- which every KERNAL and JiffyDOS
// conversation opens with -- or an input edge may OPEN a window. A CLK or DATA
// output change, which is what a VIC-bank write with stray bits looks like,
// may only re-arm one that is open.
TEST( integration_iec_window_mode2_needs_atn_to_open_a_transaction )
{
	SystemFixture f;
	f.start();
	f.mem.m_IECWindowMode = 2;

	f.mem.write8( 0xDD02, 0x3F );
	f.mem.write8( 0xDD00, 0x07 );
	for ( u32 i = 0; i < 110000; i++ ) f.mem.tickFast( 1 );
	CHECK( !f.mem.iecBusActive() );

	// CLK out asserted, no ATN, no window open: refused.
	f.mem.write8( 0xDD00, 0x17 );
	CHECK( !f.mem.iecBusActive() );
	CHECK_EQ( f.mem.m_IECArmWriteRefused, (u64)1 );

	// ATN asserted: a transaction is unambiguously starting.
	f.mem.write8( 0xDD00, 0x1F );
	CHECK( f.mem.iecBusActive() );

	// And while it is open, the bit-banging inside it re-arms as before.
	const u64 arms = f.mem.m_IECArmWrite;
	f.mem.write8( 0xDD00, 0x0F );
	CHECK_EQ( f.mem.m_IECArmWrite, arms + 1 );
	CHECK( f.mem.iecBusActive() );
}

// The instrumentation itself, because a ratio is only readable if both halves
// count what their names say. The edge window is 2000 cycles and the hold is
// 100000, which is the asymmetry that let the hold sit armed while the mirror
// gate looked quiet.
TEST( integration_iec_armed_time_and_transaction_time_are_counted_apart )
{
	SystemFixture f;
	f.start();

	f.mem.write8( 0xDD02, 0x3F );
	f.mem.write8( 0xDD00, 0x07 );
	for ( u32 i = 0; i < 110000; i++ ) f.mem.tickFast( 1 );
	f.bus.m_Memory[ 0xDD00 ] = 0x40;
	CHECK_EQ( f.mem.read8( 0xDD00 ), 0x47 );

	const u64 hold0 = f.mem.m_IECHoldArmedCycles;
	const u64 txn0 = f.mem.m_IECActivityArmedCycles;
	for ( u32 i = 0; i < 3000; i++ ) f.mem.tickFast( 1 );
	// Transaction time stops at the 2ms window; armed time keeps running.
	CHECK_EQ( f.mem.m_IECActivityArmedCycles - txn0, (u64)2000 );
	CHECK_EQ( f.mem.m_IECHoldArmedCycles - hold0, (u64)3000 );
	CHECK( !f.mem.iecBusActive() );
	CHECK( f.mem.iecThrottleActive() );
}

TEST( integration_dd02_direction_change_can_start_iec_activity )
{
	SystemFixture f;
	f.start();

	// Store a high ATN latch while PA3 is an input: no physical drive yet.
	f.mem.write8( 0xDD02, 0x00 );
	f.mem.write8( 0xDD00, 0x08 );
	for ( u32 i = 0; i < 110000; i++ ) f.mem.tickFast( 1 );
	CHECK( !f.mem.iecBusActive() );

	// Making PA3 an output now asserts the inverted open-collector line.
	f.mem.write8( 0xDD02, 0x08 );
	CHECK( f.mem.iecBusActive() );
}

TEST( integration_cia2_read_provides_iec_throttle_baseline )
{
	SystemFixture f;
	f.start();
	f.bus.m_Memory[ 0xDD00 ] = 0x17;
	CHECK_EQ( f.mem.read8( 0xDD00 ), 0x17 );
	f.mem.write8( 0xDD00, 0x07 );

	CHECK( f.mem.iecThrottleActive() );
}

TEST( integration_explicit_slow_mode_supersedes_iec_fallback )
{
	SystemFixture f;
	f.start();
	f.mem.setPacing( 0, SCPU_NORMAL_HZ );
	f.mem.write8( 0xDD00, 0x07 );
	f.mem.write8( 0xDD00, 0x17 );

	CHECK( !f.mem.iecThrottleActive() );
}

TEST( integration_iec_throttle_uses_effective_interrupt_sample_rate )
{
	struct CountingBus : CHostBus
	{
		u32 samples = 0;
		void sampleInterrupts( bool &irq, bool &nmi ) override
		{
			samples++;
			irq = m_IRQ;
			nmi = m_NMI;
		}
	} bus;

	CC64Memory mem;
	mem.attachBus( &bus );
	mem.setPacing( 0, SCPU_TURBO_HZ );
	mem.reset();
	mem.irqAsserted();
	CHECK_EQ( bus.samples, 1 );

	mem.tickFast( 1 );
	mem.irqAsserted();
	CHECK_EQ( bus.samples, 1 );		// nominal turbo cache interval is 20 cycles

	mem.write8( 0xDD00, 0x07 );
	mem.write8( 0xDD00, 0x17 );
	CHECK( mem.iecThrottleActive() );
	mem.irqAsserted();
	CHECK_EQ( bus.samples, 2 );		// effective 1MHz interval is one cycle
}

TEST( integration_reset_expires_iec_and_refreshes_interrupt_cache )
{
	struct CountingBus : CHostBus
	{
		u32 samples = 0;
		void sampleInterrupts( bool &irq, bool &nmi ) override
		{
			samples++;
			irq = m_IRQ;
			nmi = m_NMI;
		}
	} bus;

	CC64Memory mem;
	mem.attachBus( &bus );
	bus.m_NMI = true;
	CHECK( mem.nmiAsserted() );
	mem.write8( 0xDD00, 0x07 );
	mem.write8( 0xDD00, 0x17 );
	CHECK( mem.iecThrottleActive() );
	mem.tickFast( 7 );
	CHECK( mem.m_EmuCycles > 0 );
	CHECK( mem.m_MaxTickChunk > 0 );
	CHECK( mem.m_IOLogPos > 0 );
	CHECK( mem.m_CIALogPos > 0 );

	bus.m_NMI = false;
	mem.reset();
	CHECK( !mem.iecThrottleActive() );
	CHECK( !mem.iecBusActive() );
	CHECK_EQ( mem.m_IECThrottleEvents, 0 );
	CHECK_EQ( mem.m_EmuCycles, 0 );
	CHECK_EQ( mem.m_MaxTickChunk, 0 );
	CHECK_EQ( mem.m_IOLogPos, 0 );
	CHECK_EQ( mem.m_CIALogPos, 0 );
	CHECK( !mem.nmiAsserted() );
	CHECK_EQ( bus.samples, 2 );
}

TEST( integration_frame_budget_recalculates_after_speed_change )
{
	CHostBus bus;
	CSuperCPU scpu;
	u8 basic[ C64_BASIC_SIZE ];
	u8 kernal[ C64_KERNAL_SIZE ];
	std::memset( basic, 0xEA, sizeof( basic ) );
	std::memset( kernal, 0xEA, sizeof( kernal ) );

	const u8 code[] = { 0xA9, 0x00,			// LDA #$00
	                    0x8D, 0x7A, 0xD0,	// STA $D07A -- switch to 1MHz
	                    0x4C, 0x05, 0xE0 };	// JMP $E005
	std::memcpy( kernal, code, sizeof( code ) );
	kernal[ 0x1FFC ] = 0x00;
	kernal[ 0x1FFD ] = 0xE0;
	scpu.setBasicROM( basic );
	scpu.setKernalROM( kernal );
	CHECK( scpu.init( &bus, SCPU_CORE_65816, 0 ) );

	const u64 ran = scpu.runFrame();
	CHECK( !scpu.registers().fastMode() );
	CHECK( ran > 19000 );
	CHECK( ran < 21000 );	// old stale turbo budget ran about 393,000 cycles
}

TEST( integration_frame_budget_counts_automatic_iec_throttle_as_slow )
{
	CHostBus bus;
	CSuperCPU scpu;
	u8 basic[ C64_BASIC_SIZE ];
	u8 kernal[ C64_KERNAL_SIZE ];
	std::memset( basic, 0xEA, sizeof( basic ) );
	std::memset( kernal, 0xEA, sizeof( kernal ) );

	const u8 code[] = { 0xA9, 0x07, 0x8D, 0x00, 0xDD,
	                    0xA9, 0x17, 0x8D, 0x00, 0xDD,
	                    0x4C, 0x0A, 0xE0 };
	std::memcpy( kernal, code, sizeof( code ) );
	kernal[ 0x1FFC ] = 0x00;
	kernal[ 0x1FFD ] = 0xE0;
	scpu.setBasicROM( basic );
	scpu.setKernalROM( kernal );
	CHECK( scpu.init( &bus, SCPU_CORE_6502, 0 ) );

	const u64 ran = scpu.runFrame();
	CHECK( scpu.registers().fastMode() );
	CHECK( scpu.memory().iecThrottleActive() );
	CHECK( ran > 19000 );
	CHECK( ran < 21000 );
}

TEST( integration_serial_toggles_on_dd00_do_not_flush )
{
	// The IEC-killing regression, pinned. $DD00 is both the VIC bank select
	// AND the serial port: ATN, CLK and DATA live in bits 3-5 and an IEC
	// transfer toggles them thousands of times. Flushing the dirty buffer on
	// every toggle inserted milliseconds of stall between protocol edges; the
	// drive timed out mid-byte and the KERNAL hung forever in its no-timeout
	// receive wait. Only bits 0-1 -- the VIC bank -- may trigger the flush.
	SystemFixture f;
	const u8 code[] = {
		0xA9, 0x41, 0x8D, 0x00, 0x04,	// LDA #$41 / STA $0400  (dirty a byte)
		0xAD, 0x00, 0xDD,				// LDA $DD00             (seed baseline $97)
		0xA9, 0x87, 0x8D, 0x00, 0xDD,	// STA $DD00  CLK toggled  (bit 4: $97->$87)
		0xA9, 0x97, 0x8D, 0x00, 0xDD,	// STA $DD00  CLK back
		0xA9, 0xB7, 0x8D, 0x00, 0xDD,	// STA $DD00  DATA toggled (bit 5)
		0xA9, 0x96, 0x8D, 0x00, 0xDD	// STA $DD00  VIC BANK changed (bits 0-1)
	};
	f.installKernal( 0xE000, code, sizeof( code ), 0xE000 );
	f.start();
	f.wb.setOptMode( SCPU_OPT_NONE );
	f.bus.m_Memory[ 0xDD00 ] = 0x97;

	// Through the three serial-bit toggles the dirty byte must stay queued.
	// (9 instructions: the two setup, the baseline read, then three LDA/STA
	// pairs ending with the DATA toggle.)
	for ( int i = 0; i < 9; i++ ) f.cpu.step();
	CHECK_EQ( f.wb.pending(), 1 );

	// The VIC-bank change no longer flushes synchronously either -- nothing
	// does; see integration_display_state_writes_never_flush_synchronously.
	f.cpu.step(); f.cpu.step();
	CHECK_EQ( f.wb.pending(), 1 );

	f.wb.flush();
	CHECK_EQ( f.bus.m_Memory[ 0x0400 ], 0x41 );
}

TEST( integration_iec_activity_window_expires )
{
	// The black-screen regression, pinned. The serial-activity window
	// suppresses mirror flushing during a transaction; if it never counts
	// down, one boot-time serial probe suppresses mirroring FOREVER and the
	// machine runs perfectly behind a blank display. Arm it, run emulated
	// time past the window, and require it to expire.
	SystemFixture f;
	const u8 code[] = { 0xEA, 0x4C, 0x00, 0xE0 };	// NOP / JMP loop
	f.installKernal( 0xE000, code, sizeof( code ), 0xE000 );
	f.start();

	// Seed the baseline, then a serial-line transition arms the window.
	f.mem.read8( 0xDD00 );
	u8 v = f.bus.m_Memory[ 0xDD00 ];
	f.mem.write8( 0xDD00, (u8)( v ^ 0x08 ) );
	CHECK( f.mem.iecBusActive() );

	// The transaction gate spans IEC's EOI pause and remains active until the
	// full 2000-cycle quiet tail has elapsed.
	for ( u32 i = 0; i < 1999; i++ ) f.mem.tickFast( 1 );
	CHECK( f.mem.iecBusActive() );
	f.mem.tickFast( 1 );
	CHECK( !f.mem.iecBusActive() );

	// Mirror suppression and speed selection are intentionally independent:
	// the automatic 100000-cycle 1MHz fallback remains armed after the serial
	// transaction itself is quiet, so shortening the display blackout cannot
	// make the remainder of disk handling sprint at turbo speed.
	CHECK( f.mem.iecThrottleActive() );
	for ( u32 i = 2000; i < 100000; i++ ) f.mem.tickFast( 1 );
	CHECK( !f.mem.iecThrottleActive() );
}

TEST( integration_bulk_display_flip_never_waits_or_flushes_synchronously )
{
	// Raster IRQ code changes these registers at exact scanlines. Delaying the
	// I/O write while polling toward a border changes program semantics and the
	// polling itself consumes the physical bus through visible VIC fetches.
	// Large queues therefore follow the same immediate-write rule as small ones.
	SystemFixture f;
	const u8 code[] = { 0xA9, 0x3B, 0x8D, 0x18, 0xD0 };	// LDA #$3B / STA $D018
	f.installKernal( 0xE000, code, sizeof( code ), 0xE000 );
	f.start();
	f.wb.setOptMode( SCPU_OPT_NONE );

	// Stage a bitmap's worth of data -- well past the trivial threshold.
	for ( u32 i = 0; i < 100; i++ ) f.mem.write8( (u16)( 0x2000 + i ), (u8)i );
	CHECK( f.wb.pending() >= 100 );
	f.bus.resetStats();

	f.cpu.step(); f.cpu.step();		// LDA, then the flip

	CHECK_EQ( f.wb.pending(), 100 );
	CHECK_EQ( f.bus.m_Reads, 0 );		// no raster polling
	CHECK_EQ( f.bus.m_Writes, 1 );		// only the direct $D018 write
	CHECK_EQ( f.bus.m_Memory[ 0xD018 ], 0x3B );

	// The normal raster scheduler (represented here by an explicit test drain)
	// owns delivery of the queued RAM bytes.
	f.wb.flush();
	CHECK_EQ( f.bus.m_Memory[ 0x2063 ], 0x63 );
}

TEST( integration_raster_window_protects_frame_end_and_bounds_deferral )
{
	struct RasterWindowBus : CHostBus
	{
		CSuperCPU *owner = nullptr;
		u32 samples = 0;
		u32 protectedSamples = 0;
		u16 line = 100;
		u16 rasterLine() override
		{
			++samples;
			if ( owner && owner->memory().rasterCriticalWindowOpen()
			     && owner->m_RasterDeferRun != 0 )
				++protectedSamples;
			return line;
		}
	} bus;
	CSuperCPU scpu;
	u8 basic[ C64_BASIC_SIZE ];
	u8 kernal[ C64_KERNAL_SIZE ];
	std::memset( basic, 0xEA, sizeof basic );
	std::memset( kernal, 0xEA, sizeof kernal );
	const u8 code[] = {
		0xA9, 0x00,             // LDA #0
		0x8D, 0x7A, 0xD0,       // STA $D07A: select 1MHz
		0xAD, 0x12, 0xD0,       // LDA $D012: keep the timing window active
		0x4C, 0x05, 0xE0        // JMP $E005
	};
	std::memcpy( kernal, code, sizeof code );
	kernal[ 0x1FFC ] = 0x00;
	kernal[ 0x1FFD ] = 0xE0;
	scpu.setBasicROM( basic );
	scpu.setKernalROM( kernal );
	CHECK( scpu.init( &bus, SCPU_CORE_65816, 0 ) );
	scpu.writeBuffer().setOptMode( SCPU_OPT_NONE );
	scpu.setMirrorDisplayBudget( 0 );
	for ( u32 i = 0; i < 100; ++i )
		scpu.memory().write8( (u16)( 0x2000 + i ), (u8)i );
	bus.owner = &scpu;

	for ( u32 i = 0; i < 3; ++i )
		scpu.runFrame();
	CHECK( scpu.m_RasterDeferred > 0 );
	// Continuous raster polling must still reach the bounded escape. But a
	// protected opportunity must not be followed by an unguarded frame-end
	// raster query, border approach, or flush.
	CHECK( bus.samples > 0 );
	CHECK_EQ( bus.protectedSamples, 0u );
	CHECK_EQ( scpu.writeBuffer().pending(), 100u );
	// The optional coarse raster logger must honor the same decision, even
	// though it is outside the mirror-drain block.
	scpu.memory().enableVICLog( true );
	for ( u32 i = 0; i < 3; ++i )
		scpu.runFrame();
	CHECK_EQ( bus.protectedSamples, 0u );
	// Once an allowed opportunity observes the border, continuous D012
	// polling must not prevent actual delivery, either.
	bus.line = c64DisplayLastLine( bus.signals().video ) + 1;
	scpu.runFrame();
	CHECK_EQ( scpu.writeBuffer().pending(), 0u );
	CHECK_EQ( bus.protectedSamples, 0u );
}

TEST( integration_unknown_raster_never_authorizes_a_flush )
{
	struct UnknownRasterBus : CHostBus
	{
		u32 samples = 0;
		u16 rasterLine() override { samples++; return 0xFFFF; }
	} bus;

	CSuperCPU scpu;
	u8 basic[ C64_BASIC_SIZE ];
	u8 kernal[ C64_KERNAL_SIZE ];
	std::memset( basic, 0xEA, sizeof basic );
	std::memset( kernal, 0xEA, sizeof kernal );
	kernal[ 0 ] = 0x4C;		// JMP $E000
	kernal[ 1 ] = 0x00;
	kernal[ 2 ] = 0xE0;
	kernal[ 0x1FFC ] = 0x00;
	kernal[ 0x1FFD ] = 0xE0;
	scpu.setBasicROM( basic );
	scpu.setKernalROM( kernal );
	CHECK( scpu.init( &bus, SCPU_CORE_6502, 0 ) );
	scpu.writeBuffer().setOptMode( SCPU_OPT_NONE );

	for ( u32 i = 0; i < 100; i++ )
		scpu.memory().write8( (u16)( 0x2000 + i ), (u8)i );
	bus.resetStats();

	scpu.runFrame();
	CHECK( bus.samples > 0 );
	CHECK_EQ( bus.m_BurstWrites, 0 );
	CHECK_EQ( scpu.writeBuffer().pending(), 100 );

	CHECK( !c64RasterIsSafeForBulkTransfer( VIDEO_PAL, 0xFFFF ) );
	CHECK( !c64RasterIsSafeForBulkTransfer( VIDEO_PAL, 312 ) );
	CHECK( !c64RasterIsSafeForBulkTransfer( VIDEO_NTSC_R56A, 511 ) );
}

TEST( integration_mirror_drain_rechecks_raster_before_each_small_chunk )
{
	struct OneSafeSampleBus : CHostBus
	{
		u32 samples = 0;
		u16 rasterLine() override
		{
			// The first sample is in the top border. Every later sample is in
			// the visible display, so exactly one guarded chunk may leave.
			return samples++ == 0 ? 0 : 100;
		}
	} bus;

	CSuperCPU scpu;
	u8 basic[ C64_BASIC_SIZE ];
	u8 kernal[ C64_KERNAL_SIZE ];
	std::memset( basic, 0xEA, sizeof basic );
	std::memset( kernal, 0xEA, sizeof kernal );
	kernal[ 0 ] = 0x4C;              // JMP $E000
	kernal[ 1 ] = 0x00;
	kernal[ 2 ] = 0xE0;
	kernal[ 0x1FFC ] = 0x00;
	kernal[ 0x1FFD ] = 0xE0;
	scpu.setBasicROM( basic );
	scpu.setKernalROM( kernal );
	CHECK( scpu.init( &bus, SCPU_CORE_6502, 0 ) );
	scpu.writeBuffer().setOptMode( SCPU_OPT_NONE );
	scpu.setMirrorDisplayBudget( 0 );		// strict border-only

	for ( u32 i = 0; i < 200; i++ )
		scpu.memory().write8( (u16)( 0x2000 + i ), (u8)i );
	CHECK_EQ( scpu.writeBuffer().pending(), 200 );
	bus.resetStats();
	bus.samples = 0;

	scpu.runFrame();

	CHECK( bus.samples > 1 );
	CHECK_EQ( bus.m_BurstWrites, 64 );
	CHECK_EQ( scpu.writeBuffer().pending(), 136 );
}

TEST( integration_mirror_drains_inside_the_display_within_its_ration )
{
	// Border-only mirroring bounds delivery at roughly 3KB per frame, and only
	// on frames where a raster sample catches the border at all. A game that
	// redraws its moving objects every frame outruns that, so the VIC fetches a
	// mixture of several frames' bytes for exactly the parts that move while
	// static scenery -- delivered long ago -- stays perfect.
	//
	// Writing inside the picture is safe: the burst path re-samples BA and
	// hands the bus back whenever the VIC claims it, which is what a real REU
	// does. So a display ration must drain, and must stop at the ration.
	struct AlwaysDisplayBus : CHostBus
	{
		u16 rasterLine() override { return 100; }	// never in the border
	} bus;

	CSuperCPU scpu;
	u8 basic[ C64_BASIC_SIZE ];
	u8 kernal[ C64_KERNAL_SIZE ];
	std::memset( basic, 0xEA, sizeof basic );
	std::memset( kernal, 0xEA, sizeof kernal );
	kernal[ 0 ] = 0x4C;              // JMP $E000
	kernal[ 1 ] = 0x00;
	kernal[ 2 ] = 0xE0;
	kernal[ 0x1FFC ] = 0x00;
	kernal[ 0x1FFD ] = 0xE0;
	scpu.setBasicROM( basic );
	scpu.setKernalROM( kernal );
	CHECK( scpu.init( &bus, SCPU_CORE_6502, 0 ) );
	scpu.writeBuffer().setOptMode( SCPU_OPT_NONE );
	scpu.setMirrorDisplayBudget( 128 );

	for ( u32 i = 0; i < 4000; i++ )
		scpu.memory().write8( (u16)( 0x2000 + i ), (u8)i );
	const u32 staged = scpu.writeBuffer().pending();
	CHECK( staged >= 4000 );
	bus.resetStats();

	scpu.runFrame();

	// The legacy 128-byte allowance is divided across 128 opportunities. That
	// preserves roughly the former per-frame ceiling while turning each long
	// pause into a small installment.
	const u32 sent = staged - scpu.writeBuffer().pending();
	CHECK( sent >= 128 );
	CHECK( sent <= 9 * 128 );
}

TEST( integration_border_only_mode_sends_nothing_inside_the_display )
{
	// The escape hatch: MIRROR_DISPLAY_BYTES 0 restores the old rule exactly,
	// so a card can back this out without a rebuild.
	struct AlwaysDisplayBus : CHostBus
	{
		u16 rasterLine() override { return 100; }
	} bus;

	CSuperCPU scpu;
	u8 basic[ C64_BASIC_SIZE ];
	u8 kernal[ C64_KERNAL_SIZE ];
	std::memset( basic, 0xEA, sizeof basic );
	std::memset( kernal, 0xEA, sizeof kernal );
	kernal[ 0 ] = 0x4C;
	kernal[ 1 ] = 0x00;
	kernal[ 2 ] = 0xE0;
	kernal[ 0x1FFC ] = 0x00;
	kernal[ 0x1FFD ] = 0xE0;
	scpu.setBasicROM( basic );
	scpu.setKernalROM( kernal );
	CHECK( scpu.init( &bus, SCPU_CORE_6502, 0 ) );
	scpu.writeBuffer().setOptMode( SCPU_OPT_NONE );
	scpu.setMirrorDisplayBudget( 0 );

	for ( u32 i = 0; i < 200; i++ )
		scpu.memory().write8( (u16)( 0x2000 + i ), (u8)i );
	bus.resetStats();

	scpu.runFrame();
	CHECK_EQ( bus.m_BurstWrites, 0 );
	CHECK_EQ( scpu.writeBuffer().pending(), 200 );
}

TEST( integration_polling_dd00_with_idle_lines_never_latches_iec_activity )
{
	// The mirror-blackout latch-up, pinned. Re-arming the activity window on
	// any $DD00 read once it happened to be open made it self-sustaining: that
	// window is only ~2.5ms of real time at 20MHz, so ordinary code reading
	// $DD00 a few times a frame held it open forever. iecBusActive() gates ALL
	// mirroring, so the physical screen froze while the machine ran on.
	SystemFixture f;
	f.start();
	f.mem.write8( 0xDD02, 0x3F );		// PA6/PA7 inputs, as the KERNAL sets

	// A real transaction opens the window.
	f.bus.m_Memory[ 0xDD00 ] = 0x7F;		// DATA in low: a drive is talking
	f.mem.read8( 0xDD00 );
	CHECK( f.mem.iecBusActive() );

	// The drive releases. From here on the lines are idle high and the only
	// traffic is a game polling the register -- which must not hold the
	// window open. Poll far more often than the window is long.
	f.bus.m_Memory[ 0xDD00 ] = 0xFF;
	for ( u32 i = 0; i < 200000; i++ )
	{
		if ( ( i % 1000 ) == 0 ) f.mem.read8( 0xDD00 );
		f.mem.tickFast( 1 );
	}
	CHECK( !f.mem.iecBusActive() );
}

TEST( integration_frame_end_runs_the_cpu_toward_the_border_instead_of_missing_it )
{
	// Every other drain is opportunistic: sample the raster, transfer only if
	// the beam happens to be outside the picture. The border is under a
	// quarter of the frame, so a meaningful share of frames delivered NOTHING
	// -- and a frame that delivers nothing shows the previous frame's objects,
	// which is a flicker no budget size can fix.
	//
	// At frame end the machine now approaches the border deliberately: it runs
	// the emulated CPU for exactly the time the beam needs to get there. The
	// wait is useful work rather than a bus poll, and arrival is by
	// construction rather than by luck.
	struct FixedRasterBus : CHostBus
	{
		u16 line = 250;
		u16 rasterLine() override { return line; }
	};

	auto runOne = []( u16 rasterAt ) -> u64
	{
		static FixedRasterBus bus;
		bus.line = rasterAt;
		static CSuperCPU scpu;
		u8 basic[ C64_BASIC_SIZE ];
		u8 kernal[ C64_KERNAL_SIZE ];
		std::memset( basic, 0xEA, sizeof basic );
		std::memset( kernal, 0xEA, sizeof kernal );
		kernal[ 0 ] = 0x4C;              // JMP $E000
		kernal[ 1 ] = 0x00;
		kernal[ 2 ] = 0xE0;
		kernal[ 0x1FFC ] = 0x00;
		kernal[ 0x1FFD ] = 0xE0;
		scpu.setBasicROM( basic );
		scpu.setKernalROM( kernal );
		CHECK( scpu.init( &bus, SCPU_CORE_6502, 0 ) );
		scpu.writeBuffer().setOptMode( SCPU_OPT_NONE );
		scpu.setMirrorDisplayBudget( 0 );

		for ( u32 i = 0; i < 100; i++ )
			scpu.memory().write8( (u16)( 0x2000 + i ), (u8)i );
		return scpu.runFrame();
	};

	// Beam already in the bottom border: nothing to approach.
	const u64 atBorder = runOne( 250 );

	// Beam mid-picture: the frame runs on until the border is reached.
	const u64 inDisplay = runOne( 100 );

	CHECK( inDisplay > atBorder );
}

TEST( integration_frame_end_border_approach_is_repaid_next_frame )
{
	// Advancing to the physical border is useful work, but it must not be free
	// work. A fixed mid-display sample makes the first call borrow a known
	// amount; the next call must execute less ordinary frame work while repaying
	// it. Otherwise the sampled raster phase becomes a variable CPU overclock.
	struct FixedRasterBus : CHostBus
	{
		u16 rasterLine() override { return 100; }
	} bus;

	CSuperCPU scpu;
	u8 basic[ C64_BASIC_SIZE ];
	u8 kernal[ C64_KERNAL_SIZE ];
	std::memset( basic, 0xEA, sizeof basic );
	std::memset( kernal, 0xEA, sizeof kernal );
	kernal[ 0 ] = 0x4C;
	kernal[ 1 ] = 0x00;
	kernal[ 2 ] = 0xE0;
	kernal[ 0x1FFC ] = 0x00;
	kernal[ 0x1FFD ] = 0xE0;
	scpu.setBasicROM( basic );
	scpu.setKernalROM( kernal );
	CHECK( scpu.init( &bus, SCPU_CORE_6502, 0 ) );
	scpu.writeBuffer().setOptMode( SCPU_OPT_NONE );
	scpu.setMirrorDisplayBudget( 0 );

	for ( u32 i = 0; i < 100; i++ )
		scpu.memory().write8( (u16)( 0x2000 + i ), (u8)i );
	const u64 borrowingFrame = scpu.runFrame();

	// Keep a mirror outstanding so this call takes the same border-approach
	// path. Its total must nevertheless be one normal frame, not frame+approach.
	for ( u32 i = 0; i < 100; i++ )
		scpu.memory().write8( (u16)( 0x2100 + i ), (u8)i );
	const u64 repayingFrame = scpu.runFrame();

	CHECK( repayingFrame < borrowingFrame );
}

TEST( integration_active_screen_sprite_pointers_reach_the_bus_immediately )
{
	// Double-buffered sprite animation flips a pointer in vblank and starts
	// redrawing the retired block at once. A pointer flip that arrives even a
	// fraction of a frame late leaves the physical VIC displaying the block
	// the game is now overwriting -- sprites flicker between correct and
	// garbage at animation rate. So the ACTIVE screen's eight pointer bytes
	// bypass the queue and land on the real bus like a VIC register write;
	// everything else keeps queueing.
	SystemFixture f;
	f.start();
	f.wb.setOptMode( SCPU_OPT_NONE );

	// Default machine: bank 0, $D018=$14 -> screen $0400, pointers $07F8.
	f.mem.write8( 0x07F8, 0x55 );
	CHECK_EQ( f.bus.m_Memory[ 0x07F8 ], 0x55 );		// immediate, no flush

	// A neighbouring screen byte still queues.
	f.mem.write8( 0x07F0, 0x66 );
	CHECK( f.bus.m_Memory[ 0x07F0 ] != 0x66 );

	// Move the screen: $D018 top nibble selects matrix $2000, pointers $23F8.
	f.mem.write8( 0xD018, 0x80 );
	f.mem.write8( 0x23F8, 0x77 );
	CHECK_EQ( f.bus.m_Memory[ 0x23F8 ], 0x77 );
	f.mem.write8( 0x07F8, 0x99 );					// old row no longer special
	CHECK( f.bus.m_Memory[ 0x07F8 ] != 0x99 );

	// And the queued copies still flush the same values later.
	f.wb.flush();
	CHECK_EQ( f.bus.m_Memory[ 0x07F0 ], 0x66 );
	CHECK_EQ( f.bus.m_Memory[ 0x07F8 ], 0x99 );
}

TEST( integration_active_screen_base_tracks_d018_and_vic_bank )
{
	SystemFixture f;
	f.start();

	CHECK_EQ( f.mem.activeScreenBase(), 0x0400u );
	CHECK_EQ( f.mem.activeVICBankBase(), 0x0000u );
	CHECK_EQ( f.mem.activeVICScreenBase(), 0x0400u );
	f.mem.write8( 0xD018, 0xDF );			// matrix offset $3400
	CHECK_EQ( f.mem.activeScreenBase(), 0x3400u );
	f.mem.write8( 0xDD00, 0xC6 );			// VIC bank 1 ($4000)
	CHECK_EQ( f.mem.activeScreenBase(), 0x7400u );	// Metal Dust snapshot
	CHECK_EQ( f.mem.activeVICBankBase(), 0x4000u );
	CHECK_EQ( f.mem.activeVICScreenBase(), 0x7400u );
	f.mem.write8( 0xDD02, 0x00 );			// PA0/PA1 inputs float high
	CHECK_EQ( f.mem.activeScreenBase(), 0x3400u );	// back to VIC bank 0
	f.mem.write8( 0xDD02, 0x03 );			// drive both bank-select pins
	CHECK_EQ( f.mem.activeScreenBase(), 0x7400u );

	// A bank-3 matrix at $D000 is behind the real machine's I/O window. The
	// fallback keeps its guest address visible to the scheduler and relocates
	// only the physical VIC backing.
	f.mem.write8( 0xDD00, 0xC4 );			// VIC bank 3 ($C000)
	f.mem.write8( 0xD018, 0x40 );			// matrix offset $1000
	CHECK_EQ( f.mem.activeScreenBase(), 0xD000u );
	CHECK_EQ( f.mem.activeVICBankBase(), 0xC000u );
	CHECK_EQ( f.mem.activeVICScreenBase(), 0xD000u );
	CHECK( f.mem.m_DisplayRelocActive );

	// The verified real-SuperCPU map makes that physical DRAM deliverable. The
	// urgent sprite-pointer path must explicitly target RAM too; an ordinary
	// D-window bus write would select the overlaid VIC/SID/CIA registers.
	f.wb.setRAMUnderIOAccessible( true );
	f.mem.setRAMUnderIOAccessible( true );
	CHECK_EQ( f.mem.activeScreenBase(), 0xD000u );
	f.mem.write8( 0x0001, 0x34 );			// guest sees RAM under I/O
	f.mem.write8( 0xD3F8, 0x55 );
	CHECK_EQ( f.bus.m_Memory[ 0xD3F8 ], (u8)0x55 );
}

TEST( integration_active_bitmap_base_tracks_mode_d018_and_vic_bank )
{
	SystemFixture f;
	f.start();

	CHECK_EQ( f.mem.activeBitmapBase(), 0xFFFFFFFFu );
	f.mem.write8( 0xD011, 0x3B );			// bitmap mode on
	CHECK_EQ( f.mem.activeBitmapBase(), 0x0000u );
	f.mem.write8( 0xD018, 0x1C );			// bitmap high half
	CHECK_EQ( f.mem.activeBitmapBase(), 0x2000u );
	f.mem.write8( 0xDD00, 0xC6 );			// VIC bank 1
	CHECK_EQ( f.mem.activeBitmapBase(), 0x6000u );

	// Bitmap selection remains known even when the screen matrix is under I/O.
	f.mem.write8( 0xDD00, 0xC4 );			// VIC bank 3
	f.mem.write8( 0xD018, 0x40 );			// screen $D000, bitmap $C000
	CHECK_EQ( f.mem.activeScreenBase(), 0xD000u );
	CHECK_EQ( f.mem.activeBitmapBase(), 0xC000u );
	f.mem.write8( 0xD011, 0x1B );			// bitmap mode off
	CHECK_EQ( f.mem.activeBitmapBase(), 0xFFFFFFFFu );
}

TEST( integration_active_charset_base_tracks_d018_bank_and_character_rom )
{
	SystemFixture f;
	f.start();

	// Default D018=$14 selects the VIC's internal character ROM at $1000.
	CHECK_EQ( f.mem.activeCharsetBase(), 0xFFFFFFFFu );
	f.mem.write8( 0xD018, 0x02 );			// bank-0 DRAM charset $0800
	CHECK_EQ( f.mem.activeCharsetBase(), 0x0800u );
	f.mem.write8( 0xDD00, 0xC6 );			// VIC bank 1
	CHECK_EQ( f.mem.activeCharsetBase(), 0x4800u );
	f.mem.write8( 0xD018, 0x04 );			// bank-1 DRAM charset $5000
	CHECK_EQ( f.mem.activeCharsetBase(), 0x5000u );
	f.mem.write8( 0xDD00, 0xC5 );			// VIC bank 2
	f.mem.write8( 0xD018, 0x02 );			// offset $0800 is DRAM
	CHECK_EQ( f.mem.activeCharsetBase(), 0x8800u );
	f.mem.write8( 0xD018, 0x06 );			// offset $1800 is character ROM
	CHECK_EQ( f.mem.activeCharsetBase(), 0xFFFFFFFFu );
	f.mem.write8( 0xD018, 0x08 );			// offset $2000 is DRAM again
	CHECK_EQ( f.mem.activeCharsetBase(), 0xA000u );
	f.mem.write8( 0xD011, 0x3B );			// bitmap mode suppresses charset
	CHECK_EQ( f.mem.activeCharsetBase(), 0xFFFFFFFFu );
}

TEST( integration_d011_read_uses_shadowed_control_and_live_raster_high_bit )
{
	SystemFixture f;
	f.start();

	// Establish a valid hires-bitmap control latch through the same write path
	// used by the emulated CPU, then corrupt only the physical readback. GEOS
	// reads, masks bit 7 and writes this value back; the low seven bits must not
	// be allowed to inherit expansion-bus residue.
	f.mem.write8( 0xD011, 0x3B );
	f.bus.m_Memory[ 0xD011 ] = 0x7F;
	CHECK_EQ( f.mem.read8( 0xD011 ), (u8)0x3B );

	// Bit 7 remains live and comes from the physical raster counter rather than
	// the shadowed compare value.
	f.bus.m_Cycles = (u64)300 * c64CyclesPerLine( VIDEO_PAL );
	CHECK_EQ( f.mem.read8( 0xD011 ), (u8)0xBB );
}

// A bus whose $D011 and $D012 can be made to DISAGREE. CHostBus answers both
// from one rasterLineInternal(), so it is constitutionally coherent and cannot
// express the sample the card captured: bit 7 set while the low byte says the
// beam is nowhere near 256. This is the only way to drive the refusal arm.
struct SplitRasterBus : CHostBus
{
	u8 d011 = 0;
	u8 d012 = 0;
	u32 d012Reads = 0;

	u8 read( u16 addr ) override
	{
		// Answer every $40-byte mirror in $D000-$D1FF, not just the base
		// address: the mirror test's whole point is that the reconstruction
		// used to miss them, and a bus that only answered $D011 would have
		// handed the other seven a zero and hidden the hole it is looking for.
		if ( addr >= 0xD000 && addr < 0xD200 )
		{
			if ( ( addr & 0x3F ) == 0x11 ) { m_Reads++; return d011; }
			if ( ( addr & 0x3F ) == 0x12 )
				{ m_Reads++; d012Reads++; return d012; }
		}
		return CHostBus::read( addr );
	}
};

TEST( integration_d011_bit7_refused_when_it_composes_to_no_raster_line )
{
	// K471's capture, reproduced exactly: $D011 reads back $97 while $D012 says
	// 62. 256 + 62 = 318 and NTSC-R8 has 263 lines, so the sample describes a
	// line that does not exist and bit 8 can only be 0. Before this check the
	// $80 went into the latch through DEC/INC $D011, m_RasterCompare composed
	// 318, and Mario's chain waited for it for ever.
	SplitRasterBus bus;
	CC64Memory mem;
	mem.attachBus( &bus );
	mem.reset();
	bus.m_Signals.video = VIDEO_NTSC_R8;

	mem.write8( 0xD011, 0x17 );			// the guest's real control latch
	bus.d011 = 0x97;
	bus.d012 = 62;
	bus.d012Reads = 0;

	CHECK_EQ( mem.read8( 0xD011 ), (u8)0x17 );
	CHECK_EQ( bus.d012Reads, 1u );
	CHECK_EQ( mem.m_D011RST8Samples, 1u );
	CHECK_EQ( mem.m_D011RST8Rejected, 1u );
	CHECK_EQ( mem.m_D011RST8LastRejLow, (u8)62 );

	// And the write-back a DEC/INC pair would make now carries no RST8, so the
	// composed compare stays reachable.
	mem.write8( 0xD011, (u8)( mem.read8( 0xD011 ) - 1 ) );
	mem.write8( 0xD011, (u8)( mem.read8( 0xD011 ) + 1 ) );
	mem.write8( 0xD012, 62 );
	CHECK_EQ( mem.rasterCompare(), (u16)62 );
	CHECK_EQ( mem.m_D011RST8Writes, 0u );
	CHECK_EQ( mem.m_RasterCompareImpossible, 0u );
}

TEST( integration_d011_bit7_survives_when_the_beam_really_is_past_255 )
{
	// The other arm, and the one that proves this is not a mask. A guest
	// polling $D011 for raster bit 8 must still get it: lines 256..262 exist on
	// NTSC-R8, $D012 reads 0..6 there, and 256 + 5 = 261 is a real line.
	SplitRasterBus bus;
	CC64Memory mem;
	mem.attachBus( &bus );
	mem.reset();
	bus.m_Signals.video = VIDEO_NTSC_R8;

	mem.write8( 0xD011, 0x1B );
	bus.d011 = 0x80;
	bus.d012 = 5;

	CHECK_EQ( mem.read8( 0xD011 ), (u8)0x9B );
	CHECK_EQ( mem.m_D011RST8Samples, 1u );
	CHECK_EQ( mem.m_D011RST8Rejected, 0u );

	// PAL has 312 lines, so a low byte that is impossible on NTSC is ordinary
	// there. The check is the machine's line count, never a constant.
	bus.m_Signals.video = VIDEO_PAL;
	bus.d012 = 44;						// line 300
	CHECK_EQ( mem.read8( 0xD011 ), (u8)0x9B );
	CHECK_EQ( mem.m_D011RST8Rejected, 0u );
	bus.m_Signals.video = VIDEO_NTSC_R8;
	CHECK_EQ( mem.read8( 0xD011 ), (u8)0x1B );
	CHECK_EQ( mem.m_D011RST8Rejected, 1u );
}

TEST( integration_d011_read_with_bit7_clear_costs_no_confirming_transfer )
{
	// The cost argument has to be testable, not asserted. Bit 7 clear is the
	// overwhelming majority of $D011 reads -- 256 lines in 263 on NTSC -- and
	// those must not pay for a second bus access.
	SplitRasterBus bus;
	CC64Memory mem;
	mem.attachBus( &bus );
	mem.reset();
	bus.m_Signals.video = VIDEO_NTSC_R8;

	mem.write8( 0xD011, 0x1B );
	bus.d011 = 0x1B;
	bus.d012Reads = 0;
	for ( u32 i = 0; i < 16; i++ )
		CHECK_EQ( mem.read8( 0xD011 ), (u8)0x1B );
	CHECK_EQ( bus.d012Reads, 0u );
	CHECK_EQ( mem.m_D011RST8Samples, 0u );
}

TEST( integration_d011_mirrors_reconstruct_exactly_like_the_base_address )
{
	// VIC registers repeat every $40 bytes, and write8 has always shadowed the
	// whole $D000-$D1FF window into m_VICRegShadow[ 0x11 ]. The read side used
	// to test the exact address, so $D051 and its siblings handed the guest raw
	// physical bits 6..0 -- the GEOS residue hazard, reachable through a mirror.
	SplitRasterBus bus;
	CC64Memory mem;
	mem.attachBus( &bus );
	mem.reset();
	bus.m_Signals.video = VIDEO_NTSC_R8;

	mem.write8( 0xD051, 0x3B );			// a mirror write updates the shadow
	bus.d011 = 0x7F;					// residue on the physical readback
	bus.d012 = 62;
	static const u16 mirrors[] =
		{ 0xD011, 0xD051, 0xD091, 0xD0D1, 0xD111, 0xD151, 0xD191, 0xD1D1 };
	for ( u32 i = 0; i < 8; i++ )
		CHECK_EQ( mem.read8( mirrors[ i ] ), (u8)0x3B );

	bus.d011 = 0xFF;					// now with bit 7 set and incoherent
	for ( u32 i = 0; i < 8; i++ )
		CHECK_EQ( mem.read8( mirrors[ i ] ), (u8)0x3B );
	CHECK_EQ( mem.m_D011RST8Rejected, 8u );
}

TEST( integration_an_impossible_raster_compare_is_counted_and_never_anchored )
{
	// The two downstream guards. A guest that really does arm a line its
	// machine has not is reported as having done so -- m_RasterCompare is NOT
	// clamped, because a plausible neighbour would have hidden exactly the
	// reading that cost K471 a capture -- and the HDMI raster anchor refuses it
	// rather than placing every split in the frame 55 lines out.
	SplitRasterBus bus;
	CC64Memory mem;
	mem.attachBus( &bus );
	mem.reset();
	bus.m_Signals.video = VIDEO_NTSC_R8;
	mem.enableVICLog( true );

	mem.write8( 0xD012, 62 );
	CHECK_EQ( mem.m_RasterCompareImpossible, 0u );
	CHECK_EQ( mem.rasterAnchor() != 0ULL, true );
	const u64 goodAnchor = mem.rasterAnchor();

	mem.write8( 0xD011, 0x97 );			// RST8 by hand: compare becomes 318
	CHECK_EQ( mem.rasterCompare(), (u16)318 );
	CHECK_EQ( mem.m_D011RST8Writes, 1u );
	CHECK_EQ( mem.m_RasterCompareImpossible, 1u );

	// The anchor is taken BEFORE the shadow moves, on purpose -- the beam is at
	// the OLD compare when a handler re-aims it -- so the write that creates 318
	// still anchors the reachable 62. The refusal belongs to the NEXT handler,
	// the one that would anchor 318, which is exactly the frame in which K471's
	// splits would have gone 55 lines out.
	mem.tick( 300 );					// clear the 256-cycle anchor guard
	mem.write8( 0xD019, 0x01 );			// the chain's acknowledge
	CHECK_EQ( mem.m_RasterAnchorRefused, 1u );
	CHECK_EQ( mem.rasterAnchor(), goodAnchor );
}

TEST( integration_c64_cia2_seed_repeats_only_the_verified_ddra_write )
{
	CIA2SeedBus bus;
	CC64Memory mem;
	mem.attachBus( &bus );
	mem.reset();
	bus.m_LogEnabled = true;

	CHECK( mem.initializeC64CIA2() );
	CHECK_EQ( bus.verificationCalls, 1u );
	CHECK_EQ( bus.m_LogCount, 2u );
	CHECK_EQ( bus.m_Log[ 0 ].op, HOSTOP_WRITE );
	CHECK_EQ( bus.m_Log[ 0 ].addr, 0xDD02 );
	CHECK_EQ( bus.m_Log[ 0 ].value, 0x3F );
	CHECK_EQ( bus.m_Log[ 1 ].op, HOSTOP_WRITE );
	CHECK_EQ( bus.m_Log[ 1 ].addr, 0xDD02 );
	CHECK_EQ( bus.m_Log[ 1 ].value, 0x3F );

	// Ordinary guest writes remain single accesses; the repeat is confined to
	// the dedicated pure-latch initialization above.
	mem.write8( 0xDD02, 0x37 );
	CHECK_EQ( bus.m_LogCount, 3u );
	CHECK_EQ( bus.m_Log[ 2 ].value, 0x37 );
}

TEST( integration_failed_cia2_seed_verification_leaves_ddra_unknown )
{
	CIA2SeedBus bus;
	CC64Memory mem;
	mem.attachBus( &bus );
	mem.reset();
	bus.allowVerification = false;

	CHECK( !mem.initializeC64CIA2() );
	bus.m_Memory[ 0xDD02 ] = 0xA5;
	bus.resetStats();
	CHECK_EQ( mem.read8( 0xDD02 ), (u8)0xA5 );
	CHECK_EQ( bus.m_Reads, 1u );
}

TEST( integration_verified_cia2_ddra_shadow_rejects_poisoned_physical_reads )
{
	CIA2SeedBus bus;
	CC64Memory mem;
	mem.attachBus( &bus );
	mem.reset();
	CHECK( mem.initializeC64CIA2() );

	bus.m_Memory[ 0xDD02 ] = 0x00;
	bus.resetStats();
	const u8 rmw = mem.read8( 0xDD02 );
	CHECK_EQ( rmw, (u8)0x3F );
	CHECK_EQ( bus.m_Reads, 0u );
	mem.write8( 0xDD02, rmw );
	CHECK_EQ( bus.m_Memory[ 0xDD02 ], (u8)0x3F );
}

TEST( integration_cia2_pra_read_combines_latch_outputs_with_physical_inputs )
{
	CIA2SeedBus bus;
	CC64Memory mem;
	mem.attachBus( &bus );
	mem.reset();
	CHECK( mem.initializeC64CIA2() );
	mem.write8( 0xDD00, 0x35 );

	// DDRA=$3F: PA0-PA5 are outputs from the hidden latch; PA6-PA7 are live
	// physical inputs. Neither half may overwrite the other.
	bus.m_Memory[ 0xDD00 ] = 0xC0;
	CHECK_EQ( mem.read8( 0xDD00 ), (u8)0xF5 );
	bus.m_Memory[ 0xDD00 ] = 0x00;
	CHECK_EQ( mem.read8( 0xDD00 ), (u8)0x35 );
}

TEST( integration_cia2_pra_rmw_residue_cannot_move_the_vic_bank )
{
	CIA2SeedBus bus;
	CC64Memory mem;
	mem.attachBus( &bus );
	mem.reset();
	CHECK( mem.initializeC64CIA2() );
	mem.write8( 0xDD00, 0xC6 );		// VIC bank 1
	CHECK_EQ( mem.activeScreenBase(), 0x4400u );

	// Physical residue claims PA0/PA1 are both low. A read-modify-write must
	// preserve the output latch's %10 bank selection rather than changing it.
	bus.m_Memory[ 0xDD00 ] = 0x00;
	const u8 rmw = mem.read8( 0xDD00 );
	mem.write8( 0xDD00, rmw );
	CHECK_EQ( rmw & 3, (u8)2 );
	CHECK_EQ( mem.activeScreenBase(), 0x4400u );
}

TEST( integration_visible_drain_defers_screen_but_not_cold_traffic )
{
	struct FixedRasterBus : CHostBus
	{
		u16 line = 100;
		u32 maxBurst = 0;
		u16 rasterLine() override { return line; }
		void writeBurst( const C64BusWrite *writes, u32 count ) override
		{
			if ( count > maxBurst ) maxBurst = count;
			CHostBus::writeBurst( writes, count );
		}
	} bus;

	CSuperCPU scpu;
	u8 basic[ C64_BASIC_SIZE ];
	u8 kernal[ C64_KERNAL_SIZE ];
	std::memset( basic, 0xEA, sizeof basic );
	std::memset( kernal, 0xEA, sizeof kernal );
	kernal[ 0 ] = 0x4C;              // JMP $E000
	kernal[ 1 ] = 0x00;
	kernal[ 2 ] = 0xE0;
	kernal[ 0x1FFC ] = 0x00;
	kernal[ 0x1FFD ] = 0xE0;
	scpu.setBasicROM( basic );
	scpu.setKernalROM( kernal );
	CHECK( scpu.init( &bus, SCPU_CORE_6502, 0 ) );
	scpu.writeBuffer().setOptMode( SCPU_OPT_NONE );
	scpu.setMirrorDisplayBudget( 256 );

	// Put the active matrix at the FIFO head, as a scrolling screen does, then
	// queue unrelated VIC-visible RAM behind it.
	// The final eight matrix bytes are live sprite pointers and deliberately
	// bypass the queue. Queue every ordinary matrix byte, including all 16
	// normally-unused VSP-visible bytes immediately before that pointer row.
	const u32 queuedMatrixBytes = SCPU_VIC_MATRIX_BYTES - 8u;
	for ( u32 i = 0; i < queuedMatrixBytes; i++ )
		scpu.memory().write8( (u16)( 0x0400 + i ), (u8)( i ^ 0xA5 ) );
	scpu.memory().write8( 0x2000, 0x5A );

	// With the beam held in the picture, cold traffic progresses past all 1024
	// deferred matrix-ring entries but no matrix byte reaches physical DRAM.
	scpu.runFrame();
	CHECK_EQ( bus.m_Memory[ 0x2000 ], 0x5A );
	CHECK( bus.m_Memory[ 0x0400 ] != (u8)0xA5 );
	CHECK( bus.m_Memory[ 0x07F7 ] != (u8)( 1015 ^ 0xA5 ) );
	CHECK_EQ( scpu.writeBuffer().pending(), queuedMatrixBytes );

	// Once outside the picture, the same small installments prioritize and
	// finish the screen before ordinary border work. The dedicated allowance is
	// 256 bytes per opportunity, but the physical writes remain raster-checked
	// 64-byte bursts.
	bus.line = 251;						// first PAL line outside the picture
	bus.resetStats();
	bus.maxBurst = 0;
	scpu.runFrame();
	CHECK_EQ( bus.m_BurstWrites, queuedMatrixBytes );
	CHECK_EQ( bus.maxBurst, 64u );
	CHECK_EQ( bus.m_Memory[ 0x0400 ], (u8)0xA5 );
	CHECK_EQ( bus.m_Memory[ 0x07E7 ], (u8)( 999 ^ 0xA5 ) );
	CHECK_EQ( bus.m_Memory[ 0x07F7 ], (u8)( 1015 ^ 0xA5 ) );
	CHECK( !scpu.writeBuffer().hasPendingInRange(
	       0x0400, SCPU_VIC_MATRIX_BYTES ) );
}

TEST( integration_vsp_matrix_diagnostic_detects_constant_physical_shift_once )
{
	struct FixedRasterBus : CHostBus
	{
		u16 line = 260;
		u16 rasterLine() override { return line; }
	} bus;

	CSuperCPU scpu;
	u8 basic[ C64_BASIC_SIZE ];
	u8 kernal[ C64_KERNAL_SIZE ];
	std::memset( basic, 0xEA, sizeof basic );
	std::memset( kernal, 0xEA, sizeof kernal );
	kernal[ 0 ] = 0x4C;
	kernal[ 1 ] = 0x00;
	kernal[ 2 ] = 0xE0;
	kernal[ 0x1FFC ] = 0x00;
	kernal[ 0x1FFD ] = 0xE0;
	scpu.setBasicROM( basic );
	scpu.setKernalROM( kernal );
	CHECK( scpu.init( &bus, SCPU_CORE_6502, 0 ) );
	scpu.writeBuffer().setOptMode( SCPU_OPT_NONE );
	// Preserve the deliberately shifted physical image while the observer arms.
	scpu.setMirrorHalted( true );

	scpu.memory().write8( 0xD016, 0x10 );
	scpu.memory().write8( 0xD018, 0xCE );
	scpu.memory().m_RAM[ 0x9F88 ] = 0x00;
	const u32 logical = scpu.memory().activeVICScreenBase();
	const u32 physical = scpu.memory().activeScreenBase();
	CHECK( logical < 0x10000 );
	CHECK( physical < 0x10000 );

	u8 expected[ SCPU_VIC_MATRIX_BYTES ];
	for ( u32 i = 0; i < SCPU_VIC_MATRIX_BYTES; i++ )
	{
		expected[ i ] = (u8)( i * 37u + ( i >> 3 ) + 0x51u );
		scpu.memory().m_RAM[ ( logical + i ) & 0xFFFF ] = expected[ i ];
	}
	for ( u32 i = 0; i < SCPU_VIC_MATRIX_BYTES; i++ )
		bus.m_Memory[ ( physical + i ) & 0xFFFF ] =
			i + 3 < SCPU_VIC_MATRIX_BYTES ? expected[ i + 3 ] : 0xFF;

	// Establish the first phase, then change it once. The gameplay gate must
	// remain latched while the phase dwells, and the host thresholds 2/4/6 must
	// produce three independent comparisons.
	scpu.runFrame();
	scpu.memory().m_RAM[ 0x9F88 ] = 0x20;
	for ( u32 frame = 0; frame < 6; frame++ )
	{
		scpu.memory().write8( 0xD016, (u8)( 0x10 + ( frame % 3 ) ) );
		scpu.runFrame();
	}

	SCPUVSPMatrixDiagnostic d = {};
	scpuVSPMatrixDiagnostic( d );
	CHECK_EQ( d.state, (u8)2 );
	CHECK_EQ( d.gameplaySeen, (u8)1 );
	CHECK_EQ( d.capturedCount, (u8)3 );
	for ( u32 slot = 0; slot < SCPU_VSP_MATRIX_CLEAN_CAPTURES; slot++ )
	{
		const SCPUVSPMatrixCapture &capture = d.capture[ slot ];
		CHECK_EQ( capture.state, (u8)2 );
		CHECK_EQ( capture.logicalBase, logical );
		CHECK_EQ( capture.physicalBase, physical );
		CHECK_EQ( capture.rasterBefore, (u16)260 );
		CHECK_EQ( capture.rasterAfter, (u16)260 );
		CHECK_EQ( capture.safeWindow, (u8)1 );
		CHECK_EQ( capture.pendingAtCapture, 0u );
		CHECK_EQ( capture.bestShift, (s32)3 );
		CHECK_EQ( capture.bestShiftMatches, SCPU_VIC_MATRIX_BYTES - 3u );
		CHECK_EQ( capture.bestShiftCompared, SCPU_VIC_MATRIX_BYTES - 3u );
		CHECK( capture.mismatchCount > 900 );
	}
	CHECK_EQ( d.capture[ SCPU_VSP_MATRIX_STALL_SLOT ].state, (u8)0 );

	// All clean slots are bounded: later frames must not issue another readback.
	const u64 reads = bus.m_Reads;
	scpu.memory().write8( 0xD016, 0x10 );
	scpu.runFrame();
	CHECK_EQ( bus.m_Reads, reads );
}

TEST( integration_vsp_matrix_diagnostic_waits_for_a_clean_matrix )
{
	struct FixedRasterBus : CHostBus
	{
		u16 rasterLine() override { return 260; }
	} bus;

	CSuperCPU scpu;
	u8 basic[ C64_BASIC_SIZE ];
	u8 kernal[ C64_KERNAL_SIZE ];
	std::memset( basic, 0xEA, sizeof basic );
	std::memset( kernal, 0xEA, sizeof kernal );
	kernal[ 0 ] = 0x4C;
	kernal[ 1 ] = 0x00;
	kernal[ 2 ] = 0xE0;
	kernal[ 0x1FFC ] = 0x00;
	kernal[ 0x1FFD ] = 0xE0;
	scpu.setBasicROM( basic );
	scpu.setKernalROM( kernal );
	CHECK( scpu.init( &bus, SCPU_CORE_6502, 0 ) );
	scpu.writeBuffer().setOptMode( SCPU_OPT_NONE );
	scpu.setMirrorHalted( true );
	bus.resetStats();
	scpu.memory().write8( 0xD016, 0x10 );
	scpu.memory().write8( 0xD018, 0xCE );
	scpu.memory().m_RAM[ 0x9F88 ] = 0x00;
	const u32 screen = scpu.memory().activeVICScreenBase();
	scpu.memory().write8( (u16)screen, 0xA5 );

	scpu.runFrame();
	scpu.memory().m_RAM[ 0x9F88 ] = 0x20;
	for ( u32 frame = 0; frame < 3; frame++ )
	{
		scpu.memory().write8( 0xD016, (u8)( 0x10 + ( frame & 1 ) ) );
		scpu.runFrame();
	}
	SCPUVSPMatrixDiagnostic d = {};
	scpuVSPMatrixDiagnostic( d );
	CHECK_EQ( d.state, (u8)1 );
	CHECK_EQ( d.capture[ 0 ].state, (u8)1 );
	CHECK( d.capture[ 0 ].pendingWaits > 0 );
	CHECK_EQ( bus.m_Reads, 0u );

	// The at-stall slot preserves rather than drains the dirty queue. It still
	// waits for a hidden raster position, then reports the outstanding count.
	CHECK( scpuCaptureVSPStallMatrixDiagnostic( &scpu ) );
	scpuVSPMatrixDiagnostic( d );
	const SCPUVSPMatrixCapture &stall =
		d.capture[ SCPU_VSP_MATRIX_STALL_SLOT ];
	CHECK_EQ( stall.state, (u8)2 );
	CHECK_EQ( stall.kind, (u8)1 );
	CHECK( stall.pendingAtCapture > 0 );
	CHECK_EQ( scpu.writeBuffer().pendingInRange(
	          screen, SCPU_VIC_MATRIX_BYTES ), stall.pendingAtCapture );
	CHECK_EQ( bus.m_Reads, (u64)SCPU_VIC_MATRIX_BYTES );
}

TEST( integration_bitmap_drain_is_hidden_and_prioritized )
{
	struct FixedRasterBus : CHostBus
	{
		u16 line = 100;
		u32 maxBurst = 0;
		u16 rasterLine() override { return line; }
		void writeBurst( const C64BusWrite *writes, u32 count ) override
		{
			if ( count > maxBurst ) maxBurst = count;
			CHostBus::writeBurst( writes, count );
		}
	} bus;

	CSuperCPU scpu;
	u8 basic[ C64_BASIC_SIZE ];
	u8 kernal[ C64_KERNAL_SIZE ];
	std::memset( basic, 0xEA, sizeof basic );
	std::memset( kernal, 0xEA, sizeof kernal );
	kernal[ 0 ] = 0x4C;
	kernal[ 1 ] = 0x00;
	kernal[ 2 ] = 0xE0;
	kernal[ 0x1FFC ] = 0x00;
	kernal[ 0x1FFD ] = 0xE0;
	scpu.setBasicROM( basic );
	scpu.setKernalROM( kernal );
	CHECK( scpu.init( &bus, SCPU_CORE_6502, 0 ) );
	scpu.writeBuffer().setOptMode( SCPU_OPT_NONE );
	scpu.setMirrorDisplayBudget( 256 );

	// Bank 2, bitmap at $A000. Queue bitmap bytes first and unrelated cold RAM
	// behind them to prove the visible drain skips the entire 8K display span.
	scpu.memory().write8( 0xDD00, 0xC5 );
	scpu.memory().write8( 0xD011, 0x3B );
	scpu.memory().write8( 0xD018, 0x18 );
	CHECK_EQ( scpu.memory().activeBitmapBase(), 0xA000u );
	for ( u32 i = 0; i < 8000; i++ )
		scpu.memory().write8( (u16)( 0xA000 + i ), (u8)( i ^ 0x5A ) );
	scpu.memory().write8( 0x2000, 0xA5 );

	scpu.runFrame();
	CHECK_EQ( bus.m_Memory[ 0x2000 ], 0xA5 );
	CHECK( bus.m_Memory[ 0xA000 ] != (u8)0x5A );
	CHECK( scpu.writeBuffer().hasPendingInRange( 0xA000, 8000 ) );

	// In the hidden window, the bitmap is selected ahead of cold FIFO traffic
	// and completed through individually raster-checked 64-byte bursts.
	bus.line = 251;
	bus.resetStats();
	bus.maxBurst = 0;
	scpu.runFrame();
	CHECK_EQ( bus.m_BurstWrites, 8000u );
	CHECK_EQ( bus.maxBurst, 64u );
	CHECK_EQ( bus.m_Memory[ 0xA000 ], (u8)0x5A );
	CHECK_EQ( bus.m_Memory[ 0xBF3F ], (u8)( 7999 ^ 0x5A ) );
	CHECK( !scpu.writeBuffer().hasPendingInRange( 0xA000, 8000 ) );
}

TEST( integration_display_drain_avoids_sprite_fetch_lines )
{
	// Mid-display delivery must step around enabled sprites' fetch spans:
	// their BA windows are narrow and abrupt -- unlike a badline's long low --
	// and a burst write straddling one is how 3D Pool's static balls got hit
	// by its own re-render traffic. Lines clear of sprites keep delivering.
	struct FixedRasterBus : CHostBus
	{
		u16 line = 100;
		u16 rasterLine() override { return line; }
	} bus;

	CSuperCPU scpu;
	u8 basic[ C64_BASIC_SIZE ];
	u8 kernal[ C64_KERNAL_SIZE ];
	std::memset( basic, 0xEA, sizeof basic );
	std::memset( kernal, 0xEA, sizeof kernal );
	kernal[ 0 ] = 0x4C;              // JMP $E000
	kernal[ 1 ] = 0x00;
	kernal[ 2 ] = 0xE0;
	kernal[ 0x1FFC ] = 0x00;
	kernal[ 0x1FFD ] = 0xE0;
	scpu.setBasicROM( basic );
	scpu.setKernalROM( kernal );
	CHECK( scpu.init( &bus, SCPU_CORE_6502, 0 ) );
	scpu.writeBuffer().setOptMode( SCPU_OPT_NONE );
	scpu.setMirrorDisplayBudget( 256 );

	// Sprite 3 enabled with its span covering raster line 100.
	scpu.memory().write8( 0xD015, 0x08 );
	scpu.memory().write8( 0xD007, 95 );		// sprite 3 Y

	for ( u32 i = 0; i < 600; i++ )
		scpu.memory().write8( (u16)( 0x2000 + i ), (u8)( i ^ 0xA5 ) );
	const u32 staged = scpu.writeBuffer().pending();
	CHECK( staged >= 600 );
	bus.resetStats();

	// Beam parked inside the sprite's fetch span: nothing may burst.
	scpu.runFrame();
	CHECK_EQ( bus.m_BurstWrites, 0 );

	// Beam on a clear line: delivery resumes.
	bus.line = 180;
	scpu.runFrame();
	CHECK( bus.m_BurstWrites > 0 );
}
