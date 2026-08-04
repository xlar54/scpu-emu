/*
   SCPU-EMU - CMD SuperCPU emulation for the C64/C128 using a RAD Expansion Unit
   Copyright (c) 2026 SCPU-EMU contributors

   Bank 0: the C64-visible 64K address space, served from the Raspberry Pi.

   This is the piece that makes acceleration possible at all. The Pi keeps its
   own copy of the C64's DRAM and ROMs, so instruction fetches and data reads
   cost nothing on the expansion port. Only two classes of access have to leave
   the Pi:

     * reads and writes in the I/O window ($D000-$DFFF when banked in) -- these
       are VIC-II, SID, CIA and colour RAM, and must hit the real chips;
     * writes to DRAM, which have to be mirrored back into the C64 so the
       VIC-II fetches the right bytes. Mirroring is handled asynchronously via
       an IMirrorSink so the write buffer can batch it into bursts.

   The real SuperCPU works the same way, which is why its "optimization modes"
   are framed as choices about which writes get mirrored.

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
#ifndef _c64_memory_h
#define _c64_memory_h

#include "../Common/types.h"
#include "../CPU/cpu_bus.h"
#include "c64_bus.h"
#include "banking.h"

// Sink for writes that must eventually reach real C64 DRAM. Implemented by
// SuperCPU/write_buffer, which applies the optimization-mode policy and
// batches accepted writes into bus bursts.
class IMirrorSink
{
public:
	virtual ~IMirrorSink() {}
	virtual void onRamWrite( u16 addr, u8 value ) = 0;
	virtual void flush() = 0;
};

// Lets an accelerator claim addresses inside the I/O window before they are
// put on the bus. The SuperCPU decodes several ranges there ($D070-$D07F,
// $D0B0-$D0BF, $D200-$D3FF) which exist only inside the cartridge and must
// never reach the C64. Implemented by SuperCPU/registers.
class IIOInterceptor
{
public:
	virtual ~IIOInterceptor() {}
	// Return true if the access was handled and must not go to the bus.
	virtual bool ioRead( u16 addr, u8 &value ) = 0;
	virtual bool ioWrite( u16 addr, u8 value ) = 0;
};

#define C64_RAM_SIZE      0x10000
#define C64_BASIC_SIZE    0x2000
#define C64_KERNAL_SIZE   0x2000
#define C64_CHARROM_SIZE  0x1000

// 'final' is load-bearing, not decoration: it lets the compiler devirtualise
// and inline read8/write8 at the call site in CSuperCPUMemoryMap. Those are
// the hottest calls in the emulator -- several per emulated instruction -- and
// an indirect call there costs real megahertz on a Cortex-A53.
class CC64Memory final : public ICpuBus
{
public:
	CC64Memory();

	void attachBus( IC64Bus *bus ) { m_C64 = bus; }
	void setMirrorSink( IMirrorSink *sink ) { m_Mirror = sink; }
	void setIOInterceptor( IIOInterceptor *io ) { m_IO = io; }

	// Reset the emulated 6510 port to its power-on state ($00 = $2F, $01 = $37)
	// and clear shadow DRAM. Does not touch the ROM images.
	void reset();

	// --- bootmap ----------------------------------------------------------
	// A real SuperCPU comes out of reset with its OWN ROM mapped over most of
	// bank 0, so CMD's boot code runs before the C64's KERNAL -- that is where
	// the SuperCPU banner comes from. Writing $D0B6 maps it out again and
	// execution continues into the machine's own KERNAL.
	//
	// Verified against VICE's scpu64meminit.c, at the reset configuration
	// (mem_config $87): $8000-$CFFF and $E000-$FFFF come from the EPROM,
	// $0000-$7FFF stays RAM, and $D000-$DFFF stays I/O. The EPROM is indexed
	// 1:1 by the 16-bit address, so $FFFC reads image offset $FFFC.
	//
	// 'rom' must point at at least 64K and stay alive for the life of the
	// object -- it is not copied.
	void setBootmapROM( const u8 *rom ) { m_BootmapROM = rom; }
	bool hasBootmapROM() const          { return m_BootmapROM != 0; }

	// --- ROM shadow -------------------------------------------------------
	// On a SuperCPU there is NO C64 BASIC or KERNAL ROM in the map. Those
	// windows are served from the accelerator's own SRAM -- VICE's
	// scpu64meminit.c gives $A000-$BFFF as R1 (mem_sram[0x1A000]) and
	// $E000-$FFFF as KT (mem_sram[0x1E000]), i.e. bank 1 at the same offsets.
	//
	// That is not a detail: it is how the accelerator works. The SuperCPU's
	// EPROM carries its own copy of BASIC (byte-identical to Commodore's) and a
	// PATCHED KERNAL -- JiffyDOS serial routines, and the banner that reads
	// "SUPERCPU DOS 1.40 (C)1996 CMD" instead of "**** COMMODORE 64 BASIC V2
	// ****". The boot code copies both into bank 1 and then maps itself out, so
	// everything afterwards runs the patched copy, out of fast SRAM.
	//
	// Serving these windows from a read-only snapshot instead silently discards
	// those patches, which is exactly what produced a stock C64 banner on a
	// machine that had just run CMD's boot code.
	//
	// 'bank1' is the 64K of accelerator SRAM, not copied. Null falls back to the
	// snapshotted ROMs, which is what a machine with no accelerator ROM wants.
	void setROMShadow( u8 *bank1 ) { m_ROMShadow = bank1; }

	// WHICH part of bank 1 backs $E000-$FFFF depends on whether the hardware
	// registers are open. VICE calls the two cases KT and KS:
	//
	//   hwregs off  KT  mem_sram[0x10000 + addr]  ->  bank 1 $E000-$FFFF
	//   hwregs on   KS  mem_sram[0x8000  + addr]  ->  bank 1 $6000-$7FFF
	//
	// This is not a curiosity. While the registers are open -- which is the
	// state the SuperCPU's boot code puts the machine in almost immediately --
	// bank 1 $E000-$FFFF is NOT the KERNAL and is free for the boot code to use
	// as scratch. Reading our KERNAL out of there anyway means watching it get
	// overwritten, which shows up as a machine that runs the boot animation
	// perfectly and then comes up to a blank screen.
	//
	// Held as a precomputed base so the read path is one add rather than a
	// branch: $E000 for KT, $6000 for KS.
	u32 m_KernalShadowBase;

	// The live flag. CSuperCPURegisters points at this directly so a $D0B6
	// write takes effect on the very next fetch, which is what the boot code
	// depends on.
	bool m_BootmapActive;

	// --- ROM images -------------------------------------------------------
	void setBasicROM  ( const u8 *data );	// 8K
	void setKernalROM ( const u8 *data );	// 8K
	void setCharROM   ( const u8 *data );	// 4K

	// Snapshot BASIC and KERNAL out of the live machine over the bus. Requires
	// that we already hold DMA and that the halted 6510 left $01 with LORAM and
	// HIRAM set -- true after a normal KERNAL start-up. Costs 16384 bus cycles,
	// about 16ms on a PAL machine.
	//
	// The character ROM cannot be captured this way: exposing it needs CHAREN
	// low, and with the 6510 off the bus nothing can rewrite its port. Supply
	// chargen from a file instead (see ROMs/README.md).
	bool snapshotROMsFromBus();

	bool hasBasicROM()  const { return m_HasBasic; }
	bool hasKernalROM() const { return m_HasKernal; }
	bool hasCharROM()   const { return m_HasChar; }

	// --- hot path ---------------------------------------------------------
	// Plain RAM in bank 0, inline, no call. This is the overwhelming majority
	// of every emulated access -- opcode fetches, operands, zero page, the
	// stack -- and on an in-order Cortex-A53 the call and the indirect branch
	// that used to wrap it cost more than the work itself.
	//
	// Everything with any complexity is left OUT of line: the $00/$01 port,
	// bootmap, the BASIC/KERNAL shadows, I/O and open bus. Those paths already
	// do enough work that a call is noise, and keeping them out of the header
	// keeps this small enough to actually inline.
	inline u8 readFast( u16 a )
	{
		if ( a > 1 && !m_BootmapActive && c64MapRead( a, m_BankMode ) == REG_RAM )
			return m_RAM[ a ];
		return read8( a );
	}

	inline void writeFast( u16 a, u8 v )
	{
		// Note the mirror sink still has to be told: the VIC-II only sees what
		// reaches the C64's own DRAM.
		if ( a > 1 && c64MapRead( a, m_BankMode ) != REG_IO )
		{
			m_RAM[ a ] = v;
			m_RamWrites++;
			if ( m_Mirror ) m_Mirror->onRamWrite( a, v );
			return;
		}
		write8( a, v );
	}

	// --- interrupts, cached -----------------------------------------------
	// /IRQ and /NMI come from the VIC-II and the CIAs, which live in the C64's
	// 1MHz domain: the lines physically cannot change faster than about once a
	// microsecond. Sampling them per emulated instruction at 20MHz was ~14
	// samples per possible change -- and on the RAD each sample was an uncached
	// GPIO MMIO read costing on the order of a hundred ARM cycles. Two of those
	// per instruction was one of the largest single costs in the emulator, and
	// it was invisible in the bus statistics, which only count I/O and
	// mirroring traffic.
	//
	// So: one combined sample (see IC64Bus::sampleInterrupts), refreshed once
	// enough emulated time has passed for the lines to have possibly changed.
	// m_PacingCheckCycles is exactly "emulated cycles per microsecond", so it
	// is reused here; at 1MHz that is every cycle, i.e. the old behaviour.
	inline void refreshInterruptsIfDue()
	{
		// Under automatic IEC throttling, a nominal 20MHz cycle lasts one
		// microsecond. Use the effective rate so the cache does not become 20us.
		const u32 threshold = m_IECHoldCycles ? 1 : m_PacingCheckCycles;
		if ( m_IntCredit >= threshold && m_C64 )
		{
			m_C64->sampleInterrupts( m_CachedIRQ, m_CachedNMI );
			m_IntCredit = 0;
		}
	}
	inline bool irqFast() { refreshInterruptsIfDue(); return m_CachedIRQ; }
	inline bool nmiFast() { refreshInterruptsIfDue(); return m_CachedNMI; }

	// --- pacing, inline fast path -----------------------------------------
	// Runs after every instruction, so the common case -- "not enough emulated
	// time has passed to be worth looking at the clock" -- must not cost a
	// call. The clock work itself is in tickSettle(), out of line.
	inline void tickFast( u32 nCycles )
	{
		if ( m_IntCredit < 0x100000 )
			m_IntCredit += nCycles;

		const bool iecActive = ( m_IECHoldCycles != 0 );
		if ( iecActive )
			m_IECThrottledCycles += nCycles;
		if ( iecActive || ( m_SelectedEmulatedHz != 0
		                   && m_SelectedEmulatedHz <= 1000000u ) )
			m_SlowPacedCycles += nCycles;
		if ( iecActive )
		{
			m_IECHoldCycles = ( m_IECHoldCycles > nCycles )
			                ? ( m_IECHoldCycles - nCycles ) : 0;
			if ( m_IECHoldCycles == 0 && m_TimingHook )
				m_TimingHook( m_TimingHookCtx );
		}

		if ( m_HostPerEmuQ16 == 0 || !m_C64 )
			return;

		m_PacingDebtCycles += nCycles;

		if ( !iecActive && m_PacingDebtCycles < (u64)m_PacingCheckCycles )
			return;

		tickSettle( iecActive );
	}
	void tickSettle( bool iecActive );

	// --- ICpuBus ----------------------------------------------------------
	u8   read8( scpu_addr_t addr ) override;
	void write8( scpu_addr_t addr, u8 value ) override;
	bool irqAsserted() override;
	bool nmiAsserted() override;
	void tick( u32 nCycles ) override;

	// --- state, public for tracing and tests ------------------------------
	u8  m_RAM[ C64_RAM_SIZE ];
	u8  m_Basic[ C64_BASIC_SIZE ];
	u8  m_Kernal[ C64_KERNAL_SIZE ];
	u8  m_CharROM[ C64_CHARROM_SIZE ];

	// Emulated 6510 on-chip I/O port.
	u8  m_Port00;		// data direction register
	u8  m_Port01;		// port latch

	// Cached c64BankMode() for the current port/GAME/EXROM state.
	u8  m_BankMode;

	u64 m_IOReads;		// statistics: accesses that actually hit the bus
	u64 m_IOWrites;
	u64 m_RamWrites;

	void updateBankMode();

	// --- pacing -----------------------------------------------------------
	// Hold the emulated CPU to real time, checked after every instruction.
	//
	// Pacing used to happen once per frame in CSuperCPU::runFrame(), which is
	// about 20ms of granularity: the CPU sprinted through a frame's work and
	// then idled. That is fine for anything that only cares about averages, and
	// useless for anything that measures time by counting cycles. The KERNAL's
	// IEC routines bit-bang the serial lines to microsecond tolerances, so
	// under frame pacing disk access cannot work.
	//
	// Setting a rate of 0 disables pacing, which is what the host tests want.
	void setPacing( u64 hostCyclesPerSecond, u32 emulatedHz );
	void resyncPacing();

	// --- automatic 1MHz for serial bus activity ---------------------------
	// The KERNAL's IEC routines count cycles to time the bits they bang out on
	// the serial lines, and those counts assume a 1MHz CPU. Run them at 20MHz
	// and every pulse is twenty times too short, so no drive ever answers.
	//
	// A real SuperCPU has exactly this problem and solves it the same way: CMD
	// document that it "always throttles to 1MHz for disk access regardless of
	// the speed setting". Writing to CIA2 port A -- which carries ATN, CLK and
	// DATA -- arms a hold-off, during which pacing runs at 1MHz whatever speed
	// the accelerator is nominally set to.
	void setIECThrottle( bool enabled ) { m_IECThrottleEnabled = enabled; }
	bool iecThrottleActive() const { return m_IECHoldCycles > 0; }
	typedef void (*TimingHook)( void *ctx );
	void setTimingHook( TimingHook hook, void *ctx ) { m_TimingHook = hook; m_TimingHookCtx = ctx; }
	u64  m_IECThrottleEvents;

	u64 m_PacingDebtCycles;		// emulated cycles run but not yet paid for in real time
	u32 m_PacingCheckCycles;	// emulated cycles between consulting the host clock
	u64 m_PacerWaitHostCycles;	// host cycles deliberately spent waiting
	u64 m_SlowPacedCycles;		// cycles selected or forced to the 1MHz rate
	u64 m_IECThrottledCycles;	// subset forced slow by the IEC heuristic
public:
	// Interrupt cache; public so the inline accessors above can live in the
	// class body without friend gymnastics.
	u32  m_IntCredit = 0xFFFFF;	// starts "due" so the first query samples
	bool m_CachedIRQ = false;
	bool m_CachedNMI = false;
private:

private:
	IC64Bus        *m_C64;
	IMirrorSink    *m_Mirror;
	IIOInterceptor *m_IO;
	const u8       *m_BootmapROM;
	u8             *m_ROMShadow;
	bool            m_HasBasic, m_HasKernal, m_HasChar;

	// Pacing state. m_HostPerEmuQ16 is host cycles per emulated cycle in 16.16
	// fixed point, so the common case is a multiply and a shift rather than a
	// division per instruction.
	u64 m_HostPerEmuQ16;		// host cycles per emulated cycle, 16.16
	u64 m_HostPerEmuQ16Slow;	// the same at 1MHz, for IEC hold-off
	u64 m_PacingAnchor;
	u32 m_SelectedEmulatedHz;

	bool m_IECThrottleEnabled;
	u32  m_IECHoldCycles;		// emulated cycles left at forced 1MHz
	u8   m_LastCIA2PortA;
	bool m_HaveCIA2PortA;
	u8   m_LastVICControl[ 3 ];		// last written $D011 / $D016 / $D018
public:
	// The last 16 I/O accesses, newest last: addr | value<<16 | (write?1:0)<<24.
	// Costs a store on the I/O path only -- never on the RAM hot path -- and
	// exists so a frozen machine can say what it last did to the hardware. A
	// freeze is almost always a wait for an I/O event that never came, and the
	// last few accesses name the conversation that died.
	u32 m_IOLog[ 64 ];
	u8  m_IOLogPos;		// wraps at 64
private:
	u8   m_HaveVICControl;			// bitmask of which of those have a baseline
	TimingHook m_TimingHook;
	void      *m_TimingHookCtx;
};

#endif
