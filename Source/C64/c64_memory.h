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
	// Called BEFORE the shadow byte is stored, so the sink can read the OLD
	// value through the RAM pointer it was attached with and drop writes that
	// change nothing.
	// Returns true when the write was accepted by the active mirroring policy.
	// The SuperCPU uses this to model its one-deep posted write buffer; skipped
	// and same-value-eliminated writes do not occupy that hardware buffer.
	virtual bool onRamWrite( u16 addr, u8 value ) = 0;
	virtual void flush() = 0;
	virtual u32  pendingBytes() { return 0; }

	// Drain at most maxBytes, oldest first; returns bytes still pending. The
	// default drains everything -- only budgeted implementations can promise
	// raster safety.
	virtual u32 flushUpTo( u32 maxBytes ) { (void)maxBytes; flush(); return 0; }
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
	// Timing class for an intercepted access. v2 private SRAM is fast on reads
	// and uses the posted write buffer on accepted writes; hardware-register
	// writes still perform the ordinary SuperCPU I/O stretch.
	virtual bool ioAccessNeedsStretch( u16 addr, bool write ) const
		{ (void)addr; return write; }
	virtual bool ioAccessUsesWriteBuffer( u16 addr, bool write ) const
		{ (void)addr; (void)write; return false; }
	virtual bool dosExtensionEnabled() const { return false; }
	virtual const bool *dosExtensionStatePtr() const { return 0; }
	virtual bool simmWindowWritesEnabled() const { return true; }

	// True when accelerator state wants interrupt vectors fetched from the
	// accelerator's own ROM; see CC64Memory::interruptRerouteActive.
	virtual bool interruptRerouteRequested() const { return false; }
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
	void enablePostedWriteTiming( bool on )
	{
		m_PostedWriteTimingEnable = on;
		if ( !on )
		{
			m_PostedWritePending = false;
			m_MirrorBufferFreeAt = 0;
		}
	}
	void setIOInterceptor( IIOInterceptor *io )
	{
		m_IO = io;
		m_DOSExtState = io ? io->dosExtensionStatePtr() : 0;
	}

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
	void setBootmapROM( const u8 *rom, u32 length = 0x10000 )
	{
		m_BootmapROM = rom;
		m_BootmapROMLength = rom ? length : 0;
	}
	bool hasBootmapROM() const          { return m_BootmapROM != 0; }
	void setSCPU128Mode( bool on )      { m_SCPU128Mode = on; }

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

	// Establish the C64 KERNAL's CIA2 direction state before guest execution.
	// The dedicated double write handles the known first-write loss on buffered
	// C128 expansion ports; the shadow becomes authoritative only after the bus
	// backend verifies the value. Called only for the C64 personality while
	// /DMA still holds the physical host CPU off the bus.
	bool initializeC64CIA2();

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
	__attribute__((always_inline)) inline u8 readFast( u16 a )
	{
		if ( dosExtensionMapsBank1( a ) )
			return m_ROMShadow[ a ];
		if ( a > 1 && !m_BootmapActive && c64MapRead( a, m_BankMode ) == REG_RAM )
			return m_RAM[ a ];
		return read8( a );
	}

	__attribute__((always_inline)) inline void writeFast( u16 a, u8 v )
	{
		if ( dosExtensionMapsBank1( a ) )
		{
			m_ROMShadow[ a ] = v;
			return;
		}
		// Note the mirror sink still has to be told: the VIC-II only sees what
		// reaches the C64's own DRAM.
		if ( a > 1 && c64MapRead( a, m_BankMode ) != REG_IO )
		{
			// Sink first, store second: the sink reads the old byte through
			// its shadow pointer and drops writes that change nothing.
			const u8 old = m_RAM[ a ];
			m_RamWrites++;
			if ( m_Mirror && m_Mirror->onRamWrite( a, v ) )
				noteMirrorWrite();
			m_RAM[ a ] = v;

			// The active screen's sprite pointers bypass the queue and land
			// on the real bus NOW, like a VIC register. Double-buffered
			// sprite animation flips a pointer in vblank and immediately
			// starts redrawing the block the flip retired; a pointer flip
			// that arrives even a fraction of a frame late leaves the
			// physical VIC displaying the very block the game is busy
			// overwriting -- seen as sprites flickering between correct and
			// garbage at animation rate. The queued copy still flushes
			// later; coalescing holds the latest value, so re-sending it is
			// harmless. Same-value rewrites are a no-op on real hardware and
			// are skipped.
			if ( ( (u32)a & ~7u ) == m_SpritePtrBase )
			{
				m_PtrRowWrites++;
				if ( old != v )
				{
					// The VALUE delivered may differ from the value stored:
					// a bank-3 pointer selecting a shape block under the I/O
					// window is remapped to the block's relocated copy, the
					// only place the real VIC can actually be fed from. See
					// relocPointerValue().
					if ( m_C64 ) m_C64->write( a, relocPointerValue( v ) );
					updateHotShapeBlocks();
				}
			}
			return;
		}
		write8( a, v );
	}

	// --- interrupt vector reroute -----------------------------------------
	// A real SuperCPU does not fetch interrupt vectors from the C64 map at
	// all when it is running as an accelerator: it redirects the fetch into
	// its OWN EPROM at $F80000+vector, which carries a genuine 65816 vector
	// table pointing at trampolines ($FCA4 = JML $00C003 for NMI, $FC94 =
	// JML $00FBE1 for IRQ/BRK, and so on). That is why the KERNAL images can
	// get away with holding the C64 jump table across $FFE4-$FFEF, and why a
	// 65816 program may put its own code there: nothing ever reads those
	// bytes as vectors.
	//
	// VICE implements it as scpu64_interrupt_reroute() (scpu64mem.c) hooked
	// into LOAD_INT_ADDR (scpu64cpu.c). Both halves of its condition are
	// reproduced here: page $FF must be served by the accelerator's own
	// KERNAL window rather than C64 DRAM, AND the processor must be in
	// native mode or the accelerator must have its registers open / be in
	// system 1MHz / DOS-extension / RAMLink state.
	//
	// Ordinary C64 operation satisfies neither half of the second clause in
	// emulation mode, so the stock KERNAL's own vectors are used exactly as
	// before; the reroute engages only where a real card would take over.
	bool m_VectorRerouteEnable = true;		// config VECTOR_REROUTE
	u64  m_VectorReroutes = 0;				// statistics

	// --- C64-bus access cost ----------------------------------------------
	// Accesses are queued in program order and charged at the end of the
	// current instruction. Keeping every event matters: the old two-Boolean
	// model collapsed an RMW's read and two writes, or a whole eight-
	// instruction batch, into one access. fineTicksRequired() breaks a batch
	// as soon as an event appears, so only one instruction's events accumulate.
	bool m_IOStretchEnable = true;			// config IO_STRETCH
	bool m_MirrorStretchEnable = false;		// config MIRROR_STRETCH
	bool m_PostedWriteTimingEnable = false;	// VIDEO_MODE 1, no physical mirror
	enum BusTimingEvent : u8
	{
		BUS_EVENT_IO = 1,
		BUS_EVENT_CIA_WRITE,
		BUS_EVENT_MIRROR_WRITE
	};
	static const u32 BUS_EVENT_CAPACITY = 32;
	u8   m_BusEvents[ BUS_EVENT_CAPACITY ];
	u8   m_BusEventCount = 0;
	bool m_PostedWritePending = false;
	u64  m_MirrorBufferFreeAt = 0;
	u64  m_IOStretchCycles = 0;				// statistics
	u64  m_MirrorStretchCycles = 0;
	u32  m_PendingFastHalfCycles = 0;
	u8   m_FastHalfCarry = 0;
	u64  m_RegionStretchCycles = 0;

	inline void noteTimingEvent( BusTimingEvent event )
	{
		if ( m_BusEventCount < BUS_EVENT_CAPACITY )
			m_BusEvents[ m_BusEventCount++ ] = (u8)event;
	}
	inline void noteBusAccess( u16 addr, bool write )
	{
		// VICE's additional CIA cycle applies only to CIA writes. Reads and the
		// expansion-port windows at $DE00/$DF00 use the ordinary I/O path.
		const bool ciaWrite = write && ( ( addr & 0xFE00 ) == 0xDC00 );
		noteTimingEvent( ciaWrite ? BUS_EVENT_CIA_WRITE : BUS_EVENT_IO );
	}
	inline void noteMirrorWrite()
	{
		// Mode 0 pays the physical transfer in wall time, so MIRROR_STRETCH
		// remains an opt-in compatibility switch there. HDMI-exclusive mode has
		// no physical transfer at all and must account for the real card's one
		// posted slot explicitly, independent of that switch.
		if ( m_MirrorStretchEnable || m_PostedWriteTimingEnable )
		{
			noteTimingEvent( BUS_EVENT_MIRROR_WRITE );
			if ( m_PostedWriteTimingEnable ) m_PostedWritePending = true;
		}
	}
	inline void noteFastHalfCycles( u32 halfCycles )
	{
		m_PendingFastHalfCycles += halfCycles;
	}
	inline bool dosExtensionMapsBank1( u16 addr ) const
	{
		// This helper is on every bank-0 read/write fast path. The register state
		// is cached when it changes; a virtual interceptor query here made code in
		// these common ranges 50% slower even when DOS extension was off.
		return ( ( addr >= 0x1000 && addr <= 0x5FFF )
		      || ( addr >= 0x8000 && addr <= 0x9FFF ) )
		    && !m_BootmapActive && m_ROMShadow
		    && m_DOSExtState && *m_DOSExtState;
	}
	bool simmWindowWritesEnabled() const
		{ return !m_IO || m_IO->simmWindowWritesEnabled(); }

	bool interruptRerouteActive( bool emulationMode ) const
	{
		if ( !m_VectorRerouteEnable )
			return false;
		// Bootmap already maps the EPROM over $E000-$FFFF, so the ordinary
		// fetch reads the very bytes the reroute would.
		if ( m_BootmapActive || !m_ROMShadow )
			return false;
		if ( c64MapRead( 0xFF00, m_BankMode ) != REG_KERNAL )
			return false;
		return !emulationMode
		    || ( m_IO && m_IO->interruptRerouteRequested() );
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
		// CIA2 timer NMIs are delivered on the EMULATED clock, not the sampled
		// line. Checked before the credit gate: the deadline is in emulated
		// cycles and must not wait out the sampling quantum. Costs one compare
		// against an almost-always-zero u64.
		if ( m_CIA2NMIDeadline && ciaNow() >= m_CIA2NMIDeadline )
			ciaTimerNMIDue();

		// Under automatic IEC throttling, a nominal 20MHz cycle lasts one
		// microsecond. Use the effective rate so the cache does not become 20us.
		const u32 threshold = ( m_IECHoldCycles || m_IECPollHoldCycles )
		                    ? 1 : m_PacingCheckCycles;
		if ( m_IntCredit >= threshold && m_C64 )
		{
			bool irq, nmi;
			m_C64->sampleInterrupts( irq, nmi );
			m_CachedIRQ = irq;

			// While a CIA2 timer NMI is armed, the TIMERS own the NMI line:
			// delivery is synthetic, at the deadline the emulated clock
			// computes from the timer writes, and the sampled line -- which
			// carries the same interrupts, only µs-jittered -- is ignored
			// entirely. Programs arm these timers to hit an exact spot in
			// their instruction stream (Winter Games /SCPU fires one every
			// transfer chunk and its only protection for a native-mode window
			// is that the NMI always lands outside it); sampling slack broke
			// that contract. When no timer NMI is armed the line passes
			// through as always: RESTORE, FLAG and friends are unaffected.
			if ( m_CIA2NMIOwned )
				m_CachedNMI = m_SynthNMIActive;
			else
				m_CachedNMI = nmi || m_SynthNMIActive;
			m_IntCredit = 0;
		}
	}
	// Out of line: a timer deadline arrived. Exposes the NMI and re-arms
	// (continuous) or disarms (one-shot) the model.
	void ciaTimerNMIDue();
	inline bool irqFast() { refreshInterruptsIfDue(); return m_CachedIRQ; }
	inline bool nmiFast() { refreshInterruptsIfDue(); return m_CachedNMI; }

	// The core samples NMI and IRQ back to back on every instruction; this
	// combined form pays the refresh check once instead of twice, at several
	// million calls a second. The single accessors above stay fresh for
	// everyone else.
	inline void sampleIRQNMIFast( bool &irq, bool &nmi )
	{
		refreshInterruptsIfDue();
		irq = m_CachedIRQ;
		nmi = m_CachedNMI;
	}

	// --- pacing, inline fast path -----------------------------------------
	// Runs after every instruction, so the common case -- "not enough emulated
	// time has passed to be worth looking at the clock" -- must not cost a
	// call. The clock work itself is in tickSettle(), out of line.
	inline void tickFast( u32 nCycles )
	{
		m_EmuCycles += nCycles;
		if ( m_PendingFastHalfCycles )
		{
			if ( m_IOStretchEnable && effectiveCIAFactor() > 1 )
			{
				const u32 halves = m_PendingFastHalfCycles + m_FastHalfCarry;
				const u32 extra = halves >> 1;
				m_FastHalfCarry = (u8)( halves & 1 );
				m_EmuCycles += extra;
				nCycles += extra;
				m_RegionStretchCycles += extra;
			}
			m_PendingFastHalfCycles = 0;
		}

		// A C64-bus access does not cost the accelerator the handful of cycles
		// its opcode takes: it costs a WHOLE C64 cycle, because the SuperCPU
		// stalls until PHI2, performs the access at 1MHz, and only then runs
		// on. VICE charges this explicitly (scpu64cpu.c
		// scpu64_clock_read_stretch_io / _write_stretch_io_*, one
		// scpu64_maincpu_inc() per access and a second for CIA accesses, then
		// a phase relock). We charged nothing at all, and that is the deepest
		// timing error in this emulator: it makes I/O-bound code run several
		// times too fast in emulated time.
		//
		// The visible consequence is quantisation. A poll loop
		//   LDA $D012 / CMP #x / BNE
		// takes exactly ONE C64 cycle per iteration on a real SuperCPU -- the
		// nine processor cycles vanish inside the stall -- so 1000 iterations
		// are 1000 microseconds. Charging only the opcode cycles made the same
		// loop nine emulated cycles, and a CIA timer armed for 21us then fired
		// at a completely different instruction. That is what put Winter Games'
		// throwaway NMI inside a native-mode window it was written to survive.
		//
		if ( m_BusEventCount )
		{
			// EFFECTIVE emulated cycles per C64 cycle, not the nominal
			// selection: at 1MHz -- explicitly chosen, or forced by the IEC
			// throttle -- the processor already runs at the bus's own rate and
			// there is nothing to stall for, so no stretch applies and the
			// serial paths keep their per-instruction tick granularity.
			const u64 perC64 = effectiveCIAFactor();
			u64 ioExtra = 0;
			u64 mirrorExtra = 0;
			if ( perC64 > 1 )
			{
				for ( u32 i = 0; i < m_BusEventCount; i++ )
				{
					const BusTimingEvent event = (BusTimingEvent)m_BusEvents[ i ];
					if ( event == BUS_EVENT_MIRROR_WRITE )
					{
						if ( !m_MirrorStretchEnable
						     && !m_PostedWriteTimingEnable )
						{
							m_MirrorBufferFreeAt = 0;
							continue;
						}
						// The real card has one posted mirrored-write slot. A write
						// can retire in the background, but the next accepted write
						// must wait if that slot is still occupied.
						if ( m_EmuCycles < m_MirrorBufferFreeAt )
						{
							const u64 wait = m_MirrorBufferFreeAt - m_EmuCycles;
							m_EmuCycles += wait;
							mirrorExtra += wait;
						}
						const u64 phase = m_EmuCycles % perC64;
						m_MirrorBufferFreeAt = m_EmuCycles
						                     + ( phase ? perC64 - phase : perC64 );
						continue;
					}

					// Any real I/O access first drains the posted write buffer.
					// With I/O stretch disabled the physical access itself already
					// consumed the wall-clock interval, so simply mark the slot free;
					// do not bill the guest for that interval a second time.
					if ( !m_IOStretchEnable )
					{
						m_MirrorBufferFreeAt = 0;
						continue;
					}
					if ( m_EmuCycles < m_MirrorBufferFreeAt )
					{
						const u64 wait = m_MirrorBufferFreeAt - m_EmuCycles;
						m_EmuCycles += wait;
						mirrorExtra += wait;
					}

					// Complete at the next PHI2 boundary. If an earlier event in
					// this instruction already put us exactly on a boundary, this
					// event consumes the following whole C64 cycle instead of
					// collapsing into the same one.
					const u64 phase = m_EmuCycles % perC64;
					const u64 wait = phase ? perC64 - phase : perC64;
					m_EmuCycles += wait;
					ioExtra += wait;
					if ( event == BUS_EVENT_CIA_WRITE )
					{
						m_EmuCycles += perC64;
						ioExtra += perC64;
					}
				}
			}
			m_BusEventCount = 0;
			m_PostedWritePending = false;
			const u64 extra = ioExtra + mirrorExtra;
			if ( extra )
			{
				m_IOStretchCycles += ioExtra;
				m_MirrorStretchCycles += mirrorExtra;
				nCycles += (u32)extra;
			}
		}

		if ( nCycles > m_MaxTickChunk )
			m_MaxTickChunk = nCycles;
		if ( m_IntCredit < 0x100000 )
			m_IntCredit += nCycles;

		// A timer armed during the instruction just ticked gets its deadline
		// anchored NOW, with that instruction's cycles included: this is the
		// moment the byte would have reached the real chip.
		if ( m_CIA2ArmPending )
			cia2FinishArm();

		// CIA2 timer NMI deadlines are checked HERE, not only in the
		// interrupt-cache refresh: the run loop samples interrupts once per
		// slice, and a 20-cycle deadline armed mid-slice would otherwise
		// deliver milliseconds late. An armed deadline also forces fine
		// ticks (see fineTicksRequired), so this executes per instruction
		// while it matters and is one compare against zero otherwise.
		if ( m_CIA2NMIDeadline && ciaNow() >= m_CIA2NMIDeadline )
			ciaTimerNMIDue();

		// The serial-activity window counts down here. Without this decrement
		// the flag armed at CMD's boot-time drive probe and never expired:
		// mirror flushing stayed suppressed forever, the splash blanked the
		// real screen and drew into shadow that never went out -- a machine
		// running perfectly behind a black display.
		if ( m_IECActivityCycles )
			m_IECActivityCycles = ( m_IECActivityCycles > nCycles )
			                    ? ( m_IECActivityCycles - nCycles ) : 0;

		// The poll-sustained SPEED hold decays the same way; its expiry is a
		// rate transition for armed CIA2 deadlines.
		if ( m_IECPollHoldCycles )
		{
			m_IECPollHoldCycles = ( m_IECPollHoldCycles > nCycles )
			                    ? ( m_IECPollHoldCycles - nCycles ) : 0;
			if ( m_IECPollHoldCycles == 0 && m_IECHoldCycles == 0 )
			{
				if ( m_CIA2NMIDeadline )
					cia2OnRateChange();
				if ( m_TimingHook )
					m_TimingHook( m_TimingHookCtx );
			}
		}

		const bool iecActive = ( m_IECHoldCycles != 0 )
		                    || ( m_IECPollHoldCycles != 0 );
		if ( iecActive )
			m_IECThrottledCycles += nCycles;
		if ( iecActive || ( m_SelectedEmulatedHz != 0
		                   && m_SelectedEmulatedHz <= 1000000u ) )
			m_SlowPacedCycles += nCycles;
		if ( iecActive )
		{
			m_IECHoldCycles = ( m_IECHoldCycles > nCycles )
			                ? ( m_IECHoldCycles - nCycles ) : 0;
			if ( m_IECHoldCycles == 0 )
			{
				// Hold expiry is an effective-speed transition; armed CIA2
				// deadlines re-denominate (cold: once per expiry).
				if ( m_CIA2NMIDeadline )
					cia2OnRateChange();
				if ( m_TimingHook )
					m_TimingHook( m_TimingHookCtx );
			}
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
	// Replace the background bus activity lost when HDMI becomes the exclusive
	// display and physical DRAM mirroring is detached. This is intentionally a
	// separate policy switch: VIDEO_MODE 0 must never pay for it.
	void enableBusKeepalive( bool on ) { m_BusKeepalive = on; }

	// --- automatic 1MHz for serial bus activity ---------------------------
	// The KERNAL's IEC routines count cycles to time the bits they bang out on
	// the serial lines, and those counts assume a 1MHz CPU. Run them at 20MHz
	// and every pulse is twenty times too short, so no drive ever answers.
	//
	// A real SuperCPU has exactly this problem and solves it the same way: CMD
	// document that it "always throttles to 1MHz for disk access regardless of
	// the speed setting". Changes to CIA2's IEC outputs, receive-side activity,
	// and DDRA transitions arm a hold-off during which pacing runs at 1MHz
	// whatever speed the accelerator is nominally set to.
	void setIECThrottle( bool enabled ) { m_IECThrottleEnabled = enabled; }
	bool iecThrottleActive() const
	{
		return m_IECHoldCycles > 0 || m_IECPollHoldCycles > 0;
	}

	// The 65816 normally batches several instructions before ticking the C64
	// bus. That is safe at turbo speed, but not at an effective/selected 1MHz:
	// IEC code depends on the real-time spacing between individual
	// instructions even when CMD selected slow mode itself (and therefore the
	// automatic throttle timer is not armed).
	bool fineTicksRequired() const
	{
		return m_PostedWritePending
		    || m_IECHoldCycles != 0 || m_IECActivityCycles != 0
		    || m_IECPollHoldCycles != 0
		    || m_CIA2NMIDeadline != 0 || m_CIA2ArmPending != 0
		    || ( m_SelectedEmulatedHz != 0
		         && m_SelectedEmulatedHz <= 1000000u );
	}
	// True while a serial TRANSACTION is in progress, independent of what
	// speed is selected. This is the flag that suppresses mirror flushing and
	// raster polling: the slow protocol assigns meaning to pauses, so the CPU
	// must not be stalled mid-transaction whatever the pacing mode.
	bool iecBusActive() const { return m_IECActivityCycles != 0; }
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

	// --- CIA2 timer NMI retiming ------------------------------------------
	// The physical NMI line is sampled with up to a microsecond of slack and
	// delivered at instruction-batch boundaries -- jitter that is invisible
	// to almost everything and fatal to code that arms a short CIA2 timer
	// NMI to land at an exact spot in its instruction stream. We see every
	// timer register write on its way to the real chip, so the underflow
	// moment is computable on the emulated clock exactly. While such a timer
	// NMI is armed (and no other NMI source shares the mask), delivery is
	// synthetic-at-deadline and the sampled line is ignored; the moment the
	// mask clears, the line passes through again untouched.
	bool m_NMIRetimeEnable = true;		// config NMI_RETIME
	bool m_CIA2NMIOwned = false;		// timers own the line: synth-only delivery
	bool m_SynthNMIActive = false;		// synthetic level; falls on ICR read/mask clear
	u8   m_CIA2ICRMask = 0;				// CIA2 interrupt enable mask, bits 0/1 = timers
	// Underflows latch here REGARDLESS of the mask, like the chip's ICR: a
	// timer that expires masked still delivers the instant the mask is
	// enabled. Cleared by the ICR read, like the chip.
	u8   m_CIA2ICRPending = 0;
	u8   m_CIA2TACR = 0, m_CIA2TBCR = 0;
	// A START write records the arm here instead of computing the deadline on
	// the spot. The write is observed from inside write8(), where m_EmuCycles
	// has NOT yet been advanced by the storing instruction -- on real hardware
	// the byte reaches the chip on that instruction's LAST cycle, so anchoring
	// there fires every timer ~4 cycles early. Four cycles is nothing against
	// a frame, and everything against Winter Games' ~20-cycle landing window:
	// early delivery beat the loader's own STA $01 and the NMI vectored out of
	// un-set-up RAM. tickFast() finalises the anchor once the instruction's
	// cycles are in. Bit 0 = timer A, bit 1 = timer B.
	u8   m_CIA2ArmPending = 0;
	u16  m_CIA2TALatch = 0xFFFF, m_CIA2TBLatch = 0xFFFF;
	u64  m_CIA2TADeadline = 0, m_CIA2TBDeadline = 0;	// emu-cycle underflow, 0=off
	u64  m_CIA2NMIDeadline = 0;			// min armed deadline: the hot-path fast-out
	u64  m_CIA2FactorInUse = 1;			// emu-cycles-per-C64-cycle the deadlines are in
	// CIA deadlines live in EMULATED cycles. C64Tests/16-NMITIMING proved why:
	// a wall-clock deadline expired after only half the expected instructions
	// whenever its armed state forced the RAD interpreter out of batching
	// (about 10MHz unbatched versus VICE's emulated 20MHz). I/O stretch now
	// charges every real C64-bus wait into m_EmuCycles, so this clock includes
	// the physical stalls which originally motivated the host-clock model while
	// remaining independent of how fast the ARM happens to execute the core.
	inline u64 ciaNow() { return m_EmuCycles; }
	inline u64 ciaSpan( u64 n ) const { return n * m_CIA2FactorInUse; }
	u64  m_SynthNMIsDelivered = 0;		// statistics
	u64  m_CIA2Rescales = 0;			// deadline re-denominations at speed changes
	// Forensic trace of the LAST timer-NMI lifecycle, for the stall dump:
	// emu-cycle stamps at arm (deadline computed), underflow latch, and
	// synthetic exposure. The core stamps the take separately.
	// Each arm starts a new GENERATION; a stage that never happened for the
	// current one is reported as absent rather than as a wrapped subtraction
	// of two unrelated stamps. All four stamps are in ciaNow() units.
	u32  m_NMITraceGen = 0;
	u64  m_NMITraceArmAt = 0;
	u64  m_NMITraceLatchAt = 0;   u32 m_NMITraceLatchGen = 0xFFFFFFFF;
	u64  m_NMITraceExposeAt = 0;  u32 m_NMITraceExposeGen = 0xFFFFFFFF;
	u64  m_NMITraceTakeAt = 0;    u32 m_NMITraceTakeGen = 0xFFFFFFFF;

	// The core calls this when it actually dispatches an NMI, so the dump can
	// show delivery latency in the same clock as everything else.
	void noteNMITaken()
	{
		m_NMITraceTakeAt = ciaNow();
		m_NMITraceTakeGen = m_NMITraceGen;
	}
	// Stamps as microseconds since the arm, or 0xFFFFFFFF for "never happened
	// for this arm".
	u32 nmiTraceStageUS( u64 at, u32 gen ) const
	{
		if ( gen != m_NMITraceGen || at < m_NMITraceArmAt )
			return 0xFFFFFFFF;
		const u64 per = m_CIA2FactorInUse ? m_CIA2FactorInUse : 1;
		return (u32)( ( at - m_NMITraceArmAt ) / per );
	}

	// Cold: timer/mask register writes ($DD04-$DD0F) update the model.
	void cia2TimerWrite( u16 addr, u8 value );
	void cia2RecomputeNMIState();
	void cia2MaybeDeliverSynthNMI();
	// Anchor a START recorded by cia2TimerWrite; see m_CIA2ArmPending.
	void cia2FinishArm();
	// Serial-port reads sustain the 1MHz hold with a short top-up, matching
	// the real SuperCPU's access-triggered auto-slow. Mirror suppression is
	// NOT touched by polls; see the implementation.
	void notePollHold();
	// The CIA counts C64 cycles; our deadlines are stored in emulated cycles
	// under some emu-per-C64 factor. EVERY effective-speed transition must
	// re-denominate outstanding deadlines -- Winter Games arms its timer in
	// turbo and selects 1MHz ($D07B) one instruction later, and a deadline
	// left in turbo units fires 20x late in C64 time, exactly late enough to
	// land inside the native-mode window it was engineered to miss.
	void cia2OnRateChange();
	inline u64 effectiveCIAFactor() const
	{
		return ( m_IECHoldCycles || m_IECPollHoldCycles ) ? 1ULL
		     : (u64)( m_PacingCheckCycles ? m_PacingCheckCycles : 1 );
	}
private:

private:
	IC64Bus        *m_C64;
	IMirrorSink    *m_Mirror;
	IIOInterceptor *m_IO;
	const bool *m_DOSExtState = 0;
	const u8       *m_BootmapROM;
	u32             m_BootmapROMLength;
	u8             *m_ROMShadow;
	bool            m_SCPU128Mode;
	bool            m_HasBasic, m_HasKernal, m_HasChar;

	// Pacing state. m_HostPerEmuQ16 is host cycles per emulated cycle in 16.16
	// fixed point, so the common case is a multiply and a shift rather than a
	// division per instruction.
	u64 m_HostPerEmuQ16;		// host cycles per emulated cycle, 16.16
	u64 m_HostPerEmuQ16Slow;	// the same at 1MHz, for IEC hold-off
	u64 m_PacingAnchor;
	u32 m_SelectedEmulatedHz;
	bool m_BusKeepalive = false;
	u64 m_NextKeepalive = 0;

	bool m_IECThrottleEnabled;
	u32  m_IECHoldCycles;		// emulated cycles left at forced 1MHz
	u32  m_IECActivityCycles = 0;	// edge-rearmed transaction window (countdown)
	// SPEED-only hold sustained by serial-port READS (mere polls included):
	// the real SuperCPU's auto-slow re-arms on every access. Kept apart from
	// m_IECHoldCycles because iecBusActive() -- the MIRROR gate -- must never
	// see it: polls suppressing mirroring is the blackout latch-up.
	u32  m_IECPollHoldCycles = 0;
public:
	// Largest single tickFast() chunk seen. The 65816 run() loop batches bus
	// ticks for speed but must drop to per-instruction ticks while the serial
	// bus is active -- batched ticks compress the real-time spacing of bus
	// edges, which intermittently violates drive setup windows. This exists
	// so a test can PROVE the granularity contract instead of trusting it.
	u32  m_MaxTickChunk = 0;
private:
	// CIA2 port A has three different pieces of state. A port read contains
	// live PA6/PA7 input pins, so it must never replace the output-latch
	// baseline used to recognise PA3-PA5 IEC transitions. DDRA determines
	// whether a latch change actually reaches the VIC-bank/IEC pins.
	u8   m_CIA2PortALatch;
	u8   m_LastCIA2PortARead;
	u8   m_CIA2DDRA;
	bool m_HaveCIA2PortALatch;

	// Base of the ACTIVE screen's sprite-pointer row ($xxF8 masked to 8), or
	// the sentinel when the screen sits under the I/O window and cannot be
	// mirrored at all. Kept fresh by writes to $D018 and $DD00.
	u32  m_SpritePtrBase;
	u32  m_PreviousSpritePtrBase;
	u8   m_LastD018;
public:
	// First byte of the active VIC screen matrix, or 0xFFFFFFFF when that
	// matrix lies under the real machine's I/O window and cannot be mirrored.
	// The sprite-pointer tracker already follows both $D018 and CIA2's VIC-bank
	// select, so deriving the matrix here keeps the mirror scheduler on exactly
	// the same live state without another copy to go stale.
	// Read-only shadow access for passive display consumers. These expose no
	// bus object and therefore cannot turn HDMI rendering into physical traffic.
	const u8 *ramShadow() const     { return m_RAM; }
	const u8 *charROMShadow() const { return m_CharROM; }
	const u8 *colourRAMShadow() const
	{
		return m_ROMShadow ? m_ROMShadow + 0xD800 : 0;
	}
	u8 vicRegister( u8 reg ) const  { return m_VICRegShadow[ reg & 0x3F ]; }
	u8 colourRAM( u16 offset ) const
	{
		return m_ROMShadow
		     ? m_ROMShadow[ 0xD800 + ( offset & 0x03FF ) ] & 0x0F : 14;
	}
	u32 activeVICBankBase() const
	{
		const u8 outputs = m_HaveCIA2DDRA ? m_CIA2DDRA : 0x03;
		const u8 bank = m_HaveCIA2PortALatch
		              ? (u8)( ~m_CIA2PortALatch & outputs & 3 ) : 0;
		return (u32)bank << 14;
	}
	u32 activeVICScreenBase() const
	{
		// Unlike activeScreenBase(), this is a logical Pi-shadow address and is
		// therefore valid even when the selected matrix lies below physical I/O.
		return activeVICBankBase() + ( (u32)( m_LastD018 >> 4 ) << 10 );
	}

	u32 activeScreenBase() const
	{
		return m_SpritePtrBase == 0xFFFFFFFF
		     ? 0xFFFFFFFF : m_SpritePtrBase - 0x3F8;
	}
	// First byte of the active 8K bitmap, or the sentinel while bitmap mode is
	// off. Unlike activeScreenBase(), this remains meaningful when the screen
	// matrix itself lies under I/O: $D011/$D018 and the CIA2 bank pins select
	// the bitmap independently of whether the CPU can mirror the matrix row.
	u32 activeBitmapBase() const
	{
		if ( !( m_VICRegShadow[ 0x11 ] & 0x20 ) )
			return 0xFFFFFFFF;
		return activeVICBankBase() + ( ( m_LastD018 & 0x08 ) ? 0x2000 : 0 );
	}
	// First byte of the active 2K character-pattern region, or the sentinel
	// while bitmap mode is selected or the VIC is fetching its internal
	// character ROM. D018 bits 1-3 select 2K steps inside the CIA2-selected
	// 16K VIC bank. In banks 0 and 2, offsets $1000/$1800 select character ROM,
	// not physical DRAM, so there is nothing for the mirror scrub to repair.
	u32 activeCharsetBase() const
	{
		if ( m_VICRegShadow[ 0x11 ] & 0x20 )
			return 0xFFFFFFFF;
		const u8 bank = (u8)( activeVICBankBase() >> 14 );
		const u32 offset = (u32)( m_LastD018 & 0x0E ) << 10;
		if ( ( bank & 1 ) == 0 && offset >= 0x1000 && offset < 0x2000 )
			return 0xFFFFFFFF;
		return ( (u32)bank << 14 ) + offset;
	}

	// Writes aimed at the active sprite-pointer row, same-value or not. A
	// multiplexer rewrites this row several times per FRAME; the rate is the
	// diagnostic that distinguishes pointer-flip animation from multiplexing.
	u64  m_PtrRowWrites = 0;

	// --- under-I/O sprite-shape relocation --------------------------------
	// In VIC bank 3 a sprite pointer of $40-$7F selects a shape block at
	// $D000-$DFFF: RAM to the VIC, which always fetches DRAM there, but
	// unreachable to our mirror, because the halted 6510 keeps the real
	// machine's I/O banked in and a bus write lands on a chip register (see
	// CWriteBuffer::shouldMirror). A real SuperCPU-side program can still put
	// shapes there -- 3D Pool /SCPU double-buffers across banks 1 and 3 and
	// keeps thirty rotation blocks at $D080-$D77F, which is exactly why its
	// balls alternated between correct (bank-1 frame, deliverable) and garbage
	// (bank-3 frame, never delivered) at frame rate.
	//
	// The one lever we do control is the pointer VALUE we deliver. So: remap
	// each under-I/O block, on first use, to a free block in $C000-$CBFF,
	// mirror the shape bytes to the relocated copy, and deliver the remapped
	// pointer. The real VIC then fetches identical bytes from an address it
	// can be fed at. The shadow stays faithful throughout -- the program
	// rereads its own values, and only deliveries are translated.
	//
	// The relocated blocks' underlying DRAM belongs to us from then on: the
	// write buffer shields them from ordinary game-write delivery and the
	// resync sweep heals them from the under-I/O source. That is safe because
	// DRAM is only ever read by the VIC, and the VIC only reads what screen,
	// bitmap and pointers select; a block both selected raw AND stolen would
	// conflict, so allocation refuses blocks the current screen or bitmap
	// spans.
	bool m_RelocEnable = true;		// config MIRROR_D000_RELOCATE
	u8   m_PtrReloc[ 0x40 ];		// under-I/O block (V-$40) -> reloc block, $FF none
	u8   m_RelocInUse[ 0x30 ];		// reloc block -> source pointer V, $FF free
	u8   m_RelocCount = 0;			// allocated blocks; the buffer's fast-out
	u64  m_RelocAllocs = 0;			// statistics
	u64  m_RelocDelivered = 0;		// pointer values delivered translated
	u64  m_RelocExhausted = 0;		// translation wanted, no free block

	// Deliverable stand-in for pointer value v written to the ACTIVE row.
	// Identity for everything except bank-3 pointers into $D000-$DFFF.
	u8 relocPointerValue( u8 v )
	{
		if ( !m_RelocEnable || v < 0x40 || v >= 0x80
		     || m_SpritePtrBase < 0xC000 || m_SpritePtrBase == 0xFFFFFFFF )
			return v;
		const u8 slot = v - 0x40;
		if ( m_PtrReloc[ slot ] != 0xFF )
		{
			m_RelocDelivered++;
			return m_PtrReloc[ slot ];
		}
		const u8 r = allocRelocBlock( v );
		if ( r == 0xFF )
		{
			m_RelocExhausted++;
			return v;
		}
		m_RelocDelivered++;
		return r;
	}
	u8 allocRelocBlock( u8 v );

	// One bit per 64-byte block of the 64K space: set when an ACTIVE sprite
	// pointer selects that block. The write buffer defers these bytes to the
	// border so a re-rendered shape is only ever delivered whole -- the VIC
	// fetches shapes on the sprite's own display lines, and a block updated
	// mid-display shows the render's cleared/partial transient as a torn,
	// flickering sprite. Kept fresh by pointer-row writes and $D018/$DD00.
	//
	// Under-I/O pointers are tracked as their RELOCATED block when one exists:
	// that is the block actually being delivered to and fetched from.
	u64  m_HotShapeBlocks[ 1024 / 64 ] = { 0 };
	void updateHotShapeBlocks()
	{
		for ( u32 i = 0; i < 1024 / 64; i++ ) m_HotShapeBlocks[ i ] = 0;
		if ( m_SpritePtrBase == 0xFFFFFFFF )
			return;
		const u32 bank = ( m_SpritePtrBase >> 14 ) & 3;
		const u8 enabled = m_VICRegShadow[ 0x15 ];
		for ( u32 n = 0; n < 8; n++ )
		{
			if ( !( enabled & ( 1u << n ) ) )
				continue;
			u32 v = m_RAM[ (u16)( m_SpritePtrBase + n ) ];
			if ( m_RelocEnable && bank == 3 && v >= 0x40 && v < 0x80
			     && m_PtrReloc[ v - 0x40 ] != 0xFF )
				v = m_PtrReloc[ v - 0x40 ];
			const u32 blk = ( bank << 8 ) + v;
			m_HotShapeBlocks[ blk >> 6 ] |= 1ULL << ( blk & 63 );
		}
	}
private:
public:
	// Last value written to each VIC register, maintained on the I/O write
	// path. The mirror scheduler reads sprite Y positions and the enable mask
	// from here so that avoiding sprite fetch lines costs nothing on the bus.
	u8   m_VICRegShadow[ 0x40 ];
private:
	void updateSpritePtrBase()
	{
		// CIA2 PA0/PA1 select the VIC bank through inverters. Output pins use
		// the latch; input pins float high and therefore select bank bit zero.
		// Before DDRA has been observed, retain the historical/output assumption
		// so a lone $DD00 write takes effect immediately.
		const u8 outputs = m_HaveCIA2DDRA ? m_CIA2DDRA : 0x03;
		const u8 bank = m_HaveCIA2PortALatch
		              ? (u8)( ~m_CIA2PortALatch & outputs & 3 ) : 0;
		const u32 base = ( (u32)bank << 14 )
		               + ( (u32)( m_LastD018 >> 4 ) << 10 ) + 0x3F8;
		// Screen under the I/O window: the pointer row is unreachable over
		// the bus (real $01 stays $37), so the fast path must never try.
		const u32 next = ( ( base & 0xF000 ) == 0xD000 ) ? 0xFFFFFFFF : base;
		if ( next != m_SpritePtrBase )
			m_PreviousSpritePtrBase = m_SpritePtrBase;
		m_SpritePtrBase = next;
	}
	bool m_HaveCIA2PortARead;
	bool m_HaveCIA2DDRA;
public:
	// The last 16 I/O accesses, newest last: addr | value<<16 | (write?1:0)<<24.
	// Costs a store on the I/O path only -- never on the RAM hot path -- and
	// exists so a frozen machine can say what it last did to the hardware. A
	// freeze is almost always a wait for an I/O event that never came, and the
	// last few accesses name the conversation that died.
	u32 m_IOLog[ 64 ];
	u8  m_IOLogPos;		// wraps at 64

	// A second ring for CIA traffic only ($DC00-$DDFF). The general ring gets
	// flooded by whatever loop the machine is in; the CIA conversation -- DDR
	// setup, interrupt mask writes, ATN/CLK/DATA toggles -- is the part a
	// serial post-mortem actually needs, and it must survive the flood.
	u32 m_CIALog[ 64 ];
	u32 m_CIALogCyc[ 64 ];		// ARM host-cycle stamp per entry: real time, sees
								// stalls that emulated cycles cannot
	u32 m_CIALastRead = 0xFFFFFFFF;	// edge compression: identical repeat reads not logged
	u8  m_CIALogPos;

	// Free-running emulated cycle counter, for timestamping. Incremented in
	// tickFast, so it costs one add on a path that already runs per
	// instruction.
	u64 m_EmuCycles = 0;
private:
	void noteIECActivity();
	TimingHook m_TimingHook;
	void      *m_TimingHookCtx;
};

#endif
