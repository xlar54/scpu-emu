/*
   SCPU-EMU - CMD SuperCPU emulation for the C64/C128 using a RAD Expansion Unit
   Copyright (c) 2026 SCPU-EMU contributors

   The SuperCPU's 24-bit address space.

   A 65816 sees 16MB as 256 banks of 64KB. Only the first of those is the
   Commodore 64; everything above it belongs to the accelerator and never
   touches the expansion port at all, which is why it runs at full speed.

   Layout, taken from VICE's SCPU64 emulation (vice/src/scpu64/scpu64mem.c,
   mem_read2 / mem_store2) rather than from documentation -- the manuals are
   vague about the top of the map and the OCR'd scan is garbled there:

     $000000-$00FFFF   bank 0: the C64-visible 64K. Shadowed RAM plus the real
                       machine's I/O, handled by CC64Memory. The only bank that
                       can reach the expansion port.
     $010000-$01FFFF   bank 1: the SuperCPU's own SRAM. 128K of SRAM total, of
                       which bank 0's shadow is the other half.
     $020000-...       SuperRAM, up to the fitted SIMM size.
     $F60000-$F7FFFF   a 128K window onto the SIMM.
     $F80000-$FFFFFF   SuperCPU ROM, mirrored from a 512K image.
     anything else     open bus, which on a 65816 reads back as the bank byte.

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
#ifndef _scpu_memory_map_h
#define _scpu_memory_map_h

#include "../Common/types.h"
#include "../CPU/cpu_bus.h"
#include "../C64/c64_memory.h"
#include "fast_ram.h"

#define SCPU_BANK1_SIZE     0x10000		// the SuperCPU's own SRAM bank
#define SCPU_SIMM_WINDOW    0xF60000	// 128K window onto the SIMM
#define SCPU_SIMM_WINDOW_SZ 0x20000
#define SCPU_ROM_BASE       0xF80000
#define SCPU_ROM_MAXSIZE    0x80000		// 512K address space for the ROM image

// m_Bank0Accesses and m_FastAccesses are read by NOTHING in the firmware. The
// references under Source/ are: this header, the constructor and reset() in
// memory_map.cpp, and two further increments in readAbove()/writeAbove()
// (memory_map.cpp:60,100) which this macro deliberately leaves ALONE -- those
// are the cold out-of-line bank>=1 paths, so gating them would buy nothing and
// the counters simply stay partial in the firmware, where nobody reads them.
// The only readers are in Tests/SuperCPU/test_memory_map.cpp. Contrast
// m_RamWrites, which Source/App/boot.cpp:1252 and :2303 DO report and which
// therefore stays unconditional.
//
// MEASURED in the shipped object: the increment is a dependent ldr/add/str to
// the same address on EVERY emulated bank-0 access, and a host census of the
// TI throughput benchmark measured 2.806 bank-0 accesses per emulated
// instruction. That is ~8.4 ARM instructions per emulated instruction spent on
// a number the Pi never prints -- and on an in-order A53 that load-add-store
// chain also serialises, which the instruction count does not capture.
//
// The host build keeps counting, so every test assertion still holds and the
// suite is unchanged. The FIELDS stay in both builds: their position in front
// of m_Bank1 is load-bearing codegen (see the note on them below), and
// deleting them would push m_Bank0 back out of a ldr immediate's reach and
// undo the K448 layout hoist.
#ifdef SCPU_HOST_BUILD
	#define SCPU_COUNT_ACCESS( c ) ( (c)++ )
#else
	#define SCPU_COUNT_ACCESS( c ) ( (void)0 )
#endif

// final: see the note on CC64Memory. The 65816 reaches memory through this
// class, so it is on the hot path twice over.
class CSuperCPUMemoryMap final : public ICpuBus
{
public:
	CSuperCPUMemoryMap();

	// Bank 0 is delegated: it is the Commodore 64, banking, I/O and all.
	void attachBank0( CC64Memory *bank0 ) { m_Bank0 = bank0; }
	void attachFastRAM( CFastRAM *simm )  { m_SIMM = simm; }

	// The SuperCPU's own ROM image, if one was supplied. Without it the ROM
	// region reads open bus, which is fine: SCPU-EMU boots the machine's own
	// KERNAL out of bank 0 and never needs the accelerator's ROM.
	void setROM( const u8 *image, u32 length );
	bool hasROM() const { return m_ROMLength != 0; }

	// --- interrupt vector reroute -----------------------------------------
	// See CC64Memory::interruptRerouteActive. When it applies, the vector
	// comes from the accelerator's EPROM at $F80000+vector instead of the
	// C64 map.
	bool interruptRerouteActive( bool emulationMode ) const
	{
		return m_Bank0 && m_ROMLength
		    && m_Bank0->interruptRerouteActive( emulationMode );
	}
	void noteNMITaken() { if ( m_Bank0 ) m_Bank0->noteNMITaken(); }
	// Bank 0 owns the VIC shadows and the sampled interrupt lines, so the
	// core's interrupt ring asks for its context through here. One call per
	// interrupt; see CC64Memory::interruptContext for what it may and may not
	// touch.
	u64 interruptContext()
		{ return m_Bank0 ? m_Bank0->interruptContext() : 0; }

	u16 rerouteVector( u16 addr )
	{
		m_Bank0->m_VectorReroutes++;
		return (u16)( read8( SCPU_ROM_BASE + addr )
		            | ( (u16)read8( SCPU_ROM_BASE + addr + 1 ) << 8 ) );
	}
	const u8 *romImage() const { return m_ROM; }
	u32  romLength() const { return m_ROMLength; }

	void reset();

	// --- ICpuBus ----------------------------------------------------------
	// Bank 0 is inlined here and forwarded straight to CC64Memory's own inline
	// fast path, so an ordinary RAM access becomes a few instructions with no
	// call at all. Everything above bank 0 -- private SRAM, SuperRAM, the SIMM
	// window, the ROM -- goes out of line; it is rare, and the decode chain is
	// too long to be worth inlining.
	__attribute__((always_inline)) u8 read8( scpu_addr_t addr ) override
	{
		addr &= SCPU_ADDR_MASK;
		if ( addr < 0x010000 )
		{
			SCPU_COUNT_ACCESS( m_Bank0Accesses );
			return m_Bank0 ? m_Bank0->readFast( (u16)addr ) : 0xFF;
		}
		// Bank 1 inline: the SuperCPU's private SRAM is the hottest region
		// after bank 0 -- native-mode code and data live there -- and the
		// out-of-line readAbove call cost more than the access itself. The
		// counter increment is kept for the HOST build, where the tests assert
		// on it; the firmware never reads it -- see SCPU_COUNT_ACCESS above.
		if ( addr < 0x020000 )
		{
			SCPU_COUNT_ACCESS( m_FastAccesses );
			const u16 offset = (u16)addr;
			if ( __builtin_expect( offset < 2, 0 ) )
				return readBank1Port( offset );
			return m_Bank1[ offset ];
		}
		return readAbove( addr );
	}

	__attribute__((always_inline)) void write8( scpu_addr_t addr, u8 value ) override
	{
		addr &= SCPU_ADDR_MASK;
		if ( addr < 0x010000 )
		{
			SCPU_COUNT_ACCESS( m_Bank0Accesses );
			if ( m_Bank0 ) m_Bank0->writeFast( (u16)addr, value );
			return;
		}
		if ( addr < 0x020000 )
		{
			SCPU_COUNT_ACCESS( m_FastAccesses );
			const u16 offset = (u16)addr;
			if ( __builtin_expect( offset < 2, 0 ) )
			{
				writeBank1Port( offset, value );
				return;
			}
			m_Bank1[ offset ] = value;
			return;
		}
		writeAbove( addr, value );
	}

	__attribute__((always_inline)) u32 preciseCycleAccess( scpu_addr_t addr,
	                                                      bool write ) const override
	{
		addr &= SCPU_ADDR_MASK;
		return ( addr < 0x010000 && m_Bank0 )
		     ? m_Bank0->preciseVICCycleAccess( (u16)addr, write ) : 0;
	}

	u8   readAbove( scpu_addr_t addr );
	void writeAbove( scpu_addr_t addr, u8 value );

	// Interrupts and pacing forward inline to bank 0's own fast paths. These
	// run once per emulated instruction; two virtual hops each was real money.
	bool irqAsserted() override { return m_Bank0 ? m_Bank0->irqFast() : false; }
	void sampleIRQNMIFast( bool &irq, bool &nmi )
	{
		if ( m_Bank0 ) m_Bank0->sampleIRQNMIFast( irq, nmi );
		else irq = nmi = false;
	}

	// Tick per instruction while IEC is active or bank 0 is effectively at
	// 1MHz. This covers both the automatic fallback and CMD's explicit $D07A
	// slow mode; the latter may be in a serial wait even after the activity
	// timer has expired.
	bool fineTicksRequired() const { return m_Bank0 && m_Bank0->fineTicksRequired(); }
	bool nmiAsserted() override { return m_Bank0 ? m_Bank0->nmiFast() : false; }
	void tick( u32 nCycles ) override { if ( m_Bank0 ) m_Bank0->tickFast( nCycles ); }
	void tickPhaseTrim( s32 c64Cycles ) override { if ( m_Bank0 ) m_Bank0->tickPhaseTrim( c64Cycles ); }

	// Statistics, split by where an access landed. Useful for judging how much
	// of a workload escapes bank 0 -- anything that does runs at full speed.
	//
	// EDIT 2 of 3, lands with the next hunk. LAYOUT IS LOAD-BEARING HERE.
	// m_Bank1 used to be declared first, which pushed these two and m_Bank0
	// below to +0x10008, +0x10010 and +0x10018 -- confirmed by gdb ptype /o on
	// kernel8.elf as 65544 / 65552 / 65560, all past a ldr immediate's reach.
	// So the shipped K448 object emits `add x0, x0, #0x10, lsl #12` on EVERY
	// bank-0 access purely to address them: busRead8+0x34, busWrite8, and
	// every fast-switch site in stepInner (e.g. +0x9c0). They also sat a full
	// 64KB from the vtable pointer the same access had just loaded, so the
	// three hottest fields in the class cost a cache line of their own.
	//
	// Declared ahead of the array they land at +8 / +16 / +24, share the
	// object's first line with that pointer, and are addressed directly --
	// verified in the rebuilt object as `ldr x1,[x0,#8]` / `ldr x21,[x0,#24]`
	// with no add, and stepInner shrinks by 548 bytes alongside the
	// c64_memory.h hoist (0x6b10 -> 0x68ec). m_Bank1 is only ever reached by
	// name -- no offsetof, no memcpy of this object, no test asserts an offset
	// -- so this is codegen only.
	u64 m_Bank0Accesses;
	u64 m_FastAccesses;

private:
	// Bank 1's first two bytes alias the C64 CPU port. Keep that hardware
	// exception out of the always-inlined common accessor: inlining readFast /
	// writeFast here replicated the entire bank-0 path through many opcode
	// handlers even though ordinary bank-1 code never touches those bytes.
	__attribute__((noinline, cold)) u8 readBank1Port( u16 offset );
	__attribute__((noinline, cold)) void writeBank1Port( u16 offset, u8 value );

	CC64Memory *m_Bank0;
	CFastRAM   *m_SIMM;
	const u8   *m_ROM;
	u32         m_ROMLength;

public:
	// Bank 1: the accelerator's private SRAM. Declared LAST on purpose -- see
	// the note on m_Bank0Accesses above. 64KB in front of the scalars put
	// every one of them out of immediate reach and in a far cache line.
	//
	// CONSEQUENCE, so nobody trips over it later: this array now starts at
	// offset 52, so it is 4-byte aligned, not 8 (verified in the rebuilt
	// object as `ldrb w0,[x0,#52]`). That is safe because every one of its
	// uses is a byte access -- memory_map.cpp:27,41,63,104; memory_map.h:115,
	// 138; supercpu.cpp:492,684-691,990,1083-1084; registers.h:314,325; and
	// Tests/SuperCPU/test_memory_map.cpp -- and AArch64 permits unaligned
	// byte access on normal memory. Do NOT add a u16/u32/u64 cast, a typed
	// memcpy or a DMA handoff over this array without first restoring an
	// explicit alignment guarantee.
	u8 m_Bank1[ SCPU_BANK1_SIZE ];
};

#endif
