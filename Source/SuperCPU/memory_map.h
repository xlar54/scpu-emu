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
	const u8 *romImage() const { return m_ROM; }
	u32  romLength() const { return m_ROMLength; }

	void reset();

	// --- ICpuBus ----------------------------------------------------------
	// Bank 0 is inlined here and forwarded straight to CC64Memory's own inline
	// fast path, so an ordinary RAM access becomes a few instructions with no
	// call at all. Everything above bank 0 -- private SRAM, SuperRAM, the SIMM
	// window, the ROM -- goes out of line; it is rare, and the decode chain is
	// too long to be worth inlining.
	u8 read8( scpu_addr_t addr ) override
	{
		addr &= SCPU_ADDR_MASK;
		if ( addr < 0x010000 )
		{
			m_Bank0Accesses++;
			return m_Bank0 ? m_Bank0->readFast( (u16)addr ) : 0xFF;
		}
		return readAbove( addr );
	}

	void write8( scpu_addr_t addr, u8 value ) override
	{
		addr &= SCPU_ADDR_MASK;
		if ( addr < 0x010000 )
		{
			m_Bank0Accesses++;
			if ( m_Bank0 ) m_Bank0->writeFast( (u16)addr, value );
			return;
		}
		writeAbove( addr, value );
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
	void notifyInterruptAcknowledged()
	{
		if ( m_Bank0 ) m_Bank0->notifyInterruptAcknowledged();
	}
	bool nmiAsserted() override { return m_Bank0 ? m_Bank0->nmiFast() : false; }
	void tick( u32 nCycles ) override { if ( m_Bank0 ) m_Bank0->tickFast( nCycles ); }

	// Bank 1: the accelerator's private SRAM.
	u8 m_Bank1[ SCPU_BANK1_SIZE ];

	// Statistics, split by where an access landed. Useful for judging how much
	// of a workload escapes bank 0 -- anything that does runs at full speed.
	u64 m_Bank0Accesses;
	u64 m_FastAccesses;

private:
	CC64Memory *m_Bank0;
	CFastRAM   *m_SIMM;
	const u8   *m_ROM;
	u32         m_ROMLength;
};

#endif
