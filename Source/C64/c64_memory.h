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

class CC64Memory : public ICpuBus
{
public:
	CC64Memory();

	void attachBus( IC64Bus *bus ) { m_C64 = bus; }
	void setMirrorSink( IMirrorSink *sink ) { m_Mirror = sink; }
	void setIOInterceptor( IIOInterceptor *io ) { m_IO = io; }

	// Reset the emulated 6510 port to its power-on state ($00 = $2F, $01 = $37)
	// and clear shadow DRAM. Does not touch the ROM images.
	void reset();

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

	u64 m_PacingDebtCycles;		// emulated cycles run but not yet paid for in real time

private:
	IC64Bus        *m_C64;
	IMirrorSink    *m_Mirror;
	IIOInterceptor *m_IO;
	bool            m_HasBasic, m_HasKernal, m_HasChar;

	// Pacing state. m_HostPerEmuQ16 is host cycles per emulated cycle in 16.16
	// fixed point, so the common case is a multiply and a shift rather than a
	// division per instruction.
	u64 m_HostPerEmuQ16;
	u64 m_PacingAnchor;
};

#endif
