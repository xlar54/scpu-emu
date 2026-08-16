/*
   SCPU-EMU - CMD SuperCPU emulation for the C64/C128 using a RAD Expansion Unit
   Copyright (c) 2026 SCPU-EMU contributors

   CREU - Commodore 1700/1764/1750-style RAM Expansion Unit (the 8726 "REC").

   A self-contained model of the expansion controller: eleven registers at
   $DF00-$DF0A and a block of expansion RAM. It performs no I/O of its own and
   knows nothing about the C64 -- every byte it moves passes through IREUHost,
   so the same object works against the Pi's shadow memory on hardware and
   against a plain array in a host test.

   NOT WIRED IN YET. Nothing constructs this class; adding it to the machine is
   a separate decision, chiefly because CC64Memory takes a SINGLE
   IIOInterceptor and CSuperCPURegisters already holds it. Whoever connects
   this will have to chain the two rather than replace one.

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
#ifndef _scpu_reu_h
#define _scpu_reu_h

#include "../Common/types.h"

// --- REUSIZE config values --------------------------------------------------
// The numbers on the left are what goes in the .cfg file. They are deliberately
// NOT the size in any unit -- they are a selector, so the list can grow without
// renumbering, and 1 means "no REU fitted" rather than 0 so that a missing or
// mistyped key is distinguishable from a deliberate "off".
//
//   REUSIZE 1   no REU            (default)
//   REUSIZE 2   128 KB            1700
//   REUSIZE 3   256 KB            1764
//   REUSIZE 4   512 KB            1750
//   REUSIZE 5     2 MB            1750 with a common third-party expansion
//   REUSIZE 6     4 MB
//   REUSIZE 7    16 MB            the largest the 24-bit REU address reaches
//
// 1 MB is absent on purpose: it is not in the requested set. Adding it later
// means appending a new selector, not renumbering these.
#define REUSIZE_NONE    1
#define REUSIZE_128K    2
#define REUSIZE_256K    3
#define REUSIZE_512K    4
#define REUSIZE_2MB     5
#define REUSIZE_4MB     6
#define REUSIZE_16MB    7

// Register addresses, for callers deciding whether an access belongs to us.
#define REU_REG_FIRST   0xDF00
#define REU_REG_LAST    0xDF0A

// The C64 side of a transfer. Implemented by whatever owns the machine's
// memory; the REU never touches a bus directly.
//
// IMPORTANT for the SuperCPU case: a real REU writes into the C64's DRAM, which
// the VIC-II then displays. An implementation that only updates the Pi's shadow
// will produce a correct-looking machine with a stale picture, in exactly the
// way the $D000-$DFFF suppression did. Whatever implements this must mirror.
class IREUHost
{
public:
	virtual ~IREUHost() {}
	virtual u8   reuHostRead( u16 addr ) = 0;
	virtual void reuHostWrite( u16 addr, u8 value ) = 0;
};

class CREU
{
public:
	CREU();
	~CREU();

	// Allocate the expansion RAM. Takes a REUSIZE_* selector, not a size.
	// An unknown selector is treated as REUSIZE_NONE rather than guessed at.
	// Returns false only if a requested allocation failed, in which case the
	// unit reports itself absent and every register reads back as open bus.
	bool init( u8 reuSizeSelector );

	void attachHost( IREUHost *host ) { m_Host = host; }

	bool present() const   { return m_Size != 0; }
	u32  sizeBytes() const { return m_Size; }

	// Power-on / reset state. The hardware clears the command and status but
	// does NOT clear expansion RAM, and software has been known to rely on
	// contents surviving a warm reset.
	void reset();

	// --- register file ----------------------------------------------------
	// Return false when the address is not ours, so a caller chaining several
	// devices can pass it on.
	bool read( u16 addr, u8 &value );
	bool write( u16 addr, u8 value );

	// A transfer can be armed to wait for a write to $FF00 rather than run
	// immediately. The owner must call this on every write to that address,
	// INCLUDING one that changes nothing: the REU triggers on the bus cycle,
	// not on a value change, and same-value elimination upstream would
	// otherwise swallow the trigger.
	void noteFF00Write();

	// --- statistics, for tests and the diagnostic dump --------------------
	u64 transfers() const     { return m_Transfers; }
	u64 bytesMoved() const    { return m_BytesMoved; }
	u64 verifyFaults() const  { return m_VerifyFaults; }

	// Cycles the last transfer would have cost a real machine: one per byte,
	// which is what the REU's DMA achieves while it holds the CPU off the bus.
	// Exposed rather than charged internally because only the caller knows
	// which clock the emulated machine is running on.
	u32 lastTransferCycles() const { return m_LastTransferCycles; }

	// Direct access for tests and for a future save-state. Masked into range
	// like the SIMM: a real unit decodes a fixed number of address lines, so
	// an address beyond the fitted size aliases down, and size-probing
	// software depends on exactly that.
	inline u8 peek( u32 offset ) const
	{
		return m_Size ? m_RAM[ offset & m_AddressMask ] : 0xFF;
	}
	inline void poke( u32 offset, u8 value )
	{
		if ( m_Size ) m_RAM[ offset & m_AddressMask ] = value;
	}

private:
	void executeTransfer();
	void finishTransfer( bool fault );

	IREUHost *m_Host;
	u8  *m_RAM;
	u32  m_Size;			// 0 when absent
	u32  m_AddressMask;		// m_Size - 1, so aliasing is a mask not a branch

	// The register file. These are the LIVE values: a transfer without
	// autoload advances them and leaves them where it finished.
	u16 m_C64Base;
	u32 m_REUBase;			// 24 bits: low, high, bank
	u16 m_Length;
	u8  m_Command;
	u8  m_IRQMask;
	u8  m_AddressControl;

	// Shadow copies taken when the program writes the registers. Autoload
	// restores from these at the end of a transfer, which is what makes a
	// repeated block move a single register write instead of five.
	u16 m_C64Shadow;
	u32 m_REUShadow;
	u16 m_LengthShadow;

	// Status bits 7/6/5, held apart from the read-only size and version bits
	// because reading the register clears these three and nothing else.
	bool m_IRQPending;
	bool m_EndOfBlock;
	bool m_Fault;

	// True once a command has armed a transfer that is waiting for $FF00.
	bool m_PendingFF00;

	u64 m_Transfers;
	u64 m_BytesMoved;
	u64 m_VerifyFaults;
	u32 m_LastTransferCycles;
};

#endif
