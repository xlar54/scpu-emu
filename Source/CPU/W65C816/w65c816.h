/*
   SCPU-EMU - CMD SuperCPU emulation for the C64/C128 using a RAD Expansion Unit
   Copyright (c) 2026 SCPU-EMU contributors

   WDC 65C816 core.

   STATUS: interface only. The implementation is milestone 2 -- see
   Docs/roadmap.md. Milestone 1 runs CM6502 to prove the hardware path first.

   This header exists now so that the contract is fixed before the work starts,
   and so CSuperCPU can be written against it.

   Design notes for whoever implements it:

     * Reset enters *emulation mode* (E=1), where the core is a 6502 with a few
       extra instructions. That is the mode a C64 KERNAL boots in, and it must
       agree with CM6502 instruction for instruction -- Tools/trace_compare
       exists to check exactly that.
     * In native mode (E=0) the M and X flags independently select 8- or 16-bit
       accumulator and index registers. Every addressing mode and most
       operations have to branch on them; a template parameterised on <M,X> that
       the dispatcher selects between avoids re-testing the flags per access.
     * The SuperCPU implements documented 6510 opcodes only. The encodings the
       NMOS 6502 used for its undocumented instructions are 65816 instructions
       here, which is why CM6502 treats them as NOPs rather than emulating the
       NMOS behaviour -- see the note in m6502.h.
     * Bank wrapping is not uniform. Direct page wraps within bank 0, the stack
       wraps within bank 0 in emulation mode but is 16-bit in native mode, and
       absolute indexed addressing crosses banks. These are the details that
       differential testing will not catch, because the 6502 never exercises
       them.
     * Interrupt vectors differ between the two modes.

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
#ifndef _w65c816_h
#define _w65c816_h

#include "../cpu.h"

// Status register. In emulation mode bits 4 and 5 behave as the 6502's B and
// unused bits; in native mode they are X and M.
#define W65_C 0x01	// carry
#define W65_Z 0x02	// zero
#define W65_I 0x04	// IRQ disable
#define W65_D 0x08	// decimal
#define W65_X 0x10	// index registers 8-bit (native) / break (emulation)
#define W65_M 0x20	// accumulator 8-bit (native) / unused (emulation)
#define W65_V 0x40	// overflow
#define W65_N 0x80	// negative

// Native-mode vectors
#define W65_VEC_NATIVE_COP    0x00FFE4
#define W65_VEC_NATIVE_BRK    0x00FFE6
#define W65_VEC_NATIVE_ABORT  0x00FFE8
#define W65_VEC_NATIVE_NMI    0x00FFEA
#define W65_VEC_NATIVE_IRQ    0x00FFEE

// Emulation-mode vectors
#define W65_VEC_EMU_COP       0x00FFF4
#define W65_VEC_EMU_ABORT     0x00FFF8
#define W65_VEC_EMU_NMI       0x00FFFA
#define W65_VEC_EMU_RESET     0x00FFFC
#define W65_VEC_EMU_IRQ       0x00FFFE

class CW65C816 : public ICpu
{
public:
	CW65C816();

	// --- ICpu -------------------------------------------------------------
	void attachBus( ICpuBus *bus ) override { m_Bus = bus; }
	void reset() override;
	u32  step() override;
	u64  run( u64 nCycles ) override;
	u64  cycles() const override { return m_Cycles; }
	scpu_addr_t pc() const override { return ( (u32)m_PBR << 16 ) | m_PC; }
	const char *name() const override { return "WDC 65C816"; }

	// --- register file ----------------------------------------------------
	u16 m_C;		// accumulator; A is the low byte, B the high byte
	u16 m_X, m_Y;	// index registers
	u16 m_S;		// stack pointer
	u16 m_D;		// direct page register
	u16 m_PC;
	u8  m_PBR;		// program bank
	u8  m_DBR;		// data bank
	u8  m_P;		// status
	bool m_E;		// emulation mode

	bool m_Stopped;	// STP executed
	bool m_Waiting;	// WAI executed, waiting for an interrupt

private:
	ICpuBus *m_Bus;
	u64      m_Cycles;
	bool     m_NMIPrevLevel;
	bool     m_NMIPending;
};

#endif
