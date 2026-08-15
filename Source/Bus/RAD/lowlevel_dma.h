/*

  {_______            {_          {______
        {__          {_ __               {__
        {__         {_  {__               {__
     {__           {__   {__               {__
 {______          {__     {__              {__
       {__       {__       {__            {__   
         {_________         {______________		Expansion Unit
                
 RADExp - A framework for DMA interfacing with Commodore C64/C128 computers using a Raspberry Pi Zero 2 or 3A+/3B+
 Copyright (c) 2022 Carsten Dachsbacher <frenetic@dachsbacher.de>

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

#include "c128_refresh.h"

//#define DOUBLE_GPIO_WRITES

#define DELAY(rounds) \
	for ( int i = 0; i < rounds; i++ ) { \
		asm volatile( "nop" );	asm volatile( "nop" );	asm volatile( "nop" );	asm volatile( "nop" );	asm volatile( "nop" );	asm volatile( "nop" );	asm volatile( "nop" );	asm volatile( "nop" );	asm volatile( "nop" );	asm volatile( "nop" );	asm volatile( "nop" );	asm volatile( "nop" ); }

#define CPU_RESET			(!(g2&bRESET_OUT)) 
#define BUTTON_PRESSED		(!(g2&bBUTTON))

#define IO2_ACCESS			(!(g2 & bIO2))
#define IO_ADDRESS			(( ( g3 >> 10 ) & 15 ) | ( ( g3 & 15 ) << 4 ))

#define IO1_ACCESS			(!(g2 & bIO1))
#define IO1_OR_IO2_ACCESS	(IO1_ACCESS||IO2_ACCESS)
#define GET_IO12_ADDRESS	IO_ADDRESS

#define ADDRESS0to7			IO_ADDRESS
#define ADDRESS_FFxx		(!(g2 & bTriggerFF00))

#define CPU_READS_FROM_BUS	(g2 & bRW_OUT)
#define CPU_WRITES_TO_BUS	(!(g2 & bRW_OUT))

#define CPU_IRQ_LOW			(!(g2 & bIRQ_OUT))
#define CPU_NMI_LOW			(!(g2 & bNMI))

#define VIC_HALF_CYCLE		(!(g2 & bPHI))
#define CPU_HALF_CYCLE		((g2 & bPHI))
#define VIC_BA				(!(g2 & bBA))

#define PUT_DATA_ON_BUS( D ) {											\
	register u32 DD = ( (D) & 255 ) << D0;								\
	CLR_GPIO( (D_FLAG & ( ~DD )) | bOE_Dx | bDIR_Dx );					\
	SET_GPIO( DD );														\
	SET_BANK2_OUTPUT													\
	WAIT_UP_TO_CYCLE( WAIT_CYCLE_READ+TIMING_OFFSET_CBTD );				\
	SET_GPIO( bOE_Dx | bDIR_Dx ); }

#define GET_DATA_FROM_BUS( D ) {										\
	SET_BANK2_INPUT														\
	SET_GPIO( bDIR_Dx );												\
	CLR_GPIO( bOE_Dx );													\
	WAIT_UP_TO_CYCLE( WAIT_CYCLE_WRITEDATA+TIMING_OFFSET_CBTD );		\
	D = ( read32( ARM_GPIO_GPLEV0 ) >> D0 ) & 255;						\
	SET_GPIO( bOE_Dx );													\
	SET_BANK2_OUTPUT }

__attribute__( ( always_inline ) ) inline u8 countBits( u16 x )
{
	u32 t;
	asm volatile( "dup v0.2d, %0" : : "r" ( x ) );
	asm volatile( "cnt v0.16b, v0.16b" : : );
	asm volatile( "fmov %0, v0.d[1]" : "=r" ( t ) );
	return *(u8*)&t;
}


__attribute__( ( always_inline ) ) inline u8 flipByte( u8 x )
{
	u32 t;
	asm volatile( "rbit %w0, %w1" : "=r" ( t ) : "r" ( x ) );	// flip all bits in 32-bit-DWORD
	asm volatile( "rev  %w0, %w1" : "=r" ( t ) : "r" ( t ) );	// reverse 4 bytes in 32-bit-DWORD
	return *(u8*)&t;
}

#define DISABLE_ADDRESS_LATCH_AND_BUSTRANSCEIVER( releaseDMA )								\
	SET_GPIO( bLATCH_A_OE | bGAME_OUT | bOE_Dx | bRW_OUT | (releaseDMA ? bDMA_OUT : 0) );	\
	INP_GPIO_RW();																			\
	SET_BANK2_OUTPUT 

#define ENABLE_D07_FOR_WRITING		\
	CLR_GPIO( bOE_Dx | bDIR_Dx );	\
	SET_BANK2_OUTPUT 

// Ceiling on every wait for a PHI2 phase, counted in GPLEV0 reads.
//
// A half-cycle is about 500ns and one MMIO read of GPLEV0 costs on the order
// of 100 ARM cycles, so a legitimate wait here is a handful of iterations --
// a few dozen at the very worst. Anything approaching this limit means PHI2
// has stopped arriving, and no amount of further spinning will bring it back.
//
// Unbounded, that spin is fatal rather than slow: these waits sit inside the
// BA loops and inside every bus access, all of which run under runFrame(). A
// PHI2 that stops takes the frame loop with it, so the button hook never runs
// again and the Pi has to be unplugged -- which is exactly the freeze where
// pressing reset printed nothing at all. Bounding them cannot make a working
// bus worse, and it turns a dead machine into one that can still tell us
// where it is.
#define RAD_PHI_WAIT_LIMIT 20000

#define WAIT_FOR_CPU_HALFCYCLE { u32 phiSpins__ = 0; do { g2 = read32( ARM_GPIO_GPLEV0 ); } while ( VIC_HALF_CYCLE && ++phiSpins__ < RAD_PHI_WAIT_LIMIT ); if ( phiSpins__ >= RAD_PHI_WAIT_LIMIT ) radPHIWaitTimeouts++;}
#define WAIT_FOR_VIC_HALFCYCLE { u32 phiSpins__ = 0; do { g2 = read32( ARM_GPIO_GPLEV0 ); } while ( CPU_HALF_CYCLE && ++phiSpins__ < RAD_PHI_WAIT_LIMIT ); if ( phiSpins__ >= RAD_PHI_WAIT_LIMIT ) radPHIWaitTimeouts++; }

#define WAIT_FOR_VIC_HALFCYCLE_EDGE {				\
	u32 phiSpins__ = 0;							\
	do { 										\
		g2 = read32( ARM_GPIO_GPLEV0 );			\
	} while ( !(g2 & bPHI) && ++phiSpins__ < RAD_PHI_WAIT_LIMIT );	\
	phiSpins__ = 0;								\
	do { 										\
		g2 = read32( ARM_GPIO_GPLEV0 );			\
	} while ( (g2 & bPHI) && ++phiSpins__ < RAD_PHI_WAIT_LIMIT );	}


#define WAIT_FOR_CPU_HALFCYCLE_EDGE {				\
	u32 phiSpins__ = 0;							\
	do { 										\
		g2 = read32( ARM_GPIO_GPLEV0 );			\
	} while ( (g2 & bPHI) && ++phiSpins__ < RAD_PHI_WAIT_LIMIT );	\
	phiSpins__ = 0;								\
	do {										\
		g2 = read32( ARM_GPIO_GPLEV0 );			\
	} while ( !(g2 & bPHI) && ++phiSpins__ < RAD_PHI_WAIT_LIMIT );	}


// Ceiling on every wait for BA to come back, in C64 cycles.
//
// The VIC claims the bus for a badline or a sprite fetch and gives it back
// within tens of cycles, so any legitimate wait is far under this. An
// unbounded wait is not a stricter version of the same thing -- it is a
// different failure. These spins run inside runFrame(), under the mirror
// flush, so a BA that never returns takes the whole firmware down: the frame
// hook stops running, nothing polls the buttons, and the only way out is
// pulling the Pi's power. That is a real, observed freeze.
//
// On timeout we proceed with the access anyway. That is exactly what this code
// did for most of its life -- the BA re-check in busWriteByteBurst_p1 was
// commented out until recently -- so the worst case is the display glitch we
// already knew how to live with, reached only after milliseconds of a stuck
// line, instead of a machine that has to be unplugged.
#define RAD_BA_WAIT_LIMIT 4000

extern volatile u32 radBAWaitTimeouts;
extern volatile u32 radPHIWaitTimeouts;

#define HANDLE_BUS_AVAILABLE_READ \
	WAIT_UP_TO_CYCLE( busTiming.TIMING_BA_SIGNAL_AVAIL );			\
	g2 = read32( ARM_GPIO_GPLEV0 );							\
	if ( VIC_BA ) {											\
		u32 baSpins__ = 0;									\
		do {												\
			WAIT_FOR_CPU_HALFCYCLE							\
			WAIT_FOR_VIC_HALFCYCLE							\
			RESTART_CYCLE_COUNTER							\
			WAIT_UP_TO_CYCLE( busTiming.TIMING_BA_SIGNAL_AVAIL );	\
			g2 = read32( ARM_GPIO_GPLEV0 );					\
		} while ( !(g2 & bBA) && ++baSpins__ < RAD_BA_WAIT_LIMIT );	\
		if ( baSpins__ >= RAD_BA_WAIT_LIMIT ) radBAWaitTimeouts++;	\
	}


#define HANDLE_BUS_AVAILABLE \
	WAIT_UP_TO_CYCLE( busTiming.TIMING_READ_BA_WRITING );			\
	/* g2 came from the phase wait and may predate BA becoming valid. */	\
	g2 = read32( ARM_GPIO_GPLEV0 );							\
	{ u32 baSpins__ = 0;									\
	while ( !( g2 & bBA ) && baSpins__++ < RAD_BA_WAIT_LIMIT ) {		\
		WAIT_FOR_CPU_HALFCYCLE								\
		WAIT_FOR_VIC_HALFCYCLE								\
		RESTART_CYCLE_COUNTER								\
		WAIT_UP_TO_CYCLE( busTiming.TIMING_READ_BA_WRITING );		\
		g2 = read32( ARM_GPIO_GPLEV0 ); }						\
	if ( baSpins__ >= RAD_BA_WAIT_LIMIT ) radBAWaitTimeouts++; }


__attribute__( ( always_inline ) ) inline 
void busReadByte_p1( register u32 &g2, u16 addr )
{
	register u32 DD = flipByte( ( addr ) & 255 ) << D0;
	SET_GPIO( bLATCH_A0 | bLATCH_A8 | DD | bOE_Dx | bDIR_Dx );
	CLR_GPIO( ( D_FLAG & ( ~DD ) ) );
	#ifdef DOUBLE_GPIO_WRITES
	CLR_GPIO( ( D_FLAG & ( ~DD ) ) );
	#endif

	// R/W must be actively driven HIGH for a read, not merely left to the
	// internal pull-up. DISABLE_ADDRESS_LATCH_AND_BUSTRANSCEIVER ends every
	// access with INP_GPIO_RW(), so this pin is an input on entry and
	// SET_GPIO alone changes the output latch without driving anything.
	//
	// The write path always got this right -- it does OUT_GPIO( RW_OUT )
	// followed by CLR_GPIO( bRW_OUT ) -- which is exactly why writes worked
	// while reads returned $00: with R/W floating the C64 treats the cycle as
	// a write and never drives the data bus. RAD's newer read protocol adds
	// the same OUT_GPIO_RW() for this reason.
	SET_GPIO( bRW_OUT );
	OUT_GPIO( RW_OUT );

	DD = flipByte( ( ( addr ) >> 8 ) & 255 ) << D0;
	CLR_GPIO( bLATCH_A0 );
	#ifdef DOUBLE_GPIO_WRITES
	CLR_GPIO( bLATCH_A0 );
	#endif
	SET_GPIO( DD );												
	CLR_GPIO( ( D_FLAG & ( ~DD ) ) );
	#ifdef DOUBLE_GPIO_WRITES
	CLR_GPIO( ( D_FLAG & ( ~DD ) ) );
	#endif
	CLR_GPIO( bLATCH_A8 );
	SET_BANK2_INPUT

	CLR_GPIO( bOE_Dx );
	HANDLE_BUS_AVAILABLE_READ

	WAIT_UP_TO_CYCLE( busTiming.TIMING_ENABLE_ADDRLATCH );	
	CLR_GPIO( bLATCH_A_OE );
}


__attribute__( ( always_inline ) ) inline
void busReadByte_p2( register u32 &g2, bool selectIOWithGame = false )
{
	// A real SuperCPU leaves the halted 6510's CHAREN/HIRAM/LORAM pins in an
	// all-RAM group, then selects Ultimax only for a genuine $D000-$DFFF I/O
	// cycle. Do not assert /GAME during the preceding VIC half: on a synchronous
	// FPGA host that can remap the VIC's own $D000-$DFFF fetch and corrupt the
	// lower half of a bank-3 bitmap even though the following CPU read works.
	// Select I/O only after PHI2 enters the CPU half, matching RAD's original
	// DMA_READBYTE_P2_IO ordering. Ordinary RAM and mirror traffic never takes
	// this branch.
	WAIT_FOR_CPU_HALFCYCLE
	if ( selectIOWithGame ) CLR_GPIO( bGAME_OUT );
	RESTART_CYCLE_COUNTER
}

// sampleAt: ARM cycles from the PHI2 edge at which the data bus is captured.
// One point does not fit every chip: the VIC and CIAs drive data early enough
// for WAIT_CYCLE_WRITEDATA+20, but the SID drives it near the END of the
// half-cycle -- a 6510 samples on the falling edge -- and reading it at the
// shared point returns open-bus $FF for every register. Callers pass the
// SID-specific point for $D400-$D7FF and the shared one for everything else.
// C128 expansion-bus buffering needs a complete idle bus phase when changing
// from a read to a write. The delay is paid only on that transition.
static u8 busWriteTurnaroundNeeded = 0;
// One idle phase is sufficient on a C64.  The physical C128's buffered port
// passed the burst test reliably only with two phases (kernel147 onward).
// Machine detection sets this to two before the loaded calibration and runtime.
static u8 busWriteTurnaroundPasses = 1;
__attribute__( ( always_inline ) ) inline  
void busReadByte_p3( register u32 &g2, register u8 &x, bool releaseDMA,
                     u32 sampleAt, bool selectIOWithGame = false )
{ 
	WAIT_UP_TO_CYCLE( sampleAt );
	g2 = read32( ARM_GPIO_GPLEV0 );
	x = (u8)( ( g2 >> D0 ) & 255 );
	WAIT_FOR_VIC_HALFCYCLE
	if ( selectIOWithGame ) SET_GPIO( bGAME_OUT );
	RESTART_CYCLE_COUNTER
	DISABLE_ADDRESS_LATCH_AND_BUSTRANSCEIVER( releaseDMA );
	busWriteTurnaroundNeeded = 1;
} 

// Some SID revisions do not present stable OSC3 data until the very end of
// PHI2 high. A 6510 samples there, at the falling edge. Retain the final GPIO
// word observed while PHI2 is still high; the first low sample may already be
// outside the SID's data-hold interval. This follows the edge without guessing
// an ARM-cycle offset and without discarding the last known-valid phase.
__attribute__( ( always_inline ) ) inline
void busReadByte_p3FallingEdge( register u32 &g2, register u8 &x,
	                            bool releaseDMA,
	                            bool selectIOWithGame = false )
{
	u32 lastHigh = g2;
	do
	{
		lastHigh = g2;
		g2 = read32( ARM_GPIO_GPLEV0 );
	} while ( CPU_HALF_CYCLE );
	x = (u8)( ( lastHigh >> D0 ) & 255 );
	if ( selectIOWithGame ) SET_GPIO( bGAME_OUT );
	RESTART_CYCLE_COUNTER
	DISABLE_ADDRESS_LATCH_AND_BUSTRANSCEIVER( releaseDMA );
	busWriteTurnaroundNeeded = 1;
}

__attribute__( ( always_inline ) ) inline  
void busBeginBurstWrites( register u32 &g2 )
{
	c128TrafficBegin();
	CLR_GPIO( bDIR_Dx );
	SET_GPIO( bGAME_OUT );
	SET_GPIO( bRW_OUT );
	OUT_GPIO( RW_OUT );
	SET_BANK2_OUTPUT
	if ( busWriteTurnaroundNeeded )
	{
		for ( u32 turnPass__ = 0; turnPass__ < busWriteTurnaroundPasses;
		      turnPass__++ )
		{
			WAIT_FOR_CPU_HALFCYCLE
			WAIT_FOR_VIC_HALFCYCLE
			RESTART_CYCLE_COUNTER
		}
		busWriteTurnaroundNeeded = 0;
	}

	register u32 DD = 0;
	SET_GPIO( bLATCH_A0 | bLATCH_A8 | DD );
	CLR_GPIO( ( D_FLAG & ( ~DD ) ) );
}

__attribute__( ( always_inline ) ) inline  
void busEndBurstWrites( register u32 &g2 )
{
	DISABLE_ADDRESS_LATCH_AND_BUSTRANSCEIVER( false )
	SET_GPIO( bLATCH_A0 | bLATCH_A8  );

	// RAD-Doom ended a burst with CLR_GPIO( bGAME_OUT ), leaving /GAME asserted
	// -- deliberate there, because its reads expect Ultimax mode and it returns
	// to reading I/O immediately after each blit.
	//
	// Leaving it asserted here would park the C64 in Ultimax between mirror
	// flushes, which happen constantly: the VIC-II's fetches get redirected and
	// the display cannot stabilise. /GAME stays released.
	SET_GPIO( bGAME_OUT );
	c128TrafficEnd();
}

__attribute__( ( always_inline ) ) inline  
void busWriteByteBurst_p1( register u32 &g2, u16 addr, u8 data )
{
	register u32 DD = flipByte( ( addr ) & 255 ) << D0;
	SET_GPIO( DD );
	// Do not expose the low byte through both address latches before the high
	// byte has been clocked; the C128 reacts to that transient address.
	CLR_GPIO( ( D_FLAG & ( ~DD ) ) );

	DD = flipByte( ( ( addr ) >> 8 ) & 255 ) << D0;
	CLR_GPIO( bLATCH_A0 );
	CLR_GPIO( ( D_FLAG & ( ~DD ) ) );
	SET_GPIO( DD );
	DD = ( ( data ) & 255 ) << D0;
	CLR_GPIO( bLATCH_A8 );
	CLR_GPIO( ( D_FLAG & ( ~DD ) ) );
	SET_GPIO( DD );

	if ( ( g2 & bBA ) )
	{
		WAIT_UP_TO_CYCLE( busTiming.TIMING_READ_BA_WRITING );
		g2 = read32( ARM_GPIO_GPLEV0 );
	}
	u8 resync = 0;
	u32 baSpins = 0;
	while ( !( g2 & bBA ) && baSpins++ < RAD_BA_WAIT_LIMIT )
	{
		resync = 1;
		SET_GPIO( bRW_OUT );
		INP_GPIO( RW_OUT );
		SET_GPIO( bLATCH_A_OE );
		WAIT_FOR_CPU_HALFCYCLE
		WAIT_FOR_VIC_HALFCYCLE
		RESTART_CYCLE_COUNTER
		WAIT_UP_TO_CYCLE( busTiming.TIMING_READ_BA_WRITING );
		g2 = read32( ARM_GPIO_GPLEV0 );
	}
	if ( baSpins > RAD_BA_WAIT_LIMIT ) radBAWaitTimeouts++;
	if ( resync )
	{
		OUT_GPIO( RW_OUT );
		CLR_GPIO( bRW_OUT );
	}

	WAIT_UP_TO_CYCLE( busTiming.TIMING_ENABLE_RWOUT_ADDR_LATCH_WRITING - 40 );
	CLR_GPIO( bRW_OUT );
	WAIT_UP_TO_CYCLE( busTiming.TIMING_ENABLE_RWOUT_ADDR_LATCH_WRITING );
	CLR_GPIO( bLATCH_A_OE | bDIR_Dx );
	WAIT_UP_TO_CYCLE( busTiming.TIMING_ENABLE_DATA_WRITING );
	CLR_GPIO( bOE_Dx );
}

__attribute__( ( always_inline ) ) inline  
void busWriteByteBurst_p2( register u32 &g2, bool releaseDMA )
{
	WAIT_FOR_VIC_HALFCYCLE
	RESTART_CYCLE_COUNTER
	WAIT_UP_TO_CYCLE( busTiming.TIMING_DATA_HOLD );
//	WAIT_UP_TO_CYCLE( 60 );
//	SET_GPIO( bLATCH_A_OE | bOE_Dx | bRW_OUT );	
	SET_GPIO( bLATCH_A_OE | bLATCH_A0 | bLATCH_A8 | bOE_Dx | bRW_OUT );	
	//INP_GPIO( RW_OUT );
}








__attribute__( ( always_inline ) ) inline  
void busWriteByte_p1( register u32 &g2, u16 addr, u8 data,
	                  bool selectIOWithGame = false )
{
	if ( busWriteTurnaroundNeeded )
	{
		CLR_GPIO( bDIR_Dx );
		SET_BANK2_OUTPUT
		for ( u32 turnPass__ = 0; turnPass__ < busWriteTurnaroundPasses;
		      turnPass__++ )
		{
			WAIT_FOR_CPU_HALFCYCLE
			WAIT_FOR_VIC_HALFCYCLE
			RESTART_CYCLE_COUNTER
		}
		busWriteTurnaroundNeeded = 0;
	}

	register u32 DD = flipByte( ( addr ) & 255 ) << D0;
	SET_GPIO( bLATCH_A0 | bLATCH_A8 | DD );
	CLR_GPIO( bDMA_OUT | ( D_FLAG & ( ~DD ) ) );
	DD = flipByte( ( ( addr ) >> 8 ) & 255 ) << D0;
	CLR_GPIO( bDIR_Dx | bRW_OUT | bLATCH_A0 );
	SET_GPIO( DD );
	CLR_GPIO( ( D_FLAG & ( ~DD ) ) );
	DD = ( ( data ) & 255 ) << D0;
	CLR_GPIO( bLATCH_A8 );
	SET_GPIO( DD );
	CLR_GPIO( ( D_FLAG & ( ~DD ) ) );

	HANDLE_BUS_AVAILABLE
	WAIT_UP_TO_CYCLE( busTiming.TIMING_ENABLE_RWOUT_ADDR_LATCH_WRITING - 40 );
	OUT_GPIO( RW_OUT );
	CLR_GPIO( bLATCH_A_OE | bDIR_Dx );
	// Select Ultimax only for the target CPU write. In particular, do not pull
	// /GAME low while BA arbitration is still occurring in a VIC half-cycle:
	// FPGA cartridge decoders may register that level and apply it to the VIC's
	// own fetch. Assert it at the CPU-half edge, not as late as data enable: the
	// latter displayed the bitmap correctly on the 64U but then lost a live chip
	// write and froze Test Drive. This gives the decoder the complete CPU half
	// while still excluding every VIC half. p2 releases it after the falling
	// edge where the chip accepts the write.
	WAIT_FOR_CPU_HALFCYCLE
	if ( selectIOWithGame ) CLR_GPIO( bGAME_OUT );
	WAIT_UP_TO_CYCLE( busTiming.TIMING_ENABLE_DATA_WRITING );
	CLR_GPIO( bOE_Dx );
}

__attribute__( ( always_inline ) ) inline  
void busWriteByte_p2( register u32 &g2, bool releaseDMA,
	                  bool selectIOWithGame = false )
{
	WAIT_FOR_VIC_HALFCYCLE
	if ( selectIOWithGame ) SET_GPIO( bGAME_OUT );
	RESTART_CYCLE_COUNTER
	WAIT_UP_TO_CYCLE( busTiming.TIMING_DATA_HOLD );
	DISABLE_ADDRESS_LATCH_AND_BUSTRANSCEIVER( releaseDMA )
}
