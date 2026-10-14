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
#include "../Video/vic_raster.h"
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

	// --- THE MIRROR FLOOR: an inline necessary condition for onRamWrite ---
	//
	// The lowest address the ACTIVE policy can accept, maintained by the sink
	// from the same fields its own shouldMirror() reads. A write below it is
	// provably rejected, so the memory layer can skip the call outright.
	//
	// WHY THIS IS THE BIGGEST ITEM ON THE WRITE PATH. The shipped default is
	// SCPU_OPT_DEFAULT with the Z flag set, so $0000-$01FF is never mirrored --
	// and the TI FOR/NEXT benchmark lives there. MEASURED by wrapping the real
	// CWriteBuffer in a counting sink and running the benchmark on the host
	// core (10,152,416 guest cycles, matching the 10,300,324 of the firmware
	// run to 1.4%, and 1,240,896 sink calls against the "~1.27M writes" this
	// header already records):
	//
	//     $0000-$00FF   926,338   74.65%
	//     $0100-$01FF   265,386   21.39%
	//     $0400-$07FF         6    0.00%
	//     everything else 49,166   3.96%
	//     ------------------------------------------
	//     BELOW $0200  1,191,724   96.04%   <- all rejected, every time
	//
	// 96% of the benchmark's mirror-sink calls were a round trip into
	// out-of-line code that can only ever answer "no". Counted one at a time
	// off the shipped K471 write_buffer.o with the project toolchain at its
	// -O3 (onRamWrite is 0x4c0 bytes, shouldMirror 0xfc and NOT inlined into
	// it), a zero-page store in VIDEO_MODE 0 with delivery on executes:
	//
	//     onRamWrite   0x950..0x974   10   prologue + inlined map-generation
	//                  0xac8..0xad8    5   owner test, m_DeliveryEnabled
	//                  0x9f0..0x9f4    2   set up and `bl shouldMirror`
	//     shouldMirror 0x850..0x91c   12   mode tests, addr < 0x1ff, ret 0
	//                  0x9f8..0xa08    5   recompose the word/bit
	//                  0xcb8..0xcd8    9   m_WritesSkipped++, m_Synced clear
	//                  0xb9c..0xba4    3   epilogue
	//     ------------------------------------------------------------------
	//                                 46   to be told "no"
	//
	// on top of the 50 the reviewer counted for the store itself -- so the
	// sink was ~48% of a plain bank-0 DRAM store, larger than the whole decode
	// it sits behind.
	//
	// WHY A u32 AND NOT A u16: SCPU_OPT_FULL (C128 80-column mode, where the
	// VDC has its own RAM) mirrors NOTHING, and 0x10000 expresses that exactly
	// -- every write skips the sink. A u16 floor of 0xFFFF would still let
	// $FFFF through for no reason.
	//
	// WHY A PLAIN MEMBER AND NOT A VIRTUAL: the hot path has already loaded
	// m_Mirror for its own null test, so this is one ldrh off a pointer that is
	// live in a register. A getter would reintroduce the indirect call this
	// exists to remove. The DEFAULT OF 0 IS THE SAFE ONE -- no address is below
	// it, nothing is ever skipped, and a sink that never sets it behaves
	// exactly as it did before this field existed.
	//
	// shouldMirror() REMAINS THE AUTHORITY. This is a necessary condition for
	// it, derived by the same object from the same inputs in the same function
	// that sets them, in the style of m_WriteNotifyPages above: it removes a
	// call, not a rule. Nothing here may encode a policy the sink does not.
	u32 m_MirrorFloor = 0;
	// Writes the floor rejected inline. The diagnostics in boot.cpp report
	// m_WritesSkipped, which would otherwise lose its single largest component
	// the moment those writes stopped reaching the sink; keeping the count
	// exact costs one load-add-store and is worth 3 of the 46 saved.
	u64 m_MirrorFloorSkipped = 0;
	virtual void flush() = 0;
	virtual u32  pendingBytes() { return 0; }
	// The physical VIC backing map changed after a $D011/$D018/$DD00 write.
	// Implementations may re-queue the newly selected guest-shadow regions;
	// timing-only or test sinks can ignore it.
	virtual void onVICMapChanged( u32 screenBase, u32 graphicsBase,
	                              u32 graphicsLength )
		{ (void)screenBase; (void)graphicsBase; (void)graphicsLength; }

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

// Passive timing sidecar for the physical VIC accesses used by VSP/raster
// programs.  It never samples the bus itself: a $D012 read that the guest was
// already making supplies the raster anchor, and an already-completed
// $D016/$D018 write supplies the event.  Keeping this independent of the HDMI
// VIC replay log lets VIDEO_MODE 0 hardware runs explain where a real access
// landed without changing the access sequence under observation.
// Statistics-only write counters, gated the way SCPU_COUNT_ACCESS in
// Source/SuperCPU/memory_map.h already gates m_Bank0Accesses and
// m_FastAccesses -- and for the same measured reason.
//
// VERIFIED verbatim in the rebuilt w65c816.o at its -Os: writeFast is inlined
// into CW65C816::write8, where m_RamWrites++ is write8+0xc0..0xcc
// (ldr x0,[x20,#40] / add / str) and m_MirrorFloorSkipped++ is
// write8+0xe4..0xec -- 6 ARM instructions on EVERY bank-0 DRAM store, and on
// an in-order A53 two separate serialising load-add-store chains that the
// instruction count alone does not capture. The floor-reject arm is the COMMON
// arm in the TI loop: recomputeMirrorFloor() puts the floor at $0200 with
// ExcludeZPStack and BASIC's hot stores are zero page and the stack, so that
// arm costs 8 instructions today and 5 with the gate off.
//
// WHAT IT IS WORTH, honestly: CHT's only empirically calibrated figure is ~9.3
// ARM instructions per emulated instruction per jiffy, derived from a shipped
// 67->58 change. Six instructions per DRAM store is therefore well under half
// a jiffy of the measured TI 40 even at a write density near 0.5 per emulated
// instruction. This gate exists to establish an instrumentation-free BASELINE;
// it is not a throughput fix and must not be quoted as one.
//
// THE DEFAULT IS 1 IN BOTH BUILDS, deliberately. The shipped firmware stays
// behaviourally identical to K478, and boot.cpp's ramWrites= and
// mirrorSkipped= reports keep their meaning instead of quietly printing zero.
// The A/B is a rebuild with -DSCPU_DIAG_WRITE_COUNTERS=0 staged as its own
// image in build/kernels/, the convention WRITE_DECODE_CACHE and
// kernel8-451.img already follow. It is NOT a config key: a runtime gate would
// be ldrb+cbz+ldr+add+str when on and 2 instructions when off, paying 2 to
// save 3 and adding an unpredictable branch to the hottest store in the
// machine.
//
// THE HOST BUILD PINS IT TO 1 regardless, so the suite is unaffected by
// construction: Tests/SuperCPU/test_memory_map.cpp asserts m_RamWrites equal
// between writeFast and write8 in every banking state, and the integration
// tests assert m_BytesFlushed < m_RamWrites.
//
// The FIELDS stay in both builds. m_RamWrites and m_MirrorFloorSkipped sit in
// the hoisted hot block at the top of this class and in IMirrorSink's first
// cache line; removing either would move the layout the K451 field hoist was
// measured on and push neighbouring loads out of a ldr immediate's reach.
#ifdef SCPU_HOST_BUILD
	#undef SCPU_DIAG_WRITE_COUNTERS
	#define SCPU_DIAG_WRITE_COUNTERS 1
#endif
#ifndef SCPU_DIAG_WRITE_COUNTERS
	#define SCPU_DIAG_WRITE_COUNTERS 1
#endif
#if SCPU_DIAG_WRITE_COUNTERS
	#define SCPU_COUNT_WRITE( c ) ( (c)++ )
#else
	#define SCPU_COUNT_WRITE( c ) ( (void)0 )
#endif

#define SCPU_VSP_TRACE_SIZE 128
#define SCPU_VSP_RASTER_VALID 0x01
#define SCPU_VSP_ANCHOR_BADLINE_MOD 0x02
#define SCPU_VSP_ANCHOR_BADLINE_RANGE 0x04
#define SCPU_VSP_D011_DEN 0x08
// The steal that produced the release stamped in sinceReleaseHost was
// BADLINE-CLASS (>= 20 C64 cycles), not a sprite steal. Read postrel= only
// with this set: a sprite release landing between the badline release and the
// write would otherwise be reported as the badline's, with no way to tell.
// 20 splits the populations cleanly -- a single sprite is 2-3, all eight are
// 3+2N=19, a badline is 43.
#define SCPU_VSP_RELEASE_BADLINE 0x10
#define SCPU_VSP_CATCH_VALID 0x20		// catchDeficit/catchWait were recorded (K499)

// WHAT HELD THE GUEST, for VSPTimingEvent::stallCause. One bit per path that
// can run between two CW65C816::run() calls, set by CSuperCPU::runFrame() at
// the site itself rather than inside the helpers: runFrame is the only place
// that knows what it is about to call, and tagging there keeps the plumbing out
// of flushMirrorsRasterAware(), which takes an IC64Bus and has no memory layer.
#define SCPU_GUEST_STOP_FLUSH      0x01u	// flushMirrorsRasterAware()
#define SCPU_GUEST_STOP_BORDER     0x02u	// the once-per-frame border drain
#define SCPU_GUEST_STOP_SWEEP      0x04u	// CWriteBuffer::resyncSweep()
#define SCPU_GUEST_STOP_SCRUB      0x08u	// display scrub / resyncDisplayed()
#define SCPU_GUEST_STOP_RASTER     0x10u	// a CRADBus::rasterLine() query
#define SCPU_GUEST_STOP_MATRIXDIAG 0x20u	// tryVSPMatrixDiagnostic()
#define SCPU_GUEST_STOP_FRAMEEND   0x40u	// the interval outlived runFrame(),
											// so it contains the frame hook
#define SCPU_GUEST_STOP_MAPREFRESH 0x80u	// a staged VIC-map refresh

struct VSPTimingEvent
{
	u64 hostCycle;          // ARM counter after the physical write completed
	u64 emuCycle;           // accelerator-cycle counter at that point
	u64 c64Cycle;           // rate-independent C64-domain clock
	u64 pacingAnchor;       // pacer's host-clock origin for the same event
	u64 pacingDebt;         // emulated cycles accumulated but not yet settled
	u64 rasterHostCycle;    // host stamp of the most recent guest $D012 read
	u32 sequence;           // monotonic event number; exposes ring overwrites
	u32 hostPerC64;         // nominal ARM clocks per physical C64 cycle
	u16 address;            // canonical $D016 or $D018
	u8  value;              // guest-visible value
	u8  physicalValue;      // value delivered after any map translation
	u8  raster;             // low 8 bits returned by the guest's $D012 read
	u8  factor;             // current emulated-cycles-per-C64-cycle factor
	u8  gamePhase;          // bank-0 $9F88 (Mario's frame/VSP phase byte)
	u8  d016;               // guest VIC shadows at the event
	u8  d018;
	u8  flags;
	// BA cycles charged to the guest between the previous retained event and
	// this one. Mario's DEC/INC $D011 pair is the whole reason this sidecar
	// exists, and the one number that separates "the VSP badline stalled us"
	// from "nothing stalled us" is how many stolen cycles fell between the two
	// writes. Free in space: the struct already pads to 72 bytes, and it still
	// does -- sizeof measured at 72 before and after with the project's own
	// aarch64-none-elf-g++.
	u16 baStallCycles;
	// Stall EVENTS in the same window, beside the cycles. A cycle total cannot
	// say whether one badline or ten sprite steals produced it, and the two have
	// different consequences: baStallSettle's anchor advance bills the pacer for
	// `stolen` WHOLE cycles when the spin entered the first of them part-way
	// through, so it leaves the anchor a fraction of a cycle in the future ONCE
	// PER EVENT. That residual is one-signed late and bounded by the lock's aim
	// phase, so it can only account for K456's +3..+4 hold error if this reads
	// about ten.
	u16 baStallEvents;
	// THE CATCH-UP AT ENTRY (K499): countTarget - entry in ARM cycles, signed,
	// and the wait the catch-up (plus any aim) applied. A positive deficit is
	// an early arrival that was corrected; a negative one is a LATE arrival
	// that nothing corrected, which is the one half of the placement no other
	// field in this record can show -- hostCycle is stamped after the
	// primitive and grid= is a modulo of the integer period. Valid only with
	// SCPU_VSP_CATCH_VALID: the gate did not run for this write otherwise.
	s32 catchDeficit;
	u32 catchWait;
	// Host cycles from the BA RELEASE to this physical write, saturating, with
	// 0xFFFF for "no release close enough to mean anything".
	//
	// The one position in this trace that is MEASURED and not reconstructed.
	// Every other one is rebuilt from rasterHostCycle, the host stamp of the
	// guest's last $D012 read, whose own cycle-in-line nothing measures -- and
	// the stall moved where that read lands, so the ruler changed in the same
	// build as the thing being measured.
	//
	// WHAT IT IS, EXACTLY, because two offsets are easy to drop and they do not
	// cancel: the stamp is taken in baStallSettle right after the pin spin
	// returned, and CRADBus::busStealCycles returns from the sample that read BA
	// HIGH, taken TIMING_BA_SIGNAL_AVAIL (280 ARM of 1368) into the FIRST FREE
	// cycle. So this is (write completion) - (first free cycle + 0.20). The far
	// end carries the write's own in-cycle offset too, around +0.5 to +0.9 from
	// TIMING_ENABLE_DATA_WRITING, since appendVSPTiming stamps after
	// m_C64->write returns. Net: this reads about 0.3 to 0.7 cycle MORE than the
	// whole-cycle difference.
	//
	// Which cycle the first free one IS is the project's own unresolved +/-1:
	// CHT's reference invariant and this file's sibling note say the CPU resumes
	// at 54, Bauer 3.6.3 read literally (BA low 12..54) says 55. So against the
	// invariant's INC at 59 a CORRECT placement reads roughly 380..580 (3.8-5.8
	// cycles) and a write 3.5 cycles late reads roughly 730..930. The two bands
	// do not touch, which is all this has to do -- do not read a point value out
	// of it.
	u16 sinceReleaseHost;
	// WHICH END OF THE PLACEMENT MOVED. Whole PHI cycles waited out on BA by
	// the $D012 read that set this event's raster anchor, and by this event's
	// own write. Both are already counted inside the bus primitives and were
	// discarded there; see gpio_defs.cpp.
	//
	// THE QUESTION THEY SETTLE, which no existing field can. Every placement
	// number in this trace is a difference whose ends are a read tail and a
	// write tail. dprev= is write-tail to write-tail, so its ends share a
	// convention and it is blind to the anchor read entirely; raster=+delta
	// spans both, so a whole-cycle slip in it could have come from either
	// end. K488 measured the INC one whole cycle later on 16 of 21 frames
	// with its sub-cycle phase TIGHTER than the DEC's, which says the
	// variable is WHICH PHI CYCLE and not where in the cycle -- and leaves
	// open whether the bus chose that cycle on the read or on the write.
	// A nonzero anchorBASpins with a zero writeBASpins means the anchor
	// moved and the write did not; the reverse means the write did.
	u16 anchorBASpins;
	u16 writeBASpins;
	// The anchor's GUEST stamp, beside its host stamp, so the trace can print
	// counted (c64Cycle - rasterC64Cycle) next to elapsed (hostCycle -
	// rasterHostCycle) for the SAME event. The seven-cycle Pi-only shortfall in
	// Mario's CMP->DEC span cannot be localised without this pair: every
	// existing field is either host-only or crosses the $D07A factor switch.
	u64 rasterC64Cycle;
	// DISTANCE TO THE LAST BADLINE-CLASS RELEASE, in host cycles, u32 so it
	// does not saturate the way postrel= does.
	//
	// WHY A SECOND RELEASE REFERENCE EXISTS. postrel= measures from the last
	// release of ANY class and is a u16, so it dies at 65535 ARM = 47.9
	// cycles. On K489 the badline charged in Mario's frame sits 472 cycles
	// (7.26 lines) past the INC, so postrel= reads '--' on every event that
	// could have used it and the only physically anchored in-line reference
	// the trace has ever contained was unreachable in practice.
	//
	// WHAT IT BUYS, and it is the last unexamined question about this
	// artifact. A badline's BA goes low about cycle 12 of the line and
	// releases about 54 -- the project's own +/-1 is 54 against Bauer
	// 3.6.3's literal 55. That release is therefore a KNOWN IN-LINE CYCLE
	// measured off the physical pin, which is exactly what every other
	// placement field here lacks: raster= knows WHICH LINE the guest last
	// looked at but never where in it, so whether our DEC lands on line 53
	// or 54 has never been decidable -- and that one bit decides whether
	// YSCROLL 6 can match at all, i.e. whether we trigger the linecrunch
	// Mario is asking for or merely write the register.
	//
	// AND IT SELF-VALIDATES, which matters after five instrument failures.
	// The entry is reconstructible as release - stolen periods, so a
	// 43-cycle steal must put its entry near in-line 11-12. If it does not,
	// the reference is not a badline and nothing derived from it is
	// admissible. Read badstall= with it or do not read it at all.
	u32 sinceBadlineHost;
	// The length of that release, so the validation above can be done from
	// the trace alone rather than against the bastall= of a different event.
	u16 badlineReleaseLen;
	// --- THE DISPLACEMENT FIELDS ------------------------------------------
	// K478 captured Mario's DEC/INC $D011 pair with raster=$FE -- line 254 --
	// where the other 62 events in the same ring read $35, line 53, and the
	// physical VIC's own bit 8 agreed the beam was past 255. 201 lines is
	// ~13,000 C64 cycles and nothing in the CPU path can stall that long:
	// BA_STALL_MAX is 80 cycles with capped=0 and RAD_BA_WAIT_LIMIT is 4000.
	// So either a path OUTSIDE the interpreter stopped the guest for ~12.7ms,
	// or the handler was never late and entered exactly where it was sent.
	// These four fields separate those, and they are read TOGETHER:
	//
	//   stallHost large        a path outside the interpreter held the guest;
	//                          stallCause names it.
	//   stallHost small, d012Reads small, d012First near 254
	//                          nothing stopped us and the handler still entered
	//                          at 254, so the entry itself was displaced.
	//                          irqToWriteHost then says whether we OBSERVED the
	//                          assertion late or it arrived late.
	//   d012Reads large, d012First near 248
	//                          the ordinary structure, and the control: the
	//                          healthy $35 events must show Mario's own poll
	//                          walking from the bottom border down to its
	//                          target line. The K448 trace already shows the
	//                          shape -- the $D018 trio sits 186 lines after the
	//                          $3E anchor and the DEC lands 62 cycles after the
	//                          $35 one -- but not how many reads made it.
	//
	// d012First is deliberately the FIRST value read since the previous
	// retained event, not the last: m_VSPLastRaster is already the last one,
	// printed as raster=, and where the poll STARTED is the half that
	// distinguishes a late entry from a late exit.
	u32 stallHost;          // longest guest-stopped interval, host cycles
	u32 irqToWriteHost;     // observed /IRQ assertion to this write, host cycles
	u16 d012Reads;          // guest $D012 reads since the previous event
	u8  d012First;          // ... and the value the first of them returned
	u8  stallCause;         // SCPU_GUEST_STOP_* bits live in that interval
};

// --- the interrupt-control write ring ---------------------------------------
// WHAT ARMED THE COMPARE, AND WHETHER THE ACKNOWLEDGEMENT CLEARED THE LATCH.
//
// Everything else in this file's forensics answers "what did the handler do
// once it was running". None of it can say what PUT it there. The 64-entry
// interrupt ring proves VECTOR ROTATION (->$0813, ->$0846, ->$0898, ->$0991,
// sixteen each) and a 1228-cycle entry gap at S=$01F9 proves an earlier,
// NESTED entry -- but a pending VIC latch, a CIA, a stale sampled level and a
// compare armed to an unintended value all enter through the same current
// vector and are indistinguishable in those fields.
//
// So this ring records the four registers that DECIDE whether an interrupt
// happens, in the order the guest wrote them, with the writer's PC and a stamp
// in the CORE's clock domain so every entry joins the interrupt ring by cycle:
//   $D012, and $D011 when raster bit 8 is involved -- the compare being armed
//   $D019                                          -- the acknowledgement
//   $D01A                                          -- the enable mask
// Mario arms each rung from the rung before it, so "what rung 3 ($0991) armed,
// and from what line" is a join between pc= here and target= there.
//
// WHY THE $D011 GATE IS ( value | prev ) & $80 AND NOT EVERY $D011 WRITE. The
// VSP DEC/INC pair at $0871/$0874 changes YSCROLL only, so it is excluded by
// construction -- which matters, because the hold between those two writes is
// the quantity the whole VSP instrument exists to measure and an entry is ~20
// ARM instructions of real time inside it. A write that SETS bit 8, CLEARS it,
// or carries it in from a read-modify-write (the $96/$97 pair K478 caught) is
// compare programming, and is kept.
//
// PHYSICAL AND DECODED, KEPT APART DELIBERATELY. reg/value/prev/other are the
// bytes that reached the chip; compare= is OUR composition of the two shadow
// halves. A reader can recompose the compare from value and other and check it
// against ours -- until those agree the armed target is not established, which
// is the discipline K471 learned the slow way on a compare of 318.
//
// BOTH RASTERS ARE ALREADY PAID FOR; neither is a new bus cycle. graster= is
// the guest's own last $D012 read. praster= is the physical $D012 read that
// d011RasterHighBit() already performs when a guest $D011 read comes back with
// bit 7 set. Each carries an AGE, so a stale one reads as stale rather than as
// a measurement -- and these ages are WHOLE C64 CYCLES, not the hundredths
// that the VSP trace's dprev= and stall= print.
#define SCPU_ICTL_RING 128
// The three IRQ bits are the acknowledgement test. Mario acks with $0824 LDA
// #$FF / $0826 STA $D019 -- a PLAIN STORE, not a read-modify-write. If that
// store does not clear the VIC's latch on our write path, the source
// re-asserts immediately and re-enters through whatever vector is current,
// which reproduces BOTH the short entry gap and the nesting with every raster
// compare perfectly correct.
#define SCPU_ICTL_IRQ_BEFORE		0x01u	// cached /IRQ level before the write
#define SCPU_ICTL_IRQ_AFTER			0x02u	// pin sampled fresh after the write
#define SCPU_ICTL_IRQ_AFTER_VALID	0x04u	// ... that sample was taken at all
#define SCPU_ICTL_IRQ_LATE			0x08u	// cached level at the NEXT event
#define SCPU_ICTL_IRQ_LATE_VALID	0x10u
#define SCPU_ICTL_GRASTER_VALID		0x20u
#define SCPU_ICTL_PRASTER_VALID		0x40u
#define SCPU_ICTL_COMPARE_IMPOSSIBLE 0x80u	// composed compare >= rasterLines

struct VICIntCtlWrite
{
	u32 cycles;		// low 32 of the CORE's m_Cycles: joins the interrupt ring
	u32 pc;			// ( PBR << 16 ) | PC of the writer, PAST its operand
	u16 compare;	// our composed m_RasterCompare after this write
	u16 grasterAge;	// whole C64 cycles since the guest's last $D012 read
	u16 prasterAge;	// ... since the last physical bit-8 coherence read
	u16 lateHost;	// host cycles from this write to the IRQ_LATE sample
	u8  reg;		// $11, $12, $19 or $1A
	u8  value;		// what the guest wrote
	u8  prev;		// the shadow byte it replaced
	u8  other;		// the other compare half's shadow, so compare= is checkable
	u8  d01a;		// the enable mask in force at this write
	u8  graster;	// guest-observed $D012 low byte
	u8  praster;	// physically observed $D012 low byte
	u8  flags;
};

// What CC64Memory::interruptContext() packs into bits 40..47, for the fields
// the interrupt ring cannot see for itself. FRESH_SAMPLE is the one bit the
// CORE contributes: whether the level this entry acted on was sampled at this
// instruction boundary or reused from the cache, which is the stale-level
// candidate tested directly instead of bounded by argument.
#define SCPU_IRQCTX_IRQ_LEVEL		0x01u
#define SCPU_IRQCTX_FRESH_SAMPLE	0x02u
#define SCPU_IRQCTX_ACK_HELD		0x04u	// an ack since the previous entry
											// left /IRQ low immediately after
#define SCPU_IRQCTX_ACK_HELD_LATE	0x08u	// ... and at the next event too
#define SCPU_IRQCTX_RASTER_VALID	0x10u
#define SCPU_IRQCTX_COMPARE_IMPOSSIBLE 0x20u
// Also the core's: the line was ALREADY recorded asserted by an earlier
// sample, rather than found asserted at this instruction boundary. With
// IRQ_LEVEL and FRESH_SAMPLE this is the transition information a pin
// trace around CLI and RTI would give, taken from state the core already
// keeps and costing no GPIO read inside the stabiliser:
//   LEVEL=1 FRESH=1 CARRIED=0   newly asserted at this boundary
//   LEVEL=1 CARRIED=1           asserted continuously since before this
//                               instruction -- nothing released it, which
//                               is the acknowledgement-failure signature
//                               seen from the entry side.
#define SCPU_IRQCTX_LINE_CARRIED	0x40u

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
	// Called on every write to $FF00, which is how an REU armed for the
	// $FF00 trigger learns to start. A function pointer rather than an
	// interface: this sits on the RAM write path, and the REU has no business
	// appearing in the memory layer's type graph.
	typedef void ( *FF00Hook )( void *context );
	void setFF00Hook( FF00Hook hook, void *context )
		{ m_FF00Hook = hook; m_FF00HookCtx = context; }
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
	// VIDEO_MODE 1 computes collision latches from the same authoritative
	// shadow frame it presents. While HDMI is active, D01E/D01F reads consume
	// these latches instead of asking the physical VIC, whose DRAM is stale.
	void attachHDMICollisionLatches( volatile u32 *spriteSprite,
	                                 volatile u32 *spriteBackground,
	                                 const bool *hdmiActive )
	{
		m_HDMISpriteSpriteCollision = spriteSprite;
		m_HDMISpriteBackgroundCollision = spriteBackground;
		m_HDMIActive = hdmiActive;
	}
	void clearPhysicalVICCollisionLatches()
	{
		if ( m_C64 )
		{
			(void)m_C64->read( 0xD01E );
			(void)m_C64->read( 0xD01F );
		}
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
	// Read-decode input: SCPU128 mode declines both ROM arms so $FF00-$FF04
	// and the $D5xx/$D6xx windows keep reaching the real 8722 through read8().
	void setSCPU128Mode( bool on )
		{ m_SCPU128Mode = on; rebuildReadDecode(); rebuildWriteDecode(); }

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
	// The rebuild is not optional: m_ROMShadow is a read-decode input and
	// three of the four table arms are relative to it.
	void setROMShadow( u8 *bank1 )
		{ m_ROMShadow = bank1; rebuildReadDecode(); rebuildWriteDecode(); }

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

	// EDIT 3 of 3, lands with its three sibling hunks and the two c64_memory.cpp
	// hunks. THE BANK-0 DECODE INPUTS, DELIBERATELY HERE.
	//
	// These four used to be declared after the 64KB m_RAM and the three ROM
	// images. Measured with gdb ptype /o on kernel8.elf, that put m_BankMode at
	// 86122 (+0x1506A), m_DOSExtState at 86392 (+0x15178), m_ROMShadow at 86440
	// (+0x151A8) and m_SCPU128Mode at 86448 (+0x151B0). GCC picks two different
	// 4KB bases to reach them, so every bank-0 access pays two `add #imm, lsl
	// #12` materializations it does not need -- busRead8+0x80 and +0xbc,
	// stepInner+0x286c and +0x28f8 -- and the four sit in THREE 64-byte lines
	// spanning 326 bytes (m_ROMShadow and m_SCPU128Mode share one). A fourth
	// line, the object header, is already consulted for m_BootmapActive and
	// m_KernalShadowBase, in a loop that is concurrently streaming m_RAM, the
	// 64KB bank-1 shadow and the region table.
	//
	// Beside m_KernalShadowBase and m_BootmapActive they share that one header
	// line and every one is a plain immediate again: the rebuilt object reads
	// m_BankMode [x21,#13], m_SCPU128Mode [x21,#14] and m_ROMShadow [x21,#16],
	// with m_DOSExtState at +24, and both adds are gone.
	//
	// If you add a decode input to readFast or writeFast, it belongs HERE, not
	// next to whatever it is logically related to. Keep CC64Memory's
	// mem-initializer list (c64_memory.cpp) in this same order or -Wreorder
	// fires. Nothing depends on the layout: no offsetof, no memcpy of this
	// object, no test asserts an offset. But this DOES change the class's
	// in-memory layout, so every object that includes this header must be
	// rebuilt -- a stale object would corrupt memory with no diagnostic.
	u8    m_BankMode;		// cached c64BankMode() for port/GAME/EXROM
	bool  m_SCPU128Mode;
	u8   *m_ROMShadow;
	const bool *m_DOSExtState = 0;

	// THE WRITE PATH'S OWN HOT FIELDS, HOISTED FOR THE SAME MEASURED REASON.
	//
	// CW65C816::write8 is the one out-of-line copy of the whole bank-0 store
	// (0x1e8 bytes before this hunk), and storing one byte into bank-0 DRAM
	// executes 53 of its own instructions. Three of the fields it touches used to
	// be declared AFTER the 64KB m_RAM and the three ROM images, which put
	// m_RamWrites and m_Mirror ~86KB into the object -- read straight off the
	// control build's disassembly, which emits
	//
	//   c0:  add x0, x20, #0x10, lsl #12
	//   c8:  ldr x1, [x0, #21024]        <- m_RamWrites
	//   d8:  ldr x0, [x0, #21288]        <- m_Mirror
	//
	// (the exact displacements move whenever a field is added above them; the
	// K456 object had 20944 and 21208 for the same two loads, and the wasted
	// `add` is what does not move)
	//
	// on EVERY RAM write purely to address them. One wasted instruction, and
	// worse: with m_WriteBase at +176 and m_WriteNotifyPages at +304, a single
	// store touched FOUR separate 64-byte lines of CC64Memory metadata -- two of
	// them 86KB from the other two -- in a loop that is concurrently streaming
	// 64KB of m_RAM, the 64KB bank-1 shadow and the region table through a 32KB
	// L1D. The read path never had this problem: it needs no counter and no sink,
	// so EDIT 3's hoist above already left it with one line plus its table entry.
	//
	// Declared here they land inside the object's first 64-byte line beside
	// m_BankMode, m_ROMShadow and m_DOSExtState, every reference is a plain ldr
	// immediate again, and a bank-0 store touches TWO lines: this one and the one
	// holding its m_WriteBase entry.
	//
	// m_Mirror is public only because of that. setMirrorSink() remains the only
	// thing that assigns it -- nothing outside this class names it at all -- and
	// it sits beside m_RAM, m_BankMode and m_WriteBase, which are already public
	// for tracing and tests.
	//
	// Same obligations as EDIT 3: keep the mem-initializer list in
	// c64_memory.cpp in THIS order or -Wreorder fires, nothing may take
	// offsetof() or memcpy() this object, and every TU that includes this header
	// must be rebuilt -- a stale object would corrupt memory with no diagnostic.
	IMirrorSink *m_Mirror;
	u64  m_RamWrites;		// statistics; reported by boot.cpp, asserted by Tests/
	u16  m_WriteNotifyPages = 0xFFFF;	// see the note on m_WriteBase

	// --- bank-0 read decode, resolved once per mapping change -------------
	// MEASURED off the K449-era object with the project toolchain: ONE
	// BASIC-ROM opcode fetch is three out-of-line calls deep. stepInner+0x890
	// is `bl fetch8` (confirmed by its R_AARCH64_CALL26 relocation),
	// fetch8+0x1c is `bl busRead8`, busRead8+0x1c is
	// `bl preciseTimelineBeforeAccess`, and busRead8 is 0x188 bytes of
	// re-derived memory map. The opcode fetch -- the single most frequent
	// access in the machine -- was never on the inline path.
	//
	// It is always the SAME derivation and its answer is always "page P of
	// bank 0 is backed by this 4K block": REG_RAM by m_RAM + P*4K, REG_BASIC
	// by m_ROMShadow + P*4K, REG_KERNAL by m_ROMShadow + m_KernalShadowBase +
	// (P*4K - $E000). So resolve it when the mapping CHANGES and let a fetch
	// be a load. Entries are per-PAGE rather than biased whole-address bases
	// so every pointer here points inside the object that backs it: a
	// whole-address base for the KS KERNAL window would be m_ROMShadow -
	// $8000.
	//
	// A null entry means "ask read8()", which is unchanged and remains the
	// authority. The table is therefore a PURE ACCELERATOR -- every byte it
	// answers must equal read8()'s byte -- and
	// Tests/SuperCPU/test_memory_map.cpp's
	// bank0_readfast_matches_read8_in_every_banking_state enumerates exactly
	// that over all 32 PLA modes x both ROM-shadow states x both KERNAL
	// shadow bases x SCPU128 on and off. Dropping the rebuild call from that
	// test's inner loop makes it fail in the thousands (verified: 6572 of
	// 682332 checks), which is the point: a missed invalidation must not be
	// able to hide.
	//
	// EVERY INPUT, and where each is handled:
	//   m_BankMode          rebuilt at the end of updateBankMode(), the only
	//                       assignment to it in Source/ -- so $00/$01 writes
	//                       (write8), reset() and the constructor are covered.
	//                       GAME/EXROM only ever reach m_BankMode through that
	//                       same function, so the table is exactly as stale as
	//                       m_BankMode already was, never more.
	//   m_ROMShadow         setROMShadow()
	//   m_SCPU128Mode       setSCPU128Mode()
	//   m_KernalShadowBase  CSuperCPURegisters::applyKernalShadow() writes it
	//                       through a raw u32* whenever the register gate
	//                       opens or closes; that function now calls its map
	//                       hook. THE HOOK IS NOT SELF-INSTALLING: every owner
	//                       that calls trackKernalShadow() must also call
	//                       setMapHook(), or this table goes stale with no
	//                       diagnostic. CSuperCPU::initialize() does; so must
	//                       any test fixture that wires the two by hand and
	//                       then reads through readFast().
	//   c64RegionTable      built by c64BankingInit(), which this class's
	//                       constructor calls before its own updateBankMode().
	//                       Deterministic, so a later rebuild cannot change it.
	//   m_RAM               an in-object array, so its address is fixed for the
	//                       object's lifetime. This table therefore makes
	//                       CC64Memory unsafe to COPY: a copy would inherit
	//                       pointers into the original. Nothing copies it
	//                       today; keep it that way.
	//
	// TWO INPUTS ARE DELIBERATELY NOT IN THE TABLE, because nothing notifies
	// us when they change and a stale answer would be a silent wrong byte:
	//   m_BootmapActive  CSuperCPURegisters drives it through a raw bool* and
	//                    the SuperCPU boot code depends on a $D0B6 write
	//                    taking effect on the very next FETCH. readFast keeps
	//                    testing it live and the table describes the
	//                    bootmap-off map only, so a bootmap change needs no
	//                    invalidation at all.
	//   *m_DOSExtState   a bool inside CSuperCPURegisters, flipped by three
	//                    $D0Bx writes with no path back here.
	//                    dosExtensionMapsBank1() stays FIRST and live, so the
	//                    table never has to describe a DOS window either.
	// Two instructions each to make the two inputs with a catastrophic failure
	// mode structurally un-missable. Anything added to readFast's decode in
	// future belongs in ONE of these two lists, explicitly.
	//
	// BISECTION: flip READ_DECODE_CACHE to false and every entry stays null,
	// so every bank-0 read goes through read8() -- bit-identical, and slower
	// than both this and K449, so it is a CORRECTNESS bisect, not a
	// performance A/B. For a performance A/B, boot
	// build/kernels/kernel8-449.img. m_ReadDecodeEnable is read only inside
	// rebuildReadDecode(), so flipping it at run time does nothing until the
	// next rebuild.
	static const bool READ_DECODE_CACHE = true;
	bool m_ReadDecodeEnable = READ_DECODE_CACHE;
	const u8 *m_ReadBase[ 16 ] = { 0 };

	// --- bank-0 write decode, resolved once per mapping change ------------
	// The read table's sibling, and deliberately the same shape: per-PAGE 4K
	// block pointers, a null entry meaning "ask write8()", rebuilt when the
	// mapping changes. Read the note on m_ReadBase first; everything that is
	// the same there is the same here and is not repeated.
	//
	// MEASURED off the SHIPPED K451 object with the project toolchain. The
	// whole bank-0 write path exists as ONE out-of-line copy,
	// _ZN8CW65C8166write8Ejh, 0x1dc bytes, reached from 18 writeM sites and 28
	// rmwWrite sites plus four further inlined copies of writeFast in .text
	// (counted as 4 CALL26/JUMP26 each to CC64Memory::write8,
	// noteRAMWriteSideEffects and noteMirrorWrite, and 5 inlined c64RegionTable
	// sites). Storing ONE byte into bank-0 DRAM through that copy executes 60
	// of its own instructions -- 76 with the noteRAMWriteSideEffects body
	// below, counted one instruction at a time off a rebuild of the pre-edit
	// object -- and two blocks of it are pure decode:
	//
	//   +0xac..+0xcc  9 instructions re-deriving the region EVERY write:
	//                 ldrb m_BankMode / adrp c64RegionTable / add / sxtw /
	//                 ubfiz / add / ldrb / cmp #REG_IO / b.ne. The answer is
	//                 always "page P of bank 0 is REG_IO or it is not", and it
	//                 only changes when m_BankMode does.
	//   +0x130..0x158 the UNCONDITIONAL tail into noteRAMWriteSideEffects: 4
	//                 argument movs and a tail branch, then a 16-instruction
	//                 body (frame, the far `add x4,x0,#0x18,lsl#12` + ldr for
	//                 m_SpritePtrBase, two compares, frame) that answers "no"
	//                 for every write outside the sprite-pointer row and
	//                 $FF00. 21 instructions to be told nothing happened.
	//
	// So: resolve the region once per mapping change, and keep a 16-bit page
	// bitmap of the only pages those two compares can ever fire in. Both
	// collapse to a load and a branch. m_RamWrites, the mirror-sink call and
	// the store itself are untouched -- this edit removes decode, not work.
	//
	// WRITES NEVER LAND IN ROM: they fall through to the DRAM underneath (see
	// c64WriteIsIO). So every region except REG_IO resolves to the same byte of
	// m_RAM, which is why this table has ONE arm where the read table has
	// three, and why m_ROMShadow, m_SCPU128Mode and m_KernalShadowBase -- three
	// of the read table's inputs -- are not inputs here.
	//
	// A STALE WRITE ENTRY IS WORSE THAN A STALE READ ENTRY and that asymmetry
	// is the whole reason the list below is exhaustive: a stale read returns one
	// wrong byte and the next read after the rebuild is right again, while a
	// stale write STORES a byte into bank-0 DRAM that belonged to a VIC, SID,
	// CIA or colour-RAM register. The wrong byte is permanent and the register
	// write never happens at all.
	//
	// EVERY INPUT TO m_WriteBase, and where each is handled:
	//   m_BankMode     rebuildWriteDecode() at the end of updateBankMode(),
	//                  beside rebuildReadDecode() and for the same reason: that
	//                  is m_BankMode's only assignment in Source/, so $00/$01
	//                  writes, reset() and the constructor are all covered.
	//   c64RegionTable built by c64BankingInit() before the constructor's own
	//                  updateBankMode(); deterministic.
	//   m_RAM          an in-object array, fixed for the object's lifetime.
	//                  Same consequence as the read table: do not COPY
	//                  CC64Memory.
	// setROMShadow() and setSCPU128Mode() call the write rebuild too. Neither is
	// an input today -- the paragraph above says why -- but they are wired
	// anyway because the call is 16 iterations off every hot path and the
	// failure mode if a future arm does depend on them is silent memory
	// corruption rather than a visibly wrong byte.
	//
	// EVERY INPUT TO m_WriteNotifyPages:
	//   m_SpritePtrBase  rebuildWriteNotify() at the end of
	//                    updateSpritePtrBase(), which is m_SpritePtrBase's only
	//                    assignment in Source/ apart from reset()'s
	//                    0xFFFFFFFF, and reset() then calls
	//                    updateSpritePtrBase() anyway. Everything that can move
	//                    the row -- $DD00, $DD02, $D011 BMM, $D018, $D015,
	//                    setRAMUnderIOAccessible(), setUnderIORelocate(),
	//                    initializeC64CIA2() -- reaches it through that one
	//                    function, so this is one site and not nine.
	// NOT an input, deliberately: m_FF00Hook. Bit 15 is set UNCONDITIONALLY, so
	// an REU fitted or removed at any time needs no invalidation, and the
	// 0xFFFFFFFF "no reachable pointer row" sentinel folds into the same bit
	// ((0xFFFFFFFF >> 12) & 15 == 15) instead of needing a special case. Page $F
	// writes therefore keep paying the out-of-line call; they are rare (Mario's
	// STA $FFFE is one) and correctness is worth more than they cost.
	//
	// The default is 0xFFFF, not 0: m_SpritePtrBase is NOT initialised by the
	// constructor (only by reset()), so until a reset every page notifies.
	// Staleness in the conservative direction is slow; the other direction is
	// every sprite wearing its last shape, which this codebase has already
	// shipped once when writeFast and write8 drifted apart over exactly this
	// check.
	//
	// BISECTION: flip WRITE_DECODE_CACHE to false and every entry stays null,
	// so every bank-0 write goes through write8() -- bit-identical and slower
	// than K451, so it is a CORRECTNESS bisect, not a performance A/B. For a
	// performance A/B boot build/kernels/kernel8-451.img. m_WriteDecodeEnable is
	// read only inside rebuildWriteDecode(), so flipping it at run time does
	// nothing until the next rebuild. m_WriteNotifyPages = 0xFFFF by hand is the
	// matching bisect for the notify half.
	//
	// Tests/SuperCPU/test_memory_map.cpp's
	// bank0_writefast_matches_write8_in_every_banking_state is the guard, and it
	// is the write-side twin of the read table's: deleting the rebuild call from
	// its inner loop makes it fail in the thousands.
	//
	// The two halves bisect INDEPENDENTLY on purpose, because they are two
	// separate claims: WRITE_DECODE_CACHE false restores the per-write
	// c64RegionTable derivation, WRITE_NOTIFY_CACHE false restores the
	// unconditional call into noteRAMWriteSideEffects. Either alone is
	// bit-identical to K451 on that half.
	static const bool WRITE_DECODE_CACHE = true;
	static const bool WRITE_NOTIFY_CACHE = true;
	bool m_WriteDecodeEnable = WRITE_DECODE_CACHE;
	u8  *m_WriteBase[ 16 ] = { 0 };
	// m_WriteNotifyPages is declared in the hot block at the top of the class,
	// beside m_Mirror and m_RamWrites; the note there has the measurement.

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
	// bootmap, I/O and open bus. Those paths already do enough work that a
	// call is noise, and keeping them out of the header keeps this small
	// enough to actually inline.
	//
	// The BASIC and KERNAL shadows are here too, because they have no
	// complexity at all: read8() answers both with a single load out of bank 1
	// and charges nothing -- no bus access, no stretch, no counter. Reaching
	// that load through the call cost 47 executed ARM instructions, four
	// stp/ldp pairs and a 96-byte frame in a 2372-byte function (counted in
	// the shipped -O3 object), all to re-derive the two range tests, the
	// bootmap tests and the c64RegionTable lookup this inline path has just
	// done. A BASIC FOR/NEXT loop runs almost entirely out of ROM, so that
	// cold call was the COMMON case: ~116 ARM instructions per ROM byte
	// against ~70 for a RAM byte.
	//
	// Both arms decline unless the bank-1 shadow exists -- read8() then serves
	// the stock image out of m_Basic/m_Kernal -- and unless SCPU128 mode is
	// off, because there $FF00-$FF04 is the 8722's own always-visible register
	// window sitting INSIDE the KERNAL page and has to keep going out to the
	// real bus with its I/O accounting.
	__attribute__((always_inline)) inline u8 readFast( u16 a )
	{
		if ( dosExtensionMapsBank1( a ) )
			return m_ROMShadow[ a ];
		// The same two live gates in the same order as before -- see the note
		// on m_ReadBase for why those two and only those two stayed live --
		// and then one load instead of the region derivation. RAM, BASIC and
		// KERNAL all collapse into this single arm; the three arms and the
		// c64RegionTable lookup they shared now live in rebuildReadDecode().
		if ( a > 1 && !m_BootmapActive )
		{
			const u8 *page = m_ReadBase[ a >> 12 ];
			if ( page )
				return page[ a & 0x0FFF ];
		}
		return read8( a );
	}

	// Settle lag for the canonical VIC-II register page when it is physically
	// visible; 0 otherwise. This deliberately excludes the SuperCPU registers
	// in the VIC mirrors and all CIA/IEC traffic: exact intra-opcode placement
	// is being introduced for raster/VSP writes without perturbing the serial
	// timing that hardware testing already validated. HDMI-exclusive mode has
	// no physical VIC access to place.
	//
	// Reads and writes both report lag 1: the primitive lands an access in
	// cycle N when it is entered during cycle N-1 and resyncs to the falling
	// edge that begins N. A turnaround write used to report 2 because its K243
	// idle pass FOLLOWED the resync and displaced the drive by a cycle;
	// radDirectWrite now pays that pass inside the resync itself (the one idle
	// cycle every read->write sequence has -- an RMW's dummy-write cycle), so
	// the drive is on the entry's next cycle either way, and a lag of 2 here
	// would land a turnaround STA one cycle EARLY.
	// K425 BISECT: plain {0,1,2} settle lag, the exact K420 form. The bias-8
	// aim encoding is suspended -- four environmental theories died against
	// the K420-clean/K421-corrupt boundary, which leaves the K421 delta itself
	// under test. The aim members stay but are disconnected.
	__attribute__((always_inline)) inline u32 preciseVICCycleAccess( u16 a,
	                                                               bool write ) const
	{
		if ( ( a & 0xFFC0 ) != 0xD000 ) return 0;
		if ( m_HDMIActive && *m_HDMIActive ) return 0;
		if ( write )
			// K504: lag 1 only inside the raster window, where the write's K243
			// pass is paid inside its own resync (setWriteTurnaroundMinimal) and
			// the drive lands on the entry's next cycle. Every other turnaround
			// write keeps K500's lag 2, which pre-pays the pass that still
			// follows its resync, so nothing outside the window moved.
			return c64WriteIsIO( a, m_BankMode )
			     ? ( ( m_C64 && m_C64->writeTurnaroundPending()
			           && ( !m_VSPPlacementK501 || !rasterCriticalWindowOpen() ) ) ? 2u : 1u )
			     : 0;
		return c64MapRead( a, m_BankMode ) == REG_IO ? 1u : 0;
	}

	// Diagnostic placement calibration, cycles. Positive lands the access
	// LATER. The observable is the horizontal offset of a VSP screen; the
	// value that zeroes it is the machine's measured end-to-end constant.
	// IS THE GUEST INSIDE A CYCLE-COUNTED RASTER WINDOW RIGHT NOW.
	//
	// Raster code reads $D012, then counts cycles to a VIC write whose exact
	// placement is the whole point. Between those two instants the guest's
	// timeline is load-bearing and any pause we insert comes straight off
	// the window: one C64 cycle of pause is one VIC c-access is one
	// character column of horizontal displacement.
	//
	// WHY THIS EXISTS, measured rather than supposed. runFrame slices the
	// guest 128 times per emulated frame, so on NTSC a slice boundary falls
	// every 17095/128 = 134 C64 cycles. Mario's $D012-read-to-$D011-write
	// window is about 62 cycles, so a boundary lands inside it on roughly
	// 62/134 = 46% of frames. Camera measurement of the real machine over
	// 103 frames, parked, differential between a row above the manipulated
	// line and a row below it: 50% of frames displaced by 1-3 columns, 50%
	// exactly correct. 46% predicted against 50% observed, and the SHAPE
	// matches too -- discrete and binary, which a continuous timing error
	// cannot produce. A boundary either lands in the window or it does not.
	//
	// Deliberately NOT a general "is the guest doing something timing
	// sensitive" oracle. It asks one narrow question with a hard bound, and
	// callers must still bound their own consecutive deferrals: a program
	// that polls $D012 forever would otherwise postpone mirroring for ever.
	// THE MIRROR SCHEDULER'S WINDOW (K507): the raster window above, OR an
	// /IRQ assertion observed within the last 300 C64 cycles at 1MHz. The
	// narrow window opens at the $D012 read, but a stabilised raster handler
	// (Mario, 64 cycles from the IRQ to that read) can have a mirror
	// installment land BETWEEN the interrupt and the read: the guest resumes
	// late, the read sees a later line, the absorber cannot pay back more
	// than one cycle, and the whole frame lands 4-5 cells off (seen on the
	// card with MIRROR_DISPLAY_BYTES 1024). This is only for deferring
	// mirror work -- placement keeps the narrow definition -- and the
	// scheduler's consecutive-deferral limit still bounds it.
	bool mirrorDeferWindowOpen() const
	{
		if ( rasterCriticalWindowOpen() )
			return true;
		if ( !m_C64 || !m_IRQAssertHost || effectiveCIAFactor() != 1 )
			return false;
		const u64 now = m_C64->hostCycles();
		const u64 perC64 = m_HostPerEmuQ16Slow >> 16;
		return now >= m_IRQAssertHost
		    && ( now - m_IRQAssertHost ) < 300 * perC64;
	}

	bool rasterCriticalWindowOpen() const
	{
		// The anchor is only meaningful at 1MHz: raster code that counts
		// cycles runs there, and at factor 20 the same count spans a
		// twentieth of the time so the window closes before a slice.
		if ( !m_VSPHaveRasterAnchor || effectiveCIAFactor() != 1 )
			return false;
		// Same 200-cycle bound the catch-up uses, and for the same reason:
		// past it the read is stale and nothing is counting from it. Three
		// NTSC lines, comfortably over Mario's 62 and far under a frame.
		const u64 now = c64Now();
		return now >= m_VSPLastRasterC64
		    && ( now - m_VSPLastRasterC64 ) <= 200;
	}

	void setVSPAim( int readCycles, int writeCycles )
	{
		m_VSPAimRead  = (s8)readCycles;
		m_VSPAimWrite = (s8)writeCycles;
	}

	// A write landed on the active screen's sprite pointers.
	//
	// There are TWO write paths -- writeFast here and CC64Memory::write8 out of
	// line -- and both reach this. They are not allowed to diverge, so this is
	// the one copy: they HAD diverged, with the out-of-line path missing the
	// log append entirely, which is invisible until something multiplexes
	// sprites and then shows up as every sprite wearing its last shape.
	//
	// The pointers bypass the mirror queue and land on the real bus NOW, like a
	// VIC register. Double-buffered sprite animation flips a pointer in vblank
	// and immediately starts redrawing the block the flip retired; a flip that
	// arrives even a fraction of a frame late leaves the physical VIC showing
	// the very block the game is busy overwriting -- seen as sprites flickering
	// between correct and garbage at animation rate. The queued copy still
	// flushes later; coalescing holds the latest value, so re-sending is
	// harmless. Same-value rewrites are a no-op on hardware and are skipped.
	inline void noteSpritePointerWrite( u16 a, u8 old, u8 v )
	{
		m_PtrRowWrites++;
		if ( old == v ) return;

		// The VALUE delivered may differ from the value stored: a bank-3
		// pointer selecting a shape block under the I/O window is remapped to
		// the block's relocated copy, the only place the real VIC can actually
		// be fed from. See relocPointerValue().
		if ( m_C64 )
		{
			const u8 delivered = relocPointerValue( v );
			bool physical = true;
			const u16 target = vicPhysicalAddress( a, physical );
			if ( physical )
			{
				if ( m_RAMUnderIOAccessible
				     && target >= 0xD000 && target <= 0xDFFF )
					m_C64->writeRAM( target, delivered );
				else
					m_C64->write( target, delivered );
			}
		}
		updateHotShapeBlocks();

		// Timestamp it. Sprite pointers are RAM, so otherwise they are carried
		// only by the once-per-frame snapshot, which applies ONE value to the
		// whole frame and silently breaks every sprite multiplexer. Contrast
		// colour RAM, deliberately NOT timestamped, where one 1000-cell fill
		// would consume half the ring.
		//
		// CHANGES only. An unchanged write leaves RAM alone, so the replay --
		// which seeds from RAM and then follows changes -- already holds the
		// right value and has nothing to learn. Logging them anyway would let a
		// program that rewrites a pointer in a tight loop fill the 2048-entry
		// ring, force a resync, and lose every raster effect in that frame.
		appendVICLog( (u16)( VIC_LOG_REG_SPRPTR + ( a & 7 ) ), v );
	}

	// How the memory layer learns the CPU's PC and cycle without depending on
	// the CPU's type or paying anything per access.  A function pointer called
	// ONLY from the watched-write and $D012-read hooks, which between them run
	// a handful of times a frame; m_Bank0 cannot include the core's header and
	// the core must not publish its PC per instruction, so a probe called on
	// the rare path is the only arrangement that costs nothing.  Null when no
	// 65816 is installed (the 6502 core path and most host tests), and every
	// caller handles that.
	//
	// Returns ( cycles << 32 ) | ( ( PBR << 16 ) | PC ).
	typedef u64 ( *CPUProbeFn )( void *ctx );
	void setCPUProbe( CPUProbeFn fn, void *ctx )
	{
		m_CPUProbe = fn;
		m_CPUProbeCtx = ctx;
	}

	// Arm or disarm the watched-RAM write ring.  addr > 0xFFFF disables a slot;
	// the sentinel is deliberately out of range rather than 0, because $0000 is
	// a real and watchable address.  Two slots so one hardware run can cover
	// the byte that stopped AND the byte that feeds it, which is what makes a
	// single capture capable of closing the question instead of asking for
	// another run.
	void setRAMWatch( u32 slot0, u32 slot1 )
	{
		m_WatchAddr[ 0 ] = slot0;
		m_WatchAddr[ 1 ] = slot1;
		rebuildWriteNotify();
	}

	// THE ADDRESS COMPARE LIVES IN HERE, not at the call site, and that is a
	// measured decision.  With the two compares written at the call site GCC
	// inlined them -- and the probe's indirect call with them -- into all four
	// of writeFast's inline copies inside stepInner: stepInner went 0x6334 ->
	// 0x63dc, 42 instructions, on the store path.  Called unconditionally from
	// a function that only runs for notify pages, and noinline so the decision
	// cannot be re-made by a future optimiser, each of those sites costs three
	// argument movs and a bl instead.  A disarmed slot holds 0x10000, which a
	// u16 address can never equal, so this returns after one compare.
	__attribute__((noinline)) void noteWatchedWrite( u16 a, u8 old, u8 v )
	{
		if ( a != m_WatchAddr[ 0 ] && a != m_WatchAddr[ 1 ] ) return;
		const u64 probe = m_CPUProbe ? m_CPUProbe( m_CPUProbeCtx ) : 0;
		WatchedWrite &e = m_WatchRing[ m_WatchPos++ % SCPU_WATCH_RING ];
		e.pc = (u32)probe;
		e.cycles = (u32)( probe >> 32 );
		e.addr = a;
		e.value = v;
		e.old = old;
		m_WatchWrites++;
		if ( old != v ) m_WatchChanges++;
	}

	// Everything a plain RAM store has to do BESIDES storing.
	//
	// Both write paths call this and neither carries its own copy. That is not
	// tidiness: writeFast and write8 have already drifted apart once, over this
	// very sprite-pointer handling, and the divergence was invisible until
	// something multiplexed sprites. One copy, two callers.
	inline void noteRAMWriteSideEffects( u16 a, u8 old, u8 v )
	{
		if ( ( (u32)a & ~7u ) == m_SpritePtrBase )
			noteSpritePointerWrite( a, old, v );

		// An REU can be armed to begin a transfer on the next write to $FF00.
		// It triggers on the BUS CYCLE, not on a value change, so this fires
		// even when the byte is unchanged -- same-value elimination upstream
		// would otherwise swallow the trigger and the transfer would never
		// start. The address compare is one predictable instruction and the
		// hook is null unless a unit is actually fitted.
		if ( a == 0xFF00 && m_FF00Hook ) m_FF00Hook( m_FF00HookCtx );

		// Unconditional, with the compare inside the callee -- see the note on
		// noteWatchedWrite for the 42 instructions that buys back. This runs
		// only for pages m_WriteNotifyPages has armed, which with both slots
		// disarmed is the set it was armed for before this change.
		noteWatchedWrite( a, old, v );
	}

	// The mirror sink, with the floor applied. ONE copy and three callers --
	// writeFast and both of write8's RAM arms -- for exactly the reason
	// noteRAMWriteSideEffects has one copy: writeFast and write8 have already
	// drifted apart once, and
	// Tests/SuperCPU/test_memory_map.cpp's
	// bank0_writefast_matches_write8_in_every_banking_state only catches that
	// if both go through the same code.
	//
	// BISECTION: force the floor to 0 (CWriteBuffer::recomputeMirrorFloor, or
	// MIRROR_FLOOR 0 in the config) and every write reaches the sink again,
	// bit-identical to K471 and slower. That makes it a correctness bisect,
	// not a performance A/B.
	// SPLIT, and the split is the point. At -Os GCC emitted this whole thing
	// out of line with its own 7-instruction frame, which left the floor
	// reject costing 16 instructions when the test itself is four. Inlining
	// ALL of it instead would drag the accepted arm -- the indirect blr plus
	// all of noteMirrorWrite, 38 instructions in the measured object -- into
	// write8, memory_map and both c64_memory.cpp arms, paying L1I for the 4%
	// of writes that are accepted.
	//
	// So the necessary condition inlines and the work does not. Same shape as
	// noteWatchedWrite above, and for the same reason. MEASURED off the
	// shipped w65c816.o, zero-page store, floor reject: 52 instructions
	// (6 dispatch + 46 inside onRamWrite/shouldMirror) -> 8.
	__attribute__((always_inline)) inline void mirrorRamWrite( u16 a, u8 v )
	{
		if ( !m_Mirror ) return;
		if ( (u32)a < m_Mirror->m_MirrorFloor )
		{
			SCPU_COUNT_WRITE( m_Mirror->m_MirrorFloorSkipped );
			return;
		}
		mirrorRamWriteAccepted( a, v );
	}
	// Deliberately out of line; see mirrorRamWrite.
	__attribute__((noinline)) void mirrorRamWriteAccepted( u16 a, u8 v );

	__attribute__((always_inline)) inline void writeFast( u16 a, u8 v )
	{
		if ( dosExtensionMapsBank1( a ) )
		{
			m_ROMShadow[ a ] = v;
			return;
		}
		// Note the mirror sink still has to be told: the VIC-II only sees what
		// reaches the C64's own DRAM.
		//
		// The `a > 1` gate stays LIVE and first, exactly as in readFast: the
		// $00/$01 port has to reach write8(), whose updateBankMode() is what
		// rebuilds this very table. Page 0 is REG_RAM in all 32 PLA modes, so
		// m_WriteBase[ 0 ] is never null and the gate cannot be folded into it.
		if ( a > 1 )
		{
			const u32 wpage = (u32)a >> 12;
			u8 * const page = m_WriteBase[ wpage ];
			if ( page )
			{
				// One load and a branch in place of the nine-instruction
				// c64RegionTable derivation this used to do per write; see the
				// note on m_WriteBase.
				u8 * const slot = page + ( a & 0x0FFF );
				// Sink first, store second: the sink reads the old byte through
				// its shadow pointer and drops writes that change nothing.
				SCPU_COUNT_WRITE( m_RamWrites );
				mirrorRamWrite( a, v );

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
				//
				// noteRAMWriteSideEffects REMAINS THE AUTHORITY and still
				// re-asserts both of its own address tests; the bitmap is a
				// necessary condition of either of them firing, so this is a
				// pure cost reduction and not a second copy of the rule. It
				// cost 21 instructions per write to be told "no" in the shipped
				// K451 object -- 4 argument movs, a tail branch and a
				// 16-instruction body including a far load of m_SpritePtrBase
				// (`add x4,x0,#0x18,lsl#12`). Now three.
				//
				// THE OLD BYTE IS LOADED INSIDE THE BRANCH, not above the sink
				// call, and that is a correctness claim, not a tidy-up: `old` has
				// exactly TWO consumers, both reached through noteRAMWriteSideEffects:
				// noteSpritePointerWrite's `old == v` skip, and noteWatchedWrite,
				// which stores e.old into the watch ring and counts m_WatchChanges
				// on `old != v`. BOTH must still observe DRAM as it was BEFORE this
				// store.
				// Nothing between the two points can change it -- CWriteBuffer
				// holds m_RAM as `const u8 *` (attach() takes it const), so
				// onRamWrite physically cannot write the byte, and noteMirrorWrite
				// only appends a timing event. If the sink ever gains a non-const
				// view of m_RAM, this load has to move back above it.
				// The load was `ldrb w22, [x24,x25]` at write8+0xcc, unconditional
				// on every one of the ~1.27M writes in the TI benchmark, feeding a
				// pair of consumers that fire only on the pages rebuildWriteNotify()
				// arms -- the sprite-pointer row's page, the page of either armed
				// RAM_WATCH slot, and page $F unconditionally -- and it kept
				// the old byte LIVE across the sink's indirect call, which bought a
				// callee-saved register nothing else needed.
				//
				// A/B of CW65C816::write8, both objects compiled from ONE frozen
				// snapshot of every other header with the project toolchain at the
				// shipped -Os (this hunk plus the field hoist above):
				//   plain bank-0 DRAM store   53 -> 50 executed instructions
				//   stack frame               80 -> 64 bytes (x25 no longer saved)
				//   write8                  0x1e8 -> 0x1cc bytes
				//   read8                   0x144 -> 0x144, unchanged
				//   stepInner              0x6340 -> 0x633c, so the -Os dispatch unit
				//                           and its L1I footprint are untouched
				if ( __builtin_expect(
				         ( ( m_WriteNotifyPages >> wpage ) & 1u ) == 0u, 1 ) )
				{
					*slot = v;
					return;
				}
				const u8 old = *slot;
				*slot = v;
				noteRAMWriteSideEffects( a, old, v );
				return;
			}
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

	// --- the emulated BA stall --------------------------------------------
	// 0 off, 1 on except while a serial transaction window is armed, 2 always.
	// See baStallSettle() for why it exists and what bounds it.
	u8   m_BAStallMode = 0;					// config BA_STALL
	u16  m_BAStallMaxCycles = 80;			// config BA_STALL_MAX, C64 cycles
	u64  m_BAStallCycles = 0;				// statistics: cycles charged
	u64  m_BAStallEvents = 0;
	u64  m_BAStallSuppressedIEC = 0;		// samples declined inside IEC
	u64  m_VSPStallAtLastEvent = 0;			// VSPTimingEvent::baStallCycles

	// --- the serial-transaction window ------------------------------------
	// MEASURED, K456 on hardware, Mario gameplay with no drive access at all:
	// BA_STALL 1 declined 2751432 stall samples against 50319 taken, so a
	// serial window was armed 98% of the time on a machine that never touched
	// the bus. The arm that can do that is noteIECActivity()'s 100000-cycle
	// hold re-armed by a LEVEL instead of an edge -- see
	// iecSelfDrivenInputMask(): a latch bit the guest leaves set pulls CLK or
	// DATA low and PA6/PA7 read that same low back, so ONE $DD00 read per
	// frame keeps a 100ms hold permanently alive while the 2000-cycle mirror
	// window stays ~10% armed. That asymmetry is why the screen looked fine
	// and only the BA stall ever noticed.
	//
	//   0  Legacy. Every asserted level and every output change arms.
	//   1  (default) A line THE GUEST ITSELF IS PULLING DOWN is not evidence.
	//      Input edges, output changes, and a drive holding a line we released
	//      all still arm, so a real transaction is unaffected.
	//   2  Positive evidence only: a window OPENS on an ATN assertion or an
	//      input edge; a held level or a non-ATN output change may only RE-arm
	//      a window that is already open.
	u8   m_IECWindowMode = 1;				// config IEC_WINDOW
	// Armed-time against transaction-time. Counted inside the branches that
	// already test each window, so a quiet machine pays nothing, and reported
	// as its own diagnostic line: the 98% above had to be INFERRED from
	// iec-skipped, which cannot say which of the three windows was armed.
	u64  m_IECHoldArmedCycles = 0;			// the 100ms speed hold
	u64  m_IECPollArmedCycles = 0;			// the 1000-cycle poll top-up
	u64  m_IECActivityArmedCycles = 0;		// the 2ms edge window: TRANSACTION time
	u64  m_IECArmReadEdge = 0;				// arms, by the evidence that caused them
	u64  m_IECArmReadLevel = 0;
	u64  m_IECArmReadRefused = 0;			// mode 0 would have armed; this mode did not
	u64  m_IECArmWrite = 0;
	u64  m_IECArmWriteRefused = 0;
	u64  m_IECPollArms = 0;
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
	// ARMED BY A GUEST $D019/$D01A WRITE, cleared by the pin read it forces; see
	// fineTicksRequired() and the arming site in appendIntCtlWrite.
	//
	// DECLARED HERE, AND THE PLACEMENT IS THE WHOLE COST ARGUMENT. MEASURED:
	// fineTicksRequired() already reads members on four separate cache lines
	// (offsets 489, 86685, 86924, 103392 in the host build), and it runs after
	// EVERY instruction in turbo. A bool at the end of the class would have made
	// that five lines, and the second-read position means the new line would be
	// touched on the common path where m_PostedWritePending is false -- a real
	// per-instruction turbo cost, which is a hard veto on this tree.
	//
	// m_PostedWritePending is 1 byte at offset 489 and m_MirrorBufferFreeAt is a
	// u64 that must land at 496, so bytes 490-495 are PADDING that already
	// exists. Taking one of them puts this flag in the line the predicate loads
	// FIRST and unconditionally, so no new line is touched, and leaves sizeof
	// and every other member offset BYTE-IDENTICAL -- including m_EmuCycles, so
	// stepInner's displacement immediates do not move and its object stays the
	// turbo-is-untouched proof. Both facts are measured, not assumed.
	bool m_IRQResampleDue = false;
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
	// EDIT 1 of 3. LAND THIS LAST, after the two field hoists -- see the
	// measured cost note at the bottom of this comment.
	//
	// GCC kept the whole of this out of line in the shipped K448 object
	// (.text._ZNK10CC64Memory21dosExtensionMapsBank1Et, 0x5c bytes) and 29 of
	// its 31 call sites are inside stepInner's always-inline fast switch, so
	// every bank-0 access paid two argument movs, a bl, and a 13-instruction
	// body including its ret -- 16 in all -- to be told "no". Counted off the
	// K448 disassembly: body offsets 0x00..0x2c, then the ret at 0x50.
	//
	// "No" is not the uncommon case, it is the only case the TI benchmark ever
	// sees. The windows are pages 1-5 and 8-9; the benchmark runs out of BASIC
	// ROM pages $A-$B and KERNAL $E-$F with its text at $0801, its stack at
	// $0100 and its floats in zero page -- all page 0. Hence the page bitmap:
	// bits 1,2,3,4,5 are $1000-$5FFF and bits 8,9 are $8000-$9FFF, so 0x033E.
	// Both windows are page aligned, so the bitmap is EXACTLY the range test,
	// not an approximation of it.
	//
	// The state tests stay out of line and RE-ASSERT the original ranges, so
	// that expression remains the authority and the bitmap is a pure cost
	// reduction rather than a second copy of the rule. Predicate order and
	// result are unchanged, no state read is skipped that the && chain did not
	// already skip, and the out-of-line body contains no stores (walked
	// instruction by instruction in the K448 object), so declining an
	// out-of-window address earlier is not observable.
	//
	// MEASURED CODEGEN: GCC emits the COMPLEMENT and an inverted branch, not
	// the mask -- `ubfx / mov w0,#0xfffffcc1 / asr / tbnz`, where 0xfffffcc1
	// is ~0x033E. Do not grep the disassembly for 0x33e; it is not there.
	//
	// MEASURED COST: GCC keeps the in-window call inline behind the test at
	// each site, so on its own this GROWS stepInner by 380 bytes, 0x6b10 ->
	// 0x6c8c (project toolchain, -Os). The two field hoists give back 548
	// (0x6b10 -> 0x68ec); all three together are 0x6a78, i.e. -152 net. In a
	// 32KB 2-way L1I this edit is affordable only on top of those, so land
	// them first. Growth here is EXPECTED and is not evidence that the
	// noinline attribute failed to take.
	__attribute__((always_inline)) inline bool dosExtensionMapsBank1( u16 addr ) const
	{
		if ( __builtin_expect( ( ( 0x033Eu >> ( addr >> 12 ) ) & 1u ) == 0u, 1 ) )
			return false;
		// SECOND gate, same principle as the bitmap above and for the same
		// reason: the out-of-line body stays the authority and still re-asserts
		// everything, so this is a pure cost reduction and not a second copy of
		// the rule.
		//
		// MEASURED in the shipped object: the in-window "no" answer costs 24
		// instructions -- two argument movs and a bl at the call site, the
		// 18-instruction range/state body, the two-instruction return, and the
		// caller's test. This gate compiles to exactly 4 (ldr, cbz, ldrb, tbz),
		// confirmed in the rebuilt object. So -20 per bank-0 access in pages
		// $1-$5 and $8-$9.
		//
		// WHOSE WORKLOAD: the TI throughput benchmark takes this path ZERO
		// times (host census, 590113 instructions) because it lives entirely in
		// page 0 and in ROM -- which independently confirms the bitmap above
		// does what its author claimed. The windows are $1000-$5FFF and
		// $8000-$9FFF, where a GAME's code and data live, so this is a
		// Mario-side edit with NO expected effect on TI. Its value there is
		// UNMEASURED: put a counter on dosExtensionStateMapsBank1 and measure
		// before believing any estimate.
		//
		// SAFE, and this is the argument rather than an assertion: failing this
		// gate is a strictly NECESSARY condition of the body returning false.
		// The body is (range) && !m_BootmapActive && m_ROMShadow &&
		// m_DOSExtState && *m_DOSExtState, so if either of the last two is
		// false the body would have answered false too. It is one pure return
		// expression over member loads with no stores, so skipping the two
		// intervening loads cannot be observed; m_DOSExtState is a plain
		// const bool *, not volatile; and the null test short-circuits before
		// the dereference exactly as the body's own chain does. Covered by
		// test_memory_map.cpp's scpu_dos_extension_maps_its_bank1_work_areas,
		// which reads AND writes both windows with the extension ON and then
		// OFF, by bank0_readfast_matches_read8_in_every_banking_state for the
		// OFF case, and by test_reu.cpp's
		// reu_chain_preserves_the_cached_live_dos_extension_mapping.
		if ( __builtin_expect( !m_DOSExtState || !*m_DOSExtState, 1 ) )
			return false;
		return dosExtensionStateMapsBank1( addr );
	}
	__attribute__((noinline)) bool dosExtensionStateMapsBank1( u16 addr ) const
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
			// The RISING edge of the physical assertion, stamped where the
			// GPIO sample is already being paid for. Mario's chain is driven
			// by the real VIC's raster compare through the real /IRQ pin, so
			// this is the only point in the firmware that knows when the
			// hardware asked for the handler -- and therefore the only way to
			// tell "we noticed it 201 lines late" from "it was asserted 201
			// lines late". One compare on the sampling path, which runs once
			// per emulated microsecond; the host read happens only on an edge,
			// a few hundred times a second.
			if ( irq && !m_CachedIRQ )
				m_IRQAssertHost = m_C64->hostCycles();
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
			// THE STAMP THE FOLLOW-UP VERDICT NEEDS, and the only place that can
			// honestly set it: this is where a pin read actually happens.
			// m_IntCredit cannot serve -- it is advanced by tickFast, which the
			// batch loop calls once per eight instructions, so mid-batch it reads
			// 0 for a sample that is eight instructions old. This block is already
			// behind the credit gate, so it runs once per emulated microsecond and
			// not per instruction -- the one place a stamp can be afforded.
			m_IRQSampleEmu = m_EmuCycles;
			// The acknowledgement one-shot has done its job: the level the core is
			// about to act on is now post-ack. Cleared here so fine ticks last one
			// instruction and turbo is not taxed.
			m_IRQResampleDue = false;
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
		{
			// TRANSACTION time: edge-proven, 2ms, and the window whose armed
			// fraction should be near zero in a game that touches no drive.
			// Against m_IECHoldArmedCycles it IS the over-arm measurement.
			m_IECActivityArmedCycles += nCycles;
			m_IECActivityCycles = ( m_IECActivityCycles > nCycles )
			                    ? ( m_IECActivityCycles - nCycles ) : 0;
		}

		// Poll evidence decays the same way. In turbo its expiry is also a rate
		// transition; at an explicitly selected 1MHz it is activity evidence
		// only, so no effective-rate notification is needed.
		const bool explicitlySlow = m_SelectedEmulatedHz != 0
		                         && m_SelectedEmulatedHz <= 1000000u;
		if ( m_IECPollHoldCycles )
		{
			// Only counted where it actually forces the rate: at an explicitly
			// selected 1MHz the poll hold is evidence, not a speed change, and
			// counting it there would double-count m_SlowPacedCycles.
			if ( !explicitlySlow )
				m_IECPollArmedCycles += nCycles;
			m_IECPollHoldCycles = ( m_IECPollHoldCycles > nCycles )
			                    ? ( m_IECPollHoldCycles - nCycles ) : 0;
			if ( m_IECPollHoldCycles == 0 && m_IECHoldCycles == 0
			     && !explicitlySlow )
			{
				cia2OnRateChange();
				if ( m_TimingHook )
					m_TimingHook( m_TimingHookCtx );
			}
		}

		const bool iecActive = ( m_IECHoldCycles != 0 )
		                    || ( m_IECPollHoldCycles != 0 && !explicitlySlow );
		if ( iecActive )
			m_IECThrottledCycles += nCycles;
		if ( iecActive || ( m_SelectedEmulatedHz != 0
		                   && m_SelectedEmulatedHz <= 1000000u ) )
			m_SlowPacedCycles += nCycles;
		if ( iecActive )
		{
			const bool hadIECHold = m_IECHoldCycles != 0;
			if ( hadIECHold )
				m_IECHoldArmedCycles += nCycles;
			m_IECHoldCycles = ( m_IECHoldCycles > nCycles )
			                ? ( m_IECHoldCycles - nCycles ) : 0;
			if ( hadIECHold && m_IECHoldCycles == 0
			     && m_IECPollHoldCycles == 0 )
			{
				// Hold expiry is an effective-speed transition; armed CIA2
				// deadlines re-denominate (cold: once per expiry).
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
	// Out of line, and called only from tickSettle -- which is itself out of
	// line and TAIL-CALLED from tickFast. Measured in the shipped K451 object
	// with the project's own aarch64-none-elf-objdump:
	// _ZN10CC64Memory8tickFastEj+0x330 is a `b`, not a `bl`, to
	// _ZN10CC64Memory10tickSettleEb, directly after the +0x32c
	// `ldp x29, x30, [sp], #64` that tears the frame down.
	// Putting this gate at that call site in the header would have turned the
	// tail call into a call plus a live frame on EVERY settle, turbo included.
	// Inside tickSettle the turbo path gains nothing at all: in the rebuilt
	// object the pure turbo exit returns without executing one new instruction,
	// and GCC tail-merges both gate sites into a single `b` to this function.
	void baStallSettle( bool iecActive );

	// --- ICpuBus ----------------------------------------------------------
	u8   read8( scpu_addr_t addr ) override;
	void write8( scpu_addr_t addr, u8 value ) override;
	bool irqAsserted() override;
	bool nmiAsserted() override;
	void tick( u32 nCycles ) override;
	void tickPhaseTrim( s32 c64Cycles ) override;
	// VSP_AIM_WRITE as applied (K498): whole C64 cycles added to the launch
	// instant of every placed VIC write inside the raster window. Lives here,
	// at the catch-up in write8, and not in the pacer -- see the block there.
	void setVSPAimWrite( s32 c64Cycles ) { m_VSPAimWriteCycles = c64Cycles; }
	s32  vspAimWrite() const { return m_VSPAimWriteCycles; }
	// VSP_D011_SHADOW_READ (K500): see d011ShadowReadSafe() and the REG_IO arm
	// of read8.
	void setVSPD011ShadowRead( bool on ) { m_VSPD011ShadowRead = on; }
	void setVSPPlacementK501( bool on ) { m_VSPPlacementK501 = on; }
	void setVSPTraceWindowOnly( bool on ) { m_VSPTraceWindowOnly = on; }
	bool d011ShadowReadSafe();

	// --- state, public for tracing and tests ------------------------------
	u8  m_RAM[ C64_RAM_SIZE ];
	u8  m_Basic[ C64_BASIC_SIZE ];
	u8  m_Kernal[ C64_KERNAL_SIZE ];
	u8  m_CharROM[ C64_CHARROM_SIZE ];

	// Emulated 6510 on-chip I/O port.
	u8  m_Port00;		// data direction register
	u8  m_Port01;		// port latch

	// m_BankMode moved to the hot decode block near the top of the class; see
	// the measured note beside m_BootmapActive before moving it back.
	u64 m_IOReads;		// statistics: accesses that actually hit the bus
	u64 m_IOWrites;
	// m_RamWrites moved to the hot decode block near the top of the class, with
	// m_Mirror and m_WriteNotifyPages; see the measured note beside m_BankMode.

	void updateBankMode();

	// Re-resolve m_ReadBase. Public because anything that writes a decode
	// input behind its setter -- the exhaustive readFast-vs-read8 test does,
	// deliberately -- has to call this in the same breath. Idempotent and
	// cheap: 16 iterations of a switch, off every hot path.
	void rebuildReadDecode();

	// Re-resolve m_WriteBase. Public for the same reason and with the same
	// obligation -- see the note on m_WriteBase, and prefer calling BOTH
	// whenever you have hand-set m_BankMode, because forgetting this one
	// corrupts memory where forgetting the other only reads it.
	void rebuildWriteDecode();

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
		const bool explicitlySlow = m_SelectedEmulatedHz != 0
		                         && m_SelectedEmulatedHz <= 1000000u;
		return m_IECHoldCycles > 0
		    || ( m_IECPollHoldCycles > 0 && !explicitlySlow );
	}
	// Positive guest-activity evidence, distinct from the automatic-speed
	// fallback above. This remains meaningful when software explicitly chose
	// 1MHz, which is exactly when cycle-timed IEC loaders do their polling.
	bool iecPollActive() const { return m_IECPollHoldCycles != 0; }

	// The 65816 normally batches several instructions before ticking the C64
	// bus. That is safe at turbo speed, but not at an effective/selected 1MHz:
	// IEC code depends on the real-time spacing between individual
	// instructions even when CMD selected slow mode itself (and therefore the
	// automatic throttle timer is not armed).
	bool fineTicksRequired() const
	{
		// m_IRQResampleDue is the ONE-SHOT ARMED BY A $D019/$D01A WRITE: it ends
		// the instruction batch so the next interrupt decision uses a pin sample
		// taken AFTER the acknowledgement instead of the batch's opening one. It
		// clears inside refreshInterruptsIfDue the moment that sample is taken,
		// so it lives for exactly one instruction and cannot leave turbo parked
		// on fine ticks. One byte test added to a predicate already doing seven;
		// every term is a side-effect-free member read, so adding a disjunct
		// changes no existing term's truth value or evaluation.
		return m_PostedWritePending || m_IRQResampleDue
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
	// Which receive inputs are reporting the GUEST'S OWN output. CIA2 PA3-PA5
	// feed inverting open-collector drivers onto a wired-AND bus, so a latch
	// bit held high pulls its line low, and the direct inputs PA6 (CLK) and
	// PA7 (DATA) read that same low. The KERNAL proves it must: ACPTR releases
	// CLK through CLKHI at $EE85 BEFORE waiting for CLK high at $EE1B, which
	// would be pointless if the host could not see its own pull-down. A line
	// we are pulling down says nothing about whether a drive is answering.
	//
	// The one shape this mode cannot see is the guest holding CLK AND DATA
	// down at once with no edge, which empties the mask. The stock KERNAL
	// never does: all 11 STA $DD00 sites in kernal.901227-03.bin apply
	// single-bit masks only -- ORA #$08/#$10/#$20 and AND #$EF/#$DF/#$F7 --
	// and $ED8B is AND #$DF then ORA #$10, releasing DATA in the very write
	// that pulls CLK low. No ORA #$30 exists in the ROM, so a stock load
	// cannot reach the blind spot. JiffyDOS is a different ROM, but it
	// bit-bangs, and every output change still arms in this mode.
	u8 iecSelfDrivenInputMask() const
	{
		if ( !m_HaveCIA2PortALatch || !m_HaveCIA2DDRA )
			return 0;
		const u8 drive = (u8)( m_CIA2PortALatch & m_CIA2DDRA );
		return (u8)( ( ( drive & 0x10 ) ? 0x40 : 0 )
		           | ( ( drive & 0x20 ) ? 0x80 : 0 ) );
	}
	// For the diagnostic line: an over-arm capture is only actionable with the
	// latch and direction that produced it.
	u8 cia2PortALatch() const { return m_CIA2PortALatch; }
	u8 cia2DDRA() const { return m_CIA2DDRA; }
	typedef void (*TimingHook)( void *ctx );
	void setTimingHook( TimingHook hook, void *ctx ) { m_TimingHook = hook; m_TimingHookCtx = ctx; }
	u64  m_IECThrottleEvents;

	u64 m_PacingDebtCycles;		// emulated cycles run but not yet paid for in real time
	u32 m_PacingCheckCycles;	// emulated cycles between consulting the host clock
	u64 m_PacerWaitHostCycles;	// host cycles deliberately spent waiting
	// tickSettle's late branch has never been measured, and the whole "+10
	// C64 cycles" VSP arithmetic assumes it fires on every physical access.
	// Counted ONLY while the guest has explicitly selected 1MHz, which is
	// where Mario's handler runs: in turbo the pacer is behind on essentially
	// every settle (70 host cycles owed per emulated cycle against a measured
	// ~146), so an ungated count would be dominated by a regime that has
	// nothing to do with the question, and these are reset-to-date totals in
	// the SD files. Zero across a Mario run says the branch is not the source.
	u64 m_PacerLateEntries = 0;		// late arrivals at explicit 1MHz
	// Only a cadence-limited subset of those entries observes a PHI edge
	// and snaps forward; the rest hold the installed phase class with
	// arithmetic. The host-cycle total below is billed on the probing
	// subset alone, so it divides by m_PacerLateProbes -- both the SD line
	// and the console speed report are written that way.
	u64 m_PacerLateProbes = 0;		// late arrivals that probed an edge
	u64 m_PacerLateSnapHostCycles = 0;	// what those probes then added
	// THE PACING CREDIT. Measured, K449 Mario gameplay: the $D011 DEC/INC hold
	// is 8.98 C64 cycles where NMOS is exactly 6, while dEmu and dC64 are 6 on
	// every one of 21 pairs -- the model is right and the wall clock is not.
	// The late arm used to re-anchor the grid to NOW, so the grid advanced by
	// owed PLUS the overrun while the guest was charged only owed. That overrun
	// IS the physical bus time (K449 per-stage means: read 765+1193+859 = 2817
	// ARM = 2.06 C64 cycles, write-with-turnaround 960+4640+884 = 6484 = 4.74),
	// and writing it off makes the next instruction wait out a full budget
	// measured from AFTER it: 6 + bus instead of max( 6, bus ). Carrying it
	// keeps the invariant that defines the absolute grid -- the anchor advances
	// by exactly owed -- so the guest cannot end up AHEAD of the 1MHz grid; it
	// only stops ending up behind it.
	//
	// The cap is what a0215f5's anti-windup clamp was for, and it still fires:
	// a physical access genuinely costs 2.06 to 4.74 C64 cycles against a
	// one-cycle budget, so the lag grows without bound through I/O-heavy code
	// and must not be reclaimed as a single burst. Eight cycles is one long
	// instruction's worth of its own uncharged bus work. Above the cap the lag
	// is a STALL rather than bus time -- an HDMI blit, a 150us bus idle-prime,
	// core-1 contention -- and crediting one would run the guest at interpreter
	// speed for as many cycles as the cap allows. Note what the cap does NOT
	// exclude: the 90us keepalive write at the top of tickSettle costs 2.6 to
	// 4.7 cycles by those same per-stage means, UNDER the cap, so it is now
	// credited rather than forgiven and the guest makes it up unpaced. That is
	// deliberate -- the write stole real time from the guest's grid position and
	// a real 1MHz machine never paid it -- but it is a ~4-cycle unpaced burst
	// every 90us and it belongs in the same budget as everything else here.
	// INFERRED, not measured: 8 is the knob to bisect if the hold moves but
	// Mario's wrap does not.
	static constexpr u64 PACER_CREDIT_CYCLES = 8;
	u64 m_PacerCreditEntries = 0;		// late arrivals whose lag was carried
	// The OUTSTANDING lag at each credited entry, summed -- the same lag is
	// counted again at the next entry until a clamp or a probe clears it, so
	// this is not a total of distinct host cycles. Divide by credit-entries for
	// the mean outstanding lag and compare it against the cap AS THE CODE
	// COMPUTES IT: PACER_CREDIT_CYCLES * perC64 = 8 * 1368 = 10944 ARM, the
	// TRUNCATED period, not 8 * the true 1368.889 = 10951. Exactly the kind of
	// misreading CODEX caught on late-snap-host, named here so it cannot happen
	// twice.
	u64 m_PacerCreditHostCycles = 0;
	u64 m_PacerClampEntries = 0;		// late arrivals past the cap, lag forgiven
	// THE PHYSICAL PHASE TRIM pending for the next tickSettle, host cycles,
	// signed. Set by tickPhaseTrim() from a placed VIC access's VSP_AIM_*
	// value, consumed or dropped by the very next tickSettle(). Never added
	// to owed, the debt, m_EmuCycles or the anchor advance: it moves the
	// instant ONE settle exits and nothing else.
	s64 m_PacerPhaseTrimHost = 0;
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

	// Free-running C64 BUS clock (~1MHz), independent of the accelerator's
	// selected CPU rate. Raster timestamps must use this denomination: dividing
	// raw turbo CPU cycles by 63 makes every split position speed-dependent.
	//
	// Each completed rate segment is banked using the factor that was actually
	// in force for that segment. Rate-change notifications arrive after the
	// hold/selection state has changed, so consulting effectiveCIAFactor() for
	// the elapsed segment would reinterpret old turbo time at the new rate.
	inline u64 c64Now() const
	{
		const u64 factor = m_C64FactorInUse ? m_C64FactorInUse : 1;
		return m_C64ClockBase
		     + ( m_EmuCycles - m_EmuAtRateChange ) / factor;
	}
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
	inline void c64ClockRebase()
	{
		m_C64ClockBase = c64Now();
		m_EmuAtRateChange = m_EmuCycles;
		m_C64FactorInUse = effectiveCIAFactor();
	}
	inline u64 effectiveCIAFactor() const
	{
		return ( m_IECHoldCycles || m_IECPollHoldCycles ) ? 1ULL
		     : (u64)( m_PacingCheckCycles ? m_PacingCheckCycles : 1 );
	}
private:

private:
	FF00Hook        m_FF00Hook = 0;
	void           *m_FF00HookCtx = 0;
	IC64Bus        *m_C64;
	// m_Mirror moved to the hot decode block near the top of the class; see the
	// measured note there before moving it back.
	IIOInterceptor *m_IO;
	volatile u32 *m_HDMISpriteSpriteCollision = 0;
	volatile u32 *m_HDMISpriteBackgroundCollision = 0;
	const bool *m_HDMIActive = 0;
	const u8       *m_BootmapROM;
	u32             m_BootmapROMLength;
	bool            m_HasBasic, m_HasKernal, m_HasChar;

	// Pacing state. m_HostPerEmuQ16 is host cycles per emulated cycle in 16.16
	// fixed point, so the common case is a multiply and a shift rather than a
	// division per instruction.
	u64 m_HostPerEmuQ16;		// host cycles per emulated cycle, 16.16
	u64 m_HostPerEmuQ16Slow;	// the same at 1MHz, for IEC hold-off
	u64 m_PacingAnchor;
	// PHI phase lock, at explicit 1MHz only. The anchor's phase against the
	// physical PHI grid decides which side of a cycle edge every settled
	// moment lands on; without a lock that phase is a lottery re-rolled at
	// each re-anchor, and cycle-exact intra-opcode settlement upstream is
	// nullified downstream. Renewed on a cadence because the Pi and C64
	// crystals drift relative to each other and the conversion rate is
	// nominal, not measured.
	u64 m_LastPhiLockHost;
	u64 m_PhiRelockHostCycles;
	s8  m_VSPAimRead = 0;
	s8  m_VSPAimWrite = 0;
	u32 m_SelectedEmulatedHz;
	bool m_BusKeepalive = false;
	u64 m_NextKeepalive = 0;
	u64 m_C64ClockBase = 0;
	u64 m_EmuAtRateChange = 0;
	u64 m_C64FactorInUse = 1;
	VICRegWrite m_VICLog[ SCPU_VIC_LOG_SIZE ];
	volatile u32 m_VICLogHead = 0;
	volatile u32 m_VICLogGeneration = 0;
	bool m_VICLogEnabled = false;
	volatile u64 m_RasterAnchor = 0;
	u32 m_LastPreciseAnchor = 0x80000000u;
	u16 m_RasterCompare = 0;
	u32 m_LastCompareWrite = 0x80000000u;
	u16 m_LastPreciseLine = 0xFFFF;

	inline void appendVICLog( u16 reg, u8 value )
	{
		if ( !m_VICLogEnabled ) return;
		const u32 now = (u32)c64Now();
		VICRegWrite &event = m_VICLog[ m_VICLogHead
		                              & ( SCPU_VIC_LOG_SIZE - 1 ) ];
		event.cycle = now;
		event.reg = (u16)( reg & VIC_LOG_REG_MASK );
		event.value = value;
		event.lineLow = 0;
		if ( m_LastPreciseLine != 0xFFFF
		     && (u32)( now - m_LastPreciseAnchor ) < 48 )
		{
			event.reg |= VIC_LOG_LINE_VALID;
			if ( m_LastPreciseLine & 0x100 )
				event.reg |= VIC_LOG_LINE_HIGH;
			event.lineLow = (u8)m_LastPreciseLine;
		}
		__atomic_store_n( &m_VICLogHead, m_VICLogHead + 1,
		                  __ATOMIC_RELEASE );
	}

	bool m_IECThrottleEnabled;
	u32  m_IECHoldCycles;		// emulated cycles left at forced 1MHz
	u32  m_IECActivityCycles = 0;	// edge-rearmed transaction window (countdown)
	// Poll activity sustained by serial-port READS (mere polls included):
	// the real SuperCPU's auto-slow re-arms on every access. Kept apart from
	// m_IECHoldCycles because iecBusActive() -- the MIRROR gate -- must never
	// see it globally: polls suppressing all mirroring is the blackout latch-up.
	// The large deferred VIC-map replay may consult it through iecPollActive().
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
	// the sentinel when the screen sits under the I/O window and the physical
	// bus has not proved native RAM-under-I/O delivery. Kept fresh by writes to
	// $D018 and $DD00.
	u32  m_SpritePtrBase;
	u32  m_PreviousSpritePtrBase;
	u8   m_LastD018;
	bool m_RAMUnderIOAccessible = false;
public:
	// Physical DRAM is a VIC backing store, not guest-owned memory: programs
	// always read the Pi shadow. On hosts which cannot expose RAM under I/O
	// without disturbing the VIC (notably the 64U), pack the active bank-3
	// display into the three safe physical spans instead:
	//
	//   sprite pool  $C000-$CBFF
	//   matrix       $CC00-$CFFF
	//   graphics     $E000-$FFFF  (2K charset or 8K bitmap)
	//
	// Guest-visible $D011/$D018/$DD00 and RAM remain untouched. Only physical
	// delivery addresses and the value written to the real VIC's $D018 differ.
	bool m_DisplayRelocActive = false;
	u16  m_DisplayRelocScreenSource = 0;
	u16  m_DisplayRelocGraphicsSource = 0;
	u16  m_DisplayRelocGraphicsLength = 0;
	u32  m_DisplayRelocGeneration = 1;
	static const u16 DISPLAY_RELOC_SCREEN_TARGET = 0xCC00;
	static const u16 DISPLAY_RELOC_GRAPHICS_TARGET = 0xE000;

	void setUnderIORelocate( bool enabled )
	{
		m_RelocEnable = enabled;
		recomputeVICPhysicalMap();
		updateSpritePtrBase();
		updateHotShapeBlocks();
	}
	void setRAMUnderIOAccessible( bool accessible )
	{
		m_RAMUnderIOAccessible = accessible;
		if ( accessible ) m_RelocEnable = false;
		recomputeVICPhysicalMap();
		updateSpritePtrBase();
		updateHotShapeBlocks();
	}

	// Translate one guest-shadow byte to the DRAM address the physical VIC
	// consumes. `deliver=false` means the guest address currently aliases a
	// reserved physical target and must not overwrite it. This is the single
	// authority used by queued flushes, sweeps, scrubs and immediate pointers.
	inline u16 vicPhysicalAddress( u16 guest, bool &deliver ) const
	{
		deliver = true;
		if ( m_DisplayRelocActive )
		{
			if ( (u16)( guest - m_DisplayRelocScreenSource ) < 1024 )
				return (u16)( DISPLAY_RELOC_SCREEN_TARGET
				              + (u16)( guest - m_DisplayRelocScreenSource ) );
			if ( (u16)( guest - m_DisplayRelocGraphicsSource )
			     < m_DisplayRelocGraphicsLength )
				return (u16)( DISPLAY_RELOC_GRAPHICS_TARGET
				              + (u16)( guest - m_DisplayRelocGraphicsSource ) );
		}

		// Existing under-I/O sprite relocation composes after the display map.
		// A byte shared by an active bitmap and a sprite is delivered to the
		// graphics copy here; the pointer allocation still protects the common
		// real-world case where shapes are outside the active graphics span.
		if ( m_RelocCount && guest >= 0xD000 && guest <= 0xDFFF )
		{
			const u8 r = m_PtrReloc[ ( guest >> 6 ) & 0x3F ];
			if ( r != 0xFF )
				return (u16)( 0xC000 + ( (u16)r << 6 ) + ( guest & 63 ) );
		}

		if ( m_DisplayRelocActive )
		{
			if ( (u16)( guest - DISPLAY_RELOC_SCREEN_TARGET ) < 1024
			     || (u16)( guest - DISPLAY_RELOC_GRAPHICS_TARGET )
			        < m_DisplayRelocGraphicsLength )
			{
				deliver = false;
				return guest;
			}
		}
		if ( m_RelocCount && guest >= 0xC000 && guest < 0xCC00
		     && m_RelocInUse[ ( guest - 0xC000 ) >> 6 ] != 0xFF )
		{
			deliver = false;
			return guest;
		}
		return guest;
	}

	inline u8 vicPhysicalValue( u16 guest, u8 value ) const
	{
		// Relocated shape bytes can themselves fall in the last eight bytes of a
		// possible matrix slot. They are data, never pointers.
		if ( m_RelocCount && guest >= 0xD000 && guest <= 0xDFFF
		     && m_PtrReloc[ ( guest >> 6 ) & 0x3F ] != 0xFF )
			return value;
		// The production write-buffer path comes through this function. A stolen
		// $C000-$CBFF block contains relocated shape DATA, even when its final
		// eight bytes happen to have the address shape of a sprite-pointer row.
		// The legacy deliverValue() path already had this guard; omitting it here
		// corrupted blocks 15, 31 and 47 whenever a tail byte was $40-$7F.
		if ( m_RelocCount && guest >= 0xC000 && guest < 0xCC00
		     && m_RelocInUse[ ( guest - 0xC000 ) >> 6 ] != 0xFF )
			return value;
		if ( m_RelocCount && guest >= 0xC000
		     && ( guest & 0x03FF ) >= 0x03F8
		     && value >= 0x40 && value < 0x80 )
		{
			const u8 r = m_PtrReloc[ value - 0x40 ];
			if ( r != 0xFF ) return r;
		}
		return value;
	}

	inline u8 vicPhysicalD018( u8 guestValue ) const
	{
		return m_DisplayRelocActive ? (u8)( 0x38 | ( guestValue & 1 ) )
		                            : guestValue;
	}
	u32 vicPhysicalMapGeneration() const { return m_DisplayRelocGeneration; }
	void recomputeVICPhysicalMap();
	// First byte of the active VIC screen matrix, or 0xFFFFFFFF when that
	// matrix lies under an I/O window the physical bus cannot yet reach as RAM.
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

	// Passive diagnostic snapshot of CIA2 port A. The frame hook used to call
	// read8($DD00/$DD02) every 50 frames merely to preserve state for a later
	// reset dump. Those guest-visible reads re-armed the SuperCPU's automatic
	// 1MHz IEC poll hold, injecting a roughly millisecond-long slowdown about
	// once per second. All authoritative pieces are already tracked here, so
	// reconstruct the same PRA view without touching the physical bus or any
	// timing state. A live input sample is required; otherwise reporting stale
	// or invented PA6/PA7 levels would be worse than saying no sample exists.
	bool trackedCIA2Snapshot( u8 &portA, u8 &ddra ) const
	{
		if ( !m_HaveCIA2DDRA || !m_HaveCIA2PortARead )
			return false;
		ddra = m_CIA2DDRA;
		portA = m_LastCIA2PortARead;
		if ( m_HaveCIA2PortALatch )
			portA = (u8)( ( portA & (u8)~ddra )
			              | ( m_CIA2PortALatch & ddra ) );
		return true;
	}

	// Timestamp capture is explicitly enabled only by VIDEO_MODE 1. With it
	// off, the hot VIC/colour write path performs no clock read or ring store.
	void enableVICLog( bool enabled )
	{
		if ( enabled && !m_VICLogEnabled )
		{
			__atomic_store_n( &m_VICLogHead, 0u, __ATOMIC_RELEASE );
			m_LastPreciseAnchor = 0x80000000u;
			m_LastCompareWrite = 0x80000000u;
			m_LastPreciseLine = 0xFFFF;
			__atomic_store_n( &m_RasterAnchor, 0ULL, __ATOMIC_RELAXED );
			// A new producer epoch cannot be confused with a wrapped/reset head.
			__atomic_add_fetch( &m_VICLogGeneration, 1u, __ATOMIC_RELEASE );
		}
		m_VICLogEnabled = enabled;
	}
	bool vicLogEnabled() const { return m_VICLogEnabled; }
	const VICRegWrite *vicLog() const { return m_VICLog; }
	u32 vicLogHead() const
	{
		return __atomic_load_n( &m_VICLogHead, __ATOMIC_ACQUIRE );
	}
	u32 vicLogGeneration() const
	{
		return __atomic_load_n( &m_VICLogGeneration, __ATOMIC_ACQUIRE );
	}
	u64 rasterAnchor() const
	{
		return __atomic_load_n( &m_RasterAnchor, __ATOMIC_RELAXED );
	}
	u64 emuNow() const { return c64Now(); }

	// Copy the retained VSP timing sidecar oldest first.  total is monotonic
	// since reset and overwritten proves whether the requested history still
	// fitted in the ring when the diagnostic was captured.
	u32 copyVSPTrace( VSPTimingEvent *out, u32 capacity,
	                  u32 &total, u32 &overwritten ) const;
	u32 vspTraceTotal() const { return m_VSPTraceHead; }

	// The raster line the guest's compare is currently aimed at. Maintained on
	// every $D011/$D012 write whether or not VIC capture is enabled (see the
	// note beside m_RasterCompare), which is what makes it usable in a
	// post-mortem taken with VIDEO_MODE 0. "The chain is parked because a
	// compare it programmed can never match" shows up here and nowhere else.
	u16 rasterCompare() const { return m_RasterCompare; }


	inline void noteRasterPrecise( u16 line )
	{
		const u32 now = (u32)c64Now();
		const u16 exact = (u16)( line & 0x1FF );

		// An anchor says "the beam is HERE now", and the only caller passes
		// m_RasterCompare. A compare the machine cannot reach is not where the
		// beam is, and publishing it would misplace every HDMI split in the
		// frame by the difference -- K471's 318 against 263 would have put them
		// 55 lines out. Refuse instead of recording a lie; the previous anchor
		// stays, which is merely stale rather than wrong. Costs one compare on
		// a path already gated by m_VICLogEnabled at both call sites.
		if ( m_C64 && exact >= (u16)c64RasterLines( m_C64->signals().video ) )
		{
			m_RasterAnchorRefused++;
			return;
		}

		// Tag the writes this handler ALREADY made with the exact line.
		//
		// They are on the compare line by construction -- the handler only runs
		// because the beam reached it -- but they were logged before this anchor
		// existed, so appendVICLog's forward-looking window could not tag them
		// and they fall back to cycle arithmetic. That arithmetic reads the
		// anchor cycle as the START of the line, when it is really some way in:
		// interrupt latency plus however much of the handler ran before the
		// compare write. Every earlier write therefore computes one line low and
		// its raster split lands a row early.
		//
		// Not theoretical. Against VICE, 24-vicraster put all four of its splits
		// a row out -- 64 border pixels wrong per split, the rest of the frame
		// perfect. Estimating the phase instead of measuring it is not enough
		// either: one global anchor places every event in the frame, so one
		// handler's phase gets applied to another's writes and a split straddles
		// the boundary. A tag is exact and belongs to the event itself.
		//
		// Bounded to one raster line and a few events: a program that writes VIC
		// registers continuously has no "first write of the handler", and must
		// not have unrelated history relabelled.
		const u32 perLine = m_C64 ? c64CyclesPerLine( m_C64->signals().video )
		                          : 63;
		const u32 head = m_VICLogHead;
		u32 lineStart = now;
		for ( u32 back = 1; back <= 8 && back <= head; back++ )
		{
			VICRegWrite &e =
				m_VICLog[ ( head - back ) & ( SCPU_VIC_LOG_SIZE - 1 ) ];
			// Unsigned, so a stale or future stamp wraps huge and stops us.
			if ( (u32)( now - e.cycle ) >= perLine ) break;
			lineStart = e.cycle;
			if ( vicLogHasRasterLine( e ) ) continue;	// already exact
			// lineLow first, then the valid bit with a release: a consumer that
			// sees VIC_LOG_LINE_VALID is guaranteed to see the line that goes
			// with it. One that samples mid-update sees no tag and falls back to
			// arithmetic, which is merely the old behaviour for one frame.
			e.lineLow = (u8)exact;
			const u16 tagged = (u16)( ( e.reg & VIC_LOG_REG_MASK )
			                        | VIC_LOG_LINE_VALID
			                        | ( ( exact & 0x100 ) ? VIC_LOG_LINE_HIGH
			                                              : 0 ) );
			__atomic_store_n( &e.reg, tagged, __ATOMIC_RELEASE );
		}

		m_LastPreciseAnchor = now;
		m_LastPreciseLine = exact;
		__atomic_store_n( &m_RasterAnchor,
		                  ( 1ULL << 63 ) | ( (u64)lineStart << 16 )
		                                | (u64)exact,
		                  __ATOMIC_RELAXED );
	}
	inline void noteRasterCoarse( u16 line )
	{
		if ( !m_VICLogEnabled || line == 0xFFFF ) return;
		const u32 now = (u32)c64Now();
		if ( (u32)( now - m_LastPreciseAnchor ) < 25000 )
			return;
		__atomic_store_n( &m_RasterAnchor,
		                  ( 1ULL << 63 ) | ( (u64)now << 16 )
		                                | (u64)( line & 0x1FF ),
		                  __ATOMIC_RELAXED );
	}

	inline void logVICBank()
	{
		appendVICLog( 0x40, (u8)( activeVICBankBase() >> 14 ) );
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
	// Fallback for a host where native RAM-under-I/O delivery was not prepared.
	// In VIC bank 3 a sprite pointer of $40-$7F selects $D000-$DFFF DRAM, but a
	// conventional halted host has I/O over that window and cannot accept the
	// shape bytes safely. 3D Pool /SCPU exposed this by double-buffering between
	// deliverable bank 1 and then-unreachable bank 3 shapes.
	//
	// The fallback lever is the pointer VALUE we deliver. So: remap
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
	u32  m_HotShapeGeneration = 0;
	void updateHotShapeBlocks()
	{
		// The write buffer caches a completed display-time scan that found no
		// deliverable byte. Publish every rebuild so a pointer/bank/mode change
		// cannot leave that negative result live after eligibility changed.
		// Producer and consumer both run on core 0 today. Do not move queue
		// flushing to another core without synchronizing this generation with
		// the rebuilt bits; publishing first is not a cross-core protocol.
		m_HotShapeGeneration++;
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
			// The write queue is keyed by guest-shadow address. Deferral must
			// therefore mark the SOURCE block even when physical delivery is
			// redirected to a relocated copy.
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
		// Before native delivery is armed, a pointer row under I/O is unreachable
		// and its urgent write path must not touch the overlaid chip registers.
		const u32 next = !m_RAMUnderIOAccessible && !m_DisplayRelocActive
		              && ( ( base & 0xF000 ) == 0xD000 )
		               ? 0xFFFFFFFF : base;
		if ( next != m_SpritePtrBase )
			m_PreviousSpritePtrBase = m_SpritePtrBase;
		m_SpritePtrBase = next;
		rebuildWriteNotify();
	}

	// writeFast's gate for noteRAMWriteSideEffects: the set of 4K pages in
	// which either of that function's two address tests can possibly fire.
	// Called from updateSpritePtrBase() directly above, which is the single
	// assignment site every mover of the pointer row goes through -- see the
	// input list on m_WriteNotifyPages.
	//
	// Bit 15 is unconditional, which is what makes m_FF00Hook a non-input and
	// also absorbs the 0xFFFFFFFF "no reachable row" sentinel: shifted down by
	// 12 and masked to 4 bits it IS 15, so the sentinel needs no branch here.
	// The row is $xxx3F8-$xxx3FF within a 1K block, so it can never straddle
	// two 4K pages and one bit is the exact test, not an approximation.
	//
	// The watched-write slots add their pages here, which is the whole of the
	// forensics hook's hot-path cost: writeFast's existing test is unchanged
	// and noteRAMWriteSideEffects still re-asserts its own two conditions, so
	// this widens WHO pays for the out-of-line call and changes nothing about
	// WHEN any existing predicate runs. An unarmed slot (0x10000) contributes
	// no bit, so RAM_WATCH 0 RAM_WATCH2 0 reproduces the pre-change bitmap
	// exactly -- that equality is what makes this bisectable.
	void rebuildWriteNotify()
	{
		if ( !WRITE_NOTIFY_CACHE )
		{
			m_WriteNotifyPages = 0xFFFF;
			return;
		}
		u32 pages = 0x8000u | ( 1u << ( ( m_SpritePtrBase >> 12 ) & 0x0Fu ) );
		for ( u32 i = 0; i < 2; i++ )
			if ( m_WatchAddr[ i ] <= 0xFFFF )
				pages |= 1u << ( ( m_WatchAddr[ i ] >> 12 ) & 0x0Fu );
		m_WriteNotifyPages = (u16)pages;
	}
	bool m_HaveCIA2PortARead;
	bool m_HaveCIA2DDRA;

	// 0x10000, not 0: $0000 is a watchable address, so "off" must be a value no
	// u16 can hold. Both slots off is the shipped default and reproduces the
	// pre-change m_WriteNotifyPages bit for bit.
	u32 m_WatchAddr[ 2 ] = { 0x10000u, 0x10000u };
	CPUProbeFn m_CPUProbe = 0;
	void *m_CPUProbeCtx = 0;
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

	// --- watched-RAM write forensics --------------------------------------
	//
	// The 2026-10-12 hang is a main loop spinning on a RAM byte ($A765 CMP
	// $9F88 / BEQ) that stopped changing while the raster chain kept running.
	// Three different bugs produce that symptom and nothing we record today
	// separates them: (a) the byte is never written, (b) it is written the SAME
	// value repeatedly, (c) it is written correctly and the loop reads
	// something else.  A ring of the last writes carrying value, old value,
	// writer PC and cycle answers all three from one capture.
	//
	// WHERE THE HOOK LIVES, AND WHY IT IS FREE.  Not in writeFast: that path
	// already costs ~60 ARM instructions per store and the whole K450 effort
	// was spent getting decode OUT of it.  Instead this reuses the gate that is
	// already there -- m_WriteNotifyPages, the 16-bit page bitmap writeFast
	// already tests before its out-of-line call to noteRAMWriteSideEffects.
	// Arming a watch sets that page's bit, so the compare happens inside a
	// function that only runs for pages which already notify.  Writes to every
	// other page execute exactly the instructions they did before; the one bit
	// is the entire hot-path cost.  noteRAMWriteSideEffects remains the
	// authority for its own two tests, unchanged -- this adds a third test to
	// it, it does not reinterpret the first two.
	//
	// THE COST THAT IS NOT FREE, stated so nobody is surprised by it: the
	// watched address's 4K page now takes the out-of-line call on EVERY write
	// into it.  $9F88 is page $9, which Mario writes rarely.  A page-$0 watch
	// would put that call on every zero-page and stack store and must be
	// measured before it ships; RAM_WATCH 0 restores the pre-change bitmap
	// exactly.
	//
	// SELF-FREEZING, so this ring needs no detector to stop it.  If the hang is
	// "the byte stops being written" the ring stops being written too and
	// preserves the last writes before the stall by construction.  If instead
	// the writes continue, that IS case (b) or (c) and the newest entries are
	// the evidence.  Either way the ring holds what matters without a trip.
	struct WatchedWrite
	{
		// Both stamps come from the 65816 core, NOT from c64Now(): m_Cycles is
		// what the interrupt ring stamps, and the whole point of this ring is
		// to be read against that one.  Mixing the two clock domains has
		// already wrecked conclusions in this investigation.
		u32 pc;			// ( PBR << 16 ) | PC, pointing just PAST the writing
						// instruction -- the operand fetches have happened, so
						// $09B0 STY $9F88 records $09B3
		u32 cycles;		// low 32 of the core's m_Cycles
		u16 addr;
		u8  value;
		u8  old;
	};
	static const u32 SCPU_WATCH_RING = 32;
	WatchedWrite m_WatchRing[ SCPU_WATCH_RING ] = {};
	u32 m_WatchPos = 0;
	u32 m_WatchWrites = 0;		// total, so "did the ring lap?" is answerable
	u32 m_WatchChanges = 0;		// writes that actually changed the byte --
								// case (b) is m_WatchWrites high with this flat

	// --- $D012 read forensics ---------------------------------------------
	// What raster the GUEST last saw, with ages.  Costs one compare and two
	// stores on the I/O read path only, never on the RAM path.  The general
	// m_IOLog already carries the values, but not when they happened, and "the
	// chain is still reading $D012 but the compare never matches" and "the
	// chain stopped reading $D012 eight million cycles ago" are the two
	// readings this has to distinguish.
	static const u32 SCPU_D012_RING = 8;
	u8  m_D012Reads[ SCPU_D012_RING ] = {};
	u32 m_D012ReadCyc[ SCPU_D012_RING ] = {};
	u32 m_D012ReadPos = 0;
	u32 m_D012ReadCount = 0;

	// Free-running emulated cycle counter, for timestamping. Incremented in
	// tickFast, so it costs one add on a path that already runs per
	// instruction.
	u64 m_EmuCycles = 0;

	// --- the BA stall's instrument -----------------------------------------
	// DELIBERATELY HERE AND NOT BESIDE m_BAStallMode, which is where these
	// belong by subject. Those members sit AHEAD of m_RAM, so inserting 80
	// bytes there moves every offset the hot shadow paths index through.
	// MEASURED both ways with the project's own aarch64-none-elf-g++ 13.2.1 at
	// the flags Source/Makefile actually gives each unit (w65c816.o -Os,
	// c64_memory.o and memory_map.o -O3, App -O2): declared HERE,
	// CW65C816::stepInner is BYTE-IDENTICAL, 0x633c before and after, and
	// memory_map.o is unchanged. Declared beside m_BAStallMode it is the same
	// 0x633c but no longer the same code -- four displacement immediates in the
	// hottest loop move by 80 bytes. Same size, so this placement buys
	// certainty rather than instructions; an instrument gets to cost nothing at
	// all. (An earlier draft of this note claimed 0x96fc -> 0x972c, +48 bytes
	// and twelve instructions. Those are -O2 figures for a unit the firmware
	// builds at -Os, and the +48 does not reproduce at either level. Left
	// written down because measuring the wrong configuration is the easiest
	// mistake to repeat here.)
	//
	// Per-event steal lengths, 16 bins of 5 C64 cycles, bin 15 = 75 and over.
	// The mean these replace is not a usable number: K456 reported 38011 events
	// over 1200319 cycles = 31.6 apiece, which is neither a sprite steal (2-3)
	// nor a badline (43) but the average of a mixture whose top is censored by
	// RAD_BA_STALL_CEILING. capped= in the same diagnostic line says how much
	// of bin 15 was censored, so the two are read together or not at all.
	u32 m_BAStallHist[ 16 ] = {};
	// Host stamp taken in the return path of the spin, i.e. at the BA RELEASE,
	// and the length of the steal that produced it. The length is what makes
	// the stamp attributable: see SCPU_VSP_RELEASE_BADLINE.
	// Spins the anchor-setting $D012 read made, captured in
	// noteVSPRasterAnchor while radBASpinsLastRead still belongs to it.
	u16 m_VSPAnchorBASpins = 0;
	// The last release of ANY class, for postrel=.
	u64 m_BAStallLastReleaseHost = 0;
	// The last BADLINE-CLASS release, kept separately because a sprite steal
	// releasing afterwards would otherwise replace the one reference in this
	// instrument that corresponds to a known in-line cycle. Same reason
	// SCPU_VSP_RELEASE_BADLINE exists on the event flags, one level up.
	u64 m_BAStallLastBadlineHost = 0;
	u16 m_BAStallLastBadlineLen = 0;
	u16 m_BAStallLastReleaseLen = 0;
	u64 m_VSPStallEventsAtLast = 0;		// VSPTimingEvent::baStallEvents
private:
	void noteVSPRasterAnchor( u8 raster );
	// Decide what $D011 bit 7 must be for a guest read whose physical sample
	// came back with that bit SET.  Out of line on purpose: it is reached only
	// by a $D011 read that claims bit 8, which is 7 lines in 263 on NTSC, and
	// inlining it would put a second bus call and a signals() load inside
	// read8's I/O arm for every register in the window.  Returns 0x80 or 0.
	u8 d011RasterHighBit();
	void appendVSPTiming( u16 address, u8 value, u8 physicalValue,
	                      bool inWindow );
	VSPTimingEvent m_VSPTrace[ SCPU_VSP_TRACE_SIZE ] = {};
	u32 m_VSPTraceHead = 0;
	bool m_VSPHaveRasterAnchor = false;
	u8  m_VSPLastRaster = 0;
	u64 m_VSPLastRasterHost = 0;
	// THE CATCH-UP'S ANCHOR: the host instant the pacer's grid assigns to the
	// ledger position of the anchor read -- where an on-time settle for that
	// read's cycle exits, a quarter cycle after the falling edge that precedes
	// the read's own cycle (tickSettle's lock aim, 3*perC64/4 past a rise).
	// Built from the primitive's own start-of-cycle stamp, NOT from the read's
	// return above: the return is one whole cycle plus a teardown later, and a
	// count target built on it held every placed write until the START of the
	// cycle it belonged in, which the primitive's resync then carried a whole
	// cycle past (the constant skew the oracle screen showed as a cell).
	// m_VSPLastRasterHost stays what the trace has always recorded.
	u64 m_VSPLastRasterGridHost = 0;
	// The C64-CYCLE companion to the host stamp above, and the pair is the
	// whole point: the guest's $D012 read is a PHYSICAL time reference, so the
	// cycles it counts after that read have to be cycles the VIC actually
	// advanced through. One stamp is real time, the other is what we charged,
	// and the difference IS the placement error. c64Now(), never m_EmuCycles:
	// that one is accelerator-rate.
	u64 m_VSPLastRasterC64 = 0;

	void noteIECActivity();
	// Does a CIA2 output change that reaches the IEC pins count as a
	// transaction in the current window mode? See m_IECWindowMode.
	bool iecWriteEvidence( u8 driveBefore, u8 driveAfter ) const;
	// One arm site shared by the $DD00 and $DD02 write paths, so the mode rule
	// and its counters cannot drift apart between them.
	void noteIECWriteArm( bool driveKnown, u8 driveBefore, u8 driveAfter );
	TimingHook m_TimingHook;
	void      *m_TimingHookCtx;

public:
	// --- guest-stop attribution --------------------------------------------
	// CW65C816::run() is the ONLY place the emulated CPU executes, and
	// CSuperCPU::runFrame() has exactly two call sites. The interval between a
	// return and the next entry is therefore exactly the wall time the guest
	// was stopped while the physical raster kept moving -- which is the
	// quantity the K478 displacement needs and the one nothing currently
	// measures. Two PMCCNTR reads per frame slice: 128 slices at 60Hz is about
	// 15k a second, bracketing millions of instructions, and both are system
	// register reads rather than the uncached GPIO MMIO that made per
	// instruction interrupt sampling expensive.
	//
	// Deliberately NOT a per-instruction measurement. Stamping the host clock
	// inside the batch loop is the configuration this file's interrupt note
	// already records as one of the largest single costs in the emulator.
	inline void noteGuestStopped()
	{
		m_GuestStopAt = m_C64 ? m_C64->hostCycles() : 0;
		m_GuestStopCauses = 0;
		m_GuestStopValid = true;
	}
	inline void noteGuestStopCause( u32 bits ) { m_GuestStopCauses |= bits; }
	inline void noteGuestResumed()
	{
		// The first resume of a run has no stop to close, and a bus with no
		// host clock returns zero for both stamps; neither may invent a stall.
		if ( !m_GuestStopValid ) return;
		m_GuestStopValid = false;
		const u64 now = m_C64 ? m_C64->hostCycles() : 0;
		if ( now <= m_GuestStopAt ) return;
		const u64 held = now - m_GuestStopAt;
		const u32 clamped = held > 0xFFFFFFFFULL ? 0xFFFFFFFFu : (u32)held;
		// A never-drained high-water mark FIRST, because the field below is
		// drained and zeroed by appendVSPTiming on every retained event: a run
		// whose worst stop is not followed by a $D011 write loses it entirely,
		// and the 2026-10-12 displacement is exactly one such event in 63. Four
		// instructions on a path that already runs twice per frame slice, so it
		// cannot perturb what it observes.
		if ( clamped > m_GuestStopPeakHost )
		{
			m_GuestStopPeakHost = clamped;
			m_GuestStopPeakCauses = m_GuestStopCauses;
		}
		if ( clamped <= m_GuestStopMaxHost ) return;
		m_GuestStopMaxHost = clamped;
		m_GuestStopMaxCauses = m_GuestStopCauses;
	}
	// Exposed for the host tests, which have no VSP trace to drain them into.
	u32 guestStopMaxHost() const { return m_GuestStopMaxHost; }
	u32 guestStopMaxCauses() const { return m_GuestStopMaxCauses; }
	// Never drained; boot.cpp prints these as the guest-stopped peak= line.
	u32 guestStopPeakHost() const { return m_GuestStopPeakHost; }
	u32 guestStopPeakCauses() const { return m_GuestStopPeakCauses; }

	// --- RST8 forensics ----------------------------------------------------
	// $D011 bit 7 is the ONLY bit of any register this emulator folds a live
	// physical sample into a value it then KEEPS in a shadow and derives a
	// decision from (m_RasterCompare).  That is what made one bad bit fatal:
	// K471 captured a single DEC/INC $D011 pair writing $96/$97 out of 21,
	// which put RST8 in the latch, composed a compare of 318, and parked
	// Mario's chain for ever on a line NTSC does not have.
	//
	// These count the arms of d011RasterHighBit() so the next capture PROVES
	// which of the two possible causes it was instead of inferring it from the
	// absence of a symptom. samples against rejected discriminates "the beam
	// really was at 256..262" from "the bit did not come from the raster at
	// all", and Writes is the direct test asked for: with the check in place no
	// read-modify-write may put RST8 in the latch, so it must read 0 across a
	// gameplay capture.
	//
	// DECLARED LAST IN THE CLASS, after m_TimingHookCtx, and that placement is
	// the whole of the measurement note below. The obvious home -- beside the
	// $D012 read ring these are read with -- sits immediately BEFORE
	// m_EmuCycles, which stepInner indexes on every instruction. Measured with
	// the project's own aarch64-none-elf-g++ 13.2.1 at the flags Source/Makefile
	// gives each unit (w65c816.o -Os, c64_memory.o -O3): declared there, 24
	// bytes of counters moved m_EmuCycles from +105040 to +105064 and
	// CW65C816::stepInner changed from 0x633c to 0x62dc with four displacement
	// immediates rewritten in the hottest loop. Declared HERE it is 0x633c and
	// byte-identical, because nothing the core touches moves at all. Turbo is
	// 43 and must stay 43; this is how.
	u32 m_D011RST8Samples = 0;		// physical reads that came back bit 7 set
	u32 m_D011RST8Rejected = 0;		// ... and were refused as impossible
	u8  m_D011RST8LastLow = 0;		// confirming $D012 of the newest sample
	u8  m_D011RST8LastRejLow = 0;	// ... and of the newest REFUSED one
	u32 m_D011RST8Writes = 0;		// guest $D011 writes carrying RST8
	u32 m_RasterCompareImpossible = 0;	// composed compares >= rasterLines
	u32 m_RasterAnchorRefused = 0;		// anchors refused for the same reason
	// --- BA-stall re-entry spacing -----------------------------------------
	// RE-ENTRY SPACING, and it is the quantitative half of the sprite test.
	// A freeze's reconstructed ENTRY against the PREVIOUS release, because
	// near-contiguous BA-low runs are the ONLY shape in which charging per run
	// can total more than a 6510 lost: busStealCycles() exits on the first
	// BA-high sample, so one freeze is one contiguous run, and two runs
	// separated by real CPU cycles are each charged exactly the cycles the guest
	// did not get, because we return between them and the guest executes.
	//
	// TWO WINDOWS, not one, and that is a correction to this instrument's first
	// draft. We notice BA low only at an access boundary, so the soonest
	// re-entry we can OBSERVE is one guest instruction after the release -- 2 to
	// 7 C64 cycles. A 2-cycle window is therefore narrower than our own sampling
	// quantum and would read about zero under BOTH hypotheses, which falsifies
	// nothing. B2B counts re-entries within 8 cycles, wider than the longest
	// instruction in Mario's stabiliser; Tight keeps the 2-cycle reading, so the
	// narrow rule stays legible and widening becomes an analysis choice made
	// against one capture instead of two.
	//
	// B2BCycles is the exact upper bound on any correction this mechanism would
	// justify, known BEFORE it is tried -- which is what stops it quietly taking
	// cycles back out of the sled the stall repaired.
	//
	// DECLARED LAST, measured and not assumed, for the reason the
	// m_D011RST8Samples note above gives at length. Declared beside
	// m_BAStallLastReleaseLen, where these belong by subject, these bytes
	// perturb read8, write8, the inlined tickFast and a dozen cold functions --
	// displacement immediates rewritten, same instructions and same counts, but
	// no longer the same code. Declared HERE, read8, write8, tickFast and
	// stepInner are all BYTE-IDENTICAL at the flags Source/Makefile gives each
	// unit (c64_memory.o -O3, w65c816.o -Os). Both placements were built and
	// diffed per function.
	u32 m_BAStallB2B = 0;		// re-entries within 8 C64 cycles of a release
	u32 m_BAStallB2BTight = 0;	// ... of which within 2, the original window
	u32 m_BAStallB2BShort = 0;	// ... of which sprite-class in LENGTH (<= 5)
	u64 m_BAStallB2BCycles = 0;	// cycles charged to the 8-cycle set
	// Did the catch-up fire, and by how much. Events 0 means the window was
	// never open at factor 1 and the write is LATE rather than early, which
	// retires the mechanism in one round. A mean deficit near one cycle says
	// the lag came from the handler's own physical accesses; near six says a
	// slice-boundary rasterLine() landed inside the window, which is the
	// recorded dissent. Declined counts the above-cap anomalies the catch-up
	// refuses to spin out.
	u64 m_VSPCatchupEvents = 0;
	u64 m_VSPCatchupHost = 0;
	u64 m_VSPCatchupDeclined = 0;
	// VSP_AIM_WRITE (K498). Signed whole C64 cycles; the per-anchor flag marks
	// the first placed write after a $D012 anchor read, which is the one that
	// receives the aim as a floor on its own entry so an entry-bound write
	// still moves by exactly the aim. Statistics separate the aim's own wait
	// from the catch-up's.
	s32  m_VSPAimWriteCycles = 0;
	bool m_VSPAimFloorPending = false;
	u64  m_VSPAimWaitHost = 0;
	u64  m_VSPAimEvents = 0;
	// The catch-up's view of the most recent placed VIC write, for the trace
	// record (K499). Three stores inside a block that already spins.
	bool m_VSPLastCatchValid = false;
	s64  m_VSPLastCatchDeficitHost = 0;
	u64  m_VSPLastCatchWaitHost = 0;
	// VSP_D011_SHADOW_READ (K500): the gate, and how often it served a $D011
	// read without a transfer or declined one inside the window because the
	// anchor's low byte could not prove the raster MSB.
	bool m_VSPD011ShadowRead = false;
	u64  m_VSPD011ShadowReads = 0;
	u64  m_VSPD011ShadowDeclined = 0;
	// VSP_PLACEMENT_K501 (K505 bisect key): false restores K500's placement
	// for the three K501 parts (see config.h). Default true.
	bool m_VSPPlacementK501 = true;
	// VSP_TRACE_WINDOW_ONLY (K506): the trace ring keeps only windowed writes.
	bool m_VSPTraceWindowOnly = false;
	// RUN-WIDE HISTOGRAMS (K503), for the FIRST placed $D011 write after each
	// $D012 anchor read (Mario's DEC). The trace ring holds the last ~21 of
	// them; a glitch seen once or twice a minute never survives to the card.
	// Two things are counted over the whole run, each relative to the first
	// observation so no model enters: the observed /IRQ assertion to that
	// write in whole C64 cycles (the absorbed jitter shows as two adjacent
	// buckets of similar size; a third bucket is the unabsorbed frame), and the
	// anchor read's low byte (Mario: $35/$36 roughly even; anything else is a
	// read that landed outside the line boundary).
	// Absolute (K504): the relative form anchored on whatever handler produced
	// the first sample -- a title screen's -- and put all of gameplay in
	// "other". Whole cycles 0..127 and the raw anchor byte; the summary prints
	// the non-zero buckets only.
	bool m_VSPHistPending = false;
	u32  m_VSPHistIrq[ 128 ] = {};		// IRQ->first placed $D011 write, whole cycles
	u32  m_VSPHistIrqOther = 0;			// 128 cycles or more
	u32  m_VSPHistAnchor[ 256 ] = {};	// the anchor read's low byte
	// --- guest-stop attribution state --------------------------------------
	// DECLARED LAST, for the reason the m_D011RST8Samples note above gives at
	// length: bytes added before m_EmuCycles move the displacement immediates
	// stepInner uses on every instruction. These are touched twice per frame
	// slice and never by the core.
	u64 m_GuestStopAt = 0;			// host stamp of the last interpreter exit
	u64 m_IRQAssertHost = 0;		// host stamp of the newest observed /IRQ assert
	u32 m_GuestStopMaxHost = 0;		// longest stop since the last retained event
	u32 m_GuestStopCauses = 0;		// bits accumulated inside the CURRENT stop
	u32 m_GuestStopMaxCauses = 0;	// ... as they stood for the longest one
	u32 m_GuestStopPeakHost = 0;	// worst stop of the whole run, never drained
	u32 m_GuestStopPeakCauses = 0;	// ... and the cause bits for that one
	u16 m_VSPD012Reads = 0;			// guest $D012 reads since that event
	u8  m_VSPD012First = 0;			// ... and the first value returned
	bool m_GuestStopValid = false;	// a stop is open and may be closed

	// --- the interrupt-control write ring ----------------------------------
	// See VICIntCtlWrite at the top of this file for what this answers.
	//
	// DECLARED LAST, for the reason the m_D011RST8Samples note above gives at
	// length: bytes added anywhere ahead of m_EmuCycles move the displacement
	// immediates stepInner uses on every instruction. Nothing here is on a
	// per-instruction path -- the appends are per VIC interrupt-control write,
	// about eight a frame in Mario, and interruptContext() is one cold call per
	// interrupt.
	//
	// Append one entry. Called AFTER the physical write has completed, so a
	// $D019 entry's own /IRQ sample is taken against a chip that has already
	// seen the acknowledgement. Out of line: it is reached by four registers
	// out of 47 and the I/O write path has no business carrying it inline.
	__attribute__((noinline)) void appendIntCtlWrite( u8 reg, u8 value,
	                                                  u8 prev );
	// Latch the ring. reason 1 (impossible compare) locks it; reason 2 (a
	// progress-detector SUSPECT episode) re-copies until then.
	void noteIntCtlLate( u64 host );
	void freezeIntCtlRing( u32 reason );
	void noteProgressSuspects( u32 suspects );
	// compare | d01a | raster | rasterAge | flags, for one cold call per
	// interrupt. Pure observation of state already held: no bus cycle, no GPIO
	// read and -- deliberately -- no CIA access at all. $DC0D and $DD0D CLEAR
	// ON READ, so CIA interrupt status is never sampled for observation from
	// this path; what we can know about a CIA source comes from the mask
	// WRITES the guest made, which m_CIALog already carries, and from the
	// /IRQ pin's behaviour across the VIC acknowledgement.
	u64 interruptContext();
	VICIntCtlWrite m_ICtlRing[ SCPU_ICTL_RING ] = {};
	// THE LATCHED COPY, and it is the difference between a capture that
	// answers the question and one that does not. The button is pressed
	// seconds after the hang and the chain keeps writing $D019 and $D012
	// throughout it, so by then the arming write we need is long overwritten.
	// Latched at the first IMPOSSIBLE COMPARE -- the measured signature of
	// this failure, K478's 318 on a 263-line machine -- and, as a
	// hypothesis-free backstop, re-copied at each progress-detector SUSPECT
	// episode until an impossible compare locks it. Same two-stage shape, and
	// the same reasoning, as CW65C816::m_IntRingSuspect.
	VICIntCtlWrite m_ICtlFrozen[ SCPU_ICTL_RING ] = {};
	u32 m_ICtlHead = 0;
	u32 m_ICtlFrozenHead = 0;
	u32 m_ICtlFrozenReason = 0;		// 0 none, 1 impossible compare, 2 suspect
	u32 m_ICtlFrozenCycles = 0;		// core cycles at the latch
	u32 m_D019Writes = 0;			// guest acknowledgements seen
	u32 m_D019AckHeldIRQ = 0;		// ... with /IRQ still low immediately after
	u32 m_D019AckHeldIRQLate = 0;	// ... and still low at the next event
	u32 m_ICtlSuspectsSeen = 0;		// SUSPECT episodes already latched
	u64 m_ICtlFollowHost = 0;		// host stamp of the entry awaiting IRQ_LATE
	// THE EMULATED-CLOCK STAMPS THE FOLLOW-UP VERDICT IS DECIDED ON: the emulated
	// cycle of the pending control write, and of the last REAL /IRQ pin read.
	// noteIntCtlLate compares these two instead of trusting m_IntCredit, which is
	// frozen inside an instruction batch. Both start 0, and reset() re-zeroes
	// them with m_EmuCycles, so the gate refuses rather than invents.
	//
	// DECLARED HERE, WITH THE RING, BECAUSE THEY ARE COLD: written on the ring
	// path (four VIC registers out of 47) and in the once-per-emulated-microsecond
	// sampling block, never per instruction -- so they must not take space in a
	// hot line. The per-instruction flag they work with, m_IRQResampleDue, is in
	// m_PostedWritePending's line for exactly the opposite reason; see there.
	// MEASURED: adding these two leaves every pre-existing member offset
	// byte-identical (m_EmuCycles 105040, m_IntCredit 86672, m_RasterCompare
	// 103380, host build), so stepInner's displacement immediates do not move.
	u64 m_ICtlFollowEmu = 0;
	u64 m_IRQSampleEmu = 0;
	u16 m_ICtlFollowDue = 0;		// 1 + its ring index, 0 for none
	u8  m_ICtlFreezeDue = 0;		// an impossible compare awaits its latch
	u8  m_ICtlAckHeldPending = 0;	// bit 0 immediate, bit 1 late, since the
									// previous interrupt entry
	u64 m_D011RST8LastHost = 0;		// host stamp of the praster= sample
};

#endif
