/*
   SCPU-EMU - timestamped VIC-II state changes for HDMI raster replay
*/
#ifndef _scpu_vic_raster_h
#define _scpu_vic_raster_h

#include "../Common/types.h"

// One VIC-visible state change, timestamped in C64 bus cycles. Registers
// $000-$03F are VIC-II registers, $040 is the effective CIA2-selected VIC
// bank, and $100-$4FF are colour-RAM cells. Bits 14/15 carry an exact 9-bit
// raster line when the write happened in a known raster IRQ handler.
struct VICRegWrite
{
	u32 cycle;
	u16 reg;
	u8  value;
	u8  lineLow;
};

#define VIC_LOG_REG_MASK       0x07FF
#define VIC_LOG_LINE_VALID     0x4000
#define VIC_LOG_LINE_HIGH      0x8000
#define SCPU_VIC_LOG_SIZE      2048
#define VIC_RASTER_MAX_BANDS    273
#define VIC_RASTER_PAGE_MASK_BYTES 32
#define VIC_RASTER_MAX_SNAPSHOT_PAGES 96

static inline u16 vicLogRegister( const VICRegWrite &event )
{
	return (u16)( event.reg & VIC_LOG_REG_MASK );
}

static inline bool vicLogHasRasterLine( const VICRegWrite &event )
{
	return ( event.reg & VIC_LOG_LINE_VALID ) != 0;
}

static inline u16 vicLogRasterLine( const VICRegWrite &event )
{
	return (u16)( event.lineLow
	       | ( ( event.reg & VIC_LOG_LINE_HIGH ) ? 0x100 : 0 ) );
}

struct VICRasterTiming
{
	u32 cyclesPerLine;
	u32 rasterLines;
	u32 frameRows;
	s32 topRaster;
	u32 displayFirstRow;
	u32 displayRows;
};

struct VICRasterBand
{
	u16 firstRow;
	u16 rowCount;
	// Apply [applyBegin, applyEnd) before rendering this band. Indices are
	// monotonic log sequence numbers, not masked ring offsets.
	u32 applyBegin;
	u32 applyEnd;
};

enum VICRasterFrameResult
{
	VIC_RASTER_SNAPSHOT = 0,	// re-prime state from the live shadow
	VIC_RASTER_HOLD,			// newest machine frame is still open
	VIC_RASTER_REPLAY
};

struct VICRasterFrame
{
	VICRasterFrameResult result;
	u32 drawStart;
	u32 drawEnd;
	u32 tailAfter;
	u32 bandCount;
	VICRasterBand bands[ VIC_RASTER_MAX_BANDS ];
	// Events below/outside the final output row still become the starting
	// state of the next frame even though they draw no pixels in this one.
	u32 finishBegin;
	u32 finishEnd;
};

// One planned output band plus the exact VIC-visible scalar state in force
// across it. Source RAM and colour RAM are snapshotted separately once per
// completed frame; this keeps every band coherent without pretending to replay
// mid-frame memory rewrites.
struct VICRasterReplayBand
{
	u16 firstRow;
	u16 rowCount;
	u8  vic[ 0x40 ];
	u8  bank;
};

struct VICRasterReplayPlan
{
	VICRasterFrameResult result;
	bool fallback;
	bool yScrollVaries;
	u32 bandCount;
	u32 pageCount;
	u8 pageMask[ VIC_RASTER_PAGE_MASK_BYTES ];
	VICRasterReplayBand bands[ VIC_RASTER_MAX_BANDS ];
};

class CVICRasterTimeline
{
public:
	CVICRasterTimeline() : m_Tail( 0 ), m_Primed( false ) {}

	void prime( u32 head ) { m_Tail = head; m_Primed = true; }
	void invalidate() { m_Primed = false; }
	u32 tail() const { return m_Tail; }

	VICRasterFrameResult build( const VICRegWrite *log, u32 head,
	                            u32 now, u64 anchor,
	                            const VICRasterTiming &timing,
	                            VICRasterFrame &frame );

	static s32 lineOfCycle( u32 cycle, u64 anchor,
	                        const VICRasterTiming &timing );
	static s32 lineOfEvent( const VICRegWrite &event, u64 anchor,
	                        const VICRasterTiming &timing );
	static u32 rowOfEvent( const VICRegWrite &event, u64 anchor,
	                       const VICRasterTiming &timing );
	static bool yScrollVariesInDisplay( const VICRegWrite *log,
	                                    const VICRasterFrame &frame,
	                                    u64 anchor,
	                                    const VICRasterTiming &timing,
	                                    u8 initialD011 );

private:
	u32  m_Tail;
	bool m_Primed;
};

// Stateful consumer of CVICRasterTimeline. Its register/bank state lives at
// the timeline's monotonic tail, so every event is applied exactly once. A
// producer reset/generation change, an overwritten ring window, or an invalid
// anchor discards that private state and re-primes from the live shadow.
class CVICRasterReplay
{
public:
	CVICRasterReplay();

	VICRasterFrameResult build( const VICRegWrite *log, u32 head,
	                            u32 now, u64 anchor, u32 generation,
	                            const VICRasterTiming &timing,
	                            const u8 liveVIC[ 0x40 ], u8 liveBank,
	                            const u8 *liveRAM,
	                            VICRasterReplayPlan &plan );
	void invalidate();

	u32 resyncs() const { return m_Resyncs; }
	u32 lostEventResyncs() const { return m_LostEventResyncs; }
	u32 resetResyncs() const { return m_ResetResyncs; }
	u32 snapshotFallbacks() const { return m_SnapshotFallbacks; }
	u32 tail() const { return m_Timeline.tail(); }

private:
	void rePrime( u32 head, u32 generation, const u8 liveVIC[ 0x40 ],
	              u8 liveBank );
	void makeStaticPlan( VICRasterFrameResult result,
	                     const VICRasterTiming &timing,
	                     const u8 liveVIC[ 0x40 ], u8 liveBank,
	                     const u8 *liveRAM, bool fallback,
	                     VICRasterReplayPlan &plan );
	static void applyEvent( u8 vic[ 0x40 ], u8 &bank,
	                        const VICRegWrite &event );
	static bool collectPages( VICRasterReplayPlan &plan,
	                          const u8 *liveRAM );

	CVICRasterTimeline m_Timeline;
	u8   m_VIC[ 0x40 ];
	u8   m_Bank;
	u32  m_Generation;
	bool m_Primed;
	u32  m_Resyncs;
	u32  m_LostEventResyncs;
	u32  m_ResetResyncs;
	u32  m_SnapshotFallbacks;
};

#endif
