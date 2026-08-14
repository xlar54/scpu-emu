/*
   SCPU-EMU - passive HDMI presentation of VIC-II shadow state

   Core 1 only reads Pi memory and writes a dedicated indexed framebuffer.
   It cannot reach the Commodore bus.
*/
#ifndef _scpu_hdmi_display_h
#define _scpu_hdmi_display_h

#include <circle/bcmframebuffer.h>
#include <circle/logger.h>
#include "../Common/types.h"
#include "../Video/vic_raster.h"
#include "../Video/vic_renderer.h"

class CC64Memory;

class CHDMIDisplay
{
public:
	explicit CHDMIDisplay( const CC64Memory &memory );

	// Replace the firmware console with a 384x272 8bpp indexed framebuffer.
	// The VideoCore scales this small surface to the configured HDMI mode, so
	// the ARM never expands pixels. Does not start core 1.
	bool initialize( CLogger *logger );
	void showReady();
	void showFailure();
	// Observe core 0's existing CIA2 transfer total. Core 1 is the only reader;
	// core 0 never waits for or polls the renderer.
	void watchBusActivity( const volatile u64 *transfers );
	void setVideoTiming( u32 cyclesPerLine, u32 rasterLines,
	                     u32 displayFirstLine );

	// Register the renderer on otherwise-idle core 1. Returns immediately.
	bool start();
	bool firstFrameReady() const;
	// Core 1 is permanent once registered with Circle. Runtime VIC/HDMI
	// switching therefore parks the renderer instead of returning from it.
	// presentedFrames() lets core 0 wait for one newly completed frame without
	// ever waiting on core 1 inside an IEC-critical interval.
	void setPictureEnabled( bool enabled );
	bool pictureEnabled() const;
	bool pictureParked() const;
	u32 presentedFrames() const;
	volatile u32 *spriteSpriteCollisionLatch()
		{ return &m_SpriteSpriteCollision; }
	volatile u32 *spriteBackgroundCollisionLatch()
		{ return &m_SpriteBackgroundCollision; }
	void resetCollisionLatches();
	u32 maxBandUS() const;
	u32 missedBandDeadlines() const;
	u32 rasterResyncs() const { return m_RasterReplay.resyncs(); }
	u32 rasterLostResyncs() const { return m_RasterReplay.lostEventResyncs(); }
	u32 rasterResetResyncs() const { return m_RasterReplay.resetResyncs(); }
	u32 rasterFallbacks() const { return m_RasterReplay.snapshotFallbacks(); }
	void resetRuntimeDiagnostics();

private:
	enum
	{
		HDMI_RENDER_BAND_ROWS = 1,
		HDMI_FRAMEBUFFER_SPAN_BYTES = 64,
		HDMI_SHARED_SPAN_BYTES = 64,
		HDMI_IEC_SPAN_BYTES = 16,
		HDMI_IEC_SPAN_PAUSE_US = 5,
		HDMI_SERIAL_BUSY_ACCESSES = 32,
		HDMI_SERIAL_ABORT_ACCESSES = 4
	};
	static void core1Entry( void *context );
	void run();
	void renderFrame( u64 frameStart, u64 frameTicks );
	void presentRows( const u8 *source, u32 firstRow, u32 rowCount );
	void copyShared( u8 *destination, const u8 *source, u32 count );
	void observeSerialActivity();
	bool serialActive() const;
	void fillFrameBuffer( u8 colour );

	const CC64Memory &m_Memory;
	CBcmFrameBuffer  *m_FrameBuffer;
	u8               *m_FramePixels;
	u32               m_FBPitchBytes;
	u8                m_Colours[ 1024 ];
	// Core-1-private 64K VIC source image. Only pages selected by the completed
	// frame's bands are refreshed; absolute addresses keep bank-split replay
	// simple and make every rendered band read the same coherent page image.
	u8                m_VICRAM[ 0x10000 ];
	u8                m_LiveVIC[ 0x40 ];
	u8                m_BandPixels[ VIC_RENDER_WIDTH * HDMI_RENDER_BAND_ROWS ];
	CVICRenderer       m_Renderer;
	CVICRasterReplay   m_RasterReplay;
	VICRasterReplayPlan m_RasterPlan;
	VICRasterTiming    m_RasterTiming;
	u32                m_FrameHz;
	volatile u32       m_Started;
	volatile u32       m_FirstFrame;
	volatile u32       m_PictureEnabled;
	volatile u32       m_PictureParked;
	volatile u32       m_PresentedFrames;
	volatile u32       m_SpriteSpriteCollision;
	volatile u32       m_SpriteBackgroundCollision;
	volatile u64       m_MaxBandTicks;
	volatile u32       m_MissedBandDeadlines;
	const volatile u64 *m_SerialTransfers;
	u64                m_LastSerialTransfers;
	u64                m_FrameStartSerialTransfers;
	bool               m_SerialThrottled;
	u64                m_SerialThrottleFrames;
};

#endif
