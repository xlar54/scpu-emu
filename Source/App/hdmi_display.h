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

	// Register the renderer on otherwise-idle core 1. Returns immediately.
	bool start();
	bool firstFrameReady() const;
	u32 maxBandUS() const;
	u32 missedBandDeadlines() const;

private:
	enum { HDMI_RENDER_BAND_ROWS = 1 };
	static void core1Entry( void *context );
	void run();
	void renderFrame( u64 frameStart, u64 frameTicks );
	void presentRows( const u8 *source, u32 firstRow, u32 rowCount );
	void fillFrameBuffer( u8 colour );

	const CC64Memory &m_Memory;
	CBcmFrameBuffer  *m_FrameBuffer;
	u8               *m_FramePixels;
	u32               m_FBPitchBytes;
	u8                m_Colours[ 1024 ];
	u8                m_BandPixels[ VIC_RENDER_WIDTH * HDMI_RENDER_BAND_ROWS ];
	CVICRenderer       m_Renderer;
	volatile u32       m_Started;
	volatile u32       m_FirstFrame;
	volatile u64       m_MaxBandTicks;
	volatile u32       m_MissedBandDeadlines;
};

#endif
