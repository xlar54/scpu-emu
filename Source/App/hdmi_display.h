/*
   SCPU-EMU - passive HDMI presentation of VIC-II shadow state

   Core 1 only reads Pi memory and writes the already-initialised Circle
   framebuffer.  It cannot reach the Commodore bus.
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

	// Validate and borrow the console framebuffer. Does not paint it and does
	// not start core 1, so calibration and self-test remain unchanged.
	bool initialize( CBcmFrameBuffer *frameBuffer, CLogger *logger );
	void showReady();
	void showFailure();

	// Register the renderer on otherwise-idle core 1. Returns immediately.
	bool start();
	bool firstFrameReady() const;

private:
	static void core1Entry( void *context );
	static bool serialAbortCheck( void *context );
	void run();
	void renderFrame();
	bool presentChangedRows();
	void fillFrameBuffer( u8 colour );
	bool serialBlocked();

	const CC64Memory &m_Memory;
	CBcmFrameBuffer  *m_FrameBuffer;
	u16              *m_FramePixels;
	u32               m_FBWidth;
	u32               m_FBHeight;
	u32               m_FBPitchPixels;
	u32               m_Scale;
	u32               m_OriginX;
	u32               m_OriginY;
	u16               m_Palette[ 16 ];
	u8                m_Colours[ 1024 ];
	u8                m_Pixels[ VIC_RENDER_WIDTH * VIC_RENDER_HEIGHT ];
	u8                m_Previous[ VIC_RENDER_WIDTH * VIC_RENDER_HEIGHT ];
	CVICRenderer       m_Renderer;
	u32                m_LastSerialAccess;
	u64                m_SerialBlockUntil;
	u64                m_SerialBusySkips;
	u64                m_SerialAborts;
	volatile u32       m_Started;
	volatile u32       m_FirstFrame;
};

#endif
