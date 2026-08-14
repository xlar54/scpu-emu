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
	// Observe core 0's existing CIA2 transfer total. Core 1 is the only reader;
	// core 0 never waits for or polls the renderer.
	void watchBusActivity( const volatile u64 *transfers );

	// Register the renderer on otherwise-idle core 1. Returns immediately.
	bool start();
	bool firstFrameReady() const;
	u32 maxBandUS() const;
	u32 missedBandDeadlines() const;
	void resetRuntimeDiagnostics();

private:
	enum
	{
		HDMI_RENDER_BAND_ROWS = 1,
		HDMI_FRAMEBUFFER_SPAN_BYTES = 64,
		HDMI_SERIAL_BUSY_ACCESSES = 32,
		HDMI_SERIAL_ABORT_ACCESSES = 4
	};
	static void core1Entry( void *context );
	void run();
	void renderFrame( u64 frameStart, u64 frameTicks );
	bool presentRows( const u8 *source, u32 firstRow, u32 rowCount );
	bool serialActive() const;
	void fillFrameBuffer( u8 colour );

	const CC64Memory &m_Memory;
	CBcmFrameBuffer  *m_FrameBuffer;
	u8               *m_FramePixels;
	u32               m_FBPitchBytes;
	u8                m_Colours[ 1024 ];
	// Core-1-private representation of the active VIC bank. renderFrame()
	// populates only the active screen, charset/bitmap and sprite shapes.
	u8                m_VICRAM[ 0x4000 ];
	u8                m_BandPixels[ VIC_RENDER_WIDTH * HDMI_RENDER_BAND_ROWS ];
	CVICRenderer       m_Renderer;
	volatile u32       m_Started;
	volatile u32       m_FirstFrame;
	volatile u64       m_MaxBandTicks;
	volatile u32       m_MissedBandDeadlines;
	const volatile u64 *m_SerialTransfers;
	u64                m_LastSerialTransfers;
	u64                m_FrameStartSerialTransfers;
	u64                m_SerialBusySkips;
	u64                m_SerialAborts;
};

#endif
