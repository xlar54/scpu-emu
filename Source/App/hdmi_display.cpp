/*
   SCPU-EMU - passive HDMI presentation of VIC-II shadow state
*/
#include "hdmi_display.h"

#include <circle/synchronize.h>
#include <circle/util.h>

#include "../Bus/RAD/c128_refresh.h"
#include "../C64/c64_memory.h"

// The sixteen VIC-II colours as 0xRRGGBB. Exact shades vary by VIC revision;
// these are stable measured-hardware defaults.
static const u32 s_VICPalette[ 16 ] =
{
	0x000000, 0xFFFFFF, 0x813338, 0x75CEC8,
	0x8E3C97, 0x56AC4D, 0x2E2C9B, 0xEDF171,
	0x8E5029, 0x553800, 0xC46C71, 0x4A4A4A,
	0x7B7B7B, 0xA9FF9F, 0x706DEB, 0xB2B2B2
};

static inline u64 hdmiCounter()
{
	u64 value;
	asm volatile( "mrs %0, CNTPCT_EL0" : "=r" ( value ) );
	return value;
}

static inline u64 hdmiCounterFrequency()
{
	u64 value;
	asm volatile( "mrs %0, CNTFRQ_EL0" : "=r" ( value ) );
	return value ? value : 19200000;
}

static inline void hdmiPauseMicroseconds( u32 microseconds )
{
	const u64 end = hdmiCounter()
	              + hdmiCounterFrequency() * microseconds / 1000000u;
	while ( (s64)( hdmiCounter() - end ) < 0 ) asm volatile( "yield" );
}

// Hold the picture for three milliseconds after the last observed IEC event.
// Repeated serial edges refresh this on core 1, so a transfer holds one still
// frame and a dead protocol always releases the renderer on its own.
static const u32 HDMI_SERIAL_QUIET_US = 3000;
static const u32 HDMI_SOURCE_PIXELS_PER_CHUNK = 8;
static const u32 HDMI_FRAMEBUFFER_BURST_BYTES = 16384;
static const u32 HDMI_FRAMEBUFFER_PAUSE_US = 25;

CHDMIDisplay::CHDMIDisplay( const CC64Memory &memory )
	: m_Memory( memory ), m_FrameBuffer( 0 ), m_FramePixels( 0 ),
	  m_FBWidth( 0 ), m_FBHeight( 0 ), m_FBPitchPixels( 0 ),
	  m_Scale( 1 ), m_OriginX( 0 ), m_OriginY( 0 ),
	  m_LastSerialAccess( 0 ), m_SerialBlockUntil( 0 ),
	  m_SerialBusySkips( 0 ), m_SerialAborts( 0 ),
	  m_Started( 0 ), m_FirstFrame( 0 )
{
	memset( m_Palette, 0, sizeof m_Palette );
	memset( m_Colours, 14, sizeof m_Colours );
	memset( m_Pixels, 0, sizeof m_Pixels );
	memset( m_Previous, 0xFF, sizeof m_Previous );
}

bool CHDMIDisplay::initialize( CBcmFrameBuffer *frameBuffer, CLogger *logger )
{
	m_FrameBuffer = frameBuffer;
	if ( !m_FrameBuffer )
	{
		if ( logger ) logger->Write( "HDMI", LogError, "no console framebuffer" );
		return false;
	}
	if ( m_FrameBuffer->GetDepth() != 16 )
	{
		if ( logger ) logger->Write( "HDMI", LogError,
		                              "framebuffer is %u bpp; 16 required",
		                              (unsigned)m_FrameBuffer->GetDepth() );
		return false;
	}

	m_FramePixels = (u16 *)(uintptr)m_FrameBuffer->GetBuffer();
	m_FBWidth = m_FrameBuffer->GetWidth();
	m_FBHeight = m_FrameBuffer->GetHeight();
	m_FBPitchPixels = m_FrameBuffer->GetPitch() / sizeof( u16 );
	if ( !m_FramePixels || m_FBWidth < VIC_RENDER_WIDTH
	     || m_FBHeight < VIC_RENDER_HEIGHT )
	{
		if ( logger ) logger->Write( "HDMI", LogError,
		                              "framebuffer %ux%u is too small",
		                              (unsigned)m_FBWidth, (unsigned)m_FBHeight );
		m_FramePixels = 0;
		return false;
	}

	const u32 sx = m_FBWidth / VIC_RENDER_WIDTH;
	const u32 sy = m_FBHeight / VIC_RENDER_HEIGHT;
	m_Scale = sx < sy ? sx : sy;
	if ( m_Scale < 1 ) m_Scale = 1;
	m_OriginX = ( m_FBWidth - VIC_RENDER_WIDTH * m_Scale ) / 2;
	m_OriginY = ( m_FBHeight - VIC_RENDER_HEIGHT * m_Scale ) / 2;

	for ( u32 i = 0; i < 16; i++ )
	{
		const u32 rgb = s_VICPalette[ i ];
		// Circle's COLOR16 convention uses five-bit channels at 11, 6 and 0.
		m_Palette[ i ] = (u16)( ( ( ( rgb >> 19 ) & 0x1F ) << 11 )
		                         | ( ( ( rgb >> 11 ) & 0x1F ) << 6 )
		                         | ( ( rgb >> 3 ) & 0x1F ) );
	}

	if ( logger ) logger->Write( "HDMI", LogNotice,
	                            "VIC-II renderer armed: %ux%u at %ux in %ux%u",
	                            VIC_RENDER_WIDTH, VIC_RENDER_HEIGHT,
	                            (unsigned)m_Scale,
	                            (unsigned)m_FBWidth, (unsigned)m_FBHeight );
	return true;
}

bool CHDMIDisplay::start()
{
	if ( !m_FramePixels || __atomic_load_n( &m_Started, __ATOMIC_ACQUIRE ) )
		return false;
	m_LastSerialAccess = m_Memory.hdmiSerialAccessCount();
	m_SerialBlockUntil = 0;
	__atomic_store_n( &m_Started, 1, __ATOMIC_RELEASE );
	radSetCore1Task( core1Entry, this );
	return true;
}

bool CHDMIDisplay::serialBlocked()
{
	const u32 nowAccess = m_Memory.hdmiSerialAccessCount();
	const u64 now = hdmiCounter();
	if ( nowAccess != m_LastSerialAccess )
	{
		m_LastSerialAccess = nowAccess;
		m_SerialBlockUntil = now
		                   + hdmiCounterFrequency() * HDMI_SERIAL_QUIET_US
		                     / 1000000u;
	}
	return (s64)( now - m_SerialBlockUntil ) < 0;
}

bool CHDMIDisplay::serialAbortCheck( void *context )
{
	const CHDMIDisplay *display = (const CHDMIDisplay *)context;
	return display && display->serialBlocked();
}

void CHDMIDisplay::fillFrameBuffer( u8 colour )
{
	if ( !m_FramePixels ) return;
	const u16 pixel = m_Palette[ colour & 0x0F ];
	for ( u32 y = 0; y < m_FBHeight; y++ )
	{
		u16 *dst = m_FramePixels + y * m_FBPitchPixels;
		for ( u32 x = 0; x < m_FBWidth; x++ ) dst[ x ] = pixel;
	}
}

void CHDMIDisplay::showReady()
{
	fillFrameBuffer( 6 );	// C64 blue: RAD is ready; power on the Commodore
}

void CHDMIDisplay::showFailure()
{
	fillFrameBuffer( 2 );	// red: logger text follows with the failure reason
}

bool CHDMIDisplay::firstFrameReady() const
{
	return __atomic_load_n( &m_FirstFrame, __ATOMIC_ACQUIRE ) != 0;
}

void CHDMIDisplay::core1Entry( void *context )
{
	CHDMIDisplay *display = (CHDMIDisplay *)context;
	if ( display ) display->run();
	for ( ;; ) asm volatile( "wfe" );
}

void CHDMIDisplay::run()
{
	const u64 frameTicks = hdmiCounterFrequency() / 60;
	for ( ;; )
	{
		const u64 next = hdmiCounter() + frameTicks;
		renderFrame();
		// A passive renderer must yield memory bandwidth too, not merely avoid
		// the physical bus. This counter is per-system and needs no core setup.
		while ( (s64)( hdmiCounter() - next ) < 0 ) asm volatile( "yield" );
	}
}

void CHDMIDisplay::renderFrame()
{
	if ( serialBlocked() )
	{
		m_SerialBusySkips++;
		return;
	}

	// Colour RAM is cached in the v2 bank-1 SRAM shadow, not bank-0 $D800.
	for ( u32 i = 0; i < 1024; i++ )
	{
		if ( ( i & 63 ) == 0 && serialBlocked() )
		{
			m_SerialAborts++;
			return;
		}
		m_Colours[ i ] = m_Memory.colourRAM( i );
	}

	VICRenderState state;
	state.ram = m_Memory.ramShadow();
	state.charROM = m_Memory.charROMShadow();
	state.colourRAM = m_Colours;
	state.screenBase = m_Memory.activeScreenBase();
	state.charsetBase = m_Memory.activeCharsetBase();
	state.d011 = m_Memory.vicRegister( 0x11 );
	state.d016 = m_Memory.vicRegister( 0x16 );
	state.d018 = m_Memory.vicRegister( 0x18 );
	state.border = m_Memory.vicRegister( 0x20 );
	state.background = m_Memory.vicRegister( 0x21 );
	if ( m_Renderer.render( state, m_Pixels, VIC_RENDER_WIDTH,
	                       serialAbortCheck, this ) == VIC_RENDER_ABORTED )
	{
		m_SerialAborts++;
		return;
	}
	if ( !presentChangedRows() )
	{
		m_SerialAborts++;
		return;
	}
	__atomic_store_n( &m_FirstFrame, 1, __ATOMIC_RELEASE );
}

bool CHDMIDisplay::presentChangedRows()
{
	u32 bytesSincePause = 0;
	for ( u32 y = 0; y < VIC_RENDER_HEIGHT; y++ )
	{
		if ( serialBlocked() ) return false;
		const u8 *src = m_Pixels + y * VIC_RENDER_WIDTH;
		u8 *old = m_Previous + y * VIC_RENDER_WIDTH;
		if ( memcmp( src, old, VIC_RENDER_WIDTH ) == 0 ) continue;

		for ( u32 repeatY = 0; repeatY < m_Scale; repeatY++ )
		{
			u16 *row = m_FramePixels
			         + ( m_OriginY + y * m_Scale + repeatY ) * m_FBPitchPixels
			         + m_OriginX;
			for ( u32 first = 0; first < VIC_RENDER_WIDTH;
			      first += HDMI_SOURCE_PIXELS_PER_CHUNK )
			{
				if ( serialBlocked() ) return false;
				const u32 stop = first + HDMI_SOURCE_PIXELS_PER_CHUNK
				               < VIC_RENDER_WIDTH
				               ? first + HDMI_SOURCE_PIXELS_PER_CHUNK
				               : VIC_RENDER_WIDTH;
				u16 *dst = row + first * m_Scale;
				for ( u32 x = first; x < stop; x++ )
				{
					const u16 colour = m_Palette[ src[ x ] & 0x0F ];
					for ( u32 repeatX = 0; repeatX < m_Scale; repeatX++ )
						*dst++ = colour;
				}
				bytesSincePause += ( stop - first ) * m_Scale * sizeof( u16 );
				if ( serialBlocked() ) return false;
				if ( bytesSincePause >= HDMI_FRAMEBUFFER_BURST_BYTES )
				{
					bytesSincePause = 0;
					hdmiPauseMicroseconds( HDMI_FRAMEBUFFER_PAUSE_US );
					if ( serialBlocked() ) return false;
				}
			}
		}
		// Commit the change record only after every scaled copy of the source
		// row reached the framebuffer. An interrupted row therefore retries in
		// full on the next measured-quiet frame.
		memcpy( old, src, VIC_RENDER_WIDTH );
	}
	return !serialBlocked();
}
