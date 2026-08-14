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

CHDMIDisplay::CHDMIDisplay( const CC64Memory &memory )
	: m_Memory( memory ), m_FrameBuffer( 0 ), m_FramePixels( 0 ),
	  m_FBPitchBytes( 0 ),
	  m_Started( 0 ), m_FirstFrame( 0 ), m_MaxBandTicks( 0 ),
	  m_MissedBandDeadlines( 0 )
{
	memset( m_Colours, 14, sizeof m_Colours );
	memset( m_BandPixels, 0, sizeof m_BandPixels );
}

bool CHDMIDisplay::initialize( CLogger *logger )
{
	// Circle uses this same small indexed-framebuffer model for its Spectrum
	// emulator. The VideoCore performs the final HDMI scaling; the ARM writes
	// exactly one palette index for each VIC output pixel.
	m_FrameBuffer = new CBcmFrameBuffer( VIC_RENDER_WIDTH,
	                                     VIC_RENDER_HEIGHT, 8 );
	if ( !m_FrameBuffer )
	{
		if ( logger ) logger->Write( "HDMI", LogError,
		                              "cannot allocate indexed framebuffer" );
		return false;
	}

	for ( u32 i = 0; i < 16; i++ )
	{
		const u32 rgb = s_VICPalette[ i ];
		const u16 rgb565 = (u16)( ( ( ( rgb >> 19 ) & 0x1F ) << 11 )
		                             | ( ( ( rgb >> 10 ) & 0x3F ) << 5 )
		                             | ( ( rgb >> 3 ) & 0x1F ) );
		m_FrameBuffer->SetPalette( (u8)i, rgb565 );
	}

	if ( !m_FrameBuffer->Initialize() )
	{
		if ( logger ) logger->Write( "HDMI", LogError,
		                              "indexed framebuffer initialization failed" );
		delete m_FrameBuffer;
		m_FrameBuffer = 0;
		return false;
	}

	m_FramePixels = (u8 *)(uintptr)m_FrameBuffer->GetBuffer();
	m_FBPitchBytes = m_FrameBuffer->GetPitch();
	if ( !m_FramePixels || m_FrameBuffer->GetDepth() != 8
	     || m_FBPitchBytes < VIC_RENDER_WIDTH )
	{
		if ( logger ) logger->Write( "HDMI", LogError,
		                              "indexed framebuffer has invalid layout" );
		m_FramePixels = 0;
		return false;
	}

	if ( logger ) logger->Write( "HDMI", LogNotice,
	                            "VIC-II renderer armed: %ux%u indexed, pitch=%u",
	                            VIC_RENDER_WIDTH, VIC_RENDER_HEIGHT,
	                            (unsigned)m_FBPitchBytes );
	return true;
}

bool CHDMIDisplay::start()
{
	if ( !m_FramePixels || __atomic_load_n( &m_Started, __ATOMIC_ACQUIRE ) )
		return false;
	__atomic_store_n( &m_Started, 1, __ATOMIC_RELEASE );
	radSetCore1Task( core1Entry, this );
	return true;
}

void CHDMIDisplay::fillFrameBuffer( u8 colour )
{
	if ( !m_FramePixels ) return;
	for ( u32 y = 0; y < VIC_RENDER_HEIGHT; y++ )
		memset( m_FramePixels + y * m_FBPitchBytes,
		        colour & 0x0F, VIC_RENDER_WIDTH );
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

u32 CHDMIDisplay::maxBandUS() const
{
	const u64 ticks = __atomic_load_n( &m_MaxBandTicks, __ATOMIC_ACQUIRE );
	return (u32)( ticks * 1000000ull / hdmiCounterFrequency() );
}

u32 CHDMIDisplay::missedBandDeadlines() const
{
	return __atomic_load_n( &m_MissedBandDeadlines, __ATOMIC_ACQUIRE );
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
		const u64 frameStart = hdmiCounter();
		renderFrame( frameStart, frameTicks );
	}
}

void CHDMIDisplay::renderFrame( u64 frameStart, u64 frameTicks )
{
	// Colour RAM is cached in the v2 bank-1 SRAM shadow, not bank-0 $D800.
	for ( u32 i = 0; i < 1024; i++ )
		m_Colours[ i ] = m_Memory.colourRAM( i );

	VICRenderState state;
	memset( &state, 0, sizeof state );
	state.ram = m_Memory.ramShadow();
	state.charROM = m_Memory.charROMShadow();
	state.colourRAM = m_Colours;
	state.bankBase = m_Memory.activeVICBankBase();
	state.screenBase = m_Memory.activeVICScreenBase();
	state.charsetBase = m_Memory.activeCharsetBase();
	state.bitmapBase = m_Memory.activeBitmapBase();
	for ( u32 i = 0; i < sizeof state.vic; i++ )
		state.vic[ i ] = m_Memory.vicRegister( (u8)i );

	// Work one line at a time and distribute the writes uniformly through the
	// frame. Besides lowering the instantaneous memory-system pressure, the
	// 384-byte reusable buffer stays in L1 instead of dirtying a 104KB shared
	// full-frame scratch image on every pass.
	for ( u32 first = 0; first < VIC_RENDER_HEIGHT;
	      first += HDMI_RENDER_BAND_ROWS )
	{
		const u32 rows = first + HDMI_RENDER_BAND_ROWS <= VIC_RENDER_HEIGHT
		               ? HDMI_RENDER_BAND_ROWS : VIC_RENDER_HEIGHT - first;
		const u64 bandStart = hdmiCounter();
		m_Renderer.renderRows( state, m_BandPixels, VIC_RENDER_WIDTH, first, rows );
		presentRows( m_BandPixels, first, rows );
		const u64 bandEnd = hdmiCounter();
		const u64 elapsed = bandEnd - bandStart;
		u64 previousMax = __atomic_load_n( &m_MaxBandTicks, __ATOMIC_RELAXED );
		while ( elapsed > previousMax
		        && !__atomic_compare_exchange_n( &m_MaxBandTicks, &previousMax,
		                                         elapsed, false,
		                                         __ATOMIC_RELAXED,
		                                         __ATOMIC_RELAXED ) ) {}

		const u64 target = frameStart
		                 + frameTicks * ( first + rows ) / VIC_RENDER_HEIGHT;
		if ( (s64)( bandEnd - target ) >= 0 )
			__atomic_add_fetch( &m_MissedBandDeadlines, 1, __ATOMIC_RELAXED );
		else
			while ( (s64)( hdmiCounter() - target ) < 0 ) asm volatile( "yield" );
	}
	__atomic_store_n( &m_FirstFrame, 1, __ATOMIC_RELEASE );
}

void CHDMIDisplay::presentRows( const u8 *source, u32 firstRow, u32 rowCount )
{
	// At the normal 384-byte pitch each band is one wide copy. Keep the row
	// fallback for firmware that pads an 8bpp scanline beyond its alignment.
	if ( m_FBPitchBytes == VIC_RENDER_WIDTH )
	{
		memcpy( m_FramePixels + firstRow * VIC_RENDER_WIDTH,
		        source,
		        rowCount * VIC_RENDER_WIDTH );
		return;
	}

	for ( u32 y = firstRow; y < firstRow + rowCount; y++ )
		memcpy( m_FramePixels + y * m_FBPitchBytes,
		        source + ( y - firstRow ) * VIC_RENDER_WIDTH,
		        VIC_RENDER_WIDTH );
}
