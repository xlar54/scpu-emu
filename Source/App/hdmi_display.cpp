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

// The generic-timer event stream turns a selected physical-counter edge into
// an architectural event for this core. Counter bit 8 rises once every 512
// ticks (about 26.7us at the Pi 3's 19.2MHz counter), fine enough for the
// 61us row cadence without hammering the shared counter peripheral in a tight
// loop. CNTKCTL_EL1 is per-core here; core 0's bus timing is untouched.
static inline void hdmiEnableCounterEvents()
{
	u64 control;
	asm volatile( "mrs %0, CNTKCTL_EL1" : "=r" ( control ) );
	control &= ~( ( 0xFull << 4 ) | ( 1ull << 3 ) );
	control |= ( 1ull << 2 ) | ( 8ull << 4 );
	asm volatile( "msr CNTKCTL_EL1, %0\n\tisb" :: "r" ( control ) : "memory" );

	// Consume any event that was already latched before the stream was enabled.
	asm volatile( "sevl\n\twfe" ::: "memory" );
}

static inline void hdmiWaitUntil( u64 target )
{
	while ( (s64)( hdmiCounter() - target ) < 0 )
		asm volatile( "wfe" );
}

CHDMIDisplay::CHDMIDisplay( const CC64Memory &memory )
	: m_Memory( memory ), m_FrameBuffer( 0 ), m_FramePixels( 0 ),
	  m_FBPitchBytes( 0 ),
	  m_Started( 0 ), m_FirstFrame( 0 ), m_MaxBandTicks( 0 ),
	  m_MissedBandDeadlines( 0 ), m_SerialTransfers( 0 ),
	  m_LastSerialTransfers( 0 ), m_FrameStartSerialTransfers( 0 ),
	  m_SerialBusySkips( 0 ), m_SerialAborts( 0 )
{
	memset( m_Colours, 14, sizeof m_Colours );
	memset( m_VICRAM, 0, sizeof m_VICRAM );
	memset( m_BandPixels, 0, sizeof m_BandPixels );
}

void CHDMIDisplay::watchBusActivity( const volatile u64 *transfers )
{
	// Called on core 0 before start(). Establish the current count as the
	// baseline so calibration traffic which predates the renderer cannot make
	// its first frame look like an active serial transaction.
	m_SerialTransfers = transfers;
	m_LastSerialTransfers = transfers ? *transfers : 0;
	m_FrameStartSerialTransfers = m_LastSerialTransfers;
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

void CHDMIDisplay::resetRuntimeDiagnostics()
{
	__atomic_store_n( &m_MaxBandTicks, 0, __ATOMIC_RELEASE );
	__atomic_store_n( &m_MissedBandDeadlines, 0, __ATOMIC_RELEASE );
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
	hdmiEnableCounterEvents();
	for ( ;; )
	{
		const u64 frameStart = hdmiCounter();
		renderFrame( frameStart, frameTicks );
	}
}

void CHDMIDisplay::renderFrame( u64 frameStart, u64 frameTicks )
{
	// A real IEC transfer produces hundreds of CIA2 accesses per millisecond.
	// Hold the last complete HDMI picture during such a frame. This is a
	// one-way observation: core 0 never waits for core 1.
	if ( m_SerialTransfers )
	{
		const u64 now = *m_SerialTransfers;
		const u64 delta = now - m_LastSerialTransfers;
		m_LastSerialTransfers = now;
		if ( delta > HDMI_SERIAL_BUSY_ACCESSES )
		{
			m_SerialBusySkips++;
			hdmiWaitUntil( frameStart + frameTicks );
			return;
		}
		m_FrameStartSerialTransfers = now;
	}

	// Colour RAM is cached in the v2 bank-1 SRAM shadow, not bank-0 $D800.
	// Acquire its cache lines once sequentially, like the other display state.
	const u8 *liveColours = m_Memory.colourRAMShadow();
	if ( liveColours ) memcpy( m_Colours, liveColours, sizeof m_Colours );
	else               memset( m_Colours, 14, sizeof m_Colours );

	VICRenderState state;
	memset( &state, 0, sizeof state );
	const u8 *liveRAM = m_Memory.ramShadow();
	state.charROM = m_Memory.charROMShadow();
	state.colourRAM = m_Colours;
	for ( u32 i = 0; i < sizeof state.vic; i++ )
		state.vic[ i ] = m_Memory.vicRegister( (u8)i );

	// Rendering directly from m_RAM makes the live screen cache lines bounce
	// between core 0 (writer) and core 1 (reader). Snapshot each active VIC
	// dependency once in a sequential transfer, then render only from private
	// core-1 storage. Local address zero represents the selected 16KB VIC bank.
	const u32 liveBank = m_Memory.activeVICBankBase();
	const u32 liveScreen = liveBank
	                     + ( (u32)( state.vic[ 0x18 ] >> 4 ) << 10 );
	const u32 localScreen = liveScreen - liveBank;
	memcpy( m_VICRAM + localScreen, liveRAM + liveScreen, 1024 );

	const bool bitmapMode = ( state.vic[ 0x11 ] & 0x20 ) != 0;
	const u32 localBitmap = ( state.vic[ 0x18 ] & 0x08 ) ? 0x2000 : 0;
	if ( bitmapMode )
		memcpy( m_VICRAM + localBitmap,
		        liveRAM + liveBank + localBitmap, 8192 );

	const u32 localCharset = (u32)( state.vic[ 0x18 ] & 0x0E ) << 10;
	const u32 bankNumber = liveBank >> 14;
	const bool charROM = !bitmapMode && ( bankNumber & 1 ) == 0
	                  && localCharset >= 0x1000 && localCharset < 0x2000;
	if ( !bitmapMode && !charROM )
		memcpy( m_VICRAM + localCharset,
		        liveRAM + liveBank + localCharset, 2048 );

	const u8 enabledSprites = state.vic[ 0x15 ];
	for ( u32 sprite = 0; sprite < 8; sprite++ )
	{
		if ( !( enabledSprites & ( 1u << sprite ) ) ) continue;
		const u32 shape = (u32)m_VICRAM[ localScreen + 0x3F8 + sprite ] << 6;
		memcpy( m_VICRAM + shape, liveRAM + liveBank + shape, 64 );
	}

	state.ram = m_VICRAM;
	state.bankBase = 0;
	state.screenBase = localScreen;
	state.charsetBase = charROM ? 0xFFFFFFFF : localCharset;
	state.bitmapBase = localBitmap;

	// Work one line at a time and distribute the writes uniformly through the
	// frame. Besides lowering the instantaneous memory-system pressure, the
	// 384-byte reusable buffer stays in L1 instead of dirtying a 104KB shared
	// full-frame scratch image on every pass.
	for ( u32 first = 0; first < VIC_RENDER_HEIGHT;
	      first += HDMI_RENDER_BAND_ROWS )
	{
		// If a serial handshake started while the frame was being snapshotted or
		// while the previous row was rendered, abandon the frame immediately.
		if ( serialActive() )
		{
			m_SerialAborts++;
			hdmiWaitUntil( frameStart + frameTicks );
			return;
		}

		const u32 rows = first + HDMI_RENDER_BAND_ROWS <= VIC_RENDER_HEIGHT
		               ? HDMI_RENDER_BAND_ROWS : VIC_RENDER_HEIGHT - first;
		const u64 bandStart = hdmiCounter();
		m_Renderer.renderRows( state, m_BandPixels, VIC_RENDER_WIDTH, first, rows );
		if ( !presentRows( m_BandPixels, first, rows ) )
		{
			m_SerialAborts++;
			hdmiWaitUntil( frameStart + frameTicks );
			return;
		}
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
			hdmiWaitUntil( target );
	}
	__atomic_store_n( &m_FirstFrame, 1, __ATOMIC_RELEASE );
}

bool CHDMIDisplay::serialActive() const
{
	return m_SerialTransfers
	    && ( *m_SerialTransfers - m_FrameStartSerialTransfers )
	       > HDMI_SERIAL_ABORT_ACCESSES;
}

bool CHDMIDisplay::presentRows( const u8 *source, u32 firstRow, u32 rowCount )
{
	// A single 384-byte framebuffer memcpy collapsed the physical read eye on
	// core 0. The previous hardware-validated renderer never issued more than
	// roughly 80 output bytes without an acknowledgement point. Keep this
	// indexed renderer below that boundary and force each span's stores to be
	// observed before the next one begins. The serial check supplies the
	// producer-side abort without any reciprocal core-0 handshake.
	for ( u32 row = 0; row < rowCount; row++ )
	{
		u8 *dst = m_FramePixels + ( firstRow + row ) * m_FBPitchBytes;
		const u8 *src = source + row * VIC_RENDER_WIDTH;
		for ( u32 x = 0; x < VIC_RENDER_WIDTH;
		      x += HDMI_FRAMEBUFFER_SPAN_BYTES )
		{
			if ( serialActive() ) return false;
			const u32 count = x + HDMI_FRAMEBUFFER_SPAN_BYTES <= VIC_RENDER_WIDTH
			                ? HDMI_FRAMEBUFFER_SPAN_BYTES
			                : VIC_RENDER_WIDTH - x;
			memcpy( dst + x, src + x, count );
			asm volatile( "dmb ishst" ::: "memory" );
		}
	}
	return true;
}
