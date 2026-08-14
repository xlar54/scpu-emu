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

// Stay off the shared memory system while an IEC slow-lane span drains. The
// counter is a core register, and YIELD does not touch memory or involve core 0.
static inline void hdmiPauseUS( u32 microseconds )
{
	const u64 end = hdmiCounter()
	              + (u64)microseconds * hdmiCounterFrequency() / 1000000;
	while ( (s64)( hdmiCounter() - end ) < 0 )
		asm volatile( "yield" );
}

CHDMIDisplay::CHDMIDisplay( const CC64Memory &memory )
	: m_Memory( memory ), m_FrameBuffer( 0 ), m_FramePixels( 0 ),
	  m_FBPitchBytes( 0 ),
	  m_RasterTiming{ 65, 263, VIC_RENDER_HEIGHT, 5,
	                  VIC_RENDER_DISPLAY_Y, VIC_RENDER_DISPLAY_H },
	  m_FrameHz( 60 ),
	  m_Started( 0 ), m_FirstFrame( 0 ), m_PictureEnabled( 1 ),
	  m_PictureParked( 0 ),
	  m_PresentedFrames( 0 ), m_SpriteSpriteCollision( 0 ),
	  m_SpriteBackgroundCollision( 0 ), m_MaxBandTicks( 0 ),
	  m_MissedBandDeadlines( 0 ), m_SerialTransfers( 0 ),
	  m_LastSerialTransfers( 0 ), m_FrameStartSerialTransfers( 0 ),
	  m_SerialThrottled( false ), m_SerialThrottleFrames( 0 )
{
	memset( m_Colours, 14, sizeof m_Colours );
	memset( m_VICRAM, 0, sizeof m_VICRAM );
	memset( m_LiveVIC, 0, sizeof m_LiveVIC );
	memset( &m_RasterPlan, 0, sizeof m_RasterPlan );
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

void CHDMIDisplay::setVideoTiming( u32 cyclesPerLine, u32 rasterLines,
	                               u32 displayFirstLine )
{
	if ( cyclesPerLine < 48 || cyclesPerLine > 80
	     || rasterLines < 240 || rasterLines > 320 ) return;
	m_RasterTiming.cyclesPerLine = cyclesPerLine;
	m_RasterTiming.rasterLines = rasterLines;
	m_RasterTiming.frameRows = VIC_RENDER_HEIGHT;
	m_RasterTiming.topRaster = (s32)displayFirstLine
	                         - (s32)VIC_RENDER_DISPLAY_Y;
	m_RasterTiming.displayFirstRow = VIC_RENDER_DISPLAY_Y;
	m_RasterTiming.displayRows = VIC_RENDER_DISPLAY_H;
	m_FrameHz = rasterLines > 300 ? 50 : 60;
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
	fillFrameBuffer( 5 );	// green: RAD is ready; power on the Commodore
}

void CHDMIDisplay::showFailure()
{
	fillFrameBuffer( 2 );	// red: logger text follows with the failure reason
}

bool CHDMIDisplay::firstFrameReady() const
{
	return __atomic_load_n( &m_FirstFrame, __ATOMIC_ACQUIRE ) != 0;
}

void CHDMIDisplay::setPictureEnabled( bool enabled )
{
	__atomic_store_n( &m_PictureEnabled, enabled ? 1u : 0u,
	                  __ATOMIC_RELEASE );
}

bool CHDMIDisplay::pictureEnabled() const
{
	return __atomic_load_n( &m_PictureEnabled, __ATOMIC_ACQUIRE ) != 0;
}

bool CHDMIDisplay::pictureParked() const
{
	return __atomic_load_n( &m_PictureParked, __ATOMIC_ACQUIRE ) != 0;
}

u32 CHDMIDisplay::presentedFrames() const
{
	return __atomic_load_n( &m_PresentedFrames, __ATOMIC_ACQUIRE );
}

void CHDMIDisplay::resetCollisionLatches()
{
	__atomic_store_n( &m_SpriteSpriteCollision, 0u, __ATOMIC_RELEASE );
	__atomic_store_n( &m_SpriteBackgroundCollision, 0u, __ATOMIC_RELEASE );
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
	hdmiEnableCounterEvents();
	for ( ;; )
	{
		const u64 frameStart = hdmiCounter();
		const u64 frameTicks = hdmiCounterFrequency()
		                     / ( m_FrameHz ? m_FrameHz : 60 );
		if ( !pictureEnabled() )
		{
			// radSetCore1Task() cannot be undone. WFE on the counter event
			// stream parks this core without shared-memory traffic while the
			// physical VIC owns presentation.
			__atomic_store_n( &m_PictureParked, 1u, __ATOMIC_RELEASE );
			hdmiWaitUntil( frameStart + frameTicks );
			continue;
		}
		__atomic_store_n( &m_PictureParked, 0u, __ATOMIC_RELEASE );
		renderFrame( frameStart, frameTicks );
	}
}

void CHDMIDisplay::renderFrame( u64 frameStart, u64 frameTicks )
{
	VICRenderCollisions collisions = {};
	// A real IEC transfer produces hundreds of CIA2 accesses per millisecond.
	// K355 held the last complete picture during those frames. Preserve its
	// protection without freezing the display: an active frame enters a
	// producer-side slow lane with smaller spans and quiet gaps between them.
	m_SerialThrottled = false;
	if ( m_SerialTransfers )
	{
		const u64 now = *m_SerialTransfers;
		const u64 delta = now - m_LastSerialTransfers;
		m_LastSerialTransfers = now;
		m_FrameStartSerialTransfers = now;
		if ( delta > HDMI_SERIAL_BUSY_ACCESSES )
		{
			m_SerialThrottled = true;
			m_SerialThrottleFrames++;
		}
	}

	// Latch one consistent scalar endpoint. A raster write advances the ring
	// head after its shadow byte is stored, so equal head/generation samples on
	// both sides of this tiny copy prove the live state belongs to that head.
	u32 generation = 0;
	u32 head = 0;
	u8 liveBank = 0;
	for ( u32 attempt = 0; attempt < 4; attempt++ )
	{
		const u32 generationBefore = m_Memory.vicLogGeneration();
		const u32 headBefore = m_Memory.vicLogHead();
		for ( u32 i = 0; i < sizeof m_LiveVIC; i++ )
			m_LiveVIC[ i ] = m_Memory.vicRegister( (u8)i );
		liveBank = (u8)( m_Memory.activeVICBankBase() >> 14 );
		const u32 headAfter = m_Memory.vicLogHead();
		const u32 generationAfter = m_Memory.vicLogGeneration();
		head = headAfter;
		generation = generationAfter;
		if ( headBefore == headAfter
		     && generationBefore == generationAfter ) break;
	}

	const u8 *liveRAM = m_Memory.ramShadow();
	const VICRasterFrameResult rasterResult = m_RasterReplay.build(
		m_Memory.vicLog(), head, (u32)m_Memory.emuNow(),
		m_Memory.rasterAnchor(), generation, m_RasterTiming,
		m_LiveVIC, liveBank, liveRAM, m_RasterPlan );
	if ( rasterResult == VIC_RASTER_HOLD )
	{
		hdmiWaitUntil( frameStart + frameTicks );
		return;
	}

	// Colour and source RAM are one coherent per-frame image, not timestamped
	// streams. Mid-frame RAM rewrites are intentionally unsupported; register,
	// bank and VIC colour-register effects retain raster precision.
	const u8 *liveColours = m_Memory.colourRAMShadow();
	if ( liveColours ) copyShared( m_Colours, liveColours, sizeof m_Colours );
	else               memset( m_Colours, 14, sizeof m_Colours );
	for ( u32 page = 0; page < 256; page++ )
		if ( m_RasterPlan.pageMask[ page >> 3 ] & ( 1u << ( page & 7 ) ) )
			copyShared( m_VICRAM + page * 256, liveRAM + page * 256, 256 );

	// Reset may race the bounded snapshot. Never present a frame assembled from
	// two producer epochs; invalidation makes the next pass re-prime live state.
	if ( m_Memory.vicLogGeneration() != generation )
	{
		m_RasterReplay.invalidate();
		hdmiWaitUntil( frameStart + frameTicks );
		return;
	}

	for ( u32 b = 0; b < m_RasterPlan.bandCount; b++ )
	{
		const VICRasterReplayBand &band = m_RasterPlan.bands[ b ];
		VICRenderState state;
		memset( &state, 0, sizeof state );
		state.ram = m_VICRAM;
		state.charROM = m_Memory.charROMShadow();
		state.colourRAM = m_Colours;
		state.bankBase = (u32)( band.bank & 3 ) << 14;
		state.screenBase = state.bankBase
		                 + ( (u32)( band.vic[ 0x18 ] >> 4 ) << 10 );
		state.bitmapBase = state.bankBase
		                 + ( ( band.vic[ 0x18 ] & 0x08 ) ? 0x2000 : 0 );
		const u32 charset = (u32)( band.vic[ 0x18 ] & 0x0E ) << 10;
		const bool charROM = ( ( band.bank & 1 ) == 0 )
		                  && charset >= 0x1000 && charset < 0x2000;
		state.charsetBase = charROM ? 0xFFFFFFFF
		                            : state.bankBase + charset;
		state.yScrollVaries = m_RasterPlan.yScrollVaries;
		memcpy( state.vic, band.vic, sizeof state.vic );

		const u32 end = (u32)band.firstRow + band.rowCount;
		for ( u32 first = band.firstRow; first < end; first++ )
		{
			// A transfer may begin after the frame-start sample. Move into the slow
			// lane at the next row boundary and stay there for the rest of the frame.
			observeSerialActivity();
			const u64 bandStart = hdmiCounter();
			m_Renderer.renderRows( state, m_BandPixels, VIC_RENDER_WIDTH,
			                       first, HDMI_RENDER_BAND_ROWS, &collisions );
			observeSerialActivity();
			presentRows( m_BandPixels, first, HDMI_RENDER_BAND_ROWS );
			const u64 bandEnd = hdmiCounter();
			const u64 elapsed = bandEnd - bandStart;
			u64 previousMax = __atomic_load_n( &m_MaxBandTicks,
			                                       __ATOMIC_RELAXED );
			while ( elapsed > previousMax
			        && !__atomic_compare_exchange_n( &m_MaxBandTicks,
			                                         &previousMax, elapsed, false,
			                                         __ATOMIC_RELAXED,
			                                         __ATOMIC_RELAXED ) ) {}

			const u64 target = frameStart
			                 + frameTicks * ( first + 1 ) / VIC_RENDER_HEIGHT;
			if ( (s64)( bandEnd - target ) >= 0 )
				__atomic_add_fetch( &m_MissedBandDeadlines, 1,
				                    __ATOMIC_RELAXED );
			else
				hdmiWaitUntil( target );
		}
	}
	if ( collisions.spriteSprite )
		__atomic_fetch_or( &m_SpriteSpriteCollision,
		                   (u32)collisions.spriteSprite, __ATOMIC_RELEASE );
	if ( collisions.spriteBackground )
		__atomic_fetch_or( &m_SpriteBackgroundCollision,
		                   (u32)collisions.spriteBackground, __ATOMIC_RELEASE );
	__atomic_store_n( &m_FirstFrame, 1, __ATOMIC_RELEASE );
	__atomic_add_fetch( &m_PresentedFrames, 1u, __ATOMIC_RELEASE );
}

bool CHDMIDisplay::serialActive() const
{
	return m_SerialTransfers
	    && ( *m_SerialTransfers - m_FrameStartSerialTransfers )
	       > HDMI_SERIAL_ABORT_ACCESSES;
}

void CHDMIDisplay::observeSerialActivity()
{
	if ( !m_SerialThrottled && serialActive() )
	{
		m_SerialThrottled = true;
		m_SerialThrottleFrames++;
	}
}

void CHDMIDisplay::copyShared( u8 *destination, const u8 *source, u32 count )
{
	// Snapshot traffic is part of the same core-1 memory pressure as the final
	// framebuffer stores. K355 avoided it by skipping busy frames entirely, so
	// a live slow lane must bound these reads too or it loses that protection.
	for ( u32 offset = 0; offset < count; )
	{
		observeSerialActivity();
		const u32 span = m_SerialThrottled
		               ? HDMI_IEC_SPAN_BYTES : HDMI_SHARED_SPAN_BYTES;
		const u32 bytes = offset + span <= count ? span : count - offset;
		memcpy( destination + offset, source + offset, bytes );
		asm volatile( "dmb ish" ::: "memory" );
		offset += bytes;
		if ( m_SerialThrottled ) hdmiPauseUS( HDMI_IEC_SPAN_PAUSE_US );
	}
}

void CHDMIDisplay::presentRows( const u8 *source, u32 firstRow, u32 rowCount )
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
		for ( u32 x = 0; x < VIC_RENDER_WIDTH; )
		{
			observeSerialActivity();
			const u32 span = m_SerialThrottled
			               ? HDMI_IEC_SPAN_BYTES : HDMI_FRAMEBUFFER_SPAN_BYTES;
			const u32 count = x + span <= VIC_RENDER_WIDTH
			                ? span : VIC_RENDER_WIDTH - x;
			memcpy( dst + x, src + x, count );
			asm volatile( "dmb ishst" ::: "memory" );
			x += count;
			if ( m_SerialThrottled ) hdmiPauseUS( HDMI_IEC_SPAN_PAUSE_US );
		}
	}
}
