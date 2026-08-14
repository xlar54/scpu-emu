/*
   SCPU-EMU - host tests for the passive VIC-II renderer
*/
#include "../test_framework.h"
#include "../../Source/Video/vic_renderer.h"

struct VICFixture
{
	u8 ram[ 65536 ];
	u8 chars[ 4096 ];
	u8 colours[ 1024 ];
	u8 pixels[ VIC_RENDER_WIDTH * VIC_RENDER_HEIGHT ];
	VICRenderState state;
	CVICRenderer renderer;

	VICFixture()
	{
		for ( u32 i = 0; i < sizeof ram; i++ ) ram[ i ] = 0;
		for ( u32 i = 0; i < sizeof chars; i++ ) chars[ i ] = 0;
		for ( u32 i = 0; i < sizeof colours; i++ ) colours[ i ] = 14;
		for ( u32 i = 0; i < sizeof pixels; i++ ) pixels[ i ] = 0xFF;
		state.ram = ram;
		state.charROM = chars;
		state.colourRAM = colours;
		state.screenBase = 0x0400;
		state.charsetBase = 0xFFFFFFFF;
		state.d011 = 0x1B;
		state.d016 = 0x08;
		state.d018 = 0x14;
		state.border = 14;
		state.background = 6;
	}

	u8 pixel( u32 x, u32 y ) const
	{
		return pixels[ y * VIC_RENDER_WIDTH + x ];
	}
};

TEST( vic_renderer_den_clear_is_all_border )
{
	VICFixture f;
	f.state.d011 = 0x0B;
	CHECK_EQ( f.renderer.render( f.state, f.pixels, VIC_RENDER_WIDTH ),
	          VIC_RENDER_BLANK );
	CHECK_EQ( f.pixel( 0, 0 ), 14 );
	CHECK_EQ( f.pixel( VIC_RENDER_DISPLAY_X, VIC_RENDER_DISPLAY_Y ), 14 );
	CHECK_EQ( f.pixel( VIC_RENDER_WIDTH - 1, VIC_RENDER_HEIGHT - 1 ), 14 );
}

TEST( vic_renderer_standard_text_uses_border_background_and_colour_ram )
{
	VICFixture f;
	f.ram[ 0x0400 ] = 1;
	f.colours[ 0 ] = 2;
	// Character 1, top row: left-most and right-most pixels set.
	f.chars[ 8 ] = 0x81;
	CHECK_EQ( f.renderer.render( f.state, f.pixels, VIC_RENDER_WIDTH ),
	          VIC_RENDER_TEXT );
	CHECK_EQ( f.pixel( 0, 0 ), 14 );
	CHECK_EQ( f.pixel( VIC_RENDER_DISPLAY_X, VIC_RENDER_DISPLAY_Y ), 2 );
	CHECK_EQ( f.pixel( VIC_RENDER_DISPLAY_X + 1, VIC_RENDER_DISPLAY_Y ), 6 );
	CHECK_EQ( f.pixel( VIC_RENDER_DISPLAY_X + 7, VIC_RENDER_DISPLAY_Y ), 2 );
}

TEST( vic_renderer_selects_second_character_rom_half )
{
	VICFixture f;
	f.state.d018 = 0x16;
	f.ram[ 0x0400 ] = 0;
	f.colours[ 0 ] = 5;
	f.chars[ 0x0800 ] = 0x80;
	f.renderer.render( f.state, f.pixels, VIC_RENDER_WIDTH );
	CHECK_EQ( f.pixel( VIC_RENDER_DISPLAY_X, VIC_RENDER_DISPLAY_Y ), 5 );
	CHECK_EQ( f.pixel( VIC_RENDER_DISPLAY_X + 1, VIC_RENDER_DISPLAY_Y ), 6 );
}

TEST( vic_renderer_uses_ram_character_set )
{
	VICFixture f;
	f.state.charsetBase = 0x2000;
	f.ram[ 0x0400 ] = 3;
	f.colours[ 0 ] = 7;
	f.ram[ 0x2000 + 3 * 8 ] = 0x40;
	f.renderer.render( f.state, f.pixels, VIC_RENDER_WIDTH );
	CHECK_EQ( f.pixel( VIC_RENDER_DISPLAY_X, VIC_RENDER_DISPLAY_Y ), 6 );
	CHECK_EQ( f.pixel( VIC_RENDER_DISPLAY_X + 1, VIC_RENDER_DISPLAY_Y ), 7 );
}

TEST( vic_renderer_rejects_modes_not_implemented_yet )
{
	VICFixture f;
	f.state.d016 |= 0x10;
	CHECK_EQ( f.renderer.render( f.state, f.pixels, VIC_RENDER_WIDTH ),
	          VIC_RENDER_UNSUPPORTED );
	CHECK_EQ( f.pixel( VIC_RENDER_DISPLAY_X, VIC_RENDER_DISPLAY_Y ), 6 );
}

static bool abortVICRender( void *context )
{
	u32 *calls = (u32 *)context;
	return ++*calls >= 3;
}

TEST( vic_renderer_aborts_between_rows_when_observer_requests_it )
{
	VICFixture f;
	u32 calls = 0;
	CHECK_EQ( f.renderer.render( f.state, f.pixels, VIC_RENDER_WIDTH,
	                            abortVICRender, &calls ),
	          VIC_RENDER_ABORTED );
	CHECK_EQ( calls, 3u );
}
