/*
   SCPU-EMU - host tests for the passive VIC-II renderer
*/
#include "../test_framework.h"
#include "../../Source/Video/vic_renderer.h"

#include <string.h>

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
		memset( ram, 0, sizeof ram );
		memset( chars, 0, sizeof chars );
		memset( colours, 14, sizeof colours );
		memset( pixels, 0xFF, sizeof pixels );
		memset( &state, 0, sizeof state );
		state.ram = ram;
		state.charROM = chars;
		state.colourRAM = colours;
		state.bankBase = 0;
		state.screenBase = 0x0400;
		state.charsetBase = 0xFFFFFFFF;
		state.bitmapBase = 0xFFFFFFFF;
		state.vic[ 0x11 ] = 0x1B;
		state.vic[ 0x16 ] = 0x08;
		state.vic[ 0x18 ] = 0x14;
		state.vic[ 0x20 ] = 14;
		state.vic[ 0x21 ] = 6;
	}

	u8 pixel( u32 x, u32 y ) const
	{
		return pixels[ y * VIC_RENDER_WIDTH + x ];
	}

	void sprite( u32 n, u16 x, u8 y, u8 pointer, u8 colour )
	{
		state.vic[ n * 2 ] = (u8)x;
		state.vic[ n * 2 + 1 ] = y;
		if ( x & 0x100 ) state.vic[ 0x10 ] |= (u8)( 1u << n );
		state.vic[ 0x15 ] |= (u8)( 1u << n );
		state.vic[ 0x27 + n ] = colour;
		ram[ (u16)( state.screenBase + 0x3F8 + n ) ] = pointer;
	}
};

TEST( vic_renderer_den_clear_is_all_border )
{
	VICFixture f;
	f.state.vic[ 0x11 ] = 0x0B;
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
	f.chars[ 8 ] = 0x81;
	CHECK_EQ( f.renderer.render( f.state, f.pixels, VIC_RENDER_WIDTH ),
	          VIC_RENDER_STANDARD_TEXT );
	CHECK_EQ( f.pixel( 0, 0 ), 14 );
	CHECK_EQ( f.pixel( VIC_RENDER_DISPLAY_X, VIC_RENDER_DISPLAY_Y ), 2 );
	CHECK_EQ( f.pixel( VIC_RENDER_DISPLAY_X + 1, VIC_RENDER_DISPLAY_Y ), 6 );
	CHECK_EQ( f.pixel( VIC_RENDER_DISPLAY_X + 7, VIC_RENDER_DISPLAY_Y ), 2 );
}

TEST( vic_renderer_selects_second_character_rom_half )
{
	VICFixture f;
	f.state.vic[ 0x18 ] = 0x16;
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

TEST( vic_renderer_multicolor_text_decodes_pairs_and_hires_cells )
{
	VICFixture f;
	f.state.vic[ 0x16 ] = 0x18;
	f.state.vic[ 0x22 ] = 2;
	f.state.vic[ 0x23 ] = 5;
	f.ram[ 0x0400 ] = 1;
	f.colours[ 0 ] = 0x0B;
	f.chars[ 8 ] = 0x1B;
	CHECK_EQ( f.renderer.render( f.state, f.pixels, VIC_RENDER_WIDTH ),
	          VIC_RENDER_MULTICOLOR_TEXT );
	CHECK_EQ( f.pixel( 32, 36 ), 6 );
	CHECK_EQ( f.pixel( 33, 36 ), 6 );
	CHECK_EQ( f.pixel( 34, 36 ), 2 );
	CHECK_EQ( f.pixel( 35, 36 ), 2 );
	CHECK_EQ( f.pixel( 36, 36 ), 5 );
	CHECK_EQ( f.pixel( 37, 36 ), 5 );
	CHECK_EQ( f.pixel( 38, 36 ), 3 );
	CHECK_EQ( f.pixel( 39, 36 ), 3 );

	f.colours[ 0 ] = 3;
	f.chars[ 8 ] = 0x80;
	f.renderer.render( f.state, f.pixels, VIC_RENDER_WIDTH );
	CHECK_EQ( f.pixel( 32, 36 ), 3 );
	CHECK_EQ( f.pixel( 33, 36 ), 6 );
}

TEST( vic_renderer_extended_colour_text_uses_code_high_bits_for_background )
{
	VICFixture f;
	f.state.vic[ 0x11 ] = 0x5B;
	f.state.vic[ 0x24 ] = 4;
	f.ram[ 0x0400 ] = 0xC1;
	f.colours[ 0 ] = 7;
	f.chars[ 8 ] = 0x80;
	CHECK_EQ( f.renderer.render( f.state, f.pixels, VIC_RENDER_WIDTH ),
	          VIC_RENDER_EXTENDED_TEXT );
	CHECK_EQ( f.pixel( 32, 36 ), 7 );
	CHECK_EQ( f.pixel( 33, 36 ), 4 );
}

TEST( vic_renderer_standard_bitmap_uses_screen_nibbles )
{
	VICFixture f;
	f.state.vic[ 0x11 ] = 0x3B;
	f.state.bitmapBase = 0x2000;
	f.ram[ 0x0400 ] = 0xA5;
	f.ram[ 0x2000 ] = 0x80;
	CHECK_EQ( f.renderer.render( f.state, f.pixels, VIC_RENDER_WIDTH ),
	          VIC_RENDER_STANDARD_BITMAP );
	CHECK_EQ( f.pixel( 32, 36 ), 10 );
	CHECK_EQ( f.pixel( 33, 36 ), 5 );
}

TEST( vic_renderer_multicolor_bitmap_decodes_all_four_sources )
{
	VICFixture f;
	f.state.vic[ 0x11 ] = 0x3B;
	f.state.vic[ 0x16 ] = 0x18;
	f.state.bitmapBase = 0x2000;
	f.ram[ 0x0400 ] = 0xA5;
	f.colours[ 0 ] = 7;
	f.ram[ 0x2000 ] = 0x1B;
	CHECK_EQ( f.renderer.render( f.state, f.pixels, VIC_RENDER_WIDTH ),
	          VIC_RENDER_MULTICOLOR_BITMAP );
	CHECK_EQ( f.pixel( 32, 36 ), 6 );
	CHECK_EQ( f.pixel( 33, 36 ), 6 );
	CHECK_EQ( f.pixel( 34, 36 ), 10 );
	CHECK_EQ( f.pixel( 35, 36 ), 10 );
	CHECK_EQ( f.pixel( 36, 36 ), 5 );
	CHECK_EQ( f.pixel( 37, 36 ), 5 );
	CHECK_EQ( f.pixel( 38, 36 ), 7 );
	CHECK_EQ( f.pixel( 39, 36 ), 7 );
}

TEST( vic_renderer_illegal_mode_is_deterministic_background )
{
	VICFixture f;
	f.state.vic[ 0x11 ] = 0x7B;
	CHECK_EQ( f.renderer.render( f.state, f.pixels, VIC_RENDER_WIDTH ),
	          VIC_RENDER_UNSUPPORTED );
	CHECK_EQ( f.pixel( VIC_RENDER_DISPLAY_X, VIC_RENDER_DISPLAY_Y ), 6 );
}

TEST( vic_renderer_applies_fine_scroll_and_reduced_border_apertures )
{
	VICFixture f;
	f.ram[ 0x0400 ] = 1;
	f.colours[ 0 ] = 2;
	f.chars[ 8 ] = 0x80;
	f.state.vic[ 0x16 ] = 0x0B;
	f.renderer.render( f.state, f.pixels, VIC_RENDER_WIDTH );
	CHECK_EQ( f.pixel( 32, 36 ), 6 );
	CHECK_EQ( f.pixel( 35, 36 ), 2 );

	f.state.vic[ 0x16 ] = 0x00;
	f.state.vic[ 0x11 ] = 0x13;
	f.renderer.render( f.state, f.pixels, VIC_RENDER_WIDTH );
	CHECK_EQ( f.pixel( 39, 40 ), 14 );
	CHECK_EQ( f.pixel( 40, 40 ), 6 );
	CHECK_EQ( f.pixel( 40, 39 ), 14 );
}

TEST( vic_renderer_standard_sprite_uses_pointer_position_and_x_msb )
{
	VICFixture f;
	f.sprite( 0, 24, 50, 0x20, 2 );
	f.ram[ 0x0800 ] = 0x80;
	f.renderer.render( f.state, f.pixels, VIC_RENDER_WIDTH );
	CHECK_EQ( f.pixel( 32, 36 ), 2 );
	CHECK_EQ( f.pixel( 33, 36 ), 6 );

	memset( f.ram + 0x0800, 0, 64 );
	f.ram[ 0x0800 ] = 0x80;
	f.state.vic[ 0 ] = 0;
	f.state.vic[ 0x10 ] = 1;
	f.renderer.render( f.state, f.pixels, VIC_RENDER_WIDTH );
	CHECK_EQ( f.pixel( 264, 36 ), 2 );
}

TEST( vic_renderer_multicolor_sprite_decodes_shared_and_individual_colours )
{
	VICFixture f;
	f.sprite( 0, 24, 50, 0x20, 2 );
	f.state.vic[ 0x1C ] = 1;
	f.state.vic[ 0x25 ] = 5;
	f.state.vic[ 0x26 ] = 7;
	f.ram[ 0x0800 ] = 0x6C;
	f.renderer.render( f.state, f.pixels, VIC_RENDER_WIDTH );
	CHECK_EQ( f.pixel( 32, 36 ), 5 );
	CHECK_EQ( f.pixel( 33, 36 ), 5 );
	CHECK_EQ( f.pixel( 34, 36 ), 2 );
	CHECK_EQ( f.pixel( 35, 36 ), 2 );
	CHECK_EQ( f.pixel( 36, 36 ), 7 );
	CHECK_EQ( f.pixel( 37, 36 ), 7 );
	CHECK_EQ( f.pixel( 38, 36 ), 6 );
}

TEST( vic_renderer_sprite_x_and_y_expansion_repeat_pixels_and_lines )
{
	VICFixture f;
	f.sprite( 0, 24, 50, 0x20, 2 );
	f.state.vic[ 0x17 ] = 1;
	f.state.vic[ 0x1D ] = 1;
	f.ram[ 0x0800 ] = 0x80;
	f.renderer.render( f.state, f.pixels, VIC_RENDER_WIDTH );
	CHECK_EQ( f.pixel( 32, 36 ), 2 );
	CHECK_EQ( f.pixel( 33, 36 ), 2 );
	CHECK_EQ( f.pixel( 32, 37 ), 2 );
	CHECK_EQ( f.pixel( 33, 37 ), 2 );
	CHECK_EQ( f.pixel( 34, 36 ), 6 );
}

TEST( vic_renderer_sprite_priority_respects_foreground_graphics )
{
	VICFixture f;
	f.ram[ 0x0400 ] = 1;
	f.colours[ 0 ] = 5;
	f.chars[ 8 ] = 0x80;
	f.sprite( 0, 24, 50, 0x20, 2 );
	f.state.vic[ 0x1B ] = 1;
	f.ram[ 0x0800 ] = 0xC0;
	f.renderer.render( f.state, f.pixels, VIC_RENDER_WIDTH );
	CHECK_EQ( f.pixel( 32, 36 ), 5 );
	CHECK_EQ( f.pixel( 33, 36 ), 2 );
}

TEST( vic_renderer_lower_numbered_sprite_wins_overlap )
{
	VICFixture f;
	f.sprite( 0, 24, 50, 0x20, 2 );
	f.sprite( 1, 24, 50, 0x21, 3 );
	f.ram[ 0x0800 ] = 0x80;
	f.ram[ 0x0840 ] = 0x80;
	f.renderer.render( f.state, f.pixels, VIC_RENDER_WIDTH );
	CHECK_EQ( f.pixel( 32, 36 ), 2 );
}

TEST( vic_renderer_scanline_bands_match_a_whole_frame )
{
	VICFixture whole;
	VICFixture banded;
	whole.state.vic[ 0x11 ] = banded.state.vic[ 0x11 ] = 0x3B;
	whole.state.vic[ 0x16 ] = banded.state.vic[ 0x16 ] = 0x18;
	whole.state.bitmapBase = banded.state.bitmapBase = 0x2000;
	whole.ram[ 0x0400 ] = banded.ram[ 0x0400 ] = 0xA5;
	whole.colours[ 0 ] = banded.colours[ 0 ] = 7;
	whole.ram[ 0x2000 ] = banded.ram[ 0x2000 ] = 0x1B;
	whole.sprite( 0, 24, 50, 0x20, 2 );
	banded.sprite( 0, 24, 50, 0x20, 2 );
	whole.ram[ 0x0800 ] = banded.ram[ 0x0800 ] = 0x80;
	CHECK_EQ( whole.renderer.render( whole.state, whole.pixels,
	                                 VIC_RENDER_WIDTH ),
	          VIC_RENDER_MULTICOLOR_BITMAP );
	memset( banded.pixels, 0xCC, sizeof banded.pixels );
	for ( u32 first = 0; first < VIC_RENDER_HEIGHT; first += 7 )
		CHECK_EQ( banded.renderer.renderRows( banded.state,
		                                       banded.pixels
		                                         + first * VIC_RENDER_WIDTH,
		                                       VIC_RENDER_WIDTH, first, 7 ),
		          VIC_RENDER_MULTICOLOR_BITMAP );
	CHECK( memcmp( whole.pixels, banded.pixels, sizeof whole.pixels ) == 0 );
}

TEST( vic_renderer_scanline_band_does_not_touch_other_rows )
{
	VICFixture f;
	memset( f.pixels, 0xCC, sizeof f.pixels );
	f.renderer.renderRows( f.state, f.pixels + 10 * VIC_RENDER_WIDTH,
	                       VIC_RENDER_WIDTH, 10, 8 );
	for ( u32 i = 0; i < 10 * VIC_RENDER_WIDTH; i++ )
		CHECK_EQ( f.pixels[ i ], 0xCC );
	for ( u32 i = 10 * VIC_RENDER_WIDTH; i < 18 * VIC_RENDER_WIDTH; i++ )
		CHECK( f.pixels[ i ] != 0xCC );
	for ( u32 i = 18 * VIC_RENDER_WIDTH; i < sizeof f.pixels; i++ )
		CHECK_EQ( f.pixels[ i ], 0xCC );
}
