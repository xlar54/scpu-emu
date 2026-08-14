/*
   SCPU-EMU - passive VIC-II picture renderer
*/
#include "vic_renderer.h"

bool CVICRenderer::fillRect( u8 *pixels, u32 pitch, u32 x, u32 y,
	                         u32 width, u32 height, u8 colour,
	                         VICRenderAbortCheck abort, void *abortContext )
{
	for ( u32 row = 0; row < height; row++ )
	{
		if ( abort && abort( abortContext ) ) return false;
		u8 *dst = pixels + ( y + row ) * pitch + x;
		for ( u32 col = 0; col < width; col++ ) dst[ col ] = colour;
	}
	return true;
}

VICRenderMode CVICRenderer::render( const VICRenderState &state, u8 *pixels,
	                                u32 pitch, VICRenderAbortCheck abort,
	                                void *abortContext ) const
{
	if ( !pixels || pitch < VIC_RENDER_WIDTH ) return VIC_RENDER_BLANK;

	const u8 border = state.border & 0x0F;
	const u8 background = state.background & 0x0F;
	if ( !fillRect( pixels, pitch, 0, 0,
	                VIC_RENDER_WIDTH, VIC_RENDER_HEIGHT, border,
	                abort, abortContext ) )
		return VIC_RENDER_ABORTED;

	// DEN clear leaves only the border colour visible.
	if ( !( state.d011 & 0x10 ) ) return VIC_RENDER_BLANK;

	if ( !fillRect( pixels, pitch, VIC_RENDER_DISPLAY_X, VIC_RENDER_DISPLAY_Y,
	                VIC_RENDER_DISPLAY_W, VIC_RENDER_DISPLAY_H, background,
	                abort, abortContext ) )
		return VIC_RENDER_ABORTED;

	// Stage one is intentionally honest: do not draw bitmap, ECM or
	// multicolour data using the standard-text rules.
	if ( ( state.d011 & 0x60 ) || ( state.d016 & 0x10 )
	     || !state.ram || state.screenBase == 0xFFFFFFFF )
		return VIC_RENDER_UNSUPPORTED;

	const u8 *patterns = 0;
	u32 patternBase = 0;
	if ( state.charsetBase == 0xFFFFFFFF )
	{
		patterns = state.charROM;
		if ( patterns ) patternBase = ( state.d018 & 0x02 ) ? 0x0800 : 0;
	}
	else
	{
		patterns = state.ram;
		patternBase = state.charsetBase;
	}
	if ( !patterns ) return VIC_RENDER_UNSUPPORTED;

	for ( u32 row = 0; row < 25; row++ )
	{
		if ( abort && abort( abortContext ) ) return VIC_RENDER_ABORTED;
		for ( u32 col = 0; col < 40; col++ )
		{
			const u32 cell = row * 40 + col;
			const u8 code = state.ram[ (u16)( state.screenBase + cell ) ];
			const u8 foreground = state.colourRAM
			                    ? state.colourRAM[ cell & 0x03FF ] & 0x0F : 14;
			const u32 glyph = patternBase + (u32)code * 8;
			for ( u32 line = 0; line < 8; line++ )
			{
				const u8 bits = patterns[ (u16)( glyph + line ) ];
				u8 *dst = pixels
				        + ( VIC_RENDER_DISPLAY_Y + row * 8 + line ) * pitch
				        + VIC_RENDER_DISPLAY_X + col * 8;
				for ( u32 bit = 0; bit < 8; bit++ )
					dst[ bit ] = ( bits & ( 0x80 >> bit ) )
					           ? foreground : background;
			}
		}
	}
	return VIC_RENDER_TEXT;
}
