/*
   SCPU-EMU - passive VIC-II picture renderer
*/
#include "vic_renderer.h"

#include <string.h>

struct VICRenderContext
{
	VICRenderMode mode;
	const u8 *patterns;
	u32 patternBase;
	u8 background[ 4 ];
};

static inline u8 vicColour( const VICRenderState &state, u32 reg )
{
	return state.vic[ reg & 0x3F ] & 0x0F;
}

static VICRenderContext makeContext( const VICRenderState &state )
{
	VICRenderContext context;
	context.mode = VIC_RENDER_INVALID;
	context.patterns = 0;
	context.patternBase = 0;
	for ( u32 i = 0; i < 4; i++ )
		context.background[ i ] = vicColour( state, 0x21 + i );

	if ( !( state.vic[ 0x11 ] & 0x10 ) )
	{
		context.mode = VIC_RENDER_BLANK;
		return context;
	}

	const bool ecm = ( state.vic[ 0x11 ] & 0x40 ) != 0;
	const bool bmm = ( state.vic[ 0x11 ] & 0x20 ) != 0;
	const bool mcm = ( state.vic[ 0x16 ] & 0x10 ) != 0;
	if ( !ecm && !bmm )
		context.mode = mcm ? VIC_RENDER_MULTICOLOR_TEXT
		                   : VIC_RENDER_STANDARD_TEXT;
	else if ( ecm && !bmm && !mcm )
		context.mode = VIC_RENDER_EXTENDED_TEXT;
	else if ( !ecm && bmm )
		context.mode = mcm ? VIC_RENDER_MULTICOLOR_BITMAP
		                   : VIC_RENDER_STANDARD_BITMAP;

	if ( !state.ram || state.screenBase >= 0x10000 )
	{
		context.mode = VIC_RENDER_UNSUPPORTED;
		return context;
	}

	if ( context.mode == VIC_RENDER_STANDARD_TEXT
	     || context.mode == VIC_RENDER_MULTICOLOR_TEXT
	     || context.mode == VIC_RENDER_EXTENDED_TEXT )
	{
		if ( state.charsetBase == 0xFFFFFFFF )
		{
			context.patterns = state.charROM;
			context.patternBase = ( state.vic[ 0x18 ] & 0x02 ) ? 0x0800 : 0;
		}
		else if ( state.charsetBase < 0x10000 )
		{
			context.patterns = state.ram;
			context.patternBase = state.charsetBase;
		}
		if ( !context.patterns ) context.mode = VIC_RENDER_UNSUPPORTED;
	}
	else if ( ( context.mode == VIC_RENDER_STANDARD_BITMAP
	           || context.mode == VIC_RENDER_MULTICOLOR_BITMAP )
	          && state.bitmapBase >= 0x10000 )
		context.mode = VIC_RENDER_UNSUPPORTED;

	return context;
}

// Decode one complete character-cell scanline. The former per-pixel decoder
// fetched the same screen, colour and pattern bytes eight times and made an
// out-of-line call for every visible pixel. Cell decoding keeps those values
// live in registers and reduces the hot-loop call count by eight.
static inline void decodeGraphicsCell( const VICRenderState &state,
	                                   const VICRenderContext &context,
	                                   u32 cell, u32 glyphLine,
	                                   u8 pixels[ 8 ], u8 &opaqueMask )
{
	const u8 screen = state.ram[ (u16)( state.screenBase + cell ) ];
	const u8 cellColour = state.colourRAM
	                    ? state.colourRAM[ cell & 0x03FF ] & 0x0F : 14;
	opaqueMask = 0;
	switch ( context.mode )
	{
	case VIC_RENDER_STANDARD_TEXT:
	{
		const u8 bits = context.patterns[ (u16)( context.patternBase
		                                           + (u32)screen * 8
		                                           + glyphLine ) ];
		opaqueMask = bits;
		for ( u32 pixel = 0; pixel < 8; pixel++ )
			pixels[ pixel ] = bits & ( 0x80 >> pixel )
			                ? cellColour : context.background[ 0 ];
		break;
	}

	case VIC_RENDER_MULTICOLOR_TEXT:
	{
		const u8 bits = context.patterns[ (u16)( context.patternBase
		                                           + (u32)screen * 8
		                                           + glyphLine ) ];
		if ( !( cellColour & 0x08 ) )
		{
			opaqueMask = bits;
			for ( u32 pixel = 0; pixel < 8; pixel++ )
				pixels[ pixel ] = bits & ( 0x80 >> pixel )
				                ? cellColour & 0x07 : context.background[ 0 ];
		}
		else
		{
			for ( u32 pair = 0; pair < 4; pair++ )
			{
				const u8 value = (u8)( bits >> ( 6 - pair * 2 ) ) & 3;
				const u8 colour = value == 0 ? context.background[ 0 ]
				                : value == 1 ? context.background[ 1 ]
				                : value == 2 ? context.background[ 2 ]
				                             : cellColour & 0x07;
				pixels[ pair * 2 ] = pixels[ pair * 2 + 1 ] = colour;
				// VIC-II multicolour value 01 is background colour 1 and does
				// not occlude a sprite whose D01B priority bit is set. Only
				// values 10 and 11 belong to foreground graphics.
				if ( value >= 2 ) opaqueMask |= (u8)( 0xC0 >> ( pair * 2 ) );
			}
		}
		break;
	}

	case VIC_RENDER_EXTENDED_TEXT:
	{
		const u8 code = screen & 0x3F;
		const u8 bits = context.patterns[ (u16)( context.patternBase
		                                           + (u32)code * 8
		                                           + glyphLine ) ];
		opaqueMask = bits;
		const u8 background = context.background[ screen >> 6 ];
		for ( u32 pixel = 0; pixel < 8; pixel++ )
			pixels[ pixel ] = bits & ( 0x80 >> pixel )
			                ? cellColour : background;
		break;
	}

	case VIC_RENDER_STANDARD_BITMAP:
	{
		const u8 bits = state.ram[ (u16)( state.bitmapBase + cell * 8
		                                      + glyphLine ) ];
		opaqueMask = bits;
		for ( u32 pixel = 0; pixel < 8; pixel++ )
			pixels[ pixel ] = bits & ( 0x80 >> pixel )
			                ? screen >> 4 : screen & 0x0F;
		break;
	}

	case VIC_RENDER_MULTICOLOR_BITMAP:
	{
		const u8 bits = state.ram[ (u16)( state.bitmapBase + cell * 8
		                                      + glyphLine ) ];
		for ( u32 pair = 0; pair < 4; pair++ )
		{
			const u8 value = (u8)( bits >> ( 6 - pair * 2 ) ) & 3;
			const u8 colour = value == 0 ? context.background[ 0 ]
			                : value == 1 ? screen >> 4
			                : value == 2 ? screen & 0x0F
			                             : cellColour;
			pixels[ pair * 2 ] = pixels[ pair * 2 + 1 ] = colour;
			if ( value >= 2 ) opaqueMask |= (u8)( 0xC0 >> ( pair * 2 ) );
		}
		break;
	}

	default:
		memset( pixels, context.background[ 0 ], 8 );
		break;
	}
}

static inline bool outputToSource( const VICRenderState &state,
	                               u32 x, u32 y, u32 activeLeft,
	                               u32 activeRight, u32 activeTop,
	                               u32 activeBottom, u32 &sourceX,
	                               u32 &sourceY )
{
	if ( x < activeLeft || x >= activeRight || y < activeTop || y >= activeBottom )
		return false;
	const int originX = VIC_RENDER_DISPLAY_X + ( state.vic[ 0x16 ] & 7 );
	// $1B is the KERNAL's neutral 25-row value; changing YSCROLL moves the
	// matrix relative to that position while RSEL controls the border aperture.
	const int originY = VIC_RENDER_DISPLAY_Y
	                  + ( state.yScrollVaries
	                      ? 0 : (int)( state.vic[ 0x11 ] & 7 ) - 3 );
	const int sx = (int)x - originX;
	const int sy = (int)y - originY;
	if ( sx < 0 || sx >= VIC_RENDER_DISPLAY_W
	     || sy < 0 || sy >= VIC_RENDER_DISPLAY_H )
		return false;
	sourceX = (u32)sx;
	sourceY = (u32)sy;
	return true;
}

static void prepareSpriteRow( const VICRenderState &state, u32 row,
	                          VICRenderSpriteSequencer &sequencer,
	                          VICRenderState &rowState )
{
	rowState = state;
	const u8 enabled = state.vic[ 0x15 ];
	const bool continuous = sequencer.primed && sequencer.nextRow == row;
	if ( !continuous )
	{
		// A discontinuous band render has no earlier rows from which to derive
		// the internal DMA state. Seed it from sprites whose static extent
		// already covers this row; this also preserves sprites with Y < 14 at
		// the top of the HDMI aperture.
		sequencer.active = 0;
		sequencer.expanded = 0;
		for ( u32 n = 0; n < 8; n++ )
		{
			const u8 mask = (u8)( 1u << n );
			if ( !( enabled & mask ) ) continue;
			const bool expand = ( state.vic[ 0x17 ] & mask ) != 0;
			const int top = (int)state.vic[ n * 2 + 1 ] - 15;
			const int height = expand ? 42 : 21;
			if ( (int)row < top || (int)row >= top + height ) continue;
			sequencer.active |= mask;
			if ( expand ) sequencer.expanded |= mask;
			sequencer.y[ n ] = state.vic[ n * 2 + 1 ];
			sequencer.rowsLeft[ n ] = (u8)( top + height - (int)row );
		}
		sequencer.primed = true;
	}
	else
	{
		// Only an inactive sprite can respond to a new Y comparison. Writes to
		// Y or Y-expand while its DMA is active must not remap the remaining
		// rows of the sprite already being displayed.
		for ( u32 n = 0; n < 8; n++ )
		{
			const u8 mask = (u8)( 1u << n );
			if ( sequencer.active & mask || !( enabled & mask ) ) continue;
			const int top = (int)state.vic[ n * 2 + 1 ] - 15;
			if ( top != (int)row ) continue;
			const bool expand = ( state.vic[ 0x17 ] & mask ) != 0;
			sequencer.active |= mask;
			if ( expand ) sequencer.expanded |= mask;
			else          sequencer.expanded &= (u8)~mask;
			sequencer.y[ n ] = state.vic[ n * 2 + 1 ];
			sequencer.rowsLeft[ n ] = expand ? 42 : 21;
		}
	}

	// Enable, X, colour, priority and multicolour remain live per line. Only
	// the trigger-owned Y/Y-expand fields come from the sequencer.
	rowState.vic[ 0x15 ] = enabled & sequencer.active;
	rowState.vic[ 0x17 ] = (u8)( ( state.vic[ 0x17 ]
	                              & (u8)~sequencer.active )
	                            | ( sequencer.expanded
	                              & sequencer.active ) );
	for ( u32 n = 0; n < 8; n++ )
		if ( sequencer.active & ( 1u << n ) )
			rowState.vic[ n * 2 + 1 ] = sequencer.y[ n ];
	sequencer.nextRow = (u16)( row + 1 );
}

static void finishSpriteRow( VICRenderSpriteSequencer &sequencer )
{
	for ( u32 n = 0; n < 8; n++ )
	{
		const u8 mask = (u8)( 1u << n );
		if ( !( sequencer.active & mask ) ) continue;
		if ( sequencer.rowsLeft[ n ] ) sequencer.rowsLeft[ n ]--;
		if ( !sequencer.rowsLeft[ n ] )
		{
			sequencer.active &= (u8)~mask;
			sequencer.expanded &= (u8)~mask;
		}
	}
}

static void renderSpritesOnRow( const VICRenderState &state,
	                            const VICRenderContext &context, u8 *dst,
	                            u32 y, u32 activeLeft, u32 activeRight,
	                            u32 activeTop, u32 activeBottom,
	                            const u8 opaqueCells[ 40 ],
	                            VICRenderCollisions *collisions,
	                            bool drawRow )
{
	const u8 enabled = state.vic[ 0x15 ];
	if ( !enabled || !state.ram || state.screenBase >= 0x10000 ) return;
	// Deliberately NOT gated on the display window. Collision detection lives
	// in the sprite sequencer and is independent of the border unit, so
	// sprites collide wherever their data is non-transparent -- including rows
	// and columns the border covers. Only drawing is clipped, below.

	// Sprite X is nine bits, so a sprite can sit entirely outside the frame and
	// still collide with another one there. Coverage therefore spans the whole
	// reachable sprite range rather than the visible width: 511 + 8 for the
	// display origin + 48 for an expanded sprite.
	static const u32 SpriteCoverageWidth = 568;
	u8 spriteCoverage[ SpriteCoverageWidth ];
	if ( collisions ) memset( spriteCoverage, 0, sizeof spriteCoverage );

	// Sprite zero has the highest sprite-to-sprite priority, so composite from
	// seven down to zero and let the lower-numbered sprite win.
	for ( int n = 7; n >= 0; n-- )
	{
		const u8 mask = (u8)( 1u << n );
		if ( !( enabled & mask ) ) continue;
		const bool expandY = ( state.vic[ 0x17 ] & mask ) != 0;
		const bool expandX = ( state.vic[ 0x1D ] & mask ) != 0;
		const bool multicolor = ( state.vic[ 0x1C ] & mask ) != 0;
		const bool behind = ( state.vic[ 0x1B ] & mask ) != 0;
		const int top = (int)state.vic[ n * 2 + 1 ] - 15;
		const int outputLine = (int)y - top;
		const int outputHeight = expandY ? 42 : 21;
		if ( outputLine < 0 || outputLine >= outputHeight ) continue;
		const u32 spriteLine = (u32)( expandY ? outputLine >> 1 : outputLine );

		u32 spriteX = state.vic[ n * 2 ];
		if ( state.vic[ 0x10 ] & mask ) spriteX += 256;
		const int left = (int)spriteX + 8;
		const u32 pointerAddr = state.screenBase + 0x3F8 + (u32)n;
		const u8 pointer = state.ram[ (u16)pointerAddr ];
		const u32 shape = state.bankBase + (u32)pointer * 64 + spriteLine * 3;
		const u32 bits = ( (u32)state.ram[ (u16)shape ] << 16 )
		               | ( (u32)state.ram[ (u16)( shape + 1 ) ] << 8 )
		               | state.ram[ (u16)( shape + 2 ) ];
		const u32 outputWidth = expandX ? 48 : 24;
		for ( u32 outputPixel = 0; outputPixel < outputWidth; outputPixel++ )
		{
			const int xSigned = left + (int)outputPixel;
			if ( xSigned < 0 ) continue;
			const u32 x = (u32)xSigned;
			const u32 sourcePixel = expandX ? outputPixel >> 1 : outputPixel;
			u8 value;
			u8 colour;
			if ( multicolor )
			{
				value = (u8)( bits >> ( 22 - ( ( sourcePixel >> 1 ) * 2 ) ) ) & 3;
				if ( !value ) continue;
				colour = value == 1 ? vicColour( state, 0x25 )
				       : value == 2 ? vicColour( state, 0x27 + (u32)n )
				                    : vicColour( state, 0x26 );
			}
			else
			{
				value = ( bits & ( 1u << ( 23 - sourcePixel ) ) ) ? 1 : 0;
				if ( !value ) continue;
				colour = vicColour( state, 0x27 + (u32)n );
			}

			if ( collisions && x < SpriteCoverageWidth )
			{
				const u8 previous = spriteCoverage[ x ];
				if ( previous )
					collisions->spriteSprite |= (u8)( previous | mask );
				spriteCoverage[ x ] = (u8)( previous | mask );

				if ( context.mode != VIC_RENDER_UNSUPPORTED
				     && context.mode != VIC_RENDER_INVALID )
				{
					u32 sourceX, sourceY;
					if ( outputToSource( state, x, y, activeLeft, activeRight,
					                     activeTop, activeBottom,
					                     sourceX, sourceY )
					     && ( opaqueCells[ sourceX >> 3 ]
					          & ( 0x80 >> ( sourceX & 7 ) ) ) )
						collisions->spriteBackground |= mask;
				}
			}

			// Everything above is detection and happens anywhere the sprite
			// reaches. From here down is drawing, which the border does clip.
			if ( !drawRow || x < activeLeft || x >= activeRight ) continue;

			if ( behind && context.mode != VIC_RENDER_UNSUPPORTED
			     && context.mode != VIC_RENDER_INVALID )
			{
				u32 sourceX, sourceY;
				if ( outputToSource( state, x, y, activeLeft, activeRight,
				                     activeTop, activeBottom, sourceX, sourceY ) )
				{
					if ( opaqueCells[ sourceX >> 3 ]
					     & ( 0x80 >> ( sourceX & 7 ) ) ) continue;
				}
			}
			dst[ x ] = colour;
		}
	}
}

VICRenderMode CVICRenderer::render( const VICRenderState &state, u8 *pixels,
	                                u32 pitch,
	                                VICRenderCollisions *collisions,
	                                VICRenderSpriteSequencer *sprites ) const
{
	if ( collisions )
	{
		collisions->spriteSprite = 0;
		collisions->spriteBackground = 0;
	}
	return renderRows( state, pixels, pitch, 0, VIC_RENDER_HEIGHT,
	                   collisions, sprites );
}

VICRenderMode CVICRenderer::renderRows( const VICRenderState &state,
	                                    u8 *pixels, u32 pitch,
	                                    u32 firstRow, u32 rowCount,
	                                    VICRenderCollisions *collisions,
	                                    VICRenderSpriteSequencer *sprites ) const
{
	if ( !pixels || pitch < VIC_RENDER_WIDTH ) return VIC_RENDER_BLANK;
	if ( firstRow >= VIC_RENDER_HEIGHT ) rowCount = 0;
	else if ( rowCount > VIC_RENDER_HEIGHT - firstRow )
		rowCount = VIC_RENDER_HEIGHT - firstRow;

	const VICRenderContext context = makeContext( state );
	const u8 border = vicColour( state, 0x20 );
	const u8 background = vicColour( state, 0x21 );
	const u8 graphicsFill = context.mode == VIC_RENDER_INVALID
	                      ? 0 : background;
	const bool displayEnabled = context.mode != VIC_RENDER_BLANK;
	const bool columns40 = ( state.vic[ 0x16 ] & 0x08 ) != 0;
	const bool rows25 = ( state.vic[ 0x11 ] & 0x08 ) != 0;
	// CSEL narrows the display window ASYMMETRICALLY: the VIC's first visible
	// X moves 24 -> 31 and its last moves 343 -> 334, so the aperture loses
	// seven pixels on the left and nine on the right, not eight and eight.
	// Measured against VICE, which put the 38-column edges one pixel left of
	// where a symmetric trim placed them.
	const u32 activeLeft = VIC_RENDER_DISPLAY_X + ( columns40 ? 0 : 7 );
	const u32 activeRight = VIC_RENDER_DISPLAY_X + VIC_RENDER_DISPLAY_W
	                      - ( columns40 ? 0 : 9 );
	const u32 activeTop = VIC_RENDER_DISPLAY_Y + ( rows25 ? 0 : 4 );
	const u32 activeBottom = VIC_RENDER_DISPLAY_Y + VIC_RENDER_DISPLAY_H
	                       - ( rows25 ? 0 : 4 );

	const u32 endRow = firstRow + rowCount;
	for ( u32 y = firstRow; y < endRow; y++ )
	{
		VICRenderState spriteState;
		const VICRenderState *rowState = &state;
		if ( sprites )
		{
			prepareSpriteRow( state, y, *sprites, spriteState );
			rowState = &spriteState;
		}
		u8 *dst = pixels + ( y - firstRow ) * pitch;
		u8 opaqueCells[ 40 ];
		memset( opaqueCells, 0, sizeof opaqueCells );

		if ( !displayEnabled || y < activeTop || y >= activeBottom )
		{
			// A border row still runs the sprite sequencer, because collisions
			// occur there on real hardware. It draws nothing: opaqueCells stays
			// clear, so no sprite/background bit can be raised where there is
			// no foreground to collide with.
			memset( dst, border, VIC_RENDER_WIDTH );
			renderSpritesOnRow( *rowState, context, dst, y,
			                    activeLeft, activeRight, activeTop,
			                    activeBottom, opaqueCells, collisions, false );
			if ( sprites ) finishSpriteRow( *sprites );
			continue;
		}

		memset( dst, border, activeLeft );
		memset( dst + activeRight, border, VIC_RENDER_WIDTH - activeRight );
		memset( dst + activeLeft, graphicsFill, activeRight - activeLeft );
		const int originX = VIC_RENDER_DISPLAY_X + ( state.vic[ 0x16 ] & 7 );
		const int originY = VIC_RENDER_DISPLAY_Y
		                  + ( state.yScrollVaries
		                      ? 0 : (int)( state.vic[ 0x11 ] & 7 ) - 3 );
		const int sourceY = (int)y - originY;
		if ( context.mode != VIC_RENDER_UNSUPPORTED
		     && context.mode != VIC_RENDER_INVALID
		     && sourceY >= 0 && sourceY < VIC_RENDER_DISPLAY_H )
		{
			const u32 cellRow = (u32)sourceY >> 3;
			const u32 glyphLine = (u32)sourceY & 7;
			for ( u32 col = 0; col < 40; col++ )
			{
				u8 cellPixels[ 8 ];
				decodeGraphicsCell( state, context, cellRow * 40 + col,
				                    glyphLine, cellPixels, opaqueCells[ col ] );
				const int cellLeft = originX + (int)col * 8;
				const int copyLeft = cellLeft < (int)activeLeft
				                   ? (int)activeLeft : cellLeft;
				const int cellRight = cellLeft + 8;
				const int copyRight = cellRight > (int)activeRight
				                    ? (int)activeRight : cellRight;
				if ( copyLeft < copyRight )
					memcpy( dst + copyLeft, cellPixels + copyLeft - cellLeft,
					        (u32)( copyRight - copyLeft ) );
			}
		}
		renderSpritesOnRow( *rowState, context, dst, y, activeLeft, activeRight,
		                    activeTop, activeBottom, opaqueCells, collisions,
		                    true );
		if ( sprites ) finishSpriteRow( *sprites );
	}

	return context.mode;
}
