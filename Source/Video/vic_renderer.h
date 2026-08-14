/*
   SCPU-EMU - passive VIC-II picture renderer

   This component converts an immutable snapshot of Pi-side C64 shadow state
   into palette indices.  It has no Circle or RAD dependencies and, by design,
   no path to the physical Commodore bus.
*/
#ifndef _scpu_vic_renderer_h
#define _scpu_vic_renderer_h

#include "../Common/types.h"

#define VIC_RENDER_WIDTH       384
#define VIC_RENDER_HEIGHT      272
#define VIC_RENDER_DISPLAY_X    32
#define VIC_RENDER_DISPLAY_Y    36
#define VIC_RENDER_DISPLAY_W   320
#define VIC_RENDER_DISPLAY_H   200

enum VICRenderMode
{
	VIC_RENDER_BLANK = 0,
	VIC_RENDER_STANDARD_TEXT,
	VIC_RENDER_MULTICOLOR_TEXT,
	VIC_RENDER_EXTENDED_TEXT,
	VIC_RENDER_STANDARD_BITMAP,
	VIC_RENDER_MULTICOLOR_BITMAP,
	VIC_RENDER_UNSUPPORTED
};

struct VICRenderState
{
	const u8 *ram;
	const u8 *charROM;
	const u8 *colourRAM;
	u32 bankBase;
	u32 screenBase;
	u32 charsetBase;
	u32 bitmapBase;
	// FLI changes D011.YSCROLL inside the visible frame to force badlines; it
	// does not move every independently-rendered band. Use neutral YSCROLL for
	// the entire frame when the raster planner observes that pattern.
	bool yScrollVaries;
	// Last value written to each VIC-II register. Keeping the register file in
	// the render snapshot makes graphics and sprite decoding self-contained and
	// avoids dozens of independently-racing scalar reads on core 1.
	u8 vic[ 0x40 ];
};

class CVICRenderer
{
public:
	// Render one complete bordered frame into one-byte VIC colour indices.
	// pitch is measured in bytes and must be at least VIC_RENDER_WIDTH.
	VICRenderMode render( const VICRenderState &state, u8 *pixels,
	                      u32 pitch ) const;

	// Render only the selected output scanlines. pixels points at storage for
	// firstRow (not at the start of a full-frame image), so callers can reuse a
	// small band buffer. The result is byte-for-byte identical to render().
	VICRenderMode renderRows( const VICRenderState &state, u8 *pixels,
	                          u32 pitch, u32 firstRow, u32 rowCount ) const;
};

#endif
