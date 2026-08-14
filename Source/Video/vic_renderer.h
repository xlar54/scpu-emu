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
	VIC_RENDER_TEXT,
	VIC_RENDER_ABORTED,
	VIC_RENDER_UNSUPPORTED
};

typedef bool (*VICRenderAbortCheck)( void *context );

struct VICRenderState
{
	const u8 *ram;
	const u8 *charROM;
	const u8 *colourRAM;
	u32 screenBase;
	u32 charsetBase;
	u8 d011;
	u8 d016;
	u8 d018;
	u8 border;
	u8 background;
};

class CVICRenderer
{
public:
	// Render one complete bordered frame into one-byte VIC colour indices.
	// pitch is measured in bytes and must be at least VIC_RENDER_WIDTH.
	VICRenderMode render( const VICRenderState &state, u8 *pixels,
	                      u32 pitch, VICRenderAbortCheck abort = 0,
	                      void *abortContext = 0 ) const;

private:
	static bool fillRect( u8 *pixels, u32 pitch, u32 x, u32 y,
	                     u32 width, u32 height, u8 colour,
	                     VICRenderAbortCheck abort, void *abortContext );
};

#endif
