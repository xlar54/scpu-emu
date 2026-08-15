/*
   SCPU-EMU - render a captured VICE machine state through our VIC-II renderer

   Takes the RAM and I/O images that Tools/viceconf/capture.sh extracts from a
   headless xscpu64 run, rebuilds the VICRenderState the firmware would have
   built from the same registers, and writes the resulting 384x272 palette
   indices as a raw file for compare.py to diff against VICE's own screenshot.

   This is the whole point of the harness: the renderer is pure and has no
   Circle or RAD dependency, so the identical code that runs on core 1 can be
   driven from a PC against a reference emulator, with no hardware involved.
*/
#include "../../Source/Video/vic_renderer.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// VICE writes a two-byte little-endian load address ahead of a monitor "save".
static u8 *loadDump( const char *path, u32 expected, u32 &size )
{
	FILE *f = fopen( path, "rb" );
	if ( !f ) { fprintf( stderr, "cannot open %s\n", path ); return 0; }
	fseek( f, 0, SEEK_END );
	long total = ftell( f );
	fseek( f, 0, SEEK_SET );
	if ( total < 2 ) { fclose( f ); return 0; }

	// A raw ROM image has no header; a monitor save has exactly two extra bytes.
	const long skip = ( (u32)total == expected + 2 ) ? 2 : 0;
	fseek( f, skip, SEEK_SET );
	size = (u32)( total - skip );
	u8 *data = (u8 *)calloc( 1, size < expected ? expected : size );
	if ( data ) fread( data, 1, size, f );
	fclose( f );
	return data;
}

int main( int argc, char **argv )
{
	if ( argc < 5 )
	{
		fprintf( stderr,
		         "usage: render_state <ram.bin> <io.bin> <chargen.rom> <out.raw>\n" );
		return 2;
	}

	u32 ramSize = 0, ioSize = 0, romSize = 0;
	u8 *ram = loadDump( argv[ 1 ], 0x10000, ramSize );
	u8 *io = loadDump( argv[ 2 ], 0x1000, ioSize );
	u8 *rom = loadDump( argv[ 3 ], 0x1000, romSize );
	if ( !ram || !io || !rom ) return 1;

	// Colour RAM lives at $D800 inside the captured I/O window; the VIC
	// register file is the first 64 bytes of it.
	u8 colour[ 1024 ];
	for ( u32 i = 0; i < 1024; i++ ) colour[ i ] = io[ 0x800 + i ] & 0x0F;

	VICRenderState state;
	memset( &state, 0, sizeof state );
	state.ram = ram;
	state.charROM = rom;
	state.colourRAM = colour;
	state.yScrollVaries = false;
	for ( u32 i = 0; i < 0x40; i++ ) state.vic[ i ] = io[ i ];

	// Mirror CC64Memory's derivation exactly. CIA2 port A bits 0-1 select the
	// 16K VIC bank, inverted; $D018 then selects the matrix, character and
	// bitmap regions within it.
	const u8 d018 = io[ 0x18 ];
	const u8 dd00 = io[ 0xD00 ];
	const u32 bank = (u32)( ~dd00 & 0x03 );
	state.bankBase = bank << 14;
	state.screenBase = state.bankBase + ( (u32)( d018 >> 4 ) << 10 );

	const bool bitmapMode = ( io[ 0x11 ] & 0x20 ) != 0;
	state.bitmapBase = bitmapMode
	                 ? state.bankBase + ( ( d018 & 0x08 ) ? 0x2000 : 0 )
	                 : 0xFFFFFFFF;

	// In banks 0 and 2 the $1000-$1FFF window is the VIC's internal character
	// ROM rather than DRAM, which the renderer signals with the sentinel.
	if ( bitmapMode )
		state.charsetBase = 0xFFFFFFFF;
	else
	{
		const u32 offset = (u32)( d018 & 0x0E ) << 10;
		const bool romWindow = ( bank == 0 || bank == 2 )
		                    && offset >= 0x1000 && offset < 0x2000;
		state.charsetBase = romWindow ? 0xFFFFFFFF
		                              : state.bankBase + offset;
	}

	static u8 pixels[ VIC_RENDER_WIDTH * VIC_RENDER_HEIGHT ];
	CVICRenderer renderer;
	const VICRenderMode mode =
		renderer.render( state, pixels, VIC_RENDER_WIDTH );

	FILE *out = fopen( argv[ 4 ], "wb" );
	if ( !out ) { fprintf( stderr, "cannot write %s\n", argv[ 4 ] ); return 1; }
	fwrite( pixels, 1, sizeof pixels, out );
	fclose( out );

	fprintf( stderr,
	         "mode=%d bank=%u screen=$%04X charset=%s bitmap=%s "
	         "d011=$%02X d016=$%02X d018=$%02X border=%u bg=%u\n",
	         (int)mode, bank, state.screenBase,
	         state.charsetBase == 0xFFFFFFFF ? "ROM" : "RAM",
	         state.bitmapBase == 0xFFFFFFFF ? "-" : "yes",
	         io[ 0x11 ], io[ 0x16 ], d018,
	         io[ 0x20 ] & 15, io[ 0x21 ] & 15 );
	return 0;
}
