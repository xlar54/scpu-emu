#!/usr/bin/env python3
"""Check where raster splits land, against a VICE reference frame.

The band-replay path cannot be driven from a single state dump -- a raster
effect is several states inside one frame by definition -- so this tests the
part that can be checked without constructing a write log by hand, and that is
the part which was demonstrably wrong: the raster-line to output-row mapping.

A test program that changes the border colour at known raster lines produces
bands whose boundaries are directly measurable from the reference screenshot.
Our mapping is row = raster - (displayFirstLine - VIC_RENDER_DISPLAY_Y), so the
predicted boundaries follow from geometry alone. If they disagree, every raster
effect is displaced by the difference, uniformly, which is exactly the failure
that is easy to mistake for jitter.

    check_raster.py <vice.png> <first-display-line> <display-y> <line>...
"""
import sys

from PIL import Image


def main():
    if len(sys.argv) < 5:
        sys.exit("usage: check_raster.py <vice.png> <first-display-line> "
                 "<display-y> <split-raster-line>...")
    png, first_line, display_y = sys.argv[1], int(sys.argv[2]), int(sys.argv[3])
    splits = [int(a) for a in sys.argv[4:]]

    img = Image.open(png).convert("RGB")
    width, height = img.size
    px = img.load()

    # Column 0 is border on every row, whatever the mode or aperture.
    observed, previous = [], None
    for y in range(height):
        colour = px[0, y]
        if previous is not None and colour != previous:
            observed.append(y)
        previous = colour

    top = first_line - display_y
    predicted = [line - top for line in splits]

    print(f"reference    : {png} ({width}x{height})")
    print(f"topRaster    : {first_line} - {display_y} = {top}")
    print(f"predicted    : {predicted}")
    print(f"observed     : {observed}")

    if observed == predicted:
        print("result       : match")
        return 0

    # A constant difference is the informative case: it means the whole frame is
    # displaced rather than individual bands being misplaced.
    if len(observed) == len(predicted):
        deltas = {o - p for o, p in zip(observed, predicted)}
        if len(deltas) == 1:
            d = deltas.pop()
            print(f"result       : MISMATCH -- every split displaced by {d:+d} row(s)")
            print("               a uniform shift is a geometry error, not jitter")
            return 1
    print("result       : MISMATCH -- splits are individually misplaced")
    return 1


if __name__ == "__main__":
    sys.exit(main())
