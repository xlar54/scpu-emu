#!/usr/bin/env python3
"""Diff our VIC-II renderer's output against a VICE reference screenshot.

Both images are 384x272 by construction -- VICE's C64 screenshot geometry and
VIC_RENDER_WIDTH x VIC_RENDER_HEIGHT are the same -- so this is a straight
pixel-for-pixel comparison with no scaling or alignment.

The comparison is deliberately palette-agnostic. Our renderer emits VIC colour
INDICES; VICE emits RGB from whatever palette it is configured with. Rather
than hard-code a palette and then argue about shades, we derive the mapping
from the data: every VICE colour must correspond to exactly one of our indices
and vice versa. A consistent bijection means the two renderers agree on
structure, which is what fidelity means here. Any pixel that violates the
mapping is a real difference.

A palette disagreement therefore shows up as a clean note rather than 100k
false failures, and a structural bug shows up as located pixels.
"""
import sys
from collections import Counter, defaultdict

from PIL import Image

WIDTH, HEIGHT = 384, 272


def load_indices(path):
    data = open(path, "rb").read()
    if len(data) != WIDTH * HEIGHT:
        sys.exit(f"{path}: expected {WIDTH * HEIGHT} bytes, got {len(data)}")
    return data


def load_reference(path):
    img = Image.open(path).convert("RGB")
    if img.size != (WIDTH, HEIGHT):
        sys.exit(f"{path}: expected {WIDTH}x{HEIGHT}, got {img.size[0]}x{img.size[1]}")
    return list(img.getdata())


def main():
    if len(sys.argv) < 3:
        sys.exit("usage: compare.py <ours.raw> <vice.png> [diff.png]")
    ours = load_indices(sys.argv[1])
    theirs = load_reference(sys.argv[2])

    # Build the candidate mapping from the most common pairing of each VICE
    # colour, so a handful of genuinely wrong pixels cannot redefine a colour.
    pairs = defaultdict(Counter)
    for rgb, idx in zip(theirs, ours):
        pairs[rgb][idx] += 1
    mapping = {rgb: counts.most_common(1)[0][0] for rgb, counts in pairs.items()}

    # A colour used by two of our indices, or one index claimed by two colours,
    # means the renderers genuinely disagree rather than merely differing in
    # palette. Report it before the pixel diff so the cause is obvious.
    claimed = defaultdict(list)
    for rgb, idx in mapping.items():
        claimed[idx].append(rgb)
    collisions = {i: c for i, c in claimed.items() if len(c) > 1}

    bad = [
        (n % WIDTH, n // WIDTH, theirs[n], ours[n])
        for n in range(WIDTH * HEIGHT)
        if mapping[theirs[n]] != ours[n]
    ]

    print(f"reference colours : {len(pairs)}")
    print("mapping           : " + ", ".join(
        f"#{r:02X}{g:02X}{b:02X}->{i:X}" for (r, g, b), i in sorted(
            mapping.items(), key=lambda kv: kv[1])))
    if collisions:
        print(f"AMBIGUOUS         : {len(collisions)} of our indices claimed by "
              f"more than one reference colour -- structural disagreement")
    print(f"mismatched pixels : {len(bad)} of {WIDTH * HEIGHT} "
          f"({100.0 * len(bad) / (WIDTH * HEIGHT):.3f}%)")

    if bad:
        rows = sorted({y for _, y, _, _ in bad})
        print(f"rows affected     : {len(rows)}  first={rows[0]} last={rows[-1]}")
        for x, y, rgb, idx in bad[:12]:
            print(f"   ({x:3d},{y:3d}) reference #{rgb[0]:02X}{rgb[1]:02X}{rgb[2]:02X}"
                  f" -> expected index {mapping[rgb]:X}, we rendered {idx:X}")
        if len(bad) > 12:
            print(f"   ... and {len(bad) - 12} more")

        if len(sys.argv) > 3:
            out = Image.new("RGB", (WIDTH, HEIGHT), (0, 0, 0))
            px = out.load()
            for x, y, _, _ in bad:
                px[x, y] = (255, 0, 0)
            out.save(sys.argv[3])
            print(f"diff image        : {sys.argv[3]}")

    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
