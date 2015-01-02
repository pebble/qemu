#!/usr/bin/env python

"""
Convert a PNG image into a C struct
"""

import itertools
import math
import png

_NoFill = object()
def unflatten(iterable, n, fill=_NoFill):
    count = 0
    accumulator = []
    for item in iterable:
        accumulator.append(item)
        count += 1
        if count == n:
            yield tuple(accumulator)
            accumulator = []
            count = 0
    if accumulator and fill is not _NoFill:
        accumulator += [fill] * (n-count)
        yield tuple(accumulator)

def rgb_triplets(row):
    """Packs a flattened row of RGB pixel data into (r,g,b) tuples."""
    return unflatten(row, 3)

def to_rgb222(row):
    """Converts pixels from RGB888 to RGB222 format.

    The function takes an iterable of (r,g,b) tuples, each representing
    a pixel. Each pixel is downsampled to RGB222 and packed into a
    single byte. The format of the byte is 0bRrGgBb00.
    """
    def downsample(value):
        return (value >> 6) & 0x3

    for r, g, b in row:
        yield downsample(r) << 6 | downsample(g) << 4 | downsample(b) << 2

def png_to_bytes(png_reader, converter=to_rgb222):
    width, height, image, _ = png_reader.asRGB8()
    rows = [rgb_triplets(row) for row in image]
    scanlines = [converter(row) for row in rows]
    return height, width, scanlines

def png_to_cstruct(png_reader, dst_name, var_name):
    height, width, image = png_to_bytes(png_reader)

    print "// C struct format image converted from '%s' using png_to_cstruct.py" % args.src.name
    print "static uint8_t *get_%s_image(int *width, int *height) {\n" % (var_name)
    print "    *width = %d;" % (width)
    print "    *height = %d;" % (height)
    print "    static uint8_t %s[%d] = {\n" % (var_name, width*height)

    for line in image:
        print "      ",
        for i, pixel in enumerate(line):
            if (i > 0) and (i % 12) == 0:
                print "\n      ",
            print " 0x%02x," % pixel,
        print "\n"

    print "    };"
    print "    return %s;" % (var_name)
    print "}"


if __name__=='__main__':
    import argparse
    import os.path
    import re

    parser = argparse.ArgumentParser()
    parser.add_argument('src', metavar='PNGFILE', type=argparse.FileType('rb'),
                        help='image file to convert')
    args = parser.parse_args()

    # Attempt to strip ".png" suffix. Not terribly smart, but good enough.
    dst_name = os.path.splitext(args.src.name)[0] + '.c'

    # Attempt to strip ".png" suffix. Not terribly smart, but good enough.
    var_name = re.sub(r'[^a-zA-Z0-9_]', '_',
                     os.path.splitext(os.path.basename(args.src.name))[0])

    reader = png.Reader(args.src)
    png_to_cstruct(reader, dst_name, var_name)

