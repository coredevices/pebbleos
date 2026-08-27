#!/usr/bin/env python
# SPDX-FileCopyrightText: 2024 Google LLC
# SPDX-License-Identifier: Apache-2.0


import argparse
import os
import struct
import sys
from io import StringIO

import generate_c_byte_array
import png
from pebble_image_routines import (
    get_reduction_func,
    num_colors_to_bitdepth,
    rgba32_triplet_to_argb8,
)

SCALE_TO_GCOLOR8 = 64

WHITE_COLOR_MAP = {
    "white": 1,
    "black": 0,
    "transparent": 0,
}

BLACK_COLOR_MAP = {
    "white": 0,
    "black": 1,
    "transparent": 0,
}

# translates bitdepth to supported PBI format
# 0 is special case of legacy 1Bit format
bitdepth_dict = {
    0: 0,  # GBitmapFormat1Bit
    8: 1,  # GBitmapFormat8Bit
    1: 2,  # GBitmapFormat1BitPalette
    2: 3,  # GBitmapFormat2BitPalette
    4: 4,
}  # GBitmapFormat4BitPalette

FORMAT_BW = "bw"
FORMAT_COLOR = "color"
FORMAT_COLOR_RAW = "color_raw"  # forces output to be ARGB8 (no palette)

FORMAT_CHOICES = [FORMAT_BW, FORMAT_COLOR, FORMAT_COLOR_RAW]
DEFAULT_FORMAT = FORMAT_BW

TRUNCATE = "truncate"
NEAREST = "nearest"
COLOR_REDUCTION_CHOICES = [TRUNCATE, NEAREST]
DEFAULT_COLOR_REDUCTION = NEAREST

# Bitmap struct only contains a color palette for GBitmapFormat(1/2/4)BitPalette

# Bitmap struct (NB: All fields are little-endian)
# See docs/reference/formats/pbi.md for the rendered spec.
#             (uint16_t) row_size_bytes
#             (uint16_t) info_flags
#                    bit      0 : is heap allocated (must be zero for bitmap files)
#                    bits  1-3  : bitmap_format
#                    bit      4 : is palette heap allocated (must be zero for bitmap files)
#                    bits  5-11 : reserved, must be 0
#                    bits 12-15 : file version
#             (int16_t)  bounds.origin.x (legacy, ignored by the firmware)
#             (int16_t)  bounds.origin.y (legacy, ignored by the firmware)
#             (int16_t)  bounds.size.w
#             (int16_t)  bounds.size.h
#             (uint8_t)[] image data (row_size_bytes-aligned, 0-padded rows of bits)
# [optional]  (uint8_t)[] argb8 palette data (0-padded to 2 ** bitdepth)


class PebbleBitmap:
    def __init__(
        self,
        path,
        color_map=WHITE_COLOR_MAP,
        bitmap_format=DEFAULT_FORMAT,
        color_reduction_method=DEFAULT_COLOR_REDUCTION,
        crop=True,
        bitdepth=None,
        palette_name="pebble64",
    ):
        self.palette_name = palette_name
        self.version = 1
        self.path = path
        self.name, _ = os.path.splitext(os.path.basename(path))
        self.color_map = color_map
        self.palette = None  # only used in color mode for <=16 colors
        self.bitdepth = bitdepth  # number of bits per pixel, 0 for legacy b&w
        if bitmap_format == FORMAT_BW:
            self.bitdepth = 0
        self.bitmap_format = bitmap_format
        self.color_reduction_method = color_reduction_method
        width, height, pixels, metadata = png.Reader(filename=path).asRGBA8()

        # convert planar boxed row flat pixel to 2d array of (R, G, B, A)
        self._im_pixels = []
        for row in pixels:
            row_list = []
            for r, g, b, a in grouper(row, 4):
                row_list.append((r, g, b, a))
            self._im_pixels.append(row_list)

        self._im_size = (width, height)
        self._set_bbox(crop)

    def _set_bbox(self, crop=True):
        left, top = (0, 0)
        right, bottom = self._im_size

        if crop:
            alphas = [[p[3] for p in row] for row in self._im_pixels]
            alphas_transposed = list(zip(*alphas))
            for row in alphas:
                if any(row):
                    break
                top += 1
            for row in reversed(alphas):
                if any(row):
                    break
                bottom -= 1
            for row in alphas_transposed:
                if any(row):
                    break
                left += 1
            for row in list(zip(*alphas_transposed))[::-1]:
                if any(row):
                    break
                right -= 1

        self.x = left
        self.y = top
        self.w = right - left
        self.h = bottom - top

    def row_size_bytes(self):
        """
        Return the length of the bitmap's row in bytes.

        On b/w, row lengths are rounded up to the nearest word, padding up to
        3 empty bytes per row.
        On color, row lengths are rounded up to the nearest byte
        """
        if self.bitmap_format == FORMAT_COLOR_RAW:
            return self.w
        elif self.bitmap_format == FORMAT_COLOR:
            # adds (8 / bitdepth) - 1 to round up (ceil) to the next nearest byte
            return (self.w + ((8 // self.bitdepth) - 1)) // (8 // self.bitdepth)
        else:
            row_size_padded_words = (self.w + 31) // 32
            return row_size_padded_words * 4

    def info_flags(self):
        """Returns the type and version of bitmap."""
        format_value = bitdepth_dict[self.bitdepth]
        return self.version << 12 | format_value << 1

    def pbi_header(self):
        return struct.pack(
            "<HHhhhh",
            self.row_size_bytes(),
            self.info_flags(),
            self.x,
            self.y,
            self.w,
            self.h,
        )

    def image_bits_bw(self):
        """
        Return a raw b/w bitmap capable of being rendered using Pebble's bitblt graphics routines.

        The returned bitmap will always be y * row_size_bytes large.
        """

        def get_monochrome_value_for_pixel(pixel):
            if pixel[3] < 127:
                return self.color_map["transparent"]
            if ((pixel[0] + pixel[1] + pixel[2]) / 3) < 127:
                return self.color_map["black"]
            return self.color_map["white"]

        def pack_pixels_to_bitblt_word(pixels, x_offset, x_max):
            word = 0
            for column in range(32):
                x = x_offset + column
                if x < x_max:
                    pixel = pixels[x]
                    word |= get_monochrome_value_for_pixel(pixel) << (column)

            return struct.pack("<I", word)

        src_pixels = self._im_pixels
        out_pixels = []
        row_size_words = self.row_size_bytes() // 4

        for row in range(self.y, self.y + self.h):
            x_max = self._im_size[0]
            for column_word in range(row_size_words):
                x_offset = self.x + column_word * 32
                out_pixels.append(
                    pack_pixels_to_bitblt_word(src_pixels[row], x_offset, x_max)
                )

        return b"".join(out_pixels)

    def image_bits_color(self):
        """
        Return a raw color bitmap capable of being rendered using Pebble's bitblt graphics routines.
        """

        if self.bitmap_format == FORMAT_COLOR_RAW:
            self.bitdepth = 8  # forced to 8-bit depth for color_raw, no palette
        else:
            self.generate_palette()

        assert self.bitdepth is not None
        out_pixels = []
        for row in range(self.y, self.y + self.h):
            packed_count = 0
            packed_value = 0
            for column in range(self.x, self.x + self.w):
                pixel = self._im_pixels[row][column]
                r, g, b, a = (pixel[i] for i in range(4))

                # convert RGBA 32-bit image colors to pebble color table
                fn = get_reduction_func(self.palette_name, self.color_reduction_method)
                r, g, b, a = fn(r, g, b, a)
                if a == 0:
                    # clear values in transparent pixels
                    r, g, b = (0, 0, 0)

                # convert colors to ARGB8 format
                argb8 = rgba32_triplet_to_argb8(r, g, b, a)

                if self.bitdepth == 8:
                    out_pixels.append(struct.pack("B", argb8))
                else:
                    # all palettized color bitdepths (1, 2, 4)
                    # look up the color index in the palette
                    color_index = self.palette.index(argb8)
                    # shift and store the color index in a packed value
                    packed_count = (
                        packed_count + 1
                    )  # pre-increment for calculation below
                    packed_value = packed_value | (
                        color_index
                        << (self.bitdepth * (8 // self.bitdepth - (packed_count)))
                    )

                    if packed_count == 8 // self.bitdepth:
                        out_pixels.append(struct.pack("B", packed_value))
                        packed_count = 0
                        packed_value = 0

            # write out the last non-byte-aligned set for the row (ie. byte-align rows)
            if packed_count:
                out_pixels.append(struct.pack("B", packed_value))

        return b"".join(out_pixels)

    def image_bits(self):
        if self.bitmap_format == FORMAT_COLOR or self.bitmap_format == FORMAT_COLOR_RAW:
            return self.image_bits_color()
        else:
            return self.image_bits_bw()

    def header(self):
        f = StringIO()
        f.write("// GBitmap + pixel data generated by bitmapgen.py:\n\n")
        bytes = self.image_bits()
        bytes_var_name = f"s_{self.name}_pixels"
        generate_c_byte_array.write(f, bytes, bytes_var_name)
        f.write(f"static const GBitmap s_{self.name}_bitmap = {{\n")
        f.write(f"  .addr = (void*) &{bytes_var_name},\n")
        f.write(f"  .row_size_bytes = {self.row_size_bytes()},\n")
        f.write("  .info_flags = 0x%02x,\n" % self.info_flags())
        f.write("  .bounds = {\n")
        f.write(f"    .origin = {{ .x = {self.x}, .y = {self.y} }},\n")
        f.write(f"    .size = {{ .w = {self.w}, .h = {self.h} }},\n")
        f.write("  },\n")
        f.write("};\n\n")
        return f.getvalue()

    def convert_to_h(self, header_file=None):
        to_file = (
            header_file if header_file else (os.path.splitext(self.path)[0] + ".h")
        )
        with open(to_file, "w") as f:
            f.write(self.header())
        return to_file

    def convert_to_pbi(self):
        image_data = self.image_bits()  # compute before generating header

        pbi_bits = self.pbi_header()
        pbi_bits += image_data
        if self.palette and self.bitdepth < 8:
            # write out palette, padded to the bitdepth
            for i in range(2**self.bitdepth):
                value = 0
                if i < len(self.palette):
                    value = self.palette[i]
                pbi_bits += struct.pack("B", value)

        return pbi_bits

    def convert_to_pbi_file(self, pbi_file=None):
        to_file = pbi_file if pbi_file else (os.path.splitext(self.path)[0] + ".pbi")

        with open(to_file, "wb") as f:
            f.write(self.convert_to_pbi())

        return to_file

    def generate_palette(self):
        self.palette = []
        for row in range(self.y, self.y + self.h):
            for column in range(self.x, self.x + self.w):
                pixel = self._im_pixels[row][column]
                r, g, b, a = (pixel[i] for i in range(4))

                # convert RGBA 32-bit image colors to pebble color table
                fn = get_reduction_func(self.palette_name, self.color_reduction_method)
                r, g, b, a = fn(r, g, b, a)

                if a == 0:
                    # clear values in transparent pixels
                    r, g, b = (0, 0, 0)

                # store color value as ARGB8 entry in the palette
                self.palette.append(rgba32_triplet_to_argb8(r, g, b, a))

        # remove duplicate colors
        self.palette = list(set(self.palette))

        # get the bitdepth for the number of colors
        min_bitdepth = num_colors_to_bitdepth(len(self.palette))
        if self.bitdepth is None:
            self.bitdepth = min_bitdepth
        if self.bitdepth < min_bitdepth:
            raise Exception(
                f"Required bitdepth {self.bitdepth} is lower than required depth {min_bitdepth}."
            )

        self.palette.extend([0] * (self.bitdepth - len(self.palette)))


def cmd_pbi(args):
    pb = PebbleBitmap(
        args.input_png,
        bitmap_format=args.format,
        color_reduction_method=args.color_reduction_method,
        crop=not args.disable_crop,
    )
    pb.convert_to_pbi_file(args.output_pbi)


def cmd_header(args):
    pb = PebbleBitmap(
        args.input_png,
        bitmap_format=args.format,
        color_reduction_method=args.color_reduction_method,
        crop=not args.disable_crop,
    )
    print(pb.header())


def cmd_white_trans_pbi(args):
    pb = PebbleBitmap(args.input_png, WHITE_COLOR_MAP, crop=not args.disable_crop)
    pb.convert_to_pbi_file(args.output_pbi)


def cmd_black_trans_pbi(args):
    pb = PebbleBitmap(args.input_png, BLACK_COLOR_MAP, crop=not args.disable_crop)
    pb.convert_to_pbi_file(args.output_pbi)


def process_all_bitmaps():
    directory = "bitmaps"
    paths = []
    for _, _, filenames in os.walk(directory):
        for filename in filenames:
            if os.path.splitext(filename)[1] == ".png":
                paths.append(os.path.join(directory, filename))

    header_paths = []
    for path in paths:
        b = PebbleBitmap(path)
        b.convert_to_pbi_file()
        to_file = b.convert_to_h()
        header_paths.append(os.path.basename(to_file))

    f = open(os.path.join(directory, "bitmaps.h"), "w")
    print("#pragma once", file=f)
    for h in header_paths:
        print(f'#include "{h}"', file=f)
    f.close()


def grouper(iterable, n, fillvalue=None):
    from itertools import zip_longest

    args = [iter(iterable)] * n
    return zip_longest(fillvalue=fillvalue, *args)


def process_cmd_line_args():
    parser = argparse.ArgumentParser(
        description="Generate pebble-usable files from png images"
    )

    parser_parent = argparse.ArgumentParser(add_help=False)
    parser_parent.add_argument(
        "--disable_crop",
        required=False,
        action="store_true",
        help="Disable transparent region cropping for PBI output",
    )
    parser_parent.add_argument(
        "--color_reduction_method",
        metavar="method",
        required=False,
        nargs=1,
        default=NEAREST,
        choices=COLOR_REDUCTION_CHOICES,
        help="Method used to convert colors to Pebble's color palette, "
        f"options are [{NEAREST}, {TRUNCATE}]",
    )

    subparsers = parser.add_subparsers(help="commands", dest="which")

    bitmap_format = {
        "dest": "format",
        "metavar": "BITMAP_FORMAT",
        "choices": FORMAT_CHOICES,
        "nargs": "?",
        "default": DEFAULT_FORMAT,
        "help": "resulting GBitmap format",
    }
    input_png = {
        "dest": "input_png",
        "metavar": "INPUT_PNG",
        "help": "The png image to process",
    }
    output_pbi = {
        "dest": "output_pbi",
        "metavar": "OUTPUT_PBI",
        "help": "The pbi output file",
    }

    pbi_parser = subparsers.add_parser(
        "pbi", parents=[parser_parent], help="make a .pbi (pebble binary image) file"
    )

    for arg in [bitmap_format, input_png, output_pbi]:
        pbi_parser.add_argument(**arg)
    pbi_parser.set_defaults(func=cmd_pbi)

    h_parser = subparsers.add_parser(
        "header", parents=[parser_parent], help="make a .h file"
    )
    for arg in [bitmap_format, input_png]:
        h_parser.add_argument(**arg)
    h_parser.set_defaults(func=cmd_header)

    white_pbi_parser = subparsers.add_parser(
        "white_trans_pbi",
        parents=[parser_parent],
        help="make a .pbi (pebble binary image) file for a white transparency layer",
    )
    for arg in [input_png, output_pbi]:
        white_pbi_parser.add_argument(**arg)
    white_pbi_parser.set_defaults(func=cmd_white_trans_pbi)

    black_pbi_parser = subparsers.add_parser(
        "black_trans_pbi",
        parents=[parser_parent],
        help="make a .pbi (pebble binary image) file for a black transparency layer",
    )
    for arg in [input_png, output_pbi]:
        black_pbi_parser.add_argument(**arg)
    black_pbi_parser.set_defaults(func=cmd_black_trans_pbi)

    args = parser.parse_args()
    args.func(args)


def main():
    if len(sys.argv) < 2:
        # process everything in the  bitmaps folder
        process_all_bitmaps()
    else:
        # process an individual file
        process_cmd_line_args()


if __name__ == "__main__":
    main()
