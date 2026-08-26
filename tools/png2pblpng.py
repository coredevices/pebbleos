#!/usr/bin/env python
# SPDX-FileCopyrightText: 2024 Google LLC
# SPDX-License-Identifier: Apache-2.0


import itertools
from io import BytesIO

import pebble_image_routines
import png

# color reduction methods
TRUNCATE = "truncate"
NEAREST = "nearest"
COLOR_REDUCTION_CHOICES = [TRUNCATE, NEAREST]
SUPPORTED_PALETTES = ("pebble2", "pebble64")
DEFAULT_COLOR_REDUCTION = NEAREST


# Public APIs
def convert_png_to_pebble_png(
    input_filename,
    output_filename,
    palette_name,
    color_reduction_method=DEFAULT_COLOR_REDUCTION,
    bitdepth=None,
):
    """
    Convert a png to a pblpng and write it to output_filename
    """

    output_png_writer, image_data = _convert_png_to_pebble_png_writer(
        input_filename, palette_name, color_reduction_method, force_bitdepth=bitdepth
    )

    with open(output_filename, "wb") as output_file:
        output_png_writer.write_array(output_file, image_data)


def convert_png_to_pebble_png_bytes(
    input_filename,
    palette_name,
    color_reduction_method=DEFAULT_COLOR_REDUCTION,
    bitdepth=None,
):
    """
    Convert a png to a pblpng and return a string with the raw data
    """

    output_png, image_data = _convert_png_to_pebble_png_writer(
        input_filename, palette_name, color_reduction_method, force_bitdepth=bitdepth
    )

    output_str = BytesIO()
    output_png.write_array(output_str, image_data)

    return output_str.getvalue()


# Implementation
def _convert_png_to_pebble_png_writer(
    input_filename, palette_name, color_reduction_method, force_bitdepth=None
):
    input_png = png.Reader(filename=input_filename)

    # sbit breaks pypngs convert_rgb_to_rgba routine
    # and is unnecessary, as it is only an optional optimization
    # so disable it by loading the PNG pre-data and disabling sbit
    input_png.preamble()
    input_png.sbit = None

    # open as RGBA 32-bit (allows for simpler parsing cases)
    width, height, pixels, metadata = input_png.asRGBA8()

    # convert RGBA 32-bit boxed rows to list for output
    rgba32_list = grouper(itertools.chain.from_iterable(pixels), 4)

    color_reduction_func = pebble_image_routines.get_reduction_func(
        palette_name, color_reduction_method
    )
    is_grey, has_alpha, bitdepth, palette = get_palette_for_png(
        input_filename, palette_name, color_reduction_method
    )

    if force_bitdepth is not None:
        if bitdepth > force_bitdepth:
            raise Exception(
                f"Tried to force {force_bitdepth} bits; need at least {bitdepth}."
            )

        # If we're forcing a particular bitdepth, and it's not the one we were going
        # to use, skip the greyscale dance.
        if bitdepth != force_bitdepth:
            is_grey = False
        bitdepth = force_bitdepth

    transparent_grey = None
    # determine the grey value for tRNs transparency
    if is_grey and has_alpha:
        if bitdepth == 4:
            # 4 available shades of grey are occupied
            transparent_grey = 0xC  # bitdepth 4 supported value
        else:
            greyscale_list = [0, 255, 85, 170]  # in order of bitdepth required
            for lum in greyscale_list:
                # find the first unused greyscale value in terms of available bitdepth
                if (lum, lum, lum, 255) not in palette:
                    # transparent grey value for requested greyscale bitdepth
                    transparent_grey = lum >> (8 - bitdepth)
                    break

    # second pass of pixel data, converts rgba32 pixels to greyscale or palettized output
    image = []
    for r, g, b, a in rgba32_list:
        # operating on original pixel values, need to do the same color reduction
        # as when the palette was generated
        (r, g, b, a) = color_reduction_func(r, g, b, a)

        if is_grey:
            # convert red channel (as luminosity value) to a greyscale at bitdepth
            # if transparent, output the transparent_grey value for that bitdepth
            if a == 0:
                image.append(transparent_grey)
            else:
                image.append(r >> (8 - bitdepth))
        elif has_alpha:
            # append the palette index for output
            image.append(palette.index((r, g, b, a)))
        else:
            # append the palette index for output
            image.append(palette.index((r, g, b)))

    if is_grey:
        # remove the palette for greyscale output with writer
        palette = None

    output_png = png.Writer(
        width=width,
        height=height,
        compression=9,
        bitdepth=bitdepth,
        palette=palette,
        greyscale=is_grey,
        transparent=transparent_grey,
    )

    return (output_png, image)


def get_palette_for_png(input_filename, palette_name, color_reduction_method):
    input_png = png.Reader(filename=input_filename)

    # sbit breaks pypngs convert_rgb_to_rgba routine
    # and is unnecessary, as it is only an optional optimization
    # so disable it by loading the PNG pre-data and disabling sbit
    input_png.preamble()
    input_png.sbit = None

    # open as RGBA 32-bit (allows for simpler parsing cases)
    width, height, pixels, metadata = input_png.asRGBA8()

    palette = []  # rgba32 image palette
    is_grey = (
        True  # does the image only contain greyscale pixels (and only full or opaque)
    )
    has_alpha = False  # does the image contain alpha

    # iterators are one shot, so make a copy of just the iterator and not the data
    # to be able to parse the data twice (we do not modify the pixel data itself)
    # once to generate the palette
    # once to output the final pixel data as greyscale or palette indexes
    pixels, pixels2 = itertools.tee(pixels)

    # Figure out what color reduction algorithm we should be using.
    color_reduction_func = pebble_image_routines.get_reduction_func(
        palette_name, color_reduction_method
    )

    # convert RGBA 32-bit image colors to pebble color table
    for r, g, b, a in grouper(itertools.chain.from_iterable(pixels2), 4):
        (r, g, b, a) = color_reduction_func(r, g, b, a)

        if (r, g, b, a) not in palette:
            palette.append((r, g, b, a))
            # Check if image contains any transparent pixels
            if a != 0xFF:
                has_alpha = True
            # greyscale only if rgb is gray and opaque or fully transparent
            if is_grey and not (
                ((r == g == b) and a == 255) or (r, g, b, a) == (0, 0, 0, 0)
            ):
                is_grey = False

    # Calculate required bit depth

    # get the bitdepth for the number of colors
    bitdepth = pebble_image_routines.num_colors_to_bitdepth(len(palette))
    if is_grey:
        # for Greyscale, it is the required colors that set the bitdepth
        # so if image contains LightGray or DarkGray it requires bitdepth 2
        if (85, 85, 85, 255) in palette or (170, 170, 170, 255) in palette:
            # if palette contains all 4 greyscale and transparent, bump up bitdepth
            if (len(palette)) >= 5:
                grey_bitdepth = 4
            else:
                grey_bitdepth = 2
        else:
            # if palette contains black, white and transparent, bump up bitdepth
            if (len(palette)) >= 3:
                grey_bitdepth = 2
            else:
                grey_bitdepth = 1
        if grey_bitdepth > bitdepth:
            is_grey = False
        else:
            bitdepth = grey_bitdepth

    # update data for RGB output format
    if not has_alpha:
        # recreate the palette without an alpha channel to support RGB PNG
        palette = [(p_r, p_g, p_b) for p_r, p_g, p_b, p_a in palette]

    return is_grey, has_alpha, bitdepth, palette


def grouper(iterable, n, fillvalue=None):
    from itertools import zip_longest

    args = [iter(iterable)] * n
    return zip_longest(fillvalue=fillvalue, *args)


def get_ideal_palette(is_color=False):
    if is_color:
        return "pebble64"
    else:
        return "pebble2"


def main():
    import argparse

    parser = argparse.ArgumentParser(
        description="Convert PNG to 64-color palettized or grayscale PNG"
    )
    parser.add_argument("input_filename", type=str, help="png file to convert")
    parser.add_argument("output_filename", type=str, help="converted file output")
    parser.add_argument(
        "--palette",
        type=str,
        required=False,
        choices=SUPPORTED_PALETTES,
        default="pebble64",
        help="Specify the standard palette of the resulting png. Colors will be "
        "converted to this lower bit depth using the color_reduction_method "
        "arg.",
    )
    parser.add_argument(
        "--color_reduction_method",
        metavar="method",
        required=False,
        nargs=1,
        default=NEAREST,
        choices=COLOR_REDUCTION_CHOICES,
        help="Method used to convert colors to Pebble's color palette, "
        f"options are [{NEAREST}, {TRUNCATE}]",
    )
    args = parser.parse_args()

    convert_png_to_pebble_png(
        args.input_filename,
        args.output_filename,
        args.palette,
        args.color_reduction_method,
    )


if __name__ == "__main__":
    main()
