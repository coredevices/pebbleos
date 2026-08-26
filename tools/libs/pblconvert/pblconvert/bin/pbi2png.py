#!/usr/bin/env python

import sys
from ctypes import *

from PIL import Image

format_dict = {
    "GBitmapFormat1Bit": 0,
    "GBitmapFormat8Bit": 1,
    "GBitmapFormat1BitPalette": 2,
    "GBitmapFormat2BitPalette": 3,
    "GBitmapFormat4BitPalette": 4,
}


# NOTE: If this changes, please update the GBitmapDump gdb command.
class pbi_struct(Structure):
    _fields_ = [
        ("stride", c_uint16),
        ("info", c_uint16),
        ("bounds_x", c_uint16),
        ("bounds_y", c_uint16),
        ("bounds_w", c_uint16),
        ("bounds_h", c_uint16),
    ]


def flip_byte(abyte):
    return int(f"{abyte:08b}"[::-1], 2)


# converts from argb8 (2-bits per color channel) to RGBA32 (byte per channel)
def argb8_to_rgba32(argb8):
    return (
        ((argb8 >> 4) & 0x3) * 85,  # R
        ((argb8 >> 2) & 0x3) * 85,  # G
        ((argb8) & 0x3) * 85,  # B
        ((argb8 >> 6) & 0x3) * 85,
    )  # A


def pbi_format(info):
    return (info & 0xE) >> 1


def pbi_bitdepth(fmt):
    bitdepth_list = [1, 8, 1, 2, 4]
    return bitdepth_list[fmt]


def pbi_is_palettized(fmt):
    return fmt >= format_dict["GBitmapFormat1BitPalette"]


def palette_size(fmt):
    return 2 ** pbi_bitdepth(fmt)


def pbi_to_png(pbi, pixel_bytearray):
    gbitmap_version = (pbi.info >> 12) & 0x0F
    gbitmap_format = pbi_format(pbi.info)
    # if version is 2 and format is 0x01 (GBitmapFormat8Bit)
    if gbitmap_version == 1 and gbitmap_format == format_dict["GBitmapFormat8Bit"]:
        print("8-bit ARGB color image")
        pixel_rgba_array = bytearray()
        for idx, abyte in enumerate(pixel_bytearray):
            argb8 = abyte
            pixel_rgba_array.append(((argb8 >> 4) & 0x3) * 85)  # r
            pixel_rgba_array.append(((argb8 >> 2) & 0x3) * 85)  # g
            pixel_rgba_array.append(((argb8 >> 0) & 0x3) * 85)  # b
            pixel_rgba_array.append(((argb8 >> 6) & 0x3) * 85)  # a

        png = Image.frombuffer(
            "RGBA",
            (pbi.bounds_w, pbi.bounds_h),
            buffer(pixel_rgba_array),
            "raw",
            "RGBA",
            pbi.stride * 4,
            1,
        )

    elif gbitmap_version == 1 and pbi_is_palettized(gbitmap_format):
        bitdepth = pbi_bitdepth(gbitmap_format)
        print(f"{bitdepth}-bit palettized color image")

        # Create palette colors in format R, G, B, A
        palette = []
        palette_offset = pbi.stride * pbi.bounds_h
        for argb8 in pixel_bytearray[palette_offset:]:
            palette.append(argb8_to_rgba32(argb8))

        pixels = []
        # go through the image data byte by byte, and handle
        # converting the depth-packed indexes for the palette to an unpacked list
        idx = 0  # index of actual packed values including padded values
        for pxl8 in pixel_bytearray[:palette_offset]:
            for i in xrange(0, 8 / bitdepth):
                # only append actual pixels, ignoring padding pixels
                # which is the difference between the width and the stride
                if idx % (pbi.stride * (8 / bitdepth)) < pbi.bounds_w:
                    pixels.append(
                        
                            (pxl8 >> (bitdepth * (8 / bitdepth - (i + 1))))
                            & ~(~0 << bitdepth)
                        
                    )
                idx = idx + 1

        # Manually convert from paletted to RGBA
        # as PIL doesn't seem to handle palette with alpha
        rgba_pixels = []
        for pal_pxl in pixels:
            rgba_pixels.append(palette[pal_pxl])

        png = Image.new("RGBA", (pbi.bounds_w, pbi.bounds_h))
        png.putdata(rgba_pixels)

    # legacy 1-bit format
    elif gbitmap_version == 0 or (
        gbitmap_version == 1 and gbitmap_format == format_dict["GBitmapFormat1Bit"]
    ):
        print("1-bit b&w image")
        # pbi has bits in bytes reversed, so flip here
        for idx, abyte in enumerate(pixel_bytearray):
            pixel_bytearray[idx] = flip_byte(pixel_bytearray[idx])

        png = Image.frombuffer(
            "1",
            (pbi.bounds_w, pbi.bounds_h),
            buffer(pixel_bytearray),
            "raw",
            "1",
            pbi.stride,
            1,
        )
    else:
        print("Bad PBI")
        png = None

    return png


def main():
    # arguments, print an example of correct usage.
    if len(sys.argv) - 1 != 2:
        print("********************")
        print("Usage suggestion:")
        print("python " + sys.argv[0] + " <in_image.pbi> <out_image.png>")
        print("********************")
        exit()

    input_filename = sys.argv[1]
    output_filename = sys.argv[2]

    print("Converting PBI to PNG...")
    pbi = pbi_struct()
    pixel_bytearray = bytearray()
    with open(input_filename, "rb") as afile:
        afile.readinto(pbi)
        print("x:%d y:%d" % (pbi.bounds_x, pbi.bounds_y))
        print("Width:%d Height:%d" % (pbi.bounds_w, pbi.bounds_h))
        print("row stride:%d" % (pbi.stride))
        pixel_bytearray = bytearray(afile.read())

    png = pbi_to_png(pbi, pixel_bytearray)
    png.save(output_filename)


if __name__ == "__main__":
    main()
