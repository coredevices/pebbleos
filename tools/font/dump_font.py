#!/usr/bin/env python
# SPDX-FileCopyrightText: 2024 Google LLC
# SPDX-License-Identifier: Apache-2.0


import argparse
import array
import itertools
import pprint
import struct
import sys

from PIL import Image

FONT_VERSION_1 = 1
FONT_VERSION_2 = 2
FONT_VERSION_3 = 3
# Features
FEATURE_OFFSET_16 = 0x01
FEATURE_RLE4 = 0x02
GLYPH_MD_STRUCT = "BBbbb"


def dec_and_hex(i):
    return f"0x{i:x} ({i:d})"


def grouper(n, iterable, fillvalue=None):
    """grouper(3, 'ABCDEFG', 'x') --> ABC DEF Gxx"""
    args = [iter(iterable)] * n
    return itertools.zip_longest(*args, fillvalue=fillvalue)


def get_glyph(features, tbl, offset_bytes):
    bitmap_offset_bytes = offset_bytes + struct.calcsize(GLYPH_MD_STRUCT)
    header = tbl[offset_bytes:bitmap_offset_bytes]
    (width, height, left, top, adv) = struct.unpack(GLYPH_MD_STRUCT, header)

    if features & FEATURE_RLE4:
        bitmap_length_bytes = (height + 1) // 2  # RLE4 stores 2 rle_units per byte
    else:
        bitmap_length_bytes = ((height * width) + 7) // 8

    bitmap_length_bytes_word_aligned = ((bitmap_length_bytes + 3) // 4) * 4
    data = tbl[
        bitmap_offset_bytes : bitmap_offset_bytes + bitmap_length_bytes_word_aligned
    ]
    return {
        "offset_bytes": dec_and_hex(offset_bytes),
        "top": top,
        "left": left,
        "height": height,
        "width": width,
        "advance": adv,
        "bitmap": data,
    }, header


def hasher(codepoint, table_size):
    return codepoint % table_size


def print_hash_table(hash_table):
    print("index\tcount\toffset")
    for idx, sz, off in hash_table:
        print(f"{idx:d}\t{sz:d}\t{dec_and_hex(off)!s}")


def print_glyph(features, glyph_table, offset, raw, show_image):

    def decompress_glyph_RLE4(rle_stream, num_units, width):
        bitmap = []

        for b in array.array("B", rle_stream):
            # There are two RLE4 units per byte. LSNibble first.
            for i in range(2):
                # Handle the padding by skipping whatever remains
                if num_units == 0:
                    break
                num_units -= 1

                # Each unit is <xyyy> where x is the symbol and yyy is (length - 1)
                length = (b & 0x7) + 1
                symbol = 1 if ((b >> 3) & 1) == 1 else 0
                bitmap.extend([symbol] * length)
                b >>= 4

        # Correct the height, now that we know the decompressed size
        height = len(bitmap) / width

        return bitmap, height

    def glyph_bitmap_to_bitlist(g):
        height = g["height"]

        if not g["bitmap"]:
            return None, height

        if features & FEATURE_RLE4:
            b, height = decompress_glyph_RLE4(g["bitmap"], height, g["width"])
        else:
            b = []
            for w in array.array("I", g["bitmap"]):
                b.extend((w & (1 << bit)) != 0 for bit in range(32))

        return b, height

    def draw_glyph_image(bitlist, width, height):
        img = Image.new("RGB", (width, height), "black")
        pixels = img.load()
        for i in range(img.size[0]):
            for j in range(img.size[1]):
                pixels[i, j] = (0, 0, 0) if bitlist[j * width + i] else (255, 255, 255)
        img.show()

    def draw_glyph_ascii(bitlist, width, height):
        for y in range(height):
            for x in range(width):
                if bitlist[y * width + x]:
                    sys.stdout.write("X")
                else:
                    sys.stdout.write(" ")
            print()

    def draw_glyph_raw(header, bitlist, width, height):
        # Header:
        for byte in header:
            print(f"{ord(byte):02x}", end=" ")
        print(" - ", end=" ")
        # Repack the glyph data. This is required because the glyph may have been compressed
        for byte in range(height * width / 8):
            w = 0
            for bit in range(8):
                if bitlist[byte * 8 + bit]:
                    w |= 1 << bit
            print(f"{w:02x}", end=" ")
        print()

    g, header = get_glyph(features, glyph_table, offset)

    # FEATURE_RLE4 uses the height field for # RLE Units!
    bitlist, height = glyph_bitmap_to_bitlist(g)

    if raw:
        offset + struct.calcsize(GLYPH_MD_STRUCT)
        draw_glyph_raw(header, bitlist, g["width"], height)
    else:
        output = []
        output.append("offset bytes: {}".format(g["offset_bytes"]))
        output.append("top: {}".format(g["top"]))
        output.append("left: {}".format(g["left"]))
        output.append(f"height: {height}")
        output.append("width: {}".format(g["width"]))
        output.append("advance: {}".format(g["advance"]))
        output.append("bitmap:")
        print("\n".join(output))
        print()

        if bitlist:
            if show_image:
                draw_glyph_image(bitlist, g["width"], height)
            else:
                draw_glyph_ascii(bitlist, g["width"], height)


# Allow extended codepoint encoding for 'narrow Python builds'
def my_unichr(i):
    try:
        return chr(i)
    except ValueError:
        return struct.pack("i", i).decode("utf-32")


# Attempt to print unicode characters. Do the best we can when redirecting output.
# (See PYTHONIOENCODING)
def print_glyph_header(codepoint, offset, raw=False):
    if raw:
        print(f"{codepoint:08X}:", end=" ")
    else:
        print()
        print(
            f"{dec_and_hex(codepoint)}\t({my_unichr(codepoint)})\t{dec_and_hex(offset)}".encode("utf-8", "replace")
        )
        print()


def main(pfo_path, show_hash_table, offset_table, glyph, all_glyphs, show_image, raw):
    with open(pfo_path, "rb") as f:
        font = f.read()

    # Assume version 3 to start
    version = 3
    font_md_format = ["", "<BBHH", "<BBHHBB", "<BBHHBBBB"]
    font_md_size = struct.calcsize(font_md_format[version])
    (
        version,
        max_height,
        num_glyphs,
        wildcard_cp,
        table_size,
        cp_bytes,
        _struct_size,
        features,
    ) = struct.unpack(font_md_format[version], font[:font_md_size])
    if version == 3:
        pass
    elif version == 2:
        # Set the defaults
        font_md_size = struct.calcsize(font_md_format[version])
        features = 0
    else:
        raise RuntimeError(f"Error: Unexpected font file version {version}")

    # Build up the offset entry struct
    offset_table_format = "<"
    offset_table_format += "L" if cp_bytes == 4 else "H"
    offset_table_format += "H" if features & FEATURE_OFFSET_16 else "L"
    offset_entry_size = struct.calcsize(offset_table_format)

    hash_entry_format = "<BBH"
    hash_entry_size = struct.calcsize(hash_entry_format)

    def hash_iterator(tbl, num):
        for i in range(0, num * hash_entry_size, hash_entry_size):
            yield struct.unpack(hash_entry_format, tbl[i : i + hash_entry_size])

    def offset_iterator(tbl, num):
        for i in range(0, num * offset_entry_size, offset_entry_size):
            yield struct.unpack(offset_table_format, tbl[i : i + offset_entry_size])

    hash_table = [
        (a, b, c) for (a, b, c) in hash_iterator(font[font_md_size:], table_size)
    ]
    offset_tables_start = font_md_size + hash_entry_size * table_size
    glyph_table_start = offset_tables_start + offset_entry_size * (num_glyphs)
    glyph_table = font[glyph_table_start:]

    if not raw:
        print("Font info")
        pprint.pprint(
            {
                "version": version,
                "max_height": max_height,
                "num_glyphs": num_glyphs,
                "offset_table_size": num_glyphs * offset_entry_size,
                "glyph_table_start": dec_and_hex(glyph_table_start),
                "wildcard_cp": dec_and_hex(wildcard_cp),
                "codepoint bytes": cp_bytes,
                "hash_table_size": table_size,
                "font_header_size": font_md_size,
                "features": features,
                "features - offset size": 16 if (features & FEATURE_OFFSET_16) else 32,
                "features - RLE4": bool(features & FEATURE_RLE4),
            }
        )

        print()
        print(f"Hash Table start:   {font_md_size}")
        print(f"Offset Table start: {offset_tables_start}")
        print(f"Glyph Table start:  {glyph_table_start}")
        print("--------------------------")

    if all_glyphs:
        for _, sz, off in hash_table:
            off_table = dict(
                [x for x in offset_iterator(font[(offset_tables_start + off) :], sz)]
            )
            for k, v in sorted(off_table.items()):
                print_glyph_header(k, v, raw)
                print_glyph(features, glyph_table, v, raw, False)
                if not raw:
                    print()
                    print()
        return

    if show_hash_table:
        print()
        print_hash_table(hash_table)

    if offset_table:
        offset_table = int(offset_table)
        _, sz, off = hash_table[offset_table]
        if not raw:
            print(
                f"Offset Table {offset_table} offset: {offset_tables_start + off}"
            )
        off_table = dict(
            [x for x in offset_iterator(font[(offset_tables_start + off) :], sz)]
        )
        for k, v in sorted(off_table.items()):
            print_glyph_header(k, v, raw)
            print_glyph(features, glyph_table, v, raw, show_image)
            if not raw:
                print()
                print()

    if glyph:
        codepoint = int(glyph, 16)
        glyph_hash = hasher(codepoint, table_size)
        _, sz, off = hash_table[glyph_hash]
        off_table = dict(
            [x for x in offset_iterator(font[(offset_tables_start + off) :], sz)]
        )
        glyph_off = 0
        for cp, off in list(off_table.items()):
            if cp == codepoint:
                glyph_off = off_table[codepoint]
                break
        else:
            print(f"{codepoint} not in font")
            return
        print_glyph_header(codepoint, glyph_off, raw)
        print_glyph(features, glyph_table, glyph_off, raw, show_image)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-o", "--offsets", help="print hash table")
    parser.add_argument("-t", "--hashes", action="store_true", help="print hash table")
    parser.add_argument("-g", "--glyph", help="draw glyph in external image viewer")
    parser.add_argument("-G", "--ascii_glyph", help="draw glyph in ASCII")
    parser.add_argument(
        "-a", "--all_glyphs", action="store_true", help="print all glyphs"
    )
    parser.add_argument("-r", "--raw", action="store_true")
    parser.add_argument("pebble_font")
    args = parser.parse_args()

    codepoint = args.ascii_glyph if args.ascii_glyph else args.glyph

    main(
        args.pebble_font,
        args.hashes,
        args.offsets,
        codepoint,
        args.all_glyphs,
        args.ascii_glyph is None,
        args.raw,
    )
