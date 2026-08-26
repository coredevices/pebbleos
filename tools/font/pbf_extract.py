#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2024 Google LLC
# SPDX-License-Identifier: Apache-2.0

"""
Extract glyphs from a PBF font into individual PNG images and a JSON manifest.
The manifest can be edited to add/modify glyphs, then repacked with pbf_repack.py.

Usage:
    python pbf_extract.py LECO_42_NUMBERS.pbf -o leco42/
"""

import argparse
import array
import json
import os
import struct

from PIL import Image

# Font version constants
FONT_VERSION_2 = 2
FONT_VERSION_3 = 3

# Feature flags
FEATURE_OFFSET_16 = 0x01
FEATURE_RLE4 = 0x02

GLYPH_MD_STRUCT = "BBbbb"


def decompress_rle4(rle_stream, num_units, width):
    """Decompress RLE4 encoded glyph bitmap data."""
    bitmap = []
    units_remaining = num_units

    for b in array.array("B", rle_stream):
        for _ in range(2):
            if units_remaining == 0:
                break
            units_remaining -= 1
            length = (b & 0x7) + 1
            symbol = 1 if ((b >> 3) & 1) == 1 else 0
            bitmap.extend([symbol] * length)
            b >>= 4

    height = len(bitmap) // width if width > 0 else 0
    return bitmap, height


def extract_pbf(pbf_path, output_dir):
    """Extract all glyphs from a PBF file."""
    with open(pbf_path, "rb") as f:
        font_data = f.read()

    # Parse header (assume v3, then adjust)
    font_md_format_v3 = "<BBHHBBBB"
    font_md_format_v2 = "<BBHHBB"

    font_md_size = struct.calcsize(font_md_format_v3)
    header = struct.unpack(font_md_format_v3, font_data[:font_md_size])
    (
        version,
        max_height,
        num_glyphs,
        wildcard_cp,
        table_size,
        cp_bytes,
        struct_size,
        features,
    ) = header

    if version == FONT_VERSION_3:
        pass
    elif version == FONT_VERSION_2:
        font_md_size = struct.calcsize(font_md_format_v2)
        (version, max_height, num_glyphs, wildcard_cp, table_size, cp_bytes) = (
            struct.unpack(font_md_format_v2, font_data[:font_md_size])
        )
        features = 0
    else:
        raise ValueError(f"Unsupported font version: {version}")

    # Build offset entry format
    offset_table_format = "<"
    offset_table_format += "L" if cp_bytes == 4 else "H"
    offset_table_format += "H" if features & FEATURE_OFFSET_16 else "L"
    offset_entry_size = struct.calcsize(offset_table_format)

    hash_entry_format = "<BBH"
    hash_entry_size = struct.calcsize(hash_entry_format)

    # Parse hash table
    hash_table = []
    hash_table_start = font_md_size
    for i in range(table_size):
        entry_start = hash_table_start + i * hash_entry_size
        entry = struct.unpack(
            hash_entry_format, font_data[entry_start : entry_start + hash_entry_size]
        )
        hash_table.append(entry)

    offset_tables_start = font_md_size + hash_entry_size * table_size
    glyph_table_start = offset_tables_start + offset_entry_size * num_glyphs
    glyph_table = font_data[glyph_table_start:]

    # Create output directory
    os.makedirs(output_dir, exist_ok=True)
    glyphs_dir = os.path.join(output_dir, "glyphs")
    os.makedirs(glyphs_dir, exist_ok=True)

    # Extract all glyphs
    glyphs = []
    for _, count, offset in hash_table:
        for i in range(count):
            entry_start = offset_tables_start + offset + i * offset_entry_size
            codepoint, glyph_offset = struct.unpack(
                offset_table_format,
                font_data[entry_start : entry_start + offset_entry_size],
            )

            # Read glyph header
            bitmap_offset_bytes = glyph_offset + struct.calcsize(GLYPH_MD_STRUCT)
            header_data = glyph_table[glyph_offset:bitmap_offset_bytes]

            if len(header_data) < struct.calcsize(GLYPH_MD_STRUCT):
                continue

            (width, height_or_rle, left, top, advance) = struct.unpack(
                GLYPH_MD_STRUCT, header_data
            )

            # Read bitmap data
            if features & FEATURE_RLE4:
                bitmap_length = (height_or_rle + 1) // 2
            else:
                bitmap_length = ((height_or_rle * width) + 7) // 8

            bitmap_length_aligned = ((bitmap_length + 3) // 4) * 4
            bitmap_data = glyph_table[
                bitmap_offset_bytes : bitmap_offset_bytes + bitmap_length_aligned
            ]

            # Convert to bitlist
            if width == 0 or (height_or_rle == 0 and not (features & FEATURE_RLE4)):
                bitlist = []
                height = 0
            elif features & FEATURE_RLE4:
                bitlist, height = decompress_rle4(bitmap_data, height_or_rle, width)
            else:
                bitlist = []
                for w in array.array("I", bitmap_data):
                    bitlist.extend((w & (1 << bit)) != 0 for bit in range(32))
                height = height_or_rle

            # Create and save image
            filename = f"U+{codepoint:04X}.png"
            filepath = os.path.join(glyphs_dir, filename)

            if width > 0 and height > 0 and bitlist:
                # PBF format: 1 = glyph (drawn as text_color), 0 = background.
                # PIL mode '1': 0 = black, 255 = white. Write glyph bits as black.
                img = Image.new("1", (width, height), 1)  # white background
                pixels = img.load()
                for y in range(height):
                    for x in range(width):
                        idx = y * width + x
                        if idx < len(bitlist) and bitlist[idx]:
                            pixels[x, y] = 0  # black glyph pixel
                img.save(filepath)

            glyphs.append(
                {
                    "codepoint": codepoint,
                    "char": chr(codepoint) if 0x20 <= codepoint <= 0x10FFFF else None,
                    "file": f"glyphs/{filename}",
                    "width": width,
                    "height": height,
                    "left_offset": left,
                    "top_offset": top,
                    "advance": advance,
                }
            )

    # Sort glyphs by codepoint
    glyphs.sort(key=lambda g: g["codepoint"])

    # Build manifest
    manifest = {
        "source": os.path.basename(pbf_path),
        "version": version,
        "max_height": max_height,
        "wildcard_codepoint": wildcard_cp,
        "hash_table_size": table_size,
        "codepoint_bytes": cp_bytes,
        "features": {
            "offset_16": bool(features & FEATURE_OFFSET_16),
            "rle4": bool(features & FEATURE_RLE4),
        },
        "glyphs": glyphs,
    }

    manifest_path = os.path.join(output_dir, "font.json")
    with open(manifest_path, "w") as f:
        json.dump(manifest, f, indent=2)

    print(f"Extracted {len(glyphs)} glyphs to {output_dir}/")
    print(f"Manifest: {manifest_path}")
    print(f"Images:   {glyphs_dir}/")
    print()
    print("To add a new glyph:")
    print("  1. Create a 1-bit PNG in glyphs/ (white=transparent, black=glyph)")
    print("  2. Add an entry to font.json with codepoint, file, and metrics")
    print("  3. Run: python pbf_repack.py font.json -o output.pbf")

    return manifest


def main():
    parser = argparse.ArgumentParser(description="Extract PBF font to editable format")
    parser.add_argument("pbf_file", help="Input PBF font file")
    parser.add_argument("-o", "--output", required=True, help="Output directory")
    args = parser.parse_args()

    if not os.path.exists(args.pbf_file):
        print(f"Error: File not found: {args.pbf_file}")
        return 1

    extract_pbf(args.pbf_file, args.output)
    return 0


if __name__ == "__main__":
    exit(main())
