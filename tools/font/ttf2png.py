#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0
"""
Render glyphs from a TTF font to 1-bit PNG images with a JSON manifest.
"""

import argparse
import json
import os
import sys

import freetype
from PIL import Image


def parse_glyph_list(glyph_str):
    """Parse a glyph list string into a sorted list of codepoints.

    Accepts:
      - Literal characters: "0123456789"
      - Unicode escapes: "U+0030-U+0039"
      - Ranges: "a-z"
      - Mix of all the above, comma or space separated
    """
    codepoints = set()
    i = 0
    while i < len(glyph_str):
        # Skip separators
        if glyph_str[i] in (",", " "):
            i += 1
            continue

        # U+XXXX notation
        if glyph_str[i : i + 2] == "U+" or glyph_str[i : i + 2] == "u+":
            hex_start = i + 2
            hex_end = hex_start
            while (
                hex_end < len(glyph_str)
                and glyph_str[hex_end] in "0123456789abcdefABCDEF"
            ):
                hex_end += 1
            cp_start = int(glyph_str[hex_start:hex_end], 16)

            # Check for range: U+0030-U+0039
            if hex_end < len(glyph_str) and glyph_str[hex_end] == "-":
                range_start = hex_end + 1
                if (
                    range_start < len(glyph_str)
                    and glyph_str[range_start : range_start + 2].lower() == "u+"
                ):
                    range_start += 2
                hex_end2 = range_start
                while (
                    hex_end2 < len(glyph_str)
                    and glyph_str[hex_end2] in "0123456789abcdefABCDEF"
                ):
                    hex_end2 += 1
                cp_end = int(glyph_str[range_start:hex_end2], 16)
                for cp in range(cp_start, cp_end + 1):
                    codepoints.add(cp)
                i = hex_end2
            else:
                codepoints.add(cp_start)
                i = hex_end
            continue

        # Literal character, possibly with range: a-z
        cp = ord(glyph_str[i])
        if (
            i + 2 < len(glyph_str)
            and glyph_str[i + 1] == "-"
            and glyph_str[i + 2] not in (",", " ")
        ):
            cp_end = ord(glyph_str[i + 2])
            for c in range(cp, cp_end + 1):
                codepoints.add(c)
            i += 3
        else:
            codepoints.add(cp)
            i += 1

    return sorted(codepoints)


def bitmap_to_image(bitmap, width, rows):
    """Convert a FreeType mono bitmap to a 1-bit PIL Image.

    PIL mode '1': 0 = black, 255 = white. We emit black glyph pixels on a
    white background so pbf_repack (which treats black PNG pixels as the
    glyph) round-trips cleanly to the PBF polarity used by the firmware
    renderer (bit=1 = drawn as text_color).
    """
    img = Image.new("1", (width, rows), 1)  # white background
    pixels = img.load()
    for y in range(rows):
        for x in range(width):
            byte_index = y * bitmap.pitch + (x >> 3)
            bit_index = 7 - (x & 7)
            if bitmap.buffer[byte_index] & (1 << bit_index):
                pixels[x, y] = 0  # black glyph pixel
    return img


def render_font(ttf_path, pixel_size, output_dir, codepoints=None, font_name=None):
    face = freetype.Face(ttf_path)
    face.set_pixel_sizes(0, pixel_size)

    ascender = face.size.ascender >> 6
    descender = face.size.descender >> 6  # negative
    max_height = pixel_size

    if font_name is None:
        font_name = os.path.splitext(os.path.basename(ttf_path))[0]

    glyphs_dir = os.path.join(output_dir, "glyphs")
    os.makedirs(glyphs_dir, exist_ok=True)

    # If no codepoints specified, extract all glyphs from the font
    if codepoints is None:
        codepoints = []
        charcode, gindex = face.get_first_char()
        while gindex != 0:
            codepoints.append(charcode)
            charcode, gindex = face.get_next_char(charcode, gindex)
        codepoints.sort()

    glyph_entries = []
    for cp in codepoints:
        gindex = face.get_char_index(cp)
        if gindex == 0:
            print(
                f"Warning: no glyph for U+{cp:04X} ({chr(cp)!r}), skipping",
                file=sys.stderr,
            )
            continue

        face.load_char(
            cp,
            freetype.FT_LOAD_RENDER
            | freetype.FT_LOAD_MONOCHROME
            | freetype.FT_LOAD_TARGET_MONO,
        )
        glyph = face.glyph
        bmp = glyph.bitmap

        width = bmp.width
        height = bmp.rows
        left_offset = glyph.bitmap_left
        top_offset = pixel_size - glyph.bitmap_top
        advance = glyph.advance.x >> 6

        # Handle empty glyphs (e.g. space)
        if width == 0 or height == 0:
            width = max(width, 1)
            height = max(height, 1)
            img = Image.new("1", (width, height), 0)
        else:
            img = bitmap_to_image(bmp, width, height)

        filename = f"U+{cp:04X}.png"
        filepath = os.path.join(glyphs_dir, filename)
        img.save(filepath)

        glyph_entries.append(
            {
                "codepoint": cp,
                "char": chr(cp),
                "file": f"glyphs/{filename}",
                "width": width,
                "height": height,
                "left_offset": left_offset,
                "top_offset": top_offset,
                "advance": advance,
            }
        )

    manifest = {
        "source": os.path.basename(ttf_path),
        "version": 3,
        "max_height": max_height,
        "wildcard_codepoint": 0x25AF,  # U+25AF
        "hash_table_size": 255,
        "codepoint_bytes": 2,
        "features": {
            "offset_16": True,
            "rle4": False,
        },
        "glyphs": glyph_entries,
    }

    manifest_path = os.path.join(output_dir, "font.json")
    with open(manifest_path, "w") as f:
        json.dump(manifest, f, indent=2, ensure_ascii=False)
        f.write("\n")

    print(f"Wrote {len(glyph_entries)} glyphs to {output_dir}/")
    print(f"  max_height={max_height} (ascender={ascender}, descender={descender})")
    print(f"  manifest: {manifest_path}")


def main():
    parser = argparse.ArgumentParser(
        description="Render TTF font glyphs to 1-bit PNGs with a JSON manifest."
    )
    parser.add_argument("ttf", help="Path to the TTF/OTF font file")
    parser.add_argument("size", type=int, help="Pixel size to render at")
    parser.add_argument(
        "-o",
        "--output",
        default=None,
        help="Output directory (default: <fontname>_<size>)",
    )
    parser.add_argument(
        "-g",
        "--glyphs",
        default=None,
        help="Glyph list: literal chars, ranges (a-z), "
        "or Unicode (U+0030-U+0039). "
        "If omitted, all glyphs in the font are exported.",
    )
    parser.add_argument(
        "-n",
        "--name",
        default=None,
        help="Font name for the manifest (default: filename stem)",
    )
    args = parser.parse_args()

    if not os.path.isfile(args.ttf):
        parser.error(f"Font file not found: {args.ttf}")

    codepoints = None
    if args.glyphs is not None:
        codepoints = parse_glyph_list(args.glyphs)
        if not codepoints:
            parser.error("No valid codepoints parsed from --glyphs")

    output_dir = args.output
    if output_dir is None:
        stem = os.path.splitext(os.path.basename(args.ttf))[0]
        output_dir = f"{stem}_{args.size}"

    render_font(args.ttf, args.size, output_dir, codepoints, args.name)


if __name__ == "__main__":
    main()
