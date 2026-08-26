#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2024 Google LLC
# SPDX-License-Identifier: Apache-2.0

"""
Repack a PBF font from individual PNG images and a JSON manifest.
Use with pbf_extract.py to modify existing fonts or add new glyphs.

Usage:
    python pbf_repack.py leco42/font.json -o LECO_42_NUMBERS_PATCHED.pbf
"""

import argparse
import json
import os
import struct

from PIL import Image

# Feature flags
FEATURE_OFFSET_16 = 0x01
FEATURE_RLE4 = 0x02


def image_to_bitmap(img_path, expected_width=None, expected_height=None):
    """Convert a PNG image to a bitmap (list of bits, row-major)."""
    img = Image.open(img_path)

    # Convert to 1-bit if needed
    if img.mode != "1":
        img = img.convert("1")

    width, height = img.size
    pixels = img.load()

    # PBF format: 1 = glyph (drawn as text_color), 0 = background (untouched).
    # PIL mode '1': 0 = black, 255 = white. Black pixels in the PNG are the glyph.
    bitlist = []
    for y in range(height):
        for x in range(width):
            bitlist.append(1 if pixels[x, y] == 0 else 0)

    return bitlist, width, height


def bitlist_to_bytes(bitlist):
    """Convert a bitlist to packed bytes (little-endian, 32-bit aligned)."""
    if not bitlist:
        return b""

    # Pack bits into 32-bit words (little-endian bit order)
    words = []
    for i in range(0, len(bitlist), 32):
        word = 0
        for bit_idx in range(32):
            if i + bit_idx < len(bitlist) and bitlist[i + bit_idx]:
                word |= 1 << bit_idx
        words.append(word)

    return struct.pack(f"<{len(words)}I", *words)


def compress_rle4(bitlist):
    """Compress bitlist using RLE4 encoding. Returns (rle_units, compressed_bytes)."""
    if not bitlist:
        return 0, b""

    rle_units = []
    i = 0

    while i < len(bitlist):
        symbol = bitlist[i]
        run_length = 1

        # Count consecutive same symbols (max 8 per unit)
        while (
            i + run_length < len(bitlist)
            and bitlist[i + run_length] == symbol
            and run_length < 8
        ):
            run_length += 1

        # Encode: bit 3 = symbol, bits 0-2 = length-1
        unit = ((symbol & 1) << 3) | ((run_length - 1) & 0x7)
        rle_units.append(unit)
        i += run_length

    # Pack units into bytes (2 per byte, LSN first)
    result = []
    for i in range(0, len(rle_units), 2):
        byte = rle_units[i]
        if i + 1 < len(rle_units):
            byte |= rle_units[i + 1] << 4
        result.append(byte)

    return len(rle_units), bytes(result)


def build_pbf(manifest, output_path, use_rle4=False):
    """Build a PBF font from a manifest and glyph images."""
    base_dir = os.path.dirname(manifest["_manifest_path"])

    # Determine format parameters
    version = manifest.get("version", 3)
    max_height = manifest["max_height"]
    wildcard_cp = manifest.get("wildcard_codepoint", 0x25AF)
    hash_table_size = manifest.get("hash_table_size", 255)
    codepoint_bytes = manifest.get("codepoint_bytes", 2)

    # Feature flags
    use_offset_16 = manifest.get("features", {}).get("offset_16", True)
    use_rle4 = manifest.get("features", {}).get("rle4", False) or use_rle4

    features = 0
    if use_offset_16:
        features |= FEATURE_OFFSET_16
    if use_rle4:
        features |= FEATURE_RLE4

    # Load and process all glyphs
    glyphs = []
    for glyph_info in manifest["glyphs"]:
        codepoint = glyph_info["codepoint"]
        img_path = os.path.join(base_dir, glyph_info["file"])

        if os.path.exists(img_path):
            bitlist, width, height = image_to_bitmap(img_path)
        else:
            # Empty glyph
            bitlist, width, height = [], 0, 0

        # Use metrics from manifest, or derive from image
        width = glyph_info.get("width", width)
        height = glyph_info.get("height", height)
        left = glyph_info.get("left_offset", 0)
        top = glyph_info.get("top_offset", 0)
        advance = glyph_info.get("advance", width + 1)

        # Build glyph data
        if use_rle4 and bitlist:
            num_rle_units, bitmap_bytes = compress_rle4(bitlist)
            height_field = num_rle_units
        else:
            bitmap_bytes = bitlist_to_bytes(bitlist)
            height_field = height

        # Glyph header: width, height/rle_units, left, top, advance
        glyph_header = struct.pack("BBbbb", width, height_field, left, top, advance)

        # Align bitmap data to 4 bytes
        padding_needed = (4 - (len(bitmap_bytes) % 4)) % 4
        bitmap_bytes_aligned = bitmap_bytes + b"\x00" * padding_needed

        glyphs.append(
            {
                "codepoint": codepoint,
                "data": glyph_header + bitmap_bytes_aligned,
                "width": width,
                "height": height,
            }
        )

    # Sort by codepoint
    glyphs.sort(key=lambda g: g["codepoint"])
    num_glyphs = len(glyphs)

    # Note: max_height from manifest is preserved (it's the font's line height,
    # not necessarily the tallest glyph - e.g. wildcard glyph may be taller)

    # Build glyph table and collect offsets
    # IMPORTANT: Start with 4 bytes of padding because offset 0 means "not found"
    # in the firmware and triggers wildcard fallback
    glyph_table = b"\x00\x00\x00\x00"
    glyph_offsets = {}  # codepoint -> offset in glyph table

    for glyph in glyphs:
        glyph_offsets[glyph["codepoint"]] = len(glyph_table)
        glyph_table += glyph["data"]

    # Check if offsets fit in 16 bits
    max_offset = max(glyph_offsets.values()) if glyph_offsets else 0
    if max_offset > 0xFFFF and use_offset_16:
        print(
            f"Warning: Offsets exceed 16-bit range ({max_offset}), using 32-bit offsets"
        )
        use_offset_16 = False
        features &= ~FEATURE_OFFSET_16

    # Build hash table and offset tables
    # Hash table: for each hash bucket, (hash_id, count, offset_in_offset_table)
    # Offset table: sorted by hash bucket, then by codepoint within bucket

    hash_buckets = {i: [] for i in range(hash_table_size)}
    for glyph in glyphs:
        bucket_id = glyph["codepoint"] % hash_table_size
        hash_buckets[bucket_id].append(glyph["codepoint"])

    # Sort each bucket by codepoint
    for bucket_id in hash_buckets:
        hash_buckets[bucket_id].sort()

    # Determine offset entry format
    cp_format = "I" if codepoint_bytes == 4 else "H"
    off_format = "H" if use_offset_16 else "I"
    offset_entry_format = "<" + cp_format + off_format
    offset_entry_size = struct.calcsize(offset_entry_format)

    # Build offset table (ordered by hash bucket)
    offset_table = b""
    hash_table_entries = []

    for bucket_id in range(hash_table_size):
        codepoints = hash_buckets[bucket_id]
        count = len(codepoints)
        offset_in_table = len(offset_table)  # byte offset

        for cp in codepoints:
            glyph_off = glyph_offsets[cp]
            offset_table += struct.pack(offset_entry_format, cp, glyph_off)

        hash_table_entries.append((bucket_id, count, offset_in_table))

    # Build hash table
    hash_table = b""
    for bucket_id, count, offset in hash_table_entries:
        hash_table += struct.pack("<BBH", bucket_id, count, offset)

    # Build font header
    if version == 3:
        struct_size = 10  # Size of FontMetaDataV3 structure (10 bytes)
        header = struct.pack(
            "<BBHHBBBB",
            version,
            max_height,
            num_glyphs,
            wildcard_cp,
            hash_table_size,
            codepoint_bytes,
            struct_size,
            features,
        )
    else:  # version 2
        header = struct.pack(
            "<BBHHBB",
            version,
            max_height,
            num_glyphs,
            wildcard_cp,
            hash_table_size,
            codepoint_bytes,
        )

    # Combine all parts
    font_data = header + hash_table + offset_table + glyph_table

    # Write output
    with open(output_path, "wb") as f:
        f.write(font_data)

    print(f"Created {output_path}")
    print(f"  Version: {version}")
    print(f"  Glyphs: {num_glyphs}")
    print(f"  Max height: {max_height}")
    print(f"  RLE4: {use_rle4}")
    print(f"  Size: {len(font_data)} bytes")

    return True


def main():
    parser = argparse.ArgumentParser(
        description="Repack PBF font from images and manifest"
    )
    parser.add_argument("manifest", help="Input font.json manifest file")
    parser.add_argument("-o", "--output", required=True, help="Output PBF file")
    parser.add_argument("--rle4", action="store_true", help="Force RLE4 compression")
    args = parser.parse_args()

    if not os.path.exists(args.manifest):
        print(f"Error: Manifest not found: {args.manifest}")
        return 1

    with open(args.manifest) as f:
        manifest = json.load(f)

    manifest["_manifest_path"] = args.manifest

    build_pbf(manifest, args.output, use_rle4=args.rle4)
    return 0


if __name__ == "__main__":
    exit(main())
