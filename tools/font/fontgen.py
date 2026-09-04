#!/usr/bin/env python
# SPDX-FileCopyrightText: 2024 Google LLC
# SPDX-License-Identifier: Apache-2.0


import itertools
import json
import os
import re
import struct
import sys

import freetype

sys.path.append(os.path.join(os.path.dirname(__file__), "../"))

# Font v3 -- see docs/reference/formats/font.md for the rendered spec
#   FontInfo
#       (uint8_t)  version                           - v1
#       (uint8_t)  max_height                        - v1
#       (uint16_t) number_of_glyphs                  - v1
#       (uint16_t) wildcard_codepoint                - v1
#       (uint8_t)  hash_table_size                   - v2
#       (uint8_t)  codepoint_bytes                   - v2
#       (uint8_t)  size                              - v3  # Save the size of FontInfo for sanity
#       (uint8_t)  features                          - v3
#
#   font_info_struct_size is the size of the FontInfo structure. This makes extending this structure
#   in the future far simpler.
#
#   'features' is a bitmap defined as follows:
#       0: offset table offsets uint32 if 0, uint16 if 1
#       1: glyphs are bitmapped if 0, RLE4 encoded if 1
#     2-7: reserved
#
#   (uint32_t) hash_table[]
#       glyph_tables are found in the resource image by converting a codepoint into an offset from
#       the start of the resource. This conversion is implemented as a hash where collisions are
#       resolved by separate chaining. Each entry in the hash table is as follows:
#                  (uint8_t) hash value
#                  (uint8_t) offset_table_size
#                  (uint16_t) offset
#       A codepoint is converted into a hash value by the hash function -- this value is a direct
#       index into the hash table array. 'offset' is the location of the correct offset_table list
#       from the start of offset_tables, and offset_table_size gives the number of glyph_tables in
#       the list (i.e., the number of codepoints that hash to the same value).
#
#   offset_tables[][]
#       this list of tables contains offsets into the glyph_table for the codepoint.
#       each offset is counted in bytes from the start of glyph_table (v1 files
#       counted 32-bit blocks instead).
#       packed:     (codepoint_bytes [uint16_t | uint32_t]) codepoint
#                   (features[0] [uint16_t | uint32_t]) offset
#
#   glyph_table[]
#       starts with one zeroed 32-bit block: offset 0 indicates that a glyph is
#       not supported. then for each glyph, a packed 5-byte header:
#                              (uint_8) bitmap_width
#                              (uint_8) bitmap_height        NB: in v3, if RLE4 compressed, this
#                                                                field contains the number of
#                                                                RLE4 units.
#                               (int_8) offset_left
#                               (int_8) offset_top
#                               (int_8) horizontal_advance
#       followed immediately by the bitmap data (unaligned rows of bits), padded
#       with 0's at the end to make the bitmap data a multiple of 4 bytes
#       (v1 files used a different, 8-byte glyph header)

MIN_CODEPOINT = 0x20
MAX_2_BYTES_CODEPOINT = 0xFFFF
MAX_EXTENDED_CODEPOINT = 0x10FFFF
FONT_VERSION_1 = 1
FONT_VERSION_2 = 2
FONT_VERSION_3 = 3
# Set a codepoint that the font doesn't know how to render
# The watch will use this glyph as the wildcard character
WILDCARD_CODEPOINT = 0x25AF  # White vertical rectangle
ELLIPSIS_CODEPOINT = 0x2026
# Features
FEATURE_OFFSET_16 = 0x01
FEATURE_RLE4 = 0x02


HASH_TABLE_SIZE = 255
OFFSET_TABLE_MAX_SIZE = 128
MAX_GLYPHS_EXTENDED = HASH_TABLE_SIZE * OFFSET_TABLE_MAX_SIZE
MAX_GLYPHS = 256


def grouper(n, iterable, fillvalue=None):
    """grouper(3, 'ABCDEFG', 'x') --> ABC DEF Gxx"""
    args = [iter(iterable)] * n
    return itertools.zip_longest(*args, fillvalue=fillvalue)


def hasher(codepoint, num_glyphs):
    return codepoint % num_glyphs


def bits(x):
    data = []
    for i in range(8):
        data.insert(0, int((x & 1) == 1))
        x = x >> 1
    return data


class Font:
    def __init__(
        self, ttf_path, height, max_glyphs, max_glyph_size, legacy, baseline=None
    ):
        self.version = FONT_VERSION_3
        self.ttf_path = ttf_path
        self.max_height = int(height)
        # Baseline row; defaults to max_height. Extended fonts pass the base
        # font's baseline so both align on a mixed line.
        self.baseline = int(baseline) if baseline is not None else self.max_height
        self.legacy = legacy
        self.face = freetype.Face(self.ttf_path)
        self.face.set_pixel_sizes(0, self.max_height)
        self.name = self.face.family_name + b"_" + self.face.style_name
        self.wildcard_codepoint = WILDCARD_CODEPOINT
        self.number_of_glyphs = 0
        self.table_size = HASH_TABLE_SIZE
        self.tracking_adjust = 0
        self.regex = None
        self.codepoints = list(range(MIN_CODEPOINT, MAX_EXTENDED_CODEPOINT))
        self.codepoint_bytes = 2
        self.max_glyphs = max_glyphs
        self.max_glyph_size = max_glyph_size
        self.glyph_table = []
        self.hash_table = [0] * self.table_size
        self.offset_tables = [[] for i in range(self.table_size)]
        self.offset_size_bytes = 4
        self.features = 0

        self.glyph_header = "<BBbbb"

    def set_compression(self, engine):
        if self.version != FONT_VERSION_3:
            raise RuntimeError(
                f"Compression being set but version != 3 ({self.version})"
            )
        if engine == "RLE4":
            self.features |= FEATURE_RLE4
        else:
            raise RuntimeError(
                f"Unsupported compression engine: '{engine}'. Font {self.ttf_path}"
            )

    def set_version(self, version):
        self.version = version

    def set_tracking_adjust(self, adjust):
        self.tracking_adjust = adjust

    def set_regex_filter(self, regex_string):
        if regex_string != ".*":
            try:
                self.regex = re.compile(regex_string)
            except re.error as e:
                raise RuntimeError(
                    "Supplied filter argument was not a valid regular expression."
                    f"Font: {self.ttf_path}"
                ) from e
        else:
            self.regex = None

    def set_codepoint_list(self, list_path):
        with open(list_path) as codepoints_file:
            codepoints_json = json.load(codepoints_file)
        self.codepoints = [int(cp) for cp in codepoints_json["codepoints"]]

    def is_supported_glyph(self, codepoint):
        return self.face.get_char_index(codepoint) > 0 or (
            codepoint == chr(self.wildcard_codepoint)
        )

    def compress_glyph_RLE4(self, bitmap):
        # This Run Length Compression scheme works by converting runs of identical symbols to the
        # symbol and the length of the run. The length of each run of symbols is limited to
        # [1..2**(RLElen-1)]. For RLE4, the length is 3 bits (0-7), or 1-8 consecutive symbols.
        # For example: 11110111 is compressed to 1*4, 0*1, 1*3. or [(1, 4), (0, 1), (1, 3)]

        RLE_LEN = 2 ** (4 - 1)  # TODO possibly make this a parameter.
        # It would likely be a good idea to look into the 'bitstream' package for lengths that won't
        # easily fit into a byte/short/int.

        # First, generate a list of tuples (bit, count).
        unit_list = [
            (name, len(list(group))) for name, group in itertools.groupby(bitmap)
        ]

        # Second, generate a list of RLE tuples where count <= RLE_LEN. This intermediate step will
        # make it much easier to implement the binary stream packer below.
        rle_unit_list = []
        for name, length in unit_list:
            while length > 0:
                unit_len = min(length, RLE_LEN)
                rle_unit_list.append((name, unit_len))
                length -= unit_len

        # Note that num_units does not include the padding added below.
        num_units = len(rle_unit_list)

        # If the list is odd, add a padding unit
        if (num_units % 2) == 1:
            rle_unit_list.append((0, 1))

        # Now pack the tuples into a binary stream. We can't pack nibbles, so join two
        glyph_packed = []
        it = iter(rle_unit_list)
        for name, length in it:
            name2, length2 = next(it)
            packed_byte = name << 3 | (length - 1) | name2 << 7 | (length2 - 1) << 4
            glyph_packed.append(struct.pack("<B", packed_byte))

        # Pad out to the nearest 4 bytes
        while (len(glyph_packed) % 4) > 0:
            glyph_packed.append(struct.pack("<B", 0))

        return (glyph_packed, num_units)

    # Make sure that we will be able to decompress the glyph in-place
    def check_decompress_glyph_RLE4(self, glyph_packed, width, rle_units):
        # The glyph buffer before decoding is arranged as follows:
        #  [ <header> | <free space> | <encoded glyph> ]
        # Make sure that we can decode the encoded glyph to end up with the following arrangement:
        #  [ <header> |       <decoded glyph>          ]
        # without overwriting the unprocessed encoded glyph in the process

        header_size = struct.calcsize(self.glyph_header)
        dst_ptr = header_size
        src_ptr = self.max_glyph_size - len(glyph_packed)

        def glyph_packed_iterator(tbl, num):
            for i in range(num):
                yield struct.unpack("<B", tbl[i])[0]

        # Generate glyph buffer. Ignore the header
        bitmap = [0] * self.max_glyph_size
        bitmap[-len(glyph_packed) :] = glyph_packed_iterator(
            glyph_packed, len(glyph_packed)
        )

        out_num_bits = 0
        out = 0
        total_length = 0
        while rle_units > 0:
            if src_ptr >= self.max_glyph_size:
                raise RuntimeError(
                    f"Error: input stream too large for buffer. Font {self.ttf_path}"
                )

            unit_pair = bitmap[src_ptr]
            src_ptr += 1
            for i in range(min(rle_units, 2)):
                colour = (unit_pair >> 3) & 1
                length = (unit_pair & 0x07) + 1
                total_length += length

                if colour:
                    # Generate the bitpattern 111...
                    add = (1 << length) - 1
                    out |= add << out_num_bits
                out_num_bits += length

                if out_num_bits >= 8:
                    if dst_ptr >= src_ptr:
                        raise RuntimeError(
                            f"Error: unable to RLE4 decode in place! Overrun. Font {self.ttf_path}"
                        )
                    if dst_ptr >= self.max_glyph_size:
                        raise RuntimeError(
                            f"Error: output bitmap too large for buffer. Font {self.ttf_path}"
                        )
                    bitmap[dst_ptr] = out & 0xFF
                    dst_ptr += 1
                    out >>= 8
                    out_num_bits -= 8

                unit_pair >>= 4
                rle_units -= 1

        while out_num_bits > 0:
            bitmap[dst_ptr] = out & 0xFF
            dst_ptr += 1
            out >>= 8
            out_num_bits -= 8

        # Success! We can in-place decode this glyph
        return True

    def glyph_bits(self, codepoint, gindex):
        flags = (
            freetype.FT_LOAD_RENDER
            if self.legacy
            else freetype.FT_LOAD_RENDER
            | freetype.FT_LOAD_MONOCHROME
            | freetype.FT_LOAD_TARGET_MONO
        )
        self.face.load_glyph(gindex, flags)
        # Font metrics
        bitmap = self.face.glyph.bitmap
        advance = (
            self.face.glyph.advance.x // 64
        )  # Convert 26.6 fixed float format to px
        advance += self.tracking_adjust
        width = bitmap.width
        height = bitmap.rows
        left = self.face.glyph.bitmap_left
        bottom = self.baseline - self.face.glyph.bitmap_top
        pixel_mode = self.face.glyph.bitmap.pixel_mode

        glyph_packed = []
        if height and width:
            glyph_bitmap = []
            if pixel_mode == 1:  # monochrome font, 1 bit per pixel
                for i in range(bitmap.rows):
                    row = []
                    for j in range(bitmap.pitch):
                        row.extend(bits(bitmap.buffer[i * bitmap.pitch + j]))
                    glyph_bitmap.extend(row[: bitmap.width])
            elif pixel_mode == 2:  # grey font, 255 bits per pixel
                for val in bitmap.buffer:
                    glyph_bitmap.extend([1 if val > 127 else 0])
            else:
                # freetype-py should never give us a value not in (1,2)
                raise RuntimeError(
                    f"Unsupported pixel mode: {pixel_mode}. Font {self.ttf_path}"
                )

            if self.features & FEATURE_RLE4:
                # HACK WARNING: override the height with the number of RLE4 units.
                glyph_packed, height = self.compress_glyph_RLE4(glyph_bitmap)
                if height > 255:
                    raise RuntimeError(
                        "Unable to RLE4 compress -- more than 255 units required"
                        f"({height}). Font {self.ttf_path}"
                    )
                # Check that we can in-place decompress. Will raise an exception if not.
                self.check_decompress_glyph_RLE4(glyph_packed, width, height)
            else:
                for word in grouper(32, glyph_bitmap, 0):
                    w = 0
                    for index, bit in enumerate(word):
                        w |= bit << index
                    glyph_packed.append(struct.pack("<I", w))

                # Confirm that we're smaller than the cache size
                size = ((width * height) + (8 - 1)) // 8
                if size > self.max_glyph_size:
                    raise RuntimeError(
                        f"Glyph too large! codepoint {codepoint}: {size} > {self.max_glyph_size}. Font {self.ttf_path}"
                    )

        glyph_header = struct.pack(
            self.glyph_header, width, height, left, bottom, advance
        )

        return glyph_header + b"".join(glyph_packed)

    def fontinfo_bits(self):
        if self.version == FONT_VERSION_2:
            s = struct.Struct("<BBHHBB")
            return s.pack(
                self.version,
                self.max_height,
                self.number_of_glyphs,
                self.wildcard_codepoint,
                self.table_size,
                self.codepoint_bytes,
            )
        else:
            s = struct.Struct("<BBHHBBBB")
            return s.pack(
                self.version,
                self.max_height,
                self.number_of_glyphs,
                self.wildcard_codepoint,
                self.table_size,
                self.codepoint_bytes,
                s.size,
                self.features,
            )

    def build_tables(self):
        def build_hash_table(bucket_sizes):
            acc = 0
            for i in range(self.table_size):
                bucket_size = bucket_sizes[i]
                self.hash_table[i] = struct.pack("<BBH", i, bucket_size, acc)
                acc += bucket_size * (self.offset_size_bytes + self.codepoint_bytes)

        def build_offset_tables(glyph_entries):
            offset_table_format = "<"
            offset_table_format += "L" if self.codepoint_bytes == 4 else "H"
            offset_table_format += "L" if self.offset_size_bytes == 4 else "H"

            bucket_sizes = [0] * self.table_size
            for entry in glyph_entries:
                codepoint, offset = entry
                glyph_hash = hasher(codepoint, self.table_size)
                self.offset_tables[glyph_hash].append(
                    struct.pack(offset_table_format, codepoint, offset)
                )
                bucket_sizes[glyph_hash] = bucket_sizes[glyph_hash] + 1
                if bucket_sizes[glyph_hash] > OFFSET_TABLE_MAX_SIZE:
                    print(f"error: {bucket_sizes[glyph_hash]:d} > 127")
            return bucket_sizes

        def add_glyph(codepoint, next_offset, gindex, glyph_indices_lookup):
            offset = next_offset
            if gindex not in glyph_indices_lookup:
                glyph_bits = self.glyph_bits(codepoint, gindex)
                glyph_indices_lookup[gindex] = offset
                self.glyph_table.append(glyph_bits)
                next_offset += len(glyph_bits)
            else:
                offset = glyph_indices_lookup[gindex]

            if codepoint > MAX_2_BYTES_CODEPOINT:
                self.codepoint_bytes = 4

            self.number_of_glyphs += 1
            return offset, next_offset, glyph_indices_lookup

        def codepoint_is_in_subset(codepoint):
            if codepoint not in (WILDCARD_CODEPOINT, ELLIPSIS_CODEPOINT):
                if self.regex is not None and self.regex.match(chr(codepoint)) is None:
                    return False
                if codepoint not in self.codepoints:
                    return False
            return True

        glyph_entries = []
        # MJZ: The 0th offset of the glyph table is 32-bits of
        # padding, no idea why.
        self.glyph_table.append(struct.pack("<I", 0))
        self.number_of_glyphs = 0
        glyph_indices_lookup = {}
        next_offset = 4
        codepoint, gindex = self.face.get_first_char()

        # add wildcard_glyph
        offset, next_offset, glyph_indices_lookup = add_glyph(
            WILDCARD_CODEPOINT, next_offset, 0, glyph_indices_lookup
        )
        glyph_entries.append((WILDCARD_CODEPOINT, offset))

        while gindex:
            # Hard limit on the number of glyphs in a font
            if self.number_of_glyphs > self.max_glyphs:
                break

            if codepoint is WILDCARD_CODEPOINT:
                raise RuntimeError(
                    "Wildcard codepoint is used for something else in this font."
                    f"Font {self.ttf_path}"
                )

            if gindex == 0:
                raise RuntimeError(
                    f"0 index is reused by a non wildcard glyph. Font {self.ttf_path}"
                )

            if codepoint_is_in_subset(codepoint):
                offset, next_offset, glyph_indices_lookup = add_glyph(
                    codepoint, next_offset, gindex, glyph_indices_lookup
                )
                glyph_entries.append((codepoint, offset))

            codepoint, gindex = self.face.get_next_char(codepoint, gindex)

        # Decide if we need 2 byte or 4 byte offsets
        glyph_data_bytes = sum(len(glyph) for glyph in self.glyph_table)
        if self.version == FONT_VERSION_3 and glyph_data_bytes < 65536:
            self.features |= FEATURE_OFFSET_16
            self.offset_size_bytes = 2

        # Make sure the entries are sorted by codepoint
        sorted_entries = sorted(glyph_entries, key=lambda entry: entry[0])
        hash_bucket_sizes = build_offset_tables(sorted_entries)
        build_hash_table(hash_bucket_sizes)

    def bitstring(self):
        btstr = self.fontinfo_bits()
        btstr += b"".join(self.hash_table)
        for table in self.offset_tables:
            btstr += b"".join(table)
        btstr += b"".join(self.glyph_table)

        return btstr
