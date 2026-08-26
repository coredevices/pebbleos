# Font resources (PBF)

Fonts ship in firmware resources as binary PBF files, generated from TTF
input by `tools/font/fontgen.py` and parsed at runtime by
`src/fw/applib/graphics/text_resources.c`. The on-disk structs live in
`src/fw/applib/fonts/fonts_private.h` and
`src/fw/applib/graphics/text_resources.h`; `tools/font/` also contains
standalone readers (`dump_font.py`, `pbf_extract.py`, `pbf_repack.py`).

A PBF file is four consecutive sections, all multi-byte fields
little-endian:

1. `FontInfo` header
2. hash table
3. offset tables
4. glyph table

## FontInfo header

"FontInfo" is the generator's name for the on-disk header; the firmware
calls the struct `FontMetaDataV3` (`FontInfo` in `fonts_private.h` is an
unrelated runtime struct).

| Field                | Type       | Since | Notes                                          |
| -------------------- | ---------- | ----- | ---------------------------------------------- |
| `version`            | `uint8_t`  | v1    | 1, 2 or 3                                      |
| `max_height`         | `uint8_t`  | v1    | tallest glyph in the font                      |
| `number_of_glyphs`   | `uint16_t` | v1    |                                                |
| `wildcard_codepoint` | `uint16_t` | v1    | rendered for missing glyphs; default U+25AF    |
| `hash_table_size`    | `uint8_t`  | v2    | bucket count; the generator always writes 255  |
| `codepoint_bytes`    | `uint8_t`  | v2    | 2, or 4 if any codepoint exceeds U+FFFF        |
| `size`               | `uint8_t`  | v3    | size of this header (10), for future extension |
| `features`           | `uint8_t`  | v3    | see below                                      |

`features` bits (`FEATURE_OFFSET_16` / `FEATURE_RLE4` in
`fonts_private.h`):

- bit 0: glyph-table offsets are `uint16_t` if set, `uint32_t` if clear.
  The generator sets it when the glyph table fits in 64 KiB.
- bit 1: glyph bitmaps are RLE4-compressed if set, plain bitmaps if clear.
- bit 2: glyph records may include embedded GPOS anchor payloads.
- bits 3–7: reserved.

## Hash table

`hash_table_size` entries of 4 bytes each (`FontHashTableEntry`):
`uint8_t hash`, `uint8_t count`, `uint16_t offset`. A codepoint hashes to
bucket `codepoint % hash_table_size`; `offset` is the byte offset of the
bucket's entries within the offset-tables section, and `count` is the
number of entries there (collisions are chained contiguously, at most 128
entries per bucket).

## Offset tables

Each bucket is a run of `{codepoint, glyph_offset}` entries, sorted by
codepoint so the firmware can binary-search. Field widths vary per file:
codepoints are 2 or 4 bytes (`codepoint_bytes`), glyph offsets 2 or 4
bytes (`features` bit 0), giving entry sizes of 4–8 bytes
(`OffsetTableEntry_2_2` … `_4_4` in `text_resources.h`).

## Glyph table

The table begins with one 32-bit zero word, so offset 0 means "glyph not
present". Offsets are byte offsets from the start of the glyph table
(version 1 counted 32-bit words instead). Glyphs are deduplicated: several
codepoints may share one glyph offset.

Each glyph is a packed 5-byte header (`GlyphHeaderData`) followed
immediately by bitmap data:

| Field           | Type      |
| --------------- | --------- |
| `width`         | `uint8_t` |
| `height`        | `uint8_t` |
| `left_offset`   | `int8_t`  |
| `top_offset`    | `int8_t`  |
| `horiz_advance` | `int8_t`  |

With RLE4 enabled, the `height` byte instead stores the number of RLE
units (the decoder recovers the height from the decompressed size).
Version 1 used a different 8-byte header (`GlyphHeaderDataV1`).

Bitmap data is 1 bit per pixel: rows are concatenated unaligned into one
continuous bit stream, packed LSB-first into 32-bit words and zero-padded
to a multiple of 4 bytes.

When feature bit 2 is set, a glyph may include an optional embedded anchor
payload immediately after the aligned bitmap data. The current payload format
is:

- 2 bytes magic: `GA`
- 1 byte payload version (currently 1)
- 1 byte entry count
- repeated entries: `uint16 mark_codepoint`, `int16 dx_fu`, `int16 dy_fu`

`dx_fu`/`dy_fu` are signed font-unit offsets derived from GPOS MarkBasePos and
MarkMarkPos extraction. The firmware scales them to pixels at runtime using
the rendered font's ppem and the source font's units-per-em value.

RLE4 compression is a stream of 4-bit units, two per byte (low nibble
first): each unit is `[symbol:1][length:3]`, emitting `length + 1`
(1–8) copies of the symbol bit. An odd number of units is padded with a
`(0,1)` unit, and the stream is zero-padded to a multiple of 4 bytes. The
firmware decompresses in place in the glyph cache; the generator verifies
at build time that every glyph is in-place decodable. The decoder-side
description lives in `text_resources.c`.
