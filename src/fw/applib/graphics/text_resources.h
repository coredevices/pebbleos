/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "applib/fonts/fonts_private.h"
#include "applib/fonts/codepoint.h"
#include "applib/graphics/gtypes.h"
#include "pbl/util/keyed_circular_cache.h"

#include <stdint.h>

typedef struct __attribute__((__packed__)) {
  uint8_t width_px;
  union {
    uint8_t height_px;
    uint8_t num_rle_units;
  };
  int8_t left_offset_px;
  int8_t top_offset_px;
  int8_t horiz_advance;
} GlyphHeaderData;

typedef struct __attribute__((__packed__)) {
  uint8_t width_px;
  uint8_t height_px;
  int8_t left_offset_px;
  int8_t top_offset_px;
  uint8_t empty[3];
  int8_t horiz_advance;
} GlyphHeaderDataV1;

typedef struct __attribute__((__packed__)) {
  GlyphHeaderData header;
  uint32_t data[];
} GlyphData;

//! Maps a codepoint to the location of the actual font data.
typedef struct __attribute__((__packed__)) {
  Codepoint codepoint : 16;
  uint16_t offset;
} OffsetTableEntry_2_2;

typedef struct __attribute__((__packed__)) {
  Codepoint codepoint : 16;
  uint32_t offset;
} OffsetTableEntry_2_4;

typedef struct __attribute__((__packed__)) {
  Codepoint codepoint;
  uint32_t offset;
} OffsetTableEntry_4_4;

typedef struct __attribute__((__packed__)) {
  Codepoint codepoint;
  uint16_t offset;
} OffsetTableEntry_4_2;

#if !defined(MAX_FONT_GLYPH_SIZE)
  #define MAX_FONT_GLYPH_SIZE 256
#endif

// Slightly bigger than the biggest glyph we have
// This is the size in bytes for the glyph bitmap data.
#define CACHE_GLYPH_SIZE MAX_FONT_GLYPH_SIZE

typedef struct {
  uint32_t resource_offset;
  //! Whether the bitmap data in this structure is valid.
  bool is_bitmap_loaded;

  union {
    //! Glyph data without a bitmap
    GlyphHeaderData header_data;
    GlyphData glyph_data;
  };
} LineCacheData;

#define LINE_CACHE_SIZE 30

// Allow 1K max for offset tables
#define OFFSET_TABLE_MAX_SIZE (1024)

typedef struct FontCache {
  int offset_table_id;
  uint16_t offset_table_size;
  //! The currently loaded font's offset table.
  //! @note this needs to be able to accomodate legacy fonts
  union {
    OffsetTableEntry_2_2 offsets_buffer_2_2[OFFSET_TABLE_MAX_SIZE / sizeof(OffsetTableEntry_2_2)];
    OffsetTableEntry_2_4 offsets_buffer_2_4[OFFSET_TABLE_MAX_SIZE / sizeof(OffsetTableEntry_2_4)];
    OffsetTableEntry_4_2 offsets_buffer_4_2[OFFSET_TABLE_MAX_SIZE / sizeof(OffsetTableEntry_4_2)];
    OffsetTableEntry_4_4 offsets_buffer_4_4[OFFSET_TABLE_MAX_SIZE / sizeof(OffsetTableEntry_4_4)];
  };
  //! line_cache's backing storage for keys
  KeyedCircularCacheKey cache_keys[LINE_CACHE_SIZE];
  //! line_cache's backing storage for data
  LineCacheData cache_data[LINE_CACHE_SIZE];
  //! some scratch space so we don't need to create a LineCacheData on the stack
  LineCacheData cache_data_scratch;

  //! cache_key for the last used glyph
  uint32_t glyph_buffer_key;
  //! data for the last used glyph
  uint8_t glyph_buffer[sizeof(LineCacheData) + CACHE_GLYPH_SIZE];
  KeyedCircularCache line_cache;
  const FontResource *cached_font;
} FontCache;

//! @param font_cache The font cache to look up the glyph in
//! @param codepoint The codepoint to get the glyph for
//! @param font_info The font to get the glyph from
//! @param baseline_adjust_out optional: pixels to add to the glyph's top_offset when drawing,
//! non-zero only when another font (emoji or system fallback) supplied the glyph
const GlyphData *text_resources_get_glyph(FontCache *font_cache, Codepoint codepoint,
                                          FontInfo *font_info, int16_t *baseline_adjust_out);

//! Look up an anchor offset for placing mark_codepoint relative to base_codepoint.
//! Returns true and fills offset_out when an embedded GPOS anchor is available.
bool text_resources_get_mark_anchor_offset(FontCache *font_cache, Codepoint base_codepoint,
                                           Codepoint mark_codepoint, FontInfo *font_info,
                                           GPoint *offset_out);

int8_t text_resources_get_glyph_horiz_advance(FontCache *font_cache, Codepoint codepoint,
                                              FontInfo *font_info);

//! Initialize a FontInfo struct with resource contents
//! A FontInfo contains references to up to *two* font resources: a "base" font and an "extension".
//! The base font is part of the system resources pack and contains latin characters and emoji
//! The extension font contains additional characters required to display localized UI or
//! notifications.
//! @note the extension may not be installed and may be removed at any give time
//! @param app_num the ResAppNum associated with this font (i.e. system or app?)
//! @param font_resource the "base" resource id
//! @param extension_resource the "extension" resource id
//! @param font_info a pointer to the fontinfo struct to initialize
bool text_resources_init_font(ResAppNum app_num, uint32_t font_resource,
                              uint32_t extension_resource, FontInfo *font_info);

