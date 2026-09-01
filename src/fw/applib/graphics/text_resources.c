/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "text.h"
#include "text_resources.h"

#include "syscall/syscall.h"

#include "applib/fonts/fonts.h"
#include "applib/fonts/fonts_private.h"
#include "resource/resource_ids.auto.h"
#include <pbl/logging/logging.h>
#include <pbl/util/attributes.h>
#include "system/passert.h"
#include "system/profiler.h"
#include "pbl/util/math.h"
#include "pbl/util/size.h"

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#define RLE4_UNITS_BIT_WIDTH (4)
#define RLE4_UNITS_PER_BYTE  (8 / RLE4_UNITS_BIT_WIDTH)

static const size_t s_font_md_size[] = {
  0, // There currently is no font version 0. This makes decoding much easier & consistent
  sizeof(FontMetaDataV1),
  sizeof(FontMetaData),
  sizeof(FontMetaDataV3)
};


static uint8_t prv_font_hash(Codepoint codepoint, uint8_t table_size) {
  return (codepoint % table_size);
}

static Codepoint prv_offset_table_get_codepoint(FontCache *font_cache, const FontMetaData *md,
                                                int index) {
  const bool offset_16 = HAS_FEATURE(md->version, VERSION_FIELD_FEATURE_OFFSET_16);
  if (md->codepoint_bytes == 2) {
      return offset_16 ? font_cache->offsets_buffer_2_2[index].codepoint :
                         font_cache->offsets_buffer_2_4[index].codepoint;
  } else {
      return offset_16 ? font_cache->offsets_buffer_4_2[index].codepoint :
                         font_cache->offsets_buffer_4_4[index].codepoint;
  }
}

static uint32_t prv_offset_table_get_offset(FontCache *font_cache, const FontMetaData *md,
                                            int index) {
  const bool offset_16 = HAS_FEATURE(md->version, VERSION_FIELD_FEATURE_OFFSET_16);
  if (md->codepoint_bytes == 2) {
      return offset_16 ? font_cache->offsets_buffer_2_2[index].offset :
                         font_cache->offsets_buffer_2_4[index].offset;
  } else {
      return offset_16 ? font_cache->offsets_buffer_4_2[index].offset :
                         font_cache->offsets_buffer_4_4[index].offset;
  }
}

static uint32_t prv_offset_table_entry_size(const FontMetaData *md) {
  const bool offset_16 = HAS_FEATURE(md->version, VERSION_FIELD_FEATURE_OFFSET_16);
  if (md->codepoint_bytes == 2) {
      return offset_16 ? sizeof(OffsetTableEntry_2_2) : sizeof(OffsetTableEntry_2_4);
  } else {
      return offset_16 ? sizeof(OffsetTableEntry_4_2) : sizeof(OffsetTableEntry_4_4);
  }
}

static int prv_offset_table_get_id(const FontMetaData *md, Codepoint codepoint) {
  if (FONT_VERSION(md->version) == FONT_VERSION_1) {
    return (1);
  } else {
    return prv_font_hash(codepoint, md->hash_table_size);
  }
}

static int prv_load_offset_table(Codepoint codepoint, FontCache *font_cache,
                                 const FontResource *font_res) {
  const int table_id = prv_offset_table_get_id(&font_res->md, codepoint);
  if (table_id == font_cache->offset_table_id) {
    return font_cache->offset_table_size;
  }

  size_t num_bytes, offset, num_entries;
  const uint8_t version = FONT_VERSION(font_res->md.version);
  if (version == FONT_VERSION_1) {
    offset = s_font_md_size[FONT_VERSION_1];
    num_bytes = font_res->md.number_of_glyphs * prv_offset_table_entry_size(&font_res->md);
    num_entries = font_res->md.number_of_glyphs;

    PBL_ASSERTN(num_bytes <= sizeof(font_cache->offsets_buffer_2_2));
  } else {
    FontHashTableEntry table_entry;
    Codepoint hash_entry_offset = s_font_md_size[version] + table_id * sizeof(FontHashTableEntry);
    // find which bucket the codepoint was put into TODO: cache hash table?
    PBL_LOG_D_DBG(LOG_DOMAIN_TEXT, "HTE read: table_id:%d, cp:%"PRIx32", offset:%"PRIx32,
              table_id, codepoint, hash_entry_offset);

    SYS_PROFILER_NODE_START(text_render_flash);
    sys_resource_load_range(font_res->app_num, font_res->resource_id, hash_entry_offset,
                            (uint8_t*)&table_entry, sizeof(FontHashTableEntry));
    SYS_PROFILER_NODE_STOP(text_render_flash);

    offset = s_font_md_size[version] +
              (sizeof(FontHashTableEntry) * font_res->md.hash_table_size) + table_entry.offset;
    num_bytes = table_entry.count * prv_offset_table_entry_size(&font_res->md);
    num_entries = table_entry.count;

    PBL_ASSERTN(num_bytes <= sizeof(font_cache->offsets_buffer_4_4));
  }

  PBL_LOG_D_DBG(LOG_DOMAIN_TEXT, "HT read: offset: %zx, bytes: %zu", offset,
            num_bytes);
  SYS_PROFILER_NODE_START(text_render_flash);
  sys_resource_load_range(font_res->app_num, font_res->resource_id, offset,
                          (uint8_t *)font_cache->offsets_buffer_4_4, num_bytes);
  SYS_PROFILER_NODE_STOP(text_render_flash);
  font_cache->offset_table_id = table_id;
  font_cache->offset_table_size = num_entries;

  return num_entries;
}

static uint32_t prv_get_cache_key(const FontResource *font_res, Codepoint codepoint) {
  // Ideally we'd be able to use the full app_num, resource_id and codepoint combined into a key
  // in a unique matter, but unfortunately there aren't enough bits. Note that this value needs to
  // be unique, there's no collision handling and if one does occur you'll just end up reading the
  // wrong metadata. Luckily we don't need to store all the bits due to assumptions we can make.

  // We know that for a given FontCache we'll only use a combination of fonts from the running app
  // and system fonts and we'll never use custom fonts from two different app banks at the same
  // time. This means we only need a single bit to store whether the font is for the system bank
  // (bank 0) or an app bank (bank > 0).

  // resource_id is technically a full 32-bit id but in practice it's much smaller. The firmware
  // only uses 400~ unique resources at the time of writing so 14 bits (16384 resources) should be
  // enough.

  // Therefore our key layout becomes:
  // is_app:1
  // resource_id:14
  // codepoint:17

  const bool is_app = (font_res->app_num != 0);

  return (is_app ? 1 << 31 : 0) |
         ((font_res->resource_id << 17) |
         (codepoint & 0x0001FFFF));
}

static uint32_t prv_get_glyph_table_offset(FontCache *font_cache, Codepoint codepoint,
                                           const FontResource *font_res) {
  int min_idx = 0;
  int max_idx = prv_load_offset_table(codepoint, font_cache, font_res);

  uint32_t offset = 0;
  while (max_idx >= min_idx) {
    int mid_idx = (max_idx + min_idx) / 2;

    Codepoint codepoint_at_mid_idx = prv_offset_table_get_codepoint(font_cache, &font_res->md,
                                                                    mid_idx);

    if (codepoint_at_mid_idx < codepoint) {
      min_idx = mid_idx + 1;
    } else if (codepoint_at_mid_idx > codepoint) {
      max_idx = mid_idx - 1;
    } else {
      offset = prv_offset_table_get_offset(font_cache, &font_res->md, mid_idx);
      break;
    }
  }

  return offset;
}

static uint32_t prv_get_glyph_data_offset(Codepoint codepoint, FontCache *font_cache,
                                          const FontResource *font_res) {
  const uint32_t offset = prv_get_glyph_table_offset(font_cache, codepoint, font_res);

  if (offset <= 0) {
    return 0;
  }
  // Compute the offset of the glyph data (relative to the beginning
  // of the font blob).
  //
  // See: docs/reference/formats/font.md
  uint32_t address;
  const uint32_t version = FONT_VERSION(font_res->md.version);
  if (version == FONT_VERSION_1) {
    address = s_font_md_size[FONT_VERSION_1] +
              (font_res->md.number_of_glyphs * prv_offset_table_entry_size(&font_res->md)) +
              (sizeof(uint32_t) * offset);
  } else {
    address = s_font_md_size[version] +
              (sizeof(FontHashTableEntry) * font_res->md.hash_table_size) +
              (prv_offset_table_entry_size(&font_res->md) * font_res->md.number_of_glyphs) +
              offset;
  }

  return address;
}

//! Decode RLE4 decoded glyph data in place.
//! @return g or NULL on error.
#define RLE4_SYMBOL_MASK 0x08
#define RLE4_LENGTH_MASK 0x07
static GlyphData *prv_decompress_glyph_data(GlyphData *g, uint8_t *src) {
  // This RLE4 decompressor expects GlyphData to be formatted like this:
  //  [ <header> | <free space> | <encoded glyph> ]
  // *src is a pointer to the beginning of <encoded glyph>
  //
  // Once decompressed, Glyph Data will be formatted like this:
  //  [ <header> | <decoded glyph> | <free space & remenants of encoded glyph data> ]
  //
  // The glyph is decoded in-place, so obviously, it's imperative that <decoded glyph> and
  // <encoded glyph> not overlap at any time. This is checked by fontgen.py and confirmed
  // by the code below.
  //
  // RLE4 data is encoded as a stream of RLE Units.
  //   0 1 2 3
  //  +-+-+-+-+
  //  |*| Len |     Where * is the encoded symbol [0,1] and 'Len + 1' is the number of symbols
  //  +-+-+-+-+     in the run [1,8]. For example, 1000 expands to '1' and 0100 expands to '00000'
  //
  // RLE Units are packed as pairs -- two to a byte.
  //
  // Decoding is done by expanding the bit patterns into a buffer until we have at least 8 bits of
  // pixels to write.

  PBL_ASSERTN(g);
  PBL_ASSERTN(src > (uint8_t *)g->data);

  uint8_t *dst = (uint8_t *)g->data;

  unsigned total_pixels_decoded = 0;
  unsigned num_rle_units = g->header.num_rle_units;

  // Decoded pixel buffer. At least 16 bits (to hold 2 decoded RLE4s)
  uint16_t buf = 0;
  int8_t buf_num_bits = 0;

  PBL_ASSERTN(num_rle_units <= (CACHE_GLYPH_SIZE * RLE4_UNITS_PER_BYTE));

  while (num_rle_units) {
    PBL_ASSERTN(src < &((uint8_t *)g->data)[CACHE_GLYPH_SIZE]);
    uint8_t rle_unit_pair = *src++;

    for (unsigned int i = 0; i < RLE4_UNITS_PER_BYTE; ++i) {
      if (!num_rle_units) {
        break; // Handle a padded, odd number of rle units
      }

      // Number of bits in this run
      uint8_t length = (rle_unit_pair & RLE4_LENGTH_MASK) + 1;

      // Symbol of this run. We don't need to generate a pattern of 0s. ;-)
      if (rle_unit_pair & RLE4_SYMBOL_MASK) {
        uint8_t pattern = ((1 << length) - 1);  // List of 'length' 1s
        buf |= (pattern << buf_num_bits);
      }
      buf_num_bits += length;
      total_pixels_decoded += length;

      // Store 8 bits worth of pixels
      if (buf_num_bits >= 8) {
        PBL_ASSERTN(dst < src);
        *dst++ = (buf & 0xFF);
        buf >>= 8;
        buf_num_bits -= 8;
      }

      // Now process the second nibble
      rle_unit_pair >>= 4;
      num_rle_units--;
    }
  }

  // Flush out any remaining pixels
  while (buf_num_bits > 0) {
    PBL_ASSERTN(dst < &((uint8_t *)g->data)[CACHE_GLYPH_SIZE]);
    *dst++ = (buf & 0xFF);
    buf >>= 8;
    buf_num_bits -= 8;
  }

  // Fix-up the height to reflect the bit pattern instead of the number of RLE units.
  if (g->header.width_px) {
    g->header.height_px = total_pixels_decoded / g->header.width_px;
  }

  return g;
}

static bool prv_load_glyph_bitmap(Codepoint codepoint, const FontResource *font_res,
                                  LineCacheData *data) {
  GlyphData *g = &data->glyph_data;

  const size_t bitmap_offset = (FONT_VERSION(font_res->md.version) == FONT_VERSION_1) ?
                               sizeof(GlyphHeaderDataV1) : sizeof(GlyphHeaderData);
  const uint32_t bitmap_addr = data->resource_offset + bitmap_offset;

  size_t glyph_size_bytes;

  // Handle RLE4 compressed glyphs. header.height_px has been 'borrowed' to mean the number of
  // 4-bit RLE units used to encode the glyph. We determine the height by decoding the number of
  // bits and then dividing by the width. glyph.height_px must be updated!
  if (HAS_FEATURE(font_res->md.version, VERSION_FIELD_FEATURE_RLE4)) {
    // Two RLE4 units per byte. Round up to the next whole byte
    glyph_size_bytes = (g->header.height_px + (RLE4_UNITS_PER_BYTE - 1)) / RLE4_UNITS_PER_BYTE;
  } else {
    // Number of bytes, make sure we round up to the next whole byte
    glyph_size_bytes = ((g->header.width_px * g->header.height_px) + (8 - 1)) / 8;
  }

  PBL_ASSERT(glyph_size_bytes <= CACHE_GLYPH_SIZE,
             "text codepoint %"PRIx32" is %zu bytes, overflowing %zu max size", codepoint,
             glyph_size_bytes, CACHE_GLYPH_SIZE);

  if (glyph_size_bytes) {
    uint8_t *target;
    PBL_LOG_D_DBG(LOG_DOMAIN_TEXT, "GD read: cp: %"PRIx32", res_bank: %"PRIu32", res_id: %"PRIu32", "
              "offset: %"PRIx32", bytes: %zu",
              codepoint, font_res->app_num, font_res->resource_id, bitmap_addr, glyph_size_bytes);
    if (HAS_FEATURE(font_res->md.version, VERSION_FIELD_FEATURE_RLE4)) {
      // Load the glyph data at the end of the buffer
      target = &((uint8_t *)g->data)[CACHE_GLYPH_SIZE - glyph_size_bytes];
    } else {
      target = (uint8_t *)g->data;
    }
    SYS_PROFILER_NODE_START(text_render_flash);
    size_t num_bytes_loaded = sys_resource_load_range(font_res->app_num, font_res->resource_id,
                                                      bitmap_addr, target, glyph_size_bytes);
    SYS_PROFILER_NODE_STOP(text_render_flash);
    if (glyph_size_bytes && !num_bytes_loaded) {
      PBL_LOG_WRN("Failed to load glyph bitmap from resources; cp: %"PRIx32", addr: %"PRIx32,
              codepoint, bitmap_addr);
      return false;
    }

    if (HAS_FEATURE(font_res->md.version, VERSION_FIELD_FEATURE_RLE4)) {
      SYS_PROFILER_NODE_START(text_render_compress);
      g = prv_decompress_glyph_data(g, target);
      SYS_PROFILER_NODE_STOP(text_render_compress);
    }
  }

  data->is_bitmap_loaded = true;
  return true;
}

static ALWAYS_INLINE const GlyphData *prv_get_glyph_metadata_from_spi(Codepoint codepoint,
                                                                      FontCache *font_cache,
                                                                      const FontResource *font_res,
                                                                      bool need_bitmap) {
  const uint32_t cache_key = prv_get_cache_key(font_res, codepoint);
  LineCacheData *cached = NULL;

  // If we don't have bitmap caching, we have a single glyph_buffer that contains the last used
  // glyph. If this matches the glyph we're looking for right now, that's what we want to use.
  // Potentially this also has the bitmap loaded already.
  if (font_cache->glyph_buffer_key == cache_key) {
    cached = (LineCacheData *)(font_cache->glyph_buffer);
  }
  PBL_LOG_D_DBG(LOG_DOMAIN_TEXT, "looking up cp: %"PRIx32", key:%"PRIx32,
            codepoint, cache_key);

  // If the glyph_buffer doesn't match this glyph, or we have bitmap caching, check the
  // keyed_circular_cache for this glyph.
  if (!cached) {
    cached = keyed_circular_cache_get(&font_cache->line_cache, cache_key);
    // The keyed_circular_cache entry cannot store the bitmap.
    // Therefore, we need to copy the matched entry into `glyph_buffer` which does have the space
    // to store the bitmap.
    if (cached) {
      memcpy(font_cache->glyph_buffer, cached, sizeof(LineCacheData));
      font_cache->glyph_buffer_key = cache_key;
      // Point `cached` at the glyph buffer.
      cached = (LineCacheData *)(font_cache->glyph_buffer);
      cached->is_bitmap_loaded = false;
    }
  }

  if (cached) {
    if (cached->resource_offset == 0) {
      // missing character
      return NULL;
    }
    if (need_bitmap &&
        !cached->is_bitmap_loaded &&
        !prv_load_glyph_bitmap(codepoint, font_res, cached)) {
      return NULL;
    }
    return &cached->glyph_data;
  }

  // We missed the cache, so we need to build a new cache entry.
  LineCacheData *data = &font_cache->cache_data_scratch;
  data->is_bitmap_loaded = false;
  data->resource_offset = prv_get_glyph_data_offset(codepoint, font_cache, font_res);
  GlyphData *g = &data->glyph_data;

  if (data->resource_offset == 0) {
    PBL_LOG_D_DBG(LOG_DOMAIN_TEXT, "offset for cp: %"PRIx32" is NULL", codepoint);
    // Put the missing character into our cache so we don't waste time looking for it again
    keyed_circular_cache_push(&font_cache->line_cache, cache_key, data);
    return NULL;
  }

  size_t num_bytes_loaded;
  if (FONT_VERSION(font_res->md.version) == FONT_VERSION_1) {
    GlyphHeaderDataV1 header;
    PBL_LOG_D_DBG(LOG_DOMAIN_TEXT, "LGMD READ: offset: %"PRIx32", bytes: %zu",
              data->resource_offset, sizeof(header));
    SYS_PROFILER_NODE_START(text_render_flash);
    num_bytes_loaded = sys_resource_load_range(font_res->app_num, font_res->resource_id,
                                               data->resource_offset, (uint8_t *)&header,
                                               sizeof(header));
    SYS_PROFILER_NODE_STOP(text_render_flash);

    // convert to a GlyphHeaderData struct
    memcpy(&g->header, &header, sizeof(GlyphHeaderData));
    g->header.horiz_advance = header.horiz_advance;
  } else {
    PBL_LOG_D_DBG(LOG_DOMAIN_TEXT, "GMD read: cp: %"PRIx32", offset: %"PRId32", bytes: %zu", codepoint,
              data->resource_offset, sizeof(GlyphHeaderData));
    SYS_PROFILER_NODE_START(text_render_flash);
    num_bytes_loaded = sys_resource_load_range(font_res->app_num, font_res->resource_id,
                                               data->resource_offset, (uint8_t *)&g->header,
                                               sizeof(GlyphHeaderData));
    SYS_PROFILER_NODE_STOP(text_render_flash);
  }

  if (!num_bytes_loaded) {
    PBL_LOG_WRN("Failed to load glyph metadata from resources; cp: %"PRIx32", offset: %"PRIx32,
            codepoint, data->resource_offset);
    return NULL;
  }

  // Copy the info into the glyph_buffer.
  // This must be done _before_ loading the bitmap, otherwise loading the bitmap may modify the
  // metadata! We will use `glyph_buffer` as the final data, and leave `data` as the uncooked
  // version, that way we can push `data` into the circular cache.
  memcpy(font_cache->glyph_buffer, data, sizeof(LineCacheData));
  font_cache->glyph_buffer_key = cache_key;

  LineCacheData *final_data = (LineCacheData *)(font_cache->glyph_buffer);

  if (need_bitmap &&
      !prv_load_glyph_bitmap(codepoint, font_res, final_data)) {
    return NULL;
  }

  // We push `data`, which will be cooked data if the bitmap is stored along with it, or
  // the uncooked data if it's not. In reality, this only matters to compressed glyphs, since
  // compressed glyphs are the only case where the metadata gets modified.
  // The only time the data is cooked is when loading a bitmap and the glyph is compressed, in
  // which case the `num_rle_units` field is turned back into `height_px`.
  keyed_circular_cache_push(&font_cache->line_cache, cache_key, data);

  // We return `final_data` though, because that has the actual metadata info that needs to be
  // used.
  return &final_data->glyph_data;
}

static void prv_check_font_cache(FontCache *font_cache, const FontResource *font_res) {
  // Invalidate the offset table
  if (font_cache->cached_font != font_res) {
    font_cache->offset_table_id = -1;
    font_cache->cached_font = font_res;
  }
}

static bool prv_load_font_res(ResAppNum app_num, uint32_t resource_id, FontResource *font_res,
                              bool is_extended) {
  font_res->resource_id = resource_id;
  font_res->app_num = app_num;

  if (resource_id != RESOURCE_ID_FONT_FALLBACK_INTERNAL &&
      !sys_resource_is_valid(app_num, resource_id)) {
    if (!is_extended) {
      PBL_LOG_WRN("Invalid text resource id %"PRId32, resource_id);
    }
    return false;
  }

  if (app_num == SYSTEM_APP && !sys_resource_get_and_cache(app_num, resource_id)) {
    return false;
  }

  PBL_LOG_D_DBG(LOG_DOMAIN_TEXT, "FMD read: bytes:%d", (int)sizeof(FontMetaDataV3));

  FontMetaDataV3 header;
  SYS_PROFILER_NODE_START(text_render_flash);
  uint32_t bytes_read = sys_resource_load_range(app_num, resource_id, 0,
                                                (uint8_t*)&header, sizeof(FontMetaDataV3));
  SYS_PROFILER_NODE_STOP(text_render_flash);
  if (bytes_read != sizeof(FontMetaDataV3)) {
    PBL_LOG_ERR("Tried to load resource too small to have metadata for res %"PRId32,
        resource_id);
    return false;
  }

  memcpy(&font_res->md, &header, sizeof(FontMetaData));

  switch (header.version) {
    case FONT_VERSION_1:
      // no hash table, no variable codepoint size, no feature bits
      font_res->md.hash_table_size = 0;
      // Version 1 fonts do use 16 bit offsets and 16 bit codepoints. This simplifies the code above
      font_res->md.codepoint_bytes = 2;
      font_res->md.version |= VERSION_FIELD_FEATURE_OFFSET_16;
      break;
    case FONT_VERSION_2:
      break;
    case FONT_VERSION_3:
      // Make sure that the font header is internally consistent
      PBL_ASSERTN(header.size == sizeof(FontMetaDataV3));

      // HACK alert: Copy the feature bits to the top two bits of the header version.
      if (header.features & FEATURE_OFFSET_16) {
        font_res->md.version |= VERSION_FIELD_FEATURE_OFFSET_16;
      }
      if (header.features & FEATURE_RLE4) {
        font_res->md.version |= VERSION_FIELD_FEATURE_RLE4;
      }
      break;
    default:
      PBL_LOG_ERR("Unknown font resource version %"PRIu8, header.version);
      return false;
  }

  return true;
}

// @param owner_out if non-NULL, receives the FontInfo owning the returned resource. It is a font
// other than font_info only when the emoji font takes over.
static const FontResource *prv_font_res_for_codepoint(Codepoint codepoint,
                                                      const FontInfo *font_info,
                                                      const FontInfo **owner_out) {
  const FontInfo *owner = font_info;
  const FontResource *font_res = &font_info->base;

  if (!codepoint_is_latin(codepoint) &&
      !codepoint_is_emoji(codepoint) &&
      !codepoint_is_special(codepoint) &&
      font_info->extended) {
    // Latin & emoji codepoints are in base, others are in extension
    font_res = &font_info->extension;
  } else if (codepoint_is_emoji(codepoint) &&
             font_info->base.app_num == SYSTEM_APP) {
    // Size against the base: that is the baseline emoji glyphs get aligned to when drawn
    FontInfo *emoji_font = fonts_get_system_emoji_font_for_size(font_info->base.md.max_height);
    if (emoji_font) {
      owner = emoji_font;
      font_res = &emoji_font->base;
    }
  }

  if (owner_out) {
    *owner_out = owner;
  }
  return font_res;
}

static void prv_resource_changed_callback(void *data) {
  FontInfo *font_info = (FontInfo *)data;
  font_info->loaded = false;
  font_info->extended = false;
}

///////////////////////////
// Public API
bool text_resources_init_font(ResAppNum app_num, uint32_t font_resource,
                              uint32_t extended_resource, FontInfo *font_info) {
  // load the base of the font or bail
  if (!font_resource ||
      !prv_load_font_res(app_num, font_resource, &font_info->base, false /* is_extended */)) {
    return false;
  }
  // look for an extension font and load it
  if (extended_resource) {
    // if you want 3rd party apps to use extended fonts, you'll have to unwatch when they unload
    // and create a syscall for resource_watch
    PBL_ASSERTN(app_num == SYSTEM_APP);
    if (font_info->extension_changed_cb == NULL) {
      font_info->extension_changed_cb = resource_watch(app_num, extended_resource,
                                                       prv_resource_changed_callback, font_info);
    }
    font_info->extended = prv_load_font_res(app_num, extended_resource, &font_info->extension,
                                            true /* is_extended */);
  }

  font_info->max_height = MAX(font_info->extension.md.max_height, font_info->base.md.max_height);
  font_info->loaded = true;
  return true;
}

// Leaf glyph lookup against a single font. This NEVER consults the fallback font, which is what
// structurally bounds the fallback to depth 1: the fallback font is looked up with this helper, so
// it can never trigger a further fallback and no recursion or cycle is possible.
// Classification only picks which of the font's own resources to try FIRST: on a miss we retry the
// font's other own resource, so a codepoint whose class routes it to base/extension is still found
// in the other one. Skipped when the emoji font took over (owner != font_info).
// @param owner_out if non-NULL, receives the FontInfo the glyph was looked up in
static const GlyphData *prv_get_glyph_in_font(FontCache *font_cache, Codepoint codepoint,
                                              FontInfo *font_info, bool need_bitmap,
                                              const FontInfo **owner_out) {
  const FontInfo *owner = font_info;
  const FontResource *font_res = prv_font_res_for_codepoint(codepoint, font_info, &owner);
  prv_check_font_cache(font_cache, font_res);
  const GlyphData *data = prv_get_glyph_metadata_from_spi(codepoint, font_cache, font_res,
                                                          need_bitmap);
  // Routed resource missed: try the font's other own resource. Classification is a lookup-order
  // hint, not a hard partition. Skipped when the emoji font took over.
  if (!data && owner == font_info) {
    const FontResource *other = (font_res == &font_info->base)
        ? (font_info->extended ? &font_info->extension : NULL)
        : &font_info->base;
    if (other) {
      prv_check_font_cache(font_cache, other);
      data = prv_get_glyph_metadata_from_spi(codepoint, font_cache, other, need_bitmap);
    }
  }
  if (owner_out) {
    *owner_out = owner;
  }
  return data;
}

// Convert any "styled" codepoints into their correspnding plain codepoints. Currently known styled
// codepoints are in these blocks:
//
// - Mathematical Alphanumeric Symbols
// - Letterlike Symbols
// - Halfwidth and Fullwidth Forms
//
// For example:
//
// - All mathematical alphanumeric symbols will be converted to the corresponding Latin
// alphanumeric codepoints.
// - Any letterlike symbol that directly corresponds to a Latin alphanumeric will be mapped to
// that codepoint.
// - All "fullwidth Latin" codepoints will be mapped to plain Latin codepoints.
//
// If it is not a known styled codepoint, then the same codepoint will be returned unmodified.
static Codepoint prv_normalize_styled_codepoint(Codepoint codepoint) {
  // Fullwidth Latin codepoints; map to 0x21-0x7e
  if (codepoint >= 0xff01 && codepoint <= 0xff5e) {
    return codepoint - 0xff00 + 0x20;
  }

  switch (codepoint) {
    // Reserved codepoints, which are exceptions in the ranges below which should not be mapped:
    case 0x1d49d:
    case 0x1d4a0:
    case 0x1d4a1:
    case 0x1d4a3:
    case 0x1d4a4:
    case 0x1d4a7:
    case 0x1d4a8:
    case 0x1d4ad:
    case 0x1d506:
    case 0x1d50b:
    case 0x1d50c:
    case 0x1d515:
    case 0x1d51d:
    case 0x1d53a:
    case 0x1d53f:
    case 0x1d545:
    case 0x1d547:
    case 0x1d548:
    case 0x1d549:
    case 0x1d455:
    case 0x1d4ba:
    case 0x1d4bc:
    case 0x1d4c4:
      return codepoint;

    // Individual letters from the Letterlike Symbols block
    // todo: would a lookup table be better for performance & flash space for this block?
    case 0x2102:
      return 'C';
    case 0x210a:
      return 'g';
    case 0x210b:
    case 0x210c:
      return 'H';
    case 0x2110:
    case 0x2111:
      return 'I';
    case 0x2112:
      return 'L';
    case 0x2113:
      return 'l';
    case 0x2115:
      return 'N';
    case 0x2118:
    case 0x2119:
      return 'P';
    case 0x211a:
      return 'Q';
    case 0x211b:
    case 0x211c:
    case 0x211d:
      return 'R';
    case 0x2124:
    case 0x2128:
      return 'Z';
    case 0x212a:
      return 'K';
    case 0x212b:
      return 'B';
    case 0x212d:
      return 'C';
    case 0x212f:
      return 'e';
    case 0x2130:
      return 'E';
    case 0x2131:
      return 'F';
    case 0x2133:
      return 'M';
    case 0x2134:
      return 'o';
    case 0x2145:
      return 'D';
    case 0x2146:
      return 'd';
    case 0x2147:
      return 'e';
    case 0x2148:
      return 'i';
    case 0x2149:
      return 'j';
  }

  // A-Z ranges starting with a styled capital A
  const Codepoint capital_starts[] = {    
    0x1d400, 0x1d434, 0x1d468, 0x1d49c, 0x1d4d0, 0x1d504, 0x1d538, 0x1d56c, 0x1d5a0, 0x1d5d4,
    0x1d608, 0x1d63c, 0x1d680
  };
  for (unsigned int i = 0; i < ARRAY_LENGTH(capital_starts); i++) {
    if (codepoint >= capital_starts[i] && codepoint < capital_starts[i] + 26) {
      return codepoint - capital_starts[i] + 'A';
    }
  }

  // a-z ranges starting with a styled small a
  const Codepoint small_starts[] = {
    0x1d41a, 0x1d44e, 0x1d482, 0x1d4b6, 0x1d4ea, 0x1d51e, 0x1d552, 0x1d586, 0x1d5ba, 0x1d5ee,
    0x1d622, 0x1d656, 0x1d68a
  };
  for (unsigned int i = 0; i < ARRAY_LENGTH(small_starts); i++) {
    if (codepoint >= small_starts[i] && codepoint < small_starts[i] + 26) {
      return codepoint - small_starts[i] + 'a';
    }
  }

  // 0-9 ranges starting with a styled zero 0
  const Codepoint digit_starts[] = {
    0x1d6a8, 0x1d6e2, 0x1d71c, 0x1d756, 0x1d790
  };
  for (unsigned int i = 0; i < ARRAY_LENGTH(digit_starts); i++) {
    if (codepoint >= digit_starts[i] && codepoint < digit_starts[i] + 10) {
      return codepoint - digit_starts[i] + '0';
    }
  }

  // Nothing matched, so return it unmodified.
  return codepoint;
}

// A substitute font bakes top_offset against its own baseline (== base max_height for PBF), so
// drop its glyphs onto ours. Our own base/extension are already aligned; never shift up.
static int16_t prv_baseline_adjust(const FontInfo *font_info, const FontInfo *owner) {
  if (owner == NULL || owner == font_info) {
    return 0;
  }
  return MAX(0, (int16_t)font_info->base.md.max_height - (int16_t)owner->base.md.max_height);
}

static const GlyphData *prv_get_glyph(FontCache *font_cache, Codepoint codepoint,
                                      FontInfo *font_info, bool need_bitmap,
                                      int16_t *baseline_adjust_out) {
  if (baseline_adjust_out) {
    *baseline_adjust_out = 0;
  }

  if (!font_info->loaded) {
    sys_font_reload_font(font_info);
  }

  const FontInfo *owner = NULL;

  // (a) Requested codepoint in the primary font.
  const GlyphData *data = prv_get_glyph_in_font(font_cache, codepoint, font_info, need_bitmap,
                                                &owner);
  if (data) {
    if (baseline_adjust_out) {
      *baseline_adjust_out = prv_baseline_adjust(font_info, owner);
    }
    return data;
  }

  // (b) Missing from the primary font: retry the SAME codepoint once against the system fallback
  //     font, which carries a wider character set. The fallback is a process-global, not per-font
  //     state, so it lives here in the renderer rather than on FontInfo (whose layout is frozen
  //     applib ABI). It is fetched via the syscall directly (the NULL key is the fallback font),
  //     keeping text_resources independent of the fonts.c config layer. prv_get_glyph_in_font does
  //     not recurse into the fallback, so at most one extra font is consulted with no loop. Skip
  //     when the primary already is the fallback font.
  FontInfo *fallback = sys_font_get_system_font(NULL);
  if (fallback != NULL && fallback != font_info) {
    if (!fallback->loaded) {
      sys_font_reload_font(fallback);
    }
    data = prv_get_glyph_in_font(font_cache, codepoint, fallback, need_bitmap, &owner);
    if (data) {
      // Baseline is still the caller's font, not the fallback's
      if (baseline_adjust_out) {
        *baseline_adjust_out = prv_baseline_adjust(font_info, owner);
      }
      return data;
    }
  }

  // (c) Absent everywhere, and if the codepoint is a styled Latin codepoint: normalize it to the
  // plain Latin codepoint and try again, so that it will at least be readable if there isn't a
  // glyph for the styled codepoint.
  Codepoint normalized = prv_normalize_styled_codepoint(codepoint);
  if (normalized != codepoint) {
    data = prv_get_glyph_in_font(font_cache, normalized, font_info, need_bitmap, &owner);
    if (data) {
      if (baseline_adjust_out) {
        *baseline_adjust_out = prv_baseline_adjust(font_info, owner);
      }
      return data;
    }
  }

  // (d) Absent everywhere: fall back to the wildcard (square box) then ' ', looked up in the
  //     PRIMARY font, so a glyph missing from every font still yields the primary font's box or
  //     space. We use the wildcard codepoint from the base font in case the extension pack has
  //     been deleted. The fallback font's own wildcard is deliberately never used: a single
  //     missing-glyph box is the correct indicator.
  const Codepoint substitutes[] = { font_info->base.md.wildcard_codepoint, ' ' };
  for (unsigned int i = 0; i < ARRAY_LENGTH(substitutes); i++) {
    data = prv_get_glyph_in_font(font_cache, substitutes[i], font_info, need_bitmap, &owner);
    if (data) {
      if (baseline_adjust_out) {
        *baseline_adjust_out = prv_baseline_adjust(font_info, owner);
      }
      return data;
    }
  }
  PBL_LOG_WRN("failed to load glyph, fallback, or wildcard");
  return NULL;
}

int8_t text_resources_get_glyph_horiz_advance(FontCache *font_cache, const Codepoint codepoint,
                                              FontInfo *font_info) {
  // Metadata only: measuring must not pay the deep bitmap load; render pre-loads it in walk_line().
  const GlyphData *g = prv_get_glyph(font_cache, codepoint, font_info, false /* need_bitmap */,
                                     NULL);
  if (!g) {
    return 0;
  }
  return g->header.horiz_advance;
}

const GlyphData *text_resources_get_glyph(FontCache *font_cache, const Codepoint codepoint,
                                          FontInfo *font_info, int16_t *baseline_adjust_out) {
  return prv_get_glyph(font_cache, codepoint, font_info, true /* need_bitmap */,
                       baseline_adjust_out);
}
