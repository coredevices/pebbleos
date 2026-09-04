/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "../bitblt_private.h"

#include <pbl/logging/logging.h>
#include "system/passert.h"
#include "util/graphics.h"
#include "pbl/util/math.h"

#if !defined(__clang__)
#pragma GCC optimize("O2")
#endif

// Size is based on color palette
#define LOOKUP_TABLE_SIZE 64

// Blending lookup table to map from:
//   dd: 2-bit dest luminance dd,
//   ss: src luminance ss,
//   aa: src alpha
// to a final 2-bit luminance
//   result = s_blending_mask_lookup[0b00aaddss]
//        or  s_blending_mask_lookup[(aa << 4) | (dd << 2) | ss]
const GColor8Component g_bitblt_private_blending_mask_lookup[LOOKUP_TABLE_SIZE] = {
  0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 2, 2, 3, 3, 3, 3,
  0, 0, 1, 1, 1, 1, 1, 2, 1, 2, 2, 2, 2, 2, 3, 3,
  0, 1, 1, 2, 0, 1, 2, 2, 1, 1, 2, 3, 1, 2, 2, 3,
  0, 1, 2, 3, 0, 1, 2, 3, 0, 1, 2, 3, 0, 1, 2, 3,
};

void bitblt_bitmap_into_bitmap_tiled_palette_to_8bit(GBitmap* dest_bitmap,
                                                     const GBitmap* src_bitmap,
                                                     GRect dest_rect,
                                                     GPoint src_origin_offset,
                                                     GCompOp compositing_mode,
                                                     GColor8 tint_color) {
  if (!src_bitmap->palette) {
    return;
  }

  // Initialize the tint luminance lookup table if necessary
  GColor8 tint_luminance_lookup_table[GCOLOR8_COMPONENT_NUM_VALUES] = {};
  if (compositing_mode == GCompOpTintLuminance) {
    gcolor_tint_luminance_lookup_table_init(tint_color, tint_luminance_lookup_table);
  }

  const int16_t dest_begin_y = dest_rect.origin.y;
  const int16_t dest_end_y = grect_get_max_y(&dest_rect);
  const int16_t src_begin_y = src_bitmap->bounds.origin.y;
  const int16_t src_end_y = grect_get_max_y(&src_bitmap->bounds);

  const uint8_t src_bpp = gbitmap_get_bits_per_pixel(gbitmap_get_format(src_bitmap));
  const GColor *palette = src_bitmap->palette;

  int16_t src_y = src_begin_y + src_origin_offset.y;
  for (int16_t dest_y = dest_begin_y; dest_y < dest_end_y; ++dest_y, ++src_y) {
    // Wrap-around source bitmap vertically
    if (src_y >= src_end_y) {
      src_y = src_begin_y;
    }

    const GBitmapDataRowInfo dest_row_info = gbitmap_get_data_row_info(dest_bitmap, dest_y);
    uint8_t *dest = dest_row_info.data;
    const int16_t dest_delta_begin_x = MAX(dest_row_info.min_x - dest_rect.origin.x, 0);
    const int16_t dest_begin_x = dest_delta_begin_x ? dest_row_info.min_x : dest_rect.origin.x;
    const int16_t dest_end_x = MIN(grect_get_max_x(&dest_rect), dest_row_info.max_x + 1);
    if (dest_end_x < dest_begin_x) {
      continue;
    }

    const GBitmapDataRowInfo src_row_info = gbitmap_get_data_row_info(src_bitmap, src_y);
    const uint8_t *src = src_row_info.data;
    // This is the initial position that takes into account destination delta shift
    const int16_t src_initial_x = src_bitmap->bounds.origin.x + dest_delta_begin_x;
    const int16_t src_begin_x = MAX(src_row_info.min_x, src_bitmap->bounds.origin.x);
    const int16_t src_end_x = MIN(grect_get_max_x(&src_bitmap->bounds),
                                  src_row_info.max_x + 1);

    int16_t src_x = src_initial_x + src_origin_offset.x;
    for (int16_t dest_x = dest_begin_x; dest_x < dest_end_x; ++dest_x, ++src_x) {
      if (!WITHIN(src_x, src_begin_x, src_end_x - 1)) {
        // Check if content should wrap (under and over) for tiling
        if (!WITHIN(src_x, src_bitmap->bounds.origin.x,
                    grect_get_max_x(&src_bitmap->bounds) - 1)) {
          // keep correct bounds alignment for circular when tiling
          src_x = src_bitmap->bounds.origin.x +
            ((src_x - src_bitmap->bounds.origin.x) % src_bitmap->bounds.size.w);
        } else {
          // Increment source but don't draw
          continue;
        }
      }

      // src points to the info for the row, so y and stride are 0 for raw_image_get_value
      uint8_t cindex = raw_image_get_value_for_bitdepth(src, src_x, 0, 0, src_bpp);
      GColor src_color = palette[cindex];
      GColor dest_color = (GColor) dest[dest_x];
      switch (compositing_mode) {
        case GCompOpAssign:
          dest[dest_x] = src_color.argb;
          break;
        case GCompOpSet:
          dest[dest_x] = gcolor_alpha_blend(src_color, dest_color).argb;
          break;
        case GCompOpTint: {
          GColor actual_color = tint_color;
          actual_color.a = src_color.a;
          dest[dest_x] = gcolor_alpha_blend(actual_color, dest_color).argb;
          break;
        }
        case GCompOpTintLuminance: {
          const GColor actual_color =
            gcolor_perform_lookup_using_color_luminance_and_multiply_alpha(
                src_color, tint_luminance_lookup_table);
          dest[dest_x] = gcolor_alpha_blend(actual_color, dest_color).argb;
          break;
        }
        default:
          PBL_LOG_DBG("OP: %d NYI", (int)compositing_mode);
          return;
      }
    }
  }
}

void bitblt_bitmap_into_bitmap_tiled_8bit_to_8bit(GBitmap *dest_bitmap,
                                                  const GBitmap *src_bitmap,
                                                  GRect dest_rect,
                                                  GPoint src_origin_offset,
                                                  GCompOp compositing_mode,
                                                  GColor8 tint_color) {
  const int16_t dest_begin_y = dest_rect.origin.y;
  const int16_t dest_end_y = grect_get_max_y(&dest_rect);
  const int16_t src_begin_y = src_bitmap->bounds.origin.y;
  const int16_t src_end_y = grect_get_max_y(&src_bitmap->bounds);
  int16_t src_y = src_begin_y + src_origin_offset.y;

  // Default all compositing modes to GCompAssign except for GCompOpSet
  // and GCompOpOr.
  switch (compositing_mode) {
    case GCompOpAssign:
    case GCompOpAssignInverted:
    case GCompOpAnd:
    case GCompOpOr:
    case GCompOpClear: {
      for (int16_t dest_y = dest_begin_y; dest_y < dest_end_y; ++dest_y, ++src_y) {
        // Wrap-around source bitmap vertically
        if (src_y >= src_end_y) {
          src_y = src_begin_y;
        }

        const GBitmapDataRowInfo dest_row_info = gbitmap_get_data_row_info(dest_bitmap, dest_y);
        uint8_t *dest = dest_row_info.data;
        const int16_t dest_delta_begin_x = MAX(dest_row_info.min_x - dest_rect.origin.x, 0);
        const int16_t dest_begin_x = dest_delta_begin_x ? dest_row_info.min_x : dest_rect.origin.x;
        const int16_t dest_end_x = MIN(grect_get_max_x(&dest_rect), dest_row_info.max_x + 1);
        if (dest_end_x < dest_begin_x) {
          continue;
        }

        const GBitmapDataRowInfo src_row_info = gbitmap_get_data_row_info(src_bitmap, src_y);
        const uint8_t *src = src_row_info.data;
        // This is the initial position that takes into account destination delta shift
        const int16_t src_initial_x = src_bitmap->bounds.origin.x + dest_delta_begin_x;
        const int16_t src_begin_x = MAX(src_row_info.min_x, src_bitmap->bounds.origin.x);
        const int16_t src_end_x = MIN(grect_get_max_x(&src_bitmap->bounds),
                                      src_row_info.max_x + 1);

        int16_t src_x = src_initial_x + src_origin_offset.x;
        for (int16_t dest_x = dest_begin_x; dest_x < dest_end_x; ++dest_x, ++src_x) {
          if (!WITHIN(src_x, src_begin_x, src_end_x - 1)) {
            // Check if content should wrap (under and over) for tiling
            if (!WITHIN(src_x, src_bitmap->bounds.origin.x,
                        grect_get_max_x(&src_bitmap->bounds) - 1)) {
              // keep correct bounds alignment for circular when tiling
              src_x = src_bitmap->bounds.origin.x +
                ((src_x - src_bitmap->bounds.origin.x) % src_bitmap->bounds.size.w);
            } else {
              // Increment source but don't draw
              continue;
            }
          }
          dest[dest_x] = src[src_x];
        }
      }
      break;
    }
    case GCompOpTint:
    case GCompOpTintLuminance:
    case GCompOpSet:
    default: {
      // Initialize the tint luminance lookup table if necessary
      GColor8 tint_luminance_lookup_table[GCOLOR8_COMPONENT_NUM_VALUES] = {};
      if (compositing_mode == GCompOpTintLuminance) {
        gcolor_tint_luminance_lookup_table_init(tint_color, tint_luminance_lookup_table);
      }

      for (int16_t dest_y = dest_begin_y; dest_y < dest_end_y; ++dest_y, ++src_y) {
        // Wrap-around source bitmap vertically
        if (src_y >= src_end_y) {
          src_y = src_begin_y;
        }

        const GBitmapDataRowInfo dest_row_info = gbitmap_get_data_row_info(dest_bitmap, dest_y);
        uint8_t *dest = dest_row_info.data;
        const int16_t dest_delta_begin_x = MAX(dest_row_info.min_x - dest_rect.origin.x, 0);
        const int16_t dest_begin_x = dest_delta_begin_x ? dest_row_info.min_x : dest_rect.origin.x;
        const int16_t dest_end_x = MIN(grect_get_max_x(&dest_rect), dest_row_info.max_x + 1);
        if (dest_end_x < dest_begin_x) {
          continue;
        }

        const GBitmapDataRowInfo src_row_info = gbitmap_get_data_row_info(src_bitmap, src_y);
        const uint8_t *src = src_row_info.data;
        // This is the initial position that takes into account destination delta shift
        const int16_t src_initial_x = src_bitmap->bounds.origin.x + dest_delta_begin_x;
        const int16_t src_begin_x = MAX(src_row_info.min_x, src_bitmap->bounds.origin.x);
        const int16_t src_end_x = MIN(grect_get_max_x(&src_bitmap->bounds),
                                      src_row_info.max_x + 1);

        int16_t src_x = src_initial_x + src_origin_offset.x;
        for (int16_t dest_x = dest_begin_x; dest_x < dest_end_x; ++dest_x, ++src_x) {
          if (!WITHIN(src_x, src_begin_x, src_end_x - 1)) {
            // Check if content should wrap (under and over) for tiling
            if (!WITHIN(src_x, src_bitmap->bounds.origin.x,
                        grect_get_max_x(&src_bitmap->bounds) - 1)) {
              // keep correct bounds alignment for circular when tiling
              src_x = src_bitmap->bounds.origin.x +
                (((src_x - src_bitmap->bounds.origin.x)) % src_bitmap->bounds.size.w);
            } else {
              // Increment source but don't draw
              continue;
            }
          }
          GColor src_color = *(GColor8 *) &src[src_x];

          GColor actual_color = src_color;
          if (compositing_mode == GCompOpTint) {
            actual_color = tint_color;
            actual_color.a = src_color.a;
          } else if (compositing_mode == GCompOpTintLuminance) {
            actual_color = gcolor_perform_lookup_using_color_luminance_and_multiply_alpha(
                src_color, tint_luminance_lookup_table);
          }
          dest[dest_x] = gcolor_alpha_blend(actual_color, (GColor8)dest[dest_x]).argb;
        }
      }
      break;
    }
  }
}

void bitblt_bitmap_into_bitmap_tiled_1bit_to_8bit(GBitmap* dest_bitmap,
                                                  const GBitmap* src_bitmap,
                                                  GRect dest_rect,
                                                  GPoint src_origin_offset,
                                                  GCompOp compositing_mode,
                                                  GColor8 tint_color) {
  const int16_t dest_begin_y = dest_rect.origin.y;
  const int16_t dest_end_y = dest_begin_y + dest_rect.size.h;

  int16_t src_y = src_bitmap->bounds.origin.y + src_origin_offset.y;
  for (int16_t dest_y = dest_begin_y; dest_y < dest_end_y; ++dest_y, ++src_y) {
    // Wrap-around source bitmap vertically:
    if (src_y >= src_bitmap->bounds.origin.y + src_bitmap->bounds.size.h) {
      src_y = src_bitmap->bounds.origin.y;
    }

    const GBitmapDataRowInfo dest_row_info = gbitmap_get_data_row_info(dest_bitmap, dest_y);
    uint8_t *dest = dest_row_info.data;
    const int16_t dest_delta_begin_x = MAX(dest_row_info.min_x - dest_rect.origin.x, 0);
    const int16_t dest_begin_x = dest_delta_begin_x ? dest_row_info.min_x : dest_rect.origin.x;
    const int16_t dest_end_x = MIN(grect_get_max_x(&dest_rect), dest_row_info.max_x + 1);
    if (dest_end_x < dest_begin_x) {
      continue;
    }

    const int16_t corrected_src_x =
      src_bitmap->bounds.origin.x + src_origin_offset.x + dest_delta_begin_x;
    const uint32_t * const src_block_x_begin =
      ((uint32_t *)src_bitmap->addr) + corrected_src_x / 32;
    const int src_row_length_words = (src_bitmap->row_size_bytes / 4);
    const uint8_t src_line_start_idx = corrected_src_x % 32;
    const uint8_t src_line_wrap_idx = (src_bitmap->bounds.origin.x + dest_delta_begin_x) % 32;
    const uint8_t src_line_start_end_idx =
      MIN(32, src_bitmap->bounds.size.w + src_line_start_idx - (src_origin_offset.x % 32));
    const uint8_t src_line_wrap_end_idx = MIN(32, src_bitmap->bounds.size.w + src_line_wrap_idx);

    int16_t row_bits_left = dest_rect.size.w;
    uint32_t * const src_block_begin =
        (uint32_t *)src_block_x_begin + (src_y * src_row_length_words);
    uint32_t *src_block = src_block_begin;
    uint32_t src = *src_block;

    uint8_t src_start_idx = src_line_start_idx;
    uint8_t src_end_idx = MIN(src_line_start_end_idx, src_line_start_idx + row_bits_left);
    if (src_start_idx > src_end_idx) {
      continue;
    }

    const uint32_t *src_block_end = src_block_begin + src_row_length_words;

    int16_t dest_x = dest_begin_x;
    while (dest_x < dest_end_x) {
      const uint8_t number_of_bits = src_end_idx - src_start_idx;
      PBL_ASSERTN(number_of_bits <= row_bits_left);

      switch (compositing_mode) {
        case GCompOpClear:
          for (int i = src_start_idx; i < src_end_idx; ++i, ++dest_x) {
            const uint32_t bit = (1 << i);
            const bool set = src & bit;
            if (set) {
              dest[dest_x] = GColorBlack.argb;
            }
          }
          break;

        case GCompOpSet:
          for (int i = src_start_idx; i < src_end_idx; ++i, ++dest_x) {
            const uint32_t bit = (1 << i);
            const bool set = src & bit;
            if (!set) {
              dest[dest_x] = GColorWhite.argb;
            }
          }
          break;

        case GCompOpOr:
          for (int i = src_start_idx; i < src_end_idx; ++i, ++dest_x) {
            const uint32_t bit = (1 << i);
            const bool set = src & bit;
            if (set) {
              dest[dest_x] = GColorWhite.argb;
            }
          }
          break;

        case GCompOpAnd:
          for (int i = src_start_idx; i < src_end_idx; ++i, ++dest_x) {
            const uint32_t bit = (1 << i);
            const bool set = src & bit;
            if (!set) {
              dest[dest_x] = GColorBlack.argb;
            }
          }
          break;

        case GCompOpAssignInverted:
          for (int i = src_start_idx; i < src_end_idx; ++i, ++dest_x) {
            const uint32_t bit = (1 << i);
            const bool set = src & bit;
            dest[dest_x] = (set) ? GColorBlack.argb : GColorWhite.argb;
          }
          break;

        case GCompOpTint:
        case GCompOpTintLuminance:
          for (int i = src_start_idx; i < src_end_idx; ++i, ++dest_x) {
            const uint32_t bit = (1 << i);
            const bool set = src & bit;
            if (!set) {
              dest[dest_x] = tint_color.argb;
            }
          }
          break;

        default:
        case GCompOpAssign:
          for (int i = src_start_idx; i < src_end_idx; ++i, ++dest_x) {
            const uint32_t bit = (1 << i);
            const bool set = src & bit;
            dest[dest_x] = (set) ? GColorWhite.argb : GColorBlack.argb;
          }
          break;
      }

      row_bits_left -= number_of_bits;

      if (row_bits_left != 0) {
        ++src_block;
        if (src_block == src_block_end) {
          // Wrap-around source bitmap horizontally:
          src_block = src_block_begin;

          src_start_idx = src_line_wrap_idx;
          src_end_idx = MIN(src_line_wrap_end_idx, src_start_idx + row_bits_left);
        } else {
          src_start_idx = 0;
          src_end_idx = MIN(32, row_bits_left);
        }
        src = *src_block;
      }
    }
  }
}

void bitblt_bitmap_into_bitmap_tiled(GBitmap* dest_bitmap, const GBitmap* src_bitmap,
                                     GRect dest_rect, GPoint src_origin_offset,
                                     GCompOp compositing_mode, GColor tint_color) {
  if (bitblt_compositing_mode_is_noop(compositing_mode, tint_color)) {
    return;
  }

  GBitmapFormat src_fmt = gbitmap_get_format(src_bitmap);
  // Don't use gbitmap_get_format on dest_bitmap since it's always of known origin.
  // In the case of a Legacy2 app, we have a 1-bit src going into an 8-bit dest and do not
  // want to override the destination's format
  GBitmapFormat dest_fmt = dest_bitmap->info.format;

  if (src_fmt == dest_fmt) {
    switch (src_fmt) {
      case GBitmapFormat1Bit:
        bitblt_bitmap_into_bitmap_tiled_1bit_to_1bit(dest_bitmap, src_bitmap, dest_rect,
                                                     src_origin_offset, compositing_mode,
                                                     tint_color);
        break;
      case GBitmapFormat8Bit:
      case GBitmapFormat8BitCircular:
        bitblt_bitmap_into_bitmap_tiled_8bit_to_8bit(dest_bitmap, src_bitmap, dest_rect,
                                                     src_origin_offset, compositing_mode,
                                                     tint_color);
        break;
      default:
        break;
    }
  } else {
    if (dest_fmt == GBitmapFormat8Bit || dest_fmt == GBitmapFormat8BitCircular) {
      switch (src_fmt) {
        case GBitmapFormat1Bit:
          bitblt_bitmap_into_bitmap_tiled_1bit_to_8bit(dest_bitmap, src_bitmap, dest_rect,
                                                       src_origin_offset, compositing_mode,
                                                       tint_color);
          break;
        case GBitmapFormat1BitPalette:
        case GBitmapFormat2BitPalette:
        case GBitmapFormat4BitPalette:
          bitblt_bitmap_into_bitmap_tiled_palette_to_8bit(dest_bitmap, src_bitmap, dest_rect,
                                                          src_origin_offset, compositing_mode,
                                                          tint_color);
          break;
        // Circular buffer can take this path
        case GBitmapFormat8Bit:
        case GBitmapFormat8BitCircular:
          bitblt_bitmap_into_bitmap_tiled_8bit_to_8bit(dest_bitmap, src_bitmap, dest_rect,
                                                       src_origin_offset, compositing_mode,
                                                       tint_color);
          break;
        default:
          break;
      }
    } else {
      PBL_LOG_DBG("Only blitting to 8-bit supported.");
    }
  }
}
