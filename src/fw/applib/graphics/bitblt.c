/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "bitblt.h"
#include "bitblt_private.h"

#include "pbl/util/math.h"

#include "util/bitset.h"

void bitblt_into_1bit_setup_compositing_mode(GCompOp *compositing_mode,
                                             GColor tint_color) {
  if ((*compositing_mode == GCompOpTint) || (*compositing_mode == GCompOpTintLuminance)) {
    // Force our interpretation of the tint color to be black, white, or clear
    tint_color = gcolor_get_bw(tint_color);
    if (gcolor_equal(tint_color, GColorBlack)) {
      *compositing_mode = GCompOpAnd;
    } else if (gcolor_equal(tint_color, GColorWhite)) {
      *compositing_mode = GCompOpSet;
    }
  }
}

void bitblt_bitmap_into_bitmap_tiled_1bit_to_1bit(GBitmap* dest_bitmap,
                                                  const GBitmap* src_bitmap, GRect dest_rect,
                                                  GPoint src_origin_offset,
                                                  GCompOp compositing_mode,
                                                  GColor tint_color) {
  bitblt_into_1bit_setup_compositing_mode(&compositing_mode, tint_color);

  const int8_t dest_begin_x = (dest_rect.origin.x / 32);
  const uint32_t * const dest_block_x_begin = ((uint32_t*)dest_bitmap->addr) + dest_begin_x;
  const int dest_row_length_words = (dest_bitmap->row_size_bytes / 4);
  // The number of bits between the beginning of dest_block and the beginning of the nearest 32-bit block:
  const uint8_t dest_shift_at_line_begin = (dest_rect.origin.x % 32);

  const uint32_t * const src_block_x_begin =
      ((uint32_t*)src_bitmap->addr) +
      ((src_bitmap->bounds.origin.x + (src_origin_offset.x % src_bitmap->bounds.size.w)) / 32);
  const int src_row_length_words = (src_bitmap->row_size_bytes / 4);
  const uint8_t src_shift_at_line_begin = ((src_bitmap->bounds.origin.x + src_origin_offset.x) % 32);
  const uint8_t src_bits_left_at_line_begin = MIN(32 - src_shift_at_line_begin,
                                                  MIN(dest_rect.size.w + src_origin_offset.x,
                                                  src_bitmap->bounds.size.w));

  // how many 32-bit blocks do we need to bitblt on this row:
  const int16_t dest_end_x = (dest_rect.origin.x + dest_rect.size.w);
  const uint8_t num_dest_blocks_per_row = (dest_end_x / 32) + ((dest_end_x % 32) ? 1 : 0) - dest_begin_x;

  // The bitblt loops:
  const int16_t dest_y_end = dest_rect.origin.y + dest_rect.size.h;
  int16_t src_y = src_bitmap->bounds.origin.y + src_origin_offset.y;
  for (int16_t dest_y = dest_rect.origin.y; dest_y != dest_y_end; ++dest_y, ++src_y) {
    // Wrap-around source bitmap vertically:
    if (src_y >= src_bitmap->bounds.origin.y + src_bitmap->bounds.size.h) {
      src_y = src_bitmap->bounds.origin.y;
    }

    int8_t src_dest_shift = 32 + dest_shift_at_line_begin - src_shift_at_line_begin;
    uint8_t dest_shift = dest_shift_at_line_begin;
    int16_t row_bits_left = dest_rect.size.w;
    uint32_t *dest_block = (uint32_t *)dest_block_x_begin + (dest_y * dest_row_length_words);
    uint32_t * const src_block_begin = (uint32_t *)src_block_x_begin + (src_y * src_row_length_words);
    uint32_t *src_block = src_block_begin;
    uint32_t src = *src_block;
    rotl32(src, src_dest_shift);
    uint8_t src_bits_left = src_bits_left_at_line_begin;

    const uint32_t *dest_block_end = dest_block + num_dest_blocks_per_row;
    const uint32_t *src_block_end = src_block + src_row_length_words;

    while (dest_block != dest_block_end) {

      const uint8_t number_of_bits = MIN(32 - dest_shift, MIN(row_bits_left, src_bits_left));
      const uint32_t mask_outer_bit = ((number_of_bits < 31) ? (1 << number_of_bits) : 0);
      const uint32_t mask = ((mask_outer_bit - 1) << dest_shift);

      switch (compositing_mode) {
        case GCompOpClear:
          *(dest_block) &= ~(mask & src);
          break;

        case GCompOpSet:
          *(dest_block) |= mask & ~src;
          break;

        case GCompOpOr:
          *(dest_block) |= mask & src;
          break;

        case GCompOpAnd:
          *(dest_block) &= ~mask | src;
          break;

        case GCompOpAssignInverted:
          *(dest_block) ^= mask & (~src ^ *(dest_block));
          break;

        default:
        case GCompOpAssign:
          // this basically does: masked(dest_bits) = masked(src_bits)
          *(dest_block) ^= mask & (src ^ *(dest_block));
          break;
      }

      dest_shift = (dest_shift + number_of_bits) % 32;
      row_bits_left -= number_of_bits;
      src_bits_left -= number_of_bits;

      if (src_bits_left == 0 && row_bits_left != 0) {
        ++src_block;
        if (src_block == src_block_end) {
          // Wrap-around source bitmap horizontally:
          src_block = src_block_begin;
          src_bits_left = src_bits_left_at_line_begin;
          src_dest_shift = (src_dest_shift + src_bitmap->bounds.size.w) % 32;
        } else {
          src_bits_left = 32; // excessive right edge bits will be masked off eventually
        }
        src = *src_block;
        rotl32(src, src_dest_shift);
        if (dest_shift) {
          continue;
        }
      }

      // Proceed to next dest_block:
      ++dest_block;
    }
  }
}

void bitblt_bitmap_into_bitmap(GBitmap* dest_bitmap, const GBitmap* src_bitmap,
                               GPoint dest_offset, GCompOp compositing_mode, GColor8 tint_color) {
  GRect dest_rect = { dest_offset, src_bitmap->bounds.size };
  grect_clip(&dest_rect, &dest_bitmap->bounds);

  GBitmap src_clipped_bitmap = *src_bitmap;
  src_clipped_bitmap.bounds.origin = (GPoint) {
    src_bitmap->bounds.origin.x + (dest_rect.origin.x - dest_offset.x),
    src_bitmap->bounds.origin.y + (dest_rect.origin.y - dest_offset.y)
  };

  bitblt_bitmap_into_bitmap_tiled(dest_bitmap, &src_clipped_bitmap, dest_rect,
                                  GPointZero, compositing_mode, tint_color);
}

bool bitblt_compositing_mode_is_noop(GCompOp compositing_mode, GColor tint_color) {
  return (((compositing_mode == GCompOpTint) || (compositing_mode == GCompOpTintLuminance)) &&
          gcolor_is_invisible(tint_color));
}
