/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "text_render.h"

#include "gcontext.h"
#include "graphics.h"
#include "process_state/app_state/app_state.h"
#include "system/passert.h"
#include "text_resources.h"
#include "util/bitset.h"
#include "pbl/util/math.h"
#include "applib/fonts/codepoint.h"

#if !defined(__clang__)
#pragma GCC optimize ("O2")
#endif

/// Applies static x/y offsets for Thai combining marks to reduce visual overlap.
/// This is Phase 1 of Thai mark positioning: simple, empirical offsets per mark type.
/// Later phases will use GPOS anchors for per-base-glyph positioning.
struct ThaiMarkOffset {
  uint32_t codepoint;
  int8_t offset_x;
  int8_t offset_y;
};

// Thai combining marks (U+0E35 "ี", U+0E48 "่", etc.) have zero advance width
// but render at the base glyph's position plus left_offset. These static offsets
// reposition common marks to reduce overlap with base consonants.
// Tuned for Sarabun 12/14/17/20px at common bases (ก, ค, ส, ค, etc.)
//
// Phase 1: Basic offsets; Phase 2 will extract per-base anchors from GPOS.
static const struct ThaiMarkOffset THAI_MARK_OFFSETS[] = {
  // U+0E35: ี (vowel i / ee above)
  // Typical wide base (ส, ค, etc.): render -2px left, +1px down
  // Typical narrow base (ก, ข): render -4px left, +1px down
  // Compromise for mixed: -3px left gives best result on most bases.
  { .codepoint = 0x0E35, .offset_x = -3, .offset_y = 1 },

  // U+0E48: ่ (tone mark mai tho / falling tone)
  // Positioned above base; wide bases need different x than narrow.
  // Compromise: -2px left, 0px vertical keeps mark visible.
  { .codepoint = 0x0E48, .offset_x = -2, .offset_y = 0 },

  // U+0E31: ◌ั (tone mark mai tham / short vowel)
  // Similar to U+0E35 but positioned differently; try -2px.
  { .codepoint = 0x0E31, .offset_x = -2, .offset_y = 0 },

  // U+0E49: ◌้ (tone mark mai tri / rising tone)
  // High tone; similar to U+0E48 but slightly different origin.
  { .codepoint = 0x0E49, .offset_x = -2, .offset_y = 0 },
};

static const size_t THAI_MARK_OFFSETS_COUNT = sizeof(THAI_MARK_OFFSETS) / sizeof(THAI_MARK_OFFSETS[0]);

// Track the last base glyph rendered so zero-advance marks can attach to it.
static bool s_last_base_valid = false;
static Codepoint s_last_base_codepoint = 0;
static GPoint s_last_base_origin = { 0, 0 };
static bool s_last_mark_valid = false;
static GRect s_last_mark_target = GRectZero;
static Codepoint s_last_mark_codepoint = 0;
static int16_t s_last_cursor_x = 0;

/// Get the static offset for a Thai combining mark, or (0, 0) if not found.
static struct ThaiMarkOffset prv_get_thai_mark_offset(uint32_t codepoint) {
  for (size_t i = 0; i < THAI_MARK_OFFSETS_COUNT; ++i) {
    if (THAI_MARK_OFFSETS[i].codepoint == codepoint) {
      return THAI_MARK_OFFSETS[i];
    }
  }
  // No offset for this codepoint; return zero offset.
  return (struct ThaiMarkOffset) { .codepoint = 0, .offset_x = 0, .offset_y = 0 };
}

static GRect get_glyph_rect(const GlyphData* glyph) {
  GRect r = {
    .size.w = glyph->header.width_px,
    .size.h = glyph->header.height_px,
    .origin.x = glyph->header.left_offset_px,
    .origin.y = glyph->header.top_offset_px
  };

  return r;
}

#if CONFIG_SCREEN_COLOR_DEPTH_BITS == 8
/// This function returns the x coordinate of where to write the contents of a given word (32-bits)
/// of data from the 1-bit frame buffer into the 8-bit framebuffer
/// @param dest_bitmap 8-bit destination frame buffer bitmap
/// @param block_addr source address in 1-bit frame buffer of where the word is being updated
///                   within a given row; assumed to be zero-based
/// @param y_offset row offset within the source 1-bit frame buffer
T_STATIC int32_t prv_convert_1bit_addr_to_8bit_x(GBitmap *dest_bitmap, uint32_t *block_addr,
                                                 int32_t y_offset) {
  // Each byte block_addr corresponds to 8 pixels (i.e. 4-bytes in the 8-bit frame buffer).
  // Thus multiply by 8 to get the word offset within the destination 8-bit frame buffer.
  // Also need to account for the fact that the 1-bit frame buffer has 16 bits of unused space
  // on each row (thus 16 bytes need to be subtracted from the destination address since there is
  // no padding on each row of the 8-bit frame buffer.
  const int32_t padding = (32 - (dest_bitmap->bounds.size.w % 32)) % 32;
  // Calculate the overall offset in the 8-bit bitmap
  const int32_t bitmap_offset_8bit = ((uint32_t)block_addr * 8) - (padding * y_offset);
  // Calculate just the offset from the start of the target row in the 8-bit bitmap (i.e. "x")
  return bitmap_offset_8bit - (dest_bitmap->bounds.size.w * y_offset);
}
#endif

// PRO TIP: if you have to modify this function, expect to waste the rest of your day on it
void render_glyph(GContext* const ctx, const uint32_t codepoint, FontInfo* const font,
                  const GRect cursor) {
  if (codepoint_is_special(codepoint)) {
    TextRenderState *state = app_state_get_text_render_state();
    if (state->special_codepoint_handler_cb) {
      state->special_codepoint_handler_cb(ctx, codepoint, cursor,
          state->special_codepoint_handler_context);
    }
    return;
  }

  int16_t baseline_adjust = 0;
  const GlyphData* glyph = text_resources_get_glyph(&ctx->font_cache, codepoint, font,
                                                    &baseline_adjust);

  PBL_ASSERTN(glyph);
  // Bitfiddle the metrics data:
  GRect glyph_metrics = get_glyph_rect(glyph);
  // Sit a substituted glyph on the primary font's baseline. Must happen before the target and
  // clipping rects are derived from it.
  glyph_metrics.origin.y += baseline_adjust;

  // Calculate the box that we intend to draw to the screen, in screen coordinates.
  // For Thai combining marks, apply static offsets to reduce visual overlap with base glyphs.
  GRect glyph_target = {
    .origin = { .x = cursor.origin.x + glyph_metrics.origin.x,
                .y = cursor.origin.y + glyph_metrics.origin.y },
    .size = { .w = glyph_metrics.size.w,
              .h = glyph_metrics.size.h }
  };

  // If x moves backwards significantly, we likely started a new run/line.
  if (cursor.origin.x < (s_last_cursor_x - 2)) {
    s_last_base_valid = false;
    s_last_mark_valid = false;
  }
  s_last_cursor_x = cursor.origin.x;

  // For combining marks, try Phase 2 anchor lookup first, then keep Phase 1 fallback.
  if (codepoint_is_zero_advance(codepoint)) {
    bool applied_anchor = false;
    if (s_last_mark_valid) {
      GPoint mark_anchor_offset = { 0, 0 };
      if (text_resources_get_mark_anchor_offset(&ctx->font_cache, s_last_mark_codepoint,
                                                codepoint, font, &mark_anchor_offset)) {
        glyph_target.origin.x = s_last_mark_target.origin.x + mark_anchor_offset.x;
        glyph_target.origin.y = s_last_mark_target.origin.y + mark_anchor_offset.y;
        applied_anchor = true;
      }
    }
    if (s_last_base_valid) {
      GPoint anchor_offset = { 0, 0 };
      if (!applied_anchor && text_resources_get_mark_anchor_offset(&ctx->font_cache,
                                                s_last_base_codepoint,
                                                codepoint, font, &anchor_offset)) {
        glyph_target.origin.x = s_last_base_origin.x + anchor_offset.x;
        glyph_target.origin.y = s_last_base_origin.y + anchor_offset.y;
        applied_anchor = true;
      }
    }

    if (!applied_anchor) {
      struct ThaiMarkOffset offset = prv_get_thai_mark_offset(codepoint);
      glyph_target.origin.x += offset.offset_x;
      glyph_target.origin.y += offset.offset_y;
    }

    // Multiple marks can share the same base anchor. Stack overlapping marks upward while
    // preserving the font's glyph dimensions and advance metrics.
    if (s_last_mark_valid && grect_overlaps_grect(&glyph_target, &s_last_mark_target)) {
      glyph_target.origin.y = s_last_mark_target.origin.y - glyph_target.size.h - 1;
    }
    s_last_mark_target = glyph_target;
    s_last_mark_codepoint = codepoint;
    s_last_mark_valid = true;
  } else {
    s_last_base_valid = true;
    s_last_base_codepoint = codepoint;
    s_last_base_origin = glyph_target.origin;
    s_last_mark_valid = false;
    s_last_mark_codepoint = 0;
  }


  // The destination bitmap's x-coordinate and row advance. Used in the loop below.
  GBitmap* dest_bitmap = graphics_context_get_bitmap(ctx);
  const int32_t x = (int32_t)((int16_t)cursor.origin.x + (int16_t)glyph_metrics.origin.x);

  // Now clip that box against the screen/other UI elements. This rect will be the rect that we
  // actually fill with bits on the screen.
  GRect clipped_glyph_target = glyph_target;
  grect_clip(&clipped_glyph_target, &ctx->draw_state.clip_box);

  // The number of bits to be clipped off the edges
  const int left_clip = clipped_glyph_target.origin.x - glyph_target.origin.x;
  const int right_clip = MIN(glyph_target.size.w,
                             MAX(0, glyph_target.size.w - clipped_glyph_target.size.w - left_clip));

#if CONFIG_SCREEN_COLOR_DEPTH_BITS == 8
  // Set base address to 0 for 8-bit as this will be later translated to the destination bitmap
  // address - so do all calculations so everything is offset from 0
  uint32_t * base_addr = 0;
#else
  uint32_t * base_addr = ((uint32_t*)dest_bitmap->addr);
#endif

  const uint32_t * const dest_block_x_begin = base_addr +
                                              (left_clip ?
                                               MAX(0, (((x + left_clip + 31)/ 32) - 1)) : (x / 32));

  if (clipped_glyph_target.size.h == 0 || clipped_glyph_target.size.w == 0) {
    return;
  }

#if CONFIG_SCREEN_COLOR_DEPTH_BITS == 8
  // NOTE: Since all calculations are based on 1-bit calculation - use the row size from
  // the 1-bit frame buffer
  const int row_size_bytes = 4 * ((dest_bitmap->bounds.size.w / 32) +
                                  ((dest_bitmap->bounds.size.w % 32) ? 1 : 0));
#else
  const int row_size_bytes = dest_bitmap->row_size_bytes;
#endif // CONFIG_SCREEN_COLOR_DEPTH_BITS == 8

  // Number of blocks (i.e. 32-bit chunks)
  const int dest_row_length = row_size_bytes / 4;

  // The number of bits between the beginning of dest_block and glyph_block.
  // If x is negative we need to be fancy to get the rounded down remainder. This
  // is the number of bits to the right of the next 32-bit boundry to the left.
  // For example, if x is -5 we want this shift to be 27, since -32 (the nearest
  // boundry) + 27 = -5
  const uint8_t dest_shift_at_line_begin = (x >= 0) ?
      x % 32 :
      (x - ((x / 32) * 32));

  uint8_t dest_shift = dest_shift_at_line_begin;

  // The glyph bitmap starts the block after the metrics data:
  uint32_t const* glyph_block = glyph->data;

  // Set up the first piece of source glyph bitmap:
  int8_t glyph_block_bits_left = 32;
  uint32_t src = *glyph_block;

  // Use bit-rotate to align to shift the bitmap to align with the destination.
  // The advantage of rotate vs. bitwise shift is that we can use
  // the bits that wrapped around for the next dest_block
  rotl32(src, dest_shift);
  int8_t src_rotated = dest_shift;
  // how many 32-bit blocks do we need to bitblt on each row. If we're not word aligned we'll need to
  // modify an extra partial word, as we'll have an incomplete word on either side of the line segment
  // we're modifying.
  // For 1-bit, each pixel goes into one bit in dest bitmap - so 32 pixels per block
  const uint8_t num_dest_blocks_per_row = (clipped_glyph_target.size.w / 32) +
                                          (((dest_shift + left_clip) % 32) ? 1 : 0);

  // Handle clipping at the top of the character. We need to skip a number of bits in our source data.
  const unsigned int bits_to_skip = glyph_metrics.size.w * (clipped_glyph_target.origin.y - glyph_target.origin.y);
  if (bits_to_skip) {
    glyph_block += bits_to_skip / 32;
    src = *glyph_block;

    // Simulate the rotate that happens at the bottom of the bitblt loop so our source value is set
    // up just as if we actually rendered those first few lines.
    rotl32(src, (dest_shift_at_line_begin + ((0 - ((uint8_t)glyph_metrics.size.w)) % 32) * (clipped_glyph_target.origin.y - glyph_target.origin.y)) % 32);
    src_rotated = (dest_shift_at_line_begin + ((0 - ((uint8_t)glyph_metrics.size.w)) % 32) * (clipped_glyph_target.origin.y - glyph_target.origin.y)) % 32;
    glyph_block_bits_left -= bits_to_skip % 32;
  }

  for (int dest_y = clipped_glyph_target.origin.y; dest_y != clipped_glyph_target.origin.y + clipped_glyph_target.size.h; ++dest_y) {
    dest_shift = dest_shift_at_line_begin;

    // Number of bits to render on this line.
    uint8_t glyph_line_bits_left = clipped_glyph_target.size.w;

    uint32_t *dest_block = (uint32_t *)dest_block_x_begin + (dest_y * dest_row_length);
    const uint32_t *dest_block_end = dest_block + num_dest_blocks_per_row + 1;

    if (left_clip) {
      const int left_clip_shift = left_clip % 32;
      const int clipped_blocks = left_clip / 32;

      dest_shift = (dest_shift + left_clip_shift) % 32;
      glyph_block_bits_left -= left_clip_shift;

      glyph_block += clipped_blocks;

      if (glyph_block_bits_left <= 0) {
        src = *(++glyph_block);
        glyph_block_bits_left += 32;
        // Need to account for the dest_shift when loading up the new glyph block
        rotl32(src, glyph_block_bits_left + dest_shift);
        src_rotated = glyph_block_bits_left + dest_shift;
      }

      dest_block += clipped_blocks;
    }

    while (dest_block != dest_block_end && glyph_line_bits_left) {
      PBL_ASSERT(dest_block < dest_block_end, "DB=<%p> DBE=<%p>", dest_block, dest_block_end);
      PBL_ASSERTN(dest_block >= (uint32_t*) base_addr);
      PBL_ASSERTN(dest_block < (uint32_t*) base_addr + row_size_bytes *
                  (dest_bitmap->bounds.origin.y + dest_bitmap->bounds.size.h));

      // bitblt part of glyph_block:
      const uint8_t number_of_bits = MIN(32 - dest_shift, MIN(glyph_line_bits_left, glyph_block_bits_left));
      const uint32_t mask = (((1 << number_of_bits) - 1) << dest_shift);

#if CONFIG_SCREEN_COLOR_DEPTH_BITS == 8
      // dest_block points to the block if the dest image was a 1-bit buffer
      // translate this to an x coordinate in the 8-bit buffer
      const int32_t block_start_x = prv_convert_1bit_addr_to_8bit_x(dest_bitmap, dest_block,
                                                                    dest_y);
      const GBitmapDataRowInfo data_row = gbitmap_get_data_row_info(dest_bitmap, dest_y);
      // Only enter the loop if the current block is within the valid data row range
      if (block_start_x + 31 >= data_row.min_x && block_start_x <= data_row.max_x) {
        uint8_t *dest_addr = data_row.data + block_start_x;

        // For each bit in block, write that bit to the dest_bitmap
        for (unsigned int bitindex = 0; bitindex < 32; bitindex++) {
          const int32_t current_x = block_start_x + bitindex;
          // Stop iteration early if we have reached the end of the data row
          if (current_x > data_row.max_x) {
            break;
          }
          // Skip over pixels outside of the bitmap data's x coordinate range
          if (current_x < data_row.min_x) {
            continue;
          }
          // Find position in dest_bitmap that corresponds to the bit index
          // Write to that position if mask for that bit is 1
          if ((mask & src) & (1 << bitindex)) {
            GColor dest_color;
            if (ctx->draw_state.compositing_mode == GCompOpSet) {
              // Blend (i.e. for transparency) if GCompOpSet
              dest_color = gcolor_alpha_blend(ctx->draw_state.text_color,
                                              (GColor) {.argb = dest_addr[bitindex]});
            } else {
              dest_color = ctx->draw_state.text_color;
              dest_color.a = 3;
            }
            dest_addr[bitindex] = dest_color.argb;
          }
        }
      }
#else
      if (gcolor_equal(ctx->draw_state.text_color, GColorBlack)) {
        *(dest_block) &= ~(mask & src);
      } else {
        *(dest_block) |= mask & src;
      }
#endif

      dest_shift = (dest_shift + number_of_bits) % 32;
      glyph_block_bits_left -= number_of_bits;
      glyph_line_bits_left -= number_of_bits;

      if (glyph_block_bits_left <= 0) {
        // We ran out of bits in the current glyph block. Get the next glyph blob:
        src = *(++glyph_block);
        glyph_block_bits_left += 32;
        rotl32(src, dest_shift);
        src_rotated = dest_shift;
        // Continue with this dest_block if there is still space left:
        if (dest_shift) {
          continue;
        }
      }

      ++dest_block;
    }

    dest_shift += right_clip % 32;

    // emulate having drawn the right clip
    if (glyph_block_bits_left <= right_clip) {
      int jump_words = (right_clip - glyph_block_bits_left) / 32 + 1;
      glyph_block += jump_words;
      src = *glyph_block;
      rotl32(src, src_rotated);
      glyph_block_bits_left += 32 * jump_words;
    }
    glyph_block_bits_left -= right_clip;


    // Rotate the bits into the right position for the next row:
    dest_shift = dest_shift_at_line_begin - dest_shift;
    rotl32(src, dest_shift % 32);
    src_rotated = (src_rotated + dest_shift) % 32;
  }

  graphics_context_mark_dirty_rect(ctx, clipped_glyph_target);
}


void text_render_set_special_codepoint_cb(SpecialCodepointHandlerCb handler, void *context) {
  TextRenderState *state = app_state_get_text_render_state();
  state->special_codepoint_handler_cb = handler;
  state->special_codepoint_handler_context = context;
}
