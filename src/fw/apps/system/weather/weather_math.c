/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "weather_math.h"
#include "applib/ui/animation_interpolate.h"
#include <string.h>
#include <time.h>

int64_t weather_interpolate_moook_soft1(int32_t n, int64_t from, int64_t to) {
  return interpolate_moook_soft(n, from, to, 1);
}

// WHO UV-index severity color — ONE copy for every UV readout in the app
// (the expanded card's dial + the report page's UV bar tell the same story):
// 0-2 green, 3-5 yellow, 6-7 orange, 8+ red (incl. off-scale).
GColor weather_uv_severity_color(int uv) {
  return (uv <= 2)  ? GColorIslamicGreen
       : (uv <= 5)  ? GColorChromeYellow
       : (uv <= 7)  ? GColorOrange
                    : GColorRed;
}

const uint8_t weather_diurnal_curve[24] = {
   10,  6,  3,  0,  0,  3,  8, 16, 28, 42, 56, 70,
   82, 92, 98, 100, 96, 88, 76, 62, 48, 36, 26, 17,
};

// One jelly edge — leading edge delay 0, trailing edge delay 1/6 over a 5/6 point-duration, on
// the exact interpolate_moook_soft curve Timeline feeds its scale_segmented transform.
static int prv_jelly_edge(AnimationProgress m, int delay_num, int from, int to) {
  const int32_t MAX = ANIMATION_NORMALIZED_MAX;
  int32_t local = (int32_t)m - MAX * delay_num / 6;
  if (local < 0) local = 0;
  local = (int32_t)((int64_t)local * 6 / 5);
  if (local > MAX) local = MAX;
  return (int)interpolate_moook_soft(local, from, to, 3);
}

// Capture the fully-drawn screen from `ctx` and re-blit it squash-stretched. Reads the
// one-shot `scratch` snapshot so re-sampling never hits already-overwritten framebuffer
// rows.
//
// BOTH shapes. Rows are addressed ABSOLUTELY (ri.data[x], x in screen coords) and every
// write is clamped to that row's [min_x, max_x]. On rect the mask is the full width, so
// this is identical to the old code; on round the mask is the circle's chord — writing
// outside it (as `ri.data + ri.min_x` with a full-width length did) both shifts the image
// and runs off the end of the row.
//
// The resample body lives in prv_squash_resample so capture-once callers
// (weather_render_squash_cached) can re-blit from an existing snapshot without paying the
// full framebuffer copy every frame — the resample writes EVERY pixel of every row's
// chord (src or white), so nothing stale survives underneath.
static void prv_squash_resample(GBitmap *fb, const uint8_t *scratch,
                                AnimationProgress m, int mode) {
  GRect b = gbitmap_get_bounds(fb);
  const int W = b.size.w, H = b.size.h;
  const uint8_t white = GColorWhite.argb;
  if (mode == WEATHER_SQUASH_LEFT_EXIT) {
    // Horizontal jelly (the vertical grammar rotated 90°). RIGHT_IN (mode 7) was
    // trimmed with its last caller (the report's old squash-in entrance).
    int left_edge, right_edge;
    {
      // Left edge leads 0 -> -W (delay 0); right trails the HALF-lag (average of leading
      // and 1/6-lagged), both clamped to rest so the moook anticipation never pokes the
      // frame rightward (same reasoning as the UP_EXIT clamp).
      left_edge  = prv_jelly_edge(m, 0, 0, -W);
      right_edge = (prv_jelly_edge(m, 0, W, 0) + prv_jelly_edge(m, 1, W, 0)) / 2;
      if (left_edge > 0)  left_edge  = 0;
      if (right_edge > W) right_edge = W;
    }
    int dst_w = right_edge - left_edge;
    if (dst_w < 1) dst_w = 1;
    const int32_t sx_step = ((int32_t)W << 16) / dst_w;
    int vis0 = left_edge > 0 ? left_edge : 0;
    int vis1 = right_edge < W ? right_edge : W;
    if (vis1 < vis0) vis1 = vis0;
    for (int y = 0; y < H; y++) {
      GBitmapDataRowInfo ri = gbitmap_get_data_row_info(fb, (uint16_t)y);
      uint8_t *dst = ri.data;                       // absolute-x
      const uint8_t *src = scratch + (uint32_t)y * W;
      for (int x = (int)ri.min_x; x <= (int)ri.max_x; x++) {
        if (x < vis0 || x >= vis1) { weather_fb_row_set(dst, x, white); continue; }
        int sx = (int)((((int32_t)(x - left_edge)) * sx_step) >> 16);
        if (sx < 0) sx = 0; else if (sx >= W) sx = W - 1;
        weather_fb_row_set(dst, x, src[sx]);
      }
    }
    return;
  }
  int top_edge, bot_edge;
  if (mode == WEATHER_SQUASH_DOWN_EXIT) {   // down exit: bottom leads off the bottom
    // Compress the exit into the first ~75% of the animation so the screen has fully
    // cleared before the card's glance text sweeps across (see prv_draw_flying_content).
    AnimationProgress me = m + m / 3;
    if (me > ANIMATION_NORMALIZED_MAX) me = ANIMATION_NORMALIZED_MAX;
    top_edge = prv_jelly_edge(me, 1, 0, H);
    bot_edge = prv_jelly_edge(me, 0, H, 2 * H);
  } else if (mode == WEATHER_SQUASH_CLOCK_EXIT) {   // clock exit: same geometry, full timeline
    top_edge = prv_jelly_edge(m, 1, 0, H);
    bot_edge = prv_jelly_edge(m, 0, H, 2 * H);
  } else if (mode == WEATHER_SQUASH_RISE_IN) {   // rise from below: top leads up into place
    top_edge = prv_jelly_edge(m, 0, H,     0);
    bot_edge = prv_jelly_edge(m, 1, 2 * H, H);
  } else if (mode == WEATHER_SQUASH_UP_EXIT) {   // up exit: top leads off the top; the bottom
    // trails a HALF-lag (1/12 — the average of the leading and 1/6-lagged edges) for a
    // subtler stretch than the other modes. Full timeline (no haste), like mode 4.
    top_edge = prv_jelly_edge(m, 0, 0, -H);
    bot_edge = (prv_jelly_edge(m, 0, H, 0) + prv_jelly_edge(m, 1, H, 0)) / 2;
    // Never let the frame move DOWN past its rest: the moook anticipation dip would poke
    // the bottom content below the decoupled burst dot for a beat, which reads wrong.
    if (top_edge > 0) top_edge = 0;
    if (bot_edge > H) bot_edge = H;
  } else {                                       // drop-in: bottom leads down into place
    top_edge = prv_jelly_edge(m, 1, -H, 0);
    bot_edge = prv_jelly_edge(m, 0,  0, H);
  }
  int dst_h = bot_edge - top_edge;
  if (dst_h < 1) dst_h = 1;
  const int32_t sy_step = ((int32_t)H << 16) / dst_h;
  for (int ay = 0; ay < H; ay++) {
    GBitmapDataRowInfo ri = gbitmap_get_data_row_info(fb, (uint16_t)ay);
    const int lo = (int)ri.min_x, hi = (int)ri.max_x;
    if (ay >= top_edge && ay < bot_edge) {
      int sy = (int)((((int32_t)(ay - top_edge)) * sy_step) >> 16);
      if (sy < 0) sy = 0; else if (sy >= H) sy = H - 1;
      // The SOURCE row's chord differs from this row's, so anything this row exposes
      // beyond it would read pixels that were never drawn — white those.
      GBitmapDataRowInfo sri = gbitmap_get_data_row_info(fb, (uint16_t)sy);
      const uint8_t *src = scratch + (uint32_t)sy * W;
      for (int x = lo; x <= hi; x++) {
        weather_fb_row_set(ri.data, x,
                           (x >= (int)sri.min_x && x <= (int)sri.max_x) ? src[x] : white);
      }
    } else {
      for (int x = lo; x <= hi; x++) weather_fb_row_set(ri.data, x, white);
    }
  }
}

void weather_render_squash(GContext *ctx, uint8_t *scratch, AnimationProgress m, int mode) {
  GBitmap *fb = graphics_capture_frame_buffer(ctx);
  if (!fb) return;
  GRect b = gbitmap_get_bounds(fb);
  const int W = b.size.w, H = b.size.h;
  for (int y = 0; y < H; y++) {
    GBitmapDataRowInfo ri = gbitmap_get_data_row_info(fb, (uint16_t)y);
#if PBL_BW
    // Unpack the 1-bit row into the byte-per-pixel snapshot.
    for (int x = 0; x < W; x++) {
      scratch[(uint32_t)y * W + x] = weather_fb_row_get(ri.data, x);
    }
#else
    memcpy(scratch + (uint32_t)y * W, ri.data, W);   // absolute-x row
#endif
  }
  prv_squash_resample(fb, scratch, m, mode);
  graphics_release_frame_buffer(ctx, fb);
}

// Capture-once fast path (gabbro smoothness step 2): re-blit from a snapshot some earlier
// weather_render_squash call already filled — no scene render needed beneath it and no
// per-frame framebuffer copy. The caller owns knowing the snapshot is valid.
void weather_render_squash_cached(GContext *ctx, const uint8_t *scratch,
                                  AnimationProgress m, int mode) {
  GBitmap *fb = graphics_capture_frame_buffer(ctx);
  if (!fb) return;
  prv_squash_resample(fb, scratch, m, mode);
  graphics_release_frame_buffer(ctx, fb);
}

void weather_fill_weekday_abbrev(int day_offset, const char *fallback,
                                 char *buffer, size_t buffer_size) {
  if (!buffer || buffer_size == 0) return;
  time_t target = time(NULL) + (time_t)day_offset * 86400;
  struct tm *lt = localtime(&target);
  if (!lt || strftime(buffer, buffer_size, "%a", lt) == 0) {
    if (fallback && fallback[0]) {
      snprintf(buffer, buffer_size, "%.3s", fallback);
    } else {
      snprintf(buffer, buffer_size, "---");
    }
  }
  for (char *c = buffer; *c; c++) {
    if (*c >= 'a' && *c <= 'z') *c = (char)(*c - 'a' + 'A');
  }
}

uint32_t weather_scale_u32(uint32_t value, uint32_t numerator,
                           uint32_t denominator) {
  if (denominator == 0) return 0;

  uint32_t whole = value / denominator;
  uint32_t remainder = value % denominator;
  return (whole * numerator) + ((remainder * numerator) / denominator);
}

int32_t weather_scale_i32(int32_t value, int32_t numerator,
                          int32_t denominator) {
  if (denominator == 0) return 0;

  bool negative = false;
  if (value < 0) {
    value = -value;
    negative = !negative;
  }
  if (numerator < 0) {
    numerator = -numerator;
    negative = !negative;
  }
  if (denominator < 0) {
    denominator = -denominator;
    negative = !negative;
  }

  uint32_t scaled = weather_scale_u32((uint32_t)value, (uint32_t)numerator,
                                      (uint32_t)denominator);
  return negative ? -(int32_t)scaled : (int32_t)scaled;
}

int32_t weather_isqrt(int32_t value) {
  if (value <= 0) return 0;
  uint32_t x = (uint32_t)value;
  uint32_t result = 0;
  uint32_t bit = 1UL << 30;
  while (bit > x) bit >>= 2;
  while (bit != 0) {
    if (x >= result + bit) {
      x -= result + bit;
      result = (result >> 1) + bit;
    } else {
      result >>= 1;
    }
    bit >>= 2;
  }
  return (int32_t)result;
}

int32_t weather_norm_square(int32_t value) {
  if (value <= 0) return 0;
  if (value >= ANIMATION_NORMALIZED_MAX) return ANIMATION_NORMALIZED_MAX;
  return (int32_t)(((uint32_t)value * (uint32_t)value) /
                   (uint32_t)ANIMATION_NORMALIZED_MAX);
}

int32_t weather_norm_bell(int32_t value) {
  if (value <= 0 || value >= ANIMATION_NORMALIZED_MAX) return 0;
  uint32_t v = (uint32_t)value;
  uint32_t max = (uint32_t)ANIMATION_NORMALIZED_MAX;
  return (int32_t)((4U * v * (max - v)) / max);
}

void weather_capture_framebuffer_rect(GBitmap *fb, GBitmap *dst,
                                      GRect src_rect, int dst_x) {
  uint8_t *dst_data = gbitmap_get_data(dst);
  uint16_t dbpr = gbitmap_get_bytes_per_row(dst);
  int sx = src_rect.origin.x;
  int sy = src_rect.origin.y;
  int w = src_rect.size.w;
  int h = src_rect.size.h;

  for (int row = 0; row < h; row++) {
    GBitmapDataRowInfo ri = gbitmap_get_data_row_info(fb, (uint16_t)(sy + row));
    uint8_t *dst_row = dst_data + row * dbpr + dst_x;
#if PBL_BW
    // The destination is 1-bit on the BW boards (8-bit blanks are refused
    // there), so both sides are packed. dst_x is a byte offset and stays 0
    // for the 1-bit callers.
    memset(dst_row, 0xFF, dbpr);
#else
    memset(dst_row, 0xFF, (size_t)w);
#endif

    int xs = sx > (int)ri.min_x ? sx : (int)ri.min_x;
    int xe = (sx + w - 1) < (int)ri.max_x ? (sx + w - 1) : (int)ri.max_x;
    if (xs <= xe) {
#if PBL_BW
      for (int x = xs; x <= xe; x++) {
        bitset8_update(dst_row, (unsigned)(x - sx),
                       bitset8_get(ri.data, (unsigned)x));
        weather_fb_row_set(ri.data, x, GColorWhiteARGB8);
      }
#else
      size_t n = (size_t)(xe - xs + 1);
      memcpy(dst_row + (xs - sx), ri.data + xs, n);
      memset(ri.data + xs, 0xFF, n);
#endif
    }
  }
}

static void prv_fill_wrapped_radial(GContext *ctx, GRect rect, uint16_t inset,
                                    int32_t start, int32_t span) {
  const int32_t max = (int32_t)TRIG_MAX_ANGLE;
  start &= max - 1;
  if (span >= max) {
    graphics_fill_radial(ctx, rect, GOvalScaleModeFitCircle, inset, 0, max);
    return;
  }

  int32_t end = start + span;
  if (end <= max) {
    graphics_fill_radial(ctx, rect, GOvalScaleModeFitCircle, inset, start, end);
  } else {
    graphics_fill_radial(ctx, rect, GOvalScaleModeFitCircle, inset, start, max);
    graphics_fill_radial(ctx, rect, GOvalScaleModeFitCircle, inset, 0,
                         end - max);
  }
}

void weather_draw_lava_ring(GContext *ctx, GPoint center, int outer_r,
                            GColor glow_color, uint32_t phase_u,
                            uint8_t idle_progress) {
  int32_t phase = (int32_t)phase_u & ((int32_t)TRIG_MAX_ANGLE - 1);
  int32_t half = (int32_t)(TRIG_MAX_ANGLE / 2);
  int32_t span = half;
  bool idle = idle_progress > 0;
  int32_t neg = idle ? phase : (-phase & ((int32_t)TRIG_MAX_ANGLE - 1));

  if (idle_progress > WEATHER_GLOW_WRAP_SPIN_TICKS) {
    uint8_t close = idle_progress - WEATHER_GLOW_WRAP_SPIN_TICKS;
    if (close > WEATHER_GLOW_WRAP_CLOSE_TICKS) close = WEATHER_GLOW_WRAP_CLOSE_TICKS;
    span += (int32_t)close * (half / WEATHER_GLOW_WRAP_CLOSE_TICKS);
  }

  // (The faded Celeste outer halo ring was removed by design —
  // the temp keeps just the solid condition-colored ring + sparks/beads.)
  GRect ring_rect = { GPoint(center.x - outer_r, center.y - outer_r),
                      GSize(outer_r * 2, outer_r * 2) };
  graphics_context_set_fill_color(ctx, glow_color);
  prv_fill_wrapped_radial(ctx, ring_rect, 2, phase, span);
  prv_fill_wrapped_radial(ctx, ring_rect, 2, neg, span);

  if (idle) return;

  int sx1 = center.x + (int)((int32_t)sin_lookup(phase) * outer_r / TRIG_MAX_RATIO);
  int sy1 = center.y - (int)((int32_t)cos_lookup(phase) * outer_r / TRIG_MAX_RATIO);
  int sx2 = center.x + (int)((int32_t)sin_lookup(neg) * outer_r / TRIG_MAX_RATIO);
  int sy2 = center.y - (int)((int32_t)cos_lookup(neg) * outer_r / TRIG_MAX_RATIO);
  graphics_context_set_fill_color(ctx, GColorWhite);
  graphics_fill_circle(ctx, GPoint(sx1, sy1), 2);
  graphics_fill_circle(ctx, GPoint(sx2, sy2), 2);

  int32_t b_ang[3] = {
    (phase * 3 / 2) % (int32_t)TRIG_MAX_ANGLE,
    ((int32_t)(TRIG_MAX_ANGLE * 4) - phase * 2) % (int32_t)TRIG_MAX_ANGLE,
    (phase / 2 + (int32_t)(TRIG_MAX_ANGLE * 2 / 3)) % (int32_t)TRIG_MAX_ANGLE,
  };
  graphics_context_set_fill_color(ctx, glow_color);
  for (int b = 0; b < 3; b++) {
    int bx = center.x + (int)((int32_t)sin_lookup(b_ang[b]) * outer_r / TRIG_MAX_RATIO);
    int by = center.y - (int)((int32_t)cos_lookup(b_ang[b]) * outer_r / TRIG_MAX_RATIO);
    graphics_fill_circle(ctx, GPoint(bx, by), 1);
  }
}
