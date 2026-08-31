/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "ui.h"

#include "applib/pbl_std/pbl_std.h"
#include "board/display.h"
#include "pbl/services/clock.h"
#include "pbl/services/i18n/i18n.h"
#include "pbl/util/string.h"
#include "util/time/time.h"

// Compile-time display offset calculations
#define HEALTH_X_OFFSET ((DISP_COLS - LEGACY_2X_DISP_COLS) / 2)
#define HEALTH_Y_OFFSET ((DISP_ROWS - LEGACY_2X_DISP_ROWS) / 2)
#define HEALTH_INSET (18 + HEALTH_X_OFFSET / 5)

void health_ui_draw_text_in_box(GContext *ctx, const char *text, const GRect drawing_bounds,
                                const int16_t y_offset, const GFont small_font, GColor box_color,
                                GColor text_color) {
  const uint8_t text_height = fonts_get_font_height(small_font);
  const GTextOverflowMode overflow_mode = GTextOverflowModeFill;
  const GTextAlignment alignment = GTextAlignmentCenter;

  const GRect text_box = GRect(drawing_bounds.origin.x, y_offset,
                               drawing_bounds.size.w, text_height);

  GRect text_fill_box = text_box;
  text_fill_box.size = app_graphics_text_layout_get_content_size(
      text, small_font, text_box, overflow_mode, alignment);
  text_fill_box.origin.x += ((drawing_bounds.size.w - text_fill_box.size.w) / 2);

  // add a 3 px border (get content size already adds 1 px)
  text_fill_box = grect_inset(text_fill_box, GEdgeInsets(-2));

  // get content size adds 5 to the height, and the y offset is too high by a px (+ the 5px)
  const int height_correction = 5;
  text_fill_box.size.h -= height_correction;
  text_fill_box.origin.y += height_correction + 1;

  if (!gcolor_equal(box_color, GColorClear)) {
    graphics_context_set_fill_color(ctx, box_color);
    graphics_fill_rect(ctx, &text_fill_box);
  }

  if (!gcolor_equal(text_color, GColorClear)) {
    graphics_context_set_text_color(ctx, text_color);
    graphics_draw_text(ctx, text, small_font, text_box, overflow_mode, alignment, NULL);
  }
}

// Draws the rounded pill background and the "TYPICAL <day>" header, common to
// both the single-value and split layouts. Returns the pill rect with origin.y
// already nudged up to compensate for the text renderer's top padding, so
// callers can position their body rows relative to it.
static GRect prv_render_typical_pill(GContext *ctx, Layer *layer, int pill_height) {
  time_t now = rtc_get_time();
  struct tm time_tm;
  localtime_r(&now, &time_tm);
  char weekday[8];
  strftime(weekday, sizeof(weekday), "%a", &time_tm);
  toupper_str(weekday);

  char typical_text[32];
  snprintf(typical_text, sizeof(typical_text), i18n_get("TYPICAL %s", layer), weekday);

  // Push the pill lower by a third of the display's vertical health-offset.
  // Scales to the slack available — zero on legacy-sized displays (asterix,
  // flint), a few px on taller displays (obelix, getafix). Applies to both
  // activity and sleep cards on those taller displays; the extra breathing
  // room reads better on both.
  const int y = PBL_IF_RECT_ELSE(PBL_IF_BW_ELSE(122, 120), 125) + HEALTH_Y_OFFSET
                + HEALTH_Y_OFFSET / 3;
  GRect rect = GRect(0, y, layer->bounds.size.w, pill_height);
#if PBL_RECT
  rect = grect_inset(rect, GEdgeInsets(0, HEALTH_INSET));
#endif

  const GColor bg_color = PBL_IF_COLOR_ELSE(GColorYellow, GColorBlack);
  const GColor text_color = PBL_IF_COLOR_ELSE(GColorBlack, GColorWhite);
  const GFont font = fonts_get_system_font(FONT_KEY_GOTHIC_18_BOLD);

  graphics_context_set_fill_color(ctx, bg_color);
  graphics_fill_round_rect(ctx, &rect, 3, GCornersAll);

  // Compensate for the text renderer's top padding so the header glyphs sit
  // just inside the pill's visual top edge.
  rect.origin.y -= PBL_IF_RECT_ELSE(3, 2);

  graphics_context_set_text_color(ctx, text_color);

  // Row 1: TYPICAL <day>, full width.
  GRect header_rect = rect;
  header_rect.size.h = 16;
  graphics_draw_text(ctx, typical_text, font, header_rect,
                     GTextOverflowModeWordWrap, GTextAlignmentCenter, NULL);

  return rect;
}

void health_ui_render_typical_text_box(GContext *ctx, Layer *layer, const char *value_text) {
  GRect rect = prv_render_typical_pill(ctx, layer, PBL_IF_RECT_ELSE(35, 36));

  const GFont font = fonts_get_system_font(FONT_KEY_GOTHIC_18_BOLD);

  // Row 2: the single value line, full width below the header.
  GRect value_rect = rect;
  value_rect.size.h = 16;
  value_rect.origin.y += 16;
  graphics_draw_text(ctx, value_text, font, value_rect,
                     GTextOverflowModeWordWrap, GTextAlignmentCenter, NULL);
}

void health_ui_render_split_typical_text_box(GContext *ctx, Layer *layer,
                                             const char *left_value, const char *left_label,
                                             const char *right_value, const char *right_label) {
  GRect rect = prv_render_typical_pill(ctx, layer, 53);

  const GFont font = fonts_get_system_font(FONT_KEY_GOTHIC_18_BOLD);
  const GFont small_font = fonts_get_system_font(FONT_KEY_GOTHIC_14_BOLD);

  // On round we narrow the column band so the two halves sit closer to the
  // pill's center instead of marooned at the round display's edges.
  const int16_t col_band_inset = PBL_IF_RECT_ELSE(0, 40);
  const int16_t col_w = (rect.size.w - 2 * col_band_inset) / 2;

  // Row 2: values, numbers on top.
  GRect val_left = GRect(rect.origin.x + col_band_inset, rect.origin.y + 18,
                         col_w, 16);
  GRect val_right = val_left;
  val_right.origin.x += val_left.size.w;
  graphics_draw_text(ctx, left_value, font, val_left,
                     GTextOverflowModeWordWrap, GTextAlignmentCenter, NULL);
  graphics_draw_text(ctx, right_value, font, val_right,
                     GTextOverflowModeWordWrap, GTextAlignmentCenter, NULL);

  // Row 3: labels below the numbers, in black.
  const GColor label_color = GColorArmyGreen;
  GRect sub_left = val_left;
  sub_left.origin.y += 18;
  sub_left.size.h = 14;
  GRect sub_right = sub_left;
  sub_right.origin.x += sub_left.size.w;
  graphics_context_set_text_color(ctx, label_color);
  graphics_draw_text(ctx, left_label, small_font, sub_left,
                     GTextOverflowModeWordWrap, GTextAlignmentCenter, NULL);
  graphics_draw_text(ctx, right_label, small_font, sub_right,
                     GTextOverflowModeWordWrap, GTextAlignmentCenter, NULL);

  // Vertical divider between the two columns, 1px wide black.
  graphics_context_set_stroke_color(ctx, label_color);
  graphics_context_set_stroke_width(ctx, 1);
  const int16_t divider_x = rect.origin.x + rect.size.w / 2;
  graphics_draw_line(ctx, GPoint(divider_x, val_left.origin.y + 6),
                     GPoint(divider_x, sub_left.origin.y + sub_left.size.h));
}
