/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "peek_layer.h"

#include "pbl/util/trig.h"
#include "applib/ui/kino/kino_reel/scale_segmented.h"
#include "applib/ui/kino/kino_reel/transform.h"
#include "applib/ui/kino/kino_reel/unfold.h"
#include "kernel/pbl_malloc.h"
#include "kernel/ui/kernel_ui.h"
#include "pbl/services/evented_timer.h"
#include "pbl/services/timeline/notification_layout.h"
#include "pbl/services/timeline/timeline_resources.h"
#include <pbl/logging/logging.h>
#include "pbl/util/math.h"

#include "resource/resource_ids.auto.h"

//! Title text vertically centered position
#define TEXT_OFFSET_Y ((DISP_ROWS / 2) + PBL_IF_RECT_ELSE(46,  42))

//! Number text vertically bottom-aligned with Title text
#define NUMBER_OFFSET_Y (TEXT_OFFSET_Y + 2)

static void prv_update_proc(Layer *layer, GContext *ctx) {
  PeekLayer *peek_layer = (PeekLayer *)layer;
  const GRect layer_bounds = { .size = layer->bounds.size };

  if (peek_layer->bg_color.a != 0) {
    graphics_context_set_fill_color(ctx, peek_layer->bg_color);
    // Fill the peek background as a circle on round displays; this is needed to animate the
    // peek moving right as a circle to become the side bar on timeline as well as moving up to
    // become the top banner in the notifications window
#if PBL_ROUND
    // Use a radius equal to that of the notification banner to make the transition seamless
    const int32_t peek_circle_diameter = BANNER_CIRCLE_RADIUS * 2;
    GRect peek_circle_frame = (GRect) {
      .size = GSize(peek_circle_diameter, peek_circle_diameter)
    };
    grect_align(&peek_circle_frame, &layer_bounds, GAlignBottom, false /* clips */);
    graphics_fill_oval(ctx, peek_circle_frame, GOvalScaleModeFitCircle);
#else
    graphics_fill_rect(ctx, &layer_bounds);
#endif
  }

  if (peek_layer->show_dot) {
    graphics_context_set_fill_color(ctx, system_theme_get_fg_color());
    GRect dot_rect = { .size = { peek_layer->dot_diameter, peek_layer->dot_diameter } };
    grect_align(&dot_rect, &peek_layer->layer.bounds, GAlignCenter, false /* clip */);
    graphics_fill_radial(ctx, dot_rect, GOvalScaleModeFitCircle, peek_layer->dot_diameter, 0,
                         TRIG_MAX_ANGLE);
  }
}

static void prv_layout_text(PeekLayer *peek_layer) {
  const GRect layer_bounds = (GRect) { .size = peek_layer->layer.bounds.size };
  GContext *ctx = graphics_context_get_current_context();
  text_layer_set_size(&peek_layer->title.text_layer, layer_bounds.size);
  text_layer_set_size(&peek_layer->subtitle.text_layer, layer_bounds.size);
  text_layer_set_size(&peek_layer->number.text_layer, layer_bounds.size);

  const GSize number_size = text_layer_get_content_size(ctx, &peek_layer->number.text_layer);
  const GSize title_size = text_layer_get_content_size(ctx, &peek_layer->title.text_layer);
  const GSize subtitle_size = text_layer_get_content_size(ctx, &peek_layer->subtitle.text_layer);

  GPoint cursor = { (layer_bounds.size.w - subtitle_size.w) / 2,
                    -(subtitle_size.h + MAX(number_size.h, title_size.h)) / 2 };
  const int font_height_fuzz = 5; // Replace with font descenders
  layer_set_frame((Layer *)&peek_layer->subtitle.text_layer,
                  &(GRect) { { cursor.x, cursor.y + TEXT_OFFSET_Y },
                             { subtitle_size.w, subtitle_size.h + font_height_fuzz } });
  cursor.x = (layer_bounds.size.w - (title_size.w + number_size.w)) / 2;
  cursor.y += subtitle_size.h ? (subtitle_size.h + peek_layer->subtitle_margin) : 0;
  layer_set_frame((Layer *)&peek_layer->number.text_layer,
                  &(GRect) { { cursor.x, cursor.y + NUMBER_OFFSET_Y }, number_size });
  cursor.x += number_size.w;
  layer_set_frame((Layer *)&peek_layer->title.text_layer,
                  &(GRect) { { cursor.x, cursor.y + TEXT_OFFSET_Y },
                             { title_size.w, title_size.h + font_height_fuzz } });
}

//////////////////////
// API
/////////////////////

PeekLayer *peek_layer_create(GRect frame) {
  PeekLayer *peek_layer = task_malloc(sizeof(PeekLayer));
  if (peek_layer) {
    peek_layer_init(peek_layer, &frame);
  }

  return peek_layer;
}

void peek_layer_destroy(PeekLayer *peek_layer) {
  if (peek_layer) {
    peek_layer_deinit(peek_layer);
  }

  task_free(peek_layer);
}

void peek_layer_init(PeekLayer *peek_layer, const GRect *frame) {
  *peek_layer = (PeekLayer) {
    .icon_offset_y = PEEK_LAYER_ICON_OFFSET_Y,
    .subtitle_margin = PEEK_LAYER_SUBTITLE_MARGIN,
    .dot_diameter = 9,
  };
  // peek layer
  layer_init(&peek_layer->layer, frame);
  layer_set_clips(&peek_layer->layer, false);
  layer_set_update_proc(&peek_layer->layer, prv_update_proc);
  // kino layer
  kino_layer_init(&peek_layer->kino_layer,
                  &(GRect){ { 0, peek_layer->icon_offset_y }, frame->size });
  kino_layer_set_alignment(&peek_layer->kino_layer, GAlignCenter);
  layer_set_clips((Layer *)&peek_layer->kino_layer, false);
  layer_add_child((Layer *)peek_layer, (Layer *)&peek_layer->kino_layer);

  const GTextAlignment text_alignment = GTextAlignmentCenter;
  GRect text_rect = GRect(0, NUMBER_OFFSET_Y, frame->size.w, 40);
  // number layer
  text_layer_init_with_parameters(&peek_layer->number.text_layer,
                                  &text_rect,
                                  NULL, fonts_get_system_font(FONT_KEY_LECO_26_BOLD_NUMBERS_AM_PM),
                                  GColorBlack, GColorClear, text_alignment,
                                  GTextOverflowModeTrailingEllipsis);
  layer_add_child((Layer *)peek_layer, (Layer *)&peek_layer->number.text_layer);
  // title layer
  text_rect.origin.y = TEXT_OFFSET_Y;
  text_layer_init_with_parameters(&peek_layer->title.text_layer,
                                  &text_rect,
                                  NULL, fonts_get_system_font(FONT_KEY_GOTHIC_28_BOLD),
                                  GColorBlack, GColorClear, text_alignment,
                                  GTextOverflowModeTrailingEllipsis);
  layer_add_child((Layer *)peek_layer, (Layer *)&peek_layer->title.text_layer);
  // subtitle layer
  text_layer_init_with_parameters(&peek_layer->subtitle.text_layer,
                                  &text_rect,
                                  NULL, fonts_get_system_font(FONT_KEY_GOTHIC_18),
                                  GColorBlack, GColorClear, text_alignment,
                                  GTextOverflowModeTrailingEllipsis);
  layer_add_child((Layer *)peek_layer, (Layer *)&peek_layer->subtitle.text_layer);

  // initialize labels with empty strings
  peek_layer_clear_fields(peek_layer);
}

void peek_layer_deinit(PeekLayer *peek_layer) {
  evented_timer_cancel(peek_layer->hidden_fields_timer);
  peek_layer->hidden_fields_timer = EVENTED_TIMER_INVALID_ID;
  kino_layer_deinit(&peek_layer->kino_layer);
  text_layer_deinit(&peek_layer->title.text_layer);
  text_layer_deinit(&peek_layer->number.text_layer);
  text_layer_deinit(&peek_layer->subtitle.text_layer);
  layer_deinit(&peek_layer->layer);
}

void peek_layer_set_frame(PeekLayer *peek_layer, const GRect *frame) {
  layer_set_frame(&peek_layer->layer, frame);
  layer_set_frame((Layer *)&peek_layer->kino_layer,
                  &(GRect) { { 0, peek_layer->icon_offset_y }, frame->size });
}

void peek_layer_set_background_color(PeekLayer *peek_layer, GColor color) {
  peek_layer->bg_color = color;
}

void peek_layer_set_text_color(PeekLayer *peek_layer, GColor color) {
  text_layer_set_text_color(&peek_layer->number.text_layer, color);
  text_layer_set_text_color(&peek_layer->title.text_layer, color);
  text_layer_set_text_color(&peek_layer->subtitle.text_layer, color);
}

static bool prv_is_dot_size(GSize size) {
  return (size.w <= UNFOLD_DOT_SIZE_PX && size.h <= UNFOLD_DOT_SIZE_PX);
}

void peek_layer_set_icon_with_size_invert(PeekLayer *peek_layer,
                                          const TimelineResourceInfo *timeline_res,
                                          TimelineResourceSize res_size, GRect icon_from,
                                          bool invert) {
  kino_layer_set_invert_colors(&peek_layer->kino_layer, invert);
  kino_layer_set_reel(&peek_layer->kino_layer, NULL, false);

  AppResourceInfo icon_res_info;
  timeline_resources_get_id(timeline_res, res_size, &icon_res_info);
  KinoReel *from_reel = kino_reel_create_with_resource_system(icon_res_info.res_app_num,
                                                              icon_res_info.res_id);
  if (!from_reel) {
    return;
  }

  peek_layer->res_info = icon_res_info;

  GRect layer_frame;
  layer_get_global_frame((Layer *)&peek_layer->kino_layer, &layer_frame);
  if (grect_equal(&icon_from, &GRectZero)) {
    icon_from = (GRect) {
      .origin.x = layer_frame.origin.x + (layer_frame.size.w - UNFOLD_DOT_SIZE_PX) / 2,
      .origin.y = layer_frame.origin.y + (layer_frame.size.h - UNFOLD_DOT_SIZE_PX) / 2,
      .size = UNFOLD_DOT_SIZE,
    };
  }

  GSize size = kino_reel_get_size(from_reel);
  GRect icon_to = {
    .origin.x = layer_frame.origin.x + (layer_frame.size.w - size.w) / 2,
    .origin.y = layer_frame.origin.y + (layer_frame.size.h - size.h) / 2,
    .size = size,
  };

  const bool take_ownership = true;
  KinoReel *kino_reel = kino_reel_unfold_create(
      from_reel, take_ownership, layer_frame, 0,
      UNFOLD_DEFAULT_NUM_DELAY_GROUPS, UNFOLD_DEFAULT_GROUP_DELAY);
  kino_reel_transform_set_from_frame(kino_reel, icon_from);
  kino_reel_transform_set_to_frame(kino_reel, icon_to);
  kino_reel_transform_set_transform_duration(kino_reel, PEEK_LAYER_UNFOLD_DURATION);
  const int16_t expand = 8;
  kino_reel_scale_segmented_set_deflate_effect(kino_reel, expand);

  layer_set_hidden((Layer *)&peek_layer->kino_layer, true);
  peek_layer->show_dot = prv_is_dot_size(icon_from.size);
  if (peek_layer->show_dot) {
    kino_reel_unfold_set_start_as_dot(kino_reel, peek_layer->dot_diameter / 2);
  }

  kino_layer_set_reel(&peek_layer->kino_layer, kino_reel, true);
}

void peek_layer_set_icon_with_size(PeekLayer *peek_layer, const TimelineResourceInfo *timeline_res,
                                   TimelineResourceSize res_size, GRect icon_from) {
  peek_layer_set_icon_with_size_invert(peek_layer, timeline_res, res_size, icon_from, false);
}

void peek_layer_set_icon(PeekLayer *peek_layer, const TimelineResourceInfo *timeline_res) {
  peek_layer_set_icon_with_size(peek_layer, timeline_res, TimelineResourceSizeLarge, GRectZero);
}

void peek_layer_set_icon_with_invert(PeekLayer *peek_layer,
                                     const TimelineResourceInfo *timeline_res, bool invert) {
  peek_layer_set_icon_with_size_invert(peek_layer, timeline_res, TimelineResourceSizeLarge,
                                       GRectZero, invert);
}

//! This is called after both the scale to and the PDCS is complete
static void prv_scale_to_did_stop(KinoLayer *kino_layer, bool finished, void *context) {
  PeekLayer *peek_layer = context;
  GRect icon_to = kino_reel_transform_get_to_frame(
      kino_layer_get_reel(&peek_layer->kino_layer));
  // A visible kino layer keeps rendering its end-as-dot frame at the screen center; drawing the
  // static dot too shows a second dot when the layer origin is offset (e.g. peek_offset_y)
  peek_layer->show_dot = prv_is_dot_size(icon_to.size) &&
                         layer_get_hidden((Layer *)&peek_layer->kino_layer);
  kino_layer_set_callbacks(kino_layer, (KinoLayerCallbacks) { 0 }, NULL);
}

//! This is called after the scale to is complete
static void prv_scale_to_timer_callback(void *data) {
  PeekLayer *peek_layer = data;
  peek_layer->hidden_fields_timer = EVENTED_TIMER_INVALID_ID;
  peek_layer_set_fields_hidden(peek_layer, false);
}

void peek_layer_set_scale_to_image(PeekLayer *peek_layer, const TimelineResourceInfo *timeline_res,
                                   TimelineResourceSize res_size, GRect icon_to,
                                   bool align_in_frame) {
  GRect icon_from = GRectZero;
  KinoReel *prev_reel = kino_layer_get_reel(&peek_layer->kino_layer);
  if (prev_reel) {
    icon_from = kino_reel_get_elapsed(prev_reel) ? kino_reel_transform_get_to_frame(prev_reel)
                                                 : kino_reel_transform_get_from_frame(prev_reel);
  }
  kino_layer_set_reel(&peek_layer->kino_layer, NULL, false);

  KinoReel *from_reel = kino_reel_create_with_resource_system(peek_layer->res_info.res_app_num,
                                                              peek_layer->res_info.res_id);
  if (!from_reel) {
    return;
  }

  KinoReel *to_reel = NULL;
  if (timeline_res) {
    AppResourceInfo res_info;
    timeline_resources_get_id(timeline_res, res_size, &res_info);
    to_reel = kino_reel_create_with_resource_system(res_info.res_app_num,
                                                    res_info.res_id);
  }

  GSize size;
  if (peek_layer->show_dot) {
    size = UNFOLD_DOT_SIZE;
  } else {
    size = kino_reel_get_size(from_reel);
  }

  GRect layer_frame;
  layer_get_global_frame((Layer *)&peek_layer->kino_layer, &layer_frame);
  if (grect_equal(&icon_from, &GRectZero)) {
    icon_from = (GRect) {
      .origin.x = layer_frame.origin.x + (layer_frame.size.w - size.w) / 2,
      .origin.y = layer_frame.origin.y + (layer_frame.size.h - size.h) / 2,
      .size = size,
    };
  }

  if (to_reel && align_in_frame) {
    GRect rect_to_align = { icon_to.origin, kino_reel_get_size(to_reel) };
    grect_align(&rect_to_align, &icon_to, GAlignCenter, false);
    icon_to = rect_to_align;
  }

  GPoint center_from = grect_center_point(&icon_from);
  GPoint center_to = grect_center_point(&icon_to);
  GPoint target = gpoint_add(gpoint_sub(center_to, center_from), GPoint(size.w / 2, size.h / 2));

  const bool take_ownership = true;
  KinoReel *kino_reel = kino_reel_scale_segmented_create(from_reel, take_ownership, layer_frame);
  kino_reel_transform_set_from_frame(kino_reel, icon_from);
  kino_reel_transform_set_to_frame(kino_reel, icon_to);
  kino_reel_transform_set_transform_duration(kino_reel, PEEK_LAYER_SCALE_DURATION);
  kino_reel_scale_segmented_set_delay_by_distance(kino_reel, target);
  const int16_t expand = 10;
  kino_reel_scale_segmented_set_deflate_effect(kino_reel, expand);
  const int16_t bounce = 20;
  kino_reel_scale_segmented_set_bounce_effect(kino_reel, bounce);

  if (to_reel) {
    kino_reel_transform_set_to_reel(kino_reel, to_reel, take_ownership);
  }
  if (prv_is_dot_size(icon_to.size)) {
    kino_reel_scale_segmented_set_end_as_dot(kino_reel, peek_layer->dot_diameter / 2);
  }

  kino_layer_set_reel(&peek_layer->kino_layer, kino_reel, true);
  kino_layer_set_callbacks(&peek_layer->kino_layer, (KinoLayerCallbacks) {
    .did_stop = prv_scale_to_did_stop,
  }, peek_layer);

  peek_layer->hidden_fields_timer = evented_timer_register(PEEK_LAYER_SCALE_DURATION,
                                                           false,
                                                           prv_scale_to_timer_callback,
                                                           peek_layer);
}

void peek_layer_set_scale_to(PeekLayer *peek_layer, GRect icon_to) {
  const bool align_in_frame = true;
  peek_layer_set_scale_to_image(peek_layer, NULL, TimelineResourceSizeTiny, icon_to,
                                align_in_frame);
}

void peek_layer_set_duration(PeekLayer *peek_layer, uint32_t duration) {
  kino_reel_transform_set_transform_duration(
      kino_layer_get_reel(&peek_layer->kino_layer), duration);
}

static void prv_set_visible(PeekLayer *peek_layer) {
  layer_set_hidden((Layer *)&peek_layer->kino_layer, false);
  peek_layer->show_dot = false;
}

ImmutableAnimation *peek_layer_create_play_animation(PeekLayer *peek_layer) {
  prv_set_visible(peek_layer);
  return kino_layer_create_play_animation(&peek_layer->kino_layer);
}

ImmutableAnimation *peek_layer_create_play_section_animation(PeekLayer *peek_layer,
                                                             uint32_t from_elapsed_ms,
                                                             uint32_t to_elapsed_ms) {
  prv_set_visible(peek_layer);
  return kino_layer_create_play_section_animation(&peek_layer->kino_layer, from_elapsed_ms,
                                                  to_elapsed_ms);
}

void peek_layer_play(PeekLayer *peek_layer) {
  prv_set_visible(peek_layer);
  kino_layer_play(&peek_layer->kino_layer);
}

GSize peek_layer_get_size(PeekLayer *peek_layer) {
  return kino_reel_get_size(kino_layer_get_reel(&peek_layer->kino_layer));
}

static void prv_set_text(PeekTextLayer *peek_text_layer, const char *text) {
  memset(peek_text_layer->text_buffer, 0, MAX_PEEK_LAYER_TEXT_LEN);
  strncpy(peek_text_layer->text_buffer, text, MAX_PEEK_LAYER_TEXT_LEN - 1);
  text_layer_set_text(&peek_text_layer->text_layer, peek_text_layer->text_buffer);
}

void peek_layer_set_fields(PeekLayer *peek_layer, const char *number, const char *title,
                           const char *subtitle) {
  if (number) {
    prv_set_text(&peek_layer->number, number);
  }
  if (title) {
    prv_set_text(&peek_layer->title, title);
  }
  if (subtitle) {
    prv_set_text(&peek_layer->subtitle, subtitle);
  }
  prv_layout_text(peek_layer);
}

void peek_layer_clear_fields(PeekLayer *peek_layer) {
  peek_layer_set_fields(peek_layer, "", "", "");
}

void peek_layer_set_fields_hidden(PeekLayer *peek_layer, bool hidden) {
  layer_set_hidden(&peek_layer->number.text_layer.layer, hidden);
  layer_set_hidden(&peek_layer->title.text_layer.layer, hidden);
  layer_set_hidden(&peek_layer->subtitle.text_layer.layer, hidden);
}

void peek_layer_set_number(PeekLayer *peek_layer, const char *number) {
  peek_layer_set_fields(peek_layer, number, NULL, NULL);
}

void peek_layer_set_title(PeekLayer *peek_layer, const char *title) {
  peek_layer_set_fields(peek_layer, NULL, title, NULL);
}

void peek_layer_set_subtitle(PeekLayer *peek_layer, const char *subtitle) {
  peek_layer_set_fields(peek_layer, NULL, NULL, subtitle);
}

void peek_layer_set_title_font(PeekLayer *peek_layer, GFont font) {
  text_layer_set_font(&peek_layer->title.text_layer, font);
  prv_layout_text(peek_layer);
}

void peek_layer_set_subtitle_font(PeekLayer *peek_layer, GFont font, int16_t margin) {
  text_layer_set_font(&peek_layer->subtitle.text_layer, font);
  peek_layer->subtitle_margin = margin;
  prv_layout_text(peek_layer);
}

void peek_layer_set_dot_diameter(PeekLayer *peek_layer, uint8_t dot_diameter) {
  peek_layer->dot_diameter = dot_diameter;
}

void peek_layer_set_icon_offset_y(PeekLayer *peek_layer, int16_t icon_offset_y) {
  peek_layer->icon_offset_y = icon_offset_y;
}
