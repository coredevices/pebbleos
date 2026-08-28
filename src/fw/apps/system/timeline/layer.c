/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "animations.h"
#include "layer.h"
#include "model.h"
#include "relbar.h"

#include "applib/fonts/fonts.h"
#include "applib/graphics/gpath.h"
#include "applib/graphics/graphics.h"
#include "applib/preferred_content_size.h"
#include "applib/ui/action_bar_layer.h"
#include "applib/ui/kino/kino_layer.h"
#include "applib/ui/kino/kino_reel/scale_segmented.h"
#include "applib/ui/property_animation.h"
#include "applib/ui/window.h"
#include "kernel/pbl_malloc.h"
#include "kernel/ui/kernel_ui.h"
#include "kernel/ui/system_icons.h"
#include "popups/timeline/peek_animations.h"
#include "process_state/app_state/app_state.h"
#include "resource/resource_ids.auto.h"
#include "pbl/services/clock.h"
#include "pbl/services/i18n/i18n.h"
#include "pbl/services/blob_db/pin_db.h"
#include "pbl/services/timeline/attribute.h"
#include "pbl/services/timeline/layout_layer.h"
#include "pbl/services/timeline/timeline_layout.h"
#include "pbl/services/timeline/timeline_resources.h"
#include <pbl/logging/logging.h>
#include "system/passert.h"
#include "pbl/util/math.h"
#include "pbl/util/size.h"
#include "pbl/util/string.h"
#include "pbl/util/struct.h"
#include "pbl/util/trig.h"

#include <stdint.h>
#include <time.h>

#define PAST_TOP_MARGIN_EXTRA PBL_IF_RECT_ELSE(10, 38)
#define FUTURE_TOP_MARGIN_EXTRA PBL_IF_RECT_ELSE(10, 18)

typedef struct TimelineLayerStyle {
  GSize sidebar_arrow_size;
  GPoint day_sep_offset;
  uint16_t sidebar_width;
  int16_t fin_offset_x;
  int16_t past_fin_offset_y;
  int16_t future_fin_offset_y;
  uint16_t past_top_margin;
  uint16_t past_thin_pin_margin;
  uint16_t future_top_margin;
  uint16_t left_margin;
  uint16_t right_margin;
  int16_t icon_offset_y;
  uint16_t icon_right_margin;
  uint16_t fat_pin_height;
  uint16_t thin_pin_height;
  uint16_t day_sep_dot_diameter;
  uint16_t day_sep_subtitle_margin;
  int16_t past_day_sep_dot_offset_y;
  int16_t future_day_sep_dot_offset_y;
} TimelineLayerStyle;

#define MARGIN_MEDIUM PBL_IF_RECT_ELSE(4, 13)

static const TimelineLayerStyle s_style_medium = {
  .sidebar_arrow_size.w = PBL_IF_RECT_ELSE(10, 7),
  .sidebar_arrow_size.h = PBL_IF_RECT_ELSE(20, 28),
  .sidebar_width = PBL_IF_RECT_ELSE(30, 48),
  .past_fin_offset_y = PBL_IF_ROUND_ELSE(-12, 0),
  .future_fin_offset_y = PBL_IF_ROUND_ELSE(-20, 0),
  .past_top_margin = PBL_IF_RECT_ELSE(10, 18),
  .future_top_margin = PBL_IF_RECT_ELSE(10, 39),
  .left_margin = MARGIN_MEDIUM,
  .right_margin = MARGIN_MEDIUM,
  .icon_right_margin = MARGIN_MEDIUM,
  .fat_pin_height = 110,
  // PBL-42540: This property is dependent on the screen size. The thin pin height is the
  // remainder of the screen space after the fat pin.
  .thin_pin_height = PBL_IF_RECT_ELSE(66, 43),
  .day_sep_dot_diameter = 9,
  .day_sep_offset.x = PBL_IF_ROUND_ELSE(12, 0),
  .day_sep_offset.y = -12,
  .day_sep_subtitle_margin = PEEK_LAYER_SUBTITLE_MARGIN,
  .past_day_sep_dot_offset_y = PBL_IF_ROUND_ELSE(-17, 0),
  .future_day_sep_dot_offset_y = PBL_IF_ROUND_ELSE(-13, 0),
};

static const TimelineLayerStyle s_style_large = {
  .sidebar_arrow_size.w = PBL_IF_RECT_ELSE(14, 10),
  .sidebar_arrow_size.h = PBL_IF_RECT_ELSE(28, 40),
  .sidebar_width = PBL_IF_RECT_ELSE(34, 51),
  .fin_offset_x = 4,
  .future_fin_offset_y = 37,
  .past_top_margin = PBL_IF_RECT_ELSE(7, 18),
  .past_thin_pin_margin = 11,
  .future_top_margin = PBL_IF_RECT_ELSE(7, 39),
  .left_margin = 9,
  .right_margin = 14,
  .icon_offset_y = 3,
  .icon_right_margin = PBL_IF_RECT_ELSE(6, 12),
  .fat_pin_height = 131,
  // PBL-42540: This property is dependent on the screen size.
  .thin_pin_height = 88,
  .day_sep_dot_diameter = 12,
  .day_sep_offset.y = -21,
  .past_day_sep_dot_offset_y = PBL_IF_RECT_ELSE(-16, -32),
  .future_day_sep_dot_offset_y = 16,
};

static const TimelineLayerStyle * const s_styles[NumPreferredContentSizes] = {
  [PreferredContentSizeSmall] = &s_style_medium,
  [PreferredContentSizeMedium] = &s_style_medium,
  [PreferredContentSizeLarge] = &s_style_large,
  [PreferredContentSizeExtraLarge] = &s_style_large,
};

static int16_t s_height_offsets[TIMELINE_NUM_ITEMS_IN_TIMELINE_LAYER];
static const int s_visible_items[] = {1, 2};
static const int s_nonvisible_items[] = {0, TIMELINE_NUM_ITEMS_IN_TIMELINE_LAYER - 1};

static const TimelineLayerStyle *prv_get_style(void) {
  return s_styles[PreferredContentSizeDefault];
}

uint16_t timeline_layer_get_fat_pin_height(void) {
  return prv_get_style()->fat_pin_height;
}

uint16_t timeline_layer_get_ideal_sidebar_width(void) {
  return prv_get_style()->sidebar_width;
}

bool timeline_layer_sidebar_is_on_right(void) {
  return action_bar_layer_is_on_right();
}

int16_t timeline_layer_get_icon_outer_inset(void) {
  return prv_get_style()->icon_right_margin;
}

int16_t timeline_layer_get_pin_text_origin_x(void) {
  if (timeline_layer_sidebar_is_on_right()) {
    return 0;
  }
  const TimelineLayerStyle *style = prv_get_style();
  return (int16_t)(style->sidebar_width + style->right_margin - style->left_margin);
}

static void prv_inset_rect_for_sidebar(GRect *frame, int16_t sidebar_width) {
  frame->size.w -= sidebar_width;
  if (!timeline_layer_sidebar_is_on_right()) {
    frame->origin.x += sidebar_width;
  }
}

///////////////////////////////////////////////////////////
// Drawing functions
///////////////////////////////////////////////////////////

static int prv_get_scroll_delta(TimelineLayer *timeline_layer) {
  return (timeline_layer->scroll_direction == TimelineScrollDirectionUp ? -1 : 1);
}

static int prv_get_index_delta(TimelineLayer *timeline_layer) {
  return timeline_layer->move_delta * prv_get_scroll_delta(timeline_layer);
}

static void prv_set_layout_hidden(TimelineLayout *layout, bool hidden) {
  layer_set_hidden((Layer *)layout, hidden);
  layer_set_hidden((Layer *)&layout->icon_layer, hidden);
}

static LayoutLayerMode prv_get_mode(int index) {
  switch (index) {
    default:
    case 1:
      return LayoutLayerModePinnedFat;
    case 2:
      return LayoutLayerModePinnedThin;
  }
}

static void prv_get_size_for(LayoutLayerMode layout_mode, const GRect *bounds, GSize *size) {
  const TimelineLayerStyle *style = prv_get_style();
  const int16_t width = bounds->size.w - (style->left_margin + style->right_margin);
  switch (layout_mode) {
    default:
    case LayoutLayerModePinnedFat:
      *size = GSize(width, style->fat_pin_height);
      break;
    case LayoutLayerModePinnedThin:
      *size = GSize(width, style->thin_pin_height);
      break;
  }
}

static void prv_get_frame(TimelineLayer *layer, int index, GRect *frame) {
  const TimelineLayerStyle *style = prv_get_style();
  index = CLIP(index, 0, (int)ARRAY_LENGTH(s_height_offsets) - 1);
  const GRect *bounds = &layer->layer.bounds;
  frame->origin = GPoint(style->left_margin, s_height_offsets[index]);
  prv_get_size_for(prv_get_mode(index), bounds, &frame->size);
}

void timeline_layer_get_layout_frame(TimelineLayer *layer, int index, GRect *frame_out) {
  prv_get_frame(layer, index, frame_out);
}

static void prv_get_icon_frame_exact(TimelineLayer *layer, int index, GRect *icon_frame) {
  const TimelineLayerStyle *style = prv_get_style();
  GRect frame;
  prv_get_frame(layer, index, &frame);
  frame.origin.y += style->icon_offset_y;
  // On the right, grow the pin frame so the icon sits icon_right_margin from the screen edge.
  if (timeline_layer_sidebar_is_on_right()) {
    frame.size.w += style->right_margin - style->icon_right_margin;
  }
  timeline_layout_get_icon_frame(&frame, layer->scroll_direction, icon_frame);
}

#if PBL_ROUND
static void prv_get_icon_frame_centered(TimelineLayer *layer, int index, GRect *icon_frame) {
  const int center_index = 1;
  const GRect *bounds = &((Layer *)layer)->bounds;
  prv_get_icon_frame_exact(layer, center_index, icon_frame);
  icon_frame->origin.y += prv_get_scroll_delta(layer) * (index - center_index) * bounds->size.h / 2;
}
#endif

void timeline_layer_get_icon_frame(TimelineLayer *layer, int index, GRect *icon_frame) {
  return PBL_IF_RECT_ELSE(prv_get_icon_frame_exact,
                          prv_get_icon_frame_centered)(layer, index, icon_frame);
}

static void prv_get_end_of_timeline_frame(TimelineLayer *layer, int index, GRect *frame) {
  prv_get_frame(layer, index, frame);
  const bool is_future = (layer->scroll_direction == TimelineScrollDirectionDown);
  const TimelineLayerStyle *style = prv_get_style();
  gpoint_add_eq(&frame->origin,
                GPoint(style->fin_offset_x,
                       is_future ? style->future_fin_offset_y : style->past_fin_offset_y));
  prv_inset_rect_for_sidebar(frame, PBL_IF_RECT_ELSE(style->sidebar_width, 0));
}

static void prv_get_day_sep_frame(TimelineLayer *layer, int index, GRect *frame) {
  prv_get_frame(layer, index, frame);
  const bool is_future = (layer->scroll_direction == TimelineScrollDirectionDown);
  const TimelineLayerStyle *style = prv_get_style();
  frame->origin.y += is_future ? style->future_day_sep_dot_offset_y :
                                 style->past_day_sep_dot_offset_y;
  // Remove the built-in margins and subtract the sidebar
  frame->origin.x -= style->left_margin;
  frame->size.w += (style->left_margin + style->right_margin);
  prv_inset_rect_for_sidebar(frame, PBL_IF_RECT_ELSE(style->sidebar_width, 0));
}

static void prv_get_day_sep_show_frame(TimelineLayer *layer, GRect *frame) {
  const GRect *bounds = &((Layer *)layer)->bounds;
  const TimelineLayerStyle *style = prv_get_style();
  *frame = (GRect) {
    .origin = gpoint_add(bounds->origin, style->day_sep_offset),
    .size.w = bounds->size.w,
    .size.h = bounds->size.h,
  };
  prv_inset_rect_for_sidebar(frame, style->sidebar_width);
}

static void prv_create_layout(TimelineLayer *layer, TimelineIterState *state, int index) {
  TimelineItem *item = &state->pin;
  TimelineLayoutInfo *info = app_malloc_check(sizeof(TimelineLayoutInfo));
  timeline_layout_init_info(info, item, state->current_day);
  info->scroll_direction = layer->scroll_direction;
  info->app_id = item->header.parent_id;

  GRect rect;
  prv_get_frame(layer, index, &rect);
  const LayoutLayerConfig config = {
    .frame = &rect,
    .attributes = &item->attr_list,
    .mode = prv_get_mode(index),
    .app_id = &item->header.parent_id,
    .context = info,
  };
  TimelineLayout *layout = (TimelineLayout *)layout_create(item->header.layout, &config);
  layer_add_child(&layer->layouts_layer, (Layer *)layout);
  GRect icon_rect;
  timeline_layer_get_icon_frame(layer, index, &icon_rect);
  layer_set_frame((Layer *)&layout->icon_layer, &icon_rect);
  layer_add_child(&layer->layouts_layer, (Layer *)&layout->icon_layer);
  layer->layouts[index] = layout;
  layer->layouts_info[index] = info;
}

static void prv_destroy_layout(TimelineLayer *layer, int index) {
  TimelineLayout *timeline_layout = layer->layouts[index];
  timeline_layout->is_being_destroyed = true;
  layer_remove_from_parent((Layer *)timeline_layout);
  layout_destroy((LayoutLayer *)timeline_layout);
  layer->layouts[index] = NULL;

  app_free(layer->layouts_info[index]);
  layer->layouts_info[index] = NULL;
}

static void prv_destroy_nonvisible_items(TimelineLayer *layer) {
  for (int i = 0; i < (int)ARRAY_LENGTH(s_nonvisible_items); i++) {
    TimelineLayout *timeline_layout = layer->layouts[s_nonvisible_items[i]];
    if (timeline_layout) {
      prv_destroy_layout(layer, s_nonvisible_items[i]);
      layer->layouts[s_nonvisible_items[i]] = NULL;
    }
  }
}

static void prv_set_layouts_to_final_position(TimelineLayer *layer) {
  for (int i = 0; i < TIMELINE_NUM_VISIBLE_ITEMS; i++) {
    TimelineLayout *layout = layer->layouts[s_visible_items[i]];
    if (layout) {
      GRect frame;
      prv_get_frame(layer, i + 1, &frame);
      layer_set_frame((Layer *)layout, &frame);
    }
  }
}

static void prv_hide_non_current_day_items(TimelineLayer *layer) {
  for (int i = 0; i < TIMELINE_NUM_ITEMS_IN_TIMELINE_LAYER; i++) {
    TimelineLayout *layout = layer->layouts[i];
    TimelineIterState *state = timeline_model_get_iter_state(i - 1);
    if (layout && state->current_day != layer->current_day) {
      prv_set_layout_hidden(layout, true);
    }
  }
}

static void prv_reset_layouts(TimelineLayer *layer) {
  int num_items = timeline_model_get_num_items();
  for (int i = 0; i < (int)ARRAY_LENGTH(s_visible_items); i++) {
    if (layer->layouts[s_visible_items[i]]) {
      prv_destroy_layout(layer, s_visible_items[i]);
    }
    TimelineIterState *state = timeline_model_get_iter_state(i);
    if (!state) {
      continue;
    }
    TimelineNode *node = state->node;
    if (i < num_items && node) {
      prv_create_layout(layer, state, s_visible_items[i]);
    }
  }
}

static void prv_update_pins_mode(TimelineLayer *layer) {
  for (int i = 0; i < (int)ARRAY_LENGTH(s_visible_items); i++) {
    TimelineLayout *timeline_layout = layer->layouts[s_visible_items[i]];
    if (timeline_layout) {
      LayoutLayerMode mode = prv_get_mode(i + 1);
      layout_set_mode((LayoutLayer *)timeline_layout, mode);
    }
  }
}

///////////////////////////////////////////////////////////
// Drawing functions
///////////////////////////////////////////////////////////

// An animation that moves a layer from an initial position to its final position
static Animation *prv_create_layout_up_down_animation(
    TimelineLayout *timeline_layout, int to_index, TimelineLayer *timeline_layer,
    uint32_t duration, InterpolateInt64Function interpolate) {
  const int from_index = to_index + prv_get_index_delta(timeline_layer);
  GRect from, to, icon_from, icon_to;
  prv_get_frame(timeline_layer, from_index, &from);
  prv_get_frame(timeline_layer, to_index, &to);
  timeline_layer_get_icon_frame(timeline_layer, from_index, &icon_from);
  timeline_layer_get_icon_frame(timeline_layer, to_index, &icon_to);
  layer_set_frame((Layer *)timeline_layout, &from);
  return timeline_layout_create_up_down_animation(timeline_layout, &from, &to, &icon_from, &icon_to,
                                                  duration, interpolate);
}

static Animation *prv_create_end_of_timeline_animation(
    TimelineLayer *layer, int to_index, uint32_t duration, InterpolateInt64Function interpolate) {
  const int from_index = to_index + prv_get_index_delta(layer);
  GRect from_frame, to_frame;
  prv_get_end_of_timeline_frame(layer, from_index, &from_frame);
  prv_get_end_of_timeline_frame(layer, to_index, &to_frame);
  PropertyAnimation *prop_animation = property_animation_create_layer_frame(
      (Layer *)&layer->end_of_timeline, &from_frame, &to_frame);
  Animation *animation = property_animation_get_animation(prop_animation);
  animation_set_duration(animation, duration);
  animation_set_custom_interpolation(animation, interpolate);
  animation_set_handlers(animation, (AnimationHandlers) {
    .stopped = timeline_animation_layer_stopped_cut_to_end,
  }, prop_animation);
  return animation;
}

Animation *timeline_layer_create_day_sep_hide(TimelineLayer *timeline_layer) {
  GRect frame;
  layer_get_global_frame((Layer *)&timeline_layer->day_separator, &frame);
  const TimelineLayerStyle *style = prv_get_style();
  const int16_t expanded_layer_height =
      frame.size.h + 2 * (style->left_margin + style->right_margin);
  // move way off screen: 2x height * (1/2 - move_delta) in integer logic
  const int16_t TARGET_Y = (2 * expanded_layer_height * (1 - 2 * timeline_layer->move_delta)) / 2;
  const GRect scale_to = GRect(frame.origin.x + frame.size.w / 2, // go to the center
                               TARGET_Y, 0, 0);
  peek_layer_set_scale_to(&timeline_layer->day_separator, scale_to);

  // out anim
  GRect to = timeline_layer->day_separator.layer.frame;
  to.origin = GPoint(0, frame.size.h * timeline_layer->move_delta); // all the way off screen
  PropertyAnimation *prop_anim =
      property_animation_create_layer_frame((Layer *)&timeline_layer->day_separator, NULL, &to);
  Animation *anim = property_animation_get_animation(prop_anim);
  animation_set_duration(anim, TIMELINE_UP_DOWN_ANIMATION_DURATION_MS);
  animation_set_custom_interpolation(anim, timeline_animation_interpolate_moook_soft);

  peek_layer_clear_fields(&timeline_layer->day_separator);
  peek_layer_play(&timeline_layer->day_separator);

  return anim;
}

void timeline_layer_set_day_sep_frame(TimelineLayer *timeline_layer, const GRect *frame) {
  layer_set_hidden((Layer *)&timeline_layer->day_separator, false);
  peek_layer_set_frame(&timeline_layer->day_separator, frame);
}

static void prv_show_day_sep(TimelineLayer *timeline_layer, bool slide) {
  // update the day to show the right date
  timeline_layer->current_day = timeline_model_get_current_state()->current_day;
  char friendly_date[TIME_STRING_REQUIRED_LENGTH];
  char month_and_day[TIME_STRING_REQUIRED_LENGTH];
  clock_get_friendly_date(friendly_date, TIME_STRING_REQUIRED_LENGTH, timeline_layer->current_day);
  clock_get_month_named_date(month_and_day, TIME_STRING_REQUIRED_LENGTH,
                             timeline_layer->current_day);

  GRect frame;
  layer_get_global_frame((Layer *)&timeline_layer->day_separator, &frame);
  const GRect icon_from = { grect_center_point(&frame), GSizeZero };
  prv_get_day_sep_show_frame(timeline_layer, &frame);
  peek_layer_set_frame(&timeline_layer->day_separator, &frame);
  TimelineResourceInfo timeline_res = {
    .res_id = TIMELINE_RESOURCE_DAY_SEPARATOR,
  };
  peek_layer_set_icon_with_size(&timeline_layer->day_separator, &timeline_res,
                                TimelineResourceSizeLarge, icon_from);

  if (slide) {
    const bool align_in_frame = true;
    frame.origin.y += PEEK_LAYER_ICON_OFFSET_Y;
    peek_layer_set_scale_to_image(&timeline_layer->day_separator, &timeline_res,
                                  TimelineResourceSizeLarge, frame, align_in_frame);
    peek_layer_set_fields_hidden(&timeline_layer->day_separator, true);
  }

  peek_layer_set_fields(&timeline_layer->day_separator, "", friendly_date, month_and_day);
  peek_layer_play(&timeline_layer->day_separator);
}

void timeline_layer_unfold_day_sep(TimelineLayer *timeline_layer) {
  const bool slide = false;
  prv_show_day_sep(timeline_layer, slide);
}

void timeline_layer_slide_day_sep(TimelineLayer *timeline_layer) {
  const bool slide = true;
  prv_show_day_sep(timeline_layer, slide);
}

static void prv_day_sep_anim_stopped(Animation *anim, bool finished, void *context) {
  TimelineLayer *timeline_layer = context;
  if (finished) {
    timeline_layer_unfold_day_sep(timeline_layer);
  }
}

// TODO: PBL-21717 Day separator on Spalding
Animation *timeline_layer_create_day_sep_show(TimelineLayer *timeline_layer) {
  GRect *from = &((Layer *)&timeline_layer->day_separator)->frame;
  GRect to;
  prv_get_day_sep_show_frame(timeline_layer, &to);
  // Keep the x-axis values until the actual unfold to maintain alignment
  to.origin.x = from->origin.x;
  to.size.w = from->size.w;

  PropertyAnimation *prop_anim =
      property_animation_create_layer_frame((Layer *)&timeline_layer->day_separator, from, &to);
  Animation *anim = property_animation_get_animation(prop_anim);
  animation_set_handlers(anim, (AnimationHandlers) {
    .stopped = prv_day_sep_anim_stopped,
  }, timeline_layer);
  animation_set_duration(anim, TIMELINE_UP_DOWN_ANIMATION_DURATION_MS);
  animation_set_custom_interpolation(anim, timeline_animation_interpolate_moook_soft);
  return anim;
}

#define MAX_UP_DOWN_ANIMATIONS (2 * TIMELINE_NUM_ITEMS_IN_TIMELINE_LAYER + 1)

static Animation *prv_create_up_down_animation(TimelineLayer *layer, uint32_t duration,
                                               InterpolateInt64Function interpolate) {
  Animation *animations[MAX_UP_DOWN_ANIMATIONS] = {};
  int num_animations = 0;
  for (int i = 0; i < TIMELINE_NUM_ITEMS_IN_TIMELINE_LAYER; i++) {
    if (layer->layouts[i]) {
      animations[num_animations++] =
          prv_create_layout_up_down_animation(layer->layouts[i], i, layer, duration, interpolate);
      animations[num_animations++] =
          (Animation *)kino_layer_create_play_section_animation(
              &layer->layouts[i]->icon_layer, 0, TIMELINE_UP_DOWN_ANIMATION_DURATION_MS);
    } else if (i == 2 || i == 3) {
      animations[num_animations++] =
          prv_create_end_of_timeline_animation(layer, i, duration, interpolate);
      break;
    }
  }

// TODO: PBL-21982: Only support rectangular screen for now
#if PBL_RECT
  Animation *relbar_animation =
    timeline_relbar_layer_create_animation(layer, duration, interpolate);
  if (relbar_animation) {
    animations[num_animations++] = relbar_animation;
  }
#endif

  Animation *animation = animation_spawn_create_from_array(animations, num_animations);
  return animation;
}

static void prv_place_day_separator(TimelineLayer *layer) {
  if (!layer_get_hidden((Layer *)&layer->day_separator)) {
    // already on screen
    return;
  }

  GRect day_sep_frame = ((Layer *)&layer->day_separator)->frame;
  // substitute the day separator for the hidden pin
  TimelineLayout *prev = layer->layouts[s_nonvisible_items[0]];
  TimelineLayout *next = layer->layouts[s_visible_items[1]];
  if (prev && layer_get_hidden(&prev->layout_layer.layer)) {
    prv_get_day_sep_frame(layer, 0, &day_sep_frame);
  } else if (next && layer_get_hidden(&next->layout_layer.layer)) {
    prv_get_day_sep_frame(layer, 2, &day_sep_frame);
  } else {
    // don't show the day separator
    return;
  }

  layer_set_frame((Layer *)&layer->day_separator, &day_sep_frame);
  layer_set_hidden((Layer *)&layer->day_separator, false);
}

static void prv_place_end_of_timeline(TimelineLayer *timeline_layer) {
  const bool was_hidden = layer_get_hidden((Layer *)&timeline_layer->end_of_timeline);
  const bool is_hidden = (timeline_layer_should_animate_day_separator(timeline_layer) ||
                          timeline_layer->layouts[2]);
  layer_set_hidden((Layer *)&timeline_layer->end_of_timeline, is_hidden);
  GRect frame;
  prv_get_end_of_timeline_frame(timeline_layer, is_hidden ? 3 : 2, &frame);
  layer_set_frame((Layer *)&timeline_layer->end_of_timeline, &frame);
  if (was_hidden) {
    kino_layer_rewind(&timeline_layer->end_of_timeline);
  }
  kino_layer_play(&timeline_layer->end_of_timeline);
};

static void prv_up_down_stopped(Animation *animation, bool is_finished, void *context) {
  TimelineLayer *layer = context;
  prv_update_pins_mode(layer);
  prv_set_layouts_to_final_position(layer);
  prv_destroy_nonvisible_items(layer);
  prv_place_day_separator(layer);
  prv_place_end_of_timeline(layer);
}

static void prv_mode_change_update(Animation *animation, AnimationProgress normalized) {
  TimelineLayer *timeline_layer = animation_get_context(animation);
  const AnimationProgress bounce_back_length =
      (interpolate_moook_out_duration() * ANIMATION_NORMALIZED_MAX) / interpolate_moook_duration();
  if (normalized >= ANIMATION_NORMALIZED_MAX - bounce_back_length) {
    prv_update_pins_mode(timeline_layer);
    animation_unschedule(animation);
  }
}

Animation *timeline_layer_create_up_down_animation(TimelineLayer *layer, uint32_t duration,
                                                   InterpolateInt64Function interpolate) {
  Animation *animation = prv_create_up_down_animation(layer, duration, interpolate);

  animation_set_handlers(animation, (AnimationHandlers) {
    .stopped = prv_up_down_stopped,
  }, layer);

  static const AnimationImplementation s_mode_change_impl = {
    .update = prv_mode_change_update,
  };

  Animation *mode_change = animation_create();
  animation_set_implementation(mode_change, &s_mode_change_impl);
  animation_set_handlers(mode_change, (AnimationHandlers){ 0 }, layer);

  return animation_spawn_create(animation, mode_change, NULL);
}

#if PBL_ROUND
static void prv_draw_round_flip(GContext *ctx, const GRect *layer_bounds, const int sidebar_x) {
  // Use a radius larger than the screen's radius so we don't see the top/bottom of the circle
  int16_t circle_radius = DISP_COLS * 3 / 4;
  const int16_t flip_overlap_region_width = layer_bounds->size.w / 5;
  const GPoint bounds_center = grect_center_point(layer_bounds);
  const int16_t flip_point_x = bounds_center.x - (flip_overlap_region_width / 2);
  // If the origin x value is not past the flip point, draw a colored circle starting at the x pos
  if (sidebar_x <= flip_point_x) {
    // Don't draw the circle in the flip region overlap; we want an instantaneous jump past this
    // region during the flip
    const int16_t circle_left_edge_x = MIN(sidebar_x, flip_point_x);
    const GPoint circle_center = GPoint(circle_left_edge_x + circle_radius, bounds_center.y);
    graphics_fill_circle(ctx, circle_center, circle_radius);
  } else {
    // Otherwise, use fill_radial to fill the sidebar as a radial on the right side of the screen
    const GPoint circle_center = GPoint(sidebar_x - circle_radius, bounds_center.y);
    // Add half the final sidebar width to the radius so we see a bounce-back effect at the end
    const TimelineLayerStyle *style = prv_get_style();
    circle_radius += style->sidebar_width / 2;
    graphics_fill_radial_internal(ctx, circle_center, circle_radius,
                                  layer_bounds->size.w - circle_center.x,
                                  0, TRIG_MAX_ANGLE);
  }
}
#endif

static void prv_update_proc(struct Layer *layer, GContext* ctx) {
  TimelineLayer *timeline_layer = (TimelineLayer *)layer;
  const GRect *bounds = &layer->bounds;

  graphics_context_set_fill_color(ctx, GColorWhite);
  graphics_fill_rect(ctx, &(GRect) { .size = bounds->size });

  AnimationProgress progress;
  if (timeline_layer->animating_intro_or_exit &&
      animation_get_progress(timeline_layer->animation, &progress)) {
    const GPoint offset = { PEEK_ANIMATIONS_SPEED_LINES_OFFSET_X,
                            interpolate_int64_linear(progress, 0, -DISP_ROWS) };
    graphics_context_set_fill_color(ctx, GColorBlack);
    peek_animations_draw_timeline_speed_lines(ctx, offset);
  }

  const int16_t sidebar_width = timeline_layer->sidebar_width;
  const int16_t sidebar_x = timeline_layer_sidebar_is_on_right() ?
      (bounds->size.w - sidebar_width) : 0;
  const GRect sidebar_rect = GRect(sidebar_x, 0, sidebar_width, bounds->size.h);
  graphics_context_set_fill_color(ctx, timeline_layer->sidebar_color);

  // On round displays, draw the round flip effect if we're animating the intro or exit and then
  // return early so we don't draw the arrow notch
#if PBL_ROUND
  if (timeline_layer->animating_intro_or_exit) {
    prv_draw_round_flip(ctx, bounds, sidebar_rect.origin.x);
    return;
  }
#endif

  graphics_fill_rect(ctx, &sidebar_rect);
  const bool sidebar_on_right = timeline_layer_sidebar_is_on_right();
  int16_t arrow_base_x = sidebar_on_right ? (bounds->size.w - sidebar_width) : (sidebar_width - 1);
#if PBL_ROUND
  arrow_base_x += sidebar_on_right ? -1 : 1;
#endif
  const TimelineLayerStyle *style = prv_get_style();
  const GSize arrow_size = style->sidebar_arrow_size;
  const int16_t arrow_base_center_y = PBL_IF_RECT_ELSE(16, bounds->size.h / 2);
  const int16_t toward_content = sidebar_on_right ? -1 : 1;
  const int16_t arrow_point_x_offset =
      PBL_IF_RECT_ELSE(toward_content, -toward_content) * arrow_size.w;
  GPath arrow_path = {
    .num_points = 3,
    .points = (GPoint[]) { { arrow_base_x, arrow_base_center_y  - (arrow_size.h / 2) },
                           { arrow_base_x + arrow_point_x_offset, arrow_base_center_y },
                           { arrow_base_x, arrow_base_center_y  + (arrow_size.h / 2) } }
  };

  if (timeline_layer->scroll_direction == TimelineScrollDirectionUp) {
    // arrow is in a different position for past & future, but only on rectangular displays
    gpath_move_to(&arrow_path,
                  PBL_IF_RECT_ELSE(GPoint(0, (style->thin_pin_height +
                                              style->past_thin_pin_margin)), GPointZero));
  }

  graphics_context_set_antialiased(ctx, true);
  const GColor arrow_fill_color = PBL_IF_RECT_ELSE(timeline_layer->sidebar_color, GColorWhite);
  graphics_context_set_fill_color(ctx, arrow_fill_color);
  gpath_draw_filled(ctx, &arrow_path);
  graphics_context_set_antialiased(ctx, false);
}

/////////////////////////////////////////
// Public functions
/////////////////////////////////////////

// when we create a new next or previous item, we want it out of view so we can animate it in
void timeline_layer_set_next_item(TimelineLayer *layer, int index) {
  PBL_LOG_DBG("Setting next item with index %d", index);
  TimelineIterState *iter_state = timeline_model_get_iter_state_with_timeline_idx(index);
  if (!iter_state) {
    return;
  }
  if (layer->layouts[s_nonvisible_items[1]]) {
    prv_destroy_layout(layer, s_nonvisible_items[1]);
  }
  prv_create_layout(layer, iter_state, s_nonvisible_items[1]);
}

void timeline_layer_set_prev_item(TimelineLayer *layer, int index) {
  PBL_LOG_DBG("Setting prev item with index %d", index);
  TimelineIterState *iter_state = timeline_model_get_iter_state_with_timeline_idx(index);
  if (!iter_state) {
    return;
  }
  if (layer->layouts[s_nonvisible_items[0]]) {
    prv_destroy_layout(layer, s_nonvisible_items[0]);
  }
  prv_create_layout(layer, iter_state, s_nonvisible_items[0]);

  if (iter_state->current_day != layer->current_day) {
    // we moved back to an item from the previous day, display the day separator
    // by first hiding the pin of the "last" day that is about to come in and placing it there
    if (layer->layouts[0]) {
      prv_set_layout_hidden(layer->layouts[0], true);
    }
    prv_place_day_separator(layer);
  } else {
    // continue to hide the day separator
    layer_set_hidden((Layer *)&layer->day_separator, true);
  }
}

bool timeline_layer_should_animate_day_separator(TimelineLayer *layer) {
  return !layer_get_hidden((Layer *)&layer->day_separator);
}

void timeline_layer_move_data(TimelineLayer *layer, int delta) {
  PBL_ASSERTN(delta == 1 || delta == -1);
  if (delta == 1) {
    if (layer->layouts[s_nonvisible_items[0]]) {
      prv_destroy_layout(layer, s_nonvisible_items[0]);
    }
    for (int i = 0; i < TIMELINE_NUM_ITEMS_IN_TIMELINE_LAYER - 1; i++) {
      layer->layouts[i] = layer->layouts[i + 1];
      layer->layouts_info[i] = layer->layouts_info[i + 1];
    }
    layer->layouts[TIMELINE_NUM_ITEMS_IN_TIMELINE_LAYER - 1] = NULL;
    layer->layouts_info[TIMELINE_NUM_ITEMS_IN_TIMELINE_LAYER - 1] = NULL;
  } else if (delta == -1) {
    if (layer->layouts[s_nonvisible_items[1]]) {
      prv_destroy_layout(layer, s_nonvisible_items[1]);
    }
    for (int i = TIMELINE_NUM_ITEMS_IN_TIMELINE_LAYER - 1; i > 0; i--) {
      layer->layouts[i] = layer->layouts[i - 1];
      layer->layouts_info[i] = layer->layouts_info[i - 1];
    }
    layer->layouts[0] = NULL;
    layer->layouts_info[0] = NULL;
  }

  layer->move_delta = delta * prv_get_scroll_delta(layer);

  // hide other day's pins before the animation shows them
  prv_hide_non_current_day_items(layer);
}

TimelineLayout *timeline_layer_get_current_layout(TimelineLayer *timeline_layer) {
  return timeline_layer->layouts[TIMELINE_LAYER_FIRST_VISIBLE_LAYOUT];
}

void timeline_layer_reset(TimelineLayer *layer) {
  // reset the animation
  animation_unschedule(layer->animation);
  layer->animation = NULL;

  // reset the day separator
  layer_set_hidden((Layer *)&layer->day_separator, true);

  TimelineResourceInfo timeline_res = {
    .res_id = TIMELINE_RESOURCE_DAY_SEPARATOR,
  };
  peek_layer_set_icon(&layer->day_separator, &timeline_res);

  // reset the layouts
  prv_reset_layouts(layer);
  prv_destroy_nonvisible_items(layer);
  timeline_layer_set_layouts_hidden(layer, false);

  // PBL-18815: Reset the current day in case the pin was deleted. This should later be animated.
  const int index = TIMELINE_LAYER_FIRST_VISIBLE_LAYOUT;
  if (layer->layouts[index]) {
    layer->current_day = layer->layouts[index]->info->current_day;
  }

  prv_hide_non_current_day_items(layer);
  prv_place_day_separator(layer);
  prv_place_end_of_timeline(layer);

// TODO: PBL-21982: Only support rectangular screen for now
#if PBL_RECT
  timeline_relbar_layer_reset(layer);
#endif
}

void timeline_layer_set_sidebar_color(TimelineLayer *timeline_layer, GColor color) {
  timeline_layer->sidebar_color = color;
}

void timeline_layer_set_sidebar_width(TimelineLayer *timeline_layer, int16_t width) {
  timeline_layer->sidebar_width = width;
}

static void prv_sidebar_setter(void *context, int16_t value) {
  TimelineLayer *timeline_layer = context;
  timeline_layer->sidebar_width = value;
  layer_mark_dirty((Layer *)timeline_layer);
}

static int16_t prv_sidebar_getter(void *context) {
  TimelineLayer *timeline_layer = context;
  return timeline_layer->sidebar_width;
}

Animation *timeline_layer_create_sidebar_animation(TimelineLayer *timeline_layer,
                                                   int16_t to_sidebar_width) {
  static const PropertyAnimationImplementation s_implementation = {
    .base.update = (AnimationUpdateImplementation) property_animation_update_int16,
    .accessors.setter.int16 = prv_sidebar_setter,
    .accessors.getter.int16 = prv_sidebar_getter,
  };
  Animation *animation = (Animation *)property_animation_create(
      &s_implementation, timeline_layer, &timeline_layer->sidebar_width, &to_sidebar_width);
  animation_set_duration(animation, interpolate_moook_in_duration());
  animation_set_custom_interpolation(animation, interpolate_moook_in_only);
  return animation;
}

static void prv_speed_lines_update(Animation *animation, AnimationProgress progress) {}

Animation *timeline_layer_create_speed_lines_animation(TimelineLayer *timeline_layer) {
  static const AnimationImplementation s_speed_lines_impl = {
    .update =  prv_speed_lines_update,
  };
  Animation *animation = animation_create();
  animation_set_implementation(animation, &s_speed_lines_impl);
  const unsigned int num_jump_frames = 3;
  animation_set_duration(animation, num_jump_frames * ANIMATION_TARGET_FRAME_INTERVAL_MS);
  timeline_layer->animation = animation;
  return animation;
}

static Animation *prv_create_bounce_back_animation(Layer *layer, const GRect *to_orig,
                                                   GPoint direction) {
  GRect from = *to_orig;
  GRect to = *to_orig;
  gpoint_add_eq(&from.origin, GPoint(direction.x * INTERPOLATE_MOOOK_BOUNCE_BACK,
                                     direction.y * INTERPOLATE_MOOOK_BOUNCE_BACK));
  layer_set_frame(layer, &from);
  Animation *animation = (Animation *)property_animation_create_layer_frame(layer, &from, &to);
  animation_set_curve(animation, AnimationCurveEaseOut);
  animation_set_duration(animation, TIMELINE_LAYER_SLIDE_MS);
  animation_set_handlers(animation, (AnimationHandlers) {
    .stopped = timeline_animation_layer_stopped_cut_to_end,
  }, animation);
  return animation;
}

Animation *timeline_layer_create_bounce_back_animation(TimelineLayer *layer, GPoint direction) {
  Animation *animations[TIMELINE_NUM_VISIBLE_ITEMS + 2] = {};
  int num_animations = 0;

  for (int i = 0; i < TIMELINE_NUM_VISIBLE_ITEMS; i++) {
    TimelineLayout *layout = layer->layouts[s_visible_items[i]];
    if (layout) {
      GRect frame;
      prv_get_frame(layer, i + 1, &frame);
      animations[num_animations++] = prv_create_bounce_back_animation((Layer *)layout, &frame,
                                                                      direction);
    }
  }

  const GRect *day_sep_from = &((Layer *)&layer->day_separator)->frame;
  animations[num_animations++] = prv_create_bounce_back_animation((Layer *)&layer->day_separator,
                                                                  day_sep_from, direction);

  const GRect *fin_from = &layer->end_of_timeline.layer.frame;
  animations[num_animations++] = prv_create_bounce_back_animation(
      &layer->end_of_timeline.layer, fin_from, direction);

  Animation *animation = animation_spawn_create_from_array(animations, num_animations);
  return animation;
}

void timeline_layer_set_layouts_hidden(TimelineLayer *layer, bool hidden) {
  for (int i = 0; i < TIMELINE_NUM_ITEMS_IN_TIMELINE_LAYER; i++) {
    TimelineLayout *layout = layer->layouts[i];
    if (layout) {
      prv_set_layout_hidden(layout, hidden);
    }
  }

  layer_set_hidden((Layer *)&layer->end_of_timeline, hidden);
}

void timeline_layer_init(TimelineLayer *layer, const GRect *frame_ref,
                         TimelineScrollDirection scroll_direction) {
  *layer = (TimelineLayer) {};
  // timeline layer
  layer_init(&layer->layer, frame_ref);
  layer_set_clips(&layer->layer, false);
  layer_set_update_proc(&layer->layer, prv_update_proc);
  TimelineIterState *state = timeline_model_get_current_state();
  layer->current_day = NULL_SAFE_FIELD_ACCESS(state, current_day, 0);
  const TimelineLayerStyle *style = prv_get_style();
  // The arrow is inverted on round, so hide it by extending the width of the sidebar
  layer->sidebar_width = frame_ref->size.w + PBL_IF_ROUND_ELSE(style->sidebar_arrow_size.w, 0);
  // layouts
  layer->scroll_direction = scroll_direction;
  layer->move_delta = prv_get_scroll_delta(layer);
  if (scroll_direction == TimelineScrollDirectionUp) {
    s_height_offsets[0] = (PAST_TOP_MARGIN_EXTRA + style->thin_pin_height +
                           (2 * style->fat_pin_height));
    s_height_offsets[1] = (style->past_top_margin + style->thin_pin_height +
                           style->past_thin_pin_margin);
    s_height_offsets[2] = style->past_top_margin;
    s_height_offsets[3] = style->past_top_margin - 2 * style->fat_pin_height;
  } else {
    s_height_offsets[0] = FUTURE_TOP_MARGIN_EXTRA - 2 * style->fat_pin_height;
    s_height_offsets[1] = style->future_top_margin;
    s_height_offsets[2] = style->future_top_margin + style->fat_pin_height;
    s_height_offsets[3] = (style->future_top_margin + style->fat_pin_height +
                           (2 * style->fat_pin_height));
  }
  // layouts layer - contains all the pin
  layer_init(&layer->layouts_layer, &(GRect) { .size = frame_ref->size });
  layer_set_clips(&layer->layouts_layer, false);
  layer_add_child((Layer *)layer, (Layer *)&layer->layouts_layer);

  // day separator
  GRect frame;
  prv_get_day_sep_show_frame(layer, &frame);
  peek_layer_init(&layer->day_separator, &frame);
  const GFont title_font =
      system_theme_get_font_for_size(PreferredContentSizeDefault, TextStyleFont_Title);
  peek_layer_set_title_font(&layer->day_separator, title_font);
  const GFont subtitle_font =
      system_theme_get_font_for_size(PreferredContentSizeDefault, TextStyleFont_PinSubtitle);
  peek_layer_set_subtitle_font(&layer->day_separator, subtitle_font,
                               style->day_sep_subtitle_margin);

  TimelineResourceInfo timeline_res = {
    .res_id = TIMELINE_RESOURCE_DAY_SEPARATOR,
  };
  peek_layer_set_icon(&layer->day_separator, &timeline_res);
  peek_layer_set_background_color(&layer->day_separator, GColorClear);
  peek_layer_set_dot_diameter(&layer->day_separator, style->day_sep_dot_diameter);
  layer_set_hidden((Layer *)&layer->day_separator, true);
  layer_add_child((Layer *)layer, (Layer *)&layer->day_separator);

  // end-of-timeline indicator
  // TODO: PBL-21716 Fin icon layout on Spalding
  prv_get_end_of_timeline_frame(layer, 3, &frame);
  kino_layer_init(&layer->end_of_timeline, &frame);
  kino_layer_set_reel_with_resource(&layer->end_of_timeline, RESOURCE_ID_END_OF_TIMELINE);
  kino_layer_set_alignment(&layer->end_of_timeline, GAlignTop);
  layer_add_child((Layer *)layer, (Layer *)&layer->end_of_timeline);

  // populate the timeline with items
  timeline_layer_reset(layer);

// TODO: PBL-21982: Only support rectangular screen for now
#if PBL_RECT
  // Initialize Relationship bar
  timeline_relbar_layer_init(layer);
#endif
}

void timeline_layer_deinit(TimelineLayer *layer) {
  animation_unschedule_all();
  for (int i = 0; i < TIMELINE_NUM_ITEMS_IN_TIMELINE_LAYER; i++) {
    if (layer->layouts[i]) {
      prv_destroy_layout(layer, i);
    }
  }
  peek_layer_deinit(&layer->day_separator);
  kino_layer_deinit(&layer->end_of_timeline);

// TODO: PBL-21982: Only support rectangular screen for now
#if PBL_RECT
  timeline_relbar_layer_deinit(layer);
#endif
}
