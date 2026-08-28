/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "layer.h"
#include "animations.h"
#include "relbar.h"

#include "applib/graphics/graphics.h"
#include "pbl/services/timeline/timeline_layout.h"
#include <pbl/logging/logging.h>

#include <stdint.h>

#define TIMELINE_FAT_PIN_SIZE (timeline_layer_get_fat_pin_height())
#define SIDEBAR_WIDTH (timeline_layer_get_ideal_sidebar_width())

///////////////////////////////////////////////////////////
// Private functions
///////////////////////////////////////////////////////////
static void prv_rel_bar_reset_offsets(RelationshipBarLayer *relbar_layer,
                                      RelationshipBarOffsetType rel_bar_type) {
  switch (rel_bar_type) {
    case RelationshipBarOffsetTypeCurr:
      relbar_layer->curr_rel_bar.anim_offset = 0;
      break;
    case RelationshipBarOffsetTypePrev:
      relbar_layer->prev_rel_bar.anim_offset = 0;
      break;
    case RelationshipBarOffsetTypeBoth:
      relbar_layer->curr_rel_bar.anim_offset = 0;
      relbar_layer->prev_rel_bar.anim_offset = 0;
      break;
  }
}

static void prv_prev_rel_bar_setter(void *context, int16_t value) {
  TimelineLayer *timeline_layer = context;
  timeline_layer->relbar_layer.prev_rel_bar.anim_offset = value;
  layer_mark_dirty(&timeline_layer->layer);
}

static int16_t prv_prev_rel_bar_getter(void *context) {
  TimelineLayer *timeline_layer = context;
  return timeline_layer->relbar_layer.prev_rel_bar.anim_offset;
}

static void prv_curr_rel_bar_setter(void *context, int16_t value) {
  TimelineLayer *timeline_layer = context;
  timeline_layer->relbar_layer.curr_rel_bar.anim_offset = value;
  layer_mark_dirty(&timeline_layer->layer);
}

static int16_t prv_curr_rel_bar_getter(void *context) {
  TimelineLayer *timeline_layer = context;
  return timeline_layer->relbar_layer.curr_rel_bar.anim_offset;
}

static RelationshipBarType prv_get_pin_relationship(TimelineLayoutInfo *current,
                                                    TimelineLayoutInfo *next) {
  if ((!current) || (current->duration_s == 0) || (current->all_day) || (!next)) {
    // No relationship bar shown
    return RelationshipBarTypeNone;
  }

  const time_t current_end = current->end_time;
  if ((next->timestamp > current_end) && (current->current_day == next->current_day)) {
    // Next pin starts after the end of the current pin
    return RelationshipBarTypeFreeTime;
  } else if ((next->timestamp == current_end) && (current->current_day == next->current_day)) {
    // Next pin starts exactly at the end of the current pin
    return RelationshipBarTypeBackToBack;
  } else if (current->current_day != next->current_day) {
    // Don't show relationship bar when event is across days
    return RelationshipBarTypeNone;
  } else {
    // All other cases considered as overlapped
    return RelationshipBarTypeOverlap;
  }
}

static void prv_rel_bar_stopped(Animation *animation, bool is_finished, void *context) {
  // Don't show the rel bar if the animation was interrupted
  if (!is_finished) {
    TimelineLayer *layer = (TimelineLayer *)context;
    prv_rel_bar_reset_offsets(&layer->relbar_layer, RelationshipBarOffsetTypeBoth);
  }
}

#define REL_BAR_VERT_MARGIN 14
static int prv_get_line_length(TimelineLayer *timeline_layer, GRect *first_icon_frame_out,
                               GRect *second_icon_frame_out) {
  GRect first_icon_frame;
  GRect second_icon_frame;
  timeline_layer_get_icon_frame(timeline_layer, TIMELINE_LAYER_FIRST_VISIBLE_LAYOUT,
                                &first_icon_frame);
  timeline_layer_get_icon_frame(timeline_layer, TIMELINE_LAYER_FIRST_VISIBLE_LAYOUT + 1,
                                &second_icon_frame);
  if (first_icon_frame_out) {
    *first_icon_frame_out = first_icon_frame;
  }
  if (second_icon_frame_out) {
    *second_icon_frame_out = second_icon_frame;
  }

  const int total_space = second_icon_frame.origin.y - grect_get_max_y(&first_icon_frame);
  return total_space - (REL_BAR_VERT_MARGIN * 2);
}

static int prv_get_overlap_line_length(TimelineLayer *layer, GRect *first_icon_frame_out,
                                       GRect *second_icon_frame_out) {
  const int full_line_length =
      prv_get_line_length(layer, first_icon_frame_out, second_icon_frame_out);
  return (3 * full_line_length) / 5;
}

#define REL_BAR_BACK_TO_BACK_OFFSET 10
#define REL_BAR_PREV_ANIM_OFFSET 10

// Create previous rel bar animation
static Animation *prv_create_prev_rel_bar_animation(TimelineLayer *layer, uint32_t duration,
                                                    InterpolateInt64Function interpolate) {
  Animation *prev_rel_bar_anim = NULL;
  prv_rel_bar_reset_offsets(&layer->relbar_layer, RelationshipBarOffsetTypePrev);
  int16_t prev_from_rel_bar_value = 0;
  int16_t prev_to_rel_bar_value = 0;
  static const PropertyAnimationImplementation prev_implementation = {
    .base.update = (AnimationUpdateImplementation) property_animation_update_int16,
    .accessors.setter.int16 = prv_prev_rel_bar_setter,
    .accessors.getter.int16 = prv_prev_rel_bar_getter,
  };

  if (layer->relbar_layer.prev_rel_bar.rel_bar_type == RelationshipBarTypeOverlap) {
    const int overlap_line_length = prv_get_overlap_line_length(layer, NULL, NULL);
    layer->relbar_layer.prev_rel_bar.anim_offset = overlap_line_length;
    prev_from_rel_bar_value = overlap_line_length;
    prev_to_rel_bar_value = 0;
    prev_rel_bar_anim = (Animation *)property_animation_create(
        &prev_implementation, layer, &prev_from_rel_bar_value, &prev_to_rel_bar_value);
  } else if ((layer->relbar_layer.prev_rel_bar.rel_bar_type == RelationshipBarTypeBackToBack) ||
             (layer->relbar_layer.prev_rel_bar.rel_bar_type == RelationshipBarTypeFreeTime)) {
    layer->relbar_layer.prev_rel_bar.anim_offset = REL_BAR_PREV_ANIM_OFFSET;
    prev_from_rel_bar_value = REL_BAR_PREV_ANIM_OFFSET;
    prev_to_rel_bar_value = 0;
    prev_rel_bar_anim = (Animation *)property_animation_create(
        &prev_implementation, layer, &prev_from_rel_bar_value, &prev_to_rel_bar_value);
  }
  if (prev_rel_bar_anim) {
    // Delay to avoid overlapping moving icon with bars
    animation_set_delay(prev_rel_bar_anim, 0);
    animation_set_duration(prev_rel_bar_anim, duration / 3);
  }

  return prev_rel_bar_anim;
}

#define REL_BAR_CURR_OVERLAP_START_OFFSET 10
#define REL_BAR_CURR_ANIM_DELAY(delay) ((2 * delay) / 3)
#define REL_BAR_CURR_ANIM_DURATION(duration) ((2 * duration) / 3)
// Create current rel bar animation
static Animation *prv_create_curr_rel_bar_animation(TimelineLayer *layer, uint32_t duration,
                                                    InterpolateInt64Function interpolate) {
  Animation *curr_rel_bar_anim = NULL;
  prv_rel_bar_reset_offsets(&layer->relbar_layer, RelationshipBarOffsetTypeCurr);
  int16_t curr_from_rel_bar_value = 0;
  int16_t curr_to_rel_bar_value = 0;

  static const PropertyAnimationImplementation curr_implementation = {
    .base.update = (AnimationUpdateImplementation) property_animation_update_int16,
    .accessors.setter.int16 = prv_curr_rel_bar_setter,
    .accessors.getter.int16 = prv_curr_rel_bar_getter,
  };

  if (layer->relbar_layer.curr_rel_bar.rel_bar_type == RelationshipBarTypeOverlap) {
    curr_from_rel_bar_value = REL_BAR_CURR_OVERLAP_START_OFFSET;
    curr_to_rel_bar_value = prv_get_overlap_line_length(layer, NULL, NULL);
    curr_rel_bar_anim = (Animation *)property_animation_create(
        &curr_implementation, layer, &curr_from_rel_bar_value, &curr_to_rel_bar_value);
  } else if ((layer->relbar_layer.curr_rel_bar.rel_bar_type == RelationshipBarTypeBackToBack) ||
             (layer->relbar_layer.curr_rel_bar.rel_bar_type == RelationshipBarTypeFreeTime)) {
    curr_from_rel_bar_value = 0;
    curr_to_rel_bar_value = REL_BAR_BACK_TO_BACK_OFFSET;
    curr_rel_bar_anim = (Animation *)property_animation_create(
        &curr_implementation, layer, &curr_from_rel_bar_value, &curr_to_rel_bar_value);
  }

  if (curr_rel_bar_anim) {
    // Delay to avoid overlapping moving icon with bars
    animation_set_delay(curr_rel_bar_anim,  REL_BAR_CURR_ANIM_DELAY(duration));
    animation_set_duration(curr_rel_bar_anim,  REL_BAR_CURR_ANIM_DURATION(duration));
    animation_set_custom_interpolation(curr_rel_bar_anim, interpolate);
  }

  return curr_rel_bar_anim;
}

#define REL_BAR_LINE_CHECK_LENGTH 6
#define REL_BAR_LINE_WIDTH 2
#define REL_BAR_LINE_HORIZ_OFFSET ((SIDEBAR_WIDTH / 2) + (REL_BAR_LINE_WIDTH / 2))
#define REL_BAR_LINE_NOTCH_HORIZ_OFFSET ((REL_BAR_LINE_CHECK_LENGTH / 2) + (REL_BAR_LINE_WIDTH / 2))

static int16_t prv_sidebar_line_x(const GRect *layer_bounds) {
  if (timeline_layer_sidebar_is_on_right()) {
    return layer_bounds->origin.x + layer_bounds->size.w - REL_BAR_LINE_HORIZ_OFFSET;
  }
  return layer_bounds->origin.x + REL_BAR_LINE_HORIZ_OFFSET - REL_BAR_LINE_WIDTH;
}

// The overlap relationship bar uses a different horizontal baseline than the
// plain/dotted bars (SIDEBAR_WIDTH / 2 rather than REL_BAR_LINE_HORIZ_OFFSET),
// so it keeps its own mirror.
static int16_t prv_overlap_line_x(const GRect *layer_bounds) {
  if (timeline_layer_sidebar_is_on_right()) {
    return layer_bounds->origin.x + layer_bounds->size.w - (SIDEBAR_WIDTH / 2);
  }
  return layer_bounds->origin.x + (SIDEBAR_WIDTH / 2) - REL_BAR_LINE_WIDTH;
}
static void prv_draw_rel_bar_line(TimelineLayer *timeline_layer, GContext* ctx,
                                  bool current, int16_t anim_offset) {
  int16_t prev_offset = 0; // Used to animate the previous animation offset
  if (!current) {
    prev_offset = REL_BAR_PREV_ANIM_OFFSET - anim_offset;
    prev_offset = (timeline_layer->move_delta > 0) ? prev_offset : -prev_offset;
  }

  int16_t curr_offset = anim_offset; // Used to animate the previous animation offset
  if ((current && (curr_offset <= 0)) ||
      ((!current) && ((prev_offset >= REL_BAR_PREV_ANIM_OFFSET) ||
                      (prev_offset <= -REL_BAR_PREV_ANIM_OFFSET)))) {
    return;
  }

  // Choose which offset to use based on current or previous animation
  if (!current) {
    curr_offset = 0;
  } else {
    curr_offset = (-timeline_layer->move_delta) * (REL_BAR_BACK_TO_BACK_OFFSET - curr_offset);
    prev_offset = 0;
  }

  GRect first_icon_frame;
  GRect second_icon_frame;
  const int line_length = (prv_get_line_length(timeline_layer, &first_icon_frame,
                                               &second_icon_frame) - REL_BAR_LINE_WIDTH) / 2;

  // Draw two lines that are centered in the side bar
  // Bar 1
  // Filled rect used to draw line of REL_BAR_LINE_WIDTH stroke width
  graphics_context_set_fill_color(ctx, GColorWhite);
  GRect layer_bounds = timeline_layer->layer.bounds;
  GRect line;
  line.origin.x = prv_sidebar_line_x(&layer_bounds);
  // Account for the size of the icon when positioning vertically
  line.origin.y =
      grect_get_max_y(&first_icon_frame) + REL_BAR_VERT_MARGIN - curr_offset - prev_offset;
  line.size.w = REL_BAR_LINE_WIDTH;
  line.size.h = line_length + curr_offset;
  graphics_fill_rect(ctx, &line);

  // Bottom Notch for Bar 1
  // Filled rect used to draw line of REL_BAR_LINE_WIDTH stroke width
  GRect line2;
  line2.size.w = REL_BAR_LINE_CHECK_LENGTH + REL_BAR_LINE_WIDTH;
  line2.size.h = REL_BAR_LINE_WIDTH;
  line2.origin.x = line.origin.x - REL_BAR_LINE_NOTCH_HORIZ_OFFSET + 1;
  line2.origin.y = grect_get_max_y(&line) - line2.size.h;
  graphics_fill_rect(ctx, &line2);

  // Bar 2
  // Filled rect used to draw line of REL_BAR_LINE_WIDTH stroke width
  line.origin.x = prv_sidebar_line_x(&layer_bounds);
  line.origin.y = second_icon_frame.origin.y - (REL_BAR_VERT_MARGIN + line_length) - prev_offset;
  line.size.w = REL_BAR_LINE_WIDTH;
  line.size.h = line_length - curr_offset;
  graphics_fill_rect(ctx, &line);

  // Top Notch for Bar 2
  // Filled rect used to draw line of REL_BAR_LINE_WIDTH stroke width
  line2.origin.x = line.origin.x - REL_BAR_LINE_NOTCH_HORIZ_OFFSET + 1;
  line2.origin.y = line.origin.y;
  line2.size.w = REL_BAR_LINE_CHECK_LENGTH + REL_BAR_LINE_WIDTH;
  line2.size.h = REL_BAR_LINE_WIDTH;
  graphics_fill_rect(ctx, &line2);
}

#define REL_BAR_DOT_SIZE 2
static void prv_draw_rel_bar_dotted(TimelineLayer *timeline_layer, GContext* ctx,
                                    bool current, int16_t rel_bar_value) {
  int16_t prev_offset = 0; // Used to animate the previous animation offset
  if (!current) {
    prev_offset = REL_BAR_PREV_ANIM_OFFSET - rel_bar_value;
    prev_offset = (timeline_layer->move_delta > 0) ? prev_offset : -prev_offset;
  }

  int16_t curr_offset = rel_bar_value; // Used to animate the current animation offset
  if ((current && (curr_offset <= 0)) ||
      ((!current) && ((prev_offset >= REL_BAR_PREV_ANIM_OFFSET) ||
                      (prev_offset <= -REL_BAR_PREV_ANIM_OFFSET)))) {
    return;
  }


  // Choose which offset to use based on current or previous animation
  if (!current) {
    curr_offset = 0;
  } else {
    curr_offset = (-timeline_layer->move_delta) * (REL_BAR_BACK_TO_BACK_OFFSET - curr_offset);
    prev_offset = 0;
  }

  GRect first_icon_frame;
  GRect second_icon_frame;
  const int line_length =
      prv_get_line_length(timeline_layer, &first_icon_frame, &second_icon_frame) / 3;
  const int dot_padding = 1;
  const int solid_line_length = line_length - dot_padding;
  const int dot_line_length = line_length + dot_padding;

  // Bar 1
  // Filled rect used to draw line of REL_BAR_LINE_WIDTH stroke width
  graphics_context_set_fill_color(ctx, GColorWhite);
  GRect layer_bounds = timeline_layer->layer.bounds;
  GRect line;
  line.origin.x = prv_sidebar_line_x(&layer_bounds);
  // Account for the size of the icon when positioning vertically
  line.origin.y = grect_get_max_y(&first_icon_frame) +
                  REL_BAR_VERT_MARGIN - curr_offset - prev_offset;
  line.size.w = REL_BAR_LINE_WIDTH;
  line.size.h = solid_line_length + curr_offset;
  graphics_fill_rect(ctx, &line);

  // Bottom Notch for Bar 1
  // Filled rect used to draw line of REL_BAR_LINE_WIDTH stroke width
  GRect notch;
  notch.origin.x = line.origin.x - REL_BAR_LINE_NOTCH_HORIZ_OFFSET + 1;
  notch.origin.y = grect_get_max_y(&line) - REL_BAR_LINE_WIDTH;
  notch.size.w = REL_BAR_LINE_CHECK_LENGTH + REL_BAR_LINE_WIDTH;
  notch.size.h = REL_BAR_LINE_WIDTH;
  graphics_fill_rect(ctx, &notch);

  const int dot_origin_y_max =
      grect_get_max_y(&line) + dot_line_length + curr_offset - prev_offset;
  // Dots in between two bars
  GRect dot = GRect(line.origin.x, grect_get_max_y(&line) + REL_BAR_LINE_WIDTH + dot_padding,
                    REL_BAR_DOT_SIZE, REL_BAR_DOT_SIZE);
  const int dot_advance = 2 * REL_BAR_DOT_SIZE;
  for (; dot.origin.y + dot_advance <= dot_origin_y_max; dot.origin.y += dot_advance) {
    graphics_fill_rect(ctx, &dot);
  }

  // Bar 2
  // Filled rect used to draw line of REL_BAR_LINE_WIDTH stroke width
  layer_bounds = timeline_layer->layer.bounds;
  line.origin.x = prv_sidebar_line_x(&layer_bounds);
  line.origin.y = dot.origin.y + dot_padding;
  line.size.w = REL_BAR_LINE_WIDTH;
  line.size.h = solid_line_length - curr_offset;
  graphics_fill_rect(ctx, &line);

  // Top Notch for Bar 2
  // Filled rect used to draw line of REL_BAR_LINE_WIDTH stroke width
  notch.origin.x = line.origin.x - REL_BAR_LINE_NOTCH_HORIZ_OFFSET + 1;
  notch.origin.y = line.origin.y;
  notch.size.w = REL_BAR_LINE_CHECK_LENGTH + REL_BAR_LINE_WIDTH;
  notch.size.h = REL_BAR_LINE_WIDTH;
  graphics_fill_rect(ctx, &notch);
}

#define REL_BAR_OVERLAP_STROKE_WIDTH 2
#define REL_BAR_OVERLAP_SIDE_MARGIN 2
#define REL_BAR_OVERLAP_NUDGE_X 1
#define REL_BAR_OVERLAP_LINE2_HORIZ_OFFSET ((2 * REL_BAR_OVERLAP_SIDE_MARGIN) + 1)
static void prv_draw_rel_bar_overlap(TimelineLayer *timeline_layer, GContext* ctx,
                                     bool current, int16_t rel_bar_value) {
  GRect first_icon_frame;
  GRect second_icon_frame;
  const int full_line_length =
      prv_get_overlap_line_length(timeline_layer, &first_icon_frame, &second_icon_frame);

  int16_t line_length = rel_bar_value;
  int16_t y_offset = 0;
  if (!current) {
    line_length = full_line_length;
    y_offset = (((full_line_length - rel_bar_value) * REL_BAR_PREV_ANIM_OFFSET) /
               full_line_length) * timeline_layer->move_delta;
  }

  if (((line_length <= 0) && current) ||
      ((!current) && ((y_offset >= REL_BAR_PREV_ANIM_OFFSET) ||
                      (y_offset <= -REL_BAR_PREV_ANIM_OFFSET)))) {
    return;
  }

  graphics_context_set_fill_color(ctx, GColorWhite);
  graphics_context_set_antialiased(ctx, false);
  GRect layer_bounds = timeline_layer->layer.bounds;
  GPoint line1_start;
  GPoint line2_start;

  // Draw down
  line1_start.x = prv_overlap_line_x(&layer_bounds) - REL_BAR_OVERLAP_NUDGE_X -
                  REL_BAR_OVERLAP_SIDE_MARGIN;
  line1_start.y = grect_get_max_y(&first_icon_frame) + REL_BAR_VERT_MARGIN -
                  y_offset + REL_BAR_LINE_WIDTH;

  graphics_fill_rect(ctx, &GRect(line1_start.x, line1_start.y,
                                 REL_BAR_OVERLAP_STROKE_WIDTH, line_length));
  GRect notch = GRectZero;
  notch.origin.x = line1_start.x - REL_BAR_LINE_NOTCH_HORIZ_OFFSET + 1;
  notch.origin.y = line1_start.y - REL_BAR_LINE_WIDTH;
  notch.size.w = REL_BAR_LINE_CHECK_LENGTH + REL_BAR_LINE_WIDTH;
  notch.size.h = REL_BAR_LINE_WIDTH;
  graphics_fill_rect(ctx, &notch);

  // Draw up
  line2_start.x = line1_start.x + (timeline_layer_sidebar_is_on_right()
                                       ? REL_BAR_OVERLAP_LINE2_HORIZ_OFFSET
                                       : -REL_BAR_OVERLAP_LINE2_HORIZ_OFFSET);
  line2_start.y = second_icon_frame.origin.y - REL_BAR_VERT_MARGIN - y_offset -
                  REL_BAR_LINE_WIDTH;
  graphics_fill_rect(ctx, &GRect(line2_start.x, line2_start.y,
                                 REL_BAR_OVERLAP_STROKE_WIDTH, -line_length));
  notch.origin.x = line2_start.x - REL_BAR_LINE_NOTCH_HORIZ_OFFSET + 1;
  notch.origin.y = line2_start.y;
  notch.size.w = REL_BAR_LINE_CHECK_LENGTH + REL_BAR_LINE_WIDTH;
  notch.size.h = REL_BAR_LINE_WIDTH;
  graphics_fill_rect(ctx, &notch);
}

#define NUM_REL_BAR_ANIMATIONS 2
#define CURRENT_REL_BAR_PIN_INDEX 1
static void prv_update_proc(Layer *layer, GContext *ctx) {
  RelationshipBarLayer *relbar_layer = (RelationshipBarLayer *)layer;
  TimelineLayer *timeline_layer = (TimelineLayer *)relbar_layer->timeline_layer;
  // Don't draw the relationship bars if they are meant to be hidden
  // Currently drawing relationship bar for future only - no plan for past
  if (layer_get_hidden(&relbar_layer->layer) ||
      (timeline_layer->scroll_direction == TimelineScrollDirectionUp)) {
    return;
  }

  for (int index = 0; index < NUM_REL_BAR_ANIMATIONS; index++) {
    bool current = (index == CURRENT_REL_BAR_PIN_INDEX);
    RelationshipBarType rel_bar = current ?
                                  timeline_layer->relbar_layer.curr_rel_bar.rel_bar_type :
                                  timeline_layer->relbar_layer.prev_rel_bar.rel_bar_type;
    int16_t rel_bar_value = current ? timeline_layer->relbar_layer.curr_rel_bar.anim_offset :
                                      timeline_layer->relbar_layer.prev_rel_bar.anim_offset;
    switch (rel_bar) {
      case RelationshipBarTypeFreeTime:
        // Draw dotted line
        prv_draw_rel_bar_dotted(timeline_layer, ctx, current, rel_bar_value);
        break;
      case RelationshipBarTypeOverlap:
        // Draw overlapping lines
        prv_draw_rel_bar_overlap(timeline_layer, ctx, current, rel_bar_value);
        break;
      case RelationshipBarTypeBackToBack:
        prv_draw_rel_bar_line(timeline_layer, ctx, current, rel_bar_value);
        break;
      case RelationshipBarTypeNone:
        // Draw nothing
        break;
    }
  }
}

// Timer callback that displays the current relationship bar if user has stopped fast clicking
void prv_rel_bar_show(void *context) {
  TimelineLayer *layer = (TimelineLayer *)context;
  if (timeline_layer_should_animate_day_separator(layer)) {
    prv_rel_bar_reset_offsets(&layer->relbar_layer, RelationshipBarOffsetTypeCurr);
    layer_set_hidden(&layer->relbar_layer.layer, true);
    return;
  }

  layer_set_hidden(&layer->relbar_layer.layer, false);
  if (layer->relbar_layer.curr_rel_bar.rel_bar_type == RelationshipBarTypeOverlap) {
    layer->relbar_layer.curr_rel_bar.anim_offset = prv_get_overlap_line_length(layer, NULL, NULL);
  } else if ((layer->relbar_layer.curr_rel_bar.rel_bar_type == RelationshipBarTypeBackToBack) ||
             (layer->relbar_layer.curr_rel_bar.rel_bar_type == RelationshipBarTypeFreeTime)) {
    layer->relbar_layer.curr_rel_bar.anim_offset = REL_BAR_BACK_TO_BACK_OFFSET;
  }

  layer_mark_dirty((Layer*) layer);
}


#define TIMELINE_NUM_REL_BARS (TIMELINE_NUM_VISIBLE_ITEMS + 1)
static void prv_update_rel_bars(TimelineLayer *layer) {
  // Store prev, current, and next item relationship bar types
  RelationshipBarType rel_bar_types[TIMELINE_NUM_REL_BARS];

  layer->relbar_layer.prev_rel_bar.rel_bar_type = layer->relbar_layer.curr_rel_bar.rel_bar_type;

  for (int index = 0; index < TIMELINE_NUM_REL_BARS; index++) {
    if (layer->layouts_info[index]) {
      rel_bar_types[index] = prv_get_pin_relationship(layer->layouts_info[index],
                                                      layer->layouts_info[index + 1]);
    }
  }
  layer->relbar_layer.curr_rel_bar.rel_bar_type = rel_bar_types[1];
  if (layer->layouts_info[1]) {
    PBL_LOG_DBG("Current rel bar %d, duration %"PRIu32, rel_bar_types[1],
            layer->layouts_info[1]->duration_s);
  }
}

/////////////////////////////////////////
// Public functions
/////////////////////////////////////////
#define REL_BAR_TIMER_DELAY(delay) (delay / 3) // delay of timer is based on the duration
Animation *timeline_relbar_layer_create_animation(TimelineLayer *layer, uint32_t duration,
                                                  InterpolateInt64Function interpolate) {
  bool force_rel_bar_anim = false;
  // Cancel previous fast scroll timer since user has clicked again
  evented_timer_cancel(layer->relbar_layer.rel_bar_timer);
  if (timeline_animation_interpolate_moook_second_half == interpolate) {
    force_rel_bar_anim = true;
    if (duration == TIMELINE_UP_DOWN_ANIMATION_DURATION_MS) {
      // Update current state of relationship bars
      prv_update_rel_bars(layer);

      // Hide bars
      layer_set_hidden(&layer->relbar_layer.layer, true);

      // Setup a timer to display bars after a fraction of the input duration (i.e. if the user
      // has stopped scrolling fast)
      layer->relbar_layer.rel_bar_timer =
        evented_timer_register(REL_BAR_TIMER_DELAY(duration), false, prv_rel_bar_show, layer);

      // Don't schedule animation
      return NULL;
    }
  }

  prv_update_rel_bars(layer);

  bool curr_anim_needed = true;
  bool prev_anim_needed = true;
  if (timeline_layer_should_animate_day_separator(layer)) {
    curr_anim_needed = false;
    prv_rel_bar_reset_offsets(&layer->relbar_layer, RelationshipBarOffsetTypeCurr);
    layer_set_hidden(&layer->relbar_layer.layer, true);
  } else {
    layer_set_hidden(&layer->relbar_layer.layer, false);
  }

  // This check is to ensure rel bar does not show up while day separator is on screen.
  // The force animation is used to ensure the rel bar shows up after the day separator is hidden -
  // timeline_create_up_down_animation is called both when sliding between timeline items as well as
  // during the day separator animation.
  if (force_rel_bar_anim) {
    prev_anim_needed = false;
    prv_rel_bar_reset_offsets(&layer->relbar_layer, RelationshipBarOffsetTypePrev);
    curr_anim_needed = true;
  }

  // Create previous rel bar animation
  Animation *prev_rel_bar_anim = NULL;

  if (prev_anim_needed) {
    prev_rel_bar_anim = prv_create_prev_rel_bar_animation(layer, duration, interpolate);
  }

  Animation *curr_rel_bar_anim = NULL;

  if (curr_anim_needed) {
    curr_rel_bar_anim = prv_create_curr_rel_bar_animation(layer, duration, interpolate);
  }

  Animation *rel_bar_anim = NULL;
  if (prev_rel_bar_anim && curr_rel_bar_anim) {
    rel_bar_anim = animation_spawn_create(prev_rel_bar_anim, curr_rel_bar_anim, NULL);
  } else if (prev_rel_bar_anim) {
    rel_bar_anim = prev_rel_bar_anim;
  } else {
    rel_bar_anim = curr_rel_bar_anim;
  }

  if (rel_bar_anim) {
    animation_set_handlers(rel_bar_anim, (AnimationHandlers) {
      .stopped = prv_rel_bar_stopped,
    }, layer);
  }

  return rel_bar_anim;
}

void timeline_relbar_layer_reset(TimelineLayer *layer) {
  prv_update_rel_bars(layer);
  if (layer->relbar_layer.curr_rel_bar.rel_bar_type == RelationshipBarTypeOverlap) {
    layer->relbar_layer.curr_rel_bar.anim_offset = prv_get_overlap_line_length(layer, NULL, NULL);
  } else if ((layer->relbar_layer.curr_rel_bar.rel_bar_type == RelationshipBarTypeBackToBack) ||
             (layer->relbar_layer.curr_rel_bar.rel_bar_type == RelationshipBarTypeFreeTime)) {
    layer->relbar_layer.curr_rel_bar.anim_offset = REL_BAR_BACK_TO_BACK_OFFSET;
  }
}

void timeline_relbar_layer_init(TimelineLayer *timeline_layer) {
  RelationshipBarLayer *relbar_layer = &timeline_layer->relbar_layer;
  *relbar_layer = (RelationshipBarLayer){};
  // init layer
  layer_init(&relbar_layer->layer, &timeline_layer->layer.frame);
  layer_set_update_proc(&relbar_layer->layer, prv_update_proc);
  layer_add_child((Layer*)timeline_layer, (Layer *)&timeline_layer->relbar_layer);
  relbar_layer->timeline_layer = timeline_layer;
}

//! Deinitialize the timeline relationship bar layer within the \ref TimelineLayer
void timeline_relbar_layer_deinit(TimelineLayer *timeline_layer) {
  layer_deinit(&timeline_layer->relbar_layer.layer);
}
