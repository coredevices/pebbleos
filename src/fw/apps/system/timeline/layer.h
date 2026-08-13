/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "common.h"
#include "peek_layer.h"

#include "applib/graphics/text.h"
#include "applib/ui/animation.h"
#include "applib/ui/layer.h"
#include "pbl/services/evented_timer.h"
#include "pbl/services/blob_db/pin_db.h"
#include "pbl/services/timeline/item.h"
#include "pbl/services/timeline/timeline_layout.h"
#include "pbl/services/timeline/timeline_layout_animations.h"

#include <stdbool.h>

#define TIMELINE_NUM_ITEMS_IN_TIMELINE_LAYER (TIMELINE_NUM_VISIBLE_ITEMS + 2)

#define TIMELINE_LAYER_FIRST_VISIBLE_LAYOUT 1

#define TIMELINE_LAYER_TEXT_ALIGNMENT \
    PBL_IF_RECT_ELSE(GTextAlignmentLeft, GTextAlignmentRight)
#define TIMELINE_LAYER_TEXT_VERTICAL_ALIGNMENT GVerticalAlignmentTop

#define TIMELINE_LAYER_SLIDE_MS (150)

//! Relationship bars describe the relationship between two Timeline events as a visual.
typedef enum {
  //! The two timeline events are not in the same day.
  RelationshipBarTypeNone = 0,
  //! There is time between the end of the first event and the start of the second event.
  RelationshipBarTypeFreeTime,
  //! As soon as the first event starts, the second event ends.
  RelationshipBarTypeBackToBack,
  //! The first event is still in progress when the second event starts.
  RelationshipBarTypeOverlap,
} RelationshipBarType;

typedef struct {
  RelationshipBarType rel_bar_type;
  int16_t anim_offset;
} RelationshipBar;

typedef struct {
  Layer layer;
  RelationshipBar prev_rel_bar; // Used for previous relationship bar animation exit
  RelationshipBar curr_rel_bar; // Used for current on-screen relationship bar animation
  EventedTimerID rel_bar_timer; // Used to show bars after user stops fast scrolling
  void *timeline_layer;         // Necessary for the layer update proc to access the TimelineLayer
} RelationshipBarLayer;

// The timeline layer is the view(controller, sort of) for the timeline -- it uses
// TIMELINE_NUM_ITEMS_IN_TIMELINE_LAYER layout layers, timeline layouts and view slots
// layouts[1] is the first item shown, layouts[0] and layouts[TIMELINE_NUM_VISIBLE_ITEMS + 1]
// should be NULL most of the time and are used to animate out layers

typedef struct {
  Layer layer;
  Layer layouts_layer;
  TimelineLayout *layouts[TIMELINE_NUM_ITEMS_IN_TIMELINE_LAYER];
  TimelineLayoutInfo *layouts_info[TIMELINE_NUM_ITEMS_IN_TIMELINE_LAYER];
  TimelineScrollDirection scroll_direction;
  int16_t sidebar_width;
  GColor sidebar_color;
  KinoLayer end_of_timeline;
  PeekLayer day_separator;
  time_t current_day;
  // TODO: PBL-22076 Remove Timeline Layer move_delta
  // It is not good to keep too much questionably long lived state in views
  int move_delta;
  Animation *animation;
  RelationshipBarLayer relbar_layer;
  bool animating_intro_or_exit;
} TimelineLayer;

void timeline_layer_reset(TimelineLayer *layer);

void timeline_layer_set_next_item(TimelineLayer *layer, int index);

void timeline_layer_set_prev_item(TimelineLayer *layer, int index);

void timeline_layer_move_data(TimelineLayer *layer, int delta);

uint16_t timeline_layer_get_fat_pin_height(void);

uint16_t timeline_layer_get_ideal_sidebar_width(void);

//! False in Left-Handed Mode so the pin strip sits on the button side.
bool timeline_layer_sidebar_is_on_right(void);

//! Outer inset used to place pin icons inside the sidebar (mirrors the right-hand strip).
int16_t timeline_layer_get_icon_outer_inset(void);

//! Extra x origin for pin text when the sidebar is on the left, so the gap matches right_margin.
int16_t timeline_layer_get_pin_text_origin_x(void);

void timeline_layer_get_layout_frame(TimelineLayer *layer, int index, GRect *frame_out);

void timeline_layer_get_icon_frame(TimelineLayer *layer, int index, GRect *frame_out);

Animation *timeline_layer_create_up_down_animation(TimelineLayer *layer, uint32_t duration,
                                                   InterpolateInt64Function interpolate);

// TODO: move animation logic to timeline_animations.c
// Returns whether the move animation should animate the day separator
bool timeline_layer_should_animate_day_separator(TimelineLayer *layer);

void timeline_layer_set_day_sep_frame(TimelineLayer *timeline_layer, const GRect *frame);

void timeline_layer_unfold_day_sep(TimelineLayer *timeline_layer);

void timeline_layer_slide_day_sep(TimelineLayer *timeline_layer);

Animation *timeline_layer_create_day_sep_show(TimelineLayer *timeline_layer);

Animation *timeline_layer_create_day_sep_hide(TimelineLayer *timeline_layer);

//! Initialize a timeline layer
//! @param layer a pointer to the TimelineLayer to initialize
//! @param frame the frame with which to initialize the layer
//! @param scroll_direction the direction to scroll for the next item
void timeline_layer_init(TimelineLayer *layer, const GRect *frame,
                         TimelineScrollDirection scroll_direction);

//! Sets the sidebar color
void timeline_layer_set_sidebar_color(TimelineLayer *timeline_layer, GColor color);

//! Sets the sidebar width
//! @param timeline_layer Pointer to the TimelineLayer.
//! @param width Width to set the TimelineLayer sidebar to.
void timeline_layer_set_sidebar_width(TimelineLayer *timeline_layer, int16_t width);

//! Create a sidebar animation that changes the width
//! @param timeline_layer Pointer to the TimelineLayer.
//! @param to_sidebar_width Width to animate the TimelineLayer sidebar to.
Animation *timeline_layer_create_sidebar_animation(TimelineLayer *timeline_layer,
                                                   int16_t to_sidebar_width);

//! Creates a speed lines animation that simulates scrolling through Timeline
//! @param timeline_layer Pointer to the TimelineLayer.
Animation *timeline_layer_create_speed_lines_animation(TimelineLayer *timeline_layer);

//! Create a bounce back animation for all layouts
//! @param layer Pointer to the TimelineLayer.
//! @param direction The direction of the bounce back as a unit vector.
Animation *timeline_layer_create_bounce_back_animation(TimelineLayer *layer, GPoint direction);

//! Sets whether the layout layers are hidden or not
void timeline_layer_set_layouts_hidden(TimelineLayer *layer, bool hidden);

//! Get the current timeline layout
TimelineLayout *timeline_layer_get_current_layout(TimelineLayer *timeline_layer);

//! Deinitialize a timeline item layer.
void timeline_layer_deinit(TimelineLayer *layer);
