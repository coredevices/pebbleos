/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "applib/graphics/text.h"
#include "applib/ui/layer.h"
#include "applib/ui/property_animation_private.h"
#include "kernel/events.h"
#include "pbl/services/timeline/item.h"
#include "pbl/services/timeline/timeline_layout.h"

//! The timeline item layer is a mock UI used to display timeline items
//! until actual layouts are implemented. It is somewhat related to the
//! notification layer, although it does not swap between items.

typedef struct {
  Layer layer;
  //!< The line that's currently at the top of the frame.
  int16_t scroll_offset_pixels;
  PropertyAnimation *animation;
  TimelineItem *item;
  TimelineLayout *timeline_layout;
} TimelineItemLayer;

//! The layer update proc for the TimelineItemLayer
void timeline_item_layer_update_proc(Layer* layer, GContext* ctx);

//! Initialize a timeline item layer
//! @param item_layer a pointer to the TimelineItemLayer to initialize
//! @param frame the frame with which to initialize the layer
void timeline_item_layer_init(TimelineItemLayer *item_layer, const GRect *frame);

//! Deinitialize a timeline item layer. Currently a no-op
void timeline_item_layer_deinit(TimelineItemLayer *item_layer);

//! Set the timeline item displayed by the TimelineItemLayer
//! @param item_layer a pointer to the TimelineItemLayer
//! @param item a pointer to the item to use
//! @param info a pointer to the TimelineLayoutInfo to use for the item
void timeline_item_layer_set_item(TimelineItemLayer *item_layer, TimelineItem *item,
    TimelineLayoutInfo *info);

//! Down click handler for the TimelineItemLayer
void timeline_item_layer_down_click_handler(ClickRecognizerRef recognizer, void *context);

//! Up click handler for the TimelineItemLayer
void timeline_item_layer_up_click_handler(ClickRecognizerRef recognizer, void *context);

//! Convenience function to set the \ref ClickConfigProvider callback on the
//! given window to menu layer's internal click config provider. This internal
//! click configuration provider, will set up the default UP & DOWN handlers
void timeline_item_layer_set_click_config_onto_window(TimelineItemLayer *item_layer,
    struct Window *window);
