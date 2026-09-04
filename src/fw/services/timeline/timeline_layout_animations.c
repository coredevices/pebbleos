/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "pbl/services/timeline/timeline_layout_animations.h"

#include "applib/ui/animation_interpolate.h"
#include "applib/ui/kino/kino_reel/scale_segmented.h"
#include "applib/ui/kino/kino_reel/transform.h"
#include "applib/ui/ui.h"
#include "apps/system/timeline/animations.h"
#include "process_state/app_state/app_state.h"

#define CARD_TRANSITION_ICON_EXPAND 5

static void prv_change_reel(TimelineLayout *layout, KinoReel *reel) {
  // we most likely don't want that callback any more
  kino_layer_set_callbacks(&layout->icon_layer, (KinoLayerCallbacks) {}, NULL);
  kino_layer_set_reel(&layout->icon_layer, reel, true);
}

static void prv_continue_reel(TimelineLayout *layout, KinoReel *new_reel, uint32_t start) {
  const uint32_t duration = kino_reel_get_duration(new_reel);
  if (duration > start) {
    kino_layer_play_section(&layout->icon_layer, start, duration);
  }
}

static void prv_pin_to_card_second_half_stopped(KinoLayer *layer, bool finished, void *context) {
  TimelineLayout *layout = context;
  KinoReel *new_reel = kino_reel_create_with_resource_system(layout->icon_res_info.res_app_num,
                                                             layout->icon_res_info.res_id);
  prv_change_reel(layout, new_reel);
  if (!layout->is_being_destroyed) {
    // PBL-20112: Continue where we left off until this is changed to use the Window Transitioner
    prv_continue_reel(layout, new_reel, TIMELINE_CARD_TRANSITION_MS);
  }
  // Clip for the static icon
  layer_set_clips(&layout->icon_layer.layer, true);
}

static void prv_pin_to_card_second_half(TimelineLayout *pin_timeline_layout,
                                        TimelineLayout *card_timeline_layout) {
  KinoReel *from_reel = kino_reel_create_with_resource_system(
      card_timeline_layout->icon_res_info.res_app_num,
      card_timeline_layout->icon_res_info.res_id);

  if (!from_reel) {
    return;
  }

  const uint32_t duration = TIMELINE_CARD_TRANSITION_MS;
  GRect icon_from, icon_to;
  layer_get_global_frame(&pin_timeline_layout->icon_layer.layer, &icon_from);
  layer_get_global_frame(&card_timeline_layout->icon_layer.layer, &icon_to);
  // Unclip for the scaling animation
  layer_set_clips(&pin_timeline_layout->icon_layer.layer, false);
  layer_set_clips(&card_timeline_layout->icon_layer.layer, false);

  // TODO: PBL-28738 Refactor kino reel transform APIs or usage of APIs
  // There is a lot of shared code throughout, and the API is code space intensive
  const bool take_ownership = true;
  KinoReel *new_reel = kino_reel_scale_segmented_create(from_reel, take_ownership, icon_to);
  kino_reel_transform_set_from_frame(new_reel, icon_from);
  kino_reel_transform_set_transform_duration(new_reel, duration);
  kino_reel_scale_segmented_set_deflate_effect(new_reel, CARD_TRANSITION_ICON_EXPAND);
  kino_reel_scale_segmented_set_delay_by_distance(new_reel, GPoint(0, icon_to.size.h / 2));
  prv_change_reel(card_timeline_layout, new_reel);

  kino_layer_play_section(&card_timeline_layout->icon_layer, duration / 2, duration);

  kino_layer_set_callbacks(&card_timeline_layout->icon_layer, (KinoLayerCallbacks) {
    .did_stop = prv_pin_to_card_second_half_stopped,
  }, card_timeline_layout);
}

static void prv_pin_to_card_first_half_stopped(KinoLayer *layer, bool finished,
                                               void *context) {
  TimelineLayout *timeline_layout = context;

  // reset the first half of the animation
  kino_reel_set_elapsed(kino_layer_get_reel(&timeline_layout->icon_layer), 0);
  // Clip for the static icon
  layer_set_clips(&layer->layer, true);

  if (finished) {
    // begin the second half of the animation
    prv_pin_to_card_second_half(timeline_layout, timeline_layout->transition_layout);
  }

  timeline_layout->transition_layout = NULL;
}

void timeline_layout_transition_pin_to_card(TimelineLayout *pin_timeline_layout,
                                            TimelineLayout *card_timeline_layout) {
  KinoReel *from_reel = kino_reel_create_with_resource_system(
      pin_timeline_layout->icon_res_info.res_app_num,
      pin_timeline_layout->icon_res_info.res_id);

  if (!from_reel) {
    return;
  }

  const uint32_t duration = TIMELINE_CARD_TRANSITION_MS;
  GRect icon_from, icon_to;
  layer_get_global_frame(&pin_timeline_layout->icon_layer.layer, &icon_from);
  layer_get_global_frame(&card_timeline_layout->icon_layer.layer, &icon_to);
  // Unclip for the scaling animation
  layer_set_clips(&pin_timeline_layout->icon_layer.layer, false);
  layer_set_clips(&card_timeline_layout->icon_layer.layer, false);

  const bool take_ownership = true;
  KinoReel *new_reel = kino_reel_scale_segmented_create(from_reel, take_ownership, icon_from);
  kino_reel_transform_set_to_frame(new_reel, icon_to);
  kino_reel_transform_set_transform_duration(new_reel, duration);
  kino_reel_scale_segmented_set_deflate_effect(new_reel, CARD_TRANSITION_ICON_EXPAND);
  kino_reel_scale_segmented_set_delay_by_distance(new_reel, GPoint(0, icon_from.size.h / 2));
  prv_change_reel(pin_timeline_layout, new_reel);

  layer_set_clips((Layer *)pin_timeline_layout, false);
  layer_set_clips((Layer *)&pin_timeline_layout->icon_layer, false);

  kino_layer_set_callbacks(&pin_timeline_layout->icon_layer, (KinoLayerCallbacks) {
    .did_stop = prv_pin_to_card_first_half_stopped,
  }, pin_timeline_layout);
  kino_layer_play_section(&pin_timeline_layout->icon_layer, 0, duration / 2);

  pin_timeline_layout->transition_layout = card_timeline_layout;
}

static void prv_card_to_pin_stopped(KinoLayer *layer, bool finished, void *context) {
  TimelineLayout *layout = context;
  layer_set_hidden(&layout->icon_layer.layer, false);
  // Clip for the static icon
  layer_set_clips(&layout->icon_layer.layer, true);
}

void timeline_layout_transition_card_to_pin(TimelineLayout *card_timeline_layout,
                                            TimelineLayout *pin_timeline_layout) {
  KinoReel *from_reel = kino_reel_create_with_resource_system(
      card_timeline_layout->icon_res_info.res_app_num,
      card_timeline_layout->icon_res_info.res_id);

  if (!from_reel) {
    return;
  }

  const uint32_t duration = TIMELINE_CARD_TRANSITION_MS;
  GRect icon_from, icon_to;
  layer_get_global_frame((Layer *)&card_timeline_layout->icon_layer, &icon_from);
  layer_get_global_frame((Layer *)&pin_timeline_layout->icon_layer, &icon_to);
  // Unclip for the scaling animation
  layer_set_clips(&pin_timeline_layout->icon_layer.layer, false);
  layer_set_clips(&card_timeline_layout->icon_layer.layer, false);

  const bool take_ownership = true;
  KinoReel *new_reel = kino_reel_scale_segmented_create(from_reel, take_ownership, icon_from);
  kino_reel_transform_set_to_frame(new_reel, icon_to);
  kino_reel_transform_set_transform_duration(new_reel, duration / 2);
  kino_reel_scale_segmented_set_delay_by_distance(
      new_reel, GPoint(icon_from.size.w, icon_from.size.h / 2));
  prv_change_reel(card_timeline_layout, new_reel);

  kino_layer_set_callbacks(&card_timeline_layout->icon_layer, (KinoLayerCallbacks) {
    .did_stop = prv_card_to_pin_stopped,
  }, pin_timeline_layout);
  kino_layer_play(&card_timeline_layout->icon_layer);

  // for now, use the card icon for the entire animation, so hide the tiny icon
  layer_set_hidden((Layer *)&pin_timeline_layout->icon_layer, true);
}

static void prv_up_down_stopped(Animation *animation, bool finished, void *context) {
  TimelineLayout *layout = context;
  timeline_animation_layer_stopped_cut_to_end(animation, finished, context);

  // move the icon layer to where it transformed to
  KinoReel *prev_reel = kino_layer_get_reel(&layout->icon_layer);
  if (prev_reel) {
    GRect frame = ((Layer *)&layout->icon_layer)->frame;
    GRect global_frame;
    layer_get_global_frame((Layer *)&layout->icon_layer, &global_frame);
    GRect icon_to = kino_reel_transform_get_to_frame(prev_reel);
    frame.origin = gpoint_add(frame.origin, gpoint_sub(icon_to.origin, global_frame.origin));
    layer_set_frame((Layer *)&layout->icon_layer, &frame);
    // Clip for the static icon
    layer_set_clips(&layout->icon_layer.layer, true);
  }

  KinoReel *new_reel = kino_reel_create_with_resource_system(layout->icon_res_info.res_app_num,
                                                             layout->icon_res_info.res_id);
  prv_change_reel(layout, new_reel);
  if (!layout->is_being_destroyed) {
    // PBL-20111: Continue where we left off until kino reel transform frames is refactored
    // This is because continuing to render with the transform will have resulted in a jump
    // since it has been animating in global coordinates, not accounting for the property layer
    // animation of the pin containing this icon has gone through.
    prv_continue_reel(layout, new_reel, TIMELINE_UP_DOWN_ANIMATION_DURATION_MS);
  }
}

Animation *timeline_layout_create_up_down_animation(
    TimelineLayout *layout, const GRect *from, const GRect *to, const GRect *icon_from,
    const GRect *icon_to, uint32_t duration, InterpolateInt64Function interpolate) {
  KinoReel *from_reel = kino_reel_create_with_resource_system(layout->icon_res_info.res_app_num,
                                                              layout->icon_res_info.res_id);
  if (from_reel) {
    GPoint target = (GPoint) {
      .x = icon_from->size.w / 2, // pull from the middle
      .y = (icon_to->origin.y > icon_from->origin.y) ? // if going up, pull from the top
            icon_from->size.h : 0, // else pull from the bottom
    };

    const bool take_ownership = true;
    KinoReel *new_reel = kino_reel_scale_segmented_create(from_reel, take_ownership, *icon_from);
    kino_reel_transform_set_to_frame(new_reel, *icon_to);
    kino_reel_transform_set_layer_frame(new_reel, layout->icon_layer.layer.frame);
    kino_reel_transform_set_transform_duration(new_reel, duration);
    kino_reel_scale_segmented_set_delay_by_distance(new_reel, target);

    const Fixed_S32_16 POINT_DURATION = Fixed_S32_16(5 * FIXED_S32_16_ONE.raw_value / 6);
    kino_reel_scale_segmented_set_point_duration(new_reel, POINT_DURATION);
    kino_reel_scale_segmented_set_interpolate(new_reel, interpolate);

    prv_change_reel(layout, new_reel);

    // Unclip for the scaling animation
    layer_set_clips(&layout->icon_layer.layer, false);
  }

  PropertyAnimation *property_animation =
      property_animation_create_layer_frame((Layer *)layout, (GRect *)from, (GRect *)to);
  Animation *animation = property_animation_get_animation(property_animation);
  animation_set_duration(animation, duration);
  animation_set_custom_interpolation(animation, interpolate);
  animation_set_handlers(animation, (AnimationHandlers) {
    .stopped = prv_up_down_stopped,
  }, layout);
  return animation;
}
