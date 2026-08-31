/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "spinner_ui_window.h"

#include "applib/graphics/graphics.h"
#include "applib/graphics/gtypes.h"
#include "pbl/util/trig.h"
#include "applib/ui/animation.h"
#include "applib/ui/layer.h"
#include "applib/ui/property_animation.h"
#include "kernel/pbl_malloc.h"
#include "system/passert.h"

////////////////////////////////////////////////////////////
// Data structures

typedef struct {
  Window window;
  Layer background_layer;
  Layer anim_layer;
  PropertyAnimation *spinner_animation;
  AnimationImplementation spinner_anim_impl;
  GColor spinner_color;
  AnimationProgress cur_distance_normalized;
  bool should_cancel_animation;
} SpinnerUIData;


////////////////////////////////////////////////////////////
// Animation Logic

// There is a slight delay (lag) between the animation stopping and starting it again. To minimize
// this, make the animation contain multiple loops (360 degree rotations) instead of 1.
// This means that the the lag occurs once less frequently and is less noticable
#define LOOPS_PER_ANIMATION 10
#define LOOP_DURATION_MS 1500
#define SPINNER_CIRCLE_RADIUS 9

static int prv_get_background_circle_radius(const GRect *bounds) {
  const int min_dimension = MIN(bounds->size.w, bounds->size.h);
  return (min_dimension * 3) / (4 * 2); // 3/4 of min dimension is diameter
}

static void prv_draw_background_circle(Layer *layer, GContext* ctx) {
  graphics_context_set_antialiased(ctx, false);
  const GPoint center = grect_center_point(&layer->bounds);
  const int radius = prv_get_background_circle_radius(&layer->bounds);

  graphics_context_set_fill_color(ctx, GColorWhite);
  graphics_fill_circle(ctx, center, radius);
  graphics_context_set_stroke_color(ctx, GColorBlack);
  graphics_draw_circle(ctx, center, radius);
}

static void prv_draw_spinner_circles(Layer *layer, GContext* ctx) {
  // Drawing the circles with aa is just too slow and we end up backing up the rest of the system.
  // See PBL-16184
  graphics_context_set_antialiased(ctx, false);

  SpinnerUIData *data = window_get_user_data(layer_get_window(layer));

  const int bg_radius = prv_get_background_circle_radius(&layer->bounds);
  const unsigned int radius_of_path = bg_radius - SPINNER_CIRCLE_RADIUS;
  const unsigned int radius_of_spinner_circles = SPINNER_CIRCLE_RADIUS;
  const GPoint circle_center_point = grect_center_point(&layer->bounds);
  const unsigned int angle = (TRIG_MAX_ANGLE * data->cur_distance_normalized *
                              LOOPS_PER_ANIMATION) / ANIMATION_NORMALIZED_MAX;

  const GPoint circle1_location = {
    .x = (sin_lookup(angle) * radius_of_path / TRIG_MAX_RATIO) + circle_center_point.x,
    .y = (-cos_lookup(angle) * radius_of_path / TRIG_MAX_RATIO) + circle_center_point.y,
  };
  const GPoint circle2_location = {
    .x = (-sin_lookup(angle) * (-radius_of_path) / TRIG_MAX_RATIO) + circle_center_point.x,
    .y = (-cos_lookup(angle) * (-radius_of_path) / TRIG_MAX_RATIO) + circle_center_point.y,
  };

  graphics_context_set_fill_color(ctx, data->spinner_color);
  graphics_context_set_stroke_color(ctx, GColorBlack);
  graphics_fill_circle(ctx, circle1_location, radius_of_spinner_circles);
  graphics_draw_circle(ctx, circle1_location, radius_of_spinner_circles);
  graphics_fill_circle(ctx, circle2_location, radius_of_spinner_circles);
  graphics_draw_circle(ctx, circle2_location, radius_of_spinner_circles);
}

static void prv_anim_impl(struct Animation *animation,
                          const AnimationProgress distance_normalized) {
  SpinnerUIData *data = (SpinnerUIData*) animation_get_context(animation);

  // We need to artificially limit how frequent we attempt to update the screen. If we update
  // it too fast the thing we wanted to do in the background never gets done. This isn't quite
  // ideal, as around 60 steps is when things are actually smooth, but 60 is too fast and does
  // restrict the speed of our core dump. See PBL-16184
  const uint32_t steps_per_loop = 25;
  const int32_t min_delta = (ANIMATION_NORMALIZED_MAX/LOOPS_PER_ANIMATION) / steps_per_loop;
  if (data->cur_distance_normalized + min_delta < distance_normalized) {
    data->cur_distance_normalized = distance_normalized;
    layer_mark_dirty(&data->anim_layer);
  }
}

static void prv_anim_stopped(Animation *animation, bool finished, void *context) {
  SpinnerUIData *data = (SpinnerUIData*) animation_get_context(animation);
  if (!data->should_cancel_animation) {
    data->cur_distance_normalized = 0;
    animation_schedule(property_animation_get_animation(data->spinner_animation));
  }
}

////////////////////////////////////////////////////////////
// Window loading, unloading, initializing

static void prv_window_unload_handler(Window* window) {
  SpinnerUIData *data = window_get_user_data(window);
  if (data) {
    data->should_cancel_animation = true;
    property_animation_destroy(data->spinner_animation);
    kernel_free(data);
  }
}

static void prv_window_load_handler(Window* window) {
  SpinnerUIData *data = window_get_user_data(window);

  window_set_background_color(window, PBL_IF_COLOR_ELSE(GColorLightGray, GColorWhite));

  // Background circle layer
  Layer *background_layer = &data->background_layer;
  layer_set_bounds(background_layer, &window->layer.bounds);
  layer_set_update_proc(background_layer, prv_draw_background_circle);
  layer_add_child(&window->layer, background_layer);

  // Animation layer
  Layer *anim_layer = &data->anim_layer;
  layer_set_bounds(anim_layer, &window->layer.bounds);
  layer_set_update_proc(anim_layer, prv_draw_spinner_circles);
  layer_add_child(&window->layer, anim_layer);

  // See comment about loops above (animation section)
  const int animation_duration_ms = LOOP_DURATION_MS * LOOPS_PER_ANIMATION;
  const int animation_delay_ms = 0;
  data->spinner_animation = property_animation_create_layer_frame(&data->anim_layer, NULL, NULL);
  if (data->spinner_animation) {
    Animation *animation = property_animation_get_animation(data->spinner_animation);
    animation_set_duration(animation, animation_duration_ms);
    animation_set_delay(animation, animation_delay_ms);
    animation_set_curve(animation, AnimationCurveLinear);
    animation_set_auto_destroy(animation, false);

    AnimationHandlers anim_handler = {
      .stopped = prv_anim_stopped,
    };
    animation_set_handlers(animation, anim_handler, data);

    data->spinner_anim_impl = (AnimationImplementation) {
      .update = prv_anim_impl,
    };
    animation_set_implementation(animation, &data->spinner_anim_impl);

    animation_schedule(animation);
  }
}

Window* spinner_ui_window_get(GColor spinner_color) {
  SpinnerUIData *data = kernel_malloc_check(sizeof(SpinnerUIData));
  *data = (SpinnerUIData){};

  data->spinner_color = spinner_color;
  data->should_cancel_animation = false;

  Window* window = &data->window;
  window_init(window, WINDOW_NAME("Spinner UI Window"));
  window_set_user_data(window, data);
  window_set_overrides_back_button(window, true);
  window_set_window_handlers(window, &(WindowHandlers){
    .load = prv_window_load_handler,
    .unload = prv_window_unload_handler,
  });

  return window;
}
