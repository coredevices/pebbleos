/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "progress_window.h"

#include "applib/applib_malloc.auto.h"
#include "applib/fonts/fonts.h"
#include "applib/ui/app_window_stack.h"
#include "applib/ui/window_private.h"
#include "applib/ui/window_stack.h"
#include "pbl/services/compositor/compositor_transitions.h"
#include "pbl/services/timeline/timeline_resources.h"
#include "system/passert.h"
#include "pbl/util/math.h"

#define SCROLL_OUT_MS 250
#define BAR_HEIGHT PROGRESS_SUGGESTED_HEIGHT
#define BAR_WIDTH 80
#define BAR_TO_TRANS_MS 160
#define TRANS_TO_DOT_MS 90
#define DOT_TRANSITION_RADIUS 13
#define DOT_COMPOSITOR_RADIUS 7
#define DOT_OFFSET 25

#define FAKE_PROGRESS_UPDATE_INTERVAL 200
#define FAKE_PROGRESS_UPDATE_AMOUNT 2

#define INITIAL_PERCENT 0

static void prv_finished(ProgressWindow *data, bool success) {
  if (data->callbacks.finished) {
    data->callbacks.finished(data, success, data->context);
  }
}

///////////////////////////////
// Animation Related Functions
///////////////////////////////

static void prv_animation_stopped_success(Animation *animation, bool finished, void *context) {
  ProgressWindow *data = context;

  const bool success = true;
  prv_finished(data, success);
}

static void prv_finished_failure_callback(void *data) {
  const bool success = false;
  prv_finished(data, success);
}

static void prv_show_peek_layer(ProgressWindow *data) {
  if (data->is_peek_layer_used) {
    Layer *root_layer = window_get_root_layer(&data->window);
    PeekLayer *peek_layer = &data->peek_layer;
    peek_layer_play(peek_layer);
    layer_add_child(root_layer, (Layer *)peek_layer);

    const int standing_ms = 1 * MS_PER_SECOND;
    data->peek_layer_timer = evented_timer_register(PEEK_LAYER_UNFOLD_DURATION + standing_ms,
                                                    false, prv_finished_failure_callback, data);
  } else {
    prv_finished_failure_callback(data);
  }
}

static void prv_animation_stopped_failure(Animation *animation, bool finished, void *context) {
  ProgressWindow *data = context;
  prv_show_peek_layer(data);
}

static void prv_schedule_progress_success_animation(ProgressWindow *data) {
  GRect beg = data->progress_layer.layer.bounds;
  GRect mid = beg;
  GRect end = beg;

  // Morph from progress_layer to a large transition dot to the compositor dot
  // by changing the bounds of the progress_layer using 2 animations
  layer_set_clips((Layer *)&data->progress_layer, false); // Extending bounds to grow the dot
  progress_layer_set_corner_radius(&data->progress_layer, DOT_TRANSITION_RADIUS);

  mid.size.w = DOT_TRANSITION_RADIUS * 2;
  mid.size.h = DOT_TRANSITION_RADIUS * 2;
  mid.origin.x = DOT_OFFSET - DOT_TRANSITION_RADIUS + 2;
  mid.origin.y = BAR_HEIGHT - DOT_TRANSITION_RADIUS + 1; // shift to accommodate growing radius

  end.size.w = DOT_COMPOSITOR_RADIUS * 2;
  end.size.h = DOT_COMPOSITOR_RADIUS * 2;
  end.origin.x = DOT_OFFSET - DOT_COMPOSITOR_RADIUS - 1;
  end.origin.y = BAR_HEIGHT - DOT_COMPOSITOR_RADIUS - 2; // shift to accommodate growing radius

  PropertyAnimation *prop_anim =
      property_animation_create_layer_bounds((Layer *)&data->progress_layer, &beg, &mid);

  Animation *animation1 = property_animation_get_animation(prop_anim);
  animation_set_duration(animation1, BAR_TO_TRANS_MS);
  animation_set_curve(animation1, AnimationCurveEaseIn);

  prop_anim = property_animation_create_layer_bounds((Layer *)&data->progress_layer, &mid, &end);

  Animation *animation2 = property_animation_get_animation(prop_anim);
  animation_set_duration(animation2, TRANS_TO_DOT_MS);
  animation_set_curve(animation2, AnimationCurveLinear);
  animation_set_handlers(animation2, (AnimationHandlers) {
    .stopped = prv_animation_stopped_success,
  }, data);

  Animation *animation = animation_sequence_create(animation1, animation2, NULL);

  data->result_animation = animation;
  animation_schedule(animation);
}

static void prv_schedule_progress_failure_animation(ProgressWindow *data, uint32_t timeline_res_id,
                                                    const char *message, uint32_t delay) {
  // Initialize the peek layer
  if (timeline_res_id || message) {
    Layer *root_layer = window_get_root_layer(&data->window);
    PeekLayer *peek_layer = &data->peek_layer;
    peek_layer_init(peek_layer, &root_layer->frame);
    peek_layer_set_title_font(peek_layer, fonts_get_system_font(FONT_KEY_GOTHIC_24_BOLD));

    TimelineResourceInfo timeline_res = {
      .res_id = timeline_res_id,
    };
    peek_layer_set_icon(peek_layer, &timeline_res);
    peek_layer_set_title(peek_layer, message);
    peek_layer_set_background_color(peek_layer, PBL_IF_COLOR_ELSE(GColorLightGray, GColorWhite));
    data->is_peek_layer_used = true;
  }

  // Animate the progress bar out, by shrinking it's width from it's current size down to 0.
  // When this completes, prv_animation_stopped_failure will show the peek layer.
  GRect *start = &data->progress_layer.layer.frame;
  GRect stop = *start;
  stop.size.w = 0;

  PropertyAnimation *prop_anim =
      property_animation_create_layer_frame((Layer *)&data->progress_layer, start, &stop);

  Animation *animation = property_animation_get_animation(prop_anim);
  // If we failed, pause on the screen for a little.
  animation_set_delay(animation, delay);
  animation_set_duration(animation, SCROLL_OUT_MS);
  animation_set_curve(animation, AnimationCurveEaseOut);
  animation_set_handlers(animation, (AnimationHandlers) {
    .stopped = prv_animation_stopped_failure,
  }, data);

  data->result_animation = animation;
  animation_schedule(animation);
}

////////////////////////////
// Internal Helper Functions
////////////////////////////

//! Used to clean up the application's data before exiting
static void prv_cancel_fake_progress_timer(ProgressWindow *data) {
  if (data->fake_progress_timer != EVENTED_TIMER_INVALID_ID) {
    evented_timer_cancel(data->fake_progress_timer);
    data->fake_progress_timer = EVENTED_TIMER_INVALID_ID;
  }
}

static void prv_set_progress(ProgressWindow *data, int16_t progress) {
  data->progress_percent = CLIP(progress, data->progress_percent, MAX_PROGRESS_PERCENT);
  progress_layer_set_progress(&data->progress_layer, data->progress_percent);
}

static void prv_fake_update_progress(void *context) {
  ProgressWindow *data = context;

  prv_set_progress(data, data->progress_percent + FAKE_PROGRESS_UPDATE_AMOUNT);

  if (data->progress_percent >= data->max_fake_progress_percent) {
    // Hit the max, we're done
    data->fake_progress_timer = EVENTED_TIMER_INVALID_ID;
  } else {
    data->fake_progress_timer = evented_timer_register(FAKE_PROGRESS_UPDATE_INTERVAL, false,
                                                       prv_fake_update_progress, data);
  }
}

////////////////////////////
// Public API
////////////////////////////

void progress_window_set_max_fake_progress(ProgressWindow *window,
                                           int16_t max_fake_progress_percent) {
  window->max_fake_progress_percent = CLIP(max_fake_progress_percent, 0, MAX_PROGRESS_PERCENT);
}

void progress_window_set_progress(ProgressWindow *window, int16_t progress) {
  if (window->state == ProgressWindowState_FakeProgress) {
    // We've seen our first bit of real progress, stop faking it.
    prv_cancel_fake_progress_timer(window);
    window->state = ProgressWindowState_RealProgress;
  }

  prv_set_progress(window, progress);
}

void progress_window_set_result_success(ProgressWindow *window) {
  if (window->state == ProgressWindowState_Result) {
    // Ignore requests to change the result once we already have one
    return;
  }

  window->state = ProgressWindowState_Result;
  prv_cancel_fake_progress_timer(window);
  prv_set_progress(window, MAX_PROGRESS_PERCENT);
  prv_schedule_progress_success_animation(window);
}

void progress_window_set_result_failure(ProgressWindow *window, uint32_t timeline_res,
                                        const char *message, uint32_t delay) {
  if (window->state == ProgressWindowState_Result) {
    // Ignore requests to change the result once we already have one
    return;
  }

  window->state = ProgressWindowState_Result;
  prv_cancel_fake_progress_timer(window);
  prv_schedule_progress_failure_animation(window, timeline_res, message, delay);
}

void progress_window_set_callbacks(ProgressWindow *window, ProgressWindowCallbacks callbacks,
                                   void *context) {
  window->context = context;
  window->callbacks = callbacks;
}

void progress_window_set_back_disabled(ProgressWindow *window, bool disabled) {
  window_set_overrides_back_button(&window->window, disabled);
}

void progress_window_push(ProgressWindow *window, WindowStack *window_stack) {
  const bool animated = true;
  window_stack_push(window_stack, (Window *)window, animated);
}

void app_progress_window_push(ProgressWindow *window) {
  const bool animated = true;
  app_window_stack_push((Window *)window, animated);
}

void progress_window_pop(ProgressWindow *window) {
  const bool animated = true;
  window_stack_remove((Window *)window, animated);
}

void progress_window_init(ProgressWindow *data) {
  // Create and set up the window
  Window *window = &data->window;
  window_init(window, WINDOW_NAME("Progress Window"));
  window_set_background_color(window, PBL_IF_COLOR_ELSE(GColorLightGray, GColorWhite));

  const GRect *bounds = &window->layer.bounds;
  GPoint center = grect_center_point(bounds);
  const GRect progress_bounds = (GRect) {
    .origin = { center.x - (BAR_WIDTH / 2), center.y - (BAR_HEIGHT / 2) },
    .size = { BAR_WIDTH, BAR_HEIGHT },
  };

  ProgressLayer *progress_layer = &data->progress_layer;
  progress_layer_init(progress_layer, &progress_bounds);
#if PBL_COLOR
  progress_layer_set_foreground_color(progress_layer, GColorWhite);
  progress_layer_set_background_color(progress_layer, GColorBlack);
#endif
  progress_layer_set_corner_radius(progress_layer, PROGRESS_SUGGESTED_CORNER_RADIUS);
  layer_add_child(&window->layer, (Layer *)progress_layer);

  data->max_fake_progress_percent = PROGRESS_WINDOW_DEFAULT_FAKE_PERCENT;
  data->state = ProgressWindowState_FakeProgress;
  data->is_peek_layer_used = false;

  data->fake_progress_timer = evented_timer_register(FAKE_PROGRESS_UPDATE_INTERVAL, false,
                                                     prv_fake_update_progress, data);
  prv_set_progress(data, INITIAL_PERCENT);
}

void progress_window_deinit(ProgressWindow *data) {
  if (!data) {
    return;
  }

  animation_unschedule(data->result_animation);
  peek_layer_deinit(&data->peek_layer);
  data->is_peek_layer_used = false;

  prv_cancel_fake_progress_timer(data);

  if (data->peek_layer_timer != EVENTED_TIMER_INVALID_ID) {
    evented_timer_cancel(data->peek_layer_timer);
    data->peek_layer_timer = EVENTED_TIMER_INVALID_ID;
  }
}

ProgressWindow *progress_window_create(void) {
  ProgressWindow *window = applib_zalloc(sizeof(ProgressWindow));
  progress_window_init(window);
  return window;
}

void progress_window_destroy(ProgressWindow *window) {
  if (window) {
    progress_window_pop(window);
  }
  progress_window_deinit(window);
  applib_free(window);
}
