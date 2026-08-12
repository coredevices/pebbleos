/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "applib/ui/action_bar_layer.h"
#include "applib/ui/action_bar_layer_private.h"
#include "applib/ui/animation.h"
#include "applib/ui/animation_timing.h"
#include "applib/ui/window.h"
#include "applib/graphics/gtypes.h"
#include "kernel/pebble_tasks.h"
#include "process_management/pebble_process_md.h"

#include "clar.h"

#include <time.h>

#include "stubs_app_state.h"
#include "stubs_app_timer.h"
#include "stubs_bitblt.h"
#include "stubs_click.h"
#include "stubs_compiled_with_legacy2_sdk.h"
#include "stubs_gbitmap.h"
#include "stubs_heap.h"
#include "stubs_logging.h"
#include "stubs_passert.h"
#include "stubs_pbl_malloc.h"
#include "stubs_print.h"
#include "stubs_resources.h"
#include "stubs_syscalls.h"
#include "stubs_unobstructed_area.h"

static bool s_left_handed;
static PebbleTask s_task = PebbleTask_App;
static ProcessAppSDKType s_sdk_type = ProcessAppSDKType_System;
static const PebbleProcessMd s_md;

bool display_orientation_is_left(void) {
  return s_left_handed;
}

PebbleTask pebble_task_get_current(void) {
  return s_task;
}

const PebbleProcessMd *sys_process_manager_get_current_process_md(void) {
  return &s_md;
}

ProcessAppSDKType process_metadata_get_app_sdk_type(const PebbleProcessMd *md) {
  (void)md;
  return s_sdk_type;
}

PlatformType process_manager_current_platform(void) {
  return PBL_PLATFORM_TYPE_CURRENT;
}

void window_schedule_render(struct Window *window) {
  (void)window;
}

void recognizer_destroy(Recognizer *recognizer) {
  (void)recognizer;
}

void recognizer_add_to_list(Recognizer *recognizer, RecognizerList *list) {
  (void)recognizer;
  (void)list;
}

void recognizer_remove_from_list(Recognizer *recognizer, RecognizerList *list) {
  (void)recognizer;
  (void)list;
}

RecognizerManager *window_get_recognizer_manager(Window *window) {
  (void)window;
  return NULL;
}

void recognizer_manager_cancel_and_reset(RecognizerManager *manager) {
  (void)manager;
}

bool recognizer_list_iterate(RecognizerList *list, RecognizerListIteratorCb iter_cb,
                             void *context) {
  (void)list;
  (void)iter_cb;
  (void)context;
  return false;
}

void recognizer_manager_register_recognizer(RecognizerManager *manager, Recognizer *recognizer) {
  (void)manager;
  (void)recognizer;
}

void recognizer_manager_deregister_recognizer(RecognizerManager *manager, Recognizer *recognizer) {
  (void)manager;
  (void)recognizer;
}

GDrawState graphics_context_get_drawing_state(GContext *ctx) {
  (void)ctx;
  return (GDrawState){0};
}

void graphics_context_set_drawing_state(GContext *ctx, GDrawState draw_state) {
  (void)ctx;
  (void)draw_state;
}

bool graphics_release_frame_buffer(GContext *ctx, GBitmap *buffer) {
  (void)ctx;
  (void)buffer;
  return false;
}

AnimationProgress animation_timing_curve(AnimationProgress time_normalized, AnimationCurve curve) {
  (void)curve;
  return time_normalized;
}

void window_set_click_config_provider_with_context(Window *window,
                                                   ClickConfigProvider click_config_provider,
                                                   void *context) {
  (void)window;
  (void)click_config_provider;
  (void)context;
}

void window_raw_click_subscribe(ButtonId button_id, ClickHandler down_handler,
                                ClickHandler up_handler, void *context) {
  (void)button_id;
  (void)down_handler;
  (void)up_handler;
  (void)context;
}

void window_set_click_context(ButtonId button_id, void *context) {
  (void)button_id;
  (void)context;
}

void graphics_context_set_fill_color(GContext *ctx, GColor color) {
  (void)ctx;
  (void)color;
}

void graphics_context_set_compositing_mode(GContext *ctx, GCompOp mode) {
  (void)ctx;
  (void)mode;
}

void graphics_fill_rect(GContext *ctx, const GRect *rect) {
  (void)ctx;
  (void)rect;
}

void graphics_fill_oval(GContext *ctx, GRect rect, GOvalScaleMode scale_mode) {
  (void)ctx;
  (void)rect;
  (void)scale_mode;
}

void graphics_draw_bitmap_in_rect(GContext *ctx, const GBitmap *bitmap, const GRect *rect) {
  (void)ctx;
  (void)bitmap;
  (void)rect;
}

uint16_t time_ms(time_t *tloc, uint16_t *out_ms) {
  (void)tloc;
  if (out_ms) {
    *out_ms = 0;
  }
  return 0;
}

static void prv_reset(void) {
  s_left_handed = false;
  s_task = PebbleTask_App;
  s_sdk_type = ProcessAppSDKType_System;
}

void test_action_bar_layer__initialize(void) {
  prv_reset();
}

void test_action_bar_layer__cleanup(void) {
  prv_reset();
}

static int16_t prv_add_and_get_x(void) {
  Window window = {};
  layer_init(&window.layer, &GRect(0, 0, DISP_COLS, DISP_ROWS));
  ActionBarLayer bar;
  action_bar_layer_init(&bar);
  action_bar_layer_add_to_window(&bar, &window);
  const int16_t x = bar.layer.frame.origin.x;
  action_bar_layer_deinit(&bar);
  layer_deinit(&window.layer);
  return x;
}

void test_action_bar_layer__system_app_stays_right_when_not_left_handed(void) {
  cl_assert(action_bar_layer_is_on_right());
  cl_assert_equal_i(prv_add_and_get_x(), DISP_COLS - ACTION_BAR_WIDTH);
  cl_assert_equal_i(action_bar_layer_get_content_origin_x(), 0);
}

void test_action_bar_layer__system_app_moves_left_when_left_handed(void) {
  s_left_handed = true;
  cl_assert(!action_bar_layer_is_on_right());
  cl_assert_equal_i(prv_add_and_get_x(), 0);
  cl_assert_equal_i(action_bar_layer_get_content_origin_x(), ACTION_BAR_WIDTH);
}

void test_action_bar_layer__third_party_app_stays_right_when_left_handed(void) {
  s_left_handed = true;
  s_sdk_type = ProcessAppSDKType_4x;
  cl_assert(action_bar_layer_is_on_right());
  cl_assert_equal_i(prv_add_and_get_x(), DISP_COLS - ACTION_BAR_WIDTH);
}

void test_action_bar_layer__kernel_ui_moves_left_when_left_handed(void) {
  s_left_handed = true;
  s_task = PebbleTask_KernelMain;
  s_sdk_type = ProcessAppSDKType_4x;
  cl_assert(!action_bar_layer_is_on_right());
  cl_assert_equal_i(prv_add_and_get_x(), 0);
}

void test_action_bar_layer__inset_bounds_shifts_origin_when_on_left(void) {
  s_left_handed = true;
  const GRect inset = action_bar_layer_inset_bounds(GRect(0, 0, DISP_COLS, DISP_ROWS));
  cl_assert_equal_i(inset.origin.x, ACTION_BAR_WIDTH);
  cl_assert_equal_i(inset.size.w, DISP_COLS - ACTION_BAR_WIDTH);
}
