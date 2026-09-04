/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "applib/app.h"
#include "applib/ui/app_window_stack.h"
#include "applib/ui/window.h"
#include "applib/ui/text_layer.h"
#include "apps/prf/mfg_test_result.h"
#include "kernel/pbl_malloc.h"
#include <pbl/drivers/accel.h>
#include <pbl/drivers/rtc.h>
#include "process_state/app_state/app_state.h"
#include "process_management/pebble_process_md.h"
#include "pbl/services/evented_timer.h"
#include <pbl/logging/logging.h>

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#define STATUS_STRING_LEN 200

// Minimum variation required on each axis (in mG)
#define MIN_VARIATION_MG 500

#define TEST_DURATION_MS 5000
#define RESULT_DISPLAY_MS 1000
#define SAMPLE_INTERVAL_MS 100

typedef enum {
  STATE_IDLE,
  STATE_TESTING,
  STATE_RESULT,
} TestState;

static EventedTimerID s_timer;

typedef struct {
  Window window;

  TextLayer title;
  TextLayer status;
  char status_string[STATUS_STRING_LEN];

  TestState state;
  RtcTicks state_start_time;

  // Track min/max per axis
  int16_t min_x, max_x;
  int16_t min_y, max_y;
  int16_t min_z, max_z;

  uint32_t sample_count;

  // Variation per axis
  int16_t variation_x;
  int16_t variation_y;
  int16_t variation_z;

  bool test_passed;
} AppData;

static void prv_update_display(void *context) {
  AppData *data = context;

  AccelDriverSample sample;
  int ret = accel_peek(&sample);

  if (ret != 0) {
    sniprintf(data->status_string, sizeof(data->status_string),
              "ACCEL ERROR:\n%d", ret);
    text_layer_set_text(&data->status, data->status_string);
    return;
  }

  PBL_LOG_DBG("Accel (mG): X:%" PRIi16 " Y:%" PRIi16 " Z:%" PRIi16,
              sample.x, sample.y, sample.z);

  uint32_t elapsed = (uint32_t)(rtc_get_ticks() - data->state_start_time);

  switch (data->state) {
    case STATE_IDLE: {
      sniprintf(data->status_string, sizeof(data->status_string),
                "X: %" PRIi16 " mG\nY: %" PRIi16 " mG\nZ: %" PRIi16 " mG\n\nPress SEL",
                sample.x, sample.y, sample.z);
      break;
    }

    case STATE_TESTING: {
      // Track min/max for each axis
      if (data->sample_count == 0) {
        data->min_x = data->max_x = sample.x;
        data->min_y = data->max_y = sample.y;
        data->min_z = data->max_z = sample.z;
      } else {
        if (sample.x < data->min_x) data->min_x = sample.x;
        if (sample.x > data->max_x) data->max_x = sample.x;
        if (sample.y < data->min_y) data->min_y = sample.y;
        if (sample.y > data->max_y) data->max_y = sample.y;
        if (sample.z < data->min_z) data->min_z = sample.z;
        if (sample.z > data->max_z) data->max_z = sample.z;
      }
      data->sample_count++;

      // Calculate current variations
      int16_t curr_var_x = data->max_x - data->min_x;
      int16_t curr_var_y = data->max_y - data->min_y;
      int16_t curr_var_z = data->max_z - data->min_z;

      sniprintf(data->status_string, sizeof(data->status_string),
                "Testing...\nRotate device\n\nX: %" PRId16 " mG\nY: %" PRId16
                " mG\nZ: %" PRId16 " mG\n\n%" PRIu32 " sec remaining",
                curr_var_x, curr_var_y, curr_var_z,
                (TEST_DURATION_MS - elapsed) / 1000 + 1);

      if (elapsed >= TEST_DURATION_MS) {
        // Store final variations
        data->variation_x = curr_var_x;
        data->variation_y = curr_var_y;
        data->variation_z = curr_var_z;

        // Test passes if all axes vary by at least the minimum threshold
        data->test_passed =
            (data->variation_x >= MIN_VARIATION_MG &&
             data->variation_y >= MIN_VARIATION_MG &&
             data->variation_z >= MIN_VARIATION_MG);

        mfg_test_result_report(MfgTestId_Accel, data->test_passed, 0);

        data->state = STATE_RESULT;
        data->state_start_time = rtc_get_ticks();
      }
      break;
    }

    case STATE_RESULT: {
      if (elapsed >= RESULT_DISPLAY_MS) {
        app_window_stack_pop(false);
        return;
      }
      sniprintf(data->status_string, sizeof(data->status_string),
                "ACCEL: %s\n\nX: %" PRId16 " mG\nY: %" PRId16
                " mG\nZ: %" PRId16 " mG",
                data->test_passed ? "PASS" : "FAIL",
                data->variation_x, data->variation_y, data->variation_z);
      break;
    }
  }

  text_layer_set_text(&data->status, data->status_string);
}

static void prv_select_click_handler(ClickRecognizerRef recognizer, void *context) {
  AppData *data = context;

  switch (data->state) {
    case STATE_IDLE:
      // Start test
      data->state = STATE_TESTING;
      data->state_start_time = rtc_get_ticks();
      data->sample_count = 0;
      break;

    default:
      break;
  }
}

static void prv_click_config_provider(void *context) {
  window_single_click_subscribe(BUTTON_ID_SELECT, prv_select_click_handler);
}

static void prv_handle_init(void) {
  AppData *data = app_malloc_check(sizeof(AppData));
  *data = (AppData) {};

  app_state_set_user_data(data);

  Window *window = &data->window;
  window_init(window, "");
  window_set_fullscreen(window, true);

  TextLayer *title = &data->title;
  text_layer_init(title, &window->layer.bounds);
  text_layer_set_font(title, fonts_get_system_font(FONT_KEY_GOTHIC_24_BOLD));
  text_layer_set_text_alignment(title, GTextAlignmentCenter);
  text_layer_set_text(title, "ACCEL TEST");
  layer_add_child(&window->layer, &title->layer);

  TextLayer *status = &data->status;
  text_layer_init(status,
                  &GRect(5, 40,
                         window->layer.bounds.size.w - 5, window->layer.bounds.size.h - 40));
  text_layer_set_font(status, fonts_get_system_font(FONT_KEY_GOTHIC_18_BOLD));
  text_layer_set_text_alignment(status, GTextAlignmentCenter);
  layer_add_child(&window->layer, &status->layer);

  window_set_click_config_provider_with_context(window, prv_click_config_provider, data);

  data->state = STATE_IDLE;
  data->state_start_time = rtc_get_ticks();

  app_window_stack_push(window, true /* Animated */);

  s_timer = evented_timer_register(SAMPLE_INTERVAL_MS, true /* repeating */, prv_update_display, data);
}

static void s_main(void) {
  prv_handle_init();

  app_event_loop();

  evented_timer_cancel(s_timer);
}

const PebbleProcessMd* mfg_accel_app_get_info(void) {
  static const PebbleProcessMdSystem s_app_info = {
    .common.main_func = &s_main,
    // UUID: ED2E214A-D4B5-4360-B5EC-612B9E49FB95
    .common.uuid = { 0xED, 0x2E, 0x21, 0x4A, 0xD4, 0xB5, 0x43, 0x60,
                     0xB5, 0xEC, 0x61, 0x2B, 0x9E, 0x49, 0xFB, 0x95,
    },
    .name = "MfgAccel",
  };
  return (const PebbleProcessMd*) &s_app_info;
}
