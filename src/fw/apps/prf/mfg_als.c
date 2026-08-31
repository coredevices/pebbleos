/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-FileCopyrightText: 2025 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "mfg_als.h"

#include "applib/app.h"
#include "applib/ui/app_window_stack.h"
#include "applib/ui/text_layer.h"
#include "apps/prf/mfg_test_result.h"
#include <pbl/drivers/rtc.h>
#include "kernel/pbl_malloc.h"
#include "process_management/pebble_process_md.h"
#include "process_state/app_state/app_state.h"
#include "pbl/services/evented_timer.h"
#include "pbl/services/light.h"
#include <pbl/logging/logging.h>

#include <stdint.h>
#include <stdio.h>

// ALS pass/fail range (adjust these values based on your test requirements)
#ifdef CONFIG_BOARD_OBELIX
#define ALS_MIN_VALUE 600
#define ALS_MAX_VALUE 1500
#elif defined(CONFIG_BOARD_GETAFIX)
#define ALS_MIN_VALUE 8500
#define ALS_MAX_VALUE 11500
#else
#define ALS_MIN_VALUE 0
#define ALS_MAX_VALUE 65535
#endif

// Test parameters
#define COUNTDOWN_MS 2000
#define SAMPLE_DURATION_MS 3000
#define RESULT_DISPLAY_MS 1000
#define SAMPLE_INTERVAL_MS 100

typedef enum {
  ALSStateWaitForStart = 0,
  ALSStateCountdown,
  ALSStateSampling,
  ALSStatePass,
  ALSStateFail,
} ALSTestState;

#define AMBIENT_READING_STR_LEN 64

static EventedTimerID s_timer;

typedef struct {
  Window *window;
  TextLayer *status_text_layer;
  TextLayer *reading_text_layer;
  char status_text[AMBIENT_READING_STR_LEN];
  char ambient_reading[AMBIENT_READING_STR_LEN];

  ALSTestState test_state;
  RtcTicks state_start_time;
  uint64_t als_sum;
  uint32_t als_sample_count;
  uint32_t als_average;
} AmbientLightAppData;

static void prv_update_display(void *context) {
  AmbientLightAppData *data = context;
  uint32_t elapsed = (uint32_t)(rtc_get_ticks() - data->state_start_time);

  switch (data->test_state) {
    case ALSStateWaitForStart:
      snprintf(data->status_text, AMBIENT_READING_STR_LEN, "ALS Test\nPress CENTER\nto start");
      snprintf(data->ambient_reading, AMBIENT_READING_STR_LEN, " ");
      break;

    case ALSStateCountdown:
      snprintf(data->status_text, AMBIENT_READING_STR_LEN, "Place in\nlight box");
      snprintf(data->ambient_reading, AMBIENT_READING_STR_LEN, "Starting in: %"PRIu32"s",
               (COUNTDOWN_MS - elapsed) / 1000 + 1);
      if (elapsed >= COUNTDOWN_MS) {
        // Start sampling
        data->test_state = ALSStateSampling;
        data->state_start_time = rtc_get_ticks();
        data->als_sum = 0;
        data->als_sample_count = 0;
        PBL_LOG_INFO("ALS sampling started");
      }
      break;

    case ALSStateSampling: {
      // Take a sample
      uint32_t level = ambient_light_get_light_level();
      data->als_sum += level;
      data->als_sample_count++;

      snprintf(data->status_text, AMBIENT_READING_STR_LEN, "Sampling...");
      snprintf(data->ambient_reading, AMBIENT_READING_STR_LEN,
               "Time: %"PRIu32"s\nCurrent: %"PRIu32"\nSamples: %"PRIu32,
               (SAMPLE_DURATION_MS - elapsed) / 1000 + 1, level, data->als_sample_count);

      if (elapsed >= SAMPLE_DURATION_MS) {
        // Calculate average and determine pass/fail
        data->als_average = (uint32_t)(data->als_sum / data->als_sample_count);

        PBL_LOG_INFO("ALS test complete - Average: %"PRIu32" (samples: %"PRIu32")",
                     data->als_average, data->als_sample_count);

        bool passed = (
#if ALS_MIN_VALUE > 0
          data->als_average >= ALS_MIN_VALUE &&
#endif
          data->als_average <= ALS_MAX_VALUE);
        mfg_test_result_report(MfgTestId_ALS, passed, data->als_average);

        if (passed) {
          data->test_state = ALSStatePass;
          PBL_LOG_INFO("ALS test PASSED");
        } else {
          data->test_state = ALSStateFail;
          PBL_LOG_ERR("ALS test FAILED - Average %"PRIu32" outside range %d-%d",
                      data->als_average, ALS_MIN_VALUE, ALS_MAX_VALUE);
        }
        data->state_start_time = rtc_get_ticks();
      }
      break;
    }

    case ALSStatePass:
    case ALSStateFail:
      if (elapsed >= RESULT_DISPLAY_MS) {
        app_window_stack_pop(false);
        return;
      }
      snprintf(data->status_text, AMBIENT_READING_STR_LEN,
               data->test_state == ALSStatePass ? "PASS" : "FAIL");
      snprintf(data->ambient_reading, AMBIENT_READING_STR_LEN,
               "Average: %"PRIu32"\nRange: %d-%d",
               data->als_average, ALS_MIN_VALUE, ALS_MAX_VALUE);
      break;
  }

  text_layer_set_text(data->status_text_layer, data->status_text);
  text_layer_set_text(data->reading_text_layer, data->ambient_reading);
}


static void prv_select_click_handler(ClickRecognizerRef recognizer, void *context) {
  AmbientLightAppData *data = app_state_get_user_data();

  if (data->test_state == ALSStateWaitForStart) {
    // Start countdown
    data->test_state = ALSStateCountdown;
    data->state_start_time = rtc_get_ticks();

    PBL_LOG_INFO("ALS test started - countdown %d ms", COUNTDOWN_MS);
  }
}

static void prv_back_click_handler(ClickRecognizerRef recognizer, void *context) {
  app_window_stack_pop(true);
}

static void prv_config_provider(void *context) {
  window_single_click_subscribe(BUTTON_ID_SELECT, prv_select_click_handler);
  window_single_click_subscribe(BUTTON_ID_BACK, prv_back_click_handler);
}

static void prv_handle_init(void) {
  AmbientLightAppData *data = task_zalloc_check(sizeof(AmbientLightAppData));

  // Force backlight off for the duration of the test to avoid interfering
  // with ALS readings (e.g. when pressing CENTER to start sampling).
  light_allow(false);

  data->window = window_create();
  window_set_fullscreen(data->window, true);

  Layer *window_layer = window_get_root_layer(data->window);
  GRect bounds = window_layer->bounds;

  // Status text layer (top)
  data->status_text_layer = text_layer_create(GRect(0, 30, bounds.size.w, 80));
  text_layer_set_font(data->status_text_layer, fonts_get_system_font(FONT_KEY_GOTHIC_24_BOLD));
  text_layer_set_text_alignment(data->status_text_layer, GTextAlignmentCenter);
  layer_add_child(window_layer, text_layer_get_layer(data->status_text_layer));

  // Reading text layer (bottom)
  data->reading_text_layer = text_layer_create(GRect(0, 110, bounds.size.w, 80));
  text_layer_set_font(data->reading_text_layer, fonts_get_system_font(FONT_KEY_GOTHIC_18));
  text_layer_set_text_alignment(data->reading_text_layer, GTextAlignmentCenter);
  layer_add_child(window_layer, text_layer_get_layer(data->reading_text_layer));

  // Initialize state
  data->test_state = ALSStateWaitForStart;
  data->state_start_time = rtc_get_ticks();
  data->als_sum = 0;
  data->als_sample_count = 0;
  data->als_average = 0;

  // Set up click handlers
  window_set_click_config_provider(data->window, prv_config_provider);

  app_state_set_user_data(data);
  app_window_stack_push(data->window, true);

  // Register evented timer for 100ms updates
  s_timer = evented_timer_register(SAMPLE_INTERVAL_MS, true /* repeating */, prv_update_display, data);

  PBL_LOG_INFO("ALS test initialized - range: %d-%d", ALS_MIN_VALUE, ALS_MAX_VALUE);
}

static void prv_handle_deinit(void) {
  AmbientLightAppData *data = app_state_get_user_data();

  // Cancel evented timer
  evented_timer_cancel(s_timer);

  text_layer_destroy(data->status_text_layer);
  text_layer_destroy(data->reading_text_layer);
  window_destroy(data->window);
  task_free(data);

  light_allow(true);
}

static void prv_main(void) {
  prv_handle_init();
  app_event_loop();
  prv_handle_deinit();
}

const PebbleProcessMd* mfg_als_app_get_info(void) {
  static const PebbleProcessMdSystem s_ambient_light_info = {
    .common.main_func = prv_main,
    .name = "MfgALS"
  };
  return (const PebbleProcessMd*) &s_ambient_light_info;
}
