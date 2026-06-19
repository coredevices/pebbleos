/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include <stdio.h>

#include "applib/app.h"
#include "applib/ui/app_window_stack.h"
#include "applib/ui/text_layer.h"
#include "applib/ui/window.h"
#include "apps/system_app_ids.h"
#include "kernel/pbl_malloc.h"
#include "process_state/app_state/app_state.h"
#include "pbl/services/activity/activity.h"
#include "pbl/services/hrm/hrm_manager.h"
#include "system/logging.h"
#include "util/time/time.h"

#define SPO2_TEST_PERCENT_LEN 12
#define SPO2_TEST_DETAIL_LEN 160

typedef struct {
  HRMSessionRef session;
  EventServiceInfo hrm_event_info;

  // True if we force-enabled blood oxygen monitoring on entry and must restore it on exit.
  bool restore_blood_oxygen_pref;

  uint32_t sample_count;

  Window window;
  TextLayer title_layer;
  TextLayer percent_layer;
  TextLayer detail_layer;

  char percent_string[SPO2_TEST_PERCENT_LEN];
  char detail_string[SPO2_TEST_DETAIL_LEN];
} AppData;

static const char *prv_get_quality_string(HRMQuality quality) {
  switch (quality) {
    case HRMQuality_OffWrist:   return "Off Wrist";
    case HRMQuality_Worst:      return "Worst";
    case HRMQuality_Poor:       return "Poor";
    case HRMQuality_Acceptable: return "Acceptable";
    case HRMQuality_Good:       return "Good";
    case HRMQuality_Excellent:  return "Excellent";
    default:                    return "None";
  }
}

static void prv_update_display(AppData *app_data, const HRMSpO2Data *spo2) {
  if (spo2->quality == HRMQuality_OffWrist) {
    snprintf(app_data->percent_string, sizeof(app_data->percent_string), "--");
  } else {
    snprintf(app_data->percent_string, sizeof(app_data->percent_string), "%"PRIu8"%%",
             spo2->percent);
  }

  snprintf(app_data->detail_string, sizeof(app_data->detail_string),
           "Quality: %s\n"
           "Confidence: %"PRIu8"\n"
           "Valid lvl: %"PRIu8"\n"
           "Invalid: %s\n"
           "Samples: %"PRIu32,
           prv_get_quality_string(spo2->quality),
           spo2->confidence,
           spo2->valid_level,
           spo2->invalid ? "yes" : "no",
           app_data->sample_count);

  text_layer_set_text(&app_data->percent_layer, app_data->percent_string);
  text_layer_set_text(&app_data->detail_layer, app_data->detail_string);
  layer_mark_dirty(&app_data->window.layer);
}

static void prv_handle_hrm_data(PebbleEvent *e, void *context) {
  AppData *app_data = app_state_get_user_data();
  PebbleHRMEvent *hrm = &e->hrm;

  if (hrm->event_type == HRMEvent_SpO2) {
    app_data->sample_count++;
    prv_update_display(app_data, &hrm->spo2);
  } else if (hrm->event_type == HRMEvent_SubscriptionExpiring) {
    // Re-subscribe so the test keeps running for long sessions.
    app_data->session = sys_hrm_manager_app_subscribe(
        APP_ID_SPO2_TEST, 1 /*update_interval_s*/, SECONDS_PER_HOUR, HRMFeature_SpO2);
  }
}

static void prv_enable_spo2(AppData *app_data) {
  // Force blood oxygen monitoring on for the duration of the test; the HRM manager masks out the
  // SpO2/IR path entirely when the user pref is disabled. Remember the prior value to restore.
  app_data->restore_blood_oxygen_pref = !activity_prefs_blood_oxygen_is_enabled();
  if (app_data->restore_blood_oxygen_pref) {
    activity_prefs_set_blood_oxygen_enabled(true);
  }

  app_data->hrm_event_info = (EventServiceInfo) {
    .type = PEBBLE_HRM_EVENT,
    .handler = prv_handle_hrm_data,
  };
  event_service_client_subscribe(&app_data->hrm_event_info);

  app_data->session = sys_hrm_manager_app_subscribe(
      APP_ID_SPO2_TEST, 1 /*update_interval_s*/, SECONDS_PER_HOUR, HRMFeature_SpO2);
}

static void prv_disable_spo2(AppData *app_data) {
  event_service_client_unsubscribe(&app_data->hrm_event_info);
  sys_hrm_manager_unsubscribe(app_data->session);

  if (app_data->restore_blood_oxygen_pref) {
    activity_prefs_set_blood_oxygen_enabled(false);
    app_data->restore_blood_oxygen_pref = false;
  }
}

static void prv_init(void) {
  AppData *app_data = app_malloc_check(sizeof(*app_data));
  *app_data = (AppData) { 0 };
  app_state_set_user_data(app_data);

  Window *window = &app_data->window;
  window_init(window, "SpO2 Test");
  window_set_fullscreen(window, true);

  GRect bounds = window->layer.bounds;

  GRect title_frame = bounds;
  title_frame.origin.y += 6;
  title_frame.size.h = 26;
  TextLayer *title_tl = &app_data->title_layer;
  text_layer_init(title_tl, &title_frame);
  text_layer_set_font(title_tl, fonts_get_system_font(FONT_KEY_GOTHIC_18_BOLD));
  text_layer_set_text_alignment(title_tl, GTextAlignmentCenter);
  text_layer_set_text(title_tl, "SpO2 Test");
  layer_add_child(&window->layer, &title_tl->layer);

  GRect percent_frame = bounds;
  percent_frame.origin.y += 34;
  percent_frame.size.h = 42;
  TextLayer *percent_tl = &app_data->percent_layer;
  text_layer_init(percent_tl, &percent_frame);
  text_layer_set_font(percent_tl, fonts_get_system_font(FONT_KEY_BITHAM_34_MEDIUM_NUMBERS));
  text_layer_set_text_alignment(percent_tl, GTextAlignmentCenter);
  text_layer_set_text(percent_tl, "--");
  layer_add_child(&window->layer, &percent_tl->layer);

  GRect detail_frame = bounds;
  detail_frame.origin.y += 80;
  detail_frame.size.h = bounds.size.h - 80;
  detail_frame.origin.x += 8;
  detail_frame.size.w -= 16;
  TextLayer *detail_tl = &app_data->detail_layer;
  text_layer_init(detail_tl, &detail_frame);
  text_layer_set_font(detail_tl, fonts_get_system_font(FONT_KEY_GOTHIC_18));
  text_layer_set_text_alignment(detail_tl, GTextAlignmentLeft);

  if (!sys_hrm_manager_is_hrm_present()) {
    text_layer_set_text(detail_tl, "No HRM present");
  } else {
    text_layer_set_text(detail_tl, "Measuring...\nKeep watch on wrist");
    prv_enable_spo2(app_data);
  }
  layer_add_child(&window->layer, &detail_tl->layer);

  app_window_stack_push(window, true);
}

static void prv_deinit(void) {
  AppData *app_data = app_state_get_user_data();
  prv_disable_spo2(app_data);
}

static void prv_main(void) {
  prv_init();
  app_event_loop();
  prv_deinit();
}

const PebbleProcessMd *spo2_test_get_app_info(void) {
  static const PebbleProcessMdSystem s_spo2_test_app_info = {
    .name = "SpO2 Test",
    .common.uuid = { 0x6d, 0x6c, 0x3b, 0x0e, 0x2a, 0x4f, 0x4c, 0x91,
                     0x9b, 0x2d, 0x1f, 0x77, 0x84, 0x3c, 0x5e, 0xa1 },
    .common.main_func = &prv_main,
    // Hidden from the launcher: launched from the Settings -> System -> Debugging menu instead, so
    // it sits behind the developer prompt and stays out of the way for normal users.
    .common.visibility = ProcessVisibilityHidden,
  };
  return (sys_hrm_manager_is_hrm_present()) ? (const PebbleProcessMd *)&s_spo2_test_app_info : NULL;
}
