/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "workout.h"
#include "active.h"
#include "controller.h"
#include "data.h"
#include "dialog.h"
#include "summary.h"
#include "utils.h"

#include "applib/app.h"
#include "applib/ui/dialogs/expandable_dialog.h"
#include "applib/ui/ui.h"
#include "kernel/pbl_malloc.h"
#include "popups/health_tracking_ui.h"
#include "process_state/app_state/app_state.h"
#include "resource/resource_ids.auto.h"
#include "pbl/services/i18n/i18n.h"
#include "pbl/services/activity/activity.h"
#include "pbl/services/activity/health_util.h"
#include "pbl/services/activity/workout_service.h"
#include "resource/resource_ids.auto.h"
#include "popups/health_tracking_ui.h"

// Workout app versions
// 0: Invalid (app was never opened)
// 1: Initial version
#define CURRENT_WORKOUT_APP_VERSION 1

typedef struct WorkoutAppData {
  WorkoutSummaryWindow *summary_window;
  WorkoutActiveWindow *active_window;
  WorkoutDialog detected_workout_dialog;
  WorkoutDialog ended_workout_dialog;
  ActivitySession ongoing_session;
  WorkoutData workout_data;
  WorkoutController workout_controller;
} WorkoutAppData;

#define DEFAULT_ACTIVITY_TYPE (ActivitySessionType_Run)

static ActivitySessionType s_activity_type = DEFAULT_ACTIVITY_TYPE;

////////////////////////////////////////////////////////////////////////////////////////////////////
// Helpers

static void prv_prep_and_open_active_window(ActivitySessionType type) {
  WorkoutAppData *data = app_state_get_user_data();

  data->workout_controller = (WorkoutController) {
    .is_paused = workout_service_is_paused,
    .pause = workout_service_pause_workout,
    .stop = workout_service_stop_workout,
    .update_data = workout_data_update,
    .metric_to_string = workout_data_fill_metric_value,
    .get_metric_value = workout_data_get_metric_value,
    .get_distance_string = health_util_get_distance_string,
  };

  data->active_window = workout_active_create_for_activity_type(type,
                                                                &data->workout_data,
                                                                &data->workout_controller);
  workout_active_window_push(data->active_window);
}

static void prv_start_workout_cb(ActivitySessionType type) {
  workout_service_start_workout(type);
  prv_prep_and_open_active_window(type);
}

static void prv_select_workout_cb(ActivitySessionType type) {
  WorkoutAppData *data = app_state_get_user_data();

  s_activity_type = type;

  workout_summary_update_activity_type(data->summary_window, type);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
//! Detected Workout

static void prv_detected_workout_up_click_handler(ClickRecognizerRef recognizer, void *context) {
  WorkoutAppData *data = context;

  if (workout_service_takeover_activity_session(&data->ongoing_session)) {
    prv_prep_and_open_active_window(data->ongoing_session.type);
  }

  workout_dialog_pop(&data->detected_workout_dialog);
}

static void prv_detected_workout_down_click_handler(ClickRecognizerRef recognizer, void *context) {
  WorkoutAppData *data = context;

  workout_push_summary_window();

  workout_dialog_pop(&data->detected_workout_dialog);
}

static void prv_detected_workout_click_config_provider(void *context) {
  window_single_click_subscribe(BUTTON_ID_UP, prv_detected_workout_up_click_handler);
  window_single_click_subscribe(BUTTON_ID_DOWN, prv_detected_workout_down_click_handler);
}

static void prv_show_workout_detected_dialog(WorkoutAppData *data) {
  WorkoutDialog *workout_dialog = &data->detected_workout_dialog;

  workout_dialog_init(workout_dialog, "Workout Detected");
  Dialog *dialog = workout_dialog_get_dialog(workout_dialog);

  dialog_show_status_bar_layer(dialog, true);
  dialog_set_fullscreen(dialog, true);
  dialog_set_background_color(dialog, PBL_IF_COLOR_ELSE(GColorYellow, GColorWhite));
  dialog_set_text_color(dialog, GColorBlack);
  dialog_set_icon(dialog, RESOURCE_ID_WORKOUT_APP_DETECTED);
  dialog_set_icon_animate_direction(dialog, DialogIconAnimateNone);
  dialog_set_destroy_on_pop(dialog, false);

  const char *dialog_text =
      workout_utils_get_detection_text_for_activity(data->ongoing_session.type);

  workout_dialog_set_text(workout_dialog, dialog_text);

  char text_buffer[32];
  const uint32_t length_s = rtc_get_time() - data->ongoing_session.start_utc;
  health_util_format_hours_minutes_seconds(
      text_buffer, sizeof(text_buffer), length_s, true, workout_dialog);

  workout_dialog_set_subtext(workout_dialog, text_buffer);

  workout_dialog_set_click_config_provider(workout_dialog,
                                           prv_detected_workout_click_config_provider);
  workout_dialog_set_click_config_context(workout_dialog, data);

  i18n_free_all(workout_dialog);

  app_workout_dialog_push(workout_dialog);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
//! Workout Ended

static void prv_show_workout_ended_dialog(WorkoutAppData *data) {
  WorkoutDialog *workout_dialog = &data->ended_workout_dialog;

  workout_dialog_init(workout_dialog, "Workout Ended");
  Dialog *dialog = workout_dialog_get_dialog(workout_dialog);

  dialog_show_status_bar_layer(dialog, true);
  dialog_set_fullscreen(dialog, true);
  dialog_set_background_color(dialog, PBL_IF_COLOR_ELSE(GColorYellow, GColorWhite));
  dialog_set_text_color(dialog, GColorBlack);
  dialog_set_icon(dialog, RESOURCE_ID_WORKOUT_APP_END);
  dialog_set_icon_animate_direction(dialog, DialogIconAnimateNone);
  dialog_set_destroy_on_pop(dialog, false);
  dialog_set_timeout(dialog, DIALOG_TIMEOUT_DEFAULT);

  workout_dialog_set_text(workout_dialog, i18n_get("Workout\nEnded", workout_dialog));
  workout_dialog_set_action_bar_hidden(workout_dialog, true);

  i18n_free_all(workout_dialog);

  app_workout_dialog_push(workout_dialog);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public Functions

void workout_push_summary_window(void) {
  WorkoutAppData *data = app_state_get_user_data();

  data->summary_window = workout_summary_window_create(s_activity_type,
                                                       prv_start_workout_cb,
                                                       prv_select_workout_cb);
  workout_summary_window_push(data->summary_window);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Initialization

static void prv_init(void) {
    if (!activity_is_initialized()) {
    /// Workouts waiting for time sync
    static const char *msg = i18n_noop("Workout requires the time to be synced."
                                       " Please connect your phone.");
    health_tracking_ui_show_message(RESOURCE_ID_ALARM_CLOCK_TINY, msg, true);
    return;
  }

  if (!activity_prefs_tracking_is_enabled()) {
    /// Health disabled text
    static const char *msg = i18n_noop("Enable Pebble Health in the mobile app to track workouts");
    health_tracking_ui_show_message(RESOURCE_ID_HEART_TINY, msg, true);
    return;
  }

  if (activity_prefs_get_workout_app_opened_version() != CURRENT_WORKOUT_APP_VERSION) {
    /// Workout app first use text
    static const char *msg = i18n_noop("Wear your watch snug and 2 fingers' width above "
                                       "your wrist bone for best results.");
    health_tracking_ui_show_message(RESOURCE_ID_WORKOUT_APP_HR_PULSE_TINY, msg, true);
  }

  activity_prefs_set_workout_app_opened_version(CURRENT_WORKOUT_APP_VERSION);

  WorkoutAppData *data = app_zalloc_check(sizeof(WorkoutAppData));
  app_state_set_user_data(data);

  workout_service_frontend_opened();

  if (workout_service_is_workout_ongoing()) {
    if (app_launch_get_args() == WorkoutLaunchArg_EndWorkout) {
      workout_service_stop_workout();
      prv_show_workout_ended_dialog(data);
    } else {
      ActivitySessionType existing_workout_type;
      workout_service_get_current_workout_type(&existing_workout_type);
      prv_prep_and_open_active_window(existing_workout_type);
    }
    return;
  }

  const bool found_automatic_session =
      workout_utils_find_ongoing_activity_session(&data->ongoing_session);

  if (found_automatic_session) {
    prv_show_workout_detected_dialog(data);
  } else {
    workout_push_summary_window();
  }
}

static void prv_deinit(void) {
  WorkoutAppData *data = app_state_get_user_data();
  workout_service_frontend_closed();
  app_free(data);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// App Main

static void prv_main(void) {
  prv_init();
  app_event_loop();
  prv_deinit();
}

const PebbleProcessMd *workout_app_get_info(void) {
  static const PebbleProcessMdSystem s_workout_app_info = {
    .common = {
      .main_func = &prv_main,
      .uuid = {0xfe, 0xf8, 0x2c, 0x82, 0x71, 0x76, 0x4e, 0x22,
               0x88, 0xde, 0x35, 0xa3, 0xfc, 0x18, 0xd4, 0x3f},
    },
    .name = i18n_noop("Workout"),
    .icon_resource_id = RESOURCE_ID_ACTIVITY_TINY,
  };
  return (const PebbleProcessMd*) &s_workout_app_info;
}
