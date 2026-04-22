/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "health.h"
#include "card_view.h"
#include "data.h"

#include "applib/app.h"
#include "applib/ui/dialogs/expandable_dialog.h"
#include "kernel/event_loop.h"
#include "kernel/pbl_malloc.h"
#include "kernel/ui/modals/modal_manager.h"
#include "popups/health_tracking_ui.h"
#include "process_state/app_state/app_state.h"
#include "pbl/services/activity/activity.h"
#include "pbl/services/activity/activity_private.h"
#include "pbl/services/timeline/timeline.h"
#include "resource/resource_ids.auto.h"
#include <pbl/logging/logging.h>
#include "shell/system_theme.h"

// Health app versions
// 0: Invalid (app was never opened)
// 1: Initial version
// 2: Graphs moved to mobile apps
// 3: 4.0 app redesign
// 4: Added insights onboarding prompt
#define CURRENT_HEALTH_APP_VERSION 4

////////////////////////////////////////////////////////////////////////////////////////////////////
// Main Structures
//

//! Main structure for application
typedef struct HealthAppData {
  HealthCardView *health_card_view;
  HealthData *health_data;
} HealthAppData;


////////////////////////////////////////////////////////////////////////////////////////////////////
// Callbacks
//

//! Tick timer service callback
//! @param tick_time Pointer to time structure
//! @param units_changed The time units changed
static void prv_tick_timer_handler(struct tm *tick_time, TimeUnits units_changed) {
  HealthAppData *health_app_data = app_state_get_user_data();
  health_data_update_step_derived_metrics(health_app_data->health_data);
  health_card_view_mark_dirty(health_app_data->health_card_view);
}

// Activity change callback
static void prv_health_service_event_handler(HealthEventType event, void *context) {
  HealthAppData *health_app_data = context;
  if (event == HealthEventMovementUpdate) {
    const uint32_t steps_today = health_service_sum_today(HealthMetricStepCount);
    health_data_update_steps(health_app_data->health_data, steps_today);
  } else if (event == HealthEventSleepUpdate) {
    const uint32_t seconds_sleep_today = health_service_sum_today(HealthMetricSleepSeconds);
    const uint32_t seconds_restful_sleep_today =
      health_service_sum_today(HealthMetricSleepRestfulSeconds);
    health_data_update_sleep(health_app_data->health_data, seconds_sleep_today,
                             seconds_restful_sleep_today);
  } else if (event == HealthEventHeartRateUpdate) {
    health_data_update_current_bpm(health_app_data->health_data);
  } else {
    health_data_update(health_app_data->health_data);
  }
  health_card_view_mark_dirty(health_app_data->health_card_view);
}


////////////////////////////////////////////////////////////////////////////////////////////////////
// Initialization and Termination
//

//! Show insights onboarding dialog for first-time users
static void prv_show_insights_onboarding_dialog(void) {
  /// Insights onboarding message
  static const char *text = i18n_noop("Psst! Want smart tips about your activity and sleep? "
                                      "You can enable Insights in the mobile app.");
  ExpandableDialog *dialog = expandable_dialog_create_with_params(
      "Insights Onboarding",
      RESOURCE_ID_HEALTH_ICON_MOON,
      text,
      system_theme_get_fg_color(),
      system_theme_get_bg_color(),
      NULL,
      RESOURCE_ID_ACTION_BAR_ICON_CHECK,
      expandable_dialog_close_cb);
  app_expandable_dialog_push(dialog);
}

//! Initialize application
static void prv_finish_initilization_cb(bool in_focus) {
  if (in_focus) {
    HealthAppData *health_app_data = app_state_get_user_data();

    tick_timer_service_subscribe(MINUTE_UNIT, prv_tick_timer_handler);

    health_service_set_heart_rate_sample_period(1 /* interval_s */);

    // Subscribing to health events causes a `HealthEventSignificantUpdate` which
    // will trigger us to update our health data
    health_service_events_subscribe(prv_health_service_event_handler, health_app_data);

    // Unsubscribe, we only want to do this on the initial appearance (opening the app)
    app_focus_service_unsubscribe();
  }
}

static void prv_initialize(void) {
  if (!activity_prefs_tracking_is_enabled()) {
    /// Health disabled text
    static const char *msg = i18n_noop("Track your steps, sleep, and more!"
                                       " Enable Pebble Health in the mobile app.");
    health_tracking_ui_show_message(RESOURCE_ID_HEART_TINY, msg, true);
    return;
  }

  if (!activity_is_initialized()) {
    /// Health waiting for time sync
    static const char *msg = i18n_noop("Health requires the time to be synced."
                                       " Please connect your phone.");
    health_tracking_ui_show_message(RESOURCE_ID_ALARM_CLOCK_TINY, msg, true);
    return;
  }

  const uint8_t previous_version = activity_prefs_get_health_app_opened_version();
  activity_prefs_set_health_app_opened_version(CURRENT_HEALTH_APP_VERSION);

  HealthAppData *health_app_data = app_zalloc_check(sizeof(HealthAppData));

  app_state_set_user_data(health_app_data);

  health_app_data->health_data = health_data_create();
  health_data_update_quick(health_app_data->health_data);

  health_app_data->health_card_view = health_card_view_create(health_app_data->health_data);

  health_card_view_push(health_app_data->health_card_view);

  // Show insights onboarding if user hasn't seen it yet and doesn't have insights enabled
  const bool insights_enabled = activity_prefs_activity_insights_are_enabled() ||
                                activity_prefs_sleep_insights_are_enabled();
  if (previous_version < CURRENT_HEALTH_APP_VERSION && !insights_enabled) {
    prv_show_insights_onboarding_dialog();
  }

  // Finish up initializing the app a bit later. This helps reduce lag when opening the app
  app_focus_service_subscribe_handlers((AppFocusHandlers){
    .did_focus = prv_finish_initilization_cb,
  });
}

//! Terminate application
static void prv_terminate(void) {
  HealthAppData *health_app_data = app_state_get_user_data();

  // cancel explicit hr sample period
  health_service_set_heart_rate_sample_period(0 /* interval_s */);

  if (health_app_data) {
    health_card_view_destroy(health_app_data->health_card_view);

    health_data_destroy(health_app_data->health_data);

    app_free(health_app_data);
  }
}

//! Main entry point
static void prv_main(void) {
  prv_initialize();
  app_event_loop();
  prv_terminate();
}

const PebbleProcessMd *health_app_get_info(void) {
  static const PebbleProcessMdSystem s_health_app_info = {
    .common = {
      .main_func = &prv_main,
      .uuid = UUID_HEALTH_DATA_SOURCE,
#if CAPABILITY_HAS_CORE_NAVIGATION4
      .visibility = ProcessVisibilityHidden,
#endif
    },
    .icon_resource_id = RESOURCE_ID_MENU_ICON_HEALTH,
    .name = i18n_noop("Health"),
  };
  return (const PebbleProcessMd*) &s_health_app_info;
}
