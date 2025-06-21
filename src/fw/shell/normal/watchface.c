/*
 * Copyright 2024 Google LLC
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "watchface.h"

#include "apps/system_app_ids.h"
#include "apps/system_apps/launcher/launcher_app.h"
#include "apps/system_apps/settings/settings_quick_launch.h"
#include "apps/system_apps/settings/settings_quick_launch_app_menu.h"
#include "apps/system_apps/settings/settings_quick_launch_setup_menu.h"
#include "apps/system_apps/timeline/timeline.h"
#include "apps/watch/low_power/low_power_face.h"
#include "kernel/event_loop.h"
#include "kernel/low_power.h"
#include "kernel/ui/modals/modal_manager.h"
#include "popups/timeline/peek.h"
#include "process_management/app_manager.h"
#include "process_management/pebble_process_md.h"
#include "services/common/analytics/analytics.h"
#include "services/common/compositor/compositor_transitions.h"
#include "services/normal/notifications/do_not_disturb.h"
#include "system/logging.h"
#include "system/passert.h"
#include "shell/prefs.h"

#define QUICK_LAUNCH_HOLD_MS (400)

static ClickManager s_click_manager;

static bool prv_should_ignore_button_click(void) {
  if (app_manager_get_task_context()->closing_state != ProcessRunState_Running) {
    // Ignore if the app is not running (such as if it is in the process of closing)
    return true;
  }
  if (low_power_is_active()) {
    // If we're in low power mode we dont allow any interaction
    return true;
  }
  return false;
}

static void prv_launch_app_via_button(AppLaunchEventConfig *config,
                                      ClickRecognizerRef recognizer) {
  config->common.button = click_recognizer_get_button_id(recognizer);
  app_manager_put_launch_app_event(config);
}

static void prv_quick_launch_handler(ClickRecognizerRef recognizer, void *data) {
  ButtonId button = click_recognizer_get_button_id(recognizer);
  if (!quick_launch_is_enabled(button)) {
    return;
  }
  AppInstallId app_id = quick_launch_get_app(button);
  if (app_id == INSTALL_ID_INVALID) {
    app_id = app_install_get_id_for_uuid(&quick_launch_setup_get_app_info()->uuid);
  }
  prv_launch_app_via_button(&(AppLaunchEventConfig) {
    .id = app_id,
    .common.reason = APP_LAUNCH_QUICK_LAUNCH,
  }, recognizer);
}

static void prv_launch_timeline(ClickRecognizerRef recognizer, void *data) {
  static TimelineArgs s_timeline_args;
  const bool is_up = (click_recognizer_get_button_id(recognizer) == BUTTON_ID_UP);
  if (is_up) {
    PBL_LOG(LOG_LEVEL_DEBUG, "Launching timeline in past mode.");
    s_timeline_args.direction = TimelineIterDirectionPast;
    analytics_inc(ANALYTICS_DEVICE_METRIC_TIMELINE_PAST_LAUNCH_COUNT, AnalyticsClient_System);
  } else {
    PBL_LOG(LOG_LEVEL_DEBUG, "Launching timeline in future mode.");
    s_timeline_args.direction = TimelineIterDirectionFuture;
    analytics_inc(ANALYTICS_DEVICE_METRIC_TIMELINE_FUTURE_LAUNCH_COUNT, AnalyticsClient_System);
  }
  s_timeline_args.launch_into_pin = true;
  s_timeline_args.stay_in_list_view = true;
  timeline_peek_get_item_id(&s_timeline_args.pin_id);

  const CompositorTransition *animation = NULL;
  const bool is_future = (s_timeline_args.direction == TimelineIterDirectionFuture);
  const bool timeline_is_destination = true;
#if PBL_ROUND
  animation = compositor_dot_transition_timeline_get(is_future, timeline_is_destination);
#else
  const bool jump = (!uuid_is_invalid(&s_timeline_args.pin_id) && !timeline_peek_is_first_event());
  animation = jump ? compositor_peek_transition_timeline_get() :
                     compositor_slide_transition_timeline_get(is_future, timeline_is_destination,
                                                              timeline_peek_is_future_empty());
#endif
  prv_launch_app_via_button(&(AppLaunchEventConfig) {
    .id = APP_ID_TIMELINE,
    .common.args = &s_timeline_args,
    .common.transition = animation,
  }, recognizer);
}

static void prv_configure_click_handler(ButtonId button_id, ClickHandler single_click_handler) {
  ClickConfig *cfg = &s_click_manager.recognizers[button_id].config;
  cfg->long_click.delay_ms = QUICK_LAUNCH_HOLD_MS;
  cfg->long_click.handler = prv_quick_launch_handler;
  cfg->click.handler = single_click_handler;
}

static void prv_launch_launcher_app(ClickRecognizerRef recognizer, void *data) {
  static const LauncherMenuArgs s_launcher_args = { .reset_scroll = true };
  prv_launch_app_via_button(&(AppLaunchEventConfig) {
    .id = APP_ID_LAUNCHER_MENU,
    .common.args = &s_launcher_args,
  }, recognizer);
}

#if CAPABILITY_HAS_CORE_NAVIGATION4
static void prv_launch_health_or_timeline(ClickRecognizerRef recognizer, void *data) {
  if(timeline_prefs_get_past_on_up()) {
    prv_launch_timeline(recognizer, data);
  } else {
    prv_launch_app_via_button(&(AppLaunchEventConfig) {
      .id = APP_ID_HEALTH_APP,
    }, recognizer);
  }
}
#endif // CAPABILITY_HAS_CORE_NAVIGATION4

static ClickHandler prv_get_up_click_handler(void) {
#if CAPABILITY_HAS_CORE_NAVIGATION4
  return prv_launch_health_or_timeline;
#else
  return prv_launch_timeline;
#endif // CAPABILITY_HAS_CORE_NAVIGATION4
}

static void prv_dismiss_timeline_peek(ClickRecognizerRef recognizer, void *data) {
  timeline_peek_dismiss();
}

static void prv_watchface_configure_click_handlers(void) {
  prv_configure_click_handler(BUTTON_ID_UP, prv_get_up_click_handler());
  prv_configure_click_handler(BUTTON_ID_DOWN, prv_launch_timeline);
  prv_configure_click_handler(BUTTON_ID_SELECT, prv_launch_launcher_app);
  prv_configure_click_handler(BUTTON_ID_BACK, prv_dismiss_timeline_peek);
}

void watchface_init(void) {
  click_manager_init(&s_click_manager);
  prv_watchface_configure_click_handlers();
}

void watchface_handle_button_event(PebbleEvent *e) {
  if (prv_should_ignore_button_click()) {
    return;
  }
  switch (e->type) {
    case PEBBLE_BUTTON_DOWN_EVENT:
      click_recognizer_handle_button_down(&s_click_manager.recognizers[e->button.button_id]);
      break;
    case PEBBLE_BUTTON_UP_EVENT:
      click_recognizer_handle_button_up(&s_click_manager.recognizers[e->button.button_id]);
      break;
    default:
      PBL_CROAK("Invalid event type: %u", e->type);
      break;
  }
}

static void prv_watchface_launch_low_power(void) {
  PBL_LOG(LOG_LEVEL_DEBUG, "Switching default watchface to low_power_mode watchface");
  app_manager_put_launch_app_event(&(AppLaunchEventConfig) {
    .id = APP_ID_LOW_POWER_FACE,
  });
}

void watchface_launch_default(const CompositorTransition *animation) {
  app_manager_put_launch_app_event(&(AppLaunchEventConfig) {
    .id = watchface_get_default_install_id(),
    .common.transition = animation,
  });
}

static void kernel_callback_watchface_launch(void* data) {
  watchface_launch_default(NULL);
}

void command_watch(void) {
  launcher_task_add_callback(kernel_callback_watchface_launch, NULL);
}

void watchface_start_low_power(void) {
  app_manager_set_minimum_run_level(ProcessAppRunLevelNormal);
  prv_watchface_launch_low_power();
}

void watchface_reset_click_manager(void) {
  click_manager_reset(&s_click_manager);
}
