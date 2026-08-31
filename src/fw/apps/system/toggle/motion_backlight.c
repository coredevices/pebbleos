/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "motion_backlight.h"

#include "applib/app.h"
#include "applib/ui/action_toggle.h"
#include "pbl/services/i18n/i18n.h"
#include "shell/prefs.h"

static bool prv_get_state(void *context) {
  return backlight_is_motion_enabled();
}

static void prv_set_state(bool enabled, void *context) {
  backlight_set_motion_enabled(enabled);
}

static const ActionToggleImpl s_motion_backlight_action_toggle_impl = {
  .window_name = "Motion Backlight Toggle",
  .prompt_icon = RESOURCE_ID_BACKLIGHT,
  .result_icon = RESOURCE_ID_BACKLIGHT,
  .prompt_enable_message = i18n_noop("Turn On Motion Backlight?"),
  .prompt_disable_message = i18n_noop("Turn Off Motion Backlight?"),
  .result_enable_message = i18n_noop("Motion\nBacklight On"),
  .result_disable_message = i18n_noop("Motion\nBacklight Off"),
  .callbacks = {
    .get_state = prv_get_state,
    .set_state = prv_set_state,
  },
};

static void prv_main(void) {
  action_toggle_push(&(ActionToggleConfig) {
    .impl = &s_motion_backlight_action_toggle_impl,
    .set_exit_reason = true,
  });
  app_event_loop();
}

const PebbleProcessMd *motion_backlight_toggle_get_app_info(void) {
  static const PebbleProcessMdSystem s_app_info = {
    .common = {
      .main_func = &prv_main,
      .uuid = MOTION_BACKLIGHT_TOGGLE_UUID,
      .visibility = ProcessVisibilityQuickLaunch,
    },
    .name = i18n_noop("Motion Backlight"),
  };
  return &s_app_info.common;
}
