/* SPDX-FileCopyrightText: 2025 Elad Dvash */
/* SPDX-License-Identifier: Apache-2.0 */

#include "backlight_state.h"

#include "applib/app.h"
#include "applib/ui/action_toggle.h"
#include "pbl/services/i18n/i18n.h"
#include "pbl/services/light.h"
#include "shell/prefs.h"

static bool prv_get_state(void *context) {
  return backlight_is_enabled();
}

static void prv_set_state(bool enabled, void *context) {
  if (enabled != backlight_is_enabled()) {
    light_toggle_enabled();
  }
}

static const ActionToggleImpl s_backlight_state_action_toggle_impl = {
  .window_name = "Backlight Toggle",
  .prompt_icon = RESOURCE_ID_BACKLIGHT,
  .result_icon = RESOURCE_ID_BACKLIGHT,
  .prompt_enable_message = i18n_noop("Turn On Backlight?"),
  .prompt_disable_message = i18n_noop("Turn Off Backlight?"),
  .result_enable_message = i18n_noop("Backlight On"),
  .result_disable_message = i18n_noop("Backlight Off"),
  .callbacks = {
    .get_state = prv_get_state,
    .set_state = prv_set_state,
  },
};

static void prv_main(void) {
  action_toggle_push(&(ActionToggleConfig) {
    .impl = &s_backlight_state_action_toggle_impl,
    .set_exit_reason = true,
  });
  app_event_loop();
}

const PebbleProcessMd *backlight_state_toggle_get_app_info(void) {
  static const PebbleProcessMdSystem s_app_info = {
    .common = {
      .main_func = &prv_main,
      .uuid = BACKLIGHT_STATE_TOGGLE_UUID,
      .visibility = ProcessVisibilityQuickLaunch,
    },
    .name = i18n_noop("Backlight"),
  };
  return &s_app_info.common;
}
