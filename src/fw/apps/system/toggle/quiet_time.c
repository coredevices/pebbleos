/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "quiet_time.h"

#include "applib/app.h"
#include "applib/ui/action_toggle.h"
#include "pbl/services/i18n/i18n.h"
#include "pbl/services/notifications/do_not_disturb_toggle.h"

static void prv_main(void) {
  do_not_disturb_toggle_push(ActionTogglePrompt_Auto, true /* set_exit_reason */);
  app_event_loop();
}

const PebbleProcessMd *quiet_time_toggle_get_app_info(void) {
  static const PebbleProcessMdSystem s_app_info = {
    .common = {
      .main_func = &prv_main,
      .uuid = QUIET_TIME_TOGGLE_UUID,
      .visibility = ProcessVisibilityQuickLaunch,
    },
    .name = i18n_noop("Quiet Time"),
  };
  return &s_app_info.common;
}
