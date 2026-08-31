/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

//! The "Nothing" Quick Launch action. Binding a button to it makes the button
//! inert: the shell recognizes the app id and never launches anything.

#include "quick_launch_nothing.h"

#include "pbl/services/i18n/i18n.h"

static void prv_main(void) {
  // Never runs; the shell drops the launch before it gets here.
}

const PebbleProcessMd *quick_launch_nothing_get_app_info(void) {
  static const PebbleProcessMdSystem s_app_info = {
    .common = {
      .main_func = &prv_main,
      .uuid = QUICK_LAUNCH_NOTHING_UUID,
      .visibility = ProcessVisibilityQuickLaunch,
    },
    /// Quick Launch action that makes the button do nothing at all.
    .name = i18n_noop("Nothing"),
  };
  return &s_app_info.common;
}
