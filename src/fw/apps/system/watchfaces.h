/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "process_management/pebble_process_md.h"

#define WATCHFACES_APP_COLOR_PRIMARY GColorJazzberryJam

//! Launch args for the Watchfaces app. When selector_mode is set, the app
//! opens the touch carousel selector instead of the menu list.
typedef struct WatchfacesLaunchArgs {
  bool selector_mode;
} WatchfacesLaunchArgs;

const PebbleProcessMd* watchfaces_get_app_info();
