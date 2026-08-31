/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "shell/normal/quick_launch.h"

#include "applib/ui/window.h"

#include <stdbool.h>

typedef enum QuickLaunchMenuCategory {
  //! Entries that are only visible in Quick Launch, e.g. the system toggles.
  QuickLaunchMenuCategoryActions,
  //! Regular launcher apps.
  QuickLaunchMenuCategoryApps,
} QuickLaunchMenuCategory;

//! @param parent The category menu to unwind along with this one on selection.
void quick_launch_app_menu_window_push(ButtonId button, bool is_tap,
                                       QuickLaunchMenuCategory category, Window *parent);
