/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include <stdint.h>

#include "process_management/app_install_types.h"

//! This function creates a popup window displaying a missed wakeup events notification
//! along with the application names that were missed
//! Note: missed_apps_ids is free'd by this function
//! @param missed_apps_count number of app names to display
//! @param missed_app_ids an array of AppInstallIds of the app names to display
void wakeup_popup_window(uint8_t missed_apps_count, AppInstallId *missed_app_ids);

