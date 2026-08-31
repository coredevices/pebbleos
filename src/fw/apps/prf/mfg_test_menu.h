/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "apps/prf/mfg_test_result.h"
#include "process_management/pebble_process_md.h"

#include <stdbool.h>

const PebbleProcessMd* mfg_test_menu_semi_finished_app_get_info(void);
const PebbleProcessMd* mfg_test_menu_finished_app_get_info(void);

//! Check and clear the relaunch flag (set when returning from a test)
bool mfg_test_menu_should_relaunch(void);

//! Check if a test is available in the current mode
bool mfg_test_menu_is_test_available(MfgTestId test_id);
