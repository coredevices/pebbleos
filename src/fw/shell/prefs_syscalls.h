/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "shell/prefs.h"

UnitsDistance sys_shell_prefs_get_units_distance(void);

#ifdef CONFIG_ORIENTATION_MANAGER
bool sys_display_orientation_is_left(void);
#endif
