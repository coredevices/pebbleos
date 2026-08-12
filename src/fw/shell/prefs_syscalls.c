/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "prefs_syscalls.h"

#include "syscall/syscall_internal.h"

DEFINE_SYSCALL(UnitsDistance, sys_shell_prefs_get_units_distance, void) {
  return shell_prefs_get_units_distance();
}

#ifdef CONFIG_ORIENTATION_MANAGER
DEFINE_SYSCALL(bool, sys_display_orientation_is_left, void) {
  return display_orientation_is_left();
}
#endif
