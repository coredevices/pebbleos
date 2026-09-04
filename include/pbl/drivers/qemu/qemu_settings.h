/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include <stdint.h>

typedef enum {
  QemuSetting_FirstBootLogicEnable = 1,     // Returns a bool
  QemuSetting_DefaultConnected = 2,         // Returns a bool
  QemuSetting_DefaultPluggedIn = 3,         // Returns a bool
} QemuSetting;



// ---------------------------------------------------------------------------------------
// API
uint32_t qemu_setting_get(QemuSetting);
