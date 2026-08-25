/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "weather_service_app.h"

#include "syscall/syscall.h"

bool weather_service_peek(WeatherServiceSnapshot *snapshot_out) {
  return sys_weather_get_snapshot(snapshot_out);
}
