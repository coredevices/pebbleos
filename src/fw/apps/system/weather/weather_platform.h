/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "pebble_compat.h"

// v4.18 (Core Devices) exports an app-facing touch service (touch_service_subscribe
// + flat TouchEvent{type,x,y}), so touch is ENABLED on the color/touch platforms.
// In the firmware-linked system-app build the SDK's PBL_PLATFORM_* macros are not
// defined, so also honor the firmware's CONFIG_TOUCH (obelix/emery has a CST816
// touchscreen). This gates the sub-views' touch handlers (clock/list/saved/globe);
// the struct layouts they share REQUIRE this to be consistent across all TUs.
#if defined(PBL_PLATFORM_EMERY) || defined(PBL_PLATFORM_GABBRO) || \
    (defined(CONFIG_TOUCH) && CONFIG_TOUCH)
#define WEATHER_PLATFORM_TOUCH_COLOR 1
#else
#define WEATHER_PLATFORM_TOUCH_COLOR 0
#endif
