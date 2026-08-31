/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "pbl/services/touch/touch_event.h"

#include <stdbool.h>

//! Callback for touch events.
//! @param event The touch event data
//! @param context User-provided context
typedef void (*TouchServiceHandler)(const TouchEvent *event, void *context);

//! Subscribe to touch events. The touch sensor is enabled while subscribed.
//! @param handler Callback invoked for each touch event
//! @param context User context passed to the callback
void touch_service_subscribe(TouchServiceHandler handler, void *context);

//! Unsubscribe from touch events. The touch sensor is disabled if no other subscribers remain.
void touch_service_unsubscribe(void);

//! @return true if touch input is currently being delivered to apps.
//! Returns false on platforms without a touchscreen, or when touch has been
//! disabled system-wide (future feature). Apps can poll this (for example on
//! window appear) to avoid looking broken when touch is unavailable.
bool touch_service_is_enabled(void);

//! Opt this app into (or out of) touch navigation: the system gesture bridge that maps
//! swipes/taps on a MenuLayer/ScrollLayer to button presses. Third-party apps are opt-out by
//! default and receive no touch navigation even when the user has enabled it system-wide; call
//! this with \a enable true to participate. Takes effect immediately when touch navigation is
//! enabled system-wide, otherwise once the user enables it.
//! @param enable true to participate in touch navigation, false to stop participating.
void app_touch_navigation_enable(bool enable);
