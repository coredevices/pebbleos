/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "touch_event.h"

//! Touch event callback
//! @param event Touch event
//! @param context callback context
typedef void (*TouchEventHandler)(const TouchEvent *event, void *context);

//! Dispatch touch events to specified handler
//! @param touch_idx index of touch for which to dispatch events
//! @param event_handler callback to dispatch touch events to
//! @param context callback context
void touch_dispatch_touch_events(TouchIdx touch_idx, TouchEventHandler event_handler,
                                 void *context);
