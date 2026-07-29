/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

//! Touch event type
typedef enum TouchEventType {
  TouchEvent_Touchdown,
  TouchEvent_Liftoff,
  TouchEvent_PositionUpdate,
} TouchEventType;

//! Touch event data, carried directly in PebbleTouchEvent
typedef struct TouchEvent {
  TouchEventType type:8;
  //! true when the touch must not drive navigation at all: a DnD-suppressed
  //! touch (the screen stayed dark), or a screen-off touch under gesture-based
  //! wake. Latched on Touchdown and carried across the whole gesture.
  bool non_navigational;
  int16_t x;
  int16_t y;
  //! true when this touch's Touchdown turned the screen on: the gesture must
  //! not tap or swipe (it targeted the wake, not the UI), but a drag that
  //! follows is deliberate, now-visible input and may pan. Latched on
  //! Touchdown and carried across the whole gesture. Appended after y so the
  //! x/y offsets stay app-ABI-compatible (see test_touch__event_abi_unchanged).
  bool wake_touch;
} TouchEvent;

_Static_assert(sizeof(TouchEvent) <= 9, "TouchEvent must stay small; it rides inside PebbleEvent");
