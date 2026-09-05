/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include <stdint.h>

//! Gesture event type
typedef enum GestureEventType {
  GestureEvent_Tap,
  GestureEvent_DoubleTap,
  GestureEvent_Palm,
} GestureEventType;

//! Gesture event data, carried directly in PebbleGestureEvent
typedef struct GestureEvent {
  GestureEventType type:8;
  int16_t x;
  int16_t y;
} GestureEvent;