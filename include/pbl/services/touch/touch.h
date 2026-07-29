/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "touch_event.h"
#include "gesture_event.h"

#include <stdbool.h>

typedef enum TouchState {
  TouchState_FingerUp,
  TouchState_FingerDown,
} TouchState;

typedef enum TouchGesture {
  TouchGesture_Tap,
  TouchGesture_DoubleTap,
} TouchGesture;

void touch_init(void);

//! Enable or disable the kernel's touch subscription used for the touch backlight feature.
//! When disabled, the touch sensor is only active if apps have subscribed to touch events.
void touch_set_backlight_enabled(bool enabled);

//! Hold the touch sensor powered for the system nav feature. Unlike the
//! backlight subscription this takes the sensor directly (no event-service
//! subscription). Taken when the master nav pref turns on, released when off.
void touch_set_system_hold(bool held);

//! @return true when SYSTEM touch navigation is effectively enabled. Defaults
//! to off; the shell drives it via touch_set_nav_enabled() with the
//! conjunction of the master "Touch" pref and the "Touch Navigation" sub-pref.
bool touch_nav_enabled(void);

//! Set the system nav gate. Intended to be driven by the shell's pref system
//! when the effective (master AND sub-pref) nav state changes.
void touch_set_nav_enabled(bool enabled);

//! @return true while the app twin's nav dispatcher is installed for the
//! running app (system nav active, or an opted-in third-party app under the
//! master pref). Set by the app twin's install/remove ops and cleared when the
//! app's shared touch subscription is torn down, so a dead app cannot leave it
//! stuck. Feeds the dispatch gate and touch-driven backlight behavior.
bool touch_app_nav_active(void);

//! Mark whether the app twin's nav dispatcher is installed.
void touch_set_app_nav_active(bool active);

//! @return true if at least one subscriber is currently registered for touch events.
bool touch_has_app_subscribers(void);

//! Globally enable or disable touch. When disabled:
//! - The sensor is powered down, even if subscribers exist.
//! - touch_handle_update() drops incoming events at the source.
//! - touch_service_is_enabled() returns false to apps.
//! Subscribers remain subscribed and resume receiving events when re-enabled.
//! Intended to back a user-facing setting (e.g. "water mode") — the shell
//! pref system persists the value and calls this on boot.
void touch_service_set_globally_enabled(bool enabled);

//! @return the current value of the global touch-enabled flag.
bool touch_service_is_globally_enabled(void);

//! Pass a touch update to the service (called by the touch driver)
//! @param touch_state whether or not the screen is touched
//! @param x x position of touch
//! @param y y position of touch
void touch_handle_update(TouchState touch_state, int16_t x, int16_t y);

//! Handle a gesture update (called by the touch driver)
//! @param gesture gesture that was detected
//! @param x x position of gesture (if applicable)
//! @param y y position of gesture
void touch_handle_gesture(TouchGesture gesture, int16_t x, int16_t y);

//! Reset the touch service.
void touch_reset(void);

//! Emit a synthetic Liftoff for an in-progress touch, using the last known
//! coordinates, so backlight hold counters and gesture state unwind cleanly
//! when touch is torn down with a finger still on the screen. No-op if no
//! finger is currently down. Reused by the master-pref-off transaction.
void touch_release_active(void);

//! Outcome of the wake-gate decision made on a Touchdown.
typedef struct TouchWakeGateResult {
  //! true when the touch is a wake tap (or DnD-suppressed touch) that must not
  //! drive navigation. false whenever nothing drives the backlight for this
  //! touch, so no consumer needs a separate early-exit signal.
  bool latch;
} TouchWakeGateResult;

//! Pure wake-gate decision, factored out so it is unit-testable independently
//! of the kernel event loop. Given the backlight state sampled around the
//! touch-driven wake, decide whether this Touchdown is non-navigational.
//! @param backlight_driven whether a subscriber ties the backlight to touch
//! @param dnd whether DnD suppresses the touch backlight for this touch
//! @param before light_is_on() sampled before the touch-driven wake
//! @param after light_is_on() sampled after the touch-driven wake
TouchWakeGateResult touch_wake_gate_on_touchdown(bool backlight_driven, bool dnd, bool before,
                                                 bool after);

//! Stamp non_navigational onto a touch event, latching the Touchdown decision
//! across the whole gesture. @p gate is only consulted on a Touchdown event;
//! PositionUpdate and Liftoff carry the latched value.
void touch_wake_gate_stamp(TouchEvent *event, TouchWakeGateResult gate);

//! Set whether the display is rotated 180° (left-hand mode). When rotated,
//! incoming touch coordinates are mirrored to match the rotated framebuffer
//! before being dispatched to subscribers.
void touch_set_rotated(bool rotated);
