/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "recognizer.h"

#include "applib/graphics/gtypes.h"

//! @addtogroup UI
//! @{
//!   @addtogroup Recognizer
//!   @{

typedef struct PanRecognizerData PanRecognizerData;

//! Axis to which a pan recognizer is locked. A pan is only recognized when the
//! finger moves unambiguously along this axis.
typedef enum PanAxis {
  PanAxis_Horizontal,
  PanAxis_Vertical,
} PanAxis;

//! Create a pan recognizer locked to a single axis. The recognizer stays in the
//! Possible state until the finger moves unambiguously along \a axis and crosses
//! the start threshold, at which point it Starts. It Fails if the finger instead
//! moves unambiguously along the foreign axis.
//! @param event_cb event callback
//! @param user_data user data associated with recognizer
//! @param axis axis to which the pan is locked
//! @return recognizer reference
Recognizer *pan_recognizer_create(RecognizerEventCb event_cb, void *user_data, PanAxis axis);

//! Bytes of pointer-aligned storage required to hold a static (by-value) pan recognizer plus its
//! implementation data. A build-time assert in pan.c keeps it in sync with the real data size.
#define PAN_RECOGNIZER_STATIC_SIZE (RECOGNIZER_INSTANCE_SIZE + 112)

//! Initialize a pan recognizer locked to \a axis into caller-provided storage without heap
//! allocation.
//! @param storage storage of at least \ref PAN_RECOGNIZER_STATIC_SIZE bytes, pointer-aligned
//! @param event_cb event callback
//! @param user_data user data associated with recognizer
//! @param axis axis to which the pan is locked
//! @return recognizer reference (equal to \a storage), or NULL if \a event_cb is NULL
Recognizer *pan_recognizer_init_static(void *storage, RecognizerEventCb event_cb, void *user_data,
                                       PanAxis axis);

//! Get the pan recognizer data from a recognizer. Should be used in the event callback to get the
//! data for a pan recognizer event.
//! @param recognizer recognizer from which to get data
//! @return \ref PanRecognizerData reference
const PanRecognizerData *pan_recognizer_get_data(const Recognizer *recognizer);

//! Get the total movement of the pan since the touchdown point. Component-wise.
//! @param recognizer recognizer from which to get the delta
//! @return total delta from touchdown
GPoint pan_recognizer_get_total_delta(const Recognizer *recognizer);

//! Get the movement of the pan since it Started (i.e. since the start threshold was crossed). This
//! is exactly (0, 0) at the instant the recognizer transitions to Started, so live scroll that
//! consumes this value does not jump at gesture start. Component-wise.
//! @param recognizer recognizer from which to get the delta
//! @return delta since the pan started
GPoint pan_recognizer_get_delta_since_start(const Recognizer *recognizer);

//! Get the movement of the pan since the previous position update. Component-wise.
//! @param recognizer recognizer from which to get the delta
//! @return delta since the previous event
GPoint pan_recognizer_get_delta_since_prev(const Recognizer *recognizer);

//! Get the current velocity of the pan, in pixels per second, component-wise. Computed over the
//! most-recent events within a short time window (see PAN_VELOCITY_WINDOW_MS). Zero when the
//! elapsed time across the sampled events is zero.
//! @param recognizer recognizer from which to get the velocity
//! @return velocity in px/s
GPoint pan_recognizer_get_velocity(const Recognizer *recognizer);

//!   @} // end addtogroup Recognizer
//! @} // end addtogroup UI
