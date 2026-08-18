/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "recognizer.h"

#include "applib/graphics/gtypes.h"

#include <stdbool.h>
#include <stdint.h>

//! @addtogroup UI
//! @{
//!   @addtogroup Recognizer
//!   @{

typedef struct SwipeRecognizerData SwipeRecognizerData;

//! Minimum travel (component-wise, on the major axis) from the touchdown point for a path to count
//! as a swipe. A shorter flick is treated as a tap or noise. Value from the reference PT2 touch-nav
//! gesture spec. Exposed so synthetic gesture generators size their paths from the same number the
//! recognizer enforces.
#define SWIPE_MIN_LENGTH_PX (30)

//! Maximum touchdown-to-liftoff duration for a swipe; a slower drag is a pan, not a flick. Value
//! from the reference PT2 touch-nav gesture spec. Exposed alongside the minimum length so a
//! synthetic generator cannot produce a path this recognizer is bound to reject.
#define SWIPE_MAX_DURATION_MS (300)

//! Swipe direction, also used as a bitmask when configuring which directions a swipe recognizer
//! accepts. Screen coordinates grow downward, so a positive y delta is a downward swipe.
typedef enum SwipeDirection {
  SwipeDirection_None  = 0,
  SwipeDirection_Up    = 1 << 0,
  SwipeDirection_Down  = 1 << 1,
  SwipeDirection_Left  = 1 << 2,
  SwipeDirection_Right = 1 << 3,
} SwipeDirection;

//! Create a swipe recognizer that accepts the directions set in \a direction_mask. The recognizer
//! stays Possible while tracking the path and Completes on liftoff if the path is a fast, straight,
//! long-enough flick whose direction is in the mask; otherwise it Fails.
//! @param event_cb event callback
//! @param user_data user data associated with recognizer
//! @param direction_mask bitwise-OR of the \ref SwipeDirection values to accept
//! @return recognizer reference
Recognizer *swipe_recognizer_create(RecognizerEventCb event_cb, void *user_data,
                                    uint8_t direction_mask);

//! Bytes of pointer-aligned storage required to hold a static (by-value) swipe recognizer plus its
//! implementation data. A build-time assert in swipe.c keeps it in sync with the real data size.
#define SWIPE_RECOGNIZER_STATIC_SIZE (RECOGNIZER_INSTANCE_SIZE + 112)

//! Initialize a swipe recognizer accepting \a direction_mask into caller-provided storage without
//! heap allocation.
//! @param storage storage of at least \ref SWIPE_RECOGNIZER_STATIC_SIZE bytes, pointer-aligned
//! @param event_cb event callback
//! @param user_data user data associated with recognizer
//! @param direction_mask bitwise-OR of the \ref SwipeDirection values to accept
//! @return recognizer reference (equal to \a storage), or NULL if \a event_cb is NULL
Recognizer *swipe_recognizer_init_static(void *storage, RecognizerEventCb event_cb, void *user_data,
                                         uint8_t direction_mask);

//! Get the swipe recognizer data from a recognizer. Should be used in the event callback to get the
//! data for a swipe recognizer event.
//! @param recognizer recognizer from which to get data
//! @return \ref SwipeRecognizerData reference
const SwipeRecognizerData *swipe_recognizer_get_data(const Recognizer *recognizer);

//! Get the recognized swipe direction. Valid once the recognizer has Completed; otherwise
//! \ref SwipeDirection_None.
//! @param recognizer recognizer from which to get the direction
//! @return recognized swipe direction
SwipeDirection swipe_recognizer_get_direction(const Recognizer *recognizer);

//! Get the velocity of the swipe, in pixels per second, component-wise. Computed over the
//! most-recent events within a short time window. Zero when the elapsed time is zero.
//! @param recognizer recognizer from which to get the velocity
//! @return velocity in px/s
GPoint swipe_recognizer_get_velocity(const Recognizer *recognizer);

//!   @} // end addtogroup Recognizer
//! @} // end addtogroup UI
