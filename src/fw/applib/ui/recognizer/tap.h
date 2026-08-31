/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "recognizer.h"

#include "applib/graphics/gtypes.h"

typedef struct TapRecognizerData TapRecognizerData;

//! @addtogroup UI
//! @{
//!   @addtogroup Recognizer
//!   @{

//! Create a tap recognizer. The default recognizer recognizes a single tap from a single finger
//! @param event_cb event callback
//! @param user_data user data associated with recognizer
//! @return recognizer reference
Recognizer *tap_recognizer_create(RecognizerEventCb event_cb, void *user_data);

//! Bytes of pointer-aligned storage required to hold a static (by-value) tap recognizer plus its
//! implementation data. Use to declare embedded storage; a build-time assert in tap.c keeps it in
//! sync with the real data size.
#define TAP_RECOGNIZER_STATIC_SIZE (RECOGNIZER_INSTANCE_SIZE + 40)

//! Initialize a default single-tap recognizer into caller-provided storage without heap allocation.
//! @param storage storage of at least \ref TAP_RECOGNIZER_STATIC_SIZE bytes, pointer-aligned
//! @param event_cb event callback
//! @param user_data user data associated with recognizer
//! @return recognizer reference (equal to \a storage), or NULL if \a event_cb is NULL
Recognizer *tap_recognizer_init_static(void *storage, RecognizerEventCb event_cb, void *user_data);

//! Get the tap recognizer data from a recognizer. Should be used in the event callback to get the
//! the data for a tap recognizer event
//! @param recognizer recognizer from which to get data
//! @return \ref TapRecognizerData reference
const TapRecognizerData *tap_recognizer_get_data(const Recognizer *recognizer);

//! Get the coordinate of the recognized tap. The coordinate is taken from the last position update,
//! not the liftoff. Valid once the recognizer has completed.
//! @param recognizer recognizer from which to get the tap coordinate
//! @return tap coordinate
GPoint tap_recognizer_get_tap_point(const Recognizer *recognizer);

//!   @} // end addtogroup Recognizer
//! @} // end addtogroup UI

// TODO: Add configuration methods & getters for state
// https://pebbletechnology.atlassian.net/browse/PBL-28983
