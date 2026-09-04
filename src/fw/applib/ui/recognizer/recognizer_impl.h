/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "recognizer.h"

#include <stddef.h>
#include <stdbool.h>

typedef struct RecognizerImpl {
  //! Handle touch event
  //! @note This function must be implemented
  //! @param recognizer recognizer to handle event
  //! @param touch_event touch event that just occurred
  void (*handle_touch_event)(Recognizer *recognizer, const TouchEvent *touch_event);

  //! Cancel the recognizer
  //! @note This function must be implemented
  //! @param recognizer recognizer to be reset
  //! @return true if event should be fired, otherwise false
  bool (*cancel)(Recognizer *recognizer);

  //! Reset the recognizer
  //! @note This function must be implemented
  //! @param recognizer recognizer to be reset
  void (*reset)(Recognizer *recognizer);

  //! Called when the recognizer is failed by a manager. Used to clean up any timers or otherwise
  //! stop further recognition activity until the recognizer is reset.
  //! @param recognizer recognizer that failed
  void (*on_fail)(Recognizer *recognizer);

  //! Called when the recognizer is destroyed
  //! @param recognizer recognizer that will be destroyed
  void (*on_destroy)(Recognizer *recognizer);
} RecognizerImpl;

//! Create a recognizer with implementation specific data. This is used by internal and custom
//! recognizers to instantiate a recognizer from the base class. A recognizer created from this
//! function cannot be used without an implementation.
//! @note A recognizer cannot be created without implementation details
//! @param impl recognizer implementation
//! @param data recognizer-specific data (copied into created recognizer)
//! @param data_size data size
//! @param event_cb event callback
//! @param user_data context to provide to event callback
//! @return NULL if an error occurs, otherwise a pointer to the newly created recognizer
Recognizer *recognizer_create_with_data(const RecognizerImpl *impl, const void *data,
                                        size_t data_size, RecognizerEventCb event_cb,
                                        void *user_data);

//! Initialize a recognizer in caller-provided storage. Same initialization as
//! \ref recognizer_create_with_data but performs no allocation. The storage must be large enough to
//! hold a Recognizer instance plus \a data_size bytes and be suitably aligned (see
//! \ref RECOGNIZER_STATIC_STORAGE). A recognizer initialized this way is marked static and will not
//! be freed by \ref recognizer_destroy.
//! @param storage caller-provided storage for the recognizer
//! @param impl recognizer implementation
//! @param data recognizer-specific data (copied into the recognizer)
//! @param data_size data size
//! @param event_cb event callback
//! @param user_data context to provide to event callback
//! @return NULL if an error occurs, otherwise a pointer to the initialized recognizer
Recognizer *recognizer_init_static_with_data(void *storage, const RecognizerImpl *impl,
                                             const void *data, size_t data_size,
                                             RecognizerEventCb event_cb, void *user_data);

//! Get the implementation specific data for the recognizer. If the implementation specified does
//! not match the implementation belonging to the recognizer, NULL is returned
//! @param recognizer recognizer
//! @param impl pointer to implementation definition
//! @return pointer to implementation-specific data, or NULL if incorrect implementation specified
void *recognizer_get_impl_data(Recognizer *recognizer, const RecognizerImpl *impl);

//! Transition the recognizer state. This is called by the implementation to change the state of the
//! recognizer when it needs to update its state. It cannot not be called by anything else. The
//! state transition must be valid
//! @param recognizer recognizer to modify
//! @param new_state new recognizer state
void recognizer_transition_state(Recognizer *recognizer, RecognizerState new_state);
