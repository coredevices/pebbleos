/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "applib/app_timer.h"
#include "applib/ui/property_animation.h"
#include "applib/ui/dialogs/expandable_dialog.h"

#include <stdint.h>
#include <stdbool.h>

//! Callback from the dialog
typedef void(*TranscriptionConfirmationCallback)(void *callback_context);

typedef struct TranscriptionDialog {
  ExpandableDialog e_dialog;
  AppTimer *pop_timer;

  TranscriptionConfirmationCallback callback;
  void *callback_context;

  char *zero;
  char missing;
  bool was_pushed;
  bool select_pressed;
  bool keep_alive_on_select;
  PropertyAnimation *animation;

  // We cache this value so that we don't have to recompute it
  // in our animation helpers.
  uint32_t buffer_len;
} TranscriptionDialog;

//! Creates a TranscriptionDialog on the heap.
//! @return \ref TranscriptionDialog
TranscriptionDialog *transcription_dialog_create(void);

//! Initialize a transcription dialog that was already allocated
void transcription_dialog_init(TranscriptionDialog *transcription_dialog);

//! Pushes a TranscriptionDialog onto the app window stack if running on
//! the app task, otherwise the modal window stack.
//! @param transcription_dialog Pointer to the \ref TranscriptionDialog to push
//! @param window_stack Pointer to the \ref WindowStack to push to
void transcription_dialog_push(TranscriptionDialog *transcription_dialog,
                               WindowStack *window_stack);

//! Pushes a \ref TranscriptionDialog to the app's window stack
//! @param transcription_dialog Pointer to the \ref TranscriptionDialog to push
//! @note: Put a better comment here before exporting
void app_transcription_dialog_push(TranscriptionDialog *transcription_dialog);

//! Pops a \ref TranscriptionDialog from the app window stack or modal window
//! stack depending on the current task.
//! @param transcription_dialog Pointer to the \ref TranscriptionDialog to pop
void transcription_dialog_pop(TranscriptionDialog *transcription_dialog);

//! Updates the text in a \ref TranscriptionDialog.  This causes the dialog to
//! re-render and animate its contents.
//! @param transcription_dialog Pointer to the \ref TranscriptionDialog text to set
//! @param transcription The text to display
//! @param transcription_len The length of the text in the transcription
void transcription_dialog_update_text(TranscriptionDialog *transcription_dialog,
                                       char *transcription, uint16_t transcription_len);

//! Sets the callback that is called if the user confirms that the text
//! being displayed is what they intended.
//! @param transcription_dialog Pointer to the \ref TranscriptionDialog to set
//! @param callback The \ref TranscriptionConfirmationCallback to call if the user confirms text
//! @param callback_context The @c callback_context to pass to the confirmation handler
//! @note If the callback_context is NULL, then the \ref TranscriptionDialog will be passed
//!     the callback handler.
void transcription_dialog_set_callback(TranscriptionDialog *transcription_dialog,
                                       TranscriptionConfirmationCallback callback,
                                       void *callback_context);

//! @internal
//! Control whether the dialog closes when the select button is pressed
//! @param transcription_dialog Pointer to the \ref TranscriptionDialog to set
//! @param keep_alive_on_select If True the window will NOT close when select is pressed
//! @note The default is false (window will close when the selection has been made)
void transcription_dialog_keep_alive_on_select(TranscriptionDialog *transcription_dialog,
                                               bool keep_alive_on_select);
