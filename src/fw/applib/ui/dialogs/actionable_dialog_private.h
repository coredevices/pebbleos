/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

//! An ActionableDialog is a dialog that has an action bar on the right hand side
//! of the window.  The user can specify there own custom \ref ActionBarLayer to
//! override the default behaviour or specify a \ref ClickConfigProvider to tie
//! into the default \ref ActionBarLayer provided by the dialog.
#pragma once

#include "applib/graphics/gtypes.h"
#include "applib/ui/action_bar_layer.h"
#include "applib/ui/click.h"
#include "applib/ui/dialogs/dialog.h"

//! Different types of action bar. Two commonly used types are built in:
//! Confirm and Decline.  Alternatively, the user can supply their own
//! custom action bar.
typedef enum DialogActionBarType {
  //! SELECT: Confirm icon
  DialogActionBarConfirm,
  //! SELECT: Decline icon
  DialogActionBarDecline,
  //! UP: Confirm icon, DOWN: Decline icon
  DialogActionBarConfirmDecline,
  //! Provide your own action bar
  DialogActionBarCustom
} DialogActionBarType;

typedef struct ActionableDialog {
  Dialog dialog;
  DialogActionBarType action_bar_type;
  union {
    struct {
      GBitmap *select_icon;
    };
    struct {
      GBitmap *up_icon;
      GBitmap *down_icon;
    };
  };
  ActionBarLayer *action_bar;
  ClickConfigProvider config_provider;
  void *user_data;
} ActionableDialog;
