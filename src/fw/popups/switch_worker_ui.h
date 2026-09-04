/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "applib/ui/window_stack.h"

//! @param new_worker_id The new ID that we'd like to ask the user to switch to
//! @param set_as_default Whether this new worker should become the default after being accepted
//! @param window_stack Which window stack to push the dialog to
void switch_worker_confirm(AppInstallId new_worker_id, bool set_as_default,
                           WindowStack *window_stack);

