/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "applib/graphics/gtypes.h"
#include "applib/ui/window_private.h"
#include "applib/ui/window_stack.h"

// The function creates a Spinner UI window on the heap with the specified color.
// The window is cleaned up when it is popped.
// Returns a pointer to the created window
Window* spinner_ui_window_get(GColor spinner_color);
