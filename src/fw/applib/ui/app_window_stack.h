/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "window.h"
#include "window_stack.h"

//! @addtogroup UI
//! @{
//!   @addtogroup WindowStack Window Stack
//! \brief The multiple window manager
//!
//! In Pebble OS, the window stack serves as the global manager of what window is presented,
//! ensuring that input events are forwarded to the topmost window.
//! The navigation model of Pebble centers on the concept of a vertical “stack” of windows, similar
//! to mobile app interactions.
//!
//! In working with the Window Stack API, the basic operations include push and pop. When an app wants to
//! display a new window, it pushes a new window onto the stack. This appears like a window sliding in
//! from the right. As an app is closed, the window is popped off the stack and disappears.
//!
//! For more complicated operations, involving multiple windows, you can determine which windows reside
//! on the stack, using window_stack_contains_window() and remove any specific window, using window_stack_remove().
//!
//! Refer to the \htmlinclude UiFramework.html (chapter "Window Stack") for a conceptual overview
//! of the window stack and relevant code examples.
//!
//! Also see the \ref WindowHandlers of a \ref Window for the callbacks that can be added to a window
//! in order to act upon window stack transitions.
//!
//!   @{

//! Pushes the given window on the window navigation stack,
//! on top of the current topmost window of the app.
//! @param window The window to push on top
//! @param animated Pass in `true` to animate the push using a sliding animation,
//! or `false` to skip the animation.
void app_window_stack_push(Window *window, bool animated);

//! Inserts the given window below the topmost window on the window
//! navigation stack.  If there is no window on the navigation stack, this is
//! the same as calling \ref window_stack_push() , otherwise, when the topmost
//! window is popped, this window will be visible.
//! @param window The window to insert next
void app_window_stack_insert_next(Window *window);

//! Pops the topmost window on the navigation stack
//! @param animated See \ref window_stack_remove()
//! @return The window that is popped, or NULL if there are no windows to pop.
Window* app_window_stack_pop(bool animated);

//! Pops all windows.
//! See \ref window_stack_remove() for a description of the `animated` parameter and notes.
void app_window_stack_pop_all(const bool animated);

//! Removes a given window from the window stack
//! that belongs to the app task.
//! @note If there are no windows for the app left on the stack, the app
//! will be killed by the system, shortly. To avoid this, make sure
//! to push another window shortly after or before removing the last window.
//! @param window The window to remove. If the window is NULL or if it
//! is not on the stack, this function is a no-op.
//! @param animated Pass in `true` to animate the removal of the window using
//! a side-to-side sliding animation to reveal the next window.
//! This is only used in case the window happens to be on top of the window
//! stack (thus visible).
//! @return True if window was successfully removed, false otherwise.
bool app_window_stack_remove(Window *window, bool animated);

//! Gets the topmost window on the stack that belongs to the app.
//! @return The topmost window on the stack that belongs to the app or
//! NULL if no app window could be found.
Window* app_window_stack_get_top_window(void);

//! Checks if the window is on the window stack
//! @param window The window to look for on the window stack
//! @return true if the window is currently on the window stack.
bool app_window_stack_contains_window(Window *window);

//! @internal
//! @return count of the number of windows are on the app window stack
uint32_t app_window_stack_count(void);

//!   @} // end addtogroup WindowStack
//! @} // end addtogroup UI
