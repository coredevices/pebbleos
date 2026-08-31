/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "window.h"

//! Internal interface for glayer to schedule a render for the window:
//! @param window Pointer to the window to schedule
void window_schedule_render(Window *window);

//! Setup the click config provider
//! @param window Pointer to the window to setup the click config provider
void window_setup_click_config_provider(Window *window);

//! Internal interface for window_stack to signal putting a window on/off screen:
//! @param window Pointer to the window to set on the screen
//! @param new_on_screen Boolean indicating if there is a new window or an old one
//!     If the window is on screen and this is true, or the window is off-screen and
//!     this is false, then this is a no-op
//! @param call_window_appear_handlers Boolean indicating whether or not to call the
//!     window appear/disappear handler
void window_set_on_screen(Window *window, bool new_on_screen, bool call_window_appear_handlers);

//! Internal helper to calculate the frame of a window (e.g. inside a transition container layer)
//! NOTE: even if window.fullscreen==false, it still returns result.origin.y == 0
//! When rendering, window_render() takes care of it
//! @param fullscreen boolean indicating if the window is full screen or not
//! @return \ref GRect
GRect window_calc_frame(bool fullscreen);

//! @internal
//! @param window Pointer to the \ref Window to check
//! @return boolean indicating if the passed window has a status bar
bool window_has_status_bar(Window *window);

//! @param window Pointer to the \ref Window to set
//! @param overrides_back_button Boolean indicating if the back button has been overriden
//!     in the \ref ClickConfigProvider of the passed \ref Window
void window_set_overrides_back_button(Window *window, bool overrides_back_button);

//! @internal
//! Called to unload a window.
//! @param window The \ref Window to unload
void window_unload(Window *window);
