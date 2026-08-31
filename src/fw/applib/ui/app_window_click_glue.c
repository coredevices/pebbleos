/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "app_window_click_glue.h"

#include "process_state/app_state/app_state.h"
#include "applib/ui/click_internal.h"
#include "applib/ui/window.h"

////////////////////////////////////////////////
// App + Click Recognizer + Window : Glue code
//
// [MT] This is a bit ugly, because I decided to to save memory and have all windows in an app share an array of
// click recognizers (which lives in AppContext) instead of each window having its own.
// See the comment near AppContext.click_recognizer.

void app_click_config_setup_with_window(ClickManager *click_manager, struct Window *window) {
  void *context = window->click_config_context;
  if (!context) {
    // Default context is the window.
    context = window;
  }

  click_manager_clear(click_manager);

  for (unsigned int button_id = 0; button_id < NUM_BUTTONS; ++button_id) {
    // For convenience, assign the context:
    click_manager->recognizers[button_id].config.context = context;
  }

  if (window->click_config_provider) {
    window_call_click_config_provider(window, context);
  }
}

