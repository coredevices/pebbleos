/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "applib/ui/dialogs/dialog.h"

void dialog_set_background_color(Dialog *dialog, GColor background_color) {
  return;
}

void dialog_set_icon(Dialog *dialog, uint32_t icon_id) {
  return;
}

void dialog_set_fullscreen(Dialog *dialog, bool is_fullscreen) {
  return;
}

void dialog_show_status_bar_layer(Dialog *dialog, bool show_status_layer) {
  return;
}

void dialog_set_text(Dialog *dialog, const char *text) {
  return;
}

void dialog_set_text_color(Dialog *dialog, GColor text_color) {
  return;
}

void dialog_set_vibe(Dialog *dialog, bool vibe_on_show) {
  return;
}

void dialog_set_timeout(Dialog *dialog, uint32_t timeout) {
  return;
}

void dialog_set_callbacks(Dialog *dialog, const DialogCallbacks *callbacks,
                          void *callback_context) {
  return;
}

void dialog_init(Dialog *dialog, const char *dialog_name) {
  return;
}

void dialog_push(Dialog *dialog, struct WindowStack *window_stack) {
  return;
}

void app_dialog_push(Dialog *dialog) {
}

void dialog_unload(Dialog *dialog) {
  if (dialog && dialog->callbacks.unload) {
    if (dialog->callback_context) {
      dialog->callbacks.unload(dialog->callback_context);
    } else {
      dialog->callbacks.unload(dialog);
    }
  }
}

void dialog_pop(Dialog *dialog) {
  dialog_unload(dialog);
}

void dialog_load(Dialog *dialog) {
  return;
}

void dialog_appear(Dialog *dialog) {
  return;
}

void dialog_add_status_bar_layer(Dialog *dialog, GRect status_layer_frame) {
  return;
}

GDrawCommandImage *dialog_create_icon(Dialog *dialog) {
  return NULL;
}

bool dialog_init_icon_layer(Dialog *dialog, GDrawCommandImage *image,
                            GPoint origin, bool animated) {
  return false;
}

void dialog_set_icon_animate_direction(Dialog *dialog, DialogIconAnimationDirection direction) {
  return;
}

void dialog_set_destroy_on_pop(Dialog *dialog, bool destroy_on_pop) {
  return;
}
