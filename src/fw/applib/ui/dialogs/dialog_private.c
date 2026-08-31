/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "dialog_private.h"

#include "applib/app_timer.h"
#include "applib/applib_malloc.auto.h"
#include "applib/ui/dialogs/dialog.h"
#include "applib/ui/kino/kino_reel/transform.h"
#include "applib/ui/kino/kino_reel/scale_segmented.h"
#include "applib/ui/vibes.h"
#include "applib/ui/window_stack.h"
#include "process_state/app_state/app_state.h"
#include "resource/resource_ids.auto.h"
#include "system/passert.h"

static void prv_app_timer_callback(void *context) {
  dialog_pop(context);
}

void dialog_init(Dialog *dialog, const char *dialog_name) {
  PBL_ASSERTN(dialog);
  *dialog = (Dialog){};

  window_init(&dialog->window, dialog_name);
  window_set_background_color(&dialog->window, PBL_IF_COLOR_ELSE(GColorLightGray, GColorWhite));

  // initial values
  dialog->icon_anim_direction = DialogIconAnimationFromRight;
  dialog->destroy_on_pop = true;
  dialog->text_color = GColorBlack;
}

void dialog_pop(Dialog *dialog) {
  window_stack_remove(&dialog->window, DIALOG_IS_ANIMATED);
}

void dialog_push(Dialog *dialog, WindowStack *window_stack) {
  window_stack_push(window_stack, &dialog->window, DIALOG_IS_ANIMATED);
}

void app_dialog_push(Dialog *dialog) {
  dialog_push(dialog, app_state_get_window_stack());
}

// Loads the core dialog. Should be called from each dialog window's load callback.
void dialog_load(Dialog *dialog) {
  if (dialog->vibe_on_show) {
    vibes_short_pulse();
  }

  if (dialog->timeout != DIALOG_TIMEOUT_INFINITE) {
    dialog->timer = app_timer_register(dialog->timeout, prv_app_timer_callback, dialog);
  }

  // Calls the user-given load callback, if it exists.  If the user gave a non-null context,
  // the function will use that, otherwise it will default to use the default context of the
  // containing dialog.
  if (dialog->callbacks.load) {
    if (dialog->callback_context) {
      dialog->callbacks.load(dialog->callback_context);
    } else {
      dialog->callbacks.load(dialog);
    }
  }
}

// Unloads the core dialog. Should be called from each dialog window's unload callback.
void dialog_unload(Dialog *dialog) {
  app_timer_cancel(dialog->timer);

  if (dialog->show_status_layer) {
    status_bar_layer_deinit(&dialog->status_layer);
  }

  dialog_set_icon(dialog, INVALID_RESOURCE);
  text_layer_deinit(&dialog->text_layer);
  kino_layer_deinit(&dialog->icon_layer);

  if (dialog->buffer && dialog->is_buffer_owned) {
    applib_free(dialog->buffer);
  }

  // Calls the user-given unload callback, if it exists. If the user gave a non-null context,
  // the function will use that, otherwise it will default to use the default context of the
  // containing dialog.
  if (dialog->callbacks.unload) {
    if (dialog->callback_context) {
      dialog->callbacks.unload(dialog->callback_context);
    } else {
      dialog->callbacks.unload(dialog);
    }
  }
}

KinoReel *dialog_create_icon(Dialog *dialog) {
  return kino_reel_create_with_resource_system(SYSTEM_APP, dialog->icon_id);
}

bool dialog_init_icon_layer(Dialog *dialog, KinoReel *image,
                            GPoint icon_origin, bool animated) {
  if (!image) {
    return false;
  }

  const GRect icon_rect = (GRect) {
    .origin = icon_origin,
    .size = kino_reel_get_size(image)
  };

  KinoLayer *icon_layer = &dialog->icon_layer;
  kino_layer_init(icon_layer, &icon_rect);
  layer_set_clips(&icon_layer->layer, false);

  GRect from = icon_rect;
  // Animate from off screen
  GRect bounds;
  Layer *window_layer = window_get_root_layer(&dialog->window);
  layer_get_bounds(window_layer, &bounds);
  if (dialog->icon_anim_direction == DialogIconAnimationFromLeft) {
    from.origin.x = bounds.origin.x - icon_rect.size.w;
  } else if (dialog->icon_anim_direction == DialogIconAnimationFromRight) {
    from.origin.x = bounds.origin.x + bounds.size.w;
  }

  const int16_t ICON_TARGET_PT_X = icon_rect.size.w;
  const int16_t ICON_TARGET_PT_Y = (icon_rect.size.h / 2);

  KinoReel *reel = NULL;
  if (animated) {
    reel = kino_reel_scale_segmented_create(image, true, icon_rect);
    kino_reel_transform_set_from_frame(reel, from);
    kino_reel_transform_set_transform_duration(reel, 300);
    kino_reel_scale_segmented_set_deflate_effect(reel, 10);
    kino_reel_scale_segmented_set_delay_by_distance(
        reel, GPoint(ICON_TARGET_PT_X, ICON_TARGET_PT_Y));
  }

  if (!reel) {
    // Fall back to using the image reel as is which could be an animation without the scaling
    reel = image;
  }

  kino_layer_set_reel(icon_layer, reel, true);
  kino_layer_play(icon_layer);

  uint32_t icon_duration = kino_reel_get_duration(image);
  if (dialog->timeout != DIALOG_TIMEOUT_INFINITE && // Don't shorten infinite dialogs
      icon_duration != PLAY_DURATION_INFINITE && // Don't extend dialogs with infinite animations
      icon_duration > dialog->timeout) {
    // The finite image animation is longer, increase the finite dialog timeout
    dialog_set_timeout(dialog, icon_duration);
  }

  return true;
}

void dialog_add_status_bar_layer(Dialog *dialog, const GRect *status_layer_frame) {
  StatusBarLayer *status_layer = &dialog->status_layer;
  status_bar_layer_init(status_layer);
  layer_set_frame(&status_layer->layer, status_layer_frame);
  status_bar_layer_set_colors(status_layer, GColorClear, dialog->text_color);
  layer_add_child(&dialog->window.layer, &status_layer->layer);
}
