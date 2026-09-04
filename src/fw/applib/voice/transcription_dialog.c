/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "transcription_dialog.h"

#include "applib/applib_malloc.auto.h"
#include "applib/graphics/gtypes.h"
#include "applib/graphics/utf8.h"
#include "applib/ui/action_bar_layer.h"
#include "applib/ui/animation.h"
#include "applib/ui/scroll_layer.h"
#include "kernel/ui/kernel_ui.h"
#include "resource/resource_ids.auto.h"
#include "shell/system_theme.h"
#include "system/passert.h"

#define SCROLL_ANIMATION_DURATION  (300)
#define POP_WINDOW_DELAY (400)
#define CHARACTER_DELAY (20)
#define TEXT_OFFSET_VERTICAL  (6)

static void prv_show_next_character(TranscriptionDialog *transcription_dialog, int16_t to_idx) {
  // TODO: at the beginning of a word, check whether it's going to wrap when it's finished
  // type and break to the next line before it starts typing.

  Dialog *dialog = expandable_dialog_get_dialog((ExpandableDialog *)transcription_dialog);

  // Find the current index
  utf8_t *cursor = (utf8_t *)dialog->buffer;
  int16_t current_idx = 0;
  while (cursor < (utf8_t *)transcription_dialog->zero) {
    cursor = utf8_get_next(cursor);
    current_idx++;
  }
  PBL_ASSERTN(cursor == (utf8_t *)transcription_dialog->zero);
  PBL_ASSERTN(current_idx <= to_idx);

  // Restore the missing character, then get the start of the next codepoint.
  *transcription_dialog->zero = transcription_dialog->missing;
  while (current_idx++ < to_idx) {
    cursor = utf8_get_next(cursor);
  }

  char *next = (char *)cursor;
  if (next == dialog->buffer + transcription_dialog->buffer_len) {
    return;
  }

  // Move the zero terminator.
  transcription_dialog->missing = *next;
  *next = '\0';
  transcription_dialog->zero = next;
}

static void prv_set_char_index(void *subject, int16_t index) {
  TranscriptionDialog *transcription_dialog = subject;
  prv_show_next_character(transcription_dialog, index);

  Dialog *dialog = expandable_dialog_get_dialog((ExpandableDialog *)transcription_dialog);
  ScrollLayer *scroll_layer = &((ExpandableDialog *) transcription_dialog)->scroll_layer;

  TextLayer *text_layer = &dialog->text_layer;
  const GSize size = text_layer_get_content_size(graphics_context_get_current_context(),
      text_layer);
  const uint16_t font_height = fonts_get_font_height(text_layer->font);

  text_layer_set_size(text_layer, (GSize) { text_layer->layer.frame.size.w, size.h + font_height });

  const GSize scroll_size = scroll_layer_get_content_size(scroll_layer);
  const int16_t new_height = size.h + TEXT_OFFSET_VERTICAL;
  if (scroll_size.h != new_height) {
    const GRect *bounds = &scroll_layer_get_layer(scroll_layer)->bounds;
    GPoint offset = { .y = bounds->size.h - new_height };
#if PBL_ROUND
    // do paging on round display
    offset.y = ROUND_TO_MOD_CEIL(offset.y, scroll_layer->layer.frame.size.h);
#endif
    scroll_layer_set_content_size(scroll_layer,
                                  (GSize) { scroll_layer->layer.frame.size.w, new_height });
    scroll_layer_set_content_offset(scroll_layer, offset, true /* animated */);
    animation_set_duration(property_animation_get_animation(scroll_layer->animation),
        SCROLL_ANIMATION_DURATION);
  }

  layer_mark_dirty((Layer *)text_layer);
}

static void prv_start_text_animation(TranscriptionDialog *transcription_dialog) {
  static const PropertyAnimationImplementation animated_text_len = {
    .base = {
      .update = (AnimationUpdateImplementation) property_animation_update_int16,
    },
    .accessors = {
      .setter = { .int16 = prv_set_char_index }
    }
  };

  Dialog *dialog = expandable_dialog_get_dialog((ExpandableDialog *)transcription_dialog);

  // Count the number of codepoints in the message
  *transcription_dialog->zero = transcription_dialog->missing;

  int16_t count = 0;
  int16_t begin = 0;
  utf8_t *cursor = (utf8_t *)dialog->buffer;
  while ((cursor = utf8_get_next(cursor))) {
    count++;
    if (cursor == (utf8_t *)transcription_dialog->zero) {
      begin = count;
    }
  }

  transcription_dialog->animation = property_animation_create(&animated_text_len,
      transcription_dialog, NULL, NULL);
  if (!transcription_dialog->animation) {
    return;
  }
  property_animation_set_from_int16(transcription_dialog->animation, &begin);
  property_animation_set_to_int16(transcription_dialog->animation, &count);

  Animation *anim = property_animation_get_animation(transcription_dialog->animation);

  animation_set_duration(anim, (count - begin) * CHARACTER_DELAY);
  animation_set_curve(anim, AnimationCurveEaseInOut);

  // Text is shown if creating the property animation fails
  *transcription_dialog->zero = '\0';

  animation_set_curve(anim, AnimationCurveLinear);
  animation_schedule(anim);
}

static void prv_stop_text_animation(TranscriptionDialog *transcription_dialog) {
  Dialog *dialog = expandable_dialog_get_dialog((ExpandableDialog *)transcription_dialog);
  animation_unschedule(property_animation_get_animation(transcription_dialog->animation));
  *transcription_dialog->zero = transcription_dialog->missing;
  transcription_dialog->zero = dialog->buffer + transcription_dialog->buffer_len;
  transcription_dialog->missing = '\0';
  layer_mark_dirty((Layer *)&dialog->text_layer);
}

static void prv_transcription_dialog_unload(void *context) {
  TranscriptionDialog *transcription_dialog = context;
  app_timer_cancel(transcription_dialog->pop_timer);
  prv_stop_text_animation(transcription_dialog);
}

static void prv_transcription_dialog_load(void *context) {
  TranscriptionDialog *transcription_dialog = context;
  transcription_dialog->was_pushed = true;
  if (transcription_dialog->buffer_len > 0) {
    Dialog *dialog = expandable_dialog_get_dialog((ExpandableDialog *)transcription_dialog);
    transcription_dialog->zero = dialog->buffer;
    transcription_dialog->missing = dialog->buffer[0];
    prv_start_text_animation(context);
  }
}

static void prv_transcription_dialog_select_cb(void *context) {
  TranscriptionDialog *transcription_dialog = context;
  if (transcription_dialog->keep_alive_on_select) {
    action_bar_layer_clear_icon(&transcription_dialog->e_dialog.action_bar, BUTTON_ID_SELECT);
  } else {
    transcription_dialog_pop(transcription_dialog);
  }
}

static void prv_transcription_dialog_select_handler(ClickRecognizerRef recognizer, void *context) {
  TranscriptionDialog *transcription_dialog = context;
  if (transcription_dialog->select_pressed) {
    // We are waiting to pop the window, don't run the callback again
    return;
  }
  transcription_dialog->select_pressed = true;

  prv_stop_text_animation(transcription_dialog);

  if (transcription_dialog->callback) {
    if (transcription_dialog->callback_context) {
      transcription_dialog->callback(transcription_dialog->callback_context);
    } else {
      transcription_dialog->callback(transcription_dialog);
    }
  }

  transcription_dialog->pop_timer = app_timer_register(POP_WINDOW_DELAY,
      prv_transcription_dialog_select_cb, transcription_dialog);
}

void transcription_dialog_update_text(TranscriptionDialog *transcription_dialog,
                                      char *buffer, uint16_t buffer_len) {
  Dialog *dialog = expandable_dialog_get_dialog((ExpandableDialog *)transcription_dialog);

  transcription_dialog->buffer_len = buffer_len;
  dialog_set_text_buffer(dialog, buffer, false /* take_ownership */);

  if (transcription_dialog->was_pushed) {
    prv_stop_text_animation(transcription_dialog);
    prv_start_text_animation(transcription_dialog);
  }
}

void transcription_dialog_push(TranscriptionDialog *transcription_dialog,
                               WindowStack *window_stack) {
  PBL_ASSERTN(transcription_dialog);
  expandable_dialog_push((ExpandableDialog *)transcription_dialog, window_stack);
}

void app_transcription_dialog_push(TranscriptionDialog *transcription_dialog) {
  PBL_ASSERTN(transcription_dialog);
  app_expandable_dialog_push((ExpandableDialog *)transcription_dialog);
}

void transcription_dialog_pop(TranscriptionDialog *transcription_dialog) {
  PBL_ASSERTN(transcription_dialog);
  expandable_dialog_pop((ExpandableDialog *)transcription_dialog);
}

void transcription_dialog_set_callback(TranscriptionDialog *transcription_dialog,
                                        TranscriptionConfirmationCallback callback,
                                        void *callback_context) {
  PBL_ASSERTN(transcription_dialog);
  transcription_dialog->callback = callback;
  transcription_dialog->callback_context = callback_context;
}

void transcription_dialog_keep_alive_on_select(TranscriptionDialog *transcription_dialog,
                                               bool keep_alive_on_select) {
  PBL_ASSERTN(transcription_dialog);
  transcription_dialog->keep_alive_on_select = keep_alive_on_select;
}

TranscriptionDialog *transcription_dialog_create(void) {
  TranscriptionDialog *transcription_dialog = applib_type_malloc(TranscriptionDialog);
  if (!transcription_dialog) {
    return NULL;
  }

  transcription_dialog_init(transcription_dialog);
  return transcription_dialog;
}

void transcription_dialog_init(TranscriptionDialog *transcription_dialog) {
  *transcription_dialog = (TranscriptionDialog){};

  expandable_dialog_init((ExpandableDialog *)transcription_dialog, "Transcription Dialog");
  expandable_dialog_set_body_font((ExpandableDialog *)transcription_dialog,
                                  system_theme_get_font(TextStyleFont_Body));
  expandable_dialog_set_select_action((ExpandableDialog *)transcription_dialog,
      RESOURCE_ID_ACTION_BAR_ICON_CHECK, prv_transcription_dialog_select_handler);

  Dialog *dialog = expandable_dialog_get_dialog((ExpandableDialog *)transcription_dialog);
  dialog_set_callbacks(dialog, &(DialogCallbacks) {
        .unload = prv_transcription_dialog_unload,
        .load = prv_transcription_dialog_load
      }, transcription_dialog);
  dialog_show_status_bar_layer(dialog, true /* show status bar */);
  dialog_set_timeout(dialog, DIALOG_TIMEOUT_INFINITE);

  status_bar_layer_set_colors(&dialog->status_layer, GColorLightGray, GColorBlack);
}
