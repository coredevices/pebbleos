/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "click.h"
#include "click_internal.h"

#include "window_stack_private.h"

#include "process_state/app_state/app_state.h"
#include "pbl/util/size.h"

#include <stddef.h>

//! The time that the user has to hold the button before repetition kicks in.
static const uint32_t CLICK_REPETITION_DELAY_MS = 400;
//! Default minimum number of multi-clicks before the multi_click.handler gets fired
static const uint8_t MULTI_CLICK_DEFAULT_MIN = 2;
//! Default timeout after which looking for follow up clicks will be stopped
static const uint32_t MULTI_CLICK_DEFAULT_TIMEOUT_MS = 300;
//! Default delay before long click is fired
static const uint32_t LONG_CLICK_DEFAULT_DELAY_MS = 400;

typedef enum {
  ClickHandlerOffsetSingle = offsetof(ClickConfig, click.handler),
  ClickHandlerOffsetMulti = offsetof(ClickConfig, multi_click.handler),
  ClickHandlerOffsetLong = offsetof(ClickConfig, long_click.handler),
  ClickHandlerOffsetLongRelease = offsetof(ClickConfig, long_click.release_handler),
  ClickHandlerOffsetRawUp = offsetof(ClickConfig, raw.up_handler),
  ClickHandlerOffsetRawDown = offsetof(ClickConfig, raw.down_handler),
} ClickHandlerOffset;

static ClickHandler prv_get_handler(ClickRecognizer *recognizer, ClickHandlerOffset offset) {
  return *((ClickHandler*)(((uint8_t*)&recognizer->config) + offset));
}

static void prv_cancel_timer(AppTimer **timer) {
  if (*timer) {
    app_timer_cancel(*timer);
    *timer = NULL;
  }
}

static void prv_click_reset(ClickRecognizerRef recognizer_ref) {
  ClickRecognizer *recognizer = (ClickRecognizer *)recognizer_ref;
  recognizer->number_of_clicks_counted = 0;
  recognizer->is_button_down = false;
  recognizer->is_repeating = false;

  prv_cancel_timer(&recognizer->hold_timer);
  prv_cancel_timer(&recognizer->multi_click_timer);
}

static bool prv_dispatch_event(ClickRecognizer *recognizer, ClickHandlerOffset handler_offset,
                               bool needs_reset) {
  if (recognizer) {
    ClickHandler handler = prv_get_handler(recognizer, handler_offset);
    if (handler) {
      void *context;
      if ((handler_offset == ClickHandlerOffsetRawUp || handler_offset == ClickHandlerOffsetRawDown) &&
          recognizer->config.raw.context != NULL) {
        // The context for raw click events is overridable:
        context = recognizer->config.raw.context;
      } else {
        context = recognizer->config.context;
      }

      handler(recognizer, context);
    }

    if (needs_reset) {
      prv_click_reset(recognizer);
    }

    return true;
  } else {
    return false;
  }
}

inline static bool prv_is_hold_to_repeat_enabled(ClickRecognizer *recognizer) {
  return (recognizer->config.click.repeat_interval_ms >= 30);
}

inline static bool prv_is_multi_click_enabled(ClickRecognizer *recognizer) {
  return (recognizer->config.multi_click.handler != NULL);
}

inline static bool prv_is_long_click_enabled(ClickRecognizer *recognizer) {
  return (recognizer->config.long_click.handler != NULL || recognizer->config.long_click.release_handler != NULL);
}

static void prv_auto_repeat_single_click(ClickRecognizer *recognizer) {
  if (!recognizer->is_button_down) {
    // If this button isn't being held down anymore, don't re-register the timer.
    return;
  }
  ++(recognizer->number_of_clicks_counted);
  // Start the repetition timer:
  // Note: We're not using the timer_register_repeating() here, so we have the possibility
  //       of changing the interval in the handler.
  recognizer->hold_timer = app_timer_register(recognizer->config.click.repeat_interval_ms,
      (AppTimerCallback)prv_auto_repeat_single_click, recognizer);
  recognizer->is_repeating = true;

  // Fire right once:
  const bool needs_reset = false;
  prv_dispatch_event(recognizer, ClickHandlerOffsetSingle, needs_reset);
}

static void prv_repetition_delay_callback(void *data) {
  ClickRecognizer *recognizer = (ClickRecognizer *) data;

  // User has been holding the button down for more than the repetition delay.
  prv_auto_repeat_single_click(recognizer);
}

static uint8_t prv_multi_click_get_min(ClickRecognizer *recognizer) {
  if (false == prv_is_multi_click_enabled(recognizer)) {
    return 0;
  }
  if (recognizer->config.multi_click.min == 0) {
    return MULTI_CLICK_DEFAULT_MIN;
  }
  return recognizer->config.multi_click.min;
}

static uint8_t prv_multi_click_get_max(ClickRecognizer *recognizer) {
  if (false == prv_is_multi_click_enabled(recognizer)) {
    return 0;
  }
  if (recognizer->config.multi_click.max == 0) {
    return prv_multi_click_get_min(recognizer);
  }
  return recognizer->config.multi_click.max;
}

static uint32_t prv_multi_click_get_timeout(ClickRecognizer *recognizer) {
  if (false == prv_is_multi_click_enabled(recognizer)) {
    return 0;
  }
  if (recognizer->config.multi_click.timeout == 0) {
    return MULTI_CLICK_DEFAULT_TIMEOUT_MS;
  }
  return recognizer->config.multi_click.timeout;
}

static uint32_t prv_long_click_get_delay(ClickRecognizer *recognizer) {
  if (false == prv_is_long_click_enabled(recognizer)) {
    return 0;
  }
  if (recognizer->config.long_click.delay_ms == 0) {
    return LONG_CLICK_DEFAULT_DELAY_MS;
  }
  return recognizer->config.long_click.delay_ms;
}

inline static bool prv_can_more_clicks_follow(ClickRecognizer *recognizer) {
  if (recognizer->number_of_clicks_counted >= prv_multi_click_get_max(recognizer)) {
    return false;
  }
  return true;
}

uint8_t click_number_of_clicks_counted(ClickRecognizerRef recognizer_ref) {
  ClickRecognizer *recognizer = (ClickRecognizer*)recognizer_ref;
  return recognizer->number_of_clicks_counted;
}

ButtonId click_recognizer_get_button_id(ClickRecognizerRef recognizer_ref) {
  ClickRecognizer *recognizer = (ClickRecognizer*)recognizer_ref;
  return recognizer->button;
}

bool click_recognizer_is_repeating(ClickRecognizerRef recognizer_ref) {
  ClickRecognizer *recognizer = (ClickRecognizer*)recognizer_ref;
  return recognizer->is_repeating;
}

bool click_recognizer_is_held_down(ClickRecognizerRef recognizer_ref) {
  return (((ClickRecognizer *)recognizer_ref)->is_button_down);
}

ClickConfig *click_recognizer_get_config(ClickRecognizerRef recognizer_ref) {
  ClickRecognizer *recognizer = (ClickRecognizer*)recognizer_ref;
  return &recognizer->config;
}

static void prv_long_click_callback(void *data) {
  ClickRecognizer *recognizer = (ClickRecognizer *) data;

  recognizer->hold_timer = NULL;
  const bool needs_reset = false;
  prv_dispatch_event(recognizer, ClickHandlerOffsetLong, needs_reset);
}

//! Called at the end of a click pattern, either on the button up, or after a multi-click timeout:
static void prv_click_pattern_done(ClickRecognizer *recognizer) {
  // In case multi_click is also configured, if there was only one click, regard it as
  // a "single click" after the multi-click timeout passed and this callback is called:
  if (recognizer->number_of_clicks_counted >= 1 && recognizer->is_repeating == false) {
    int clicks_over = recognizer->number_of_clicks_counted;
    for(int i = 0; i < clicks_over; i++) {
      prv_dispatch_event(recognizer, ClickHandlerOffsetSingle, false);
    }
  }
  prv_click_reset(recognizer);
}

static void prv_multi_click_timeout_callback(void *data) {
  ClickRecognizer *recognizer = (ClickRecognizer *) data;

  recognizer->multi_click_timer = NULL;
  if (recognizer->config.multi_click.last_click_only &&
      (recognizer->number_of_clicks_counted >= prv_multi_click_get_min(recognizer)) &&
      (recognizer->number_of_clicks_counted <= prv_multi_click_get_max(recognizer))) {
    const bool needs_reset = true;
    prv_dispatch_event(recognizer, ClickHandlerOffsetMulti, needs_reset);
  } else {
    prv_click_pattern_done(recognizer);
  }
}

void command_put_button_event(const char* button_index, const char* click_type) {
  int button = atoi(button_index);
  const bool needs_reset = false;
  ClickHandlerOffset offset;

  if ((button < 0  || button > NUM_BUTTONS)) {
    return;
  }

  switch(*click_type) {
    case 's':
      offset = ClickHandlerOffsetSingle;
      break;
    case 'm':
      offset = ClickHandlerOffsetMulti;
      break;
    case 'l':
      offset = ClickHandlerOffsetLong;
      break;
    case 'r':
      offset = ClickHandlerOffsetLongRelease;
      break;
    case 'u':
      offset = ClickHandlerOffsetRawUp;
      break;
    case 'd':
      offset = ClickHandlerOffsetRawDown;
      break;
    default:
      return;
  }

  prv_dispatch_event(&(app_state_get_click_manager()->recognizers[button]), offset, needs_reset);
}

void click_recognizer_handle_button_down(ClickRecognizer *recognizer) {
  recognizer->is_button_down = true;

  prv_cancel_timer(&recognizer->multi_click_timer);

  const bool needs_reset = false;
  prv_dispatch_event(recognizer, ClickHandlerOffsetRawDown, needs_reset);

  if (prv_is_long_click_enabled(recognizer)) {
    const uint32_t long_click_delay = prv_long_click_get_delay(recognizer);
    recognizer->hold_timer = app_timer_register(
        long_click_delay, prv_long_click_callback, recognizer);
  } else {
    const bool local_is_hold_to_repeat_enabled = prv_is_hold_to_repeat_enabled(recognizer);
    if (local_is_hold_to_repeat_enabled) {
      // If there's a repeat interval configured, start the repetition delay timer:
      recognizer->hold_timer = app_timer_register(
          CLICK_REPETITION_DELAY_MS, prv_repetition_delay_callback, recognizer);
    }
    if (false == prv_is_multi_click_enabled(recognizer)) {
      // No long click nor multi click, fire handler immediately on button down:
      ++(recognizer->number_of_clicks_counted);
      const bool needs_reset = (false == local_is_hold_to_repeat_enabled);
      prv_dispatch_event(recognizer, ClickHandlerOffsetSingle, needs_reset);
    }
  }
}

void click_recognizer_handle_button_up(ClickRecognizer *recognizer) {
  const bool needs_reset = false;
  prv_dispatch_event(recognizer, ClickHandlerOffsetRawUp, needs_reset);

  if (recognizer->is_button_down == false) {
    // Ignore this button up event. Most likely, the recognizer has been
    // reset while the button was still pressed down.
    return;
  }
  recognizer->is_button_down = false;

  const bool local_is_long_click_enabled = prv_is_long_click_enabled(recognizer);
  const bool local_is_multi_click_enabled = prv_is_multi_click_enabled(recognizer);
  //const bool local_is_hold_to_repeat_enabled = is_hold_to_repeat_enabled(recognizer);

  if (false == local_is_long_click_enabled &&
      false == local_is_multi_click_enabled) {
    // Handler already fired in button down.
    prv_click_reset(recognizer);
    return;
  }

  ++(recognizer->number_of_clicks_counted);

  const bool has_long_click_been_fired = (local_is_long_click_enabled && recognizer->hold_timer == NULL);
  if (has_long_click_been_fired) {
    const bool needs_reset = true;
    prv_dispatch_event(recognizer, ClickHandlerOffsetLongRelease, needs_reset);
    return;
  }

  prv_cancel_timer(&recognizer->hold_timer);

  if (local_is_multi_click_enabled && false == recognizer->is_repeating) {
    const bool local_can_more_clicks_follow = prv_can_more_clicks_follow(recognizer);
    bool should_fire_multi_click_handler = ((recognizer->config.multi_click.last_click_only && local_can_more_clicks_follow) == false);
    bool reset_using_event = false;

    if (should_fire_multi_click_handler) {
      if ((recognizer->number_of_clicks_counted >= prv_multi_click_get_min(recognizer)) &&
          (recognizer->number_of_clicks_counted <= prv_multi_click_get_max(recognizer))) {
        reset_using_event = (false == local_can_more_clicks_follow);
        prv_dispatch_event(recognizer, ClickHandlerOffsetMulti, reset_using_event);
      }
    }

    if (prv_can_more_clicks_follow(recognizer)) {
      const uint32_t timeout = prv_multi_click_get_timeout(recognizer);
      recognizer->multi_click_timer = app_timer_register(
          timeout, prv_multi_click_timeout_callback, recognizer);
      return;
    } else {
      if (reset_using_event) {
        return;
      }
    }
    // fall-through if no more clicks can follow,
    // and we're not resetting using a click event that has been put.
  }

  prv_click_pattern_done(recognizer);
}

void click_manager_init(ClickManager* click_manager) {
  for (unsigned int button_id = 0;
       button_id < ARRAY_LENGTH(click_manager->recognizers); ++button_id) {
    ClickRecognizer *recognizer = &click_manager->recognizers[button_id];
    recognizer->button = button_id;
    prv_click_reset(recognizer);
  }
}

void click_manager_clear(ClickManager* click_manager) {
  for (unsigned int button_id = 0;
       button_id < ARRAY_LENGTH(click_manager->recognizers); ++button_id) {
    prv_click_reset(&click_manager->recognizers[button_id]);
    click_manager->recognizers[button_id].config = (ClickConfig){};
  }
}

static void prv_fire_long_release_if_held(ClickRecognizer *recognizer) {
  if (recognizer->is_button_down &&
      recognizer->hold_timer == NULL &&
      recognizer->config.long_click.release_handler != NULL) {
    prv_dispatch_event(recognizer, ClickHandlerOffsetLongRelease, false /* needs_reset */);
  }
}

void click_recognizer_reset(ClickRecognizer *recognizer) {
  prv_fire_long_release_if_held(recognizer);
  prv_click_reset(recognizer);
}

void click_manager_reset(ClickManager* click_manager) {
  for (unsigned int button_id = 0; button_id < NUM_BUTTONS; button_id++) {
    prv_fire_long_release_if_held(&click_manager->recognizers[button_id]);
    prv_click_reset(&click_manager->recognizers[button_id]);
  }
}

