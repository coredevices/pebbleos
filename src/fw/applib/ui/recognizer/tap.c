/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "tap.h"

#include "recognizer.h"
#include "recognizer_impl.h"

#include "pbl/drivers/rtc.h"
#include "pbl/util/math.h"
#include "syscall/syscall.h"

#include <string.h>

// Maximum touchdown-to-liftoff duration for a press to count as a tap; a longer
// contact is treated as a hold, not a tap. Value from the reference PT2 touch-nav
// gesture spec.
#define TAP_MAX_DURATION_MS (300)

// Component-wise movement budget from the touchdown point. A finger that drifts
// further in either axis is a drag, not a tap. Value from the reference PT2
// touch-nav gesture spec; revisit once measured on production hardware.
#define TAP_MOVEMENT_THRESHOLD_PX (10)

struct TapRecognizerData {
  // Recognizer config
  struct {
    uint16_t taps_required;
    uint16_t fingers_required;
    GPoint movement_threshold;
  } config;

  // Gesture state
  struct {
    uint16_t taps_detected;
    uint16_t fingers_down;
    GPoint touch_down_point;    // Touchdown point, used for the movement check
    GPoint tap_point;           // Last PositionUpdate point; the reported tap coord
    RtcTicks touch_down_ticks;  // Touchdown time, used for the duration check
  } state;
};

static void prv_handle_touch_event(Recognizer *recognizer, const TouchEvent *touch_event);
static void prv_reset(Recognizer *recognizer);
static bool prv_cancel(Recognizer *recognizer);

static const RecognizerImpl s_tap_recognizer_impl = {
  .handle_touch_event = prv_handle_touch_event,
  .reset = prv_reset,
  .cancel = prv_cancel
};

static bool prv_moved_too_far(const TapRecognizerData *data, const TouchEvent *touch_event) {
  const int16_t dx = ABS(touch_event->x - data->state.touch_down_point.x);
  const int16_t dy = ABS(touch_event->y - data->state.touch_down_point.y);
  return (dx > data->config.movement_threshold.x) ||
         (dy > data->config.movement_threshold.y);
}

static uint32_t prv_touch_duration_ms(const TapRecognizerData *data) {
  const RtcTicks elapsed = sys_get_ticks() - data->state.touch_down_ticks;
  return (uint32_t)((elapsed * MS_PER_SECOND) / RTC_TICKS_HZ);
}

static void prv_handle_touch_event(Recognizer *recognizer, const TouchEvent *touch_event) {
  TapRecognizerData *data = recognizer_get_impl_data((Recognizer *)recognizer,
                                                     &s_tap_recognizer_impl);

  switch (touch_event->type) {
    case TouchEvent_Touchdown:
      // Record the contact point and time; the tap coord starts here and is
      // refined by later position updates.
      data->state.touch_down_point = GPoint(touch_event->x, touch_event->y);
      data->state.tap_point = data->state.touch_down_point;
      data->state.touch_down_ticks = sys_get_ticks();
      data->state.fingers_down = 1;
      break;

    case TouchEvent_PositionUpdate:
      // Track the latest coordinate; liftoff coordinates are ignored.
      data->state.tap_point = GPoint(touch_event->x, touch_event->y);
      if (prv_moved_too_far(data, touch_event)) {
        recognizer_transition_state(recognizer, RecognizerState_Failed);
      }
      break;

    case TouchEvent_Liftoff:
      if (prv_touch_duration_ms(data) <= TAP_MAX_DURATION_MS) {
        data->state.taps_detected++;
        recognizer_transition_state(recognizer, RecognizerState_Completed);
      } else {
        recognizer_transition_state(recognizer, RecognizerState_Failed);
      }
      break;
  }
}

static void prv_reset(Recognizer *recognizer) {
  TapRecognizerData *data = recognizer_get_impl_data((Recognizer *)recognizer,
                                                     &s_tap_recognizer_impl);
  memset(&data->state, 0, sizeof(data->state));
}

static bool prv_cancel(Recognizer *recognizer) {
  prv_reset(recognizer);
  return false;
}

Recognizer *tap_recognizer_create(RecognizerEventCb event_cb, void *user_data) {
  TapRecognizerData data = {
    .config = {
      .taps_required = 1,
      .fingers_required = 1,
      .movement_threshold = GPoint(TAP_MOVEMENT_THRESHOLD_PX, TAP_MOVEMENT_THRESHOLD_PX),
    },
  };

  return recognizer_create_with_data(&s_tap_recognizer_impl, &data, sizeof(data), event_cb,
                                     user_data);
}

Recognizer *tap_recognizer_init_static(void *storage, RecognizerEventCb event_cb, void *user_data) {
  _Static_assert(RECOGNIZER_INSTANCE_SIZE + sizeof(TapRecognizerData) <= TAP_RECOGNIZER_STATIC_SIZE,
                 "TAP_RECOGNIZER_STATIC_SIZE too small for a static tap recognizer");
  TapRecognizerData data = {
    .config = {
      .taps_required = 1,
      .fingers_required = 1,
      .movement_threshold = GPoint(TAP_MOVEMENT_THRESHOLD_PX, TAP_MOVEMENT_THRESHOLD_PX),
    },
  };

  return recognizer_init_static_with_data(storage, &s_tap_recognizer_impl, &data, sizeof(data),
                                          event_cb, user_data);
}

const TapRecognizerData *tap_recognizer_get_data(const Recognizer *recognizer) {
  return recognizer_get_impl_data((Recognizer *)recognizer, &s_tap_recognizer_impl);
}

GPoint tap_recognizer_get_tap_point(const Recognizer *recognizer) {
  const TapRecognizerData *data = recognizer_get_impl_data((Recognizer *)recognizer,
                                                           &s_tap_recognizer_impl);
  if (!data) {
    // SDK-reachable with a NULL or non-tap recognizer: reject, don't crash.
    return GPointZero;
  }
  return data->state.tap_point;
}

void tap_recognizer_set_num_taps_required(Recognizer *recognizer, int num_taps) {
  TapRecognizerData *data = recognizer_get_impl_data(recognizer, &s_tap_recognizer_impl);
  if (!data) {
    return;
  }
  data->config.taps_required = num_taps;
}
