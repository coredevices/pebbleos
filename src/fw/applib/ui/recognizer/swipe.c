/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "swipe.h"

#include "recognizer.h"
#include "recognizer_impl.h"

#include "pbl/drivers/rtc.h"
#include "pbl/util/math.h"
#include "syscall/syscall.h"

#include <string.h>

// Minimum travel (component-wise, on the major axis) from the touchdown point for a path to count
// as a swipe. A shorter flick is treated as a tap or noise. Value from the reference PT2 touch-nav
// gesture spec.
#define SWIPE_MIN_LENGTH_PX (30)

// Once the major-axis travel exceeds this, the path is committed enough that we start enforcing
// straightness: any further wandering on the minor axis fails the swipe early. The drag threshold
// value comes from the reference PT2 touch-nav gesture spec.
#define SWIPE_STRAIGHTNESS_MIN_PX (10)

// Maximum touchdown-to-liftoff duration for a swipe; a slower drag is a pan, not a flick. Value
// from the reference PT2 touch-nav gesture spec.
#define SWIPE_MAX_DURATION_MS (300)

// Number of most-recent position samples retained for the velocity estimate.
#define SWIPE_VELOCITY_SAMPLE_COUNT (3)

// Only samples no older than this (relative to the newest sample) contribute to the velocity
// estimate. Value from the reference PT2 touch-nav gesture spec.
#define SWIPE_VELOCITY_WINDOW_MS (100)

typedef struct SwipeVelocitySample {
  GPoint point;
  RtcTicks ticks;
} SwipeVelocitySample;

struct SwipeRecognizerData {
  // Recognizer config
  struct {
    uint8_t direction_mask;
  } config;

  // Gesture state
  struct {
    GPoint touch_down_point;   // Touchdown point; the path is measured from here
    GPoint last_point;         // Most recent position update point (liftoff coords are ignored)
    RtcTicks touch_down_ticks; // Touchdown time, used for the duration check
    SwipeDirection direction;  // Recognized direction, valid once Completed
    // Velocity sample ring buffer; sample_head indexes the newest, filled up to sample_count entries
    SwipeVelocitySample samples[SWIPE_VELOCITY_SAMPLE_COUNT];
    uint8_t sample_head;
    uint8_t sample_count;
  } state;
};

static void prv_handle_touch_event(Recognizer *recognizer, const TouchEvent *touch_event);
static void prv_reset(Recognizer *recognizer);
static bool prv_cancel(Recognizer *recognizer);

static const RecognizerImpl s_swipe_recognizer_impl = {
  .handle_touch_event = prv_handle_touch_event,
  .reset = prv_reset,
  .cancel = prv_cancel
};

static uint32_t prv_ticks_to_ms(RtcTicks ticks) {
  return (uint32_t)((ticks * MS_PER_SECOND) / RTC_TICKS_HZ);
}

static uint32_t prv_touch_duration_ms(const SwipeRecognizerData *data) {
  return prv_ticks_to_ms(sys_get_ticks() - data->state.touch_down_ticks);
}

static void prv_record_sample(SwipeRecognizerData *data, GPoint point, RtcTicks ticks) {
  const uint8_t next = (data->state.sample_count == 0) ?
      0 : (uint8_t)((data->state.sample_head + 1) % SWIPE_VELOCITY_SAMPLE_COUNT);
  data->state.samples[next] = (SwipeVelocitySample) { .point = point, .ticks = ticks };
  data->state.sample_head = next;
  if (data->state.sample_count < SWIPE_VELOCITY_SAMPLE_COUNT) {
    data->state.sample_count++;
  }
}

// Compute component-wise velocity in px/s over the most-recent samples that are no older than
// SWIPE_VELOCITY_WINDOW_MS relative to the newest sample. Returns (0, 0) when the elapsed time
// across those samples is zero, so we never divide by zero.
static GPoint prv_compute_velocity(const SwipeRecognizerData *data) {
  if (data->state.sample_count < 2) {
    return GPointZero;
  }
  const uint8_t head = data->state.sample_head;
  const SwipeVelocitySample *newest = &data->state.samples[head];

  const SwipeVelocitySample *oldest = newest;
  for (uint8_t i = 1; i < data->state.sample_count; i++) {
    const uint8_t idx =
        (uint8_t)((head + SWIPE_VELOCITY_SAMPLE_COUNT - i) % SWIPE_VELOCITY_SAMPLE_COUNT);
    const SwipeVelocitySample *candidate = &data->state.samples[idx];
    if (prv_ticks_to_ms(newest->ticks - candidate->ticks) > SWIPE_VELOCITY_WINDOW_MS) {
      break;
    }
    oldest = candidate;
  }

  const uint32_t dt_ms = prv_ticks_to_ms(newest->ticks - oldest->ticks);
  if (dt_ms == 0) {
    return GPointZero;
  }
  const int32_t vx = ((int32_t)(newest->point.x - oldest->point.x) * MS_PER_SECOND) / (int32_t)dt_ms;
  const int32_t vy = ((int32_t)(newest->point.y - oldest->point.y) * MS_PER_SECOND) / (int32_t)dt_ms;
  return GPoint((int16_t)vx, (int16_t)vy);
}

// Resolve the swipe direction from the total delta, using the dominant axis. Screen y grows
// downward, so a positive y delta is downward.
static SwipeDirection prv_direction_from_delta(GPoint delta) {
  const int32_t adx = ABS(delta.x);
  const int32_t ady = ABS(delta.y);
  if (adx >= ady) {
    return (delta.x >= 0) ? SwipeDirection_Right : SwipeDirection_Left;
  }
  return (delta.y >= 0) ? SwipeDirection_Down : SwipeDirection_Up;
}

static void prv_handle_touch_event(Recognizer *recognizer, const TouchEvent *touch_event) {
  SwipeRecognizerData *data = recognizer_get_impl_data((Recognizer *)recognizer,
                                                       &s_swipe_recognizer_impl);

  switch (touch_event->type) {
    case TouchEvent_Touchdown: {
      const GPoint point = GPoint(touch_event->x, touch_event->y);
      const RtcTicks now = sys_get_ticks();
      data->state.touch_down_point = point;
      data->state.last_point = point;
      data->state.touch_down_ticks = now;
      data->state.direction = SwipeDirection_None;
      data->state.sample_count = 0;
      data->state.sample_head = 0;
      prv_record_sample(data, point, now);
      break;
    }

    case TouchEvent_PositionUpdate: {
      const GPoint point = GPoint(touch_event->x, touch_event->y);
      prv_record_sample(data, point, sys_get_ticks());
      data->state.last_point = point;

      const GPoint total_delta = gpoint_sub(point, data->state.touch_down_point);
      const int32_t adx = ABS(total_delta.x);
      const int32_t ady = ABS(total_delta.y);
      const int32_t major = MAX(adx, ady);
      const int32_t minor = MIN(adx, ady);

      // Too crooked: once the path is committed (major axis past the drag threshold), the minor-axis
      // projection must stay within half the major axis, otherwise this is not a straight swipe.
      if ((major > SWIPE_STRAIGHTNESS_MIN_PX) && ((minor * 2) > major)) {
        recognizer_transition_state(recognizer, RecognizerState_Failed);
        break;
      }
      // Too slow: a swipe is a quick flick; a longer contact is a pan.
      if (prv_touch_duration_ms(data) > SWIPE_MAX_DURATION_MS) {
        recognizer_transition_state(recognizer, RecognizerState_Failed);
        break;
      }
      break;
    }

    case TouchEvent_Liftoff: {
      // Liftoff coordinates are ignored (the driver reports finger-up at (0, 0)); the gesture end is
      // the last position update.
      const GPoint total_delta = gpoint_sub(data->state.last_point, data->state.touch_down_point);
      const int32_t adx = ABS(total_delta.x);
      const int32_t ady = ABS(total_delta.y);
      const int32_t major = MAX(adx, ady);
      const int32_t minor = MIN(adx, ady);

      const bool long_enough = (major >= SWIPE_MIN_LENGTH_PX);
      const bool fast_enough = (prv_touch_duration_ms(data) <= SWIPE_MAX_DURATION_MS);
      // Clear major axis: the minor-axis projection is within half the major axis.
      const bool has_clear_major = (major > 0) && ((minor * 2) <= major);

      const SwipeDirection direction = prv_direction_from_delta(total_delta);
      const bool direction_allowed = (data->config.direction_mask & direction) != 0;

      if (long_enough && fast_enough && has_clear_major && direction_allowed) {
        data->state.direction = direction;
        recognizer_transition_state(recognizer, RecognizerState_Completed);
      } else {
        recognizer_transition_state(recognizer, RecognizerState_Failed);
      }
      break;
    }
  }
}

static void prv_reset(Recognizer *recognizer) {
  SwipeRecognizerData *data = recognizer_get_impl_data((Recognizer *)recognizer,
                                                       &s_swipe_recognizer_impl);
  memset(&data->state, 0, sizeof(data->state));
}

static bool prv_cancel(Recognizer *recognizer) {
  prv_reset(recognizer);
  return false;
}

Recognizer *swipe_recognizer_create(RecognizerEventCb event_cb, void *user_data,
                                    uint8_t direction_mask) {
  SwipeRecognizerData data = {
    .config = {
      .direction_mask = direction_mask,
    },
  };

  return recognizer_create_with_data(&s_swipe_recognizer_impl, &data, sizeof(data), event_cb,
                                     user_data);
}

Recognizer *swipe_recognizer_init_static(void *storage, RecognizerEventCb event_cb, void *user_data,
                                         uint8_t direction_mask) {
  _Static_assert(
      RECOGNIZER_INSTANCE_SIZE + sizeof(SwipeRecognizerData) <= SWIPE_RECOGNIZER_STATIC_SIZE,
      "SWIPE_RECOGNIZER_STATIC_SIZE too small for a static swipe recognizer");
  SwipeRecognizerData data = {
    .config = {
      .direction_mask = direction_mask,
    },
  };

  return recognizer_init_static_with_data(storage, &s_swipe_recognizer_impl, &data, sizeof(data),
                                          event_cb, user_data);
}

const SwipeRecognizerData *swipe_recognizer_get_data(const Recognizer *recognizer) {
  return recognizer_get_impl_data((Recognizer *)recognizer, &s_swipe_recognizer_impl);
}

SwipeDirection swipe_recognizer_get_direction(const Recognizer *recognizer) {
  const SwipeRecognizerData *data = recognizer_get_impl_data((Recognizer *)recognizer,
                                                             &s_swipe_recognizer_impl);
  if (!data) {
    // SDK-reachable with a NULL or non-swipe recognizer: reject, don't crash.
    return SwipeDirection_None;
  }
  return data->state.direction;
}

GPoint swipe_recognizer_get_velocity(const Recognizer *recognizer) {
  const SwipeRecognizerData *data = recognizer_get_impl_data((Recognizer *)recognizer,
                                                             &s_swipe_recognizer_impl);
  if (!data) {
    return GPointZero;
  }
  return prv_compute_velocity(data);
}
