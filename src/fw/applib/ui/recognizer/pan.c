/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "pan.h"

#include "recognizer.h"
#include "recognizer_impl.h"

#include "pbl/drivers/rtc.h"
#include "pbl/util/math.h"

#include <string.h>

// Component-wise movement budget from the touchdown point that the finger must
// exceed on the locked axis before the pan Starts. A smaller drift is treated as
// noise, not a pan. Value from the reference PT2 touch-nav gesture spec.
#define PAN_START_THRESHOLD_PX (10)

// Ratio by which the dominant axis of movement must exceed the other axis for the
// movement to count as unambiguous. Below this the movement is diagonal/ambiguous
// and the recognizer waits for more. Value from the reference PT2 touch-nav
// gesture spec.
#define PAN_AXIS_DOMINANCE (2)

// Number of most-recent position samples retained for the velocity estimate.
#define PAN_VELOCITY_SAMPLE_COUNT (3)

// Only samples no older than this (relative to the newest sample) contribute to
// the velocity estimate. Value from the reference PT2 touch-nav gesture spec.
#define PAN_VELOCITY_WINDOW_MS (100)

typedef struct PanVelocitySample {
  GPoint point;
  RtcTicks ticks;
} PanVelocitySample;

struct PanRecognizerData {
  // Recognizer config
  struct {
    PanAxis axis_lock;
    uint16_t start_threshold_px;
    uint16_t axis_dominance;
  } config;

  // Gesture state
  struct {
    GPoint touch_down_point;   // Touchdown point; total_delta is measured from here
    GPoint start_point;        // Point at which the pan Started; delta_since_start is measured here
    GPoint prev_point;         // Previous event point; delta_since_prev is measured from here
    GPoint last_point;         // Most recent position update point (liftoff coords are ignored)
    // Velocity sample ring buffer; sample_head indexes the newest, filled up to sample_count entries
    PanVelocitySample samples[PAN_VELOCITY_SAMPLE_COUNT];
    uint8_t sample_head;
    uint8_t sample_count;
  } state;
};

static void prv_handle_touch_event(Recognizer *recognizer, const TouchEvent *touch_event);
static void prv_reset(Recognizer *recognizer);
static bool prv_cancel(Recognizer *recognizer);

static const RecognizerImpl s_pan_recognizer_impl = {
  .handle_touch_event = prv_handle_touch_event,
  .reset = prv_reset,
  .cancel = prv_cancel
};

static uint32_t prv_ticks_to_ms(RtcTicks ticks) {
  return (uint32_t)((ticks * MS_PER_SECOND) / RTC_TICKS_HZ);
}

static void prv_record_sample(PanRecognizerData *data, GPoint point, RtcTicks ticks) {
  const uint8_t next = (data->state.sample_count == 0) ?
      0 : (uint8_t)((data->state.sample_head + 1) % PAN_VELOCITY_SAMPLE_COUNT);
  data->state.samples[next] = (PanVelocitySample) { .point = point, .ticks = ticks };
  data->state.sample_head = next;
  if (data->state.sample_count < PAN_VELOCITY_SAMPLE_COUNT) {
    data->state.sample_count++;
  }
}

// Compute component-wise velocity in px/s over the most-recent samples that are no older than
// PAN_VELOCITY_WINDOW_MS relative to the newest sample. Returns (0, 0) when the elapsed time across
// those samples is zero, so we never divide by zero.
static GPoint prv_compute_velocity(const PanRecognizerData *data) {
  if (data->state.sample_count < 2) {
    return GPointZero;
  }
  const uint8_t head = data->state.sample_head;
  const PanVelocitySample *newest = &data->state.samples[head];

  // Walk back from the newest sample to find the oldest sample still inside the window.
  const PanVelocitySample *oldest = newest;
  for (uint8_t i = 1; i < data->state.sample_count; i++) {
    const uint8_t idx = (uint8_t)((head + PAN_VELOCITY_SAMPLE_COUNT - i) % PAN_VELOCITY_SAMPLE_COUNT);
    const PanVelocitySample *candidate = &data->state.samples[idx];
    if (prv_ticks_to_ms(newest->ticks - candidate->ticks) > PAN_VELOCITY_WINDOW_MS) {
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

// Determine whether the movement is unambiguously along one axis, and if so, whether that axis is
// horizontal. Ambiguous (diagonal) movement returns is_dominant == false.
static bool prv_axis_dominance(const PanRecognizerData *data, GPoint total_delta,
                               bool *out_is_horizontal) {
  const int32_t adx = ABS(total_delta.x);
  const int32_t ady = ABS(total_delta.y);
  const int32_t factor = data->config.axis_dominance;
  if (adx > (ady * factor)) {
    *out_is_horizontal = true;
    return true;
  }
  if (ady > (adx * factor)) {
    *out_is_horizontal = false;
    return true;
  }
  return false;
}

static void prv_handle_touch_event(Recognizer *recognizer, const TouchEvent *touch_event) {
  PanRecognizerData *data = recognizer_get_impl_data((Recognizer *)recognizer,
                                                     &s_pan_recognizer_impl);

  switch (touch_event->type) {
    case TouchEvent_Touchdown: {
      const GPoint point = GPoint(touch_event->x, touch_event->y);
      const RtcTicks now = rtc_get_ticks();
      data->state.touch_down_point = point;
      data->state.start_point = point;
      data->state.prev_point = point;
      data->state.last_point = point;
      data->state.sample_count = 0;
      data->state.sample_head = 0;
      prv_record_sample(data, point, now);
      break;
    }

    case TouchEvent_PositionUpdate: {
      const GPoint point = GPoint(touch_event->x, touch_event->y);
      const RtcTicks now = rtc_get_ticks();
      prv_record_sample(data, point, now);
      // prev_point lags one event behind last_point so delta_since_prev is the delta between the
      // two most-recent events.
      data->state.prev_point = data->state.last_point;
      data->state.last_point = point;

      const GPoint total_delta = gpoint_sub(point, data->state.touch_down_point);

      const RecognizerState state = recognizer_get_state(recognizer);
      if (state == RecognizerState_Possible) {
        bool is_horizontal;
        // (0) Ambiguous movement: neither axis clearly dominates. Wait for more.
        if (!prv_axis_dominance(data, total_delta, &is_horizontal)) {
          break;
        }
        // (b) Threshold on the dominant axis must be crossed before we decide.
        const int32_t major = is_horizontal ? ABS(total_delta.x) : ABS(total_delta.y);
        if (major <= data->config.start_threshold_px) {
          break;
        }
        // (a) Dominant axis matches the locked axis -> Start; foreign axis -> Fail.
        const bool matches_lock = (is_horizontal && (data->config.axis_lock == PanAxis_Horizontal)) ||
                                  (!is_horizontal && (data->config.axis_lock == PanAxis_Vertical));
        if (matches_lock) {
          // Anchor delta_since_start at the current point so it is exactly (0, 0) when Started
          // fires; this prevents a visual jump for live scroll that consumes delta_since_start.
          data->state.start_point = point;
          recognizer_transition_state(recognizer, RecognizerState_Started);
        } else {
          recognizer_transition_state(recognizer, RecognizerState_Failed);
        }
      } else {
        // Already Started/Updated: emit an Updated event with the growing delta.
        recognizer_transition_state(recognizer, RecognizerState_Updated);
      }
      break;
    }

    case TouchEvent_Liftoff:
      // Liftoff coordinates are ignored (the driver reports finger-up at (0, 0)); the gesture end is
      // the last position update.
      if (recognizer_has_triggered(recognizer)) {
        recognizer_transition_state(recognizer, RecognizerState_Completed);
      } else {
        // Never crossed the threshold on the locked axis: not a pan.
        recognizer_transition_state(recognizer, RecognizerState_Failed);
      }
      break;
  }
}

static void prv_reset(Recognizer *recognizer) {
  PanRecognizerData *data = recognizer_get_impl_data((Recognizer *)recognizer,
                                                     &s_pan_recognizer_impl);
  memset(&data->state, 0, sizeof(data->state));
}

static bool prv_cancel(Recognizer *recognizer) {
  prv_reset(recognizer);
  return false;
}

Recognizer *pan_recognizer_create(RecognizerEventCb event_cb, void *user_data, PanAxis axis) {
  PanRecognizerData data = {
    .config = {
      .axis_lock = axis,
      .start_threshold_px = PAN_START_THRESHOLD_PX,
      .axis_dominance = PAN_AXIS_DOMINANCE,
    },
  };

  return recognizer_create_with_data(&s_pan_recognizer_impl, &data, sizeof(data), event_cb,
                                     user_data);
}

Recognizer *pan_recognizer_init_static(void *storage, RecognizerEventCb event_cb, void *user_data,
                                       PanAxis axis) {
  _Static_assert(RECOGNIZER_INSTANCE_SIZE + sizeof(PanRecognizerData) <= PAN_RECOGNIZER_STATIC_SIZE,
                 "PAN_RECOGNIZER_STATIC_SIZE too small for a static pan recognizer");
  PanRecognizerData data = {
    .config = {
      .axis_lock = axis,
      .start_threshold_px = PAN_START_THRESHOLD_PX,
      .axis_dominance = PAN_AXIS_DOMINANCE,
    },
  };

  return recognizer_init_static_with_data(storage, &s_pan_recognizer_impl, &data, sizeof(data),
                                          event_cb, user_data);
}

const PanRecognizerData *pan_recognizer_get_data(const Recognizer *recognizer) {
  return recognizer_get_impl_data((Recognizer *)recognizer, &s_pan_recognizer_impl);
}

GPoint pan_recognizer_get_total_delta(const Recognizer *recognizer) {
  const PanRecognizerData *data = recognizer_get_impl_data((Recognizer *)recognizer,
                                                           &s_pan_recognizer_impl);
  if (!data) {
    // SDK-reachable with a NULL or non-pan recognizer: reject, don't crash.
    return GPointZero;
  }
  return gpoint_sub(data->state.last_point, data->state.touch_down_point);
}

GPoint pan_recognizer_get_delta_since_start(const Recognizer *recognizer) {
  const PanRecognizerData *data = recognizer_get_impl_data((Recognizer *)recognizer,
                                                           &s_pan_recognizer_impl);
  if (!data) {
    return GPointZero;
  }
  return gpoint_sub(data->state.last_point, data->state.start_point);
}

GPoint pan_recognizer_get_delta_since_prev(const Recognizer *recognizer) {
  const PanRecognizerData *data = recognizer_get_impl_data((Recognizer *)recognizer,
                                                           &s_pan_recognizer_impl);
  if (!data) {
    return GPointZero;
  }
  return gpoint_sub(data->state.last_point, data->state.prev_point);
}

GPoint pan_recognizer_get_velocity(const Recognizer *recognizer) {
  const PanRecognizerData *data = recognizer_get_impl_data((Recognizer *)recognizer,
                                                           &s_pan_recognizer_impl);
  if (!data) {
    return GPointZero;
  }
  return prv_compute_velocity(data);
}
