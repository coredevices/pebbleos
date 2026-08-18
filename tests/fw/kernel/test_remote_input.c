/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "kernel/remote_input.h"

#include "applib/ui/recognizer/swipe.h"
#include <pbl/drivers/display/display.h>
#include "kernel/events.h"
#include "pbl/services/touch/touch.h"
#include "pbl/util/math.h"
#include "pbl/util/size.h"

#include <string.h>

#include "clar.h"

// Stubs
///////////////////////////////////////////////////////////////////////////////
#include "stubs_logging.h"
#include "stubs_mutex.h"
#include "stubs_passert.h"

#include "fake_pbl_malloc.h"
#include "fake_new_timer.h"

#define MAX_RECORDED 32

static PebbleEvent s_events[MAX_RECORDED];
static int s_event_count;

void event_put(PebbleEvent *event) {
  cl_assert(s_event_count < MAX_RECORDED);
  s_events[s_event_count++] = *event;
}

typedef struct RecordedTouch {
  TouchState state;
  int16_t x;
  int16_t y;
} RecordedTouch;

static RecordedTouch s_touches[MAX_RECORDED];
static int s_touch_count;
static bool s_injection_available;
//! Whether a sample is accepted once the gesture is under way, as opposed to whether a new one may
//! start; the touch service can withdraw the sensor mid-path.
static bool s_injection_accepts;

bool touch_handle_injected_update(TouchInjectPhase phase, int16_t x, int16_t y) {
  if (!s_injection_accepts) {
    return false;
  }
  cl_assert(s_touch_count < MAX_RECORDED);
  const TouchState state =
      (phase == TouchInjectPhase_End) ? TouchState_FingerUp : TouchState_FingerDown;
  s_touches[s_touch_count++] = (RecordedTouch){.state = state, .x = x, .y = y};
  return true;
}

bool touch_injection_is_available(void) {
  return s_injection_available;
}

static uint8_t s_ack[8];
static int s_ack_length;
static int s_ack_count;

bool comm_session_send_data(CommSession *session, uint16_t endpoint_id, const uint8_t *data,
                           size_t length, uint32_t timeout_ms) {
  cl_assert(length <= sizeof(s_ack));
  memcpy(s_ack, data, length);
  s_ack_length = length;
  s_ack_count++;
  return true;
}

// Helpers
///////////////////////////////////////////////////////////////////////////////

// The service creates exactly one timer, and the fake hands out ids from 1.
#define INPUT_TIMER_ID 1

//! Run the injected sequence to completion, so the test sees what the rest of the system would.
static void prv_run_sequence(void) {
  for (int i = 0; i < MAX_RECORDED; i++) {
    if (!stub_new_timer_is_scheduled(INPUT_TIMER_ID)) {
      return;
    }
    stub_new_timer_fire(INPUT_TIMER_ID);
  }
  cl_fail("injected sequence never finished");
}

//! Injected coordinates are the ones the UI observes; left-hand mode is the touch service's
//! business (see test_touch), so these read the samples straight back.
static int16_t prv_observed_y(int i) {
  return s_touches[i].y;
}

static int16_t prv_observed_x(int i) {
  return s_touches[i].x;
}

// Tests
///////////////////////////////////////////////////////////////////////////////

void test_remote_input__initialize(void) {
  remote_input_init();
  s_event_count = 0;
  s_touch_count = 0;
  s_ack_count = 0;
  s_ack_length = 0;
  s_injection_available = true;
  s_injection_accepts = true;
}

void test_remote_input__cleanup(void) {
  prv_run_sequence();
  // Held buttons are module state that would otherwise leak into the next test.
  remote_input_button_set(0);
}

void test_remote_input__button_press_emits_down_then_up(void) {
  cl_assert_equal_i(RemoteInputResult_Ok, remote_input_button_press(BUTTON_ID_SELECT, 1, 50, 0));
  prv_run_sequence();

  cl_assert_equal_i(2, s_event_count);
  cl_assert_equal_i(PEBBLE_BUTTON_DOWN_EVENT, s_events[0].type);
  cl_assert_equal_i(BUTTON_ID_SELECT, s_events[0].button.button_id);
  cl_assert_equal_i(PEBBLE_BUTTON_UP_EVENT, s_events[1].type);
  cl_assert_equal_i(BUTTON_ID_SELECT, s_events[1].button.button_id);
}

void test_remote_input__button_press_repeats(void) {
  cl_assert_equal_i(RemoteInputResult_Ok, remote_input_button_press(BUTTON_ID_UP, 3, 20, 10));
  prv_run_sequence();

  cl_assert_equal_i(6, s_event_count);
  for (int i = 0; i < s_event_count; i++) {
    cl_assert_equal_i(BUTTON_ID_UP, s_events[i].button.button_id);
    cl_assert_equal_i((i % 2 == 0) ? PEBBLE_BUTTON_DOWN_EVENT : PEBBLE_BUTTON_UP_EVENT,
                      s_events[i].type);
  }
}

void test_remote_input__button_hold_time_separates_down_from_up(void) {
  cl_assert_equal_i(RemoteInputResult_Ok, remote_input_button_press(BUTTON_ID_BACK, 1, 700, 0));

  // First callback presses the button, and must schedule the release a full hold time later,
  // otherwise a "long press" would be indistinguishable from a short one.
  stub_new_timer_fire(INPUT_TIMER_ID);
  cl_assert_equal_i(PEBBLE_BUTTON_DOWN_EVENT, s_events[0].type);
  cl_assert_equal_i(700, stub_new_timer_timeout(INPUT_TIMER_ID));
}

void test_remote_input__zero_presses_leaves_no_button_held(void) {
  cl_assert_equal_i(RemoteInputResult_Ok, remote_input_button_press(BUTTON_ID_SELECT, 0, 20, 0));
  prv_run_sequence();
  cl_assert_equal_i(0, s_event_count);
}

void test_remote_input__rejects_unknown_button(void) {
  cl_assert_equal_i(RemoteInputResult_Invalid,
                    remote_input_button_press(NUM_BUTTONS, 1, 20, 0));
  cl_assert_equal_i(0, s_event_count);
}

void test_remote_input__second_request_is_busy(void) {
  cl_assert_equal_i(RemoteInputResult_Ok, remote_input_button_press(BUTTON_ID_SELECT, 1, 20, 0));
  cl_assert_equal_i(RemoteInputResult_Busy, remote_input_button_press(BUTTON_ID_UP, 1, 20, 0));
  // Swipes share the timer, so they are single-flight against button presses too.
  cl_assert_equal_i(RemoteInputResult_Busy,
                    remote_input_swipe(RemoteInputSwipeDirection_Up, 150));

  prv_run_sequence();

  // ...and the next request is accepted once the sequence has drained.
  cl_assert_equal_i(RemoteInputResult_Ok, remote_input_button_press(BUTTON_ID_UP, 1, 20, 0));
}

void test_remote_input__button_set_holds_until_released(void) {
  cl_assert_equal_i(RemoteInputResult_Ok, remote_input_button_set(1 << BUTTON_ID_SELECT));

  // The button goes down and stays down: no timer runs it back up.
  cl_assert_equal_i(1, s_event_count);
  cl_assert_equal_i(PEBBLE_BUTTON_DOWN_EVENT, s_events[0].type);
  cl_assert_equal_i(BUTTON_ID_SELECT, s_events[0].button.button_id);
  cl_assert(!stub_new_timer_is_scheduled(INPUT_TIMER_ID));

  cl_assert_equal_i(RemoteInputResult_Ok, remote_input_button_set(0));
  cl_assert_equal_i(2, s_event_count);
  cl_assert_equal_i(PEBBLE_BUTTON_UP_EVENT, s_events[1].type);
  cl_assert_equal_i(BUTTON_ID_SELECT, s_events[1].button.button_id);
}

void test_remote_input__button_set_holds_a_chord(void) {
  cl_assert_equal_i(RemoteInputResult_Ok,
                    remote_input_button_set((1 << BUTTON_ID_BACK) | (1 << BUTTON_ID_SELECT)));

  cl_assert_equal_i(2, s_event_count);
  cl_assert_equal_i(PEBBLE_BUTTON_DOWN_EVENT, s_events[0].type);
  cl_assert_equal_i(BUTTON_ID_BACK, s_events[0].button.button_id);
  cl_assert_equal_i(PEBBLE_BUTTON_DOWN_EVENT, s_events[1].type);
  cl_assert_equal_i(BUTTON_ID_SELECT, s_events[1].button.button_id);

  // Releasing one leaves the other held.
  cl_assert_equal_i(RemoteInputResult_Ok, remote_input_button_set(1 << BUTTON_ID_SELECT));
  cl_assert_equal_i(3, s_event_count);
  cl_assert_equal_i(PEBBLE_BUTTON_UP_EVENT, s_events[2].type);
  cl_assert_equal_i(BUTTON_ID_BACK, s_events[2].button.button_id);
}

void test_remote_input__button_set_only_emits_changes(void) {
  cl_assert_equal_i(RemoteInputResult_Ok, remote_input_button_set(1 << BUTTON_ID_UP));
  cl_assert_equal_i(1, s_event_count);

  // Re-asserting the same mask is a no-op, not a second press.
  cl_assert_equal_i(RemoteInputResult_Ok, remote_input_button_set(1 << BUTTON_ID_UP));
  cl_assert_equal_i(1, s_event_count);
}

void test_remote_input__button_set_releases_before_pressing(void) {
  cl_assert_equal_i(RemoteInputResult_Ok, remote_input_button_set(1 << BUTTON_ID_UP));
  // Swapping one held button for another must not hold both at once, even momentarily.
  cl_assert_equal_i(RemoteInputResult_Ok, remote_input_button_set(1 << BUTTON_ID_DOWN));

  cl_assert_equal_i(3, s_event_count);
  cl_assert_equal_i(PEBBLE_BUTTON_UP_EVENT, s_events[1].type);
  cl_assert_equal_i(BUTTON_ID_UP, s_events[1].button.button_id);
  cl_assert_equal_i(PEBBLE_BUTTON_DOWN_EVENT, s_events[2].type);
  cl_assert_equal_i(BUTTON_ID_DOWN, s_events[2].button.button_id);
}

void test_remote_input__button_set_rejects_unknown_buttons(void) {
  cl_assert_equal_i(RemoteInputResult_Invalid, remote_input_button_set(1 << NUM_BUTTONS));
  cl_assert_equal_i(0, s_event_count);
}

void test_remote_input__held_buttons_block_sequences(void) {
  cl_assert_equal_i(RemoteInputResult_Ok, remote_input_button_set(1 << BUTTON_ID_BACK));

  cl_assert_equal_i(RemoteInputResult_Busy, remote_input_button_press(BUTTON_ID_UP, 1, 20, 0));
  cl_assert_equal_i(RemoteInputResult_Busy,
                    remote_input_swipe(RemoteInputSwipeDirection_Up, 150));

  // Releasing the hold lets sequences through again.
  cl_assert_equal_i(RemoteInputResult_Ok, remote_input_button_set(0));
  cl_assert_equal_i(RemoteInputResult_Ok, remote_input_button_press(BUTTON_ID_UP, 1, 20, 0));
}

void test_remote_input__running_sequence_blocks_button_set(void) {
  cl_assert_equal_i(RemoteInputResult_Ok, remote_input_button_press(BUTTON_ID_SELECT, 1, 20, 0));
  cl_assert_equal_i(RemoteInputResult_Busy, remote_input_button_set(1 << BUTTON_ID_BACK));

  prv_run_sequence();
  cl_assert_equal_i(RemoteInputResult_Ok, remote_input_button_set(1 << BUTTON_ID_BACK));
}

void test_remote_input__endpoint_acks_a_button_set_command(void) {
  const uint8_t msg[] = {0x02, (1 << BUTTON_ID_BACK) | (1 << BUTTON_ID_SELECT)};
  remote_input_protocol_msg_callback(NULL, msg, sizeof(msg));

  cl_assert_equal_i(1, s_ack_count);
  cl_assert_equal_i(2, s_ack_length);
  cl_assert_equal_i(0x02, s_ack[0]);
  cl_assert_equal_i(RemoteInputResult_Ok, s_ack[1]);
  cl_assert_equal_i(2, s_event_count);
}

void test_remote_input__swipe_up_travels_up_in_a_straight_line(void) {
  cl_assert_equal_i(RemoteInputResult_Ok, remote_input_swipe(RemoteInputSwipeDirection_Up, 150));
  prv_run_sequence();

  // Touchdown, the position updates, then liftoff.
  cl_assert(s_touch_count >= 4);
  cl_assert_equal_i(TouchState_FingerDown, s_touches[0].state);
  cl_assert_equal_i(TouchState_FingerUp, s_touches[s_touch_count - 1].state);

  const int last_down = s_touch_count - 2;
  for (int i = 1; i <= last_down; i++) {
    cl_assert_equal_i(TouchState_FingerDown, s_touches[i].state);
    // Straight: the swipe recognizer fails a path that wanders off the major axis.
    cl_assert_equal_i(prv_observed_x(0), prv_observed_x(i));
    // Monotonically upward, so the direction can't be ambiguous.
    cl_assert(prv_observed_y(i) < prv_observed_y(i - 1));
  }

  // Long enough for the recognizer's 30px minimum.
  cl_assert(prv_observed_y(0) - prv_observed_y(last_down) >= 30);
}

void test_remote_input__swipe_right_travels_right(void) {
  cl_assert_equal_i(RemoteInputResult_Ok, remote_input_swipe(RemoteInputSwipeDirection_Right, 150));
  prv_run_sequence();

  const int last_down = s_touch_count - 2;
  cl_assert_equal_i(prv_observed_y(0), prv_observed_y(last_down));
  cl_assert(prv_observed_x(last_down) - prv_observed_x(0) >= 30);
}

void test_remote_input__swipe_left_travels_left(void) {
  cl_assert_equal_i(RemoteInputResult_Ok, remote_input_swipe(RemoteInputSwipeDirection_Left, 150));
  prv_run_sequence();

  const int last_down = s_touch_count - 2;
  cl_assert_equal_i(prv_observed_y(0), prv_observed_y(last_down));
  cl_assert(prv_observed_x(0) - prv_observed_x(last_down) >= 30);
}

void test_remote_input__swipe_down_travels_down(void) {
  cl_assert_equal_i(RemoteInputResult_Ok, remote_input_swipe(RemoteInputSwipeDirection_Down, 150));
  prv_run_sequence();

  const int last_down = s_touch_count - 2;
  cl_assert_equal_i(prv_observed_x(0), prv_observed_x(last_down));
  cl_assert(prv_observed_y(last_down) - prv_observed_y(0) >= 30);
}

void test_remote_input__swipe_rejected_when_injection_unavailable(void) {
  // Touch off, or a real finger already on the screen: report it rather than acknowledging a
  // gesture the touch service would drop on the floor.
  s_injection_available = false;
  cl_assert_equal_i(RemoteInputResult_Invalid,
                    remote_input_swipe(RemoteInputSwipeDirection_Up, 150));
  cl_assert_equal_i(0, s_touch_count);
}

void test_remote_input__rejects_swipe_slower_than_the_recognizer_allows(void) {
  // Above the recognizer's maximum the path is a pan; acknowledging it would promise a swipe that
  // can never be reported.
  cl_assert_equal_i(RemoteInputResult_Invalid,
                    remote_input_swipe(RemoteInputSwipeDirection_Up, SWIPE_MAX_DURATION_MS + 1));
  cl_assert_equal_i(0, s_touch_count);

  cl_assert_equal_i(RemoteInputResult_Ok,
                    remote_input_swipe(RemoteInputSwipeDirection_Up, SWIPE_MAX_DURATION_MS));
}

void test_remote_input__swipe_aborts_when_injection_is_refused(void) {
  cl_assert_equal_i(RemoteInputResult_Ok, remote_input_swipe(RemoteInputSwipeDirection_Up, 150));
  // Touchdown lands, then the touch service takes the sensor away mid-path.
  stub_new_timer_fire(INPUT_TIMER_ID);
  cl_assert_equal_i(1, s_touch_count);
  s_injection_accepts = false;
  stub_new_timer_fire(INPUT_TIMER_ID);

  // The sequence stops instead of leaving a half-finished path behind, and frees its claim so the
  // next request is not stuck reporting Busy.
  cl_assert(!stub_new_timer_is_scheduled(INPUT_TIMER_ID));
  s_injection_accepts = true;
  cl_assert_equal_i(RemoteInputResult_Ok, remote_input_button_press(BUTTON_ID_UP, 1, 20, 0));
}

void test_remote_input__swipe_stays_on_screen(void) {
  const RemoteInputSwipeDirection directions[] = {
    RemoteInputSwipeDirection_Up, RemoteInputSwipeDirection_Down,
    RemoteInputSwipeDirection_Left, RemoteInputSwipeDirection_Right,
  };
  for (unsigned d = 0; d < ARRAY_LENGTH(directions); d++) {
    s_touch_count = 0;
    cl_assert_equal_i(RemoteInputResult_Ok, remote_input_swipe(directions[d], 150));
    prv_run_sequence();
    for (int i = 0; i < s_touch_count; i++) {
      cl_assert(s_touches[i].x >= 0 && s_touches[i].x < DISP_COLS);
      cl_assert(s_touches[i].y >= 0 && s_touches[i].y < DISP_ROWS);
    }
  }
}

void test_remote_input__rejects_unknown_swipe_direction(void) {
  cl_assert_equal_i(RemoteInputResult_Invalid,
                    remote_input_swipe(RemoteInputSwipeDirectionCount, 150));
  cl_assert_equal_i(0, s_touch_count);
}

void test_remote_input__endpoint_acks_a_button_command(void) {
  // command=button, button=down, presses=2, hold=0x0032, gap=0x000a
  const uint8_t msg[] = {0x00, 0x03, 0x02, 0x00, 0x32, 0x00, 0x0a};
  remote_input_protocol_msg_callback(NULL, msg, sizeof(msg));

  cl_assert_equal_i(1, s_ack_count);
  cl_assert_equal_i(2, s_ack_length);
  cl_assert_equal_i(0x00, s_ack[0]);
  cl_assert_equal_i(RemoteInputResult_Ok, s_ack[1]);

  prv_run_sequence();
  cl_assert_equal_i(4, s_event_count);
  cl_assert_equal_i(BUTTON_ID_DOWN, s_events[0].button.button_id);
}

void test_remote_input__endpoint_acks_a_swipe_command(void) {
  // command=swipe, direction=left, duration=0 (watch default)
  const uint8_t msg[] = {0x01, 0x02, 0x00, 0x00};
  remote_input_protocol_msg_callback(NULL, msg, sizeof(msg));

  cl_assert_equal_i(RemoteInputResult_Ok, s_ack[1]);
  prv_run_sequence();

  const int last_down = s_touch_count - 2;
  cl_assert(prv_observed_x(0) - prv_observed_x(last_down) >= 30);
}

void test_remote_input__endpoint_rejects_malformed_messages(void) {
  const uint8_t truncated[] = {0x00, 0x03};
  remote_input_protocol_msg_callback(NULL, truncated, sizeof(truncated));
  cl_assert_equal_i(RemoteInputResult_Invalid, s_ack[1]);

  const uint8_t unknown_command[] = {0x7f, 0x00, 0x00, 0x00};
  remote_input_protocol_msg_callback(NULL, unknown_command, sizeof(unknown_command));
  cl_assert_equal_i(0x7f, s_ack[0]);
  cl_assert_equal_i(RemoteInputResult_Invalid, s_ack[1]);

  remote_input_protocol_msg_callback(NULL, unknown_command, 0);
  cl_assert_equal_i(RemoteInputResult_Invalid, s_ack[1]);

  cl_assert_equal_i(0, s_event_count);
  cl_assert_equal_i(0, s_touch_count);
}
