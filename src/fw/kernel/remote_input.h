/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include <pbl/drivers/button_id.h>
#include "pbl/services/comm_session/session.h"

#include <stddef.h>
#include <stdint.h>

//! Result of an injected input request. Doubles as the status byte of the
//! remote input endpoint's ack.
typedef enum RemoteInputResult {
  RemoteInputResult_Ok = 0,
  RemoteInputResult_Busy = 1,     //!< another injected sequence is still running
  RemoteInputResult_Invalid = 2,  //!< bad arguments, or unsupported on this board
} RemoteInputResult;

//! Logical swipe direction, resolved to a touch path by the watch.
typedef enum RemoteInputSwipeDirection {
  RemoteInputSwipeDirection_Up = 0,
  RemoteInputSwipeDirection_Down = 1,
  RemoteInputSwipeDirection_Left = 2,
  RemoteInputSwipeDirection_Right = 3,
  RemoteInputSwipeDirectionCount,
} RemoteInputSwipeDirection;

//! Default swipe duration, comfortably inside the swipe recognizer's limit.
#define REMOTE_INPUT_SWIPE_DEFAULT_DURATION_MS 150

//! Initialize the injected input service.
void remote_input_init(void);

//! Synthesize button presses, as if the button had been pressed by hand.
//! @param button The button to press
//! @param presses Number of press/release pairs. 0 is a no-op success.
//! @param hold_ms How long each press is held down
//! @param gap_ms Delay between successive presses
RemoteInputResult remote_input_button_press(ButtonId button, uint32_t presses, uint32_t hold_ms,
                                            uint32_t gap_ms);

//! Hold down exactly the given set of buttons, releasing any others that were held. Bit x of
//! @p buttons is ButtonId x, so a mask with two bits set holds a chord, and 0 releases everything.
//!
//! Unlike remote_input_button_press() this leaves the buttons down: nothing releases them but a
//! later call, so a caller that disconnects mid-hold leaves the watch holding a button.
//!
//! Reports Busy while a press or swipe sequence is running, and those report Busy while any
//! button is held here.
//! @param buttons Bitmask of buttons to hold down
RemoteInputResult remote_input_button_set(uint8_t buttons);

//! Synthesize a straight swipe across the middle of the screen. Injected through the touch
//! service, so @p direction is always the direction the UI sees, whatever the display rotation.
//! @param direction Which way the finger travels
//! @param duration_ms Touchdown-to-liftoff duration. Anything above the swipe recognizer's
//!        maximum is Invalid: the recognizer would read it as a pan and never report a swipe.
RemoteInputResult remote_input_swipe(RemoteInputSwipeDirection direction, uint16_t duration_ms);

//! Remote input endpoint handler. Answers every message with a {command, RemoteInputResult} ack.
void remote_input_protocol_msg_callback(CommSession *session, const uint8_t *data, size_t length);
