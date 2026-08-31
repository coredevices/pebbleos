/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "pbl/services/comm_session/session.h"

//! Callback for handling a screenshot request message from the client
void screenshot_protocol_msg_callback(CommSession *session, const uint8_t* data, unsigned int length);
