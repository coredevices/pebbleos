/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "pbl/util/uuid.h"

//! Launcher App Message is deprecated and on Android >= 2.3 and other devices that pass the
//! support flags for the AppRunState endpoint will use that endpoint (0x34) instead.  That
//! endpoint should be used for sending messages on start/stop status of applications and
//! sending/receiving application states.  The LauncherAppMessage endpoint is kept for
//! backwards compatibility with older mobile applications.
void launcher_app_message_send_app_state_deprecated(const Uuid *uuid, bool running);
