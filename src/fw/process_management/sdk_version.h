/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once
#include "pebble_process_info.h"
#include <stdbool.h>

//! Inspects the app metadata whether the app supports app messaging.
//! Only if this returns true, the .messaging_info field of PebbleAppHandlers can be used.
//! @return true if the app is built with an SDK that supports app messaging or not
bool sdk_version_is_app_messaging_supported(const Version * const sdk_version);
