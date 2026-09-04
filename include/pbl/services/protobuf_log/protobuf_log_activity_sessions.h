/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "protobuf_log.h"

#include "pbl/services/activity/activity.h"

#include <stdbool.h>

ProtobufLogRef protobuf_log_activity_sessions_create(void);

bool protobuf_log_activity_sessions_add(ProtobufLogRef ref, time_t sample_utc,
                                       ActivitySession *session);

bool protobuf_log_activity_sessions_decode(pebble_pipeline_Event *event_in,
                                           ActivitySession *session_out);
