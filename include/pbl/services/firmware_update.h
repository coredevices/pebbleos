/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include <stdbool.h>

#include "kernel/events.h"

void firmware_update_init(void);

unsigned int firmware_update_get_percent_progress(void);

void firmware_update_event_handler(PebbleSystemMessageEvent* event);
void firmware_update_pb_event_handler(PebblePutBytesEvent *event);

typedef enum {
  FirmwareUpdateStopped = 0,
  FirmwareUpdateRunning = 1,
  FirmwareUpdateCancelled = 2,
  FirmwareUpdateFailed = 3,
} FirmwareUpdateStatus;

FirmwareUpdateStatus firmware_update_current_status(void);

bool firmware_update_is_in_progress(void);
