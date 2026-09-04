/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "pbl/util/attributes.h"

#include <stdint.h>

typedef enum {
  MsgIdDataTransfer = 0x02,
  MsgIdStopTransfer = 0x03,
} MsgId;

typedef struct PACKED {
  MsgId msg_id;
  AudioEndpointSessionId session_id;
  uint8_t frame_count;
  uint8_t frames[];
} DataTransferMsg;

typedef struct PACKED {
  MsgId msg_id;
  AudioEndpointSessionId session_id;
} StopTransferMsg;
