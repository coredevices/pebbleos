/*
 * Copyright 2024 Google LLC
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "debug.h"
#include "advanced_logging.h"

#include "flash_logging.h"
#include "debug_reboot_reason.h"

#include "drivers/watchdog.h"
#include "flash_region/flash_region.h"
#include "kernel/events.h"
#include "kernel/logging_private.h"
#include "kernel/pbl_malloc.h"
#include "kernel/pebble_tasks.h"
#if MEMFAULT
#include "memfault/core/platform/core.h"
#endif
#include "mfg/mfg_serials.h"
#include "process_management/app_manager.h"
#include "services/common/analytics/analytics.h"
#include "services/common/comm_session/session.h"
#include "services/common/comm_session/session_send_buffer.h"
#include "services/common/system_task.h"
#include "services/normal/process_management/app_storage.h"
#include "system/bootbits.h"
#include "system/hexdump.h"
#include "system/logging.h"
#include "system/passert.h"
#include "system/reboot_reason.h"
#include "system/version.h"
#include "util/attributes.h"
#include "util/build_id.h"

#include <inttypes.h>

static const uint16_t ENDPOINT_ID = 2002;

typedef struct PACKED {
  uint8_t command;
  uint32_t cookie;
} BluetoothHeader;

typedef struct BluetoothDumpLineCallbackData {
  bool in_progress;
  CommSession *comm_session;
  int generation;
  uint32_t cookie;
} BluetoothDumpLineCallbackData;

BluetoothDumpLineCallbackData s_bt_dump_chunk_callback_data;

static void prv_put_status_event(DebugInfoEventState state) {
  PebbleEvent event = {
    .type = PEBBLE_GATHER_DEBUG_INFO_EVENT,
    .debug_info = {
      .source = DebugInfoSourceFWLogs,
      .state = state,
    },
  };
  event_put(&event);
}

static bool prv_bt_log_dump_line_cb(uint8_t *message, uint32_t total_length) {
  CommSession *session = s_bt_dump_chunk_callback_data.comm_session;

  // keep us sending data quickly
  comm_session_set_responsiveness(
      s_bt_dump_chunk_callback_data.comm_session, BtConsumerPpLogDump, ResponseTimeMin, 5);

  const uint16_t required_length = total_length + 1 + 4;
  SendBuffer *sb = comm_session_send_buffer_begin_write(session, ENDPOINT_ID, required_length,
                                                        COMM_SESSION_DEFAULT_TIMEOUT);
  if (!sb) {
    PBL_LOG(LOG_LEVEL_DEBUG, "Failed to get send buffer");
    return false;
  }

  BluetoothHeader header = {
    .command = 0x80,
    .cookie = s_bt_dump_chunk_callback_data.cookie,
  };
  comm_session_send_buffer_write(sb, (const uint8_t *) &header, sizeof(header));
  comm_session_send_buffer_write(sb, message, total_length);
  comm_session_send_buffer_end_write(sb);
  return true;
}

// Called by flash_dump_log_file() when the log has been completely dumped
static void prv_bt_log_dump_completed_cb(bool success) {
  BluetoothHeader header = {
    .cookie = s_bt_dump_chunk_callback_data.cookie
  };
  // Send a "no logs" message if the generation did not exist and the remote supports
  // "infinite log dumping"
  CommSession *session = s_bt_dump_chunk_callback_data.comm_session;
  if (!success && comm_session_has_capability(session, CommSessionInfiniteLogDumping)) {
    header.command = 0x82;
    comm_session_send_data(s_bt_dump_chunk_callback_data.comm_session, ENDPOINT_ID,
                           (uint8_t *) &header, sizeof(header),
                           COMM_SESSION_DEFAULT_TIMEOUT);
  } else {
    // Otherwise, just send a "done" message
    header.command = 0x81;
    comm_session_send_data(s_bt_dump_chunk_callback_data.comm_session, ENDPOINT_ID,
                           (uint8_t *) &header, sizeof(header), COMM_SESSION_DEFAULT_TIMEOUT);
  }

  s_bt_dump_chunk_callback_data.in_progress = false;

  // Ok to enter a lower power less responsive state
  comm_session_set_responsiveness(
      s_bt_dump_chunk_callback_data.comm_session, BtConsumerPpLogDump, ResponseTimeMax, 0);
  prv_put_status_event(DebugInfoStateFinished);
}

static void prv_flash_logging_bluetooth_dump(
    CommSession *session, int generation, uint32_t cookie) {
  PBL_ASSERT_RUNNING_FROM_EXPECTED_TASK(PebbleTask_KernelBackground);
  if (s_bt_dump_chunk_callback_data.in_progress) {
    PBL_LOG(LOG_LEVEL_ERROR, "Already in the middle of dumping logs");
    return;
  }

  prv_put_status_event(DebugInfoStateStarted);

  // Temporarily disable logging so we don't log forever.
  flash_logging_set_enabled(false);

  s_bt_dump_chunk_callback_data.in_progress = true;
  s_bt_dump_chunk_callback_data.generation = generation;
  s_bt_dump_chunk_callback_data.comm_session = session;
  s_bt_dump_chunk_callback_data.cookie = cookie;

  flash_dump_log_file(s_bt_dump_chunk_callback_data.generation, prv_bt_log_dump_line_cb,
                      prv_bt_log_dump_completed_cb);
  flash_logging_set_enabled(true);
}

void dump_log_protocol_msg_callback(CommSession *session, const uint8_t* data, size_t length) {
  uint32_t cookie;
  int generation = 0;
  if (data[0] == 0x10 || data[0] == 0x11) {
    if (length != 6) {
      PBL_LOG(LOG_LEVEL_ERROR, "Invalid dump log message received -- length %u", length);
      return;
    }

    generation = data[1];
    cookie = *((uint32_t*) (data + 2));
  } else {
    if (length != 5) {
      PBL_LOG(LOG_LEVEL_ERROR, "Invalid dump log message received -- length %u", length);
      return;
    }

    cookie = *((uint32_t*) (data + 1));
  }

  switch (*data) {
  case 0x00:
    prv_flash_logging_bluetooth_dump(session, 0, cookie);
    break;
  case 0x01:
    prv_flash_logging_bluetooth_dump(session, 1, cookie);
    break;
  case 0x10:
    prv_flash_logging_bluetooth_dump(session, generation, cookie);
    break;
  case 0x02:
  case 0x03:
  case 0x11:
    break;
  }
}

void debug_init(McuRebootReason mcu_reboot_reason) {
  advanced_logging_init();

  // Log the firmware version in the first flash log line:
  DEBUG_LOG(LOG_LEVEL_INFO, "%s (platform: %u, hw: %s, sn: %s, pcba: %s)",
      TINTIN_METADATA.version_tag,
      TINTIN_METADATA.hw_platform,
      mfg_get_hw_version(),
      mfg_get_serial_number(),
      mfg_get_pcba_serial_number());

  // Log the firmware build id to flash:
  char build_id_string[64];
  version_copy_current_build_id_hex_string(build_id_string, 64);
  DEBUG_LOG(LOG_LEVEL_INFO, "BUILD ID: %s", build_id_string);

  debug_reboot_reason_print(mcu_reboot_reason);

  #if MEMFAULT
  memfault_platform_boot();
  #endif
}

void debug_print_last_launched_app(void) {
  // Get the slot of the last launched app
  // so we know what was running when we rebooted
  uint32_t last_launched_app_slot = reboot_get_slot_of_last_launched_app();

  // check if last app launched was a system app
  if (last_launched_app_slot == (uint32_t)SYSTEM_APP_BANK_ID) {
    DEBUG_LOG(LOG_LEVEL_INFO, "Last launched app: <System_App>");
  } else if ((last_launched_app_slot != (uint32_t)INVALID_BANK_ID)) {
    PebbleProcessInfo last_launched_app;
    uint8_t build_id[BUILD_ID_EXPECTED_LEN];
    AppStorageGetAppInfoResult result = app_storage_get_process_info(&last_launched_app,
                                                                     build_id,
                                                                     (AppInstallId)last_launched_app_slot,
                                                                     PebbleTask_App);

    if (result == GET_APP_INFO_SUCCESS) {
      DEBUG_LOG(LOG_LEVEL_INFO, "Last launched app: %s", last_launched_app.name);
      PBL_HEXDUMP(LOG_LEVEL_INFO, build_id, sizeof(build_id));
    }
  }
}
