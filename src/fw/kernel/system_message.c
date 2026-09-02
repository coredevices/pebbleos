/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "kernel/system_message.h"

#include "flash_region/filesystem_regions.h"
#include "kernel/events.h"
#include "pbl/services/comm_session/protocol.h"
#include "pbl/services/comm_session/session.h"
#include "pbl/services/firmware_update.h"
#include "pbl/services/put_bytes/put_bytes.h"
#include "pbl/services/system_task.h"
#include <pbl/logging/logging.h>
#include "system/passert.h"
#include "system/reboot_reason.h"
#include "system/reset.h"
#include "pbl/util/attributes.h"

#include <stdbool.h>
#include <inttypes.h>

static const uint16_t ENDPOINT_ID = 0x12;

void system_message_send(SystemMessageType type) {
  uint8_t buffer[2] = { 0x00, type };
  CommSession *system_session = comm_session_get_system_session();
  comm_session_send_data(system_session, ENDPOINT_ID, buffer, sizeof(buffer), COMM_SESSION_DEFAULT_TIMEOUT);
  PBL_LOG_DBG("Sending sysmsg: %u", type);
}

static void prv_reset_kernel_bg_cb(void *unused) {
  PBL_LOG_ALWAYS("Rebooting to install firmware...");
  RebootReason reason = { RebootReasonCode_SoftwareUpdate, 0 };
  reboot_reason_set(&reason);
  system_reset();
}

static void prv_ui_update_reset_delay_timer_callback(void *unused) {
  system_task_add_callback(prv_reset_kernel_bg_cb, NULL);
}

static void prv_handle_firmware_complete_msg(void) {
  uint32_t timeout = 3000;

  // Wait 3 seconds before rebooting so there is time to show the update complete screen
  PBL_LOG_ALWAYS("Delaying reset by 3s so the UI can update...");
  TimerID timer = new_timer_create();  // Don't bother cleaning up this timer, we're going to reset
  PBL_ASSERTN(timer != TIMER_INVALID_ID);
  new_timer_start(timer, timeout, prv_ui_update_reset_delay_timer_callback, NULL, 0);
}

//! Note: For now we just call into storage directly for the status of FW installs. Someday,
//! it would be nice for this exchange to take place as part of PutBytes
extern bool pb_storage_get_status(PutBytesObjectType obj_type, PbInstallStatus *status);
static void prv_handle_firmware_status_request(CommSession *session) {
  struct PACKED {
    uint8_t  deprecated;
    uint8_t  type;
    uint8_t  rsvd[2];
    uint32_t resource_bytes_written;
    uint32_t resource_crc;
    uint32_t firmware_bytes_written;
    uint32_t firmware_crc;
  } fw_status_resp = {
    .type = SysMsgFirmwareStatusResponse,
  };

  PbInstallStatus status = { };
  if (pb_storage_get_status(ObjectFirmware, &status)) {
    fw_status_resp.firmware_bytes_written = status.num_bytes_written;
    fw_status_resp.firmware_crc = status.crc_of_bytes;
  }

  if (pb_storage_get_status(ObjectSysResources, &status)) {
    fw_status_resp.resource_bytes_written = status.num_bytes_written;
    fw_status_resp.resource_crc = status.crc_of_bytes;
  }

  PBL_LOG_DBG("FW Status Resp: res %"PRIu32" : 0x%x fw %"PRIu32" : 0x%x",
          fw_status_resp.resource_bytes_written, (int)fw_status_resp.resource_crc,
          fw_status_resp.firmware_bytes_written, (int)fw_status_resp.firmware_crc);

  comm_session_send_data(session, ENDPOINT_ID, (uint8_t *)&fw_status_resp, sizeof(fw_status_resp),
                         COMM_SESSION_DEFAULT_TIMEOUT);
}

void sys_msg_protocol_msg_callback(CommSession *session, const uint8_t* data, size_t length) {
  PBL_ASSERT_RUNNING_FROM_EXPECTED_TASK(PebbleTask_KernelBackground);

  SystemMessageType t = data[1];

  PBL_LOG_DBG("Received sysmsg: %u", t);

  switch (t) {
  case SysMsgFirmwareAvailable_Deprecated: {
    PBL_LOG_DBG("Deprecated available message received.");
    break;
  }

  case SysMsgFirmwareStart: {
    PBL_LOG_VERBOSE("About to receive new firmware!");

    uint32_t bytes_transferred = 0;
    uint32_t total_size = 0;

    bool smooth_progress_supported =
        comm_session_has_capability(session, CommSessionSmoothFwInstallProgressSupport) &&
        (length >= sizeof(SysMsgSmoothFirmwareStartPayload));

    if (smooth_progress_supported) {
      SysMsgSmoothFirmwareStartPayload *payload = (SysMsgSmoothFirmwareStartPayload *)data;
      bytes_transferred = payload->bytes_already_transferred;
      total_size = bytes_transferred + payload->bytes_to_transfer;
      PBL_LOG_INFO("Starting FW update, %"PRIu32" of %"PRIu32" bytes already transferred",
                   bytes_transferred, total_size);
    }

    PebbleEvent e = {
      .type = PEBBLE_SYSTEM_MESSAGE_EVENT,
      .firmware_update = {
        .type = smooth_progress_supported ?
            PebbleSystemMessageFirmwareUpdateStart : PebbleSystemMessageFirmwareUpdateStartLegacy,
        .bytes_transferred = bytes_transferred,
        .total_transfer_size = total_size,
      }
    };
    event_put(&e);
    break;
  }

  case SysMsgFirmwareStatus:
    prv_handle_firmware_status_request(session);
    break;

  case SysMsgFirmwareComplete: {
    PBL_LOG_VERBOSE("Firmware transfer succeeded, okay to restart!");
    PebbleEvent e = {
      .type = PEBBLE_SYSTEM_MESSAGE_EVENT,
      .firmware_update.type = PebbleSystemMessageFirmwareUpdateComplete,
    };
    event_put(&e);
    prv_handle_firmware_complete_msg();
    break;
  }

  case SysMsgFirmwareFail: {
    PBL_LOG_VERBOSE("Firmware transfer failed, time to clean up!");
    PebbleEvent e = {
      .type = PEBBLE_SYSTEM_MESSAGE_EVENT,
      .firmware_update.type = PebbleSystemMessageFirmwareUpdateFailed,
    };
    event_put(&e);
    break;
  }

  case SysMsgFirmwareUpToDate: {
    PBL_LOG_VERBOSE("Firmware is up to date!");
    PebbleEvent e = {
      .type = PEBBLE_SYSTEM_MESSAGE_EVENT,
      .firmware_update.type = PebbleSystemMessageFirmwareUpToDate,
    };
    event_put(&e);
    break;
  }

  default:
    PBL_LOG_ERR("Invalid message received, type is %u", data[1]);
    break;
  }
}

void system_message_send_firmware_start_response(FirmwareUpdateStatus status) {
  struct PACKED {
    uint8_t zero;
    uint8_t type;
    uint8_t status;
  } msg = {
    .zero = 0x00,
    .type = SysMsgFirmwareStartResponse,
    .status = status
  };

  CommSession *session = comm_session_get_system_session();
  comm_session_send_data(session, ENDPOINT_ID, (const uint8_t*) &msg, sizeof(msg), COMM_SESSION_DEFAULT_TIMEOUT);
}
