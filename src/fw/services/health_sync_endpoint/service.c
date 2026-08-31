/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "pbl/services/comm_session/session.h"
#include "pbl/services/system_task.h"
#include "pbl/services/data_logging/data_logging_service.h"
#include <pbl/logging/logging.h>
#include "pbl/util/attributes.h"

PBL_LOG_MODULE_DEFINE(service_health_sync_endpoint, CONFIG_SERVICE_HEALTH_SYNC_ENDPOINT_LOG_LEVEL);

#define HEALTH_SYNC_ENDPOINT_ID 911
#define ACK 0x1
#define NACK 0x2

typedef enum HealthSyncEndpointCmd {
  HealthSyncEndpointCmd_Sync = 0x1,
  HealthSyncEndpointCmd_Ack = 0x11,
} HealthSyncEndpointCmd;

typedef struct PACKED HealthSyncEndpointSyncMsg {
  HealthSyncEndpointCmd cmd : 8;
  uint32_t seconds_since_sync;
} HealthSyncEndpointSyncMsg;

typedef struct PACKED HealthSyncEndpointAckMsg {
  HealthSyncEndpointCmd cmd : 8;
  uint8_t ack_nack;
} HealthSyncEndpointAckMsg;

static void prv_send_ack_nack(bool ok) {
  const HealthSyncEndpointAckMsg msg = {
    .cmd = HealthSyncEndpointCmd_Ack,
    .ack_nack = ok ? ACK : NACK,
  };

  comm_session_send_data(comm_session_get_system_session(),
                         HEALTH_SYNC_ENDPOINT_ID,
                         (uint8_t*)&msg,
                         sizeof(HealthSyncEndpointAckMsg),
                         COMM_SESSION_DEFAULT_TIMEOUT);
}

#include "pbl/services/activity/activity_algorithm.h"

static void prv_sync_health_system_task_cb(void *unused) {
  if (activity_tracking_on()) {
    // tell the activity service to pool the minutes it's got so far
    activity_algorithm_send_minutes();
  }

  // send all data logging data
  dls_send_all_sessions();
  // ACK
  prv_send_ack_nack(true /*ok*/);
}

static void prv_handle_sync(const uint8_t *msg, size_t len) {
  if (len < sizeof(HealthSyncEndpointSyncMsg)) {
    PBL_LOG_ERR("Invalid SYNC msg received, length: %u", len);
    return;
  }

  PBL_LOG_DBG("Received health SYNC request");

  system_task_add_callback(prv_sync_health_system_task_cb, NULL);
}

void health_sync_protocol_msg_callback(CommSession *session, const uint8_t *msg, size_t len) {
  if (len < 1) {
    PBL_LOG_ERR("Invalid message received, length: %u", len);
  }

  HealthSyncEndpointCmd cmd = *msg;
  switch (cmd) {
    case HealthSyncEndpointCmd_Sync:
      prv_handle_sync(msg, len);
      break;

    default:
      PBL_LOG_WRN("Unexpected command received, 0x%x", cmd);
      return;
  }
}
