/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "pbl/services/comm_session/session_remote_version.h"

#include "comm/bt_lock.h"

#include "kernel/events.h"

#include "pbl/services/comm_session/session_internal.h"
#include "pbl/services/comm_session/session_remote_os.h"
#include "kernel/event_loop.h"

#include "pbl/util/attributes.h"
#include "util/net.h"
#include <pbl/logging/logging.h>

PBL_LOG_MODULE_DECLARE(service_comm_session, CONFIG_SERVICE_COMM_SESSION_LOG_LEVEL);


extern bool comm_session_is_valid(const CommSession *session);

#define MAX_REQUEST_RETRIES (3)

//! Defined in session.c. We should only be setting the capabilities here
extern void comm_session_set_capabilities(
    CommSession *session, CommSessionCapability capability_flags);

typedef enum {
  CommSessionVersionCommandRequest = 0x00,
  CommSessionVersionCommandResponse = 0x01,
} CommSessionVersionCommand;

// The 1.x mobile app response
struct PACKED VersionsPhoneResponseV1 {
  uint32_t pebble_library_version;
  uint32_t session_capabilities_bitfield;
  uint32_t platform_bitfield;
};

// The 2.x mobile apps return a longer response than the 1.x apps do.
struct PACKED VersionsPhoneResponseV2 {
  uint32_t pebble_library_version;
  uint32_t session_capabilities_bitfield;
  uint32_t platform_bitfield;
  uint8_t  response_version;  // Set to 2 in this format of the response.
  uint8_t  major_version;     // major version number of the mobile app, i.e. 2
  uint8_t  minor_version;     // minfo version number of the mobile app, i.e. 0
  uint8_t  bugfix_version;    // bugfix version number of the mobile app, i.e. 1
};

// The 3.x mobile apps return a longer response than the 2.x apps do
struct PACKED VersionsPhoneResponseV3 {
  uint32_t pebble_library_version_deprecated; // Deprecated as of v3.x
  uint32_t session_capabilities_bitfield;     // Deprecated as of v3.x
  uint32_t platform_bitfield;
  uint8_t  response_version;  // Set to 2 in this format of the response.
  uint8_t  major_version;     // major version number of the mobile app, i.e. 2
  uint8_t  minor_version;     // minfo version number of the mobile app, i.e. 0
  uint8_t  bugfix_version;    // bugfix version number of the mobile app, i.e. 1

  //! Pebble Protocol capabilities that the other side supports
  CommSessionCapability protocol_capabilities;
};

static const uint16_t SESSION_REMOTE_VERSION_ENDPOINT_ID = 0x0011;

static void prv_comm_session_perform_version_request_bg_cb(void *data) {
  CommSession *session = (CommSession *) data;
  const uint8_t command = CommSessionVersionCommandRequest;
  // No need to check validity of session here, comm_session_send_data already does this
  comm_session_send_data(session, SESSION_REMOTE_VERSION_ENDPOINT_ID,
                         &command, sizeof(command), COMM_SESSION_DEFAULT_TIMEOUT);
}

static void prv_schedule_request(CommSession *session) {
  bt_lock();
  if (!comm_session_is_valid(session)) {
    session = NULL;
    goto unlock;
  }
unlock:
  bt_unlock();

  if (session) {
    launcher_task_add_callback(prv_comm_session_perform_version_request_bg_cb, session);
  }
}

static void prv_handle_phone_versions_response(CommSession *session,
                                               const uint8_t *data, size_t length) {
  int request_version = 0;

  // Check which version of the response we are being given based on the length of the
  // message that the callback was given.
  if (length >= sizeof(struct VersionsPhoneResponseV3)) {
    request_version = 3;
  } else if (length >= sizeof(struct VersionsPhoneResponseV2)) {
    request_version = 2;
  } else if (length >= sizeof(struct VersionsPhoneResponseV1)) {
    request_version = 1;
  } else {
    PBL_LOG_ERR("Invalid version request");
    return;
  }

  struct VersionsPhoneResponseV3 *response = (struct VersionsPhoneResponseV3 *)data;

  CommSessionCapability capability_flags = 0;

  // If this is an old V1 response, use defaults for new fields introduced since V1
  // NOTE: The 1.X Android mobile app has a bug which causes it to send double length responses -
  // where the response message is repeated twice. This results in the
  // CommSessionVersionCommandResponse byte (value of 1) for the second copy landing in the
  // response_version field. That is why we only accept when this field is exactly 2, otherwise we
  // treat it as a V1 response.
  if (request_version >= 2) {
    PBL_LOG_DBG("Connected to Mobile App %"PRIu8 ".%"PRIu8 "-%"PRIu8,
            response->major_version, response->minor_version, response->bugfix_version);

    // For 3.X mobile applications, they will return additional bits in their response to correspond
    // to supporting certain endpoints over their deprecated counterparts, so assign them here after
    // the check for 2.X.
    if (request_version >= 3) {
      capability_flags = response->protocol_capabilities;
    }
  }

  comm_session_set_capabilities(session, capability_flags);
  const uint32_t platform_bits = ntohl(response->platform_bitfield);

  const bool is_system = comm_session_is_system(session);
  PBL_LOG_INFO("Phone app: is_system=%u, plf=0x%"PRIx32", capabilities=0x%"PRIx32,
          is_system, platform_bits, (uint32_t)capability_flags);

  // Only emit for the Pebble app, not 3rd party companion apps:
  if (is_system) {
    PebbleEvent event = {
      .type = PEBBLE_REMOTE_APP_INFO_EVENT,
      .bluetooth.app_info_event = {
        .os = (platform_bits & RemoteBitmaskOS),
      },
    };
    event_put(&event);
  }
}

void session_remote_version_protocol_msg_callback(CommSession *session_ref,
                                                  const uint8_t *data, size_t length) {
  switch (data[0]) {
    case CommSessionVersionCommandResponse: {
      prv_handle_phone_versions_response(session_ref, data + 1, length - 1);
      break;
    }

    default:
      PBL_LOG_D_ERR(LOG_DOMAIN_COMM, "Invalid message received. First byte is %u", data[0]);
      break;
  }
}

//! bt_lock() is expected to be taken by the caller!
void session_remote_version_start_requests(CommSession *session) {
  // Ask for the phone's version + capabilities:
  prv_schedule_request(session);
}
