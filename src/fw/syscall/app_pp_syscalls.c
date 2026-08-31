/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "applib/app_message/app_message_internal.h"
#include "process_management/app_install_manager.h"
#include "process_management/app_manager.h"
#include "pbl/services/comm_session/app_session_capabilities.h"
#include "pbl/services/comm_session/session.h"
#include "syscall/syscall_internal.h"

#include <stdbool.h>
#include <stdint.h>

static bool prv_is_endpoint_allowed(uint16_t endpoint_id) {
  return (endpoint_id == APP_MESSAGE_ENDPOINT_ID);
}

DEFINE_SYSCALL(CommSession *, sys_app_pp_get_comm_session, void) {
  CommSession *app_session = NULL;
  comm_session_sanitize_app_session(&app_session);
  return app_session;
}

DEFINE_SYSCALL(bool, sys_app_pp_send_data, CommSession *session, uint16_t endpoint_id,
               const uint8_t* data, uint16_t length) {
  if (PRIVILEGE_WAS_ELEVATED) {
    syscall_assert_userspace_buffer(data, length);
  }
  if (!prv_is_endpoint_allowed(endpoint_id)) {
    syscall_failed();
  }

  comm_session_sanitize_app_session(&session);
  if (!session) {
    // No session connected that can serve the currently running app
    return false;
  }

  const AppInstallId app_id = app_manager_get_current_app_id();
  app_install_mark_prioritized(app_id, true /* can_expire */);

  // TODO: apply some heuristic to decide whether to put connection in fast mode or not:
  // https://pebbletechnology.atlassian.net/browse/PBL-21538
  comm_session_set_responsiveness(session, BtConsumerPpAppMessage, ResponseTimeMin,
                                  MIN_LATENCY_MODE_TIMEOUT_APP_MESSAGE_SECS);

  // FIXME: Let the app task wait indefinitely for now
  const uint32_t timeout_ms = ~0;
  return comm_session_send_data(session, endpoint_id, data, length, timeout_ms);
}

DEFINE_SYSCALL(bool, sys_app_pp_has_capability, CommSessionCapability capability) {
  return comm_session_current_app_session_cache_has_capability(capability);
}