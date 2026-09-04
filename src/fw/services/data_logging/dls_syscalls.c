/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "pbl/services/data_logging/data_logging_service.h"
#include "pbl/services/data_logging/dls_private.h"

#include "syscall/syscall_internal.h"
#include <pbl/logging/logging.h>

PBL_LOG_MODULE_DECLARE(service_data_logging, CONFIG_SERVICE_DATA_LOGGING_LOG_LEVEL);

DEFINE_SYSCALL(DataLoggingSessionRef, sys_data_logging_create, uint32_t tag,
               DataLoggingItemType item_type, uint16_t item_size,
               void *buffer, bool resume) {
  // Apps allocate the circular buffer themselves in their own heap and hand the
  // pointer to the kernel via this syscall. Without validation, a malicious app
  // could point at kernel memory and turn dls_log into an arbitrary kernel write.
  if (PRIVILEGE_WAS_ELEVATED && buffer != NULL) {
    syscall_assert_userspace_buffer(buffer, DLS_SESSION_MIN_BUFFER_SIZE);
  }
  return dls_create_current_process(tag, item_type, item_size, buffer, resume);
}

DEFINE_SYSCALL(void, sys_data_logging_finish, DataLoggingSessionRef session_ref) {
  // dls_is_session_valid() (below) walks the kernel-owned s_logging_sessions
  // list to confirm the session pointer is one we handed out, so a forged
  // session_ref simply returns invalid and never reaches dls_finish().
  DataLoggingSession* session = (DataLoggingSession*)session_ref;

  if (!dls_is_session_valid(session)) {
    PBL_LOG_WRN("finish: Invalid session %p", session);
    return; // TODO: Return error code?
  }

  dls_finish(session);
}

DEFINE_SYSCALL(DataLoggingResult, sys_data_logging_log,
               DataLoggingSessionRef session_ref, void* data, uint32_t num_items) {
  DataLoggingSession* session = (DataLoggingSession*)session_ref;

  if (!dls_is_session_valid(session)) {
    PBL_LOG_WRN("log: Invalid session %p", session);
    return DATA_LOGGING_INVALID_PARAMS;
  }
  if (data == NULL) {
    PBL_LOG_WRN("log: NULL data pointer");
    return DATA_LOGGING_INVALID_PARAMS;
  }

  if (PRIVILEGE_WAS_ELEVATED) {
    syscall_assert_userspace_buffer(data, num_items * session->item_size);
  }

  return dls_log(session, data, num_items);
}
