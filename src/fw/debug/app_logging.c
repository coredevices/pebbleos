/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "pbl/util/attributes.h"
#include <pbl/logging/logging.h>
#include "applib/app_logging.h"

#include <stdint.h>

#include "logging/logging_private.h"
#include "kernel/util/stack_info.h"
#include "pbl/services/comm_session/session.h"
#include "syscall/syscall_internal.h"

static const uint16_t APP_LOGGING_ENDPOINT = 2006;

static AppLoggingMode s_app_logging_mode = AppLoggingDisabled;

bool app_log_is_bt_enabled(void) {
  return s_app_logging_mode == AppLoggingEnabled;
}

static const uint32_t MIN_STACK_FOR_SEND_DATA = 400;

DEFINE_SYSCALL(void, sys_app_log, size_t length, void *log_buffer) {
  if (PRIVILEGE_WAS_ELEVATED) {
    syscall_assert_userspace_buffer(log_buffer, length);
  }

  AppLogBinaryMessage *message = log_buffer;

  // First log to serial, we always do this.
  kernel_pbl_log_serial(&message->log_msg, false);

  // Now check to see if app logging is enabled over bluetooth.
  if (s_app_logging_mode == AppLoggingDisabled) {
    return;
  }

  // Then log to the app logging endpoint (if we have enough stack space)
  uint32_t stack_space = stack_free_bytes();
  if (stack_space > MIN_STACK_FOR_SEND_DATA) {
    CommSession *session = comm_session_get_system_session();
    if (session) {
      comm_session_send_data(session, APP_LOGGING_ENDPOINT, (uint8_t*)log_buffer, length, COMM_SESSION_DEFAULT_TIMEOUT);
    }
  }
}

void app_log_protocol_msg_callback(CommSession *session, const uint8_t *data, const size_t length) {
  typedef struct PACKED AppLogCommand {
    uint8_t commandType;
  } AppLogCommand;

  enum AppLogCommandType {
    APP_LOG_COMMAND_DISABLE_LOGGING = 0,
    APP_LOG_COMMAND_ENABLE_LOGGING = 1,
  };

  AppLogCommand *command = (AppLogCommand *)data;
  switch(command->commandType) {
  case APP_LOG_COMMAND_ENABLE_LOGGING:
    s_app_logging_mode = AppLoggingEnabled;
    break;
  case APP_LOG_COMMAND_DISABLE_LOGGING:
    s_app_logging_mode = AppLoggingDisabled;
    break;
  default:
    PBL_LOG_WRN("Invalid app log command 0x%x", command->commandType);
  }
}

