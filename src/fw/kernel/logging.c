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

#include "pulse_logging.h"

#include "pebble_tasks.h"
#include "logging_private.h"
#include "util/stack_info.h"

#include "console/console_internal.h"
#include "console/prompt.h"
#include "console/serial_console.h"
#include "debug/advanced_logging.h"
#include "drivers/rtc.h"
#include "system/logging.h"

#include "mcu/interrupts.h"
#include "mcu/privilege.h"

#include "util/math.h"
#include "util/net.h"
#include "util/string.h"

#include "FreeRTOS.h"
#include "task.h"

#include <ctype.h>
#include <stdio.h>
#include <time.h>

#ifndef PBL_LOG_LEVEL
  #define PBL_LOG_LEVEL LOG_LEVEL_DEBUG
#endif

int g_pbl_log_level = PBL_LOG_LEVEL;
bool g_pbl_log_enabled = true;

static bool prv_check_serial_log_enabled(int level) {
  return (g_pbl_log_enabled) &&
         (level == LOG_LEVEL_ALWAYS ||
           (level <= g_pbl_log_level));
}

#if !PULSE_EVERYWHERE
#define TIMESTAMP_BUFFER_SIZE 40
static void prv_log_timestamp(void) {
  // Enough stack space to use sprintfs?
  uint32_t stack_space = stack_free_bytes();
  if (stack_space < LOGGING_MIN_STACK_FOR_SPRINTF) {
    serial_console_write_log_message(LOGGING_STACK_FULL_MSG);
    serial_console_write_log_message(" ");
    return;
  }

  char buffer[TIMESTAMP_BUFFER_SIZE];

  time_t time_seconds;
  uint16_t time_ms;
  rtc_get_time_ms(&time_seconds, &time_ms);
  struct tm time_seconds_calendar;
  gmtime_r(&time_seconds, &time_seconds_calendar);

  sniprintf(buffer, TIMESTAMP_BUFFER_SIZE, "%02u:%02u:%02u.%03u ",
      time_seconds_calendar.tm_hour, time_seconds_calendar.tm_min, time_seconds_calendar.tm_sec, time_ms);

  serial_console_write_log_message(buffer);
}

static void prv_log_serial(
    uint8_t log_level, const char* src_filename, int src_line_number, const char* message) {
  if (!serial_console_is_logging_enabled() && log_level != LOG_LEVEL_ALWAYS) {
    return;
  }

  // Log the log level and the current task+privilege level
  {
    unsigned char task_char = pebble_task_get_char(pebble_task_get_current());
    if (mcu_state_is_privileged()) {
      task_char = toupper(task_char);
    }

    char buffer[] = { pbl_log_get_level_char(log_level), ' ', task_char, ' ', 0 };
    serial_console_write_log_message(buffer);
  }

  // Start out with the timestamp
  prv_log_timestamp();

  // Write out the filename
  src_filename = GET_FILE_NAME(src_filename);
  serial_console_write_log_message(src_filename);

  // Write out the line number
  {
    char line_number_buffer[12];
    itoa_int(src_line_number, line_number_buffer, 10);
    serial_console_write_log_message(":");
    serial_console_write_log_message(line_number_buffer);
    serial_console_write_log_message("> ");
  }

  // Write the actual log message.
  serial_console_write_log_message(message);

  // Append our newlines and our trailing null
  serial_console_write_log_message("\r\n");
}
#endif // PULSE_EVERYWHERE

void kernel_pbl_log_serial(LogBinaryMessage *log_message, bool async) {
  if (!prv_check_serial_log_enabled(log_message->log_level)) {
    return;
  }

#if PULSE_EVERYWHERE
  if (async) {
    pulse_logging_log(log_message->log_level, log_message->filename,
                      htons(log_message->line_number), log_message->message);
  } else {
    pulse_logging_log_sync(
        log_message->log_level, log_message->filename,
        htons(log_message->line_number), log_message->message);
  }
#else
  prv_log_serial(log_message->log_level, log_message->filename,
                 htons(log_message->line_number), log_message->message);
#endif
}

void kernel_pbl_log_flash(LogBinaryMessage *log_message, bool async) {
  int length = sizeof(*log_message) + log_message->message_length;

  static const uint8_t FLASH_LOG_LEVEL = LOG_LEVEL_INFO;
  if (g_pbl_log_enabled &&
      (log_message->log_level == LOG_LEVEL_ALWAYS ||
       (log_message->log_level <= FLASH_LOG_LEVEL))) {
    pbl_log_advanced((const char*) log_message, length, async);
  }
}

void kernel_pbl_log(LogBinaryMessage* log_message, bool async) {
  kernel_pbl_log_serial(log_message, async);

  if (!portIN_CRITICAL() && !mcu_state_is_isr() &&
      xTaskGetSchedulerState() != taskSCHEDULER_SUSPENDED) {
    kernel_pbl_log_flash(log_message, async);
  }
}

void kernel_pbl_log_from_fault_handler(
    const char *src_filename, uint16_t src_line_number, const char *message) {
#if PULSE_EVERYWHERE
  pulse_logging_log_sync(LOG_LEVEL_ALWAYS, src_filename,
                         src_line_number, message);
#else
  serial_console_write_log_message(message);
  serial_console_write_log_message("\r\n");
#endif
}

void kernel_pbl_log_from_fault_handler_fmt(
    const char *src_filename, uint16_t src_line_number, char *buffer,
    unsigned int buffer_size, const char *fmt, ...) {
  va_list ap;
  va_start(ap, fmt);
  vsniprintf(buffer, buffer_size, fmt, ap);
  va_end(ap);

  kernel_pbl_log_from_fault_handler(src_filename, src_line_number, buffer);
}

// Serial Commands
///////////////////////////////////////////////////////////
void command_log_level_set(const char* level) {
  char buffer[32];
  g_pbl_log_level = atoi(level);
  prompt_send_response_fmt(buffer, 32, "Log level set to: %i", g_pbl_log_level);
}

void command_log_level_get(void) {
  char buffer[32];
  prompt_send_response_fmt(buffer, 32, "Log level: %i", g_pbl_log_level);
}

