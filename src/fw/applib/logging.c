/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include <pbl/drivers/rtc.h>
#include "syscall/syscall.h"

#include "logging/logging_private.h"
#include "kernel/kernel_applib_state.h"

#include "process_state/app_state/app_state.h"
#include "process_state/worker_state/worker_state.h"

#include "pebbleos/chip_id.h"

#include <pbl/logging/log_hashing.h>
#include <pbl/logging/logging.h>
#include "system/passert.h"
#include "util/net.h"
#include "pbl/util/string.h"

#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define NEW_LOG_HEADER "NL" NEW_LOG_VERSION
_Static_assert((CORE_ID_MAIN_MCU & PACKED_CORE_MASK) == CORE_ID_MAIN_MCU, "Core number invalid");
#define str(s) xstr(s)
#define xstr(s) #s

#ifdef CONFIG_LOG_HASHED
// Define the .log_string section format.
static const char prv_NewLogHeader[] __attribute__((nocommon, used, section(".log_string.header")))
    = NEW_LOG_HEADER "=<file>:<line>:<level>:<color>:<msg>,"\
                     "CORE_ID=" str(CORE_ID_MAIN_MCU) ",CORE_NAME=Tintin";

// Confirm the size calculations. If these fail, update tools/loghashing/check_elf_log_strings.py
// We can't currently handle 64 bit values.
_Static_assert(sizeof(long int)  <= 4, "long int larger than expected");
_Static_assert(sizeof(size_t)    <= 4, "size_t larger than expected");
_Static_assert(sizeof(ptrdiff_t) <= 4, "ptrdiff_t larger than expected");

#endif

// -------------------------------------------------------------------------------------------
// If we should use a default log message (because stack space is too limited to use sprintf)
// then copy it into 'msg' and return true
static bool prv_use_default_log_msg(LogBinaryMessage *msg, const int max_message_length) {

  // We want to avoid vnsiprintf if we don't have sufficient stack space, so fill in
  //  a default log message
  uint32_t stack_space = sys_stack_free_bytes();
  if (stack_space < LOGGING_MIN_STACK_FOR_SPRINTF) {
    strncpy(msg->message, LOGGING_STACK_FULL_MSG, max_message_length);
    msg->message[max_message_length-1] = 0;
    msg->message_length = strlen(msg->message);
    return true;
  } else {
    return false;
  }
}


// -------------------------------------------------------------------------------------------
static void prv_sprintf_to_msg(LogBinaryMessage *msg, const uint32_t max_message_len,
                const char* fmt, va_list fmt_args) {

  int message_length = vsniprintf(msg->message + msg->message_length,
              max_message_len - msg->message_length, fmt, fmt_args);
  msg->message_length += message_length;
  if (msg->message_length > max_message_len) {
    msg->message_length = max_message_len;
  }
}


// -------------------------------------------------------------------------------------------
int pbl_log_binary_format(char* buffer, int buffer_len,
                          const uint8_t log_level,
                          const char* src_filename_path, int src_line_number,
                          const char* fmt, va_list args) {

  PBL_ASSERTN((unsigned int) buffer_len > sizeof(LogBinaryMessage));

  LogBinaryMessage* msg = (LogBinaryMessage*) buffer;

  time_t time_seconds = sys_get_time();
  msg->timestamp = htonl(time_seconds);

  msg->log_level = log_level;
  msg->line_number = htons(src_line_number & 0xffff);
  msg->message_length = 0;

  // Ensure we only send the last 15 characters of a filename
  const char* filename = GET_FILE_NAME(src_filename_path);
  int filename_length = strlen(filename);
  if (filename_length > 15) {
    // If we have to truncate, truncate at the beginning as opposed to the end.
    filename = filename + (filename_length - 15);
  }
  strncpy(msg->filename, filename, sizeof(msg->filename));

  // Copy the log message into the struct and set the message_length param.
  const int max_message_length = buffer_len - sizeof(LogBinaryMessage);

  // Use vnsiprintf only if we don't have sufficient stack space
  if (!prv_use_default_log_msg(msg, max_message_length)) {
    prv_sprintf_to_msg(msg, max_message_length, fmt, args);
  }

  return sizeof(*msg) + msg->message_length;
}

int pbl_log_get_bin_format(char* buffer, int buffer_len, const uint8_t log_level,
    const char* src_filename_path, int src_line_number, const char* fmt, ...) {
  va_list args;
  va_start(args, fmt);
  int len =  pbl_log_binary_format(buffer, buffer_len, log_level, src_filename_path,
      src_line_number, fmt, args);
  va_end(args);
  return (len);
}


// Return a pointer to the LogState to use. The LogState contains the buffers for formatting the log message.
// There are two possible LogState instances: one for the app task and one for all other (privileged) tasks (which
// is guarded by a mutex).
// Returns NULL if a logging operation is already in progress
static LogState *prv_get_log_state() {
  LogState* log_state = NULL;

  PebbleTask task = pebble_task_get_current();
  if (task == PebbleTask_App) {
    log_state = app_state_get_log_state();
  } else if (task == PebbleTask_Worker) {
    log_state = worker_state_get_log_state();
  }

  if (log_state) {
    if (log_state->in_progress) {
      return NULL;
    }
    log_state->in_progress = true;
    return log_state;
  } else {
    return kernel_applib_get_log_state();
  }
}

// Release the LogState buffer obtained by prv_get_log_state()
static void prv_release_log_state(LogState *state) {
  PebbleTask task = pebble_task_get_current();
  if (task == PebbleTask_App || task == PebbleTask_Worker) {
    state->in_progress = false;
  } else {
    kernel_applib_release_log_state(state);
  }
}


static void prv_log_internal(bool async, uint8_t log_level, const char* src_filename,
                         int src_line_number, const char* fmt, va_list args) {
  LogState *state = prv_get_log_state();
  if (!state) {
    return;
  }

  va_list bin_args;
  va_copy(bin_args, args);

  pbl_log_binary_format(state->buffer, sizeof(state->buffer), log_level, src_filename, src_line_number, fmt, bin_args);
  sys_pbl_log((LogBinaryMessage*) state->buffer, async);

  va_end(bin_args);
  prv_release_log_state(state);
}

#ifdef CONFIG_LOG_HASHED

void pbl_log_hashed_sync(const uint32_t packed_loghash, ...) {
  va_list fmt_args;
  va_start(fmt_args, packed_loghash);

  pbl_log_hashed_vargs(false, CORE_ID_MAIN_MCU, packed_loghash, fmt_args);

  va_end(fmt_args);
}


void pbl_log_hashed_async(const uint32_t packed_loghash, ...) {
  va_list fmt_args;
  va_start(fmt_args, packed_loghash);

  pbl_log_hashed_vargs(true, CORE_ID_MAIN_MCU, packed_loghash, fmt_args);

  va_end(fmt_args);
}

// Core Number must be shifted to the correct position.
void pbl_log_hashed_core(const uint32_t core_number, const uint32_t packed_loghash, ...) {
  va_list fmt_args;
  va_start(fmt_args, packed_loghash);

  pbl_log_hashed_vargs(true, core_number, packed_loghash, fmt_args);

  va_end(fmt_args);
}

// Core Number must be shifted to the correct position.
void pbl_log_hashed_vargs(const bool async, const uint32_t core_number,
                          const uint32_t packed_loghash, va_list fmt_args) {

  LogState *state = prv_get_log_state();
  if (!state) {
    return;
  }

  unsigned num_fmt_conversions = (packed_loghash >> PACKED_NUM_FMT_OFFSET) & PACKED_NUM_FMT_MASK;
  unsigned str_index_1 = (packed_loghash >> PACKED_STR1FMT_OFFSET) & PACKED_STR1FMT_MASK;
  unsigned str_index_2 = (packed_loghash >> PACKED_STR2FMT_OFFSET) & PACKED_STR2FMT_MASK;
  unsigned level = (packed_loghash >> PACKED_LEVEL_OFFSET) & PACKED_LEVEL_MASK;

  // Add the core number to the hash. This won't matter once we go full binary.
  unsigned hash = ((packed_loghash >> PACKED_HASH_OFFSET) & PACKED_HASH_MASK) | core_number;

  int buffer_len = sizeof(state->buffer);
  char* buffer = state->buffer;

  // Fill in the log message fields
  LogBinaryMessage* msg = (LogBinaryMessage*) buffer;

  time_t time_seconds = sys_get_time();
  msg->timestamp = htonl(time_seconds);
  msg->message_length = 0;

  /*
   * The file name and line number are stored in the log_strings section
   * so the waf console displays it.
   */
  msg->line_number = 0;
  msg->filename[0] = '\0';

  /*
   * Unpack the log level
   * Duplicate _VERBOSE -- let's have a reasonable entry for every value should something go
   * wrong on the packing end.
   */
  const uint8_t level_map[8] = { LOG_LEVEL_ALWAYS, LOG_LEVEL_ERROR, LOG_LEVEL_WARNING,
                                 LOG_LEVEL_INFO, LOG_LEVEL_DEBUG, LOG_LEVEL_DEBUG_VERBOSE,
                                 LOG_LEVEL_DEBUG_VERBOSE, LOG_LEVEL_DEBUG_VERBOSE };
  msg->log_level = level_map[level];

  /*
   * Copy the log message into the struct and set the message_length param.
   * Use a temporary to handle the case
   * where we log over 256 characters and the result of vsniprint
   * won't fit into msg->message_length which is only a uint8_t.
   */
  const int max_message_length = buffer_len - sizeof(LogBinaryMessage);

  // Only use vsniprintf if we have sufficient stack space
  if (!prv_use_default_log_msg(msg, max_message_length)) {

    // add the hashed value for the 'New Log' message
    sprintf(msg->message, "NL:%x", hash);
    msg->message_length += strlen(msg->message);

    if (num_fmt_conversions != 0) {
      char expanded_fmt_buffer[64];
      // Fix the fmt string to include spaces before each % and `` around %s
      memset(expanded_fmt_buffer, 0, sizeof(expanded_fmt_buffer));
      for (unsigned index = 0; index < num_fmt_conversions; ++index) {
        if (((str_index_1 != 0) && ((index + 1) == str_index_1)) ||
            ((str_index_2 != 0) && ((index + 1) == str_index_2))) {
          strcat(expanded_fmt_buffer, " `%s`");
        } else {
          strcat(expanded_fmt_buffer, " %x"); // add a space
        }
      }

      prv_sprintf_to_msg(msg, max_message_length, expanded_fmt_buffer, fmt_args);
    }
  }

  sys_pbl_log((LogBinaryMessage*) state->buffer, async);
  prv_release_log_state(state);
}

#endif /* CONFIG_LOG_HASHED */

void pbl_log_vargs(uint8_t log_level, const char *src_filename,
                   int src_line_number, const char *fmt, va_list args) {
  const bool async = true;
  prv_log_internal(async, log_level, src_filename, src_line_number, fmt, args);
}

void pbl_log(uint8_t log_level, const char* src_filename,
             int src_line_number, const char* fmt, ...) {
  va_list args;
  va_start(args, fmt);
  const bool async = true;
  prv_log_internal(async, log_level, src_filename, src_line_number, fmt, args);
  va_end(args);
}

void pbl_log_sync(uint8_t log_level, const char* src_filename,
                  int src_line_number, const char* fmt, ...) {
  va_list args;
  va_start(args, fmt);

  const bool async = false;
  prv_log_internal(async, log_level, src_filename, src_line_number, fmt, args);

  va_end(args);
}
