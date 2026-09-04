/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "pulse_logging.h"

#include "logging_private.h"
#include "kernel/pebble_tasks.h"

#include "console/pulse_protocol_impl.h"
#include "kernel/events.h"

#include "pbl/mcu/interrupts.h"
#include "pbl/mcu/privilege.h"
#include "pbl/util/attributes.h"
#include "pbl/util/circular_buffer.h"
#include "pbl/util/math.h"
#include "pbl/util/string.h"

#include "FreeRTOS.h"
#include "task.h"

#include <ctype.h>

//! This is the format for a PULSEv2 log message when sent out over the wire.
typedef struct PACKED MessageContents {
  uint8_t message_type;
  char src_filename[16];
  char log_level_char;
  unsigned char task_char;
  uint64_t time_ms;
  uint16_t line_number;
  //! Not null-terminated message contents
  char message[128];
} MessageContents;

typedef struct PACKED {
  uint64_t timestamp_ms;
  uint8_t log_level;
} BufferedLogInfo;

//! For ISR contexts, we can't write log message directly. Instead we write them to this
//! circular buffer and flush them out when the ISR is completed.
static CircularBuffer s_isr_log_buffer;

//! Underlying storage for s_isr_log_buffer
static uint8_t s_isr_log_buffer_storage[256];


static uint64_t prv_get_timestamp_ms(void) {
  time_t time_s;
  uint16_t time_ms;
  rtc_get_time_ms(&time_s, &time_ms);
  return ((uint64_t) time_s * 1000) + time_ms;
}

static size_t prv_serialize_log_header(MessageContents *contents,
                                       uint8_t log_level, uint64_t timestamp_ms, PebbleTask task,
                                       const char *src_filename, uint16_t src_line_number) {
  contents->message_type = 1; // Text

  // Log the log level and the current task+privilege level
  contents->log_level_char = pbl_log_get_level_char(log_level);
#ifdef CONFIG_LOG_TASK_PREFIX
  contents->task_char = pebble_task_get_char(task);
  if (mcu_state_is_privileged()) {
    contents->task_char = toupper(contents->task_char);
  }
#else
  contents->task_char = '-';
#endif

  contents->time_ms = timestamp_ms;

  // Obtain the filename
  strncpy(contents->src_filename, GET_FILE_NAME(src_filename), sizeof(contents->src_filename));

  // Obtain the line number
  contents->line_number = src_line_number;

  return offsetof(MessageContents, message);
}


//! Serialize a message into contents, returning the number of bytes used.
static size_t prv_serialize_log(MessageContents *contents,
                                uint8_t log_level, uint64_t timestamp_ms, PebbleTask task,
                                const char *src_filename, uint16_t src_line_number,
                                const char *message) {

  prv_serialize_log_header(contents, log_level, timestamp_ms, task,
                           src_filename, src_line_number);

  // Write the actual log message.
  strncpy(contents->message, message, sizeof(contents->message));

  size_t payload_length = MIN(sizeof(MessageContents),
                              offsetof(MessageContents, message) + strlen(message));

  return payload_length;
}

static void prv_send_pulse_packet(uint8_t log_level, const char *src_filename,
                                  uint16_t src_line_number, const char *message) {
  MessageContents *contents = pulse_push_send_begin(PULSE_PROTOCOL_LOGGING);

  const size_t payload_length = prv_serialize_log(
      contents, log_level, prv_get_timestamp_ms(), pebble_task_get_current(),
      src_filename, src_line_number, message);

  pulse_push_send(contents, payload_length);
}


static bool prv_isr_buffer_read_and_consume(void *buffer, size_t read_length) {
  portENTER_CRITICAL();

  const bool result = circular_buffer_copy(&s_isr_log_buffer, buffer, read_length);
  if (result) {
    circular_buffer_consume(&s_isr_log_buffer, read_length);
  }

  portEXIT_CRITICAL();

  return result;
}


static void prv_event_cb(void *data) {
  while (true) {
    // No need to worry about reading a partial message here, we write messages to the circular
    // buffer while disabling interrupts the whole time.

    uint32_t log_length;
    if (!prv_isr_buffer_read_and_consume(&log_length, sizeof(log_length))) {
      // No more messages to read if we can't read a length
      break;
    }

    if (log_length == sizeof(uint32_t)) {
      // We dropped a message, log a message to that effect

      MessageContents *contents = pulse_push_send_begin(PULSE_PROTOCOL_LOGGING);

      const size_t payload_length = prv_serialize_log(
          contents, LOG_LEVEL_ERROR, prv_get_timestamp_ms(), PebbleTask_Unknown,
          "", 0, "ISR Message Dropped!");

      pulse_push_send(contents, payload_length);
    } else {
      BufferedLogInfo log_info;
      prv_isr_buffer_read_and_consume(&log_info, sizeof(log_info));

      MessageContents *contents = pulse_push_send_begin(PULSE_PROTOCOL_LOGGING);

      const size_t header_length = prv_serialize_log_header(
          contents, log_info.log_level, log_info.timestamp_ms, PebbleTask_Unknown, "", 0);

      const size_t message_length = log_length - sizeof(uint32_t) - sizeof(BufferedLogInfo);
      prv_isr_buffer_read_and_consume(&contents->message, message_length);

      pulse_push_send(contents, header_length + message_length);
    }
  }
}

static void prv_enqueue_log_message(uint8_t log_level, const char *message) {
  const bool buffer_was_empty = (circular_buffer_get_read_space_remaining(&s_isr_log_buffer) == 0);

  // Need to prevent other interrupts from corrupting the log buffer while we're writing to it
  portENTER_CRITICAL();

  if (circular_buffer_get_write_space_remaining(&s_isr_log_buffer) < sizeof(uint32_t)) {
    // Completely out of space, can't do anything.
    portEXIT_CRITICAL();
    return;
  }

  const size_t message_length = MIN(strlen(message), 128);
  const uint32_t required_space = sizeof(uint32_t) + sizeof(BufferedLogInfo) + message_length;

  if (circular_buffer_get_write_space_remaining(&s_isr_log_buffer) < required_space) {
    // Not enough space for the full message, just write an empty message with only the length
    // word to indicate we're dropping the message.
    const uint32_t insufficient_space_length = sizeof(uint32_t);
    circular_buffer_write(&s_isr_log_buffer,
                          &insufficient_space_length, sizeof(insufficient_space_length));
  } else {
    circular_buffer_write(&s_isr_log_buffer, &required_space, sizeof(required_space));

    const BufferedLogInfo log_info = {
      .timestamp_ms = prv_get_timestamp_ms(),
      .log_level = log_level
    };
    circular_buffer_write(&s_isr_log_buffer, &log_info, sizeof(log_info));

    circular_buffer_write(&s_isr_log_buffer, message, message_length);
  }

  if (buffer_was_empty) {
    PebbleEvent e = {
      .type = PEBBLE_CALLBACK_EVENT,
      .callback = {
        .callback = prv_event_cb
      }
    };
    event_put_isr(&e);
  }

  portEXIT_CRITICAL();
}

void pulse_logging_init(void) {
  circular_buffer_init(&s_isr_log_buffer,
                       s_isr_log_buffer_storage, sizeof(s_isr_log_buffer_storage));
}

void pulse_logging_log(uint8_t log_level, const char* src_filename,
                       uint16_t src_line_number, const char* message) {
  if (portIN_CRITICAL() || mcu_state_is_isr() ||
      xTaskGetSchedulerState() == taskSCHEDULER_SUSPENDED) {
    // We're in a state where we can't immediately send the message, save it to an internal buffer
    // instead for later sending.
    prv_enqueue_log_message(log_level, message);
  } else {
    // Send the log line inline
    prv_send_pulse_packet(log_level, src_filename, src_line_number, message);
  }
}

void pulse_logging_log_buffer_flush(void) {
  prv_event_cb(NULL);
}

void pulse_logging_log_sync(uint8_t log_level, const char *src_filename,
                            uint16_t src_line_number, const char *message) {
  // Send the log line inline, even if we're in a critical section or ISR
  prv_send_pulse_packet(log_level, src_filename, src_line_number, message);
}

void *pulse_logging_log_sync_begin(
    uint8_t log_level, const char *src_filename, uint16_t src_line_number) {
  MessageContents *contents = pulse_push_send_begin(PULSE_PROTOCOL_LOGGING);
  prv_serialize_log_header(contents, log_level, prv_get_timestamp_ms(),
                           pebble_task_get_current(), src_filename,
                           src_line_number);
  contents->message[0] = '\0';
  return contents;
}

void pulse_logging_log_sync_append(void *ctx, const char *message) {
  MessageContents *contents = ctx;
  strncat(contents->message, message, sizeof(contents->message) - strlen(contents->message) - 1);
}

void pulse_logging_log_sync_send(void *ctx) {
  MessageContents *contents = ctx;
  size_t payload_length = MIN(
      sizeof(MessageContents),
      offsetof(MessageContents, message) + strlen(contents->message));
  pulse_push_send(contents, payload_length);
}
