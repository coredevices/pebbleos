/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#ifndef CONFIG_PULSE_EVERYWHERE

#include "pulse.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "console/cobs.h"
#include "console/console_internal.h"
#include "console/dbgserial.h"
#include "console/pulse_internal.h"
#include "console/pulse_llc.h"
#include "console/pulse_protocol_impl.h"
#include "kernel/pbl_malloc.h"
#include "pbl/kernel/mutex.h"
#include "pbl/services/new_timer/new_timer.h"
#include "pbl/services/system_task.h"
#include "system/passert.h"
#include "pbl/util/attributes.h"
#include "util/legacy_checksum.h"
#include "pbl/util/likely.h"
#include "pbl/util/math.h"
#include "pbl/util/size.h"

#define FRAME_POOL_SIZE (3)

#define FRAME_DELIMITER '\0'
#define LINK_HEADER_LEN (1)


typedef struct IncomingPulseFrame {
  uint16_t length;
  bool taken;
  char data[MAX_SIZE_AFTER_COBS_ENCODING(PULSE_MAX_RECEIVE_UNIT)];
} IncomingPulseFrame;

static IncomingPulseFrame *s_receive_buffers[FRAME_POOL_SIZE];
static IncomingPulseFrame *s_current_receive_buffer;

static CobsDecodeContext s_frame_decode_ctx;
static bool s_drop_rest_of_frame;

static PBL_MUTEX_DEFINE(s_tx_buffer_mutex);
static char s_tx_buffer[MAX_SIZE_AFTER_COBS_ENCODING(
        PULSE_MAX_SEND_SIZE + PULSE_MIN_FRAME_LENGTH) + COBS_OVERHEAD(PULSE_MAX_SEND_SIZE)];

typedef void (*ProtocolHandlerFunc)(void *packet, size_t length);
typedef void (*LinkStateChangedHandlerFunc)(PulseLinkState link_state);

typedef struct PACKED ProtocolHandler {
  uint8_t number;
  ProtocolHandlerFunc handler;
  LinkStateChangedHandlerFunc link_state_handler;
} ProtocolHandler;

static const ProtocolHandler s_supported_protocols[] = {
#define REGISTER_PROTOCOL(n, f1, f2) { \
    .number = (n), \
    .handler = (f1), \
    .link_state_handler = (f2) \
  },
#include "console/pulse_protocol_registry.def"
#undef REGISTER_PROTOCOL
};

static TimerID s_keepalive_timer = TIMER_INVALID_ID;


static void prv_reset_receive_buffer(IncomingPulseFrame *buf) {
  buf->length = 0;
  cobs_streaming_decode_start(&s_frame_decode_ctx, &buf->data,
                              sizeof(buf->data));
}

static IncomingPulseFrame* prv_take_receive_buffer(void) {
  IncomingPulseFrame *buf = NULL;
  for (unsigned int i = 0; i < ARRAY_LENGTH(s_receive_buffers); ++i) {
    if (s_receive_buffers[i]->taken == false) {
      buf = s_receive_buffers[i];
      buf->taken = true;
      prv_reset_receive_buffer(buf);
      return buf;
    }
  }
  return NULL;
}

static void prv_return_receive_buffer(IncomingPulseFrame *buf) {
  buf->taken = false;
}

static void prv_keepalive_timeout_expired(void *data) {
  pulse_end();
}

static void prv_reset_keepalive_timer(void) {
  if (s_keepalive_timer) {
    new_timer_start(s_keepalive_timer,
                    PULSE_KEEPALIVE_TIMEOUT_DECISECONDS * 100,
                    prv_keepalive_timeout_expired,
                    NULL,
                    TIMER_START_FLAG_FAIL_IF_EXECUTING);
  }
}

static void prv_handlers_notify_state_changed(PulseLinkState link_state) {
  for (unsigned int i = 0; i < ARRAY_LENGTH(s_supported_protocols); ++i) {
    s_supported_protocols[i].link_state_handler(link_state);
  }
}

void pulse_early_init(void) {
}

void pulse_init(void) {
}

void pulse_start(void) {
  for (unsigned int i = 0; i < ARRAY_LENGTH(s_receive_buffers); ++i) {
    s_receive_buffers[i] = kernel_malloc_check(sizeof(IncomingPulseFrame));
    prv_return_receive_buffer(s_receive_buffers[i]);
  }
  s_current_receive_buffer = prv_take_receive_buffer();
  s_drop_rest_of_frame = false;

  s_keepalive_timer = new_timer_create();
  PBL_ASSERTN(s_keepalive_timer != TIMER_INVALID_ID);

  pulse_init();

  serial_console_set_state(SERIAL_CONSOLE_STATE_PULSE);
  pulse_llc_send_link_opened_msg();
  prv_reset_keepalive_timer();
  prv_handlers_notify_state_changed(PulseLinkState_Open);
}

void pulse_end(void) {
  prv_handlers_notify_state_changed(PulseLinkState_Closed);
  pulse_llc_send_link_closed_msg();
  for (unsigned int i = 0; i < ARRAY_LENGTH(s_receive_buffers); ++i) {
    kernel_free(s_receive_buffers[i]);
  }
  s_current_receive_buffer = NULL;

  new_timer_delete(s_keepalive_timer);
  s_keepalive_timer = TIMER_INVALID_ID;

  dbgserial_restore_baud_rate();
  serial_console_set_state(SERIAL_CONSOLE_STATE_LOGGING);
}

static void prv_process_received_frame(void *frame_ptr) {
  IncomingPulseFrame *frame = frame_ptr;
  uint32_t fcs;
  // Comply with strict aliasing rules. The memcpy is optimized away.
  memcpy(&fcs, &frame->data[frame->length - sizeof(fcs)], sizeof(fcs));
  uint32_t crc = legacy_defective_checksum_memory(
      &frame->data, frame->length - sizeof(fcs));

  if (fcs == crc) {
    prv_reset_keepalive_timer();
    uint8_t protocol = (uint8_t)frame->data[0];
    bool protocol_found = false;
    for (unsigned int i = 0; i < ARRAY_LENGTH(s_supported_protocols); ++i) {
      if (s_supported_protocols[i].number == protocol) {
        protocol_found = true;
        s_supported_protocols[i].handler(
            &frame->data[sizeof(protocol)],
            frame->length - sizeof(protocol) - sizeof(fcs));
        break;
      }
    }
    if (!protocol_found) {
      pulse_llc_unknown_protocol_handler(
          protocol, &frame->data[sizeof(protocol)],
          frame->length - sizeof(protocol) - sizeof(fcs));
    }
  }
  prv_return_receive_buffer(frame);
}

static void prv_assert_tx_buffer(void *buf) {
  // Ensure the buffer is actually a PULSE transmit buffer
  bool buf_valid = false;
  if (buf == s_tx_buffer + COBS_OVERHEAD(PULSE_MAX_SEND_SIZE) + LINK_HEADER_LEN) {
    buf_valid = true;
  }
  PBL_ASSERT(buf_valid, "Buffer is not from the PULSE transmit buffer pool");
}

void pulse_handle_character(char c, bool *should_context_switch) {
  // TODO: discard a frame outright if a framing error occurs
  if (s_current_receive_buffer == NULL) {
    s_current_receive_buffer = prv_take_receive_buffer();
    if (s_current_receive_buffer == NULL) {
      // No buffers are available to store the char; drop it.
      if (c != FRAME_DELIMITER) {
        s_drop_rest_of_frame = true;
      }
      return;
    }
  }

  if (UNLIKELY(c == FRAME_DELIMITER)) {
    s_drop_rest_of_frame = false;
    size_t decoded_length = cobs_streaming_decode_finish(&s_frame_decode_ctx);
    if (decoded_length >= PULSE_MIN_FRAME_LENGTH && decoded_length < SIZE_MAX) {
      // Potentially valid frame; queue up for further processing.
      s_current_receive_buffer->length = decoded_length;
      system_task_add_callback_from_isr(prv_process_received_frame,
                                        s_current_receive_buffer,
                                        should_context_switch);
      // Prepare to receive the next character.
      s_current_receive_buffer = prv_take_receive_buffer();
    } else {
      // Not a valid frame; throw it away.
      prv_reset_receive_buffer(s_current_receive_buffer);
    }
  } else if (s_drop_rest_of_frame) {
    // The frame has already been found to be bad and we haven't yet
    // seen the start of the next frame.
  } else if (UNLIKELY(s_current_receive_buffer->length >=
             sizeof(s_current_receive_buffer->data))) {
    // Frame too long; invalid.
    s_drop_rest_of_frame = true;
    prv_reset_receive_buffer(s_current_receive_buffer);
  } else {
    if (!cobs_streaming_decode(&s_frame_decode_ctx, c)) {
      s_drop_rest_of_frame = true;
    }
  }
}

void *pulse_best_effort_send_begin(const uint8_t protocol) {
  pbl_mutex_lock(&s_tx_buffer_mutex, PBL_FOREVER);
  s_tx_buffer[COBS_OVERHEAD(PULSE_MAX_SEND_SIZE)] = protocol;

  // Expose only the payload of the message
  return s_tx_buffer + COBS_OVERHEAD(PULSE_MAX_SEND_SIZE) + 1;
}

void pulse_best_effort_send(void *buf, const size_t payload_length) {
  prv_assert_tx_buffer(buf);
  PBL_ASSERT(payload_length <= PULSE_MAX_SEND_SIZE, "PULSE frame payload too long");

  // Rewind the pointer to the beginning of the buffer
  char *frame = ((char *) buf) - COBS_OVERHEAD(PULSE_MAX_SEND_SIZE) - LINK_HEADER_LEN;
  size_t length = LINK_HEADER_LEN + payload_length;
  uint32_t fcs = legacy_defective_checksum_memory(
      frame + COBS_OVERHEAD(PULSE_MAX_SEND_SIZE), length);

  memcpy(&frame[length + COBS_OVERHEAD(PULSE_MAX_SEND_SIZE)], &fcs, sizeof(fcs));
  length += sizeof(fcs);
  length = cobs_encode(frame, frame + COBS_OVERHEAD(PULSE_MAX_SEND_SIZE), length);

  // TODO: DMA
  dbgserial_putchar_lazy(FRAME_DELIMITER);
  for (size_t i = 0; i < length; ++i) {
    dbgserial_putchar_lazy(frame[i]);
  }
  dbgserial_putchar_lazy(FRAME_DELIMITER);

  pbl_mutex_unlock(&s_tx_buffer_mutex);
}

void pulse_best_effort_send_cancel(void *buf) {
  prv_assert_tx_buffer(buf);
  pbl_mutex_unlock(&s_tx_buffer_mutex);
}

void pulse_change_baud_rate(uint32_t new_baud) {
  dbgserial_change_baud_rate(new_baud);
}
#endif
