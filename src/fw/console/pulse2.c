/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#ifdef CONFIG_PULSE_EVERYWHERE

#include "pulse.h"
#include "pulse2_reliable_retransmit_timer.h"
#include "pulse2_transport_impl.h"
#include "pulse_internal.h"

#include "console/cobs.h"
#include "console/console_internal.h"
#include "console/control_protocol.h"
#include "console/control_protocol_impl.h"
#include "console/dbgserial.h"
#include <pbl/drivers/rtc.h>
#include <pbl/drivers/task_watchdog.h>
#include "kernel/pbl_malloc.h"
#include "kernel/pebble_tasks.h"
#include "pbl/mcu/interrupts.h"
#include "pbl/kernel/mutex.h"
#include "pbl/services/regular_timer.h"
#include "system/passert.h"
#include "pbl/util/attributes.h"
#include "pbl/util/crc32.h"
#include "pbl/util/likely.h"
#include "pbl/util/math.h"
#include "util/net.h"
#include "pbl/util/size.h"

#include "FreeRTOS.h"
#include "pbl/kernel/msgq.h"
#include "pbl/kernel/sem.h"
#include "task.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#define LCP_PROTOCOL_NUMBER (0xC021)

#define FRAME_MAX_SEND_SIZE PULSE_MAX_RECEIVE_UNIT
#define RX_QUEUE_SIZE (PULSE_MAX_RECEIVE_UNIT * 3)
#define RX_MAX_FRAME_SIZE (PULSE_MAX_RECEIVE_UNIT + PULSE_MIN_FRAME_LENGTH)

#define FRAME_DELIMITER '\x55'
#define LINK_HEADER_LEN sizeof(net16)


// Link Control Protocol
// =====================

static void prv_on_lcp_up(PPPControlProtocol *this) {
#define ON_PACKET(...)
#define ON_INIT(...)
#define ON_LINK_STATE_CHANGE(ON_UP, ON_DOWN) ON_UP();
#include "console/pulse2_transport_registry.def"
#undef ON_PACKET
#undef ON_INIT
#undef ON_LINK_STATE_CHANGE
}

static void prv_on_lcp_down(PPPControlProtocol *this) {
#define ON_PACKET(...)
#define ON_INIT(...)
#define ON_LINK_STATE_CHANGE(ON_UP, ON_DOWN) ON_DOWN();
#include "console/pulse2_transport_registry.def"
#undef ON_PACKET
#undef ON_INIT
#undef ON_LINK_STATE_CHANGE
}

static void prv_on_code_reject(PPPControlProtocol *this,
                               struct LCPPacket *packet) {
  // TODO
}

static void prv_on_protocol_reject(PPPControlProtocol *this,
                                   struct LCPPacket *packet) {
  // TODO
}

static void prv_on_echo_request(PPPControlProtocol *this,
                                struct LCPPacket *packet) {
  if (this->state->link_state == LinkState_Opened) {
    struct LCPPacket *reply = pulse_link_send_begin(this->protocol_number);
    memcpy(reply, packet, ntoh16(packet->length));
    reply->code = ControlCode_EchoReply;
    pulse_link_send(reply, ntoh16(packet->length));
  }
}

static void prv_on_echo_reply(PPPControlProtocol *this,
                              struct LCPPacket *packet) {
  // TODO
}

static bool prv_handle_extended_lcp_codes(PPPControlProtocol *this,
                                          LCPPacket *packet) {
  switch (packet->code) {
    case ControlCode_ProtocolReject:
      prv_on_protocol_reject(this, packet);
      return true;
    case ControlCode_EchoRequest:
      prv_on_echo_request(this, packet);
      return true;
    case ControlCode_EchoReply:
      prv_on_echo_reply(this, packet);
      return true;
    case ControlCode_DiscardRequest:
      return true;
    default:
      return false;
  }
}

static PPPControlProtocolState s_lcp_state = {};

static PPPControlProtocol s_lcp_protocol = {
  .protocol_number = LCP_PROTOCOL_NUMBER,
  .state = &s_lcp_state,
  .on_this_layer_up = prv_on_lcp_up,
  .on_this_layer_down = prv_on_lcp_down,
  .on_receive_code_reject = prv_on_code_reject,
  .on_receive_unrecognized_code = prv_handle_extended_lcp_codes,
};

PPPControlProtocol * const PULSE2_LCP = &s_lcp_protocol;

static void prv_lcp_handle_unknown_protocol(uint16_t protocol, void *body,
                                            size_t body_len) {
  // TODO: send Protocol-Reject
}

static void prv_lcp_on_packet(void *packet, size_t length) {
  ppp_control_protocol_handle_incoming_packet(PULSE2_LCP, packet, length);
}


// Data link layer
// ===============

// PULSE task
// ----------
//
// This task handles both the processing of bytes received over dbgserial and
// running the reliable transport receive expiry timer.

static TaskHandle_t s_pulse_task_handle;
static PBL_MSGQ_DEFINE(s_pulse_task_queue, sizeof(uint8_t), RX_QUEUE_SIZE);
// Wake up the PULSE task to process the receive queue or start the timer.
static PBL_SEM_DEFINE(s_pulse_task_service_semaphore, 0, 1);
static volatile bool s_pulse_task_idle = true;

static uint8_t s_current_rx_frame[RX_MAX_FRAME_SIZE];

static PBL_MUTEX_DEFINE(s_tx_buffer_mutex);
static char s_tx_buffer[MAX_SIZE_AFTER_COBS_ENCODING(
        FRAME_MAX_SEND_SIZE + PULSE_MIN_FRAME_LENGTH) + COBS_OVERHEAD(FRAME_MAX_SEND_SIZE)];

// Lock for exclusive access to the reliable timer state.
static PBL_MUTEX_DEFINE(s_reliable_timer_state_lock);
// Ticks since boot for timer expiry if timer is pending, or 0 if not pending.
static volatile RtcTicks s_reliable_timer_expiry_time_tick;
static volatile uint8_t s_reliable_timer_sequence_number;

static void prv_process_received_frame(size_t frame_length) {
  if (frame_length < PULSE_MIN_FRAME_LENGTH || frame_length == SIZE_MAX) {
    // Decoding failed, this frame is bogus
    return;
  }

  uint32_t fcs;
  if (crc32(CRC32_INIT, s_current_rx_frame, frame_length) == CRC32_RESIDUE) {
    net16 protocol_be;
    memcpy(&protocol_be, s_current_rx_frame, sizeof(protocol_be));
    uint16_t protocol = ntoh16(protocol_be);
    void *body = &s_current_rx_frame[sizeof(protocol_be)];
    size_t body_len = frame_length - sizeof(protocol_be) - sizeof(fcs);
    switch (protocol) {
      case LCP_PROTOCOL_NUMBER:
        prv_lcp_on_packet(body, body_len);
        break;
#define ON_PACKET(NUMBER, HANDLER) \
      case NUMBER: \
        HANDLER(body, body_len); \
        break;
#define ON_INIT(...)
#define ON_LINK_STATE_CHANGE(...)
#include "console/pulse2_transport_registry.def"
#undef ON_PACKET
#undef ON_INIT
#undef ON_LINK_STATE_CHANGE
      default:
        prv_lcp_handle_unknown_protocol(protocol, body, body_len);
    }
  }
}

void pulse2_reliable_retransmit_timer_start(unsigned int timeout_ms,
                                            uint8_t sequence_number) {
  pbl_mutex_lock(&s_reliable_timer_state_lock, PBL_FOREVER);
  RtcTicks timeout_ticks = timeout_ms * RTC_TICKS_HZ / 1000;
  s_reliable_timer_expiry_time_tick = rtc_get_ticks() + timeout_ticks;
  s_reliable_timer_sequence_number = sequence_number;
  // Wake up the PULSE task to get it to notice the newly-started timer.
  pbl_sem_give(&s_pulse_task_service_semaphore);
  pbl_mutex_unlock(&s_reliable_timer_state_lock);
}

void pulse2_reliable_retransmit_timer_cancel(void) {
  pbl_mutex_lock(&s_reliable_timer_state_lock, PBL_FOREVER);
  s_reliable_timer_expiry_time_tick = 0;
  // No need to wake up the PULSE task. It will notice that the timer was
  // cancelled when it wakes up to service the timer.
  pbl_mutex_unlock(&s_reliable_timer_state_lock);
}

// Check the state of the timer.
//
// If there is no timer running, portMAX_DELAY is returned.
//
// If there is a timer running but it has not expired yet, the number of ticks
// (not milliseconds) remaining before the timer expires is returned.
//
// If there is a timer running that has expired, the timer state is cleared so
// that subsequent calls do not expire the same timer twice, sequence_number is
// filled with the sequence number of the expired timer, and 0 is returned.
static TickType_t prv_poll_timer(uint8_t *const sequence_number) {
  pbl_mutex_lock(&s_reliable_timer_state_lock, PBL_FOREVER);
  RtcTicks timer_expiry_tick = s_reliable_timer_expiry_time_tick;
  TickType_t timeout = portMAX_DELAY;
  if (timer_expiry_tick) {  // A timer is pending
    RtcTicks now = rtc_get_ticks();
    if (now >= timer_expiry_tick) {  // Timer has expired
      // Clear the timer pending state.
      s_reliable_timer_expiry_time_tick = 0;
      timeout = 0;
      *sequence_number = s_reliable_timer_sequence_number;
    } else {
      _Static_assert(pdMS_TO_TICKS(1000) == RTC_TICKS_HZ,
                     "RtcTicks uses different units than FreeRTOS ticks");
      timeout = timer_expiry_tick - now;
    }
  }
  pbl_mutex_unlock(&s_reliable_timer_state_lock);
  return timeout;
}

static void prv_pulse_task_feed_watchdog(void) {
  task_watchdog_bit_set(PebbleTask_PULSE);
}

static void prv_pulse_task_idle_timer_callback(void* data) {
  if (s_pulse_task_idle && pbl_msgq_num_used(&s_pulse_task_queue) == 0) {
    prv_pulse_task_feed_watchdog();
  }
}

static void prv_pulse_task_main(void *unused) {
  task_watchdog_mask_set(PebbleTask_PULSE);

  static RegularTimerInfo idle_watchdog_timer = {
    .cb = prv_pulse_task_idle_timer_callback
  };
  regular_timer_add_seconds_callback(&idle_watchdog_timer);

  CobsDecodeContext frame_decode_ctx;
  cobs_streaming_decode_start(&frame_decode_ctx, s_current_rx_frame,
                              RX_MAX_FRAME_SIZE);

  while (true) {
    uint8_t timer_sequence_number;
    TickType_t timeout = prv_poll_timer(&timer_sequence_number);

    if (timeout && pbl_msgq_num_used(&s_pulse_task_queue) == 0) {
      s_pulse_task_idle = true;
      pbl_sem_take(&s_pulse_task_service_semaphore, PBL_TICKS(timeout));
      s_pulse_task_idle = false;

      // Read the timer state again in case it changed while we were waiting.
      timeout = prv_poll_timer(&timer_sequence_number);
    }

    // Even if the timer expired, drain the received bytes queue first.
    // We don't want to risk the queue filling up while the timer
    // handler is running.
    char c;
    while (pbl_msgq_get(&s_pulse_task_queue, &c, PBL_NO_WAIT) == 0) {
      if (UNLIKELY(c == FRAME_DELIMITER)) {
        size_t decoded_length = cobs_streaming_decode_finish(&frame_decode_ctx);
        prv_process_received_frame(decoded_length);
        cobs_streaming_decode_start(&frame_decode_ctx, s_current_rx_frame,
                                    RX_MAX_FRAME_SIZE);
        // Break out after processing one complete frame so that we handle
        // the timer and kick the watchdog within a reasonable amount of
        // time, even if the queue is filling as fast as we can drain it.
        break;
      } else {
        if (c == '\0') {
          c = FRAME_DELIMITER;
        }
        cobs_streaming_decode(&frame_decode_ctx, c);
      }
    }

    // Finally handle the timer.
    if (timeout == 0) {
      pulse2_reliable_retransmit_timer_expired_handler(timer_sequence_number);
    }

    prv_pulse_task_feed_watchdog();
  }
}

static void prv_forge_terminate_ack(void) {
  // Send an LCP Terminate-Ack without invoking the control_protocol
  // API since we need to send these packets from precarious situations
  // when the OS and PULSE may not have been initialized yet.
  uint8_t *packet = pulse_link_send_begin(LCP_PROTOCOL_NUMBER);
  packet[0] = 6;  // Code: Terminate-Ack
  packet[1] = 255;  // Identifier
  packet[2] = 0;  // Length MSB
  packet[3] = 4;  // Length LSB
  pulse_link_send(packet, 4);
}

void pulse_early_init(void) {
  // Forge an LCP Terminate-Ack packet to synchronize the host's state
  // in case we crashed without terminating the connection.
  prv_forge_terminate_ack();
}

void pulse_init(void) {
}

void pulse_start(void) {
  TaskParameters_t task_params = {
    .pvTaskCode = prv_pulse_task_main,
    .pcName = "PULSE",
    .usStackDepth = 1024 / sizeof( StackType_t ),
    .uxPriority = (tskIDLE_PRIORITY + 3) | portPRIVILEGE_BIT,
    .puxStackBuffer = NULL,
  };

  pebble_task_create(PebbleTask_PULSE, &task_params, &s_pulse_task_handle);

  // FIXME: the initializers could be run more than once if pulse_start is
  // called more than one time. These initializers can't be run during
  // pulse_init since that runs even earlier than NewTimer init, which is
  // required for LCP to initialize.
  ppp_control_protocol_init(PULSE2_LCP);
#define ON_PACKET(...)
#define ON_INIT(INITIALIZER) INITIALIZER();
#define ON_LINK_STATE_CHANGE(...)
#include "console/pulse2_transport_registry.def"
#undef ON_PACKET
#undef ON_INIT
#undef ON_LINK_STATE_CHANGE

  serial_console_set_state(SERIAL_CONSOLE_STATE_PULSE);
  ppp_control_protocol_lower_layer_is_up(PULSE2_LCP);
  ppp_control_protocol_open(PULSE2_LCP);
}

void pulse_end(void) {
  ppp_control_protocol_close(PULSE2_LCP, PPPCPCloseWait_WaitForClosed);
}

static void prv_assert_tx_buffer(void *buf) {
  // Ensure the buffer is actually a PULSE transmit buffer
  bool buf_valid = false;
  if (buf == s_tx_buffer + COBS_OVERHEAD(FRAME_MAX_SEND_SIZE) + LINK_HEADER_LEN) {
    buf_valid = true;
  }
  PBL_ASSERT(buf_valid, "Buffer is not from the PULSE transmit buffer pool");
}

void pulse_handle_character(char c, bool *should_context_switch) {
  pbl_msgq_put(&s_pulse_task_queue, &c, PBL_NO_WAIT);
  pbl_sem_give(&s_pulse_task_service_semaphore);
  *should_context_switch = false;
}

static bool prv_safe_to_touch_mutex(void) {
  return !(portIN_CRITICAL() || mcu_state_is_isr() ||
           xTaskGetSchedulerState() != taskSCHEDULER_RUNNING);
}

void *pulse_link_send_begin(const uint16_t protocol) {
  if (prv_safe_to_touch_mutex()) {
    pbl_mutex_lock(&s_tx_buffer_mutex, PBL_FOREVER);
  }

  net16 header = hton16(protocol);
  memcpy(s_tx_buffer + COBS_OVERHEAD(FRAME_MAX_SEND_SIZE),
         &header, sizeof(header));
  return s_tx_buffer + COBS_OVERHEAD(FRAME_MAX_SEND_SIZE) + sizeof(header);
}

void pulse_link_send(void *buf, const size_t payload_length) {
  prv_assert_tx_buffer(buf);
  PBL_ASSERT(payload_length <= FRAME_MAX_SEND_SIZE, "PULSE frame payload too long");

  // Rewind the pointer to the beginning of the buffer
  char *frame = ((char *) buf) - COBS_OVERHEAD(FRAME_MAX_SEND_SIZE) - LINK_HEADER_LEN;
  size_t length = LINK_HEADER_LEN + payload_length;
  uint32_t fcs = crc32(CRC32_INIT, frame + COBS_OVERHEAD(FRAME_MAX_SEND_SIZE),
                       length);

  memcpy(&frame[length + COBS_OVERHEAD(FRAME_MAX_SEND_SIZE)], &fcs, sizeof(fcs));
  length += sizeof(fcs);
  length = cobs_encode(frame, frame + COBS_OVERHEAD(FRAME_MAX_SEND_SIZE), length);

  // TODO: DMA
  dbgserial_putchar_lazy(FRAME_DELIMITER);
  for (size_t i = 0; i < length; ++i) {
    if (frame[i] == FRAME_DELIMITER) {
      dbgserial_putchar_lazy('\0');
    } else {
      dbgserial_putchar_lazy(frame[i]);
    }
  }
  dbgserial_putchar_lazy(FRAME_DELIMITER);

  if (prv_safe_to_touch_mutex()) {
    pbl_mutex_unlock(&s_tx_buffer_mutex);
  }
}

void pulse_link_send_cancel(void *buf) {
  prv_assert_tx_buffer(buf);
  pbl_mutex_unlock(&s_tx_buffer_mutex);
}

size_t pulse_link_max_send_size(void) {
  return FRAME_MAX_SEND_SIZE - LINK_HEADER_LEN;
}

#endif
