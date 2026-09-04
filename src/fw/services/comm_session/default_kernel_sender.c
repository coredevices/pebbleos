/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "comm/bt_lock.h"
#include <pbl/drivers/rtc.h>
#include "kernel/pbl_malloc.h"
#include "pbl/services/comm_session/protocol.h"
#include "pbl/services/comm_session/session_send_buffer.h"
#include "pbl/services/comm_session/session_send_queue.h"
#include <pbl/logging/logging.h>
#include "pbl/util/attributes.h"
#include "pbl/util/likely.h"
#include "pbl/util/math.h"
#include "util/net.h"

#include "FreeRTOS.h"
#include "semphr.h"

PBL_LOG_MODULE_DECLARE(service_comm_session, CONFIG_SERVICE_COMM_SESSION_LOG_LEVEL);

typedef struct SendBuffer {
  //! Save some memory by making this a union.
  //! @note This is the first field, so that we can just cast between
  //! (SendBuffer *) and (SessionSendQueueJob *)
  union {
    //! The targeted session, this field is valid until ..._write_end has been called.
    CommSession *session;

    //! This fields is valid after ...write_end has returned.
    SessionSendQueueJob queue_job;
  };

  //! Length of payload[] in bytes
  size_t payload_buffer_length;

  //! It's tempting to use header.length, but this is big endian... :(
  size_t written_length;

  //! Number of bytes that have been consumed so far
  size_t consumed_length;

  struct PACKED {
    //! The remainder of this struct is the Pebble Protocol message (header + payload):
    PebbleProtocolHeader header;
    uint8_t payload[];
  };
} SendBuffer;

#define DEFAULT_KERNEL_SENDER_MAX_PAYLOAD_SIZE ((size_t)1024)

//! @note This does not include sizeof(SendBuffer) by design, to avoid letting the implementation
//! affect the maximum number of (smaller) Pebble Protocol messages can be allocated. For example,
//! the Audio endpoint likes to send out a stream of small Pebble Protocol messages. We don't want
//! to accidentally cut the max number when sizeof(SendBuffer) would increase for whatever reason.
//!
//! @note We leave it up to the caller of the exported comm_session_send_* APIs to implement a
//! retry mechanism when we are OOM. A lot of callers just implicitly assume things will work and
//! the payload get dropped on the floor.
//! TODO: I don't know where we stand heap wise on older platforms like spalding. We don't really
//! have any analytics in place to track this. Before changing the behavior, let's back it with
//! some data. For now ...  live and let live
#define DEFAULT_KERNEL_SENDER_MAX_BYTES_ALLOCATED \
  ((sizeof(PebbleProtocolHeader) + DEFAULT_KERNEL_SENDER_MAX_PAYLOAD_SIZE))

// -------------------------------------------------------------------------------------------------
//! Semaphore that is signaled when data has been consumed by the transport,
//! when it calls to comm_default_kernel_sender_consume(). This semaphore is used to block calls to
//! comm_session_send_buffer_begin_write() in case there is not enough space left.
//! @note This semaphore *must never* be taken when bt_lock() is held or deadlock will happen!
//! Giving the semaphore when bt_lock() is held is fine though.
static SemaphoreHandle_t s_default_kernel_sender_write_semaphore;

//! Total number of bytes worth of Pebble Protocol messages (incl. header) allocated by this module.
//! This excludes sizeof(SendBuffer), see comment with DEFAULT_KERNEL_SENDER_MAX_BYTES_ALLOCATED.
static size_t s_default_kernel_sender_bytes_allocated;

// -------------------------------------------------------------------------------------------------

extern bool comm_session_is_current_task_send_next_task(CommSession *session);
extern bool comm_session_is_valid(const CommSession *session);
extern void comm_session_send_next_immediately(CommSession *session);

// -------------------------------------------------------------------------------------------------
//! To be called once at boot
void comm_default_kernel_sender_init(void) {
  s_default_kernel_sender_write_semaphore = xSemaphoreCreateBinary();
}


// -------------------------------------------------------------------------------------------------
// Helpers

static uint32_t prv_remaining_ms(uint32_t timeout_ms_in, RtcTicks start_ticks) {
  const RtcTicks now = rtc_get_ticks();
  const uint32_t elapsed_ms = (((now - start_ticks) * 1000) / RTC_TICKS_HZ);
  if (timeout_ms_in > elapsed_ms) {
    return timeout_ms_in - elapsed_ms;
  }
  return 0;
}

static SendBuffer *prv_create_send_buffer(CommSession *session, uint16_t endpoint_id,
                                          size_t payload_buffer_length) {
  bt_lock_assert_held(true /* assert_is_held */);
  const size_t num_bytes_allocated_after = (s_default_kernel_sender_bytes_allocated +
                                            sizeof(PebbleProtocolHeader) + payload_buffer_length);
  if (num_bytes_allocated_after > DEFAULT_KERNEL_SENDER_MAX_BYTES_ALLOCATED) {
    return NULL;
  }
  const size_t allocation_size = (sizeof(SendBuffer) + payload_buffer_length);
  s_default_kernel_sender_bytes_allocated = num_bytes_allocated_after;
  // Use ...alloc_check() here. If this appears to be an issue, we could consider giving this
  // module its own Heap:
  SendBuffer *sb = (SendBuffer *)kernel_zalloc_check(allocation_size);
  *sb = (const SendBuffer) {
    .payload_buffer_length = payload_buffer_length,
    .consumed_length = 0,
    .session = session,
    .header = {
      .endpoint_id = htons(endpoint_id),
      .length = 0,
    },
  };
  return sb;
}

static void prv_destroy_send_buffer(SendBuffer *sb) {
  bt_lock_assert_held(true /* assert_is_held */);
  s_default_kernel_sender_bytes_allocated -= (sizeof(PebbleProtocolHeader)
                                              + sb->payload_buffer_length);
  kernel_free(sb);
  xSemaphoreGive(s_default_kernel_sender_write_semaphore);
}

// -------------------------------------------------------------------------------------------------
// Interfaces towards Send Queue:

static size_t prv_get_remaining_length(const SendBuffer *sb) {
  return (sizeof(PebbleProtocolHeader) + sb->written_length - sb->consumed_length);
}

static const uint8_t *prv_get_read_pointer(const SendBuffer *sb) {
  return ((const uint8_t *)&sb->header + sb->consumed_length);
}

static size_t prv_send_job_impl_get_length(const SessionSendQueueJob *send_job) {
  return prv_get_remaining_length((SendBuffer *)send_job);
}

static size_t prv_send_job_impl_copy(const SessionSendQueueJob *send_job, int start_offset,
                                     size_t length, uint8_t *data_out) {
  SendBuffer *sb = (SendBuffer *)send_job;
  const size_t length_remaining = prv_get_remaining_length(sb);
  const size_t length_after_offset = (length_remaining - start_offset);
  const size_t length_to_copy = MIN(length_after_offset, length);
  memcpy(data_out, prv_get_read_pointer(sb) + start_offset, length_to_copy);
  return length_to_copy;
}

static size_t prv_send_job_impl_get_read_pointer(const SessionSendQueueJob *send_job,
                                                 const uint8_t **data_out) {
  SendBuffer *sb = (SendBuffer *)send_job;
  *data_out = prv_get_read_pointer(sb);
  return prv_get_remaining_length(sb);
}

static void prv_send_job_impl_consume(const SessionSendQueueJob *send_job, size_t length) {
  SendBuffer *sb = (SendBuffer *)send_job;
  sb->consumed_length += length;
}

static void prv_send_job_impl_free(SessionSendQueueJob *send_job) {
  prv_destroy_send_buffer((SendBuffer *)send_job);
}

T_STATIC const SessionSendJobImpl s_default_kernel_send_job_impl = {
  .get_length = prv_send_job_impl_get_length,
  .copy = prv_send_job_impl_copy,
  .get_read_pointer = prv_send_job_impl_get_read_pointer,
  .consume = prv_send_job_impl_consume,
  .free = prv_send_job_impl_free,
};

// -------------------------------------------------------------------------------------------------
// Interfaces towards subsystems that need to send data out

size_t comm_session_send_buffer_get_max_payload_length(const CommSession *session) {
  size_t max_length = 0;
  bt_lock();
  {
    if (comm_session_is_valid(session)) {
      max_length = DEFAULT_KERNEL_SENDER_MAX_PAYLOAD_SIZE;
    }
  }
  bt_unlock();
  return max_length;
}

SendBuffer * comm_session_send_buffer_begin_write(CommSession *session, uint16_t endpoint_id,
                                                  size_t required_payload_length,
                                                  uint32_t timeout_ms) {
  if (!session) {
    return NULL;
  }
  if (required_payload_length > DEFAULT_KERNEL_SENDER_MAX_PAYLOAD_SIZE) {
    PBL_LOG_WRN("Message for endpoint_id %u exceeds maximum length (length=%"PRIu32")",
            endpoint_id, (uint32_t)required_payload_length);
    return NULL;
  }

  RtcTicks start_ticks = rtc_get_ticks();
  SendBuffer *sb = NULL;

  while (true) {
    bool is_timeout = false;
    bool is_current_task_send_next_task;

    bt_lock();
    {
      if (!comm_session_is_valid(session)) {
        bt_unlock();
        return NULL;
      }
      sb = prv_create_send_buffer(session, endpoint_id, required_payload_length);
      is_current_task_send_next_task = comm_session_is_current_task_send_next_task(session);
    }
    bt_unlock();

    if (sb) {
      return sb;
    }

    // Check for timeout
    uint32_t remaining_ms = prv_remaining_ms(timeout_ms, start_ticks);
    if (remaining_ms == 0) {
      is_timeout = true;
    } else {
      if (is_current_task_send_next_task) {
        // If there is no space and this is called from the task that performs the sending,
        // the "send_next" callback is waiting in the task queue after this callback.
        // Therefore, data will never get sent out unless it's done right now:
        comm_session_send_next_immediately(session);
      } else {
        // Wait for the sending process to free up some space in the send buffer:
        is_timeout = (xSemaphoreTake(s_default_kernel_sender_write_semaphore,
                                     remaining_ms) == pdFALSE);
      }
    }

    if (is_timeout) {
      PBL_LOG_WRN("Failed to get send buffer (bytes=%"PRIu32", endpoint_id=%"PRIu16", to=%"PRIu32")",
              (uint32_t)required_payload_length, endpoint_id, (uint32_t)is_timeout);
      return NULL;
    }
  }  // while(true)
}

bool comm_session_send_buffer_write(SendBuffer *sb, const uint8_t *data, size_t length) {
  if (UNLIKELY((sb->payload_buffer_length - sb->written_length) < length)) {
    return false;
  }
  memcpy(sb->payload + sb->header.length + sb->written_length, data, length);
  sb->written_length += length;
  return true;
}

void comm_session_send_buffer_end_write(SendBuffer *sb) {
  CommSession *session = sb->session;
  // Clear out the ListNode and set impl:
  sb->queue_job = (const SessionSendQueueJob) {
    .impl = &s_default_kernel_send_job_impl,
  };
  sb->header.length = ntohs(sb->written_length);
  comm_session_send_queue_add_job(session, (SessionSendQueueJob **)&sb);
}

// -------------------------------------------------------------------------------------------------
// Interfaces for testing

SemaphoreHandle_t comm_session_send_buffer_write_semaphore(void) {
  return s_default_kernel_sender_write_semaphore;
}

void comm_default_kernel_sender_deinit(void) {
  vSemaphoreDelete(s_default_kernel_sender_write_semaphore);
  s_default_kernel_sender_write_semaphore = NULL;
}
