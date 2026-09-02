/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "pbl/services/comm_session/default_kernel_sender.h"
#include "pbl/services/comm_session/session_send_buffer.h"
#include "pbl/services/comm_session/session_transport.h"
#include "pbl/services/comm_session/session_internal.h"
#include "pbl/services/comm_session/session_send_queue.h"
#include "pbl/services/comm_session/protocol.h"

#include "util/net.h"
#include "pbl/util/size.h"

#include "pbl/kernel/sem.h"

#include "clar.h"

extern SendBuffer * comm_session_send_buffer_create(bool is_system);
extern void comm_session_send_buffer_destroy(SendBuffer *sb);
extern struct pbl_sem * comm_session_send_buffer_write_semaphore(void);
extern T_STATIC const SessionSendJobImpl s_default_kernel_send_job_impl;
extern void comm_default_kernel_sender_deinit(void);
extern void comm_session_send_queue_cleanup(CommSession *session);

// Stubs
///////////////////////////////////////////////////////////

#include "stubs_bt_lock.h"
#include "stubs_logging.h"
#include "stubs_passert.h"
#include "stubs_analytics.h"

// Fakes
///////////////////////////////////////////////////////////

#include "fake_kernel_malloc.h"
#include "fake_sem.h"
#include "fake_rtc.h"

static CommSession s_session;

static CommSession *s_valid_session;

static void prv_cleanup_send_buffer(SendBuffer *sb) {
  s_default_kernel_send_job_impl.free((SessionSendQueueJob *)sb);
}

bool comm_session_is_valid(const CommSession *session) {
  if (!session) {
    return false;
  }
  return (s_valid_session == session);
}

static int s_send_next_count = 0;
void comm_session_send_next(CommSession *session) {
  ++s_send_next_count;
}

void comm_session_send_next_immediately(CommSession *session) {
  // Pretend to send out all the data:
  size_t read_space = comm_session_send_queue_get_length(session);
  comm_session_send_queue_consume(session, read_space);
}

static bool s_is_current_task_send_next_task = false;
bool comm_session_is_current_task_send_next_task(CommSession *session) {
  return s_is_current_task_send_next_task;
}

// Tests
///////////////////////////////////////////////////////////

static const uint16_t ENDPOINT_ID = 1234;
static const uint32_t TIMEOUT_MS = 500;

void test_session_send_buffer__initialize(void) {
  s_is_current_task_send_next_task = false;
  s_session = (const CommSession) {};
  fake_kernel_malloc_init();
  fake_kernel_malloc_enable_stats(true);
  fake_kernel_malloc_mark();
  s_send_next_count = 0;
  comm_default_kernel_sender_init();
}

void test_session_send_buffer__cleanup(void) {
  comm_default_kernel_sender_deinit();

  // Check for leaks:
  fake_kernel_malloc_mark_assert_equal();
  fake_kernel_malloc_deinit();
}

void test_session_send_buffer__null_session(void) {
  cl_assert_equal_p(NULL, comm_session_send_buffer_begin_write(NULL, ENDPOINT_ID, 1, TIMEOUT_MS));
}

void test_session_send_buffer__begin_write_with_more_than_max_payload(void) {
  s_valid_session = &s_session;

  size_t max_length = comm_session_send_buffer_get_max_payload_length(&s_session);
  SendBuffer *write_sb = comm_session_send_buffer_begin_write(&s_session, ENDPOINT_ID,
                                                              max_length + 1,
                                                              TIMEOUT_MS);
  cl_assert_equal_p(write_sb, NULL);
}

pbl_tick_t prv_session_closed_yield_cb(struct pbl_sem *handle) {
  if (s_valid_session) {
    comm_session_send_queue_cleanup(s_valid_session);
    s_valid_session = NULL;
  }
  return 10;
}

pbl_tick_t prv_receive_but_no_bytes_freed_yield_cb(struct pbl_sem *handle) {
  fake_rtc_increment_ticks(100);
  pbl_sem_give(handle);
  return 100;
}

void test_session_send_buffer__not_enough_space_in_time(void) {
  s_valid_session = &s_session;

  // Fill the send buffer completely:
  const size_t max_length = comm_session_send_buffer_get_max_payload_length(&s_session);
  SendBuffer *write_sb = comm_session_send_buffer_begin_write(&s_session, ENDPOINT_ID,
                                                              max_length /* required_free_length */,
                                                              TIMEOUT_MS);
  cl_assert(write_sb);
  uint8_t fake_data[max_length];
  memset(fake_data, 0, max_length);
  comm_session_send_buffer_write(write_sb, fake_data, max_length);
  comm_session_send_buffer_end_write(write_sb);

  // Set a yield callback that gives the semph in time but does not clear out the send buffer:
  struct pbl_sem * write_semph = comm_session_send_buffer_write_semaphore();
  fake_sem_set_yield_callback(write_semph, prv_receive_but_no_bytes_freed_yield_cb);

  // Try to begin writing again, requesting only one byte:
  SendBuffer *write_sb2 = comm_session_send_buffer_begin_write(&s_session, ENDPOINT_ID,
                                                               1 /* required_free_length */,
                                                               TIMEOUT_MS);
  cl_assert_equal_p(write_sb2, NULL);

  prv_cleanup_send_buffer(write_sb);
}

void test_session_send_buffer__multiple_smaller_messages(void) {
  s_valid_session = &s_session;

  // This length excludes the sizeof(PebbleProtocolHeader).
  size_t bytes_free = comm_session_send_buffer_get_max_payload_length(&s_session);
  bytes_free += sizeof(PebbleProtocolHeader);

  const int num_sbs = 1 + (bytes_free / (sizeof(PebbleProtocolHeader) + 1 /* payload_length */));
  SendBuffer *write_sb[num_sbs];
  memset(write_sb, 0, sizeof(write_sb));

  for (int i = 0; bytes_free > 0 && i < ARRAY_LENGTH(write_sb); ++i) {
    size_t payload_length = 1;

    bytes_free -= sizeof(PebbleProtocolHeader) + payload_length;

    // If we cannot fit another message after this one, increment the length to use up the space:
    if (bytes_free <= (sizeof(PebbleProtocolHeader) + payload_length)) {
      payload_length += bytes_free;
      bytes_free = 0;
    }

    write_sb[i] = comm_session_send_buffer_begin_write(&s_session, ENDPOINT_ID,
                                                       payload_length, TIMEOUT_MS);
    uint8_t fake_data[payload_length];
    memset(fake_data, 0, payload_length);
    comm_session_send_buffer_write(write_sb[i], fake_data, payload_length);
    comm_session_send_buffer_end_write(write_sb[i]);
  }

  // Can't write another message:
  cl_assert_equal_p(NULL, comm_session_send_buffer_begin_write(&s_session, ENDPOINT_ID,
                                                               1 /* length */, TIMEOUT_MS));

  for (int i = 0; i < ARRAY_LENGTH(write_sb); ++i) {
    if (!write_sb[i]) {
      break;
    }
    prv_cleanup_send_buffer(write_sb[i]);
  }
}

void test_session_send_buffer__not_enough_space_kernel_bg(void) {
  s_valid_session = &s_session;

  // Fill the send buffer completely:
  const size_t max_length = comm_session_send_buffer_get_max_payload_length(&s_session);
  SendBuffer *write_sb = comm_session_send_buffer_begin_write(&s_session, ENDPOINT_ID,
                                                              max_length /* required_free_length */,
                                                              TIMEOUT_MS);
  uint8_t fake_data[max_length];
  memset(fake_data, 0, max_length);
  comm_session_send_buffer_write(write_sb, fake_data, max_length);
  comm_session_send_buffer_end_write(write_sb);

  // Pretend the current task is the same task that processes "send_next".
  // Pretend to execute a callback that was scheduled already before the previous write caused
  // a "send_next" callback to be scheduled.
  s_is_current_task_send_next_task = true;

  // Set a yield callback that gives the semph in time but does not clear out the send buffer:
  struct pbl_sem * write_semph = comm_session_send_buffer_write_semaphore();
  fake_sem_set_yield_callback(write_semph, prv_receive_but_no_bytes_freed_yield_cb);

  // Try to begin writing again, requesting only one byte:
  SendBuffer *write_sb2 = comm_session_send_buffer_begin_write(&s_session, ENDPOINT_ID,
                                                               1 /* required_free_length */,
                                                               TIMEOUT_MS);

  // Because the ..._begin_write() call happened from the BT02 task, expect the data to be
  // sent out immediately (we'd timeout or deadlock if an infinite timeout was set)
  cl_assert(write_sb2);
  comm_session_send_buffer_end_write(write_sb2);

  // write_sb is already cleaned up because it got sent out
  prv_cleanup_send_buffer(write_sb2);
}

void test_session_send_buffer__writing_but_then_session_closed(void) {
  s_valid_session = &s_session;

  // Fill the send buffer completely:
  const size_t max_length = comm_session_send_buffer_get_max_payload_length(&s_session);
  SendBuffer *write_sb = comm_session_send_buffer_begin_write(&s_session, ENDPOINT_ID,
                                                              max_length /* required_free_length */,
                                                              TIMEOUT_MS);
  uint8_t fake_data[max_length];
  memset(fake_data, 0, max_length);
  comm_session_send_buffer_write(write_sb, fake_data, max_length);
  comm_session_send_buffer_end_write(write_sb);

  // Set a yield callback that gives the semph in time but closes the session:
  struct pbl_sem * write_semph = comm_session_send_buffer_write_semaphore();
  fake_sem_set_yield_callback(write_semph, prv_session_closed_yield_cb);

  // Try to begin writing again, requesting only one byte:
  write_sb = comm_session_send_buffer_begin_write(&s_session, ENDPOINT_ID,
                                                  1 /* required_free_length */,
                                                  TIMEOUT_MS);
  cl_assert_equal_p(write_sb, NULL);

  // ..send_buffer_destroy() is already called in the yield cb
}

void test_session_send_buffer__write_beyond_available_space(void) {
  s_valid_session = &s_session;

  // Fill the send buffer completely:
  const size_t max_length = comm_session_send_buffer_get_max_payload_length(&s_session);
  SendBuffer *write_sb = comm_session_send_buffer_begin_write(&s_session, ENDPOINT_ID,
                                                              max_length /* required_free_length */,
                                                              TIMEOUT_MS);
  uint8_t fake_data[max_length];
  memset(fake_data, 0, max_length);
  cl_assert_equal_b(comm_session_send_buffer_write(write_sb, fake_data, max_length), true);

  // Try writing another byte (expect false returned):
  cl_assert_equal_b(comm_session_send_buffer_write(write_sb, fake_data, max_length), false);
  comm_session_send_buffer_end_write(write_sb);

  prv_cleanup_send_buffer(write_sb);
}

void test_session_send_buffer__send_queue_interface(void) {
  s_valid_session = &s_session;

  // Fill the send buffer completely:
  const size_t max_payload_length = comm_session_send_buffer_get_max_payload_length(&s_session);
  SendBuffer *write_sb = comm_session_send_buffer_begin_write(&s_session, ENDPOINT_ID,
                                                              max_payload_length /* required_free_length */,
                                                              TIMEOUT_MS);
  uint8_t fake_data_payload[max_payload_length];
  for (int i = 0; i < max_payload_length; ++i) {
    fake_data_payload[i] = i % 0xff;
  }
  // Write in two parts:
  size_t second_write_length = max_payload_length - (max_payload_length / 2);
  cl_assert_equal_b(comm_session_send_buffer_write(write_sb,
                                                   fake_data_payload,
                                                   max_payload_length - second_write_length), true);
  cl_assert_equal_b(comm_session_send_buffer_write(write_sb,
                                                   fake_data_payload + second_write_length,
                                                   second_write_length), true);

  cl_assert_equal_i(s_send_next_count, 0);
  comm_session_send_buffer_end_write(write_sb);
  // Expect comm_session_send_next to be called to trigger the transport:
  cl_assert_equal_i(s_send_next_count, 1);

  // Exercise the transport interface:
  const SessionSendQueueJob *job = (const SessionSendQueueJob *)write_sb;
  // ..._get_read_space_remaining():
  size_t expected_bytes_incl_pebble_protocol_header =
              max_payload_length + sizeof(PebbleProtocolHeader);
  size_t length = s_default_kernel_send_job_impl.get_length(job);
  cl_assert_equal_i(length, expected_bytes_incl_pebble_protocol_header);

  // ..._copy():
  uint8_t pp_data_out[expected_bytes_incl_pebble_protocol_header];
  size_t bytes_copied =
      s_default_kernel_send_job_impl.copy(job, 0,
                                          expected_bytes_incl_pebble_protocol_header,
                                          pp_data_out);
  cl_assert_equal_i(bytes_copied, expected_bytes_incl_pebble_protocol_header);
  PebbleProtocolHeader *header = (PebbleProtocolHeader *) pp_data_out;
  cl_assert_equal_i(header->length, htons(max_payload_length));
  cl_assert_equal_i(header->endpoint_id, htons(ENDPOINT_ID));
  cl_assert_equal_i(memcmp(pp_data_out + sizeof(PebbleProtocolHeader),
                           fake_data_payload, max_payload_length), 0);

  // ..._copy() with offset:
  int offset = 2;
  bytes_copied =
      s_default_kernel_send_job_impl.copy(job, offset,
                                          expected_bytes_incl_pebble_protocol_header,
                                          pp_data_out);
  cl_assert_equal_i(bytes_copied, expected_bytes_incl_pebble_protocol_header - offset);
  header = (PebbleProtocolHeader *) (pp_data_out - offset);
  cl_assert_equal_i(header->endpoint_id, htons(ENDPOINT_ID));
  cl_assert_equal_i(memcmp(pp_data_out + sizeof(PebbleProtocolHeader) - offset,
                           fake_data_payload, max_payload_length - offset), 0);

  // ..._get_read_pointer():
  uint16_t bytes_read = 0;
  uint16_t read_space;
  const uint8_t *data_out;
  while ((read_space = s_default_kernel_send_job_impl.get_read_pointer(job, &data_out))) {
    PebbleProtocolHeader *header = (PebbleProtocolHeader *) data_out;
    if (bytes_read == 0) {
      cl_assert(read_space >= sizeof(PebbleProtocolHeader));
      cl_assert_equal_i(header->length, htons(max_payload_length));
      cl_assert_equal_i(header->endpoint_id, htons(ENDPOINT_ID));
      cl_assert_equal_i(memcmp(data_out + sizeof(PebbleProtocolHeader),
                               fake_data_payload,
                               read_space - sizeof(PebbleProtocolHeader)), 0);
    } else {
      cl_assert_equal_i(memcmp(data_out,
                               fake_data_payload + bytes_read - sizeof(PebbleProtocolHeader),
                               read_space), 0);
    }
    s_default_kernel_send_job_impl.consume(job, read_space);
    bytes_read += read_space;
  }
  cl_assert_equal_i(bytes_read, expected_bytes_incl_pebble_protocol_header);

  cl_assert_equal_i(comm_session_send_queue_get_length(s_valid_session), 0);

  prv_cleanup_send_buffer(write_sb);
}
