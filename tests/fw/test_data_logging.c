/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "applib/data_logging.h"
#include "pbl/util/uuid.h"

#include "process_management/pebble_process_md.h"

#include "pbl/services/filesystem/pfs.h"
#include "pbl/services/filesystem/flash_translation.h"

#include "pbl/services/comm_session/protocol.h"
#include "pbl/services/comm_session/session.h"
#include "pbl/services/comm_session/session_send_buffer.h"
#include "pbl/services/comm_session/session_transport.h"

#include "pbl/services/data_logging/data_logging_service.h"
#include "pbl/services/data_logging/dls_private.h"
#include "pbl/services/data_logging/dls_list.h"
#include "pbl/services/data_logging/dls_storage.h"

#include "pbl/services/regular_timer.h"
#include <pbl/logging/logging.h>
#include "system/passert.h"

#include "util/legacy_checksum.h"
#include "pbl/util/list.h"
#include "pbl/util/math.h"
#include "pbl/util/size.h"
#include "pbl/util/string.h"

#include "clar.h"

// Stubs
#include "fake_app_manager.h"
#include "fake_pebble_tasks.h"
#include "fake_system_task.h"
#include "fake_spi_flash.h"
#include "fake_session.h"
#include "fake_new_timer.h"
#include "fake_pbl_malloc.h"
#include "fake_rtc.h"
#include "stubs_analytics.h"
#include "stubs_bt_lock.h"
#include "stubs_hexdump.h"
#include "stubs_logging.h"
#include "stubs_mutex.h"
#include "stubs_passert.h"
#include "stubs_prompt.h"
#include "stubs_rand_ptr.h"
#include "stubs_serial.h"
#include "stubs_sleep.h"
#include "stubs_syscall_internal.h"
#include "stubs_task_watchdog.h"
#include "stubs_reboot_reason.h"

#include <stdlib.h>
#include <stdio.h>



#include "pbl/kernel/sched.h"
pbl_tick_t pbl_uptime_ticks(void) {
  return 1337;
}

#include "kernel/memory_layout.h"
const MpuRegion* memory_layout_get_app_region(void) {
  return NULL;
}
bool memory_layout_is_buffer_in_region(const MpuRegion *region, const void *buf, size_t length) {
  return true;
}

// We can't include all of stubs_process_manager because it conflicts with fake_app_manager.h
bool process_manager_send_event_to_process(PebbleTask task, PebbleEvent* e) {
  return true;
}

// ------------------------------------------------------------------------------------
// Comm session fake support
void data_logging_protocol_msg_callback(CommSession *session, const uint8_t *data, size_t length);

static CommSession *s_session;

static DataLoggingSendDataMessage s_prev_send_data_hdr;
static uint8_t s_prev_send_data[COMM_MAX_OUTBOUND_PAYLOAD_SIZE];
static uint32_t s_prev_send_data_bytes;

static void prv_transport_sent_data_cb(uint16_t endpoint_id,
                                       const uint8_t* data, unsigned int data_length) {
  PBL_LOG_INFO("Received %d bytes of data from watch", data_length);
  if (data_length >= sizeof(s_prev_send_data_hdr)) {
    memcpy(&s_prev_send_data_hdr, data, sizeof(s_prev_send_data_hdr));
    data_length -= sizeof(s_prev_send_data_hdr);
    data += sizeof(s_prev_send_data_hdr);
    if (data_length > 0) {
      s_prev_send_data_bytes = data_length;
      memcpy(s_prev_send_data, data, data_length);
    } else {
      s_prev_send_data_bytes = 0;
    }
  }
}


// ----------------------------------------------------------------------------------------
// Setup
static void prv_init_fake_flash(void) {
  fake_spi_flash_init(0, 0x1000000);
  pfs_init(false);
  pfs_format(false /* write erase headers */);

  PBL_LOG_INFO("\nFile system size: %d, avail: %d", (int)pfs_get_size(),
          (int)get_available_pfs_space());
}

// ----------------------------------------------------------------------------------------
// fill a buffer, return it's CRC32
static uint32_t prv_get_random_buffer(uint8_t **buf, unsigned int size) {
  uint8_t *temp = calloc(size, sizeof(uint8_t));

  for (unsigned int i = 0; i < size; i++) {
    temp[i] = (uint8_t)(rand() % 10);
  }

  *buf = temp;
  return legacy_defective_checksum_memory(temp, size);
}

// ----------------------------------------------------------------------------------------
static void prv_data_log_chain(DataLoggingSessionRef logging_session, uint8_t *buf,
                               int item_size, int num_items) {
  int num_bytes = item_size * num_items;
  while (num_bytes > 0) {
    unsigned int chunk_size;
    if (item_size > DLS_SESSION_MAX_BUFFERED_ITEM_SIZE) {
      // Must be unbuffered
      chunk_size = item_size;
    } else {
      chunk_size = MIN(num_bytes, DLS_SESSION_MAX_BUFFERED_ITEM_SIZE);
      chunk_size -= (chunk_size % item_size);
    }
    PBL_ASSERTN(chunk_size <= num_bytes);
    data_logging_log(logging_session, buf, chunk_size / item_size);
    fake_system_task_callbacks_invoke_pending();
    buf += chunk_size;
    num_bytes -= chunk_size;
  }
}

// ----------------------------------------------------------------------------------------
static void prv_check_session_data(DataLoggingSessionRef logging_session, uint32_t crc,
                                   unsigned int num_bytes) {
  uint8_t buffer[num_bytes];
  uint32_t read_bytes = dls_test_read(logging_session, buffer, num_bytes);
  cl_assert(read_bytes == num_bytes);

  uint32_t session_crc = legacy_defective_checksum_memory(buffer, num_bytes);
  cl_assert(crc == session_crc);

  dls_test_consume(logging_session, num_bytes);
  cl_assert(dls_test_get_num_bytes(logging_session) == 0);
}

// ----------------------------------------------------------------------------------------
// log some random data, return its crc32
static uint32_t prv_log_random_data(DataLoggingSessionRef logging_session, int item_size,
                                    int num_items) {
  PBL_LOG_INFO("Logging %d bytes", item_size * num_items);
  uint8_t *random_buf;
  uint32_t random_crc = prv_get_random_buffer(&random_buf, item_size * num_items);

  prv_data_log_chain(logging_session, random_buf, item_size, num_items);

  free (random_buf);
  return (random_crc);
}

// ----------------------------------------------------------------------------------------
static void prv_log_consume_random(DataLoggingSessionRef logging_session, int item_size,
                                   int num_items) {
  uint32_t random_crc = prv_log_random_data(logging_session, item_size, num_items);
  prv_check_session_data(logging_session, random_crc, item_size * num_items);
}

// ----------------------------------------------------------------------------------------
void test_data_logging__initialize(void) {
  regular_timer_init();
  prv_init_fake_flash();
  stub_pebble_tasks_set_current(PebbleTask_KernelBackground);
  dls_clear();
  dls_init();
  fake_system_task_callbacks_invoke_pending();

  // Create the system comm session
  //Transport *transport = (Transport *) ~0;
  //s_session = comm_session_open(transport, &s_transport_imp,
  //                                         TransportDestinationSystem);
  fake_comm_session_init();
  Transport *transport = fake_transport_create(TransportDestinationSystem, NULL,
                                               prv_transport_sent_data_cb);
  s_session = fake_transport_set_connected(transport, true /* connected */);
}

// ----------------------------------------------------------------------------------------
void test_data_logging__cleanup(void) {
  regular_timer_deinit();
  fake_comm_session_cleanup();
}

// ----------------------------------------------------------------------------------------
void test_data_logging__log_consume(void) {
  DataLoggingSessionRef logging_sessions[10];
  const int item_size = 1;

  // Create sessions
  for (int i = 0; i < 10; i++) {
    logging_sessions[i] = data_logging_create(i, DATA_LOGGING_UINT, item_size, false);
    cl_assert(logging_sessions[i]);
    fake_system_task_callbacks_invoke_pending();
  }

  // Log Consume
  for (int i = 0; i < 10; i++) {
    prv_log_consume_random(logging_sessions[i], item_size, rand() % 12345);
  }
}

// ----------------------------------------------------------------------------------------
void test_data_logging__log_consume_non_buffered(void) {
  DataLoggingSessionRef logging_sessions[10];
  Uuid system_uuid = UUID_SYSTEM;

  // Test that we can log items > DLS_SESSION_MAX_BUFFERED_ITEM_SIZE when non-buffered
  const int item_size = 2 * DLS_SESSION_MAX_BUFFERED_ITEM_SIZE;

  // Create sessions
  for (int i = 0; i < 10; i++) {
    logging_sessions[i] =  (DataLoggingSessionRef)dls_create(i, DATA_LOGGING_BYTE_ARRAY, item_size,
                            false /*buffered*/, false /*resume*/, &system_uuid);
    cl_assert(logging_sessions[i]);
  }

  // Log Consume
  for (int i = 0; i < 10; i++) {
    prv_log_consume_random(logging_sessions[i], item_size, rand() % 16);
  }
}


// ----------------------------------------------------------------------------------------
void test_data_logging__log_consume_large_items(void) {
  DataLoggingSessionRef logging_sessions[10];
  int item_size[10];

  // Create sessions
  for (int i = 0; i < 10; i++) {
    item_size[i] = 50 + (rand() % 250);
    logging_sessions[i] = data_logging_create(i, DATA_LOGGING_BYTE_ARRAY, item_size[i], false);
    cl_assert(logging_sessions[i]);
    fake_system_task_callbacks_invoke_pending();
  }

  // Log Consume
  for (int i = 0; i < 10; i++) {
    prv_log_consume_random(logging_sessions[i], item_size[i], rand() % 123);
  }
}


// ----------------------------------------------------------------------------------------
// Test writing and consuming so much that we are forced to reallocate the file partway
// through
void test_data_logging__log_realloc(void) {
  DataLoggingSessionRef logging_sessions[5];
  const int item_size = 1;

  // Create sessions
  for (int i = 0; i < 5; i++) {
    logging_sessions[i] = data_logging_create(i, DATA_LOGGING_UINT, item_size, false);
    cl_assert(logging_sessions[i]);
    fake_system_task_callbacks_invoke_pending();
  }

  // Log Consume
  for (int i = 0; i < 5; i++) {
    // Each write is 1/8 to 1/4 of the initial file size.
    int num_bytes = DLS_FILE_INIT_SIZE_BYTES/8 + (rand() % DLS_FILE_INIT_SIZE_BYTES/8);

    // By doing 16 loops, we are sure to cycle through the allocated file size at least twice.
    for (int j=0; j<16; j++) {
      prv_log_consume_random(logging_sessions[i], item_size, num_bytes);
    }
  }
}


// ----------------------------------------------------------------------------------------
// Filling up the file system. We should be limited to creating DLS_MAX_DATA_BYTES worth
// of storage
void test_data_logging__fill_quota(void) {
  const int item_size = 1;
  const int num_sessions = 5;
  DataLoggingSessionRef logging_sessions[DLS_MAX_NUM_SESSIONS];

  // Create sessions
  for (int i = 0; i < num_sessions; i++) {
    logging_sessions[i] = data_logging_create(i, DATA_LOGGING_UINT, item_size, false);
    cl_assert(logging_sessions[i]);
    fake_system_task_callbacks_invoke_pending();
  }

  // This should fill up the file system
  int bytes_per_session = 2 * DLS_TOTAL_STORAGE_BYTES / num_sessions;
  for (int i = 0; i < num_sessions; i++) {
    prv_log_random_data(logging_sessions[i], item_size, bytes_per_session);
  }

  // Check the total capacity, it should be no more than DLS_TOTAL_STORAGE_BYTES, but close
  // to DLS_MAX_DATA_BYTES
  int total_bytes = 0;
  for (int i = 0; i < num_sessions; i++) {
    uint32_t size = dls_test_get_num_bytes(logging_sessions[i]);
    PBL_LOG_INFO("Size of session %d: %d", i, size);
    total_bytes += size;
  }

  PBL_LOG_INFO("total bytes: %d", total_bytes);
  cl_assert(total_bytes < DLS_TOTAL_STORAGE_BYTES);

  // We should still be able to create more sessions up to the max
  for (int i = num_sessions; i < DLS_MAX_NUM_SESSIONS; i++) {
    logging_sessions[i] = data_logging_create(i, DATA_LOGGING_UINT, item_size, false);
    cl_assert(logging_sessions[i]);
    fake_system_task_callbacks_invoke_pending();
    prv_log_random_data(logging_sessions[i], item_size, DLS_FILE_INIT_SIZE_BYTES);
  }

  // Check the total capacity, it should still be no more than DLS_TOTAL_STORAGE_BYTES.
  total_bytes = 0;
  for (int i = 0; i < num_sessions; i++) {
    uint32_t size = dls_test_get_num_bytes(logging_sessions[i]);
    PBL_LOG_INFO("Size of session %d: %d", i, size);
    total_bytes += size;
  }

  PBL_LOG_INFO("total bytes: %d", total_bytes);
  cl_assert(total_bytes < DLS_TOTAL_STORAGE_BYTES);
}


// ----------------------------------------------------------------------------------------
// Test logging a LOT of data.
void test_data_logging__large_session(void) {
  const int item_size = DLS_ENDPOINT_MAX_PAYLOAD;
  DataLoggingSessionRef logging_session;
  Uuid system_uuid = UUID_SYSTEM;

  logging_session = (DataLoggingSessionRef)dls_create(0, DATA_LOGGING_BYTE_ARRAY, item_size,
                            false /*buffered*/, false /*resume*/, &system_uuid);
  cl_assert(logging_session);
  fake_system_task_callbacks_invoke_pending();

  // We should be able to create a really large session.
  int num_bytes = DLS_MAX_DATA_BYTES/2;
  int num_items = num_bytes / item_size;
  cl_assert(num_bytes > 0);
  prv_log_consume_random(logging_session, item_size, num_items);
}


// ----------------------------------------------------------------------------------------
void test_data_logging__interleave(void) {
  DataLoggingSessionRef logging_sessions[10];
  uint32_t crcs[10];
  uint8_t *buf[10];
  uint32_t buf_size[10];
  uint32_t bytes_left[10];
  const int item_size = 1;

  for (int i = 0; i < 10; i++) {
    logging_sessions[i] = data_logging_create(i, DATA_LOGGING_UINT, item_size, false);
    cl_assert(logging_sessions[i]);
    fake_system_task_callbacks_invoke_pending();

    buf_size[i] = bytes_left[i] = rand() % (50 * 300);
    crcs[i] = prv_get_random_buffer(&(buf[i]), buf_size[i]);
  }

  bool did_some = true;
  while (did_some) {
    did_some = false;
    for (int i = 0; i < 10; i++) {
      unsigned int nb = rand() % 300;
      nb = MIN(nb, bytes_left[i]);
      if (!nb) {
        continue;
      }
      did_some = true;
      prv_data_log_chain(logging_sessions[i], buf[i], item_size, nb);
      bytes_left[i] -= nb;
      buf[i] += nb;
    }
  }

  for (int i = 0; i < 10; i++) {
    prv_check_session_data(logging_sessions[i], crcs[i], buf_size[i]);
  }
}


// ----------------------------------------------------------------------------------------
static void prv_do_recovery_test(int num_sessions) {
  unsigned int num_bytes[num_sessions];
  uint32_t crcs[num_sessions];
  const int item_size = 1;

  // Log some random data
  for (int i = 0; i < num_sessions; i++) {
    DataLoggingSessionRef logging_session = data_logging_create(i, DATA_LOGGING_UINT, item_size,
                                                                false);
    cl_assert(logging_session);
    num_bytes[i] = rand() % 12345;
    crcs[i] = prv_log_random_data(logging_session, item_size, num_bytes[i]);
  }

  // Clear the logging sessions from RAM
  dls_list_remove_all();
  DataLoggingSessionRef logging_session = dls_list_get_next(NULL);
  cl_assert(logging_session == NULL);

  // Reset regular timer. dls_init() will add the same timer info again
  regular_timer_deinit();
  regular_timer_init();

  // Rebuild the list from flash
  dls_init();
  fake_system_task_callbacks_invoke_pending();

  // Check the sessions
  for (int i = 0; i < num_sessions; i++) {
    logging_session = dls_list_get_next(logging_session);
    cl_assert(logging_session != NULL);

    uint32_t tag = dls_test_get_tag(logging_session);
    prv_check_session_data(logging_session, crcs[tag], num_bytes[tag]);
  }
  logging_session = dls_list_get_next(logging_session);
  cl_assert(logging_session == NULL);
}

void test_data_logging__recover_one(void) {
  prv_do_recovery_test(1);
}

void test_data_logging__recover_five(void) {
  prv_do_recovery_test(5);
}

// ----------------------------------------------------------------------------------------
//! Try passing garbage pointers to sessions to data logging functions.
void test_data_logging__invalid_session_garbage(void) {
  uint32_t data[] = { 1, 2, 3 };

  // Make sure logging to bogus sessions does the right thing.
  cl_assert_equal_i(data_logging_log(0, data, ARRAY_LENGTH(data)), DATA_LOGGING_INVALID_PARAMS);
  cl_assert_equal_i(data_logging_log(&data, data, ARRAY_LENGTH(data)), DATA_LOGGING_INVALID_PARAMS);

  // Make sure closing invalid sessions doesn't crash. It's defined to be a no-op
  data_logging_finish(0);
  data_logging_finish(&data);
}

// ----------------------------------------------------------------------------------------
//! Try using sessions after we've closed them.
void test_data_logging__invalid_session_use_after_close(void) {
  uint32_t data[] = { 1, 2, 3 };

  DataLoggingSessionRef session = data_logging_create(0x1234, DATA_LOGGING_UINT, 4, false);
  cl_assert(session);
  fake_system_task_callbacks_invoke_pending();

  data_logging_finish(session);
  fake_system_task_callbacks_invoke_pending();

  // Log to the session after it's closed.
  cl_assert_equal_i(data_logging_log(session, data, ARRAY_LENGTH(data)),
                    DATA_LOGGING_INVALID_PARAMS);

  // Finish the session again without a crash.
  data_logging_finish(0);
}

// ----------------------------------------------------------------------------------------
//! Try passing invalid params to data_logging_log
void test_data_logging__invalid_params(void) {
  uint32_t data[] = { 1, 2, 3 };

  DataLoggingSessionRef session = data_logging_create(0x1234, DATA_LOGGING_UINT, 4, false);
  cl_assert(session);
  fake_system_task_callbacks_invoke_pending();

  // Log to the session after it's closed.
  cl_assert_equal_i(data_logging_log(session, NULL, 4),
                    DATA_LOGGING_INVALID_PARAMS);
  cl_assert_equal_i(data_logging_log(NULL, data, ARRAY_LENGTH(data)),
                    DATA_LOGGING_INVALID_PARAMS);

  // Finish the session without a crash
  data_logging_finish(0);
}

// ----------------------------------------------------------------------------------------
// Test emptying the session using dls_private_send_session
static void prv_endpoint_test(bool buffered, const int item_size, const int num_items) {
  DataLoggingSessionRef logging_session;

  // Create session
  Uuid system_uuid = UUID_SYSTEM;
  const PebbleProcessMd *md = sys_process_manager_get_current_process_md();
  const Uuid *uuid;
  if (buffered) {
    uuid = &md->uuid;
  } else {
    uuid = &system_uuid;
  }
  logging_session = (DataLoggingSessionRef)dls_create(0, DATA_LOGGING_BYTE_ARRAY, item_size,
                            buffered, false /*resume*/, uuid);
  cl_assert(logging_session);
  fake_system_task_callbacks_invoke_pending();

  // This sends the open session request out the transport
  fake_comm_session_process_send_next();


  // Generate the received ack from the phone endpoint
  CommSession *session = comm_session_get_system_session();
  uint8_t ack_data[] = { ~DLS_ENDPOINT_CMD_MASK | DataLoggingEndpointCmdAck,
                     dls_test_get_session_id(logging_session)};
  data_logging_protocol_msg_callback(session, ack_data, 2);
  fake_system_task_callbacks_invoke_pending();

  // Log the data
  uint32_t random_crc = prv_log_random_data(logging_session, item_size, num_items);

  // Finish up the session so that all data gets sent out the endpoint
  data_logging_finish(logging_session);


  // ---------------------------------------------------------------------------
  // Consume it using the method used by the data logging endpoint
  const int buf_size = num_items * item_size;
  uint8_t *rcv_buffer = calloc(num_items, item_size);
  PBL_ASSERTN(rcv_buffer);

  uint32_t rcv_bytes = 0;
  s_prev_send_data_bytes = 0;
  dls_private_send_session(logging_session, true /*empty*/);
  const int items_per_send = (COMM_MAX_OUTBOUND_PAYLOAD_SIZE - sizeof(DataLoggingSendDataMessage))
                           / item_size;
  const int num_sends = num_items / items_per_send;
  for (int i = 0; i < num_sends + 5; i++) {

    // This sends a chunk out and it should show up in our prv_transport_sent_data_cb callback
    fake_comm_session_process_send_next();
    cl_assert(s_prev_send_data_bytes <= buf_size - rcv_bytes);
    memcpy(rcv_buffer + rcv_bytes, s_prev_send_data, s_prev_send_data_bytes);
    rcv_bytes += s_prev_send_data_bytes;
    s_prev_send_data_bytes = 0;

    // Provide acknowledgement from phone, this should trigger another dls_private_send_session
    data_logging_protocol_msg_callback(session, ack_data, 2);
    fake_system_task_callbacks_invoke_pending();
  }

  // Verify the received data
  uint32_t session_crc = legacy_defective_checksum_memory(rcv_buffer, rcv_bytes);
  cl_assert(random_crc == session_crc);

  // Free buffer
  free(rcv_buffer);
}


// ----------------------------------------------------------------------------------------
// Test using the endpoint to empty the session
void test_data_logging__send_session_1(void) {
  prv_endpoint_test(true /*buffered*/, 1, 1000);
}

// ----------------------------------------------------------------------------------------
// Test using the endpoint to empty a session using large item sizes
void test_data_logging__send_session_large(void) {
  prv_endpoint_test(false /*buffered*/, DLS_ENDPOINT_MAX_PAYLOAD, 20);
}

// ----------------------------------------------------------------------------------------
// Test using the endpoint to empty a session using medium item sizes
void test_data_logging__send_session_medium(void) {
  prv_endpoint_test(true /*buffered*/, 90, 20);
}

// ----------------------------------------------------------------------------------------
// Test using the endpoint to empty a session using small item sizes. The item size of 19
//  exposes issue PBL-21331
void test_data_logging__send_session_small(void) {
  prv_endpoint_test(true /*buffered*/, 19, 45);
}


