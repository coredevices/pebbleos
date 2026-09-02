/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "pbl/services/put_bytes/put_bytes.h"

#include "pbl/services/comm_session/session_receive_router.h"
#include "pbl/os/tick.h"
#include "system/bootbits.h"
#include "system/firmware_storage.h"
#include <pbl/logging/logging.h>
#include "pbl/util/attributes.h"
#include "util/net.h"

#include <bluetooth/conn_event_stats.h>

#include "FreeRTOS.h"
#include "pbl/kernel/sem.h"

#include "clar.h"

#include <limits.h>

#include "fake_events.h"
#include "fake_pbl_malloc.h"
#include "fake_new_timer.h"
#include "fake_put_bytes_storage_mem.h"
#include "fake_queue.h"
#include "fake_sem.h"
#include "fake_rtc.h"
#include "fake_session.h"
#include "fake_spi_flash.h"
#include "fake_system_task.h"

#include "stubs_bt_lock.h"
#include "stubs_freertos.h"
#include "stubs_hexdump.h"
#include "stubs_logging.h"
#include "stubs_mutex.h"
#include "stubs_passert.h"
#include "stubs_pfs.h"
#include "stubs_prompt.h"
#include "stubs_serial.h"
#include "stubs_task_watchdog.h"
#include "stubs_tick.h"

extern struct pbl_sem * put_bytes_get_semaphore(void);
extern TimerID put_bytes_get_timer_id(void);
extern uint32_t put_bytes_get_index(void);
extern uint8_t prv_put_bytes_get_max_batched_pb_ops(void);

extern const ReceiverImplementation g_put_bytes_receiver_impl;

static const PebbleProtocolEndpoint s_put_bytes_endpoint = (const PebbleProtocolEndpoint) {
  .endpoint_id = 0xBEEF,
  .handler = NULL,
  .access_mask = PebbleProtocolAccessPrivate,
  .receiver_imp = &g_put_bytes_receiver_impl,
  .receiver_opt = NULL,
};

// Fakes
//////////////////////////////////////////////////////////

uint32_t s_boot_bits_orred;
void boot_bit_set(BootBitValue bit) {
  s_boot_bits_orred |= bit;
}

static bool s_firmware_update_is_in_progress;
bool firmware_update_is_in_progress(void) {
  return s_firmware_update_is_in_progress;
}

void psleep(int millis) {
}

void app_storage_get_file_name(char *name, size_t buf_length,
                               AppInstallId app_id, PebbleTask task) {
  strcpy(name, "t");
}

void bluetooth_analytics_handle_put_bytes_stats(bool successful, uint8_t type, uint32_t total_size,
                                                uint32_t elapsed_time_ms,
                                                const SlaveConnEventStats *orig_stats) {
}

bool bt_driver_analytics_get_conn_event_stats(SlaveConnEventStats *stats) {
  return false;
}

typedef enum {
  CmdInit = 0x01,
  CmdPut = 0x02,
  CmdCommit = 0x03,
  CmdAbort = 0x04,
  CmdInstall = 0x05,
  CmdInvalid = 0xff,
} Cmd;

typedef enum {
  ResponseAck = 0x01,
  ResponseNack = 0x02,
} Response;

// Send an INIT message
typedef struct PACKED {
  Cmd cmd:8;
  uint32_t total_size;
  PutBytesObjectType type:8;
  union {
    struct {
      uint8_t index;
      char filename[];
    };
    uint32_t cookie;
  };
} InitRequest;

typedef struct PACKED {
  Cmd cmd:8;
  uint32_t cookie;
  uint32_t payload_size;
  uint8_t payload[];
} PutRequest;

typedef struct PACKED {
  Cmd cmd:8;
  uint32_t cookie;
} InstallRequest;

typedef struct PACKED {
  Cmd cmd:8;
  uint32_t cookie;
} AbortRequest;

typedef struct PACKED {
  Cmd cmd:8;
  uint32_t cookie;
  uint32_t crc;
} CommitRequest;

typedef struct PACKED {
  Response response:8;
  uint32_t cookie;
} ResponseMsg;

static int s_acks_received;
static int s_nacks_received;
static uint32_t s_last_response_cookie;

static CommSession *s_session;

// Helpers
///////////////////////////////////////////////////////////

#define VALID_OBJECT_SIZE (4)
#define PUT_BYTES_TIMEOUT_MS (30000)
#define EXPECTED_CRC (0x12345678)
#define EXPECTED_COOKIE (0xabcd1234)
#define EXPECT_INIT_TIMEOUT_MS (1000)


static void(*s_do_before_write)(void);

static void prv_receive_data(CommSession *session, const uint8_t* data, size_t length) {
  Receiver *r = g_put_bytes_receiver_impl.prepare(session, &s_put_bytes_endpoint, length);
  if (r) {
    if (s_do_before_write) {
      s_do_before_write();
    }
    g_put_bytes_receiver_impl.write(r, data, length);
    g_put_bytes_receiver_impl.finish(r);
  } else {
    PBL_LOG_ERR("No receiver returned!");
  }
}

static void prv_receive_init(uint32_t total_size, PutBytesObjectType object_type) {
  InitRequest init_msg = (InitRequest) {
    .cmd = CmdInit,
    .total_size = htonl(total_size),
    .type = object_type,
    .cookie = htonl(1),
  };
  prv_receive_data(s_session, (const uint8_t *) &init_msg, sizeof(init_msg));
}

static void prv_receive_init_cookie(uint32_t total_size, PutBytesObjectType object_type,
                                    uint32_t cookie) {
  InitRequest init_msg = (InitRequest) {
    .cmd = CmdInit,
    .total_size = htonl(total_size),
    .type = object_type | (1 << 7),
    .cookie = htonl(cookie),
  };
  prv_receive_data(s_session, (const uint8_t *) &init_msg, sizeof(init_msg));
}

static void prv_receive_init_file(uint32_t total_size, const char *fn, size_t fn_len) {
  uint8_t buffer[sizeof(InitRequest) + fn_len];

  InitRequest *init_msg = (InitRequest *)buffer;
  *init_msg = (InitRequest) {
    .cmd = CmdInit,
    .total_size = htonl(total_size),
    .type = ObjectFile,
  };
  memcpy(&init_msg->filename[0], fn, fn_len);
  prv_receive_data(s_session, buffer, sizeof(buffer));
}

static void prv_receive_put(uint32_t cookie, const uint8_t *payload, uint32_t payload_size) {
  uint8_t buffer[sizeof(PutRequest) + payload_size];

  PutRequest *put_msg = (PutRequest *)buffer;
  *put_msg = (PutRequest) {
    .cmd = CmdPut,
    .cookie = htonl(cookie),
    .payload_size = htonl(payload_size),
  };
  memcpy(&put_msg->payload[0], payload, payload_size);
  prv_receive_data(s_session, buffer, sizeof(buffer));
}

static void prv_receive_commit(uint32_t cookie, uint32_t crc) {
  CommitRequest commit_msg = (CommitRequest) {
    .cmd = CmdCommit,
    .cookie = htonl(cookie),
    .crc = htonl(crc),
  };
  prv_receive_data(s_session, (const uint8_t *)&commit_msg, sizeof(commit_msg));
}

static void prv_receive_abort(uint32_t cookie) {
  AbortRequest abort_msg = (AbortRequest) {
    .cmd = CmdAbort,
    .cookie = htonl(cookie),
  };
  prv_receive_data(s_session, (const uint8_t *) &abort_msg, sizeof(abort_msg));
}

static void prv_receive_install(uint32_t cookie) {
  InstallRequest install_msg = (InstallRequest) {
    .cmd = CmdInstall,
    .cookie = htonl(cookie),
  };
  prv_receive_data(s_session, (const uint8_t *) &install_msg, sizeof(install_msg));
}

#define assert_ack_count(c) \
  { \
    fake_comm_session_process_send_next(); \
    cl_assert_equal_i(s_acks_received, c); \
  }

#define assert_nack_count(c) \
  { \
    fake_comm_session_process_send_next(); \
    cl_assert_equal_i(s_nacks_received, c); \
  }

#define assert_cleanup_event(object_type_, object_size_) \
  PebbleEvent event = fake_event_get_last(); \
  cl_assert_equal_i(event.type, PEBBLE_PUT_BYTES_EVENT); \
  cl_assert_equal_i(event.put_bytes.type, PebblePutBytesEventTypeCleanup); \
  cl_assert_equal_i(event.put_bytes.object_type, object_type_); \
  cl_assert_equal_i(event.put_bytes.total_size, object_size_); \
  cl_assert_equal_i(event.put_bytes.progress_percent, 0); \
  cl_assert_equal_b(event.put_bytes.failed, true); \

static void prv_receive_init_fw_object(void) {
  prv_receive_init(VALID_OBJECT_SIZE, ObjectFirmware);
  fake_comm_session_process_send_next();
  fake_system_task_callbacks_invoke_pending();
}

static void prv_process_and_reset_test_counters(void) {
  fake_comm_session_process_send_next();
  fake_system_task_callbacks_invoke_pending();
  s_acks_received = 0;
  s_nacks_received = 0;
}

static void prv_receive_init_and_put_fw_object(void) {
  prv_receive_init(VALID_OBJECT_SIZE, ObjectFirmware);
  fake_comm_session_process_send_next();
  fake_system_task_callbacks_invoke_pending();

  const uint8_t chunk[] = { 0xaa, 0xbb, 0xcc, 0xdd };
  prv_receive_put(s_last_response_cookie, chunk, sizeof(chunk));
  prv_process_and_reset_test_counters();
}

static void prv_receive_init_put_and_commit_fw_object(void) {
  prv_receive_init_and_put_fw_object();
  prv_receive_commit(s_last_response_cookie, EXPECTED_CRC);
  prv_process_and_reset_test_counters();
}

static void prv_receive_init_put_commit_and_install(PutBytesObjectType object_type) {
  prv_receive_init(VALID_OBJECT_SIZE, object_type);
  prv_process_and_reset_test_counters();

  const uint8_t chunk[] = { 0xaa, 0xbb, 0xcc, 0xdd };
  prv_receive_put(s_last_response_cookie, chunk, sizeof(chunk));
  prv_process_and_reset_test_counters();

  prv_receive_commit(s_last_response_cookie, EXPECTED_CRC);
  prv_process_and_reset_test_counters();

  prv_receive_install(s_last_response_cookie);
}

// Tests
///////////////////////////////////////////////////////////

static void prv_system_msg_sent_callback(uint16_t endpoint_id,
                                         const uint8_t* data, unsigned int data_length) {
  if (endpoint_id != 0xBEEF) {
    // Not the put bytes endpoint, ignore
    return;
  }

  // We should only be getting ACKs and NACKs back on this endpoint, which are both 5 bytes long
  cl_assert_equal_i(data_length, 5);

  ResponseMsg *response_msg = (ResponseMsg *)data;
  s_last_response_cookie = ntohl(response_msg->cookie);
  if (response_msg->response == ResponseAck) {
    ++s_acks_received;
  } else if (response_msg->response == ResponseNack) {
    ++s_nacks_received;
  }
}

void test_put_bytes__initialize(void) {
  fake_pb_storage_mem_reset();
  fake_pb_storage_mem_set_crc(EXPECTED_CRC);
  fake_comm_session_init();
  fake_event_reset_count();

  Transport *transport = fake_transport_create(TransportDestinationSystem, NULL,
                                               prv_system_msg_sent_callback);
  s_session = fake_transport_set_connected(transport, true /* connected */);
  cl_assert_equal_p(comm_session_get_system_session(), s_session);

  prv_process_and_reset_test_counters();
  s_last_response_cookie = 0;
  s_boot_bits_orred = 0;
  s_do_before_write = NULL;

  // Common for most tests:
  s_firmware_update_is_in_progress = true;

  fake_spi_flash_init(0, 0x1000000);

  put_bytes_init();
}

void test_put_bytes__cleanup(void) {
  put_bytes_deinit();

  fake_comm_session_cleanup();
  fake_system_task_callbacks_cleanup();
  fake_event_clear_last();

  fake_pbl_malloc_check_net_allocs();
  fake_pbl_malloc_clear_tracking();
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Misc


static pbl_tick_t prv_taking_too_long_yield_cb(struct pbl_sem *queue) {
  return milliseconds_to_ticks(1000);
}

void test_put_bytes__lock_contention_upon_prepare_message(void) {
  // When the PutBytes lock is taken for a long time when a PutBytes message is prepared,
  // expect to receive a Nack:

  // Take and hold for a long time:
  pbl_sem_take(put_bytes_get_semaphore(), PBL_FOREVER);
  fake_sem_set_yield_callback(put_bytes_get_semaphore(), prv_taking_too_long_yield_cb);

  prv_receive_init(4, ObjectFirmware);

  // Release it:
  pbl_sem_give(put_bytes_get_semaphore());
  fake_sem_set_yield_callback(put_bytes_get_semaphore(), NULL);

  assert_nack_count(1);
}

static void prv_hold_lock_before_write(void) {
  // Take and hold for a long time:
  pbl_sem_take(put_bytes_get_semaphore(), PBL_FOREVER);
  fake_sem_set_yield_callback(put_bytes_get_semaphore(), prv_taking_too_long_yield_cb);
}

void test_put_bytes__lock_contention_upon_write_message(void) {
  // When the PutBytes lock is taken for a long time when a PutBytes message is written,
  // expect to receive a Nack:

  s_do_before_write = prv_hold_lock_before_write;

  prv_receive_init(4, ObjectFirmware);

  // Release it:
  pbl_sem_give(put_bytes_get_semaphore());
  fake_sem_set_yield_callback(put_bytes_get_semaphore(), NULL);

  assert_nack_count(1);
}

static void prv_cancel_before_write_second_message(void) {
  put_bytes_cancel();
}

void test_put_bytes__cancel_between_prepare_and_finish(void) {
  // When the put_bytes_cancel() is called while the PutBytes message is written (between "prepare"
  // and "finish"), expect to receive a Nack:

  prv_receive_init(4, ObjectWatchApp);
  assert_ack_count(1);
  assert_nack_count(0);

  s_do_before_write = prv_cancel_before_write_second_message;

  const uint8_t payload[] = { 0xaa, 0xbb, 0xcc };
  prv_receive_put(s_last_response_cookie, payload, sizeof(payload));

  assert_nack_count(1);
}

void test_put_bytes__invalid_command_opcode(void) {
  uint8_t invalid_cmd[] = { CmdInvalid };
  prv_receive_data(s_session, (const uint8_t *) invalid_cmd, sizeof(invalid_cmd));

  // Messages with invalid command opcodes are NACK'd:
  assert_ack_count(0);
  assert_nack_count(1);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Init Message

void test_put_bytes__init_firmware(void) {
  prv_receive_init(VALID_OBJECT_SIZE, ObjectFirmware);

  // All good!
  assert_ack_count(1);
  assert_nack_count(0);

  // Expect "Start" event:
  PebbleEvent event = fake_event_get_last();
  cl_assert_equal_i(event.type, PEBBLE_PUT_BYTES_EVENT);
  cl_assert_equal_i(event.put_bytes.type, PebblePutBytesEventTypeStart);
  cl_assert_equal_i(event.put_bytes.object_type, ObjectFirmware);
  cl_assert_equal_i(event.put_bytes.total_size, VALID_OBJECT_SIZE);
  cl_assert_equal_i(event.put_bytes.progress_percent, 0);
  cl_assert_equal_b(event.put_bytes.failed, false);
}

void test_put_bytes__init_while_already_busy(void) {
  prv_receive_init(VALID_OBJECT_SIZE, ObjectFirmware);
  prv_receive_init(VALID_OBJECT_SIZE, ObjectFirmware);
  assert_nack_count(1);
}

void test_put_bytes__init_too_large(void) {
  prv_receive_init(UINT_MAX, ObjectFirmware);

  // Fail due to massive total_size in our init message
  assert_ack_count(0);
  assert_nack_count(1);
}

void test_put_bytes__init_msg_incomplete(void) {
  const uint8_t incomplete_init_msg = CmdInit;
  prv_receive_data(s_session, (const uint8_t *) &incomplete_init_msg,
                                  sizeof(incomplete_init_msg));
  assert_ack_count(0);
  assert_nack_count(1);
}

void test_put_bytes__init_invalid_object_type(void) {
  PutBytesObjectType invalid_object_type = 0xff;
  prv_receive_init(VALID_OBJECT_SIZE, invalid_object_type);

  // Fail due to massive total_size in our init message
  assert_ack_count(0);
  assert_nack_count(1);
}

void test_put_bytes__init_firmware_object_while_not_in_fw_update_mode(void) {
  s_firmware_update_is_in_progress = false;
  prv_receive_init(VALID_OBJECT_SIZE, ObjectFirmware);
  assert_ack_count(0);
  assert_nack_count(1);
}

void test_put_bytes__init_recovery_object_while_not_in_fw_update_mode(void) {
  s_firmware_update_is_in_progress = false;
  prv_receive_init(VALID_OBJECT_SIZE, ObjectRecovery);
  assert_ack_count(0);
  assert_nack_count(1);
}

void test_put_bytes__init_sys_resources_object_while_not_in_fw_update_mode(void) {
  s_firmware_update_is_in_progress = false;
  prv_receive_init(VALID_OBJECT_SIZE, ObjectSysResources);
  assert_ack_count(0);
  assert_nack_count(1);
}

void test_put_bytes__init_app_resources_okay_while_not_in_fw_update_mode(void) {
  s_firmware_update_is_in_progress = false;
  prv_receive_init_cookie(VALID_OBJECT_SIZE, ObjectAppResources, EXPECTED_COOKIE);
  assert_ack_count(1);
  assert_nack_count(0);

  cl_assert_equal_i(EXPECTED_COOKIE, put_bytes_get_index());
}

void test_put_bytes__init_watch_app_okay_while_not_in_fw_update_mode(void) {
  s_firmware_update_is_in_progress = false;
  prv_receive_init_cookie(VALID_OBJECT_SIZE, ObjectWatchApp, EXPECTED_COOKIE);
  assert_ack_count(1);
  assert_nack_count(0);

  cl_assert_equal_i(EXPECTED_COOKIE, put_bytes_get_index());
}

void test_put_bytes__init_file_okay_while_not_in_fw_update_mode(void) {
  s_firmware_update_is_in_progress = false;
  const char fn[] = "test.txt";
  prv_receive_init_file(VALID_OBJECT_SIZE, fn, strlen(fn) + 1);
  assert_ack_count(1);
  assert_nack_count(0);
}

void test_put_bytes__init_worker_okay_while_not_in_fw_update_mode(void) {
  s_firmware_update_is_in_progress = false;
  prv_receive_init_cookie(VALID_OBJECT_SIZE, ObjectWatchWorker, EXPECTED_COOKIE);
  assert_ack_count(1);
  assert_nack_count(0);

  cl_assert_equal_i(EXPECTED_COOKIE, put_bytes_get_index());
}

void test_put_bytes__init_nack_upon_oom(void) {
  fake_malloc_set_largest_free_block(1024);  // PutBytes allocates ~2K
  prv_receive_init(1024 * 1024, ObjectFirmware);

  fake_malloc_set_largest_free_block(~0);
  assert_ack_count(0);
  assert_nack_count(1);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Put Message

void test_put_bytes__put_message_too_short(void) {
  prv_receive_init_fw_object();
  prv_process_and_reset_test_counters();

  const uint8_t incomplete_put_msg = CmdPut;
  prv_receive_data(s_session, (const uint8_t *) &incomplete_put_msg,
                                  sizeof(incomplete_put_msg));
  assert_ack_count(0);
  assert_nack_count(1);
}

void test_put_bytes__put_message_length_field_too_long(void) {
  prv_receive_init_fw_object();
  prv_process_and_reset_test_counters();

  const size_t payload_size = 2;
  const uint8_t chunk[] = { 0xaa, 0xbb };
  uint8_t buffer[sizeof(PutRequest) + payload_size];

  PutRequest *put_msg = (PutRequest *)buffer;
  *put_msg = (PutRequest) {
    .cmd = CmdPut,
    .cookie = htonl(s_last_response_cookie),
    .payload_size = htonl(payload_size) + 1 /* one off! */,
  };
  memcpy(&put_msg->payload[0], chunk, payload_size);
  prv_receive_data(s_session, buffer, sizeof(buffer));

  assert_ack_count(0);
  assert_nack_count(1);
}

void test_put_bytes__invalid_session_cookie(void) {
  prv_receive_init_fw_object();
  prv_process_and_reset_test_counters();

  const uint8_t chunk[] = { 0xaa, 0xbb, 0xcc };
  prv_receive_put(~s_last_response_cookie, chunk, sizeof(chunk));

  assert_ack_count(0);
  assert_nack_count(1);
}

void test_put_bytes__not_in_fw_update_mode(void) {
  prv_receive_init_fw_object();
  prv_process_and_reset_test_counters();

  s_firmware_update_is_in_progress = false;

  const uint8_t chunk[] = { 0xaa, 0xbb, 0xcc };
  prv_receive_put(s_last_response_cookie, chunk, sizeof(chunk));

  assert_ack_count(0);
  assert_nack_count(1);
}

void test_put_bytes__previous_chunk_not_acked_yet(void) {
  uint8_t max_put_ops = prv_put_bytes_get_max_batched_pb_ops();
  prv_receive_init(VALID_OBJECT_SIZE * max_put_ops, ObjectFirmware);
  prv_process_and_reset_test_counters();

  const uint8_t chunk[] = { 0xaa, 0xbb, 0xcc };
  uint8_t max_pb_ops = prv_put_bytes_get_max_batched_pb_ops();
  for (int i = 0; i <= max_pb_ops; i++) {
    prv_receive_put(s_last_response_cookie, chunk, sizeof(chunk));
  }

  assert_ack_count(max_pb_ops);
  assert_nack_count(1);
}

void test_put_bytes__chunk_too_large(void) {
  prv_receive_init_fw_object();
  prv_process_and_reset_test_counters();

  size_t chunk_size = 1024 * 1024;
  uint8_t *chunk = kernel_malloc(chunk_size);
  prv_receive_put(s_last_response_cookie, chunk, chunk_size);
  kernel_free(chunk);

  assert_ack_count(0);
  assert_nack_count(1);
}

void test_put_bytes__app_cancelled_before_chunk_got_processed(void) {
  prv_receive_init_cookie(VALID_OBJECT_SIZE, ObjectWatchApp, EXPECTED_COOKIE);
  prv_process_and_reset_test_counters();

  const uint8_t chunk[] = { 0xaa, 0xbb, 0xcc };
  prv_receive_put(s_last_response_cookie, chunk, sizeof(chunk));

  put_bytes_cancel();

  assert_cleanup_event(ObjectWatchApp, VALID_OBJECT_SIZE);

  if (prv_put_bytes_get_max_batched_pb_ops() > 1) {
    // With pre-acking, the put will have already been ack'ed and then a Nack will follow
    assert_ack_count(1);
  } else {
    assert_ack_count(0);
  }
  assert_nack_count(1);
}

void test_put_bytes__chunk_written_to_storage_and_progress_event_put(void) {
  prv_receive_init_fw_object();
  prv_process_and_reset_test_counters();

  const uint8_t chunk[] = { 0xaa, 0xbb, 0xcc };
  prv_receive_put(s_last_response_cookie, chunk, sizeof(chunk));

  assert_ack_count(1);
  assert_nack_count(0);

  fake_pb_storage_mem_assert_contents_written(chunk, sizeof(chunk));

  PebbleEvent event = fake_event_get_last();
  cl_assert_equal_i(event.type, PEBBLE_PUT_BYTES_EVENT);
  cl_assert_equal_i(event.put_bytes.type, PebblePutBytesEventTypeProgress);
  cl_assert_equal_i(event.put_bytes.object_type, ObjectFirmware);
  cl_assert_equal_i(event.put_bytes.bytes_transferred, sizeof(chunk));
  cl_assert_equal_i(event.put_bytes.progress_percent, 100 * sizeof(chunk) / VALID_OBJECT_SIZE);
  cl_assert_equal_b(event.put_bytes.failed, false);
}

static uint32_t s_next_value_to_write;
static void prv_cb_before_write(void) {
  prv_receive_put(s_last_response_cookie, (uint8_t *)&s_next_value_to_write, VALID_OBJECT_SIZE);
}

void test_put_bytes__receive_batched_messages(void) {
  uint8_t max_batched_ops = prv_put_bytes_get_max_batched_pb_ops();
  int num_ops = 500;

  if (max_batched_ops < 2) { // This race condition is not possible if we aren't pre-Acking
    return;
  }

  prv_receive_init(VALID_OBJECT_SIZE * num_ops, ObjectFirmware);
  fake_comm_session_process_send_next();
  fake_system_task_callbacks_invoke_pending();

  uint8_t buffer[num_ops * VALID_OBJECT_SIZE];
  for (size_t i = 0; i < sizeof(buffer); i += VALID_OBJECT_SIZE) {
    uint32_t towrite = i;
    memcpy(&buffer[i], &towrite, sizeof(towrite));
  }

  // Make sure we can receive new data in the middle of a pb_storage_append operation
  for (int i = 0; i < num_ops; i += 2) {
    int idx = i * VALID_OBJECT_SIZE;
    prv_receive_put(s_last_response_cookie, &buffer[idx], VALID_OBJECT_SIZE);

    idx += VALID_OBJECT_SIZE;
    memcpy(&s_next_value_to_write, &buffer[idx], VALID_OBJECT_SIZE);
    fake_pb_storage_register_cb_before_write(prv_cb_before_write);

    fake_comm_session_process_send_next();
    fake_system_task_callbacks_invoke_pending();
  }

  fake_pb_storage_mem_assert_contents_written(buffer, sizeof(buffer));
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Commit Message

void test_put_bytes__commit_message_too_short(void) {
  prv_receive_init_and_put_fw_object();

  const uint8_t incomplete_put_msg = CmdCommit;
  prv_receive_data(s_session, (const uint8_t *) &incomplete_put_msg,
                                  sizeof(incomplete_put_msg));
  assert_ack_count(0);
  assert_nack_count(1);
}

void test_put_bytes__commit_message_sent_while_previous_put_was_not_acked_yet(void) {
  uint8_t max_put_ops = prv_put_bytes_get_max_batched_pb_ops();
  prv_receive_init(VALID_OBJECT_SIZE * max_put_ops, ObjectFirmware);
  prv_process_and_reset_test_counters();

  const uint8_t chunk[] = { 0xaa, 0xbb, 0xcc, 0xdd };
  for (int i = 0; i < max_put_ops; i++) {
    prv_receive_put(s_last_response_cookie, chunk, sizeof(chunk));
  }

  prv_receive_commit(s_last_response_cookie, EXPECTED_CRC);
  assert_ack_count(max_put_ops);  // For the Put(s)
  assert_nack_count(1); // For the Commit
}

void test_put_bytes__commit_message_cookie_mismatch(void) {
  prv_receive_init_and_put_fw_object();

  prv_receive_commit(~s_last_response_cookie, EXPECTED_CRC);
  assert_ack_count(0);
  assert_nack_count(1);
}

void test_put_bytes__commit_message_while_not_in_fw_update_mode(void) {
  prv_receive_init_and_put_fw_object();

  s_firmware_update_is_in_progress = false;
  prv_receive_commit(s_last_response_cookie, EXPECTED_CRC);
  assert_ack_count(0);
  assert_nack_count(1);
}

void test_put_bytes__commit_message_crc_mismatch(void) {
  prv_receive_init_and_put_fw_object();

  prv_receive_commit(s_last_response_cookie, ~EXPECTED_CRC);
  assert_ack_count(0);
  assert_nack_count(1);
}

void test_put_bytes__commit_message_fw_description_is_written(void) {
  prv_receive_init_and_put_fw_object();
  prv_receive_commit(s_last_response_cookie, EXPECTED_CRC);
  fake_comm_session_process_send_next();
  fake_system_task_callbacks_invoke_pending();

  // Assert the FW description got written at the beginning of the storage:
  const FirmwareDescription fw_descr = {
    .description_length = sizeof(FirmwareDescription),
    .firmware_length = VALID_OBJECT_SIZE,
    .checksum = EXPECTED_CRC,
  };
  fake_pb_storage_mem_assert_fw_description_written(&fw_descr);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Abort Message

void test_put_bytes__abort_message_too_short(void) {
  prv_receive_init_and_put_fw_object();
  prv_process_and_reset_test_counters();

  const uint8_t incomplete_abort_msg = CmdAbort;
  prv_receive_data(s_session, (const uint8_t *) &incomplete_abort_msg,
                                  sizeof(incomplete_abort_msg));
  assert_ack_count(0);
  assert_nack_count(1);
}

void test_put_bytes__abort_message_cookie_mismatch(void) {
  prv_receive_init_and_put_fw_object();
  prv_process_and_reset_test_counters();

  prv_receive_abort(~s_last_response_cookie);

  assert_ack_count(0);
  assert_nack_count(1);
}

void test_put_bytes__abort_message_ok(void) {
  prv_receive_init_and_put_fw_object();
  prv_process_and_reset_test_counters();

  prv_receive_abort(s_last_response_cookie);

  assert_ack_count(1);
  assert_nack_count(0);
  assert_cleanup_event(ObjectFirmware, VALID_OBJECT_SIZE);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Install Message

void test_put_bytes__install_message_while_not_idle(void) {
  prv_receive_init(VALID_OBJECT_SIZE, ObjectFirmware);
  prv_process_and_reset_test_counters();

  prv_receive_install(s_last_response_cookie);
  assert_ack_count(0);
  assert_nack_count(1);
}

void test_put_bytes__install_message_too_short(void) {
  prv_receive_init_put_and_commit_fw_object();

  const uint8_t incomplete_install_msg = CmdInstall;
  prv_receive_data(s_session, (const uint8_t *) &incomplete_install_msg,
                                  sizeof(incomplete_install_msg));
  assert_ack_count(0);
  assert_nack_count(1);
}

void test_put_bytes__install_message_while_not_in_fw_update_mode(void) {
  prv_receive_init_put_and_commit_fw_object();

  s_firmware_update_is_in_progress = false;
  prv_receive_install(s_last_response_cookie);
  assert_ack_count(0);
  assert_nack_count(1);
}

void test_put_bytes__install_message_cookie_mismatch(void) {
  prv_receive_init_put_and_commit_fw_object();
  prv_receive_install(~s_last_response_cookie);
  assert_ack_count(0);
  assert_nack_count(1);
}

void test_put_bytes__install_message_prf_boot_bit_set(void) {
  prv_receive_init_put_commit_and_install(ObjectRecovery);
  assert_ack_count(1);
  assert_nack_count(0);
  cl_assert_equal_i((s_boot_bits_orred & BOOT_BIT_NEW_PRF_AVAILABLE), BOOT_BIT_NEW_PRF_AVAILABLE);
}

void test_put_bytes__install_message_fw_and_sys_resources_boot_bits_set(void) {
  // Firmware object:
  prv_receive_init_put_commit_and_install(ObjectFirmware);
  assert_ack_count(1);
  assert_nack_count(0);

  // Expect boot bit not to be set yet:
  cl_assert_equal_i((s_boot_bits_orred & BOOT_BIT_NEW_FW_AVAILABLE), 0);

  // System Resources object:
  prv_receive_init_put_commit_and_install(ObjectSysResources);
  assert_ack_count(1);
  assert_nack_count(0);

  // Finally, expect both boot bits to be set at once:
  cl_assert_equal_i((s_boot_bits_orred & BOOT_BIT_NEW_FW_AVAILABLE), BOOT_BIT_NEW_FW_AVAILABLE);
  cl_assert_equal_i((s_boot_bits_orred & BOOT_BIT_NEW_SYSTEM_RESOURCES_AVAILABLE),
                    BOOT_BIT_NEW_SYSTEM_RESOURCES_AVAILABLE);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Timeouts

void test_put_bytes__init_starts_timeout_timer(void) {
  prv_receive_init_fw_object();

  TimerID timer_id = put_bytes_get_timer_id();
  cl_assert_equal_b(true, stub_new_timer_is_scheduled(timer_id));
  cl_assert_equal_i(PUT_BYTES_TIMEOUT_MS, stub_new_timer_timeout(timer_id));
}

void test_put_bytes__put_chunk_restarts_timeout_timer(void) {
  prv_receive_init_fw_object();

  // Stop the timer, so we can easily detect it gets restarted again:
  TimerID timer_id = put_bytes_get_timer_id();
  new_timer_stop(timer_id);

  const uint8_t chunk[] = { 0xaa, 0xbb, 0xcc };
  prv_receive_put(s_last_response_cookie, chunk, sizeof(chunk));
  fake_system_task_callbacks_invoke_pending();

  cl_assert_equal_b(true, stub_new_timer_is_scheduled(timer_id));
  cl_assert_equal_i(PUT_BYTES_TIMEOUT_MS, stub_new_timer_timeout(timer_id));
}

void test_put_bytes__after_timeout_cleanup_and_allow_init_again(void) {
  prv_receive_init_fw_object();
  assert_ack_count(1);
  assert_nack_count(0);

  stub_new_timer_fire(put_bytes_get_timer_id());
  fake_system_task_callbacks_invoke_pending();

  cl_assert_equal_b(fake_pb_storage_mem_get_last_success(), false);

  assert_cleanup_event(ObjectFirmware, VALID_OBJECT_SIZE);

  // Send "Init" again:
  prv_receive_init_fw_object();
  assert_ack_count(2);
  assert_nack_count(0);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// put_bytes_expect_init

void test_put_bytes__expect_init_noop_while_not_idle(void) {
  cl_assert(EXPECT_INIT_TIMEOUT_MS != PUT_BYTES_TIMEOUT_MS);

  prv_receive_init_fw_object();

  put_bytes_expect_init(EXPECT_INIT_TIMEOUT_MS);

  // The timer is still not overridden by the "expect_init" timer:
  cl_assert_equal_i(stub_new_timer_timeout(put_bytes_get_timer_id()), PUT_BYTES_TIMEOUT_MS);
}

void test_put_bytes__expect_init_no_event_when_init_received(void) {
  put_bytes_expect_init(EXPECT_INIT_TIMEOUT_MS);

  prv_receive_init_fw_object();

  // The timer is overridden by the 30s Put Bytes timeout:
  cl_assert_equal_i(stub_new_timer_timeout(put_bytes_get_timer_id()), PUT_BYTES_TIMEOUT_MS);

  fake_event_reset_count();
  stub_new_timer_fire(put_bytes_get_timer_id());
  fake_system_task_callbacks_invoke_pending();

  // Expect only "Cleanup" event:
  cl_assert_equal_i(fake_event_get_count(), 1);
  assert_cleanup_event(ObjectFirmware, VALID_OBJECT_SIZE);
}

void test_put_bytes__expect_init_event_upon_timeout(void) {
  put_bytes_expect_init(EXPECT_INIT_TIMEOUT_MS);

  stub_new_timer_fire(put_bytes_get_timer_id());

  PebbleEvent event = fake_event_get_last();
  cl_assert_equal_i(event.type, PEBBLE_PUT_BYTES_EVENT);
  cl_assert_equal_i(event.put_bytes.type, PebblePutBytesEventTypeInitTimeout);
  cl_assert_equal_i(event.put_bytes.object_type, ObjectUnknown);
  cl_assert_equal_i(event.put_bytes.total_size, 0);
  cl_assert_equal_i(event.put_bytes.progress_percent, 0);
  cl_assert_equal_b(event.put_bytes.failed, true);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// put_bytes_handle_remote_app_event

void test_put_bytes__session_closed_after_fw_init(void) {
  prv_receive_init_fw_object();

  PebbleCommSessionEvent app_event = {
    .is_open = false,
    .is_system = true
  };

  // Close the BT session, have put_bytes react
  put_bytes_handle_comm_session_event(&app_event);
  fake_system_task_callbacks_invoke_pending();

  assert_cleanup_event(ObjectFirmware, VALID_OBJECT_SIZE);
}


void test_put_bytes__session_closed_after_expect_init(void) {
  put_bytes_expect_init(EXPECT_INIT_TIMEOUT_MS);

  PebbleCommSessionEvent app_event = {
    .is_open = false,
    .is_system = true
  };

  // Close the BT session, have put_bytes react
  put_bytes_handle_comm_session_event(&app_event);
  fake_system_task_callbacks_invoke_pending();

  assert_cleanup_event(ObjectUnknown, 0);
}
