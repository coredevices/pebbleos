/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "pbl/kernel/irq.h"
#include "pbl/services/put_bytes/put_bytes.h"
#include "pbl/services/put_bytes/put_bytes_storage.h"

#include "comm/bluetooth_analytics.h"
#include "kernel/events.h"
#include "kernel/pbl_malloc.h"
#include "kernel/system_message.h"
#include "pbl/kernel/types.h"
#include "resource/resource_storage_file.h"
#include "pbl/services/comm_session/session.h"
#include "pbl/services/comm_session/session_receive_router.h"
#include "pbl/services/firmware_update.h"
#include "pbl/services/new_timer/new_timer.h"
#include "pbl/services/system_task.h"
#include "pbl/services/process_management/app_storage.h"
#include "system/bootbits.h"
#include "system/firmware_storage.h"
#include <pbl/logging/logging.h>
#include "system/passert.h"
#include "pbl/util/attributes.h"
#include "pbl/util/math.h"
#include "util/net.h"
#include <bluetooth/analytics.h>

#include "pbl/kernel/sem.h"

#include <string.h>

PBL_LOG_MODULE_DEFINE(service_put_bytes, CONFIG_SERVICE_PUT_BYTES_LOG_LEVEL);

typedef enum {
  PutBytesIdle = 0x00,
  PutBytesInit = 0x01,
  PutBytesPut = 0x02,
  PutBytesCommit = 0x03,
  PutBytesAbort = 0x04,
  PutBytesInstall = 0x05,
  NumPutBytesCommands
} PutBytesCommand;

typedef struct PACKED {
  uint32_t init_req_magic;
  uint32_t append_offset;
} InitRequestExtraInfo;

typedef struct PACKED {
  PutBytesCommand cmd:8;
  uint32_t total_size;
  PutBytesObjectType type:7;
  bool has_cookie:1;
  union {
    struct {
      const uint8_t index; ///< The 0-indexed position for 'banked' objects.
      char filename[];
    };
    uint32_t cookie;
  };
  //! Note: 'filename' can be variable length so this field can't be safely accessed directly
  //! but rather should be recovered by the size of the blob this request is packed in
  InitRequestExtraInfo extra_info;
} InitRequest;

typedef struct PACKED {
  PutBytesCommand cmd:8;
  uint32_t token;
} SharedHeader;

typedef struct PACKED {
  SharedHeader header;
  uint32_t length;
  uint8_t data[];
} PutRequest;

typedef struct PACKED {
  SharedHeader header;
  uint32_t crc;
} CommitRequest;

typedef struct PACKED {
  SharedHeader header;
} AbortRequest;

typedef struct PACKED {
  SharedHeader header;
} InstallRequest;

static const uint16_t PB_ENDPOINT_ID = 0xBEEF;
static const uint32_t PUT_TIMEOUT_MS = 30000;

//! @note The 2044 bytes is historically the size of the biggest allowable chunk of data with the
//! "Put" request message. There is no fundamental reason why this could not be bigger, but today
//! there is no way to communicate the maximum allowable chunk size, it's been hard-coded in the
//! clients (i.e. mobile app, libpebble, ...) so it would require a protocol change / extension
//! to increase this. We could experiment with increasing this in the future for slightly faster
//! object transfers (less time spent ACK'ing).
static const size_t PUT_BYTES_PP_BUFFER_SIZE = (2044 + sizeof(PutRequest));

typedef enum {
  ResponseAck = 0x01,
  ResponseNack = 0x02,
  NumResponseCodes
} ResponseCode;

#define MAX_BATCHED_PB_PUT_OPS 3

typedef struct {
  uint8_t *buffer;
  uint32_t request_length;
} PutBytesJob;

typedef struct {
  bool enable_preack;
  bool need_to_ack_later;
  uint8_t num_allocated_pb_jobs;
  PutBytesJob job[MAX_BATCHED_PB_PUT_OPS];
  uint8_t read_idx;
  uint8_t num_ops_pending;
} PutBytesPendingJobs;

typedef struct {
  TimerID timer_id;

  uint32_t token;
  PutBytesObjectType type;
  bool has_cookie;
  uint32_t index;
  PutBytesCommand current_command;
  bool is_success;

  uint32_t total_size;
  uint32_t append_offset;
  uint32_t remaining_bytes;
  PutBytesStorage storage;

  //! the time in ticks at which the put bytes init request completed
  RtcTicks start_ticks;
  SlaveConnEventStats conn_event_stats;

  //! Holds PB commands. Will enqueue multiple PutRequests when pre-acking is enabled
  PutBytesPendingJobs pb_pending_jobs;

  //! Storage for the Pebble Protocol ReceiverImplementation:
  struct {
    //! Backing storage for the received Put Bytes message. This buffer points to one of the buffers
    //! allocated within `pb_pending_jobs`
    uint8_t *buffer;

    //! The length in bytes of the message in `buffer`. When the message is handled, this must
    //! be reset to 0, to indicate no message is pending processing.
    uint32_t length;

    //! The position into `buffer` where to write the next received Pebble Protocol data.
    uint32_t pos;

    //! True if the message should be NACK'd without even processing it.
    //! This field should only be accessed from BT02 and therefore requires no locking.
    bool should_nack;
  } receiver;
} PutBytesState;

static PutBytesState s_pb_state;

static struct InstallableObject {
  uint32_t token;
  //! The type of the installable object.
  //! Also doubles as a marker for whether this object type has been committed recently,
  //! see prv_set_fw_update_bootbits_if_completed()
  PutBytesObjectType type;
  uint32_t index;
} s_ready_to_install[NumObjects];

static PBL_SEM_DEFINE(s_pb_semaphore, 1, 1);

//! Marks that the receiver state is now free to use
static void prv_receiver_reset(void);

static void prv_send_response(ResponseCode code, uint32_t token);

static void prv_lock_pb_job_state(void) {
  pbl_irq_lock();
}

static void prv_unlock_pb_job_state(void) {
  pbl_irq_unlock();
}

//! Simply returns the next free buffer from the PB jobs array. Returns NULL if none are available
static uint8_t *prv_get_next_pb_job_buffer(void) {
  PutBytesPendingJobs *put_jobs = &s_pb_state.pb_pending_jobs;

  uint8_t write_idx, jobs_pending;
  bool enable_preack;
  prv_lock_pb_job_state();
  {
    write_idx = (put_jobs->read_idx + put_jobs->num_ops_pending) % put_jobs->num_allocated_pb_jobs;
    enable_preack = put_jobs->enable_preack;
    jobs_pending = put_jobs->num_ops_pending;
  }
  prv_unlock_pb_job_state();

  // If pre-acking is disabled, only one request can be in flight at any given time!
  if (!enable_preack && (jobs_pending > 0)) {
    return NULL;
  }

  if (jobs_pending == put_jobs->num_allocated_pb_jobs) {
    return NULL; // Remote has sent data without us ACKing the previous payload!
  }

  return put_jobs->job[write_idx].buffer;
}

//! Marks the PB job as written to
static void prv_finalize_pb_job(void) {
  PutBytesPendingJobs *put_jobs = &s_pb_state.pb_pending_jobs;

  PutBytesJob *job;
  prv_lock_pb_job_state();
  {
    PBL_ASSERTN(put_jobs->num_ops_pending != put_jobs->num_allocated_pb_jobs);
    uint8_t write_idx = (put_jobs->read_idx + put_jobs->num_ops_pending) %
        put_jobs->num_allocated_pb_jobs;
    job = &put_jobs->job[write_idx];
    job->request_length = s_pb_state.receiver.length;
    put_jobs->num_ops_pending++;
  }
  prv_unlock_pb_job_state();
}

//! Frees up 'num_jobs' in the put_jobs array
static void prv_mark_pb_jobs_complete(size_t num_jobs) {
  prv_lock_pb_job_state();
  {
    PutBytesPendingJobs *put_jobs = &s_pb_state.pb_pending_jobs;
    PBL_ASSERTN(num_jobs <= put_jobs->num_ops_pending);
    put_jobs->num_ops_pending -= num_jobs;
    put_jobs->read_idx = (put_jobs->read_idx + num_jobs) % put_jobs->num_allocated_pb_jobs;
  }
  prv_unlock_pb_job_state();
}

//! Only to be called by prv_receiver_write() when a new PutBytesPut starts to roll in
static void prv_pre_ack_if_space_in_put_job_queue(void) {
  PutBytesPendingJobs *put_jobs = &s_pb_state.pb_pending_jobs;

  PBL_ASSERTN(s_pb_state.receiver.buffer[0] == PutBytesPut);

  bool pre_ack = false;
  prv_lock_pb_job_state();
  {
    if (put_jobs->enable_preack) {
      // Don't Pre-Ack if the payload that is arriving will fill our job queue
      pre_ack = ((put_jobs->num_ops_pending + 1) < put_jobs->num_allocated_pb_jobs);
    }
    put_jobs->need_to_ack_later = !pre_ack;
  }
  prv_unlock_pb_job_state();

  if (pre_ack) {
    prv_send_response(ResponseAck, s_pb_state.token);
  } else if (put_jobs->enable_preack) {
    PBL_LOG_DBG("Not enough buffer room to pre-ack PB packet");
  }
}

static void prv_deinit_put_job_queue(void) {
  PutBytesPendingJobs *put_jobs = &s_pb_state.pb_pending_jobs;
  for (int i = 0; i < MAX_BATCHED_PB_PUT_OPS; i++) {
    kernel_free(put_jobs->job[i].buffer);
    put_jobs->job[i].buffer = NULL;
  }
}

static bool prv_init_put_job_queue_if_necessary(void) {
  PutBytesPendingJobs *put_jobs = &s_pb_state.pb_pending_jobs;

  // Things are already initialized if at least the first job buffer is non-NULL
  if (put_jobs->job[0].buffer != NULL) {
    return true;
  }

  put_jobs->enable_preack = true;

  int i;
  for (i = 0; i < MAX_BATCHED_PB_PUT_OPS; i++) {
    // Note: If heap pressure becomes an issue, we could also consider only
    // using pre-acking if there is a certain amount of space free in the heap
    uint8_t *buffer = (uint8_t *) kernel_zalloc(PUT_BYTES_PP_BUFFER_SIZE);
    if (!buffer) {
      if (i == 0) {
        PBL_LOG_ERR("Not enough memory to service PB request, abort!");
        prv_deinit_put_job_queue();
        return false;
      } if (i == 1) {
        PBL_LOG_DBG("Not enough memory for PB pre-ack, falling back to legacy mode");
        put_jobs->enable_preack = false;
        break;
      } else {
        break;
      }
    }
    put_jobs->job[i].buffer = buffer;
  }
  put_jobs->num_allocated_pb_jobs = i;
  return true;
}

static void prv_set_responsiveness(ResponseTimeState state, uint16_t timeout_secs) {
  comm_session_set_responsiveness(comm_session_get_system_session(),
                                  BtConsumerPpPutBytes, state, timeout_secs);
}

static void prv_send_nack_from_system_task(void *data) {
  const uint32_t token = (uintptr_t)data;
  prv_send_response(ResponseNack, token);
}

static void prv_add_nack_system_callback(uint32_t token) {
  system_task_add_callback(prv_send_nack_from_system_task, (void *)(uintptr_t)token);
}

static void prv_add_nack_no_token_system_callback(void) {
  prv_add_nack_system_callback(0);
}

static void prv_cleanup(void) {
  PBL_LOG_DBG("Put bytes cleanup. Tok: %"PRIu32, s_pb_state.token);

  prv_deinit_put_job_queue();
  s_pb_state.receiver = (__typeof__(s_pb_state.receiver)) {};

  if (s_pb_state.timer_id) {
    new_timer_delete(s_pb_state.timer_id);
    s_pb_state.timer_id = TIMER_INVALID_ID;
  }

  pb_storage_deinit(&s_pb_state.storage, s_pb_state.is_success);

  // Stay at ResponseTimeMin for a bit so that we don't force a quick transition between
  // Min -> Max -> Min. The Dialog chip would disconnect with reasons 0x1f. Also, it doesn't really
  // make sense to transition for just 2 seconds anyways. However, during an App/File install
  // PutBytes, we will stay at Min for an extra 10 seconds after the entire transaction is
  // completed. Marginal power hit, but shouldn't happen often since PutBytes itself doesn't
  // happen too often.
  prv_set_responsiveness(ResponseTimeMin, 10);

  PebbleEvent event = {
    .type = PEBBLE_PUT_BYTES_EVENT,
    .put_bytes = {
      .type = PebblePutBytesEventTypeCleanup,
      .object_type = s_pb_state.type,
      .has_cookie = s_pb_state.has_cookie,
      .progress_percent = 0,
      .total_size = s_pb_state.total_size,
      .failed = !s_pb_state.is_success,
    },
  };

  event_put(&event);

  // NOTE: Preserve the type field because that is checked by the install handler after we
  //  cleanup (cleanup is called after a commit).
  PutBytesObjectType type = s_pb_state.type;
  s_pb_state = (PutBytesState) {
    .type = type
  };
}

static void prv_cleanup_from_system_task(void* data) {
  pbl_sem_take(&s_pb_semaphore, PBL_FOREVER);
  prv_cleanup();
  pbl_sem_give(&s_pb_semaphore);
}

static void prv_cleanup_async(void) {
  system_task_add_callback(prv_cleanup_from_system_task, NULL);
}

static void prv_fail(uint32_t token) {
  prv_cleanup_async();
  prv_add_nack_system_callback(token);
}

static void prv_timer_callback(void* data) {
  PBL_LOG_WRN("Put bytes Tok: %"PRIu32" timed out after %"PRIu32"ms, cleaning up.",
      s_pb_state.token, PUT_TIMEOUT_MS);
  prv_cleanup_async();
}

static bool prv_has_valid_fw_update_state_for_object_type(PutBytesObjectType type) {
#ifndef CONFIG_RECOVERY_FW
  if (!firmware_update_is_in_progress()) {
    bool is_fw_update_object = (type == ObjectFirmware ||
                                type == ObjectRecovery ||
                                type == ObjectSysResources);
    if (is_fw_update_object) {
      PBL_LOG_ERR("Cannot handle object type=<0x%x> when not in FW update mode", type);
      return false;
    }
  }
  return true;
#else // safe mode FW
  return true;
#endif
}

static bool prv_has_invalid_fw_update_state(const PutBytesCommand command) {
  if (command == PutBytesAbort ||
      command == PutBytesInit) {
    return false;
  }
  // Check only for Put, Commit, Install
  return !prv_has_valid_fw_update_state_for_object_type(s_pb_state.type);
}

static unsigned int prv_get_progress_percent(void) {
  return ((s_pb_state.total_size - s_pb_state.remaining_bytes) * 100) / (s_pb_state.total_size);
}

static void prv_send_response(ResponseCode code, uint32_t token) {
  struct {
    uint8_t response_code;
    uint32_t token;
  } PACKED msg = {
    .response_code = code,
    .token = htonl(token)
  };

  bool success = comm_session_send_data(comm_session_get_system_session(), PB_ENDPOINT_ID,
                                        (uint8_t*) &msg, sizeof(msg), COMM_SESSION_DEFAULT_TIMEOUT);
  if (!success) {
    PBL_LOG_WRN("PutBytes timeout sending response");
  }
}

static void prv_cleanup_and_send_response(ResponseCode code) {
  // Save this value, as it'll be cleaned up by prv_cleanup but we'll need them to send the
  // response. We want to cleanup first before sending the response so that we tell the phone
  // that we're ready for the next message after we've done all of our housekeeping.
  uint32_t token = s_pb_state.token;
  prv_cleanup();
  prv_send_response(code, token);
}

static void prv_commit_object(uint32_t crc) {
#ifndef CONFIG_PBLBOOT
  if (s_pb_state.type == ObjectFirmware || s_pb_state.type == ObjectRecovery) {
    FirmwareDescription fw_descr = {
      .description_length = sizeof(FirmwareDescription),
      .firmware_length = s_pb_state.total_size + s_pb_state.append_offset,
      .checksum = crc
    };

    fw_descr.checksum = pb_storage_calculate_crc(&s_pb_state.storage, PutBytesCrcType_CRC32);
    pb_storage_write(&s_pb_state.storage, 0, (uint8_t *)&fw_descr, sizeof(FirmwareDescription));
  }
#endif

  struct InstallableObject* o = s_ready_to_install + (s_pb_state.type - 1);
  o->token = s_pb_state.token;
  o->type = s_pb_state.type;
  o->index = s_pb_state.index;
}

static void prv_finish_fw_update_if_completed(void) {
  if (s_ready_to_install[ObjectFirmware - 1].type != ObjectFirmware ||
      s_ready_to_install[ObjectSysResources - 1].type != ObjectSysResources) {
    return;  // Haven't received both FW and System Resources yet
  }
  PBL_LOG_DBG("Got both FW bin and sys resources!");

  s_ready_to_install[ObjectFirmware - 1].type = 0;
  s_ready_to_install[ObjectSysResources - 1].type = 0;

  bool set_res_bit = true;

  if (set_res_bit) {
    boot_bit_set(BOOT_BIT_NEW_SYSTEM_RESOURCES_AVAILABLE);
  }

  boot_bit_set(BOOT_BIT_NEW_FW_AVAILABLE);
}

static void prv_do_install(uint32_t token) {
  struct InstallableObject* o = NULL;
  for (int i = 0; i < NumObjects; ++i) {
    if (s_ready_to_install[i].token == token) {
      o = &s_ready_to_install[i];
      break;
    }
  }

  if (token == 0 || o == NULL) {
    PBL_LOG_ERR("Token does not exist; got 0x%" PRIx32, token);
    prv_cleanup_and_send_response(ResponseNack);
    return;
  }

  PBL_LOG_DBG("PutBytes install CB. Tok: %"PRIu32", type: %d", token, o->type);

  switch (o->type) {
  case ObjectFirmware:
  case ObjectSysResources:
    prv_finish_fw_update_if_completed();
    break;
  case ObjectRecovery:
    boot_bit_set(BOOT_BIT_NEW_PRF_AVAILABLE);
    // >>> Fall-through! <<<
  default:
    break;
  }

  o->token = 0;
  s_pb_state.is_success = true;
  PBL_LOG_VERBOSE("Installation succeeded!");

  prv_mark_pb_jobs_complete(1);
  // Clean up the current command state before sending an ACK
  prv_cleanup_and_send_response(ResponseAck);
}

static void prv_do_abort(void) {
  PBL_LOG_DBG("PutBytes abort CB. Tok: %"PRIu32".", s_pb_state.token);
  prv_mark_pb_jobs_complete(1);
  prv_cleanup_and_send_response(ResponseAck);
}

static bool prv_has_invalid_token(const PutBytesCommand command, uint32_t request_token) {
  if (command == PutBytesInit ||
      command == PutBytesInstall) {
    return false;
  }
  if (s_pb_state.token != request_token) {
    PBL_LOG_ERR("%d: Token does not match; got 0x%" PRIx32 ", expected 0x%" PRIx32,
            command, request_token, s_pb_state.token);
    return true;
  }
  return false;
}

static size_t prv_expected_minimum_length_by_command(const PutBytesCommand command) {
  switch (command) {
    case PutBytesInit:
      return offsetof(InitRequest, index);
    case PutBytesPut:
      return sizeof(PutRequest);
    case PutBytesCommit:
      return sizeof(CommitRequest);
    case PutBytesAbort:
      return sizeof(AbortRequest);
    case PutBytesInstall:
      return sizeof(InstallRequest);
    default:
      WTF;
  }
}

static bool prv_has_invalid_request_length(const PutBytesCommand command, uint32_t actual_length) {
  const size_t expected_length = prv_expected_minimum_length_by_command(command);
  const bool has_invalid_length = (actual_length < expected_length);
  if (has_invalid_length) {
    PBL_LOG_ERR("Invalid message length for command %"PRIu32"; expected=%"PRIu32", actual=%"PRIu32,
            (uint32_t)command, (uint32_t)expected_length, (uint32_t)actual_length);
  }
  return has_invalid_length;
}

static bool prv_is_object_allowed(PutBytesObjectType type) {
#ifdef CONFIG_RECOVERY_FW
  switch (type) {
    case ObjectFirmware:
    case ObjectSysResources:
      return true;
    default:
      PBL_LOG_WRN("Can't update Object Type %u from PRF!", type);
      return false;
  }
#else
  return true;
#endif
}

static bool prv_is_init_object_type_invalid(PutBytesObjectType type) {
  if (type == 0 || type >= NumObjects) {
    PBL_LOG_ERR("Invalid object type, got 0x%x", type);
    return true;
  }
  if (!prv_is_object_allowed(type)) {
    return true;
  }
  if (!prv_has_valid_fw_update_state_for_object_type(type)) {
    PBL_LOG_ERR("Not in FW update state");
    return true;
  }
  return false;
}

static bool prv_parse_init_index(const InitRequest *init_request, uint32_t *index_out) {
  if (init_request->has_cookie) {
    // Currently cookie is only used with the app fetch endpoint. This app fetch will send a cookie
    // along with the message, and when put_bytes is done later my the mobile apps, it will send
    // the same cookie back in the Init message.

    *index_out = ntohl(init_request->cookie);
  } else {
    // legacy putbytes requests, bank numbers
    if (init_request->index >= MAX_APP_BANKS) {
      PBL_LOG_ERR("Attempting to put byte in invalid bank #%d", init_request->index);
      return false;
    }

    *index_out = init_request->index + 1;
  }
  return true;
}

static void prv_create_timer_if_needed(void) {
  if (s_pb_state.timer_id != TIMER_INVALID_ID) {
    new_timer_stop(s_pb_state.timer_id);
  } else {
    s_pb_state.timer_id = new_timer_create();
  }
}

static bool prv_setup_storage_for_init_request(const InitRequest *request, uint32_t index) {
  PutBytesStorageInfo *storage_info = NULL;

  switch (request->type) {
#ifndef CONFIG_RECOVERY_FW
    case ObjectFile: {
      storage_info = kernel_malloc_check(sizeof(PutBytesStorageInfo) +
                                         strlen(request->filename) + 1);
      strcpy(storage_info->filename, request->filename);
      break;
    }
    case ObjectWatchApp:
    case ObjectWatchWorker: {
      PebbleTask task = (request->type == ObjectWatchApp) ? PebbleTask_App : PebbleTask_Worker;
      char filename[APP_FILENAME_MAX_LENGTH + 1];
      app_storage_get_file_name(filename, sizeof(filename), index, task);
      storage_info = kernel_malloc_check(sizeof(PutBytesStorageInfo) + strlen(filename) + 1);
      strcpy(storage_info->filename, filename);
      break;
    }
    case ObjectAppResources: {
      char filename[APP_RESOURCE_FILENAME_MAX_LENGTH + 1];
      // The +1 is to make up for the fact that app banks start at 0, res banks start at 1. Fixing
      // in D641
      resource_storage_get_file_name(filename, sizeof(filename), index);
      storage_info = kernel_malloc_check(sizeof(PutBytesStorageInfo) + strlen(filename) + 1);
      strcpy(storage_info->filename, filename);
      break;
    }
#else
    case ObjectFile:
    case ObjectWatchApp:
    case ObjectWatchWorker:
    case ObjectAppResources:
      return false;
#endif
    case ObjectFirmware:
    case ObjectSysResources:
      // Clear out, in case a prior, non-installed FW or sys resources transfer was still dangling:
      s_ready_to_install[request->type - 1].type = 0;
      /* FALLTHRU */
    default: {
      storage_info = kernel_malloc_check(sizeof(PutBytesStorageInfo));
      storage_info->index = request->index;
    }
  }

  const bool success = pb_storage_init(&s_pb_state.storage, s_pb_state.type, s_pb_state.total_size,
                                       storage_info, s_pb_state.append_offset);
  kernel_free(storage_info);
  return success;
}

static void prv_do_init(void) {
  bool success = false;

  InitRequest* request = (InitRequest*) s_pb_state.receiver.buffer;

  if (prv_is_init_object_type_invalid(request->type)) {
    goto exit;
  }

  uint32_t index;
  if (!prv_parse_init_index(request, &index)) {
    goto exit;
  }

  uint32_t append_offset = 0;
  if (s_pb_state.receiver.length > offsetof(InitRequest, extra_info)) {
    // We compute the offset this way because filename installs can be variable length. In the
    // future, if we used this feature for files, this would allow for the same struct construction
    // on the mobile side
    uint32_t extra_info_offset = s_pb_state.receiver.length - sizeof(InitRequestExtraInfo);
    InitRequestExtraInfo *info =
        (InitRequestExtraInfo *)&s_pb_state.receiver.buffer[extra_info_offset];
    const uint32_t append_offset_magic = 0xBE4354EF;
    if (ntohl(info->init_req_magic) == append_offset_magic) {
      append_offset = ntohl(info->append_offset);
      PBL_LOG_INFO("Restarting FW Update at offset %"PRIu32, append_offset);
    }
  }

  // Setup our state
  const uint32_t size = ntohl(request->total_size);
  s_pb_state.total_size = size;
  s_pb_state.append_offset = append_offset;
  s_pb_state.remaining_bytes = size;
  s_pb_state.type = request->type;
  s_pb_state.has_cookie = request->has_cookie;
  s_pb_state.index = index;
  s_pb_state.current_command = PutBytesInit;
  s_pb_state.is_success = false;

  // Generate a token
  const uint32_t r = rand();
  s_pb_state.token = MAX(1, r);

  PBL_LOG_DBG("PutBytes Init CB. Type: %d, Idx: %"PRIu32", Size: %"PRIu32" Tok: %"PRIu32,
          (int) s_pb_state.type, s_pb_state.index,
          s_pb_state.total_size, s_pb_state.token);

  success = prv_setup_storage_for_init_request(request, index);

  s_pb_state.start_ticks = rtc_get_ticks();

  if (!success) {
    PBL_LOG_WRN("Failed to init storage");
    goto exit;
  }

  PebbleEvent event = {
    .type = PEBBLE_PUT_BYTES_EVENT,
    .put_bytes = {
      .type = PebblePutBytesEventTypeStart,
      .object_type = s_pb_state.type,
      .has_cookie = s_pb_state.has_cookie,
      .progress_percent = 0,
      .total_size = s_pb_state.total_size,
      .failed = false
    },
  };
  event_put(&event);

  prv_create_timer_if_needed();
  success = new_timer_start(s_pb_state.timer_id, PUT_TIMEOUT_MS, prv_timer_callback, &s_pb_state,
                            0 /*flags*/);

exit:
  prv_mark_pb_jobs_complete(1);
  prv_send_response(success ? ResponseAck : ResponseNack,
                    success ? s_pb_state.token : 0);

  if (!success) {
    prv_cleanup();
  }
}

static bool prv_check_putrequest_for_errors(const PutRequest *request_hdr,
                                            uint32_t tot_request_size);

static bool prv_do_put(const PutRequest *request, uint32_t request_size, uint32_t token) {
  uint32_t data_length = ntohl(request->length);

  pbl_sem_take(&s_pb_semaphore, PBL_FOREVER);
  uint32_t remaining_bytes = s_pb_state.remaining_bytes;
  pbl_sem_give(&s_pb_semaphore);

  if (prv_check_putrequest_for_errors(request, request_size) ||
      (data_length > remaining_bytes)) {
    prv_fail(token);
    return false;
  }

  PBL_LOG_DBG("PutBytes put CB. type: %"PRIu32", length: %"PRIu32,
          (uint32_t)s_pb_state.type, data_length);

  pb_storage_append(&s_pb_state.storage, request->data, data_length);

  pbl_sem_take(&s_pb_semaphore, PBL_FOREVER);
  s_pb_state.remaining_bytes -= data_length;
  pbl_sem_give(&s_pb_semaphore);

  return true;
}

static void prv_do_commit(void) {
  uint32_t elapsed_time_ms = pbl_ticks_to_ms(rtc_get_ticks() - s_pb_state.start_ticks);

  const CommitRequest *request = (const CommitRequest *)s_pb_state.receiver.buffer;

  uint32_t crc = ntohl(request->crc);
  uint32_t calculated_crc = pb_storage_calculate_crc(&s_pb_state.storage, PutBytesCrcType_Legacy);
  bool commit_succeeded = (calculated_crc == crc);

  if (elapsed_time_ms > 0) {
    int bytes_per_sec = (int)((s_pb_state.total_size * 1000) / elapsed_time_ms);
    PBL_LOG_DBG("PutBytes pushed %d bytes/sec", bytes_per_sec);
  }

  bluetooth_analytics_handle_put_bytes_stats(
      commit_succeeded, s_pb_state.type,
      s_pb_state.total_size, elapsed_time_ms, &s_pb_state.conn_event_stats);

  if (commit_succeeded) {
    s_pb_state.is_success = true;
    PBL_LOG_DBG("PutBytes commit CB. CRC matches! Calculated CRC is 0x%"PRIx32
            " expected 0x%"PRIx32, calculated_crc, crc);
    prv_commit_object(crc);
  } else {
    PBL_LOG_ERR("PutBytes commit CB. Calculated CRC is 0x%"PRIx32" expected 0x%"PRIx32,
            calculated_crc, crc);
  }

  s_pb_state.is_success &= commit_succeeded;
  prv_mark_pb_jobs_complete(1);
  prv_cleanup_and_send_response((commit_succeeded) ? ResponseAck : ResponseNack);
}

static bool prv_is_valid_command_for_current_state(PutBytesCommand command) {
  switch (s_pb_state.current_command) {
  case PutBytesIdle:
    return (command == PutBytesInit || command == PutBytesInstall);
  case PutBytesInit:
    return (command == PutBytesPut || command == PutBytesAbort);
  case PutBytesPut:
    return (command == PutBytesPut || command == PutBytesCommit || command == PutBytesAbort);
  case PutBytesCommit:
  case PutBytesAbort:
  case PutBytesInstall:
  default:
    return false;
  }
}

static bool prv_is_invalid_command_for_current_state(PutBytesCommand command) {
  if (!prv_is_valid_command_for_current_state(command)) {
    PBL_LOG_ERR("PutBytes command 0x%x not permitted in current state 0x%x",
            command, s_pb_state.current_command);
    return true;
  }
  return false;
}

static uint32_t prv_parse_token(const PutBytesCommand command, const SharedHeader *header) {
  if (command == PutBytesInit) {
    return 0;
  }
  return ntohl(header->token);
}

static bool prv_check_for_state_error(PutBytesCommand cmd, uint32_t token, uint32_t req_length) {
  const bool has_error = (prv_is_invalid_command_for_current_state(cmd) ||
                          prv_has_invalid_request_length(cmd, req_length) ||
                          prv_has_invalid_token(cmd, token) ||
                          prv_has_invalid_fw_update_state(cmd));

  return has_error;
}

static bool prv_check_putrequest_for_errors(const PutRequest *request_hdr,
                                            uint32_t tot_request_size) {
  uint32_t req_size = tot_request_size - sizeof(PutRequest);
  uint32_t data_length = ntohl(request_hdr->length);
  if (data_length > req_size) {
    PBL_LOG_ERR("Length value longer than buffer");
    return true;
  }

  uint32_t request_token = prv_parse_token(PutBytesPut, &request_hdr->header);
  if (prv_check_for_state_error(PutBytesPut, request_token, tot_request_size)) {
    return true;
  }

  return false;
}

static void prv_process_put_requests_system_task_cb(void *unused) {
  PutBytesPendingJobs *put_jobs = &s_pb_state.pb_pending_jobs;
  prv_lock_pb_job_state();
  uint8_t num_put_jobs = put_jobs->num_ops_pending;
  uint8_t read_idx = put_jobs->read_idx;
  uint32_t initial_remaining_bytes = s_pb_state.remaining_bytes;
  prv_unlock_pb_job_state();

  if (num_put_jobs == 0) {
    if (s_pb_state.current_command == PutBytesIdle) {
      // We terminated the PB transfer before we were able to process the PutRequest
      prv_send_response(ResponseNack, s_pb_state.token);
    }
    return;
  }

  uint32_t token = 0;
  for (int job_idx = 0; job_idx < num_put_jobs; job_idx++) {
    PutBytesJob *job = &put_jobs->job[(read_idx + job_idx) % put_jobs->num_allocated_pb_jobs];

    // Process requests until we run into a different command
    const PutBytesCommand cmd = job->buffer[0];
    if (cmd != PutBytesPut) {
      num_put_jobs = job_idx;
      break;
    }

    token = prv_parse_token(PutBytesPut, (SharedHeader *)job->buffer);

    if (!prv_do_put((PutRequest *)job->buffer, job->request_length, token)) {
      // consume the jobs, they are all going to fail
      prv_mark_pb_jobs_complete(num_put_jobs);
      return;
    }
  }

  pbl_sem_take(&s_pb_semaphore, PBL_FOREVER);
  {
    s_pb_state.current_command = PutBytesPut;
    uint32_t bytes_transferred = initial_remaining_bytes - s_pb_state.remaining_bytes;

    PebbleEvent event = {
      .type = PEBBLE_PUT_BYTES_EVENT,
      .put_bytes = {
        .type = PebblePutBytesEventTypeProgress,
        .object_type = s_pb_state.type,
        .has_cookie = s_pb_state.has_cookie,
        .progress_percent = prv_get_progress_percent(),
        .bytes_transferred = bytes_transferred,
        .failed = false,
      },
    };
    event_put(&event);
  }
  pbl_sem_give(&s_pb_semaphore);

  // (re)start timer for next event
  PBL_ASSERTN(new_timer_start(s_pb_state.timer_id, PUT_TIMEOUT_MS, prv_timer_callback,
                              &s_pb_state, 0 /*flags*/));

  prv_mark_pb_jobs_complete(num_put_jobs);

  // At this point we have updated the outstanding jobs. Check to see if a job started to arrive in
  // the meantime which needs to be ack'ed now that space is free
  bool do_ack;
  prv_lock_pb_job_state();
  {
    do_ack = put_jobs->need_to_ack_later;
    put_jobs->need_to_ack_later = false;
  }
  prv_unlock_pb_job_state();

  if (do_ack) { // If we did not pre-ack, we need to ack the packet now!
    prv_send_response(ResponseAck, token);
  }
}

static void prv_process_msg_system_task_callback(void *unused) {
  pbl_sem_take(&s_pb_semaphore, PBL_FOREVER);

  if (!s_pb_state.receiver.buffer ||
      s_pb_state.receiver.length == 0) {
    PBL_LOG_WRN("No message pending, PutBytes cancelled in the mean time?");
    prv_send_response(ResponseNack, s_pb_state.token);
    goto finally;
  }

  const PutBytesCommand cmd = s_pb_state.receiver.buffer[0];

  // Validation:
  const uint32_t request_token =
      prv_parse_token(cmd, (const SharedHeader *)s_pb_state.receiver.buffer);
  const bool has_error = prv_check_for_state_error(cmd, request_token, s_pb_state.receiver.length);

  if (has_error) {
    prv_fail(request_token);
    goto finally;
  }

  s_pb_state.current_command = cmd;

  switch (cmd) {
    case PutBytesInit:
      prv_do_init();
      break;
    case PutBytesPut:
      WTF; // Put Requests have their own handler
      break;
    case PutBytesCommit:
      prv_do_commit();
      break;
    case PutBytesAbort:
      prv_do_abort();
      break;
    case PutBytesInstall:
      prv_do_install(request_token);
      break;
    default:
      // This case is unreachable due to the prv_is_invalid_command_for_current_state() test.
      break;
  }

finally:
  prv_receiver_reset();
  pbl_sem_give(&s_pb_semaphore);
}

void put_bytes_init(void) {
}

void put_bytes_cancel(void) {
  PBL_ASSERT_TASK(PebbleTask_KernelBackground);

  if ((pbl_sem_take(&s_pb_semaphore, PBL_FOREVER) != 0)) {
    PBL_LOG_ERR("Failed to acquire the put-bytes semaphore");
    return;
  }

  if (s_pb_state.current_command == PutBytesIdle) {
    PBL_LOG_DBG("Attempted to cancel put_bytes while idle, %d",
            s_pb_state.current_command);
  } else if (s_pb_state.type == ObjectWatchApp ||
             s_pb_state.type == ObjectAppResources ||
             s_pb_state.type == ObjectWatchWorker) {
    PBL_LOG_DBG("Forcefully cancelling put_bytes transfer of app binaries");
    prv_cleanup();
  } else {
    PBL_LOG_DBG("Attempted to cancel put_bytes with a non desired type, %d",
            s_pb_state.type);
  }

  pbl_sem_give(&s_pb_semaphore);
}

// Only used by unit test
void put_bytes_deinit(void) {
  pbl_sem_deinit(&s_pb_semaphore);

  if (s_pb_state.timer_id != TIMER_INVALID_ID) {
    new_timer_delete(s_pb_state.timer_id);
  }
  pb_storage_deinit(&s_pb_state.storage, false);
  prv_deinit_put_job_queue();

  s_pb_state = (PutBytesState){};
  memset(&s_ready_to_install, 0, sizeof(s_ready_to_install));
}

static void prv_expect_init_timeout_cb(void *data) {
  pbl_sem_take(&s_pb_semaphore, PBL_FOREVER);

  if (s_pb_state.timer_id) {
    new_timer_delete(s_pb_state.timer_id);
    s_pb_state.timer_id = TIMER_INVALID_ID;
  }

  PebbleEvent event = {
    .type = PEBBLE_PUT_BYTES_EVENT,
    .put_bytes = {
      .type = PebblePutBytesEventTypeInitTimeout,
      .object_type = ObjectUnknown,
      .has_cookie = false,
      .progress_percent = 0,
      .total_size = 0,
      .failed = true,
    },
  };
  event_put(&event);

  pbl_sem_give(&s_pb_semaphore);
}

void put_bytes_expect_init(uint32_t timeout_ms) {
  pbl_sem_take(&s_pb_semaphore, PBL_FOREVER);

  if (s_pb_state.current_command != PutBytesIdle) {
    PBL_LOG_ERR("Called put_bytes_expect while put_bytes is not idle");
    pbl_sem_give(&s_pb_semaphore);
    return;
  }

  // Just in case this is called more than once
  prv_create_timer_if_needed();
  bool success = new_timer_start(s_pb_state.timer_id, timeout_ms, prv_expect_init_timeout_cb, NULL,
                                 0 /*flags*/);
  PBL_ASSERTN(success);
  pbl_sem_give(&s_pb_semaphore);
}

void put_bytes_handle_comm_session_event(const PebbleCommSessionEvent *
                                         comm_session_event) {
  if (comm_session_event->is_system) {
    prv_cleanup_async();
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// ReceiverImplementation

static bool prv_receiver_contains_put_request(void) {
  return (s_pb_state.receiver.buffer[0] == PutBytesPut);
}

static bool prv_is_message_pending_processing(void) {
  return (s_pb_state.receiver.length != 0);
}

static void prv_receiver_reset(void) {
  s_pb_state.receiver.length = 0;
  s_pb_state.receiver.buffer = NULL;
}

static bool prv_take_lock_with_short_timeout(void) {
  // This code executes on BT02, so don't stall for too long. If the lock is taken, there is
  // probably a Put Bytes session going on already anyway.
  const pbl_tick_t SEMAPHORE_TIMEOUT_TICKS = pbl_ms_to_ticks(25);
  if ((pbl_sem_take(&s_pb_semaphore, PBL_TICKS(SEMAPHORE_TIMEOUT_TICKS)) != 0)) {
    PBL_LOG_ERR("Failed to acquire the put-bytes semaphore, retry");
    return false;
  }
  return true;
}

static bool prv_prepare(size_t total_payload_length) {
  if (total_payload_length > PUT_BYTES_PP_BUFFER_SIZE) {
    PBL_LOG_ERR("Put Bytes message too big");
    return false;
  }

  if (!s_pb_state.receiver.buffer) {
    if (!prv_init_put_job_queue_if_necessary()) {
      return false; // OOM
    }
    s_pb_state.receiver.buffer = prv_get_next_pb_job_buffer();
    s_pb_state.receiver.length = 0;
  }

  if (prv_is_message_pending_processing()) {
    return false;
  }

  s_pb_state.receiver.length = total_payload_length;
  s_pb_state.receiver.pos = 0;
  s_pb_state.receiver.should_nack = false;

  return (s_pb_state.receiver.buffer != NULL);
}

Receiver *prv_receiver_prepare(CommSession *session, const PebbleProtocolEndpoint *endpoint,
                               size_t total_payload_length) {
  // This receiver should only be used for the Put Bytes endpoint (which has a NULL handler,
  // because this receiver calls the internal, static functions in this file directly).
  // It should only be used with the System session, we use comm_session_get_system_session()
  // directly, instead of passing around the session as a variable.
  PBL_ASSERTN(endpoint->handler == NULL);

  bool success = false;
  if (prv_take_lock_with_short_timeout()) {
    success = prv_prepare(total_payload_length);
    pbl_sem_give(&s_pb_semaphore);
  }

  if (!success) {
    prv_add_nack_no_token_system_callback();
    return NULL;
  }

  // This is just symbolic, It just has to be non-NULL, really.
  // Since there is just once instance, we statically refer to it everywhere.
  return (Receiver *)&s_pb_state.receiver;
}

static void prv_validate_and_preack_request_if_needed(const uint8_t *rcvd_data, uint8_t length) {
  if (prv_receiver_contains_put_request()) { // beginning of a PutRequest msg?
    PutRequest *request = (PutRequest *)rcvd_data;
    uint32_t tot_payload_size = s_pb_state.receiver.length;

    // We only need the PutRequest Header to do some sanity checking so perform the check
    // now so we don't pre-ACK malformed packets
    if ((tot_payload_size < sizeof(PutRequest)) || ((length >= sizeof(PutRequest)) &&
        prv_check_putrequest_for_errors(request, tot_payload_size))) {
      s_pb_state.receiver.should_nack = true;
    }

    if (!s_pb_state.receiver.should_nack) {
      prv_pre_ack_if_space_in_put_job_queue();
    }
  }
}

void prv_receiver_write(Receiver *receiver, const uint8_t *data, size_t length) {
  if (!prv_take_lock_with_short_timeout()) {
    s_pb_state.receiver.should_nack = true;
    return;
  }
  if (!prv_is_message_pending_processing()) {
    // Could happen if put_bytes_cancel() was called after "prepare"
    PBL_LOG_WRN("No message pending, PutBytes cancelled? Will NACK.");
    s_pb_state.receiver.should_nack = true;
    goto finally;
  }
        PBL_ASSERTN(s_pb_state.receiver.buffer &&
              s_pb_state.receiver.pos + length <= PUT_BYTES_PP_BUFFER_SIZE);

  memcpy(s_pb_state.receiver.buffer + s_pb_state.receiver.pos, data, length);

  if (s_pb_state.receiver.pos == 0) {
    prv_validate_and_preack_request_if_needed(data, length);
  }

  s_pb_state.receiver.pos += length;

finally:
  pbl_sem_give(&s_pb_semaphore);
}

void prv_receiver_cleanup(Receiver *receiver) {
  // Got disconnected while in the middle of receiving a message, clean up:
  prv_cleanup_async();

  // No point in trying to Nack, because we got disconnected...
}

void prv_receiver_finish(Receiver *receiver) {
  if (s_pb_state.receiver.should_nack) {
    PBL_LOG_WRN("NACK'ing from ..._finish");
    prv_add_nack_no_token_system_callback();
    prv_receiver_reset();
    return;
  }

  // We are still processing PB data, keep the BT connection fast
  prv_set_responsiveness(ResponseTimeMin, MIN_LATENCY_MODE_TIMEOUT_PUT_BYTES_SECS);

  prv_finalize_pb_job();
  if (prv_receiver_contains_put_request()) {
    // The PutRequest handler has no reliance on the receiver struct so mark processing as done
    prv_receiver_reset();
    system_task_add_callback(prv_process_put_requests_system_task_cb, NULL);
  } else {
    system_task_add_callback(prv_process_msg_system_task_callback, NULL);
  }

  // Don't clean up, the receiver.buffer will be re-used for the entire Put Bytes session.
  // The Put Bytes code will clean it up itself.
}

const ReceiverImplementation g_put_bytes_receiver_impl = {
  .prepare = prv_receiver_prepare,
  .write = prv_receiver_write,
  .finish = prv_receiver_finish,
  .cleanup = prv_receiver_cleanup,
};

////////////////////////////////////////////////////////////////////////////////////////////////////
// For Unit Testing

struct pbl_sem * put_bytes_get_semaphore(void) {
  return &s_pb_semaphore;
}

TimerID put_bytes_get_timer_id(void) {
  return s_pb_state.timer_id;
}

uint32_t put_bytes_get_index(void) {
  return s_pb_state.index;
}

#ifdef UNITTEST
T_STATIC uint8_t prv_put_bytes_get_max_batched_pb_ops(void) {
  return MAX_BATCHED_PB_PUT_OPS;
}
#endif
