/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "applib/data_logging.h"
#include <pbl/drivers/rtc.h>
#include "flash_region/flash_region.h"
#include "kernel/pebble_tasks.h"
#include "pbl/kernel/mutex.h"
#include "pbl/services/comm_session/protocol.h"
#include "system/hexdump.h"
#include "pbl/util/attributes.h"
#include "util/shared_circular_buffer.h"
#include "util/units.h"
#include "pbl/util/uuid.h"

#include <stdint.h>
#include <stdlib.h>
#include <time.h>

#define DLS_HEXDUMP(data, length) \
          PBL_HEXDUMP_D(LOG_DOMAIN_DATA_LOGGING, LOG_LEVEL_DEBUG, data, length)


// File name is formatted as: ("%s%d", DLS_FILE_NAME_PREFIX, session_id)
#define DLS_FILE_NAME_PREFIX          "dls_storage_"
static const uint32_t DLS_FILE_NAME_MAX_LEN = 20;
static const uint32_t DLS_FILE_INIT_SIZE_BYTES = KiBYTES(4);

// Limits on how much free space we try to reserver for a session file
static const uint32_t DLS_MIN_FILE_FREE_BYTES = KiBYTES(8);
static const uint32_t DLS_MAX_FILE_FREE_BYTES = KiBYTES(100);

// Min amount of available space at the end of a file before we decide to grow it
static const uint32_t DLS_MIN_FREE_BYTES = KiBYTES(1);

// Max # of sessions we allow
static const uint32_t DLS_MAX_NUM_SESSIONS = 20;

// Maximum total amount of storage we are allowed to use on the file system.
static const uint32_t DLS_TOTAL_STORAGE_BYTES = KiBYTES(640);

// Maximum amount of space allowed for data over and above the minimum allotment per session
#define DLS_MAX_DATA_BYTES  (DLS_TOTAL_STORAGE_BYTES  \
                             - (DLS_MAX_NUM_SESSIONS * DLS_FILE_INIT_SIZE_BYTES))

typedef enum {
  //! A session is active when it's first created and it's still being logged to.
  DataLoggingStatusActive = 0x01,
  //! A session is inactive when we have data to spool to the phone but the app that created the
  //! session has since closed or the app has closed it by calling dls_finish.
  DataLoggingStatusInactive = 0x02,
} DataLoggingStatus;


// Endpoint commands
typedef enum {
  DataLoggingEndpointCmdOpen = 0x01,
  DataLoggingEndpointCmdData = 0x02,
  DataLoggingEndpointCmdClose = 0x03,
  DataLoggingEndpointCmdReport = 0x04,
  DataLoggingEndpointCmdAck = 0x05,
  DataLoggingEndpointCmdNack = 0x06,
  DataLoggingEndpointCmdTimeout = 0x07,
  DataLoggingEndpointCmdEmptySession = 0x08,
  DataLoggingEndpointCmdGetSendEnableReq = 0x09,
  DataLoggingEndpointCmdGetSendEnableRsp = 0x0A,
  DataLoggingEndpointCmdSetSendEnable = 0x0B,
} DataLoggingEndpointCmd;

//! Every command starts off with a 8-bit command byte. Commands from the phone will have their
//! top bit set, where commands from watch will have the top bit cleared. See
//! DataLoggingEndpointCmd for the values of the other 7 bits.
static const uint8_t DLS_ENDPOINT_CMD_MASK = 0x7f;


#define DLS_INVALID_FILE (-1)
typedef struct DataLoggingSessionStorage {
  //! Handle to the pfs file we are using. Set to DLS_INVALID_FILE if no storage yet
  int fd;

  //! Which byte offset in the file we are writing to
  uint32_t write_offset;

  //! Which byte offset in the file we are reading from
  uint32_t read_offset;

  //! Number of unread bytes in storage
  uint32_t num_bytes;
} DataLoggingSessionStorage;


// Our little comm state machine...
//
//     +----------+  Rx Ack    +----------+    Tx Data   +----------+
//     | Opening  |----------->| Idle     |+------------>| Sending  |
//     +----------+            +----------+              +----------+
//                                  ^                         |
//                                  |       Rx Ack            |
//                                  +-------------------------+

typedef enum {
  //! The session is opening and waiting for the phone to acknowledge our open command.
  DataLoggingSessionCommStateOpening,
  //! The session is idle, ready to send data
  DataLoggingSessionCommStateIdle,
  //! The session has sent data to the phone and is waiting for an ack
  DataLoggingSessionCommStateSending,
} DataLoggingSessionCommState;

typedef struct {
  //! A session ID that is chosen by the watch and is unique to all the session IDs that the
  //! watch knows about.
  uint8_t session_id;

  DataLoggingSessionCommState state:8;

  //! The number of times this session got nacked
  uint8_t nack_count;

  //! How many bytes we've sent to the phone that haven't been acked yet.
  int num_bytes_pending;

  //! The time in RtcTicks at which the current state will timeout while waiting for an ack. Set
  //! to zero if we're not waiting for one.
  RtcTicks ack_timeout;
} DataLoggingSessionComm;


//! Information needed while a session is active (watch app still adding more data).
typedef struct {
  struct pbl_mutex mutex;
  SharedCircularBuffer buffer;    //! A data buffer
  SharedCircularBufferClient buffer_client;
  uint8_t *buffer_storage;        //! Storage for the buffer
  //! true if buffer_storage is in kernel heap, else it's in dls_create() caller's heap
  bool buffer_in_kernel_heap:1;
  //! bool used to rate control how often we ask the system task to write us out to flash.
  bool write_request_pending:1;
  //! bool used to record the fact that a session should be inactivated once it is unlocked
  //! (by dls_unlock_session())
  bool inactivate_pending:1;
  //! Incremented/decremented under global list mutex. This structure can only be freed up when
  //! this reaches 0.
  uint8_t open_count;
} DataLoggingActiveState;


//! Data logging session metadata, struct in memory
typedef struct DataLoggingSession {
  // FIXME use a ListNode instead of this custom list
  struct DataLoggingSession *next; //!< The next logging_session in the linked list

  Uuid app_uuid;
  uint32_t tag;
  PebbleTask task;

  DataLoggingItemType item_type:4;
  DataLoggingStatus status:4;
  uint16_t item_size;

  // A timestamp of when this session was first created.
  time_t session_created_timestamp;

  DataLoggingSessionComm comm;

  DataLoggingSessionStorage storage;

  //! This pointer only allocated for active sessions
  DataLoggingActiveState *data;
} DataLoggingSession;


bool dls_private_send_session(DataLoggingSession *logging_session, bool empty);

//! Must be called on the system task
//! @param data unused
void dls_private_handle_disconnect(void *data);

//! Get/Set the current send_enable setting
bool dls_private_get_send_enable(void);
void dls_private_set_send_enable(bool setting);


typedef struct PACKED {
  uint8_t command;
  uint8_t session_id;
  uint32_t items_left_hereafter;
  uint32_t crc32;
  uint8_t bytes[];
} DataLoggingSendDataMessage;


//! Size of the buffer we create for buffered sessions. This is the largest item size allowed
//! for buffered sessions.
static const uint32_t DLS_SESSION_MAX_BUFFERED_ITEM_SIZE = 300;

//! Size of the buffer we create for buffered sessions. This must be 1 bigger than
//! DLS_SESSION_MAX_BUFFERED_ITEM_SIZE because we build a circular buffer out of it
#define DLS_SESSION_MIN_BUFFER_SIZE  (DLS_SESSION_MAX_BUFFERED_ITEM_SIZE + 1)

//! Max payload we can send when we send logging data to the phone. This is the largest item
//! size allowed for non-buffered sessions.
static const uint32_t DLS_ENDPOINT_MAX_PAYLOAD = (COMM_MAX_OUTBOUND_PAYLOAD_SIZE
                                                  - sizeof(DataLoggingSendDataMessage));


//! Unit tests only
int dls_test_read(DataLoggingSession *logging_session, uint8_t *buffer, int num_bytes);

//! Unit tests only
int dls_test_consume(DataLoggingSession *logging_session, int num_bytes);

//! Unit tests only
int dls_test_get_num_bytes(DataLoggingSession *logging_session);

//! Unit tests only
int dls_test_get_tag(DataLoggingSession *logging_session);

//! Unit tests only
uint8_t dls_test_get_session_id(DataLoggingSession *logging_session);
