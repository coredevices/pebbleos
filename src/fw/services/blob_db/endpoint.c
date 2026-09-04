/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "pbl/services/blob_db/endpoint_private.h"
#include "pbl/services/blob_db/settings_blob_db.h"

#include "kernel/pebble_tasks.h"
#include "pbl/services/bluetooth/bluetooth_persistent_storage.h"
#include "pbl/services/comm_session/session.h"
#include "pbl/services/analytics/analytics.h"
#include <pbl/logging/logging.h>
#include "system/passert.h"
#include "system/status_codes.h"
#include "system/hexdump.h"
#include "pbl/util/attributes.h"

#include <stdbool.h>

PBL_LOG_MODULE_DECLARE(service_blob_db, CONFIG_SERVICE_BLOB_DB_LOG_LEVEL);

//! @file endpoint.c
//! BlobDB Endpoint
//!
//! There are 3 commands implemented in this endpoint: INSERT, DELETE, and CLEAR
//!
//! <b>INSERT:</b> This command will insert a key and value into the database specified.
//!
//! \code{.c}
//! 0x01 <uint16_t token> <uint8_t DatabaseId>
//! <uint8_t key_size M> <uint8_t[M]> key_bytes>
//! <uint16_t value_size N> <uint8_t[N]> value_bytes>
//! \endcode
//!
//! <b>DELETE:</b> This command will delete an entry with the key in the database specified.
//!
//! \code{.c}
//! 0x04 <uint16_t token> <uint8_t DatabaseId>
//! <uint8_t key_size M> <uint8_t[M]> key_bytes>
//! \endcode
//!
//! <b>CLEAR:</b> This command will clear all entries in the database specified.
//!
//! \code{.c}
//! 0x05 <uint16_t token> <uint8_t DatabaseId>
//! \endcode
//!
//! <b>INSERT_WITH_TIMESTAMP:</b> Insert with timestamp for conflict resolution.
//! If watch data is newer, returns BLOB_DB_DATA_STALE and phone should resync.
//!
//! \code{.c}
//! 0x0D <uint16_t token> <uint8_t DatabaseId> <uint32_t timestamp>
//! <uint8_t key_size M> <uint8_t[M] key_bytes>
//! <uint16_t value_size N> <uint8_t[N] value_bytes>
//! \endcode

//! BlobDB Endpoint ID
static const uint16_t BLOB_DB_ENDPOINT_ID = 0xb1db;

static const uint8_t KEY_DATA_LENGTH __attribute__((unused)) = (sizeof(uint8_t) + sizeof(uint8_t));
static const uint8_t VALUE_DATA_LENGTH = (sizeof(uint16_t) + sizeof(uint8_t));

//! Message Length Constants
static const uint8_t MIN_INSERT_LENGTH = 8;
static const uint8_t MIN_INSERT_WITH_TIMESTAMP_LENGTH = 12; // + 4 bytes for timestamp
static const uint8_t MIN_DELETE_LENGTH = 5;
static const uint8_t MIN_CLEAR_LENGTH  = 3;

static bool s_bdb_accepting_messages;

static void prv_send_response(CommSession *session, BlobDBToken token, BlobDBResponse result) {
  struct PACKED BlobDBResponseMsg {
    BlobDBToken token;
    BlobDBResponse result;
  } response = {
    .token = token,
    .result = result,
  };

  comm_session_send_data(session, BLOB_DB_ENDPOINT_ID, (uint8_t*)&response, sizeof(response),
                         COMM_SESSION_DEFAULT_TIMEOUT);
}

static BlobDBResponse prv_interpret_db_ret_val(status_t ret_val) {
  switch (ret_val) {
    case S_SUCCESS:
      return BLOB_DB_SUCCESS;
    case E_DOES_NOT_EXIST:
      return BLOB_DB_KEY_DOES_NOT_EXIST;
    case E_RANGE:
      return BLOB_DB_INVALID_DATABASE_ID;
    case E_INVALID_ARGUMENT:
      return BLOB_DB_INVALID_DATA;
    case E_OUT_OF_STORAGE:
      return BLOB_DB_DATABASE_FULL;
    case E_INVALID_OPERATION:
      return BLOB_DB_DATA_STALE;
    default:
      PBL_LOG_WRN("BlobDB return value caught by default case");
      return BLOB_DB_GENERAL_FAILURE;
  }
}

static const uint8_t *prv_read_ptr(const uint8_t *iter, const uint8_t *iter_end,
                                   const uint8_t **out_buf, uint16_t buf_len) {

  // >= because we will be reading more bytes after this point
  if ((buf_len == 0) || ((iter + buf_len) > iter_end)) {
    PBL_LOG_WRN("BlobDB: read invalid length");
    return NULL;
  }

  // grab pointer to start of buf
  *out_buf = (uint8_t*)iter;
  iter += buf_len;

  return iter;
}

static const uint8_t *prv_read_key_size(const uint8_t *iter, const uint8_t *iter_end,
                                        uint8_t *out_int) {

  // copy length from iter to out_len, then save a local copy of the length
  *out_int = *iter++;
  return iter;
}

static const uint8_t *prv_read_value_size(const uint8_t *iter, const uint8_t *iter_end,
                                          uint16_t *out_int) {

  // copy length from iter to out_len, then save a local copy of the length
  *out_int = *(uint16_t*)iter;
  iter += sizeof(uint16_t);
  return iter;
}

static BlobDBToken prv_try_read_token(const uint8_t *data, uint32_t length) {
  if (length < sizeof(BlobDBToken)) {
    return 0;
  }

  return *(BlobDBToken*)data;
}

static void prv_handle_database_insert(CommSession *session, const uint8_t *data, uint32_t length) {
  if (length < MIN_INSERT_LENGTH) {
    prv_send_response(session, prv_try_read_token(data, length), BLOB_DB_INVALID_DATA);
    return;
  }

  const uint8_t *iter = data;
  BlobDBToken token;
  BlobDBId db_id;

  // Read token and db_id
  iter = endpoint_private_read_token_db_id(iter, &token, &db_id);

  // read key length and key bytes ptr
  uint8_t key_size;
  const uint8_t *key_bytes = NULL;
  iter = prv_read_key_size(iter, data + length, &key_size);
  iter = prv_read_ptr(iter, data + length, &key_bytes, key_size);

  // If read past end or there is not enough data left in buffer for a value size and data to exist
  if (!iter || (iter > (data + length - VALUE_DATA_LENGTH))) {
    prv_send_response(session, token, BLOB_DB_INVALID_DATA);
    return;
  }

  // read value length and value bytes ptr
  uint16_t value_size;
  const uint8_t *value_bytes = NULL;
  iter = prv_read_value_size(iter, data + length, &value_size);
  iter = prv_read_ptr(iter, data + length, &value_bytes, value_size);

  // If we read too many bytes or didn't read all the bytes (2nd test)
  if (!iter || (iter != (data + length))) {
    prv_send_response(session, token, BLOB_DB_INVALID_DATA);
    return;
  }

  // perform action on database and return result
  status_t ret = blob_db_insert(db_id, key_bytes, key_size, value_bytes, value_size);
  prv_send_response(session, token, prv_interpret_db_ret_val(ret));
}


static void prv_handle_database_insert_with_timestamp(CommSession *session, const uint8_t *data,
                                                      uint32_t length) {
  if (length < MIN_INSERT_WITH_TIMESTAMP_LENGTH) {
    prv_send_response(session, prv_try_read_token(data, length), BLOB_DB_INVALID_DATA);
    return;
  }

  const uint8_t *iter = data;
  BlobDBToken token;
  BlobDBId db_id;

  // Read token and db_id
  iter = endpoint_private_read_token_db_id(iter, &token, &db_id);

  // Read timestamp (4 bytes, little-endian)
  uint32_t timestamp = *(uint32_t *)iter;
  iter += sizeof(uint32_t);

  // read key length and key bytes ptr
  uint8_t key_size;
  const uint8_t *key_bytes = NULL;
  iter = prv_read_key_size(iter, data + length, &key_size);
  iter = prv_read_ptr(iter, data + length, &key_bytes, key_size);

  // If read past end or there is not enough data left in buffer for a value size and data to exist
  if (!iter || (iter > (data + length - VALUE_DATA_LENGTH))) {
    prv_send_response(session, token, BLOB_DB_INVALID_DATA);
    return;
  }

  // read value length and value bytes ptr
  uint16_t value_size;
  const uint8_t *value_bytes = NULL;
  iter = prv_read_value_size(iter, data + length, &value_size);
  iter = prv_read_ptr(iter, data + length, &value_bytes, value_size);

  // If we read too many bytes or didn't read all the bytes (2nd test)
  if (!iter || (iter != (data + length))) {
    prv_send_response(session, token, BLOB_DB_INVALID_DATA);
    return;
  }

  // Only Settings BlobDB supports timestamped insert
  if (db_id != BlobDBIdSettings) {
    // Fall back to regular insert for other databases
    status_t ret = blob_db_insert(db_id, key_bytes, key_size, value_bytes, value_size);
    prv_send_response(session, token, prv_interpret_db_ret_val(ret));
    return;
  }

  // Use timestamped insert for Settings - will reject if watch data is newer
  status_t ret = settings_blob_db_insert_with_timestamp(key_bytes, key_size,
                                                        value_bytes, value_size,
                                                        (time_t)timestamp);
  prv_send_response(session, token, prv_interpret_db_ret_val(ret));
}


static void prv_handle_database_delete(CommSession *session, const uint8_t *data, uint32_t length) {
  if (length < MIN_DELETE_LENGTH) {
    prv_send_response(session, prv_try_read_token(data, length), BLOB_DB_INVALID_DATA);
    return;
  }

  const uint8_t *iter = data;
  BlobDBToken token;
  BlobDBId db_id;

  // Read token and db_id
  iter = endpoint_private_read_token_db_id(iter, &token, &db_id);

  // Read key length and key bytes
  uint8_t key_size;
  const uint8_t *key_bytes = NULL;
  iter = prv_read_key_size(iter, data + length, &key_size);
  iter = prv_read_ptr(iter, data + length, &key_bytes, key_size);

  // If we read too many bytes or key_size is 0 or didn't read all the bytes
  if (!iter || (iter != (data + length))) {
    prv_send_response(session, token, BLOB_DB_INVALID_DATA);
    return;
  }

  // perform action on database and return result
  status_t ret = blob_db_delete(db_id, key_bytes, key_size);
  prv_send_response(session, token, prv_interpret_db_ret_val(ret));
}


static void prv_handle_database_clear(CommSession *session, const uint8_t *data, uint32_t length) {
  if (length < MIN_CLEAR_LENGTH) {
    prv_send_response(session, prv_try_read_token(data, length), BLOB_DB_INVALID_DATA);
    return;
  }

  BlobDBToken token;
  BlobDBId db_id;

  // Read token and db_id
  endpoint_private_read_token_db_id(data, &token, &db_id);

  // perform action on database and return result
  status_t ret = blob_db_flush(db_id);
  prv_send_response(session, token, prv_interpret_db_ret_val(ret));

  // Mark the device as faithful after successfully flushing
  if (ret == S_SUCCESS) {
    bt_persistent_storage_set_unfaithful(false /* We are now faithful */);
  }
}

static void prv_blob_db_msg_decode_and_handle(
    CommSession *session, BlobDBCommand cmd, const uint8_t *data, size_t data_length) {
  switch (cmd) {
    case BLOB_DB_COMMAND_INSERT:
      PBL_LOG_DBG("Got INSERT");
      prv_handle_database_insert(session, data, data_length);
      break;
    case BLOB_DB_COMMAND_DELETE:
      PBL_LOG_DBG("Got DELETE");
      prv_handle_database_delete(session, data, data_length);
      break;
    case BLOB_DB_COMMAND_CLEAR:
      PBL_LOG_DBG("Got CLEAR");
      prv_handle_database_clear(session, data, data_length);
      break;
    case BLOB_DB_COMMAND_INSERT_WITH_TIMESTAMP:
      PBL_LOG_DBG("Got INSERT_WITH_TIMESTAMP");
      prv_handle_database_insert_with_timestamp(session, data, data_length);
      break;
    // Commands not implemented.
    case BLOB_DB_COMMAND_READ:
    case BLOB_DB_COMMAND_UPDATE:
      PBL_LOG_ERR("BlobDB Command not implemented");
      // Fallthrough
    default:
      PBL_LOG_ERR("Invalid BlobDB message received, cmd is %u", cmd);
      prv_send_response(session, prv_try_read_token(data, data_length), BLOB_DB_INVALID_OPERATION);
      break;
  }
}

void blob_db_protocol_msg_callback(CommSession *session, const uint8_t* data, size_t length) {
  PBL_ASSERT_TASK(PebbleTask_KernelBackground);

  PBL_HEXDUMP_D(LOG_DOMAIN_BLOBDB, LOG_LEVEL_DEBUG, data, length);

  // Each BlobDB message is required to have at least a Command and a Token
  static const uint8_t MIN_RAW_DATA_LEN = sizeof(BlobDBCommand) + sizeof(BlobDBToken);
  if (length < MIN_RAW_DATA_LEN) {
    PBL_LOG_ERR("Got a blob_db message that was too short, len: %zu", length);
    prv_send_response(session, 0, BLOB_DB_INVALID_DATA);
    return;
  }

  const BlobDBCommand cmd = *data;
  data += sizeof(BlobDBCommand); // fwd to message contents
  const size_t data_length = length - sizeof(BlobDBCommand);

  if (!s_bdb_accepting_messages) {
    prv_send_response(session, prv_try_read_token(data, length), BLOB_DB_TRY_LATER);
    return;
  }

  prv_blob_db_msg_decode_and_handle(session, cmd, data, data_length);
}

void blob_db_set_accepting_messages(bool enabled) {
  s_bdb_accepting_messages = enabled;
}
