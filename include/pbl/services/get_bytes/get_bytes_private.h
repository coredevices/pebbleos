/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "get_bytes.h"

#include <stdint.h>

#include "kernel/core_dump_private.h"
#include "pbl/services/comm_session/session.h"
#include "pbl/util/attributes.h"

// This matches the entry we put into protocol_endpoints_table.h
static const uint16_t GET_BYTES_ENDPOINT_ID = 9000;

// -----------------------------------------------------------------------------------------------
// Support structures for returning the core dump over the comm session protocol.

// A protocol request/response header
typedef struct PACKED {
  uint8_t  cmd_id;               // A value from GetBytesCmd
  uint8_t  transaction_id;
} GetBytesHeader;

// The GET_BYTES_CMD_GET_COREDUMP request consists of only a GetBytesHeader

// The GET_BYTES_CMD_GET_FILE request consists of a GetBytesFileHeader
typedef struct PACKED {
  GetBytesHeader hdr;
  uint8_t  filename_len;
  char     filename[];
} GetBytesFileHeader;

typedef struct PACKED {
  GetBytesHeader hdr;
  uint32_t start_addr;
  uint32_t len;
} GetBytesFlashHeader;

// Various values for GetBytesHeader.cmd_id
typedef enum {
  // Sent initially to start a coredump transfer
  // Will return the last coredump which was saved to flash
  GET_BYTES_CMD_GET_COREDUMP = 0,
  // Sent in response to GET_BYTES_CMD_GET_COREDUMP
  GET_BYTES_CMD_OBJECT_INFO = 1,
  // Sent after GET_BYTES_CMD_OBJECT_INFO if error_code was 0.
  GET_BYTES_CMD_OBJECT_DATA = 2,
  // Sent initially to start a file transfer
  GET_BYTES_CMD_GET_FILE = 3,
  // Sent initially to start a flash transfer
  GET_BYTES_CMD_GET_FLASH = 4,
  // Sent initially to start a coredump transfer
  // Will only return a coredump if it has not previously been read
  GET_BYTES_CMD_GET_NEW_COREDUMP = 5,
} GetBytesCmd;

// The GET_BYTES_CMD_OBJECT_INFO response has this format
typedef struct PACKED {
  GetBytesHeader hdr;
  uint8_t   error_code;  // 0 = no error and multiple GET_BYTES_CMD_OBJECT_DATA response will follow
  uint32_t  num_bytes;   // total size of core dump image (will be 0 if error_code != 0).
} GetBytesRspObjectInfo;

// The GET_BYTES_CMD_OBJECT_DATA response has this format
typedef struct PACKED {
  GetBytesHeader hdr;
  uint32_t  byte_offset;         // starting byte offset of this data chunk
  uint8_t data[];
} GetBytesRspObjectData;

// Using to send error response asynchronously
typedef struct GetBytesErrorResponse {
  CommSession *session;
  int8_t transaction_id;
  GetBytesInfoErrorCode result;
} GetBytesErrorResponse;
