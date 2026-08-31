/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

/*
 * This file defines the structures required for Binary Logging. Please see
 * https://docs.google.com/document/d/1AyRGwr8CiilAViha56EiuRSZFiW0fWMcsTfzkNxByZ8
 * for more information.
 */

// SLIP Framing (if not using PULSE). Packet is: END, <packet>, <crc32>, END
#define END 0xC0
#define ESC 0xDB
#define ESC_END 0xDC
#define ESC_ESC 0xDD

// Version
typedef struct BinLogMessage_Version {
  union {
    struct {
      uint8_t reserved:4;
      uint8_t unhashed_msg:1;
      uint8_t parameterized:1;
      uint8_t tick_count:1;
      uint8_t time_date:1;
    };
    uint8_t version;
  };
} BinLogMessage_Version;

#define BINLOGMSG_VERSION_UNHASHED_MSG  (1 << 3)
#define BINLOGMSG_VERSION_PARAMETERIZED (1 << 2)
#define BINLOGMSG_VERSION_TICK_COUNT    (1 << 1)
#define BINLOGMSG_VERSION_TIME_DATE     (1 << 0)

_Static_assert(sizeof(BinLogMessage_Version) == 1, "BinLogMessage_Version size != 1");

// Time_Full
// The times are in UTC. All values are 0 based (i.e., hour is [0,23], minute is [0,59],
// second is [0,59], millisecond is [0,999]).
typedef struct Time_Full {
  uint32_t reserved:5;
  uint32_t hour:5;
  uint32_t minute:6;
  uint32_t second:6;
  uint32_t millisecond:10;
} Time_Full;

// Time_Tick
// Total ticks = (count_high << 32) | count.
typedef struct Time_Tick {
  uint16_t reserved;
  uint16_t count_high;
  uint32_t count;
} Time_Tick;

// Date
// The year is an offset from 2000 (e.g. year = 16 is 2016).
// All remaining values are 1 based (i.e., month is [1,12], day is [1,31]).
// An invalid/unknown date is identified by (year, month, day) = (0, 0, 0). Thus, Date = 0 is an
// invalid date, not the start of the epoch, which would be (0, 1, 1).
typedef struct Date {
  uint16_t year:7;
  uint16_t month:4;
  uint16_t day:5;
} Date;

// MessageID
typedef struct MessageID {
  union {
    struct {
      uint32_t msg_number:19; // LSB
      uint32_t task_id:4;
      uint32_t str_index_1:3;
      uint32_t str_index_2:3;
      uint32_t reserved:1;
      uint32_t core_number:2; // MSB
    };
    uint32_t msg_id;
  };
} MessageID;

_Static_assert(sizeof(MessageID) == 4, "MessageID size != 4");

typedef struct BinLogMessage_Header {
  uint8_t version;
  uint8_t length;
} BinLogMessage_Header;

typedef struct BinLogMessage_Header_v0 {
  uint8_t version;
  uint8_t length;
  uint8_t reserved[2];
} BinLogMessage_Header_v0;
#define BINLOGMSG_VERSION_HEADER_V0 (0)

typedef struct BinLogMessage_Header_v1 {
  uint8_t version;
  uint8_t length;
  Date date;
  Time_Full time;
} BinLogMessage_Header_v1;
#define BINLOGMSG_VERSION_HEADER_V1 (BINLOGMSG_VERSION_TIME_DATE)

typedef struct BinLogMessage_Header_v2 {
  uint8_t version;
  uint8_t length;
  uint8_t reserved[2];
  Time_Tick tick_count;
} BinLogMessage_Header_v2;
#define BINLOGMSG_VERSION_HEADER_V2 (BINLOGMSG_VERSION_TICK_COUNT)

typedef struct BinLogMessage_Header_v3 {
  uint8_t version;
  uint8_t length;
  Date date;
  Time_Full time;
  Time_Tick tick_count;
} BinLogMessage_Header_v3;
#define BINLOGMSG_VERSION_HEADER_V3 (BINLOGMSG_VERSION_TIME_DATE | BINLOGMSG_VERSION_TICK_COUNT)

typedef struct BinLogMessage_ParamBody {
  MessageID msgid;
  uint32_t  payload[0];
} BinLogMessage_ParamBody;

typedef struct BinLogMessage_StringParam {
  uint8_t length;
  uint8_t string[0]; // string[length]
  // uint8_t padding[((length + sizeof(length) + 3) % 4)]
} BinLogMessage_StringParam;

typedef uint32_t BinLogMessage_IntParam;

typedef struct BinLogMessage_UnhashedBody {
  uint16_t line_number;
  uint8_t filename[16];
  uint8_t reserved:2;
  uint8_t core_number:2;
  uint8_t task_id:4;
  uint8_t level;
  uint8_t length;
  uint8_t string[0]; // string[length];
  // uint8_t padding[];
} BinLogMessage_UnhashedBody;


/*
int len = MAX(strlen(log_string), 255 - sizeof(BinLogMessage_Header_vX))
typedef struct BinLogMessage_SimpleBody {
  uint8_t string[len];
  uint8_t padding[((sizeof(BinLogMessage_Header_vX) + len + 3) % 4)];
} BinLogMessage_SimpleBody;
*/


typedef struct BinLogMessage_Param_v0 {
  BinLogMessage_Header_v0 header;
  BinLogMessage_ParamBody body;
} BinLogMessage_Param_v0;
#define BINLOGMSG_VERSION_PARAM_V0 (BINLOGMSG_VERSION_HEADER_V0 | BINLOGMSG_VERSION_PARAMETERIZED)

typedef struct BinLogMessage_Param_v1 {
  BinLogMessage_Header_v1 header;
  BinLogMessage_ParamBody body;
} BinLogMessage_Param_v1;
#define BINLOGMSG_VERSION_PARAM_V1 (BINLOGMSG_VERSION_HEADER_V1 | BINLOGMSG_VERSION_PARAMETERIZED)

typedef struct BinLogMessage_Param_v2 {
  BinLogMessage_Header_v2 header;
  BinLogMessage_ParamBody body;
} BinLogMessage_Param_v2;
#define BINLOGMSG_VERSION_PARAM_V2 (BINLOGMSG_VERSION_HEADER_V2 | BINLOGMSG_VERSION_PARAMETERIZED)

typedef struct BinLogMessage_Param_v3 {
  BinLogMessage_Header_v3 header;
  BinLogMessage_ParamBody body;
} BinLogMessage_Param_v3;
#define BINLOGMSG_VERSION_PARAM_V3 (BINLOGMSG_VERSION_HEADER_V3 | BINLOGMSG_VERSION_PARAMETERIZED)

typedef struct BinLogMessage_Unhashed_v0 {
  BinLogMessage_Header_v0 header;
  BinLogMessage_UnhashedBody body;
} BinLogMessage_Unhashed_v0;
#define BINLOGMSG_VERSION_UNHASHED_V0 (BINLOGMSG_VERSION_HEADER_V0 | BINLOGMSG_VERSION_UNHASHED_MSG)

typedef struct BinLogMessage_Unhashed_v1 {
  BinLogMessage_Header_v1 header;
  BinLogMessage_UnhashedBody body;
} BinLogMessage_Unhashed_v1;
#define BINLOGMSG_VERSION_UNHASHED_V1 (BINLOGMSG_VERSION_HEADER_V1 | BINLOGMSG_VERSION_UNHASHED_MSG)

typedef struct BinLogMessage_Unhashed_v2 {
  BinLogMessage_Header_v2 header;
  BinLogMessage_UnhashedBody body;
} BinLogMessage_Unhashed_v2;
#define BINLOGMSG_VERSION_UNHASHED_V2 (BINLOGMSG_VERSION_HEADER_V2 | BINLOGMSG_VERSION_UNHASHED_MSG)

typedef struct BinLogMessage_Unhashed_v3 {
  BinLogMessage_Header_v3 header;
  BinLogMessage_UnhashedBody body;
} BinLogMessage_Unhashed_v3;
#define BINLOGMSG_VERSION_UNHASHED_V3 (BINLOGMSG_VERSION_HEADER_V3 | BINLOGMSG_VERSION_UNHASHED_MSG)

typedef struct BinLogMessage_String_v1 {
  BinLogMessage_Header_v1 header;
  uint8_t string[0];
} BinLogMessage_String_v1;
#define BINLOGMSG_VERSION_STRING_V1 (BINLOGMSG_VERSION_HEADER_V1)
