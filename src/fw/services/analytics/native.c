/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include <string.h>

#include "console/prompt.h"
#include <pbl/drivers/rtc.h>
#include "pbl/os/mutex.h"
#include "pbl/services/analytics/backend.h"
#include "pbl/services/system_task.h"
#include "pbl/services/data_logging/data_logging_service.h"
#include <pbl/logging/logging.h>
#include "system/passert.h"
#include "pbl/util/attributes.h"
#include "pbl/util/build_id.h"
#include "pbl/util/math.h"
#include "pbl/util/size.h"
#include "pbl/util/uuid.h"

PBL_LOG_MODULE_DEFINE(service_analytics, CONFIG_SERVICE_ANALYTICS_LOG_LEVEL);

#define NATIVE_HEARTBEAT_RECORD_VERSION 3

/* Heartbeat record logged to DLS */
struct PACKED native_heartbeat_record {
  uint8_t version;
  uint64_t timestamp;
  uint8_t build_id[BUILD_ID_EXPECTED_LEN];
#define PBL_ANALYTICS_METRIC_DEFINE_UNSIGNED(key) uint32_t metric_##key;
#define PBL_ANALYTICS_METRIC_DEFINE_SIGNED(key) int32_t metric_##key;
#define PBL_ANALYTICS_METRIC_DEFINE_SCALED_UNSIGNED(key, scale) \
  uint32_t metric_##key;                                        \
  uint16_t metric_##key##_scale;
#define PBL_ANALYTICS_METRIC_DEFINE_SCALED_SIGNED(key, scale) \
  int32_t metric_##key;                                       \
  uint16_t metric_##key##_scale;
#define PBL_ANALYTICS_METRIC_DEFINE_TIMER(key) uint32_t metric_##key;
#define PBL_ANALYTICS_METRIC_DEFINE_STRING(key, len) char metric_##key[(len) + 1];
#include "pbl/services/analytics/analytics.def"
#undef PBL_ANALYTICS_METRIC_DEFINE_UNSIGNED
#undef PBL_ANALYTICS_METRIC_DEFINE_SIGNED
#undef PBL_ANALYTICS_METRIC_DEFINE_SCALED_UNSIGNED
#undef PBL_ANALYTICS_METRIC_DEFINE_SCALED_SIGNED
#undef PBL_ANALYTICS_METRIC_DEFINE_TIMER
#undef PBL_ANALYTICS_METRIC_DEFINE_STRING
};

/* The record is logged as a raw byte blob, so it must have no padding. */
_Static_assert(
    sizeof(struct native_heartbeat_record) ==
        sizeof(uint8_t) + sizeof(uint64_t) + BUILD_ID_EXPECTED_LEN
#define PBL_ANALYTICS_METRIC_DEFINE_UNSIGNED(key) +sizeof(uint32_t)
#define PBL_ANALYTICS_METRIC_DEFINE_SIGNED(key) +sizeof(int32_t)
#define PBL_ANALYTICS_METRIC_DEFINE_SCALED_UNSIGNED(key, scale) +sizeof(uint32_t) + sizeof(uint16_t)
#define PBL_ANALYTICS_METRIC_DEFINE_SCALED_SIGNED(key, scale) +sizeof(int32_t) + sizeof(uint16_t)
#define PBL_ANALYTICS_METRIC_DEFINE_TIMER(key) +sizeof(uint32_t)
#define PBL_ANALYTICS_METRIC_DEFINE_STRING(key, len) +((len) + 1)
#include "pbl/services/analytics/analytics.def"
#undef PBL_ANALYTICS_METRIC_DEFINE_UNSIGNED
#undef PBL_ANALYTICS_METRIC_DEFINE_SIGNED
#undef PBL_ANALYTICS_METRIC_DEFINE_SCALED_UNSIGNED
#undef PBL_ANALYTICS_METRIC_DEFINE_SCALED_SIGNED
#undef PBL_ANALYTICS_METRIC_DEFINE_TIMER
#undef PBL_ANALYTICS_METRIC_DEFINE_STRING
    ,
    "native_heartbeat_record must be packed (no padding)");

/* Type-specific internal index enums (dense, no gaps) */

enum native_integer_index {
#define PBL_ANALYTICS_METRIC_DEFINE_UNSIGNED(key) NATIVE_INTEGER_IDX_##key,
#define PBL_ANALYTICS_METRIC_DEFINE_SIGNED(key) NATIVE_INTEGER_IDX_##key,
#define PBL_ANALYTICS_METRIC_DEFINE_SCALED_UNSIGNED(key, scale) NATIVE_INTEGER_IDX_##key,
#define PBL_ANALYTICS_METRIC_DEFINE_SCALED_SIGNED(key, scale) NATIVE_INTEGER_IDX_##key,
#define PBL_ANALYTICS_METRIC_DEFINE_TIMER(key)
#define PBL_ANALYTICS_METRIC_DEFINE_STRING(key, len)
#include "pbl/services/analytics/analytics.def"
#undef PBL_ANALYTICS_METRIC_DEFINE_UNSIGNED
#undef PBL_ANALYTICS_METRIC_DEFINE_SIGNED
#undef PBL_ANALYTICS_METRIC_DEFINE_SCALED_UNSIGNED
#undef PBL_ANALYTICS_METRIC_DEFINE_SCALED_SIGNED
#undef PBL_ANALYTICS_METRIC_DEFINE_TIMER
#undef PBL_ANALYTICS_METRIC_DEFINE_STRING
  NATIVE_INTEGER_COUNT,
};

enum native_timer_index {
#define PBL_ANALYTICS_METRIC_DEFINE_UNSIGNED(key)
#define PBL_ANALYTICS_METRIC_DEFINE_SIGNED(key)
#define PBL_ANALYTICS_METRIC_DEFINE_SCALED_UNSIGNED(key, scale)
#define PBL_ANALYTICS_METRIC_DEFINE_SCALED_SIGNED(key, scale)
#define PBL_ANALYTICS_METRIC_DEFINE_TIMER(key) NATIVE_TIMER_IDX_##key,
#define PBL_ANALYTICS_METRIC_DEFINE_STRING(key, len)
#include "pbl/services/analytics/analytics.def"
#undef PBL_ANALYTICS_METRIC_DEFINE_UNSIGNED
#undef PBL_ANALYTICS_METRIC_DEFINE_SIGNED
#undef PBL_ANALYTICS_METRIC_DEFINE_SCALED_UNSIGNED
#undef PBL_ANALYTICS_METRIC_DEFINE_SCALED_SIGNED
#undef PBL_ANALYTICS_METRIC_DEFINE_TIMER
#undef PBL_ANALYTICS_METRIC_DEFINE_STRING
  NATIVE_TIMER_COUNT,
};

enum native_string_index {
#define PBL_ANALYTICS_METRIC_DEFINE_UNSIGNED(key)
#define PBL_ANALYTICS_METRIC_DEFINE_SIGNED(key)
#define PBL_ANALYTICS_METRIC_DEFINE_SCALED_UNSIGNED(key, scale)
#define PBL_ANALYTICS_METRIC_DEFINE_SCALED_SIGNED(key, scale)
#define PBL_ANALYTICS_METRIC_DEFINE_TIMER(key)
#define PBL_ANALYTICS_METRIC_DEFINE_STRING(key, len) NATIVE_STRING_IDX_##key,
#include "pbl/services/analytics/analytics.def"
#undef PBL_ANALYTICS_METRIC_DEFINE_UNSIGNED
#undef PBL_ANALYTICS_METRIC_DEFINE_SIGNED
#undef PBL_ANALYTICS_METRIC_DEFINE_SCALED_UNSIGNED
#undef PBL_ANALYTICS_METRIC_DEFINE_SCALED_SIGNED
#undef PBL_ANALYTICS_METRIC_DEFINE_TIMER
#undef PBL_ANALYTICS_METRIC_DEFINE_STRING
  NATIVE_STRING_COUNT,
};

/* Mapping tables: global key enum -> type-specific index (-1 if N/A) */

static const int8_t s_key_to_integer[] = {
#define PBL_ANALYTICS_METRIC_DEFINE_UNSIGNED(key) NATIVE_INTEGER_IDX_##key,
#define PBL_ANALYTICS_METRIC_DEFINE_SIGNED(key) NATIVE_INTEGER_IDX_##key,
#define PBL_ANALYTICS_METRIC_DEFINE_SCALED_UNSIGNED(key, scale) NATIVE_INTEGER_IDX_##key,
#define PBL_ANALYTICS_METRIC_DEFINE_SCALED_SIGNED(key, scale) NATIVE_INTEGER_IDX_##key,
#define PBL_ANALYTICS_METRIC_DEFINE_TIMER(key) -1,
#define PBL_ANALYTICS_METRIC_DEFINE_STRING(key, len) -1,
#include "pbl/services/analytics/analytics.def"
#undef PBL_ANALYTICS_METRIC_DEFINE_UNSIGNED
#undef PBL_ANALYTICS_METRIC_DEFINE_SIGNED
#undef PBL_ANALYTICS_METRIC_DEFINE_SCALED_UNSIGNED
#undef PBL_ANALYTICS_METRIC_DEFINE_SCALED_SIGNED
#undef PBL_ANALYTICS_METRIC_DEFINE_TIMER
#undef PBL_ANALYTICS_METRIC_DEFINE_STRING
};

static const int8_t s_key_to_timer[] = {
#define PBL_ANALYTICS_METRIC_DEFINE_UNSIGNED(key) -1,
#define PBL_ANALYTICS_METRIC_DEFINE_SIGNED(key) -1,
#define PBL_ANALYTICS_METRIC_DEFINE_SCALED_UNSIGNED(key, scale) -1,
#define PBL_ANALYTICS_METRIC_DEFINE_SCALED_SIGNED(key, scale) -1,
#define PBL_ANALYTICS_METRIC_DEFINE_TIMER(key) NATIVE_TIMER_IDX_##key,
#define PBL_ANALYTICS_METRIC_DEFINE_STRING(key, len) -1,
#include "pbl/services/analytics/analytics.def"
#undef PBL_ANALYTICS_METRIC_DEFINE_UNSIGNED
#undef PBL_ANALYTICS_METRIC_DEFINE_SIGNED
#undef PBL_ANALYTICS_METRIC_DEFINE_SCALED_UNSIGNED
#undef PBL_ANALYTICS_METRIC_DEFINE_SCALED_SIGNED
#undef PBL_ANALYTICS_METRIC_DEFINE_TIMER
#undef PBL_ANALYTICS_METRIC_DEFINE_STRING
};

static const int8_t s_key_to_string[] = {
#define PBL_ANALYTICS_METRIC_DEFINE_UNSIGNED(key) -1,
#define PBL_ANALYTICS_METRIC_DEFINE_SIGNED(key) -1,
#define PBL_ANALYTICS_METRIC_DEFINE_SCALED_UNSIGNED(key, scale) -1,
#define PBL_ANALYTICS_METRIC_DEFINE_SCALED_SIGNED(key, scale) -1,
#define PBL_ANALYTICS_METRIC_DEFINE_TIMER(key) -1,
#define PBL_ANALYTICS_METRIC_DEFINE_STRING(key, len) NATIVE_STRING_IDX_##key,
#include "pbl/services/analytics/analytics.def"
#undef PBL_ANALYTICS_METRIC_DEFINE_UNSIGNED
#undef PBL_ANALYTICS_METRIC_DEFINE_SIGNED
#undef PBL_ANALYTICS_METRIC_DEFINE_SCALED_UNSIGNED
#undef PBL_ANALYTICS_METRIC_DEFINE_SCALED_SIGNED
#undef PBL_ANALYTICS_METRIC_DEFINE_TIMER
#undef PBL_ANALYTICS_METRIC_DEFINE_STRING
};

/* Type-specific storage */

static int32_t s_integer_values[NATIVE_INTEGER_COUNT];

static struct {
  uint32_t value_ms;
  bool running;
  RtcTicks start_ticks;
} s_timers[NATIVE_TIMER_COUNT];

/* Per-string dedicated buffers (sized to declared length) */
#define PBL_ANALYTICS_METRIC_DEFINE_UNSIGNED(key)
#define PBL_ANALYTICS_METRIC_DEFINE_SIGNED(key)
#define PBL_ANALYTICS_METRIC_DEFINE_SCALED_UNSIGNED(key, scale)
#define PBL_ANALYTICS_METRIC_DEFINE_SCALED_SIGNED(key, scale)
#define PBL_ANALYTICS_METRIC_DEFINE_TIMER(key)
#define PBL_ANALYTICS_METRIC_DEFINE_STRING(key, len) static char s_string_##key[(len) + 1];
#include "pbl/services/analytics/analytics.def"
#undef PBL_ANALYTICS_METRIC_DEFINE_UNSIGNED
#undef PBL_ANALYTICS_METRIC_DEFINE_SIGNED
#undef PBL_ANALYTICS_METRIC_DEFINE_SCALED_UNSIGNED
#undef PBL_ANALYTICS_METRIC_DEFINE_SCALED_SIGNED
#undef PBL_ANALYTICS_METRIC_DEFINE_TIMER
#undef PBL_ANALYTICS_METRIC_DEFINE_STRING

/* String pointer and length lookup tables */

static char *const s_string_ptrs[] = {
#define PBL_ANALYTICS_METRIC_DEFINE_UNSIGNED(key)
#define PBL_ANALYTICS_METRIC_DEFINE_SIGNED(key)
#define PBL_ANALYTICS_METRIC_DEFINE_SCALED_UNSIGNED(key, scale)
#define PBL_ANALYTICS_METRIC_DEFINE_SCALED_SIGNED(key, scale)
#define PBL_ANALYTICS_METRIC_DEFINE_TIMER(key)
#define PBL_ANALYTICS_METRIC_DEFINE_STRING(key, len) s_string_##key,
#include "pbl/services/analytics/analytics.def"
#undef PBL_ANALYTICS_METRIC_DEFINE_UNSIGNED
#undef PBL_ANALYTICS_METRIC_DEFINE_SIGNED
#undef PBL_ANALYTICS_METRIC_DEFINE_SCALED_UNSIGNED
#undef PBL_ANALYTICS_METRIC_DEFINE_SCALED_SIGNED
#undef PBL_ANALYTICS_METRIC_DEFINE_TIMER
#undef PBL_ANALYTICS_METRIC_DEFINE_STRING
};

static const uint8_t s_string_lens[] = {
#define PBL_ANALYTICS_METRIC_DEFINE_UNSIGNED(key)
#define PBL_ANALYTICS_METRIC_DEFINE_SIGNED(key)
#define PBL_ANALYTICS_METRIC_DEFINE_SCALED_UNSIGNED(key, scale)
#define PBL_ANALYTICS_METRIC_DEFINE_SCALED_SIGNED(key, scale)
#define PBL_ANALYTICS_METRIC_DEFINE_TIMER(key)
#define PBL_ANALYTICS_METRIC_DEFINE_STRING(key, len) (len),
#include "pbl/services/analytics/analytics.def"
#undef PBL_ANALYTICS_METRIC_DEFINE_UNSIGNED
#undef PBL_ANALYTICS_METRIC_DEFINE_SIGNED
#undef PBL_ANALYTICS_METRIC_DEFINE_SCALED_UNSIGNED
#undef PBL_ANALYTICS_METRIC_DEFINE_SCALED_SIGNED
#undef PBL_ANALYTICS_METRIC_DEFINE_TIMER
#undef PBL_ANALYTICS_METRIC_DEFINE_STRING
};

static PebbleMutex *s_mutex;
static DataLoggingSession *s_dls_session;

extern const ElfExternalNote TINTIN_BUILD_ID;

static void prv_record_metrics(struct native_heartbeat_record *record, bool reset) {
  uint32_t timer_value_ms[NATIVE_TIMER_COUNT];

  memset(record, 0, sizeof(*record));

  record->version = NATIVE_HEARTBEAT_RECORD_VERSION;
  record->timestamp = rtc_get_time();
  memcpy(record->build_id, &TINTIN_BUILD_ID.data[TINTIN_BUILD_ID.name_length],
         MIN(BUILD_ID_EXPECTED_LEN, TINTIN_BUILD_ID.data_length));

  RtcTicks now = rtc_get_ticks();
  for (size_t i = 0; i < NATIVE_TIMER_COUNT; i++) {
    timer_value_ms[i] = s_timers[i].value_ms;
    if (s_timers[i].running) {
      RtcTicks elapsed = now - s_timers[i].start_ticks;
      timer_value_ms[i] += (elapsed * 1000) / RTC_TICKS_HZ;
      if (reset) {
        s_timers[i].value_ms = timer_value_ms[i];
        s_timers[i].start_ticks = now;
      }
    }
  }

  /* Build heartbeat record from type-specific storage */
#define PBL_ANALYTICS_METRIC_DEFINE_UNSIGNED(key) \
  record->metric_##key = (uint32_t)s_integer_values[NATIVE_INTEGER_IDX_##key];
#define PBL_ANALYTICS_METRIC_DEFINE_SIGNED(key) \
  record->metric_##key = s_integer_values[NATIVE_INTEGER_IDX_##key];
#define PBL_ANALYTICS_METRIC_DEFINE_SCALED_UNSIGNED(key, scale)                \
  record->metric_##key = (uint32_t)s_integer_values[NATIVE_INTEGER_IDX_##key]; \
  record->metric_##key##_scale = (scale);
#define PBL_ANALYTICS_METRIC_DEFINE_SCALED_SIGNED(key, scale)        \
  record->metric_##key = s_integer_values[NATIVE_INTEGER_IDX_##key]; \
  record->metric_##key##_scale = (scale);
#define PBL_ANALYTICS_METRIC_DEFINE_TIMER(key) \
  record->metric_##key = timer_value_ms[NATIVE_TIMER_IDX_##key];
#define PBL_ANALYTICS_METRIC_DEFINE_STRING(key, len)    \
  strncpy(record->metric_##key, s_string_##key, (len)); \
  record->metric_##key[(len)] = '\0';
#include "pbl/services/analytics/analytics.def"
#undef PBL_ANALYTICS_METRIC_DEFINE_UNSIGNED
#undef PBL_ANALYTICS_METRIC_DEFINE_SIGNED
#undef PBL_ANALYTICS_METRIC_DEFINE_SCALED_UNSIGNED
#undef PBL_ANALYTICS_METRIC_DEFINE_SCALED_SIGNED
#undef PBL_ANALYTICS_METRIC_DEFINE_TIMER
#undef PBL_ANALYTICS_METRIC_DEFINE_STRING

  if (reset) {
    /* Reset storage for next heartbeat period, keeping running timers active */
    memset(s_integer_values, 0, sizeof(s_integer_values));
    for (size_t i = 0; i < NATIVE_TIMER_COUNT; i++) {
      s_timers[i].value_ms = 0;
    }
    for (size_t i = 0; i < NATIVE_STRING_COUNT; i++) {
      s_string_ptrs[i][0] = '\0';
    }
  }
}

void pbl_analytics__native_init(void) {
  s_mutex = mutex_create();
  PBL_ASSERTN(s_mutex != NULL);
}

void pbl_analytics__native_heartbeat(void) {
  struct native_heartbeat_record record;

  mutex_lock(s_mutex);
  prv_record_metrics(&record, true);
  mutex_unlock(s_mutex);

  if (s_dls_session == NULL) {
    Uuid system_uuid = UUID_SYSTEM;

    s_dls_session = dls_create(DlsSystemTagAnalyticsNativeHeartbeat, DATA_LOGGING_BYTE_ARRAY,
                               sizeof(struct native_heartbeat_record), true, false, &system_uuid);
    PBL_ASSERTN(s_dls_session != NULL);
  }

  DataLoggingResult result = dls_log(s_dls_session, &record, 1);
  if (result != DATA_LOGGING_SUCCESS) {
    PBL_LOG_ERR("Native analytics DLS log failed: %d", result);
  }
}

static void prv_set_signed(enum pbl_analytics_key key, int32_t signed_value) {
  int8_t idx = s_key_to_integer[key];
  if (idx < 0) {
    return;
  }
  mutex_lock(s_mutex);
  s_integer_values[idx] = signed_value;
  mutex_unlock(s_mutex);
}

static void prv_set_unsigned(enum pbl_analytics_key key, uint32_t unsigned_value) {
  int8_t idx = s_key_to_integer[key];
  if (idx < 0) {
    return;
  }
  mutex_lock(s_mutex);
  s_integer_values[idx] = (int32_t)unsigned_value;
  mutex_unlock(s_mutex);
}

static void prv_set_string(enum pbl_analytics_key key, const char *value) {
  int8_t idx = s_key_to_string[key];
  if (idx < 0) {
    return;
  }
  mutex_lock(s_mutex);
  strncpy(s_string_ptrs[idx], value, s_string_lens[idx]);
  s_string_ptrs[idx][s_string_lens[idx]] = '\0';
  mutex_unlock(s_mutex);
}

static void prv_timer_start(enum pbl_analytics_key key) {
  int8_t idx = s_key_to_timer[key];
  if (idx < 0) {
    return;
  }
  mutex_lock(s_mutex);
  if (!s_timers[idx].running) {
    s_timers[idx].running = true;
    s_timers[idx].start_ticks = rtc_get_ticks();
  }
  mutex_unlock(s_mutex);
}

static void prv_timer_stop(enum pbl_analytics_key key) {
  int8_t idx = s_key_to_timer[key];
  if (idx < 0) {
    return;
  }
  mutex_lock(s_mutex);
  if (s_timers[idx].running) {
    RtcTicks elapsed = rtc_get_ticks() - s_timers[idx].start_ticks;
    s_timers[idx].value_ms += (int32_t)((elapsed * 1000) / RTC_TICKS_HZ);
    s_timers[idx].running = false;
  }
  mutex_unlock(s_mutex);
}

static void prv_add(enum pbl_analytics_key key, int32_t amount) {
  int8_t idx = s_key_to_integer[key];
  if (idx < 0) {
    return;
  }
  mutex_lock(s_mutex);
  s_integer_values[idx] += amount;
  mutex_unlock(s_mutex);
}

const struct pbl_analytics_backend_ops pbl_analytics__native_ops = {
    .set_signed = prv_set_signed,
    .set_unsigned = prv_set_unsigned,
    .set_string = prv_set_string,
    .timer_start = prv_timer_start,
    .timer_stop = prv_timer_stop,
    .add = prv_add,
};

void command_analytics_native_metrics_dump(void) {
  struct native_heartbeat_record record;
  char buffer[64];

  prv_record_metrics(&record, false);

#define PBL_ANALYTICS_METRIC_DEFINE_UNSIGNED(key) \
  prompt_send_response_fmt(buffer, sizeof(buffer), STRINGIFY(key) "=%" PRIu32, record.metric_##key);
#define PBL_ANALYTICS_METRIC_DEFINE_SIGNED(key) \
  prompt_send_response_fmt(buffer, sizeof(buffer), STRINGIFY(key) "=%" PRId32, record.metric_##key);
#define PBL_ANALYTICS_METRIC_DEFINE_SCALED_UNSIGNED(key, scale)                            \
  prompt_send_response_fmt(buffer, sizeof(buffer), STRINGIFY(key) "=%" PRIu32 ".%" PRIu32, \
                           record.metric_##key / (scale),                                  \
                           record.metric_##key - (record.metric_##key / (scale)) * (scale));
#define PBL_ANALYTICS_METRIC_DEFINE_SCALED_SIGNED(key, scale)         \
  prompt_send_response_fmt(                                           \
      buffer, sizeof(buffer), STRINGIFY(key) "=%" PRId32 ".%" PRIu32, \
      record.metric_##key / (scale),                                  \
      (uint32_t)(record.metric_##key - (record.metric_##key / (scale)) * (scale)));
#define PBL_ANALYTICS_METRIC_DEFINE_TIMER(key)                                       \
  prompt_send_response_fmt(buffer, sizeof(buffer), STRINGIFY(key) "=%" PRId32 " ms", \
                           record.metric_##key);
#define PBL_ANALYTICS_METRIC_DEFINE_STRING(key, len) \
  prompt_send_response_fmt(buffer, sizeof(buffer), STRINGIFY(key) "=%s", record.metric_##key);
#include "pbl/services/analytics/analytics.def"
#undef PBL_ANALYTICS_METRIC_DEFINE_UNSIGNED
#undef PBL_ANALYTICS_METRIC_DEFINE_SIGNED
#undef PBL_ANALYTICS_METRIC_DEFINE_SCALED_UNSIGNED
#undef PBL_ANALYTICS_METRIC_DEFINE_SCALED_SIGNED
#undef PBL_ANALYTICS_METRIC_DEFINE_TIMER
#undef PBL_ANALYTICS_METRIC_DEFINE_STRING
}