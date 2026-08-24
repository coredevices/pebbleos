/* SPDX-CopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include <stdio.h>
#include <time.h>

#include "memfault/components.h"
#include "memfault/panics/arch/arm/cortex_m.h"
#include "memfault/ports/reboot_reason.h"
#include "memfault_chunk_collector.h"
#include "memfault_pebble_coredump.h"

#include "mfg/mfg_serials.h"
#include "pbl/os/mutex.h"
#include "pbl/services/clock.h"
#include "pbl/services/new_timer/new_timer.h"
#include "pbl/util/misc.h"
#include <pbl/logging/logging.h>
#include "system/version.h"

#include "FreeRTOS.h"
#include "task.h"

// Buffer used to store formatted string for output
#define MEMFAULT_DEBUG_LOG_BUFFER_SIZE_BYTES \
  (sizeof("2024-11-27T14:19:29Z|123456780 I ") + MEMFAULT_DATA_EXPORT_BASE64_CHUNK_MAX_LEN)

// Reboot tracking storage, must be placed in no-init RAM to keep state after reboot
MEMFAULT_PUT_IN_SECTION(".noinit.mflt_reboot_info")
static uint8_t s_reboot_tracking[MEMFAULT_REBOOT_TRACKING_REGION_SIZE];

// Memfault logging storage
static uint8_t s_log_buf_storage[2048];

// Use Memfault logging level to filter messages
static eMemfaultPlatformLogLevel s_min_log_level = MEMFAULT_RAM_LOGGER_DEFAULT_MIN_LOG_LEVEL;

void memfault_platform_get_device_info(sMemfaultDeviceInfo *info) {
  const char *mfg_serial_number = mfg_get_serial_number();
  const char *mfg_hw_rev = mfg_get_hw_version();

  *info = (sMemfaultDeviceInfo){
    .device_serial = (mfg_serial_number[0] != '\0') ? mfg_serial_number : "unknown",
    .hardware_version = (mfg_hw_rev[0] != '\0') ? mfg_hw_rev : "unknown",
    .software_type = "pebbleos",
    .software_version = TINTIN_METADATA.version_tag,
  };
}

void memfault_platform_log(eMemfaultPlatformLogLevel level, const char *fmt, ...) {
  // Logging can be reentrant into memfault (i.e., logging can call into
  // memfault because there is a flash write).  This could cause a deadlock
  // if someone holds the flash lock and increments an analytic while
  // memfault is otherwise logging (and holds the memfault lock and waits
  // for the flash lock).  Memfault logs must all be logged at higher than
  // FLASH_LOG_LEVEL.
# if FLASH_LOG_LEVEL >= LOG_LEVEL_DEBUG
#   warning memfault logging cannot be used if debug messages are logged to flash
    return;
# endif

  va_list args;
  va_start(args, fmt);


  if (level >= s_min_log_level) {
    pbl_log_vargs(LOG_LEVEL_DEBUG, __FILE__, __LINE__, fmt, args);
  }

  va_end(args);
}

void memfault_platform_log_raw(const char *fmt, ...) {
# if FLASH_LOG_LEVEL >= LOG_LEVEL_DEBUG
    return;
# endif

  char log_buf[MEMFAULT_DEBUG_LOG_BUFFER_SIZE_BYTES];
  va_list args;
  va_start(args, fmt);

  vsnprintf(log_buf, sizeof(log_buf), fmt, args);
  PBL_LOG_DBG("%s", log_buf);

  va_end(args);
}

bool memfault_platform_time_get_current(sMemfaultCurrentTime *time_output) {
  // Debug: print time fields
  struct tm tm_time;
  clock_get_time_tm(&tm_time);
  MEMFAULT_LOG_DEBUG("Time: %u-%u-%u %u:%u:%u", tm_time.tm_year + 1900, tm_time.tm_mon + 1,
                     tm_time.tm_mday, tm_time.tm_hour, tm_time.tm_min, tm_time.tm_sec);

  // If pre-2023, something is wrong
  if ((tm_time.tm_year < 123) || (tm_time.tm_year > 200)) {
    return false;
  }

  time_t time_now = mktime(&tm_time);

  // load the timestamp and return true for a valid timestamp
  *time_output = (sMemfaultCurrentTime){
    .type = kMemfaultCurrentTimeType_UnixEpochTimeSec,
    .info = {
      .unix_timestamp_secs = (uint64_t)time_now,
    },
  };
  return true;
}

void memfault_platform_reboot_tracking_boot(void) {
  sResetBootupInfo reset_info = { 0 };
  memfault_reboot_reason_get(&reset_info);
  memfault_reboot_tracking_boot(s_reboot_tracking, &reset_info);
}

static PebbleRecursiveMutex *s_memfault_lock;
static bool s_boot_in_progress;

void memfault_lock(void) {
  // During boot initialization, skip locking to avoid deadlock when
  // metrics iteration is called from boot context (e.g., during
  // size computation in memfault_metrics_boot)
  if (s_boot_in_progress || !s_memfault_lock) {
    return;
  }

  uint32_t myLR = PBL_RETURN_ADDRESS();
  mutex_lock_recursive_with_timeout_and_lr(s_memfault_lock, portMAX_DELAY, myLR);
}

void memfault_unlock(void) {
  if (s_boot_in_progress || !s_memfault_lock) {
    return;
  }
  mutex_unlock_recursive(s_memfault_lock);
}

// We can't write to flash or read from flash yet (including for serial
// numbers!), but we do at least want to be able to log metrics ASAP.
void memfault_platform_boot_early(void) {
  s_memfault_lock = mutex_create_recursive();

  memfault_platform_reboot_tracking_boot();
}

int memfault_platform_boot(void) {
#ifndef MEMFAULT_FORCE
  if (!version_is_release_build()) {
    MEMFAULT_LOG_INFO("Memfault disabled for non-release build (%s)",
                      TINTIN_METADATA.version_tag);
    return 0;
  }
#endif

  // Set boot flag to disable locking during initialization
  s_boot_in_progress = true;

  // Tracing requires being able to read flash, so don't do that in early boot!
  static uint8_t s_event_storage[1024];
  const sMemfaultEventStorageImpl *evt_storage =
    memfault_events_storage_boot(s_event_storage, sizeof(s_event_storage));
  memfault_trace_event_boot(evt_storage);

  // Initialize log buffer early so reconstruction can find the log region
  // addresses via memfault_log_get_regions().
  memfault_log_boot(s_log_buf_storage, MEMFAULT_ARRAY_SIZE(s_log_buf_storage));

  // Reconstruct a Memfault coredump from the PebbleOS flash coredump if one
  // exists. This must happen before collect_reset_info() so that the reboot
  // event includes coredump_saved=true, allowing the Memfault cloud to
  // associate the coredump with this reboot.
  memfault_pebble_coredump_reconstruct();

  memfault_reboot_tracking_collect_reset_info(evt_storage);

  sMemfaultMetricBootInfo boot_info = {
    .unexpected_reboot_count = memfault_reboot_tracking_get_crash_count(),
  };
  memfault_metrics_boot(evt_storage, &boot_info);

  memfault_build_info_dump();
  memfault_device_info_dump();

  init_memfault_chunk_collection();
  MEMFAULT_LOG_INFO("Memfault Initialized!");

  // Clear boot flag - locking is now enabled
  s_boot_in_progress = false;

  return 0;
}

// [MJ] FIXME: We shouldinstead use the Memfault FreeRTOS port, but it requires
// a newer version of FreeRTOS + assumes FreeRTOS timers are available.
uint64_t memfault_platform_get_time_since_boot_ms(void) {
  static uint64_t s_elapsed_ticks = 0;
  static uint32_t s_last_tick_count = 0;

  taskENTER_CRITICAL();
  {
    const uint32_t curr_tick_count = xTaskGetTickCount();

    // NB: Since we are doing unsigned arithmetic here, this operation will still work when
    // xTaskGetTickCount() has overflowed and wrapped around
    s_elapsed_ticks += (curr_tick_count - s_last_tick_count);
    s_last_tick_count = curr_tick_count;
  }
  taskEXIT_CRITICAL();

  return (s_elapsed_ticks * 1000) / configTICK_RATE_HZ;
}

static MemfaultPlatformTimerCallback *s_memfault_heartbeat_callback;

bool memfault_platform_metrics_timer_boot(uint32_t period_sec,
                                          MemfaultPlatformTimerCallback callback) {
  s_memfault_heartbeat_callback = callback;
  return true;
}

void memfault_platform_heartbeat(void) {
  s_memfault_heartbeat_callback();
}
