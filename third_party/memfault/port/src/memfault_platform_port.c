//! @file
//!
//! Copyright (c) Memfault, Inc.
//! See LICENSE for details

#include <stdio.h>
#include <time.h>

#include "memfault/components.h"
#include "memfault/panics/arch/arm/cortex_m.h"
#include "memfault/ports/freertos.h"
#include "memfault/ports/freertos_coredump.h"
#include "memfault/ports/reboot_reason.h"

#include "mfg/mfg_serials.h"
#include "os/mutex.h"
#include "services/common/clock.h"
#include "services/common/new_timer/new_timer.h"
#include "system/logging.h"

// Buffer used to store formatted string for output
#define MEMFAULT_DEBUG_LOG_BUFFER_SIZE_BYTES \
  (sizeof("2024-11-27T14:19:29Z|123456780 I ") + MEMFAULT_DATA_EXPORT_BASE64_CHUNK_MAX_LEN)

#define MEMFAULT_COREDUMP_MAX_TASK_REGIONS ((MEMFAULT_PLATFORM_MAX_TRACKED_TASKS)*2)

// Reboot tracking storage, must be placed in no-init RAM to keep state after reboot
MEMFAULT_PUT_IN_SECTION(".noinit.mflt_reboot_info")
static uint8_t s_reboot_tracking[MEMFAULT_REBOOT_TRACKING_REGION_SIZE];

// Memfault logging storage
static uint8_t s_log_buf_storage[512];

// Use Memfault logging level to filter messages
static eMemfaultPlatformLogLevel s_min_log_level = MEMFAULT_RAM_LOGGER_DEFAULT_MIN_LOG_LEVEL;

void memfault_platform_get_device_info(sMemfaultDeviceInfo *info) {
  const char *mfg_serial_number = mfg_get_serial_number();
  const char *mfg_hw_rev = mfg_get_hw_version();

  *info = (sMemfaultDeviceInfo){
    .device_serial = (mfg_serial_number[0] != '\0') ? mfg_serial_number : "unknown",
    .hardware_version = (mfg_hw_rev[0] != '\0') ? mfg_hw_rev : "unknown",
    .software_type = "qemu-app",
    .software_version = "1.0.0",
  };
}

void memfault_platform_log(eMemfaultPlatformLogLevel level, const char *fmt, ...) {
#if 1
  va_list args;
  va_start(args, fmt);

  if (level >= s_min_log_level) {
    // If needed, additional data could be emitted in the log line (timestamp,
    // etc). Here we'll insert ANSI color codes depending on log level.
    switch (level) {
      case kMemfaultPlatformLogLevel_Debug:
        pbl_log_vargs(LOG_LEVEL_DEBUG, __FILE__, __LINE__, fmt, args);
        break;
      case kMemfaultPlatformLogLevel_Info:
        pbl_log_vargs(LOG_LEVEL_INFO, __FILE__, __LINE__, fmt, args);
        break;
      case kMemfaultPlatformLogLevel_Warning:
        pbl_log_vargs(LOG_LEVEL_WARNING, __FILE__, __LINE__, fmt, args);
        break;
      case kMemfaultPlatformLogLevel_Error:
        pbl_log_vargs(LOG_LEVEL_ERROR, __FILE__, __LINE__, fmt, args);
        break;
      default:
        break;
    }
  }

  va_end(args);
#endif
}

void memfault_platform_log_raw(const char *fmt, ...) {
  char log_buf[MEMFAULT_DEBUG_LOG_BUFFER_SIZE_BYTES];
  va_list args;
  va_start(args, fmt);

  vsnprintf(log_buf, sizeof(log_buf), fmt, args);
  printf("%s\n", log_buf);

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

void memfault_lock(void) {
  register uint32_t LR __asm ("lr");
  uint32_t myLR = LR;
  mutex_lock_recursive_with_timeout_and_lr(s_memfault_lock, portMAX_DELAY, myLR);
}

void memfault_unlock(void) {
  mutex_unlock_recursive(s_memfault_lock);
}

int memfault_platform_boot(void) {
  s_memfault_lock = mutex_create_recursive();

  memfault_platform_reboot_tracking_boot();

  static uint8_t s_event_storage[1024];
  const sMemfaultEventStorageImpl *evt_storage =
    memfault_events_storage_boot(s_event_storage, sizeof(s_event_storage));
  memfault_trace_event_boot(evt_storage);

  memfault_reboot_tracking_collect_reset_info(evt_storage);

  sMemfaultMetricBootInfo boot_info = {
    .unexpected_reboot_count = memfault_reboot_tracking_get_crash_count(),
  };
  memfault_metrics_boot(evt_storage, &boot_info);

  memfault_log_boot(s_log_buf_storage, MEMFAULT_ARRAY_SIZE(s_log_buf_storage));

  memfault_metrics_battery_boot();

  memfault_build_info_dump();
  memfault_device_info_dump();
  MEMFAULT_LOG_INFO("Memfault Initialized!");

  return 0;
}

static uint32_t prv_read_psp_reg(void) {
  uint32_t reg_val;
  __asm volatile("mrs %0, psp" : "=r"(reg_val));
  return reg_val;
}

static sMfltCoredumpRegion s_coredump_regions[MEMFAULT_COREDUMP_MAX_TASK_REGIONS +
                                              2   /* active stack(s) */
                                              + 1 /* _kernel variable */
                                              + 1 /* __memfault_capture_start */
                                              + 2 /* s_task_tcbs + s_task_watermarks */
];

extern uint32_t __memfault_capture_bss_end;
extern uint32_t __memfault_capture_bss_start;

const sMfltCoredumpRegion *memfault_platform_coredump_get_regions(
  const sCoredumpCrashInfo *crash_info, size_t *num_regions) {
  int region_idx = 0;
  const size_t active_stack_size_to_collect = 512;

  // first, capture the active stack (and ISR if applicable)
  const bool msp_was_active = (crash_info->exception_reg_state->exc_return & (1 << 2)) == 0;

  size_t stack_size_to_collect = memfault_platform_sanitize_address_range(
    crash_info->stack_address, MEMFAULT_PLATFORM_ACTIVE_STACK_SIZE_TO_COLLECT);

  s_coredump_regions[region_idx] =
    MEMFAULT_COREDUMP_MEMORY_REGION_INIT(crash_info->stack_address, stack_size_to_collect);
  region_idx++;

  if (msp_was_active) {
    // System crashed in an ISR but the running task state is on PSP so grab that too
    void *psp = (void *)prv_read_psp_reg();

    // Collect a little bit more stack for the PSP since there is an
    // exception frame that will have been stacked on it as well
    const uint32_t extra_stack_bytes = 128;
    stack_size_to_collect = memfault_platform_sanitize_address_range(
      psp, active_stack_size_to_collect + extra_stack_bytes);
    s_coredump_regions[region_idx] =
      MEMFAULT_COREDUMP_MEMORY_REGION_INIT(psp, stack_size_to_collect);
    region_idx++;
  }

  // Scoop up memory regions necessary to perform unwinds of the FreeRTOS tasks
  const size_t memfault_region_size =
    (uint32_t)&__memfault_capture_bss_end - (uint32_t)&__memfault_capture_bss_start;

  s_coredump_regions[region_idx] =
    MEMFAULT_COREDUMP_MEMORY_REGION_INIT(&__memfault_capture_bss_start, memfault_region_size);
  region_idx++;

  region_idx += memfault_freertos_get_task_regions(
    &s_coredump_regions[region_idx], MEMFAULT_ARRAY_SIZE(s_coredump_regions) - region_idx);

  *num_regions = region_idx;
  return &s_coredump_regions[0];
}

#include "FreeRTOS.h"

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

static TimerID s_memfault_heartbeat_timer;

static void prv_memfault_metrics_timer_cb(void *data) {
  MemfaultPlatformTimerCallback *callback = (MemfaultPlatformTimerCallback *)data;
  callback();
}

bool memfault_platform_metrics_timer_boot(uint32_t period_sec,
                                          MemfaultPlatformTimerCallback callback) {
  s_memfault_heartbeat_timer = new_timer_create();
  new_timer_start(s_memfault_heartbeat_timer, period_sec * 1000, prv_memfault_metrics_timer_cb,
                  (void *)callback, TIMER_START_FLAG_REPEATING);
  return true;
}
