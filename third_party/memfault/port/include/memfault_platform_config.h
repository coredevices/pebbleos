//! @file
//!
//! Copyright (c) Memfault, Inc.
//! See LICENSE for details
//!
//! Application platform config. Add Memfault configs here.

#pragma once

#define MEMFAULT_USE_GNU_BUILD_ID 1
#define MEMFAULT_PLATFORM_COREDUMP_STORAGE_RAM_SIZE 8192
#define MEMFAULT_COREDUMP_COLLECT_LOG_REGIONS 0
#define MEMFAULT_COREDUMP_COLLECT_HEAP_STATS 1
#define MEMFAULT_FREERTOS_PORT_HEAP_STATS_ENABLE 1
#define MEMFAULT_COREDUMP_HEAP_STATS_LOCK_ENABLE 0
#define MEMFAULT_METRICS_HEARTBEAT_INTERVAL_SECS (60 * 60)
#define MEMFAULT_COMPACT_LOG_ENABLE 0

#define MEMFAULT_COLLECT_MPU_STATE 1

#define MEMFAULT_COREDUMP_COMPUTE_THREAD_STACK_USAGE 1

// Todo, we will hook more deeply into the Pebble logging system
#define MEMFAULT_PLATFORM_HAS_LOG_CONFIG 0

// base64-encoded chunks can overflow the maximum serial output length, so use a
// slightly shorter length here
#define MEMFAULT_DATA_EXPORT_CHUNK_MAX_LEN 50

#define MEMFAULT_REBOOT_REASON_CUSTOM_ENABLE 1

#define MEMFAULT_METRICS_BATTERY_ENABLE 1
