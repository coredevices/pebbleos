/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "pebble_tasks.h"

#include "kernel/memory_layout.h"
#include "kernel/pbl_malloc.h"

#include "process_management/app_manager.h"
#include "process_management/worker_manager.h"
#include "pbl/services/analytics/analytics.h"
#include "syscall/syscall_internal.h"
#include "system/passert.h"
#include "pbl/util/size.h"

#include "FreeRTOS.h"
#include "task.h"

TaskHandle_t g_task_handles[NumPebbleTask] KERNEL_READONLY_DATA = { 0 };

// Cycles consumed by tasks that have already been destroyed in each slot.
// Captured at unregister time so the analytics heartbeat can keep accounting
// for App/Worker activity across short-lived task instances.
static uint32_t s_dead_task_cycles[NumPebbleTask];

static void prv_task_register(PebbleTask task, TaskHandle_t task_handle) {
  g_task_handles[task] = task_handle;
}

static uint32_t prv_read_task_run_time(TaskHandle_t handle) {
  UBaseType_t num_tasks = uxTaskGetNumberOfTasks();
  TaskStatus_t *statuses = kernel_malloc(num_tasks * sizeof(TaskStatus_t));
  if (!statuses) {
    return 0;
  }
  UBaseType_t count = uxTaskGetSystemState(statuses, num_tasks, NULL);
  uint32_t cycles = 0;
  for (UBaseType_t i = 0; i < count; i++) {
    if (statuses[i].xHandle == handle) {
      cycles = statuses[i].ulRunTimeCounter;
      break;
    }
  }
  kernel_free(statuses);
  return cycles;
}

void pebble_task_unregister(PebbleTask task) {
  TaskHandle_t handle = g_task_handles[task];
  if (handle == NULL) {
    return;
  }
  uint32_t cycles = prv_read_task_run_time(handle);
  // Clear the handle before crediting the cycles: the collector reads
  // s_dead_task_cycles before walking the task list, so this ordering
  // ensures cycles are never seen in both buckets simultaneously.
  g_task_handles[task] = NULL;
  s_dead_task_cycles[task] += cycles;
}

const char* pebble_task_get_name(PebbleTask task) {
  if (task >= NumPebbleTask) {
    if (task == PebbleTask_Unknown) {
      return "Unknown";
    }
    WTF;
  }

  TaskHandle_t task_handle = g_task_handles[task];
  if (!task_handle) {
    return "Unknown";
  }
  return (const char*) pcTaskGetTaskName(task_handle);
}

// NOTE: The logging support calls toupper() this character if the task is currently running privileged, so
//  these identifiers should be all lower case and case-insensitive. 
char pebble_task_get_char(PebbleTask task) {
  switch (task) {
  case PebbleTask_KernelMain:
    return 'm';
  case PebbleTask_KernelBackground:
    return 's';
  case PebbleTask_Worker:
    return 'w';
  case PebbleTask_App:
    return 'a';
  case PebbleTask_BTHost:
    return 'b';
  case PebbleTask_BTController:
    return 'c';
  case PebbleTask_BTHCI:
    return 'd';
  case PebbleTask_NewTimers:
    return 't';
  case PebbleTask_PULSE:
    return 'p';
  case NumPebbleTask:
  case PebbleTask_Unknown:
    ;
  }

  return '?';
}

PebbleTask pebble_task_get_current(void) {
  TaskHandle_t task_handle = xTaskGetCurrentTaskHandle();
  return pebble_task_get_task_for_handle(task_handle);
}

PebbleTask pebble_task_get_task_for_handle(TaskHandle_t task_handle) {
  for (int i = 0; i < (int) ARRAY_LENGTH(g_task_handles); ++i) {
    if (g_task_handles[i] == task_handle) {
      return i;
    }
  }
  return PebbleTask_Unknown;
}

TaskHandle_t pebble_task_get_handle_for_task(PebbleTask task) {
  return g_task_handles[task];
}

static uint16_t prv_task_get_stack_free(PebbleTask task) {
  // If task doesn't exist, return a dummy with max value
  if (g_task_handles[task] == NULL) {
    return 0xFFFF;
  }
  return uxTaskGetStackHighWaterMark(g_task_handles[task]);
}

void pebble_task_suspend(PebbleTask task) {
  PBL_ASSERTN(task < NumPebbleTask);
  vTaskSuspend(g_task_handles[task]);
}

void pbl_analytics_external_collect_stack_free(void) {
  PBL_ANALYTICS_SET_UNSIGNED(stack_free_kernel_main_bytes, prv_task_get_stack_free(PebbleTask_KernelMain));
  PBL_ANALYTICS_SET_UNSIGNED(stack_free_kernel_background_bytes, prv_task_get_stack_free(PebbleTask_KernelBackground));
  PBL_ANALYTICS_SET_UNSIGNED(stack_free_newtimers_bytes, prv_task_get_stack_free(PebbleTask_NewTimers));
  PBL_ANALYTICS_SET_UNSIGNED(stack_free_app_syscall_bytes, syscall_app_stack_free_bytes());
  PBL_ANALYTICS_SET_UNSIGNED(stack_free_worker_syscall_bytes, syscall_worker_stack_free_bytes());
}

static const enum pbl_analytics_key s_task_cpu_pct_keys[NumPebbleTask] = {
    [PebbleTask_KernelMain] = PBL_ANALYTICS_KEY(task_cpu_kernel_main_pct),
    [PebbleTask_KernelBackground] = PBL_ANALYTICS_KEY(task_cpu_kernel_background_pct),
    [PebbleTask_Worker] = PBL_ANALYTICS_KEY(task_cpu_worker_pct),
    [PebbleTask_App] = PBL_ANALYTICS_KEY(task_cpu_app_pct),
    [PebbleTask_BTHost] = PBL_ANALYTICS_KEY(task_cpu_bt_host_pct),
    [PebbleTask_BTController] = PBL_ANALYTICS_KEY(task_cpu_bt_controller_pct),
    [PebbleTask_BTHCI] = PBL_ANALYTICS_KEY(task_cpu_bt_hci_pct),
    [PebbleTask_NewTimers] = PBL_ANALYTICS_KEY(task_cpu_new_timers_pct),
    [PebbleTask_PULSE] = PBL_ANALYTICS_KEY(task_cpu_pulse_pct),
};

void pbl_analytics_external_collect_task_cpu_stats(void) {
  static uint32_t s_prev_total_task_cycles[NumPebbleTask];
  static uint32_t s_prev_idle_run_time;
  static uint32_t s_prev_total_run_time;

  // Snapshot dead-task cycles before walking the live task list. Combined with
  // the (clear-handle, then update-accumulator) ordering in
  // pebble_task_unregister(), this guarantees that cycles from a task dying
  // mid-collection are never double-counted; in the worst case they show up
  // one heartbeat late.
  uint32_t dead_cycles[NumPebbleTask];
  for (int task = 0; task < NumPebbleTask; task++) {
    dead_cycles[task] = s_dead_task_cycles[task];
  }

  UBaseType_t num_tasks = uxTaskGetNumberOfTasks();
  TaskStatus_t *statuses = kernel_malloc(num_tasks * sizeof(TaskStatus_t));
  if (!statuses) {
    return;
  }

  uint32_t total_run_time;
  UBaseType_t count = uxTaskGetSystemState(statuses, num_tasks, &total_run_time);

  uint32_t delta_total = total_run_time - s_prev_total_run_time;
  s_prev_total_run_time = total_run_time;

  TaskHandle_t idle_handle = xTaskGetIdleTaskHandle();
  uint32_t curr_task_run_time[NumPebbleTask] = {0};
  uint32_t curr_idle_run_time = 0;
  for (UBaseType_t i = 0; i < count; i++) {
    if (statuses[i].xHandle == idle_handle) {
      curr_idle_run_time = statuses[i].ulRunTimeCounter;
      continue;
    }
    PebbleTask task = pebble_task_get_task_for_handle(statuses[i].xHandle);
    if (task < NumPebbleTask) {
      curr_task_run_time[task] = statuses[i].ulRunTimeCounter;
    }
  }

  kernel_free(statuses);

  for (int task = 0; task < NumPebbleTask; task++) {
    uint32_t total = dead_cycles[task] + curr_task_run_time[task];
    uint32_t delta = total - s_prev_total_task_cycles[task];
    s_prev_total_task_cycles[task] = total;
    uint32_t pct = delta_total ? (uint32_t)(((uint64_t)delta * 10000U) / delta_total) : 0;
    sys_pbl_analytics_set_unsigned(s_task_cpu_pct_keys[task], pct);
  }

  uint32_t idle_delta = curr_idle_run_time - s_prev_idle_run_time;
  s_prev_idle_run_time = curr_idle_run_time;
  uint32_t idle_pct =
      delta_total ? (uint32_t)(((uint64_t)idle_delta * 10000U) / delta_total) : 0;
  PBL_ANALYTICS_SET_UNSIGNED(task_cpu_idle_pct, idle_pct);
}

QueueHandle_t pebble_task_get_to_queue(PebbleTask task) {
  QueueHandle_t queue;
  switch (task) {
    case PebbleTask_KernelMain:
      queue = event_get_to_kernel_queue(pebble_task_get_current());
      break;
    case PebbleTask_Worker:
      queue = worker_manager_get_task_context()->to_process_event_queue;
      break;
    case PebbleTask_App:
      queue = app_manager_get_task_context()->to_process_event_queue;
      break;
    case PebbleTask_KernelBackground:
      queue = NULL;
      break;
    default:
      WTF;
  }
  return queue;
}

void pebble_task_create(PebbleTask pebble_task, TaskParameters_t *task_params,
                        TaskHandle_t *handle) {
  MpuRegion app_region;
  MpuRegion worker_region;
  switch (pebble_task) {
    case PebbleTask_App:
      mpu_init_region_from_region(&app_region, memory_layout_get_app_region(),
                                  true /* allow_user_access */);
      mpu_init_region_from_region(&worker_region, memory_layout_get_worker_region(),
                                  false /* allow_user_access */);
      break;
    case PebbleTask_Worker:
      mpu_init_region_from_region(&app_region, memory_layout_get_app_region(),
                                  false /* allow_user_access */);
      mpu_init_region_from_region(&worker_region, memory_layout_get_worker_region(),
                                  true /* allow_user_access */);
      break;
    case PebbleTask_KernelMain:
    case PebbleTask_KernelBackground:
    case PebbleTask_BTHost:
    case PebbleTask_BTController:
    case PebbleTask_BTHCI:
    case PebbleTask_NewTimers:
    case PebbleTask_PULSE:
      mpu_init_region_from_region(&app_region, memory_layout_get_app_region(),
                                  false /* allow_user_access */);
      mpu_init_region_from_region(&worker_region, memory_layout_get_worker_region(),
                                  false /* allow_user_access */);
      break;
    default:
      WTF;
  }

  const MpuRegion *stack_guard_region = NULL;
#ifndef CONFIG_MPU_TYPE_ARMV8M
  // Per-task stack overflow detection: on ARMv7-M we plant a no-access
  // MPU region at the bottom of each task's stack. On ARMv8-M, the
  // FreeRTOS CM33 port sets PSPLIM to the same address via portSetupTCB(),
  // so the hardware stack-pointer-limit check catches overflows directly
  // -- and the ARMv8-M MPU AP encoding can't actually express "no access"
  // anyway (the closest is priv-RO, which is half a guard at best).
  // Skip programming the redundant MPU region and reclaim the slot.
  switch (pebble_task) {
    case PebbleTask_App:
      stack_guard_region = memory_layout_get_app_stack_guard_region();
      break;
    case PebbleTask_Worker:
      stack_guard_region = memory_layout_get_worker_stack_guard_region();
      break;
    case PebbleTask_KernelMain:
      stack_guard_region = memory_layout_get_kernel_main_stack_guard_region();
      break;
    case PebbleTask_KernelBackground:
      stack_guard_region = memory_layout_get_kernel_bg_stack_guard_region();
      break;
    case PebbleTask_BTHost:
    case PebbleTask_BTController:
    case PebbleTask_BTHCI:
    case PebbleTask_NewTimers:
    case PebbleTask_PULSE:
      break;
    default:
      WTF;
  }
#endif

  const MpuRegion *region_ptrs[portNUM_CONFIGURABLE_REGIONS] = {
    &app_region,
    &worker_region,
    stack_guard_region,
    NULL
  };
  mpu_set_task_configurable_regions(task_params->xRegions, region_ptrs);

  TaskHandle_t new_handle;
  PBL_ASSERT(xTaskCreateRestricted(task_params, &new_handle) == pdTRUE, "Could not start task %s",
             task_params->pcName);
  if (handle) {
    *handle = new_handle;
  }
  prv_task_register(pebble_task, new_handle);
}

void pebble_task_configure_idle_task(void) {
  // We don't have the opportunity to configure the IDLE task before FreeRTOS
  // creates it, so we have to configure the MPU regions properly after the
  // fact. This is only an issue on platforms with a cache, as altering the base
  // address, length or cacheability attributes of MPU regions (i.e. during
  // context switches) causes cache incoherency when data is read/written to the
  // memory covered by the regions before or after the change. This is
  // problematic from the IDLE task as ISRs inherit the MPU configuration of the
  // task that is currently running at the time.
  MpuRegion app_region;
  MpuRegion worker_region;
  mpu_init_region_from_region(&app_region, memory_layout_get_app_region(),
                              false /* allow_user_access */);
  mpu_init_region_from_region(&worker_region, memory_layout_get_worker_region(),
                              false /* allow_user_access */);
  const MpuRegion *region_ptrs[portNUM_CONFIGURABLE_REGIONS] = {
    &app_region,
    &worker_region,
    NULL,
    NULL
  };
  MemoryRegion_t region_config[portNUM_CONFIGURABLE_REGIONS] = {};
  mpu_set_task_configurable_regions(region_config, region_ptrs);
  vTaskAllocateMPURegions(xTaskGetIdleTaskHandle(), region_config);
}
