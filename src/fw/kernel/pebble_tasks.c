/*
 * Copyright 2024 Google LLC
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "pebble_tasks.h"

#include "kernel/memory_layout.h"

#include "process_management/app_manager.h"
#include "process_management/worker_manager.h"
#include "services/common/analytics/analytics.h"
#include "services/common/analytics/analytics_metric.h"
#include "system/passert.h"
#include "util/size.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

TaskHandle_t g_task_handles[NumPebbleTask] KERNEL_READONLY_DATA = { 0 };

static void prv_task_register(PebbleTask task, TaskHandle_t task_handle) {
  g_task_handles[task] = task_handle;
}

void pebble_task_unregister(PebbleTask task) {
  g_task_handles[task] = NULL;
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

void analytics_external_collect_stack_free(void) {
  analytics_set(ANALYTICS_DEVICE_METRIC_STACK_FREE_KERNEL_MAIN,
    prv_task_get_stack_free(PebbleTask_KernelMain), AnalyticsClient_System);
  analytics_set(ANALYTICS_DEVICE_METRIC_STACK_FREE_KERNEL_BACKGROUND,
    prv_task_get_stack_free(PebbleTask_KernelBackground), AnalyticsClient_System);

  analytics_set(ANALYTICS_DEVICE_METRIC_STACK_FREE_BLUETOPIA_BIG,
    prv_task_get_stack_free(PebbleTask_BTHost), AnalyticsClient_System);
  analytics_set(ANALYTICS_DEVICE_METRIC_STACK_FREE_BLUETOPIA_MEDIUM,
    prv_task_get_stack_free(PebbleTask_BTController), AnalyticsClient_System);
  analytics_set(ANALYTICS_DEVICE_METRIC_STACK_FREE_BLUETOPIA_SMALL,
    prv_task_get_stack_free(PebbleTask_BTHCI), AnalyticsClient_System);

  analytics_set(ANALYTICS_DEVICE_METRIC_STACK_FREE_NEWTIMERS,
    prv_task_get_stack_free(PebbleTask_NewTimers), AnalyticsClient_System);
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

  const MpuRegion *region_ptrs[portNUM_CONFIGURABLE_REGIONS] = {
    // FIXME(SF32LB52): Not supported on ARMv8 MPU yet
#ifdef MICRO_FAMILY_SF32LB52
    &app_region,
    &worker_region,
    stack_guard_region,
    NULL
#endif
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
