/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include <string.h>

#include "pbl/kernel/debug.h"
#include "pbl/kernel/irq.h"

#include "kernel_freertos.h"

_Static_assert((int)PBL_THREAD_REG_COUNT == (int)portCANONICAL_REG_COUNT, "register layout mismatch");

struct walk_ctx {
  pbl_thread_info_fn fn;
  void *ctx;
};

static void prv_walk_cb(const xPORT_TASK_INFO *const task_info, void *data) {
  const struct walk_ctx *w = data;
  struct pbl_thread_info info = {
    .thread = kfr_thread_from_handle(task_info->taskHandle),
    .name = task_info->pcName,
    .id = (uintptr_t)task_info->taskHandle,
    .current = task_info->taskHandle == xTaskGetCurrentTaskHandle(),
  };
  memcpy(info.regs, task_info->registers, sizeof(info.regs));
  w->fn(&info, w->ctx);
}

void pbl_thread_foreach(pbl_thread_info_fn fn, void *ctx) {
  struct walk_ctx w = { .fn = fn, .ctx = ctx };
  vTaskListWalk(prv_walk_cb, &w);
}

void pbl_thread_saved_regs(const struct pbl_thread *t, struct pbl_thread_saved_regs *regs) {
  TaskHandle_t h = kfr_handle(t);
  *regs = (struct pbl_thread_saved_regs){
    .pc = ulTaskDebugGetStackedPC(h),
    .lr = ulTaskDebugGetStackedLR(h),
    .control = ulTaskDebugGetStackedControl(h),
  };
}

void pbl_thread_stack_info(const struct pbl_thread *t, struct pbl_thread_stack_info *info) {
  TaskHandle_t h = kfr_handle(t);
  *info = (struct pbl_thread_stack_info){
    .start = ulTaskGetStackStart(h),
    .size = t->stack_size,
    .high_water = uxTaskGetStackHighWaterMark(h) * sizeof(StackType_t),
  };
}

static enum pbl_thread_state prv_state(eTaskState s) {
  switch (s) {
    case eRunning:
      return PBL_THREAD_RUNNING;
    case eReady:
      return PBL_THREAD_READY;
    case eBlocked:
      return PBL_THREAD_BLOCKED;
    case eSuspended:
      return PBL_THREAD_SUSPENDED;
    default:
      return PBL_THREAD_DEAD;
  }
}

size_t pbl_thread_stats_snapshot(struct pbl_thread_stats *out, size_t max,
                                 uint32_t *total_run_time) {
  static TaskStatus_t s_status[CONFIG_KERNEL_MAX_THREADS];

  vTaskSuspendAll();
  UBaseType_t count = uxTaskGetSystemState(s_status, CONFIG_KERNEL_MAX_THREADS, total_run_time);
  size_t n = 0;
  for (UBaseType_t i = 0; i < count && n < max; i++, n++) {
    out[n] = (struct pbl_thread_stats){
      .thread = kfr_thread_from_handle(s_status[i].xHandle),
      .name = s_status[i].pcTaskName,
      .number = s_status[i].xTaskNumber,
      .run_time = s_status[i].ulRunTimeCounter,
      .stack_high_water = s_status[i].usStackHighWaterMark * sizeof(StackType_t),
      .state = prv_state(s_status[i].eCurrentState),
    };
  }
  xTaskResumeAll();
  return n;
}

size_t pbl_thread_count(void) { return uxTaskGetNumberOfTasks(); }
