/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include <errno.h>
#include <string.h>

#include "pbl/kernel/thread.h"
#include "pbl/mcu/interrupts.h"

#include "kernel_freertos.h"

static struct pbl_thread s_idle = { .name = "IDLE", .privileged = true };
static uint32_t s_next_id;

struct pbl_thread *kfr_thread_from_handle(TaskHandle_t h) {
  if (h == NULL) {
    return NULL;
  }
  struct pbl_thread *t = pvTaskGetThreadLocalStoragePointer(h, KFR_TLS_THREAD);
  if (t == NULL && xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED &&
      h == xTaskGetIdleTaskHandle()) {
    t = pbl_thread_idle();
  }
  return t;
}

// The port expects ulParameters to hold the RASR/RLAR attributes and
// derives RBAR from pvBaseAddress: the full register on ARMv8-M, the
// aligned block base on ARMv7-M.
static void prv_fill_regions(MemoryRegion_t *out, const MpuRegion *const *regions) {
  _Static_assert(PBL_THREAD_MAX_MEM_REGIONS == portNUM_CONFIGURABLE_REGIONS,
                 "PBL_THREAD_MAX_MEM_REGIONS must match the FreeRTOS port");
  for (unsigned int i = 0; i < portNUM_CONFIGURABLE_REGIONS; i++) {
    const MpuRegion *r = regions ? regions[i] : NULL;
    if (r == NULL) {
      out[i] = (MemoryRegion_t){ 0 };
      continue;
    }
    OS_ASSERT(r->region_num == portFIRST_CONFIGURABLE_REGION + i);
    uint32_t base_reg, attr_reg;
    mpu_get_register_settings(r, &base_reg, &attr_reg);
#ifdef CONFIG_MPU_TYPE_ARMV8M
    uintptr_t base = base_reg;
#else
    uintptr_t base = base_reg & ~(MPU_RBAR_VALID_Msk | MPU_RBAR_REGION_Msk);
#endif
    out[i] = (MemoryRegion_t){
      .pvBaseAddress = (void *)base,
      .ulLengthInBytes = r->size,
      .ulParameters = attr_reg,
    };
  }
}

// FreeRTOS tasks must never return; let ours.
static void prv_entry(void *arg) {
  struct pbl_thread *t = arg;
  t->backend.entry(t->backend.arg);
  t->backend.dead = true;
  vTaskDelete(NULL);
}

int pbl_thread_create(struct pbl_thread *t, const struct pbl_thread_attr *attr) {
  OS_ASSERT(attr->stack != NULL && attr->stack_size != 0);
  OS_ASSERT(attr->prio <= PBL_PRIO_MAX);

  memset(t, 0, sizeof(*t));
  t->id = ++s_next_id;
  strncpy(t->name, attr->name, sizeof(t->name) - 1);
  t->prio = attr->prio;
  t->privileged = attr->privileged;
  t->stack = attr->stack;
  t->stack_size = attr->stack_size;
  t->backend.entry = attr->entry;
  t->backend.arg = attr->arg;

  TaskParameters_t params = {
    .pvTaskCode = prv_entry,
    .pcName = t->name,
    .usStackDepth = attr->stack_size / sizeof(StackType_t),
    .pvParameters = t,
    .uxPriority = attr->prio | (attr->privileged ? portPRIVILEGE_BIT : 0),
    .puxStackBuffer = attr->stack,
  };
  prv_fill_regions(params.xRegions, attr->regions);

  // Hold the scheduler so the thread cannot run before its back-pointer is set.
  vTaskSuspendAll();
  TaskHandle_t handle = NULL;
  BaseType_t rc = xTaskCreateRestricted(&params, &handle);
  if (rc == pdPASS) {
    t->backend.handle = handle;
    vTaskSetThreadLocalStoragePointer(handle, KFR_TLS_THREAD, t);
  }
  xTaskResumeAll();

  return rc == pdPASS ? 0 : -ENOMEM;
}

void pbl_thread_abort(struct pbl_thread *t) {
  if (t == NULL) {
    t = pbl_thread_current();
  }
  if (t) {
    t->backend.dead = true;
  }
  vTaskDelete(kfr_handle(t));
}

void pbl_thread_suspend(struct pbl_thread *t) { vTaskSuspend(kfr_handle(t)); }

void pbl_thread_resume(struct pbl_thread *t) { vTaskResume(kfr_handle(t)); }

void pbl_thread_yield(void) { taskYIELD(); }

void pbl_thread_sleep(pbl_timeout_t timeout) { vTaskDelay(kfr_ticks(timeout)); }

struct pbl_thread *pbl_thread_current(void) {
  return kfr_thread_from_handle(xTaskGetCurrentTaskHandle());
}

struct pbl_thread *pbl_thread_idle(void) {
  if (s_idle.backend.handle == NULL) {
    TaskHandle_t h = xTaskGetIdleTaskHandle();
    OS_ASSERT(h != NULL);
    s_idle.backend.handle = h;
    vTaskSetThreadLocalStoragePointer(h, KFR_TLS_THREAD, &s_idle);
  }
  return &s_idle;
}

void pbl_thread_prio_set(struct pbl_thread *t, pbl_prio_t prio) {
  OS_ASSERT(prio <= PBL_PRIO_MAX);
  t->prio = prio;
  vTaskPrioritySet(kfr_handle(t), prio);
}

pbl_prio_t pbl_thread_prio_get(const struct pbl_thread *t) {
  return uxTaskPriorityGet(kfr_handle(t)) & ~portPRIVILEGE_BIT;
}

enum pbl_thread_state pbl_thread_state(const struct pbl_thread *t) {
  if (t->backend.dead) {
    return PBL_THREAD_DEAD;
  }
  switch (eTaskGetState(kfr_handle(t))) {
    case eRunning:
      return PBL_THREAD_RUNNING;
    case eReady:
      return PBL_THREAD_READY;
    case eBlocked:
      return PBL_THREAD_BLOCKED;
    case eSuspended:
      return PBL_THREAD_SUSPENDED;
    case eDeleted:
    default:
      return PBL_THREAD_DEAD;
  }
}

void pbl_thread_regions_set(struct pbl_thread *t, const MpuRegion *const *regions) {
  MemoryRegion_t mem[portNUM_CONFIGURABLE_REGIONS];
  prv_fill_regions(mem, regions);
  vTaskAllocateMPURegions(kfr_handle(t), mem);
}
