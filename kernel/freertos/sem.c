/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include <errno.h>

#include "pbl/kernel/irq.h"
#include "pbl/kernel/sem.h"

#include "kernel_freertos.h"
#include "semphr.h"

void pbl_sem_init(struct pbl_sem *s, uint32_t initial, uint32_t limit) {
  OS_ASSERT(limit > 0 && initial <= limit);
  *s = (struct pbl_sem)PBL_SEM_INITIALIZER(initial, limit);
  s->backend.handle = xSemaphoreCreateCounting(limit, initial);
  OS_ASSERT(s->backend.handle != NULL);
}

void pbl_sem_kobj_init(void *s) {
  struct pbl_sem *sem = s;
  pbl_sem_init(sem, sem->initial, sem->limit);
}

void pbl_sem_deinit(struct pbl_sem *s) {
  vSemaphoreDelete(s->backend.handle);
  s->backend.handle = NULL;
}

int pbl_sem_take(struct pbl_sem *s, pbl_timeout_t timeout) {
  if (pbl_in_isr()) {
    OS_ASSERT(pbl_timeout_is_no_wait(timeout));
    BaseType_t woken = pdFALSE;
    if (xSemaphoreTakeFromISR(s->backend.handle, &woken) != pdTRUE) {
      return -EBUSY;
    }
    portEND_SWITCHING_ISR(woken);
    return 0;
  }
  if (xSemaphoreTake(s->backend.handle, kfr_ticks(timeout)) != pdTRUE) {
    return kfr_wait_error(timeout);
  }
  return 0;
}

void pbl_sem_give(struct pbl_sem *s) {
  if (pbl_in_isr()) {
    BaseType_t woken = pdFALSE;
    xSemaphoreGiveFromISR(s->backend.handle, &woken);
    portEND_SWITCHING_ISR(woken);
  } else {
    xSemaphoreGive(s->backend.handle);
  }
}

void pbl_sem_reset(struct pbl_sem *s) {
  OS_ASSERT(!pbl_in_isr());
  xQueueReset(s->backend.handle);
  for (uint32_t i = 0; i < s->initial; i++) {
    xSemaphoreGive(s->backend.handle);
  }
}

uint32_t pbl_sem_count(const struct pbl_sem *s) {
  return pbl_in_isr() ? uxQueueMessagesWaitingFromISR(s->backend.handle)
                      : uxQueueMessagesWaiting(s->backend.handle);
}
