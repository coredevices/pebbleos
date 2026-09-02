/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include <errno.h>

#include "pbl/kernel/irq.h"
#include "pbl/kernel/msgq.h"
#include "pbl/kernel/poll.h"

#include "kernel_freertos.h"
#include "queue.h"

void pbl_msgq_init(struct pbl_msgq *q, void *buf, size_t msg_size, uint32_t max_msgs) {
  OS_ASSERT(msg_size > 0 && max_msgs > 0);
  *q = (struct pbl_msgq)PBL_MSGQ_INITIALIZER(buf, msg_size, max_msgs);
  q->backend.handle = xQueueCreate(max_msgs, msg_size);
  OS_ASSERT(q->backend.handle != NULL);
}

void pbl_msgq_kobj_init(void *obj) {
  struct pbl_msgq *q = obj;
  pbl_msgq_init(q, q->buf, q->msg_size, q->max_msgs);
}

void pbl_msgq_deinit(struct pbl_msgq *q) {
  OS_ASSERT(q->group == NULL);
  vQueueDelete(q->backend.handle);
  q->backend.handle = NULL;
}

static int prv_send(struct pbl_msgq *q, const void *msg, pbl_timeout_t timeout,
                    BaseType_t position) {
  if (pbl_in_isr()) {
    OS_ASSERT(pbl_timeout_is_no_wait(timeout));
    BaseType_t woken = pdFALSE;
    if (xQueueGenericSendFromISR(q->backend.handle, msg, &woken, position) != pdTRUE) {
      return -EBUSY;
    }
    portEND_SWITCHING_ISR(woken);
    return 0;
  }
  if (xQueueGenericSend(q->backend.handle, msg, kfr_ticks(timeout), position) != pdTRUE) {
    return kfr_wait_error(timeout);
  }
  return 0;
}

int pbl_msgq_put(struct pbl_msgq *q, const void *msg, pbl_timeout_t timeout) {
  return prv_send(q, msg, timeout, queueSEND_TO_BACK);
}

int pbl_msgq_put_front(struct pbl_msgq *q, const void *msg, pbl_timeout_t timeout) {
  return prv_send(q, msg, timeout, queueSEND_TO_FRONT);
}

int pbl_msgq_get(struct pbl_msgq *q, void *msg, pbl_timeout_t timeout) {
  if (pbl_in_isr()) {
    OS_ASSERT(pbl_timeout_is_no_wait(timeout));
    BaseType_t woken = pdFALSE;
    if (xQueueReceiveFromISR(q->backend.handle, msg, &woken) != pdTRUE) {
      return -EBUSY;
    }
    portEND_SWITCHING_ISR(woken);
    return 0;
  }
  if (xQueueReceive(q->backend.handle, msg, kfr_ticks(timeout)) != pdTRUE) {
    return kfr_wait_error(timeout);
  }
  return 0;
}

int pbl_msgq_peek(struct pbl_msgq *q, void *msg) {
  BaseType_t rc = pbl_in_isr() ? xQueuePeekFromISR(q->backend.handle, msg)
                               : xQueuePeek(q->backend.handle, msg, 0);
  return rc == pdTRUE ? 0 : -EBUSY;
}

// Resetting a member of a queue set leaves stale entries in the set, so the
// set is rebuilt from the remaining members' counts (see queue.c).
void pbl_msgq_purge(struct pbl_msgq *q) {
  OS_ASSERT(!pbl_in_isr());
  struct pbl_poll_group *g = q->group;
  if (g == NULL || g->backend.handle == NULL) {
    xQueueReset(q->backend.handle);
    return;
  }

  vTaskSuspendAll();
  xQueueReset(q->backend.handle);
  xQueueReset(g->backend.handle);
  for (struct pbl_msgq *m = g->members; m != NULL; m = m->group_next) {
    QueueHandle_t h = m->backend.handle;
    uint32_t pending = uxQueueMessagesWaiting(h);
    for (uint32_t i = 0; i < pending; i++) {
      xQueueSend(g->backend.handle, &h, 0);
    }
  }
  xTaskResumeAll();
}

uint32_t pbl_msgq_num_used(const struct pbl_msgq *q) {
  return pbl_in_isr() ? uxQueueMessagesWaitingFromISR(q->backend.handle)
                      : uxQueueMessagesWaiting(q->backend.handle);
}

uint32_t pbl_msgq_num_free(const struct pbl_msgq *q) {
  return uxQueueSpacesAvailable(q->backend.handle);
}
