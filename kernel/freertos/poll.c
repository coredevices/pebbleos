/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "pbl/kernel/irq.h"
#include "pbl/kernel/poll.h"

#include "kernel_freertos.h"
#include "queue.h"

void pbl_poll_group_init(struct pbl_poll_group *g) {
  *g = (struct pbl_poll_group)PBL_POLL_GROUP_INITIALIZER;
}

void pbl_poll_group_kobj_init(void *g) { pbl_poll_group_init(g); }

// A queue set holds one entry per pending message across all members, so it
// is rebuilt with the new capacity on every add. Members must be empty at
// that point, which FreeRTOS requires anyway.
static void prv_rebuild_set(struct pbl_poll_group *g) {
  QueueSetHandle_t old = g->backend.handle;
  if (old != NULL) {
    for (struct pbl_msgq *m = g->members; m != NULL; m = m->group_next) {
      if (m->backend.handle != NULL) {
        xQueueRemoveFromSet(m->backend.handle, old);
      }
    }
    vQueueDelete(old);
  }

  QueueSetHandle_t set = xQueueCreateSet(g->capacity);
  OS_ASSERT(set != NULL);
  for (struct pbl_msgq *m = g->members; m != NULL; m = m->group_next) {
    OS_ASSERT(xQueueAddToSet(m->backend.handle, set) == pdPASS);
  }
  g->backend.handle = set;
}

void pbl_poll_group_add(struct pbl_poll_group *g, struct pbl_msgq *q) {
  OS_ASSERT(q->group == NULL);
  OS_ASSERT(uxQueueMessagesWaiting(q->backend.handle) == 0);
  q->group = g;
  q->group_next = g->members;
  g->members = q;
  g->capacity += q->max_msgs;
  prv_rebuild_set(g);
}

struct pbl_msgq *pbl_poll_group_wait(struct pbl_poll_group *g, pbl_timeout_t timeout) {
  OS_ASSERT(!pbl_in_isr());
  OS_ASSERT(g->backend.handle != NULL);
  QueueSetMemberHandle_t ready = xQueueSelectFromSet(g->backend.handle, kfr_ticks(timeout));
  if (ready == NULL) {
    return NULL;
  }
  for (struct pbl_msgq *m = g->members; m != NULL; m = m->group_next) {
    if (m->backend.handle == ready) {
      return m;
    }
  }
  OS_ASSERT(false);
  return NULL;
}

bool pbl_poll_group_is_empty(const struct pbl_poll_group *g) {
  if (g->backend.handle == NULL) {
    return true;
  }
  return (pbl_in_isr() ? uxQueueMessagesWaitingFromISR(g->backend.handle)
                       : uxQueueMessagesWaiting(g->backend.handle)) == 0;
}
