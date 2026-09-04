/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "pbl/kernel/poll.h"

#include "kernel.h"

void pbl_poll_group_init(struct pbl_poll_group *g) {
  *g = (struct pbl_poll_group)PBL_POLL_GROUP_INITIALIZER;
}

void pbl_poll_group_add(struct pbl_poll_group *g, struct pbl_msgq *q) {
  KERNEL_ASSERT(q->group == NULL);
  pbl_irq_lock();
  q->group = g;
  // Keep members in the order they were added.
  struct pbl_msgq **link = &g->members;
  while (*link) {
    link = &(*link)->group_next;
  }
  *link = q;
  g->capacity += q->max_msgs;
  pbl_irq_unlock();
}

// Scans the members starting after the one returned last time so a busy
// queue cannot starve the others.
static struct pbl_msgq *prv_ready_member(struct pbl_poll_group *g) {
  struct pbl_msgq *cursor = g->backend.cursor;
  for (struct pbl_msgq *m = cursor ? cursor->group_next : g->members; m != NULL; m = m->group_next) {
    if (m->backend.count > 0) {
      g->backend.cursor = m;
      return m;
    }
  }
  for (struct pbl_msgq *m = g->members; m != NULL; m = m->group_next) {
    if (m->backend.count > 0) {
      g->backend.cursor = m;
      return m;
    }
    if (m == cursor) {
      break;
    }
  }
  return NULL;
}

struct pbl_msgq *pbl_poll_group_wait(struct pbl_poll_group *g, pbl_timeout_t timeout) {
  KERNEL_ASSERT(!arch_in_isr() && g->members != NULL);
  struct pbl_msgq *ready;
  pbl_irq_lock();
  while ((ready = prv_ready_member(g)) == NULL) {
    if (pbl_timeout_is_no_wait(timeout) || sched_block(&g->backend.waitq, timeout) != 0) {
      break;
    }
  }
  pbl_irq_unlock();
  return ready;
}

bool pbl_poll_group_is_empty(const struct pbl_poll_group *g) {
  for (struct pbl_msgq *m = g->members; m != NULL; m = m->group_next) {
    if (m->backend.count > 0) {
      return false;
    }
  }
  return true;
}
