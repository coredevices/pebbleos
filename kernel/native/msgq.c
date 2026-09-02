/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include <string.h>

#include "pbl/kernel/msgq.h"
#include "pbl/kernel/poll.h"

#include "kernel.h"

void pbl_msgq_init(struct pbl_msgq *q, void *buf, size_t msg_size, uint32_t max_msgs) {
  KERNEL_ASSERT(buf != NULL && msg_size > 0 && max_msgs > 0);
  *q = (struct pbl_msgq)PBL_MSGQ_INITIALIZER(buf, msg_size, max_msgs);
}

void pbl_msgq_kobj_init(void *obj) {
  struct pbl_msgq *q = obj;
  pbl_msgq_init(q, q->buf, q->msg_size, q->max_msgs);
}

void pbl_msgq_deinit(struct pbl_msgq *q) { KERNEL_ASSERT(q->group == NULL); }

static uint8_t *prv_slot(struct pbl_msgq *q, uint32_t index) {
  return (uint8_t *)q->buf + index * q->msg_size;
}

static void prv_notify_getter(struct pbl_msgq *q) {
  struct pbl_thread *t = waitq_pop(&q->backend.getters);
  if (t) {
    sched_wake(t, 0);
    return;
  }
  if (q->group) {
    t = waitq_pop(&q->group->backend.waitq);
    if (t) {
      sched_wake(t, 0);
    }
  }
}

static int prv_put(struct pbl_msgq *q, const void *msg, pbl_timeout_t timeout, bool front) {
  int rc = 0;
  pbl_irq_lock();
  while (q->backend.count == q->max_msgs) {
    if (pbl_timeout_is_no_wait(timeout) || arch_in_isr()) {
      rc = -EBUSY;
      goto out;
    }
    rc = sched_block(&q->backend.putters, timeout);
    if (rc != 0) {
      goto out;
    }
  }
  if (front) {
    q->backend.head = (q->backend.head + q->max_msgs - 1) % q->max_msgs;
    memcpy(prv_slot(q, q->backend.head), msg, q->msg_size);
  } else {
    memcpy(prv_slot(q, q->backend.tail), msg, q->msg_size);
    q->backend.tail = (q->backend.tail + 1) % q->max_msgs;
  }
  q->backend.count++;
  prv_notify_getter(q);
out:
  pbl_irq_unlock();
  return rc;
}

int pbl_msgq_put(struct pbl_msgq *q, const void *msg, pbl_timeout_t timeout) {
  return prv_put(q, msg, timeout, false);
}

int pbl_msgq_put_front(struct pbl_msgq *q, const void *msg, pbl_timeout_t timeout) {
  return prv_put(q, msg, timeout, true);
}

int pbl_msgq_get(struct pbl_msgq *q, void *msg, pbl_timeout_t timeout) {
  int rc = 0;
  pbl_irq_lock();
  while (q->backend.count == 0) {
    if (pbl_timeout_is_no_wait(timeout) || arch_in_isr()) {
      rc = -EBUSY;
      goto out;
    }
    rc = sched_block(&q->backend.getters, timeout);
    if (rc != 0) {
      goto out;
    }
  }
  memcpy(msg, prv_slot(q, q->backend.head), q->msg_size);
  q->backend.head = (q->backend.head + 1) % q->max_msgs;
  q->backend.count--;
  struct pbl_thread *t = waitq_pop(&q->backend.putters);
  if (t) {
    sched_wake(t, 0);
  }
out:
  pbl_irq_unlock();
  return rc;
}

int pbl_msgq_peek(struct pbl_msgq *q, void *msg) {
  int rc = -EBUSY;
  pbl_irq_lock();
  if (q->backend.count > 0) {
    memcpy(msg, prv_slot(q, q->backend.head), q->msg_size);
    rc = 0;
  }
  pbl_irq_unlock();
  return rc;
}

void pbl_msgq_purge(struct pbl_msgq *q) {
  pbl_irq_lock();
  q->backend.head = q->backend.tail = q->backend.count = 0;
  struct pbl_thread *t;
  while ((t = waitq_pop(&q->backend.putters)) != NULL) {
    sched_wake(t, 0);
  }
  pbl_irq_unlock();
}

uint32_t pbl_msgq_num_used(const struct pbl_msgq *q) { return q->backend.count; }

uint32_t pbl_msgq_num_free(const struct pbl_msgq *q) { return q->max_msgs - q->backend.count; }
