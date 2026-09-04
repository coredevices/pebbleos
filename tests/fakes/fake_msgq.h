/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "pbl/util/circular_buffer.h"
#include "pbl/util/list.h"

#include <pbl/kernel/msgq.h>

#include <errno.h>
#include <stdlib.h>
#include <string.h>

//! Message queue fake backed by a ring buffer. A yield callback stands in for
//! the other task while a put or get would block; it returns the ticks used.
typedef pbl_tick_t (*FakeMsgqYieldCallback)(struct pbl_msgq *q);

typedef struct FakeMsgq {
  ListNode node;
  struct pbl_msgq *q;
  FakeMsgqYieldCallback yield_cb;
  CircularBuffer ring;
  uint8_t *storage;
} FakeMsgq;

static FakeMsgq *s_fake_msgq_list;

static bool prv_fake_msgq_find(ListNode *node, void *context) {
  return ((FakeMsgq *)node)->q == context;
}

static FakeMsgq *prv_fake_msgq_get(struct pbl_msgq *q) {
  FakeMsgq *fake = (FakeMsgq *)list_find((ListNode *)s_fake_msgq_list, prv_fake_msgq_find, q);
  if (fake == NULL) {
    fake = malloc(sizeof(FakeMsgq));
    const size_t size = q->msg_size * q->max_msgs;
    *fake = (FakeMsgq) { .q = q, .storage = malloc(size) };
    circular_buffer_init(&fake->ring, fake->storage, size);
    s_fake_msgq_list = (FakeMsgq *)list_prepend((ListNode *)s_fake_msgq_list, (ListNode *)fake);
  }
  return fake;
}

void fake_msgq_reset(void) {
  ListNode *iter = (ListNode *)s_fake_msgq_list;
  while (iter) {
    ListNode *next = iter->next;
    free(((FakeMsgq *)iter)->storage);
    free(iter);
    iter = next;
  }
  s_fake_msgq_list = NULL;
}

void fake_msgq_set_yield_callback(struct pbl_msgq *q, FakeMsgqYieldCallback yield_cb) {
  prv_fake_msgq_get(q)->yield_cb = yield_cb;
}

void pbl_msgq_init(struct pbl_msgq *q, void *buf, size_t msg_size, uint32_t max_msgs) {
  *q = (struct pbl_msgq)PBL_MSGQ_INITIALIZER(buf, msg_size, max_msgs);
  FakeMsgq *fake = prv_fake_msgq_get(q);
  circular_buffer_init(&fake->ring, fake->storage, msg_size * max_msgs);
}

void pbl_msgq_deinit(struct pbl_msgq *q) {
  FakeMsgq *fake = prv_fake_msgq_get(q);
  list_remove((ListNode *)fake, (ListNode **)&s_fake_msgq_list, NULL);
  free(fake->storage);
  free(fake);
}

static bool prv_fake_msgq_wait(FakeMsgq *fake, pbl_timeout_t timeout, pbl_tick_t *waited) {
  if (pbl_timeout_is_no_wait(timeout) || !fake->yield_cb) {
    return false;
  }
  *waited += fake->yield_cb(fake->q);
  return pbl_timeout_is_forever(timeout) || *waited < timeout.ticks;
}

int pbl_msgq_put(struct pbl_msgq *q, const void *msg, pbl_timeout_t timeout) {
  FakeMsgq *fake = prv_fake_msgq_get(q);
  pbl_tick_t waited = 0;
  while (circular_buffer_get_write_space_remaining(&fake->ring) < q->msg_size) {
    if (!prv_fake_msgq_wait(fake, timeout, &waited)) {
      return pbl_timeout_is_no_wait(timeout) ? -EBUSY : -EAGAIN;
    }
  }
  circular_buffer_write(&fake->ring, msg, q->msg_size);
  return 0;
}

int pbl_msgq_put_front(struct pbl_msgq *q, const void *msg, pbl_timeout_t timeout) {
  return pbl_msgq_put(q, msg, timeout);
}

int pbl_msgq_get(struct pbl_msgq *q, void *msg, pbl_timeout_t timeout) {
  FakeMsgq *fake = prv_fake_msgq_get(q);
  pbl_tick_t waited = 0;
  while (circular_buffer_get_read_space_remaining(&fake->ring) < q->msg_size) {
    if (!prv_fake_msgq_wait(fake, timeout, &waited)) {
      return pbl_timeout_is_no_wait(timeout) ? -EBUSY : -EAGAIN;
    }
  }
  circular_buffer_copy(&fake->ring, msg, q->msg_size);
  circular_buffer_consume(&fake->ring, q->msg_size);
  return 0;
}

int pbl_msgq_peek(struct pbl_msgq *q, void *msg) {
  FakeMsgq *fake = prv_fake_msgq_get(q);
  if (circular_buffer_get_read_space_remaining(&fake->ring) < q->msg_size) {
    return -EBUSY;
  }
  circular_buffer_copy(&fake->ring, msg, q->msg_size);
  return 0;
}

void pbl_msgq_purge(struct pbl_msgq *q) {
  FakeMsgq *fake = prv_fake_msgq_get(q);
  circular_buffer_init(&fake->ring, fake->storage, q->msg_size * q->max_msgs);
}

uint32_t pbl_msgq_num_used(const struct pbl_msgq *q) {
  FakeMsgq *fake = prv_fake_msgq_get((struct pbl_msgq *)q);
  return circular_buffer_get_read_space_remaining(&fake->ring) / q->msg_size;
}

uint32_t pbl_msgq_num_free(const struct pbl_msgq *q) {
  return q->max_msgs - pbl_msgq_num_used(q);
}
