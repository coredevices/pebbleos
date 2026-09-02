/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "pbl/kernel/types.h"

struct pbl_poll_group;

//! Fixed-size message ring; put/get copy @c msg_size bytes. Usable from ISRs with PBL_NO_WAIT.
struct pbl_msgq {
  void *buf;  // msg_size * max_msgs bytes; unused by backends that keep their own storage
  size_t msg_size;
  uint32_t max_msgs;
  struct pbl_poll_group *group;
  struct pbl_msgq *group_next;
  struct pbl_msgq_backend backend;
};

#define PBL_MSGQ_INITIALIZER(buffer, size, max) \
  { .buf = (buffer), .msg_size = (size), .max_msgs = (max) }

//! A file-scope compound literal has static storage duration, so the buffer
//! needs no name and the definition can be prefixed with static.
#if PBL_KERNEL_MSGQ_NEEDS_BUF
#define PBL_MSGQ_STATIC_BUF(size, max) ((uint32_t[((size) * (max) + 3) / 4]){0})
#else
#define PBL_MSGQ_STATIC_BUF(size, max) NULL
#endif

#define PBL_MSGQ_DEFINE(name, size, max)                                                 \
  struct pbl_msgq name = PBL_MSGQ_INITIALIZER(PBL_MSGQ_STATIC_BUF(size, max), size, max); \
  PBL_KOBJ_REGISTER(name, pbl_msgq_kobj_init)

//! @p buf may be NULL when PBL_KERNEL_MSGQ_NEEDS_BUF is 0.
void pbl_msgq_init(struct pbl_msgq *q, void *buf, size_t msg_size, uint32_t max_msgs);
void pbl_msgq_kobj_init(void *def);

//! Required before the memory of a dynamically allocated queue is reused. Not for group members.
void pbl_msgq_deinit(struct pbl_msgq *q);

//! @return 0, -EAGAIN on timeout, -EBUSY with PBL_NO_WAIT.
int pbl_msgq_put(struct pbl_msgq *q, const void *msg, pbl_timeout_t timeout);
int pbl_msgq_put_front(struct pbl_msgq *q, const void *msg, pbl_timeout_t timeout);
int pbl_msgq_get(struct pbl_msgq *q, void *msg, pbl_timeout_t timeout);
int pbl_msgq_peek(struct pbl_msgq *q, void *msg);
void pbl_msgq_purge(struct pbl_msgq *q);
uint32_t pbl_msgq_num_used(const struct pbl_msgq *q);
uint32_t pbl_msgq_num_free(const struct pbl_msgq *q);
