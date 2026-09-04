/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "pbl/kernel/msgq.h"

//! Wait on several message queues at once. Members are added before the
//! first wait; only one thread may wait on a group.
struct pbl_poll_group {
  struct pbl_msgq *members;
  uint32_t capacity;
  struct pbl_poll_group_backend backend;
};

#define PBL_POLL_GROUP_INITIALIZER { .members = NULL, .capacity = 0 }

#define PBL_POLL_GROUP_DEFINE(name) struct pbl_poll_group name = PBL_POLL_GROUP_INITIALIZER

void pbl_poll_group_init(struct pbl_poll_group *g);

//! @p q must be empty and may belong to one group only.
void pbl_poll_group_add(struct pbl_poll_group *g, struct pbl_msgq *q);

//! @return a member with a pending message, or NULL on timeout.
struct pbl_msgq *pbl_poll_group_wait(struct pbl_poll_group *g, pbl_timeout_t timeout);

bool pbl_poll_group_is_empty(const struct pbl_poll_group *g);
