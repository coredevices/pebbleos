/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include <errno.h>

#include "pbl/kernel/debug.h"
#include "pbl/kernel/irq.h"
#include "pbl/kernel/sched.h"
#include "pbl/kernel/thread.h"
#include "pbl/os/assert.h"

#include "arch.h"

//! Internal interface between the objects, the scheduler and the arch code.
//! Everything here is called with interrupts locked unless noted.

extern struct pbl_thread *pbl_cur;

#define KERNEL_ASSERT(x) OS_ASSERT(x)

//! Woken by resume after being suspended while blocked.
#define KWAKE_INTERRUPTED (-EINTR)

void sched_init(void);
void sched_thread_start(struct pbl_thread *t);
void sched_thread_remove(struct pbl_thread *t);
void sched_thread_suspend(struct pbl_thread *t);
void sched_thread_resume(struct pbl_thread *t);
void sched_prio_set(struct pbl_thread *t, pbl_prio_t base, pbl_prio_t effective);
void sched_yield_current(void);
void sched_request_switch(void);

//! Blocks the current thread on @p wq (NULL for a plain sleep) until woken or
//! the timeout expires. Releases and re-takes the interrupt lock; the caller
//! must hold it exactly once. @return the wake code, -EAGAIN on timeout.
int sched_block(struct pbl_waitq *wq, pbl_timeout_t timeout);

//! Makes a blocked thread ready with @p rc as its wake code.
void sched_wake(struct pbl_thread *t, int rc);

void waitq_insert(struct pbl_waitq *wq, struct pbl_thread *t);
void waitq_remove(struct pbl_waitq *wq, struct pbl_thread *t);
struct pbl_thread *waitq_pop(struct pbl_waitq *wq);

//! Priority-inheritance helpers used by the mutex.
void sched_inherit(struct pbl_thread *owner, pbl_prio_t prio);
void sched_disinherit(struct pbl_thread *owner);

//! Resets the interrupt lock nesting when the first thread starts.
void irq_reset(void);

//! Creates the idle thread and picks the first thread to run.
void sched_start_prepare(void);

//! Arch entry points into the scheduler.
struct pbl_thread *sched_switch_in(void);  // picks and accounts the next thread
void sched_tick(void);
bool sched_idle_confirm(void);
void sched_idle_slept(pbl_tick_t elapsed);
struct pbl_thread *sched_idle_thread(void);
extern struct pbl_thread *pbl_all_threads;
