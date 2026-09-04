/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include <string.h>

#include "pbl/kernel/idle.h"

#include "kernel.h"

#define NUM_PRIO CONFIG_KERNEL_NUM_PRIORITIES
_Static_assert(NUM_PRIO <= 32, "the ready bitmap is 32 bits wide");

struct pbl_thread *pbl_cur;
struct pbl_thread *pbl_all_threads;

static struct pbl_thread *s_ready_head[NUM_PRIO];
static struct pbl_thread *s_ready_tail[NUM_PRIO];
static uint32_t s_ready_mask;
static struct pbl_thread *s_timeouts;
static pbl_tick_t s_ticks;
static uint32_t s_sched_lock;
static bool s_switch_deferred;
static bool s_started;
static uint32_t s_next_number;

static struct pbl_thread s_idle_thread;
PBL_THREAD_STACK_DEFINE(s_idle_stack, CONFIG_KERNEL_IDLE_STACK_SIZE);

static inline bool prv_before(pbl_tick_t a, pbl_tick_t b) { return (int32_t)(a - b) < 0; }

// ---- ready lists ------------------------------------------------------------

static void prv_ready_push(struct pbl_thread *t) {
  pbl_prio_t p = t->prio;
  t->backend.next = NULL;
  if (s_ready_tail[p]) {
    s_ready_tail[p]->backend.next = t;
  } else {
    s_ready_head[p] = t;
    s_ready_mask |= 1u << p;
  }
  s_ready_tail[p] = t;
  t->backend.state = PBL_THREAD_READY;
}

static void prv_ready_remove(struct pbl_thread *t) {
  pbl_prio_t p = t->prio;
  struct pbl_thread **link = &s_ready_head[p];
  struct pbl_thread *prev = NULL;
  while (*link && *link != t) {
    prev = *link;
    link = &(*link)->backend.next;
  }
  if (*link == NULL) {
    return;
  }
  *link = t->backend.next;
  if (s_ready_tail[p] == t) {
    s_ready_tail[p] = prev;
  }
  if (s_ready_head[p] == NULL) {
    s_ready_mask &= ~(1u << p);
  }
  t->backend.next = NULL;
}

static struct pbl_thread *prv_pick(void) {
  KERNEL_ASSERT(s_ready_mask != 0);
  return s_ready_head[31 - __builtin_clz(s_ready_mask)];
}

static void prv_rotate(pbl_prio_t p) {
  struct pbl_thread *t = s_ready_head[p];
  if (t && t->backend.next) {
    prv_ready_remove(t);
    prv_ready_push(t);
  }
}

void sched_request_switch(void) {
  if (s_sched_lock) {
    s_switch_deferred = true;
    return;
  }
  if (s_started && prv_pick() != pbl_cur) {
    arch_switch_request();
  }
}

// ---- timeouts ---------------------------------------------------------------

static void prv_timeout_add(struct pbl_thread *t, pbl_tick_t ticks) {
  t->backend.wake_at = s_ticks + ticks;
  struct pbl_thread **link = &s_timeouts;
  while (*link && !prv_before(t->backend.wake_at, (*link)->backend.wake_at)) {
    link = &(*link)->backend.timeout_next;
  }
  t->backend.timeout_next = *link;
  *link = t;
  t->backend.on_timeout_list = true;
}

static void prv_timeout_remove(struct pbl_thread *t) {
  if (!t->backend.on_timeout_list) {
    return;
  }
  struct pbl_thread **link = &s_timeouts;
  while (*link && *link != t) {
    link = &(*link)->backend.timeout_next;
  }
  if (*link) {
    *link = t->backend.timeout_next;
  }
  t->backend.on_timeout_list = false;
}

static void prv_expire_timeouts(void) {
  while (s_timeouts && !prv_before(s_ticks, s_timeouts->backend.wake_at)) {
    struct pbl_thread *t = s_timeouts;
    s_timeouts = t->backend.timeout_next;
    t->backend.on_timeout_list = false;
    if (t->backend.waitq) {
      waitq_remove(t->backend.waitq, t);
      t->backend.waitq = NULL;
    }
    t->backend.wake_rc = -EAGAIN;
    prv_ready_push(t);
  }
}

static pbl_tick_t prv_ticks_until_next_timeout(void) {
  if (s_timeouts == NULL) {
    return PBL_TICK_FOREVER;
  }
  int32_t delta = (int32_t)(s_timeouts->backend.wake_at - s_ticks);
  return delta > 0 ? (pbl_tick_t)delta : 0;
}

// ---- wait queues ------------------------------------------------------------

void waitq_insert(struct pbl_waitq *wq, struct pbl_thread *t) {
  struct pbl_thread **link = &wq->head;
  while (*link && (*link)->prio >= t->prio) {
    link = &(*link)->backend.next;
  }
  t->backend.next = *link;
  *link = t;
}

void waitq_remove(struct pbl_waitq *wq, struct pbl_thread *t) {
  struct pbl_thread **link = &wq->head;
  while (*link && *link != t) {
    link = &(*link)->backend.next;
  }
  if (*link) {
    *link = t->backend.next;
    t->backend.next = NULL;
  }
}

struct pbl_thread *waitq_pop(struct pbl_waitq *wq) {
  struct pbl_thread *t = wq->head;
  if (t) {
    wq->head = t->backend.next;
    t->backend.next = NULL;
  }
  return t;
}

// ---- blocking and waking ----------------------------------------------------

int sched_block(struct pbl_waitq *wq, pbl_timeout_t timeout) {
  struct pbl_thread *t = pbl_cur;
  KERNEL_ASSERT(!arch_in_isr());
  KERNEL_ASSERT(s_sched_lock == 0);

  prv_ready_remove(t);
  t->backend.state = PBL_THREAD_BLOCKED;
  t->backend.waitq = wq;
  t->backend.wake_rc = 0;
  if (wq) {
    waitq_insert(wq, t);
  }
  if (!pbl_timeout_is_forever(timeout)) {
    prv_timeout_add(t, timeout.ticks);
  }
  arch_switch_request();

  // Drop the lock so the switch can happen; we come back here once woken.
  pbl_irq_unlock();
  pbl_irq_lock();

  return t->backend.wake_rc;
}

void sched_wake(struct pbl_thread *t, int rc) {
  KERNEL_ASSERT(t->backend.state == PBL_THREAD_BLOCKED);
  prv_timeout_remove(t);
  t->backend.waitq = NULL;
  t->backend.wake_rc = rc;
  prv_ready_push(t);
  if (t->prio > pbl_cur->prio) {
    sched_request_switch();
  }
}

// ---- priority inheritance ---------------------------------------------------

void sched_prio_set(struct pbl_thread *t, pbl_prio_t base, pbl_prio_t effective) {
  t->backend.base_prio = base;
  if (effective == t->prio) {
    return;
  }
  if (t->backend.state == PBL_THREAD_READY || t->backend.state == PBL_THREAD_RUNNING) {
    prv_ready_remove(t);
    t->prio = effective;
    prv_ready_push(t);
    if (t == pbl_cur) {
      t->backend.state = PBL_THREAD_RUNNING;
    }
  } else if (t->backend.state == PBL_THREAD_BLOCKED && t->backend.waitq) {
    waitq_remove(t->backend.waitq, t);
    t->prio = effective;
    waitq_insert(t->backend.waitq, t);
  } else {
    t->prio = effective;
  }
  sched_request_switch();
}

void sched_inherit(struct pbl_thread *owner, pbl_prio_t prio) {
  if (prio > owner->prio) {
    sched_prio_set(owner, owner->backend.base_prio, prio);
  }
}

void sched_disinherit(struct pbl_thread *owner) {
  if (owner->backend.mutexes_held == 0 && owner->prio != owner->backend.base_prio) {
    sched_prio_set(owner, owner->backend.base_prio, owner->backend.base_prio);
  }
}

// ---- thread lifecycle -------------------------------------------------------

void sched_thread_start(struct pbl_thread *t) {
  t->backend.number = ++s_next_number;
  t->backend.base_prio = t->prio;
  t->backend.all_next = pbl_all_threads;
  pbl_all_threads = t;
  t->backend.switched_in_at = s_ticks;
  prv_ready_push(t);
  sched_request_switch();
}

static void prv_detach(struct pbl_thread *t) {
  switch (t->backend.state) {
    case PBL_THREAD_READY:
    case PBL_THREAD_RUNNING:
      prv_ready_remove(t);
      break;
    case PBL_THREAD_BLOCKED:
      prv_timeout_remove(t);
      if (t->backend.waitq) {
        waitq_remove(t->backend.waitq, t);
        t->backend.waitq = NULL;
      }
      break;
    default:
      break;
  }
}

void sched_thread_remove(struct pbl_thread *t) {
  prv_detach(t);
  t->backend.state = PBL_THREAD_DEAD;
  struct pbl_thread **link = &pbl_all_threads;
  while (*link && *link != t) {
    link = &(*link)->backend.all_next;
  }
  if (*link) {
    *link = t->backend.all_next;
  }
  sched_request_switch();
}

void sched_thread_suspend(struct pbl_thread *t) {
  if (t->backend.state == PBL_THREAD_SUSPENDED || t->backend.state == PBL_THREAD_DEAD) {
    return;
  }
  // A thread suspended while blocked sees the call fail once resumed.
  bool was_blocked = t->backend.state == PBL_THREAD_BLOCKED;
  prv_detach(t);
  t->backend.state = PBL_THREAD_SUSPENDED;
  t->backend.wake_rc = was_blocked ? KWAKE_INTERRUPTED : 0;
  sched_request_switch();
}

void sched_thread_resume(struct pbl_thread *t) {
  if (t->backend.state != PBL_THREAD_SUSPENDED) {
    return;
  }
  prv_ready_push(t);
  sched_request_switch();
}

void sched_yield_current(void) {
  prv_rotate(pbl_cur->prio);
  sched_request_switch();
}

// ---- arch entry points ------------------------------------------------------

struct pbl_thread *sched_switch_in(void) {
  struct pbl_thread *prev = pbl_cur;
  struct pbl_thread *next = prv_pick();
  if (prev != next) {
    prev->backend.run_time += s_ticks - prev->backend.switched_in_at;
    if (prev->backend.state == PBL_THREAD_RUNNING) {
      prev->backend.state = PBL_THREAD_READY;
    }
    next->backend.switched_in_at = s_ticks;
    pbl_cur = next;
  }
  next->backend.state = PBL_THREAD_RUNNING;
  return next;
}

void sched_tick(void) {
  s_ticks++;
  prv_expire_timeouts();
  // Round-robin among threads sharing the running priority.
  if (s_ready_head[pbl_cur->prio] && s_ready_head[pbl_cur->prio]->backend.next) {
    prv_rotate(pbl_cur->prio);
  }
  sched_request_switch();
}

bool sched_idle_confirm(void) {
  return prv_pick() == &s_idle_thread && !s_switch_deferred;
}

void sched_idle_slept(pbl_tick_t elapsed) {
  s_ticks += elapsed;
  prv_expire_timeouts();
  sched_request_switch();
}

struct pbl_thread *sched_idle_thread(void) { return &s_idle_thread; }

static void prv_idle_entry(void *arg) {
  (void)arg;
  for (;;) {
    pbl_irq_lock();
    // Give other threads at the idle priority their turn first.
    if (s_ready_head[PBL_PRIO_IDLE]->backend.next) {
      sched_yield_current();
    }
    pbl_tick_t idle_ticks = prv_ticks_until_next_timeout();
    pbl_irq_unlock();
    arch_idle(idle_ticks);
  }
}

// ---- public scheduler API ---------------------------------------------------

void sched_start_prepare(void) {
  arch_init();

  struct pbl_thread_attr attr = {
    .name = "IDLE",
    .entry = prv_idle_entry,
    .prio = PBL_PRIO_IDLE,
    .privileged = true,
    .stack = s_idle_stack,
    .stack_size = sizeof(s_idle_stack),
  };
  int rc = pbl_thread_create(&s_idle_thread, &attr);
  KERNEL_ASSERT(rc == 0);

  pbl_irq_lock();
  s_started = true;
  pbl_cur = prv_pick();
  pbl_cur->backend.state = PBL_THREAD_RUNNING;
  pbl_cur->backend.switched_in_at = s_ticks;
}

void pbl_kernel_start(void) {
  sched_start_prepare();
  arch_start();
}

bool pbl_kernel_is_started(void) { return s_started; }

bool pbl_kernel_is_running(void) { return s_started && s_sched_lock == 0; }

void pbl_sched_lock(void) {
  pbl_irq_lock();
  s_sched_lock++;
  pbl_irq_unlock();
}

void pbl_sched_unlock(void) {
  pbl_irq_lock();
  KERNEL_ASSERT(s_sched_lock > 0);
  if (--s_sched_lock == 0 && s_switch_deferred) {
    s_switch_deferred = false;
    sched_request_switch();
  }
  pbl_irq_unlock();
}

bool pbl_sched_is_locked(void) { return s_sched_lock > 0; }

pbl_tick_t pbl_uptime_ticks(void) { return s_ticks; }

void pbl_thread_yield(void) {
  pbl_irq_lock();
  sched_yield_current();
  pbl_irq_unlock();
}

void pbl_thread_sleep(pbl_timeout_t timeout) {
  if (pbl_timeout_is_no_wait(timeout)) {
    pbl_thread_yield();
    return;
  }
  pbl_irq_lock();
  sched_block(NULL, timeout);
  pbl_irq_unlock();
}

// ---- idle interface ---------------------------------------------------------

bool pbl_idle_confirm(void) { return sched_idle_confirm(); }

void pbl_idle_slept(pbl_tick_t elapsed) {
  pbl_irq_lock();
  sched_idle_slept(elapsed);
  pbl_irq_unlock();
}

void pbl_kernel_tick_isr(void) {
  pbl_irq_lock();
  sched_tick();
  pbl_irq_unlock();
}

void sched_reset_for_test(void) {
  memset(&s_idle_thread, 0, sizeof(s_idle_thread));
  pbl_cur = NULL;
  pbl_all_threads = NULL;
  for (int i = 0; i < NUM_PRIO; i++) {
    s_ready_head[i] = s_ready_tail[i] = NULL;
  }
  s_ready_mask = 0;
  s_timeouts = NULL;
  s_ticks = 0;
  s_sched_lock = 0;
  s_switch_deferred = false;
  s_started = false;
  s_next_number = 0;
}
