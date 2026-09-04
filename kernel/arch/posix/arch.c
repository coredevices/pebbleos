/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "pbl/kernel/idle.h"

#include "kernel.h"
#include "kernel_test.h"

// One pthread per kernel thread; the one holding s_cpu is the one the
// kernel considers running. Switches hand s_cpu over explicitly, so the
// scheduling is as deterministic as on the target.

static pthread_mutex_t s_cpu = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t s_stop_cond = PTHREAD_COND_INITIALIZER;
static bool s_switch_pending;
static bool s_in_isr;
static bool s_stopped;

// pthreads of threads that died without being joined yet.
static pthread_t s_zombies[64];
static size_t s_num_zombies;

static void prv_add_zombie(pthread_t tid) {
  if (s_num_zombies < sizeof(s_zombies) / sizeof(s_zombies[0])) {
    s_zombies[s_num_zombies++] = tid;
  }
}

// A cancelled thread leaves its condition wait owning s_cpu; give it back.
static void prv_release_cpu(void *arg) {
  (void)arg;
  pthread_mutex_unlock(&s_cpu);
}

static void *prv_thread_main(void *arg) {
  struct pbl_thread *t = arg;
  pthread_mutex_lock(&s_cpu);
  pthread_cleanup_push(prv_release_cpu, NULL);
  while (!t->backend.arch.run) {
    pthread_cond_wait(&t->backend.arch.wake, &s_cpu);
  }
  t->backend.arch.entry(t->backend.arch.arg);
  pbl_thread_abort(NULL);
  pthread_cleanup_pop(1);
  return NULL;
}

void arch_init(void) {}

void arch_thread_init(struct pbl_thread *t, void (*entry)(void *), void *arg) {
  pthread_cond_init(&t->backend.arch.wake, NULL);
  t->backend.arch.entry = entry;
  t->backend.arch.arg = arg;
  t->backend.arch.created = false;
  t->backend.arch.run = false;
}

void arch_thread_regions_set(struct pbl_thread *t, const MpuRegion *const *regions) {
  (void)t;
  (void)regions;
}

static void prv_run(struct pbl_thread *t) {
  t->backend.arch.run = true;
  if (!t->backend.arch.created) {
    t->backend.arch.created = true;
    pthread_create(&t->backend.arch.tid, NULL, prv_thread_main, t);
  } else {
    pthread_cond_signal(&t->backend.arch.wake);
  }
}

static void prv_switch(void) {
  s_switch_pending = false;
  struct pbl_thread *prev = pbl_cur;
  struct pbl_thread *next = sched_switch_in();
  if (next == prev) {
    return;
  }
  prv_run(next);
  if (prev->backend.state == PBL_THREAD_DEAD) {
    prv_add_zombie(prev->backend.arch.tid);
    pthread_mutex_unlock(&s_cpu);
    pthread_exit(NULL);
  }
  prev->backend.arch.run = false;
  while (!prev->backend.arch.run) {
    pthread_cond_wait(&prev->backend.arch.wake, &s_cpu);
  }
}

void arch_switch_request(void) { s_switch_pending = true; }

void arch_thread_exit(void) {
  // pbl_thread_abort() already switched away and ended this pthread.
  pthread_mutex_unlock(&s_cpu);
  pthread_exit(NULL);
}

void arch_thread_aborted(struct pbl_thread *t) {
  if (t->backend.arch.created) {
    prv_add_zombie(t->backend.arch.tid);
  }
}

bool arch_in_isr(void) { return s_in_isr; }

void arch_irq_disable(void) {}

void arch_irq_enable(void) {
  if (!s_in_isr && s_switch_pending && pbl_kernel_is_started()) {
    prv_switch();
  }
}

void arch_thread_saved_regs(const struct pbl_thread *t, struct pbl_thread_saved_regs *regs) {
  (void)t;
  *regs = (struct pbl_thread_saved_regs){ 0 };
}

void arch_thread_info_regs(const struct pbl_thread *t, uint32_t regs[PBL_THREAD_REG_COUNT]) {
  (void)t;
  memset(regs, 0, sizeof(uint32_t) * PBL_THREAD_REG_COUNT);
}

// Idle means nothing can run until the next timeout: jump the clock there.
void arch_idle(pbl_tick_t max_ticks) {
  if (s_stopped) {
    pthread_mutex_lock(&s_cpu);
    pthread_cond_signal(&s_stop_cond);
    pthread_mutex_unlock(&s_cpu);
    for (;;) {
      pause();
    }
  }
  if (max_ticks == PBL_TICK_FOREVER) {
    fprintf(stderr, "kernel test: every thread is blocked forever\n");
    abort();
  }
  pbl_irq_lock();
  sched_idle_slept(max_ticks);
  pbl_irq_unlock();
}

void arch_start(void) {
  irq_reset();
  pthread_mutex_lock(&s_cpu);
  prv_run(pbl_cur);
  while (!s_stopped) {
    pthread_cond_wait(&s_stop_cond, &s_cpu);
  }
  pthread_mutex_unlock(&s_cpu);
  for (struct pbl_thread *t = pbl_all_threads; t != NULL; t = t->backend.all_next) {
    if (t->backend.arch.created) {
      pthread_cancel(t->backend.arch.tid);
      pthread_join(t->backend.arch.tid, NULL);
    }
  }
  // Only the test harness gets here; it re-enters through pbl_test_kernel_run().
  pthread_exit(NULL);
}

// ---- test harness -----------------------------------------------------------

void pbl_test_kernel_run(void) {
  s_stopped = false;
  s_switch_pending = false;
  s_in_isr = false;
  sched_start_prepare();
  irq_reset();
  pthread_mutex_lock(&s_cpu);
  prv_run(pbl_cur);
  while (!s_stopped) {
    pthread_cond_wait(&s_stop_cond, &s_cpu);
  }
  pthread_mutex_unlock(&s_cpu);
  for (struct pbl_thread *t = pbl_all_threads; t != NULL; t = t->backend.all_next) {
    if (t->backend.arch.created) {
      pthread_cancel(t->backend.arch.tid);
      pthread_join(t->backend.arch.tid, NULL);
    }
  }
  for (size_t i = 0; i < s_num_zombies; i++) {
    pthread_cancel(s_zombies[i]);
    pthread_join(s_zombies[i], NULL);
  }
  s_num_zombies = 0;
  sched_reset_for_test();
}

void pbl_test_kernel_stop(void) {
  s_stopped = true;
  pthread_cond_signal(&s_stop_cond);
  // Park this thread; the harness tears it down.
  struct pbl_thread *me = pbl_cur;
  me->backend.arch.run = false;
  while (!me->backend.arch.run) {
    pthread_cond_wait(&me->backend.arch.wake, &s_cpu);
  }
}

void pbl_test_isr_enter(void) { s_in_isr = true; }

void pbl_test_isr_exit(void) {
  s_in_isr = false;
  if (s_switch_pending) {
    prv_switch();
  }
}

void pbl_test_tick(uint32_t ticks) {
  for (uint32_t i = 0; i < ticks; i++) {
    pbl_test_isr_enter();
    pbl_kernel_tick_isr();
    pbl_test_isr_exit();
  }
}
