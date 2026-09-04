/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "clar.h"

#include "pbl/kernel/kernel.h"
#include "pbl/os/assert.h"

#include "kernel_test.h"

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// The kernel on the POSIX arch: threads are pthreads that run one at
// a time under the kernel's scheduling decisions, and time only moves when a
// test delivers ticks or every thread is blocked.

NORETURN os_assertion_failed(const char *filename, int line) {
  fprintf(stderr, "kernel assert at %s:%d\n", filename, line);
  abort();
}

NORETURN os_assertion_failed_lr(const char *filename, int line, uint32_t lr) {
  os_assertion_failed(filename, line);
}

#define STACK 4096
static uint8_t s_stacks[8][STACK] __attribute__((aligned(8)));
static struct pbl_thread s_threads[8];

static char s_trace[128];

static void prv_trace(char c) {
  size_t n = strlen(s_trace);
  if (n + 1 < sizeof(s_trace)) {
    s_trace[n] = c;
    s_trace[n + 1] = '\0';
  }
}

static struct pbl_thread *prv_spawn(int i, const char *name, pbl_prio_t prio,
                                    void (*entry)(void *), void *arg) {
  struct pbl_thread_attr attr = {
    .name = name,
    .entry = entry,
    .arg = arg,
    .prio = prio,
    .privileged = true,
    .stack = s_stacks[i],
    .stack_size = STACK,
  };
  cl_assert_equal_i(pbl_thread_create(&s_threads[i], &attr), 0);
  return &s_threads[i];
}

void test_kernel__initialize(void) {
  s_trace[0] = '\0';
  memset(s_threads, 0, sizeof(s_threads));
}

void test_kernel__cleanup(void) {
}

// ---- scheduling -------------------------------------------------------------

static void prv_lo_entry(void *arg) {
  prv_trace('L');
  pbl_test_kernel_stop();
}

static void prv_hi_entry(void *arg) {
  prv_trace('H');
  pbl_thread_sleep(PBL_TICKS(5));
  prv_trace('h');
}

void test_kernel__priority_preempts(void) {
  prv_spawn(0, "lo", 1, prv_lo_entry, NULL);
  prv_spawn(1, "hi", 3, prv_hi_entry, NULL);
  pbl_test_kernel_run();
  // hi runs first, sleeps, lo runs to the stop while hi is asleep
  cl_assert_equal_s(s_trace, "HL");
}

static void prv_sleep_then_stop(void *arg) {
  pbl_tick_t start = pbl_uptime_ticks();
  pbl_thread_sleep(PBL_TICKS(10));
  cl_assert_equal_i(pbl_uptime_ticks() - start, 10);
  pbl_thread_sleep(PBL_MSEC(0));  // yield only
  prv_trace('S');
  pbl_test_kernel_stop();
}

void test_kernel__sleep_advances_virtual_time(void) {
  prv_spawn(0, "sleeper", 2, prv_sleep_then_stop, NULL);
  pbl_test_kernel_run();
  cl_assert_equal_s(s_trace, "S");
}

static void prv_rr_entry(void *arg) {
  char c = (char)(uintptr_t)arg;
  for (int i = 0; i < 3; i++) {
    prv_trace(c);
    pbl_thread_yield();
  }
  if (c == 'B') {
    pbl_test_kernel_stop();
  }
}

void test_kernel__yield_round_robins_equal_priority(void) {
  prv_spawn(0, "a", 2, prv_rr_entry, (void *)'A');
  prv_spawn(1, "b", 2, prv_rr_entry, (void *)'B');
  pbl_test_kernel_run();
  cl_assert_equal_s(s_trace, "ABABAB");
}

static void prv_spin_entry(void *arg) {
  char c = (char)(uintptr_t)arg;
  for (int i = 0; i < 4; i++) {
    prv_trace(c);
    // Deliver a tick "from an ISR" while running: time slicing rotates us out.
    pbl_test_tick(1);
  }
  if (c == 'B') {
    pbl_test_kernel_stop();
  }
}

void test_kernel__tick_time_slices(void) {
  prv_spawn(0, "a", 2, prv_spin_entry, (void *)'A');
  prv_spawn(1, "b", 2, prv_spin_entry, (void *)'B');
  pbl_test_kernel_run();
  cl_assert_equal_s(s_trace, "ABABABAB");
}

// ---- semaphores -------------------------------------------------------------

static PBL_SEM_DEFINE(s_sem, 0, 1);

static void prv_taker(void *arg) {
  prv_trace('t');
  cl_assert_equal_i(pbl_sem_take(&s_sem, PBL_FOREVER), 0);
  prv_trace('T');
  cl_assert_equal_i(pbl_sem_take(&s_sem, PBL_TICKS(3)), -EAGAIN);
  prv_trace('x');
  cl_assert_equal_i(pbl_sem_take(&s_sem, PBL_NO_WAIT), -EBUSY);
  pbl_test_kernel_stop();
}

static void prv_giver(void *arg) {
  prv_trace('g');
  pbl_sem_give(&s_sem);
  prv_trace('G');
}

void test_kernel__semaphore_handoff_and_timeout(void) {
  pbl_sem_init(&s_sem, s_sem.initial, s_sem.limit);
  prv_spawn(0, "taker", 3, prv_taker, NULL);
  prv_spawn(1, "giver", 2, prv_giver, NULL);
  pbl_test_kernel_run();
  // taker blocks, giver gives and is preempted by the higher-priority taker,
  // which then times out on its second take after the giver finished
  cl_assert_equal_s(s_trace, "tgTGx");
}

static void prv_isr_taker(void *arg) {
  cl_assert_equal_i(pbl_sem_take(&s_sem, PBL_FOREVER), 0);
  prv_trace('T');
  pbl_test_kernel_stop();
}

static void prv_isr_giver(void *arg) {
  prv_trace('i');
  pbl_test_isr_enter();
  pbl_sem_give(&s_sem);
  prv_trace('g');
  pbl_test_isr_exit();  // the higher-priority taker runs on ISR exit
  prv_trace('r');
}

void test_kernel__semaphore_give_from_isr(void) {
  pbl_sem_init(&s_sem, s_sem.initial, s_sem.limit);
  prv_spawn(0, "taker", 3, prv_isr_taker, NULL);
  prv_spawn(1, "giver", 2, prv_isr_giver, NULL);
  pbl_test_kernel_run();
  cl_assert_equal_s(s_trace, "igT");
}

void test_kernel__semaphore_counts_to_limit(void) {
  struct pbl_sem s;
  pbl_sem_init(&s, 0, 2);
  pbl_sem_give(&s);
  pbl_sem_give(&s);
  pbl_sem_give(&s);
  cl_assert_equal_i(pbl_sem_count(&s), 2);
  pbl_sem_reset(&s);
  cl_assert_equal_i(pbl_sem_count(&s), 0);
}

// ---- mutexes ----------------------------------------------------------------

static PBL_MUTEX_DEFINE(s_mutex);

static void prv_pi_low(void *arg) {
  cl_assert_equal_i(pbl_mutex_lock(&s_mutex, PBL_FOREVER), 0);
  prv_trace('l');
  // Stay busy through a tick: high and medium wake, high blocks on the
  // mutex and lends us its priority, so medium must not run yet.
  pbl_test_tick(1);
  cl_assert_equal_i(pbl_thread_current()->prio, 4);
  prv_trace('L');
  pbl_mutex_unlock(&s_mutex);
  cl_assert_equal_i(pbl_thread_current()->prio, 1);
  prv_trace('d');
}

static void prv_pi_medium(void *arg) {
  pbl_thread_sleep(PBL_TICKS(1));
  prv_trace('M');
}

static void prv_pi_high(void *arg) {
  pbl_thread_sleep(PBL_TICKS(1));
  prv_trace('h');
  cl_assert_equal_i(pbl_mutex_lock(&s_mutex, PBL_FOREVER), 0);
  cl_assert(pbl_mutex_is_owner(&s_mutex));
  prv_trace('H');
  pbl_mutex_unlock(&s_mutex);
  pbl_thread_sleep(PBL_TICKS(5));
  pbl_test_kernel_stop();
}

void test_kernel__mutex_priority_inheritance(void) {
  pbl_mutex_init(&s_mutex);
  prv_spawn(0, "low", 1, prv_pi_low, NULL);
  prv_spawn(1, "med", 2, prv_pi_medium, NULL);
  prv_spawn(2, "high", 4, prv_pi_high, NULL);
  pbl_test_kernel_run();
  // low takes the lock; high blocks and boosts low over medium; low hands
  // the mutex to high, then medium outranks the deboosted low
  cl_assert_equal_s(s_trace, "lhLHMd");
}

static void prv_recursive(void *arg) {
  cl_assert_equal_i(pbl_mutex_lock(&s_mutex, PBL_FOREVER), 0);
  cl_assert_equal_i(pbl_mutex_lock(&s_mutex, PBL_FOREVER), 0);
  cl_assert_equal_i(s_mutex.count, 2);
  pbl_mutex_unlock(&s_mutex);
  cl_assert(pbl_mutex_is_owner(&s_mutex));
  pbl_mutex_unlock(&s_mutex);
  cl_assert(!pbl_mutex_is_owner(&s_mutex));
  pbl_test_kernel_stop();
}

void test_kernel__mutex_recursive(void) {
  pbl_mutex_init(&s_mutex);
  prv_spawn(0, "r", 2, prv_recursive, NULL);
  pbl_test_kernel_run();
}

static void prv_holder(void *arg) {
  pbl_mutex_lock(&s_mutex, PBL_FOREVER);
  pbl_thread_sleep(PBL_TICKS(10));
  pbl_mutex_unlock(&s_mutex);
}

static void prv_timed_locker(void *arg) {
  pbl_thread_sleep(PBL_TICKS(1));
  cl_assert_equal_i(pbl_mutex_lock(&s_mutex, PBL_NO_WAIT), -EBUSY);
  cl_assert_equal_i(pbl_mutex_lock(&s_mutex, PBL_TICKS(3)), -EAGAIN);
  cl_assert_equal_i(pbl_mutex_lock(&s_mutex, PBL_FOREVER), 0);
  cl_assert(pbl_uptime_ticks() >= 10);
  pbl_mutex_unlock(&s_mutex);
  pbl_test_kernel_stop();
}

void test_kernel__mutex_timeouts(void) {
  pbl_mutex_init(&s_mutex);
  prv_spawn(0, "holder", 2, prv_holder, NULL);
  prv_spawn(1, "locker", 3, prv_timed_locker, NULL);
  pbl_test_kernel_run();
}

// ---- message queues ---------------------------------------------------------

static PBL_MSGQ_DEFINE(s_q, sizeof(int), 2);

static void prv_producer(void *arg) {
  for (int i = 1; i <= 4; i++) {
    cl_assert_equal_i(pbl_msgq_put(&s_q, &i, PBL_FOREVER), 0);
    prv_trace('0' + i);
  }
}

static void prv_consumer(void *arg) {
  pbl_thread_sleep(PBL_TICKS(1));  // let the producer fill the queue and block
  cl_assert_equal_i(pbl_msgq_num_used(&s_q), 2);
  for (int i = 1; i <= 4; i++) {
    int v;
    cl_assert_equal_i(pbl_msgq_get(&s_q, &v, PBL_FOREVER), 0);
    cl_assert_equal_i(v, i);
    prv_trace('a' + i - 1);
  }
  int v;
  cl_assert_equal_i(pbl_msgq_get(&s_q, &v, PBL_TICKS(2)), -EAGAIN);
  cl_assert_equal_i(pbl_msgq_get(&s_q, &v, PBL_NO_WAIT), -EBUSY);
  pbl_test_kernel_stop();
}

void test_kernel__msgq_blocks_full_and_empty(void) {
  pbl_msgq_init(&s_q, s_q.buf, s_q.msg_size, s_q.max_msgs);
  prv_spawn(0, "producer", 2, prv_producer, NULL);
  prv_spawn(1, "consumer", 3, prv_consumer, NULL);
  pbl_test_kernel_run();
  // producer puts 1,2 then blocks; every put wakes the higher-priority
  // consumer, which drains the message before the producer can trace it
  cl_assert_equal_s(s_trace, "12abc3d4");
}

void test_kernel__msgq_front_peek_purge(void) {
  int buf[3];
  struct pbl_msgq q;
  pbl_msgq_init(&q, buf, sizeof(int), 3);
  int v = 1;
  cl_assert_equal_i(pbl_msgq_put(&q, &v, PBL_NO_WAIT), 0);
  v = 2;
  cl_assert_equal_i(pbl_msgq_put_front(&q, &v, PBL_NO_WAIT), 0);
  cl_assert_equal_i(pbl_msgq_peek(&q, &v), 0);
  cl_assert_equal_i(v, 2);
  cl_assert_equal_i(pbl_msgq_num_free(&q), 1);
  pbl_msgq_purge(&q);
  cl_assert_equal_i(pbl_msgq_num_used(&q), 0);
  cl_assert_equal_i(pbl_msgq_peek(&q, &v), -EBUSY);
}

// ---- poll groups ------------------------------------------------------------

static PBL_MSGQ_DEFINE(s_qa, sizeof(int), 4);
static PBL_MSGQ_DEFINE(s_qb, sizeof(int), 4);
static PBL_POLL_GROUP_DEFINE(s_group);

static void prv_poll_waiter(void *arg) {
  cl_assert(pbl_poll_group_is_empty(&s_group));
  cl_assert(pbl_poll_group_wait(&s_group, PBL_TICKS(2)) == NULL);
  struct pbl_msgq *ready = pbl_poll_group_wait(&s_group, PBL_FOREVER);
  cl_assert(ready == &s_qb);
  int v;
  cl_assert_equal_i(pbl_msgq_get(ready, &v, PBL_NO_WAIT), 0);
  cl_assert_equal_i(v, 7);
  // both queues loaded: the scan rotates so neither starves
  v = 1;
  pbl_msgq_put(&s_qa, &v, PBL_NO_WAIT);
  pbl_msgq_put(&s_qb, &v, PBL_NO_WAIT);
  cl_assert(pbl_poll_group_wait(&s_group, PBL_NO_WAIT) == &s_qa);
  pbl_msgq_get(&s_qa, &v, PBL_NO_WAIT);
  cl_assert(pbl_poll_group_wait(&s_group, PBL_NO_WAIT) == &s_qb);
  pbl_msgq_get(&s_qb, &v, PBL_NO_WAIT);
  cl_assert(pbl_poll_group_wait(&s_group, PBL_NO_WAIT) == NULL);
  pbl_test_kernel_stop();
}

static void prv_poll_poster(void *arg) {
  pbl_thread_sleep(PBL_TICKS(5));
  int v = 7;
  pbl_msgq_put(&s_qb, &v, PBL_NO_WAIT);
}

void test_kernel__poll_group(void) {
  pbl_msgq_init(&s_qa, s_qa.buf, s_qa.msg_size, s_qa.max_msgs);
  pbl_msgq_init(&s_qb, s_qb.buf, s_qb.msg_size, s_qb.max_msgs);
  pbl_poll_group_init(&s_group);
  pbl_poll_group_add(&s_group, &s_qa);
  pbl_poll_group_add(&s_group, &s_qb);
  prv_spawn(0, "waiter", 3, prv_poll_waiter, NULL);
  prv_spawn(1, "poster", 2, prv_poll_poster, NULL);
  pbl_test_kernel_run();
}

// ---- thread lifecycle -------------------------------------------------------

static void prv_victim(void *arg) {
  for (;;) {
    prv_trace('v');
    pbl_thread_sleep(PBL_TICKS(1));
  }
}

static void prv_controller(void *arg) {
  struct pbl_thread *victim = arg;
  pbl_thread_sleep(PBL_TICKS(2));
  pbl_thread_suspend(victim);
  cl_assert_equal_i(pbl_thread_state(victim), PBL_THREAD_SUSPENDED);
  size_t before = strlen(s_trace);
  pbl_thread_sleep(PBL_TICKS(5));
  cl_assert_equal_i(strlen(s_trace), before);
  pbl_thread_resume(victim);
  pbl_thread_sleep(PBL_TICKS(3));
  cl_assert(strlen(s_trace) > before);
  pbl_thread_abort(victim);
  cl_assert_equal_i(pbl_thread_state(victim), PBL_THREAD_DEAD);
  cl_assert_equal_i(pbl_thread_count(), 2);  // idle + this one
  pbl_test_kernel_stop();
}

void test_kernel__suspend_resume_abort(void) {
  struct pbl_thread *victim = prv_spawn(0, "victim", 2, prv_victim, NULL);
  prv_spawn(1, "ctl", 3, prv_controller, victim);
  pbl_test_kernel_run();
}

static void prv_short_lived(void *arg) {
  prv_trace('s');
  // returning from the entry ends the thread
}

static void prv_watcher(void *arg) {
  pbl_thread_sleep(PBL_TICKS(1));
  cl_assert_equal_i(pbl_thread_state(&s_threads[0]), PBL_THREAD_DEAD);
  prv_trace('w');
  pbl_test_kernel_stop();
}

void test_kernel__thread_exit_on_return(void) {
  prv_spawn(0, "short", 3, prv_short_lived, NULL);
  prv_spawn(1, "watch", 2, prv_watcher, NULL);
  pbl_test_kernel_run();
  cl_assert_equal_s(s_trace, "sw");
}

static void prv_stats_entry(void *arg) {
  pbl_thread_sleep(PBL_TICKS(3));
  struct pbl_thread_stats stats[4];
  uint32_t total;
  size_t n = pbl_thread_stats_snapshot(stats, 4, &total);
  cl_assert_equal_i(n, 2);
  cl_assert_equal_i(total, 3);
  struct pbl_thread_stack_info info;
  pbl_thread_stack_info(pbl_thread_current(), &info);
  // On the POSIX arch the thread runs on a pthread stack, so the kernel
  // stack is untouched and the high water is the whole buffer.
  cl_assert(info.high_water > 0 && info.high_water <= STACK);
  cl_assert_equal_i(info.start, (uintptr_t)s_stacks[0]);
  pbl_test_kernel_stop();
}

void test_kernel__stats_and_stack_info(void) {
  prv_spawn(0, "stats", 2, prv_stats_entry, NULL);
  pbl_test_kernel_run();
}

static void prv_prio_raiser(void *arg) {
  struct pbl_thread *other = arg;
  prv_trace('a');
  pbl_thread_prio_set(other, 4);  // other now outranks us and runs at once
  prv_trace('b');
  pbl_test_kernel_stop();
}

static void prv_raised(void *arg) {
  prv_trace('R');
  cl_assert_equal_i(pbl_thread_prio_get(pbl_thread_current()), 4);
}

void test_kernel__prio_set_preempts(void) {
  struct pbl_thread *other = prv_spawn(0, "other", 1, prv_raised, NULL);
  prv_spawn(1, "raiser", 3, prv_prio_raiser, other);
  pbl_test_kernel_run();
  cl_assert_equal_s(s_trace, "aRb");
}

static void prv_sched_locked(void *arg) {
  prv_trace('a');
  pbl_sched_lock();
  pbl_sem_give(&s_sem);  // would wake the higher-priority waiter
  prv_trace('b');
  cl_assert(!pbl_kernel_is_running() && pbl_sched_is_locked());
  pbl_sched_unlock();  // the switch happens here
  prv_trace('c');
  pbl_test_kernel_stop();
}

static void prv_sched_waiter(void *arg) {
  pbl_sem_take(&s_sem, PBL_FOREVER);
  prv_trace('W');
}

void test_kernel__sched_lock_defers_switch(void) {
  pbl_sem_init(&s_sem, s_sem.initial, s_sem.limit);
  prv_spawn(0, "waiter", 3, prv_sched_waiter, NULL);
  prv_spawn(1, "locker", 2, prv_sched_locked, NULL);
  pbl_test_kernel_run();
  cl_assert_equal_s(s_trace, "abWc");
}
