/* SPDX-FileCopyrightText: 2025 Google LLC */
/* SPDX-FileCopyrightText: 2015-2024 The Apache Software Foundation */
/* SPDX-License-Identifier: Apache-2.0 */

// This is derived from the freertos port provided by NimBLE
// and modified to suit Pebble OS (timers, mutexes).

#ifndef _NIMBLE_NPL_OS_H_
#define _NIMBLE_NPL_OS_H_

#include "pbl/kernel/irq.h"
#include "pbl/kernel/sched.h"
#include <assert.h>
#include <stdint.h>
#include <string.h>

#include "kernel/pbl_malloc.h"
#include "pbl/kernel/mutex.h"
#include "pbl/drivers/rtc.h"
#include "pbl/kernel/msgq.h"
#include "pbl/kernel/thread.h"
#include "pbl/kernel/sem.h"
#include "pbl/services/new_timer/new_timer.h"

#include "os/os_cputime.h"

#if NRF52_SERIES
#include <pbl/drivers/clocksource.h>
#include "pbl/soc/nrf/sleep.h"
#endif

#define BLE_NPL_OS_ALIGNMENT 4

#define BLE_NPL_TIME_FOREVER PBL_TICK_FOREVER

typedef uint32_t ble_npl_time_t;
typedef int32_t ble_npl_stime_t;

struct ble_npl_event {
  bool queued;
  ble_npl_event_fn *fn;
  void *arg;
};

#define BLE_NPL_EVENTQ_DEPTH 32

struct ble_npl_eventq {
  struct pbl_msgq q;
  struct ble_npl_event *buf[BLE_NPL_EVENTQ_DEPTH];
};

struct ble_npl_callout {
  TimerID handle;

  //! whether we have made changes to this timer from ISR state and need
  //! someone to go sync them back up on the main thread for us
  bool update_pending;

  //! if we are pending_update, there might have been some ticks between
  //! when we asked to do work, and when it happens; set an actual target
  //! time in ticks for newtimer, rather than deferring the number of ticks
  //! to start counting when we actually schedule
  RtcTicks update_target_time_ticks;

  //! event to be pushed for update from ISR
  struct ble_npl_event update_ev;

  struct ble_npl_eventq *evq;
  struct ble_npl_event ev;
  uint64_t ticks;
};

struct ble_npl_mutex {
  struct pbl_mutex handle;
};

struct ble_npl_sem {
  struct pbl_sem handle;
};

#include "npl_pebble.h"

static inline bool ble_npl_os_started(void) {
  return pbl_kernel_is_started();
}

static inline void *ble_npl_get_current_task_id(void) { return pbl_thread_current(); }

static inline void ble_npl_eventq_init(struct ble_npl_eventq *evq) {
  pbl_msgq_init(&evq->q, evq->buf, sizeof(struct ble_npl_event *), BLE_NPL_EVENTQ_DEPTH);
}

static inline struct ble_npl_event *ble_npl_eventq_get(struct ble_npl_eventq *evq,
                                                       ble_npl_time_t tmo) {
  return npl_pebble_eventq_get(evq, tmo);
}

static inline void ble_npl_eventq_put(struct ble_npl_eventq *evq, struct ble_npl_event *ev) {
  npl_pebble_eventq_put(evq, ev);
}

static inline void ble_npl_eventq_remove(struct ble_npl_eventq *evq, struct ble_npl_event *ev) {
  npl_pebble_eventq_remove(evq, ev);
}

static inline void ble_npl_event_run(struct ble_npl_event *ev) { ev->fn(ev); }

static inline bool ble_npl_eventq_is_empty(struct ble_npl_eventq *evq) {
  return pbl_msgq_num_used(&evq->q) == 0;
}

static inline void ble_npl_event_init(struct ble_npl_event *ev, ble_npl_event_fn *fn, void *arg) {
  memset(ev, 0, sizeof(*ev));
  ev->fn = fn;
  ev->arg = arg;
}

static inline bool ble_npl_event_is_queued(struct ble_npl_event *ev) { return ev->queued; }

static inline void *ble_npl_event_get_arg(struct ble_npl_event *ev) { return ev->arg; }

static inline void ble_npl_event_set_arg(struct ble_npl_event *ev, void *arg) { ev->arg = arg; }

static inline ble_npl_error_t ble_npl_mutex_init(struct ble_npl_mutex *mu) {
  return npl_pebble_mutex_init(mu);
}

static inline ble_npl_error_t ble_npl_mutex_pend(struct ble_npl_mutex *mu, ble_npl_time_t timeout) {
  return npl_pebble_mutex_pend(mu, timeout);
}

static inline ble_npl_error_t ble_npl_mutex_release(struct ble_npl_mutex *mu) {
  return npl_pebble_mutex_release(mu);
}

static inline ble_npl_error_t ble_npl_sem_init(struct ble_npl_sem *sem, uint16_t tokens) {
  return npl_pebble_sem_init(sem, tokens);
}

static inline ble_npl_error_t ble_npl_sem_pend(struct ble_npl_sem *sem, ble_npl_time_t timeout) {
  return npl_pebble_sem_pend(sem, timeout);
}

static inline ble_npl_error_t ble_npl_sem_release(struct ble_npl_sem *sem) {
  return npl_pebble_sem_release(sem);
}

static inline uint16_t ble_npl_sem_get_count(struct ble_npl_sem *sem) {
  return pbl_sem_count(&sem->handle);
}

static inline void ble_npl_callout_init(struct ble_npl_callout *co, struct ble_npl_eventq *evq,
                                        ble_npl_event_fn *ev_cb, void *ev_arg) {
  npl_pebble_callout_init(co, evq, ev_cb, ev_arg);
}

static inline ble_npl_error_t ble_npl_callout_reset(struct ble_npl_callout *co,
                                                    ble_npl_time_t ticks) {
  return npl_pebble_callout_reset(co, ticks);
}

static inline void ble_npl_callout_stop(struct ble_npl_callout *co) { npl_pebble_callout_stop(co); }

static inline bool ble_npl_callout_is_active(struct ble_npl_callout *co) {
  return npl_pebble_callout_is_active(co);
}

static inline ble_npl_time_t ble_npl_callout_get_ticks(struct ble_npl_callout *co) {
  return npl_pebble_callout_get_ticks(co);
}

static inline uint32_t ble_npl_callout_remaining_ticks(struct ble_npl_callout *co,
                                                       ble_npl_time_t time) {
  return npl_pebble_callout_remaining_ticks(co, time);
}

static inline void ble_npl_callout_set_arg(struct ble_npl_callout *co, void *arg) {
  co->ev.arg = arg;
}

static inline ble_npl_time_t ble_npl_time_get(void) {
  /* This downconverts from an RtcTicks (uint64_t) to a uint32_t.  This is
   * safe because nothing in NimBLE uses an absolute ble_npl_time_get (i.e.,
   * it does not turn it into an absolute ble_npl_time_t), or a
   * ble_npl_time_get that is long-lived; it is always compared by
   * subtracting and casting to a ble_npl_stime_t, rather than by looking
   * for equality.
   *
   * (Some of the `mesh` code uses ble_npl_time_get in ways that look
   * distressing, but luckily, we do not use any of that code.  And, more to
   * the point, uint32_t is a non-optional definition of ble_npl_time_t: for
   * instance, ble_gap.c fails to build with any other definition of
   * ble_npl_time_t.)
   */
  return rtc_get_ticks();
}

static inline ble_npl_error_t ble_npl_time_ms_to_ticks(uint32_t ms, ble_npl_time_t *out_ticks) {
  return npl_pebble_time_ms_to_ticks(ms, out_ticks);
}

static inline ble_npl_error_t ble_npl_time_ticks_to_ms(ble_npl_time_t ticks, uint32_t *out_ms) {
  return npl_pebble_time_ticks_to_ms(ticks, out_ms);
}

static inline ble_npl_time_t ble_npl_time_ms_to_ticks32(uint32_t ms) { return ms; }

static inline uint32_t ble_npl_time_ticks_to_ms32(ble_npl_time_t ticks) { return ticks; }

static inline void ble_npl_time_delay(ble_npl_time_t ticks) { pbl_thread_sleep(PBL_TICKS(ticks)); }

#if NIMBLE_CFG_CONTROLLER
void ble_npl_hw_set_isr(int irqn, void (*addr)(void));
#endif

static inline uint32_t ble_npl_hw_enter_critical(void) {
  pbl_irq_lock();
  return 0;
}

static inline void ble_npl_hw_exit_critical(uint32_t ctx) { pbl_irq_unlock(); }

static inline bool ble_npl_hw_is_in_critical(void) {
  return pbl_irq_is_locked();
}
#define realloc kernel_realloc

#if NRF52_SERIES

static inline void nrf52_clock_hfxo_request(void) {
  /* Any time NimBLE wants to turn on the HFXO, we are also going to have
   * pretty stringent wake latency requirements, so we must not do expensive
   * and slow things like waking the NOR flash up from DPD on interrupts.
   * So, we disable full sleep at the same time as actually enabling the
   * HFXO.
   */
  soc_nrf_sleep_full_block();
  clocksource_hfxo_request();
}

static inline void nrf52_clock_hfxo_release(void) {
  clocksource_hfxo_release();
  soc_nrf_sleep_full_release();
}

#endif

#endif /* _NPL_H_ */
