/* SPDX-FileCopyrightText: 2025 Google LLC */
/* SPDX-FileCopyrightText: 2015-2024 The Apache Software Foundation */
/* SPDX-License-Identifier: Apache-2.0 */

// This is derived from the freertos port provided by NimBLE
// and modified to suit Pebble OS (timers, mutexes).

#include <assert.h>
#include <stddef.h>
#include <string.h>

#include "pbl/mcu/interrupts.h"
#include "nimble/nimble_npl.h"
#include "nimble/nimble_port.h"
#include "pbl/kernel/mutex.h"
#include "pbl/os/tick.h"
#include "pbl/services/new_timer/new_timer.h"
#include <pbl/logging/logging.h>
#include "system/passert.h"

struct ble_npl_event *npl_pebble_eventq_get(struct ble_npl_eventq *evq, ble_npl_time_t tmo) {
  struct ble_npl_event *ev = NULL;
  BaseType_t woken;
  BaseType_t ret;

  if (mcu_state_is_isr()) {
    assert(tmo == 0);
    ret = xQueueReceiveFromISR(evq->q, &ev, &woken);
    portYIELD_FROM_ISR(woken);
  } else {
    ret = xQueueReceive(evq->q, &ev, tmo);
  }
  assert(ret == pdPASS || ret == errQUEUE_EMPTY);

  if (ev) {
    ev->queued = false;
  }

  return ev;
}

void npl_pebble_eventq_put(struct ble_npl_eventq *evq, struct ble_npl_event *ev) {
  BaseType_t woken;
  BaseType_t ret;

  if (ev->queued) {
    return;
  }

  ev->queued = true;

  if (mcu_state_is_isr()) {
    ret = xQueueSendToBackFromISR(evq->q, &ev, &woken);
    portYIELD_FROM_ISR(woken);
  } else {
    ret = xQueueSendToBack(evq->q, &ev, vPortInCritical() ? 0U : portMAX_DELAY);
  }

  assert(ret == pdPASS);
}

void npl_pebble_eventq_remove(struct ble_npl_eventq *evq, struct ble_npl_event *ev) {
  struct ble_npl_event *tmp_ev;
  BaseType_t ret;
  int i;
  int count;
  BaseType_t woken, woken2;

  if (!ev->queued) {
    return;
  }

  /*
   * XXX We cannot extract element from inside FreeRTOS queue so as a quick
   * workaround we'll just remove all elements and add them back except the
   * one we need to remove. This is silly, but works for now - we probably
   * better use counting semaphore with os_queue to handle this in future.
   */

  if (mcu_state_is_isr()) {
    woken = pdFALSE;

    count = uxQueueMessagesWaitingFromISR(evq->q);
    for (i = 0; i < count; i++) {
      ret = xQueueReceiveFromISR(evq->q, &tmp_ev, &woken2);
      assert(ret == pdPASS);
      woken |= woken2;

      if (tmp_ev == ev) {
        continue;
      }

      ret = xQueueSendToBackFromISR(evq->q, &tmp_ev, &woken2);
      assert(ret == pdPASS);
      woken |= woken2;
    }

    portYIELD_FROM_ISR(woken);
  } else {
    vPortEnterCritical();

    count = uxQueueMessagesWaiting(evq->q);
    for (i = 0; i < count; i++) {
      ret = xQueueReceive(evq->q, &tmp_ev, 0);
      assert(ret == pdPASS);

      if (tmp_ev == ev) {
        continue;
      }

      ret = xQueueSendToBack(evq->q, &tmp_ev, 0);
      assert(ret == pdPASS);
    }

    vPortExitCritical();
  }

  ev->queued = 0;
}

ble_npl_error_t npl_pebble_mutex_init(struct ble_npl_mutex *mu) {
  if (!mu) {
    return BLE_NPL_INVALID_PARAM;
  }

  pbl_mutex_init(&mu->handle);

  return BLE_NPL_OK;
}

ble_npl_error_t npl_pebble_mutex_pend(struct ble_npl_mutex *mu, ble_npl_time_t timeout) {
  if (!mu) {
    return BLE_NPL_INVALID_PARAM;
  }

  if (mcu_state_is_isr()) {
    WTF;
  }

  return (pbl_mutex_lock(&mu->handle, PBL_TICKS(timeout)) == 0) ? BLE_NPL_OK : BLE_NPL_TIMEOUT;
}

ble_npl_error_t npl_pebble_mutex_release(struct ble_npl_mutex *mu) {
  if (!mu) {
    return BLE_NPL_INVALID_PARAM;
  }

  pbl_mutex_unlock(&mu->handle);

  return BLE_NPL_OK;
}

ble_npl_error_t npl_pebble_sem_init(struct ble_npl_sem *sem, uint16_t tokens) {
  if (!sem) {
    return BLE_NPL_INVALID_PARAM;
  }

  pbl_sem_init(&sem->handle, tokens, 128);
  return BLE_NPL_OK;
}

ble_npl_error_t npl_pebble_sem_pend(struct ble_npl_sem *sem, ble_npl_time_t timeout) {
  if (!sem) {
    return BLE_NPL_INVALID_PARAM;
  }

  if (mcu_state_is_isr()) {
    assert(timeout == 0);
  }

  return pbl_sem_take(&sem->handle, PBL_TICKS(timeout)) == 0 ? BLE_NPL_OK : BLE_NPL_TIMEOUT;
}

ble_npl_error_t npl_pebble_sem_release(struct ble_npl_sem *sem) {
  if (!sem) {
    return BLE_NPL_INVALID_PARAM;
  }

  pbl_sem_give(&sem->handle);
  return BLE_NPL_OK;
}

static void os_callout_timer_cb(void *timer);

//! npl_pebble_callout_do_update is the core logic of the update, and can
//! happen from a few places.  It can happen directly from
//! npl_pebble_callout_reset, but if that's called while we're in an ISR, it
//! can get deferred to an eventq for later, since new_timer is not
//! ISR-safe.  And, if there's an update that is waiting for an eventq that
//! didn't get called yet, but the timer did get called, this function can
//! also get called from when the callout otherwise would have happened, to
//! go reset the timer and try again later.
static void npl_pebble_callout_do_update(struct ble_npl_callout *co) {
  if (!co->update_pending) {
    return;
  }

  // It's possible that we "missed" -- that is to say, something like the
  // following happened:
  //
  //  * npl_pebble_callout_reset got called in an ISR, with a small number
  //    of `ticks`.
  //  * An `update_ev` is enqueued to go actually do the update.
  //  * The CPU is heavily loaded, and more than `ticks` pass.
  //  * Eventually, the `update_ev` comes to the front of the `eventq`, and
  //    finally triggers the npl_pebble_callout_do_update.
  //  * But the remaining time on the timer is negative; we are late!  (This
  //    is, of course, perfectly allowed: a timer is a promise to do
  //    something *not sooner* than a certain time, and any amount of time
  //    later.  If NimBLE needs something done with hard real time results,
  //    it needs to use its own built-in TIMER and RTC.)
  //  * We don't want to just trigger the event from this thread; for the
  //    sake of consistency, and avoiding exacerbating extremely rare bugs
  //    that would be quite difficult to track down, we want to ensure that
  //    callouts are always triggered from the same context.  So, we defer
  //    it for the minimum amount of time possible, but we make sure that it
  //    is triggered through the same new_timer path that it always would be
  //    otherwise.
  int rem_ticks = co->update_target_time_ticks - rtc_get_ticks();
  if (rem_ticks < 1) {
    // Oh, well...
    rem_ticks = 1;
  }

  new_timer_stop(co->handle);
  PBL_ASSERTN(new_timer_start(co->handle, ticks_to_milliseconds(rem_ticks), os_callout_timer_cb, co, 0));

  co->update_pending = false;
}

static void npl_pebble_callout_do_update_from_eventq(struct ble_npl_event *ev) {
  npl_pebble_callout_do_update((struct ble_npl_callout *)ev->arg);
}

static void os_callout_timer_cb(void *timer) {
  struct ble_npl_callout *co = timer;

  // check to see if we are pending an update; if so, trigger it and do
  // nothing, since we were already asked to delay until later
  if (co->update_pending) {
    npl_pebble_callout_do_update(co);
    return;
  }

  if (co->evq) {
    ble_npl_eventq_put(co->evq, &co->ev);
  } else {
    co->ev.fn(&co->ev);
  }
}

void npl_pebble_callout_init(struct ble_npl_callout *co, struct ble_npl_eventq *evq,
                             ble_npl_event_fn *ev_cb, void *ev_arg) {
  memset(co, 0, sizeof(*co));

  co->handle = new_timer_create();
  PBL_ASSERTN(co->handle != TIMER_INVALID_ID);
  co->evq = evq;
  co->ticks = 0;
  co->update_ev.queued = false;

  ble_npl_event_init(&co->ev, ev_cb, ev_arg);
}


ble_npl_error_t npl_pebble_callout_reset(struct ble_npl_callout *co, ble_npl_time_t ticks) {
  co->ticks = ticks;

  co->update_pending = 1;
  co->update_target_time_ticks = rtc_get_ticks() + ticks;

  // we may be in an ISR; if we are, we make the event queue actually
  // schedule the timer for us later.  otherwise, we can do it ourselves.
  if (mcu_state_is_isr()) {
    // it's a two-step operation here: we insert the update request into the
    // eventq (which *is* an ISR-safe operation), and then the eventq,
    // whenever it's processed, actually schedules the timer based on the
    // update_target_time_ticks.
    co->update_ev.fn = npl_pebble_callout_do_update_from_eventq;
    co->update_ev.arg = (void *)co;
    ble_npl_eventq_put(nimble_port_get_dflt_eventq(), &co->update_ev);
  } else {
    npl_pebble_callout_do_update(co);
  }

  return BLE_NPL_OK;
}

void npl_pebble_callout_stop(struct ble_npl_callout *co) { new_timer_stop(co->handle); }

bool npl_pebble_callout_is_active(struct ble_npl_callout *co) {
  return new_timer_scheduled(co->handle, NULL);
}

ble_npl_time_t npl_pebble_callout_get_ticks(struct ble_npl_callout *co) { return co->ticks; }

ble_npl_time_t npl_pebble_callout_remaining_ticks(struct ble_npl_callout *co, ble_npl_time_t now) {
  uint32_t rt = 0;
  new_timer_scheduled(co->handle, &rt);
  return rt;
}

ble_npl_error_t npl_pebble_time_ms_to_ticks(uint32_t ms, ble_npl_time_t *out_ticks) {
  uint64_t ticks;

  ticks = milliseconds_to_ticks(ms);
  if (ticks > UINT32_MAX) {
    return BLE_NPL_EINVAL;
  }

  *out_ticks = ticks;

  return 0;
}

ble_npl_error_t npl_pebble_time_ticks_to_ms(ble_npl_time_t ticks, uint32_t *out_ms) {
  uint64_t ms;

  ms = ticks_to_milliseconds(ticks);
  if (ms > UINT32_MAX) {
    return BLE_NPL_EINVAL;
  }

  *out_ms = ms;

  return 0;
}

