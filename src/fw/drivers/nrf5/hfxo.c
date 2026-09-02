/* SPDX-FileCopyrightText: 2025 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "pbl/kernel/irq.h"
#include <stdint.h>

#include <system/passert.h>

#include <hal/nrf_clock.h>

static uint8_t prv_refcnt;

void clocksource_hfxo_request(void) {
  pbl_irq_lock();

  PBL_ASSERT(prv_refcnt < UINT8_MAX, "HFXO refcount overflow");

  if (prv_refcnt == 0U && !nrf_clock_hf_is_running(NRF_CLOCK, NRF_CLOCK_HFCLK_HIGH_ACCURACY)) {
    nrf_clock_event_clear(NRF_CLOCK, NRF_CLOCK_EVENT_HFCLKSTARTED);
    nrf_clock_task_trigger(NRF_CLOCK, NRF_CLOCK_TASK_HFCLKSTART);
    while (!nrf_clock_event_check(NRF_CLOCK, NRF_CLOCK_EVENT_HFCLKSTARTED)) {
    }
  }

  prv_refcnt++;

  pbl_irq_unlock();
}

void clocksource_hfxo_release(void) {
  pbl_irq_lock();

  PBL_ASSERT(prv_refcnt != 0U, "HFXO refcount underflow");

  prv_refcnt--;
  if (prv_refcnt == 0U) {
    nrf_clock_task_trigger(NRF_CLOCK, NRF_CLOCK_TASK_HFCLKSTOP);
  }

  pbl_irq_unlock();
}
