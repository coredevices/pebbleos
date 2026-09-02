/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include <stdio.h>
#include <inttypes.h>

#include "drivers/flash.h"
#include "drivers/rtc.h"
#include "drivers/task_watchdog.h"
#include "console/prompt.h"
#include "kernel/util/idle.h"
#include "pbl/services/analytics/analytics.h"
#include "pbl/soc/nrf/sleep.h"

#include <cmsis_core.h>

#include <hal/nrf_nvmc.h>

#include "pbl/kernel/idle.h"

static RtcTicks s_analytics_sleep_ticks = 0;
static RtcTicks s_analytics_full_sleep_ticks = 0;

static const RtcTicks EARLY_WAKEUP_TICKS = 2;
static const RtcTicks MIN_FULL_SLEEP_TICKS = 5;

void pbl_soc_idle(pbl_tick_t max_ticks) {
  if (!rtc_alarm_is_initialized() || !idle_is_allowed()) {
    return;
  }

  __disable_irq();

  if (pbl_idle_confirm()) {
    if (max_ticks < MIN_FULL_SLEEP_TICKS || !soc_nrf_sleep_full_is_allowed()) {
      RtcTicks sleep_start_ticks = rtc_get_ticks();

      NRF_NVMC->ICACHECNF &= ~NVMC_ICACHECNF_CACHEEN_Msk;

      __DSB();
      __WFI();
      __ISB();

      NRF_NVMC->ICACHECNF |= NVMC_ICACHECNF_CACHEEN_Msk;

      s_analytics_sleep_ticks += rtc_get_ticks() - sleep_start_ticks;
    } else {
      const RtcTicks sleep_ticks = max_ticks - EARLY_WAKEUP_TICKS;
      RtcTicks elapsed_ticks;

      flash_power_down_for_stop_mode();

      rtc_alarm_set(sleep_ticks);
      rtc_systick_pause();

      NRF_NVMC->ICACHECNF &= ~NVMC_ICACHECNF_CACHEEN_Msk;

      __DSB();
      __WFI();
      __ISB();

      NRF_NVMC->ICACHECNF |= NVMC_ICACHECNF_CACHEEN_Msk;

      rtc_systick_resume();
      elapsed_ticks = rtc_alarm_get_elapsed_ticks();
      pbl_idle_slept(elapsed_ticks);

      flash_power_up_after_stop_mode();
      task_watchdog_step_elapsed_time_ms((elapsed_ticks * 1000) / RTC_TICKS_HZ);

      s_analytics_full_sleep_ticks += elapsed_ticks;
    }
  }

  __enable_irq();
}

bool pbl_soc_tick_enable(void) {
  rtc_enable_synthetic_systick();
  return true;
}

// CPU analytics
///////////////////////////////////////////////////////////

static uint32_t s_last_ticks = 0;

void dump_current_runtime_stats(void) {
  uint32_t sleep_ticks = s_analytics_sleep_ticks;
  uint32_t full_sleep_ticks = s_analytics_full_sleep_ticks;

  uint32_t now_ticks = rtc_get_ticks();
  uint32_t total_ticks = now_ticks - s_last_ticks;
  uint32_t running_ticks = total_ticks - full_sleep_ticks - sleep_ticks;

  char buf[160];
  snprintf(buf, sizeof(buf), "Run:     %"PRIu32" ticks (%"PRIu32" %%)",
           running_ticks, (running_ticks * 100) / total_ticks);
  prompt_send_response(buf);
  snprintf(buf, sizeof(buf), "Sleep 0: %"PRIu32" ticks (%"PRIu32" %%)",
           sleep_ticks, (sleep_ticks * 100) / total_ticks);
  prompt_send_response(buf);
  snprintf(buf, sizeof(buf), "Sleep 1: %"PRIu32" ticks (%"PRIu32" %%)",
           full_sleep_ticks, (full_sleep_ticks * 100) / total_ticks);
  prompt_send_response(buf);
  snprintf(buf, sizeof(buf), "Total:   %"PRIu32" ticks", total_ticks);
  prompt_send_response(buf);
}

void pbl_analytics_external_collect_cpu_stats(void) {
  uint32_t full_sleep_ticks = s_analytics_full_sleep_ticks;
  uint32_t sleep_ticks = s_analytics_sleep_ticks;

  RtcTicks now_ticks = rtc_get_ticks();
  uint32_t total_ticks = (uint32_t)(now_ticks - s_last_ticks);
  uint32_t running_ticks = total_ticks - full_sleep_ticks - sleep_ticks;

  // Calculate percentages
  uint16_t running_pct = 0;
  uint16_t full_sleep_pct = 0;
  uint16_t sleep_pct = 0;

  if (total_ticks > 0) {
    running_pct = (uint16_t)((running_ticks * 10000ULL) / total_ticks);
    full_sleep_pct = (uint16_t)((full_sleep_ticks * 10000ULL) / total_ticks);
    sleep_pct = (uint16_t)((sleep_ticks * 10000ULL) / total_ticks);
  }

  PBL_ANALYTICS_SET_UNSIGNED(cpu_running_pct, running_pct);
  PBL_ANALYTICS_SET_UNSIGNED(cpu_sleep0_pct, sleep_pct);
  PBL_ANALYTICS_SET_UNSIGNED(cpu_sleep1_pct, full_sleep_pct);
  PBL_ANALYTICS_SET_UNSIGNED(cpu_sleep2_pct, 0);

  s_last_ticks = now_ticks;
  s_analytics_sleep_ticks = 0;
  s_analytics_full_sleep_ticks = 0;
}