/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include <inttypes.h>
#include <stdio.h>

#include "board/board.h"
#include "console/prompt.h"
#include "drivers/flash.h"
#include "drivers/rtc.h"
#include "drivers/sf32lb52/rc10k.h"
#include "drivers/task_watchdog.h"
#include "kernel/util/idle.h"
#include "pbl/services/analytics/analytics.h"
#include "pbl/soc/sf32lb/sleep.h"
#include "pbl/util/math.h"

#include <bf0_hal.h>

#include <ipc_queue.h>

#include "FreeRTOS.h"
#include "task.h"

// HAL tick counter (milliseconds) - used by HAL timeout functions
extern __IO uint32_t uwTick;

static LPTIM_HandleTypeDef s_lptim = {
    .Instance = LPTIM1,
};

// CPU analytics tracking
typedef enum {
  SleepTypeNone = 0,
  SleepTypeWfi,
  SleepTypeDeepWfi,
} SleepType;

static volatile SleepType s_last_sleep_type;
static RtcTicks s_analytics_wfi_ticks;
static RtcTicks s_analytics_deepwfi_ticks;
static RtcTicks s_analytics_deepsleep_ticks;
static RtcTicks s_last_ticks;
static uint32_t s_analytics_ipc_not_idle_count;
static bool s_force_wfi;

//! Early wake-up ticks (to avoid over-sleeping due to wake-up latency)
static const uint32_t EARLY_WAKEUP_TICKS = 4;
//! Minimum ticks to enter deep sleep
static const uint32_t MIN_DEEPSLEEP_TICKS = RTC_TICKS_HZ / 20;
//! Maximum LPTIM counter value (24-bit)
static const uint32_t MAX_LPTIM_CNT = 0xFFFFFFUL;

static uint32_t s_iser_bak[16];

static void prv_wdt_feed(uint16_t elapsed_ticks) {
  static uint32_t wdt_feed_ticks;

  wdt_feed_ticks += elapsed_ticks;
  if (wdt_feed_ticks >= (RTC_TICKS_HZ / (1000 / TASK_WATCHDOG_FEED_PERIOD_MS))) {
    wdt_feed_ticks = 0U;
    task_watchdog_feed();
  }
}

static void prv_save_iser(void) {
  uint32_t i;
  for (i = 0; i < 16; i++) {
    s_iser_bak[i] = NVIC->ISER[i];
    NVIC->ICER[i] = 0xFFFFFFFF;
    __DSB();
    __ISB();
  }
}

static void prv_restore_iser(void) {
  uint32_t i;
  for (i = 0; i < 16; i++) {
    __COMPILER_BARRIER();
    NVIC->ISER[i] = s_iser_bak[i];
    __COMPILER_BARRIER();
  }
}

static inline void prv_enter_wfi(void) {
  s_last_sleep_type = SleepTypeWfi;
  __WFI();
}

static void prv_enter_deepwfi(void) {
  s_last_sleep_type = SleepTypeDeepWfi;

  flash_power_down_for_stop_mode();

  __DSB();
  __ISB();

  SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
  __WFI();
  SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;
}

static void prv_enter_deepslep(void) {
  QSPIPortState *flash_state;
  uint32_t dll1_freq = 0UL;
  int clk_src;

  flash_state = QSPI_FLASH->qspi->state;

  prv_save_iser();

  HAL_FLASH_NOP_CMD(&flash_state->ctx.handle);
  HAL_FLASH_DEEP_PWRDOWN(&flash_state->ctx.handle);
  HAL_Delay_us(flash_state->t_enter_deep_us);

  NVIC_EnableIRQ(AON_IRQn);

  clk_src = HAL_RCC_HCPU_GetClockSrc(RCC_CLK_MOD_SYS);
  if (clk_src == RCC_SYSCLK_DLL1) {
    HAL_RCC_HCPU_ClockSelect(RCC_CLK_MOD_SYS, RCC_SYSCLK_HRC48);
    dll1_freq = HAL_RCC_HCPU_GetDLL1Freq();
    HAL_RCC_HCPU_DisableDLL1();
  }

  HAL_HPAON_DISABLE_PAD();
  HAL_HPAON_DISABLE_VHP();

  HAL_HPAON_CLEAR_HP_ACTIVE();
  HAL_HPAON_SET_POWER_MODE(AON_PMR_DEEP_SLEEP);

  __WFI();
  __NOP();
  __NOP();
  __NOP();
  __NOP();
  __NOP();
  __NOP();
  __NOP();
  __NOP();
  __NOP();
  __NOP();

  HAL_HPAON_ENABLE_PAD();
  HAL_HPAON_ENABLE_VHP();

  HAL_HPAON_SET_HP_ACTIVE();
  HAL_HPAON_CLEAR_POWER_MODE();

  // Wait for HXT48 to be ready
  if (dll1_freq != 0UL) {
    while (0 == (hwp_hpsys_aon->ACR & HPSYS_AON_ACR_HXT48_RDY)) {
      __NOP();
    }
  }

  if (clk_src == RCC_SYSCLK_DLL1) {
    HAL_RCC_HCPU_EnableDLL1(dll1_freq);
    HAL_RCC_HCPU_ClockSelect(RCC_CLK_MOD_SYS, RCC_SYSCLK_DLL1);
    HAL_Delay_us(0);
  }

  HAL_FLASH_RELEASE_DPD(&flash_state->ctx.handle);
  HAL_Delay_us(flash_state->t_exit_deep_us);

  prv_restore_iser();
}

static uint32_t prv_calc_elapsed_ticks(uint32_t gtimer_cyc) {
  static uint16_t s_err_milli_ticks;
  uint32_t elapsed_milli_ticks;
  uint32_t elapsed_ticks;

  elapsed_milli_ticks = rc10k_cyc_to_milli_ticks(gtimer_cyc);
  elapsed_ticks = elapsed_milli_ticks / 1000U;
  s_err_milli_ticks += (elapsed_milli_ticks % 1000U);
  if (s_err_milli_ticks >= 1000U) {
    elapsed_ticks++;
    s_err_milli_ticks -= 1000U;
  }

  return elapsed_ticks;
}

void vPortSuppressTicksAndSleep(TickType_t xExpectedIdleTime) {
  if (!idle_is_allowed()) {
    return;
  }

  if (!ipc_queue_check_idle()) {
    s_analytics_ipc_not_idle_count++;
    return;
  }

  __disable_irq();

  if (eTaskConfirmSleepModeStatus() != eAbortSleep) {
    SocSf32lbSleepLevel max_level = soc_sf32lb_sleep_max_level();

    // Deep sleep needs a minimum idle window.
    if (xExpectedIdleTime < MIN_DEEPSLEEP_TICKS) {
      max_level = MIN(max_level, SOC_SF32LB_DEEPWFI);
    }

    // The debug flag forces plain WFI (deep WFI and deep sleep disabled).
    if (s_force_wfi) {
      max_level = MIN(max_level, SOC_SF32LB_WFI);
    }

    switch (max_level) {
      case SOC_SF32LB_WFI:
        prv_enter_wfi();
        break;
      case SOC_SF32LB_DEEPWFI:
        prv_enter_deepwfi();
        break;
      case SOC_SF32LB_DEEPSLEEP: {
        uint32_t gtimer_start;
        uint32_t gtimer_stop;
        uint32_t gtimer_delta;
        uint32_t sleep_ticks;
        uint32_t lptim_ticks;
        uint32_t elapsed_ticks;

        // stop systick
        SysTick->CTRL &= ~(SysTick_CTRL_ENABLE_Msk | SysTick_CTRL_TICKINT_Msk);
        SCB->ICSR = SCB_ICSR_PENDSTCLR_Msk;

        // configure LPTIM to wake us up after expected idle time
        sleep_ticks = xExpectedIdleTime - EARLY_WAKEUP_TICKS;
        lptim_ticks = MIN(sleep_ticks * rc10k_get_freq_hz() / RTC_TICKS_HZ,
                          MAX_LPTIM_CNT);
        HAL_LPTIM_Counter_Start_IT(&s_lptim, lptim_ticks);

        gtimer_start = HAL_GTIMER_READ();

        prv_enter_deepslep();

        // NOTE: GTIMER needs at least 1 LP clock cycle to be updated after sleep,
        // so spin until we see a change
        do {
          gtimer_stop = HAL_GTIMER_READ();
        } while (gtimer_stop == gtimer_start);

        if (gtimer_stop < gtimer_start) {
          gtimer_delta = (UINT32_MAX - gtimer_start) + gtimer_stop + 1UL;
        } else {
          gtimer_delta = gtimer_stop - gtimer_start;
        }

        elapsed_ticks = prv_calc_elapsed_ticks(gtimer_delta);

        vTaskStepTick(elapsed_ticks);

        // increment HAL tick counter by elapsed ticks
        uwTick += elapsed_ticks;

        prv_wdt_feed(elapsed_ticks);

        // Force RTC synchronization of shadow registers
        hwp_rtc->ISR &= RTC_RSF_MASK;

        // Track deep sleep time for analytics
        s_analytics_deepsleep_ticks += elapsed_ticks;

        // stop LPTIM
        HAL_LPTIM_Counter_Stop_IT(&s_lptim);

        // enable systick
        SysTick->CTRL |= (SysTick_CTRL_ENABLE_Msk | SysTick_CTRL_TICKINT_Msk);
        break;
      }
      default:
        break;
    }
  }

  __enable_irq();
}

bool vPortEnableTimer() {
  HAL_LPTIM_InitDefault(&s_lptim);
  HAL_LPTIM_Init(&s_lptim);

  // configure SYSTICK
  // - use HXT48 as TICK reference clock
  // - divide TICK reference clock to 1.92MHz (48MHz / 25) (TICK_CLK)
  // - enable TICK_CLK as SYSTICK clock source
  // - configure SYSTICK to generate interrupt at RTC_TICKS_HZ rate
  //
  // HXT48│\                        ┌────────────────┐
  //  ────┼ \                       │         SYSTICK│
  // HRC48│  │  ┌───────┐  TICK_CLK ││\              │
  //  ────┼  ├──│TICKDIV├───────────┼┤ │   ┌──────┐  │
  //   ...│  │  └───────┘.         ┌┼┤ ├───│RELOAD├──┤
  //      │ /                      │││/    └──────┘  │
  //      │/                       ││                │
  //           HCLK                ││                │
  //          ─────────────────────┘└────────────────┘

  HAL_RCC_HCPU_ClockSelect(RCC_CLK_MOD_HP_TICK, RCC_CLK_TICK_HXT48);
  // delay to avoid systick config failure (undocumented silicon issue)
  HAL_Delay_us(200);
  HAL_RCC_HCPU_SetTickDiv(25);
  HAL_SYSTICK_Config(1920000 / RTC_TICKS_HZ);
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_TICK_CLK);

  SysTick->CTRL |= (SysTick_CTRL_ENABLE_Msk | SysTick_CTRL_TICKINT_Msk);

  // configure clock dividers for deep WFI:
  // HCLK = 48MHz / 12 = 4MHz
  // PCLK1 = 4MHz / 2^0 = 4MHz
  // PCLK2 = 4MHz / 2^1 = 2MHz
  HAL_RCC_HCPU_SetDeepWFIDiv(12, 0, 1);

  // TODO(SF32LB52): to use deep WFI when audio is ON, HCLK needs to remain
  // at 48MHz (div=1). Also, clock needs to be forced ON during deep WFI
  // (FORCE_HP bit in HPSYS_RCC_DBGR register)

  return true;
}

void AON_IRQHandler(void)
{
    uint32_t status;

    NVIC_DisableIRQ(AON_IRQn);
    HAL_HPAON_CLEAR_POWER_MODE();

    status = HAL_HPAON_GET_WSR();
    status &= ~HPSYS_AON_WSR_PIN_ALL;
    HAL_HPAON_CLEAR_WSR(status);
}

void SysTick_Handler(void) {
  extern void xPortSysTickHandler(void);
  xPortSysTickHandler();

  HAL_IncTick();

  prv_wdt_feed(1U);

  if (s_last_sleep_type == SleepTypeWfi) {
    s_analytics_wfi_ticks++;
  } else if (s_last_sleep_type == SleepTypeDeepWfi) {
    s_analytics_deepwfi_ticks++;
  }

  s_last_sleep_type = SleepTypeNone;

  // TODO(SF32LB52): we may need to handle tick loss compensation when using
  // SysTick due to flash erase times (runs with IRQs disabled to not interfere
  // with XIP, and can easily span multiple ticks)
}

void dump_current_runtime_stats(void) {
  uint32_t wfi_ticks = s_analytics_wfi_ticks;
  uint32_t deepwfi_ticks = s_analytics_deepwfi_ticks;
  uint32_t deepsleep_ticks = s_analytics_deepsleep_ticks;

  RtcTicks now_ticks = rtc_get_ticks();
  uint32_t total_ticks = (uint32_t)(now_ticks - s_last_ticks);
  uint32_t running_ticks = total_ticks - wfi_ticks - deepwfi_ticks - deepsleep_ticks;

  char buf[160];
  snprintf(buf, sizeof(buf), "Run:       %"PRIu32" ticks (%"PRIu32" %%)",
           running_ticks, (running_ticks * 100) / total_ticks);
  prompt_send_response(buf);
  snprintf(buf, sizeof(buf), "WFI:       %"PRIu32" ticks (%"PRIu32" %%)",
           wfi_ticks, (wfi_ticks * 100) / total_ticks);
  prompt_send_response(buf);
  snprintf(buf, sizeof(buf), "Deep WFI:  %"PRIu32" ticks (%"PRIu32" %%)",
           deepwfi_ticks, (deepwfi_ticks * 100) / total_ticks);
  prompt_send_response(buf);
  snprintf(buf, sizeof(buf), "Deepsleep: %"PRIu32" ticks (%"PRIu32" %%)",
           deepsleep_ticks, (deepsleep_ticks * 100) / total_ticks);
  prompt_send_response(buf);
  snprintf(buf, sizeof(buf), "Tot:       %"PRIu32" ticks", total_ticks);
  prompt_send_response(buf);
}

void command_force_wfi(const char *arg) {
  if (arg[0] == '1') {
    s_force_wfi = true;
    prompt_send_response("WFI forced ON (deep WFI and deep sleep disabled)");
  } else {
    s_force_wfi = false;
    prompt_send_response("WFI forced OFF (deep WFI and deep sleep allowed)");
  }
}

void pbl_analytics_external_collect_cpu_stats(void) {
  uint32_t wfi_ticks = s_analytics_wfi_ticks;
  uint32_t deepwfi_ticks = s_analytics_deepwfi_ticks;
  uint32_t deepsleep_ticks = s_analytics_deepsleep_ticks;

  RtcTicks now_ticks = rtc_get_ticks();
  uint32_t total_ticks = (uint32_t)(now_ticks - s_last_ticks);
  uint32_t running_ticks = total_ticks - wfi_ticks - deepwfi_ticks - deepsleep_ticks;

  // Calculate percentages
  uint16_t running_pct = 0;
  uint16_t wfi_pct = 0;
  uint16_t deepwfi_pct = 0;
  uint16_t deepsleep_pct = 0;

  if (total_ticks > 0) {
    running_pct = (uint16_t)((running_ticks * 10000ULL) / total_ticks);
    wfi_pct = (uint16_t)((wfi_ticks * 10000ULL) / total_ticks);
    deepwfi_pct = (uint16_t)((deepwfi_ticks * 10000ULL) / total_ticks);
    deepsleep_pct = (uint16_t)((deepsleep_ticks * 10000ULL) / total_ticks);
  }

  // SF32LB52: sleep0 = WFI, sleep1 = Deep WFI, sleep2 = Deep sleep
  PBL_ANALYTICS_SET_UNSIGNED(cpu_running_pct, running_pct);
  PBL_ANALYTICS_SET_UNSIGNED(cpu_sleep0_pct, wfi_pct);
  PBL_ANALYTICS_SET_UNSIGNED(cpu_sleep1_pct, deepwfi_pct);
  PBL_ANALYTICS_SET_UNSIGNED(cpu_sleep2_pct, deepsleep_pct);
  PBL_ANALYTICS_SET_UNSIGNED(sifli_ipc_not_idle_count, s_analytics_ipc_not_idle_count);

  s_last_ticks = now_ticks;
  s_analytics_wfi_ticks = 0;
  s_analytics_deepwfi_ticks = 0;
  s_analytics_deepsleep_ticks = 0;
  s_analytics_ipc_not_idle_count = 0;
}
