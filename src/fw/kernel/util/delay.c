/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "delay.h"
#include "pbl/util/attributes.h"

#ifdef CONFIG_SOC_NRF52
#include <drivers/nrfx_common.h>
#include <soc/nrfx_coredep.h>
#elif defined(CONFIG_SOC_SF32LB52)
#include <bf0_hal.h>
#endif

#ifdef CONFIG_SOC_NRF52
void NOINLINE delay_us(uint32_t us) {
  nrfx_coredep_delay_us(us);
}

void delay_init(void) {
}

#elif defined(CONFIG_SOC_SF32LB52)
void NOINLINE delay_us(uint32_t us) {
  HAL_Delay_us(us);
}

void delay_init(void) {
}

#elif defined(CONFIG_QEMU)
#include <cmsis_core.h>

void NOINLINE delay_us(uint32_t us) {
  // Use DWT cycle counter for accurate delays
  uint32_t cycles = us * (SystemCoreClock / 1000000);
  uint32_t start = DWT->CYCCNT;
  while ((DWT->CYCCNT - start) < cycles) {}
}

void delay_init(void) {
  // Enable DWT cycle counter
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}
#endif
