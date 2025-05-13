/*
 * Copyright 2024 Google LLC
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "delay.h"
#include "util/attributes.h"
#include "util/units.h"

#define STM32F2_COMPATIBLE
#define STM32F4_COMPATIBLE
#define STM32F7_COMPATIBLE
#define NRF5_COMPATIBLE
#define SF32LB_COMPATIBLE
#include <mcu.h>

#include <inttypes.h>

#if MICRO_FAMILY_NRF5

#include <drivers/nrfx_common.h>
#include <soc/nrfx_coredep.h>

void NOINLINE delay_us(uint32_t us) {
  nrfx_coredep_delay_us(us);
}

void delay_init(void) {
}

#elif MICRO_FAMILY_SF32LB

void NOINLINE delay_us(uint32_t us) {
  HAL_Delay_us(us);
}

void delay_init(void) {
}

#else

#if MICRO_FAMILY_STM32F7
# define INSTRUCTIONS_PER_LOOP   (1)
#elif MICRO_FAMILY_STM32F2 || MICRO_FAMILY_STM32F4
# define INSTRUCTIONS_PER_LOOP   (3)
#else
# error "Unexpected micro family"
#endif

static uint32_t s_loops_per_us = 0;

void NOINLINE delay_us(uint32_t us) {

  uint32_t delay_loops = us * s_loops_per_us;

  __asm volatile (
      "spinloop:                             \n"
      "  subs %[delay_loops], #1             \n"
      "  bne spinloop                        \n"
      : [delay_loops] "+r" (delay_loops) // read-write operand
      :
      : "cc"
       );
}

void delay_init(void) {
  // The loop above consists of 2 instructions (output of arm-none-eabi-objdump -d
  // delay.X.o):
  //
  //  subs  r0, #1
  //  bne.w 4 <spinloop>
  //
  // Subtract consumes 1 cycle & the conditional branch consumes 1 + P (pipeline fill delay,
  // 1-3 cycles) if the branch is taken, or 1 if not taken. For this situation, it appears that P=1
  // on the STM32F2/F4, so the loop takes 3 and 2 cycles respectively. The Cortex-M7 (STM32F7) has a
  // superscalar dual-issue architecture which allows for 1-cycle loops (including the subtract).
  //
  // @ 64MHz 1 instructions is ~15.6ns which translates to the previously measured 47ns for one loop
  // Thus we can derive that to get a duration of 1µs from an arbitrary clock frequency the count
  // value needs to be:
  //    count = 1e-6 / (1/F * 3) where F is the core clock frequency
  //
  // An additional note is that delay_us is always executed from flash. The
  // instruction cache in the cortex M3 & M4 cores is pretty good at saving
  // instructions with simple branches which means we don't stall on flash
  // reads after the first loop. Counterintuitively, executing from SRAM
  // actually adds a extra delay cycle on instruction fetches and can be
  // stalled if peripherals are doing DMAs. (See PBL-22265 for more details)

  RCC_ClocksTypeDef clocks;
  RCC_GetClocksFreq(&clocks);
  // Get the frequency in MHz so we don't overflow a uint32_t.
  const uint32_t frequency_mhz = clocks.HCLK_Frequency / MHZ_TO_HZ(1);
  const uint32_t clock_period_ps = PS_PER_US / frequency_mhz;
  s_loops_per_us = PS_PER_US / (clock_period_ps * INSTRUCTIONS_PER_LOOP);

  // we always want to delay for more than the time specified so round up
  // if the numbers don't divide evenly
  if ((PS_PER_US % (clock_period_ps * INSTRUCTIONS_PER_LOOP)) != 0) {
    s_loops_per_us += 1;
  }
}

#endif
