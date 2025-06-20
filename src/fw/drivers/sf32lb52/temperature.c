/*
 * Copyright 2025 SiFli Technologies(Nanjing) Co., Ltd
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

#include "drivers/temperature.h"

#include "bf0_hal.h"
#include "board/board.h"
#include "console/prompt.h"
#include "system/logging.h"
#include "system/passert.h"
#include "kernel/util/delay.h"

#define SLOPE_NUM (2971)  // approximate slope molecule
#define SLOPE_DEN (40)    // approximate slope denominator
#define OFFSET (277539)
#define ROUND_ADD (SLOPE_DEN / 2)

void temperature_init(void) {
  HAL_RCC_EnableModule(RCC_MOD_TSEN);
  hwp_hpsys_cfg->ANAU_CR |= HPSYS_CFG_ANAU_CR_EN_BG;
}

static void prv_tsen_enable(TSEN_TypeDef *tsen) {
  tsen->TSEN_CTRL_REG &= ~TSEN_TSEN_CTRL_REG_ANAU_TSEN_RSTB;
  tsen->TSEN_CTRL_REG |= TSEN_TSEN_CTRL_REG_ANAU_TSEN_EN | TSEN_TSEN_CTRL_REG_ANAU_TSEN_PU;
  tsen->TSEN_CTRL_REG |= TSEN_TSEN_CTRL_REG_ANAU_TSEN_RSTB;
  delay_us(20);
  tsen->TSEN_CTRL_REG |= TSEN_TSEN_CTRL_REG_ANAU_TSEN_RUN;
}

static void prv_tsen_disable(TSEN_TypeDef *tsen) {
  tsen->TSEN_CTRL_REG &= ~(TSEN_TSEN_CTRL_REG_ANAU_TSEN_EN | TSEN_TSEN_CTRL_REG_ANAU_TSEN_PU);
}

int32_t temperature_read(void) {
  int32_t temp = 0;

  uint32_t count = 0;
  prv_tsen_enable(hwp_tsen);
  while ((hwp_tsen->TSEN_IRQ & TSEN_TSEN_IRQ_TSEN_IRSR) == 0) {
    HAL_Delay(1);
    count++;
    if (count > HAL_TSEN_MAX_DELAY) {
      temp = INT32_MIN;
      break;
    }
  }
  hwp_tsen->TSEN_IRQ |= TSEN_TSEN_IRQ_TSEN_ICR;
  if (temp != INT32_MIN) {
    // The Celsius conversion formula is: (DATA + 3000)/10100 * 749.2916 − 277.5391
    // In order to calculate milli-Celsius degrees, we can perform the following conversions:
    // COEF_NUM/COEF_DEN ≃ 749.2916/10100 × 1000
    // (DATA+3000)*COEF_NUM/COEF_DEN - OFFSET
    uint32_t raw = hwp_tsen->TSEN_RDATA;
    uint32_t D = raw + 3000;                   // D = DATA + 3000
    uint32_t num = D * SLOPE_NUM + ROUND_ADD;  // discard four, but treat five as whole
    uint32_t tmp = num / SLOPE_DEN;
    temp = tmp - OFFSET;
  }
  prv_tsen_disable(hwp_tsen);

  return temp;
}

void command_temperature_read(void) {
  char buffer[32];
  prompt_send_response_fmt(buffer, sizeof(buffer), "%" PRId32 " ", temperature_read());
}
