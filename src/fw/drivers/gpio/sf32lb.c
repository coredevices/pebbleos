/* SPDX-FileCopyrightText: 2025 SiFli Technologies(Nanjing) Co., Ltd */
/* SPDX-License-Identifier: Apache-2.0 */

#include <pbl/drivers/gpio.h>
#include "system/passert.h"
#include "board/board.h"

void gpio_output_init(const OutputConfig *pin_config, GPIOOType_TypeDef otype) {
  GPIO_InitTypeDef GPIO_InitStruct;

  HAL_RCC_EnableModule(RCC_MOD_GPIO1);

  GPIO_InitStruct.Pin = pin_config->gpio_pin;
  if (otype == GPIO_OType_OD) {
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  } else if (otype == GPIO_OType_PP) {
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT;
  } else {
    WTF;
  }
  HAL_PIN_Set(PAD_PA00 + pin_config->gpio_pin, GPIO_A0 + pin_config->gpio_pin, PIN_NOPULL, 1); 
  GPIO_InitStruct.Pull = GPIO_NOPULL;

  HAL_GPIO_Init(pin_config->gpio, &GPIO_InitStruct);
}

void gpio_input_init(const InputConfig *input_cfg) {
  gpio_input_init_pull_up_down(input_cfg, GPIO_PuPd_NOPULL);
}

void gpio_input_init_pull_up_down(const InputConfig *input_cfg, GPIOPuPd_TypeDef pupd) {
  GPIO_InitTypeDef GPIO_InitStruct;

  HAL_RCC_EnableModule(RCC_MOD_GPIO1);

  GPIO_InitStruct.Pin = input_cfg->gpio_pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;

  int flag = 0;
  /* pullup/down is handled in pinmux hal. */
  if (pupd == GPIO_PuPd_UP) {
    flag = PIN_PULLUP;
  } else if (pupd == GPIO_PuPd_DOWN) {
    flag = PIN_PULLDOWN;
  } else {
    flag = PIN_NOPULL;
  }

  HAL_PIN_Set(PAD_PA00 + input_cfg->gpio_pin, GPIO_A0 + input_cfg->gpio_pin, flag, 1);
  HAL_GPIO_Init(input_cfg->gpio, &GPIO_InitStruct);
}

bool gpio_input_read(const InputConfig *input_cfg) {
  bool value = HAL_GPIO_ReadPin(input_cfg->gpio, input_cfg->gpio_pin);
  return value;
}

void gpio_output_set(const OutputConfig *pin_config, bool asserted) {
  if (!pin_config->active_high) {
    asserted = !asserted;
  }

  HAL_GPIO_WritePin(pin_config->gpio, pin_config->gpio_pin, asserted);
}