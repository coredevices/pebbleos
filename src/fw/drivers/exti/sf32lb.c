/* SPDX-FileCopyrightText: 2025 SiFli Technologies(Nanjing) Co., Ltd */
/* SPDX-License-Identifier: Apache-2.0 */

#include <pbl/drivers/exti.h>

#include <stdbool.h>

#include "board/board.h"
#include "kernel/events.h"
#include <pbl/logging/logging.h>

PBL_LOG_MODULE_DEFINE(driver_exti_sf32lb, CONFIG_DRIVER_EXTI_LOG_LEVEL);

#define EXTI_MAX_GPIO1_PIN_NUM 16

typedef struct {
  uint32_t gpio_pin;
  ExtiHandlerCallback callback;
} ExtiHandlerConfig_t;

static ExtiHandlerConfig_t s_exti_gpio1_handler_configs[EXTI_MAX_GPIO1_PIN_NUM];

static GPIO_TypeDef *prv_gpio_get_instance(GPIO_TypeDef *hgpio, uint16_t gpio_pin,
                                           uint16_t *offset) {
  uint16_t inst_idx;
  GPIO_TypeDef *gpiox;

  HAL_ASSERT(gpio_pin < GPIO1_PIN_NUM);

  if (gpio_pin >= GPIO1_PIN_NUM) {
    return (GPIO_TypeDef *)NULL;
  }

  // There are many groups of similar registers in the GPIO, and because of register length limitations, up to 32 gpio can be operated in each group.
  inst_idx = gpio_pin >> 5;
  *offset = gpio_pin & 31;

  gpiox = (GPIO_TypeDef *)hgpio + inst_idx;

  return gpiox;
}

static void prv_insert_handler(GPIO_TypeDef *hgpio, uint8_t gpio_pin, ExtiHandlerCallback cb) {
  // Find the handler index for this pin
  uint8_t index = 0;
  while (index < EXTI_MAX_GPIO1_PIN_NUM &&
         s_exti_gpio1_handler_configs[index].callback != NULL) {
    index++;
  }
  if (index >= EXTI_MAX_GPIO1_PIN_NUM) {
    // No available slot
    return;
  }
  // Store the callback and index
  s_exti_gpio1_handler_configs[index].gpio_pin = gpio_pin;
  s_exti_gpio1_handler_configs[index].callback = cb;
}

void exti_configure_pin(ExtiConfig cfg, ExtiTrigger trigger, ExtiHandlerCallback cb) {
  GPIO_InitTypeDef init;
  int flags;

  init.Pin = cfg.gpio_pin;
  init.Pull = GPIO_NOPULL;

  switch (cfg.pull) {
    case GPIO_PuPd_UP:
      flags = PIN_PULLUP;
      break;
    case GPIO_PuPd_DOWN:
      flags = PIN_PULLDOWN;
      break;
    default:
      flags = PIN_NOPULL;
      break;
  }

  switch (trigger) {
    case ExtiTrigger_Rising:
      init.Mode = GPIO_MODE_IT_RISING;
      break;
    case ExtiTrigger_Falling:
      init.Mode = GPIO_MODE_IT_FALLING;
      break;
    case ExtiTrigger_RisingFalling:
      init.Mode = GPIO_MODE_IT_RISING_FALLING;
      break;
  }

  HAL_NVIC_DisableIRQ(GPIO1_IRQn);

  HAL_PIN_Set(PAD_PA00 + cfg.gpio_pin, GPIO_A0 + cfg.gpio_pin, flags, 1);
  HAL_GPIO_Init(cfg.peripheral, &init);

  prv_insert_handler(cfg.peripheral, cfg.gpio_pin, cb);

  HAL_NVIC_SetPriority(GPIO1_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(GPIO1_IRQn);
}

void exti_enable(ExtiConfig cfg) {
  uint16_t offset;
  GPIO_TypeDef *gpiox = prv_gpio_get_instance(cfg.peripheral, cfg.gpio_pin, &offset);
  gpiox->IESR = (1 << offset);
}

void exti_disable(ExtiConfig cfg) {
  uint16_t offset;
  GPIO_TypeDef *gpiox = prv_gpio_get_instance(cfg.peripheral, cfg.gpio_pin, &offset);
  gpiox->IECR = (1 << offset);
  gpiox->ISR = (1 << offset);
}

void HAL_GPIO_EXTI_Callback(GPIO_TypeDef *hgpio, uint16_t GPIO_Pin) {
  for (uint8_t index = 0; index < EXTI_MAX_GPIO1_PIN_NUM; index++) {
    if (s_exti_gpio1_handler_configs[index].callback != NULL &&
        s_exti_gpio1_handler_configs[index].gpio_pin == GPIO_Pin) {
      bool should_context_switch = false;

      s_exti_gpio1_handler_configs[index].callback(&should_context_switch);
      return;
    }
  }

  PBL_LOG_WRN("No handler found for GPIO pin %u", GPIO_Pin);
}

void GPIO1_IRQHandler(void) {
  HAL_GPIO_IRQHandler(hwp_gpio1);
}