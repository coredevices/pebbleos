/* SPDX-FileCopyrightText: 2025 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include <pbl/drivers/exti.h>

#include "board/board.h"
#include "kernel/events.h"
#include "system/passert.h"

#include <nrfx.h>

#include <stdbool.h>

// NRF5 emulates EXTI using GPIOTE

static void prv_exti_handler(nrfx_gpiote_pin_t pin, nrfx_gpiote_trigger_t trigger, void *p_context) {
  ExtiHandlerCallback cb = (ExtiHandlerCallback) p_context;
  
  bool should_context_switch = false;
  cb(&should_context_switch);
  
  portEND_SWITCHING_ISR(should_context_switch);
}

void exti_configure_pin(ExtiConfig cfg, ExtiTrigger trigger, ExtiHandlerCallback cb) {
  nrfx_err_t err;
  if (!nrfx_gpiote_init_check(&cfg.peripheral)) {
    err = nrfx_gpiote_init(&cfg.peripheral, NRFX_GPIOTE_DEFAULT_CONFIG_IRQ_PRIORITY);
    PBL_ASSERTN(err == NRFX_SUCCESS);
  }
  uint8_t channel = cfg.channel;
  nrfx_gpiote_trigger_config_t tcfg = {
    .trigger = trigger == ExtiTrigger_Rising ? NRFX_GPIOTE_TRIGGER_LOTOHI :
               trigger == ExtiTrigger_Falling ? NRFX_GPIOTE_TRIGGER_HITOLO :
               trigger == ExtiTrigger_RisingFalling ? NRFX_GPIOTE_TRIGGER_TOGGLE :
               NRFX_GPIOTE_TRIGGER_NONE,
    .p_in_channel = &channel
  };
  nrfx_gpiote_handler_config_t hcfg = {
    .handler = prv_exti_handler,
    .p_context = cb
  };
  nrfx_gpiote_input_pin_config_t pcfg = {
    .p_pull_config = NULL,
    .p_trigger_config = &tcfg,
    .p_handler_config = &hcfg
  };
  
  err = nrfx_gpiote_input_configure(&cfg.peripheral, cfg.gpio_pin, &pcfg);
  PBL_ASSERTN(err == NRFX_SUCCESS);
  
  nrfx_gpiote_trigger_disable(&cfg.peripheral, cfg.gpio_pin);
}

void exti_enable(ExtiConfig cfg) {
  nrfx_gpiote_trigger_enable(&cfg.peripheral, cfg.gpio_pin, true /* int_enable */);
}

void exti_disable(ExtiConfig cfg) {
  nrfx_gpiote_trigger_disable(&cfg.peripheral, cfg.gpio_pin);
}

