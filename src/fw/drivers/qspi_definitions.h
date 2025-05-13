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

#pragma once

#include "board/board.h"

#include "freertos_types.h"

#include <stdbool.h>
#include <stdint.h>

#if MICRO_FAMILY_SF32LB52
#include "bf0_hal_dma.h"
#include "bf0_hal_mpi.h"
#endif

#define QSPI_NUM_DATA_PINS (4)

typedef struct QSPIPortState {
#if MICRO_FAMILY_SF32LB52
  QSPI_FLASH_CTX_T ctx;
  DMA_HandleTypeDef hdma;
#else
  SemaphoreHandle_t dma_semaphore;
  int use_count;
#endif
} QSPIPortState;

typedef const struct QSPIPort {
  QSPIPortState *state;
#if MICRO_FAMILY_NRF5
  uint16_t auto_polling_interval;
  uint32_t cs_gpio;
  uint32_t clk_gpio;
  uint32_t data_gpio[QSPI_NUM_DATA_PINS];
#elif MICRO_FAMILY_SF32LB52
  qspi_configure_t cfg;
  uint16_t clk_div;
  struct dma_config dma;
#else
  uint16_t auto_polling_interval;
  uint32_t clock_speed_hz;
  uint32_t clock_ctrl;
  AfConfig cs_gpio;
  AfConfig clk_gpio;
  AfConfig data_gpio[QSPI_NUM_DATA_PINS];
  DMARequest *dma;
#endif
} QSPIPort;

//! Initialize the QSPI peripheral, the pins, and the DMA
void qspi_init(QSPIPort *dev, uint32_t flash_size);
