/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "pbl/kernel/sem.h"
#include "board/board.h"


#include <stdbool.h>
#include <stdint.h>

#ifdef CONFIG_SOC_SF32LB52
#include "bf0_hal_dma.h"
#include "bf0_hal_mpi.h"
#endif

#define QSPI_NUM_DATA_PINS (4)

typedef struct QSPIPortState {
#ifdef CONFIG_SOC_NRF52
  struct pbl_sem sem;
  bool initialized;
#elif defined(CONFIG_SOC_SF32LB52)
  QSPI_FLASH_CTX_T ctx;
  DMA_HandleTypeDef hdma;
  qspi_configure_t cfg;
  struct dma_config dma;
  uint32_t t_enter_deep_us;
  uint32_t t_exit_deep_us;
  bool initialized;
#else
  struct pbl_sem dma_semaphore;
  int use_count;
#endif
} QSPIPortState;

typedef const struct QSPIPort {
  QSPIPortState *state;
#ifdef CONFIG_SOC_NRF52
  uint32_t clk_freq_hz;
  uint32_t cs_gpio;
  uint32_t clk_gpio;
  uint32_t data_gpio[QSPI_NUM_DATA_PINS];
#elif defined(CONFIG_SOC_SF32LB52)
  uint16_t clk_div;
#endif
} QSPIPort;

//! Initialize the QSPI peripheral, the pins, and the DMA
void qspi_init(QSPIPort *dev, uint32_t flash_size);
