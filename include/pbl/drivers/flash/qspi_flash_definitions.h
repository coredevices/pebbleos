/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "board/board.h"
#include <pbl/drivers/qspi.h>
#include "qspi_flash_part_definitions.h"

typedef struct QSPIFlashState {
  QSPIFlashPart *part;
  bool coredump_mode;
  bool fast_read_ddr_enabled;
} QSPIFlashState;

typedef enum QSPIFlashReadMode {
  QSPI_FLASH_READ_FASTREAD,
  QSPI_FLASH_READ_READ2O,
  QSPI_FLASH_READ_READ2IO,
  QSPI_FLASH_READ_READ4O,
  QSPI_FLASH_READ_READ4IO,
} QSPIFlashReadMode;

typedef enum QSPIFlashWriteMode {
  QSPI_FLASH_WRITE_PP,
  QSPI_FLASH_WRITE_PP2O,
  QSPI_FLASH_WRITE_PP4O,
  QSPI_FLASH_WRITE_PP4IO,
} QSPIFlashWriteMode;

typedef const struct QSPIFlash {
  QSPIFlashState *state;
  QSPIPort *qspi;
  bool default_fast_read_ddr_enabled;
  QSPIFlashReadMode read_mode;
  QSPIFlashWriteMode write_mode;
  OutputConfig reset_gpio;
} QSPIFlash;
