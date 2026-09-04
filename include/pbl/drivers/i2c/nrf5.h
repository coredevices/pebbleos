/* SPDX-FileCopyrightText: 2025 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-variable"
#include <nrfx_twim.h>
#pragma GCC diagnostic pop

typedef struct I2CBusHal {
  nrfx_twim_t twim;
  nrf_twim_frequency_t frequency; ///< Bus clock speed
} I2CBusHal;
