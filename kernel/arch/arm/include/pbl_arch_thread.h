/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include <stdint.h>

//! MPU region registers written on every switch to the thread: four
//! (RBAR, RASR/RLAR) pairs for the configurable regions.
struct pbl_arch_thread {
  uint32_t mpu[8];
};
