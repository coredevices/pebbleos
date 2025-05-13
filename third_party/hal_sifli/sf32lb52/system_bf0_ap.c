/*
 * Copyright 2025 Core Devices LLC
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

#include "bf0_hal.h"
#include "register.h"

#define DCACHE_SIZE 16384
#define ICACHE_SIZE (DCACHE_SIZE << 1)

#if defined(__VTOR_PRESENT) && (__VTOR_PRESENT == 1U)
uint32_t __Vectors;
#endif

uint32_t SystemCoreClock = 48000000UL;

void SystemCoreClockUpdate(void) {}

enum {
  ATTR_CODE_IDX,
  ATTR_RAM_IDX,
  ATTR_DEVICE_IDX,
};

#define ATTR_CODE ARM_MPU_ATTR(ARM_MPU_ATTR_MEMORY_(0, 0, 1, 0), ARM_MPU_ATTR_MEMORY_(0, 0, 1, 0))
#define ATTR_RAM ARM_MPU_ATTR(ARM_MPU_ATTR_NON_CACHEABLE, ARM_MPU_ATTR_NON_CACHEABLE)
#define ATTR_DEVICE ARM_MPU_ATTR(ARM_MPU_ATTR_DEVICE, ARM_MPU_ATTR_DEVICE_nGnRnE)

// FIXME(SF32LB52): ARMv8 MPU support is not complete, so for now, configure
// the MPU here as needed by the system to run.
static void prv_mpu_config(void) {
  uint32_t rbar, rlar;

  SCB_InvalidateDCache();
  SCB_InvalidateICache();

  ARM_MPU_Disable();

  for (uint8_t i = 0U; i < MPU_REGION_NUM; i++) {
    ARM_MPU_ClrRegion(i);
  }

  ARM_MPU_SetMemAttr(ATTR_CODE_IDX, ATTR_CODE);
  ARM_MPU_SetMemAttr(ATTR_RAM_IDX, ATTR_RAM);
  ARM_MPU_SetMemAttr(ATTR_DEVICE_IDX, ATTR_DEVICE);

  // PSRAM and FLASH2, region 1
  // Non-shareable, RO, any privilege, executable
  rbar = ARM_MPU_RBAR(0x10000000, ARM_MPU_SH_NON, 1, 1, 0);
  rlar = ARM_MPU_RLAR(0x1fffffff, ATTR_CODE_IDX);
  ARM_MPU_SetRegion(0U, rbar, rlar);

  // Peripheral space
  // Non-shareable, RW, any privilege, non-executable
  rbar = ARM_MPU_RBAR(0x40000000, ARM_MPU_SH_NON, 0, 1, 1);
  rlar = ARM_MPU_RLAR(0x5fffffff, ATTR_DEVICE_IDX);
  ARM_MPU_SetRegion(1U, rbar, rlar);

  // hpsys ram
  // Non-shareable, RW, any privilege, executable
  rbar = ARM_MPU_RBAR(0x20000000, ARM_MPU_SH_NON, 0, 1, 0);
  rlar = ARM_MPU_RLAR(0x2027ffff, ATTR_RAM_IDX);
  ARM_MPU_SetRegion(2U, rbar, rlar);

  // lpsys ram
  // Non-shareable, RW, any privilege, executable
  rbar = ARM_MPU_RBAR(0x203fc000, ARM_MPU_SH_NON, 0, 1, 0);
  rlar = ARM_MPU_RLAR(0x204fffff, ATTR_RAM_IDX);
  ARM_MPU_SetRegion(3U, rbar, rlar);

  ARM_MPU_Enable(MPU_CTRL_HFNMIENA_Msk);
}

int mpu_dcache_invalidate(void *data, uint32_t size) {
  int r = 0;

  if (IS_DCACHED_RAM(data)) {
    if (size > DCACHE_SIZE) {
      SCB_InvalidateDCache();
      r = 1;
    } else
      SCB_InvalidateDCache_by_Addr(data, size);
  }

  return r;
}

int mpu_icache_invalidate(void *data, uint32_t size) {
  int r = 0;

  if (IS_DCACHED_RAM(data)) {
    if (size > ICACHE_SIZE) {
      SCB_InvalidateICache();
      r = 1;
    } else
      SCB_InvalidateICache_by_Addr(data, size);
  }

  return r;
}

pm_power_on_mode_t SystemPowerOnModeGet(void) { return PM_COLD_BOOT; }

void SystemInit(void) {
#if defined(__VTOR_PRESENT) && (__VTOR_PRESENT == 1U)
  SCB->VTOR = (uint32_t)&__Vectors;
#endif

  // enable CP0/CP1/CP2 Full Access
  SCB->CPACR |= (3U << (0U * 2U)) | (3U << (1U * 2U)) | (3U << (2U * 2U));

#if defined(__FPU_USED) && (__FPU_USED == 1U)
  SCB->CPACR |= ((3U << 10U * 2U) | // enable CP10 Full Access
                 (3U << 11U * 2U)); // enable CP11 Full Access
#endif

  prv_mpu_config();

  SCB_EnableICache();
  SCB_EnableDCache();
}