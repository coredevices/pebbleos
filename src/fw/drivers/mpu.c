/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include <pbl/drivers/mpu.h>

#include "pbl/mcu/cache.h"
#include "system/passert.h"


#include <cmsis_core.h>

extern const uint32_t __SRAM_size__[];
#if !defined(SRAM_BASE)
#if defined(CONFIG_SOC_NRF52)
#include <drivers/nrfx_common.h>
#define SRAM_BASE (0x20000000UL)
#elif defined(CONFIG_SOC_SF32LB52)
#define SRAM_BASE (0x20000000UL)
#endif
#endif
#define SRAM_END (SRAM_BASE + (uint32_t)__SRAM_size__)

void mpu_disable(void) {
  ARM_MPU_Disable();
}

bool mpu_memory_is_cachable(const void *addr) {
  if (!dcache_is_enabled()) {
    return false;
  }
  // TODO PBL-37601: We're assuming only SRAM is cachable for now for simplicity sake. We should
  // account for MPU configuration and also the fact that memory-mapped QSPI access goes through the
  // cache.
  return ((uint32_t)addr >= SRAM_BASE) && ((uint32_t)addr < SRAM_END);
}

void mpu_init_region_from_region(MpuRegion *copy, const MpuRegion *from, bool allow_user_access) {
  // Caller-side invariant: `from` is a PrivRW region (App/Worker RAM).
  // Toggle user RW based on which task is about to run.
  PBL_ASSERTN(from->permissions == MpuPermissions_PrivRW);
  *copy = *from;
  copy->permissions = allow_user_access ? MpuPermissions_PrivRW_UserRW
                                        : MpuPermissions_PrivRW;
}
