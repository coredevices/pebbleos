/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "memory_layout.h"

#include "logging/logging_private.h"
#include "pbl/util/math.h"
#include "pbl/util/size.h"
#include "pbl/util/string.h"

#include <inttypes.h>
#include <string.h>


static const char* const MEMORY_REGION_NAMES[] = {
  // Keep the four RESERVED entries in lockstep with MemoryRegion_Reserved*
  // in memory_layout.h. SiFli's fifth region (mailbox) overlaps with the
  // "UNPRIV_FLASH" slot at index 4 -- Pebble never programs that index on
  // SF32LB52, so the cosmetic label is the only casualty.
#ifdef CONFIG_SOC_SF32LB52
  "RESERVED0",
  "RESERVED1",
  "RESERVED2",
  "RESERVED3",
#endif
  "UNPRIV_FLASH",
  "UNPRIV_RO_BSS",
  "UNPRIV_RO_DATA",
  "ISR_STACK_GUARD",
  "Task Specific 1",
  "Task Specific 2",
  "Task Specific 3",
  "Task Specific 4"
};


static const char *prv_permissions_str(MpuPermissions p) {
  switch (p) {
    case MpuPermissions_NoAccess:      return "NoAccess";
    case MpuPermissions_PrivRW:        return "PrivRW";
    case MpuPermissions_PrivRW_UserRO: return "PrivRW_UserRO";
    case MpuPermissions_PrivRW_UserRW: return "PrivRW_UserRW";
    case MpuPermissions_PrivRO:        return "PrivRO";
    case MpuPermissions_PrivRO_UserRO: return "PrivRO_UserRO";
    case MpuPermissionsCount:          break;
  }
  return "?";
}

void memory_layout_dump_mpu_regions_to_dbgserial(void) {
  char buffer[90];

  for (size_t i = 0; i < ARRAY_LENGTH(MEMORY_REGION_NAMES); ++i) {
    MpuRegion region = mpu_get_region(i);

    if (!region.enabled) {
      PBL_LOG_FROM_FAULT_HANDLER_FMT(buffer, sizeof(buffer), "%u Not enabled", i);
      continue;
    }

    PBL_LOG_FROM_FAULT_HANDLER_FMT(
        buffer, sizeof(buffer),
        "%u < %-22s>: Addr %p Size 0x%08"PRIx32" %s Perms: %s",
        i, MEMORY_REGION_NAMES[i], (void*) region.base_address, region.size,
        region.executable ? "X" : "-", prv_permissions_str(region.permissions));
  }
}

#ifdef UNITTEST
static const uint32_t __privileged_functions_start__ = 0;
static const uint32_t __privileged_functions_size__ = 0;
static const uint32_t __unpriv_ro_bss_start__ = 0;
static const uint32_t __unpriv_ro_bss_size__ = 0;
static const uint32_t __isr_stack_start__ = 0;
static const uint32_t __stack_guard_size__ = 0;

static const uint32_t __APP_RAM__ = 0;
static const uint32_t __APP_RAM_size__ = 0;
static const uint32_t __WORKER_RAM__ = 0;
static const uint32_t __WORKER_RAM_size__ = 0;

static const uint32_t __FLASH_start__ = 0;
static const uint32_t __FLASH_size__ = 0;

static const uint32_t __kernel_main_stack_start__ = 0;
static const uint32_t __kernel_bg_stack_start__ = 0;
#else
extern const uint32_t __privileged_functions_start__[];
extern const uint32_t __privileged_functions_size__[];
extern const uint32_t __unpriv_ro_bss_start__[];
extern const uint32_t __unpriv_ro_bss_size__[];
extern const uint32_t __isr_stack_start__[];
extern const uint32_t __stack_guard_size__[];

extern const uint32_t __APP_RAM__[];
extern const uint32_t __APP_RAM_size__[];
extern const uint32_t __WORKER_RAM__[];
extern const uint32_t __WORKER_RAM_size__[];

extern const uint32_t __FLASH_start__[];
extern const uint32_t __FLASH_size__[];

extern const uint32_t __kernel_main_stack_start__[];
extern const uint32_t __kernel_bg_stack_start__[];
#endif

// Kernel read only RAM. Parts of RAM that it's kosher for unprivileged apps to read
static const MpuRegion s_readonly_bss_region = {
  .region_num = MemoryRegion_ReadOnlyBss,
  .enabled = true,
  .base_address = (uint32_t) __unpriv_ro_bss_start__,
  .size = (uint32_t) __unpriv_ro_bss_size__,
  .cache_policy = MpuCachePolicy_WriteBackWriteAllocate,
  .permissions = MpuPermissions_PrivRW_UserRO,
};

#ifndef CONFIG_SOC_SF32LB52
// ISR stack guard
static const MpuRegion s_isr_stack_guard_region = {
  .region_num = MemoryRegion_IsrStackGuard,
  .enabled = true,
  .base_address = (uint32_t) __isr_stack_start__,
  .size = (uint32_t) __stack_guard_size__,
  .cache_policy = MpuCachePolicy_NotCacheable,
  .permissions = MpuPermissions_NoAccess,
};
#endif

static const MpuRegion s_app_stack_guard_region = {
  .region_num = MemoryRegion_TaskStackGuard,
  .enabled = true,
  .base_address = (uint32_t) __APP_RAM__,
  .size = (uint32_t) __stack_guard_size__,
  .cache_policy = MpuCachePolicy_NotCacheable,
  .permissions = MpuPermissions_NoAccess,
};

static const MpuRegion s_worker_stack_guard_region = {
  .region_num = MemoryRegion_TaskStackGuard,
  .enabled = true,
  .base_address = (uint32_t) __WORKER_RAM__,
  .size = (uint32_t) __stack_guard_size__,
  .cache_policy = MpuCachePolicy_NotCacheable,
  .permissions = MpuPermissions_NoAccess,
};

static const MpuRegion s_app_region = {
  .region_num = MemoryRegion_AppRAM,
  .enabled = true,
  // App process .text is relocated into App RAM, so the App task needs
  // to be able to execute from this region.
  .executable = true,
  .base_address = (uintptr_t) __APP_RAM__,
  .size = (uint32_t) __APP_RAM_size__,
  .cache_policy = MpuCachePolicy_WriteBackWriteAllocate,
  .permissions = MpuPermissions_PrivRW,
};

static const MpuRegion s_worker_region = {
  .region_num = MemoryRegion_WorkerRAM,
  .enabled = true,
  // Worker process .text is relocated into Worker RAM.
  .executable = true,
  .base_address = (uintptr_t) __WORKER_RAM__,
  .size = (uint32_t) __WORKER_RAM_size__,
  .cache_policy = MpuCachePolicy_WriteBackWriteAllocate,
  .permissions = MpuPermissions_PrivRW,
};

static const MpuRegion s_microflash_region = {
  .region_num = MemoryRegion_Flash,
  .enabled = true,
  // The firmware itself runs from flash.
  .executable = true,
  .base_address = (uint32_t) __FLASH_start__,
  .size = (uint32_t) __FLASH_size__,
  .cache_policy = MpuCachePolicy_WriteThrough,
  .permissions = MpuPermissions_PrivRO_UserRO,
};

static const MpuRegion s_kernel_main_stack_guard_region = {
  .region_num = MemoryRegion_TaskStackGuard,
  .enabled = true,
  .base_address = (uint32_t) __kernel_main_stack_start__,
  .size = (uint32_t) __stack_guard_size__,
  .cache_policy = MpuCachePolicy_NotCacheable,
  .permissions = MpuPermissions_NoAccess,
};

static const MpuRegion s_kernel_bg_stack_guard_region = {
  .region_num = MemoryRegion_TaskStackGuard,
  .enabled = true,
  .base_address = (uint32_t) __kernel_bg_stack_start__,
  .size = (uint32_t) __stack_guard_size__,
  .cache_policy = MpuCachePolicy_NotCacheable,
  .permissions = MpuPermissions_NoAccess,
};

void memory_layout_setup_mpu(void) {
  // Flash parts...
  // Read only for executing code and loading data out of.

  // Unprivileged flash, by default anyone can read any part of flash.
  // On SF32LB52 the SiFli HAL already programs a user-RO executable region
  // covering __FLASH_start__..(__FLASH_start__ + __FLASH_size__ - 1) in
  // SystemInit(), so we skip our own.
#ifndef CONFIG_SOC_SF32LB52
  mpu_set_region(&s_microflash_region);
#endif

  // RAM parts
  // The background memory map only allows privileged access. We need to add aditional regions to
  // enable access to unprivileged code.

  mpu_set_region(&s_readonly_bss_region);
  // ARMv8-M tracks per-task stack overflow via PSPLIM in the FreeRTOS CM33
  // port, so the dedicated ISR stack guard region (no-access) is redundant
  // and the AP encoding can't even express "no access" precisely. Skip it
  // on SF32LB52 to reclaim the slot.
#ifndef CONFIG_SOC_SF32LB52
  mpu_set_region(&s_isr_stack_guard_region);
#endif

  mpu_enable();
}

const MpuRegion* memory_layout_get_app_region(void) {
  return &s_app_region;
}

const MpuRegion* memory_layout_get_readonly_bss_region(void) {
  return &s_readonly_bss_region;
}

const MpuRegion* memory_layout_get_app_stack_guard_region(void) {
  return &s_app_stack_guard_region;
}

const MpuRegion* memory_layout_get_worker_region(void) {
  return &s_worker_region;
}

const MpuRegion* memory_layout_get_worker_stack_guard_region(void) {
  return &s_worker_stack_guard_region;
}

const MpuRegion* memory_layout_get_microflash_region(void) {
  return &s_microflash_region;
}

const MpuRegion* memory_layout_get_kernel_main_stack_guard_region(void) {
  return &s_kernel_main_stack_guard_region;
}

const MpuRegion* memory_layout_get_kernel_bg_stack_guard_region(void) {
  return &s_kernel_bg_stack_guard_region;
}

bool memory_layout_is_pointer_in_region(const MpuRegion *region, const void *ptr) {
  uintptr_t p = (uintptr_t) ptr;
  return (p >= region->base_address && p < (region->base_address + region->size));
}

bool memory_layout_is_buffer_in_region(const MpuRegion *region, const void *buf, size_t length) {
  return memory_layout_is_pointer_in_region(region, buf) && memory_layout_is_pointer_in_region(region, (char *)buf + length - 1);
}

bool memory_layout_is_cstring_in_region(const MpuRegion *region, const char *str, size_t max_length) {
  uintptr_t region_end = region->base_address + region->size;

  if ((uintptr_t) str < region->base_address || (uintptr_t) str >= region_end) {
    return false;
  }

  const char *str_max_end = MIN((const char*) region_end, str + max_length);

  size_t str_len = strnlen(str, str_max_end - str);

  if (str[str_len] != 0) {
    // No null between here and the end of the memory region.
    return false;
  }

  return true;
}
