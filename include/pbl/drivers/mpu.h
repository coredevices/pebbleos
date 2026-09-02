/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include <stdint.h>
#include <stdbool.h>


typedef enum MpuCachePolicy {
  MpuCachePolicy_NotCacheable,
  MpuCachePolicy_WriteThrough,
  MpuCachePolicy_WriteBackWriteAllocate,
  MpuCachePolicy_WriteBackNoWriteAllocate,
} MpuCachePolicy;

// Symbolic access-permission set. The backends map these to the hardware AP
// encoding (3 bits on ARMv7-M, 2 bits on ARMv8-M). Two values are degraded
// on ARMv8-M because the two-bit AP field cannot express them precisely:
//   - NoAccess: ARMv8-M has no "deny everything" encoding; the closest
//     match is PrivRO, which still allows privileged reads. Don't rely on
//     this for thread stack-overflow detection -- use PSPLIM instead.
//   - PrivRW_UserRO: ARMv8-M cannot split write access by privilege; this
//     is aliased to PrivRW_UserRW (user gains write access too).
typedef enum MpuPermissions {
  MpuPermissions_NoAccess,
  MpuPermissions_PrivRW,
  MpuPermissions_PrivRW_UserRO,
  MpuPermissions_PrivRW_UserRW,
  MpuPermissions_PrivRO,
  MpuPermissions_PrivRO_UserRO,
  MpuPermissionsCount,
} MpuPermissions;

// Describes a memory region by its real (base, size). The ARMv7-M backend
// internally rounds up to a power-of-two block and computes the subregion
// mask required by the hardware; the ARMv8-M backend programs the limit
// register directly. Callers never see these implementation details.
typedef struct MpuRegion {
  uint8_t region_num:4;
  bool enabled:1;
  // Allow instruction fetch from this region. Maps to XN=0 in the
  // hardware encoding (RASR.XN on ARMv7-M, RBAR.XN on ARMv8-M). The
  // default (false) sets XN=1 -- safer, since most regions hold data
  // and shouldn't be executable. Flash, App RAM, Worker RAM and any
  // text section relocated to RAM need to opt in.
  bool executable:1;
  uintptr_t base_address;
  uint32_t size;
  MpuCachePolicy cache_policy;
  MpuPermissions permissions;
} MpuRegion;

void mpu_enable(void);

void mpu_disable(void);

void mpu_set_region(const MpuRegion* region);

MpuRegion mpu_get_region(int region_num);

void mpu_get_register_settings(const MpuRegion* region, uint32_t *base_address_reg,
                               uint32_t *attributes_reg);


bool mpu_memory_is_cachable(const void *addr);

void mpu_init_region_from_region(MpuRegion *copy, const MpuRegion *from, bool allow_user_access);
