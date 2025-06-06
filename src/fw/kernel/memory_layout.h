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

#include "drivers/mpu.h"

#include <stddef.h>

#define KERNEL_READONLY_DATA SECTION(".kernel_unpriv_ro_bss")

enum MemoryRegionAssignments {
  // FIXME(SF32LB52): system_bf0_ap.c uses now up to 4 regions as MPU is not fully implemented.
#ifdef MICRO_FAMILY_SF32LB52
  MemoryRegion_Reserved0,
  MemoryRegion_Reserved1,
  MemoryRegion_Reserved2,
  MemoryRegion_Reserved3,
#endif
  MemoryRegion_Flash,
  MemoryRegion_ReadOnlyBss,
  MemoryRegion_ReadOnlyData,
  MemoryRegion_IsrStackGuard,
  MemoryRegion_AppRAM,
  MemoryRegion_WorkerRAM,
  MemoryRegion_TaskStackGuard,
  MemoryRegion_Task4
};

void memory_layout_dump_mpu_regions_to_dbgserial(void);

void memory_layout_setup_mpu(void);

const MpuRegion* memory_layout_get_app_region(void);
const MpuRegion* memory_layout_get_app_stack_guard_region(void);

const MpuRegion* memory_layout_get_readonly_bss_region(void);
const MpuRegion* memory_layout_get_microflash_region(void);

const MpuRegion* memory_layout_get_worker_region(void);
const MpuRegion* memory_layout_get_worker_stack_guard_region(void);

const MpuRegion* memory_layout_get_kernel_main_stack_guard_region(void);
const MpuRegion* memory_layout_get_kernel_bg_stack_guard_region(void);

bool memory_layout_is_pointer_in_region(const MpuRegion *region, const void *ptr);
bool memory_layout_is_buffer_in_region(const MpuRegion *region, const void *buf, size_t length);
bool memory_layout_is_cstring_in_region(const MpuRegion *region, const char *str, size_t max_length);
