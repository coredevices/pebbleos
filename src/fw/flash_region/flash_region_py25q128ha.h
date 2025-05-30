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

#pragma once

#define PAGE_SIZE_BYTES (0x100)

#define SECTOR_SIZE_BYTES (0x10000)
#define SECTOR_ADDR_MASK (~(SECTOR_SIZE_BYTES - 1))

#define SUBSECTOR_SIZE_BYTES (0x1000)
#define SUBSECTOR_ADDR_MASK (~(SUBSECTOR_SIZE_BYTES - 1))

#define FLASH_REGION_BASE_ADDRESS 0x12000000

// A bit of preprocessor magic to help with automatically calculating flash region addresses
//////////////////////////////////////////////////////////////////////////////

#define FLASH_REGION_DEF(MACRO, arg)                                                        \
  MACRO(PTABLE,                  0x0010000 /*    64K */, arg) /* 0x12000000 - 0x1200FFFF */ \
  MACRO(BOOTLOADER,              0x0010000 /*    64K */, arg) /* 0x12010000 - 0x1201FFFF */ \
  MACRO(FIRMWARE,                0x0300000 /*  3072K */, arg) /* 0x12020000 - 0x1231FFFF */ \
  MACRO(FIRMWARE_SCRATCH,        0x0300000 /*  3072K */, arg) /* 0x12320000 - 0x1261FFFF */ \
  MACRO(SYSTEM_RESOURCES_BANK_0, 0x0200000 /*  2048K */, arg) /* 0x12620000 - 0x1281FFFF */ \
  MACRO(SYSTEM_RESOURCES_BANK_1, 0x0200000 /*  2048K */, arg) /* 0x12820000 - 0x12A1FFFF */ \
  MACRO(SAFE_FIRMWARE,           0x0080000 /*   512K */, arg) /* 0x12A20000 - 0x12A9FFFF */ \
  MACRO(FILESYSTEM,              0x0520000 /*  5248K */, arg) /* 0x12AA0000 - 0x12FBFFFF */ \
  MACRO(RSVD1,                   0x000F000 /*    60K */, arg) /* 0x12FC0000 - 0x12FCEFFF */ \
  MACRO(DEBUG_DB,                0x0020000 /*   128K */, arg) /* 0x12FCF000 - 0x12FEEFFF */ \
  MACRO(RSVD2,                   0x000F000 /*    58K */, arg) /* 0x12FEF000 - 0x12FFDFFF */ \
  MACRO(MFG_INFO,                0x0001000 /*     4K */, arg) /* 0x12FFE000 - 0x12FFEFFF */ \
  MACRO(SHARED_PRF_STORAGE,      0x0001000 /*     4K */, arg) /* 0x12FFF000 - 0x12FFFFFF */

#include "flash_region_def_helper.h"

// Flash region _BEGIN and _END addresses
//////////////////////////////////////////////////////////////////////////////

#define FLASH_REGION_PTABLE_BEGIN FLASH_REGION_START_ADDR(PTABLE)
#define FLASH_REGION_PTABLE_END FLASH_REGION_END_ADDR(PTABLE)

#define FLASH_REGION_BOOTLOADER_BEGIN FLASH_REGION_START_ADDR(BOOTLOADER)
#define FLASH_REGION_BOOTLOADER_END FLASH_REGION_END_ADDR(BOOTLOADER)

#define FLASH_REGION_FIRMWARE_BEGIN FLASH_REGION_START_ADDR(FIRMWARE)
#define FLASH_REGION_FIRMWARE_END FLASH_REGION_END_ADDR(FIRMWARE)

#define FLASH_REGION_FIRMWARE_SCRATCH_BEGIN FLASH_REGION_START_ADDR(FIRMWARE_SCRATCH)
#define FLASH_REGION_FIRMWARE_SCRATCH_END FLASH_REGION_END_ADDR(FIRMWARE_SCRATCH)

#define FLASH_REGION_SYSTEM_RESOURCES_BANK_0_BEGIN FLASH_REGION_START_ADDR(SYSTEM_RESOURCES_BANK_0)
#define FLASH_REGION_SYSTEM_RESOURCES_BANK_0_END FLASH_REGION_END_ADDR(SYSTEM_RESOURCES_BANK_0)

#define FLASH_REGION_SYSTEM_RESOURCES_BANK_1_BEGIN FLASH_REGION_START_ADDR(SYSTEM_RESOURCES_BANK_1)
#define FLASH_REGION_SYSTEM_RESOURCES_BANK_1_END FLASH_REGION_END_ADDR(SYSTEM_RESOURCES_BANK_1)

#define FLASH_REGION_SAFE_FIRMWARE_BEGIN FLASH_REGION_START_ADDR(SAFE_FIRMWARE)
#define FLASH_REGION_SAFE_FIRMWARE_END FLASH_REGION_END_ADDR(SAFE_FIRMWARE)

#define FLASH_REGION_FILESYSTEM_BEGIN FLASH_REGION_START_ADDR(FILESYSTEM)
#define FLASH_REGION_FILESYSTEM_END FLASH_REGION_END_ADDR(FILESYSTEM)
#define FLASH_FILESYSTEM_BLOCK_SIZE SUBSECTOR_SIZE_BYTES

#define FLASH_REGION_DEBUG_DB_BEGIN FLASH_REGION_START_ADDR(DEBUG_DB)
#define FLASH_REGION_DEBUG_DB_END FLASH_REGION_END_ADDR(DEBUG_DB)
#define FLASH_DEBUG_DB_BLOCK_SIZE SUBSECTOR_SIZE_BYTES

#define FLASH_REGION_MFG_INFO_BEGIN FLASH_REGION_START_ADDR(MFG_INFO)
#define FLASH_REGION_MFG_INFO_END FLASH_REGION_END_ADDR(MFG_INFO)

#define FLASH_REGION_SHARED_PRF_STORAGE_BEGIN FLASH_REGION_START_ADDR(SHARED_PRF_STORAGE)
#define FLASH_REGION_SHARED_PRF_STORAGE_END FLASH_REGION_END_ADDR(SHARED_PRF_STORAGE)

#define BOARD_NOR_FLASH_SIZE FLASH_REGION_START_ADDR(_COUNT) - FLASH_REGION_BASE_ADDRESS

// Static asserts to make sure everything worked out
//////////////////////////////////////////////////////////////////////////////

// make sure all the sizes are multiples of the subsector size (4k)
FLASH_REGION_SIZE_CHECK(SUBSECTOR_SIZE_BYTES)

// make sure the total size is what we expect (16mb)
_Static_assert(BOARD_NOR_FLASH_SIZE == 0x1000000, "Flash size should be 16mb");
