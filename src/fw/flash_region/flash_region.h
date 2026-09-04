/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include <stdint.h>

#ifdef CONFIG_FLASH_QEMU
#include "flash_region_qemu.h"
#elif defined(CONFIG_FLASH_GD25LQ255E)
#include "flash_region_gd25lq255e.h"
#elif defined(CONFIG_FLASH_GD25Q256E)
#include "flash_region_gd25q256e.h"
#endif

#ifdef CONFIG_PBLBOOT
// We assume that if we have pblboot, we use the two slots with direct XIP
#if defined(CONFIG_RECOVERY_FW) && !defined(CONFIG_MFG) && !defined(CONFIG_RECOVERY_FW_AS_FW)
// On recovery we always write to slot 0
#define FLASH_REGION_FIRMWARE_DEST_BEGIN FLASH_REGION_FIRMWARE_SLOT_0_BEGIN
#define FLASH_REGION_FIRMWARE_DEST_END FLASH_REGION_FIRMWARE_SLOT_0_END
#else
#if CONFIG_FIRMWARE_SLOT == 0
// If we're running from slot 0, write to slot 1
#define FLASH_REGION_FIRMWARE_DEST_BEGIN FLASH_REGION_FIRMWARE_SLOT_1_BEGIN
#define FLASH_REGION_FIRMWARE_DEST_END FLASH_REGION_FIRMWARE_SLOT_1_END
#else
// If we're running from slot 1, write to slot 0
#define FLASH_REGION_FIRMWARE_DEST_BEGIN FLASH_REGION_FIRMWARE_SLOT_0_BEGIN
#define FLASH_REGION_FIRMWARE_DEST_END FLASH_REGION_FIRMWARE_SLOT_0_END
#endif
#endif
#else
// We assume that if we don't have pblboot, we only have one slot (scratch area)
#define FLASH_REGION_FIRMWARE_DEST_BEGIN FLASH_REGION_FIRMWARE_SLOT_1_BEGIN
#define FLASH_REGION_FIRMWARE_DEST_END FLASH_REGION_FIRMWARE_SLOT_1_END

// If we don't have pblboot, use firmware slot to store CD
#define FLASH_REGION_CD_BEGIN FLASH_REGION_FIRMWARE_DEST_BEGIN
#define FLASH_REGION_CD_END FLASH_REGION_FIRMWARE_DEST_END
#endif

#define FLASH_REGION_FIRMWARE_DEST_START (FLASH_REGION_FIRMWARE_DEST_BEGIN + FIRMWARE_OFFSET)
#define FLASH_REGION_SAFE_FIRMWARE_START (FLASH_REGION_SAFE_FIRMWARE_BEGIN + FIRMWARE_OFFSET)

// NOTE: The following functions are deprecated! New code should use the
// asynchronous version, flash_erase_optimal_range, in flash.h.

//! Erase at least (max_start, min_end) but no more than (min_start, max_end) using as few erase
//! operations as possible. (min_start, max_end) must be both 4kb aligned, as that's the smallest
//! unit that we can erase.
void flash_region_erase_optimal_range(uint32_t min_start, uint32_t max_start, uint32_t min_end,
                                      uint32_t max_end);

//! The same as flash_region_erase_optimal_range but first disables the task watchdog for the
//! current task.
void flash_region_erase_optimal_range_no_watchdog(uint32_t min_start, uint32_t max_start,
                                                  uint32_t min_end, uint32_t max_end);
