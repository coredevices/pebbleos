/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include <pbl/drivers/flash.h>
#include <pbl/drivers/flash/flash_impl.h>
#include <pbl/drivers/watchdog.h>
#include "flash_region/flash_region.h"
#include "kernel/util/delay.h"

#include "kernel/core_dump_private.h"

static bool s_active;

//! We have our own flash driver for coredump support because it must not use
//! any FreeRTOS constructs & we want to keep it as simple as possible. In
//! addition we want the flexibility to be able to reset the flash driver to
//! get it into a working state

void cd_flash_init(void) {
  // Reset & (re)initialize the flash HW
  flash_impl_init(true /* coredump_mode */);

  // Protect the PRF region from writes
  flash_impl_write_protect(
      FLASH_REGION_SAFE_FIRMWARE_BEGIN,
      (FLASH_REGION_SAFE_FIRMWARE_END - SECTOR_SIZE_BYTES));

  s_active = true;
}

bool cd_flash_active(void) {
  return s_active;
}

void cd_flash_erase_region(uint32_t start_addr, uint32_t total_bytes) {
  CD_ASSERTN(((start_addr & SUBSECTOR_ADDR_MASK) == start_addr) &&
             ((total_bytes & SUBSECTOR_ADDR_MASK) == total_bytes));

  while (total_bytes > 0) {
    watchdog_feed();

    uint32_t erase_size = 0;
    if (((start_addr & SECTOR_ADDR_MASK) == start_addr) &&
        (total_bytes >= SECTOR_SIZE_BYTES)) {
      erase_size = SECTOR_SIZE_BYTES;
      flash_impl_erase_sector_begin(start_addr);
    } else if ((start_addr & SUBSECTOR_ADDR_MASK) == start_addr &&
               (total_bytes >= SUBSECTOR_SIZE_BYTES)) {
      erase_size = SUBSECTOR_SIZE_BYTES;
      flash_impl_erase_subsector_begin(start_addr);
    } else {
      // Unaligned start address or unaligned erase size
      CD_ASSERTN(0);
    }

    status_t status;
    while ((status = flash_impl_get_erase_status()) == E_BUSY) delay_us(100);
    CD_ASSERTN(status == S_SUCCESS);

    total_bytes -= erase_size;
    start_addr += erase_size;
  }

  watchdog_feed();
}

uint32_t cd_flash_write_bytes(const void *buffer_ptr, uint32_t start_addr,
                              const uint32_t buffer_size) {
  CD_ASSERTN(((start_addr + buffer_size) <= CORE_DUMP_FLASH_END) &&
      (int)start_addr >= CORE_DUMP_FLASH_START);

  const uint8_t *buffer = buffer_ptr;
  uint32_t remaining = buffer_size;

  while (remaining) {
    int written = flash_impl_write_page_begin(buffer, start_addr, remaining);
    CD_ASSERTN(PASSED(written));
    status_t status;
    while ((status = flash_impl_get_write_status()) == E_BUSY) {
      delay_us(10);
    }
    CD_ASSERTN(PASSED(status));
    buffer += written;
    start_addr += written;
    remaining -= written;
  }
  return buffer_size;
}

void cd_flash_read_bytes(void* buffer_ptr, uint32_t start_addr,
    uint32_t buffer_size) {
  flash_impl_read_sync(buffer_ptr, start_addr, buffer_size);
}

status_t cd_flash_read_security_register(uint32_t addr, uint8_t *val) {
  return flash_impl_read_security_register(addr, val);
}

status_t cd_flash_security_register_is_locked(uint32_t addr, bool *locked) {
  return flash_impl_security_register_is_locked(addr, locked);
}