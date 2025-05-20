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

#include <stdbool.h>
#include <stdint.h>

#include "drivers/flash/qspi_flash_part_definitions.h"
#include "system/status_codes.h"

typedef const struct QSPIFlash QSPIFlash;
typedef const struct QSPIFlashPart QSPIFlashPart;

//! Initialize the QSPI flash
//! @param coredump_mode If true, don't use anything that might not be available mid-crash, such
//!                      as FreeRTOS calls or other system services.
void qspi_flash_init(QSPIFlash *dev, QSPIFlashPart *part, bool coredump_mode);

bool qspi_flash_is_in_coredump_mode(QSPIFlash *dev);

//! Check if the WHOAMI matches the expected value
bool qspi_flash_check_whoami(QSPIFlash *dev);

//! Check if an in-progress erase is complete
status_t qspi_flash_is_erase_complete(QSPIFlash *dev);

//! Begin an erase
status_t qspi_flash_erase_begin(QSPIFlash *dev, uint32_t addr, bool is_subsector);

//! Suspend an erase
status_t qspi_flash_erase_suspend(QSPIFlash *dev, uint32_t addr);

//! Resume a suspended erase
void qspi_flash_erase_resume(QSPIFlash *dev, uint32_t addr);

//! Performs a blocking read
void qspi_flash_read_blocking(QSPIFlash *dev, uint32_t addr, void *buffer, uint32_t length);

// Begins a write operation
int qspi_flash_write_page_begin(QSPIFlash *dev, const void *buffer, uint32_t addr, uint32_t length);

//! Gets the status of an in-progress write operation
status_t qspi_flash_get_write_status(QSPIFlash *dev);

//! Sets whether or not the QSPI flash is in low-power mode
void qspi_flash_set_lower_power_mode(QSPIFlash *dev, bool active);

//! Check whether a sector/subsector is blank
status_t qspi_flash_blank_check(QSPIFlash *dev, uint32_t addr, bool is_subsector);

//! Sets the values of the bits (masked by `mask`) in the register (read by `read_instruction` and
//! written via `write_instruction`) to `value`
void qspi_flash_ll_set_register_bits(QSPIFlash *dev, uint8_t read_instruction,
                                     uint8_t write_instruction, uint8_t value, uint8_t mask);

//! Enable write/erase protection on the given QSPI flash part.
//! Requires the `write_protection_enable` and `read_protection_status` instructions.
//! Return value of the `read_protection_status` instruction is checked against
//! `block_lock.protection_enabled_mask` to test for success.
status_t qspi_flash_write_protection_enable(QSPIFlash *dev);

//! Lock the given sector from write/erase operations.
//! Sector locked with the `block_lock` instruction, and confirmed with `block_lock_status`
//! If the `block_lock` instruction requires extra data, `block_lock.has_lock_data`
//!   and `block_lock.lock_data` can be used.
//! When checking `block_lock_status`, the returned status value is
//!   compared against `block_lock.locked_check`
status_t qspi_flash_lock_sector(QSPIFlash *dev, uint32_t addr);

//! Unlock all sectors so they can be written/erased.
//! Operation is performed by the `block_unlock_all` instruction.
status_t qspi_flash_unlock_all(QSPIFlash *dev);

//! Read security register
status_t qspi_flash_read_security_register(QSPIFlash *dev, uint32_t addr, uint8_t *val);

//! Check if the security registers are locked
status_t qspi_flash_security_registers_are_locked(QSPIFlash *dev, bool *locked);

//! Erase security register
status_t qspi_flash_erase_security_register(QSPIFlash *dev, uint32_t addr);

//! Write security register
status_t qspi_flash_write_security_register(QSPIFlash *dev, uint32_t addr, uint8_t val);

//! Obtain security registers information
const FlashSecurityRegisters *qspi_flash_security_registers_info(QSPIFlash *dev);

#ifdef RECOVERY_FW
//! Lock security registers
//! @warning This is a one time operation and will permanently lock the security registers.
status_t qspi_flash_lock_security_registers(QSPIFlash *dev);
#endif // RECOVERY_FW
