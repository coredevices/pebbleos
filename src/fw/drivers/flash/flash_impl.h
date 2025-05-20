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
#include <stddef.h>
#include <stdint.h>

#include "drivers/flash.h"
#include "system/status_codes.h"

//! Flash Low-Level API
//!
//! Unless otherwise specified, this API is non-reentrant. It is unsafe to
//! call a function in one thread while another function is being executed in a
//! second thread, and it is unsafe to call these functions from within a
//! flash_impl callback.

typedef uint32_t FlashAddress;

//! Initialize the low-level flash implementation and hardware into a known
//! state where it is ready to accept commands.
//!
//! This function configures microcontroller peripherals. It should be guarded
//! with periph_config_acquire_lock/periph_config_release_lock.
//!
//! @param coredump_mode True if we need this flash driver to not rely on any other system
//!                      services such as FreeRTOS being available because we're in the middle
//!                      of a core dump. This may result in slower operations.
status_t flash_impl_init(bool coredump_mode);

//! Enable or disable synchronous burst mode, if supported.
//!
//! Burst mode is disabled whenever \ref flash_impl_init is called.
//!
//! The result is undefined if this function is called while any other flash
//! operation is in progress.
status_t flash_impl_set_burst_mode(bool enable);

//! Return the base address of the sector overlapping the given address.
//!
//! This function is reentrant.
FlashAddress flash_impl_get_sector_base_address(FlashAddress addr);

//! Return the base address of the subsector overlapping the given address.
//!
//! This function is reentrant.
FlashAddress flash_impl_get_subsector_base_address(FlashAddress addr);

//! Query the flash hardware for its capacity in bytes.
size_t flash_impl_get_capacity(void);

//! Enter a low-power state.
//!
//! Once in a low-power mode, all operations may fail until
//! \ref flash_impl_exit_low_power_mode is called. This function is idempotent.
status_t flash_impl_enter_low_power_mode(void);

//! Exit a low-power state.
//!
//! Return the flash to a fully operational mode. This may be a time-intensive
//! operation. This function is idempotent.
status_t flash_impl_exit_low_power_mode(void);

//! Read data into a buffer.
//!
//! The result is undefined if this function is called while a write or erase is
//! in progress.
status_t flash_impl_read_sync(void *buffer, FlashAddress addr, size_t len);

//! Initiate a DMA-accelerated flash read.
//!
//! The caller must ensure that the DMA transfer will not be interfered with
//! by any clock changes or stoppages externally. (read: inhibit stop mode)
//!
//! This function will return immediately once the transfer has begun.
//! \ref flash_impl_on_read_dma_complete_from_isr will be called from an
//! interrupt context to signal that the transfer has completed. The effect of
//! calling flash_impl_read_dma_begin a second time while another DMA transfer
//! is currently in progress is undefined.
//!
//! The result is undefined if this function is called while a write or erase is
//! in progress.
status_t flash_impl_read_dma_begin(void *buffer, FlashAddress addr,
                                   size_t len);

//! Called from an interrupt context when the DMA read has completed. It is
//! guaranteed that the call is made from an interrupt of low enough priority
//! that RTOS API calls are safe to use, and that it is a tail-call from the end
//! of the implementation's ISR (read: portEND_SWITCHING_ISR is permissible).
//!
//! @param result S_SUCCESS iff the read completed successfully.
extern void flash_impl_on_read_dma_complete_from_isr(status_t result);

//! If the flash part requires write protection to be explicitly enabled, enable it.
void flash_impl_enable_write_protection(void);

//! Write protect a region of flash. Only one region may be protected at any
//! given time.
//!
//! The result is undefined if this function is called while a write or erase is
//! in progress.
status_t flash_impl_write_protect(FlashAddress start_sector,
                                  FlashAddress end_sector);

//! Remove write protection.
//!
//! The result is undefined if this function is called while a write or erase is
//! in progress.
status_t flash_impl_unprotect(void);

//! Write a page of bytes to flash.
//!
//! @param buffer The source buffer.
//! @param addr Destination flash address.
//! @param len Length to write.
//! @return The number of bytes that will be written to flash, assuming that the
//!         write completes successfully, or a StatusCode error if there was an
//!         error starting the write operation.
//!
//! Each call to flash_impl_write_page_begin begins a single flash write
//! operation, writing the maximum amount of data supported by the hardware in
//! a single operation. Multiple page writes may be required to write a complete
//! buffer to flash.
//!
//! Example usage:
//! \code
//!   while (len) {
//!     int written = flash_impl_write_page_begin(buffer, addr, len));
//!     if (written < 0) {
//!       // Handle error
//!     }
//!     status_t status;
//!     while ((status = flash_impl_get_write_status()) == E_AGAIN) {
//!       continue;
//!     }
//!     if (status != S_SUCCESS) {
//!       // Handle error
//!     }
//!     buffer += written;
//!     addr += written;
//!     len -= written;
//!   }
//! \endcode
//!
//! The result is undefined if this function is called while a read or erase is
//! in progress. It is an error to call this function while a write is
//! in progress or suspended.
int flash_impl_write_page_begin(const void *buffer, FlashAddress addr,
                                size_t len);

//! Poll the status of a flash page write.
//!
//! @return S_SUCCESS if the write has succeeded, E_ERROR if the write has
//!         failed, E_BUSY if the write is still in progress or E_AGAIN if the
//!         write is suspended.
status_t flash_impl_get_write_status(void);

//! Suspend an in-progress write so that reads and erases are permitted.
//!
//! @param addr The address passed to the \ref flash_impl_write_page_begin
//!        call which initiated the write being suspended.
status_t flash_impl_write_suspend(FlashAddress addr);

//! Resume a previously-suspended write.
//!
//! @param addr The address passed to \ref flash_impl_write_suspend.
//!
//! The result is undefined if this function is called while a read or write is
//! in progress.
status_t flash_impl_write_resume(FlashAddress addr);

//! Erase the subsector which overlaps the given address.
//!
//! The result is undefined if this function is called while a read or write is
//! in progress. It is an error to call this function while an erase is
//! suspended.
status_t flash_impl_erase_subsector_begin(FlashAddress subsector_addr);

//! Erase the sector which overlaps the given address.
//!
//! The result is undefined if this function is called while a read or write is
//! in progress. It is an error to call this function while an erase is
//! suspended.
status_t flash_impl_erase_sector_begin(FlashAddress sector_addr);

//! Erase the entire flash.
//!
//! The result is undefined if this function is called while a read or write is
//! in progress. It is an error to call this function while an erase is
//! suspended.
status_t flash_impl_erase_bulk_begin(void);

//! Poll the status of a flash erase.
//!
//! @return S_SUCCESS if the erase has succeeded, E_ERROR if the erase has
//!         failed, E_BUSY if the erase is still in progress or E_AGAIN if the
//!         erase is suspended.
status_t flash_impl_get_erase_status(void);

//! Returns the typical duration of a subsector erase, in milliseconds.
//!
//! This function is reentrant.
uint32_t flash_impl_get_typical_subsector_erase_duration_ms(void);

//! Returns the typical duration of a sector erase, in milliseconds.
//!
//! This function is reentrant.
uint32_t flash_impl_get_typical_sector_erase_duration_ms(void);

//! Suspend an in-progress erase so that reads and writes are permitted.
//!
//! @param addr The sector address passed to the
//!   \ref flash_impl_erase_subsector_begin or
//!   \ref flash_impl_erase_sector_begin call which initiated the erase being
//!   suspended.
//!
//! @return S_SUCCESS if the erase has been suspended, S_NO_ACTION_REQUIRED if
//!         there was no erase in progress at the time, or an error code.
status_t flash_impl_erase_suspend(FlashAddress addr);

//! Resume a previously-suspended erase.
//!
//! @param addr The address passed to \ref flash_impl_erase_suspend.
//!
//! The result is undefined if this function is called while a read or write is
//! in progress.
status_t flash_impl_erase_resume(FlashAddress addr);

//! Check whether the subsector overlapping the specified address is blank
//! (reads as all 1's).
//!
//! @param addr An address within the subsector being checked.
//!
//! @return S_TRUE if blank, S_FALSE if any bit in the sector has been
//!         programmed, or E_BUSY if another flash operation is in progress.
//!
//! This operation is hardware-accelerated if possible. This operation may not
//! be performed if any reads, writes, or erases are in progress or suspended,
//! and this operation cannot be suspended once initiated. The result is
//! undefined if any other flash operation is initiated or in progress while a
//! blank check operation is in progress.
//!
//! @warning This function may return S_TRUE on a subsector where an erase
//!          operation was terminated prematurely. While such a subsector may
//!          read back as blank, data loss may occur and writes may fail if the
//!          subsector is not erased fully before it is written to.
status_t flash_impl_blank_check_subsector(FlashAddress addr);

//! Check whether the sector overlapping the specified address is blank (reads
//! as all 1's).
//!
//! @param addr An address within the sector being checked.
//!
//! @return S_TRUE if blank, S_FALSE if any bit in the sector has been
//!         programmed, or E_BUSY if another flash operation is in progress.
//!
//! This operation is hardware-accelerated if possible. This operation may not
//! be performed if any reads, writes, or erases are in progress or suspended,
//! and this operation cannot be suspended once initiated. The result is
//! undefined if any other flash operation is initiated or in progress while a
//! blank check operation is in progress.
//!
//! @warning This function may return S_TRUE on a sector where an erase
//!          operation was terminated prematurely. While such a sector may read
//!          back as blank, data loss may occur and writes may fail if the
//!          sector is not erased fully before it is written to.
status_t flash_impl_blank_check_sector(FlashAddress addr);

//! Save the address of an erase in progress to a nonvolatile location. The
//! erase address, along with the fact that an erase is in progress, must be
//! able to survive a system crash and reboot.
//!
//! @note Writing this data to the same flash array that is being erased is
//!       almost certainly a bad idea.
//!
//! @param is_subsector True if the erase is a subsector.
//!
//! @param addr The address being erased.
//!
//! @return S_SUCCESS if the data was successfully stored.
status_t flash_impl_set_nvram_erase_status(bool is_subsector,
                                           FlashAddress addr);

//! Save to a nonvolatile location the fact that no erase is in progress.
//!
//! @return S_SUCCESS if the status was successfully stored.
status_t flash_impl_clear_nvram_erase_status(void);

//! Retrieve the erase status previously set by
//! flash_impl_set_nvram_erase_status or flash_impl_clear_nvram_erase_status.
//!
//! @param [out] is_subsector The value of is_subsector passed to the most
//!   most recent call to flash_impl_set_nvram_erase_status if the status was
//!   not subsequently cleared by flash_impl_clear_nvram_erase_status. The
//!   pointer should not be written to if the erase status was cleared.
//!
//! @param [out] addr The address passed to the most recent call to
//!   flash_impl_set_nvram_erase_status if the status was not subsequently
//!   cleared by flash_impl_clear_nvram_erase_status. The address should not be
//!   written to if the erase status was cleared.
//!
//! @return S_TRUE if an erase was in progress; S_FALSE otherwise.
status_t flash_impl_get_nvram_erase_status(bool *is_subsector,
                                           FlashAddress *addr);

void flash_impl_use(void);
void flash_impl_release(void);
void flash_impl_release_many(uint32_t num_locks);

//! Read security register
//!
//! @param addr The address of the security register to read.
//!
//! @param [out] val The value of the security register read.
//!
//! @retval S_SUCCESS if the read was successful
//! @retval StatusCode if the read failed
status_t flash_impl_read_security_register(uint32_t addr, uint8_t *val);

//! Check if the security registers are locked
//!
//! @param [out] locked True if the security registers are locked
//!
//! @retval S_SUCCESS if the check was successful
//! @retval StatusCode if the check failed
status_t flash_impl_security_registers_are_locked(bool *locked);

//! Erase security register
//!
//! @param addr The address of the security register to erase.
//!
//! @retval S_SUCCESS if the erase was successful
//! @retval StatusCode if the erase failed
status_t flash_impl_erase_security_register(uint32_t addr);

//! Write security register
//!
//! @param addr The address of the security register to write.
//!
//! @param val The value to write to the security register.
//!
//! @retval S_SUCCESS if the write was successful
//! @retval StatusCode if the write failed
status_t flash_impl_write_security_register(uint32_t addr, uint8_t val);

//! Obtain security registers information
//!
//! @returns The information about the security registers.
const FlashSecurityRegisters *flash_impl_security_registers_info(void);

#ifdef RECOVERY_FW
//! Lock security registers
//!
//! @warning This is a one time operation and will permanently lock the security registers.
//!
//! @retval S_SUCCESS if the lock was successful
//! @retval StatusCode if the lock failed
status_t flash_impl_lock_security_registers(void);
#endif // RECOVERY_FW
