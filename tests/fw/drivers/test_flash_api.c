/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "clar.h"
#include "fake_new_timer.h"
#include "stubs_sem.h"
#include "stubs_analytics.h"
#include "stubs_irq.h"
#include "stubs_logging.h"
#include "stubs_mutex.h"
#include "stubs_passert.h"
#include "stubs_pebble_tasks.h"
#include "stubs_prompt.h"
#include "stubs_sleep.h"
#include "stubs_task_watchdog.h"
#include "stubs_worker_manager.h"

#include <pbl/drivers/flash.h>
#include <pbl/drivers/flash/flash_impl.h>

void flash_api_reset_for_test(void);
TimerID flash_api_get_erase_poll_timer_for_test(void);


status_t return_success(void) {
  return S_SUCCESS;
}

status_t return_error(void) {
  return E_ERROR;
}

status_t flash_impl_init(bool coredump_mode) {
  return S_SUCCESS;
}

void flash_impl_use(void) {}
void flash_impl_release_many(uint32_t num_locks) {}


int get_subsector_base_calls = 0;
FlashAddress flash_impl_get_subsector_base_address(FlashAddress addr) {
  get_subsector_base_calls++;
  return addr & 0xffffff00;
}

int get_sector_base_calls = 0;
FlashAddress flash_impl_get_sector_base_address(FlashAddress addr) {
  get_sector_base_calls++;
  return addr & 0xfffff000;
}

int erase_subsector_begin_calls = 0;
status_t erase_subsector_begin_return = S_SUCCESS;
status_t flash_impl_erase_subsector_begin(FlashAddress addr) {
  erase_subsector_begin_calls++;
  return erase_subsector_begin_return;
}

int erase_sector_begin_calls = 0;
status_t erase_sector_begin_return = S_SUCCESS;
status_t flash_impl_erase_sector_begin(FlashAddress addr) {
  erase_sector_begin_calls++;
  return erase_sector_begin_return;
}

int get_erase_status_calls = 0;
status_t (*get_erase_status_fn)(void) = return_success;
status_t flash_impl_get_erase_status(void) {
  get_erase_status_calls++;
  return get_erase_status_fn();
}

int blank_check_subsector_calls = 0;
status_t blank_check_subsector_return = S_FALSE;
status_t flash_impl_blank_check_subsector(FlashAddress addr) {
  blank_check_subsector_calls++;
  return blank_check_subsector_return;
}

int blank_check_sector_calls = 0;
status_t blank_check_sector_return = S_FALSE;
status_t flash_impl_blank_check_sector(FlashAddress addr) {
  blank_check_sector_calls++;
  return blank_check_sector_return;
}

status_t flash_impl_enter_low_power_mode(void) {
  return S_SUCCESS;
}

status_t flash_impl_exit_low_power_mode(void) {
  return S_SUCCESS;
}

status_t flash_impl_erase_suspend(FlashAddress addr) {
  return S_SUCCESS;
}

status_t flash_impl_erase_resume(FlashAddress addr) {
  return S_SUCCESS;
}

uint32_t flash_impl_get_typical_subsector_erase_duration_ms(void) {
  return 100;
}

uint32_t flash_impl_get_typical_sector_erase_duration_ms(void) {
  return 100;
}

status_t flash_impl_get_write_status(void) {
  return E_UNKNOWN;
}

status_t flash_impl_read_sync(void *buffer, FlashAddress addr, size_t len) {
  return E_UNKNOWN;
}

status_t flash_impl_set_burst_mode(bool enable) {
  return E_UNKNOWN;
}

status_t flash_impl_unprotect(void) {
  return S_SUCCESS;
}

int flash_impl_write_page_begin(const void *buffer, FlashAddress addr,
                                size_t len) {
  return E_UNKNOWN;
}

void flash_impl_enable_write_protection(void) {
}

status_t flash_impl_write_protect(FlashAddress start_sector,
                                  FlashAddress end_sector) {
  return E_UNKNOWN;
}

status_t flash_impl_set_nvram_erase_status(bool is_subsector,
                                           FlashAddress addr) {
  return S_SUCCESS;
}

status_t flash_impl_clear_nvram_erase_status(void) {
  return S_SUCCESS;
}

status_t flash_impl_get_nvram_erase_status(bool *is_subsector,
                                           FlashAddress *addr) {
  return S_FALSE;
}

status_t flash_impl_read_security_register(uint32_t addr, uint8_t *val) {
  return S_SUCCESS;
}

status_t flash_impl_security_register_is_locked(uint32_t address, bool *locked) {
  *locked = false;
  return S_SUCCESS;
}

status_t flash_impl_erase_security_register(uint32_t addr) {
  return S_SUCCESS;
}

status_t flash_impl_write_security_register(uint32_t addr, uint8_t val) {
  return S_SUCCESS;
}

const FlashSecurityRegisters *flash_impl_security_registers_info(void) {
  return NULL;
}

void flash_erase_init(void) {
}


void *callback_context = NULL;
status_t callback_status = -12345;
static void callback(void *context, status_t status) {
  callback_context = context;
  callback_status = status;
}

void test_flash_api__initialize(void) {
  callback_context = NULL;
  callback_status = -12345;

  get_sector_base_calls = 0;
  get_subsector_base_calls = 0;
  erase_subsector_begin_calls = 0;
  erase_sector_begin_calls = 0;
  get_erase_status_calls = 0;
  get_erase_status_fn = return_success;
  blank_check_subsector_calls = 0;
  blank_check_sector_calls = 0;

  flash_api_reset_for_test();
  flash_init();
}

void test_flash_api__cleanup(void) {
  stub_new_timer_cleanup();
}

void test_flash_api__erase_subsector_calls_right_impl_func(void) {
  flash_erase_subsector(0, callback, NULL);
  cl_assert_equal_i(erase_subsector_begin_calls, 1);
  cl_assert_equal_i(erase_sector_begin_calls, 0);
}

void test_flash_api__erase_sector_calls_right_impl_func(void) {
  flash_erase_sector(0, callback, NULL);
  cl_assert_equal_i(erase_sector_begin_calls, 1);
  cl_assert_equal_i(erase_subsector_begin_calls, 0);
}

///////////////////////////////////////////////////////////////////////

status_t erase_status_return_error_once(void) {
  get_erase_status_fn = return_success;
  return E_ERROR;
}

void test_flash_api__retry_erase_on_first_error(void) {
  get_erase_status_fn = erase_status_return_error_once;
  flash_erase_sector_blocking(0);
  cl_assert_equal_i(erase_sector_begin_calls, 2);
}

///////////////////////////////////////////////////////////////////////

bool uncorrectable_erase_error_cb_called = false;
void uncorrectable_erase_error_cb(void *context, status_t result) {
  cl_assert_equal_i(E_ERROR, result);
  uncorrectable_erase_error_cb_called = true;
}

void test_flash_api__handle_uncorrectable_erase_error(void) {
  get_erase_status_fn = return_error;
  TimerID erase_timer = flash_api_get_erase_poll_timer_for_test();
  flash_erase_sector(0, uncorrectable_erase_error_cb, NULL);
  int i;
  for (i = 0; i < 20 && !uncorrectable_erase_error_cb_called; ++i) {
    cl_assert(stub_new_timer_is_scheduled(erase_timer));
    stub_new_timer_fire(erase_timer);
  }
  cl_assert(i > 1 && i < 20);
  cl_assert_equal_i(uncorrectable_erase_error_cb_called, true);
}
