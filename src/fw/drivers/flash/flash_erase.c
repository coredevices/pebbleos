/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include <pbl/drivers/flash.h>
#include <pbl/drivers/flash/flash_internal.h>

#include "flash_region/flash_region.h"
#include "pbl/services/new_timer/new_timer.h"
#include <pbl/logging/logging.h>
#include "system/passert.h"
#include "pbl/util/attributes.h"
#include "pbl/util/math.h"

#include "FreeRTOS.h"
#include "semphr.h"

PBL_LOG_MODULE_DECLARE(driver_flash, CONFIG_DRIVER_FLASH_LOG_LEVEL);


static SemaphoreHandle_t s_erase_mutex = NULL;
static struct FlashRegionEraseState {
  uint32_t next_erase_addr;
  uint32_t end_addr;
  FlashOperationCompleteCb on_complete;
  void *on_complete_context;
} s_erase_state;

static void prv_erase_next_async(void *ignored);

T_STATIC void prv_lock_erase_mutex(void);
T_STATIC void prv_unlock_erase_mutex(void);
#if !UNITTEST
void flash_erase_init(void) {
  s_erase_mutex = xSemaphoreCreateBinary();
  xSemaphoreGive(s_erase_mutex);
}

static void prv_lock_erase_mutex(void) {
  xSemaphoreTake(s_erase_mutex, portMAX_DELAY);
}

static void prv_unlock_erase_mutex(void) {
  xSemaphoreGive(s_erase_mutex);
}
#endif

static void prv_async_erase_done_cb(void *ignored, status_t result) {
  if (PASSED(result) &&
      s_erase_state.next_erase_addr < s_erase_state.end_addr) {
    // Chain the next erase from a new callback to prevent recursion (and the
    // potential for a stack overflow) if the flash_erase_sector calls the
    // completion callback asynchronously.
    if (!new_timer_add_work_callback(prv_erase_next_async, NULL)) {
      PBL_LOG_ERR("Failed to enqueue callback; aborting erase");
      prv_unlock_erase_mutex();
      s_erase_state.on_complete(s_erase_state.on_complete_context, E_INTERNAL);
    }
  } else {
    prv_unlock_erase_mutex();
    s_erase_state.on_complete(s_erase_state.on_complete_context, result);
  }
}

static void prv_erase_next_async(void *ignored) {
  uint32_t addr = s_erase_state.next_erase_addr;
  if ((addr & ~SECTOR_ADDR_MASK) == 0 &&
      addr + SECTOR_SIZE_BYTES <= s_erase_state.end_addr) {
    s_erase_state.next_erase_addr += SECTOR_SIZE_BYTES;
    flash_erase_sector(addr, prv_async_erase_done_cb, NULL);
  } else {
    // Fall back to a subsector erase
    s_erase_state.next_erase_addr += SUBSECTOR_SIZE_BYTES;
    PBL_ASSERTN(s_erase_state.next_erase_addr <= s_erase_state.end_addr);
    flash_erase_subsector(addr, prv_async_erase_done_cb, NULL);
  }
}

void flash_erase_optimal_range(
    uint32_t min_start, uint32_t max_start, uint32_t min_end, uint32_t max_end,
    FlashOperationCompleteCb on_complete, void *context) {
  PBL_ASSERTN(((min_start & (~SUBSECTOR_ADDR_MASK)) == 0) &&
              ((max_end & (~SUBSECTOR_ADDR_MASK)) == 0) &&
              (min_start <= max_start) &&
              (max_start <= min_end) &&
              (min_end <= max_end));

  // We want to erase the sector that starts immediately below max_start but
  // after min_start. If no sector boundary exists between the two, we need to
  // start erasing sectors after min_start and backfill with subsector erases.
  int32_t sector_start = (max_start & SECTOR_ADDR_MASK);
  int32_t subsector_start = (max_start & SUBSECTOR_ADDR_MASK);
  if (sector_start < (int32_t) min_start) {
    sector_start += SECTOR_SIZE_BYTES;
  }

  // We want to erase ending after min_end but before max_end. If that ends
  // running past the end of max_end, we need to erase starting with the sector
  // before and fill in with subsector erases.
  int32_t sector_end = ((min_end - 1) & SECTOR_ADDR_MASK) + SECTOR_SIZE_BYTES;
  int32_t subsector_end =
      ((min_end - 1) & SUBSECTOR_ADDR_MASK) + SUBSECTOR_SIZE_BYTES;
  if (sector_end > (int32_t) max_end) {
    sector_end -= SECTOR_SIZE_BYTES;
  }

  int32_t start_addr = MIN(sector_start, subsector_start);
  int32_t end_addr = MAX(sector_end, subsector_end);

  if (sector_start >= sector_end) {
    // Can't erase any full sectors; just erase subsectors the whole way.
    start_addr = subsector_start;
    end_addr = subsector_end;
  }

  if (start_addr == end_addr) {
    // Nothing to do!
    on_complete(context, S_NO_ACTION_REQUIRED);
    return;
  }

  prv_lock_erase_mutex();

  s_erase_state = (struct FlashRegionEraseState) {
    .next_erase_addr = start_addr,
    .end_addr = end_addr,
    .on_complete = on_complete,
    .on_complete_context = context,
  };

  prv_erase_next_async(NULL);
}
