/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "kernel/util/factory_reset.h"

#include <pbl/drivers/rtc.h>
#include <pbl/drivers/task_watchdog.h>
#include "flash_region/filesystem_regions.h"
#include "kernel/event_loop.h"
#include "kernel/util/standby.h"
#include "process_management/worker_manager.h"
#include "pbl/services/event_service.h"
#include "pbl/services/shared_prf_storage/shared_prf_storage.h"
#include "pbl/services/system_task.h"
#include "pbl/services/runlevel.h"
#include "shell/normal/app_idle_timeout.h"
#include "system/bootbits.h"
#include "system/firmware_storage.h"
#include <pbl/logging/logging.h>
#include "system/reboot_reason.h"
#include "system/reset.h"
#include "kernel/util/sleep.h"

#if !defined(CONFIG_RECOVERY_FW)
#include "pbl/services/blob_db/pin_db.h"
#include "pbl/services/blob_db/reminder_db.h"
#include "pbl/services/filesystem/pfs.h"
#include "pbl/services/timeline/event.h"
#endif

static bool s_in_factory_reset = false;

static void prv_factory_reset_non_pfs_data() {
  PBL_LOG_SYNC_INFO("Factory resetting...");

  // This function can block the system task for a long time.
  // Prevent callbacks being added to the system task so it doesn't overflow.
  system_task_block_callbacks(true /* block callbacks */);
  launcher_block_popups(true);

  worker_manager_disable();
  event_service_clear_process_subscriptions(PebbleTask_App);

  shared_prf_storage_wipe_all();

  services_set_runlevel(RunLevel_BareMinimum);
  app_idle_timeout_stop();

  while (worker_manager_get_current_worker_md()) {
    // busy loop until the worker is killed
    psleep(3);
  }

  rtc_timezone_clear();
}

void factory_reset_set_reason_and_reset(void) {
  RebootReason reason = { RebootReasonCode_FactoryResetReset, 0 };
  reboot_reason_set(&reason);
  system_reset();
}

static void prv_factory_reset_post(bool should_shutdown) {
  if (should_shutdown) {
    enter_standby(RebootReasonCode_FactoryResetShutdown);
  } else {
    factory_reset_set_reason_and_reset();
  }
}

void factory_reset(bool should_shutdown) {
  s_in_factory_reset = true;

  prv_factory_reset_non_pfs_data();

#if !defined(CONFIG_RECOVERY_FW)
  // Commit to booting PRF before the filesystem wipe: it can take minutes,
  // and a reset in that window (watchdog, crash, power loss) must land in
  // PRF instead of rebooting a half-wiped normal firmware.
  // "First use" is part of the PRF image for Snowy.
  boot_bit_set(BOOT_BIT_FORCE_PRF);
#ifdef CONFIG_PBLBOOT
  // Invalidate both firmware slots so the bootloader doesn't boot into them
  firmware_storage_invalidate_firmware_slot(0);
  firmware_storage_invalidate_firmware_slot(1);
#endif

  // pfs_format() holds the PFS mutex across the erase, blocking concurrent
  // writes from the App task that would otherwise survive into a freshly-erased region.
  pfs_format(false /* write_erase_headers */);
#else
  filesystem_regions_erase_all();
#endif

  prv_factory_reset_post(should_shutdown);
}

#if !defined(CONFIG_RECOVERY_FW)
void close_db_files() {
  // Deinit the databases and any clients
  timeline_event_deinit();
  reminder_db_deinit();
  pin_db_deinit();
}

void factory_reset_fast(void *unused) {
  s_in_factory_reset = true;

  // disable the watchdog... we've got lots to do before we reset
  task_watchdog_mask_clear(pebble_task_get_current());

  close_db_files();

  prv_factory_reset_non_pfs_data();

  pfs_remove_files(NULL);

  prv_factory_reset_post(false /* should_shutdown */);
}
#endif // !defined(CONFIG_RECOVERY_FW)

//! Used by the mfg flow to kick us out the MFG firmware and into the consumer PRF that's stored
//! on the external flash.
void command_enter_consumer_mode(void) {
  boot_bit_set(BOOT_BIT_FORCE_PRF);
  factory_reset(true /* should_shutdown */);
}

bool factory_reset_ongoing(void) {
  return s_in_factory_reset;
}
