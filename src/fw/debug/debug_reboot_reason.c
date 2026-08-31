/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "debug.h"

#include "comm/ble/gatt_service_changed.h"
#include "kernel/core_dump.h"
#include "kernel/event_loop.h"
#include "popups/crashed_ui.h"
#include <pbl/logging/logging.h>
#include "system/reboot_reason.h"

static RebootReasonCode s_last_reboot_reason_code = RebootReasonCode_Unknown;
RebootReasonCode reboot_reason_get_last_reboot_reason(void) {
  return s_last_reboot_reason_code;
}

void debug_reboot_reason_print(McuRebootReason mcu_reboot_reason) {
  RebootReason reason;
  reboot_reason_get(&reason);
  s_last_reboot_reason_code = reason.code;

  // We're out of flash space, scrape a few bytes back!
  static const char* rebooted_due_to = " rebooted due to ";

  const char* restarted_safely_string = "Safely";
  if (!reason.restarted_safely) {
    restarted_safely_string = "Dangerously";
  }

  // Leave this NULL to do your own printing.
  const char *reason_string = NULL;
  switch (reason.code) {
  // Normal stuff
  case RebootReasonCode_Unknown:
    reason_string = "We don't know why we %s rebooted.";
    break;
  case RebootReasonCode_LowBattery:
    reason_string = "%s%sLowBattery";
    break;
  case RebootReasonCode_SoftwareUpdate:
    gatt_service_changed_server_handle_fw_update();
    reason_string = "%s%sSoftwareUpdate";
    break;
  case RebootReasonCode_ResetButtonsHeld:
    // Since we forced the reset, it isn't unexpected
    reason_string = "%s%sResetButtonsHeld";
    break;
  case RebootReasonCode_ShutdownMenuItem:
    reason_string = "%s%sLowBattery";
    break;
  case RebootReasonCode_FactoryResetReset:
    reason_string = "%s%sFactoryResetReset";
    break;
  case RebootReasonCode_FactoryResetShutdown:
    reason_string = "%s%sFactoryResetShutdown";
    break;
  case RebootReasonCode_MfgShutdown:
    reason_string = "%s%sMfgShutdown";
    break;
  case RebootReasonCode_Serial:
    reason_string = "%s%sSerial";
    break;
  case RebootReasonCode_RemoteReset:
    reason_string = "%s%sa Remote Reset";
    break;
  case RebootReasonCode_ForcedCoreDump:
    reason_string = "%s%sa Forced Coredump";
    break;
  case RebootReasonCode_PrfIdle:
    reason_string = "%s%sIdle PRF";
    break;
  // Error occurred
  case RebootReasonCode_Assert:
    reason_string = "%s%sAssert: LR %#"PRIxPTR;
    break;
  case RebootReasonCode_HardFault:
    reason_string = "%s%sHardFault: LR %#"PRIxPTR;
    break;
  case RebootReasonCode_LauncherPanic:
    reason_string = "%s%sLauncherPanic: code 0x%"PRIx32;
    break;
  case RebootReasonCode_ClockFailure:
    reason_string = "%s%sClock Failure";
    break;
  case RebootReasonCode_WorkerHardFault:
    reason_string = "%s%sWorker HardFault";
    break;
  case RebootReasonCode_OutOfMemory:
    reason_string = "%s%sOOM";
    break;
  case RebootReasonCode_BtCoredump:
    reason_string = "%s%sBT Coredump";
    break;
  default:
    reason_string = "%s%sUnrecognized Reason";
    break;
  // Error occurred
  case RebootReasonCode_Watchdog:
    PBL_LOG_WRN("%s%sWatchdog: Bits 0x%" PRIx8 ", Mask 0x%" PRIx8,
              restarted_safely_string, rebooted_due_to, reason.data8[0], reason.data8[1]);

    if (reason.watchdog.stuck_task_pc != 0) {
      PBL_LOG_WRN("Stuck task PC: 0x%" PRIx32 ", LR: 0x%" PRIx32,
                reason.watchdog.stuck_task_pc, reason.watchdog.stuck_task_lr);

      if (reason.watchdog.stuck_task_callback) {
        PBL_LOG_WRN("Stuck callback: 0x%" PRIx32,
                  reason.watchdog.stuck_task_callback);
      }
    }
    break;
  case RebootReasonCode_StackOverflow:
    PebbleTask task = (PebbleTask) reason.data8[0];
    PBL_LOG_WRN("%s%sStackOverflow: Task #%d", restarted_safely_string,
              rebooted_due_to, task);
    break;
  case RebootReasonCode_EventQueueFull:
    PBL_LOG_WRN("%s%sEvent Queue Full", restarted_safely_string, rebooted_due_to);
    PBL_LOG_WRN("LR: 0x%"PRIx32" Current: 0x%"PRIx32" Dropped: 0x%"PRIx32,
              reason.event_queue.push_lr,
              reason.event_queue.current_event,
              reason.event_queue.dropped_event);
    break;
  }
  // Generic reason string
  if (reason_string) {
    pbl_log(LOG_LEVEL_WARNING, __FILE__, __LINE__, reason_string,
            restarted_safely_string, rebooted_due_to, reason.extra.value);
  }

  if (is_unread_coredump_available()) {
    PBL_LOG_INFO("Unread coredump file is present!");
  }

  PBL_LOG_INFO("MCU reset reason mask: 0x%x", (int)mcu_reboot_reason.reset_mask);

  // Core dumps always get an alert display, since the user asked for it.
  if (reason.code == RebootReasonCode_ForcedCoreDump) {
    crashed_ui_show_forced_core_dump();
  }

  reboot_reason_clear();
}
