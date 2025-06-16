#include "system/reboot_reason.h"
#include "memfault/ports/reboot_reason.h"

// FIXME: Rather than switching over all of Pebble's reboot tracking to memfault
// today, let's perform a translation from Pebble's tracking to Memfault's.

static eMemfaultRebootReason prv_pbl_reboot_to_mflt_reboot(RebootReasonCode reason) {
  switch (reason) {
    case RebootReasonCode_Unknown:
      return kMfltRebootReason_Unknown;
    case RebootReasonCode_LowBattery:
      return kMfltRebootReason_LowPower;
    case RebootReasonCode_SoftwareUpdate:
      return kMfltRebootReason_FirmwareUpdate;
    case RebootReasonCode_ResetButtonsHeld:
      return kMfltRebootReason_ButtonReset;
    case RebootReasonCode_ShutdownMenuItem:
      return kMfltRebootReason_UserShutdown;
    case RebootReasonCode_FactoryResetShutdown:
      return kMfltRebootReason_FactoryResetShutdown;
    case RebootReasonCode_MfgShutdown:
      return kMfltRebootReason_MfgShutdown;
    case RebootReasonCode_Serial:
      return kMfltRebootReason_Serial;
    case RebootReasonCode_RemoteReset:
      return kMfltRebootReason_RemoteReset;
    case RebootReasonCode_PrfReset:
      return kMfltRebootReason_PrfReset;
    case RebootReasonCode_ForcedCoreDump:
      return kMfltRebootReason_ForcedCoreDump;
    case RebootReasonCode_PrfIdle:
      return kMfltRebootReason_PrfIdle;
    case RebootReasonCode_PrfResetButtonsHeld:
      return kMfltRebootReason_PrfResetButtonsHeld;
    case RebootReasonCode_Watchdog:
      return kMfltRebootReason_SoftwareWatchdog;
    case RebootReasonCode_Assert:
      return kMfltRebootReason_Assert;
    case RebootReasonCode_StackOverflow:
      return kMfltRebootReason_StackOverflow;
    case RebootReasonCode_HardFault:
      return kMfltRebootReason_HardFault;
    case RebootReasonCode_LauncherPanic:
      return kMfltRebootReason_LauncherPanic;
    case RebootReasonCode_ClockFailure: // Not used on 3.x
      return kMfltRebootReason_ClockFailure;
    case RebootReasonCode_AppHardFault: // Not used on 3.x
      return kMfltRebootReason_AppHardFault;
    case RebootReasonCode_EventQueueFull:
      return kMfltRebootReason_EventQueueFull;
    case RebootReasonCode_WorkerHardFault: // Off by default, compile in with WORKER_CRASH_CAUSES_RESET
      return kMfltRebootReason_WorkerHardFault;
    case RebootReasonCode_OutOfMemory:
      return kMfltRebootReason_OutOfMemory;
    case RebootReasonCode_FactoryResetReset:
      return kMfltRebootReason_FactoryResetReset;
    case RebootReasonCode_DialogBootFault:
      return kMfltRebootReason_DialogBootFault;
    case RebootReasonCode_BtCoredump:
      return kMfltRebootReason_BtCoredump;
    case RebootReasonCode_CoreDump:  // Core dump initiated without a more specific reason set
      return kMfltRebootReason_CoreDump;
    case RebootReasonCode_CoreDumpEntryFailed:
      return kMfltRebootReason_CoreDumpEntryFailed;
  }

  return kMfltRebootReason_Unknown;
}

void memfault_reboot_reason_get(sResetBootupInfo *reset_info) {
  // TODO: currently reboot_reason_get() is not implemented on the NRF5
  // platform, see reboot_reason.c
  RebootReason reason = { RebootReasonCode_Unknown, 0 };
  reboot_reason_get(&reason);
  *reset_info = (sResetBootupInfo){
    .reset_reason_reg = reason.code,
    .reset_reason = prv_pbl_reboot_to_mflt_reboot(reason.code),
  };
}
