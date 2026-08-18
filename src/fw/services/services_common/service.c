/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

//! @file services.c
//!
//! This file should control the initialization of the various services in the right order.
//! I'll slowly move initialization routines into here as we continue to refactor services.
//! For now this will just be woefully incomplete.

#include "pbl/services/services_common.h"

#include "mfg/mfg_info.h"

#include "pbl/services/accel_manager.h"
#include "pbl/services/bluetooth/bluetooth_persistent_storage.h"
#include "pbl/services/comm_session/app_session_capabilities.h"
#include "pbl/services/comm_session/default_kernel_sender.h"
#include "pbl/services/comm_session/session.h"
#include "pbl/services/cron.h"
#include "pbl/services/firmware_update.h"
#include "pbl/services/hrm/hrm_manager.h"
#include "pbl/services/light.h"
#include "kernel/remote_input.h"
#include "pbl/services/poll_remote.h"
#include "pbl/services/put_bytes/put_bytes.h"
#include "pbl/services/shared_prf_storage/shared_prf_storage.h"
#include "pbl/services/touch/touch.h"
#include <pbl/drivers/touch/touch_sensor.h>
#include "pbl/services/vibe_pattern.h"
#include "pbl/services/runlevel_impl.h"
#include "pbl/util/size.h"

void services_common_init(void) {
  firmware_update_init();
  put_bytes_init();
  poll_remote_init();
  accel_manager_init();
  light_init();

  cron_service_init();

  shared_prf_storage_init();
  bt_persistent_storage_init();

  comm_default_kernel_sender_init();
#if !defined(CONFIG_RECOVERY_FW)
  comm_session_app_session_capabilities_init();
#endif
  comm_session_init();

  remote_input_init();

  bt_ctl_init();

#ifdef CONFIG_SERVICE_TOUCH
  // Gate on the same symbol that compiles touch.c and its consumers (e.g. the remote input
  // endpoint), so no configuration can reach the touch service before touch_init() has run.
  touch_init();
#endif

#ifdef CONFIG_HRM
  hrm_manager_init();
#endif
}

static struct ServiceRunLevelSetting s_runlevel_settings[] = {
  {
    .set_enable_fn = accel_manager_enable,
    .enable_mask = R_Stationary | R_FirmwareUpdate | R_Normal,
  },
  {
    .set_enable_fn = light_allow,
    .enable_mask = R_LowPower | R_FirmwareUpdate | R_Normal,
  },
  {
    .set_enable_fn = vibe_service_set_enabled,
    .enable_mask = R_LowPower | R_FirmwareUpdate | R_Normal
  },
  {
    .set_enable_fn = bt_ctl_set_enabled,
    .enable_mask = R_FirmwareUpdate | R_Normal,
  },
#if defined(CONFIG_TOUCH) && defined(CONFIG_RECOVERY_FW)
  // Only keep touch enabled on recovery (and so manufacturing as well)
  // Once supported in main firmware, this should be removed.
  {
    .set_enable_fn = touch_sensor_set_enabled,
    .enable_mask = R_Normal,
  },
#endif
#ifdef CONFIG_HRM
  {
    .set_enable_fn = hrm_manager_enable,
    .enable_mask = R_Normal,
  },
#endif
};

void services_common_set_runlevel(RunLevel runlevel) {
  for (size_t i = 0; i < ARRAY_LENGTH(s_runlevel_settings); ++i) {
    struct ServiceRunLevelSetting *service = &s_runlevel_settings[i];
    service->set_enable_fn(((1 << runlevel) & service->enable_mask) != 0);
  }
}

