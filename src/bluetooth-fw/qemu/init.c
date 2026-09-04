/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "comm/bt_lock.h"
#include <pbl/drivers/qemu/qemu_serial.h>
#include <pbl/drivers/qemu/qemu_settings.h>
#include "kernel/event_loop.h"
#include <pbl/logging/logging.h>

#include <bluetooth/init.h>
#include <bluetooth/qemu_transport.h>

// ----------------------------------------------------------------------------------------
void bt_driver_init(void) {
  // We need the QEMU serial driver
  qemu_serial_init();
  bt_lock_init();
}

bool bt_driver_start(BTDriverConfig *config) {
  // For QEMU there's no "disconnected" state — the host process is always
  // attached.  Used to defer this to a launcher_task callback so app_message
  // callbacks were registered first, but that opened a race where the host
  // sent WatchVersionRequest after seeing "Ready for communication" but
  // before the deferred callback ran, and the request was silently dropped.
  // Open the session synchronously; the WatchVersionRequest path is at the
  // comm_session layer and doesn't require app_message to be wired up.
  if (qemu_setting_get(QemuSetting_DefaultConnected)) {
    qemu_transport_set_connected(true);
  }
  return true;
}

void bt_driver_stop(void) {
  qemu_transport_set_connected(false);
  qemu_transport_close_session();
}

void bt_driver_power_down_controller_on_boot(void) {
  // no-op
}
