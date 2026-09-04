/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "test_args_rx.h"
#include "test_args_tx.h"

#include "process_management/process_manager.h"
#include <pbl/logging/logging.h>

static void s_main(void) {
  const TestArgsData *args = process_manager_get_current_process_args();
  if (args == NULL) {
    PBL_LOG_DBG("Got no args.");
  } else {
    PBL_LOG_DBG("Got argument 0x%x", args->data);
  }
}

const PebbleProcessMd* test_args_receiver_get_app_info() {
  static const PebbleProcessMdSystem test_args_receiver_demo_app_info = {
    .common.uuid = {0x48, 0xBB, 0xB5, 0x04, 0x5A, 0x56, 0x40, 0x73, 0xAF,
      0xEA, 0x5D, 0x83, 0x8D, 0x43, 0x01, 0xA4},
    .common.main_func = s_main,
    .name = "Args Receiver Demo"
  };
  return (const PebbleProcessMd*) &test_args_receiver_demo_app_info;
}
