/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "kernel/panic.h"

#include "kernel/event_loop.h"
#include "kernel/ui/modals/modal_manager.h"
#include "shell/system_app_state_machine.h"
#include <pbl/logging/logging.h>
#include "system/passert.h"

static uint32_t s_current_error = 0;

void launcher_panic(uint32_t error_code) {
  PBL_ASSERT_TASK(PebbleTask_KernelMain);

  s_current_error = error_code;

  PBL_LOG_ERR("!!!SAD WATCH 0x%"PRIX32" SAD WATCH!!!", error_code);

  if (modal_manager_get_top_window()) {
    modal_manager_pop_all();
  }

  modal_manager_set_min_priority(ModalPriorityMax);

  system_app_state_machine_panic();
}

uint32_t launcher_panic_get_current_error(void) {
  return s_current_error;
}

void command_sim_panic_cb(void* data) {
  PebbleEvent event = {
    .type = PEBBLE_PANIC_EVENT,
    .panic = {
      .error_code = (uint32_t)data,
    },
  };
  event_put(&event);
}

extern void command_sim_panic(const char *error_code_str) {
  uint32_t error_code = atoi(error_code_str);

  launcher_task_add_callback(command_sim_panic_cb, (void*) error_code);
}
