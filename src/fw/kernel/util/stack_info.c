/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "pbl/kernel/debug.h"

#include "pbl/mcu/interrupts.h"

extern uint32_t __isr_stack_start__[];

uint32_t stack_free_bytes(void) {

  // Get the current SP
  register uint32_t SP __asm ("sp");
  uint32_t cur_sp = SP;

  // Default stack
  uint32_t start = (uint32_t) __isr_stack_start__;

  // On ISR stack?
  if (!mcu_state_is_isr()) {
    struct pbl_thread *thread = pbl_thread_current();
    if (thread != NULL) {
      // NULL before the first thread starts
      struct pbl_thread_stack_info info;
      pbl_thread_stack_info(thread, &info);
      start = (uint32_t)info.start;
    }
  }

  return cur_sp - start;
}
