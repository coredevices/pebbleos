/* SPDX-FileCopyrightText: 2025 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include <stdint.h>

#include "system/passert.h"
#include "pbl/util/attributes.h"

#include <bf0_hal.h>

//! These symbols are defined in the linker script for use in initializing
//! the data sections. uint8_t since we do arithmetic with section lengths.
//! These are arrays to avoid the need for an & when dealing with linker symbols.
extern uint8_t __data_load_start[];
extern uint8_t __data_start[];
extern uint8_t __data_end[];
extern uint8_t __bss_start[];
extern uint8_t __bss_end[];

extern uint8_t __ramfunc_load_start[];
extern uint8_t __ramfunc_start[];
extern uint8_t __ramfunc_end[];

extern uint8_t __isr_stack_start__[];

extern int main(void);

NAKED_FUNC NORETURN Reset_Handler(void) {
  // Set MSPLIM to protect the ISR stack
  __set_MSPLIM((uint32_t)__isr_stack_start__);
  // PSPLIM is set per-task by FreeRTOS during context switches
  __set_PSPLIM((uint32_t)(0));

  // Copy data section from flash to RAM
  for (int i = 0; i < (__data_end - __data_start); i++) {
    __data_start[i] = __data_load_start[i];
  }

  for (int i = 0; i < (__ramfunc_end - __ramfunc_start); i++) {
    __ramfunc_start[i] = __ramfunc_load_start[i];
  }

  // Clear the bss section, assumes .bss goes directly after .data
  memset(__bss_start, 0, __bss_end - __bss_start);

  SystemInit();

  main();

  PBL_CROAK("main returned, this should never happen");
}
