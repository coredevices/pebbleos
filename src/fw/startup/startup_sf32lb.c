/*
 * Copyright 2025 SiFli
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

//! Initial firmware startup, contains the vector table that the bootloader loads.
//! Based on "https://github.com/pfalcon/cortex-uni-startup/blob/master/startup.c"
//! by Paul Sokolovsky (public domain)

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "board/board.h"
#include "mcu/cache.h"
#include "util/attributes.h"

uint32_t __Vectors;

uint32_t BSP_GetOtpBase(void) {
  return MPI2_MEM_BASE;
}

//! These symbols are defined in the linker script for use in initializing
//! the data sections. uint8_t since we do arithmetic with section lengths.
//! These are arrays to avoid the need for an & when dealing with linker symbols.
extern uint8_t __data_load_start[];
extern uint8_t __data_start[];
extern uint8_t __data_end[];
extern uint8_t __bss_start[];
extern uint8_t __bss_end[];
extern uint8_t _estack[];

extern uint8_t __retm_ro_load_start[];
extern uint8_t __retm_ro_start[];
extern uint8_t __retm_ro_end[];

//! Firmware main function, ResetHandler calls this
extern int main(void);

//! STM32 system initialization function, defined in the standard peripheral library
extern void SystemInit(void);

static void boot_uart_tx(char *data, int len) {
  int i;

  for (i = 0; i < len; i++) {
    while ((USART1->ISR & UART_FLAG_TXE) == 0);
    USART1->TDR = (uint32_t)data[i];
  }
}

//! This function is what gets called when the processor first
//! starts execution following a reset event. The data and bss
//! sections are initialized, then we call the firmware's main
//! function
__attribute__((naked)) NORETURN Reset_Handler(void) {
  // Set MSPLIM
  //   __set_MSPLIM((uint32_t)(&__STACK_LIMIT));
  //   __set_PSPLIM((uint32_t)(&__STACK_LIMIT));
  // TODO:
  __set_MSPLIM((uint32_t)(0));
  __set_PSPLIM((uint32_t)(0));
  boot_uart_tx("Reset_Handler\n", 14);

  // Copy data section from flash to RAM
  // memcpy(__data_start, __data_load_start, __data_end - __data_start);
  for (int i = 0; i < (__data_end - __data_start); i++) {
    __data_start[i] = __data_load_start[i];
  }
  boot_uart_tx("data copied\n", 12);

  // memcpy(__retm_ro_start, __retm_ro_load_start, __retm_ro_end - __retm_ro_start);
  for (int i = 0; i < (__retm_ro_end - __retm_ro_start); i++) {
    __retm_ro_start[i] = __retm_ro_load_start[i];
  }
  boot_uart_tx("retm copied\n", 12);

  // Clear the bss section, assumes .bss goes directly after .data
  
  memset(__bss_start, 0, __bss_end - __bss_start);
  boot_uart_tx("bss cleared\n", 12);

  SystemInit();

  icache_enable();
  dcache_enable();

  main();

  // Main shouldn't return
  while (true) {
  }
}
