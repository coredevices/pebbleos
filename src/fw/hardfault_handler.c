/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "logging/logging_private.h"
#include "system/die.h"
#include "system/reboot_reason.h"
#include "system/reset.h"
#include "pbl/util/attributes.h"
#include "util/bitset.h"
#include "pbl/util/size.h"
#include "pbl/util/string.h"

#include <cmsis_core.h>

#include <inttypes.h>
#include <stdarg.h>
#include <stdio.h>

#include "kernel/pebble_tasks.h"

void fault_handler_dump_stacked_args(char buffer[80], unsigned int* stacked_args) {
  unsigned int stacked_r0 = ((unsigned long) stacked_args[0]);
  unsigned int stacked_r1 = ((unsigned long) stacked_args[1]);
  unsigned int stacked_r2 = ((unsigned long) stacked_args[2]);
  unsigned int stacked_r3 = ((unsigned long) stacked_args[3]);

  unsigned int stacked_r12 = ((unsigned long) stacked_args[4]);
  unsigned int stacked_lr = ((unsigned long) stacked_args[5]);
  unsigned int stacked_pc = ((unsigned long) stacked_args[6]);
  unsigned int stacked_psr = ((unsigned long) stacked_args[7]);

  PBL_LOG_FROM_FAULT_HANDLER_FMT(buffer, 80, "R0 = 0x%x", stacked_r0);
  PBL_LOG_FROM_FAULT_HANDLER_FMT(buffer, 80, "R1 = 0x%x", stacked_r1);
  PBL_LOG_FROM_FAULT_HANDLER_FMT(buffer, 80, "R2 = 0x%x", stacked_r2);
  PBL_LOG_FROM_FAULT_HANDLER_FMT(buffer, 80, "R3 = 0x%x", stacked_r3);
  PBL_LOG_FROM_FAULT_HANDLER_FMT(buffer, 80, "R12 = 0x%x", stacked_r12);
  PBL_LOG_FROM_FAULT_HANDLER_FMT(buffer, 80, "SP = %p", &stacked_args[8]);
  PBL_LOG_FROM_FAULT_HANDLER_FMT(
      buffer, 80, "LR [R14] = 0x%x  subroutine call return address", stacked_lr);
  PBL_LOG_FROM_FAULT_HANDLER_FMT(
      buffer, 80, "PC [R15] = 0x%x  program counter", stacked_pc);
  PBL_LOG_FROM_FAULT_HANDLER_FMT(buffer, 80, "PSR = 0x%x", stacked_psr);

  //BREAKPOINT;

  // NOTE: If you want to get a stack trace at this point. Set a breakpoint here (you can compile in the above
  // BREAKPOINT call if you want) and issue the following commands in gdb:
  //    set var $sp=<value of SP above>
  //    set var $lr=<value of LR above>
  //    set var $pc=<value of PC above>
  //    bt
}

typedef struct IndexToName {
  unsigned int index;
  const char *name;
} IndexToName;

static void print_set_indexes(char buffer[80], const uint8_t *bitset, const IndexToName *mappings, unsigned int num_mappings) {
  for (unsigned int i = 0; i < num_mappings; ++i) {
    if (bitset8_get(bitset, mappings[i].index)) {
      PBL_LOG_FROM_FAULT_HANDLER_FMT(buffer, 80, "    %s = yes", mappings[i].name);
    }
  }
}

void fault_handler_dump_cfsr(char buffer[80]) {
  // See http://infocenter.arm.com/help/topic/com.arm.doc.dui0552a/DUI0552A_cortex_m3_dgug.pdf for the register
  // definition.

  const uint32_t cfsr = SCB->CFSR;
  PBL_LOG_FROM_FAULT_HANDLER_FMT(buffer, 80, "CFSR (Configurable Fault) = 0x%"PRIx32, cfsr);

  // Usage Fault Status Register
  const uint16_t ufsr = (cfsr >> 16) & 0xffff;
  static const IndexToName ufsr_mappings[] = {
    { 9, "DIVBYZERO" },
    { 8, "UNALIGNED" },
    { 3, "NOCP" },
    { 2, "INVPC" },
    { 1, "INVSTATE" },
    { 0, "UNDEFINSTR" }
  };
  if (ufsr != 0) {
    PBL_LOG_FROM_FAULT_HANDLER("  Usage Fault Status Register:");
    print_set_indexes(buffer, (const uint8_t*) &ufsr, ufsr_mappings, ARRAY_LENGTH(ufsr_mappings));
  }

  // Bus Fault Status Register
  const uint8_t bfsr = (cfsr >> 8) & 0xff;
  static const IndexToName bfsr_mappings[] = {
    { 4, "STKERR" },
    { 3, "UNSTKERR" },
    { 2, "IMPRECISERR" },
    { 1, "PRECISERR" },
    { 0, "IBUSERR" },
  };
  if (bfsr != 0) {
    PBL_LOG_FROM_FAULT_HANDLER("  Bus Fault Status Register:");

    if (bfsr & (1 << 7)) {
      PBL_LOG_FROM_FAULT_HANDLER_FMT(buffer, 80, "    BFARVALID = yes 0x%"PRIx32, SCB->BFAR);
    }

    print_set_indexes(buffer, &bfsr, bfsr_mappings, ARRAY_LENGTH(bfsr_mappings));
  }

  // Memory Management Fault Status Register
  const uint8_t mmfsr = cfsr & 0xff;
  static const IndexToName mmfsr_mappings[] = {
    { 4, "MSTKERR" },
    { 3, "MUNSTKERR" },
    { 1, "DACCVIOL" },
    { 0, "IACCVIOL" }
  };
  if (mmfsr != 0) {
    PBL_LOG_FROM_FAULT_HANDLER("  Memory Management Fault Status Register:");

    if (mmfsr & (1 << 7)) {
      PBL_LOG_FROM_FAULT_HANDLER_FMT(buffer, 80, "    MMFARVALID = yes 0x%"PRIx32, SCB->MMFAR);
    }

    print_set_indexes(buffer, &mmfsr, mmfsr_mappings, ARRAY_LENGTH(mmfsr_mappings));
  }
}

void fault_handler_dump(char buffer[80], unsigned int *stacked_args) {
  fault_handler_dump_stacked_args(buffer, stacked_args);
  fault_handler_dump_cfsr(buffer);
  PBL_LOG_FROM_FAULT_HANDLER_FMT(
      buffer, 80, "Task: %s", pebble_task_get_name(pebble_task_get_current()));
}

static void hard_fault_handler_c(unsigned int* hardfault_args) {
  // Prefer LR (PC is often madness on a hardfault). Fall back through PC,
  // BFAR, MMFAR so crash reports always have a non-zero address to
  // fingerprint on.
  unsigned int stacked_lr = ((unsigned long) hardfault_args[5]);
  if (stacked_lr == 0) {
    stacked_lr = ((unsigned long) hardfault_args[6]);
  }
  if (stacked_lr == 0 && (SCB->CFSR & (1 << 15))) {  // BFARVALID
    stacked_lr = SCB->BFAR;
  }
  if (stacked_lr == 0 && (SCB->CFSR & (1 << 7))) {  // MMFARVALID
    stacked_lr = SCB->MMFAR;
  }
  RebootReason reason = { .code = RebootReasonCode_HardFault, .extra = { .value = stacked_lr } };
  reboot_reason_set(&reason);

  // Yay, ripping stuff from the internet!
  // http://blog.frankvh.com/2011/12/07/cortex-m3-m4-hard-fault-handler/
  char buffer[80];

  // To inspect the SCB in GDB: p (*((SCB_Type *) 0xE000ED00))

  PBL_LOG_FROM_FAULT_HANDLER("\r\n\r\n[Hard fault handler - You dun goofed]");

  PBL_LOG_FROM_FAULT_HANDLER_FMT(
      buffer, 80, "SHCSR (System Handler)    = 0x%"PRIx32, SCB->SHCSR);
  PBL_LOG_FROM_FAULT_HANDLER_FMT(
      buffer, 80, "HFSR (Hard Fault)         = 0x%"PRIx32, SCB->HFSR);
  PBL_LOG_FROM_FAULT_HANDLER_FMT(
      buffer, 80, "    Forced = %s", bool_to_str(SCB->HFSR & SCB_HFSR_FORCED_Msk));

  fault_handler_dump(buffer, hardfault_args);

  reset_due_to_software_failure();
}

void HardFault_Handler(void) {
  // Grab the stack pointer, shove it into a register and call
  // the c function above.
  __asm("tst lr, #4\n"
        "ite eq\n"
        "mrseq r0, msp\n"
        "mrsne r0, psp\n"
        "b %0\n" :: "i" (hard_fault_handler_c));
}
