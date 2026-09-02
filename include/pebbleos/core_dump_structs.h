/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

/*
 * core_dump_structs.h
 *
 * This file specifies core_dump structures previously defined in fw/kernel/core_dump_private.h
 * This is so the Dialog BLE core_dump code can use the same structures.
 *
 */

#include "portmacro.h"
#include "pbl/util/attributes.h"

// r0-r12, sp, lr, pc, xpsr
#define CORE_DUMP_NUM_REGISTERS 17

// Structure of thread info stored within a CORE_DUMP_CHUNK_KEY_THREAD chunk in the core dump
#define CORE_DUMP_THREAD_NAME_SIZE  16
typedef struct PACKED {
  int8_t    name[CORE_DUMP_THREAD_NAME_SIZE];       // Name, includes null termination
  uint32_t  id;             // thread id
  uint8_t   running;        // true if this thread is running
  uint32_t  registers[CORE_DUMP_NUM_REGISTERS];  // registers [r0-r12, sp, lr, pc, xpsr]
} CoreDumpThreadInfo;

// Structure of extra registers stored within a CORE_DUMP_CHUNK_KEY_EXTRA_REG chunk in the
// core dump
typedef struct PACKED {
  uint32_t  msp;
  uint32_t  psp;
  uint8_t   primask;
  uint8_t   basepri;
  uint8_t   faultmask;
  uint8_t   control;
} CoreDumpExtraRegInfo;



// We save all the important registers on entry to core_dump_reset() in a structure of this type
// on the core_dump_reset() stack and save a pointer to it in the s_saved_registers global.
// IMPORTANT!: There is assembly code near the top of core_dump_reset() that makes assumptions
// about the order and packing of this structure.
typedef struct PACKED {
  uint32_t core_reg[CORE_DUMP_NUM_REGISTERS];
  CoreDumpExtraRegInfo extra_reg;
} CoreDumpSavedRegisters;
