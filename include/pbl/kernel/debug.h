/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "pbl/kernel/thread.h"

//! Introspection for core dumps, fault handling, the task watchdog and telemetry.

//! Canonical register order; the core dump format depends on it.
enum pbl_thread_reg {
  PBL_THREAD_REG_R0,
  PBL_THREAD_REG_R1,
  PBL_THREAD_REG_R2,
  PBL_THREAD_REG_R3,
  PBL_THREAD_REG_R4,
  PBL_THREAD_REG_R5,
  PBL_THREAD_REG_R6,
  PBL_THREAD_REG_R7,
  PBL_THREAD_REG_R8,
  PBL_THREAD_REG_R9,
  PBL_THREAD_REG_R10,
  PBL_THREAD_REG_R11,
  PBL_THREAD_REG_R12,
  PBL_THREAD_REG_SP,
  PBL_THREAD_REG_LR,
  PBL_THREAD_REG_PC,
  PBL_THREAD_REG_XPSR,
  PBL_THREAD_REG_COUNT,
};

struct pbl_thread_info {
  struct pbl_thread *thread;  // NULL for threads the kernel created itself
  const char *name;
  uintptr_t id;
  bool current;
  uint32_t regs[PBL_THREAD_REG_COUNT];
};

typedef void (*pbl_thread_info_fn)(const struct pbl_thread_info *info, void *ctx);

//! Visits every thread with its saved registers. Safe to call from a fault handler.
void pbl_thread_foreach(pbl_thread_info_fn fn, void *ctx);

//! Registers of a thread that is not running, read from its saved context.
struct pbl_thread_saved_regs {
  uintptr_t pc;
  uintptr_t lr;
  uint32_t control;
};

void pbl_thread_saved_regs(const struct pbl_thread *t, struct pbl_thread_saved_regs *regs);

struct pbl_thread_stack_info {
  uintptr_t start;   // lowest address
  size_t size;       // bytes
  size_t high_water; // bytes never used
};

void pbl_thread_stack_info(const struct pbl_thread *t, struct pbl_thread_stack_info *info);

struct pbl_thread_stats {
  struct pbl_thread *thread;
  const char *name;
  uint32_t number;
  uint32_t run_time;
  size_t stack_high_water;  // bytes
  enum pbl_thread_state state;
};

//! Snapshot of every thread. @return the number of entries written.
size_t pbl_thread_stats_snapshot(struct pbl_thread_stats *out, size_t max, uint32_t *total_run_time);

size_t pbl_thread_count(void);
