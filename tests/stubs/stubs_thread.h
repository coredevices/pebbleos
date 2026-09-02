/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include <pbl/kernel/debug.h>
#include <pbl/kernel/thread.h>

static uint32_t s_app_task_control_reg = 0;

int pbl_thread_create(struct pbl_thread *t, const struct pbl_thread_attr *attr) {
  return 0;
}

void pbl_thread_abort(struct pbl_thread *t) {
}

void pbl_thread_suspend(struct pbl_thread *t) {
}

void pbl_thread_resume(struct pbl_thread *t) {
}

void pbl_thread_yield(void) {
}

void pbl_thread_sleep(pbl_timeout_t timeout) {
}

struct pbl_thread *pbl_thread_current(void) {
  return NULL;
}

struct pbl_thread *pbl_thread_idle(void) {
  return NULL;
}

void pbl_thread_prio_set(struct pbl_thread *t, pbl_prio_t prio) {
}

pbl_prio_t pbl_thread_prio_get(const struct pbl_thread *t) {
  return 0;
}

enum pbl_thread_state pbl_thread_state(const struct pbl_thread *t) {
  return PBL_THREAD_READY;
}

void pbl_thread_regions_set(struct pbl_thread *t, const MpuRegion *const *regions) {
}

void pbl_thread_saved_regs(const struct pbl_thread *t, struct pbl_thread_saved_regs *regs) {
  *regs = (struct pbl_thread_saved_regs) { .control = s_app_task_control_reg };
}

void pbl_thread_stack_info(const struct pbl_thread *t, struct pbl_thread_stack_info *info) {
  *info = (struct pbl_thread_stack_info) { 0 };
}

// Stubs
////////////////////////////////////

void stub_control_reg(uint32_t reg) {
  s_app_task_control_reg = reg;
}
