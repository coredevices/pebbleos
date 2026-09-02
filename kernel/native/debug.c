/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include <string.h>

#include "kernel.h"

#define STACK_FILL 0xa5a5a5a5u

void pbl_thread_foreach(pbl_thread_info_fn fn, void *ctx) {
  for (struct pbl_thread *t = pbl_all_threads; t != NULL; t = t->backend.all_next) {
    struct pbl_thread_info info = {
      .thread = t,
      .name = t->name,
      .id = (uintptr_t)t,
      .current = (t == pbl_cur),
    };
    if (!info.current) {
      arch_thread_info_regs(t, info.regs);
    }
    fn(&info, ctx);
  }
}

void pbl_thread_saved_regs(const struct pbl_thread *t, struct pbl_thread_saved_regs *regs) {
  arch_thread_saved_regs(t, regs);
}

void pbl_thread_stack_info(const struct pbl_thread *t, struct pbl_thread_stack_info *info) {
  const uint32_t *words = t->stack;
  size_t untouched = 0;
  while (untouched < t->stack_size / sizeof(uint32_t) && words[untouched] == STACK_FILL) {
    untouched++;
  }
  *info = (struct pbl_thread_stack_info){
    .start = (uintptr_t)t->stack,
    .size = t->stack_size,
    .high_water = untouched * sizeof(uint32_t),
  };
}

size_t pbl_thread_stats_snapshot(struct pbl_thread_stats *out, size_t max,
                                 uint32_t *total_run_time) {
  size_t n = 0;
  pbl_irq_lock();
  if (total_run_time) {
    *total_run_time = pbl_uptime_ticks();
  }
  for (struct pbl_thread *t = pbl_all_threads; t != NULL && n < max; t = t->backend.all_next) {
    struct pbl_thread_stack_info stack;
    pbl_thread_stack_info(t, &stack);
    out[n++] = (struct pbl_thread_stats){
      .thread = t,
      .name = t->name,
      .number = t->backend.number,
      .run_time = t->backend.run_time,
      .stack_high_water = stack.high_water,
      .state = (enum pbl_thread_state)t->backend.state,
    };
  }
  pbl_irq_unlock();
  return n;
}

size_t pbl_thread_count(void) {
  size_t n = 0;
  for (struct pbl_thread *t = pbl_all_threads; t != NULL; t = t->backend.all_next) {
    n++;
  }
  return n;
}
