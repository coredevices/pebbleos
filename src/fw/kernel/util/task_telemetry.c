/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "pbl/kernel/debug.h"
#include "pbl/util/size.h"
#include "console/prompt.h"
#include "kernel/pbl_malloc.h"

#if 0
void command_print_task_list(void) {
  char str_buffer[100];

  prompt_send_response("name                  state    pri   fstk        num");

  struct pbl_thread_stats stats[CONFIG_KERNEL_MAX_THREADS];
  size_t count = pbl_thread_stats_snapshot(stats, ARRAY_LENGTH(stats), NULL);

  for (size_t i = 0; i < count; i++) {
    char status;
    switch (stats[i].state) {
      case PBL_THREAD_READY:
        status = 'R';
        break;
      case PBL_THREAD_RUNNING:
        status = 'X';
        break;
      case PBL_THREAD_BLOCKED:
        status = 'B';
        break;
      case PBL_THREAD_SUSPENDED:
        status = 'S';
        break;
      case PBL_THREAD_DEAD:
        status = 'D';
        break;
      default:
        status = '?';
        break;
    }

    prompt_send_response_fmt(str_buffer, sizeof(str_buffer), "%-16s %6c %8u %8u %8u",
                             stats[i].name,
                             status,
                             stats[i].thread ? (unsigned int)stats[i].thread->prio : 0U,
                             (unsigned int)stats[i].stack_high_water,
                             (unsigned int)stats[i].number);
  }
}
#endif
