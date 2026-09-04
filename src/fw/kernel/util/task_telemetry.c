/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "console/prompt.h"
#include "kernel/pbl_malloc.h"

#include "FreeRTOS.h"
#include "task.h"

#if 0
void command_print_task_list(void) {
#if ( configUSE_TRACE_FACILITY == 1 )
  char str_buffer[100];

  prompt_send_response(
      "name                  state    pri   fstk        num   stk_beg     stk_ptr");

  int num_tasks = uxTaskGetNumberOfTasks();
  TaskStatus_t *task_info = kernel_malloc(num_tasks * sizeof( TaskStatus_t ));
  if (!task_info) {
    return;
  }
  num_tasks = uxTaskGetSystemState( task_info, num_tasks, NULL );

  // Print info on each task
  char status;
  for (int i=0; i<num_tasks; i++) {
    switch(task_info[i].eCurrentState) {
      case eReady:
        status = 'R';
        break;
      case eBlocked:
        status = 'B';
        break;
      case eSuspended:
        status = 'S';
        break;
      case eDeleted:
        status = 'D';
        break;
      default:
        status = '?';
        break;
    }

    prompt_send_response_fmt(str_buffer, sizeof(str_buffer), "%-16s %6c %8u %8u %8u     %p  %p",
                             task_info[i].pcTaskName,
                             status,
                             (unsigned int) task_info[i].uxCurrentPriority,
                             (unsigned int)(sizeof(StackType_t) * task_info[i].usStackHighWaterMark),
                             (unsigned int)task_info[i].xTaskNumber,
                             (void *)task_info[i].pxStack,
                             (void *)task_info[i].pxTopOfStack);
  }
  kernel_free(task_info);

#else
  prompt_send_response("Not available");
#endif
}
#endif
