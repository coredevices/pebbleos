/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "syscall/syscall_internal.h"
#include "system/profiler.h"

#include <cmsis_core.h>


// ------------------------------------------------------------------------------------
// Find node by ptr
static bool prv_ptr_list_filter(ListNode* list_node, void* data) {
  ProfilerNode* node = (ProfilerNode*)list_node;
  return (node == data);
}


ProfilerNode *prv_find_node(ProfilerNode *find_node) {
  ListNode* node = list_find(g_profiler.nodes, prv_ptr_list_filter, (void*)find_node);

  return (ProfilerNode *)node;
}

DEFINE_SYSCALL(void, sys_profiler_init, void) {
  profiler_init();
}

DEFINE_SYSCALL(void, sys_profiler_start, void) {
  profiler_start();
}

DEFINE_SYSCALL(void, sys_profiler_stop, void) {
  profiler_stop();
}

DEFINE_SYSCALL(void, sys_profiler_print_stats, void) {
  profiler_print_stats();
}

DEFINE_SYSCALL(void, sys_profiler_node_start, ProfilerNode *node) {
  if (PRIVILEGE_WAS_ELEVATED) {
    if (!list_contains(g_profiler.nodes, (ListNode *)node)) {
      // Instead of calling syscall_failed(), simply return. If PROFILE_INIT has not been
      // executed yet, there won't be any nodes in the list.
      return;
    }
  }

  node->start = DWT->CYCCNT;
}

DEFINE_SYSCALL(void, sys_profiler_node_stop, ProfilerNode *node) {

  // Capture the cycle count as soon as possible, before we validate the node argument
  uint32_t dwt_cyc_cnt = DWT->CYCCNT;

  if (PRIVILEGE_WAS_ELEVATED) {
    if (!list_contains(g_profiler.nodes, (ListNode *)node)) {
      // Instead of calling syscall_failed(), simply return. If PROFILE_INIT has not been
      // executed yet, there won't be any nodes in the list.
      return;
    }
  }

  profiler_node_stop(node, dwt_cyc_cnt);
}
