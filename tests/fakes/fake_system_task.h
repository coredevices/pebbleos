/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "pbl/services/system_task.h"
#include "fake_pebble_tasks.h"

#include "pbl/util/list.h"

#include "clar_asserts.h"

#include <string.h>
#include <stdint.h>

typedef struct {
  ListNode node;
  SystemTaskEventCallback callback;
  void *data;
} SystemTaskCallbackNode;

static ListNode *s_system_task_callback_head = NULL;
static bool s_invoke_as_current = false;
static uint32_t system_task_available_space = ~(uint32_t)0;

bool system_task_add_callback(SystemTaskEventCallback cb, void *data) {
  SystemTaskCallbackNode *node = (SystemTaskCallbackNode *) malloc(sizeof(SystemTaskCallbackNode));
  cl_assert(node != NULL);
  list_init(&node->node);

  cl_assert(cb);
  node->callback = cb;
  node->data = data;

  s_system_task_callback_head = list_prepend(s_system_task_callback_head, &node->node);
  cl_assert(s_system_task_callback_head);
  system_task_available_space--;
  return true;
}

bool system_task_add_callback_from_isr(SystemTaskEventCallback cb, void *data,
                                       bool *should_context_switch) {
  *should_context_switch = false;
  return system_task_add_callback(cb, data);
}

bool system_task_add_callback_from_isr_droppable(SystemTaskEventCallback cb, void *data,
                                                 bool *should_context_switch) {
  *should_context_switch = false;
  return system_task_add_callback(cb, data);
}

uint32_t system_task_get_available_space(void) {
  return system_task_available_space;
}

void system_task_set_available_space(uint32_t space) {
  system_task_available_space = space;
}

////////////////////////////////////
// Stub:
//
void stub_invoke_system_task_as_current(void) {
  s_invoke_as_current = !s_invoke_as_current;
}

////////////////////////////////////
// Fake manipulation:
//
void *s_fake_system_task_current_cb;

void fake_system_task_callbacks_invoke(int num_to_invoke) {
  PebbleTask current_task = pebble_task_get_current();
  if (!s_invoke_as_current) {
    stub_pebble_tasks_set_current(PebbleTask_KernelBackground);
  }

  // Start at tail ("oldest" callback):
  SystemTaskCallbackNode *node = (SystemTaskCallbackNode *) list_get_tail(s_system_task_callback_head);
  while (node && num_to_invoke) {
    // do callback first, in case callback enqueues more callbacks
    if (node->callback) {
      s_fake_system_task_current_cb = node->callback;
      node->callback(node->data);
      s_fake_system_task_current_cb = NULL;
    }
    SystemTaskCallbackNode *prev = (SystemTaskCallbackNode *) list_get_prev(&node->node);
    list_remove(&node->node, &s_system_task_callback_head, NULL);
    free(node);
    node = prev;

    system_task_available_space++;
    num_to_invoke--;
  }

  stub_pebble_tasks_set_current(current_task);
}

void fake_system_task_callbacks_invoke_pending(void) {
  // sometimes the cb's may add new jobs so we need to keep looping until no
  // more are left
  while (s_system_task_callback_head) {
    fake_system_task_callbacks_invoke(list_count(s_system_task_callback_head));
  }
}

void fake_system_task_callbacks_cleanup(void) {
  SystemTaskCallbackNode *node = (SystemTaskCallbackNode *) s_system_task_callback_head;
  while (node) {
    SystemTaskCallbackNode *next = (SystemTaskCallbackNode *) list_get_next(&node->node);
    list_remove(&node->node, &s_system_task_callback_head, NULL);
    free(node);
    node = next;
  }
  cl_assert(s_system_task_callback_head == NULL);
}

void system_task_watchdog_feed(void) {
}

uint32_t fake_system_task_count_callbacks(void) {
  return list_count(s_system_task_callback_head);
}

void system_task_enable_raised_priority(bool is_raised) {
}

bool system_task_is_ready_to_run(void) {
  return true;
}

void* system_task_get_current_callback(void) {
  return s_fake_system_task_current_cb;
}
