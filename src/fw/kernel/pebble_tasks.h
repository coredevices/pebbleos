/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "kernel/memory_layout.h"

#include <stdint.h>

#include "pbl/kernel/msgq.h"
#include "pbl/kernel/thread.h"

//! This is an enumeration of different tasks we've had in our system. Please don't rearrange
//! these numbers! For example, the value of PebbleTask_Timers is hardcoded into our syscall
//! assembly and terrible things will happen if you move this around.
typedef enum PebbleTask {
  PebbleTask_KernelMain,
  PebbleTask_KernelBackground,
  PebbleTask_Worker,
  PebbleTask_App,

  PebbleTask_BTHost,        // Bluetooth Host
  PebbleTask_BTController,  // Bluetooth Controller
  PebbleTask_BTHCI,         // Bluetooth HCI

  PebbleTask_NewTimers,

  PebbleTask_PULSE,

  NumPebbleTask,

  PebbleTask_Unknown
} PebbleTask;

typedef uint16_t PebbleTaskBitset;

_Static_assert((1 << (8*sizeof(PebbleTaskBitset))) >= (1 << NumPebbleTask),
               "The type of PebbleTaskBitset is not wide enough to "
               "track all tasks in the PebbleTask enum");

void pebble_task_register(PebbleTask task, struct pbl_thread *thread);
void pebble_task_unregister(PebbleTask task);

const char* pebble_task_get_name(PebbleTask task);

//! @return a single character that indicates the task
char pebble_task_get_char(PebbleTask task);

PebbleTask pebble_task_get_current(void);

PebbleTask pebble_task_get_task_for_thread(const struct pbl_thread *thread);

//! @return the thread running @p task, or NULL while it is not running.
struct pbl_thread *pebble_task_get_thread(PebbleTask task);

void pebble_task_suspend(PebbleTask task);

//! @return The queue handle to send events to the given task.
struct pbl_msgq *pebble_task_get_to_queue(PebbleTask task);

//! Creates the thread for @p pebble_task with the memory regions it needs;
//! @p attr->regions is filled in here.
struct pbl_thread *pebble_task_create(PebbleTask pebble_task, struct pbl_thread_attr *attr);

void pebble_task_configure_idle_task(void);
