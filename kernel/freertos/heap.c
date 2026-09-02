/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "pbl/os/malloc.h"

#include "kernel_freertos.h"

// FreeRTOS control blocks come from the application heap.
#undef pvPortMalloc
void *pvPortMalloc(size_t size) { return os_malloc(size); }

#undef vPortFree
void vPortFree(void *ptr) { os_free(ptr); }
