/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "pbl_malloc.h"

#include "kernel_heap.h"
#include "pebble_tasks.h"

#include "process_management/app_manager.h"
#include "process_state/app_state/app_state.h"
#include "process_state/worker_state/worker_state.h"
#include "system/passert.h"
#include "pbl/util/heap.h"
#include "pbl/util/misc.h"

#include <string.h>

Heap *task_heap_get_for_current_task(void) {
  if (pebble_task_get_current() == PebbleTask_App) {
    return app_state_get_heap();
  } else if (pebble_task_get_current() == PebbleTask_Worker) {
    return worker_state_get_heap();
  }
  return kernel_heap_get();
}

static char* prv_strdup(Heap *heap, const char* s, uintptr_t lr) {
  char *dup = heap_zalloc(heap, strlen(s) + 1, lr);
  if (dup) {
    strcpy(dup, s);
  }
  return dup;
}

// task_* functions that map to other heaps depending on the current task
///////////////////////////////////////////////////////////
#if defined(CONFIG_MALLOC_INSTRUMENTATION)
void *task_malloc_with_pc(size_t bytes, uintptr_t client_pc) {
  return heap_malloc(task_heap_get_for_current_task(), bytes, client_pc);
}
#endif

void *task_malloc(size_t bytes) {
  uintptr_t saved_lr = PBL_RETURN_ADDRESS();

  return heap_malloc(task_heap_get_for_current_task(), bytes, saved_lr);
}

void *task_malloc_check(size_t bytes) {
  uintptr_t saved_lr = PBL_RETURN_ADDRESS();

  Heap *heap = task_heap_get_for_current_task();
  void *mem = heap_malloc(heap, bytes, saved_lr);

  if (!mem && bytes != 0) {
    PBL_CROAK_OOM(bytes, saved_lr, heap);
  }
  return mem;
}

#if defined(CONFIG_MALLOC_INSTRUMENTATION)
void task_free_with_pc(void *ptr, uintptr_t client_pc) {
  heap_free(task_heap_get_for_current_task(), ptr, client_pc);
}
#endif

void task_free(void* ptr) {
  uintptr_t saved_lr = PBL_RETURN_ADDRESS();

  heap_free(task_heap_get_for_current_task(), ptr, saved_lr);
}

void *task_realloc(void* ptr, size_t size) {
  uintptr_t saved_lr = PBL_RETURN_ADDRESS();

  return heap_realloc(task_heap_get_for_current_task(), ptr, size, saved_lr);
}

void *task_zalloc(size_t size) {
  uintptr_t saved_lr = PBL_RETURN_ADDRESS();

  return heap_zalloc(task_heap_get_for_current_task(), size, saved_lr);
}

void *task_zalloc_check(size_t bytes) {
  uintptr_t saved_lr = PBL_RETURN_ADDRESS();

  Heap *heap = task_heap_get_for_current_task();
  void *mem = heap_zalloc(heap, bytes, saved_lr);

  if (!mem && bytes != 0) {
    PBL_CROAK_OOM(bytes, saved_lr, heap);
  }
  return mem;
}

void *task_calloc(size_t count, size_t size) {
  uintptr_t saved_lr = PBL_RETURN_ADDRESS();

  return heap_calloc(task_heap_get_for_current_task(), count, size, saved_lr);
}

void *task_calloc_check(size_t count, size_t size) {
  uintptr_t saved_lr = PBL_RETURN_ADDRESS();

  Heap *heap = task_heap_get_for_current_task();
  void *mem = heap_calloc(heap, count, size, saved_lr);

  const size_t bytes = count * size;
  if (!mem && bytes != 0) {
    PBL_CROAK_OOM(bytes, saved_lr, heap);
  }
  return mem;
}

char *task_strdup(const char *s) {
  uintptr_t saved_lr = PBL_RETURN_ADDRESS();

  return prv_strdup(task_heap_get_for_current_task(), s, saved_lr);
}

// app_* functions that allocate on the app heap
///////////////////////////////////////////////////////////
void *app_malloc(size_t bytes) {
  uintptr_t saved_lr = PBL_RETURN_ADDRESS();

  return heap_malloc(app_state_get_heap(), bytes, saved_lr);
}

void *app_malloc_check(size_t bytes) {
  uintptr_t saved_lr = PBL_RETURN_ADDRESS();

  Heap *heap = app_state_get_heap();
  void *mem = heap_malloc(heap, bytes, saved_lr);
  if (!mem && bytes != 0) {
    PBL_CROAK_OOM(bytes, saved_lr, heap);
  }
  return mem;
}

void app_free(void *ptr) {
  uintptr_t saved_lr = PBL_RETURN_ADDRESS();

  heap_free(app_state_get_heap(), ptr, saved_lr);
}

void *app_realloc(void *ptr, size_t bytes) {
  uintptr_t saved_lr = PBL_RETURN_ADDRESS();

  return heap_realloc(app_state_get_heap(), ptr, bytes, saved_lr);
}

void *app_zalloc(size_t size) {
  uintptr_t saved_lr = PBL_RETURN_ADDRESS();

  return heap_zalloc(app_state_get_heap(), size, saved_lr);
}

void *app_zalloc_check(size_t bytes) {
  uintptr_t saved_lr = PBL_RETURN_ADDRESS();

  Heap *heap = app_state_get_heap();
  void *mem = heap_zalloc(heap, bytes, saved_lr);

  if (!mem && bytes != 0) {
    PBL_CROAK_OOM(bytes, saved_lr, heap);
  }
  return mem;
}

void *app_calloc(size_t count, size_t size) {
  uintptr_t saved_lr = PBL_RETURN_ADDRESS();

  return (heap_calloc(app_state_get_heap(), count, size, saved_lr));
}

void *app_calloc_check(size_t count, size_t size) {
  uintptr_t saved_lr = PBL_RETURN_ADDRESS();

  Heap *heap = app_state_get_heap();
  void *mem = heap_calloc(heap, count, size, saved_lr);

  const size_t bytes = count * size;
  if (!mem && bytes != 0) {
    PBL_CROAK_OOM(bytes, saved_lr, heap);
  }
  return mem;
}

char *app_strdup(const char *s) {
  uintptr_t saved_lr = PBL_RETURN_ADDRESS();

  return prv_strdup(app_state_get_heap(), s, saved_lr);
}

// kernel_* functions that allocate on the kernel heap
///////////////////////////////////////////////////////////
void *kernel_malloc(size_t bytes) {
  uintptr_t saved_lr = PBL_RETURN_ADDRESS();

  return heap_malloc(kernel_heap_get(), bytes, saved_lr);
}

void *kernel_malloc_check(size_t bytes) {
  uintptr_t saved_lr = PBL_RETURN_ADDRESS();

  Heap *heap = kernel_heap_get();
  void *mem = heap_malloc(heap, bytes, saved_lr);
  if (!mem && bytes != 0) {
    PBL_CROAK_OOM(bytes, saved_lr, heap);
  }
  return mem;
}

void *kernel_calloc(size_t count, size_t size) {
  uintptr_t saved_lr = PBL_RETURN_ADDRESS();

  return (heap_calloc(kernel_heap_get(), count, size, saved_lr));
}

void *kernel_calloc_check(size_t count, size_t size) {
  uintptr_t saved_lr = PBL_RETURN_ADDRESS();

  Heap *heap = kernel_heap_get();
  void *mem = heap_calloc(heap, count, size, saved_lr);

  const size_t bytes = count * size;
  if (!mem && bytes != 0) {
    PBL_CROAK_OOM(bytes, saved_lr, heap);
  }
  return mem;
}

void *kernel_realloc(void *ptr, size_t bytes) {
  uintptr_t saved_lr = PBL_RETURN_ADDRESS();

  return heap_realloc(kernel_heap_get(), ptr, bytes, saved_lr);
}

void *kernel_zalloc(size_t size) {
  uintptr_t saved_lr = PBL_RETURN_ADDRESS();

  return heap_zalloc(kernel_heap_get(), size, saved_lr);
}

void *kernel_zalloc_check(size_t bytes) {
  uintptr_t saved_lr = PBL_RETURN_ADDRESS();

  Heap *heap = kernel_heap_get();
  void *mem = heap_zalloc(heap, bytes, saved_lr);

  if (!mem && bytes != 0) {
    PBL_CROAK_OOM(bytes, saved_lr, heap);
  }
  return mem;
}

void kernel_free(void *ptr) {
  uintptr_t saved_lr = PBL_RETURN_ADDRESS();

  heap_free(kernel_heap_get(), ptr, saved_lr);
}

char *kernel_strdup(const char *s) {
  uintptr_t saved_lr = PBL_RETURN_ADDRESS();

  return prv_strdup(kernel_heap_get(), s, saved_lr);
}

char *kernel_strdup_check(const char *s) {
  uintptr_t saved_lr = PBL_RETURN_ADDRESS();

  Heap *heap = kernel_heap_get();
  void *mem = prv_strdup(heap, s, saved_lr);
  if (!mem) {
    PBL_CROAK_OOM(strlen(s) + 1, saved_lr, heap);
  }

  return mem;
}

// Wrappers (Jay-Z, Tupac, etc)
// We want to keep these around for code bases that we don't own. For example, libc will want
// malloc to exist, and libc should use the appropriate heap based on the task.
////////////////////////////////////////////////////////////
void *__wrap_malloc(size_t bytes) {
  uintptr_t saved_lr = PBL_RETURN_ADDRESS();

  return heap_malloc(task_heap_get_for_current_task(), bytes, saved_lr);
}

void __wrap_free(void *ptr) {
  uintptr_t saved_lr = PBL_RETURN_ADDRESS();

  heap_free(task_heap_get_for_current_task(), ptr, saved_lr);
}

void *__wrap_realloc(void *ptr, size_t size) {
  uintptr_t saved_lr = PBL_RETURN_ADDRESS();

  return heap_realloc(task_heap_get_for_current_task(), ptr, size, saved_lr);
}

void *__wrap_calloc(size_t count, size_t size) {
  uintptr_t saved_lr = PBL_RETURN_ADDRESS();

  return heap_calloc(task_heap_get_for_current_task(), count, size, saved_lr);
}

