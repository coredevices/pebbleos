/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "pbl/drivers/mpu.h"
#include "pbl/kernel/types.h"

#define PBL_THREAD_NAME_LEN 16
#define PBL_THREAD_MAX_MEM_REGIONS 4

typedef void (*pbl_thread_entry_t)(void *arg);

enum pbl_thread_state {
  PBL_THREAD_READY,
  PBL_THREAD_RUNNING,
  PBL_THREAD_BLOCKED,
  PBL_THREAD_SUSPENDED,
  PBL_THREAD_DEAD,
};

struct pbl_thread_attr {
  const char *name;
  pbl_thread_entry_t entry;
  void *arg;
  pbl_prio_t prio;
  bool privileged;
  void *stack;        // lowest address; caller owns the memory
  size_t stack_size;  // bytes
  //! MPU regions switched in with the thread. NULL entries are ignored.
  const MpuRegion *regions[PBL_THREAD_MAX_MEM_REGIONS];
};

struct pbl_thread {
  struct pbl_thread_backend backend;  // first: the arch code relies on its offset
  uint32_t id;  // unique per creation, never 0
  char name[PBL_THREAD_NAME_LEN];
  pbl_prio_t prio;
  bool privileged;
  void *stack;
  size_t stack_size;
  void *tls[CONFIG_KERNEL_THREAD_TLS_SLOTS];
};

//! Declare a stack with the alignment the MPU port needs for a guard region.
#define PBL_THREAD_STACK_DEFINE(name, size) \
  static uint8_t name[size] __attribute__((aligned(CONFIG_KERNEL_STACK_ALIGN)))

//! Returning from the entry function ends the thread.
int pbl_thread_create(struct pbl_thread *t, const struct pbl_thread_attr *attr);
void pbl_thread_abort(struct pbl_thread *t);  // NULL = self
void pbl_thread_suspend(struct pbl_thread *t);  // NULL = self
void pbl_thread_resume(struct pbl_thread *t);
void pbl_thread_yield(void);
void pbl_thread_sleep(pbl_timeout_t timeout);

//! @return the running thread, or the interrupted thread from an ISR.
struct pbl_thread *pbl_thread_current(void);
struct pbl_thread *pbl_thread_idle(void);

void pbl_thread_prio_set(struct pbl_thread *t, pbl_prio_t prio);
pbl_prio_t pbl_thread_prio_get(const struct pbl_thread *t);
enum pbl_thread_state pbl_thread_state(const struct pbl_thread *t);

static inline const char *pbl_thread_name(const struct pbl_thread *t) { return t->name; }

//! Distinguishes successive threads created in the same struct.
static inline uint32_t pbl_thread_id(const struct pbl_thread *t) { return t->id; }

//! Replaces the MPU regions of a thread; used for the idle thread, whose
//! regions cannot be passed at creation.
void pbl_thread_regions_set(struct pbl_thread *t, const MpuRegion *const *regions);

static inline void *pbl_thread_tls_get(const struct pbl_thread *t, unsigned int slot) {
  return t->tls[slot];
}
static inline void pbl_thread_tls_set(struct pbl_thread *t, unsigned int slot, void *v) {
  t->tls[slot] = v;
}

//! Implemented by the application; called when a thread overran its stack.
void pbl_thread_stack_overflow(struct pbl_thread *t, const char *name);

//! Implemented by the application; whether the code at @p caller_pc may raise
//! privilege through the kernel's supervisor call.
bool pbl_kernel_privilege_raise_allowed(uint32_t caller_pc);

//! Implemented by the application: top of a dedicated privileged stack for
//! the current thread's syscalls (base in @p base_out), or NULL to run them
//! on the caller's stack.
uint32_t *pbl_kernel_syscall_stack(uintptr_t *base_out);

//! Implemented by the application; called once privilege has been raised
//! with the caller's stack pointer and the stacked return address slot.
void pbl_kernel_syscall_entered(uintptr_t orig_sp, uintptr_t *lr_slot);
