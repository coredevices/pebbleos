/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "syscall_internal.h"

#include "applib/app_logging.h"
#include "kernel/pebble_tasks.h"
#include "pbl/mcu/privilege.h"
#include "process_management/app_manager.h"
#include "process_management/pebble_process_md.h"
#include "process_management/process_loader.h"
#include "process_management/process_manager.h"
#include "syscall/syscall.h"
#include <pbl/logging/logging.h>
#include "system/passert.h"
#include "pbl/util/size.h"

#include "FreeRTOS.h"
#include "task.h"

#include <stdint.h>
#include <stddef.h>

// Run App/Worker syscalls on a dedicated privileged stack instead of the
// caller's small unprivileged one, so a task that exhausts its stack faults
// unprivileged (only that process dies) instead of rebooting the system.
// Enabled on ARMv8-M (needs PSPLIM); ARMv7-M keeps the old behaviour.
#if !defined(SYSCALL_PRIVILEGED_STACK)
#  if defined(CONFIG_MPU_TYPE_ARMV8M)
#    define SYSCALL_PRIVILEGED_STACK 1
#  else
#    define SYSCALL_PRIVILEGED_STACK 0
#  endif
#endif

// Indices into FreeRTOS thread local storage
#define TLS_SYSCALL_LR_IDX 0
#define TLS_SYSCALL_SP_IDX 1

// Helper functions to access FreeRTOS TLS
static uintptr_t prv_get_syscall_sp(void) {
  return (uintptr_t)pvTaskGetThreadLocalStoragePointer(NULL, TLS_SYSCALL_SP_IDX);
}

static void prv_set_syscall_sp(uintptr_t new_sp) {
  vTaskSetThreadLocalStoragePointer(NULL, TLS_SYSCALL_SP_IDX, (void *)new_sp);
}

USED uintptr_t get_syscall_lr(void) {
  return (uintptr_t)pvTaskGetThreadLocalStoragePointer(NULL, TLS_SYSCALL_LR_IDX);
}

static void prv_set_syscall_lr(uintptr_t new_lr) {
  vTaskSetThreadLocalStoragePointer(NULL, TLS_SYSCALL_LR_IDX, (void *)new_lr);
}

typedef struct McuUnprivilegedCallContext {
  // Continuation state for mcu_call_unprivileged(). The callback runs as
  // native app code, so keep the privileged return path out of its stack.
  uint32_t saved_r4_r11[8];
  uintptr_t caller_lr;
  uintptr_t entry_sp;
  uintptr_t outer_syscall_lr;
  uintptr_t outer_syscall_sp;
  bool active;
  TaskHandle_t handle;
} McuUnprivilegedCallContext;

_Static_assert(sizeof(((McuUnprivilegedCallContext *)0)->saved_r4_r11) == 32,
               "mcu_call_unprivileged asm assumes push {r4-r11} saves 32 bytes");
_Static_assert(sizeof(uintptr_t) == sizeof(uint32_t),
               "mcu_call_unprivileged asm assumes 32-bit pointers");

static McuUnprivilegedCallContext s_unprivileged_call_ctx[NumPebbleTask];

static McuUnprivilegedCallContext *prv_unprivileged_call_ctx_for_current_task(void) {
  const PebbleTask task = pebble_task_get_current();
  if (task >= NumPebbleTask) {
    return NULL;
  }
  return &s_unprivileged_call_ctx[task];
}

static USED void mcu_call_unprivileged_enter(void (*fn)(void *), void *ctx,
                                             uintptr_t caller_lr, uintptr_t entry_sp,
                                             const uint32_t *saved_regs) {
  (void)ctx;

  PBL_ASSERTN(mcu_state_is_thread_privileged());
  PBL_ASSERTN(fn != NULL);

  McuUnprivilegedCallContext *state = prv_unprivileged_call_ctx_for_current_task();
  PBL_ASSERTN(state != NULL);

  const TaskHandle_t handle = xTaskGetCurrentTaskHandle();
  if (state->active && state->handle != handle) {
    // Slots are indexed by PebbleTask. If a task died mid-callback and a new
    // FreeRTOS task reused the PebbleTask, discard the old call state first.
    *state = (McuUnprivilegedCallContext) { 0 };
  }

  PBL_ASSERTN(!state->active);

  *state = (McuUnprivilegedCallContext) {
    .active = true,
    .handle = handle,
    .caller_lr = caller_lr,
    .entry_sp = entry_sp,
    .outer_syscall_lr = get_syscall_lr(),
    .outer_syscall_sp = prv_get_syscall_sp(),
  };

  for (size_t i = 0; i < ARRAY_LENGTH(state->saved_r4_r11); ++i) {
    state->saved_r4_r11[i] = saved_regs[i];
  }
}

EXTERNALLY_VISIBLE void mcu_call_unprivileged_resume(void);

static uintptr_t prv_mcu_call_unprivileged_reentry_return_pc(void) {
  extern const uint16_t __mcu_call_unprivileged_svc[];

  // A 16-bit SVC returns to the next halfword.
  return (uintptr_t)__mcu_call_unprivileged_svc + sizeof(uint16_t);
}

static McuUnprivilegedCallContext *prv_active_unprivileged_call_ctx_for_current_task(void) {
  McuUnprivilegedCallContext *state = prv_unprivileged_call_ctx_for_current_task();
  if (state == NULL || !state->active ||
      state->handle != xTaskGetCurrentTaskHandle()) {
    return NULL;
  }
  return state;
}

bool mcu_call_unprivileged_reentry_is_allowed(uint32_t caller_pc) {
  if (caller_pc != prv_mcu_call_unprivileged_reentry_return_pc()) {
    return false;
  }

  return prv_active_unprivileged_call_ctx_for_current_task() != NULL;
}

bool mcu_call_unprivileged_reentry_setup(uintptr_t orig_sp, uintptr_t *lr_ptr) {
  (void)orig_sp;

  // lr_ptr is the saved LR field in the Cortex-M basic exception frame:
  //   r0 r1 r2 r3 r12 lr pc xpsr
  // Check the stacked return PC here too, so setup consumes state only for the
  // same re-entry SVC that the SVC gate accepted.
  uintptr_t *stacked_r0 = lr_ptr - 5;
  uintptr_t *stacked_pc = lr_ptr + 1;
  if ((uint32_t)*stacked_pc != prv_mcu_call_unprivileged_reentry_return_pc()) {
    return false;
  }

  McuUnprivilegedCallContext *state = prv_active_unprivileged_call_ctx_for_current_task();
  if (state == NULL) {
    return false;
  }

  state->active = false;
  state->handle = NULL;

  // The inner SVC uses the same TLS slots as the app syscall that entered XS.
  // Put them back before the runtime continues.
  prv_set_syscall_lr(state->outer_syscall_lr);
  prv_set_syscall_sp(state->outer_syscall_sp);

  // Return from the exception into a fixed firmware continuation, not back
  // into the callback. The continuation restores SP before using the stack.
  *stacked_r0 = (uintptr_t)state;
  *stacked_pc = (uintptr_t)&mcu_call_unprivileged_resume;
  *lr_ptr = 0;

  return true;
}

//! Run @p fn unprivileged for the duration of one call, then return to the
//! caller still in privileged thread mode.
//!
//! Used when privileged firmware invokes an app-supplied callback that should
//! run with app privileges. The callback runs as app code, so MPU violations
//! stay in the app/worker task.
//!
//! The re-entry SVC is deliberately outside .syscall_text. It is only accepted
//! while this call is active for the current task, and the SVC handler resumes
//! through saved kernel state instead of the callback's stack.
EXTERNALLY_VISIBLE NAKED_FUNC USED
void mcu_call_unprivileged(void (*fn)(void *), void *ctx) {
  __asm volatile (
    // Keep fn/ctx across the setup call. Copy r4-r11 too; native app code is
    // not trusted to preserve the privileged caller's callee-saved registers.
    "  push {r4-r11}                    \n"
    "  push {r0, r1}                    \n"
    "  mov r2, lr                       \n" // r2 = privileged caller LR
    // These offsets are tied to the two pushes above:
    //   push {r4-r11} = 32 bytes, push {r0,r1} = 8 bytes.
    "  add r3, sp, #40                  \n" // r3 = entry SP before pushes
    "  add r12, sp, #8                  \n" // r12 = saved r4-r11 pointer
    "  sub sp, sp, #8                   \n" // keep stack 8-byte aligned for C call
    "  str r12, [sp]                    \n" // 5th arg: saved_regs pointer
    "  bl mcu_call_unprivileged_enter   \n"
    "  add sp, sp, #8                   \n" // discard 5th arg + alignment pad
    "  pop {r2, r3}                     \n" // r2 = fn, r3 = ctx
    "  add sp, sp, #32                  \n" // saved r4-r11 copied to kernel state

    // Drop privilege inline (CONTROL.nPRIV = 1). Calling the C helper would
    // clobber the app callback registers we just restored.
    "  mrs r0, control                  \n"
    "  orr r0, r0, #1                   \n"
    "  msr control, r0                  \n"
    "  isb                              \n"

    // Invoke fn(ctx) unprivileged. MPU-violating accesses fault here.
    "  mov r0, r3                       \n"
    "  blx r2                           \n"

    // Re-enter privileged mode. The handler accepts this exact callsite only
    // while this mcu_call_unprivileged() call is active for the current task.
    "  .global __mcu_call_unprivileged_svc \n"
    "__mcu_call_unprivileged_svc:       \n"
    "  svc 2                            \n"
    // Deliberate fail-closed trap. If the SVC was denied or not rewritten, do
    // not keep executing in the callback's control flow.
    "  udf #0                           \n"
  );
}

EXTERNALLY_VISIBLE NAKED_FUNC USED
void mcu_call_unprivileged_resume(void) {
  __asm volatile (
    // r0 is the state pointer stamped into the frame by Handler mode. Restore
    // SP before any stack use.
    "  ldr r12, [r0, #%c[caller_lr_off]] \n"
    "  ldr r1, [r0, #%c[entry_sp_off]]  \n"
    "  mov sp, r1                       \n"
    "  adds r0, r0, #%c[regs_off]       \n"
    "  ldmia r0, {r4-r11}               \n"
    "  bx r12                           \n"
    :
    : [caller_lr_off] "i" (offsetof(McuUnprivilegedCallContext, caller_lr)),
      [entry_sp_off] "i" (offsetof(McuUnprivilegedCallContext, entry_sp)),
      [regs_off] "i" (offsetof(McuUnprivilegedCallContext, saved_r4_r11))
  );
}

NORETURN syscall_failed(void) {
  register uint32_t lr __asm("lr");
  uint32_t saved_lr = lr;

  PBL_ASSERT(mcu_state_is_privileged(), "Insufficient Privileges!");

  PBL_LOG_WRN("Bad syscall!");

  sys_app_fault(saved_lr);

  // sys_die is no return, but it's a syscall so I don't want to mark it with that attribute
  while(1) { }
}

void syscall_assert_userspace_buffer(const void* buf, size_t num_bytes) {
  PebbleTask task = pebble_task_get_current();

  void *user_stack_end = (void *)prv_get_syscall_sp();

  if (process_manager_is_address_in_region(task, buf, user_stack_end)
      && process_manager_is_address_in_region(
          task, (uint8_t *)buf + num_bytes -1, user_stack_end)) {
    return;
  }

  APP_LOG(APP_LOG_LEVEL_ERROR, "syscall failure! %p..%p is not in app space.", buf, (char *)buf + num_bytes);
  PBL_LOG_ERR("syscall failure! %p..%p is not in app space.", buf, (char *)buf + num_bytes);
  syscall_failed();
}

#if SYSCALL_PRIVILEGED_STACK
// Dedicated privileged stacks for App/Worker syscalls. Plain .bss statics land
// in the privileged-only .kernel_bss output (KERNEL_RAM): unreadable by app
// code, zeroed at boot. (Not section(".kernel_bss") -- that would orphan them.)
#define SYSCALL_STACK_WORDS 512u  // 2 KiB each; size against measured high-water.
static StackType_t s_app_syscall_stack[SYSCALL_STACK_WORDS] __attribute__((aligned(8)));
static StackType_t s_worker_syscall_stack[SYSCALL_STACK_WORDS] __attribute__((aligned(8)));

// Port hook: top of the current task's dedicated syscall stack (base in
// *base_out), or NULL to keep it on the caller's stack. App + Worker only;
// moddable apps use mcu_call_unprivileged() and stay on their own stack.
uint32_t *xApplicationGetSyscallStack(uintptr_t *base_out) {
  StackType_t *stack;
  switch (pebble_task_get_current()) {
    case PebbleTask_App:
#ifdef CONFIG_MODDABLE_XS
      {
        const PebbleProcessMd *md = app_manager_get_current_app_md();
        if (md != NULL && md->is_moddable_app) {
          return NULL;
        }
      }
#endif
      stack = s_app_syscall_stack;
      break;
    case PebbleTask_Worker:
      stack = s_worker_syscall_stack;
      break;
    default:
      return NULL;
  }
  *base_out = (uintptr_t)&stack[0];
  return (uint32_t *)&stack[SYSCALL_STACK_WORDS];
}

static bool prv_psp_in_syscall_stack(uintptr_t psp, const StackType_t *stack) {
  return (psp > (uintptr_t)&stack[0]) && (psp <= (uintptr_t)&stack[SYSCALL_STACK_WORDS]);
}

// Task SP/PSPLIM to restore if the finishing syscall ran on a dedicated stack,
// packed as (psplim << 32 | sp) to return in r0:r1; 0 = no switch needed.
USED uint64_t syscall_stack_restore_target(void) {
  const uintptr_t psp = __get_PSP();
  if (prv_psp_in_syscall_stack(psp, s_app_syscall_stack) ||
      prv_psp_in_syscall_stack(psp, s_worker_syscall_stack)) {
    const uint32_t sp = (uint32_t)prv_get_syscall_sp();  // slot1 = pre-syscall task SP
    const uint32_t psplim = (uint32_t)ulTaskGetStackStart(xTaskGetCurrentTaskHandle());
    return ((uint64_t)psplim << 32) | sp;
  }
  return 0;
}

// Peak unused bytes on a syscall stack: scan the zeroed .bss from the base to
// the first written word (the high-water). For sizing during validation.
static uint16_t prv_syscall_stack_free_bytes(const StackType_t *stack) {
  uint32_t i = 0;
  while (i < SYSCALL_STACK_WORDS && stack[i] == 0) {
    i++;
  }
  return (uint16_t)(i * sizeof(StackType_t));
}

uint16_t syscall_app_stack_free_bytes(void) {
  return prv_syscall_stack_free_bytes(s_app_syscall_stack);
}

uint16_t syscall_worker_stack_free_bytes(void) {
  return prv_syscall_stack_free_bytes(s_worker_syscall_stack);
}

// Drop privilege and return to the task. If the syscall ran on a dedicated
// stack, restore PSP/PSPLIM to the task stack first (while still privileged).
EXTERNALLY_VISIBLE void NAKED_FUNC USED prv_drop_privilege(void) {
  __asm volatile (
    " push {r0, r1} \n"                       // save syscall return value
    " bl process_manager_handle_syscall_exit \n"
    " bl get_syscall_lr \n"                    // r0 = real return address
    " push {r0, r1} \n"                        // save real LR (r1 = pad; keeps 8-byte align)
    " bl syscall_stack_restore_target \n"      // r0 = restore SP (0 = none), r1 = restore PSPLIM
    " mov r2, r0 \n"                           // r2 = restore SP
    " mov r3, r1 \n"                           // r3 = restore PSPLIM
    " pop {r0, r1} \n"                         // r0 = real LR
    " mov r12, r0 \n"                          // r12 = real LR (caller-saved; no bl follows)
    " pop {r0, r1} \n"                         // r0,r1 = syscall return value
    " cbz r2, 1f \n"                           // skip stack switch if not relocated
    " msr psp, r2 \n"                          // back to the app stack (higher addr; safe vs low limit)
    " msr psplim, r3 \n"                       // restore the app stack limit
    " isb \n"
    "1: \n"
    " mrs r2, control \n"                      // drop privilege: CONTROL.nPRIV = 1
    " orr r2, r2, #1 \n"
    " msr control, r2 \n"
    " isb \n"
    " bx r12 \n"                               // return into the app
  );
}
#else
uint16_t syscall_app_stack_free_bytes(void) { return 0xFFFF; }
uint16_t syscall_worker_stack_free_bytes(void) { return 0xFFFF; }

// Drop privileges and return to the address stored in thread local storage
// Has to preserve r0 and r1 so the syscall's return value is passed through
EXTERNALLY_VISIBLE void NAKED_FUNC USED prv_drop_privilege(void) {
  __asm volatile (
    " push {r0, r1} \n"
    " bl process_manager_handle_syscall_exit \n"
    " bl get_syscall_lr \n"
    " push { r0 } \n" // push the correct lr onto the stack

    " mov r0, #0 \n" // mcu_state_set_thread_privilege(false)
    " bl mcu_state_set_thread_privilege \n"

    " pop {lr} \n" // Pop correct return address
    " pop {r0, r1} \n" // Restore the return values of the syscall

    " bx lr \n" // Leave the syscall
  );
}
#endif // SYSCALL_PRIVILEGED_STACK

// Just jump straight into the drop privilege code
EXTERNALLY_VISIBLE void NAKED_FUNC USED prv_drop_privilege_wrapper(void) {
  __asm volatile("b prv_drop_privilege\n");
}

// This function needs to preserve the argument registers and stack exactly as they were on
// entry, so the arguments are passed correctly into the syscall. The key purpose of this
// function is to determine whether or not the caller is privileged. If the caller is
// unprivileged, this function returns normally to the syscall wrapper, and svc 2 is
// called elevating privileges. If the caller was already privileged, this function
// returns past the svc 2 instruction so privileges are not elevated.
void NAKED_FUNC USED syscall_internal_maybe_skip_privilege(void) {
  __asm volatile (
    // Save argument registers
    " push {r0-r3, lr} \n"
    " bl mcu_state_is_privileged \n"
    " cmp r0, #1 \n" // Were we privileged?

    " pop {r0-r3, lr} \n" // Restore state

    " it eq \n" // If we were privileged, return past the svc function
    " addeq lr, #2 \n" // svc 2 is 2 bytes long

    // Store our return address in ip, which isn't caller or callee saved
    // since the linker can modify it
    " mov ip, lr \n"

    // Set lr to the wrapper's return address. This saves code space so the
    // wrapper doesn't have to do this itself. Also we need to check this value
    // here.
    " pop {lr} \n"

    " push {ip} \n" // Save the wrapper address on the stack

    // The following can occur with nested syscalls, when the 2nd syscall is at
    // the end of the first. Since PRIVILEGE_WAS_ELEVATED depends on the return
    // address of the function being equal to syscall_internal_drop_privilege,
    // changing to the wrapper prevents a false positive in the nested syscall

    // if lr == syscall_internal_drop_privilege,
    // lr = syscall_internal_drop_privilege_wrapper
    " ldr ip, =prv_drop_privilege \n"
    " cmp lr, ip \n"
    " it eq \n"
    " ldreq lr, =prv_drop_privilege_wrapper \n"

    " pop {pc} \n" // Return to the wrapper
  );
}

// This is more space efficient than inlining the equality
// expression into every syscall since the address literal
// only needs to be stored at the end of this one function
bool syscall_internal_check_return_address(void * ret_addr) {
  return ret_addr == &prv_drop_privilege;
}

// This function is called by the SVC handler with the pre-syscall
// stack pointer, and a pointer to the saved LR on the stack.
// It then stores the SP and LR in thread local storage,
// and updates the saved LR to point at the drop privilege code.
void vSetupSyscallRegisters(uintptr_t orig_sp, uintptr_t *lr_ptr) {
  if (mcu_call_unprivileged_reentry_setup(orig_sp, lr_ptr)) {
    return;
  }

  // These calls should be safe because the scheduler needs to call the svc handler
  // before the current task is changed. Since this function is called from the svc
  // handler, modifying the current task will always finish before a context switch

  // Save the correct return address so the drop privilege code knows where to
  // return to.
  prv_set_syscall_lr(*lr_ptr);

  // Save the value of the SP before entry into the syscall so
  // syscall_assert_userspace_buffer can ensure that a user provided buffer doesn't
  // point into the syscall's stack frame, and that the syscall has enough space
  prv_set_syscall_sp(orig_sp);

  // Set the return address of the syscall to be the drop privilege code
  *lr_ptr = (uintptr_t)&prv_drop_privilege;
}
