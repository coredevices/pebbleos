/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include <stddef.h>
#include <string.h>

#include <cmsis_core.h>

#include "pbl/drivers/mpu.h"
#include "pbl/kernel/idle.h"
#include "pbl/mcu/interrupts.h"

#include "kernel.h"

#if defined(__VFP_FP__) && !defined(__SOFTFP__)
#define HAS_FPU 1
#else
#define HAS_FPU 0
#endif

#ifdef CONFIG_MPU_TYPE_ARMV8M
#define HAS_PSPLIM 1
#else
#define HAS_PSPLIM 0
#endif

#define NUM_MPU_REGIONS 4
#ifdef CONFIG_SOC_SF32LB52
#define FIRST_MPU_REGION 8
#elif defined(CONFIG_QEMU) && defined(CONFIG_CORTEX_M33)
#define FIRST_MPU_REGION 4
#else
#define FIRST_MPU_REGION 4
#endif

#define SVC_START 0
#define SVC_YIELD 1
#define SVC_RAISE_PRIVILEGE 2

#define INITIAL_XPSR 0x01000000u
#define INITIAL_EXC_RETURN 0xfffffffdu
#define CONTROL_PRIVILEGED 0x02u
#define CONTROL_UNPRIVILEGED 0x03u
#define EXC_RETURN_FP_INACTIVE 0x10u
#define XPSR_STACK_PADDING 0x200u
#define FPCCR_ASPEN_LSPEN (0x3u << 30)

// System handler priority registers, byte-indexed like the NVIC ones.
#define SHPR_BYTES ((volatile uint8_t *)0xE000ED18u)
#define SHPR_SVCALL 7
#define SHPR_PENDSV 10
#define SHPR_SYSTICK 11

//! Saved context, lowest address first. Matches the layout core dump tooling
//! learned from the FreeRTOS port so the canonical register walk is unchanged.
struct saved_context {
  uint32_t control;
  uint32_t r4_r11[8];
#if HAS_PSPLIM
  uint32_t psplim;
#endif
  uint32_t exc_return;
  // s16-s31 follow when exc_return says the FP context is active, then the
  // hardware frame: r0-r3, r12, lr, pc, xpsr (and s0-s15, fpscr, reserved).
  uint32_t r0;
  uint32_t r1;
  uint32_t r2;
  uint32_t r3;
  uint32_t r12;
  uint32_t lr;
  uint32_t pc;
  uint32_t xpsr;
};

#define NUM_EXTRA_FP_REGS 16
#define NUM_BASIC_FP_REGS 18

_Static_assert(offsetof(struct pbl_thread, backend.sp) == 0, "saved SP must be first");
_Static_assert(offsetof(struct pbl_thread, backend.arch.mpu) == 4, "MPU words must follow the SP");
_Static_assert(NUM_MPU_REGIONS == PBL_THREAD_MAX_MEM_REGIONS, "region count mismatch");

static inline bool prv_fp_active(uint32_t exc_return) {
  return (exc_return & EXC_RETURN_FP_INACTIVE) == 0;
}

// ---- threads ----------------------------------------------------------------

static void prv_thread_return(void) { pbl_thread_abort(NULL); }

void arch_thread_init(struct pbl_thread *t, void (*entry)(void *), void *arg) {
  uintptr_t top = ((uintptr_t)t->stack + t->stack_size) & ~7u;
  struct saved_context *ctx = (struct saved_context *)(top - sizeof(struct saved_context));
  memset(ctx, 0, sizeof(*ctx));
  ctx->control = t->privileged ? CONTROL_PRIVILEGED : CONTROL_UNPRIVILEGED;
#if HAS_PSPLIM
  ctx->psplim = (uint32_t)t->stack;
#endif
  ctx->exc_return = INITIAL_EXC_RETURN;
  ctx->r0 = (uint32_t)arg;
  ctx->lr = (uint32_t)prv_thread_return;
  ctx->pc = (uint32_t)entry;
  ctx->xpsr = INITIAL_XPSR;
  t->backend.sp = ctx;
}

// The port expects the attribute word to hold RASR/RLAR verbatim and derives
// RBAR from the base: on ARMv7-M the region number and VALID bit are ORed in
// so no RNR write is needed per region.
void arch_thread_regions_set(struct pbl_thread *t, const MpuRegion *const *regions) {
  for (unsigned int i = 0; i < NUM_MPU_REGIONS; i++) {
    const MpuRegion *r = regions ? regions[i] : NULL;
    uint32_t rbar = 0;
    uint32_t attr = 0;
    if (r != NULL) {
      KERNEL_ASSERT(r->region_num == FIRST_MPU_REGION + i);
      uint32_t base_reg;
      mpu_get_register_settings(r, &base_reg, &attr);
#ifdef CONFIG_MPU_TYPE_ARMV8M
      rbar = base_reg;
#else
      rbar = (base_reg & ~(MPU_RBAR_VALID_Msk | MPU_RBAR_REGION_Msk)) | MPU_RBAR_VALID_Msk |
             (FIRST_MPU_REGION + i);
#endif
    }
    t->backend.arch.mpu[2 * i] = rbar;
    t->backend.arch.mpu[2 * i + 1] = attr;
  }
#if __DCACHE_PRESENT
  SCB_CleanDCache();
#endif
}

void arch_thread_aborted(struct pbl_thread *t) { (void)t; }

void arch_thread_exit(void) {
  // The switch requested by the abort takes over as soon as we get here.
  for (;;) {
    __WFI();
  }
}

// ---- interrupts -------------------------------------------------------------

bool arch_in_isr(void) { return mcu_state_is_isr(); }

void arch_irq_disable(void) {
  __set_BASEPRI(PBL_IRQ_PRIO_MAX_SYSCALL);
  __DSB();
  __ISB();
}

void arch_irq_enable(void) { __set_BASEPRI(0); }

void arch_switch_request(void) {
  SCB->ICSR = SCB_ICSR_PENDSVSET_Msk;
  __DSB();
  __ISB();
}

// ---- context switch ---------------------------------------------------------

#define STR_(x) #x
#define STR(x) STR_(x)

#if HAS_FPU
#define SAVE_FP "  tst r14, #0x10 \n it eq \n vstmdbeq r0!, {s16-s31} \n"
#define RESTORE_FP "  tst r14, #0x10 \n it eq \n vldmiaeq r0!, {s16-s31} \n"
#else
#define SAVE_FP ""
#define RESTORE_FP ""
#endif

#if HAS_PSPLIM
#define SAVE_REGS "  mrs r1, control \n mrs r12, psplim \n stmdb r0!, {r1, r4-r12, r14} \n"
#define RESTORE_REGS "  ldmia r0!, {r3, r4-r12, r14} \n msr control, r3 \n msr psplim, r12 \n"
#else
#define SAVE_REGS "  mrs r1, control \n stmdb r0!, {r1, r4-r11, r14} \n"
#define RESTORE_REGS "  ldmia r0!, {r3, r4-r11, r14} \n msr control, r3 \n"
#endif

// r1 -> thread's MPU words; clobbers r2, r4-r11.
#define RESTORE_MPU                                                          \
  "  ldr r2, =0xe000ed98 \n"            /* MPU_RNR */                      \
  "  mov r4, #" STR(FIRST_MPU_REGION) " \n"                                  \
  "  str r4, [r2] \n"                                                        \
  "  ldr r2, =0xe000ed9c \n"            /* MPU_RBAR and its aliases */      \
  "  ldmia r1!, {r4-r11} \n"                                                 \
  "  stmia r2!, {r4-r11} \n"

__attribute__((naked)) void PendSV_Handler(void) {
  __asm volatile(
      "  mrs r0, psp \n"
      "  isb \n"
      "  ldr r3, =pbl_cur \n"
      "  ldr r2, [r3] \n"
      SAVE_FP
      SAVE_REGS
      "  str r0, [r2] \n"              /* pbl_cur->backend.sp */
      "  stmdb sp!, {r3, r14} \n"
      "  mov r0, %0 \n"
      "  msr basepri, r0 \n"
      "  dsb \n"
      "  isb \n"
      "  bl sched_switch_in \n"        /* returns the next thread */
      "  mov r4, r0 \n"
      "  mov r0, #0 \n"
      "  msr basepri, r0 \n"
      "  ldmia sp!, {r3, r14} \n"
      "  ldr r0, [r4] \n"              /* next->backend.sp */
      "  add r1, r4, #4 \n"            /* next->backend.arch.mpu */
      RESTORE_MPU
      RESTORE_REGS
      RESTORE_FP
      "  msr psp, r0 \n"
      "  isb \n"
      "  bx r14 \n"
      "  .ltorg \n"
      ::"i"(PBL_IRQ_PRIO_MAX_SYSCALL));
}

// Loads the first thread's context. MSP is reset to the top of the ISR stack.
__attribute__((naked)) static void prv_restore_first_thread(void) {
  __asm volatile(
      "  ldr r0, =0xE000ED08 \n"       /* VTOR: initial MSP is the first vector */
      "  ldr r0, [r0] \n"
      "  ldr r0, [r0] \n"
      "  msr msp, r0 \n"
      "  ldr r3, =pbl_cur \n"
      "  ldr r4, [r3] \n"
      "  ldr r0, [r4] \n"
      "  add r1, r4, #4 \n"
      RESTORE_MPU
      RESTORE_REGS
      "  msr psp, r0 \n"
      "  mov r0, #0 \n"
      "  msr basepri, r0 \n"
      "  isb \n"
      "  bx r14 \n"
      "  .ltorg \n");
}

// ---- supervisor calls -------------------------------------------------------

static uintptr_t prv_original_sp(const uint32_t *frame, uint32_t exc_return) {
  uintptr_t sp = (uintptr_t)frame + 8 * sizeof(uint32_t);
  if (prv_fp_active(exc_return)) {
    sp += NUM_BASIC_FP_REGS * sizeof(uint32_t);
  }
  if (frame[7] & XPSR_STACK_PADDING) {
    sp += 4;
  }
  return sp;
}

//! Stack-argument words mirrored onto the syscall stack: syscalls with more
//! than four arguments read the extras from the caller's stack.
#define SYSCALL_STACK_ARG_WORDS 16u

// Boards without a dedicated syscall stack run syscalls on the caller's.
__attribute__((weak)) uint32_t *pbl_kernel_syscall_stack(uintptr_t *base_out) {
  (void)base_out;
  return NULL;
}

// Moves the exception frame onto the thread's syscall stack, if it has one,
// so the syscall body runs there. Returns the stacked-LR slot.
static uint32_t *prv_relocate_to_syscall_stack(uint32_t *frame, uint32_t exc_return,
                                               uintptr_t orig_sp) {
  uintptr_t base = 0;
  uint32_t *top = pbl_kernel_syscall_stack(&base);
  if (top == NULL) {
    return &frame[5];
  }

  uint32_t *args = top - SYSCALL_STACK_ARG_WORDS;
  memcpy(args, (const void *)orig_sp, SYSCALL_STACK_ARG_WORDS * sizeof(uint32_t));

  uint32_t frame_words = prv_fp_active(exc_return) ? 8u + NUM_BASIC_FP_REGS : 8u;
  uint32_t *dst = args - frame_words;
  memcpy(dst, frame, frame_words * sizeof(uint32_t));
  dst[7] &= ~XPSR_STACK_PADDING;

#if HAS_PSPLIM
  __set_PSPLIM(base);
#else
  (void)base;
#endif
  __set_PSP((uint32_t)dst);
  __ISB();
  return &dst[5];
}

static void prv_raise_privilege(uint32_t *frame, uint32_t exc_return) {
  uint32_t caller_pc = frame[6];
  if (!pbl_kernel_privilege_raise_allowed(caller_pc)) {
    return;
  }
  uintptr_t orig_sp = prv_original_sp(frame, exc_return);
  uint32_t *lr_slot = prv_relocate_to_syscall_stack(frame, exc_return, orig_sp);
  pbl_kernel_syscall_entered(orig_sp, (uintptr_t *)lr_slot);
  __set_CONTROL(__get_CONTROL() & ~CONTROL_nPRIV_Msk);
  __ISB();
}

__attribute__((noinline, used)) static void prv_svc(uint32_t *frame, uint32_t exc_return) {
  uint8_t number = ((uint8_t *)frame[6])[-2];
  switch (number) {
    case SVC_START:
      // SVC at the same priority as the other kernel exceptions from here on.
      SHPR_BYTES[SHPR_SVCALL] = (uint8_t)PBL_IRQ_PRIO_KERNEL;
      prv_restore_first_thread();
      break;
    case SVC_YIELD:
      arch_switch_request();
      break;
    case SVC_RAISE_PRIVILEGE:
      prv_raise_privilege(frame, exc_return);
      break;
    default:
      break;
  }
}

__attribute__((naked)) void SVC_Handler(void) {
  __asm volatile(
      "  tst lr, #4 \n"
      "  ite eq \n"
      "  mrseq r0, msp \n"
      "  mrsne r0, psp \n"
      "  mov r1, lr \n"
      "  b prv_svc \n");
}

// ---- start and tick ---------------------------------------------------------

void arch_init(void) {}

void arch_start(void) {
  // PendSV and SysTick at the lowest priority; SVC follows once started.
  SHPR_BYTES[SHPR_PENDSV] = (uint8_t)PBL_IRQ_PRIO_KERNEL;
  SHPR_BYTES[SHPR_SYSTICK] = (uint8_t)PBL_IRQ_PRIO_KERNEL;

#if HAS_FPU
  SCB->CPACR |= (0xfu << 20);
  FPU->FPCCR |= FPCCR_ASPEN_LSPEN;
#endif

  if (!pbl_soc_tick_enable()) {
    SysTick->LOAD = (SystemCoreClock / PBL_TICK_HZ) - 1u;
    SysTick->VAL = 0;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk;
  }

  irq_reset();
  __asm volatile("  dsb \n svc %0 \n isb \n" ::"i"(SVC_START));
  for (;;) {
  }
}

#ifndef CONFIG_SOC_SF32LB52
void SysTick_Handler(void) { pbl_kernel_tick_isr(); }
#endif

void arch_idle(pbl_tick_t max_ticks) {
  // Short gaps are not worth the sleep entry; the idle loop just comes back.
  if (max_ticks >= 2) {
    pbl_soc_idle(max_ticks);
  }
}

// ---- introspection ----------------------------------------------------------

void arch_thread_saved_regs(const struct pbl_thread *t, struct pbl_thread_saved_regs *regs) {
  if (t == pbl_cur && arch_in_isr()) {
    // The running thread's registers are live on its stack, not saved.
    const uint32_t *frame = (const uint32_t *)__get_PSP();
    *regs = (struct pbl_thread_saved_regs){
      .pc = frame[6], .lr = frame[5], .control = __get_CONTROL() };
    return;
  }
  const struct saved_context *ctx = t->backend.sp;
  const uint32_t *hw = &ctx->r0;
  if (prv_fp_active(ctx->exc_return)) {
    hw += NUM_EXTRA_FP_REGS;
  }
  *regs = (struct pbl_thread_saved_regs){ .pc = hw[6], .lr = hw[5], .control = ctx->control };
}

void arch_thread_info_regs(const struct pbl_thread *t, uint32_t regs[PBL_THREAD_REG_COUNT]) {
  const struct saved_context *ctx = t->backend.sp;
  for (int i = 0; i < 8; i++) {
    regs[PBL_THREAD_REG_R4 + i] = ctx->r4_r11[i];
  }
  const uint32_t *hw = &ctx->r0;
  bool fp = prv_fp_active(ctx->exc_return);
  if (fp) {
    hw += NUM_EXTRA_FP_REGS;
  }
  regs[PBL_THREAD_REG_R0] = hw[0];
  regs[PBL_THREAD_REG_R1] = hw[1];
  regs[PBL_THREAD_REG_R2] = hw[2];
  regs[PBL_THREAD_REG_R3] = hw[3];
  regs[PBL_THREAD_REG_R12] = hw[4];
  regs[PBL_THREAD_REG_LR] = hw[5];
  regs[PBL_THREAD_REG_PC] = hw[6];
  regs[PBL_THREAD_REG_XPSR] = hw[7];
  uintptr_t sp = (uintptr_t)&hw[8];
  if (fp) {
    sp += NUM_BASIC_FP_REGS * sizeof(uint32_t);
  }
  if (hw[7] & XPSR_STACK_PADDING) {
    sp += 4;
  }
  regs[PBL_THREAD_REG_SP] = sp;
}
