# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

pbl_clar_test(test_native_kernel
  SOURCES
    kernel/tick.c
    kernel/native/sched.c
    kernel/native/thread.c
    kernel/native/mutex.c
    kernel/native/sem.c
    kernel/native/msgq.c
    kernel/native/poll.c
    kernel/native/irq.c
    kernel/native/debug.c
    kernel/native/arch/posix/arch.c
  OVERRIDES kernel_native
  INCLUDES
    kernel/native
    kernel/native/arch
    kernel/native/arch/posix/include
  DEFINES
    CONFIG_KERNEL_BACKEND_NATIVE=1
    CONFIG_KERNEL_IDLE_STACK_SIZE=1024
)
