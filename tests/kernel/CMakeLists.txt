# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

pbl_clar_test(test_kernel
  SOURCES
    kernel/tick.c
    kernel/sched.c
    kernel/thread.c
    kernel/mutex.c
    kernel/sem.c
    kernel/msgq.c
    kernel/poll.c
    kernel/irq.c
    kernel/debug.c
    kernel/arch/posix/arch.c
  INCLUDES
    kernel
    kernel/arch
    kernel/arch/posix/include
  DEFINES
    CONFIG_KERNEL_IDLE_STACK_SIZE=1024
)
