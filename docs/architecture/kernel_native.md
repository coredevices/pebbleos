# Native kernel backend

`kernel/native` implements `include/pbl/kernel` without FreeRTOS and is the
default backend; `CONFIG_KERNEL_BACKEND_FREERTOS=y` selects the FreeRTOS shim
in `kernel/freertos` instead.

## Layout

```
sched.c        ready lists, timeouts, tick, block/wake, idle thread, start
thread.c       thread lifecycle and priorities
mutex.c        recursive mutex with priority inheritance
sem.c          counting semaphore
msgq.c         message queue
poll.c         poll group
irq.c          nestable interrupt lock
debug.c        thread walk, saved registers, stack and run-time stats
kernel.h       the interface between the objects, the scheduler and the arch
arch/arch.h    what an arch must provide
arch/arm/      Cortex-M port: context switch, SVC, MPU, SysTick, idle
arch/posix/    host port: threads are pthreads, time is virtual; for tests
kernel_test.h  harness entry points used by tests/kernel
```

## Design

**Scheduling.** Fixed priorities, `CONFIG_KERNEL_NUM_PRIORITIES` of them,
one FIFO ready list per priority and a bitmap picked with `clz`, so
choosing the next thread is O(1). Threads of equal priority round-robin on
every tick (as FreeRTOS does with time slicing) and on `pbl_thread_yield()`.
The running thread stays on its ready list; blocking removes it.

**Blocking.** Every object owns a `struct pbl_waitq`: a singly linked list
ordered by priority, FIFO within a priority. `sched_block()` moves the
current thread onto the object's wait queue and the timeout list, requests a
switch and drops the interrupt lock; it returns with the lock re-taken and
the wake code. Semaphores and mutexes hand their token straight to the woken
thread, so a wake is never spurious and a lower-priority giver cannot steal
it back. Queues and poll groups wake one waiter and let it re-check.

**Timeouts.** One list sorted by absolute wake tick, wrap-safe compare. The
tick expires the head, sleeps use the same list, and the idle thread asks it
how long it may sleep. Insertion is O(n) in the number of pending timeouts,
which on this firmware is a handful.

**Priority inheritance.** Single level, the same as the FreeRTOS fork's
light mutex: a waiter raises the owner to its own priority; the owner drops
back to its base priority when it holds no mutex any more. A boosted thread
is re-sorted in its ready list or wait queue.

**Interrupts.** `pbl_irq_lock()` raises BASEPRI to
`CONFIG_KERNEL_IRQ_PRIO_MAX_SYSCALL` with a nesting counter, exactly the
FreeRTOS critical section. ISRs above that priority may not call the kernel.
Object operations run under that lock; there is no separate scheduler lock
inside the kernel. `pbl_sched_lock()` defers switches until unlocked.

**Idle.** The idle thread yields to other threads at its priority, then
calls the arch idle hook with the ticks until the next timeout. On Cortex-M
that forwards to the SoC's `pbl_soc_idle()` when the gap is at least two
ticks, and the SoC reports the sleep back with `pbl_idle_slept()`, which
advances the clock and expires timeouts.

**Threads.** `struct pbl_thread` is caller-owned; the backend state lives in
its `backend` member, first in the struct so the arch code can find the
saved stack pointer at offset zero. Stacks are filled with a pattern at
creation for the high-water mark. Returning from the entry function ends
the thread. A thread suspended while blocked sees the blocking call fail
with `-EINTR` when resumed; the process manager only suspends threads it is
about to kill or resumes immediately.

**No allocation.** Nothing in the backend calls an allocator; `PBL_*_DEFINE`
initialisers are complete and `pbl_kernel_init()` returns at once.

## Cortex-M arch

The saved context keeps the layout of the FreeRTOS port: CONTROL, r4-r11,
PSPLIM on ARMv8-M, EXC_RETURN, then the hardware frame, with s16-s31 in
between when the FP context is active. Core dumps and the fault handler
read that layout unchanged. PendSV saves the context, picks the next thread
under BASEPRI, rewrites the four per-thread MPU regions through the RBAR
alias registers and restores. Supervisor calls keep the port's numbers: 0
starts the scheduler, 1 yields, 2 raises privilege. The privilege-raise path
copies the exception frame onto the thread's dedicated syscall stack when
`pbl_kernel_syscall_stack()` provides one and hands the caller's stack
pointer and stacked return slot to `pbl_kernel_syscall_entered()`, which is
what the firmware's syscall island expects.

## POSIX arch and tests

Each kernel thread is a pthread; exactly one runs at a time, and control
passes when the kernel would switch. Interrupts are simulated: a test wraps
calls in `pbl_test_isr_enter()`/`exit()` and delivers ticks with
`pbl_test_tick()`. When every thread is blocked, the idle hook jumps the
clock to the next timeout. This makes the scheduling decisions exactly
reproducible, so `tests/kernel/test_native_kernel.c` can assert trace
strings such as "lhLHMd" for the priority-inheritance scenario.

## Compared with other kernels

| | FreeRTOS (fork) | Zephyr | NuttX | ThreadX | Native |
| --- | --- | --- | --- | --- | --- |
| Ready queue | list per priority, `clz` optional | dumb list / scalable / multiq | list per priority | bitmap + list | bitmap + list |
| Time slicing | per tick | configurable | configurable | per thread | per tick |
| Timeouts | two delayed lists | single delta list | wdog list | per-timer wheel | single absolute list |
| Wait queues | priority ordered | priority ordered | priority ordered | FIFO or priority | priority ordered |
| Mutex PI | single level | single level | multi level | single level | single level |
| Sem/mutex wake | wake, retry | handoff | handoff | handoff | handoff |
| Static objects | since v9 | yes | no | yes | yes |
| Idle | idle task hook | `k_cpu_idle` + tickless | tickless | tickless | idle thread + SoC hook |
| Stack overflow | pattern / MPU guard | MPU guard / canary | canary | analysis | PSPLIM (v8-M), MPU guard, pattern |
| Thread exit | must not return | returns | returns | returns | returns |

What was taken from where:

- Direct handoff on semaphores and mutexes (Zephyr, ThreadX). Removes the
  give-then-retry race and the spurious wakeups the fork's queue-based
  semaphores have.
- Priority-ordered wait queues with FIFO within a priority (everyone).
- Completed static initialisers, so nothing has to run before objects are
  usable (Zephyr's `K_*_DEFINE`).
- A poll group that scans members from a rotating cursor (Zephyr's `k_poll`
  fairness), and no rule about draining members only through the group,
  which the FreeRTOS queue set imposes and which this firmware's event loop
  tiptoes around.
- Threads may return from their entry function (everyone but FreeRTOS).
- Saved registers of the running thread are read from its live exception
  frame when asked from an ISR, so the task watchdog reports the real stuck
  PC instead of a stale context (a FreeRTOS limitation here).
- Thread struct reuse is checked: creating a thread in a struct whose
  previous thread is still alive asserts.

Not done, on purpose or for later:

- Multi-level priority inheritance (NuttX). The firmware never nests locks
  deeply enough for it to matter, and the fork did not have it either.
- Timer wheels (ThreadX) or delta lists (Zephyr) for timeouts. The pending
  timeout count here is small; an O(n) insert keeps the code obvious.
- Run-time statistics in cycles from the DWT counter instead of ticks. QEMU
  has no DWT, so it is left for a board with one.
- Per-thread preemption threshold (ThreadX) and memory domains (Zephyr).
  The MPU regions stay per thread as the firmware defines them today.
- Multicore. Everything assumes one CPU.

## Benchmarks

The numbers below come from a throwaway micro-benchmark harness that ran in
place of KernelMain on QEMU; it is not kept in the tree. Times came from the
tick counter refined with the SysTick down-counter, so they are QEMU virtual
time: only the relative numbers between the two backends on the same board
mean anything.

Results in ns per iteration (2026-09-02, `-Os`, all boot services still
running in the background):

| Benchmark | gabbro shim | gabbro native | flint shim | flint native |
| --- | ---: | ---: | ---: | ---: |
| irq_lock_unlock | 151 | 142 | 82 | 102 |
| mutex_lock_unlock (uncontended) | 927 | 413 | 610 | 283 |
| sem_give_take (no wait) | 658 | 347 | 398 | 247 |
| msgq_put_get (4 bytes, no wait) | 723 | 602 | 458 | 341 |
| yield_switch (two equal-priority threads) | 10869 | 11323 | 10756 | 10588 |
| sem_pingpong (round trip, two switches) | 28647 | 23715 | 26421 | 21401 |
| msgq_pingpong (round trip) | 30805 | 25265 | 26433 | 22376 |
| poll_wake (three-queue group, one wake) | 29109 | 24520 | 25681 | 23205 |
| irq_to_thread (pended IRQ to waiting thread) | 2000 | 1620 | 1384 | 1244 |
| thread_create_exit | 33149 | 24441 | 29712 | 21871 |
| sleep_1_tick | 997605 | 1001495 | 998345 | 996250 |

Reading the numbers:

- Uncontended primitives are 1.2x to 2.2x faster natively. The shim pays for
  the FreeRTOS queue machinery plus the wrapper, the native objects are a
  counter or a byte ring with an owner pointer.
- Anything that context switches is dominated by PendSV plus the MPU
  reprogramming and the QEMU cost of the exception itself, so the two
  backends land within 20% of each other. `yield_switch` is the same code
  path in both and shows the noise floor of the method: about 5%.
- `sleep_1_tick` is a sanity check that the tick source runs at
  `CONFIG_KERNEL_TICK_HZ`.
- `thread_create_exit` sleeps a tick every 16 iterations (excluded from the
  measurement) so that FreeRTOS's idle task can reclaim deleted TCBs; the
  native backend needs no reclamation since the thread object is static.

Code size of the kernel library alone (`arm-none-eabi-size`, `-Os`):

| Board | FreeRTOS shim + FreeRTOS | Native |
| --- | --- | --- |
| qemu_gabbro (Cortex-M33) | 12541 text / 57 data / 744 bss | 4725 text / 0 data / 1218 bss |
| qemu_flint (Cortex-M4) | 12403 text / 57 data / 744 bss | 4725 text / 0 data / 1218 bss |

The native bss is mostly the 1 KiB idle stack; the shim's idle task stack
comes out of the FreeRTOS heap instead.

## Verification

- `tests/kernel/test_native_kernel`: 18 host tests on the POSIX arch.
- qemu_gabbro and qemu_flint: boot to the watchface, launch an app from the
  launcher and back out, a gdb breakpoint sweep on every assert and fault
  entry point, and the benchmark suite. Not yet run on real hardware.
