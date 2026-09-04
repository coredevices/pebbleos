# Kernel

`kernel/` owns every RTOS primitive the firmware uses. The public API lives in
`include/pbl/kernel/` and is the only threading interface the rest of the tree
may use. The implementation is PebbleOS's own; its internals are described in
[kernel internals](kernel_native.md).

## Layout

```
include/pbl/kernel/     public API: types, irq, thread, mutex, sem, msgq, poll, sched, idle, debug
kernel/                 backend-independent code: tick conversion
kernel/native/          scheduler, objects and the Cortex-M and POSIX ports
kernel/native/include/pbl/kernel/backend.h   per-object private state
```

## Objects

| Object | Struct | Static define |
| --- | --- | --- |
| Thread | `struct pbl_thread` | `PBL_THREAD_STACK_DEFINE` + `pbl_thread_create` |
| Mutex | `struct pbl_mutex` | `PBL_MUTEX_DEFINE` |
| Semaphore | `struct pbl_sem` | `PBL_SEM_DEFINE` |
| Message queue | `struct pbl_msgq` | `PBL_MSGQ_DEFINE` |
| Poll group | `struct pbl_poll_group` | `PBL_POLL_GROUP_DEFINE` |

Conventions:

- Every object is a caller-owned struct; there is no create/destroy pair that
  hands out heap handles. A `PBL_*_DEFINE` is a complete static initialiser,
  so a defined object is usable before the scheduler starts and needs no init
  call. `pbl_*_init()` exists for objects that live in dynamically allocated
  memory.
- Blocking calls take a `pbl_timeout_t` (`PBL_NO_WAIT`, `PBL_FOREVER`,
  `PBL_MSEC()`, `PBL_TICKS()`). The struct type stops ticks/ms mix-ups.
- Return `int`: 0 on success, `-EAGAIN` on timeout, `-EBUSY` for `PBL_NO_WAIT`.
- No `_from_isr` variants: `pbl_in_isr()` picks the path and any needed
  context switch is requested inside the call.
- Mutexes are recursive with owner and lock-site tracking. Non-recursive
  intent is expressed with `pbl_mutex_assert_held(m, false)`.
- Priorities: higher is more urgent, `PBL_PRIO_IDLE` .. `PBL_PRIO_MAX`.
- Returning from a thread entry function ends the thread.

### Threads

`pbl_thread_create()` takes a `struct pbl_thread_attr`: entry, priority,
privileged flag, a caller-owned stack and up to four `MpuRegion`s that are
switched in with the thread. `pbl_thread.tls[]` holds the per-thread pointers
the syscall layer needs.

### Introspection

`debug.h` covers what core dumps, fault handling, stack checks and telemetry
need: a thread walk with saved registers in the canonical order the core dump
format expects, saved PC/LR/CONTROL of a blocked thread, stack bounds and
high-water marks, and a run-time stats snapshot.

### Idle

The SoC tickless-idle code talks to the kernel through `pbl/kernel/idle.h`:
`pbl_soc_idle()` and `pbl_soc_tick_enable()` are implemented per SoC, and
`pbl_idle_confirm()`, `pbl_idle_slept()` and `pbl_kernel_tick_isr()` are what
the kernel provides in return.

## Configuration

`kernel/Kconfig` holds the tick rate, the number of priorities, the NVIC
priority levels the kernel uses, the thread limit, the TLS slot count and the
stack alignment the MPU guard needs.
