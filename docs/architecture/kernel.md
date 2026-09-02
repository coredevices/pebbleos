# Kernel

`kernel/` owns every RTOS primitive the firmware uses. The public API lives in
`include/pbl/kernel/` and is the only threading interface the rest of the tree
may use. FreeRTOS is an implementation detail of `kernel/freertos/`.

## Layout

```
include/pbl/kernel/     public API: types, irq, thread, mutex, sem, msgq, poll, sched, debug
kernel/                 backend-independent code: kobj init walk, tick conversion
kernel/freertos/        shim over third_party/freertos
kernel/freertos/include/pbl/kernel/backend.h   per-object backend state (a FreeRTOS handle)
```

Exactly one backend directory is on the include path, so `backend.h` selects
the per-object private state at compile time.

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
  hands out heap handles.
- Blocking calls take a `pbl_timeout_t` (`PBL_NO_WAIT`, `PBL_FOREVER`,
  `PBL_MSEC()`, `PBL_TICKS()`). The struct type stops ticks/ms mix-ups.
- Return `int`: 0 on success, `-EAGAIN` on timeout, `-EBUSY` for `PBL_NO_WAIT`.
- No `_from_isr` variants: `pbl_in_isr()` picks the path and any needed
  context switch is requested inside the call.
- Mutexes are recursive with owner and lock-site tracking. Non-recursive
  intent is expressed with `pbl_mutex_assert_held(m, false)`.
- Priorities: higher is more urgent, `PBL_PRIO_IDLE` .. `PBL_PRIO_MAX`.

### Static definition under a runtime-initialised backend

FreeRTOS objects need runtime construction, so each `PBL_*_DEFINE` also emits
a `struct pbl_kobj_init` record into `.pbl_kobj_init`; `pbl_kernel_init()`
walks that section once before anything else runs. A backend whose
initialisers are complete at compile time makes the walk a no-op.

The FreeRTOS fork is V8.2.1 and predates `xCreateStatic()`, so the shim keeps
the FreeRTOS control block on the heap behind `backend.handle`. That matches
today's cost and goes away with a native backend.

### Threads

`pbl_thread_create()` takes a `struct pbl_thread_attr`: entry, priority,
privileged flag, a caller-owned stack and up to four `MpuRegion`s that are
switched in with the thread. The shim keeps the `struct pbl_thread *`
back-pointer in the last FreeRTOS TLS slot; `pbl_thread.tls[]` replaces the
remaining FreeRTOS TLS pointers.

### Introspection

`debug.h` covers what core dumps, fault handling, stack checks and telemetry
read from FreeRTOS internals: a thread walk with saved registers in the
canonical order the core dump format expects, saved PC/LR/CONTROL of a
blocked thread, stack bounds and high-water marks, and a run-time stats
snapshot.

## Boundary

`third_party/freertos` exports its include directories only to
`kernel/freertos`, which also holds `FreeRTOSConfig.h` and the port hooks.
Anything else that includes a FreeRTOS header fails to compile. The SoC
tickless-idle code talks to the kernel through `pbl/kernel/idle.h`; NimBLE's
NPL port is written against the pbl API.
