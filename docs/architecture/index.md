# Architecture overview

A guided tour of how PebbleOS is put together, deep-linking to the design
prose that lives in the source tree. The in-source comments are canonical —
these pages summarize and point, they do not duplicate.

## Layering

PebbleOS is a FreeRTOS-based firmware (kernel vendored as a submodule under
`third_party/freertos`). The main source layers, as described on the
<a href="../apidoc/index.html">API reference</a> main page:

- `src/fw/applib` — application framework and UI, the API surface exposed to
  watchapps ([SDK export](../development/sdk_export.md) describes how
  functions get there).
- `src/fw/services` — system services (Bluetooth, filesystem, activity, …).
- `src/fw/kernel` — task management, events, memory.
- `src/fw/drivers` — hardware drivers (public interfaces under
  `include/pbl/drivers`).
- `subsys/` — OS subsystems shared beyond the firmware tree; currently
  logging, included via the `pbl/logging/` header path.

Alongside these sit `src/fw/shell` (launcher/watchface UX flow),
`src/fw/process_management` (app lifecycle) and `src/fw/comm` (phone
communication).

## Task model

The firmware runs a fixed set of FreeRTOS tasks, enumerated in
`src/fw/kernel/pebble_tasks.h`: KernelMain, KernelBackground, Worker, App,
the Bluetooth tasks (host, controller, HCI), NewTimers and
[PULSE](../reference/pulse2/pulse2.md). `main()`
(`src/fw/main.c`) performs SoC init and spawns KernelMain, which brings up
the rest of the system.

## Boot and firmware variants

The bootloader is not part of this tree; it selects which firmware image to
launch, coordinated through boot bits (see `BOOT_BIT_*` usage in
`src/fw/main.c`) and the slot metadata in `src/fw/system/firmware_storage.h`.

Normal firmware and PRF (Pebble Recovery Firmware — the minimal fallback
image used to reinstall the main firmware) are separate compile-time
variants: `./pbl configure --variant=prf` (see
[build options](../development/options.md)) applies `src/fw/prj_prf.conf` on
top of the base config, disabling the JS engine and marking the image as
recovery firmware.

## Processes and apps

- **App identity** — an installed app is referred to by several identifiers
  (`AppInstallId`, `AppInstallEntry`, UUID, `PebbleProcessMd`); the comment
  at the top of `src/fw/process_management/app_install_manager.h` explains
  which to use where and which are deprecated.
- **Shell flow** — which app launches at startup and what happens when an
  app exits is a deliberately flat state machine rooted in the launcher and
  the watchface; see the diagrammed comment at the top of
  `src/fw/shell/normal/system_app_state_machine.c`.
- **Privilege boundary** — third-party app and worker code runs unprivileged
  (processes built into the firmware stay privileged); anything
  touching OS state crosses into the kernel through a `sys_*` syscall
  (declared in `src/fw/syscall/syscall.h`) defined with `DEFINE_SYSCALL`
  from `src/fw/syscall/syscall_internal.h`, which raises privileges on
  entry and drops them on return unless the caller was already privileged.

## Memory layout

`src/fw/linker/pebbleos.ld` ("Section concepts!" comment) documents the
flash/RAM picture: VMA vs LMA, the kernel data/bss/stack/heap region, and
the fixed app region where third-party app code, data and heap live.
`src/fw/linker/memory.ld` explains how SRAM is carved between kernel, app
and worker regions, and `src/fw/linker/regions.ld` notes the MPU
power-of-two constraints.

## Bluetooth

`src/fw/comm/ble/` is the host-side BLE layer and carries substantial design
prose:

- `gap_le_connect.c` — connection management and the "connection intent"
  abstraction that virtualizes the link across multiple clients.
- `gap_le_advert.c` — round-robin scheduling of multiple advertising jobs
  onto a controller that only holds one payload at a time (the comment
  predates the current controllers, but the scheduler it describes is still
  in use).
- `gap_le_connect_params.c` — a primer on connection intervals, slave
  latency and supervision timeouts, and why the firmware deviates from the
  spec-recommended parameter-update pause for iOS.

Beneath it, the transport is a pluggable backend selected per SoC in
`src/bluetooth-fw/` — NimBLE (`third_party/nimble`) for all current boards,
plus QEMU and stub backends.

## Storage

PFS, the Pebble File System, lives in `src/fw/services/filesystem/`. The API
and on-flash layout are documented in `pfs.h`; the wear-leveling strategy
(round-robin page allocation tracking the last written page) is described
alongside the allocator in `pfs.c`.

## Drivers

- **Accelerometer** — `include/pbl/drivers/accel.h` explains the split
  between the dumb low-level driver and the accel service that owns
  buffering, clients and subsampling, so the same service code runs on any
  accel part.
- **Flash** — `src/fw/drivers/flash/README.md` documents the two flash APIs:
  the main one, and a coredump-only path that must work without OS services.

## Coredumps

The on-flash coredump image format (header plus chunked records, including
per-thread register sets) is documented in `src/fw/kernel/core_dump.c`.

## Design documents

Longer design documents live as their own pages:

```{toctree}
:maxdepth: 1
activity/index.md
```
