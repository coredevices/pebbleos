# libOS

libOS holds the platform helpers the firmware and the Dialog Bluetooth FW share:
assertions, the allocator hooks and the Cortex-M MCU state helpers. Threading
lives in `kernel/`.

## Dependencies:

- libc
- libutil
- A handful of platform specific functions, see platform.c
