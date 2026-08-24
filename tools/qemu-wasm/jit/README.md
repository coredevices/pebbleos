# TCG→wasm JIT build (ktock/qemu-wasm overlay)

The TCI interpreter plateaus around 4.5 fps on UI scrolling. This overlay
builds the emulator on ktock/qemu-wasm branch `wasm64-tcg-b` (QEMU
10.2.50 with the TCG WebAssembly JIT backend), which compiles hot
translation blocks to native wasm modules.

The overlay now carries BOTH machine sets — the generic boards
(pebble-emery/flint/gabbro) and the classic STM32 boards (pebble-bb2,
pebble-snowy-bb, pebble-s4-bb, pebble-silk-bb) — in one binary, so
`web/qemu-classic/` is no longer needed once this build ships. Classic
boot-to-ready on this container (icount shift=5,sleep=on, same as the
TCI numbers): basalt ~18s, chalk ~18-20s, diorite ~31-33s, aplite ~72s
(TCI: 22/24/35/70). Boot is virtual-time bound, so the JIT's win is
runtime speed (display refresh holds the 30 fps cap where classic TCI
sat at single digits), not boot time. Classic boot time is dominated by
firmware-side fixed delays; icount shift changes barely move it.

Measured (node, 4 shared cores, pebble-emery v4.35.0, continuous
launcher scroll):

| build | boot to ready | scroll fps |
|---|---|---|
| unpatched TCI | 91 s | 2.1 |
| patched TCI (`../patches/0001`) | 38 s | 4.5 |
| **wasm JIT (this overlay)** | **15 s** | **16.9** |

## Contents

- `overlay/` — the generic-machine device set, ported from the
  coredevices pebble-10.1 tree to QEMU 10.2 APIs. Drift handled:
  core headers moved under `hw/core/` (`irq.h`, `sysbus.h`, `boards.h`,
  `qdev-*.h`), `CharBackend` renamed `CharFrontend`, `audio/audio.h`
  replaced by a rewritten audio subsystem (`pebble_audio.c` here is a
  register-compatible drain-only stub), and machines must declare
  `arm_machine_interfaces` to be visible in the single-binary machine
  registry (`machines-qom.h`) — without it `-machine help` lists nothing.
  The classic set (`hw/arm/pebble.c`, `pebble_silk.c`, the
  `pebble_stm32f2xx/f4xx` SoCs, `stm32_pebble_*` peripherals, snowy and
  memory-LCD displays) is ported the same way, plus: `pebble_mx25u.c`
  and `stm32_pebble_dma.c`; the legacy `pebble_control_create()` path is
  re-enabled and arms the browser rx ring poll timer; `pebble.c` keeps
  `pebble_set_button_state()` (the stub that pebble_gpio.c used to carry
  is dropped); `hw/block/pflash_cfi02.c` is the coredevices version —
  it carries Macronix MX29VS128FB extensions (write-to-buffer 0x25,
  read-status 0x70, CFI entry at 0x555) without which the snowy/s4
  bootloader dies with SAD WATCH 0xfe504502 (ERROR_BAD_SPI_FLASH) — and
  the wasm-safe coalesced block persistence. `pebble_control.c` also carries the browser serial
  bridge (`pebble_wasm_serial_ctrl()` ring buffers, emscripten-only)
  that `web/pebble-transport.js` uses for app installs.
- `0001-ktock-tree-build-wiring.patch` — meson/Kconfig/devices-config
  wiring plus the browser-glue link flags for the ktock tree.
- `build-deps64.sh` — emsdk 4.0.23 and the MEMORY64=2 dependency chain
  (zlib github mirror, libffi 3.5.2 release tarball for the wasm64
  target, pixman/glib reusing the wasm32 source trees). Proxy-blocked
  hosts (zlib.net, gitlab.freedesktop.org) are already routed around.

## Build

```sh
git clone --single-branch --branch wasm64-tcg-b https://github.com/ktock/qemu-wasm
cp -r overlay/* qemu-wasm/
(cd qemu-wasm && git apply < 0001-ktock-tree-build-wiring.patch)
./build-deps64.sh
cd qemu-wasm && mkdir build-wasm-jit && cd build-wasm-jit
source /home/user/wasm-deps64/env.sh
emconfigure ../configure --static --cpu=wasm64 --target-list=arm-softmmu \
  --without-default-features --enable-system --disable-tools --disable-docs \
  --disable-pie --without-default-devices --with-devices-arm=pebble \
  --extra-cflags="-O3 -msimd128 -sMEMORY64=2" --extra-ldflags="-sMEMORY64=2" \
  --enable-wasm64-32bit-address-limit
ninja qemu-system-arm.js
```

Caveat: meson caches cross-file contents at setup time. If
`configs/meson/emscripten.txt` changes after configure (e.g. the
`-sASYNCIFY_STACK_SIZE=1048576` link flag), reconfigure from a fresh
build dir — `ninja` alone will not pick it up.

Artifacts: `qemu-system-arm.js` + `.wasm` (~9 MB; emsdk 4.x emits no
separate worker file). Drop-in for the web shell — same boot args, same
glue exports (MEMORY64=2 lowers pointers to 32-bit at the JS boundary).

Retire this overlay when the TCG wasm backend series merges into
upstream QEMU and the coredevices fork rebases onto it.
