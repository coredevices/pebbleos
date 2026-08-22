# TCG→wasm JIT build (ktock/qemu-wasm overlay)

The TCI interpreter plateaus around 4.5 fps on UI scrolling. This overlay
builds the emulator on ktock/qemu-wasm branch `wasm64-tcg-b` (QEMU
10.2.50 with the TCG WebAssembly JIT backend), which compiles hot
translation blocks to native wasm modules.

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
  `pebble_gpio.c` gains `pebble_set_button_state()` (pebble.c provided it
  on the legacy machines); the legacy STM32 path in `pebble_control.c`
  is compiled out.
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

Artifacts: `qemu-system-arm.js` + `.wasm` (~9 MB; emsdk 4.x emits no
separate worker file). Drop-in for the web shell — same boot args, same
glue exports (MEMORY64=2 lowers pointers to 32-bit at the JS boundary).

Retire this overlay when the TCG wasm backend series merges into
upstream QEMU and the coredevices fork rebases onto it.
