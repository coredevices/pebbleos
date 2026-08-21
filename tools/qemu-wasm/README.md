# PebbleOS in the browser (QEMU + WebAssembly)

QEMU compiled to WebAssembly boots real PebbleOS firmware and renders the
watch display to an HTML canvas — no install, no server-side emulation.

## History

Pebble's original emulator was a QEMU 2.5 fork with STM32 peripheral models.
[ericmigi/pebble-qemu-wasm](https://github.com/ericmigi/pebble-qemu-wasm)
ported those device models to QEMU 10.1 and solved the browser-integration
problems (emscripten pthreads, main-thread proxying, canvas rendering,
input injection); this shell is derived from it. What changed since: the
[coredevices/qemu](https://github.com/coredevices/qemu) fork replaced the
STM32 models with generic virtual Pebble machines (`pebble-emery`,
`pebble-flint`, `pebble-gabbro` in `hw/arm/pebble_generic.c`) that expose
simple MMIO peripherals — a plain framebuffer display, 4-button GPIO,
touch, UARTs — instead of emulating a specific MCU. That is the same QEMU
`./pbl qemu` uses natively (see `docs/development/qemu.md`), so the browser
build now tracks mainline PebbleOS instead of a firmware fork.

`web/coi-serviceworker.min.js` is
[coi-serviceworker](https://github.com/gzuidhof/coi-serviceworker) v0.1.7
(MIT, Guido Zuidhof and contributors), copied via ericmigi/pebble-qemu-wasm.

## Quick start

```sh
# 1. Build QEMU for wasm32 (one-off, ~30 min)
./build-qemu-wasm.sh ~/coredevices/qemu

# already have a build? just copy the artifacts:
./get-artifacts.sh ~/coredevices/qemu/build-wasm

# 2. Fetch release firmware
./fetch-firmware.sh emery

# 3. Serve and open
./serve.py 8080
open http://localhost:8080
```

Click Boot. The page fetches the firmware images (~34 MB) and the WASM
binary, then boots. The boot logo appears within seconds; a full boot to
the launcher takes about a minute (TCI interpreter, measured under node on
4 cores).

The dev server sends `Cross-Origin-Opener-Policy` /
`Cross-Origin-Embedder-Policy` headers required for `SharedArrayBuffer`
(emscripten pthreads). On static hosts without those headers the bundled
coi-serviceworker provides them after one reload.

URL parameters: `?board=emery|flint|gabbro`, `?auto` (boot immediately),
`?audio` (enable the SDL audio backend).

## Headless smoke test

```sh
node smoke-test.mjs --board emery --seconds 300
```

Boots the WASM build under node with `-display none`, tails the UART2
debug console, and exits 0 once boot markers (ending with
`Ready for communication.`) appear and the exported display surface
reports the board's resolution with an advancing frame counter.

## How it works

- **Machine**: `-machine pebble-emery -kernel qemu_micro_flash.bin -drive
  if=mtd,format=raw,file=qemu_spi_flash.bin -display none`. Code flash is
  mapped at 0x0, so the raw release image boots directly via `-kernel`;
  the 32 MB SPI flash image backs the external-flash device.
- **Display**: firmware writes pixels to framebuffer MMIO at 0x50000000;
  `hw/display/pebble_display.c` converts them (ARGB2222 or 1bpp) to a
  QemuConsole surface. Under emscripten a virtual-clock timer re-renders
  that surface every 33 ms even with `-display none`, and the
  `pebble_wasm_display_*` exports hand the page a heap pointer to the
  32bpp surface plus a frame counter. The page polls the counter on a
  30 ms interval and blits changed frames (BGRX to RGBA) into a canvas
  `ImageData` — the same shared-memory scheme the old shell proved out.
- **Input**: `hw/gpio/pebble_gpio.c` exports
  `pebble_wasm_button_state_addr()`, the heap address of a button
  bitmask (bit 0 Back, 1 Up, 2 Select, 3 Down). Key and pointer events
  `Atomics.store` the mask; a 16 ms virtual-clock poll inside the device
  applies press and release edges, so held buttons work.
- **Serial**: UART2 (debug console) is routed to a MEMFS file the page
  polls; UART1 (pebble-tool control protocol) is unconnected in the
  browser.

## Controls

| Input                       | Button |
|-----------------------------|--------|
| Arrow Left, Escape, Backspace | Back |
| Arrow Up / `w`              | Up     |
| Arrow Right, Enter, `s`     | Select |
| Arrow Down / `x`            | Down   |

On-screen buttons track pointer down/up, so press-and-hold works.

## Known limitations

- TCI interpreter: roughly 1-2 orders of magnitude slower than native
  TCG. Expect multi-minute boots and single-digit FPS.
- No app install: pebble-tool speaks TCP to UART1, which has no transport
  in the browser build.
- No touch yet on emery/gabbro: natively `./pbl touch` injects pointer
  events over QMP, which the browser build doesn't run (see NOTES.md).
- flint and gabbro machines exist but are untested in the browser; the
  board selector will boot them if firmware is fetched.
- Audio is off by default (`?audio` to try the SDL backend).

For the native QEMU workflow (`./pbl qemu`, buttons via monitor `sendkey`,
touch via QMP, gdb, screenshots) see `docs/development/qemu.md`.

## Files

- `web/index.html` — emulator page
- `web/coi-serviceworker.min.js` — COOP/COEP fallback for static hosts
- `serve.py` — dev server with COOP/COEP headers
- `build-qemu-wasm.sh` — reproducible emsdk + deps + QEMU build
- `get-artifacts.sh` — copy `qemu-system-arm.{js,wasm}` into `web/`
- `fetch-firmware.sh` — download release firmware into `web/firmware/`
- `smoke-test.mjs` — headless boot test under node
- `NOTES.md` — link-flag requirements for the emscripten build
