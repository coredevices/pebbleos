# Emscripten link-flag notes

The reference build (coredevices/qemu `build-wasm/`, emsdk 3.1.50) links
`qemu-system-arm.js` with, among others:

```
-pthread -sASYNCIFY=1 -sPROXY_TO_PTHREAD=1 -sFORCE_FILESYSTEM
-sALLOW_TABLE_GROWTH -sTOTAL_MEMORY=2GB -sWASM_BIGINT -sEXPORT_ES6=1
-sASYNCIFY_IMPORTS=ffi_call_js
-sEXPORTED_RUNTIME_METHODS=addFunction,removeFunction,TTY,FS,HEAPU8,HEAPU32,callMain
-sPTHREAD_POOL_SIZE=4 -sUSE_SDL=2
```

(from `configs/meson/emscripten.txt` in the qemu tree plus the wasm-deps
LDFLAGS; `-sEXPORT_ES6=1` implies MODULARIZE, hence the dynamic `import()`
in `web/index.html` and `smoke-test.mjs`, and `"type": "module"` in
`web/package.json`.)

## What the shell needs

Covered by the current flags: `FS` (serial log polling, firmware upload in
`preRun`), `HEAPU8`/`HEAPU32` (display blit, button mask writes), and the
`EMSCRIPTEN_KEEPALIVE` browser glue in `hw/display/pebble_display.c`
(`pebble_wasm_display_{width,height,stride,data,frame_count}`) and
`hw/gpio/pebble_gpio.c` (`pebble_wasm_button_state_addr`), which need no
`-sEXPORTED_FUNCTIONS` entry. No extra relink flags are required for the
page as written.

## Possible future flags / glue

- `ENV` in `-sEXPORTED_RUNTIME_METHODS` — set guest environment variables
  from JS before `main()` runs (`PEBBLE_QEMU_FIRST_BOOT_LOGIC_ENABLE`,
  `PEBBLE_QEMU_START_CONNECTED`, `PEBBLE_QEMU_START_PLUGGED_IN` are read
  by `pebble_generic.c`). Without it the defaults (start connected)
  apply.
- Touch injection (emery/gabbro): natively `./pbl touch` drives
  `hw/misc/pebble_touch.c` through QMP `input-send-event`, which the
  browser build doesn't run. Needs a QMP-less glue export in the touch
  device (e.g. a shared-memory x/y/pressed record polled on a
  virtual-clock timer, like the button mask). Do not fake it from JS.

## SDL display experiment (not wired into the page)

The build also contains QEMU's SDL2 UI compiled against emscripten's SDL2
port (`-sUSE_SDL=2`, `CONFIG_SDL`). In principle `-display sdl` with a
`Module.canvas` renders without any of the `pebble_wasm_*` glue, and would
also route keyboard/pointer events through QEMU's input layer (including
`pebble_touch`). Unverified under `-sPROXY_TO_PTHREAD` (SDL calls are
proxied to the main thread); if it is ever made to work,
`-sOFFSCREENCANVAS_SUPPORT=1` is worth trying to render from the QEMU
pthread directly. The shared-memory path above is the proven, primary
mechanism.
