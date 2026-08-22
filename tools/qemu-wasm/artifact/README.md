# Single-file emulator page

Packs the whole emulator — QEMU wasm, runtime JS, pthread worker, and the
release firmware images — into one self-contained HTML file (gzip + base64,
unpacked in the browser with `DecompressionStream`). Useful anywhere only a
single static page can be hosted, e.g. Claude artifacts.

The page needs a host that serves COOP/COEP headers (SharedArrayBuffer);
it probes the environment first and explains what is missing instead of
hanging.

Use the size-reduced QEMU build so the page fits common limits: configure
with `--without-default-devices --with-devices-arm=pebble` (the Pebble
machines only — 25 MB wasm instead of 39 MB, ~13.8 MB page total).

```sh
python3 assemble.py <qemu-build-dir> pebble-emulator.html
node page-test.mjs pebble-emulator.html coi 300 shot.png    # boots it headless
node page-test.mjs pebble-emulator.html nocoi 30 shot.png   # diagnostics path
```

`assemble.py` also patches one emscripten 3.1.50 line whose
`new URL(..., import.meta.url)` throws when the module is imported from a
blob: URL inside the pthread worker (see comment in the script).
