# In-browser app compiler

Builds a Pebble SDK app from source in the browser and hands the emulator
a ready-to-install `.pbw`. Paste a GitHub repo URL into the page's
"Build & run" box; nothing is uploaded and no server is involved.

Building `coredevices/pebble-timer` takes about four seconds once the
toolchain is cached, and roughly a minute the first time while ~39 MB of
tools download.

## How it fits together

`appbuild-worker.js` runs the whole thing off the main thread:

1. Fetch the repository zip (direct, falling back to the pkjs proxy —
   codeload.github.com sends no CORS headers) and the toolchain from
   `../tools/`, which is cached in the Cache API after the first build.
2. `appbuilder.js` reads `package.json`, then drives the pipeline.
3. `resources.js` turns the app's images and fonts into an
   `app_resources.pbpack`.
4. `codegen.js` writes the files waf would have generated —
   `appinfo.auto.c`, `resource_ids.auto.{c,h}`, `message_keys.auto.{c,h}`
   and the linker script.
5. `wasi-run.js` runs `clang.wasm` over each source and then `lld.wasm`
   over the objects, against an in-memory filesystem.
6. `jsbundle.js` bundles the app's PebbleKit JS with esbuild.
7. `elfpack.js` converts the ELF to a flat binary, injects the app
   metadata and writes the `.pbw`.

The toolchain binaries come from `tools/wasm-toolchain/` (LLVM 19.1.7,
ARM backend only, built for wasm32-wasi-threads); `web/tools/` holds them
gzipped alongside `sdkpack-emery.zip`, which `make-sdkpack.sh` assembles
from an exported SDK.

## What differs from a waf build

FreeType is not available in the browser, so glyphs are rasterised with
canvas and thresholded to 1bpp. Text renders correctly but a glyph can
differ from FreeType's by a pixel.

Two SDK behaviours are reproduced rather than corrected, because matching
waf's output is what matters: `bitmapgen._set_bbox()` never crops empty
right-hand columns (it transposes its columns back into rows before
computing that edge), and PBI palette order is left to us since the SDK
takes it from a Python set, whose iteration order cannot be reproduced —
the rendered image is identical either way.

## Tests

Run from this directory, against a golden waf build of pebble-timer:

    node test-resources.mjs [path-to-pebble-timer]
    node test-elfpack.mjs

`test-resources.mjs` checks the pbpack container byte-for-byte, compares
every resource against the golden pack, and falls back to comparing
decoded pixels where the byte layout legitimately differs.
