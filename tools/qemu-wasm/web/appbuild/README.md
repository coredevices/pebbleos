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
2. `appbuilder.js` reads `package.json` (or a legacy `appinfo.json`),
   then drives the pipeline. `deps.js` installs the project's SDK
   packages from npm — headers and the prebuilt per-platform archive for
   a C library, the module itself for a JS one like Clay.
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

Steps 3 to 5 run once per platform: as with the real SDK, one `.pbw`
carries every target platform it could build, so the result installs on
any board. A project with a `worker_src/` also gets its background
worker built and bundled.

The toolchain binaries come from `tools/wasm-toolchain/` (LLVM 19.1.7,
ARM backend only, built for wasm32-wasi-threads); `web/tools/` holds them
gzipped alongside one `sdkpack-<platform>.zip` per board, which
`make-sdkpack.sh` assembles from an exported SDK. The build follows the
page's board selector; emery, flint and gabbro are supported, and the
compiler itself is shared between them so switching boards only fetches
that board's ~2.7 MB SDK.

Per-board build parameters (defines, app memory, colour vs black and
white) are transcribed into `codegen.js` from the SDK's own
`pebble_sdk_platform.py`. They are not worth hand-maintaining: an app
built without `PBL_ROUND` compiles its rectangular layout without
complaining.

## What it does not build

Moddable (`projectType: "moddable"`) and Rocky projects are JavaScript
compiled by Moddable's XS toolchain, not by clang, so they are rejected
with that reason rather than a compile error. Building SDK *libraries*
(`projectType: "package"`) is likewise out of scope — a library is
consumed here, not produced. Only the platforms with a
`sdkpack-<platform>.zip` can be targeted; the frozen legacy boards
(aplite, basalt, chalk, diorite) have no PebbleOS SDK export.

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
