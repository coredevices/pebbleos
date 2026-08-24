# Migrating to clang/LLVM and replacing waf

Status: proposal. This documents a recommended path for building PebbleOS
with clang/LLVM instead of arm-none-eabi-gcc, and for replacing waf with a
modern build system. It is based on a full audit of the tree (build
machinery inventory and a clang-compatibility pass over the firmware
source, including codegen verification of the risky patterns against
clang 18).

## 1. Why

- **One toolchain, every host.** Clang is a cross-compiler by
  construction and works as an in-process library. That is what makes
  firmware compilation possible in a browser (clang compiled to
  WebAssembly/WASI) and on iOS (libclang/LLD linked into an app —
  iOS forbids `fork`/`exec`, which rules out gcc's multi-process driver
  there entirely). Android's official NDK is already LLVM.
- **Determinism.** The same clang binary on Linux, macOS, browser, and
  phone produces bit-identical firmware. The in-browser app SDK derived
  from this tree already demonstrates this: clang-in-wasm builds are
  byte-identical to native clang builds.
- **Tooling.** clangd, clang-tidy, sanitizers for host tests, ThinLTO,
  and `compile_commands.json` all come from the same ecosystem the
  firmware would now be built with.
- **waf is the bottleneck, not the C.** The audit found ~19,000 lines of
  build description, of which ~90% is mechanical source lists. The
  firmware source itself is unusually clang-clean.

## 2. Where the tree is today

What is **already portable** (no work needed):

- Kconfig: real kconfiglib with Zephyr-style layering
  (`boards/<b>/defconfig` → `prj.conf` → `prj_<variant>.conf` → CLI
  overrides), 196 Kconfig files, `menuconfig`. Only a small
  waf-specific shim (`tools/waf/kconfig.py` mirroring CONFIG_* into
  DEFINES for waf's dependency scanner) disappears with waf.
- Board model: `boards/<b>/<b>.yml` parsed by `tools/waf/boards.py`
  (no waflib import).
- Flash/debug runners: `tools/runners/{core,openocd,nrfutil,sftool}.py`
  are already standalone.
- Most artifact writers: `bitmapgen.py`, `fontgen.py`, `pbpack.py`,
  `mkbundle.py`, `png2pblpng.py`, the native-SDK generator — plain
  Python, callable as scripts.

What is **coupled to waf** (~7,500 LOC of Python against the waf API +
~11,350 LOC of wscript): the root `wscript`, `tools/waf/*` (linker-script
snippet aggregation, libc selection, toolchain setup, the 795-line unit
test framework), `tools/resources/waftools/*` (the resource pipeline and
its eight generated `.auto.*` outputs), the SDK export, and 297
`wscript`/`wscript_build` files — of which only 26 contain custom logic.

What is **gcc-specific** in the source (from the clang audit):

- 36 `register x __asm("lr")` / `__asm("sp")` named-register-variable
  sites (33 in `src/fw/kernel/pbl_malloc.c`). **Clang silently deletes
  these reads** — verified against clang 18. This is the single true
  correctness landmine and is fixed independently of everything else by
  switching to `__builtin_return_address(0)` / an explicit
  `mov %0, sp` asm, a no-op under gcc.
- `-specs=` (picolibc/nano/nosys) — a gcc driver feature clang ignores
  silently; libc include/lib paths must be spelled out explicitly.
- `-fvar-tracking-assignments`, `-mcpu=star-mc1` (SF32LB52) — unknown to
  clang (clang 18); star-mc1 needs a `cortex-m33` substitution until
  LLVM grows the CPU.
- Stale scaffolding: the existing `--use_clang` path searches 2013-era
  toolchain paths and, worse, **disables `-Wall -Wextra -Werror`**
  (which is exactly what would catch the named-register bug).
- Three graphics hot-path files rely on `#pragma GCC optimize` (silently
  dropped by clang; needs per-file `-O3`/`-O2` flags instead).
- Nine layout `_Static_assert`s are keyed on `#ifndef __clang__` (meant
  for host tests) and must be re-keyed on `UNITTEST` so a clang firmware
  build keeps its layout verification.
- The log-hash macro's address arithmetic folds into relocation addends
  under gcc but not clang: ~8 bytes × ~2,350 `PBL_LOG` sites ≈ 18 KB of
  flash. Correctness-neutral; recoverable by restructuring one macro.

Everything else checked out clean: the `DEFINE_SYSCALL` naked-function
machinery compiles byte-identically under clang, the fault handlers and
literal-pool asm assemble correctly, there are no nested functions,
computed gotos, or standalone `.S` files, and all ten linker scripts use
only constructs LLD parses (verified against ld.lld 18).

## 3. Recommended target architecture

**Toolchain:** clang + the integrated assembler, `--target=arm-none-eabi`,
`-fshort-enums` (matching the gcc ABI), picolibc built with clang
(picolibc CI-tests clang upstream and is already the default libc in
Kconfig). Link with LLD and compiler-rt builtins once parity is proven;
GNU ld + libgcc are retained as the first-step link to de-risk the
compiler switch. Keep gcc buildable throughout the transition via CI on
both toolchains.

**Build system: CMake (ninja generator) + the existing kconfiglib
layer.** Reasons over Meson/Bazel:

- The multi-toolchain problem. One logical build needs three
  toolchains: ARM firmware, host clang for unit tests, and the ARM PIE
  app ABI for stored apps. CMake handles this as three configures of
  the same tree with different toolchain files, orchestrated by `pbl` —
  which is exactly how the waf "envs" work today, so the mental model
  survives. Meson's two-machine model makes the third environment
  awkward.
- Embedded ecosystem gravity: Zephyr, nRF Connect, Pico SDK, ESP-IDF —
  contributors and vendor HALs assume CMake; Kconfig-plus-CMake is a
  proven pairing (Zephyr's architecture, which this tree already
  resembles).
- CTest maps naturally onto the 287 clar test binaries; clangd and IDEs
  get `compile_commands.json` for free.
- Ninja as the execution layer is a portability asset: the build graph
  is a plain text format that a future in-browser firmware build can
  execute directly with the wasm toolchain, without porting CMake
  itself to the browser.

**Codegen stays Python.** Every generator (resources, fonts, timeline,
app registry, endpoints table, applib malloc, linker snippets, loghash,
bundling) becomes/remains a standalone CLI invoked from
`add_custom_command`. Most already are; the waf-coupled ones are thin
wrappers around portable logic.

**`pbl` remains the UX.** `./pbl configure|build|test|bundle` keeps
working; it drives CMake instead of waf and exports a JSON config
contract to replace today's habit of reading waf's `_cache.py`.

**The exported SDK is out of scope initially.** The SDK ships its own
waf (42K vendored LOC + 2,815 LOC of `sdk/waftools`) and every
third-party app has a `wscript`. Breaking that is an ecosystem decision,
not a build refactor. The firmware migration is structured so the SDK
export keeps producing today's waf-based SDK unchanged; a modern SDK
build is a separate future project (the in-browser SDK already
demonstrates a waf-free app build).

## 4. Migration plan

Phases are ordered so that each lands independently, is verifiable, and
never breaks the gcc build. Phases 1–2 (compiler) and 3 (build system)
are separable tracks; 1–2 unlock browser/mobile compilation regardless
of what drives the build.

### Phase 0 — source hygiene (no build change; safe under gcc)

1. Replace the 36 named-register sites with
   `__builtin_return_address(0)` / explicit SP asm.
2. Re-key the nine `#ifndef __clang__` layout asserts on `UNITTEST`.
3. Drop stale `#ifndef __clang__` guards that disable useful attributes
   (`format(printf)` on `app_log`).
4. Delete the no-op `DISCARD : { libgcc.a(*) }` linker-script sections.

### Phase 1 — clang firmware under waf (milestone: asterix boots)

1. Rewrite clang toolchain discovery in `tools/waf/pebble_arm_gcc.py`
   (`--target=arm-none-eabi` + sysroot; kill the 2013 paths).
2. Gate `-fvar-tracking-assignments` off for clang; re-enable
   `-Wall -Wextra -Werror` for clang and burn down the warning wave
   (swap gcc-only `-Wno-*` pragmas for clang equivalents).
3. Replace `-specs=` in `tools/waf/libc.py` with explicit
   `-isystem/-L/-lc` (+ picolibc's linker script); optionally build
   picolibc with clang.
4. Fix `ldscript.py`'s `CC_NAME == "gcc"` early-return so the generated
   linker script is used.
5. Keep GNU ld and `-lgcc`. Add per-file `-O3`/`-O2` for the three
   graphics files. `AR` → `llvm-ar` when LTO is enabled.
6. CI: add a clang-asterix job next to the gcc jobs; boot-test both in
   QEMU.

### Phase 2 — pure-LLVM link and size parity

1. Switch to LLD: add `--entry=Reset_Handler`, replace
   `--require-defined` with `-u`, adapt the map-file consumer
   (`tools/analyze_fw_static_memory_usage.py`) to LLD's map format.
2. Optionally swap libgcc for compiler-rt builtins (required only for
   ThinLTO and for a gcc-free browser/iOS toolchain).
3. Measure flash deltas per board; restructure the `NEW_LOG_HASH`
   address expression if the ~18 KB regression matters on the tightest
   boards; then evaluate ThinLTO against today's gcc LTO.

### Phase 3 — CMake replaces waf for the firmware

Gate for every step: **artifact parity**. A comparison harness builds
both systems and diffs `system_resources.pbpack`, `pebbleos.{bin,hex}`
(pblboot header included), the `.pbz` bundle contents, QEMU flash
images, and the loghash dict. Generated `.auto.*` sources need only
semantic equivalence.

1. Skeleton: toolchain files (ARM clang, host clang, ARM PIE),
   kconfiglib integration (reuse `tools/waf/kconfig.py` minus the waf
   shim), board selection, `pbl` driving three build trees.
2. Mechanically translate the ~270 declarative `wscript_build` files
   (source lists + static libs); this is scriptable. Cross-directory
   accumulators (`FW_APPS`/`SERVICES`/`DRIVERS`) become CMake global
   properties consumed at link time.
3. Port the custom steps as CLIs: resource pipeline (JSON layer merge →
   `.reso` → resball → pbpack + eight generated outputs), linker-script
   snippet aggregation + preprocessing, firmware image/bundle chain,
   i18n, per-target codegen (app registry, endpoints, applib malloc),
   loghash post-link reduction.
4. Stored apps: an `ExternalProject`-style sub-build against the
   SDK tree generated earlier in the same build, mirroring today's
   `stored_apps` env.
5. Unit tests: rebuild the clar harness generation on CTest; port
   `pebble_test.py` semantics (per-test sources/stubs/fakes/overrides,
   platform matrix). This is the largest single piece and lands last.
6. Run both build systems in CI until parity has held for a release
   cycle; then retire waf for the firmware. SDK export continues to
   emit the waf-based SDK from the new build.

### Phase 4 — obelix (SF32LB52)

Separate milestone gated on: `-mcpu=star-mc1` (substitute
`cortex-m33` and validate DSP/FP codegen against the SiFli HAL, or wait
for LLVM support), warning cleanup in the vendor HAL, and the prebuilt
gcc-compiled `third_party/nonfree` blobs (AAPCS-compatible with a
clang link, but pin the link to non-LTO across that boundary).

## 5. Risks

| Risk | Mitigation |
|---|---|
| Silent codegen differences (the named-register class) | Phase 0 fixes; `-Werror` re-enabled under clang; QEMU boot + unit tests on every CI build |
| SDK ABI table ordering (`pebble.auto.c`) breaks silently | Generator is unchanged Python; parity harness diffs the emitted table |
| Flash size regressions on tight boards | Per-board size gates already exist; loghash macro fix recovers the known 18 KB |
| Packaging byte-drift (pblboot header, pbpack, pbz) | Same Python writers on both sides; parity diff is the gate |
| Two build systems during transition | Time-boxed by the parity gate; CI runs both so drift is caught same-day |
| Vendor HAL / nonfree blobs (obelix) | Quarantined to Phase 4; every other board ships first |
| Test-harness port stalls the migration | Firmware can switch while tests stay on waf temporarily; harness is last in Phase 3, not a blocker for Phases 1–2 |

## 6. Effort summary

| Track | Estimate |
|---|---|
| Phase 0 hygiene | days |
| Phase 1 clang boots asterix under waf | 1–2 weeks |
| Phase 2 LLD/compiler-rt + size parity | 1–2 weeks |
| Phase 3 CMake replaces waf (with parity harness) | 6–10 weeks, mostly mechanical; test harness is half of it |
| Phase 4 obelix | 2+ weeks, partly blocked on LLVM/star-mc1 |

Phases 1–2 alone deliver the strategic payoff — a firmware that builds
with a toolchain that runs on Linux, macOS, in the browser, and inside
an iOS/Android app.
