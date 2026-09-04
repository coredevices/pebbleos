# Running and writing tests

Unit tests live under `tests/` and run on the host (not on device or QEMU),
using a vendored copy of the [clar](https://github.com/clar-test/clar) test
framework in `tools/clar/`. Code under test is compiled for the host together
with fakes and stubs that replace hardware and OS dependencies.

## Running tests

The tests are a CMake project of their own, separate from the firmware
build because they build for the host rather than for the watch. `pbl
test` configures `build-test/`, builds every test and runs them under
ctest:

```shell
pbl test
```

Anything `pbl test` does not recognise is passed straight to ctest, so
its selection and reporting options are all available:

- `-R REGEX`: run the tests whose name matches, e.g. `pbl test -R animation`
- `-L LABEL`: run by label; every test is labelled with its directory
  (`tests/fw/ui`) and its platform (`obelix`, `gabbro`, `asterix`)
- `-N`: list the tests that would run instead of running them
- `--rerun-failed`: re-run only what failed last time
- `-j N`: run N tests at once (the default is one per core)
- `--stop-on-failure`: stop at the first failure instead of running on

and `pbl test`'s own options:

- `-C` / `--coverage`: collect coverage and generate an lcov HTML report at
  `build-test/lcov-html/index.html`
- `--no-images`: skip the image fixtures, which only some tests need and
  which are the slow part of a clean build
- `--build-only`: build the test binaries without running them

Each test is a standalone binary under `build-test/`, so a failing one can
be run directly or under a debugger:

```shell
gdb build-test/fw/ui/test_animation/runme
```

Results are written as JUnit XML to `build-test/junit.xml`, and a
`compile_commands.json` for the tests lands in `build-test/`. The `Test`
GitHub Actions workflow (`.github/workflows/test.yml`) runs the whole suite
on pull requests that touch source, test or tooling paths.

Test binaries link against the DUMA memory checker by default, so
out-of-bounds heap accesses abort the test immediately.

## Writing a test

A test is a C file named `test_<suite>.c` under a `tests/` subdirectory,
plus a `pbl_clar_test()` entry in that directory's `CMakeLists.txt`. The clar
generator (`tools/clar/clar.py`) scans the file for functions named
`test_<suite>__<case>` — no manual registration is needed:

```c
#include "clar.h"

#include "pbl/util/crc32.h"

void test_crc32__initialize(void) {
  // optional: runs before each test case
}

void test_crc32__cleanup(void) {
  // optional: runs after each test case
}

void test_crc32__empty(void) {
  cl_assert_equal_i(crc32(0, NULL, 0), 0);
}
```

The available assertion macros (`cl_assert`, `cl_assert_equal_i`,
`cl_assert_equal_s`, `cl_must_pass`, ...) are defined in
`tests/test_includes/clar_asserts.h`.

The `CMakeLists.txt` entry declares which product sources, fakes and test
files to compile:

```cmake
pbl_clar_test(test_crc32)
```

`crc32.c` lives in `libutil`, which `pbl_clar_test()` always links; code
that is not part of the always-linked libraries — and any fakes from
`tests/fakes/` — is listed explicitly under `SOURCES`, as paths relative to
the repository root:

```cmake
pbl_clar_test(test_pfs
  SOURCES
    src/fw/services/filesystem/pfs.c
    tests/fakes/fake_rtc.c
  OVERRIDES dummy_board
)
```

`pbl_clar_test()` (defined in `cmake/modules/pbl_test.cmake`) builds one
binary per test file/platform combination and registers it with ctest. Its
arguments:

- `SOURCE`: the test file, when it is not `<name>.c` next to the
  `CMakeLists.txt`
- `SOURCES`: code under test plus any fakes from `tests/fakes/`;
  `@PLATFORM@` and `@BITDEPTH@` in a path expand per platform
- `PLATFORMS`: build the test once per listed platform (e.g.
  `obelix gabbro`); the default is a single generic platform
- `DEFINES`, `INCLUDES`, `LIBS`: extra compile defines, include paths and
  link libraries
- `OVERRIDES`: opt-in header override directories (see below)
- `DEPENDS`: targets to build first, for a test that needs a generated
  header
- `TEST_IMAGES`: the test renders against the image fixtures

A source needed by several tests is compiled once and shared between them
whenever they agree on every compile flag, so adding a test that matches an
existing one costs a single translation unit.

## Fakes, stubs and header overrides

Three mechanisms replace dependencies of the code under test:

- **Fakes** (`tests/fakes/`): functional host implementations, e.g.
  `fake_rtc.c`, `fake_spi_flash.c`. Compile them in via `sources_ant_glob`.
- **Stubs** (`tests/stubs/`): headers with no-op implementations, for
  dependencies the test never exercises.
- **Header overrides** (`tests/overrides/`): directories mirroring the
  source tree whose headers shadow the real ones on the include path.
  `tests/overrides/default/` is always active for every test; other
  directories are opted into per test via `override_includes`. Read
  `tests/overrides/README.md` and `tests/overrides/default/README.md`
  before adding one — in particular, avoid adding to `default/`.

## Broken tests

`pbl_test_broken()` in `tests/CMakeLists.txt` lists tests that are not
built. Their `pbl_clar_test()` declarations stay where they are, so what
the test needs is still on record when someone picks it back up.

## JavaScript tests

The JavaScript sources in `sdk/include/` have their own mocha-based tests in
`sdk/tests/include/`; they are not run by `pbl test`. See
`sdk/tests/include/README.md` for how to run them with `npm test`.
