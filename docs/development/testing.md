# Running and writing tests

Unit tests live under `tests/` and run on the host (not on device or QEMU),
using a vendored copy of the [clar](https://github.com/clar-test/clar) test
framework in `tools/clar/`. Code under test is compiled for the host together
with fakes and stubs that replace hardware and OS dependencies.

## Running tests

Tests are built and run with waf. Configure first (any board works; CI uses
`qemu_gabbro`), then:

```shell
./pbl test
```

Useful options (see `./pbl test --help` for the full list):

- `-M REGEX` / `--match REGEX`: only build/run test files matching the regex,
  e.g. `./pbl test -M test_animation`
- `-L` / `--list_tests`: list the tests in the matched files instead of
  running them
- `-T NAME` / `--test_name NAME`: run a single test case, e.g.
  `./pbl test -M test_animation -T unschedule`
- `-D` / `--debug_test`: run the test under GDB (use with `-M` to select the
  test)
- `-C` / `--coverage`: collect coverage and generate an lcov HTML report at
  `build/test/tests/lcov-html/index.html`
- `--show_output`: print test output while running
- `--no_run`: build the test binaries without running them
- `-k`: keep going after a failing test (used by CI)

Results are also written as JUnit XML to `build/test/junit.xml`. The `Test`
GitHub Actions workflow (`.github/workflows/test.yml`) runs the whole suite
on pull requests that touch source, test or tooling paths.

Test binaries link against the DUMA memory checker by default, so
out-of-bounds heap accesses abort the test immediately.

## Writing a test

A test is a C file named `test_<suite>.c` under a `tests/` subdirectory,
plus a `clar()` entry in that directory's `wscript_build`. The clar
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

The `wscript_build` entry declares which product sources, fakes and test
files to compile:

```python
from tools.waf.pebble_test import clar

clar(ctx, test_sources_ant_glob="test_crc32.c")
```

`crc32.c` lives in `libutil`, which `clar()` always links; code that is not
part of the always-linked libraries — and any fakes from `tests/fakes/` —
is listed explicitly in `sources_ant_glob`, e.g.
`sources_ant_glob="src/fw/services/... tests/fakes/fake_rtc.c"`.

`clar()` (defined in `tools/waf/pebble_test.py`) builds one binary per test
file/platform combination into `build/test/tests/...` and runs it. Commonly
used keyword arguments:

- `sources_ant_glob` / `sources`: code under test plus any fakes from
  `tests/fakes/`
- `test_sources_ant_glob` / `test_sources`: the test file(s)
- `defines`, `test_libs`, `add_includes`: extra compile defines, link
  libraries and include paths
- `override_includes`: opt-in header override directories (see below)
- `platforms`: build the test once per listed platform (e.g.
  `['asterix', 'obelix']`); the default is a single generic platform

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

`BROKEN_TESTS` in `tests/wscript_build` lists test files that are excluded
from the build; matches are reported with a red `Skipping ...` line during
the test run.

## JavaScript tests

The JavaScript sources in `sdk/include/` have their own mocha-based tests in
`sdk/tests/include/`; they are not run by `./pbl test`. See
`sdk/tests/include/README.md` for how to run them with `npm test`.
