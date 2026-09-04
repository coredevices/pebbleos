# Build system

The firmware is built with [CMake](https://cmake.org), following a model
borrowed from Zephyr. Ninja is the generator the project uses:

```shell
pbl configure --board asterix
pbl build
```

`pbl configure` runs CMake with `-GNinja` into `build/`, and passes
anything else through, so Kconfig symbols are overridden the usual way:

```shell
pbl configure --board obelix@pvt --variant prf -DCONFIG_RELEASE=y
```

`pbl` itself is documented in {doc}`pbl`. Plain CMake works just as well;
it only saves the typing:

```shell
cmake -B build -GNinja -DBOARD=asterix
cmake --build build
```

The unit tests are a CMake project of their own -- they build for the host,
not for the watch -- out of their own build directory; see {doc}`testing`.

## The library model

Each directory that contributes code declares a library and feeds it with
source files:

```cmake
pbl_library()
pbl_library_sources(service.c util.c)
```

- Every declared library ends up in the single firmware image, so nothing
  has to list them. Dependencies between firmware libraries do not need
  declaring either: they resolve at the final link.
- The library is named after its directory
  (`src/fw/services/timeline` → `src__fw__services__timeline`). Give it a
  name of its own with `pbl_library_named()` when something refers to it,
  or when one directory builds more than one library.
- The target is only created once the library gets its first source file,
  so a directory whose sources are all configured out contributes nothing.

`pbl_library_kind(stlib)` builds a static library the linker pulls objects
from on demand, instead of the default `objects`, whose every object file
is linked. Use it for third-party code that ships more than the firmware
references.

Private compile settings use the matching per-library calls:

```cmake
pbl_library_include_directories(.)
pbl_library_compile_definitions(HAVE_CONFIG_H)
pbl_library_compile_options(-Wno-sign-compare)
```

Conditional pieces key off the Kconfig symbols, which CMake sees as
ordinary variables:

```cmake
pbl_library_sources_ifdef(CONFIG_ACCEL_LSM6DSO lsm6dso/lsm6dso.c)
pbl_add_subdirectory_ifdef(CONFIG_MIC mic)
```

## Exposing headers

Register the directory the headers live in:

```cmake
pbl_include_directories(include)
pbl_compile_definitions(FOO_ENABLED=1)
```

Paths are relative to the calling `CMakeLists.txt`, and the matching build
directory is added too, so generated headers are found next to their
hand-written neighbours.

Headers are global; there is no per-consumer opt-in. A library that is not
always needed is gated behind a Kconfig symbol instead, as Zephyr does:
its directory is added with `pbl_add_subdirectory_ifdef()`, and consumers
`select` (or `imply`) the symbol in their own Kconfig, e.g. `BT_FW_NIMBLE`
selects `MBEDTLS` and `SERVICE_VOICE` selects `SPEEX`.

Prebuilt vendor archives are registered with
`pbl_library_import(name path...)`, which looks for `lib<name>.a`.

## Linker script fragments

Libraries that need linker script additions register them against one of
the master script's hook points:

```cmake
pbl_linker_sources(memory linker.ld)
```

See `cmake/modules/linker.cmake` for the available hooks.

## Generated sources

Generated headers are produced by custom commands and collected under the
`pbl_generated_headers` target, which every library waits for. Generators
live in `tools/cmake/`:

- `kconfig.py` — runs Kconfig, writes `.config`, `autoconf.h` and the
  CMake variables the build reads.
- `resources.py` — resolves the board's resource maps into build rules, in
  batches so that a few hundred resources do not cost a few hundred
  interpreter start-ups, and runs the resource, pbpack and
  resource-table steps.
- `generate.py` — the remaining source generators (version header, app
  registry, protocol endpoint table, applib allocator tables, log string
  hashing, stored apps).
- `firmware.py` — image steps: size checks, bundling, QEMU flash images.

`autoconf.h` is force-included into every compilation, so Kconfig symbols
reach the sources, the headers and the linker script alike.

## Directory layout

```
CMakeLists.txt              the firmware image: what is built and linked
cmake/toolchain/            the bare-metal ARM toolchain
cmake/modules/              the build model, split by concern
tools/cmake/                the generators the build shells out to
```

The one build system left besides CMake is the waf under `sdk/`, which is
not used to build anything here: it is packaged into the SDK for app
developers to build their own projects with. See {doc}`sdk_export`.
