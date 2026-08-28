# Build system

PebbleOS is built with [waf](https://waf.io). Each directory that contributes
code has a `wscript_build` file; the firmware is assembled following a model
borrowed from Zephyr, implemented in `tools/waf/pbl_build.py`:

- Every library registers itself with `bld.pbl_library()`. All registered
  libraries are linked into the single firmware image built by
  `bld.pbl_program()`, so no other place needs to list them.
- Include directories and defines registered with
  `bld.pbl_include_directories()` and `bld.pbl_compile_definitions()` are
  visible to every library. They are declared once, next to the headers they
  expose, instead of being requested by each consumer through `use=[...]`.
- Dependencies between firmware libraries do not need to be declared: they
  resolve at the final link.

## Adding a library

A typical `wscript_build` is a single call:

```python
bld.pbl_library(bld.path.ant_glob('*.c'))
```

`pbl_library()` accepts:

- `source`: the source files, relative to the `wscript_build` directory.
- `name`: defaults to the directory path with `__` separators
  (e.g. `subsys__logging`). Only needed when something refers to the library
  by name.
- `kind`: `objects` (default) links every object file; `stlib` builds a
  static library from which the linker only pulls the objects it needs. Use
  `stlib` for third-party code that ships more than the firmware references.
- `use`: extra generators to pull in, for third-party headers that are
  deliberately not global (e.g. `use=['speex_includes']`).
- `includes`, `defines`, `cflags`: private to the library, as in plain waf.

Conditional pieces use the Kconfig symbols in `bld.env`:

```python
bld.pbl_library_ifdef('CONFIG_SOC_NRF52', ['nrf5/qspi.c'], name='driver_qspi')
bld.pbl_recurse_ifdef('CONFIG_MIC', 'mic')
```

## Exposing headers

Register the directory where it lives:

```python
bld.pbl_include_directories('include')
bld.pbl_compile_definitions('FOO_ENABLED=1')
```

Paths are relative to the calling `wscript_build`; the matching build-tree
directory is added too, so generated headers are found. A `Node` can be
passed for build-only directories.

Keep headers global unless their names are likely to collide with firmware
headers (third-party code with generic names such as `list.h` or `os.h`).
Those libraries export their include directories on their own generator and
consumers opt in with `use=`.

## Linker script fragments

Libraries that need linker script additions register them with
`bld.linker_sources(location, 'fragment.ld')`, see `tools/waf/ldscript.py`
for the available hook points.
