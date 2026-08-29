# Exposing functions to the SDK

Third-party apps do not link against the firmware directly: the app SDK is
generated from the firmware sources at build time. When you expose a new
function to apps (i.e. anything declared in an `applib/` header that user apps
can call), three things must change together — the firmware build alone will
not surface it to apps:

1. **Implement the applib wrapper and syscall** — add the function to the
   appropriate `src/fw/applib/.../<area>.c/.h`, declare the `sys_*` syscall in
   `src/fw/syscall/syscall.h`, and define it with `DEFINE_SYSCALL` (from
   `src/fw/syscall/syscall_internal.h`), either in the matching
   `src/fw/syscall/<area>_syscalls.c` or alongside the implementation it
   wraps.
2. **Register the symbol** in
   `tools/generate_native_sdk/exported_symbols.json` under the matching
   group, with an `addedRevision` matching the new SDK revision.
3. **Bump the SDK revision** in
   `src/fw/process_management/pebble_process_info.h`: increment
   `PROCESS_INFO_CURRENT_SDK_VERSION_MINOR` and add a comment line above the
   `#define` following the existing pattern, e.g. `// sdk.major:0x5
.minor:0x66 -- <description> (rev 105)`. The `rev` number in the comment
   must match `addedRevision` from step 2.

Forgetting steps 2 or 3 means the function compiles into the firmware but is
invisible to the app SDK build, so third-party apps can't link against it.

```{warning}
It is **not** possible to add publicly exposed functions to an already
released firmware/SDK combination. The generated `pebble.auto.c`
function-pointer table must be compiled into the firmware the SDK targets:
an app built against a newer SDK calls a trampoline that indexes past the
end of an older firmware's table and crashes. New exports always ship as a
new firmware plus a new SDK build.
```

## How the SDK generator works

The generator (`tools/generate_native_sdk/generate_pebble_native_sdk_files.py`)
runs automatically as part of the firmware build (normal variant only — PRF
and test builds skip it). It exports the white-listed functions, `typedef`s
and `#define`s from the firmware tree and produces the files needed to build
native watchapps, all under `build/`:

- `build/sdk/<platform>/include/pebble.h` — typedefs, defines and function
  prototypes for apps (plus `pebble_worker.h` for background workers and a
  few version/fonts headers)
- `build/sdk/<platform>/lib/libpebble.a` — static library containing
  trampolines that call the exported functions in flash
- `build/src/fw/pebble.auto.c` — `g_pbl_system_tbl`, the table of function
  pointers the trampolines use to find an exported function's address;
  compiled into the firmware image

The rest of the SDK distribution is still assembled by waf, which the
firmware build itself no longer uses. `sdk/wscript_build` copies the common
files from `sdk/` into `build-test/sdk/common/` (including the app project
templates under `sdk/defaults/`) and bundles the SDK waftools into the
`waf` binary app developers use to build their apps. Run it with:

```shell
./pbl waf configure --board $BOARD
./pbl waf sdk
```

## `exported_symbols.json` format

```json
{
  "revision": "<exported symbols revision number>",
  "version": "x.x",
  "files": ["<files to parse>"],
  "exports": ["<symbols to export>"]
}
```

Each exported symbol has a `type` of `function`, `define`, `type`,
`forward_struct`, or `group`:

```json
{
  "type": "function",
  "name": "<symbol name>",
  "sortName": "<sort order>",
  "addedRevision": "<revision number>"
}
```

A `group` nests further `exports` under a `name`. Functions support
additional flags (`internal`, `removed`, `deprecated`, `appOnly`,
`workerOnly`, `implName`, `skipDefinition`) — see existing entries and
`tools/generate_native_sdk/exports.py` for their meaning.

Notes:

- Functions are sorted by `addedRevision`, then alphabetically (by `sortName`
  if present, else `name`) within a revision. This ordering is the ABI — it
  is why new functions must use a new revision: it guarantees new firmware
  stays backwards compatible with apps compiled against an older
  `libpebble.a`.
- `types` are emitted in the order listed; put typedefs after the typedefs
  they depend on (`includeAfter` is the escape hatch for ordering
  exceptions).
- The generator errors out on exports it cannot find in the parsed headers
  and on inconsistent revision numbers, but it does not verify that the
  resulting `pebble.h` compiles — review its output.
- The comment ledger above `PROCESS_INFO_CURRENT_SDK_VERSION_MINOR` is the
  only mapping between SDK revisions and version minors: no formula relates
  them (the minor once jumped `0x19` → `0x20` between revs 35 and 36, and a
  few minors and revs are skipped or doubled up). Treat the ledger as
  append-only history.
