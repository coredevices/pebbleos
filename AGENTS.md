# PebbleOS

PebbleOS is the operating system running on Pebble smartwatches.

## Organization

- `docs`: project documentation
- `resources`: firmware resources (icons, fonts, etc.)
- `sdk`: application SDK generation files
- `src`: firmware source
- `subsys`: OS subsystems, e.g. logging
- `tests`: tests
- `third_party`: third-party code in git submodules, also includes glue code
- `tools`: a variety of tools or scripts used in multiple areas, from build
  system, tests, etc.
- `tools/libs`: Python packages used in multiple areas, e.g. log dehashing,
  console, etc.
- `tools/cmake`: generators the CMake builds shell out to
- `tools/waf`: scripts used by waf, which still packages the SDK

## Documentation

Contributor documentation lives in `docs/` (published at
https://pebbleos-core.readthedocs.io). Prefer pointing to or extending those
pages over duplicating knowledge here: `docs/development/contributing.md`
(DCO, commit and AI-usage rules), `docs/development/sdk_export.md` (SDK
export machinery), `docs/development/qemu.md` (emulator workflow).

## Code style

- clang-format for C code
- ruff for Python code
- Keep code comments short and concise. Extended descriptions can be kept in
  the Git commit message.
- Do not put references to issues in the code, only add those to the Git commit message.

## Logging

- `PBL_LOG_WRN` / `PBL_LOG_ERR` are for warnings and errors — use them as
  the names suggest.
- Default to `PBL_LOG_DBG` for routine lifecycle / state-transition logs.
  Reserve `PBL_LOG_INFO` for events that genuinely warrant attention in a
  default-level log capture; if a code path can fire repeatedly under
  normal use (e.g. play/pause spam, frequent state changes), it must not
  log at INFO.

## Firmware development

- Configure: `./pbl configure --board BOARD_NAME`

  - Board names can be obtained from `./pbl --help`
  - `-DCONFIG_RELEASE=y` enables release mode
  - `-DCONFIG_MFG=y` enables manufacturing mode
  - `--variant=normal|prf` selects build variant (default: normal)

- Build firmware: `./pbl build`
- Run tests: `./pbl test`

## Adding a new SDK function

Exposing a function to third-party apps requires three coordinated changes
(applib wrapper + syscall, `exported_symbols.json` registration, SDK
revision bump) — the firmware build alone won't surface it to apps. Follow
`docs/development/sdk_export.md` whenever an `applib/` function should
become callable from user apps.

## Git rules

Main rules:

- Commit using `-s` git option, so commits have `Signed-Off-By`
- Always indicate commit is co-authored by the current AI model
- Commit in small chunks, trying to preserve bisectability
- Commit format is `area: short description`, with longer description in the
  body if necessary
- Run `gitlint` on every commit to verify rules are followed

Others:

- If fixing Linear or GitHub issues, include in the commit body a line with
  `Fixes XXX`, where XXX is the issue number.
