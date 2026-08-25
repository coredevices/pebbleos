# Configuration Options

When configuring the build (`./pbl configure ...`) there are several options you can enable or tune.
Below you will find a list of the most relevant ones.

## Choosing your target

There are a number of target boards to choose from when building PebbleOS. You can do so by
using the (`--board`) flag followed by:

:`asterix`: (Core Devices) Pebble 2 Duo
:`obelix@bb2`, `obelix@dvt`, `obelix@pvt`: (Core Devices) Pebble Time 2
:`getafix@evt`, `getafix@dvt`, `getafix@dvt2`: (Core Devices) Pebble Round 2
:`qemu_emery`, `qemu_flint`, `qemu_gabbro`: dedicated QEMU targets (see {doc}`qemu`)

Keep in mind that some targets may not currently compile as-is.

## Variant

:`--variant`:
Build variant, `normal` (main firmware) or `prf` (recovery firmware).

## Release build

:`-DCONFIG_RELEASE=y`:
Build a release-mode firmware. Strips debug aids, enables shipping
defaults, and reduces battery usage compared to a debug build. Pass
this to `./pbl configure`.

## Main features

:`-DCONFIG_MODDABLE_XS=y` / `-DCONFIG_MODDABLE_XS=n`:
Force the Moddable SDK's XS JavaScript engine on or off, overriding
the board defconfig. Pass to `./pbl configure`. See {doc}`moddable`.
PRF (recovery) builds always disable the engine regardless of this
value.

## Manufacturing

:`-DCONFIG_MFG=y`:
Enable manufacturing-only functionality in the PRF build.

## Debugging

:`-DCONFIG_NO_WATCHDOG=y`:
Disable watchdog

## Flashing

The `flash`, `run` and `debug` commands talk to a connected device through a
_runner_. Each board declares its supported runners in its board manifest
(e.g. `boards/<board>/<board>.yml`) and the first one is used by default. For
the OpenOCD runner, the probe and target configuration lives in the board's
`support/openocd.cfg`.

:`--runner`:
Override the board's default runner for `flash`/`run`/`debug`.

:`--tty`:
Serial port used by the `sftool` runner.

:`--resources`:
Also flash system resources alongside the firmware (`sftool` runner).

## Logging

:`-DCONFIG_DEFAULT_LOG_LEVEL_<LEVEL>=y`:
Default log level, where `<LEVEL>` is one of `ERROR`, `WARNING`,
`INFO` (default), `DEBUG` or `DEBUG_VERBOSE`.

:`-DCONFIG_LOG_HASHED=n`:
Disable log messages hashing.
This will increase ROM usage, but will not require a dictionary file to decode logs.

These and many more options can also be browsed and changed interactively with
`./pbl menuconfig` after configuring.
