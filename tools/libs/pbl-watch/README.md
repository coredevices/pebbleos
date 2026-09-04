# pbl-watch

Internal developer commands for interacting with Pebble watches, built on
`pebble-tool` and `libpebble2` (e.g. `install_firmware`, `install_lang`,
`coredump`, `flash_logs`). Installs a `pbl-watch` entry point:

    pip install ./tools/libs/pbl-watch

It is separate from the `pbl` developer CLI (`tools/libs/pbl-cli`), which
drives the build, the emulator and the binary runners.

To add a command, create a file in `pblwatch/commands/` with a class
inheriting from `BaseCommand` (or `PebbleCommand` if it connects to a
watch): the docstring becomes the help text, the `command` field names the
command, and `__call__` runs it. Examples can be found
[in pebble-tool](https://github.com/pebble/pebble-tool/tree/master/pebble_tool/commands).
