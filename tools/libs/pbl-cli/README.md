# pbl

The PebbleOS developer CLI. It drives the CMake firmware build, puts the
result on a device or an emulator, and hosts the operational commands
around that.

    pip install -e ./tools/libs/pbl-cli   # or: pip install -r requirements.txt
    pbl configure --board asterix
    pbl build

## How it fits together

The package is a small core plus one module per command, in the shape of
Zephyr's `west`:

| | |
| --- | --- |
| `pbl/app/main.py`  | finds the workspace, assembles the commands, dispatches |
| `pbl/command.py`   | `PblCommand`, the base class every command derives from |
| `pbl/workspace.py` | the checkout: located by walking up to `pbl.yml`, which also carries the directory layout and the extension commands |
| `pbl/build.py`     | a build directory, read back from its own `.config` and `CMakeCache.txt` |
| `pbl/runners.py`   | the binary runners (openocd, sftool, nrfutil) |
| `pbl/commands/`    | the commands themselves |

Nothing about the checkout is baked into the CLI: the board, the variant,
the artifact names and every `CONFIG_` symbol come from the build's own
byproducts, and the boards' runners and emulator decorations come from
`boards/<board>/<board>.yml`.

## Adding a command

Drop a module in `pbl/commands/` with a `PblCommand` subclass; it is
discovered by scanning the package, so there is no registry to update.

```python
from pbl.command import PblCommand


class Size(PblCommand):
    group = "build"

    def __init__(self):
        super().__init__("size", "Report the firmware's size")

    def do_add_parser(self, parser_adder):
        parser = self.add_subparser(parser_adder)
        parser.add_argument("--sections", action="store_true")
        return parser

    def do_run(self, args, unknown):
        build = self.build_dir()
        return self.run_cmd(["arm-none-eabi-size", build.elf])
```

`self.build_dir()` returns the configured build (and fails with a useful
message when there is none), `self.run_cmd()` and `self.check_cmd()` run
things from the workspace root and honor `--dry-run`, and raising
`CommandError` is how a command fails.

## Extension commands

A command that should not live in the package goes in `pbl.yml` instead.
It is only imported when it is the command being run, and is listed under
"extension commands" rather than in one of the groups:

```yaml
commands:
  - file: tools/pbl_commands/size.py
    commands:
      - name: size
        class: Size
        help: Report the firmware's size
```
