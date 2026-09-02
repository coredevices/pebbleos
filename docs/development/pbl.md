# The `pbl` CLI

`pbl` is the developer command line for this repository: it configures and
builds the firmware, puts the result on a device or an emulator, and hosts
the operational commands around that.

It is a Python package, installed along with the rest of the project's
dependencies:

```shell
pip install -r requirements.txt
```

After that `pbl` is on the `PATH` whenever the virtual environment is
active. Run it from anywhere inside the checkout:

```shell
pbl configure --board asterix
pbl build
pbl flash
```

`pbl --help` lists every command, grouped; `pbl <command> --help` shows one
command's own options.

## Shared options

Two options are accepted by every command, before or after the command
name:

- `-b, --build-dir DIR` — use a different build directory. It defaults to
  the one `pbl.yml` names (`build/`, or `build-test/` for `pbl test`), and
  `PBL_BUILD_DIR` overrides that in turn.
- `--dry-run` — print the commands that would run instead of running them.

## Where its knowledge comes from

`pbl` holds no facts about the checkout of its own:

- The **workspace** is wherever `pbl.yml` is, found by walking up from the
  current directory. That file also names the build, test and language
  directories.
- The **build** is read back from its own byproducts: `.config` for every
  `CONFIG_` symbol, `CMakeCache.txt` for the board, the variant and the
  project name the artifacts are named after. Configuring is therefore the
  only way to tell `pbl` about a build, and reconfiguring is enough to
  change its mind.
- The **board** contributes the rest through `boards/<board>/<board>.yml`:
  its revisions, the binary runners it supports, and the SDL decorations
  the emulator can draw around its screen.

## Structure

The package lives in `tools/libs/pbl-cli/`, and is a small core plus one
module per command, in the shape of Zephyr's `west`:

```
pbl/app/main.py    finds the workspace, assembles the commands, dispatches
pbl/command.py     PblCommand, the base class every command derives from
pbl/workspace.py   the checkout: pbl.yml, the layout, the extensions
pbl/build.py       a build directory and what can be read back from it
pbl/runners.py     the binary runners (openocd, sftool, nrfutil)
pbl/commands/      the commands themselves
```

### Adding a command

Drop a module in `pbl/commands/` with a `PblCommand` subclass. Commands are
discovered by scanning the package, so there is no registry to update;
classes whose name starts with an underscore are treated as shared bases
rather than commands.

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

`self.build_dir()` returns the configured build, failing with a useful
message when there is none; `self.run_cmd()` and `self.check_cmd()` run
things from the workspace root and honor `--dry-run`; raising
`CommandError` is how a command fails. `group` picks the section of the
top-level help the command is listed under.

### Extension commands

A command that should not live in the package is declared in `pbl.yml`
instead. It is only imported when it is the command being run, and is
listed under "extension commands" rather than in one of the groups:

```yaml
commands:
  - file: tools/pbl_commands/size.py
    commands:
      - name: size
        class: Size
        help: Report the firmware's size
```

## Binary runners

Flashing and debugging a real device go through a runner, the same
abstraction Zephyr uses. A board lists the ones it supports in its
manifest, first one first:

```yaml
runners:
  - openocd
  - nrfutil
```

`pbl flash`, `reset`, `run`, `bork`, `openocd` and `debug` all dispatch to
it, and `-r/--runner` picks a different one. Each runner contributes its
own options (e.g. `--tty` for sftool), so they show up on all of those
commands. New runners are modules in `pbl/runners/`, registered in that
package's `RUNNERS` table.
