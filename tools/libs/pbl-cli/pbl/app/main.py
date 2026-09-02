# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

"""The ``pbl`` entry point: find the workspace, assemble the commands and
dispatch one of them."""

import argparse
import signal
import sys

from pbl import __version__, util
from pbl.command import GROUPS, global_options
from pbl.commands import builtin_commands
from pbl.errors import CommandError
from pbl.workspace import Workspace, WorkspaceError

DESCRIPTION = """\
The PebbleOS developer CLI: configure and build the firmware, put it on a
device or an emulator, and everything around that.
"""


def _order(commands, extensions):
    """The help text's sections, in the order GROUPS lists them. A command
    whose group is not one of those is listed as an 'other' command."""
    known = set(GROUPS)
    for group in GROUPS:
        members = [
            c
            for c in commands
            if c.group == group or (group == "other" and c.group not in known)
        ]
        if members:
            yield group, sorted(members, key=lambda c: c.name)
    if extensions:
        yield "extension", sorted(extensions, key=lambda e: e.name)


def _epilog(commands, extensions):
    width = max((len(c.name) for c in list(commands) + list(extensions)), default=0)
    lines = []
    for group, members in _order(commands, extensions):
        lines.append(f"\n{group} commands:")
        lines += [f"  {c.name:<{width}}  {c.help}" for c in members]
    lines.append("\nRun 'pbl <command> --help' for a command's own options.")
    return "\n".join(lines)


def _peek_command(argv):
    """The command name in ``argv``, ignoring the shared options."""
    known = argparse.ArgumentParser(add_help=False, parents=[global_options()])
    _, rest = known.parse_known_args(argv)
    for arg in rest:
        if not arg.startswith("-"):
            return arg
    return None


def _load_commands(workspace, selected):
    """Every command the CLI offers. Extensions are only imported when they
    are the one being run; the rest just appear in the help."""
    commands = builtin_commands()
    if workspace is None:
        return commands, []

    extensions = []
    for spec in workspace.extensions():
        if spec.name in commands:
            util.wrn(f"extension command {spec.name} shadows a built-in one, ignoring")
            continue
        extensions.append(spec)
        if spec.name == selected:
            commands[spec.name] = spec.load()
    return commands, extensions


def _build_parser(commands, extensions, workspace):
    declared = {e.name for e in extensions}
    builtins = [c for c in commands.values() if c.name not in declared]
    parser = argparse.ArgumentParser(
        prog="pbl",
        description=DESCRIPTION,
        epilog=_epilog(builtins, extensions),
        formatter_class=argparse.RawDescriptionHelpFormatter,
        parents=[global_options()],
    )
    parser.add_argument("--version", action="version", version=f"pbl {__version__}")
    parser_adder = parser.add_subparsers(dest="command", metavar="<command>")

    for command in commands.values():
        command.workspace = workspace
        command.add_parser(parser_adder)
    return parser


def _run(argv):
    # Help and version still work outside a checkout; commands do not.
    workspace = None
    reason = None
    try:
        workspace = Workspace.find()
        workspace.activate()
    except WorkspaceError as e:
        reason = e

    commands, extensions = _load_commands(workspace, _peek_command(argv))
    parser = _build_parser(commands, extensions, workspace)
    args, unknown = parser.parse_known_args(argv)

    if not args.command:
        parser.print_help()
        return 1
    if workspace is None:
        raise reason

    return commands[args.command].run(args, unknown, workspace)


def main(argv=None):
    argv = list(sys.argv[1:] if argv is None else argv)
    try:
        return _run(argv)
    except CommandError as e:
        if e.msg:
            util.err(e.msg)
        return e.returncode
    except WorkspaceError as e:
        util.err(str(e))
        return 1
    except KeyboardInterrupt:
        # Ctrl+C reached the whole process group, so whatever was running has
        # already stopped; report it the way a shell expects instead of
        # unwinding through the wait() it interrupted.
        return 128 + signal.SIGINT


if __name__ == "__main__":
    sys.exit(main())
