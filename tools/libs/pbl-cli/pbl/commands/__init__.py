# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

"""The commands shipped with the CLI.

Every module in this package that defines :class:`~pbl.command.PblCommand`
subclasses contributes them; adding a command is adding a file here.
Names starting with an underscore are shared bases, not commands.
Out-of-tree commands are declared in the workspace's ``pbl.yml`` instead.
"""

import importlib
import inspect
import pkgutil

from pbl.command import PblCommand


def builtin_commands():
    """Discover and instantiate every built-in command."""
    commands = {}
    for info in pkgutil.iter_modules(__path__, __name__ + "."):
        if info.name.rpartition(".")[2].startswith("_"):
            continue
        module = importlib.import_module(info.name)
        for obj in vars(module).values():
            if (
                inspect.isclass(obj)
                and issubclass(obj, PblCommand)
                and obj.__module__ == module.__name__
                and not obj.__name__.startswith("_")
                and not inspect.isabstract(obj)
            ):
                command = obj()
                commands[command.name] = command
    return commands
