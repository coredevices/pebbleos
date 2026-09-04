# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

"""Errors a command may raise; the CLI turns them into an exit status."""


class CommandError(RuntimeError):
    """A command failed. The message, if any, is printed as an error."""

    def __init__(self, msg=None, returncode=1):
        super().__init__(msg)
        self.msg = msg
        self.returncode = returncode


class CommandContextError(CommandError):
    """The command cannot run here: nothing is configured, no such board, ..."""
