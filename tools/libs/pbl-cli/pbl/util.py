# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

"""Terminal output helpers shared by the CLI and its commands."""

import os
import sys

_CODES = {"red": "31", "green": "32", "yellow": "33", "cyan": "36"}


def _colorize(color, msg):
    code = _CODES.get(color)
    if code is None:
        return msg
    return f"\033[{code}m{msg}\033[0m"


def _use_color(stream):
    if os.environ.get("NO_COLOR"):
        return False
    return stream.isatty()


def _emit(stream, color, args):
    msg = " ".join(str(a) for a in args)
    if color and _use_color(stream):
        msg = _colorize(color, msg)
    print(msg, file=stream)


def inf(*args, color="cyan"):
    """Progress and status, on stdout."""
    _emit(sys.stdout, color, args)


def wrn(*args):
    """Warning, on stderr."""
    _emit(sys.stderr, "yellow", ("warning:",) + args)


def err(*args):
    """Error, on stderr."""
    _emit(sys.stderr, "red", ("error:",) + args)
