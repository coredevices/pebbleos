#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

"""Print the PATH an installed PebbleOS SDK wants, if there is one.

The SDK ships the toolchain the firmware is expected to be built with
(picolibc included), so the CMake build looks for its binaries first.
"""

import os
import sys

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, REPO_ROOT)

import contextlib
import io

from tools.pebble_sdk_locator import activate_sdk

# The locator announces what it found on stdout; keep that off the PATH
# line CMake reads back and report it as a message instead.
notes = io.StringIO()
with contextlib.redirect_stdout(notes):
    activate_sdk(REPO_ROOT)
print(notes.getvalue().strip(), file=sys.stderr)
print(os.environ["PATH"])
