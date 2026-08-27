# SPDX-FileCopyrightText: 2024 Google LLC
# SPDX-License-Identifier: Apache-2.0

from .. import PebbleCommander, exceptions, parsers


@PebbleCommander.command()
def reset(cmdr):
    """Reset the device."""
    cmdr.send_prompt_command("reset")


@PebbleCommander.command()
def crash(cmdr):
    """Crash the device."""
    cmdr.send_prompt_command("crash")


@PebbleCommander.command()
def factory_reset(cmdr, fast=False):
    """Perform a factory reset.

    If `fast` is specified as true or "fast", do a fast factory reset.
    """
    if parsers.str2bool(fast, also_true=["fast"]):
        fast = " fast"
    else:
        fast = ""

    ret = cmdr.send_prompt_command(f"factory reset{fast}")
    if ret:
        raise exceptions.PromptResponseError(ret)
