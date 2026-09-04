# SPDX-FileCopyrightText: 2024 Google LLC
# SPDX-License-Identifier: Apache-2.0

from .. import PebbleCommander, exceptions


@PebbleCommander.command()
def set_time(cmdr, new_time):
    """Set the time.

    `new_time` should be in epoch seconds.
    """
    new_time = int(str(new_time), 0)
    if new_time < 1262304000:
        raise exceptions.ParameterError("time must be later than 2010-01-01")
    ret = cmdr.send_prompt_command(f"set time {new_time}")
    if not ret[0].startswith("Time is now"):
        raise exceptions.PromptResponseError(ret)
    return ret


@PebbleCommander.command()
def timezone_clear(cmdr):
    """Clear timezone settings."""
    ret = cmdr.send_prompt_command("timezone clear")
    if ret:
        raise exceptions.PromptResponseError(ret)
