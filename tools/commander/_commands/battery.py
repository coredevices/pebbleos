# SPDX-FileCopyrightText: 2024 Google LLC
# SPDX-License-Identifier: Apache-2.0

from .. import PebbleCommander, exceptions, parsers


@PebbleCommander.command()
def battery_force_charge(cmdr, charging=True):
    """Force the device to believe it is or isn't charging."""
    if parsers.str2bool(charging):
        charging = "enable"
    else:
        charging = "disable"
    ret = cmdr.send_prompt_command(f"battery chargeopt {charging}")
    if ret:
        raise exceptions.PromptResponseError(ret)


@PebbleCommander.command()
def battery_status(cmdr):
    """Get current battery status."""
    return cmdr.send_prompt_command("battery status")
