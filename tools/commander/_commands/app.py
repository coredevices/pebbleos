# SPDX-FileCopyrightText: 2024 Google LLC
# SPDX-License-Identifier: Apache-2.0

from .. import PebbleCommander, exceptions


@PebbleCommander.command()
def app_list(cmdr):
    """List applications."""
    return cmdr.send_prompt_command("app list")


@PebbleCommander.command()
def app_load_metadata(cmdr):
    """Ghetto metadata loading for pbw_image.py"""
    ret = cmdr.send_prompt_command("app load metadata")
    if not ret[0].startswith("OK"):
        raise exceptions.PromptResponseError(ret)


@PebbleCommander.command()
def app_launch(cmdr, idnum):
    """Launch an application."""
    idnum = int(str(idnum), 0)
    if idnum == 0:
        raise exceptions.ParameterError(f"idnum out of range: {idnum:d}")
    ret = cmdr.send_prompt_command(f"app launch {idnum:d}")
    if not ret[0].startswith("OK"):
        raise exceptions.PromptResponseError(ret)


@PebbleCommander.command()
def app_remove(cmdr, idnum):
    """Remove an application."""
    idnum = int(str(idnum), 0)
    if idnum == 0:
        raise exceptions.ParameterError(f"idnum out of range: {idnum:d}")
    ret = cmdr.send_prompt_command(f"app remove {idnum:d}")
    if not ret[0].startswith("OK"):
        raise exceptions.PromptResponseError(ret)


@PebbleCommander.command()
def app_resource_bank(cmdr, idnum=0):
    """Get resource bank info for an application."""
    idnum = int(str(idnum), 0)
    if idnum < 0:
        raise exceptions.ParameterError(f"idnum out of range: {idnum:d}")
    ret = cmdr.send_prompt_command(f"resource bank info {idnum:d}")
    if not ret[0].startswith("OK "):
        raise exceptions.PromptResponseError(ret)
    return [ret[0][3:]]


@PebbleCommander.command()
def app_next_id(cmdr):
    """Get next free application ID."""
    return cmdr.send_prompt_command("app next id")


@PebbleCommander.command()
def app_available(cmdr, idnum):
    """Check if an application is available."""
    idnum = int(str(idnum), 0)
    if idnum == 0:
        raise exceptions.ParameterError(f"idnum out of range: {idnum:d}")
    return cmdr.send_prompt_command(f"app available {idnum:d}")
