# SPDX-FileCopyrightText: 2024 Google LLC
# SPDX-License-Identifier: Apache-2.0

from .. import PebbleCommander, exceptions


@PebbleCommander.command()
def pfs_prepare(cmdr, size):
    """Prepare for file creation."""
    size = int(str(size), 0)
    if size <= 0:
        raise exceptions.ParameterError(f"size out of range: {size:d}")
    # TODO: I guess catch errors
    ret = cmdr.send_prompt_command(f"pfs prepare {size:d}")
    if not ret[0].startswith("Success"):
        raise exceptions.PromptResponseError(ret)


# TODO: pfs-write
# Can't do it with pulse prompt :(


@PebbleCommander.command()
def pfs_litter(cmdr):
    """Fragment the filesystem.

    Creates a bunch of fragmentation in the filesystem by creating a large
    number of small files and only deleting a small number of them.
    """
    ret = cmdr.send_prompt_command("litter pfs")
    if not ret[0].startswith("OK "):
        raise exceptions.PromptResponseError(ret)
    return [ret[0][3:]]
