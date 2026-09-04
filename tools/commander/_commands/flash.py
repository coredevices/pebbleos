# SPDX-FileCopyrightText: 2024 Google LLC
# SPDX-License-Identifier: Apache-2.0

from .. import PebbleCommander, exceptions

# TODO: flash-write
# Can't do it with pulse prompt :(


@PebbleCommander.command()
def flash_erase(cmdr, address, length):
    """Erase flash area."""
    address = int(str(address), 0)
    length = int(str(length), 0)
    if address < 0:
        raise exceptions.ParameterError(f"address out of range: {address:d}")
    if length <= 0:
        raise exceptions.ParameterError(f"length out of range: {length:d}")
    # TODO: I guess catch errors
    ret = cmdr.send_prompt_command(f"erase flash 0x{address:X} {length:d}")
    if not ret[1].startswith("OK"):
        raise exceptions.PromptResponseError(ret)


@PebbleCommander.command()
def flash_crc(cmdr, address, length):
    """Calculate CRC of flash area."""
    address = int(str(address), 0)
    length = int(str(length), 0)
    if address < 0:
        raise exceptions.ParameterError(f"address out of range: {address:d}")
    if length <= 0:
        raise exceptions.ParameterError(f"length out of range: {length:d}")
    # TODO: I guess catch errors
    ret = cmdr.send_prompt_command(f"crc flash 0x{address:X} {length:d}")
    if not ret[0].startswith("CRC: "):
        raise exceptions.PromptResponseError(ret)
    return [ret[0][5:]]


@PebbleCommander.command()
def prf_address(cmdr):
    """Get address of PRF."""
    ret = cmdr.send_prompt_command("prf image address")
    if not ret[0].startswith("OK "):
        raise exceptions.PromptResponseError(ret)
    return [ret[0][3:]]
