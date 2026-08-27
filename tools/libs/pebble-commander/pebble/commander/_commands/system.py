# SPDX-FileCopyrightText: 2024 Google LLC
# SPDX-License-Identifier: Apache-2.0

from .. import PebbleCommander, exceptions, parsers


@PebbleCommander.command()
def version(cmdr):
    """Get version information."""
    return cmdr.send_prompt_command("version")


@PebbleCommander.command()
def boot_bit_set(cmdr, bit, value):
    """Set some boot bits.

    `bit` should be between 0 and 31.
    `value` should be a boolean.
    """
    bit = int(str(bit), 0)
    value = int(parsers.str2bool(value))
    if not 0 <= bit <= 31:
        raise exceptions.ParameterError(f"bit index out of range: {bit:d}")
    ret = cmdr.send_prompt_command(f"boot bit set {bit:d} {value:d}")
    if not ret[0].startswith("OK"):
        raise exceptions.PromptResponseError(ret)
