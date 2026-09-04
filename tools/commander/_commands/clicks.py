# SPDX-FileCopyrightText: 2024 Google LLC
# SPDX-License-Identifier: Apache-2.0

from .. import PebbleCommander, exceptions


@PebbleCommander.command()
def click_short(cmdr, button):
    """Click a button."""
    button = int(str(button), 0)
    if not 0 <= button <= 3:
        raise exceptions.ParameterError(f"button out of range: {button:d}")
    ret = cmdr.send_prompt_command(f"click short {button:d}")
    if not ret[0].startswith("OK"):
        raise exceptions.PromptResponseError(ret)


@PebbleCommander.command()
def click_long(cmdr, button, hold_ms=20):
    """Hold a button.

    `hold_ms` is how many ms to hold the button down before releasing.
    """
    return cmdr.click_multiple(button, hold_ms=hold_ms)


@PebbleCommander.command()
def click_multiple(cmdr, button, count=1, hold_ms=20, delay_ms=0):
    """Rhythmically click a button."""
    button = int(str(button), 0)
    count = int(str(count), 0)
    hold_ms = int(str(hold_ms), 0)
    delay_ms = int(str(delay_ms), 0)
    if not 0 <= button <= 3:
        raise exceptions.ParameterError(f"button out of range: {button:d}")
    if not count > 0:
        raise exceptions.ParameterError(f"count out of range: {count:d}")
    if hold_ms < 0:
        raise exceptions.ParameterError(f"hold_ms out of range: {hold_ms:d}")
    if delay_ms < 0:
        raise exceptions.ParameterError(f"delay_ms out of range: {delay_ms:d}")
    ret = cmdr.send_prompt_command(
        "click multiple {button:d} {count:d} {hold_ms:d} {delay_ms:d}".format(
            **locals()
        )
    )
    if not ret[0].startswith("OK"):
        raise exceptions.PromptResponseError(ret)
