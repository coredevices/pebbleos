# SPDX-FileCopyrightText: 2024 Google LLC
# SPDX-License-Identifier: Apache-2.0

import time

from .. import PebbleCommander, exceptions

# Injected input is single-flight on the watch: a click that arrives while an
# earlier sequence is still draining (or a remote controller holds a button)
# answers BUSY. Wait it out instead of failing the caller's script.
BUSY_RETRIES = 50
BUSY_RETRY_DELAY_S = 0.1


def _send_click_command(cmdr, command):
    for _ in range(BUSY_RETRIES):
        ret = cmdr.send_prompt_command(command)
        if not ret[0].startswith("BUSY"):
            break
        time.sleep(BUSY_RETRY_DELAY_S)
    if not ret[0].startswith("OK"):
        raise exceptions.PromptResponseError(ret)


@PebbleCommander.command()
def click_short(cmdr, button):
    """Click a button."""
    button = int(str(button), 0)
    if not 0 <= button <= 3:
        raise exceptions.ParameterError("button out of range: %d" % button)
    _send_click_command(cmdr, "click short %d" % button)


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
        raise exceptions.ParameterError("button out of range: %d" % button)
    if not count > 0:
        raise exceptions.ParameterError("count out of range: %d" % count)
    if hold_ms < 0:
        raise exceptions.ParameterError("hold_ms out of range: %d" % hold_ms)
    if delay_ms < 0:
        raise exceptions.ParameterError("delay_ms out of range: %d" % delay_ms)
    _send_click_command(
        cmdr,
        "click multiple {button:d} {count:d} {hold_ms:d} {delay_ms:d}".format(
            **locals()
        ),
    )
