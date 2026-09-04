# SPDX-FileCopyrightText: 2024 Google LLC
# SPDX-License-Identifier: Apache-2.0


import re

from . import exceptions


def str2bool(s, also_true=(), also_false=()):
    s = str(s).lower()
    if s in ("yes", "on", "t", "1", "true", "enable") or s in also_true:
        return True
    if s in ("no", "off", "f", "0", "false", "disable") or s in also_false:
        return False
    raise exceptions.ParameterError(f"{s} not a valid bool string.")


def str2mac(s):
    s = str(s)
    if not re.match(r"[0-9a-fA-F]{2}(:[0-9a-fA-F]{2}){5}", s):
        raise exceptions.ParameterError(f"{s} is not a valid MAC address")
    mac = []
    for byte in str(s).split(":"):
        mac.append(int(byte, 16))
    return tuple(mac)
