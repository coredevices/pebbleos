# SPDX-FileCopyrightText: 2024 Google LLC
# SPDX-License-Identifier: Apache-2.0

from . import flash_imaging, pulse_logging, pulse_prompt
from .exceptions import PulseError
from .socket import Connection

Connection.register_extension("flash", flash_imaging.FlashImagingProtocol)
Connection.register_extension("logging", pulse_logging.LoggingProtocol)
Connection.register_extension("prompt", pulse_prompt.PromptProtocol)

__all__ = [
    "Connection",
    "PulseError",
    "flash_imaging",
    "pulse_logging",
    "pulse_prompt",
]
