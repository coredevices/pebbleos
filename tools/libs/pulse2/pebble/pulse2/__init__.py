# SPDX-FileCopyrightText: 2024 Google LLC
# SPDX-License-Identifier: Apache-2.0

from . import link, transports

# Public aliases for the classes that users will interact with directly.
from .link import Interface

link.Link.register_transport("best-effort", transports.BestEffortApplicationTransport)
link.Link.register_transport("reliable", transports.ReliableTransport)

__all__ = [
    "Interface",
    "link",
    "transports",
]
