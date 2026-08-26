#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2026 Ahmed Hussein
# SPDX-License-Identifier: Apache-2.0

"""Install a .pbl language pack onto a running Pebble emulator."""

import argparse

from libpebble2.communication import PebbleConnection
from libpebble2.communication.transports.websocket import WebsocketTransport
from libpebble2.protocol.transfers import ObjectType
from libpebble2.services.putbytes import PutBytes


def main():
    p = argparse.ArgumentParser()
    p.add_argument("pbl", help="Path to .pbl language pack")
    p.add_argument("--ws", default="ws://localhost:63117/", help="pypkjs websocket URL")
    args = p.parse_args()

    pebble = PebbleConnection(WebsocketTransport(args.ws))
    pebble.connect()
    pebble.run_async()
    print(f"Connected to {args.ws}")

    with open(args.pbl, "rb") as f:
        data = f.read()
    print(f"Installing {args.pbl} ({len(data)} bytes) as PFS file 'lang'...")

    pb = PutBytes(pebble, ObjectType.File, data, bank=0, filename="lang")
    pb.send()
    print("Done.")


if __name__ == "__main__":
    main()
