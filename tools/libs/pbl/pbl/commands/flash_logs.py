# SPDX-FileCopyrightText: 2025 Federico Bechini
# SPDX-License-Identifier: Apache-2.0


import os

from libpebble2.exceptions import GetBytesError
from libpebble2.protocol.transfers import GetBytesInfoResponse
from libpebble2.services.getbytes import GetBytesService
from pebble_tool.commands.base import PebbleCommand
from pebble_tool.exceptions import ToolError


class FlashLogsCommand(PebbleCommand):
    """Dump flash logs (PBL_LOG) from the watch."""

    command = "flash_logs"

    @classmethod
    def add_parser(cls, parser):
        parser = super().add_parser(parser)
        parser.add_argument(
            "--board",
            required=True,
            type=str.lower,
            help="Board name (e.g., aplite, basalt, asterix)",
        )
        return parser

    def __call__(self, args):
        super().__call__(args)
        get_bytes = GetBytesService(self.pebble)

        # Map board names to (start_address, size)
        # Sizes are mostly 128KB (0x20000)
        FLASH_LOG_REGIONS = {
            # Legacy Platforms
            "aplite": (0x3E0000, 0x20000),
            "tintin": (0x3E0000, 0x20000),
            # Silk / Diorite
            "diorite": (0x280000, 0x20000),
            "silk": (0x280000, 0x20000),
            # Asterix
            "asterix": (0x1FD0000, 0x20000),
            # Obelix / Getafix
            "obelix": (0x1FCF000, 0x20000),
            "getafix": (0x1FCF000, 0x20000),
        }

        # Normalize board name
        board = args.board

        region = FLASH_LOG_REGIONS.get(board)
        if not region:
            # Try simple aliasing or partial matching if needed, but for now strict map
            print(f"Error: Unknown board '{board}'.")
            print(
                "Supported boards: {}".format(
                    ", ".join(sorted(FLASH_LOG_REGIONS.keys()))
                )
            )
            return

        flash_log_start, flash_log_size = region

        print(f"Board: {board}")
        print(
            f"Reading flash log region: 0x{flash_log_start:X} - 0x{flash_log_start + flash_log_size:X} ({flash_log_size // 1024} KB)"
        )

        try:
            flash_data = get_bytes.get_flash_region(flash_log_start, flash_log_size)
            print(f"Read {len(flash_data)} bytes from flash")

            # Save to file
            import datetime

            filename = datetime.datetime.now().strftime(
                f"flash_logs_{board}_%Y-%m-%d_%H-%M-%S.bin"
            )
            filepath = os.path.abspath(filename)
            with open(filename, "wb") as log_file:
                log_file.write(flash_data)
            print(f"Saved flash logs to {filepath}")

            print("\nTo parse and dehash the logs:")
            print(f"  tools/dehash_flash_logs.py {filename}")

        except GetBytesError as ex:
            if ex.code == GetBytesInfoResponse.ErrorCode.DoesNotExist:
                raise ToolError(
                    "Could not read flash region. This may require non-release firmware."
                )
            else:
                raise
