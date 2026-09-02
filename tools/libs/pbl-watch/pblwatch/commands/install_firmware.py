# Copyright (c) 2025 Joshua Wise
# SPDX-License-Identifier: Apache-2.0


from libpebble2.protocol.system import SystemMessage
from libpebble2.services.putbytes import PutBytes, PutBytesType
from libpebble2.util.bundle import PebbleBundle
from pebble_tool.commands.base import PebbleCommand
from progressbar import (
    Bar,
    FileTransferSpeed,
    FormatLabel,
    Percentage,
    ProgressBar,
    Timer,
)


class InstallFirmwareCommand(PebbleCommand):
    """Install a .pbz onto the watch."""

    command = "install-firmware"

    def __init__(self):
        self.label = FormatLabel("{variables[task]}", new_style=True)
        self.progress_bar = ProgressBar(
            widgets=[
                self.label,
                Percentage(),
                Bar(marker="=", left="[", right="]"),
                " ",
                FileTransferSpeed(),
                " ",
                Timer(format="%s"),
            ]
        )
        self.started = False

    def __call__(self, args):
        super().__call__(args)

        self.pebble.send_and_read(
            SystemMessage(message_type=SystemMessage.Type.FirmwareUpdateStart),
            SystemMessage,
        )

        bundle = PebbleBundle(args.filename)

        self.progress_bar.variables["task"] = "pebbleos.bin            "
        firmware_bytes = bundle.zip.read(bundle.get_firmware_info()["name"])
        pb = PutBytes(self.pebble, PutBytesType.Firmware, firmware_bytes, bank=0)
        pb.register_handler("progress", self._handle_progress)
        pb.send()
        self.progress_bar.finish()
        self.started = False

        self.progress_bar.variables["task"] = "system_resources.pbpack "
        resource_bytes = bundle.zip.read(bundle.get_resource_path())
        pb = PutBytes(self.pebble, PutBytesType.SystemResources, resource_bytes, bank=0)
        pb.register_handler("progress", self._handle_progress)
        pb.send()
        self.progress_bar.finish()

        self.pebble.send_packet(
            SystemMessage(message_type=SystemMessage.Type.FirmwareUpdateComplete)
        )

    def _handle_progress(self, this_interval, progress, total):
        if not self.started:
            self.progress_bar.max_value = total
            self.progress_bar.start()
            self.started = True
        self.progress_bar.update(progress)

    @classmethod
    def add_parser(cls, parser):
        parser = super().add_parser(parser)
        parser.add_argument("filename", nargs="?", type=str, help="Filename of pbz")
        return parser
