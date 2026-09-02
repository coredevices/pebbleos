
import datetime

from libpebble2.exceptions import GetBytesError
from libpebble2.protocol.transfers import GetBytesInfoResponse
from libpebble2.services.getbytes import GetBytesService
from pebble_tool.commands.base import PebbleCommand
from pebble_tool.exceptions import ToolError
from progressbar import Bar, FileTransferSpeed, Percentage, ProgressBar, Timer


class CoredumpCommand(PebbleCommand):
    """Takes a screenshot from the watch."""

    command = "coredump"

    def __init__(self):
        self.progress_bar = ProgressBar(
            widgets=[
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
        get_bytes = GetBytesService(self.pebble)
        get_bytes.register_handler("progress", self._handle_progress)

        self.progress_bar.start()
        try:
            core_data = get_bytes.get_coredump(args.fresh)
        except GetBytesError as ex:
            if ex.code == GetBytesInfoResponse.ErrorCode.DoesNotExist:
                raise ToolError("No coredump on device")
            else:
                raise

        self.progress_bar.finish()

        filename = self._generate_filename() if args.filename is None else args.filename
        with open(filename, "wb") as core_file:
            core_file.write(core_data)
        print(f"Saved coredump to {filename}")

    def _handle_progress(self, progress, total):
        if not self.started:
            self.progress_bar.maxval = total
            self.started = True
        self.progress_bar.update(progress)

    @classmethod
    def _generate_filename(cls):
        return datetime.datetime.now().astimezone().strftime(
            "pebble_coredump_%Y-%m-%d_%H-%M-%S.core"
        )

    @classmethod
    def add_parser(cls, parser):
        parser = super().add_parser(parser)
        parser.add_argument(
            "filename", nargs="?", type=str, help="Filename of coredump"
        )
        parser.add_argument(
            "--fresh", action="store_true", help="Require a fresh coredump"
        )
        return parser
