from libpebble2.services.putbytes import PutBytes, PutBytesType
from pebble_tool.commands.base import PebbleCommand
from progressbar import Bar, FileTransferSpeed, Percentage, ProgressBar, Timer


class InstallLangCcommand(PebbleCommand):
    """Install a language pack on a watch"""

    command = "install-lang"

    def __call__(self, args):
        super().__call__(args)

        progress_bar = ProgressBar(
            widgets=[
                Percentage(),
                Bar(marker="=", left="[", right="]"),
                " ",
                FileTransferSpeed(),
                " ",
                Timer(format="%s"),
            ]
        )

        with open(args.lang_file, "rb") as f:
            lang_pack = f.read()

        progress_bar.maxval = len(lang_pack)
        progress_bar.start()

        def _handle_progress(sent, total_sent, total_length):
            progress_bar.update(total_sent)

        pb = PutBytes(
            self.pebble, PutBytesType.File, lang_pack, bank=0, filename="lang"
        )
        pb.register_handler("progress", _handle_progress)
        pb.send()

        progress_bar.finish()

    @classmethod
    def add_parser(cls, parser):
        parser = super().add_parser(parser)
        parser.add_argument("lang_file", help="Language file to install")
        return parser
