import logging

import pebble_tool
from libpebble2.communication.transports.pulse import PULSETransport
from libpebble2.exceptions import PebbleError

from .commands import coredump, flash_logs, install_firmware, install_lang, test

# TODO: unopened logging ports cause super noisy logs, fix this in the
# pulse package then remove this
logging.getLogger("pebble.pulse2.transports").setLevel(logging.ERROR)


class PebbleTransportPULSE(pebble_tool.commands.base.PebbleTransportConfiguration):
    transport_class = PULSETransport
    name = "pulse"

    @classmethod
    def _connect_args(cls, args):
        try:
            from pebble import pulse2
        except ImportError:
            raise PebbleError(
                "pulse2 package not installed: it is required for PULSE transport"
            )

        (url,) = super()._connect_args(args)
        interface = pulse2.Interface.open_dbgserial(url=url)
        link = interface.get_link()
        return (link,)

    @classmethod
    def add_argument_handler(cls, parser):
        parser.add_argument(
            "--pulse",
            type=str,
            help="Use this option to connect to your Pebble via"
            " the PULSE transport. Equivalent to PEBBLE_PULSE.",
        )


def run_tool(args=None):
    pebble_tool.run_tool(args)

__all__ = [
    "coredump",
    "flash_logs",
    "install_firmware",
    "install_lang",
    "test",
]
