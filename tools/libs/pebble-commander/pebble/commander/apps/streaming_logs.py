# SPDX-FileCopyrightText: 2024 Google LLC
# SPDX-License-Identifier: Apache-2.0


import collections
import struct
from datetime import datetime


class LogMessage(
    collections.namedtuple(
        "LogMessage", "log_level task timestamp file_name line_number message"
    )
):
    __slots__ = ()
    response_struct = struct.Struct("<c16sccQH")

    def __str__(self):
        msec_timestamp = self.timestamp.strftime("%H:%M:%S.%f")[:-3]
        template = (
            "{self.log_level} {self.task} {msec_timestamp} "
            "{self.file_name}:{self.line_number}> {self.message}"
        )

        return template.format(self=self, msec_timestamp=msec_timestamp)

    @classmethod
    def parse(cls, packet):
        result = cls.response_struct.unpack(packet[: cls.response_struct.size])
        msg = packet[cls.response_struct.size :].decode("utf8", errors="ignore")

        log_level = result[2].decode("utf8")
        task = result[3].decode("utf8")
        timestamp = datetime.fromtimestamp(result[4] / 1000.0)
        file_name = result[1].split(b"\x00", 1)[0].decode("utf8")  # NUL terminated
        line_number = result[5]

        return cls(log_level, task, timestamp, file_name, line_number, msg)


class StreamingLogs:
    """App for receiving log messages streamed by the firmware."""

    PORT_NUMBER = 0x0003

    def __init__(self, interface):
        try:
            self.socket = interface.simplex_transport.open_socket(self.PORT_NUMBER)
        except AttributeError:
            raise TypeError(
                "LoggingApp must be bound directly to an Interface, not a Link"
            )

    def receive(self, block=True, timeout=None):
        return LogMessage.parse(self.socket.receive(block, timeout))

    def close(self):
        self.socket.close()
