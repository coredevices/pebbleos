# SPDX-FileCopyrightText: 2024 Google LLC
# SPDX-License-Identifier: Apache-2.0


import collections
import struct
from datetime import datetime

import pebble.pulse2.exceptions

from .. import exceptions


class Prompt:
    PORT_NUMBER = 0x3E20

    def __init__(self, link):
        self.socket = link.open_socket("reliable", self.PORT_NUMBER)

    def command_and_response(self, command_string, timeout=20):
        log = []
        self.socket.send(command_string.encode())

        is_done = False
        while not is_done:
            try:
                response = PromptResponse.parse(self.socket.receive(timeout=timeout))
                if response.is_done_response:
                    is_done = True
                elif response.is_message_response:
                    log.append(response.message.decode())
            except pebble.pulse2.exceptions.ReceiveQueueEmpty:
                raise exceptions.CommandTimedOut
        return log

    def close(self):
        self.socket.close()


class PromptResponse(
    collections.namedtuple("PromptResponse", "response_type timestamp message")
):
    DONE_RESPONSE = 101
    MESSAGE_RESPONSE = 102

    response_struct = struct.Struct("<BQ")

    @property
    def is_done_response(self):
        return self.response_type == self.DONE_RESPONSE

    @property
    def is_message_response(self):
        return self.response_type == self.MESSAGE_RESPONSE

    @classmethod
    def parse(cls, response):
        result = cls.response_struct.unpack(response[: cls.response_struct.size])

        response_type = result[0]
        timestamp = datetime.fromtimestamp(result[1] / 1000.0)
        message = response[cls.response_struct.size :]

        return cls(response_type, timestamp, message)
