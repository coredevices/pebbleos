# SPDX-FileCopyrightText: 2024 Google LLC
# SPDX-License-Identifier: Apache-2.0

import collections
import struct
from datetime import datetime

from . import exceptions, socket


class PromptProtocol:
    PROTOCOL_NUMBER = 0x04

    def __init__(self, connection):
        self.socket = socket.ProtocolSocket(connection, self.PROTOCOL_NUMBER)

    def command_and_response(self, command_string, timeout=1):
        log = []

        cmd = PromptCommand(command_string)

        for _ in range(5):
            self.socket.send(cmd.packet)
            try:
                response = PromptResponse.parse(self.socket.receive(timeout=timeout))
            except exceptions.ReceiveQueueEmpty:
                continue

            # Retry the command if we don't get an ACK
            if not response.is_ack_response():
                continue

            # Receive messages until DONE
            is_done = False
            retries = 0

            while not is_done and retries < 3:
                try:
                    response = PromptResponse.parse(
                        self.socket.receive(timeout=timeout)
                    )

                    if response.is_done_response():
                        is_done = True
                    elif response.is_message_response():
                        log.append(response.message)

                        retries = 0
                except exceptions.ReceiveQueueEmpty:
                    self.socket.send(cmd.packet)

                    retries += 1

            if retries == 3:
                raise exceptions.CommandTimedOut("Lost connection while waiting")

            return log

        raise exceptions.CommandTimedOut("Command not acknowledged")


class PromptResponse(
    collections.namedtuple("PromptResponse", "response_type timestamp message")
):
    ACK_RESPONSE = 100
    DONE_RESPONSE = 101
    MESSAGE_RESPONSE = 102

    response_struct = struct.Struct("<BQ")

    def is_ack_response(self):
        return self.response_type == self.ACK_RESPONSE

    def is_done_response(self):
        return self.response_type == self.DONE_RESPONSE

    def is_message_response(self):
        return self.response_type == self.MESSAGE_RESPONSE

    @classmethod
    def parse(cls, response):
        result = cls.response_struct.unpack(response[: cls.response_struct.size])

        response_type = result[0]
        timestamp = datetime.fromtimestamp(result[1] / 1000.0).astimezone()
        message = response[cls.response_struct.size :]

        return cls(response_type, timestamp, message)


class PromptCommand:
    _cookie = 0

    def __init__(self, body):
        self.body = body
        self.cookie = self._get_cookie()

    @property
    def packet(self):
        return chr(self.cookie) + str(self.body)

    @classmethod
    def _get_cookie(cls):
        cookie = cls._cookie
        cls._cookie = (cls._cookie + 1) % 256
        return cookie
