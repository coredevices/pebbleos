# SPDX-FileCopyrightText: 2024 Google LLC
# SPDX-License-Identifier: Apache-2.0

"""PULSE Control Message Protocol"""


import codecs
import collections
import enum
import logging
import struct
import threading

from . import exceptions
from . import logging as pulse2_logging

logger = logging.getLogger(__name__)
logger.addHandler(logging.NullHandler())


class ParseError(exceptions.PulseException):
    pass


@enum.unique
class PCMPCode(enum.Enum):
    Echo_Request = 1
    Echo_Reply = 2
    Discard_Request = 3
    Port_Closed = 129
    Unknown_Code = 130


class PCMPPacket(collections.namedtuple("PCMPPacket", "code information")):
    __slots__ = ()

    @classmethod
    def parse(cls, packet):
        packet = bytes(packet)
        if len(packet) < 1:
            raise ParseError("packet too short")
        return cls(code=struct.unpack("B", packet[0:1])[0], information=packet[1:])

    @staticmethod
    def build(code, information):
        return struct.pack("B", code) + bytes(information)


class PulseControlMessageProtocol:
    """This protocol is unique in that it is logically part of the
    transport but is layered on top of the transport over the wire.
    To keep from needing to create a new thread just for reading from
    the socket, the implementation acts both like a socket and protocol
    all in one.
    """

    PORT = 0x0001

    on_port_closed = None

    @classmethod
    def bind(cls, transport):
        return transport.open_socket(cls.PORT, factory=cls)

    def __init__(self, transport, port):
        assert port == self.PORT
        self.logger = pulse2_logging.TaggedAdapter(
            logger, {"tag": f"PCMP({type(transport).__name__})"}
        )
        self.transport = transport
        self.closed = False
        self.ping_lock = threading.RLock()
        self.ping_cb = None
        self.ping_attempts_remaining = 0
        self.ping_timer = None

    def close(self):
        if self.closed:
            return
        with self.ping_lock:
            self.ping_cb = None
            if self.ping_timer:
                self.ping_timer.cancel()
        self.closed = True
        self.transport.unregister_socket(self.PORT)

    def send_unknown_code(self, bad_code):
        self.transport.send(
            self.PORT,
            PCMPPacket.build(PCMPCode.Unknown_Code.value, struct.pack("B", bad_code)),
        )

    def send_echo_request(self, data):
        self.transport.send(
            self.PORT, PCMPPacket.build(PCMPCode.Echo_Request.value, data)
        )

    def send_echo_reply(self, data):
        self.transport.send(
            self.PORT, PCMPPacket.build(PCMPCode.Echo_Reply.value, data)
        )

    def on_receive(self, raw_packet):
        try:
            packet = PCMPPacket.parse(raw_packet)
        except ParseError:
            self.logger.exception("Received malformed packet")
            return
        try:
            code = PCMPCode(packet.code)
        except ValueError:
            self.logger.error("Received packet with unknown code %d", packet.code)
            self.send_unknown_code(packet.code)
            return

        if code == PCMPCode.Discard_Request:
            pass
        elif code == PCMPCode.Echo_Request:
            self.send_echo_reply(packet.information)
        elif code == PCMPCode.Echo_Reply:
            with self.ping_lock:
                if self.ping_cb:
                    self.ping_timer.cancel()
                    self.ping_cb(True)
                    self.ping_cb = None
            self.logger.debug(
                "Echo-Reply: %s", codecs.encode(packet.information, "hex")
            )
        elif code == PCMPCode.Port_Closed:
            if len(packet.information) == 2:
                if self.on_port_closed:
                    (closed_port,) = struct.unpack("!H", packet.information)
                    self.on_port_closed(closed_port)
            else:
                self.logger.error(
                    "Remote peer sent malformed Port-Closed packet: %s",
                    codecs.encode(packet.information, "hex"),
                )
        elif code == PCMPCode.Unknown_Code:
            if len(packet.information) == 1:
                self.logger.error(
                    "Remote peer sent Unknown-Code(%d) packet",
                    struct.unpack("B", packet.information)[0],
                )
            else:
                self.logger.error(
                    "Remote peer sent malformed Unknown-Code packet: %s",
                    codecs.encode(packet.information, "hex"),
                )
        else:
            assert False, "Known code not handled"

    def ping(self, result_cb, attempts=3, timeout=1.0):
        """Test the link quality by sending Echo-Request packets and
        listening for Echo-Reply packets from the remote peer.

        The ping is performed asynchronously. The `result_cb` callable
        will be called when the ping completes. It will be called with
        a single positional argument: a truthy value if the remote peer
        responded to the ping, or a falsy value if all ping attempts
        timed out.
        """
        if attempts < 1:
            raise ValueError("attempts must be positive")
        if timeout <= 0:
            raise ValueError("timeout must be positive")
        with self.ping_lock:
            if self.ping_cb:
                raise exceptions.AlreadyInProgressError(
                    "another ping is currently in progress"
                )
            self.ping_cb = result_cb
            self.ping_attempts_remaining = attempts - 1
            self.ping_timeout = timeout
            self.send_echo_request(b"")
            self.ping_timer = threading.Timer(timeout, self._ping_timer_expired)
            self.ping_timer.daemon = True
            self.ping_timer.start()

    def _ping_timer_expired(self):
        with self.ping_lock:
            if not self.ping_cb:
                # The Echo-Reply packet must have won the race
                return
            if self.ping_attempts_remaining:
                self.ping_attempts_remaining -= 1
                self.send_echo_request(b"")
                self.ping_timer = threading.Timer(
                    self.ping_timeout, self._ping_timer_expired
                )
                self.ping_timer.daemon = True
                self.ping_timer.start()
            else:
                self.ping_cb(False)
                self.ping_cb = None
