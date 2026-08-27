# SPDX-FileCopyrightText: 2024 Google LLC
# SPDX-License-Identifier: Apache-2.0

import logging
import struct
import sys
import threading
import time
import traceback
import uuid
import weakref
from queue import Queue
from typing import ClassVar

import serial
import stm32_crc
from cobs import cobs

from . import exceptions

logger = logging.getLogger(__name__)

DBGSERIAL_PORT_SETTINGS = {"baudrate": 115200, "timeout": 0.1, "interCharTimeout": 0.01}


def frame_splitter(istream, size=1024, timeout=1, delimiter="\0"):
    """Returns an iterator which yields complete frames."""
    partial = []
    start_time = time.time()
    while not istream.closed:
        data = istream.read(size)
        logger.debug("frame_splitter: received %r", data)
        while True:
            left, delim, data = data.partition(delimiter)
            if left:
                partial.append(left)
            if delim:
                frame = "".join(partial)
                partial = []
                if frame:
                    yield frame
            if not data:
                break
        if timeout > 0 and time.time() > start_time + timeout:
            yield


def decode_frame(frame):
    """Decodes a PULSE frame.

    Returns a tuple (protocol, payload) of the decoded frame.
    Raises FrameDecodeError if the frame is not valid.
    """
    try:
        data = cobs.decode(frame)
    except cobs.DecodeError as e:
        raise exceptions.FrameDecodeError(e.message)
    if len(data) < 5:
        raise exceptions.FrameDecodeError("frame too short")
    fcs = struct.unpack("<I", data[-4:])[0]
    crc = stm32_crc.crc32(data[:-4])
    if fcs != crc:
        raise exceptions.FrameDecodeError(f"FCS 0x{fcs:08x} != CRC 0x{crc:08x}")
    protocol = ord(data[0])
    return (protocol, data[1:-4])


def encode_frame(protocol, payload):
    frame = struct.pack("<B", protocol)
    frame += payload.encode()
    fcs = stm32_crc.crc32(frame)
    frame += struct.pack("<I", fcs)
    return cobs.encode(frame)


class Connection:
    """A socket for sending and receiving datagrams over the PULSE serial
    protocol.
    """

    PROTOCOL_LLC = 0x01

    LLC_LINK_OPEN_REQUEST = "\x01\x03\x08\x08\x08PULSEv1\r\n"
    LLC_LINK_CLOSE_REQUEST = "\x03"
    LLC_ECHO_REQUEST = "\x05"
    LLC_CHANGE_BAUD = "\x07"

    LLC_LINK_OPENED = 0x02
    LLC_LINK_CLOSED = 0x04
    LLC_ECHO_REPLY = 0x06

    EXTENSIONS: ClassVar = {}

    # Maximum round-trip time
    rtt = 0.4

    def __init__(self, iostream, infinite_reconnect=False):
        self.infinite_reconnect = infinite_reconnect
        self.iostream = iostream
        self.closed = False
        try:
            self.initial_port_settings = self.iostream.getSettingsDict()
        except AttributeError:
            self.initial_port_settings = None
        self.port_settings_altered = False
        # Whether the link is open for sending.
        self._link_open = threading.Event()
        # Whether the link has been severed.
        self._link_closed = threading.Event()
        self.send_lock = threading.RLock()
        self.echoes_inflight = weakref.WeakValueDictionary()
        self.protocol_handlers = weakref.WeakValueDictionary()
        self.receive_thread = threading.Thread(target=self.run_receive_thread)
        self.receive_thread.daemon = True
        self.receive_thread.start()
        self._open_link()

        self.keepalive_thread = threading.Thread(target=self.run_keepalive_thread)
        self.keepalive_thread.daemon = True
        self.keepalive_thread.start()

        # Instantiate and bind all known extensions
        for name, factory in self.EXTENSIONS.iteritems():
            setattr(self, name, factory(self))

    @classmethod
    def register_extension(cls, name, factory):
        """Register a PULSE connection extension.

        When a Connection object is instantiated, the object returned by
        factory(connection_object) is assigned to connection_object.<name>.
        """
        try:
            getattr(cls, name)
        except AttributeError:
            cls.EXTENSIONS[name] = factory
        else:
            raise ValueError(
                f"extension name {name!r} clashes with existing attribute"
            )

    @classmethod
    def open_dbgserial(cls, url=None, infinite_reconnect=False):
        if url is None:
            raise exceptions.TTYAutodetectionUnavailable
        if url == "qemu":
            url = "socket://localhost:12345"
        ser = serial.serial_for_url(url, **DBGSERIAL_PORT_SETTINGS)

        if url.startswith("socket://"):
            # Socket class for PySerial does some pointless buffering
            # setting a very small timeout effectively negates it
            ser._timeout = 0.00001

        return cls(ser, infinite_reconnect=infinite_reconnect)

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.close()

    def __del__(self):
        self.close()

    def send(self, protocol, payload):
        if self.closed:
            raise exceptions.PulseError("I/O operation on closed connection")
        frame = b"".join((b"\0", encode_frame(protocol, payload), b"\0"))
        logger.debug("Connection: sending %r", frame)
        with self.send_lock:
            self.iostream.write(frame)

    def run_receive_thread(self):
        logger.debug("Connection: receive thread started")
        receiver = frame_splitter(self.iostream, timeout=0)
        while True:
            try:
                protocol, payload = decode_frame(next(receiver))
            except exceptions.FrameDecodeError:
                continue
            except Exception:  # noqa: BLE001
                # Probably a PySerial exception complaining about reading from a
                # closed port. Eat the exception and shut down the thread; users
                # don't need to see the stack trace.
                logger.debug(
                    "Connection: exception in receive thread:\n%s",
                    traceback.format_exc(),
                )
                break
            logger.debug(
                "Connection:run_receive_thread: protocol=%d payload=%r",
                protocol,
                payload,
            )
            if protocol == self.PROTOCOL_LLC:  # LLC can't be overridden
                self.llc_handler(payload)
                continue
            try:
                handler = self.protocol_handlers[protocol]
            except KeyError:
                self.default_receiver(protocol, payload)
            else:
                handler.on_receive(payload)
        logger.debug("Connection: receive thread exiting")

    def default_receiver(self, protocol, frame):
        logger.info(
            "Connection:default_receiver: received frame with protocol %d: %r",
            protocol,
            frame,
        )

    def register_protocol_handler(self, protocol, handler):
        """Register a handler for frames bearing the specified protocol number.

        handler.on_receive(payload) is called for each frame received with the
        protocol number.

        Protocol handlers can be unregistered by calling this function with a
        handler of None.
        """
        if not handler:
            try:
                del self.protocol_handlers[protocol]
            except KeyError:
                pass
            return
        if protocol in self.protocol_handlers:
            raise exceptions.ProtocolAlreadyRegistered(
                f"Protocol {protocol:d} is already registered by {self.protocol_handlers[protocol]!r}"
            )
        if not hasattr(handler, "on_receive"):
            raise ValueError("%r does not have an on_receive method")
        self.protocol_handlers[protocol] = handler

    def llc_handler(self, frame):
        opcode = ord(frame[0])
        if opcode == self.LLC_LINK_OPENED:
            # MTU and MRU are from the perspective of this side of the
            # connection
            version, mru, mtu, timeout = struct.unpack("<xBHHB", frame)
            self.version = version
            # The server reports the MTU inclusive of protocol number and FCS,
            # but we only care about the maximum payload length.
            self.mtu = mtu - 5
            self.mru = mru
            # Timeout is specified in deciseconds. Convert to seconds.
            self.timeout = timeout / 10.0
            self._link_closed.clear()
            self._link_open.set()
        elif opcode == self.LLC_LINK_CLOSED:
            logger.info("PULSE connection closed.")
            self._link_closed.set()
        elif opcode == self.LLC_ECHO_REPLY:
            self._on_echo_reply(frame[1:])
        else:
            logger.warning("Received LLC frame with unknown type %d: %r", opcode, frame)

    def run_keepalive_thread(self):
        """The keepalive thread monitors the link, reopening it if necessary."""
        logger.debug("Connection: keepalive thread started")
        OPEN, TEST_LIVENESS, RECONNECT = range(3)
        state = OPEN
        next_state = state
        ping_attempts = 0
        ping_wait = self.rtt
        while True:
            # Check whether the link is being closed from our side before
            # trying to keep it alive.
            if not self._link_open.is_set():
                return

            if state == OPEN:
                time.sleep(1)
                if self._link_closed.is_set():
                    next_state = RECONNECT
                else:
                    next_state = TEST_LIVENESS
            elif state == TEST_LIVENESS:
                if ping_attempts < 3:
                    ping_attempts += 1
                    ping_wait *= 2  # Exponential backoff
                    if self.ping(ping_wait):
                        next_state = OPEN
                    else:
                        logger.info(
                            "No response to keepalive ping -- strike %d", ping_attempts
                        )
                else:
                    logger.info("Connection: keepalive timed out.")
                    next_state = RECONNECT
            elif state == RECONNECT:
                # Lock out everyone from sending so that applications don't send
                # to a connection that's in an indeterminate state.
                with self.send_lock:
                    if self.port_settings_altered:
                        # Ensure that the server has timed out and reset its
                        # baud rate so we don't get into the bad situation where
                        # we try to reconnect at the default baud rate but the
                        # server is listening at a different rate, which is
                        # practically guaranteed to fail.
                        logger.info(
                            "Letting connection time out before "
                            "attempting to reconnect."
                        )
                        time.sleep(self.timeout + self.rtt)
                    self._link_open.clear()
                    while not self._link_open.is_set():
                        try:
                            self._open_link()
                        except exceptions.PulseError as e:
                            logger.warning("Connection: reconnect failed. %s", e)
                            if not self.infinite_reconnect:
                                break
                            logger.warning("Will try again.")
                            logger.info("Backing off for a while before retrying.")
                            time.sleep(self.timeout + self.rtt)
                        else:
                            next_state = OPEN
            else:
                assert False, f"Invalid state {state:d}"

            if next_state != state and next_state == TEST_LIVENESS:
                ping_attempts = 0
                ping_wait = self.rtt
            state = next_state

    def _open_link(self):
        self.closed = False
        if self.initial_port_settings:
            self.iostream.applySettingsDict(self.initial_port_settings)
        for attempt in range(5):
            logger.info("Opening link (attempt %d)...", attempt)
            self.send(self.PROTOCOL_LLC, self.LLC_LINK_OPEN_REQUEST)
            if self._link_open.wait(self.rtt):
                logger.info("Established PULSE connection!")
                logger.info(
                    "Version=%d  MTU=%d  MRU=%d  Timeout=%.1f",
                    self.version,
                    self.mtu,
                    self.mru,
                    self.timeout,
                )
                break
        else:
            self._link_closed.set()
            self.closed = True
            raise exceptions.PulseError("Could not establish connection")

    def close(self):
        self._link_open.clear()
        if not self._link_closed.is_set():
            for attempt in range(3):
                self.send(self.PROTOCOL_LLC, self.LLC_LINK_CLOSE_REQUEST)
                if self._link_closed.wait(self.rtt):
                    break
            else:
                logger.warning("Could not confirm link close.")
                self._link_closed.set()
        self.iostream.close()
        self.closed = True

    def ping(self, timeout=None):
        if not timeout:
            timeout = 2 * self.rtt
        nonce = uuid.uuid4().bytes
        is_received = threading.Event()
        self.echoes_inflight[nonce] = is_received
        self.send(self.PROTOCOL_LLC, self.LLC_ECHO_REQUEST + nonce)
        return is_received.wait(timeout)

    def _on_echo_reply(self, payload):
        try:
            receive_event = self.echoes_inflight[payload]
            receive_event.set()
        except KeyError:
            pass

    def change_baud_rate(self, new_baud):
        # Fail fast if the IO object doesn't support changing the baud rate
        self.send(self.PROTOCOL_LLC, self.LLC_CHANGE_BAUD + struct.pack("<I", new_baud))
        # Be extra sure that the message has been sent and it's safe to adjust
        # the baud rate on the port.
        time.sleep(0.1)
        self.iostream.baudrate = new_baud
        self.port_settings_altered = True


class ProtocolSocket:
    """A socket for sending and receiving datagrams of a single protocol over a
    PULSE connection.

    It is also an example of a Connection protocol handler implementation.
    """

    def __init__(self, connection, protocol):
        self.connection = connection
        self.protocol = protocol
        self.receive_queue = Queue.Queue()
        self.connection.register_protocol_handler(protocol, self)

    def on_receive(self, frame):
        self.receive_queue.put(frame)

    def receive(self, block=True, timeout=None):
        try:
            return self.receive_queue.get(block, timeout)
        except Queue.Empty:
            raise exceptions.ReceiveQueueEmpty

    def send(self, frame):
        self.connection.send(self.protocol, frame)

    @property
    def mtu(self):
        return self.connection.mtu


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    with Connection.open_dbgserial(sys.argv[1]) as sock:
        sock.change_baud_rate(921600)
        for _ in range(20):
            time.sleep(0.5)
            send_time = time.time()
            if sock.ping():
                print("Ping rtt=%.2f ms" % ((time.time() - send_time) * 1000))
            else:
                print("No echo")
