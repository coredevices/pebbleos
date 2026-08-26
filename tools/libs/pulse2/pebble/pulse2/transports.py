# SPDX-FileCopyrightText: 2024 Google LLC
# SPDX-License-Identifier: Apache-2.0


import logging
import threading
import time

try:
    import queue
except ImportError:
    import Queue as queue

import construct

from . import exceptions, pcmp, ppp, stats
from . import logging as pulse2_logging

logger = logging.getLogger(__name__)
logger.addHandler(logging.NullHandler())


class Socket:
    """A socket for sending and receiving packets over a single port
    of a PULSE2 transport.
    """

    def __init__(self, transport, port):
        self.transport = transport
        self.port = port
        self.closed = False
        self.receive_queue = queue.Queue()

    def on_receive(self, packet):
        self.receive_queue.put((True, packet))

    def receive(self, block=True, timeout=None):
        if self.closed:
            raise exceptions.SocketClosed("I/O operation on closed socket")
        try:
            info_good, info = self.receive_queue.get(block, timeout)
            if not info_good:
                assert self.closed
                raise exceptions.SocketClosed("Socket closed during receive")
            return info
        except queue.Empty:
            raise exceptions.ReceiveQueueEmpty

    def send(self, information):
        if self.closed:
            raise exceptions.SocketClosed("I/O operation on closed socket")
        self.transport.send(self.port, information)

    def close(self):
        if self.closed:
            return
        self.closed = True
        self.transport.unregister_socket(self.port)
        # Wake up the thread blocking on a receive (if any) so that it
        # can abort the receive quickly.
        self.receive_queue.put((False, None))

    @property
    def mtu(self):
        return self.transport.mtu


class TransportControlProtocol(ppp.ControlProtocol):
    def __init__(self, interface, transport, ncp_protocol, display_name=None):
        ppp.ControlProtocol.__init__(self, display_name)
        self.interface = interface
        self.ncp_protocol = ncp_protocol
        self.transport = transport

    def up(self):
        ppp.ControlProtocol.up(self, self.interface.connect(self.ncp_protocol))

    def this_layer_up(self, *args):
        self.transport.this_layer_up()

    def this_layer_down(self, *args):
        self.transport.this_layer_down()


BestEffortPacket = construct.Struct(
    "BestEffortPacket",
    construct.UBInt16("port"),
    construct.UBInt16("length"),
    construct.Field("information", lambda ctx: ctx.length - 4),
    ppp.OptionalGreedyString("padding"),
)


class BestEffortTransportBase:
    def __init__(self, interface, link_mtu):
        self.logger = pulse2_logging.TaggedAdapter(logger, {"tag": type(self).__name__})
        self.sockets = {}
        self.closed = False
        self._mtu = link_mtu - 4
        self.link_socket = interface.connect(self.PROTOCOL_NUMBER)
        self.link_socket.on_packet = self.packet_received

    def send(self, port, information):
        if len(information) > self.mtu:
            raise ValueError(
                "Packet length (%d) exceeds transport MTU (%d)"
                % (len(information), self.mtu)
            )
        packet = BestEffortPacket.build(
            construct.Container(
                port=port,
                length=len(information) + 4,
                information=information,
                padding=b"",
            )
        )
        self.link_socket.send(packet)

    def packet_received(self, packet):
        if self.closed:
            self.logger.warning("Received packet on closed transport")
            return
        try:
            fields = BestEffortPacket.parse(packet)
        except (construct.ConstructError, ValueError):
            self.logger.exception("Received malformed packet")
            return
        if len(fields.information) + 4 != fields.length:
            self.logger.error(
                "Received truncated or corrupt packet (expected %d, got %d data bytes)",
                fields.length - 4,
                len(fields.information),
            )
            return

        if fields.port in self.sockets:
            self.sockets[fields.port].on_receive(fields.information)
        else:
            self.logger.warning("Received packet for unopened port %04X", fields.port)

    def open_socket(self, port, factory=Socket):
        if self.closed:
            raise ValueError("Cannot open socket on closed transport")
        if port in self.sockets and not self.sockets[port].closed:
            raise KeyError("Another socket is already opened on port 0x%04x" % port)
        socket = factory(self, port)
        self.sockets[port] = socket
        return socket

    def unregister_socket(self, port):
        del self.sockets[port]

    def down(self):
        """Called by the Link when the link layer goes down.

        This closes the Transport object. Once closed, the Transport
        cannot be reopened.
        """
        self.closed = True
        self.close_all_sockets()
        self.link_socket.close()

    def close_all_sockets(self):
        # A socket could try to unregister itself when closing, which
        # would modify the sockets dict. Make a copy of the sockets
        # collection before closing them so that we are not iterating
        # over the dict when it could get modified.
        for socket in list(self.sockets.values()):
            socket.close()
        self.sockets = {}

    @property
    def mtu(self):
        return self._mtu


class BestEffortApplicationTransport(BestEffortTransportBase):
    NCP_PROTOCOL_NUMBER = 0xBA29
    PROTOCOL_NUMBER = 0x3A29

    def __init__(self, interface, link_mtu):
        BestEffortTransportBase.__init__(self, interface=interface, link_mtu=link_mtu)
        self.opened = threading.Event()
        self.ncp = TransportControlProtocol(
            interface=interface,
            transport=self,
            ncp_protocol=self.NCP_PROTOCOL_NUMBER,
            display_name="BestEffortControlProtocol",
        )
        self.ncp.up()
        self.ncp.open()

    def this_layer_up(self):
        # We can't let PCMP bind itself using the public open_socket
        # method as the method will block until self.opened is set, but
        # it won't be set until we use PCMP Echo to test that the
        # transport is ready to carry traffic. So we must manually bind
        # the port without waiting.
        self.pcmp = pcmp.PulseControlMessageProtocol(
            self, pcmp.PulseControlMessageProtocol.PORT
        )
        self.sockets[pcmp.PulseControlMessageProtocol.PORT] = self.pcmp
        self.pcmp.on_port_closed = self.on_port_closed
        self.pcmp.ping(self._ping_done)

    def _ping_done(self, ping_check_succeeded):
        # Don't need to do anything in the success case as receiving
        # any packet is enough to set the transport as Opened.
        if not ping_check_succeeded:
            self.logger.warning("Ping check failed. Restarting transport.")
            self.ncp.restart()

    def this_layer_down(self):
        self.opened.clear()
        self.close_all_sockets()

    def send(self, *args, **kwargs):
        if self.closed:
            raise exceptions.TransportNotReady("I/O operation on closed transport")
        if not self.ncp.is_Opened():
            raise exceptions.TransportNotReady(
                "I/O operation before transport is opened"
            )
        BestEffortTransportBase.send(self, *args, **kwargs)

    def packet_received(self, packet):
        if self.ncp.is_Opened():
            self.opened.set()
            BestEffortTransportBase.packet_received(self, packet)
        else:
            self.logger.warning(
                "Received packet before the transport is open. Discarding."
            )

    def open_socket(self, port, timeout=30.0, factory=Socket):
        if not self.opened.wait(timeout):
            return None
        return BestEffortTransportBase.open_socket(self, port, factory)

    def down(self):
        self.ncp.down()
        BestEffortTransportBase.down(self)

    def on_port_closed(self, closed_port):
        self.logger.info(
            "Remote peer says port 0x%04X is closed; closing socket", closed_port
        )
        try:
            self.sockets[closed_port].close()
        except KeyError:
            self.logger.exception("No socket is open on port 0x%04X!", closed_port)


class SimplexTransport(BestEffortTransportBase):
    PROTOCOL_NUMBER = 0x5021

    def __init__(self, interface):
        BestEffortTransportBase.__init__(self, interface=interface, link_mtu=0)

    def send(self, *args, **kwargs):
        raise NotImplementedError

    @property
    def mtu(self):
        return 0


ReliableInfoPacket = construct.Struct(
    "ReliableInfoPacket",
    # BitStructs are parsed MSBit-first
    construct.EmbeddedBitStruct(
        construct.BitField("sequence_number", 7),  # N(S) in LAPB
        construct.Const(construct.Bit("discriminator"), 0),
        construct.BitField("ack_number", 7),  # N(R) in LAPB
        construct.Flag("poll"),
    ),
    construct.UBInt16("port"),
    construct.UBInt16("length"),
    construct.Field("information", lambda ctx: ctx.length - 6),
    ppp.OptionalGreedyString("padding"),
)


ReliableSupervisoryPacket = construct.BitStruct(
    "ReliableSupervisoryPacket",
    construct.Const(construct.Nibble("reserved"), 0b0000),
    construct.Enum(
        construct.BitField("kind", 2),
        RR=0b00,
        RNR=0b01,
        REJ=0b10,
    ),
    construct.Const(construct.BitField("discriminator", 2), 0b01),
    construct.BitField("ack_number", 7),  # N(R) in LAPB
    construct.Flag("poll"),
    construct.Alias("final", "poll"),
)


def build_reliable_info_packet(sequence_number, ack_number, poll, port, information):
    return ReliableInfoPacket.build(
        construct.Container(
            sequence_number=sequence_number,
            ack_number=ack_number,
            poll=poll,
            port=port,
            information=information,
            length=len(information) + 6,
            discriminator=None,
            padding=b"",
        )
    )


def build_reliable_supervisory_packet(kind, ack_number, poll=False, final=False):
    return ReliableSupervisoryPacket.build(
        construct.Container(
            kind=kind,
            ack_number=ack_number,
            poll=poll or final,
            final=None,
            reserved=None,
            discriminator=None,
        )
    )


class ReliableTransport:
    """The reliable transport protocol, also known as TRAIN.

    The protocol is based on LAPB from ITU-T Recommendation X.25.
    """

    NCP_PROTOCOL_NUMBER = 0xBA33
    COMMAND_PROTOCOL_NUMBER = 0x3A33
    RESPONSE_PROTOCOL_NUMBER = 0x3A35

    MODULUS = 128

    max_retransmits = 10  # N2 system parameter in LAPB
    retransmit_timeout = 0.2  # T1 system parameter

    def __init__(self, interface, link_mtu):
        self.logger = pulse2_logging.TaggedAdapter(logger, {"tag": type(self).__name__})
        self.send_queue = queue.Queue()
        self.opened = threading.Event()
        self.closed = False
        self.last_sent_packet = None
        # The sequence number of the next in-sequence I-packet to be Tx'ed
        self.send_variable = 0  # V(S) in LAPB
        self.retransmit_count = 0
        self.waiting_for_ack = False
        self.last_ack_number = 0  # N(R) of the most recently received packet
        self.transmit_lock = threading.RLock()
        self.retransmit_timer = None

        # The expected sequence number of the next received I-packet
        self.receive_variable = 0  # V(R) in LAPB

        self.sockets = {}
        self._mtu = link_mtu - 6
        self.command_socket = interface.connect(self.COMMAND_PROTOCOL_NUMBER)
        self.response_socket = interface.connect(self.RESPONSE_PROTOCOL_NUMBER)
        self.command_socket.on_packet = self.command_packet_received
        self.response_socket.on_packet = self.response_packet_received
        self.ncp = TransportControlProtocol(
            interface=interface,
            transport=self,
            ncp_protocol=self.NCP_PROTOCOL_NUMBER,
            display_name="ReliableControlProtocol",
        )
        self.ncp.up()
        self.ncp.open()

    @property
    def mtu(self):
        return self._mtu

    def reset_stats(self):
        self.stats = {
            "info_packets_sent": 0,
            "info_packets_received": 0,
            "retransmits": 0,
            "out_of_order_packets": 0,
            "round_trip_time": stats.OnlineStatistics(),
        }
        self.last_packet_sent_time = None

    def this_layer_up(self):
        self.send_variable = 0
        self.receive_variable = 0
        self.retransmit_count = 0
        self.last_ack_number = 0
        self.waiting_for_ack = False
        self.reset_stats()
        # We can't let PCMP bind itself using the public open_socket
        # method as the method will block until self.opened is set, but
        # it won't be set until the peer sends us a packet over the
        # transport. But we want to bind the port without waiting.
        self.pcmp = pcmp.PulseControlMessageProtocol(
            self, pcmp.PulseControlMessageProtocol.PORT
        )
        self.sockets[pcmp.PulseControlMessageProtocol.PORT] = self.pcmp
        self.pcmp.on_port_closed = self.on_port_closed

        # Send an RR command packet to elicit an RR response from the
        # remote peer. Receiving a response from the peer confirms that
        # the transport is ready to carry traffic, at which point we
        # will allow applications to start opening sockets.
        self.send_supervisory_command(kind="RR", poll=True)
        self.start_retransmit_timer()

    def this_layer_down(self):
        self.opened.clear()
        if self.retransmit_timer:
            self.retransmit_timer.cancel()
            self.retransmit_timer = None
        self.close_all_sockets()
        self.logger.info(
            "Info packets sent=%d retransmits=%d",
            self.stats["info_packets_sent"],
            self.stats["retransmits"],
        )
        self.logger.info(
            "Info packets received=%d out-of-order=%d",
            self.stats["info_packets_received"],
            self.stats["out_of_order_packets"],
        )
        self.logger.info("Round-trip %s ms", self.stats["round_trip_time"])

    def open_socket(self, port, timeout=30.0, factory=Socket):
        if self.closed:
            raise ValueError("Cannot open socket on closed transport")
        if port in self.sockets and not self.sockets[port].closed:
            raise KeyError("Another socket is already opened on port 0x%04x" % port)
        if not self.opened.wait(timeout):
            return None
        socket = factory(self, port)
        self.sockets[port] = socket
        return socket

    def unregister_socket(self, port):
        del self.sockets[port]

    def down(self):
        self.closed = True
        self.close_all_sockets()
        self.command_socket.close()
        self.response_socket.close()
        self.ncp.down()

    def close_all_sockets(self):
        for socket in list(self.sockets.values()):
            socket.close()
        self.sockets = {}

    def on_port_closed(self, closed_port):
        self.logger.info(
            "Remote peer says port 0x%04X is closed; closing socket", closed_port
        )
        try:
            self.sockets[closed_port].close()
        except KeyError:
            self.logger.exception("No socket is open on port 0x%04X!", closed_port)

    def _send_info_packet(self, port, information):
        packet = build_reliable_info_packet(
            sequence_number=self.send_variable,
            ack_number=self.receive_variable,
            poll=True,
            port=port,
            information=information,
        )
        self.command_socket.send(packet)
        self.stats["info_packets_sent"] += 1
        self.last_packet_sent_time = time.time()

    def send(self, port, information):
        if self.closed:
            raise exceptions.TransportNotReady("I/O operation on closed transport")
        if not self.opened.is_set():
            raise exceptions.TransportNotReady(
                "Attempted to send a packet while the reliable transport is not open"
            )
        if len(information) > self.mtu:
            raise ValueError(
                "Packet length (%d) exceeds transport MTU (%d)"
                % (len(information), self.mtu)
            )
        self.send_queue.put((port, information))
        self.pump_send_queue()

    def process_ack(self, ack_number):
        with self.transmit_lock:
            if not self.waiting_for_ack:
                # Could be in the timer recovery condition (waiting for
                # a response to an RR Poll command). This is a bit
                # hacky and should probably be changed to use an
                # explicit state machine when this transport is
                # extended to support Go-Back-N ARQ.
                if self.retransmit_timer:
                    self.retransmit_timer.cancel()
                    self.retransmit_timer = None
                    self.retransmit_count = 0
            if (ack_number - 1) % self.MODULUS == self.send_variable:
                if self.retransmit_timer:
                    self.retransmit_timer.cancel()
                    self.retransmit_timer = None
                self.retransmit_count = 0
                self.waiting_for_ack = False
                self.send_variable = (self.send_variable + 1) % self.MODULUS
                if self.last_packet_sent_time:
                    self.stats["round_trip_time"].update(
                        (time.time() - self.last_packet_sent_time) * 1000
                    )

    def pump_send_queue(self):
        with self.transmit_lock:
            if not self.waiting_for_ack:
                try:
                    port, information = self.send_queue.get_nowait()
                    self.last_sent_packet = (port, information)
                    self.waiting_for_ack = True
                    self._send_info_packet(port, information)
                    self.start_retransmit_timer()
                except queue.Empty:
                    pass

    def start_retransmit_timer(self):
        if self.retransmit_timer:
            self.retransmit_timer.cancel()
        self.retransmit_timer = threading.Timer(
            self.retransmit_timeout, self.retransmit_timeout_expired
        )
        self.retransmit_timer.daemon = True
        self.retransmit_timer.start()

    def retransmit_timeout_expired(self):
        with self.transmit_lock:
            self.retransmit_count += 1
            if self.retransmit_count < self.max_retransmits:
                self.stats["retransmits"] += 1
                if self.last_sent_packet:
                    self._send_info_packet(*self.last_sent_packet)
                else:
                    # No info packet to retransmit; must be an RR command
                    # that needs to be retransmitted.
                    self.send_supervisory_command(kind="RR", poll=True)
                self.start_retransmit_timer()
            else:
                self.logger.warning("Reached maximum number of retransmit attempts")
                self.ncp.restart()

    def send_supervisory_command(self, kind, poll=False):
        with self.transmit_lock:
            command = build_reliable_supervisory_packet(
                kind=kind, poll=poll, ack_number=self.receive_variable
            )
            self.command_socket.send(command)

    def send_supervisory_response(self, kind, final=False):
        with self.transmit_lock:
            response = build_reliable_supervisory_packet(
                kind=kind, final=final, ack_number=self.receive_variable
            )
            self.response_socket.send(response)

    def command_packet_received(self, packet):
        if not self.ncp.is_Opened():
            self.logger.warning(
                "Received command packet before transport is open. Discarding."
            )
            return

        # Information packets have the LSBit of the first byte cleared.
        is_info = (bytearray(packet[0:1])[0] & 0b1) == 0
        try:
            if is_info:
                fields = ReliableInfoPacket.parse(packet)
            else:
                fields = ReliableSupervisoryPacket.parse(packet)
        except (construct.ConstructError, ValueError):
            self.logger.exception("Received malformed command packet")
            self.ncp.restart()
            return

        self.opened.set()

        if is_info:
            if fields.sequence_number == self.receive_variable:
                self.receive_variable = (self.receive_variable + 1) % self.MODULUS
                self.stats["info_packets_received"] += 1
                if len(fields.information) + 6 == fields.length:
                    if fields.port in self.sockets:
                        self.sockets[fields.port].on_receive(fields.information)
                    else:
                        self.logger.warning(
                            "Received packet on closed port %04X", fields.port
                        )
                else:
                    self.logger.error(
                        "Received truncated or corrupt info packet "
                        "(expected %d data bytes, got %d)",
                        fields.length - 6,
                        len(fields.information),
                    )
            else:
                self.stats["out_of_order_packets"] += 1
            self.send_supervisory_response(kind="RR", final=fields.poll)
        else:
            if fields.kind not in ("RR", "REJ"):
                self.logger.error(
                    "Received a %s command packet, which is not "
                    "yet supported by this implementation",
                    fields.kind,
                )
                # Pretend it is an RR packet
            self.process_ack(fields.ack_number)
            if fields.poll:
                self.send_supervisory_response(kind="RR", final=True)
            self.pump_send_queue()

    def response_packet_received(self, packet):
        if not self.ncp.is_Opened():
            self.logger.error(
                "Received response packet before transport is open. Discarding."
            )
            return

        # Information packets cannot be responses; we only need to
        # handle receiving Supervisory packets.
        try:
            fields = ReliableSupervisoryPacket.parse(packet)
        except (construct.ConstructError, ValueError):
            self.logger.exception("Received malformed response packet")
            self.ncp.restart()
            return

        self.opened.set()
        self.process_ack(fields.ack_number)
        self.pump_send_queue()

        if fields.kind not in ("RR", "REJ"):
            self.logger.error(
                "Received a %s response packet, which is not "
                "yet supported by this implementation.",
                fields.kind,
            )
