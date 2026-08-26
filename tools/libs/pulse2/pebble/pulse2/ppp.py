# SPDX-FileCopyrightText: 2024 Google LLC
# SPDX-License-Identifier: Apache-2.0

"""Implementation of the Point-to-Point Protocol.

Yes, /that/ point-to-point protocol, RFC 1661. All of it. It would not
be unreasonable to describe the lower layers of PULSEv2 as PPP in a
custom framing.
"""


import collections
import enum
import functools
import itertools
import logging
import struct
import threading

import construct
from transitions.extensions import LockedMachine

from . import exceptions
from . import logging as pulse2_logging

logger = logging.getLogger(__name__)
logger.addHandler(logging.NullHandler())


class PPPException(exceptions.PulseException):
    pass


class UnencapsulationError(PPPException):
    pass


class ParseError(PPPException):
    pass


class LinkStateError(PPPException):
    """An operation is not permitted in the current link state."""


def encapsulate(protocol, information):
    return struct.pack("!H", protocol) + bytes(information)


PPPFrame = collections.namedtuple("PPPFrame", "protocol information")


def unencapsulate(datagram):
    datagram = bytes(datagram)
    if len(datagram) < 2:
        raise UnencapsulationError("datagram too short")
    return PPPFrame(
        protocol=struct.unpack("!H", datagram[:2])[0], information=datagram[2:]
    )


def OptionalGreedyString(name):
    return construct.StringAdapter(
        construct.OptionalGreedyRange(construct.Field(name, 1))
    )


class Constructors:
    """Namespace for Construct parsers."""

    LCPPacket = construct.Struct(
        "LCPPacket",
        construct.Byte("code"),
        construct.Byte("identifier"),
        construct.UBInt16("length"),
        construct.Field("data", lambda ctx: ctx.length - 4),
        OptionalGreedyString("padding"),
    )

    Option = construct.Struct(
        "option",
        construct.UBInt8("type"),
        construct.NoneOf(construct.UBInt8("length"), [0, 1]),
        construct.Field("data", lambda ctx: ctx.length - 2),
    )

    OptionList = construct.Struct(
        "options",
        construct.Rename("options", construct.GreedyRange(Option)),
        construct.Terminator,
    )

    ProtocolReject = construct.Struct(
        "protocol_reject",
        construct.UBInt16("rejected_protocol"),
        OptionalGreedyString("rejected_information"),
    )

    MagicPlusData = construct.Struct(
        None,
        construct.UBInt32("magic_number"),
        OptionalGreedyString("data"),
    )


def _parse_or_reraise(parser, data):
    try:
        parsed_data = parser.parse(data)
    except (construct.ConstructError, ValueError) as e:
        raise ParseError(str(e))
    return parsed_data


class LCPEncapsulation(
    collections.namedtuple("LCPEncapsulation", "code identifier data padding")
):
    """Parse or construct an LCP packet with an arbitrary body."""

    __slots__ = ()

    header_length = 4

    @classmethod
    def parse(cls, packet):
        fields = _parse_or_reraise(Constructors.LCPPacket, packet)
        if len(fields.data) + cls.header_length != fields.length:
            raise ParseError("packet truncated or corrupt")
        return cls(fields.code, fields.identifier, fields.data, fields.padding)

    @classmethod
    def build(cls, code, identifier, data):
        return Constructors.LCPPacket.build(
            construct.Container(
                code=code,
                identifier=identifier,
                length=len(data) + cls.header_length,
                data=data,
                padding=b"",
            )
        )


Option = collections.namedtuple("Option", "type data")


class OptionList:
    """Parse or construct an option list as found in the body of
    Configure-* LCP packets.
    """

    def __init__(self):
        raise NotImplementedError

    @staticmethod
    def parse(data):
        if not data:
            return []
        options = _parse_or_reraise(Constructors.OptionList, data).options
        return [Option(opt.type, opt.data) for opt in options]

    @staticmethod
    def build(options):
        return b"".join(
            Constructors.Option.build(
                construct.Container(type=type_, length=len(data) + 2, data=data)
            )
            for type_, data in options
        )


class ProtocolReject(
    collections.namedtuple("ProtocolReject", "rejected_protocol rejected_information")
):
    """Parse or construct the body of a Protocol-Reject packet."""

    __slots__ = ()

    @classmethod
    def parse(cls, data):
        return cls(**_parse_or_reraise(Constructors.ProtocolReject, data))


class MagicNumberAndData(
    collections.namedtuple("MagicNumberAndData", "magic_number data")
):
    """Parse or construct a packet body that begins with a Magic-Number field.

    This is useful for Echo-Request, Echo-Reply, Discard-Request and
    Identification packets.
    """

    __slots__ = ()

    @classmethod
    def parse(cls, data):
        return cls(**_parse_or_reraise(Constructors.MagicPlusData, data))

    @staticmethod
    def build(magic_number, data):
        return Constructors.MagicPlusData.build(
            construct.Container(magic_number=magic_number, data=data)
        )


@enum.unique
class ControlCode(enum.Enum):
    """Code values shared by all Control Protocols"""

    Configure_Request = 1
    Configure_Ack = 2
    Configure_Nak = 3
    Configure_Reject = 4
    Terminate_Request = 5
    Terminate_Ack = 6
    Code_Reject = 7


@enum.unique
class LCPCode(enum.Enum):
    """LCP-specific Code values"""

    Protocol_Reject = 8
    Echo_Request = 9
    Echo_Reply = 10
    Discard_Request = 11
    Identification = 12


class ConfigurationAccepted:
    pass


ConfigurationAccepted = ConfigurationAccepted()


def flatten_transitions_table(transitions):
    flattened = []
    for action, table in transitions:
        for entry in table:
            if isinstance(entry, dict):
                modified_entry = dict(entry)
                modified_entry["trigger"] = action
                flattened.append(modified_entry)
            else:  # assuming list-like
                flattened.append([action] + entry)
    return flattened


class ControlProtocol:
    """The base PPP Control Protocol state machine.

    Only the states and events common to LCP and NCPs are implemented
    in this class. The concrete control protocol implementations are
    expected to inherit from this class and implement the specifics
    as required.

    This implementation is not reentrant! Care must be taken to ensure
    that methods are not called concurrently on different threads.
    """

    # The number of Terminate-Request packets sent without receiving a
    # Terminate-Ack before assuming that the peer is unable to respond.
    max_terminate = 2

    # Number of Configure-Request packets sent without receiving a
    # valid Configure-Ack, Configure-Nak or Configure-Reject before
    # assuming that the peer is unable to respond.
    max_configure = 10

    # Number of Configure-Nak packets sent without sending a
    # Configure-Ack before assuming that configuration is not
    # converging. TODO
    max_failure = 5

    # Restart timer expiry duration, in seconds.
    restart_timeout = 0.4

    # FIXME PBL-34320 proper MTU/MRU support
    mtu = 1500

    states = [
        # Lower layer is unavailable, no Open event has occurred.
        {"name": "Initial", "on_enter": "stop_restart_timer"},
        # Administrative open initiated, but lower layer unavailable.
        {"name": "Starting", "on_enter": "stop_restart_timer"},
        # Lower layer available, but no Open event.
        {"name": "Closed", "on_enter": "stop_restart_timer"},
        # Link is available, PPP connection has been terminated.
        {"name": "Stopped", "on_enter": "stop_restart_timer"},
        # An attempt has been made to terminate the connection.
        "Closing",
        # Similar to Closing.
        "Stopping",
        # An attempt is made to configure the connection.
        "Req-Sent",
        # Configure-Ack has been received, no Configure-Ack sent.
        "Ack-Rcvd",
        # Configure-Request sent, -Ack received, no -Ack sent.
        "Ack-Sent",
        # Configure-Ack both sent and received.
        {"name": "Opened", "on_enter": "stop_restart_timer"},
    ]

    # List of 2-tuples (trigger, transition-list)
    # where transition-list is a list of transitions in the same format
    # that the transitions LockedMachine constructor expects minus the trigger
    # field. Listing the same trigger for many transitions gets very
    # repetitive.
    transitions = flatten_transitions_table(
        [
            # The lower layer is ready to carry packets.
            (
                "_up",
                [
                    ["Initial", "Closed"],
                    {
                        "source": "Starting",
                        "dest": "Req-Sent",
                        "before": [
                            "init_configure_restart_count",
                            "send_configure_request",
                        ],
                    },
                ],
            ),
            # The lower layer is no longer available to carry packets.
            (
                "_down",
                [
                    ["Closed", "Initial"],
                    {
                        "source": "Stopped",
                        "dest": "Starting",
                        "before": "this_layer_started",
                    },
                    ["Closing", "Initial"],
                    [["Stopping", "Req-Sent", "Ack-Rcvd", "Ack-Sent"], "Starting"],
                    {
                        "source": "Opened",
                        "dest": "Starting",
                        "before": "this_layer_down",
                    },
                ],
            ),
            # The link is administratively allowed to be opened.
            (
                "open",
                [
                    {
                        "source": "Initial",
                        "dest": "Starting",
                        "before": "this_layer_started",
                    },
                    ["Starting", "Starting"],
                    {
                        "source": "Closed",
                        "dest": "Req-Sent",
                        "before": [
                            "init_configure_restart_count",
                            "send_configure_request",
                        ],
                    },
                    ["Stopped", "Stopped"],
                    ["Closing", "Stopping"],
                    ["Stopping", "Stopping"],
                    ["Req-Sent", "Req-Sent"],
                    ["Ack-Rcvd", "Ack-Rcvd"],
                    ["Ack-Sent", "Ack-Sent"],
                    ["Opened", "Opened"],
                ],
            ),
            # The link is not allowed to be opened.
            (
                "close",
                [
                    ["Initial", "Initial"],
                    {
                        "source": "Starting",
                        "dest": "Initial",
                        "before": "_this_layer_finished",
                    },
                    ["Closed", "Closed"],
                    ["Stopped", "Closed"],
                    ["Closing", "Closing"],
                    ["Stopping", "Closing"],
                    {
                        "source": ["Req-Sent", "Ack-Rcvd", "Ack-Sent"],
                        "dest": "Closing",
                        "before": [
                            "init_terminate_restart_count",
                            "send_terminate_request",
                        ],
                    },
                    {
                        "source": "Opened",
                        "dest": "Closing",
                        "before": [
                            "this_layer_down",
                            "init_terminate_restart_count",
                            "send_terminate_request",
                        ],
                    },
                ],
            ),
            # TO+ Event
            (
                "timeout_retry",
                [
                    {
                        "source": "Closing",
                        "dest": "Closing",
                        "before": "send_terminate_request",
                    },
                    {
                        "source": "Stopping",
                        "dest": "Stopping",
                        "before": "send_terminate_request",
                    },
                    {
                        "source": ["Req-Sent", "Ack-Rcvd"],
                        "dest": "Req-Sent",
                        "before": "retransmit_configure_request",
                    },
                    {
                        "source": "Ack-Sent",
                        "dest": "Ack-Sent",
                        "before": "retransmit_configure_request",
                    },
                ],
            ),
            # TO- Event
            (
                "timeout_giveup",
                [
                    {
                        "source": "Closing",
                        "dest": "Closed",
                        "before": "_this_layer_finished",
                    },
                    {
                        "source": ["Stopping", "Req-Sent", "Ack-Rcvd", "Ack-Sent"],
                        "dest": "Stopped",
                        "before": "_this_layer_finished",
                    },
                ],
            ),
            # RCR+ Event
            (
                "receive_configure_request_acceptable",
                [
                    {
                        "source": "Closed",
                        "dest": "Closed",
                        "before": "send_terminate_ack",
                    },
                    {
                        "source": "Stopped",
                        "dest": "Ack-Sent",
                        "before": [
                            "init_configure_restart_count",
                            "send_configure_request",
                            "send_configure_ack",
                        ],
                    },
                    ["Closing", "Closing"],
                    ["Stopping", "Stopping"],
                    {
                        "source": ["Req-Sent", "Ack-Sent"],
                        "dest": "Ack-Sent",
                        "before": "send_configure_ack",
                    },
                    {
                        "source": "Ack-Rcvd",
                        "dest": "Opened",
                        "before": ["send_configure_ack"],
                        "after": ["this_layer_up"],
                    },
                    {
                        "source": "Opened",
                        "dest": "Ack-Sent",
                        "before": [
                            "this_layer_down",
                            "send_configure_request",
                            "send_configure_ack",
                        ],
                    },
                ],
            ),
            # RCR- Event
            (
                "receive_configure_request_unacceptable",
                [
                    {
                        "source": "Closed",
                        "dest": "Closed",
                        "before": "send_terminate_ack",
                    },
                    {
                        "source": "Stopped",
                        "dest": "Req-Sent",
                        "before": [
                            "init_configure_restart_count",
                            "send_configure_request",
                            "send_configure_nak_or_rej",
                        ],
                    },
                    ["Closing", "Closing"],
                    ["Stopping", "Stopping"],
                    {
                        "source": ["Req-Sent", "Ack-Sent"],
                        "dest": "Req-Sent",
                        "before": "send_configure_nak_or_rej",
                    },
                    {
                        "source": "Ack-Rcvd",
                        "dest": "Ack-Rcvd",
                        "before": "send_configure_nak_or_rej",
                    },
                    {
                        "source": "Opened",
                        "dest": "Req-Sent",
                        "before": [
                            "this_layer_down",
                            "send_configure_request",
                            "send_configure_nak_or_rej",
                        ],
                    },
                ],
            ),
            # RCA Event
            (
                "receive_configure_ack",
                [
                    {
                        "source": "Closed",
                        "dest": "Closed",
                        "before": "send_terminate_ack",
                    },
                    {
                        "source": "Stopped",
                        "dest": "Stopped",
                        "before": "send_terminate_ack",
                    },
                    ["Closing", "Closing"],
                    ["Stopping", "Stopping"],
                    {
                        "source": "Req-Sent",
                        "dest": "Ack-Rcvd",
                        "before": "init_configure_restart_count",
                    },
                    {
                        "source": "Ack-Rcvd",
                        "dest": "Req-Sent",
                        "before": "send_configure_request",
                    },
                    {
                        "source": "Ack-Sent",
                        "dest": "Opened",
                        "before": ["init_configure_restart_count"],
                        "after": ["this_layer_up"],
                    },
                    {
                        "source": "Opened",
                        "dest": "Req-Sent",
                        "before": ["this_layer_down", "send_configure_request"],
                    },
                ],
            ),
            # RCN Event
            (
                "receive_configure_nak_or_rej",
                [
                    {
                        "source": "Closed",
                        "dest": "Closed",
                        "before": "send_terminate_ack",
                    },
                    {
                        "source": "Stopped",
                        "dest": "Stopped",
                        "before": "send_terminate_ack",
                    },
                    ["Closing", "Closing"],
                    ["Stopping", "Stopping"],
                    {
                        "source": "Req-Sent",
                        "dest": "Req-Sent",
                        "before": [
                            "init_configure_restart_count",
                            "send_configure_request",
                        ],
                    },
                    {
                        "source": "Ack-Rcvd",
                        "dest": "Req-Sent",
                        "before": "send_configure_request",
                    },
                    {
                        "source": "Ack-Sent",
                        "dest": "Ack-Sent",
                        "before": [
                            "init_configure_restart_count",
                            "send_configure_request",
                        ],
                    },
                    {
                        "source": "Opened",
                        "dest": "Req-Sent",
                        "before": ["this_layer_down", "send_configure_request"],
                    },
                ],
            ),
            # RTR Event
            (
                "receive_terminate_request",
                [
                    {
                        "source": "Closed",
                        "dest": "Closed",
                        "before": "send_terminate_ack",
                    },
                    {
                        "source": "Stopped",
                        "dest": "Stopped",
                        "before": "send_terminate_ack",
                    },
                    {
                        "source": "Closing",
                        "dest": "Closing",
                        "before": "send_terminate_ack",
                    },
                    {
                        "source": "Stopping",
                        "dest": "Stopping",
                        "before": "send_terminate_ack",
                    },
                    {
                        "source": ["Req-Sent", "Ack-Rcvd", "Ack-Sent"],
                        "dest": "Req-Sent",
                        "before": "send_terminate_ack",
                    },
                    {
                        "source": "Opened",
                        "dest": "Stopping",
                        "before": [
                            "this_layer_down",
                            "zero_restart_count",
                            "send_terminate_ack",
                        ],
                    },
                ],
            ),
            # RTA Event
            (
                "receive_terminate_ack",
                [
                    ["Closed", "Closed"],
                    ["Stopped", "Stopped"],
                    {
                        "source": "Closing",
                        "dest": "Closed",
                        "before": "_this_layer_finished",
                    },
                    {
                        "source": "Stopping",
                        "dest": "Stopped",
                        "before": "_this_layer_finished",
                    },
                    [["Req-Sent", "Ack-Rcvd"], "Req-Sent"],
                    {
                        "source": "Opened",
                        "dest": "Req-Sent",
                        "before": ["this_layer_down", "send_configure_request"],
                    },
                ],
            ),
            # The RUC event is intentionally left out of the state table.
            # Since that event never triggers a state transition, it is
            # handled as a special case in the code.
            # RXJ+ Event
            (
                "receive_code_reject_permitted",
                [
                    ["Closed", "Closed"],
                    ["Stopped", "Stopped"],
                    ["Closing", "Closing"],
                    ["Stopping", "Stopping"],
                    [["Req-Sent", "Ack-Rcvd"], "Req-Sent"],
                    ["Ack-Sent", "Ack-Sent"],
                    ["Opened", "Opened"],
                ],
            ),
            # RXJ- Event
            (
                "receive_code_reject_catastrophic",
                [
                    {
                        "source": ["Closed", "Closing"],
                        "dest": "Closed",
                        "before": "_this_layer_finished",
                    },
                    {
                        "source": [
                            "Stopped",
                            "Stopping",
                            "Req-Sent",
                            "Ack-Rcvd",
                            "Ack-Sent",
                        ],
                        "dest": "Stopped",
                        "before": "_this_layer_finished",
                    },
                    {
                        "source": "Opened",
                        "dest": "Stopping",
                        "before": [
                            "this_layer_down",
                            "init_terminate_restart_count",
                            "send_terminate_request",
                        ],
                    },
                ],
            ),
            # There are no transitions for RXR events because none of the
            # packets which trigger that event are supported by the base
            # Control Protocol state machine.
        ]
    )

    def __init__(self, display_name=None):
        if not display_name:
            display_name = type(self).__name__
        self.logger = pulse2_logging.TaggedAdapter(logger, {"tag": display_name})
        self.machine = LockedMachine(
            model=self,
            states=self.states,
            initial="Initial",
            transitions=self.transitions,
            name=display_name,
            ignore_invalid_triggers=True,
        )
        self.is_finished = threading.Event()
        self.restart_timer = None
        self.restart_timer_generation_id = 0
        self.restart_count = 0
        self.configure_fail_count = self.max_failure
        self.configure_request_identifier = 255
        self.code_reject_identifier = itertools.cycle(range(256))
        self.configure_options = None
        self.socket = None

    def restart(self):
        with self.machine.rlock:
            self.logger.debug("Restarting: down...")
            self._down()
            self.logger.debug("Restarting: up...")
            self._up()
            self.logger.debug("Restarting: done")

    def up(self, socket):
        with self.machine.rlock:
            self.socket = socket
            self.socket.on_packet = self.packet_received
            self.socket.on_close = self.down
            self._up()

    def down(self):
        with self.machine.rlock:
            self._down()
            if self.socket:
                self.socket.on_close = None
                self.socket.close()
                self.socket = None

    def shutdown(self):
        """Gracefully close the link and block until the link is finished."""
        with self.machine.rlock:
            if self.state in ("Initial", "Starting", "Closed", "Stopped"):
                # Already finished; nothing to do.
                return
            self.is_finished.clear()
        self.close()
        self.is_finished.wait()

    # Restart timer

    def start_restart_timer(self, timeout):
        with self.machine.rlock:
            self.logger.debug("Starting restart timer")
            if self.restart_timer is not None:
                self.restart_timer.cancel()
                self.restart_timer_generation_id += 1
            self.restart_timer = threading.Timer(
                timeout,
                functools.partial(
                    self.restart_timer_expired, self.restart_timer_generation_id
                ),
            )
            self.restart_timer.daemon = True
            self.restart_timer.start()

    def stop_restart_timer(self, *args):
        with self.machine.rlock:
            self.logger.debug("Stopping restart timer")
            if self.restart_timer is not None:
                self.restart_timer.cancel()
                self.restart_timer = None
                self.restart_timer_generation_id += 1

    def _decrement_and_start_restart_timer(self):
        self.restart_count -= 1
        assert self.restart_count >= 0
        self.start_restart_timer(self.restart_timeout)

    def restart_timer_expired(self, generation_id):
        with self.machine.rlock:
            if self.restart_timer_generation_id != generation_id:
                return
            if self.restart_count > 0:
                self.timeout_retry()
            else:
                self.timeout_giveup()

    # Actions

    def this_layer_up(self, *args):
        """Signal to upper layers that the automaton is entering the
        Opened state.

        Subclasses should override this method.
        """

    def this_layer_down(self, *args):
        """Signal to upper layers that the automaton is leaving the
        Opened state.

        Subclasses should override this method.
        """

    def this_layer_started(self):
        """Signal to lower layers that the automaton is entering the
        Starting state and that the lower layer is needed for the link.

        Subclasses should override this method.
        """

    def this_layer_finished(self):
        """Signal to lower layers that the lower layer is no longer
        needed for the link.

        Subclasses should override this method.
        """

    def send_packet(self, packet):
        """Send a packet out to the lower layer.

        :param packet: packet to send
        :type packet: bytes
        """
        self.socket.send(packet)

    # Actions handled internally

    def _this_layer_finished(self):
        self.is_finished.set()
        self.this_layer_finished()

    def init_configure_restart_count(self, *args, **kwargs):
        self.restart_count = self.max_configure

    def init_terminate_restart_count(self):
        self.restart_count = self.max_terminate

    def zero_restart_count(self):
        self.restart_count = 0
        self.start_restart_timer(self.restart_timeout)

    def send_configure_request(self, *args):
        self._decrement_and_start_restart_timer()
        self.configure_request_identifier = (
            self.configure_request_identifier + 1
        ) % 256
        options = OptionList.build(self.get_configure_request_options())
        packet = LCPEncapsulation.build(
            ControlCode.Configure_Request.value,
            self.configure_request_identifier,
            options,
        )
        self.last_sent_configure_options = options
        self.last_sent_configure_request = packet
        self.send_packet(packet)

    def retransmit_configure_request(self):
        self._decrement_and_start_restart_timer()
        self.send_packet(self.last_sent_configure_request)

    def send_configure_ack(self, identifier, options):
        self.send_packet(
            LCPEncapsulation.build(
                ControlCode.Configure_Ack.value, identifier, OptionList.build(options)
            )
        )

    def send_configure_nak_or_rej(self):
        if self.configure_fail_count <= 0:
            # TODO convert nak to reject; strip out locally-desired
            # options.
            # FIXME find an appropriate place to reinitialize the fail
            # count.
            pass
        raise NotImplementedError

    def send_terminate_request(self):
        self._decrement_and_start_restart_timer()
        # FIXME identifier
        self.send_packet(
            LCPEncapsulation.build(ControlCode.Terminate_Request.value, 42, b"")
        )

    def send_terminate_ack(self, *args):
        # FIXME identifier
        self.send_packet(
            LCPEncapsulation.build(ControlCode.Terminate_Ack.value, 42, b"")
        )

    def send_code_reject(self, rejected_packet):
        # Truncate rejected_packet to fit within the link MTU
        max_length = self.mtu - LCPEncapsulation.header_length
        assert max_length > 0
        rejected_packet = rejected_packet[:max_length]
        self.send_packet(
            LCPEncapsulation.build(
                ControlCode.Code_Reject.value,
                next(self.code_reject_identifier),
                rejected_packet,
            )
        )

    # Events not handled by the state table

    def packet_received(self, packet):
        """The lower layer must call this method whenever a packet
        is received which is addressed to this protocol.

        The packet must already have any lower layer headers (including
        the protocol number) removed.
        """
        with self.machine.rlock:
            self._packet_received(packet)

    def _packet_received(self, packet):
        if self.state in ("Initial", "Starting"):
            # No packets should be received in these states.
            self.logger.info("Received unexpected packet in state %s", self.state)
            return

        try:
            encapsulation = LCPEncapsulation.parse(packet)
        except ParseError:
            self.logger.exception("Packet parsing failed")
            return

        try:
            code = ControlCode(encapsulation.code)
        except ValueError:
            result = self.handle_unknown_code(
                encapsulation.code, encapsulation.identifier, encapsulation.data
            )
            if result is NotImplemented:
                # Receive-Unknown-Code
                self.send_code_reject(packet)

            return

        if code in (
            ControlCode.Configure_Request,
            ControlCode.Configure_Ack,
            ControlCode.Configure_Nak,
            ControlCode.Configure_Reject,
        ):
            if self.state in ("Closing", "Stopping"):
                # Waiting for Terminate-Ack; ignoring configure requests.
                return
            try:
                options = OptionList.parse(encapsulation.data)
            except ParseError:
                self.logger.exception("Parsing option list failed")
                return

            if code == ControlCode.Configure_Request:
                self._handle_configure_request(encapsulation.identifier, options)
            else:
                if encapsulation.identifier != self.configure_request_identifier:
                    # Invalid packet; silently discard
                    self.logger.info(
                        "Received response packet with mismatched "
                        "identifier: expected %(expected)d, "
                        "received %(received)d",
                        {
                            "expected": self.configure_request_identifier,
                            "received": encapsulation.identifier,
                        },
                    )
                    return
                elif code == ControlCode.Configure_Ack:
                    if encapsulation.data == self.last_sent_configure_options:
                        self.receive_configure_ack(options)
                    else:
                        # Invalid packet; silently discard
                        self.logger.warning(
                            "Received Configure-Ack with mismatched options"
                        )
                        return
                elif code == ControlCode.Configure_Nak:
                    self.handle_configure_nak(options)
                    self.receive_configure_nak_or_reject()
                elif code == ControlCode.Configure_Reject:
                    self.handle_configure_reject(options)
                    self.receive_configure_nak_or_reject()
                else:
                    assert False, "impossible state"
        elif code == ControlCode.Terminate_Request:
            self.receive_terminate_request()
        elif code == ControlCode.Terminate_Ack:
            self.receive_terminate_ack()
        elif code == ControlCode.Code_Reject:
            try:
                rejected_packet = LCPEncapsulation.parse(encapsulation.data)
            except ParseError:
                # Invalid packet
                self.logger.exception("Error parsing Code-Reject response")
                return
            try:
                ControlCode(rejected_packet.code)
                # Known code; not supporting any of these is catastrophic.
                self.logger.error(
                    "Remote peer rejected a packet with code %s; "
                    "the connection cannot proceed without this code "
                    "being supported",
                    rejected_packet.code,
                )
                self.receive_code_reject_catastrophic()
            except ValueError:
                # Unknown code
                try:
                    self.handle_code_reject(rejected_packet)
                except CodeRejectCatastrophic:
                    self.logger.exception(
                        "Remote peer rejected a packet which must be "
                        "supported for the connection to proceed"
                    )
                    self.receive_code_reject_catastrophic()
        else:
            assert False, "known code not handled"

    def _handle_configure_request(self, identifier, options):
        response = self.handle_incoming_configure_request(options)
        if response == ConfigurationAccepted:
            self.receive_configure_request_acceptable(identifier, options)
        else:
            # TODO assert that the response options have not been
            # reordered.
            self.receive_configure_request_unacceptable(identifier, response)

    # Negotiation of incoming options (remote configuration of local)
    def handle_incoming_configure_request(self, options):
        """Implementations will need to parse the options list and
        determine if the options are acceptable.

        If the complete set of options are acceptable, the
        implementation must configure itself according to the options,
        then return `ConfigurationAccepted`.

        If any of the options are unrecognizable, the implementation
        must return an instance of `ConfigurationRejected` containing
        all of the options that were not recognized, in the same order
        that they were received.

        If all of the options are recognized but contain unacceptable
        values, or if the implementation wants to request the
        negotiation of an option which the sender of the configuration
        request did not include, the implementation must return an
        instance of `ConfigurationNak` containing the options list that
        should be sent in a Configure-Nak packet (all acceptable options
        filtered out).
        """
        if not options:
            return ConfigurationAccepted
        return ConfigurationRejected(options)

    # Negotiation of outgoing options (configure remote peer)
    def get_configure_request_options(self):
        """Return the list of Options to be sent to the remote peer in
        the next Configure-Request packet.
        """
        return []

    def handle_configure_reject(self, rejected_options):
        """Handle options that were rejected by the remote peer.
        Implementations must keep track of state so that the next call
        to `get_configure_request_options` will reflect the rejected
        options.

        If the session cannot proceed because an option was rejected
        which the implementation requires be negotiated,
        `NegotiationFailure` should be raised.
        """

    def handle_configure_nak(self, unacceptable_options):
        """Handle options which were not acceptable by the remote peer.
        Implementations must update their configuration state so that
        the next call to `get_configure_request_options` will
        reflect the values that the remote peer has deemed unacceptable.
        """

    def handle_configure_accepted(self, options):
        """Handle the remote peer accepting the options list."""

    # Other protocol housekeeping
    def handle_unknown_code(self, code, identifier, data):
        """Handle a Control Protocol code number which is not recognized
        by the base control protocol implementation.
        Return `NotImplemented` if the code is also unrecognized by this
        handler, which will trigger the sending of an Unknown-Code
        packet in response.
        """
        return NotImplemented

    def handle_code_reject(self, rejected_packet):
        """Handle a Code-Reject packet received from the peer containing
        a code which the base control protocol implementation does not
        recognize.

        Raise `CodeRejectCatastrophic` if a rejection of that code
        cannot be recovered from.
        """


class LinkControlProtocol(ControlProtocol):
    on_link_up = None
    on_link_down = None
    on_link_started = None
    on_link_finished = None

    def __init__(self, interface):
        self.interface = interface
        self.echo_request_identifier = itertools.cycle(range(256))
        self.last_sent_echo_request_identifier = None
        self.last_sent_echo_request_data = None
        self.ping_lock = threading.RLock()
        self.ping_cb = None
        self.ping_attempts_remaining = 0
        self.ping_timeout = None
        self.ping_timer = None
        ControlProtocol.__init__(self)

    def up(self):
        ControlProtocol.up(self, self.interface.connect(0xC021))

    def handle_unknown_code(self, code, identifier, data):
        try:
            code = LCPCode(code)
        except ValueError:
            return NotImplemented

        if code == LCPCode.Protocol_Reject:
            pass  # TODO tell NCP that it's been rejected
        elif code == LCPCode.Echo_Request:
            self._handle_echo_request(identifier, data)
        elif code == LCPCode.Echo_Reply:
            self._handle_echo_reply(identifier, data)
        elif code == LCPCode.Discard_Request:
            return
        elif code == LCPCode.Information:
            pass  # TODO
        else:
            assert False, "supported code not handled"

    def ping(self, result_cb, attempts=3, timeout=1.0):
        """Test the link quality by sending Echo-Request packets and
        listening for Echo-Response packets from the remote peer.

        The ping is performed asynchronously. The `result_cb` callable
        will be called when the ping completes. It will be called with
        a single positional argument: a truthy value if the remote peer
        responded to the ping, or a falsy value if all ping attempts
        timed out.

        The remote peer will only respond if it also considers the link
        to be in the Opened state, i.e. that the link is ready to carry
        traffic.
        """
        if self.state != "Opened":
            raise LinkStateError("cannot ping when LCP is not Opened")
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
            self._send_echo_request(b"")
            self.ping_timer = threading.Timer(timeout, self._ping_timer_expired)
            self.ping_timer.daemon = True
            self.ping_timer.start()

    def _send_echo_request(self, data):
        self.last_sent_echo_request_identifier = next(self.echo_request_identifier)
        self.last_sent_echo_request_data = data
        self.send_packet(
            LCPEncapsulation.build(
                LCPCode.Echo_Request.value,
                self.last_sent_echo_request_identifier,
                MagicNumberAndData.build(magic_number=0, data=data),
            )
        )

    def _handle_echo_request(self, identifier, data):
        if self.state != "Opened":
            return
        try:
            request = MagicNumberAndData.parse(data)
            if request.magic_number != 0:
                # The Magic-Number option is not implemented, so an
                # Echo-Request packet MUST be transmitted with the
                # Magic-Number field set to zero. An Echo-Request with
                # any other value must therefore be malformed.
                self.logger.info(
                    "Received malformed Echo-Request packet: packet "
                    "contains nonzero Magic-Number value 0x%08x",
                    request.magic_number,
                )
                return
            self.send_packet(
                LCPEncapsulation.build(
                    LCPCode.Echo_Reply.value,
                    identifier,
                    MagicNumberAndData.build(magic_number=0, data=request.data),
                )
            )
        except ParseError:
            self.logger.exception("Error parsing Echo-Request packet")

    def _handle_echo_reply(self, identifier, data):
        if self.state != "Opened":
            return
        try:
            reply = MagicNumberAndData.parse(data)
            if reply.magic_number != 0:
                # The Magic-Number option is not implemented, so an
                # Echo-Reply packet MUST be transmitted with the
                # Magic-Number field set to zero. An Echo-Reply with
                # any other value must therefore be malformed.
                self.logger.info(
                    "Received malformed Echo-Reply packet: packet "
                    "contains nonzero Magic-Number value 0x%08x",
                    reply.magic_number,
                )
                return
            if (
                identifier != self.last_sent_echo_request_identifier
                or reply.data != self.last_sent_echo_request_data
            ):
                return
            with self.ping_lock:
                if self.ping_cb:
                    self.ping_timer.cancel()
                    self.ping_cb(True)
                    self.ping_cb = None
        except ParseError:
            self.logger.exception("Error parsing Echo-Reply packet")

    def _ping_timer_expired(self):
        with self.ping_lock:
            if not self.ping_cb:
                # _handle_echo_reply must have won the race
                return
            if self.ping_attempts_remaining:
                self.ping_attempts_remaining -= 1
                self._send_echo_request(b"")
                self.ping_timer = threading.Timer(
                    self.ping_timeout, self._ping_timer_expired
                )
                self.ping_timer.daemon = True
                self.ping_timer.start()
            else:
                self.ping_cb(False)
                self.ping_cb = None

    def this_layer_up(self, *args):
        self.logger.info("This layer up")
        if self.on_link_up:
            self.on_link_up()

    def this_layer_down(self, *args):
        self.logger.info("This layer down")
        with self.ping_lock:
            self.ping_cb = False
            if self.ping_timer:
                self.ping_timer.cancel()
        if self.on_link_down:
            self.on_link_down()

    def this_layer_started(self):
        self.logger.info("This layer started")
        if self.on_link_started:
            self.on_link_started()

    def this_layer_finished(self):
        self.logger.info("This layer finished")
        if self.on_link_finished:
            self.on_link_finished()
