# SPDX-FileCopyrightText: 2024 Google LLC
# SPDX-License-Identifier: Apache-2.0


import unittest

try:
    from unittest import mock
except ImportError:
    from unittest import mock


from pebble.pulse2 import exceptions, ppp

from . import timer_helper
from .fake_timer import FakeTimer


class TestPPPEncapsulation(unittest.TestCase):
    def test_ppp_encapsulate(self):
        self.assertEqual(
            ppp.encapsulate(0xC021, b"Information"), b"\xc0\x21Information"
        )


class TestPPPUnencapsulate(unittest.TestCase):
    def test_ppp_unencapsulate(self):
        protocol, information = ppp.unencapsulate(b"\xc0\x21Information")
        self.assertEqual((protocol, information), (0xC021, b"Information"))

    def test_unencapsulate_empty_frame(self):
        with self.assertRaises(ppp.UnencapsulationError):
            ppp.unencapsulate(b"")

    def test_unencapsulate_too_short_frame(self):
        with self.assertRaises(ppp.UnencapsulationError):
            ppp.unencapsulate(b"\x21")

    def test_unencapsulate_empty_information(self):
        protocol, information = ppp.unencapsulate(b"\xc0\x21")
        self.assertEqual((protocol, information), (0xC021, b""))


class TestConfigurationOptionsParser(unittest.TestCase):
    def test_no_options(self):
        options = ppp.OptionList.parse(b"")
        self.assertEqual(len(options), 0)

    def test_one_empty_option(self):
        options = ppp.OptionList.parse(b"\xaa\x02")
        self.assertEqual(len(options), 1)
        self.assertEqual(options[0].type, 0xAA)
        self.assertEqual(options[0].data, b"")

    def test_one_option_with_length(self):
        options = ppp.OptionList.parse(b"\xab\x07Data!")
        self.assertEqual((0xAB, b"Data!"), options[0])

    def test_multiple_options_empty_first(self):
        options = ppp.OptionList.parse(b"\x22\x02\x23\x03a\x21\x04ab")
        self.assertEqual([(0x22, b""), (0x23, b"a"), (0x21, b"ab")], options)

    def test_multiple_options_dataful_first(self):
        options = ppp.OptionList.parse(b"\x31\x08option\x32\x02")
        self.assertEqual([(0x31, b"option"), (0x32, b"")], options)

    def test_option_with_length_too_short(self):
        with self.assertRaises(ppp.ParseError):
            ppp.OptionList.parse(b"\x41\x01")

    def test_option_list_with_malformed_option(self):
        with self.assertRaises(ppp.ParseError):
            ppp.OptionList.parse(b"\x0a\x02\x0b\x01\x0c\x03a")

    def test_truncated_terminal_option(self):
        with self.assertRaises(ppp.ParseError):
            ppp.OptionList.parse(b"\x61\x02\x62\x03a\x63\x0ccandleja")


class TestConfigurationOptionsBuilder(unittest.TestCase):
    def test_no_options(self):
        serialized = ppp.OptionList.build([])
        self.assertEqual(b"", serialized)

    def test_one_empty_option(self):
        serialized = ppp.OptionList.build([ppp.Option(0xAA, b"")])
        self.assertEqual(b"\xaa\x02", serialized)

    def test_one_option_with_length(self):
        serialized = ppp.OptionList.build([ppp.Option(0xBB, b"Data!")])
        self.assertEqual(b"\xbb\x07Data!", serialized)

    def test_two_options(self):
        serialized = ppp.OptionList.build(
            [ppp.Option(0xCC, b"foo"), ppp.Option(0xDD, b"xyzzy")]
        )
        self.assertEqual(b"\xcc\x05foo\xdd\x07xyzzy", serialized)


class TestLCPEnvelopeParsing(unittest.TestCase):
    def test_packet_no_padding(self):
        parsed = ppp.LCPEncapsulation.parse(b"\x01\xab\x00\x0aabcdef")
        self.assertEqual(parsed.code, 1)
        self.assertEqual(parsed.identifier, 0xAB)
        self.assertEqual(parsed.data, b"abcdef")
        self.assertEqual(parsed.padding, b"")

    def test_padding(self):
        parsed = ppp.LCPEncapsulation.parse(b"\x01\xab\x00\x0aabcdefpadding")
        self.assertEqual(parsed.data, b"abcdef")
        self.assertEqual(parsed.padding, b"padding")

    def test_truncated_packet(self):
        with self.assertRaises(ppp.ParseError):
            ppp.LCPEncapsulation.parse(b"\x01\xab\x00\x0aabcde")

    def test_bogus_length(self):
        with self.assertRaises(ppp.ParseError):
            ppp.LCPEncapsulation.parse(b"\x01\xbc\x00\x03")

    def test_empty_data(self):
        parsed = ppp.LCPEncapsulation.parse(b"\x03\x01\x00\x04")
        self.assertEqual((3, 1, b"", b""), parsed)


class TestLCPEnvelopeBuilder(unittest.TestCase):
    def test_build_empty_data(self):
        serialized = ppp.LCPEncapsulation.build(1, 0xFE, b"")
        self.assertEqual(b"\x01\xfe\x00\x04", serialized)

    def test_build_with_data(self):
        serialized = ppp.LCPEncapsulation.build(3, 0x2A, b"Hello, world!")
        self.assertEqual(b"\x03\x2a\x00\x11Hello, world!", serialized)


class TestProtocolRejectParsing(unittest.TestCase):
    def test_protocol_and_info(self):
        self.assertEqual(
            (0xABCD, b"asdfasdf"), ppp.ProtocolReject.parse(b"\xab\xcdasdfasdf")
        )

    def test_empty_info(self):
        self.assertEqual((0xF00D, b""), ppp.ProtocolReject.parse(b"\xf0\x0d"))

    def test_truncated_packet(self):
        with self.assertRaises(ppp.ParseError):
            ppp.ProtocolReject.parse(b"\xab")


class TestMagicNumberAndDataParsing(unittest.TestCase):
    def test_magic_and_data(self):
        self.assertEqual(
            (0xABCDEF01, b"datadata"),
            ppp.MagicNumberAndData.parse(b"\xab\xcd\xef\x01datadata"),
        )

    def test_magic_no_data(self):
        self.assertEqual(
            (0xFEEDFACE, b""), ppp.MagicNumberAndData.parse(b"\xfe\xed\xfa\xce")
        )

    def test_truncated_packet(self):
        with self.assertRaises(ppp.ParseError):
            ppp.MagicNumberAndData.parse(b"abc")


class TestMagicNumberAndDataBuilder(unittest.TestCase):
    def test_build_empty_data(self):
        serialized = ppp.MagicNumberAndData.build(0x12345678, b"")
        self.assertEqual(b"\x12\x34\x56\x78", serialized)

    def test_build_with_data(self):
        serialized = ppp.MagicNumberAndData.build(0xABCDEF01, b"foobar")
        self.assertEqual(b"\xab\xcd\xef\x01foobar", serialized)

    def test_build_with_named_attributes(self):
        serialized = ppp.MagicNumberAndData.build(magic_number=0, data=b"abc")
        self.assertEqual(b"\0\0\0\0abc", serialized)


class TestControlProtocolRestartTimer(unittest.TestCase):
    def setUp(self):
        FakeTimer.clear_timer_list()
        timer_patcher = mock.patch("threading.Timer", new=FakeTimer)
        timer_patcher.start()
        self.addCleanup(timer_patcher.stop)

        self.uut = ppp.ControlProtocol()
        self.uut.timeout_retry = mock.Mock()
        self.uut.timeout_giveup = mock.Mock()
        self.uut.restart_count = 5

    def test_timeout_event_called_if_generation_ids_match(self):
        self.uut.restart_timer_expired(self.uut.restart_timer_generation_id)
        self.uut.timeout_retry.assert_called_once_with()

    def test_timeout_event_not_called_if_generation_ids_mismatch(self):
        self.uut.restart_timer_expired(42)
        self.uut.timeout_retry.assert_not_called()
        self.uut.timeout_giveup.assert_not_called()

    def test_timeout_event_not_called_after_stopped(self):
        self.uut.start_restart_timer(1)
        self.uut.stop_restart_timer()
        FakeTimer.TIMERS[-1].expire()
        self.uut.timeout_retry.assert_not_called()
        self.uut.timeout_giveup.assert_not_called()

    def test_timeout_event_not_called_from_old_timer_after_restart(self):
        self.uut.start_restart_timer(1)
        zombie_timer = FakeTimer.get_active_timers()[-1]
        self.uut.start_restart_timer(1)
        zombie_timer.expire()
        self.uut.timeout_retry.assert_not_called()
        self.uut.timeout_giveup.assert_not_called()

    def test_timeout_event_called_only_once_after_restart(self):
        self.uut.start_restart_timer(1)
        self.uut.start_restart_timer(1)
        for timer in FakeTimer.TIMERS:
            timer.expire()
        self.uut.timeout_retry.assert_called_once_with()
        self.uut.timeout_giveup.assert_not_called()


class InstrumentedControlProtocol(ppp.ControlProtocol):
    methods_to_mock = ["this_layer_up", "this_layer_down", "this_layer_started", "this_layer_finished", "send_packet", "start_restart_timer", "stop_restart_timer"]
    attributes_to_mock = ("restart_timer",)

    def __init__(self):
        ppp.ControlProtocol.__init__(self)
        for method in self.methods_to_mock:
            setattr(self, method, mock.Mock())
        for attr in self.attributes_to_mock:
            setattr(self, attr, mock.NonCallableMock())


class ControlProtocolTestMixin:
    CONTROL_CODE_ENUM = ppp.ControlCode

    def _map_control_code(self, code):
        try:
            return int(code)
        except ValueError:
            return self.CONTROL_CODE_ENUM[code].value

    def assert_packet_sent(self, code, identifier, body=b""):
        self.fsm.send_packet.assert_called_once_with(
            ppp.LCPEncapsulation.build(self._map_control_code(code), identifier, body)
        )
        self.fsm.send_packet.reset_mock()

    def incoming_packet(self, code, identifier, body=b""):
        self.fsm.packet_received(
            ppp.LCPEncapsulation.build(self._map_control_code(code), identifier, body)
        )


class TestControlProtocolFSM(ControlProtocolTestMixin, unittest.TestCase):
    def setUp(self):
        self.addCleanup(timer_helper.cancel_all_timers)
        self.fsm = InstrumentedControlProtocol()

    def test_open_down(self):
        self.fsm.open()
        self.fsm.this_layer_started.assert_called_once_with()
        self.fsm.this_layer_up.assert_not_called()
        self.fsm.this_layer_down.assert_not_called()
        self.fsm.this_layer_finished.assert_not_called()

    def test_closed_up(self):
        self.fsm.up(mock.Mock())
        self.fsm.this_layer_up.assert_not_called()
        self.fsm.this_layer_down.assert_not_called()
        self.fsm.this_layer_started.assert_not_called()
        self.fsm.this_layer_finished.assert_not_called()

    def test_trivial_handshake(self):
        self.fsm.open()
        self.fsm.up(mock.Mock())
        self.assert_packet_sent("Configure_Request", 0)
        self.incoming_packet("Configure_Ack", 0)
        self.incoming_packet("Configure_Request", 17)
        self.assert_packet_sent("Configure_Ack", 17)
        self.assertEqual("Opened", self.fsm.state)
        self.assertTrue(self.fsm.this_layer_up.called)
        self.assertEqual(self.fsm.restart_count, self.fsm.max_configure)

    def test_terminate_cleanly(self):
        self.test_trivial_handshake()
        self.fsm.close()
        self.fsm.this_layer_down.assert_called_once_with()
        self.assert_packet_sent("Terminate_Request", 42)

    def test_remote_terminate(self):
        self.test_trivial_handshake()
        self.incoming_packet("Terminate_Request", 42)
        self.assert_packet_sent("Terminate_Ack", 42)
        self.assertTrue(self.fsm.this_layer_down.called)
        self.assertTrue(self.fsm.start_restart_timer.called)
        self.fsm.this_layer_finished.assert_not_called()
        self.fsm.restart_timer_expired(self.fsm.restart_timer_generation_id)
        self.assertTrue(self.fsm.this_layer_finished.called)
        self.assertEqual("Stopped", self.fsm.state)

    def test_remote_rejects_configure_request_code(self):
        self.fsm.open()
        self.fsm.up(mock.Mock())
        received_packet = self.fsm.send_packet.call_args[0][0]
        self.assert_packet_sent("Configure_Request", 0)
        self.incoming_packet("Code_Reject", 3, received_packet)
        self.assertEqual("Stopped", self.fsm.state)
        self.assertTrue(self.fsm.this_layer_finished.called)

    def test_receive_extended_code(self):
        self.fsm.handle_unknown_code = mock.Mock()
        self.test_trivial_handshake()
        self.incoming_packet(42, 11, b"Life, the universe and everything")
        self.fsm.handle_unknown_code.assert_called_once_with(
            42, 11, b"Life, the universe and everything"
        )

    def test_receive_unimplemented_code(self):
        self.test_trivial_handshake()
        self.incoming_packet(0x55, 0)
        self.assert_packet_sent("Code_Reject", 0, b"\x55\0\0\x04")

    def test_code_reject_truncates_rejected_packet(self):
        self.test_trivial_handshake()
        self.incoming_packet(0xAA, 0x20, b"a" * 1496)  # 1500-byte Info
        self.assert_packet_sent("Code_Reject", 0, b"\xaa\x20\x05\xdc" + b"a" * 1492)

    def test_code_reject_identifier_changes(self):
        self.test_trivial_handshake()
        self.incoming_packet(0xAA, 0)
        self.assert_packet_sent("Code_Reject", 0, b"\xaa\0\0\x04")
        self.incoming_packet(0xAA, 0)
        self.assert_packet_sent("Code_Reject", 1, b"\xaa\0\0\x04")

    # Local events: up, down, open, close
    # Option negotiation: reject, nak
    # Exceptional situations: catastrophic code-reject
    # Restart negotiation after opening
    # Remote Terminate-Req, -Ack at various points in the lifecycle
    # Negotiation infinite loop
    # Local side gives up on negotiation
    # Corrupt packets received


class TestLCPReceiveEchoRequest(ControlProtocolTestMixin, unittest.TestCase):
    CONTROL_CODE_ENUM = ppp.LCPCode

    def setUp(self):
        self.addCleanup(timer_helper.cancel_all_timers)
        self.fsm = ppp.LinkControlProtocol(mock.Mock())
        self.fsm.send_packet = mock.Mock()
        self.fsm.state = "Opened"

    def send_echo_request(self, identifier=0, data=b"\0\0\0\0"):
        result = self.fsm.handle_unknown_code(
            ppp.LCPCode.Echo_Request.value, identifier, data
        )
        self.assertIsNot(result, NotImplemented)

    def test_echo_request_is_dropped_when_not_in_opened_state(self):
        self.fsm.state = "Ack-Sent"
        self.send_echo_request()
        self.fsm.send_packet.assert_not_called()

    def test_echo_request_elicits_reply(self):
        self.send_echo_request()
        self.assert_packet_sent("Echo_Reply", 0, b"\0\0\0\0")

    def test_echo_request_with_data_is_echoed_in_reply(self):
        self.send_echo_request(5, b"\0\0\0\0datadata")
        self.assert_packet_sent("Echo_Reply", 5, b"\0\0\0\0datadata")

    def test_echo_request_missing_magic_number_field_is_dropped(self):
        self.send_echo_request(data=b"")
        self.fsm.send_packet.assert_not_called()

    def test_echo_request_with_nonzero_magic_number_is_dropped(self):
        self.send_echo_request(data=b"\0\0\0\x01")
        self.fsm.send_packet.assert_not_called()


class TestLCPPing(ControlProtocolTestMixin, unittest.TestCase):
    CONTROL_CODE_ENUM = ppp.LCPCode

    def setUp(self):
        FakeTimer.clear_timer_list()
        timer_patcher = mock.patch("threading.Timer", new=FakeTimer)
        timer_patcher.start()
        self.addCleanup(timer_patcher.stop)

        self.fsm = ppp.LinkControlProtocol(mock.Mock())
        self.fsm.send_packet = mock.Mock()
        self.fsm.state = "Opened"

    def respond_to_ping(self):
        [echo_request_packet], _ = self.fsm.send_packet.call_args
        self.assertEqual(b"\x09"[0], echo_request_packet[0])
        echo_response_packet = b"\x0a" + echo_request_packet[1:]
        self.fsm.packet_received(echo_response_packet)

    def test_ping_when_lcp_is_not_opened_is_an_error(self):
        cb = mock.Mock()
        self.fsm.state = "Ack-Rcvd"
        with self.assertRaises(ppp.LinkStateError):
            self.fsm.ping(cb)
        cb.assert_not_called()

    def test_zero_attempts_is_an_error(self):
        with self.assertRaises(ValueError):
            self.fsm.ping(mock.Mock(), attempts=0)

    def test_negative_attempts_is_an_error(self):
        with self.assertRaises(ValueError):
            self.fsm.ping(mock.Mock(), attempts=-1)

    def test_zero_timeout_is_an_error(self):
        with self.assertRaises(ValueError):
            self.fsm.ping(mock.Mock(), timeout=0)

    def test_negative_timeout_is_an_error(self):
        with self.assertRaises(ValueError):
            self.fsm.ping(mock.Mock(), timeout=-0.1)

    def test_straightforward_ping(self):
        cb = mock.Mock()
        self.fsm.ping(cb)
        cb.assert_not_called()
        self.assertEqual(1, self.fsm.send_packet.call_count)
        self.respond_to_ping()
        cb.assert_called_once_with(True)

    def test_one_timeout_before_responding(self):
        cb = mock.Mock()
        self.fsm.ping(cb, attempts=2)
        FakeTimer.TIMERS[-1].expire()
        cb.assert_not_called()
        self.assertEqual(2, self.fsm.send_packet.call_count)
        self.respond_to_ping()
        cb.assert_called_once_with(True)

    def test_one_attempt_with_no_reply(self):
        cb = mock.Mock()
        self.fsm.ping(cb, attempts=1)
        FakeTimer.TIMERS[-1].expire()
        self.assertEqual(1, self.fsm.send_packet.call_count)
        cb.assert_called_once_with(False)

    def test_multiple_attempts_with_no_reply(self):
        cb = mock.Mock()
        self.fsm.ping(cb, attempts=2)
        timer_one = FakeTimer.TIMERS[-1]
        timer_one.expire()
        timer_two = FakeTimer.TIMERS[-1]
        self.assertIsNot(timer_one, timer_two)
        timer_two.expire()
        self.assertEqual(2, self.fsm.send_packet.call_count)
        cb.assert_called_once_with(False)

    def test_late_reply(self):
        cb = mock.Mock()
        self.fsm.ping(cb, attempts=1)
        FakeTimer.TIMERS[-1].expire()
        self.respond_to_ping()
        cb.assert_called_once_with(False)

    def test_this_layer_down_during_ping(self):
        cb = mock.Mock()
        self.fsm.ping(cb)
        self.fsm.this_layer_down()
        FakeTimer.TIMERS[-1].expire()
        cb.assert_not_called()

    def test_echo_reply_with_wrong_identifier(self):
        cb = mock.Mock()
        self.fsm.ping(cb, attempts=1)
        [echo_request_packet], _ = self.fsm.send_packet.call_args
        echo_response_packet = bytearray(echo_request_packet)
        echo_response_packet[0] = 0x0A
        echo_response_packet[1] += 1
        self.fsm.packet_received(bytes(echo_response_packet))
        cb.assert_not_called()
        FakeTimer.TIMERS[-1].expire()
        cb.assert_called_once_with(False)

    def test_echo_reply_with_wrong_data(self):
        cb = mock.Mock()
        self.fsm.ping(cb, attempts=1)
        [echo_request_packet], _ = self.fsm.send_packet.call_args
        # Generate a syntactically valid Echo-Reply with the right
        # identifier but completely different data.
        identifier = bytearray(echo_request_packet)[1]
        echo_response_packet = bytes(
            b"\x0a"
            + bytearray([identifier])
            + b"\0\x26\0\0\0\0bad reply bad reply bad reply."
        )
        self.fsm.packet_received(bytes(echo_response_packet))
        cb.assert_not_called()
        FakeTimer.TIMERS[-1].expire()
        cb.assert_called_once_with(False)

    def test_successive_pings_use_different_identifiers(self):
        self.fsm.ping(mock.Mock(), attempts=1)
        [echo_request_packet_1], _ = self.fsm.send_packet.call_args
        identifier_1 = bytearray(echo_request_packet_1)[1]
        self.respond_to_ping()
        self.fsm.ping(mock.Mock(), attempts=1)
        [echo_request_packet_2], _ = self.fsm.send_packet.call_args
        identifier_2 = bytearray(echo_request_packet_2)[1]
        self.assertNotEqual(identifier_1, identifier_2)

    def test_unsolicited_echo_reply_doesnt_break_anything(self):
        self.fsm.packet_received(b"\x0a\0\0\x08\0\0\0\0")

    def test_malformed_echo_reply(self):
        cb = mock.Mock()
        self.fsm.ping(cb, attempts=1)
        # Only three bytes of Magic-Number
        self.fsm.packet_received(b"\x0a\0\0\x07\0\0\0")
        cb.assert_not_called()

    # Trying to start a second ping while the first ping is still happening
    def test_starting_a_ping_while_another_is_active_is_an_error(self):
        cb = mock.Mock()
        self.fsm.ping(cb, attempts=1)
        cb2 = mock.Mock()
        with self.assertRaises(exceptions.AlreadyInProgressError):
            self.fsm.ping(cb2, attempts=1)
        FakeTimer.TIMERS[-1].expire()
        cb.assert_called_once_with(False)
        cb2.assert_not_called()


# General tests:
# - Length too short for a valid packet
# - Packet truncated (length field > packet len)
# - Packet with padding

# OptionList codes:
#   1  Configure-Request
#   2  Configure-Ack
#   3  Configure-Nak
#   4  Configure-Reject

# Raw data codes:
#   5  Terminate-Request
#   6  Terminate-Ack
#   7  Code-Reject

# 8  Protocol-Reject
#   - Empty Rejected-Information field
#   - Rejected-Protocol field too short


# Magic number + data codes:
#   10 Echo-Reply
#   11 Discard-Request
#   12 Identification (RFC 1570)
