# SPDX-FileCopyrightText: 2024 Google LLC
# SPDX-License-Identifier: Apache-2.0


import threading
import unittest

try:
    from unittest import mock
except ImportError:
    from unittest import mock

import construct
from pebble.pulse2 import exceptions, pcmp, transports

from . import timer_helper
from .fake_timer import FakeTimer

# Save a reference to the real threading.Timer for tests which need to
# use timers even while threading.Timer is patched with FakeTimer.
RealThreadingTimer = threading.Timer


class CommonTransportBeforeOpenedTestCases:
    def test_send_raises_exception(self):
        with self.assertRaises(exceptions.TransportNotReady):
            self.uut.send(0xDEAD, b"not gonna get through")

    def test_open_socket_returns_None_when_ncp_fails_to_open(self):
        self.assertIsNone(self.uut.open_socket(0xBEEF, timeout=0))


class CommonTransportTestCases:
    def test_send_raises_exception_after_transport_is_closed(self):
        self.uut.down()
        with self.assertRaises(exceptions.TransportNotReady):
            self.uut.send(0xAAAA, b"asdf")

    def test_socket_is_closed_when_transport_is_closed(self):
        socket = self.uut.open_socket(0xABCD, timeout=0)
        self.uut.down()
        self.assertTrue(socket.closed)
        with self.assertRaises(exceptions.SocketClosed):
            socket.send(b"foo")

    def test_opening_two_sockets_on_same_port_is_an_error(self):
        socket1 = self.uut.open_socket(0xABCD, timeout=0)
        with self.assertRaises(KeyError):
            socket2 = self.uut.open_socket(0xABCD, timeout=0)

    def test_closing_a_socket_allows_another_to_be_opened(self):
        socket1 = self.uut.open_socket(0xABCD, timeout=0)
        socket1.close()
        socket2 = self.uut.open_socket(0xABCD, timeout=0)

    def test_opening_socket_fails_after_transport_down(self):
        self.uut.this_layer_down()
        self.assertIsNone(self.uut.open_socket(0xABCD, timeout=0))

    def test_opening_socket_succeeds_after_transport_bounces(self):
        self.uut.this_layer_down()
        self.uut.this_layer_up()
        self.uut.open_socket(0xABCD, timeout=0)


class TestBestEffortTransportBeforeOpened(
    CommonTransportBeforeOpenedTestCases, unittest.TestCase
):
    def setUp(self):
        control_protocol_patcher = mock.patch(
            "pebble.pulse2.transports.TransportControlProtocol"
        )
        control_protocol_patcher.start()
        self.addCleanup(control_protocol_patcher.stop)
        self.uut = transports.BestEffortApplicationTransport(
            interface=mock.MagicMock(), link_mtu=1500
        )
        self.uut.ncp.is_Opened.return_value = False

    def test_open_socket_waits_for_ncp_to_open(self):
        self.uut.ncp.is_Opened.return_value = True

        def on_ping(cb, *args):
            self.uut.packet_received(
                transports.BestEffortPacket.build(
                    construct.Container(
                        port=0x0001, length=5, information=b"\x02", padding=b""
                    )
                )
            )
            cb(True)

        with mock.patch.object(pcmp.PulseControlMessageProtocol, "ping") as mock_ping:
            mock_ping.side_effect = on_ping
            open_thread = RealThreadingTimer(0.01, self.uut.this_layer_up)
            open_thread.daemon = True
            open_thread.start()
            self.assertIsNotNone(self.uut.open_socket(0xBEEF, timeout=0.5))
            open_thread.join()


class TestBestEffortTransport(CommonTransportTestCases, unittest.TestCase):
    def setUp(self):
        self.addCleanup(timer_helper.cancel_all_timers)
        self.uut = transports.BestEffortApplicationTransport(
            interface=mock.MagicMock(), link_mtu=1500
        )
        self.uut.ncp.receive_configure_request_acceptable(0, [])
        self.uut.ncp.receive_configure_ack()
        self.uut.packet_received(
            transports.BestEffortPacket.build(
                construct.Container(
                    port=0x0001, length=5, information=b"\x02", padding=b""
                )
            )
        )

    def test_send(self):
        self.uut.send(0xABCD, b"information")
        self.uut.link_socket.send.assert_called_with(
            transports.BestEffortPacket.build(
                construct.Container(
                    port=0xABCD, length=15, information=b"information", padding=b""
                )
            )
        )

    def test_send_from_socket(self):
        socket = self.uut.open_socket(0xABCD, timeout=0)
        socket.send(b"info")
        self.uut.link_socket.send.assert_called_with(
            transports.BestEffortPacket.build(
                construct.Container(
                    port=0xABCD, length=8, information=b"info", padding=b""
                )
            )
        )

    def test_receive_from_socket_with_empty_queue(self):
        socket = self.uut.open_socket(0xABCD, timeout=0)
        with self.assertRaises(exceptions.ReceiveQueueEmpty):
            socket.receive(block=False)

    def test_receive_from_socket(self):
        socket = self.uut.open_socket(0xABCD, timeout=0)
        self.uut.packet_received(
            transports.BestEffortPacket.build(
                construct.Container(
                    port=0xABCD, length=8, information=b"info", padding=b""
                )
            )
        )
        self.assertEqual(b"info", socket.receive(block=False))

    def test_receive_on_unopened_port_doesnt_reach_socket(self):
        socket = self.uut.open_socket(0xABCD, timeout=0)
        self.uut.packet_received(
            transports.BestEffortPacket.build(
                construct.Container(
                    port=0xFACE, length=8, information=b"info", padding=b""
                )
            )
        )
        with self.assertRaises(exceptions.ReceiveQueueEmpty):
            socket.receive(block=False)

    def test_receive_malformed_packet(self):
        self.uut.packet_received(b"garbage")

    def test_send_equal_to_mtu(self):
        self.uut.send(0xAAAA, b"a" * 1496)

    def test_send_greater_than_mtu(self):
        with self.assertRaisesRegex(ValueError, "Packet length"):
            self.uut.send(0xAAAA, b"a" * 1497)

    def test_transport_down_closes_link_socket_and_ncp(self):
        self.uut.down()
        self.uut.link_socket.close.assert_called_with()
        self.assertIsNone(self.uut.ncp.socket)

    def test_pcmp_port_closed_message_closes_socket(self):
        socket = self.uut.open_socket(0xABCD, timeout=0)
        self.assertFalse(socket.closed)
        self.uut.packet_received(
            transports.BestEffortPacket.build(
                construct.Container(
                    port=0x0001, length=7, information=b"\x81\xab\xcd", padding=b""
                )
            )
        )
        self.assertTrue(socket.closed)

    def test_pcmp_port_closed_message_without_socket(self):
        self.uut.packet_received(
            transports.BestEffortPacket.build(
                construct.Container(
                    port=0x0001, length=7, information=b"\x81\xaa\xaa", padding=b""
                )
            )
        )


class TestReliableTransportPacketBuilders(unittest.TestCase):
    def test_build_info_packet(self):
        self.assertEqual(
            b"\x1e\x3f\xbe\xef\x00\x14Data goes here",
            transports.build_reliable_info_packet(
                sequence_number=15,
                ack_number=31,
                poll=True,
                port=0xBEEF,
                information=b"Data goes here",
            ),
        )

    def test_build_receive_ready_packet(self):
        self.assertEqual(
            b"\x01\x18",
            transports.build_reliable_supervisory_packet(kind="RR", ack_number=12),
        )

    def test_build_receive_ready_poll_packet(self):
        self.assertEqual(
            b"\x01\x19",
            transports.build_reliable_supervisory_packet(
                kind="RR", ack_number=12, poll=True
            ),
        )

    def test_build_receive_ready_final_packet(self):
        self.assertEqual(
            b"\x01\x19",
            transports.build_reliable_supervisory_packet(
                kind="RR", ack_number=12, final=True
            ),
        )

    def test_build_receive_not_ready_packet(self):
        self.assertEqual(
            b"\x05\x18",
            transports.build_reliable_supervisory_packet(kind="RNR", ack_number=12),
        )

    def test_build_reject_packet(self):
        self.assertEqual(
            b"\x09\x18",
            transports.build_reliable_supervisory_packet(kind="REJ", ack_number=12),
        )


class TestReliableTransportBeforeOpened(
    CommonTransportBeforeOpenedTestCases, unittest.TestCase
):
    def setUp(self):
        self.addCleanup(timer_helper.cancel_all_timers)
        self.uut = transports.ReliableTransport(
            interface=mock.MagicMock(), link_mtu=1500
        )

    def test_open_socket_waits_for_ncp_to_open(self):
        self.uut.ncp.is_Opened = mock.Mock()
        self.uut.ncp.is_Opened.return_value = True
        self.uut.command_socket.send = lambda packet: self.uut.response_packet_received(
            transports.build_reliable_supervisory_packet(
                kind="RR", ack_number=0, final=True
            )
        )
        open_thread = RealThreadingTimer(0.01, self.uut.this_layer_up)
        open_thread.daemon = True
        open_thread.start()
        self.assertIsNotNone(self.uut.open_socket(0xBEEF, timeout=0.5))
        open_thread.join()


class TestReliableTransportConnectionEstablishment(unittest.TestCase):
    expected_rr_packet = transports.build_reliable_supervisory_packet(
        kind="RR", ack_number=0, poll=True
    )

    def setUp(self):
        FakeTimer.clear_timer_list()
        timer_patcher = mock.patch("threading.Timer", new=FakeTimer)
        timer_patcher.start()
        self.addCleanup(timer_patcher.stop)

        control_protocol_patcher = mock.patch(
            "pebble.pulse2.transports.TransportControlProtocol"
        )
        control_protocol_patcher.start()
        self.addCleanup(control_protocol_patcher.stop)

        self.uut = transports.ReliableTransport(
            interface=mock.MagicMock(), link_mtu=1500
        )
        assert isinstance(self.uut.ncp, mock.MagicMock)
        self.uut.ncp.is_Opened.return_value = True
        self.uut.this_layer_up()

    def send_rr_response(self):
        self.uut.response_packet_received(
            transports.build_reliable_supervisory_packet(
                kind="RR", ack_number=0, final=True
            )
        )

    def test_rr_packet_is_sent_after_this_layer_up_event(self):
        self.uut.command_socket.send.assert_called_once_with(self.expected_rr_packet)

    def test_rr_command_is_retransmitted_until_response_is_received(self):
        for _ in range(3):
            FakeTimer.TIMERS[-1].expire()
        self.send_rr_response()

        self.assertFalse(FakeTimer.get_active_timers())
        self.assertEqual(
            self.uut.command_socket.send.call_args_list,
            [mock.call(self.expected_rr_packet)] * 4,
        )
        self.assertIsNotNone(self.uut.open_socket(0xABCD, timeout=0))

    def test_transport_negotiation_restarts_if_no_responses(self):
        for _ in range(self.uut.max_retransmits):
            FakeTimer.TIMERS[-1].expire()
        self.assertFalse(FakeTimer.get_active_timers())
        self.assertIsNone(self.uut.open_socket(0xABCD, timeout=0))
        self.uut.ncp.restart.assert_called_once_with()


class TestReliableTransport(CommonTransportTestCases, unittest.TestCase):
    def setUp(self):
        FakeTimer.clear_timer_list()
        timer_patcher = mock.patch("threading.Timer", new=FakeTimer)
        timer_patcher.start()
        self.addCleanup(timer_patcher.stop)

        control_protocol_patcher = mock.patch(
            "pebble.pulse2.transports.TransportControlProtocol"
        )
        control_protocol_patcher.start()
        self.addCleanup(control_protocol_patcher.stop)

        self.uut = transports.ReliableTransport(
            interface=mock.MagicMock(), link_mtu=1500
        )
        assert isinstance(self.uut.ncp, mock.MagicMock)
        self.uut.ncp.is_Opened.return_value = True
        self.uut.this_layer_up()
        self.uut.command_socket.send.reset_mock()
        self.uut.response_packet_received(
            transports.build_reliable_supervisory_packet(
                kind="RR", ack_number=0, final=True
            )
        )

    def test_send_with_immediate_ack(self):
        self.uut.send(0xBEEF, b"Just some packet data")
        self.uut.command_socket.send.assert_called_once_with(
            transports.build_reliable_info_packet(
                sequence_number=0,
                ack_number=0,
                poll=True,
                port=0xBEEF,
                information=b"Just some packet data",
            )
        )
        self.assertEqual(1, len(FakeTimer.get_active_timers()))
        self.uut.response_packet_received(
            transports.build_reliable_supervisory_packet(
                kind="RR", ack_number=1, final=True
            )
        )
        self.assertTrue(all(t.cancelled for t in FakeTimer.TIMERS))

    def test_send_with_one_timeout_before_ack(self):
        self.uut.send(0xABCD, b"this will be sent twice")
        active_timers = FakeTimer.get_active_timers()
        self.assertEqual(1, len(active_timers))
        active_timers[0].expire()
        self.assertEqual(1, len(FakeTimer.get_active_timers()))
        self.uut.command_socket.send.assert_has_calls(
            [
                mock.call(
                    transports.build_reliable_info_packet(
                        sequence_number=0,
                        ack_number=0,
                        poll=True,
                        port=0xABCD,
                        information=b"this will be sent twice",
                    )
                )
            ]
            * 2
        )
        self.uut.response_packet_received(
            transports.build_reliable_supervisory_packet(
                kind="RR", ack_number=1, final=True
            )
        )
        self.assertTrue(all(t.cancelled for t in FakeTimer.TIMERS))

    def test_send_with_no_response(self):
        self.uut.send(0xD00D, b"blarg")
        for _ in range(self.uut.max_retransmits):
            FakeTimer.get_active_timers()[-1].expire()
        self.uut.ncp.restart.assert_called_once_with()

    def test_receive_info_packet(self):
        socket = self.uut.open_socket(0xCAFE, timeout=0)
        self.uut.command_packet_received(
            transports.build_reliable_info_packet(
                sequence_number=0,
                ack_number=0,
                poll=True,
                port=0xCAFE,
                information=b"info",
            )
        )
        self.assertEqual(b"info", socket.receive(block=False))
        self.uut.response_socket.send.assert_called_once_with(
            transports.build_reliable_supervisory_packet(
                kind="RR", ack_number=1, final=True
            )
        )

    def test_receive_duplicate_packet(self):
        socket = self.uut.open_socket(0xBA5E, timeout=0)
        packet = transports.build_reliable_info_packet(
            sequence_number=0,
            ack_number=0,
            poll=True,
            port=0xBA5E,
            information=b"all your base are belong to us",
        )
        self.uut.command_packet_received(packet)
        self.assertEqual(b"all your base are belong to us", socket.receive(block=False))
        self.uut.response_socket.reset_mock()
        self.uut.command_packet_received(packet)
        self.uut.response_socket.send.assert_called_once_with(
            transports.build_reliable_supervisory_packet(
                kind="RR", ack_number=1, final=True
            )
        )
        with self.assertRaises(exceptions.ReceiveQueueEmpty):
            socket.receive(block=False)

    def test_queueing_multiple_packets_to_send(self):
        packets = [
            (0xFEED, b"Some data"),
            (0x6789, b"More data"),
            (0xFEED, b"Third packet"),
        ]
        for protocol, information in packets:
            self.uut.send(protocol, information)

        for seq, (port, information) in enumerate(packets):
            self.uut.command_socket.send.assert_called_once_with(
                transports.build_reliable_info_packet(
                    sequence_number=seq,
                    ack_number=0,
                    poll=True,
                    port=port,
                    information=information,
                )
            )
            self.uut.command_socket.send.reset_mock()
            self.uut.response_packet_received(
                transports.build_reliable_supervisory_packet(
                    kind="RR", ack_number=seq + 1, final=True
                )
            )

    def test_send_equal_to_mtu(self):
        self.uut.send(0xAAAA, b"a" * 1494)

    def test_send_greater_than_mtu(self):
        with self.assertRaisesRegex(ValueError, "Packet length"):
            self.uut.send(0xAAAA, b"a" * 1496)

    def test_send_from_socket(self):
        socket = self.uut.open_socket(0xABCD, timeout=0)
        socket.send(b"info")
        self.uut.command_socket.send.assert_called_with(
            transports.build_reliable_info_packet(
                sequence_number=0,
                ack_number=0,
                poll=True,
                port=0xABCD,
                information=b"info",
            )
        )

    def test_receive_from_socket_with_empty_queue(self):
        socket = self.uut.open_socket(0xABCD, timeout=0)
        with self.assertRaises(exceptions.ReceiveQueueEmpty):
            socket.receive(block=False)

    def test_receive_from_socket(self):
        socket = self.uut.open_socket(0xABCD, timeout=0)
        self.uut.command_packet_received(
            transports.build_reliable_info_packet(
                sequence_number=0,
                ack_number=0,
                poll=True,
                port=0xABCD,
                information=b"info info info",
            )
        )
        self.assertEqual(b"info info info", socket.receive(block=False))

    def test_receive_on_unopened_port_doesnt_reach_socket(self):
        socket = self.uut.open_socket(0xABCD, timeout=0)
        self.uut.command_packet_received(
            transports.build_reliable_info_packet(
                sequence_number=0,
                ack_number=0,
                poll=True,
                port=0x3333,
                information=b"info",
            )
        )
        with self.assertRaises(exceptions.ReceiveQueueEmpty):
            socket.receive(block=False)

    def test_receive_malformed_command_packet(self):
        self.uut.command_packet_received(b"garbage")
        self.uut.ncp.restart.assert_called_once_with()

    def test_receive_malformed_response_packet(self):
        self.uut.response_packet_received(b"garbage")
        self.uut.ncp.restart.assert_called_once_with()

    def test_transport_down_closes_link_sockets_and_ncp(self):
        self.uut.down()
        self.uut.command_socket.close.assert_called_with()
        self.uut.response_socket.close.assert_called_with()
        self.uut.ncp.down.assert_called_with()

    def test_pcmp_port_closed_message_closes_socket(self):
        socket = self.uut.open_socket(0xABCD, timeout=0)
        self.assertFalse(socket.closed)
        self.uut.command_packet_received(
            transports.build_reliable_info_packet(
                sequence_number=0,
                ack_number=0,
                poll=True,
                port=0x0001,
                information=b"\x81\xab\xcd",
            )
        )
        self.assertTrue(socket.closed)

    def test_pcmp_port_closed_message_without_socket(self):
        self.uut.command_packet_received(
            transports.build_reliable_info_packet(
                sequence_number=0,
                ack_number=0,
                poll=True,
                port=0x0001,
                information=b"\x81\xaa\xaa",
            )
        )


class TestSocket(unittest.TestCase):
    def setUp(self):
        self.uut = transports.Socket(mock.Mock(), 1234)

    def test_empty_receive_queue(self):
        with self.assertRaises(exceptions.ReceiveQueueEmpty):
            self.uut.receive(block=False)

    def test_empty_receive_queue_blocking(self):
        with self.assertRaises(exceptions.ReceiveQueueEmpty):
            self.uut.receive(timeout=0.001)

    def test_receive(self):
        self.uut.on_receive(b"data")
        self.assertEqual(b"data", self.uut.receive(block=False))
        with self.assertRaises(exceptions.ReceiveQueueEmpty):
            self.uut.receive(block=False)

    def test_receive_twice(self):
        self.uut.on_receive(b"one")
        self.uut.on_receive(b"two")
        self.assertEqual(b"one", self.uut.receive(block=False))
        self.assertEqual(b"two", self.uut.receive(block=False))

    def test_receive_interleaved(self):
        self.uut.on_receive(b"one")
        self.assertEqual(b"one", self.uut.receive(block=False))
        self.uut.on_receive(b"two")
        self.assertEqual(b"two", self.uut.receive(block=False))

    def test_send(self):
        self.uut.send(b"data")
        self.uut.transport.send.assert_called_once_with(1234, b"data")

    def test_close(self):
        self.uut.close()
        self.uut.transport.unregister_socket.assert_called_once_with(1234)

    def test_send_after_close_is_an_error(self):
        self.uut.close()
        with self.assertRaises(exceptions.SocketClosed):
            self.uut.send(b"data")

    def test_receive_after_close_is_an_error(self):
        self.uut.close()
        with self.assertRaises(exceptions.SocketClosed):
            self.uut.receive(block=False)

    def test_blocking_receive_after_close_is_an_error(self):
        self.uut.close()
        with self.assertRaises(exceptions.SocketClosed):
            self.uut.receive(timeout=0.001)

    def test_close_during_blocking_receive_aborts_the_receive(self):
        thread_started = threading.Event()
        result = [None]

        def test_thread():
            thread_started.set()
            try:
                self.uut.receive(timeout=0.3)
            except Exception as e:
                result[0] = e

        thread = threading.Thread(target=test_thread)
        thread.daemon = True
        thread.start()
        assert thread_started.wait(timeout=0.5)
        self.uut.close()
        thread.join()
        self.assertIsInstance(result[0], exceptions.SocketClosed)

    def test_close_is_idempotent(self):
        self.uut.close()
        self.uut.close()
        self.assertEqual(1, self.uut.transport.unregister_socket.call_count)
