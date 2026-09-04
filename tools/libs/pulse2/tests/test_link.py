# SPDX-FileCopyrightText: 2024 Google LLC
# SPDX-License-Identifier: Apache-2.0


import time
import unittest

try:
    from unittest import mock
except ImportError:
    from unittest import mock

try:
    import queue
except ImportError:
    import Queue as queue

from pebble.pulse2 import exceptions, framing, link, ppp


class FakeIOStream:
    def __init__(self):
        self.read_queue = queue.Queue()
        self.write_queue = queue.Queue()
        self.closed = False

    def read(self, length):
        if self.closed:
            raise OSError("I/O operation on closed FakeIOStream")
        try:
            return self.read_queue.get(timeout=0.001)
        except queue.Empty:
            return b""

    def write(self, data):
        if self.closed:
            raise OSError("I/O operation on closed FakeIOStream")
        self.write_queue.put(data)

    def close(self):
        self.closed = True

    def pop_all_written_data(self):
        data = []
        try:
            while True:
                data.append(self.write_queue.get_nowait())
        except queue.Empty:
            pass
        return data


class TestInterface(unittest.TestCase):
    def setUp(self):
        self.iostream = FakeIOStream()
        self.uut = link.Interface(self.iostream)
        self.addCleanup(self.iostream.close)
        # Speed up test execution by overriding the LCP timeout
        self.uut.lcp.restart_timeout = 0.001
        self.uut.lcp.ping = self.fake_ping
        self.ping_should_succeed = True

    def fake_ping(self, cb, *args, **kwargs):
        cb(self.ping_should_succeed)

    def test_send_packet(self):
        self.uut.send_packet(0x8889, b"data")
        self.assertIn(
            framing.encode_frame(ppp.encapsulate(0x8889, b"data")),
            self.iostream.pop_all_written_data(),
        )

    def test_connect_returns_socket(self):
        self.assertIsNotNone(self.uut.connect(0xF0F1))

    def test_send_from_socket(self):
        socket = self.uut.connect(0xF0F1)
        socket.send(b"data")
        self.assertIn(
            framing.encode_frame(ppp.encapsulate(0xF0F1, b"data")),
            self.iostream.pop_all_written_data(),
        )

    def test_interface_closing_closes_sockets_and_iostream(self):
        socket1 = self.uut.connect(0xF0F1)
        socket2 = self.uut.connect(0xF0F3)
        self.uut.close()
        self.assertTrue(socket1.closed)
        self.assertTrue(socket2.closed)
        self.assertTrue(self.iostream.closed)

    def test_iostream_closing_closes_interface_and_sockets(self):
        socket = self.uut.connect(0xF0F1)
        self.iostream.close()
        time.sleep(0.01)  # Wait for receive thread to notice
        self.assertTrue(self.uut.closed)
        self.assertTrue(socket.closed)

    def test_opening_two_sockets_on_same_protocol_is_an_error(self):
        self.uut.connect(0xF0F1)
        with self.assertRaisesRegex(ValueError, "socket is already bound"):
            self.uut.connect(0xF0F1)

    def test_closing_socket_allows_another_to_be_opened(self):
        socket1 = self.uut.connect(0xF0F1)
        socket1.close()
        socket2 = self.uut.connect(0xF0F1)
        self.assertIsNot(socket1, socket2)

    def test_sending_from_closed_interface_is_an_error(self):
        self.uut.close()
        with self.assertRaisesRegex(ValueError, "closed interface"):
            self.uut.send_packet(0x8889, b"data")

    def test_get_link_returns_None_when_lcp_is_down(self):
        self.assertIsNone(self.uut.get_link(timeout=0))

    def test_get_link_from_closed_interface_is_an_error(self):
        self.uut.close()
        with self.assertRaisesRegex(ValueError, "closed interface"):
            self.uut.get_link(timeout=0)

    def test_get_link_when_lcp_is_up(self):
        self.uut.on_link_up()
        self.assertIsNotNone(self.uut.get_link(timeout=0))

    def test_link_object_is_closed_when_lcp_goes_down(self):
        self.uut.on_link_up()
        link = self.uut.get_link(timeout=0)
        self.assertFalse(link.closed)
        self.uut.on_link_down()
        self.assertTrue(link.closed)

    def test_lcp_bouncing_doesnt_reopen_old_link_object(self):
        self.uut.on_link_up()
        link1 = self.uut.get_link(timeout=0)
        self.uut.on_link_down()
        self.uut.on_link_up()
        link2 = self.uut.get_link(timeout=0)
        self.assertTrue(link1.closed)
        self.assertFalse(link2.closed)

    def test_close_gracefully_shuts_down_lcp(self):
        self.uut.lcp.receive_configure_request_acceptable(0, b"")
        self.uut.lcp.receive_configure_ack()
        self.uut.close()
        self.assertTrue(self.uut.lcp.is_finished.is_set())

    def test_ping_failure_triggers_lcp_restart(self):
        self.ping_should_succeed = False
        self.uut.lcp.restart = mock.Mock()
        self.uut.on_link_up()
        self.assertIsNone(self.uut.get_link(timeout=0))
        self.uut.lcp.restart.assert_called_once_with()


class TestInterfaceSocket(unittest.TestCase):
    def setUp(self):
        self.interface = mock.MagicMock()
        self.uut = link.InterfaceSocket(self.interface, 0xF2F1)

    def test_socket_is_not_closed_when_constructed(self):
        self.assertFalse(self.uut.closed)

    def test_send(self):
        self.uut.send(b"data")
        self.interface.send_packet.assert_called_once_with(0xF2F1, b"data")

    def test_close_sets_socket_as_closed(self):
        self.uut.close()
        self.assertTrue(self.uut.closed)

    def test_close_unregisters_socket_with_interface(self):
        self.uut.close()
        self.interface.unregister_socket.assert_called_once_with(0xF2F1)

    def test_close_calls_on_close_handler(self):
        on_close = mock.Mock()
        self.uut.on_close = on_close
        self.uut.close()
        on_close.assert_called_once_with()

    def test_send_after_close_is_an_error(self):
        self.uut.close()
        with self.assertRaises(exceptions.SocketClosed):
            self.uut.send(b"data")

    def test_handle_packet(self):
        self.uut.on_packet = mock.Mock()
        self.uut.handle_packet(b"data")
        self.uut.on_packet.assert_called_once_with(b"data")

    def test_handle_packet_does_not_call_on_packet_handler_after_close(self):
        on_packet = mock.Mock()
        self.uut.on_packet = on_packet
        self.uut.close()
        self.uut.handle_packet(b"data")
        on_packet.assert_not_called()

    def test_context_manager(self):
        with self.uut as uut:
            self.assertIs(self.uut, uut)
            self.assertFalse(self.uut.closed)
        self.assertTrue(self.uut.closed)

    def test_close_is_idempotent(self):
        on_close = mock.Mock()
        self.uut.on_close = on_close
        self.uut.close()
        self.uut.close()
        self.assertEqual(1, self.interface.unregister_socket.call_count)
        self.assertEqual(1, on_close.call_count)


class TestLink(unittest.TestCase):
    def setUp(self):
        transports_patcher = mock.patch.dict(
            link.Link.TRANSPORTS, {"fake": mock.Mock()}, clear=True
        )
        transports_patcher.start()
        self.addCleanup(transports_patcher.stop)

        self.uut = link.Link(mock.Mock(), 1500)

    def test_open_socket(self):
        socket = self.uut.open_socket(transport="fake", port=0xABCD, timeout=1.0)
        self.uut.transports["fake"].open_socket.assert_called_once_with(0xABCD, 1.0)
        self.assertIs(socket, self.uut.transports["fake"].open_socket())

    def test_down(self):
        self.uut.down()
        self.assertTrue(self.uut.closed)
        self.uut.transports["fake"].down.assert_called_once_with()

    def test_on_close_callback_when_going_down(self):
        self.uut.on_close = mock.Mock()
        self.uut.down()
        self.uut.on_close.assert_called_once_with()

    def test_open_socket_after_down_is_an_error(self):
        self.uut.down()
        with self.assertRaisesRegex(ValueError, "closed Link"):
            self.uut.open_socket("fake", 0xABCD)

    def test_open_socket_with_bad_transport_name(self):
        with self.assertRaisesRegex(KeyError, "Unknown transport 'bad'"):
            self.uut.open_socket("bad", 0xABCD)
