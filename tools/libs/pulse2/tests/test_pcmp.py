# SPDX-FileCopyrightText: 2024 Google LLC
# SPDX-License-Identifier: Apache-2.0

import unittest

try:
    from unittest import mock
except ImportError:
    from unittest import mock

from pebble.pulse2 import pcmp

from .fake_timer import FakeTimer


class TestPCMP(unittest.TestCase):
    def setUp(self):
        self.uut = pcmp.PulseControlMessageProtocol(mock.Mock(), 1)

    def test_close_unregisters_the_socket(self):
        self.uut.close()
        self.uut.transport.unregister_socket.assert_called_once_with(1)

    def test_close_is_idempotent(self):
        self.uut.close()
        self.uut.close()
        self.assertEqual(1, self.uut.transport.unregister_socket.call_count)

    def test_send_unknown_code(self):
        self.uut.send_unknown_code(42)
        self.uut.transport.send.assert_called_once_with(1, b"\x82\x2a")

    def test_send_echo_request(self):
        self.uut.send_echo_request(b"abcdefg")
        self.uut.transport.send.assert_called_once_with(1, b"\x01abcdefg")

    def test_send_echo_reply(self):
        self.uut.send_echo_reply(b"abcdefg")
        self.uut.transport.send.assert_called_once_with(1, b"\x02abcdefg")

    def test_on_receive_empty_packet(self):
        self.uut.on_receive(b"")
        self.uut.transport.send.assert_not_called()

    def test_on_receive_message_with_unknown_code(self):
        self.uut.on_receive(b"\x00")
        self.uut.transport.send.assert_called_once_with(1, b"\x82\x00")

    def test_on_receive_malformed_unknown_code_message_1(self):
        self.uut.on_receive(b"\x82")
        self.uut.transport.send.assert_not_called()

    def test_on_receive_malformed_unknown_code_message_2(self):
        self.uut.on_receive(b"\x82\x00\x01")
        self.uut.transport.send.assert_not_called()

    def test_on_receive_discard_request(self):
        self.uut.on_receive(b"\x03")
        self.uut.transport.send.assert_not_called()

    def test_on_receive_discard_request_with_data(self):
        self.uut.on_receive(b"\x03asdfasdfasdf")
        self.uut.transport.send.assert_not_called()

    def test_on_receive_echo_request(self):
        self.uut.on_receive(b"\x01")
        self.uut.transport.send.assert_called_once_with(1, b"\x02")

    def test_on_receive_echo_request_with_data(self):
        self.uut.on_receive(b"\x01a")
        self.uut.transport.send.assert_called_once_with(1, b"\x02a")

    def test_on_receive_echo_reply(self):
        self.uut.on_receive(b"\x02")
        self.uut.transport.send.assert_not_called()

    def test_on_receive_echo_reply_with_data(self):
        self.uut.on_receive(b"\x02abc")
        self.uut.transport.send.assert_not_called()

    def test_on_receive_port_closed_with_no_handler(self):
        self.uut.on_receive(b"\x81\xab\xcd")
        self.uut.transport.send.assert_not_called()

    def test_on_receive_port_closed(self):
        self.uut.on_port_closed = mock.Mock()
        self.uut.on_receive(b"\x81\xab\xcd")
        self.uut.on_port_closed.assert_called_once_with(0xABCD)

    def test_on_receive_malformed_port_closed_message_1(self):
        self.uut.on_port_closed = mock.Mock()
        self.uut.on_receive(b"\x81\xab")
        self.uut.on_port_closed.assert_not_called()

    def test_on_receive_malformed_port_closed_message_2(self):
        self.uut.on_port_closed = mock.Mock()
        self.uut.on_receive(b"\x81\xab\xcd\xef")
        self.uut.on_port_closed.assert_not_called()


class TestPing(unittest.TestCase):
    def setUp(self):
        FakeTimer.clear_timer_list()
        timer_patcher = mock.patch("threading.Timer", new=FakeTimer)
        timer_patcher.start()
        self.addCleanup(timer_patcher.stop)
        self.uut = pcmp.PulseControlMessageProtocol(mock.Mock(), 1)

    def test_successful_ping(self):
        cb = mock.Mock()
        self.uut.ping(cb)
        self.uut.on_receive(b"\x02")
        cb.assert_called_once_with(True)
        self.assertFalse(FakeTimer.get_active_timers())

    def test_ping_succeeds_after_retry(self):
        cb = mock.Mock()
        self.uut.ping(cb, attempts=2)
        FakeTimer.TIMERS[-1].expire()
        self.uut.on_receive(b"\x02")
        cb.assert_called_once_with(True)
        self.assertFalse(FakeTimer.get_active_timers())

    def test_ping_succeeds_after_multiple_retries(self):
        cb = mock.Mock()
        self.uut.ping(cb, attempts=3)
        timer1 = FakeTimer.TIMERS[-1]
        timer1.expire()
        timer2 = FakeTimer.TIMERS[-1]
        self.assertIsNot(timer1, timer2)
        timer2.expire()
        self.uut.on_receive(b"\x02")
        cb.assert_called_once_with(True)
        self.assertFalse(FakeTimer.get_active_timers())

    def test_failed_ping(self):
        cb = mock.Mock()
        self.uut.ping(cb, attempts=1)
        FakeTimer.TIMERS[-1].expire()
        cb.assert_called_once_with(False)
        self.assertFalse(FakeTimer.get_active_timers())

    def test_ping_fails_after_multiple_retries(self):
        cb = mock.Mock()
        self.uut.ping(cb, attempts=3)
        for _ in range(3):
            FakeTimer.TIMERS[-1].expire()
        cb.assert_called_once_with(False)
        self.assertFalse(FakeTimer.get_active_timers())

    def test_socket_close_aborts_ping(self):
        cb = mock.Mock()
        self.uut.ping(cb, attempts=3)
        self.uut.close()
        cb.assert_not_called()
        self.assertFalse(FakeTimer.get_active_timers())
