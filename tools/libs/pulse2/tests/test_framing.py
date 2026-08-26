# SPDX-FileCopyrightText: 2024 Google LLC
# SPDX-License-Identifier: Apache-2.0


import unittest

from pebble.pulse2 import framing


class TestEncodeFrame(unittest.TestCase):
    def test_empty_frame(self):
        # CRC-32 of nothing is 0
        # COBS encoding of b'\0\0\0\0' is b'\x01\x01\x01\x01\x01' (5 bytes)
        self.assertEqual(framing.encode_frame(b""), b"\x55\x01\x01\x01\x01\x01\x55")

    def test_simple_data(self):
        self.assertEqual(
            framing.encode_frame(b"abcdefg"), b"\x55\x0cabcdefg\xa6\x6a\x2a\x31\x55"
        )

    def test_flag_in_datagram(self):
        # ASCII 'U' is 0x55 hex
        self.assertEqual(
            framing.encode_frame(b"QUACK"), b"\x55\x0aQ\0ACK\xdf\x8d\x80\x74\x55"
        )

    def test_flag_in_fcs(self):
        # crc32(b'R') -> 0x5767df55
        # Since there is an \x55 byte in the FCS, it must be substituted,
        # just like when that byte value is present in the datagram itself.
        self.assertEqual(framing.encode_frame(b"R"), b"\x55\x06R\0\xdf\x67\x57\x55")


class TestFrameSplitter(unittest.TestCase):
    def setUp(self):
        self.splitter = framing.FrameSplitter()

    def test_basic_functionality(self):
        self.splitter.write(b"\x55abcdefg\x55foobar\x55asdf\x55")
        self.assertEqual(list(self.splitter), [b"abcdefg", b"foobar", b"asdf"])

    def test_wait_for_sync(self):
        self.splitter.write(b"garbage data\x55frame 1\x55")
        self.assertEqual(list(self.splitter), [b"frame 1"])

    def test_doubled_flags(self):
        self.splitter.write(b"\x55abcd\x55\x55efgh\x55")
        self.assertEqual(list(self.splitter), [b"abcd", b"efgh"])

    def test_multiple_writes(self):
        self.splitter.write(b"\x55ab")
        self.assertEqual(list(self.splitter), [])
        self.splitter.write(b"cd\x55")
        self.assertEqual(list(self.splitter), [b"abcd"])

    def test_lots_of_writes(self):
        for char in b"\x55abcd\x55ef":
            self.splitter.write(bytearray([char]))
        self.assertEqual(list(self.splitter), [b"abcd"])

    def test_iteration_pops_frames(self):
        self.splitter.write(b"\x55frame 1\x55frame 2\x55frame 3\x55")
        self.assertEqual(next(iter(self.splitter)), b"frame 1")
        self.assertEqual(list(self.splitter), [b"frame 2", b"frame 3"])

    def test_stopiteration_latches(self):
        # The iterator protocol requires that once an iterator raises
        # StopIteration, it must continue to do so for all subsequent calls
        # to its next() method.
        self.splitter.write(b"\x55frame 1\x55")
        iterator = iter(self.splitter)
        self.assertEqual(next(iterator), b"frame 1")
        with self.assertRaises(StopIteration):
            next(iterator)
            next(iterator)
        self.splitter.write(b"\x55frame 2\x55")
        with self.assertRaises(StopIteration):
            next(iterator)
        self.assertEqual(list(self.splitter), [b"frame 2"])

    def test_max_frame_length(self):
        splitter = framing.FrameSplitter(max_frame_length=6)
        splitter.write(b"\x5512345\x55123456\x551234567\x551234\x5512345678\x55")
        self.assertEqual(list(splitter), [b"12345", b"123456", b"1234"])

    def test_dynamic_max_length_1(self):
        self.splitter.write(b"\x5512345")
        self.splitter.max_frame_length = 6
        self.splitter.write(b"6\x551234567\x551234\x55")
        self.assertEqual(list(self.splitter), [b"123456", b"1234"])

    def test_dynamic_max_length_2(self):
        self.splitter.write(b"\x551234567")
        self.splitter.max_frame_length = 6
        self.splitter.write(b"89\x55123456\x55")
        self.assertEqual(list(self.splitter), [b"123456"])


class TestDecodeTransparency(unittest.TestCase):
    def test_easy_decode(self):
        self.assertEqual(framing.decode_transparency(b"\x06abcde"), b"abcde")

    def test_escaped_flag(self):
        self.assertEqual(framing.decode_transparency(b"\x06Q\0ACK"), b"QUACK")

    def test_flag_byte_in_frame(self):
        with self.assertRaises(framing.DecodeError):
            framing.decode_transparency(b"\x06ab\x55de")

    def test_truncated_cobs_block(self):
        with self.assertRaises(framing.DecodeError):
            framing.decode_transparency(b"\x0aabc")


class TestStripFCS(unittest.TestCase):
    def test_frame_too_short(self):
        with self.assertRaises(framing.CorruptFrame):
            framing.strip_fcs(b"abcd")

    def test_good_fcs(self):
        self.assertEqual(framing.strip_fcs(b"abcd\x11\xcd\x82\xed"), b"abcd")

    def test_frame_corrupted(self):
        with self.assertRaises(framing.CorruptFrame):
            framing.strip_fcs(b"abce\x11\xcd\x82\xed")

    def test_fcs_corrupted(self):
        with self.assertRaises(framing.CorruptFrame):
            framing.strip_fcs(b"abcd\x13\xcd\x82\xed")


class TestDecodeFrame(unittest.TestCase):
    def test_it_works(self):
        # Not much to test; decode_frame is just chained decode_transparency
        # with strip_fcs, and both of those have already been tested separately.
        self.assertEqual(framing.decode_frame(b"\x0aQ\0ACK\xdf\x8d\x80t"), b"QUACK")
