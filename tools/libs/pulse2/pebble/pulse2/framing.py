# SPDX-FileCopyrightText: 2024 Google LLC
# SPDX-License-Identifier: Apache-2.0

"""
PULSEv2 Framing

This module handles encoding and decoding of datagrams in PULSEv2 frames: flag
delimiters, transparency encoding and Frame Check Sequence. The content of the
datagrams themselves are not examined or parsed.
"""


import binascii
import struct

try:
    import queue
except ImportError:  # Py2
    import Queue as queue

from cobs import cobs

FLAG = 0x55
CRC32_RESIDUE = binascii.crc32(b"\0" * 4)


class FramingException(Exception):
    pass


class DecodeError(FramingException):
    pass


class CorruptFrame(FramingException):
    pass


class FrameSplitter:
    """Takes a byte stream and partitions it into frames.

    Empty frames (two consecutive flag bytes) are silently discarded.
    No transparency conversion is applied to the contents of the frames.

    FrameSplitter objects support iteration for retrieving split frames.

    >>> splitter = FrameSplitter()
    >>> splitter.write(b'\x55foo\x55bar\x55')
    >>> list(splitter)
    [b'foo', b'bar']

    """

    def __init__(self, max_frame_length=0):
        self.frames = queue.Queue()
        self.input_buffer = bytearray()
        self.max_frame_length = max_frame_length
        self.waiting_for_sync = True

    def write(self, data):
        """Write bytes into the splitter for processing."""
        for char in bytearray(data):
            if self.waiting_for_sync:
                if char == FLAG:
                    self.waiting_for_sync = False
            else:
                if char == FLAG:
                    if self.input_buffer:
                        self.frames.put_nowait(bytes(self.input_buffer))
                        self.input_buffer = bytearray()
                else:
                    if (
                        not self.max_frame_length
                        or len(self.input_buffer) < self.max_frame_length
                    ):
                        self.input_buffer.append(char)
                    else:
                        self.input_buffer = bytearray()
                        self.waiting_for_sync = True

    def __iter__(self):
        while True:
            try:
                yield self.frames.get_nowait()
            except queue.Empty:
                return


def decode_transparency(frame_bytes):
    """Decode the transparency encoding applied to a PULSEv2 frame.

    Returns the decoded frame, or raises `DecodeError`.
    """
    frame_bytes = bytearray(frame_bytes)
    if FLAG in frame_bytes:
        raise DecodeError("flag byte in encoded frame")
    try:
        return cobs.decode(bytes(frame_bytes.replace(b"\0", bytearray([FLAG]))))
    except cobs.DecodeError as e:
        raise DecodeError(str(e))


def strip_fcs(frame_bytes):
    """Validates the FCS in a PULSEv2 frame.

    The frame is returned with the FCS removed if the FCS check passes.
    A `CorruptFrame` exception is raised if the FCS check fails.

    The frame must not be transparency-encoded.
    """
    if len(frame_bytes) <= 4:
        raise CorruptFrame("frame too short")
    if binascii.crc32(frame_bytes) != CRC32_RESIDUE:
        raise CorruptFrame("FCS check failure")
    return frame_bytes[:-4]


def decode_frame(frame_bytes):
    """Decode and validate a PULSEv2-encoded frame.

    Returns the datagram extracted from the frame, or raises a
    `FramingException` or subclass if there was an error decoding the frame.
    """
    return strip_fcs(decode_transparency(frame_bytes))


def encode_frame(datagram):
    """Encode a datagram in a PULSEv2 frame."""
    datagram = bytearray(datagram)
    fcs = binascii.crc32(datagram) & 0xFFFFFFFF
    fcs_bytes = struct.pack("<I", fcs)
    datagram.extend(fcs_bytes)
    flag = bytearray([FLAG])
    frame = cobs.encode(bytes(datagram)).replace(flag, b"\0")
    return flag + frame + flag
