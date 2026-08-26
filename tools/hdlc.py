# SPDX-FileCopyrightText: 2024 Google LLC
# SPDX-License-Identifier: Apache-2.0

try:
    import Queue
except ImportError:
    # Py3 support
    import queue as Queue

HDLC_FRAME_START = 0x7E
HDLC_ESCAPE = 0x7D
HDLC_ESCAPE_MASK = 0x20


class HDLCDecoder:
    _STATE_SYNC, _STATE_DATA, _STATE_ESCAPE = range(3)

    def __init__(self):
        self._frames = Queue.Queue()
        self._state = self._STATE_SYNC
        self._buffer = bytearray()

    def write(self, data):
        for b in bytearray(data):
            if self._state == self._STATE_SYNC:
                # waiting for the first FRAME_START byte
                if b == HDLC_FRAME_START:
                    self._state = self._STATE_DATA
            elif self._state == self._STATE_DATA:
                if b == HDLC_FRAME_START:
                    # this is the end of the frame (and the start of the next one)
                    if self._buffer:
                        self._frames.put_nowait(bytes(self._buffer))
                    self._buffer = bytearray()
                elif b == HDLC_ESCAPE:
                    # escape the next byte
                    self._state = self._STATE_ESCAPE
                else:
                    # this a valid byte of data
                    self._buffer.append(b)
            elif self._state == self._STATE_ESCAPE:
                if b == HDLC_FRAME_START:
                    # invalid byte combination - drop this frame and start the next one
                    self._buffer = bytearray()
                else:
                    # escape this character
                    self._buffer.append(b ^ HDLC_ESCAPE_MASK)
                self._state = self._STATE_DATA
            else:
                assert False, "Invalid state!"

    def get_frame(self):
        try:
            return self._frames.get_nowait()
        except Queue.Empty:
            return None


def hdlc_encode_data(data):
    frame = bytearray()
    frame.append(HDLC_FRAME_START)
    for b in bytearray(data):
        if b == HDLC_FRAME_START or b == HDLC_ESCAPE:
            b ^= HDLC_ESCAPE_MASK
            frame.append(HDLC_ESCAPE)
        frame.append(b)
    frame.append(HDLC_FRAME_START)
    return bytes(frame)
