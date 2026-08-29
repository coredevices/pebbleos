#!/usr/bin/env python
# SPDX-FileCopyrightText: 2024 Google LLC
# SPDX-License-Identifier: Apache-2.0


"""
Sparse Length Encoding

A variant of run-length encoding which is tuned specifically to encode binary
data with long runs of zeroes interspersed with random (poorly-compressible)
data.

The format is fairly simple. The encoded data is a stream of octets (bytes)
beginning with a one-octet header. This header octet is the 'escape byte' that
indicates to the decoder that it and the following octets should be treated
specially. The encoder selects this escape byte to be an octet which occurs
least frequently (or not at all) in the decoded data.

The following octets of the encoded data are emitted literally until an escape
byte is encountered. The escape byte marks the start of  an 'escape sequence'
comprised of the escape byte itself and one or two following bytes.

 - The escape byte followed by 0x00 indicates the end of input.
 - The escape byte followed by 0x01 means 'emit a literal escape byte'
 - The escape byte followed by a byte "b" between 0x02 and 0x7f inclusive means
   'emit b zeroes'. This two-byte sequence can encode a run of length 2-127.
 - The escape byte followed by a byte "b" equal to or greater than 0x80
   (i.e. with the MSB set) means 'take the next byte "c" and emit
   ((b & 0x7f) << 8 | c)+0x80 zeroes'. This three-byte sequence can encode a run
   of length 128-32895.

The minimum overhead for this encoding scheme is three bytes: header and
end-of-input escape sequence.
"""

from collections import Counter
from itertools import groupby

_MAX_COUNT = 0x807F  # max is ((0x7F << 8) | (0xFF) + 0x80


def encode(source):
    # Analyze the source data to select the escape byte. To keep things simple, we don't allow 0 to
    # be the escape character.
    frequency = Counter({n: 0 for n in range(1, 256)})
    frequency.update(source)

    # most_common() doesn't define what happens if there's a tie in frequency. Let's always pick
    # the lowest value of that frequency to make the encoding predictable.
    occurrences = frequency.most_common()
    escape = min(byte for byte, count in occurrences if count == occurrences[-1][1])

    yield bytes([escape])

    for b, g in groupby(source):
        if b == 0x00:
            # this is a run of zeros
            count = len(list(g))
            while count >= 0x80:
                # encode the number of zeros using two bytes
                unit = min(count, _MAX_COUNT)
                count -= unit
                unit -= 0x80
                yield bytes([escape])
                yield bytes([((unit >> 8) & 0x7F) | 0x80])
                yield bytes([unit & 0xFF])
            if count == 1:
                # can't encode a length of 1 zero, so just emit it directly
                yield bytes([0x00])
            elif 1 < count < 0x80:
                # encode the number of zeros using one byte
                yield bytes([escape])
                yield bytes([count])
            elif count < 0:
                raise RuntimeError("Encoding malfunctioned")
        else:
            # simply insert the characters (and escape the escape character)
            for _ in g:
                yield bytes([b])
                if b == escape:
                    yield bytes([1])

    yield bytes([escape])
    yield bytes([0x00])


def decode(stream):
    stream = iter(stream)
    escape = next(stream)

    while True:
        char = next(stream)

        if char == escape:
            code = next(stream)
            if code == 0x00:
                return
            elif code == 0x01:
                yield bytes([escape])
            else:
                if code & 0x80 == 0:
                    count = code
                else:
                    count = (((code & 0x7F) << 8) | next(stream)) + 0x80
                    assert count <= _MAX_COUNT

                for _ in range(count):
                    yield bytes([0x00])
        else:
            yield bytes([char])


if __name__ == "__main__":
    import sys
    from pathlib import Path

    if len(sys.argv) == 1:
        # run unit tests
        import unittest

        class TestSparseLengthEncoding(unittest.TestCase):
            def test_empty(self):
                raw_data = b""
                encoded_data = b"".join(encode(raw_data))
                b"".join(decode(encoded_data))
                self.assertEqual(encoded_data, b"\x01\x01\x00")

            def test_no_zeros(self):
                raw_data = b"\x02\xff\xef\x99"
                encoded_data = b"".join(encode(raw_data))
                b"".join(decode(encoded_data))
                self.assertEqual(encoded_data, b"\x01\x02\xff\xef\x99\x01\x00")

            def test_one_zero(self):
                raw_data = b"\x00"
                encoded_data = b"".join(encode(raw_data))
                b"".join(decode(encoded_data))
                self.assertEqual(encoded_data, b"\x01\x00\x01\x00")

            def test_small_number_of_zeros(self):
                # under 0x80 zeros
                raw_data = b"\0" * 0x0040
                encoded_data = b"".join(encode(raw_data))
                decoded_data = b"".join(decode(encoded_data))
                self.assertEqual(encoded_data, b"\x01\x01\x40\x01\x00")
                self.assertEqual(decoded_data, raw_data)

            def test_medium_number_of_zeros(self):
                # between 0x80 and 0x807f zeros
                raw_data = b"\0" * 0x1800
                encoded_data = b"".join(encode(raw_data))
                decoded_data = b"".join(decode(encoded_data))
                self.assertEqual(encoded_data, b"\x01\x01\x97\x80\x01\x00")
                self.assertEqual(decoded_data, raw_data)

            def test_remainder_one(self):
                # leaves a remainder of 1 zero
                raw_data = b"\0" * (0x807F + 1)
                encoded_data = b"".join(encode(raw_data))
                decoded_data = b"".join(decode(encoded_data))
                self.assertEqual(encoded_data, b"\x01\x01\xff\xff\x00\x01\x00")
                self.assertEqual(decoded_data, raw_data)

            def test_remainder_under_128(self):
                # leaves a remainder of 100 zeros
                raw_data = b"\0" * (0x807F + 100)
                encoded_data = b"".join(encode(raw_data))
                decoded_data = b"".join(decode(encoded_data))
                self.assertEqual(encoded_data, b"\x01\x01\xff\xff\x01\x64\x01\x00")
                self.assertEqual(decoded_data, raw_data)

        unittest.main()
    elif len(sys.argv) == 2:
        # encode the specified file
        data = Path(sys.argv[1]).read_bytes()
        encoded = b"".join(encode(data))
        if b"".join(decode(encoded)) != data:
            raise RuntimeError("Invalid encoding")
        sys.stdout.buffer.write(encoded)
    else:
        raise RuntimeError("Invalid arguments")
