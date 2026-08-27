# SPDX-FileCopyrightText: 2024 Google LLC
# SPDX-License-Identifier: Apache-2.0

CRC_POLY = 0x04C11DB7


def precompute_table(bits):
    lookup_table = []
    for i in range(2**bits):
        rr = i << (32 - bits)
        for x in range(bits):
            if rr & 0x80000000:
                rr = (rr << 1) ^ CRC_POLY
            else:
                rr <<= 1
        lookup_table.append(rr & 0xFFFFFFFF)
    return lookup_table


lookup_table = precompute_table(8)


def process_word(data, crc=0xFFFFFFFF):
    if len(data) < 4:
        # The CRC data is "padded" in a very unique and confusing fashion.
        data = data[::-1] + b"\0" * (4 - len(data))

    for b in reversed(data):
        crc = ((crc << 8) ^ lookup_table[(crc >> 24) ^ b]) & 0xFFFFFFFF
    return crc


def process_buffer(buf, c=0xFFFFFFFF):
    word_count = (len(buf) + 3) // 4

    crc = c
    for i in range(word_count):
        crc = process_word(buf[i * 4 : (i + 1) * 4], crc)
    return crc


def crc32(data):
    return process_buffer(data)


if __name__ == "__main__":
    import sys
    from pathlib import Path

    assert 0x89F3BAB2 == process_buffer(b"123 567 901 34")
    assert 0xAFF19057 == process_buffer(b"123456789")
    assert 0x519B130 == process_buffer(b"\xfe\xff\xfe\xff")
    assert 0x495E02CA == process_buffer(b"\xfe\xff\xfe\xff\x88")

    print("All tests passed!")

    # arg1 == path to file to crc
    # arg2 == only crc first N bytes of file specified in arg 1
    if len(sys.argv) >= 2:
        if len(sys.argv) >= 3:
            with open(sys.argv[1], "rb") as f:
                b = f.read(int(sys.argv[2]))
        else:
            b = Path(sys.argv[1]).read_bytes()
        crc = crc32(b)
        print(f"{crc:d} or 0x{crc:x}")
