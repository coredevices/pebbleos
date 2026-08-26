#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2025 Federico Bechini
# SPDX-License-Identifier: Apache-2.0

"""
Parser for Pebble flash logs (PBL_LOG).
Parses the binary circular buffer format and extracts log messages.
"""

import argparse
import json
import os
import struct
import sys
from datetime import datetime

# Setup paths for dehash libraries
PYTHON_LIBS_PATH = os.path.join(os.path.dirname(__file__), "libs")
LOG_HASHING_PATH = os.path.join(os.path.dirname(__file__), "..", "tools", "log_hashing")

if PYTHON_LIBS_PATH not in sys.path:
    sys.path.insert(0, PYTHON_LIBS_PATH)
if LOG_HASHING_PATH not in sys.path:
    sys.path.insert(0, LOG_HASHING_PATH)

try:
    import logdehash

    DEHASH_AVAILABLE = True
except ImportError:
    DEHASH_AVAILABLE = False

# Firmware Constants
LOG_MAGIC = 0x21474F4C  # "LOG!"
LOG_VERSION = 0x1
LOG_FLAGS_VALID = 0x1
LOG_PAGE_SIZE = 0x2000  # 8KB
MAX_MSG_LEN = 253

FLASH_LOGGING_HEADER_SIZE = 4 + 1 + 20 + 1 + 1 + 1  # 28 bytes
LOG_RECORD_HEADER_SIZE = 2
LOG_BINARY_MESSAGE_BASE_SIZE = 4 + 1 + 1 + 2 + 16  # 24 bytes


def parse_flash_logging_header(data, offset):
    if offset + FLASH_LOGGING_HEADER_SIZE > len(data):
        return None

    (magic,) = struct.unpack_from("<I", data, offset)
    if magic != LOG_MAGIC:
        return None

    version = data[offset + 4]
    if version != LOG_VERSION:
        return None

    return {
        "build_id": data[offset + 5 : offset + 5 + 20].hex(),
        "log_file_id": data[offset + 25],
        "log_chunk_id": data[offset + 26],
        "offset": offset,
    }


def parse_log_binary_message(data, offset, msg_length):
    if offset + LOG_BINARY_MESSAGE_BASE_SIZE + msg_length > len(data):
        return None

    (timestamp,) = struct.unpack_from(">I", data, offset)
    log_level = data[offset + 4]
    (line_number,) = struct.unpack_from(">H", data, offset + 6)

    filename_bytes = data[offset + 8 : offset + 8 + 16]
    null_pos = filename_bytes.find(b"\x00")
    if null_pos >= 0:
        filename_bytes = filename_bytes[:null_pos]
    filename = filename_bytes.decode("utf-8", errors="ignore")

    message_bytes = data[offset + 24 : offset + 24 + msg_length]
    message = message_bytes.decode("utf-8", errors="ignore").rstrip("\x00")

    return {
        "timestamp": timestamp,
        "log_level": log_level,
        "line_number": line_number,
        "filename": filename,
        "message": message,
    }


def parse_flash_logs(flash_data):
    logs = []
    # Find all pages with valid headers
    pages = []
    for page_start in range(0, len(flash_data), LOG_PAGE_SIZE):
        header = parse_flash_logging_header(flash_data, page_start)
        if header:
            pages.append((page_start, header))

    # Sort pages by file_id and chunk_id
    pages.sort(key=lambda x: (x[1]["log_file_id"], x[1]["log_chunk_id"]))

    for page_start, header in pages:
        page_offset = page_start + FLASH_LOGGING_HEADER_SIZE
        while page_offset < page_start + LOG_PAGE_SIZE:
            if page_offset + LOG_RECORD_HEADER_SIZE > len(flash_data):
                break

            flags = flash_data[page_offset]
            length = flash_data[page_offset + 1]

            if length == 0 or length > MAX_MSG_LEN:
                break

            if (flags & LOG_FLAGS_VALID) == 0:
                msg_offset = page_offset + LOG_RECORD_HEADER_SIZE
                if msg_offset + length <= len(flash_data):
                    msg_length = flash_data[msg_offset + 5]
                    log_msg = parse_log_binary_message(
                        flash_data, msg_offset, msg_length
                    )
                    if log_msg:
                        logs.append(log_msg)

                page_offset += LOG_RECORD_HEADER_SIZE + length
            else:
                break
    return logs


_dehasher = None


def get_dehasher(loghash_dict_path):
    global _dehasher
    if _dehasher is None and DEHASH_AVAILABLE and loghash_dict_path:
        try:
            _dehasher = logdehash.LogDehash("", monitor_dict_file=False)
            with open(loghash_dict_path, "r") as f:
                _dehasher.load_log_strings_from_dict(json.load(f))
        except Exception as e:
            print(f"Warning: Failed to load dehash dictionary: {e}")
            _dehasher = None
    return _dehasher


def format_log_message(log_msg, dehasher=None):
    try:
        dt = datetime.fromtimestamp(log_msg["timestamp"])
        ts = dt.strftime("%H:%M:%S.%f")[:-3]
    except:
        ts = f"0x{log_msg['timestamp']:08X}"

    level = {0: "A", 1: "E", 2: "W", 3: "I", 4: "D"}.get(log_msg["log_level"], "?")
    msg = log_msg["message"]

    if dehasher and msg.startswith("NL:"):
        result = dehasher.dehash(f":0> {msg}")
        if result and "formatted_msg" in result:
            msg = result["formatted_msg"]

    filename = log_msg["filename"] or "unknown"
    return f"{level} {ts} {filename}:{log_msg['line_number']}> {msg}"


def main():
    parser = argparse.ArgumentParser(description="Parse Pebble flash logs")
    parser.add_argument("file", help="Binary flash log file")
    parser.add_argument("--filter", help="Filter messages containing text")
    parser.add_argument("--output", help="Output file")
    parser.add_argument("--show", action="store_true", help="Show logs in stdout")
    parser.add_argument("--dehash", help="Path to loghash_dict.json")
    args = parser.parse_args()

    if not os.path.exists(args.file):
        print(f"Error: File {args.file} does not exist")
        sys.exit(1)

    with open(args.file, "rb") as f:
        data = f.read()

    # Try to find default dictionary if not provided
    dehash_path = args.dehash
    if not dehash_path:
        default_dict = os.path.join(
            os.path.dirname(__file__),
            "..",
            "build",
            "src",
            "fw",
            "loghash_dict.json",
        )
        if os.path.exists(default_dict):
            dehash_path = default_dict

    logs = parse_flash_logs(data)
    dehasher = get_dehasher(dehash_path)

    if args.filter:
        logs = [l for l in logs if args.filter in l["message"]]

    output_lines = [format_log_message(l, dehasher) for l in logs]
    output_text = "\n".join(output_lines)

    current_ts = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    out_file = (
        args.output or os.path.splitext(args.file)[0] + f"_parsed_{current_ts}.txt"
    )
    with open(out_file, "w", encoding="utf-8") as f:
        f.write(output_text)

    print(f"Successfully parsed {len(logs)} messages. Saved to: {out_file}")

    if args.show:
        for line in output_lines:
            print(line)


if __name__ == "__main__":
    main()
