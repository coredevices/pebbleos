# SPDX-FileCopyrightText: 2024 Google LLC
# SPDX-License-Identifier: Apache-2.0

import argparse
import array
import struct

import prompt
import serial
import stm32_crc as crc
from hdlc import HDLCDecoder
from serial_port_wrapper import SerialPortWrapper

""" 
This script will invoke the microphone test command on snowy, read the
PCM encoded audio data from the serial port and store it to a .wav file, 
which can be played back from any audio player. It can be invoked via
the './waf record' command or by running the script from the command line.

The output sample rate and sample size when recording using the waf script
can be configured by modifying the 'freq' and 'sample_size' variables, 
respectively.
"""

FREQ = 8000  # Default Output sample rate from microphone (8000 or 16000)
SAMPLE_SIZE = 2  # Default sample size: 1 for 8-bit data, 2 for 16-bit data samples (must match what is output by watch)
seconds = 1


def store_wav(samples, filename, sample_size, freq):
    size = len(samples) * sample_size
    width = 8 * sample_size
    byte_rate = freq * sample_size

    wav = "RIFF"
    wav += struct.pack("i", size + 44)  # file size
    wav += "WAVE"
    wav += "fmt "
    wav += struct.pack("i", 16)  # fmt data length
    wav += struct.pack("h", 1)  # audio format (1 = PCM)
    wav += struct.pack("h", 1)  # channels
    wav += struct.pack("i", freq)  # sample rate
    wav += struct.pack("i", byte_rate)  # byte rate
    wav += struct.pack("h", sample_size)  # block alignment (bytes per block)
    wav += struct.pack("h", width)  # bits per sample
    wav += "data"
    wav += struct.pack("i", size)  # data size

    for sample in samples:
        wav += struct.pack("B" if sample_size == 1 else "H", sample)

    with open(filename, "wb") as f:
        f.write(wav)
        f.close()
        print(f"{len(samples)} samples packed into {filename}")


def receive_hdlc_data(s, sample_size):
    count = {"i": 0}

    decoder = HDLCDecoder()
    data = []
    while True:
        raw_data = s.read(1000)
        if len(raw_data) == 0:
            break
        decoder.write(raw_data)
        for frame in iter(decoder.get_frame, None):
            count["i"] += 1
            if len(frame) > 4:
                frame = bytearray(frame)
                frame_crc = (
                    frame[-4] | (frame[-3] << 8) | (frame[-2] << 16) | (frame[-1] << 24)
                )
                if crc.crc32(array.array("B", frame[:-4]).tostring()) == frame_crc:
                    data.extend(frame[:-4])
                else:
                    print(count["i"])

    print("total frames received =", str(count["i"]))
    samples = []
    for i in range(0, len(data), sample_size):
        try:
            d = 0
            if sample_size == 1:
                d = data[i]
            elif sample_size == 2:
                d = data[i] | (data[i + 1] << 8)
            samples.append(d)
        except:
            print(f"conversion failed on word {i / sample_size}")

    return samples


def open_serial_port(tty, baud_rate):

    s = serial.serial_for_url(tty, baud_rate, timeout=2)
    if s is not None:
        print("opened", tty, "at", str(baud_rate), "bps")
    else:
        print("failed to open", tty)
    return s


def record_from_tty(
    tty_prompt,
    tty_accessory,
    t=seconds,
    filename="test.wav",
    sample_size=SAMPLE_SIZE,
    sample_rate=FREQ,
    volume=100,
    accessory=False,
):
    if tty_prompt == tty_accessory:
        # sending commands via accessory connector, so set baud rate correctly
        s = SerialPortWrapper(tty_prompt, baud_rate=115200)
    else:
        s = SerialPortWrapper(tty_prompt)

    try:
        prompt.go_to_prompt(s)
        print(
            "record {}-bit audio data for {}s at {}Hz (~{} samples)".format(
                "8" if sample_size == 1 else "16", t, str(sample_rate), t * sample_rate
            )
        )
        cmd = "mic start {} {} {} {}".format(
            t, "8" if sample_size == 1 else "16", sample_rate, volume
        )
        prompt.issue_command(s, cmd)
        s.close()

        if (sample_size == 2) and (sample_rate == 16000):
            s = open_serial_port(tty_accessory, 460800)
        elif (sample_size == 2) or (sample_rate == 16000):
            s = open_serial_port(tty_accessory, 230400)
        else:
            s = open_serial_port(tty_accessory, 115200)

        samples = receive_hdlc_data(s, sample_size)
        print(f"{len(samples)} samples read")
        if len(samples) != (sample_rate * t):
            print(
                f"Not enough samples received ({len(samples)}/{sample_rate * t})"
            )
        else:
            print("Output file: " + filename)
            store_wav(samples, filename, sample_size, sample_rate)

    finally:
        s.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--tty_prompt",
        type=str,
        help="Prompt tty (e.g. /dev/cu.usbserial-xxxxxxxB or /dev/ttyUSB0). If this"
        "is specified, then the accessory port must be specified",
    )
    parser.add_argument(
        "--tty_accessory",
        type=str,
        help="Accessory port tty (e.g. /dev/cu.usbserial-xxxxxxxB or /dev/ttyUSB0)."
        "If the commands are being sent via the accessory port, this and only this,"
        "tty must be specified.",
    )
    parser.add_argument(
        "-o",
        "--output",
        type=str,
        default="test.wav",
        help="Output file name. Default: 'test.wav'",
    )
    parser.add_argument(
        "-d",
        "--duration",
        type=int,
        default=seconds,
        help="Number of seconds of audio that will be recorded. Default: 1s, Max: 60s",
    )
    parser.add_argument(
        "-w",
        "--width",
        type=int,
        choices=[8, 16],
        default=16,
        help="Sample data width (8- or 16-bit). Default: 16-bit",
    )
    parser.add_argument(
        "-r",
        "--rate",
        type=int,
        choices=[8000, 16000],
        default=8000,
        help="Sample rate in Hz. Default: 8000",
    )
    parser.add_argument(
        "-v", "--volume", type=int, default=100, help="Volume (1 - 1000). Default: 100"
    )
    args = parser.parse_args()

    if args.tty_accessory and not args.tty_prompt:
        tty_accessory = args.tty_accessory
        tty_prompt = args.tty_accessory
    elif args.tty_prompt and not args.tty_accessory:
        raise Exception(
            "If the prompt tty is specified, the accessory port tty must be specified "
            "too!"
        )
    else:
        tty_accessory = args.tty_accessory
        tty_prompt = args.tty_prompt

    sample_size = args.width / 8
    record_from_tty(
        tty_prompt,
        tty_accessory,
        args.duration,
        args.output,
        sample_size,
        args.rate,
        min(args.volume, 1000),
    )
