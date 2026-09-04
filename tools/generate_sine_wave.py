#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2025 Core Devices, LLC
# SPDX-License-Identifier: Apache-2.0

import argparse
import math
import sys


def generate_sine_wave_c_file(
    sample_rate, frequency, output_file, array_name="sine_wave", amplitude=0.9
):
    # Calculate samples per period (rounded to nearest integer)
    samples_per_period = round(sample_rate / frequency)

    # Actual frequency after rounding
    actual_frequency = sample_rate / samples_per_period

    # Maximum value for 16-bit signed integer
    max_value = 32767
    scaled_amplitude = int(max_value * amplitude)

    print("Generating sine wave:")
    print(f"  Sample rate: {sample_rate} Hz")
    print(f"  Requested frequency: {frequency} Hz")
    print(f"  Actual frequency: {actual_frequency:.2f} Hz")
    print(f"  Samples per period: {samples_per_period}")
    print(f"  Total stereo samples: {samples_per_period * 2}")
    print(f"  Amplitude: {amplitude} ({scaled_amplitude}/{max_value})")
    print(f"  Output file: {output_file}")

    # Generate sine wave samples
    samples = []
    for i in range(samples_per_period):
        # Calculate sine value for this sample
        angle = 2.0 * math.pi * i / samples_per_period
        sine_value = int(scaled_amplitude * math.sin(angle))

        # Add stereo samples (left and right channels identical)
        samples.extend([sine_value, sine_value])

    # Generate C file content
    c_content = f'''
#include "{output_file.rsplit(".", 1)[0]}.h"

/* Stereo sine wave data (L, R, L, R, ...) */
int16_t {array_name}[SINE_WAVE_TOTAL_SAMPLES] = {{
'''

    # Add the sample data (16 values per line for readability)
    for i in range(0, len(samples), 16):
        line_samples = samples[i : i + 16]
        formatted_line = "    " + ", ".join(f"{sample:6d}" for sample in line_samples)
        if i + 16 < len(samples):
            formatted_line += ","
        c_content += formatted_line + "\n"

    c_content += "};\n"

    # Write to C file
    try:
        with open(output_file, "w") as f:
            f.write(c_content)
        print(f"Successfully generated {output_file}")
    except OSError as e:
        print(f"Error writing C file: {e}", file=sys.stderr)
        return False

    # Generate header file
    header_file = output_file.rsplit(".", 1)[0] + ".h"

    header_content = f"""
#pragma once

#include <stdint.h>

#define SINE_WAVE_SAMPLE_RATE {sample_rate}
#define SINE_WAVE_FREQUENCY {actual_frequency:.0f}
#define SINE_WAVE_SAMPLES_PER_PERIOD {samples_per_period}
#define SINE_WAVE_TOTAL_SAMPLES {samples_per_period * 2}

/* Stereo sine wave data (L, R, L, R, ...) */
extern int16_t {array_name}[SINE_WAVE_TOTAL_SAMPLES];
"""

    # Write to header file
    try:
        with open(header_file, "w") as f:
            f.write(header_content)
        print(f"Successfully generated {header_file}")
    except OSError as e:
        print(f"Error writing header file: {e}", file=sys.stderr)
        return False

    return True


def main():
    parser = argparse.ArgumentParser(
        description="Generate C file with static sine wave array and header file",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Generate 1kHz sine at 44.1kHz sample rate
  python3 generate_sine_wave.py --sample-rate 44100 --frequency 1000 --output sine_1khz.c
  
  # Generate 440Hz sine at 16kHz sample rate with custom amplitude
  python3 generate_sine_wave.py -s 16000 -f 440 -o sine_440hz.c --amplitude 0.5
  
  # Generate with custom array name
  python3 generate_sine_wave.py -s 48000 -f 1000 -o test_tone.c --array-name test_tone_data
        """,
    )

    parser.add_argument(
        "-s",
        "--sample-rate",
        type=int,
        required=True,
        help="Sample rate in Hz (e.g., 44100, 48000, 16000)",
    )

    parser.add_argument(
        "-f",
        "--frequency",
        type=int,
        required=True,
        help="Sine wave frequency in Hz (e.g., 1000, 440, 880)",
    )

    parser.add_argument(
        "-o",
        "--output",
        type=str,
        required=True,
        help="Output C file path (e.g., sine_wave.c)",
    )

    parser.add_argument(
        "--array-name",
        type=str,
        default="sine_wave",
        help="Name of the C array (default: sine_wave)",
    )

    parser.add_argument(
        "--amplitude",
        type=float,
        default=0.9,
        help="Amplitude factor from 0.0 to 1.0 (default: 0.9)",
    )

    args = parser.parse_args()

    # Validate arguments
    if args.sample_rate <= 0:
        print("Error: Sample rate must be positive", file=sys.stderr)
        return 1

    if args.frequency <= 0:
        print("Error: Frequency must be positive", file=sys.stderr)
        return 1

    if args.frequency >= args.sample_rate / 2:
        print(
            "Error: Frequency must be less than Nyquist frequency (sample_rate/2)",
            file=sys.stderr,
        )
        return 1

    if not (0.0 <= args.amplitude <= 1.0):
        print("Error: Amplitude must be between 0.0 and 1.0", file=sys.stderr)
        return 1

    if not args.array_name.isidentifier():
        print("Error: Array name must be a valid C identifier", file=sys.stderr)
        return 1

    # Generate the sine wave C file
    success = generate_sine_wave_c_file(
        args.sample_rate, args.frequency, args.output, args.array_name, args.amplitude
    )

    return 0 if success else 1


if __name__ == "__main__":
    sys.exit(main())
