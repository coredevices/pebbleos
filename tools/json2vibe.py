#!/usr/bin/env python
# SPDX-FileCopyrightText: 2024 Google LLC
# SPDX-License-Identifier: Apache-2.0


import argparse
import json
import os

from libpebble2.protocol.base import PebblePacket
from libpebble2.protocol.base.types import *


class VibeNote(PebblePacket):
    vibe_duration_ms = Uint16()
    brake_duration_ms = Uint8()
    strength = Int8()


class VibeNoteList(PebblePacket):
    notes = FixedList(VibeNote)


class VibePattern(PebblePacket):
    indices = FixedList(Uint8())


class VibePatternRepeatDelay(PebblePacket):
    duration = Uint16()


class VibeAttribute(PebblePacket):
    id = Uint8()
    length = Uint16()
    attribute = Union(
        id,
        {
            0x01: VibeNoteList,
            0x02: VibePattern,
            0x03: VibePatternRepeatDelay,
        },
        length=length,
    )


class VibeAttributeList(PebblePacket):
    num_attributes = Uint8()
    attributes = FixedList(VibeAttribute, count=num_attributes)


class VibeScore(PebblePacket):
    version = Uint16()
    reserved = Padding(4)
    length = Uint16()
    attr_list = Embed(VibeAttributeList, length=length)


class VibeFile(PebblePacket):
    class Meta:
        endianness = "<"

    fourcc = FixedString(length=4, default="VIBE")
    score = Embed(VibeScore)


def serialize(json_data):
    CURRENT_VERSION = 1
    NEGATIVE_VIBE_STRENGTH_MAX = -100
    POSITIVE_VIBE_STRENGTH_MAX = 100
    # Keep in sync with VIBE_SCORE_MAX_REPEAT_DELAY_MS in vibe_score.c; the
    # firmware refuses to load scores exceeding it.
    MAX_REPEAT_DELAY_MS = 60000

    if not 0 <= json_data.get("repeat_delay_ms", 0) <= MAX_REPEAT_DELAY_MS:
        raise ValueError(
            '"repeat_delay_ms" {} out of bounds. Values between 0 and {} only.'.format(
                json_data["repeat_delay_ms"], MAX_REPEAT_DELAY_MS
            )
        )

    for note in json_data["notes"]:
        if not (
            NEGATIVE_VIBE_STRENGTH_MAX <= note["strength"] <= POSITIVE_VIBE_STRENGTH_MAX
        ):
            raise ValueError(
                '"strength" {} out of bounds. Values between -100 and 100 only.'.format(
                    note["strength"]
                )
            )

    # construct an object to be fed into the VibeFileAdapter
    note_dictionary = {note["id"]: i for i, note in enumerate(json_data["notes"])}

    vibe_attribute_list = [
        VibeAttribute(
            attribute=VibeNoteList(
                notes=[
                    VibeNote(
                        vibe_duration_ms=note["vibe_duration_ms"],
                        brake_duration_ms=note["brake_duration_ms"],
                        strength=note["strength"],
                    )
                    for note in json_data["notes"]
                ]
            )
        ),
        VibeAttribute(
            attribute=VibePattern(
                indices=[note_dictionary[x] for x in json_data["pattern"]]
            )
        ),
    ]
    if "repeat_delay_ms" in json_data:
        vibe_attribute_list.append(
            VibeAttribute(
                attribute=VibePatternRepeatDelay(duration=json_data["repeat_delay_ms"])
            )
        )

    obj = VibeFile(
        score=VibeScore(
            version=CURRENT_VERSION,
            attr_list=VibeAttributeList(attributes=vibe_attribute_list),
        )
    )

    # do the dirty work
    return obj.serialise()


def convert(file_path, out_path=None):
    if out_path is None:
        out_path = os.path.splitext(file_path)[0] + ".vibe"

    with open(out_path, "wb") as o:
        convert_to_file(file_path, o)


def convert_to_file(input_file_path, output_file):
    with open(input_file_path, "r") as f:
        data = json.loads(f.read())

    output_file.write(serialize(data))


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("json_file")
    args = parser.parse_args()

    convert(args.json_file)
