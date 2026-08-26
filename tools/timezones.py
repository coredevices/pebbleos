#!/usr/bin/python
# SPDX-FileCopyrightText: 2024 Google LLC
# SPDX-License-Identifier: Apache-2.0



import argparse
import re
import struct
import sys
from collections import namedtuple
from functools import cmp_to_key

# dst-rules are formatted in 2 forms
# Jordan Oct lastFri 0:00 -
# US Mar Sun>=8 2:00 D
# so need starting day_of_month (mday), direction (increment or decrement)
# as well as dst_start or dst_end (D or S)
dstrule = namedtuple("dstrule", "dstzone ds month wday flag mday hour minute")

tz_continent_list = (
    "Africa",
    "America",
    "Antarctica",
    "Asia",
    "Atlantic",
    "Australia",
    "Europe",
    "Indian",
    "Pacific",
    "Etc",
)
tz_continent_dict = {name: index for index, name in enumerate(tz_continent_list)}

# This dictionary contains all of the daylight_savings_time zones, including No-DST ('-')
# Note the order of list matters, as it shouldn't change from release to release or else you'll
# have a very confused watch after an upgrade until the timezone is set again. We also hardcode
# certain indexes in the firmware, see the assertions below.
dstzone_list = (
    "-",
    "AN",
    "AS",
    "AT",
    "AV",
    "Azer",
    "Brazil",
    "C-Eur",
    "Canada",
    "Chatham",
    "ChileAQ",
    "Cuba",
    "E-Eur",
    "E-EurAsia",
    "EU",
    "EUAsia",
    "Egypt",
    "Fiji",
    "Haiti",
    "Jordan",
    "LH",
    "Lebanon",
    "Mexico",
    "Morocco",
    "NZ",
    "Namibia",
    "Palestine",
    "Para",
    "RussiaAsia",
    "Syria",
    "Thule",
    "US",
    "Uruguay",
    "W-Eur",
    "WS",
    "Zion",
    "Mongol",
    "Moldova",
    "Iran",
    "Chile",
    "Tonga",
    "Eire",
)
dstzone_dict = {name: index for index, name in enumerate(dstzone_list)}

# Make sure some of these values don't move around, because the firmware code in
# fw/util/time/time.h depends on certain special case timezones having certain values (see the
# DSTID_* defines). This is gross and brittle and it would be better to generate a header.
# PBL-30559
assert dstzone_dict["Brazil"] == 6
assert dstzone_dict["LH"] == 20


def dstrule_cmp(a, b):
    """
    Sort a list in the following order:
    1) By dstzone
    2) So that for each pair of dstzones, the 'DS' field has the following order D, S, -
    """

    if a.dstzone < b.dstzone:
        return -1
    elif a.dstzone > b.dstzone:
        return 1
    else:
        if a.ds == "D":
            return -1
        elif b.ds == "D":
            return 1
        elif a.ds == "S":
            return -1
        elif b.ds == "S":
            return 1
        else:
            return 0


def dstrules_parse(tzfile):
    """Top level wrapper, greps the raw zoneinfo file
    Args:
    Returns:
            None
    """

    dstrule_list = []

    # struct tm->tm_mon is 0 indexed
    month_dict = {
        "Jan": 0,
        "Feb": 1,
        "Mar": 2,
        "Apr": 3,
        "May": 4,
        "Jun": 5,
        "Jul": 6,
        "Aug": 7,
        "Sep": 8,
        "Oct": 9,
        "Nov": 10,
        "Dec": 11,
    }

    # days in months for leapyear (feb is 1 day more)
    month_days = [31, 29, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31]

    # we need an 'any' wday for rules that enact on a specific day
    # the number picked is meaningless. Corresponds to DSTRULE_WDAY_ANY in clock.c
    wday_dict = {
        "Sun": 0,
        "Mon": 1,
        "Tue": 2,
        "Wed": 3,
        "Thu": 4,
        "Fri": 5,
        "Sat": 6,
        "Any": 255,
    }

    with open(tzfile, "r") as infile:
        lines = infile.readlines()
        for line_num, line in enumerate(lines):
            match_list = re.finditer(
                r"^Rule\s+(?P<dstzone>[-A-Za-z]+)\s+[0-9]+\s+max\s+-\s+"
                r"(?P<month>[A-Za-z]+)\s+(?P<wday_stuff>[>=A-Za-z0-9]+)\s+"
                r"(?P<time>[:0-9]+)(?P<timemode>[swugz]*)\s+(?P<save>[:0-9]+)[su]*\s+"
                r"(?P<DS>[DS-])",
                line,
            )
            if match_list:
                for match in match_list:
                    try:
                        dstzone = dstzone_dict[match.group("dstzone")]
                    except KeyError:
                        print(
                            "ERROR: Unknown dstzone found on line {}: {}".format(
                                line_num, match.group("dstzone")
                            ),
                            file=sys.stderr,
                        )
                        print(
                            "You'll need to add this new dstzone to the dstzone_list.",
                            file=sys.stderr,
                        )
                        raise

                    # Determine DS from SAVE field: SAVE > 0 means entering DST ('D'),
                    # SAVE = 0 means leaving DST ('S'). The LETTER field in tzdata is not
                    # a reliable indicator (e.g., EU uses 'S' for Summer time which is DST).
                    save = match.group("save")
                    # Parse SAVE field (can be "0", "1:00", "0:30", etc.)
                    if ":" in save:
                        save_hours, save_mins = save.split(":")
                        save_total = int(save_hours) * 60 + int(save_mins)
                    else:
                        save_total = int(save) * 60
                    ds = "D" if save_total > 0 else "S"
                    wday_stuff = match.group("wday_stuff")
                    month = month_dict[match.group("month")]
                    hour, minute = match.group("time").split(":")
                    flag = 0
                    for modech in match.group("timemode"):
                        if modech == "s":  # Standard time (not wall time)
                            flag |= 2
                        elif (
                            modech == "u" or modech == "g" or modech == "z"
                        ):  # UTC time
                            flag |= 4
                        elif modech == "w":  # Wall time
                            flag |= 8
                        else:
                            raise Exception("hurf char")

                    if "last" in wday_stuff:  # Last wday of the month
                        # pick the last day of the month
                        mday = month_days[month]
                        wday = wday_dict[wday_stuff[4:]]
                        flag |= 1
                    elif ">=" in wday_stuff:  # wday after mday
                        # get the number after '>='
                        mday = wday_stuff.split("=")[1]
                        # get the dayname before '>='
                        wday = wday_dict[wday_stuff.split(">")[0]]
                    else:  # specific mday
                        mday = int(wday_stuff)
                        wday = wday_dict["Any"]

                    new_rule = dstrule(
                        dstzone,
                        ds,
                        month,
                        wday,
                        int(flag),
                        int(mday),
                        int(hour),
                        int(minute),
                    )
                    dstrule_list.append(new_rule)

    dstrule_list.sort(key=cmp_to_key(dstrule_cmp))

    return dstrule_list


def build_zoneinfo_list(tzfile):
    """
    Top level wrapper, searches the raw zoneinfo file
    Args:
    Returns:
            None
    """

    zoneinfo_list = []

    with open(tzfile, "r") as infile:
        lines = infile.readlines()
        region = ""
        continent = ""
        for line in lines:
            # Parse blocks that look like this
            # Zone America/Toronto    -5:17:32 -  LMT 1895
            #            -5:00   Canada  E%sT    1919
            #            -5:00   Toronto E%sT    1942 Feb  9  2:00s
            #            -5:00   Canada  E%sT    1946
            #            -5:00   Toronto E%sT    1974
            #            -5:00   Canada  E%sT
            #
            # We first find the zone line, that tells us what the continent and region names are
            # for this zone, and then we look for the final line that tells us the current GMT
            # offset, the dst rule, and then the timezone abbreviation. We discard any outdated
            # information.

            if line.startswith("Zone"):
                match = re.match(
                    r"^Zone\s+"
                    r"(?P<continent>[A-Za-z]+)\/(?P<region>[-_\/A-Za-z]+)",
                    line,
                )
                if match:
                    continent = match.group("continent")
                    region = match.group("region")
                    # fixup regions with an extra specifier, such as America/Indiana/Indianapolis
                    region = region.split("/")[-1]

                    full_region = continent + "/" + region

                    # Don't include Troll, Antarctica as their DST is 2 hours and overlapping rules
                    # not even a city, actually just a station :
                    # http://mm.icann.org/pipermail/tz/2014-February/020605.html
                    if full_region == "Antarctica/Troll" or full_region == "Africa/Cairo" or full_region == "Africa/Casablanca" or full_region == "Africa/El_Aaiun" or full_region == "Australia/Lord_Howe":
                        region = ""

            # Now look to see if we've found the final line of the block
            match = re.match(
                r"^(Zone\s+[-_\/A-Za-z]+\s+|\s+)"  # Leading spaces or zone name
                # The gmt offset (like 4:00 or -3:30)
                r"(?P<offset>[-0-9:]+)\s+"
                # The name of the dstrule, such as US, or - if no DST
                r"(?P<dst_name>[-A-Za-z]+)\s+"
                # The short name of the timezone, like E%sT (EST or EDT), %z or VET
                # Or a GMT offset like +06
                r"(?P<tz_abbr>([A-Z%sz\/]+)|\+\d+)"
                # Trailing spaces and comments, no year or dates allowed
                r"(\s+\#.*)?$",
                line,
                re.VERBOSE,
            )

            if match and region:
                tz_abbr = match.group("tz_abbr")
                if tz_abbr == "%z":
                    m = re.match(
                        r"(?P<sign>[-+]?)(?P<hours>\d+):(?P<minutes>\d+)",
                        match.group("offset"),
                    )
                    if not m:
                        raise Exception(
                            f"Unsupported offset {match.group('offset')} for region {region}"
                        )

                    sign = m.group("sign") or "+"
                    hours = int(m.group("hours"))
                    minutes = int(m.group("minutes"))
                    tz_abbr = f"{sign}{hours:02d}{minutes:02d}"
                else:
                    tz_abbr = match.group("tz_abbr").replace("%s", "*")
                    if tz_abbr.startswith("GMT/") or tz_abbr.startswith("IST/"):
                        tz_abbr = tz_abbr[4:]

                zoneinfo_list.append(
                    continent
                    + " "
                    + region
                    + " "
                    + match.group("offset")
                    + " "
                    + tz_abbr
                    + " "
                    + match.group("dst_name")
                )
                region = ""

    # return the list, alphabetically sorted
    zoneinfo_list.sort()
    return zoneinfo_list


def zonelink_parse(tzfile):
    """
    Top level wrapper, searches the raw zoneinfo file
    Args:
    Returns:
            None
    """

    zonelink_list = []

    with open(tzfile, "r") as infile:
        lines = infile.readlines()
        for line in lines:
            # Parse blocks that look like this
            # Link America/Los_Angeles US/Pacific

            # It's a link!
            if line.startswith("Link"):
                match = re.match(
                    r"^Link\s+(?P<target>[-_\/A-Za-z]+)\s+"
                    r"(?P<linkname>[-_\/A-Za-z]+)\s*",
                    line,
                )
                if match:
                    target = match.group("target")
                    linkname = match.group("linkname")
                    # fixup regions with an extra specifier, such as America/Indiana/Indianapolis
                    target_parts = target.split("/")
                    target = target_parts[0]
                    if len(target_parts) != 1:
                        target += "/" + target_parts[-1]
                    zonelink_list.append(target + " " + linkname)

    # return the list, alphabetically sorted
    zonelink_list.sort()
    return zonelink_list


def zoneinfo_to_bin(zoneinfo_list, dstrule_list, zonelink_list, output_bin):
    # Corresponds to TIMEZONE_LINK_NAME_LENGTH in clock.c
    # only reason we need 33 characters is for that
    # troublemaker 'America/Argentina/ComodRivadavia'
    TIMEZONE_LINK_NAME_LENGTH = 33

    # format
    # 1 byte + 15 bytes + 2 bytes + 5 bytes + 1 byte = 24 bytes
    # Continent_index City gmt_offset_minutes tz_abbr dst_id

    # Unsigned short - count of entries
    output_bin.write(struct.pack("<H", len(zoneinfo_list)))
    # Unsigned short - count of DST rules
    output_bin.write(struct.pack("<H", len(dstzone_dict.values())))
    # Unsigned short - count of links
    output_bin.write(struct.pack("<H", len(zonelink_list)))

    region_id_list = []
    # write all the timezones to file
    for line in zoneinfo_list:
        continent, region, gmt_offset_minutes, tz_abbr, dst_zone = line.split(" ")

        # output the timezone continent index
        continent_index = tz_continent_dict[continent]
        output_bin.write(struct.pack("B", continent_index))
        region_id_list.append(continent + "/" + region)

        # fixup and output the timezone region name
        output_bin.write(
            region.ljust(15, "\0").encode("utf8")
        )  # 15-character region zero padded

        # fixup the gmt offset to be integer minutes
        if ":" in gmt_offset_minutes:
            hours, minutes = gmt_offset_minutes.split(":")
        else:
            hours = gmt_offset_minutes
            minutes = 0

        if int(hours) < 0:
            gmt_offset_minutes = int(hours) * 60 - int(minutes)
        else:
            gmt_offset_minutes = int(hours) * 60 + int(minutes)
        # signed short, for negative gmtoffsets
        output_bin.write(struct.pack("<h", gmt_offset_minutes))

        # fix timezone abbreviations that no longer have a DST mode
        if dst_zone not in dstzone_dict:
            tz_abbr = tz_abbr.replace("*", "S")
        if len(tz_abbr) > 5:
            raise Exception(f"Timezone abbreviation too long: {tz_abbr}")
        output_bin.write(
            tz_abbr.ljust(5, "\0").encode("utf8")
        )  # 5-character region zero padded

        # dst table entry, 0 for NONE (ie. dash '-')
        if dst_zone in dstzone_dict:
            dstzone_index = dstzone_dict[dst_zone]
        else:
            dstzone_index = (
                0  # Includes '-', 'SA', 'CR', ... that no longer support DST
            )
        output_bin.write(struct.pack("B", dstzone_index))

    # Each DST rule is serialised as 8 bytes (see TimezoneDSTRule). The reader in
    # service.c treats every DST ID as a fixed-size pair of rules
    # (DST_RULE_PAIR_BYTES = 16) so it can compute the offset of the link section
    # as (dst_rule_count - 1) * 16. Every DST ID must therefore occupy exactly
    # 16 bytes, regardless of how many rules tzdata actually supplies. Most zones
    # have a start (D) and end (S) rule; some have none (e.g. zones that dropped
    # DST) and a few have only one (e.g. negative-DST zones like Eire). Pad the
    # shortfall and truncate any excess so the stride stays fixed.
    DST_RULE_BYTES = 8
    DST_RULES_PER_ID = 2
    DST_RULE_PAIR_BYTES = DST_RULE_BYTES * DST_RULES_PER_ID
    for id in sorted(dstzone_dict.values()):
        # Don't write anything for the "No Rule" dst rule, aka "-".
        if id == 0:
            continue

        # Look up the zone in our dstrule list. There should be two entries for each valid id,
        # one for the start and one for the end of DST. Note that it may not be found if it's a
        # dstrule that doesn't exist anymore.
        dstrules = [r for r in dstrule_list if r.dstzone == id][:DST_RULES_PER_ID]
        bytes_written = 0
        for dstrule in dstrules:
            output_bin.write(struct.pack("c", dstrule.ds.encode("utf8")))
            output_bin.write(struct.pack("B", dstrule.wday))
            output_bin.write(struct.pack("B", dstrule.flag))
            output_bin.write(struct.pack("B", dstrule.month))
            output_bin.write(struct.pack("B", dstrule.mday))
            output_bin.write(struct.pack("B", dstrule.hour))
            output_bin.write(struct.pack("B", dstrule.minute))
            output_bin.write(struct.pack("B", 0))
            bytes_written += DST_RULE_BYTES
        # Pad to a full fixed-size DST ID entry.
        output_bin.write(bytearray(DST_RULE_PAIR_BYTES - bytes_written))

    # write all the timezone links to file
    for line in zonelink_list:
        target, linkname = line.split(" ")
        try:
            region_id = region_id_list.index(target)
        except ValueError as e:
            print("Couldn't find region, skipping:", e)
            continue
        output_bin.write(struct.pack("<H", region_id))
        output_bin.write(linkname.ljust(TIMEZONE_LINK_NAME_LENGTH, "\0").encode("utf8"))


def build_and_create_tzdata(olson_database, output_text, output_bin):
    zoneinfo_list = build_zoneinfo_list(olson_database)

    # save output as text for reference
    with open(output_text, "wb") as output_txt:
        output_txt.writelines(f"{zoneinfo}\n".encode() for zoneinfo in zoneinfo_list)

    dstrule_list = dstrules_parse(olson_database)

    zonelink_list = zonelink_parse(olson_database)

    with open(output_bin, "wb") as f:
        zoneinfo_to_bin(zoneinfo_list, dstrule_list, zonelink_list, f)


def main():
    parser = argparse.ArgumentParser(description="Parse olson timezone database.")

    parser.add_argument("olson_database", type=str)
    parser.add_argument("tzdata_text", type=str)
    parser.add_argument("tzdata_binary", type=str)

    args = parser.parse_args()
    build_and_create_tzdata(args.olson_database, args.tzdata_text, args.tzdata_binary)


if __name__ == "__main__":
    main()
