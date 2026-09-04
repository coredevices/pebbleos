# Timezone database

The firmware's timezone list is a binary resource
(`RESOURCE_ID_TIMEZONE_DATABASE`) built from an IANA tzdata snapshot
(`resources/normal/base/tzdata/timezones_olson.txt`) by
`tools/timezones.py`, wired into the resource build by
`tools/cmake/resources.py`. It is read by
`src/fw/services/timezone_database/service.c`; the DST-rule struct is
`TimezoneDSTRule` in `include/pbl/services/timezone_database.h`.

Layout (little-endian): a 6-byte header, then region records, DST rules
and link records.

## Header

`uint16_t region_count`, `uint16_t dst_rule_count`, `uint16_t link_count`.

## Region records — 24 bytes each

| Field           | Size | Notes                                                                                                            |
| --------------- | ---- | ---------------------------------------------------------------------------------------------------------------- |
| continent index | 1    | into a fixed 10-entry list: Africa, America, Antarctica, Asia, Atlantic, Australia, Europe, Indian, Pacific, Etc |
| city name       | 15   | NUL-padded; full name is `continent/city`                                                                        |
| GMT offset      | 2    | `int16_t`, minutes                                                                                               |
| tz abbreviation | 5    | NUL-padded; may contain `*` as a `%s` placeholder for the D/S letter                                             |
| DST rule id     | 1    | 0 = no DST                                                                                                       |

## DST rules — 16 bytes per id

Each DST id owns a pair of 8-byte `TimezoneDSTRule` records, start rule
then end rule:

| Field            | Size | Notes                                                                                                                                                                          |
| ---------------- | ---- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| `ds_label`       | 1    | `'D'` enter DST, `'S'` leave, `'\0'` none                                                                                                                                      |
| `wday`           | 1    | 0 = Sunday; 255 = any day                                                                                                                                                      |
| `flag`           | 1    | bit 0: "last <wday>" (day-decrement); bit 1: rule time is standard time; bit 2: UTC; bit 3: wall time (written by the builder, never read — wall time is the firmware default) |
| `month`          | 1    | 0-based                                                                                                                                                                        |
| `mday`           | 1    | 1-based                                                                                                                                                                        |
| `hour`, `minute` | 2    |                                                                                                                                                                                |
| padding          | 1    |                                                                                                                                                                                |

Rule id 0 ("no DST") is counted in `dst_rule_count` but not stored, so
rule id _n_ lives at `(n - 1) × 16` bytes into the section. The mapping
from tzdata rule names to ids (`dstzone_list` in `tools/timezones.py`)
is append-only: the firmware hardcodes some ids (e.g. Brazil = 6), so
the order must never change between database releases.

## Link records — 35 bytes each

`uint16_t region_id` plus a 33-byte NUL-padded alias (sized for the
longest IANA name). Links resolve legacy aliases — old mobile OSes still
send names like `US/Pacific` — to a region record.
