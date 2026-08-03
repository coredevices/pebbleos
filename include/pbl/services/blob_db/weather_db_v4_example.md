# Weather BlobDB v4 record — worked example

A fully-populated **v4** `WeatherDBEntry` (see `weather_db.h`), field-by-field with
real numbers and the exact bytes, so the phone side can encode it and diff against
a known-good reference. Offsets below were read straight from the compiled firmware
(`arm-none-eabi-gdb` on `tintin_fw.elf`) — they are authoritative.

## Width rules (read this first — they are NOT obvious)
- The struct is **`PACKED`** — no padding between fields.
- All multi-byte integers are **little-endian**.
- **`WeatherType` is 1 byte** here (`current_weather_type`, `tomorrow_weather_type`).
  The firmware shrinks the enum — do NOT encode it as 4 bytes.
- **`time_t` (`last_update_time_utc`) is 4 bytes** (int32, unix seconds).
- `bool` (`is_current_location`) is 1 byte.
- Fixed portion = **118 bytes**; then the trailing string block.
  `sizeof(WeatherDBEntry)` = 120 (118 + the 2-byte `SerializedArray` header).

## The sample: "San Francisco", currently 18° and Sunny

| Off | Sz | Field | Value | Bytes (LE) |
|----:|---:|-------|-------|------------|
| 0  | 1 | version                  | 4 (v4)            | `04` |
| 1  | 2 | current_temp             | 18                | `12 00` |
| 3  | 1 | current_weather_type     | 7 = Sun           | `07` |
| 4  | 2 | today_high_temp          | 21                | `15 00` |
| 6  | 2 | today_low_temp           | 13                | `0D 00` |
| 8  | 1 | tomorrow_weather_type    | 0 = PartlyCloudy  | `00` |
| 9  | 2 | tomorrow_high_temp       | 18                | `12 00` |
| 11 | 2 | tomorrow_low_temp        | 11                | `0B 00` |
| 13 | 4 | last_update_time_utc     | 1750000000        | `80 E1 4E 68` |
| 17 | 1 | is_current_location      | true              | `01` |
| 18 | 1 | minor_version            | 0                 | `00` |
| 19 | 2 | today_feels_like_temp    | 17                | `11 00` |
| 21 | 2 | today_uv_index_x10       | 50  (UV 5.0)      | `32 00` |
| 23 | 2 | today_precip_probability | 20  (%)           | `14 00` |
| 25 | 2 | today_wind_speed         | 12                | `0C 00` |
| 27 | 2 | today_wind_direction     | 270 (degrees)     | `0E 01` |
| 29 | 2 | latitude_e2              | 3777  (37.77 N)   | `C1 0E` |
| 31 | 2 | longitude_e2             | -12242 (122.42 W) | `2E D0` |
| 33 | 1 | num_daily                | 7                 | `07` |
| 34 | 35| daily[7]                 | see below         | … |
| 69 | 1 | today_hourly_count       | 24                | `18` |
| 70 | 24| today_hourly_weather_type[24] | all 7 (Sun)  | `07` ×24 |
| 94 | 24| today_hourly_temp[24]    | diurnal curve     | see below |
| 118| var| pstring16s              | "San Francisco","Clear" | see below |

### daily[7] — each entry is 5 bytes: `int16 high, int16 low, uint8 type`
| Day | high | low | type | Bytes |
|----:|----:|---:|------|-------|
| 0 (today)    | 21 | 13 | 7 Sun          | `15 00 0D 00 07` |
| 1 (tomorrow) | 18 | 11 | 0 PartlyCloudy | `12 00 0B 00 00` |
| 2 | 23 | 11 | 1 CloudyDay  | `17 00 0B 00 01` |
| 3 | 13 |  5 | 3 LightRain  | `0D 00 05 00 03` |
| 4 | 21 | 10 | 4 HeavyRain  | `15 00 0A 00 04` |
| 5 | 16 |  7 | 0 PartlyCloudy | `10 00 07 00 00` |
| 6 | 20 | 12 | 7 Sun        | `14 00 0C 00 07` |

### today_hourly_temp[24] (int8, °, hours 0..23)
`13 13 13 13 13 13 13 14 15 16 17 18 19 20 20 21 20 20 19 17 16 15 15 14`
(today_hourly_weather_type[24] is all `07` = Sun in this sample.)

### The trailing string block (offset 118)
`SerializedArray { uint16 data_size; <bytes> }` wrapping two `PascalString16`
values in order: **[0] LocationName, [1] ShortPhrase**.
`PascalString16 { uint16 str_length; char str_value[str_length] }` (no null).

```
16 00                                            data_size = 22
0D 00 53 61 6E 20 46 72 61 6E 63 69 73 63 6F     len=13 "San Francisco"
05 00 43 6C 65 61 72                             len=5  "Clear"
```

## Full record (142 bytes) — diff your encoder against this
```
04 12 00 07 15 00 0D 00 00 12 00 0B 00 80 E1 4E
68 01 00 11 00 32 00 14 00 0C 00 0E 01 C1 0E 2E
D0 07 15 00 0D 00 07 12 00 0B 00 00 17 00 0B 00
01 0D 00 05 00 03 15 00 0A 00 04 10 00 07 00 00
14 00 0C 00 07 18 07 07 07 07 07 07 07 07 07 07
07 07 07 07 07 07 07 07 07 07 07 07 07 07 0D 0D
0D 0D 0D 0D 0D 0E 0F 10 11 12 13 14 14 15 14 14
13 11 10 0F 0F 0E 16 00 0D 00 53 61 6E 20 46 72
61 6E 63 69 73 63 6F 05 00 43 6C 65 61 72
```

## "Unknown" sentinels (when a value isn't available)
- temperatures (current/today/tomorrow/feels-like/daily high+low): `INT16_MAX` = 32767
- today_uv_index_x10: `-1`   · today_precip_probability: `-1`
- today_wind_speed: `0`      · today_wind_direction: `0xFFFF` (65535)
- latitude_e2 / longitude_e2: `INT16_MIN` = -32768
- daily[].weather_type: `255`

## Validate
Encode the sample values above on the phone, write to the weather BlobDB, and the
bytes must match the 142-byte block. The watch firmware reads both v3 and v4; send
v4 only when it advertises the `weather_db_v4_support` capability bit.
