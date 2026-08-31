/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include <pbl/logging/logging.h>

#include "ams_util.h"

#include "pbl/util/math.h"

// -------------------------------------------------------------------------------------------------
// Parsing C-string with real number to an integer using a given multiplication factor

bool ams_util_float_string_parse(const char *number_str, uint32_t number_str_length,
                                 int32_t multiplier, int32_t *number_out) {
  if (!number_str || number_str_length == 0 || number_str[0] == 0 || multiplier == 0) {
    return false;
  }
  const int32_t base = 10;
  bool has_more = true;
  bool is_negative = false;
  bool number_started = false;
  uint32_t decimal_divisor = 0;
  int64_t result = 0;
  const char * const number_str_end = number_str + number_str_length;
  do {
    const char c = *number_str;
    switch (c) {
      case '\0':
        has_more = false;
        break;

      case '0' ... '9': {
        number_started = true;
        result *= base;
        result += (c - '0') * multiplier;
        if (decimal_divisor) {
          decimal_divisor *= base;
        }
        break;
      }

      case '-': {
        if (number_started || is_negative) {
          return false;  // Encountered minus in the middle of a number or multiple minus signs
        }
        is_negative = true;
        break;
      }

      case ',':
      case '.': {
        number_started = true;
        if (decimal_divisor) {
          return false;  // Encountered multiple separators
        }
        decimal_divisor = 1;
        break;
      }

      default:
        return false;
    }
  } while (has_more && (++number_str < number_str_end));

  if (decimal_divisor > 1) {
    result /= (decimal_divisor / base);
    const bool round_up = ((ABS(result) % base) >= (base / 2));
    result /= base;
    if (round_up) {
      if (result > 0) {
        ++result;
      } else {
        --result;
      }
    }
  }

  if (is_negative) {
    result *= -1;
  }

  if (result > INT32_MAX || result < INT32_MIN) {
    return false;  // overflow
  }

  if (number_started && number_out) {
    *number_out = result;
  }

  return number_started;
}

// -------------------------------------------------------------------------------------------------
// Parsing comma-separated value

uint8_t ams_util_csv_parse(const char *csv_value, uint32_t csv_length,
                           void *context, AMSUtilCSVCallback callback) {
  if (csv_value == NULL || csv_length == 0) {
    return 0;
  }

  uint8_t values_parsed_count = 0;
  bool should_continue = true;
  const char *value_begin = csv_value;
  while (should_continue) {
    uint32_t idx = 0;
    const uint8_t end_idx = (csv_value + csv_length) - value_begin;
    while (true) {
      const char c = value_begin[idx];
      const bool is_terminated = (c == '\0' || idx == end_idx);
      if (c == ',' || is_terminated) {
        should_continue = callback(value_begin, idx, values_parsed_count, context);
        value_begin += idx + 1;
        ++values_parsed_count;
        if (is_terminated) {
          goto finally;
        }
        break;
      }
      ++idx;
    }
  }
finally:
  return values_parsed_count;
}
