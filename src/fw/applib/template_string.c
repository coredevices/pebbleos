/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "template_string.h"
#include "template_string_private.h"

#include "pbl/services/i18n/i18n.h"
#include "syscall/syscall.h"
#include "system/passert.h"
#include "pbl/util/attributes.h"
#include "pbl/util/math.h"
#include "pbl/util/size.h"

#include <ctype.h>
#include <limits.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#define MAX_FILTER_NAME_LENGTH 16

#define SUPPORT_MONTH 0
#define SUPPORT_YEAR 0

typedef enum {
  PredicateCondition_Invalid,
  // This order is important; >= must be next after >, and <= must be next after <.
  // This is so that the parser can just add 1 to the enum value when it finds the = character.
  PredicateCondition_L,
  PredicateCondition_LE,
  PredicateCondition_G,
  PredicateCondition_GE,
} PredicateCondition;

typedef enum {
  FormatUnits_None,
  FormatUnits_Abbreviated,
  FormatUnits_Full,
} FormatUnits;

typedef struct {
  const char name[MAX_FILTER_NAME_LENGTH];
  // Filters must manually advance state->position to the parenthesis that indicates the end of
  // the filter arguments.
  void (*cb)(TemplateStringState *state);
} FilterImplementation;

static void prv_filter_format(TemplateStringState *state);
static void prv_filter_time_until(TemplateStringState *state);
static void prv_filter_time_since(TemplateStringState *state);
static void prv_filter_end(TemplateStringState *state);

static const FilterImplementation s_filter_impls[] = {
  { "format", prv_filter_format, },
  { "time_until", prv_filter_time_until, },
  { "time_since", prv_filter_time_since, },
  { "end", prv_filter_end, },
};

static void prv_handle_escape_character(TemplateStringState *state) {
  if (*state->position == '\\') {
    state->position++;
    if (*state->position == '\0') {
      state->error->status = TemplateStringErrorStatus_InvalidEscapeCharacter;
    }
  }
}

static bool prv_predicate_check(char ch) {
  return ((ch == '>') || (ch == '<'));
}

static bool prv_format_string_ending(char ch) {
  return ((ch == ',') || (ch == ')'));
}

static bool prv_predicate_valid_splitter(char ch) {
  return ((ch == ':') || prv_format_string_ending(ch));
}

T_STATIC intmax_t prv_template_predicate_time(TemplateStringState *state) {
  bool negative = false;
  if (*state->position == '-') {
    negative = true;
    state->position++;
  }

  bool total_value_valid = false;
  intmax_t total_value = 0;

  while (!total_value_valid || !prv_predicate_valid_splitter(*state->position)) {
    const char *endpos;
    int value = strtol(state->position, (char **)&endpos, 10);
    if (endpos == state->position) {
      state->error->status = TemplateStringErrorStatus_InvalidTimeUnit;
      return 0;
    }
    state->position = endpos;
    int multiplier = 1;
    switch (*state->position) {
      // NOTE: This number of seconds is a hack! See PBL-39903
#if SUPPORT_YEAR
      case 'y': multiplier = 365 * SECONDS_PER_DAY; break;
#endif
      // NOTE: This number of seconds is a hack! See PBL-39903
#if SUPPORT_MONTH
      case 'm': multiplier = 30 * SECONDS_PER_DAY; break;
#endif
      case 'd': multiplier = SECONDS_PER_DAY; break;
      case 'H': multiplier = SECONDS_PER_HOUR; break;
      case 'M': multiplier = SECONDS_PER_MINUTE; break;
      case 'S': multiplier = 1; break;
      default:
        state->error->status = TemplateStringErrorStatus_InvalidTimeUnit;
        return 0;
    }
    state->position++;
    total_value += value * multiplier;
    total_value_valid = true;
  }
  if (negative) {
    total_value = -total_value;
  }

  return total_value;
}


T_STATIC bool prv_template_predicate_match(TemplateStringState *state, PredicateCondition *cond,
                                           intmax_t *value) {
  *cond = PredicateCondition_Invalid;
  if (*state->position == '<') {
    *cond = PredicateCondition_L;
  } else if (*state->position == '>') {
    *cond = PredicateCondition_G;
  } else {
    WTF;
  }
  state->position++;

  if (*state->position == '=') {
    (*cond)++;
    state->position++;
  } else if (!isdigit((unsigned char)*state->position)) {
    state->error->status = TemplateStringErrorStatus_InvalidTimeUnit;
    return false;
  }

  *value = prv_template_predicate_time(state);
  if (state->error->status != TemplateStringErrorStatus_Success) {
    return false;
  }

  switch (*cond) {
    case PredicateCondition_G:
      return (state->filter_state > *value);
    case PredicateCondition_GE:
      return (state->filter_state >= *value);
    case PredicateCondition_L:
      return (state->filter_state < *value);
    case PredicateCondition_LE:
      return (state->filter_state <= *value);
    default:
      WTF;
  }
}

static const char * const s_Tstrings[3][3] = {
  { "H",  "M",  "S",  },
  { "aH", "aM", "aS", },
  { "uH", "uM", "uS", },
};

static const char * const s_splitters[3][2] = {
  /// The first separator in `<hour>:<minute>:<second>`
  { i18n_ctx_noop("TmplStringSep", ":"),
  /// The second separator in `<hour>:<minute>:<second>`
    i18n_ctx_noop("TmplStringSep", ":"), },
  /// The first separator in `<hour> hr <minute> min <second> sec`
  { i18n_ctx_noop("TmplStringSep", " "),
  /// The second separator in `<hour> hr <minute> min <second> sec`
    i18n_ctx_noop("TmplStringSep", " "), },
  /// The first separator in `<hour> hours, <minute> minutes, and <second> seconds`
  { i18n_ctx_noop("TmplStringSep", ", "),
  /// The second separator in `<hour> hours, <minute> minutes, and <second> seconds`
    i18n_ctx_noop("TmplStringSep", ", and "), },
};

/*
Flag truth table.

fmt   = >= 1 hour               >= 1 minute             other
%T    = %H:%0M:%0S              %M:%0S                  %S
%uT   = %uH, %uM, and %uS       %uM, and %uS            %uS
%aT   = %aH %aM %aS             %aM %aS                 %aS
%0T   = %0H:%0M:%0S             %0M:%0S                 %0S
%fT   = %fH:%0M:%0S             %fM:%0S                 %fS

%0uT  = %0uH, %0uM, and %0uS    %0uM, and %0uS          %0uS
%0aT  = %0aH %0aM %0aS          %0aM %0aS               %0aS
%0fT  = %0fH:%0M:%0S            %0fM:%0S                %0fS

%fuT  = %fuH, %uM, and %uS      %fuM, and %uS           %fuS
%faT  = %faH %aM %aS            %faM %aS                %faS

%0fuT = %f0uH, %0uM, and %0uS   %f0uM, and %0uS         %f0uS
%0faT = %f0aH %0aM %0aS         %f0aM %0aS              %f0aS
*/

/*
0 flag adds 0 flag to all sub-specs
f flag adds f flag to first sub-spec
ua. are unique
*/

static void prv_append_string_i18n(TemplateStringState *state, const char *str) {
  size_t len = sys_i18n_get_length(str);
  if (state->output_remaining < len) {
    len = state->output_remaining;
  }
  if (!len) {
    return;
  }
  // len needs +1 in order for i18n_get_with_buffer to write all the characters we want.
  // It's ok that it writes the NUL at the end because we've already reserved that space.
  sys_i18n_get_with_buffer(str, state->output, len + 1);
  state->output += len;
  state->output_remaining -= len;
}

static void prv_append_number(TemplateStringState *state, const char *fmt, int value) {
  size_t len = snprintf(NULL, 0, fmt, value);
  if (state->output_remaining < len) {
    len = state->output_remaining;
  }
  if (!len) {
    return;
  }
  // len needs +1 in order for snprintf to write all the characters we want.
  // It's ok that it writes the NUL at the end because we've already reserved that space.
  snprintf(state->output, len + 1, fmt, value);
  state->output += len;
  state->output_remaining -= len;
}

static void prv_append_char(TemplateStringState *state, char c) {
  if (state->output_remaining >= 1) {
    *state->output++ = c;
    state->output_remaining--;
  }
}

// Suffix columns: [0] singular, [1] plural ("few" for languages like Polish:
// 2-4), [2] "many" (Polish: 0 and 5+). prv_plural_index() picks the column;
// two-form languages never select [2].
static const char * const s_second_strings[3][3] = {
  /// Suffixes for seconds with no units
  { i18n_ctx_noop("TmplStringSing", ""),
    i18n_ctx_noop("TmplStringPlur", ""),
    i18n_ctx_noop("TmplStringMany", "")},
  /// Suffixes for seconds with abbreviated units
  { i18n_ctx_noop("TmplStringSing", " sec"),
    i18n_ctx_noop("TmplStringPlur", " sec"),
    i18n_ctx_noop("TmplStringMany", " sec")},
  /// Suffixes for seconds with full units
  { i18n_ctx_noop("TmplStringSing", " second"),
    i18n_ctx_noop("TmplStringPlur", " seconds"),
    i18n_ctx_noop("TmplStringMany", " seconds")},
};

static const char * const s_minute_strings[3][3] = {
  /// Suffixes for minutes with no units
  { i18n_ctx_noop("TmplStringSing", ""),
    i18n_ctx_noop("TmplStringPlur", ""),
    i18n_ctx_noop("TmplStringMany", "")},
  /// Suffixes for minutes with abbreviated units
  { i18n_ctx_noop("TmplStringSing", " min"),
    i18n_ctx_noop("TmplStringPlur", " min"),
    i18n_ctx_noop("TmplStringMany", " min")},
  /// Suffixes for minutes with full units
  { i18n_ctx_noop("TmplStringSing", " minute"),
    i18n_ctx_noop("TmplStringPlur", " minutes"),
    i18n_ctx_noop("TmplStringMany", " minutes")},
};

static const char * const s_hour_strings[3][3] = {
  /// Suffixes for hours with no units
  { i18n_ctx_noop("TmplStringSing", ""),
    i18n_ctx_noop("TmplStringPlur", ""),
    i18n_ctx_noop("TmplStringMany", "")},
  /// Suffixes for hours with abbreviated units
  { i18n_ctx_noop("TmplStringSing", " hr"),
    i18n_ctx_noop("TmplStringPlur", " hr"),
    i18n_ctx_noop("TmplStringMany", " hr")},
  /// Suffixes for hours with full units
  { i18n_ctx_noop("TmplStringSing", " hour"),
    i18n_ctx_noop("TmplStringPlur", " hours"),
    i18n_ctx_noop("TmplStringMany", " hours")},
};

static const char * const s_day_strings[3][3] = {
  /// Suffixes for days with no units
  { i18n_ctx_noop("TmplStringSing", ""),
    i18n_ctx_noop("TmplStringPlur", ""),
    i18n_ctx_noop("TmplStringMany", "")},
  /// Suffixes for days with abbreviated units
  { i18n_ctx_noop("TmplStringSing", " d"),
    i18n_ctx_noop("TmplStringPlur", " d"),
    i18n_ctx_noop("TmplStringMany", " d")},
  /// Suffixes for days with full units
  { i18n_ctx_noop("TmplStringSing", " day"),
    i18n_ctx_noop("TmplStringPlur", " days"),
    i18n_ctx_noop("TmplStringMany", " days")},
};

#if SUPPORT_MONTH
static const char * const s_month_strings[3][3] = {
  /// Suffixes for months with no units
  { i18n_ctx_noop("TmplStringSing", ""),
    i18n_ctx_noop("TmplStringPlur", ""),
    i18n_ctx_noop("TmplStringMany", "")},
  /// Suffixes for months with abbreviated units
  { i18n_ctx_noop("TmplStringSing", " mo"),
    i18n_ctx_noop("TmplStringPlur", " mo"),
    i18n_ctx_noop("TmplStringMany", " mo")},
  /// Suffixes for months with full units
  { i18n_ctx_noop("TmplStringSing", " month"),
    i18n_ctx_noop("TmplStringPlur", " months"),
    i18n_ctx_noop("TmplStringMany", " months")},
};
#endif

#if SUPPORT_YEAR
static const char * const s_year_strings[3][3] = {
  /// Suffixes for years with no units
  { i18n_ctx_noop("TmplStringSing", ""),
    i18n_ctx_noop("TmplStringPlur", ""),
    i18n_ctx_noop("TmplStringMany", "")},
  /// Suffixes for years with abbreviated units
  { i18n_ctx_noop("TmplStringSing", " yr"),
    i18n_ctx_noop("TmplStringPlur", " yr"),
    i18n_ctx_noop("TmplStringMany", " yr")},
  /// Suffixes for years with full units
  { i18n_ctx_noop("TmplStringSing", " year"),
    i18n_ctx_noop("TmplStringPlur", " years"),
    i18n_ctx_noop("TmplStringMany", " years")},
};
#endif

// Pick the plural suffix column for the active locale.
//   0 = singular, 1 = plural / "few", 2 = "many"
// Most languages have a two-form rule (singular vs plural) and never select
// column 2. Polish has three forms; its rule mirrors the Plural-Forms header
// in the pl_PL catalog.
static int prv_plural_index(intmax_t value) {
  char locale[ISO_LOCALE_LENGTH];
  sys_i18n_get_locale(locale);
  uintmax_t n = (value < 0) ? (uintmax_t)(-value) : (uintmax_t)value;

  if (locale[0] == 'p' && locale[1] == 'l') {
    // Polish: 1 -> one; n%10 in 2..4 (but not n%100 in 12..14) -> few; else many.
    if (n == 1) {
      return 0;
    }
    unsigned n10 = n % 10, n100 = n % 100;
    if (n10 >= 2 && n10 <= 4 && (n100 < 10 || n100 >= 20)) {
      return 1;
    }
    return 2;
  }

  // Default two-form rule: 1 -> singular, everything else -> plural.
  return (n == 1) ? 0 : 1;
}

static void prv_do_conversion(TemplateStringState *state, intmax_t value, int divide, int mod,
                              const char * const suffix_strings[3][3], FormatUnits add_units,
                              bool zero_pad, bool should_mod) {
  int remain = (value % divide);
  if (!state->time_was_until) {
    // We want to go in reverse for 'since'
    remain = divide - remain;
  } else {
    // Add 1 because the next eval time is how long until the result changes.
    remain++;
  }

  if (state->eval_cond) {
    if (remain < state->eval_cond->eval_time) {
      state->eval_cond->eval_time = remain;
    }
  }

  value /= divide;
  if (should_mod && (mod != 0)) {
    value %= mod;
  }
  prv_append_number(state, zero_pad ? "%02d" : "%d", value);
  prv_append_string_i18n(state, suffix_strings[add_units][prv_plural_index(value)]);
}

// This is a recursive function, so watch out!
// The recursion happens on the %R and %T cases only, and will only recurse once.
// So when adding stack variables, realize the stack usage may be doubled!
T_STATIC const char *prv_template_format_specifier(TemplateStringState *state, const char *input,
                                                   intmax_t value) {
  if (*input == '%') { // Escaped %
    prv_append_char(state, *input);
    input++;
    return input;
  }

  FormatUnits add_units = FormatUnits_None;
  bool zero_pad = false;
  bool modulus = true;
  bool checking_flags = true;
  while (checking_flags) {
    switch (*input) {
      case 'a':
        add_units = FormatUnits_Abbreviated;
        break;
      case 'u':
        add_units = FormatUnits_Full;
        break;
      case '-':
        value = -value;
        break;
      case '0':
        zero_pad = true;
        break;
      case 'f':
        modulus = false;
        break;
      default:
        checking_flags = false;
        break;
    }
    if (checking_flags) {
      input++;
    }
  }
  if (value < 0) {
    prv_append_char(state, '-');
    value = -value;
  }

  int macro_units = 2;
  if (value >= SECONDS_PER_MINUTE) {
    macro_units--;
  }
  if (value >= SECONDS_PER_HOUR) {
    macro_units--;
  }
  int macro_end = 3;

  // conversion specifiers
  switch (*input) {
#if SUPPORT_YEAR
    case 'y': // year
      // NOTE: This number of seconds to divide by is a hack! See PBL-39903
      prv_do_conversion(state, value, 365 * SECONDS_PER_DAY, 100, s_year_strings,
                        add_units, zero_pad, modulus);
      break;
#endif
#if SUPPORT_MONTH
    case 'm': // month
      // NOTE: This number of seconds to divide by is a hack! See PBL-39903
      prv_do_conversion(state, value, 30 * SECONDS_PER_DAY, 12, s_month_strings,
                        add_units, zero_pad, modulus);
      break;
#endif
    case 'd': // day
      // NOTE: This number of modulus is a hack! See PBL-39903
#if SUPPORT_MONTH
      prv_do_conversion(state, value, SECONDS_PER_DAY, 30, s_day_strings,
                        add_units, zero_pad, modulus);
#else
      prv_do_conversion(state, value, SECONDS_PER_DAY, 0, s_day_strings,
                        add_units, zero_pad, modulus);
#endif
      break;
    case 'H': // hour
      prv_do_conversion(state, value, SECONDS_PER_HOUR, HOURS_PER_DAY, s_hour_strings,
                        add_units, zero_pad, modulus);
      break;
    case 'M': // minute
      prv_do_conversion(state, value, SECONDS_PER_MINUTE, MINUTES_PER_HOUR, s_minute_strings,
                        add_units, zero_pad, modulus);
      break;
    case 'S': // second
      prv_do_conversion(state, value, 1, SECONDS_PER_MINUTE, s_second_strings,
                        add_units, zero_pad, modulus);
      break;
    case 'R': // H:M
      // R is mostly the same as T, just without seconds.
      macro_end--;
      // fall-thru
    case 'T': { // H:M:S
      char macro_spec[16];
      // Always show the last unit, even if it's 0.
      macro_units = MIN(macro_units, macro_end - 1);

      for (int i = macro_units; i < macro_end; i++) {
        char *macro_ptr = macro_spec;
        if (zero_pad || ((i != macro_units) && (add_units == FormatUnits_None))) {
          *macro_ptr++ = '0';
        }
        if (!modulus && (i == macro_units)) {
          *macro_ptr++ = 'f';
        }
        strcpy(macro_ptr, s_Tstrings[add_units][i]);
        macro_ptr += strlen(s_Tstrings[add_units][i]);
        *macro_ptr = '\0';
        prv_template_format_specifier(state, macro_spec, value);
        if (i != macro_end - 1) {
          prv_append_string_i18n(state, s_splitters[add_units][i >= macro_end - 2]);
        }
      }
      break;
    }
    default:
      state->error->status = TemplateStringErrorStatus_InvalidConversionSpecifier;
      return input;
  }
  // Skip the conversion specifier.
  input++;
  return input;
}

static bool prv_format_predicate(TemplateStringState *state, bool previously_matched) {
  bool match = true;
  if (!prv_predicate_check(*state->position)) {
    return match;
  }

  PredicateCondition predicate_cond;
  intmax_t predicate_value = 0;
  // If this is a predicate, we need to evaluate it for a match.
  match = prv_template_predicate_match(state, &predicate_cond, &predicate_value);
  if (state->error->status != TemplateStringErrorStatus_Success) {
    return false;
  }
  // Predicate matcher will only leave on :,) or error, so this should never trip.
  if (!prv_predicate_valid_splitter(*state->position)) {
    WTF;
  }

  int wait_time;
  // Need to handle predicates differently based on whether the value is incrementing or
  // decrementing over time.
  PredicateCondition cond_to_expire;
  PredicateCondition cond_to_valid;
  if (!state->time_was_until) {
    // Value increments over time, so a < will expire and a > will become valid.
    cond_to_expire = PredicateCondition_L;
    cond_to_valid = PredicateCondition_G;
    wait_time = predicate_value - state->filter_state;
  } else {
    // Value decrements over time, so a < will become valid and a > will expire.
    cond_to_expire = PredicateCondition_G;
    cond_to_valid = PredicateCondition_L;
    wait_time = state->filter_state - predicate_value;
  }

  if (!previously_matched && match && ((predicate_cond == cond_to_expire) ||
                                       (predicate_cond == cond_to_expire + 1))) {
    // This predicate could expire over time.

    // If the conditional is equal, add 1 to the wait time, because the equals case stays
    // valid on the specified value.
    if (predicate_cond == cond_to_expire + 1) {
      wait_time++;
    }
  } else if (!match && ((predicate_cond == cond_to_valid) ||
                        (predicate_cond == cond_to_valid + 1))) {
    // This predicate could become valid over time.

    // If the conditional is not equal, add 1 to the wait time, because only the equals case
    // becomes valid on the specified value.
    if (predicate_cond == cond_to_valid) {
      wait_time++;
    }
  } else {
    wait_time = INT_MAX;
  }

  if (state->eval_cond) {
    if (wait_time < state->eval_cond->eval_time) {
      state->eval_cond->eval_time = wait_time;
    }
  }

  // Only characters possible here are :,)
  if (*state->position == ':') {
    state->position++;
  }
  return match;
}

static void prv_format_process_format_string(TemplateStringState *state, char delimiter) {
  // Predicate matched (or is default case), so let's parse the string.
  while ((*state->position != delimiter) && (*state->position != '\0')) {
    if (*state->position != '%') { // Not a format character
      prv_handle_escape_character(state);
      if (state->error->status != TemplateStringErrorStatus_Success) {
        return;
      }
      prv_append_char(state, *state->position);
      state->position++;
    } else {
      // Skip over the %
      state->position++;
      state->position = prv_template_format_specifier(state, state->position,
                                                      state->filter_state);
      if (state->error->status != TemplateStringErrorStatus_Success) {
        return;
      }
    }
  }
  if (*state->position == '\0') {
    state->error->status = TemplateStringErrorStatus_MissingClosingQuote;
    return;
  }
  // Skip the delimiter
  state->position++;
}

static void prv_format_skip_format_string(TemplateStringState *state, char delimiter) {
  // No match, so let's move along to the next one.
  while ((*state->position != delimiter) && (*state->position != '\0')) {
    prv_handle_escape_character(state);
    if (state->error->status != TemplateStringErrorStatus_Success) {
      return;
    }
    state->position++;
  }
  if (*state->position == '\0') {
    state->error->status = TemplateStringErrorStatus_MissingClosingQuote;
    return;
  }
  // Skip the delimiter
  state->position++;
}

static void prv_filter_format(TemplateStringState *state) {
  bool match;
  bool previously_matched = false;
  bool did_output = false;

  // We need to iterate all the way through for finding the proper 'next' time.
  while (*state->position != ')') {
    match = prv_format_predicate(state, previously_matched);
    if (state->error->status != TemplateStringErrorStatus_Success) {
      return;
    }

    // A force-default case
    if (prv_format_string_ending(*state->position)) {
      state->position++;
      continue;
    }

    // Get the delimiter being used.
    const char delimiter = *state->position;
    if ((delimiter != '\'') && (delimiter != '"')) {
      state->error->status = TemplateStringErrorStatus_MissingOpeningQuote;
      return;
    }
    state->position++;

    if (match && !previously_matched) {
      prv_format_process_format_string(state, delimiter);
      did_output = true;
      previously_matched = true;
    } else {
      prv_format_skip_format_string(state, delimiter);
    }

    if (state->error->status != TemplateStringErrorStatus_Success) {
      return;
    }

    if (!prv_format_string_ending(*state->position)) {
      state->error->status = TemplateStringErrorStatus_InvalidArgumentSeparator;
      return;
    } else if (*state->position == ',') {
      state->position++;
      if (*state->position == ')') {
        state->error->status = TemplateStringErrorStatus_MissingArgument;
        return;
      }
    }
  }

  if (!did_output) {
    // If no output was generated, it's an error.
    state->error->status = TemplateStringErrorStatus_CantResolve;
  }

  // format() must be the last filter, and ends the sequence.
  state->filters_complete = true;
}

static void prv_filter_time_until(TemplateStringState *state) {
  char *endptr;
  time_t target_time = strtol(state->position, &endptr, 10);
  if (*endptr != ')') {
    state->error->status = TemplateStringErrorStatus_MissingClosingParen;
    return;
  }
  state->position = endptr;
  state->filter_state = target_time - state->vars->current_time;
  state->time_was_until = true;
}

static void prv_filter_time_since(TemplateStringState *state) {
  prv_filter_time_until(state);
  if (state->error->status != TemplateStringErrorStatus_Success) {
    return;
  }
  state->filter_state = -state->filter_state;
  state->time_was_until = false;
}

static void prv_filter_end(TemplateStringState *state) {
  state->filters_complete = true;
}

T_STATIC void prv_template_evaluate_filter(TemplateStringState *state, const char *filter_name,
                                           const char *parameters_start) {
  for (size_t i = 0; i < ARRAY_LENGTH(s_filter_impls); i++) {
    if (strcmp(s_filter_impls[i].name, filter_name) == 0) {
      state->position = parameters_start;
      s_filter_impls[i].cb(state);
      return;
    }
  }
  state->error->status = TemplateStringErrorStatus_UnknownFilter;
}

static void prv_template_eval(TemplateStringState *state) {
  while ((*state->position != '}') && (*state->position != '\0')) {
    if (state->filters_complete) {
      state->error->status = TemplateStringErrorStatus_FormatBeforeLast;
      return;
    }
    // Find the filter's opening paren
    const char *filter_name_paren = strchr(state->position, '(');
    if (!filter_name_paren) {
      state->error->status = TemplateStringErrorStatus_MissingOpeningParen;
      return;
    }

    // Copy out the filter name
    char filter_name[MAX_FILTER_NAME_LENGTH];
    size_t len = MIN(MAX_FILTER_NAME_LENGTH - 1, filter_name_paren - state->position);
    strncpy(filter_name, state->position, len);
    filter_name[len] = '\0';

    prv_template_evaluate_filter(state, filter_name, filter_name_paren + 1);
    if (state->error->status != TemplateStringErrorStatus_Success) {
      return;
    }

    if (*state->position != ')') {
      state->error->status = TemplateStringErrorStatus_MissingClosingParen;
      return;
    }

    // Advance pointer to the character after the filter
    state->position++;

    if (*state->position == '|') {
      state->position++;
      continue;
    } else if (*state->position == '}') {
      continue;
    } else {
      state->error->status = TemplateStringErrorStatus_MissingClosingBrace;
      return;
    }
  }

  // Must end on a closing brace.
  if (*state->position != '}') {
    state->error->status = TemplateStringErrorStatus_MissingClosingBrace;
    return;
  }
  // Did not generate an output.
  if (!state->filters_complete) {
    state->error->status = TemplateStringErrorStatus_NoResultGenerated;
    return;
  }
  // Skip past the closing brace.
  state->position++;
}

bool template_string_evaluate(const char *input_template_string, char *output, size_t output_size,
                              TemplateStringEvalConditions *eval_cond,
                              const TemplateStringVars *vars, TemplateStringError *error) {
  TemplateStringState state = {
    .position = input_template_string,
    .output = output,
    .output_remaining = output_size,
    .eval_cond = eval_cond,
    .vars = vars,
    .error = error,

    .time_was_until = false,
    .filter_state = 0,
  };

  if (!state.position || !state.vars || !state.error) {
    if (state.error) {
      state.error->status = TemplateStringErrorStatus_InvalidParameter;
    }
    return false;
  }

  // We have no output space, so don't bother trying to write anything.
  // By unifying these states, we can just check `output_remaining` against zero for writing.
  if (!state.output || !state.output_remaining) {
    state.output = NULL;
    state.output_remaining = 0;
  } else {
    // Subtract 1 for the null terminator.
    state.output_remaining--;
  }

  if (state.eval_cond) {
    state.eval_cond->eval_time = INT_MAX;
    state.eval_cond->force_eval_on_time = false;
  }

  state.error->status = TemplateStringErrorStatus_Success;

  while (*state.position != '\0') {
    if (*state.position != '{') {
      prv_handle_escape_character(&state);
      if (state.error->status != TemplateStringErrorStatus_Success) {
        break;
      }
      prv_append_char(&state, *state.position);
      state.position++;
    } else { // Template
      state.position++;
      prv_template_eval(&state);
      if (state.error->status != TemplateStringErrorStatus_Success) {
        break;
      }
      state.time_was_until = false;
      state.filter_state = 0;
      state.filters_complete = false;
    }
  }

  if (state.error->status != TemplateStringErrorStatus_Success) {
    // get the position index.
    state.error->index_in_string = state.position - input_template_string;
  }

  // Null terminator
  if (state.output) {
    *state.output = '\0';
  }

  if (state.eval_cond) {
    // Adjust eval_time if it never got set.
    if (state.eval_cond->eval_time == INT_MAX) {
      // If we never set the re-evaluation time, set `eval_time` to 0.
      // This is the value we specified for "we don't need to re-evaluate".
      state.eval_cond->eval_time = 0;
    } else {
      // `eval_time` is an absolute timestamp, so add the input time to the relative time offset.
      state.eval_cond->eval_time += state.vars->current_time;
      state.eval_cond->force_eval_on_time = true;
    }
  }

  return (state.error->status == TemplateStringErrorStatus_Success);
}
