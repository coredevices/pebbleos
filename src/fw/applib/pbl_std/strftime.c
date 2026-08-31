/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

// This was bad and huge and ugly. Now it's good and small and ugly.

// NOTE: PBL-22056
// Our old strftime had a bug where a negative gmtoff that wasn't at least an hour would still show
// up as positive for %z. Obviously this is wrong, but in the interest of compatibility and code
// size, we're keeping it.

// NOTE:
// Our old strftime had support for the POSIX-2008 '+' flag. Because it takes a lot of code to
// support, and is practically useless, we don't support it. I don't think this will break
// anything, but it's worth noting.

#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "time.h"
#include "timelocal.h"
#include "applib/i18n.h"
#include "pbl/services/i18n/i18n.h"
#include "syscall/syscall.h"
#include "pbl/util/math.h"

#define INTFMT_PADSPACE (0)
#define INTFMT_PADZERO  (1)

// Used for wrong specifiers that want Monday as first day of the week.
static int prv_week_of_year(const struct tm *t, bool monday_is_first_day) {
  int wday = t->tm_wday; // Week day in range 0-6 (Sun-Sat)
  if (monday_is_first_day) {
    wday = (wday + 6) % 7;
  }
  // Boost the year day up so the division gets the right result.
  return (t->tm_yday + 7 - wday) / 7;
}

static int prv_full_year(int year) {
  return year + TM_YEAR_ORIGIN;
}

static int prv_iso8601_base_week(const struct tm *t) {
  // Not quite the same as prv_week_of_year
  // The ISO-8601 week count is defined as the number of weeks with Thursday in it.
  // Who knows why...
  return (t->tm_yday + 10 - ((t->tm_wday + 6) % 7)) / 7;
}

// Here be dragons
static int prv_year_week_count(int year, const struct tm *t,
                               int normal_compare, int leap_compare) {
/*
Find first wday of the year.

        CurWday
CurYday|  0  |  1  |  2  |  3  |  4  |  5  |  6  |
     0 | Sun | Mon | Tue | Wed | Thu | Fri | Sat |
     1 | Sat | Sun | Mon | Tue | Wed | Thu | Fri |
     2 | Fri | Sat | Sun | Mon | Tue | Wed | Thu |

wday - yday

6 - 0 = 6 = Sat
6 - 2 = 4 = Thu
0 - 2 = -2+7 = 5 = Fri

(wday - yday % 7)
*/
  // First wday of the year
  int wday = (((t->tm_wday - t->tm_yday) % 7) + 7) % 7;

  // Don't ask me, I didn't decide this.
  if (wday == normal_compare || (YEAR_IS_LEAP(year) && wday == leap_compare)) {
    return 53;
  } else {
    return 52;
  }
}

static int prv_iso8601_adjust(const struct tm *t, int year) {
  const int week = prv_iso8601_base_week(t);
  if (week == 0) {
    return -1;
  } else if (week > prv_year_week_count(year, t, 4, 3)) {
    // 53 weeks if the current year started on a Thursday,
    // orrrrr Wednesday and this year is a leap year.
    return 1;
  } else {
    return 0;
  }
}

static int prv_iso8601_year(const struct tm *t) {
  const int year = prv_full_year(t->tm_year);
  return year + prv_iso8601_adjust(t, year);
}

static int prv_iso8601_week(const struct tm *t) {
  const int year = prv_full_year(t->tm_year);
  switch (prv_iso8601_adjust(t, year)) {
    case -1:
      // 53 weeks if the current year started on a Friday,
      // orrrrrr Saturday and last year was a leap year.
      return prv_year_week_count(year - 1, t, 5, 6);
    case 1:
      return 1;
    default:
      return prv_iso8601_base_week(t);
  }
}

// Sorry I made a mess, it was in the name of size.
size_t localized_strftime(char * restrict dest_str, size_t maxsize, const char * restrict fmt,
                          const struct tm * restrict t, const char *locale) {
  const struct lc_time_T *time_locale = time_locale_get();
  size_t left = maxsize;
  const int year = prv_full_year(t->tm_year);
  const int hour_12h = (t->tm_hour % 12 == 0) ? 12 : (t->tm_hour % 12);
  // Only use i18n if we're in the kernel, or the app locale is the system locale.
  const bool use_i18n = !locale ||
                        (strncmp(locale, app_get_system_locale(), ISO_LOCALE_LENGTH) == 0);

  while (left) {
    // Copy up to the next '%'
    char *ptr = strchr(fmt, '%');
    if (ptr == NULL) {
      // Get the ending \0 of the string
      // Equivalent to ptr = fmt + strlen(fmt), but a bit faster/smaller
      ptr = strchr(fmt, '\0');
    }
    size_t length = (ptrdiff_t)(ptr - fmt);

    // Verify we have enough space for the copy
    if (left <= length) {
      goto _out_of_size;
    }
    memcpy(dest_str, fmt, length);
    dest_str += length;
    left -= length;

    fmt = ptr + 1;
    // End of string
    if (*ptr == '\0' || *fmt == '\0') {
      break;
    }

    char pad = '\0';
    size_t width = 0;

    // Process flags
    // These are the only ones our old impl cared about.
    if (*fmt == '0' || *fmt == '+') {
      pad = *fmt++;
    }

    // Process field width
    if (*fmt >= '1' && *fmt <= '9') {
      width = strtol(fmt, &ptr, 10);
      fmt = ptr;
    }

    // Process modifiers (SU)
    if (*fmt == 'E' || *fmt == 'O') {
      // We just drop them on the floor. Uh oh spaghetti-o
      // Not like the old implementation really did them either.
      fmt++;
    }

// Helper macros to make goto stuff look cleaner
#define FMT_STRCOPY(V) do { \
  i18nstr = NULL; \
  cpystr = V; \
  goto _fmt_strcopy; \
} while (0)
#define FMT_STRCOPY_I18N(V) do { \
  i18nstr = V; \
  goto _fmt_strcopy; \
} while (0)

#define FMT_INTCOPY(V, L, F) do { \
  cpyint_val = V; \
  cpyint_len = L; \
  cpyint_flag = F; \
  goto _fmt_intcopy; \
} while (0)

#define FMT_RECURSE(FMT) do { \
  i18nstr = NULL; \
  cpystr = FMT; \
  goto _fmt_recurse; \
} while (0)
#define FMT_RECURSE_I18N(FMT) do { \
  i18nstr = FMT; \
  goto _fmt_recurse; \
} while (0)

    // Terrible local state for goto hell
    const char *cpystr = NULL;
    const char *i18nstr;
    int cpyint_val;
    size_t cpyint_len;
    unsigned int cpyint_flag;
    // Process conversion specifiers
    switch (*fmt) {
      case 'a':
        FMT_STRCOPY_I18N(time_locale->wday[t->tm_wday % DAYS_PER_WEEK]);
_fmt_strcopy:
        // old strftime doesn't use 'width' for strings
        if (!use_i18n && i18nstr) {
          cpystr = i18nstr;
          i18nstr = NULL;
        }
        if (i18nstr) {
          length = sys_i18n_get_length(i18nstr);
        } else {
          length = strlen(cpystr);
        }
        if (left <= length) {
          goto _out_of_size;
        }
        if (i18nstr) {
          sys_i18n_get_with_buffer(i18nstr, dest_str, length + 1);
        } else {
          memcpy(dest_str, cpystr, length);
        }
        dest_str += length;
        left -= length;
        break;
      case 'A':
        FMT_STRCOPY_I18N(time_locale->weekday[t->tm_wday % DAYS_PER_WEEK]);
        break;
      case 'h': // SU
      case 'b':
        FMT_STRCOPY_I18N(time_locale->mon[t->tm_mon % MONTHS_PER_YEAR]);
        break;
      case 'B':
        FMT_STRCOPY_I18N(time_locale->month[t->tm_mon % MONTHS_PER_YEAR]);
        break;
      case 'c':
        FMT_RECURSE_I18N(time_locale->c_fmt);
_fmt_recurse:
        if (!use_i18n && i18nstr) {
          cpystr = i18nstr;
          i18nstr = NULL;
        }
        if (i18nstr) {
          cpystr = i18n_get(i18nstr, dest_str);
        }
        length = localized_strftime(dest_str, left, cpystr, t, locale);
        if (i18nstr) {
          i18n_free(i18nstr, dest_str);
        }
        if (length == 0) {
          goto _out_of_size;
        }
        dest_str += length;
        left -= length;
        break;
      case 'C': // SU
        FMT_INTCOPY(year / 100, 2, INTFMT_PADZERO);
_fmt_intcopy:
        {
          const char *intfmt;
          if (pad != '\0' || cpyint_flag == INTFMT_PADZERO) {
            intfmt = "%0*d";
          } else {
            intfmt = "%*d";
          }
          width = MAX(width, cpyint_len);
          length = snprintf(NULL, 0, intfmt, width, cpyint_val);
          if (left <= length) {
            goto _out_of_size;
          }
          sprintf(dest_str, intfmt, width, cpyint_val);
          dest_str += length;
          left -= length;
        }
        break;
      case 'd':
        FMT_INTCOPY(t->tm_mday, 2, INTFMT_PADZERO);
        break;
      case 'D': // SU
        FMT_RECURSE("%m/%d/%y");
        break;
      case 'e': // SU
        FMT_INTCOPY(t->tm_mday, 2, INTFMT_PADSPACE);
        break;
      case 'F': // C99
        FMT_RECURSE("%Y-%m-%d");
        break;
      case 'g': // TZ
        FMT_INTCOPY(prv_iso8601_year(t) % 100, 2, INTFMT_PADZERO);
        break;
      case 'G': // TZ
        FMT_INTCOPY(prv_iso8601_year(t), 4, INTFMT_PADZERO);
        break;
      case 'H':
        FMT_INTCOPY(t->tm_hour, 2, INTFMT_PADZERO);
        break;
      case 'I':
        FMT_INTCOPY(hour_12h, 2, INTFMT_PADZERO);
        break;
      case 'j':
        FMT_INTCOPY(t->tm_yday+1, 3, INTFMT_PADZERO);
        break;
      case 'k': // TZ
        FMT_INTCOPY(t->tm_hour, 2, INTFMT_PADSPACE);
        break;
      case 'l': // TZ
        FMT_INTCOPY(hour_12h, 2, INTFMT_PADSPACE);
        break;
      case 'm':
        FMT_INTCOPY(t->tm_mon + 1, 2, INTFMT_PADZERO);
        break;
      case 'M':
        FMT_INTCOPY(t->tm_min, 2, INTFMT_PADZERO);
        break;
      case 'r': // SU
        FMT_RECURSE_I18N(time_locale->r_fmt);
        break;
      case 'p':
        if (t->tm_hour < 12) {
          FMT_STRCOPY_I18N(time_locale->am_pm_upcase[0]);
        } else {
          FMT_STRCOPY_I18N(time_locale->am_pm_upcase[1]);
        }
        break;
      case 'P': // GNU
        if (t->tm_hour < 12) {
          FMT_STRCOPY_I18N(time_locale->am_pm_downcase[0]);
        } else {
          FMT_STRCOPY_I18N(time_locale->am_pm_downcase[1]);
        }
        break;
      case 'R': // SU
        FMT_RECURSE("%H:%M");
        break;
      case 'S':
        FMT_INTCOPY(t->tm_sec, 2, INTFMT_PADZERO);
        break;
      case 'T': // SU
        FMT_RECURSE("%H:%M:%S");
        break;
      case 'u': // SU
        if (t->tm_wday == 0) {
          FMT_INTCOPY(7, 1, INTFMT_PADZERO);
          break;
        }
        // fall-thru
      case 'w':
        FMT_INTCOPY(t->tm_wday, 1, INTFMT_PADZERO);
        break;
      case 'U':
        // Week starting on Sunday
        FMT_INTCOPY(prv_week_of_year(t, false), 2, INTFMT_PADZERO);
        break;
      case 'V': // SU
        FMT_INTCOPY(prv_iso8601_week(t), 2, INTFMT_PADZERO);
        break;
      case 'W':
        // Week starting on Monday, like savages
        FMT_INTCOPY(prv_week_of_year(t, true), 2, INTFMT_PADZERO);
        break;
      case 'x':
        FMT_RECURSE_I18N(time_locale->x_fmt);
        break;
      case 'X':
        FMT_RECURSE_I18N(time_locale->X_fmt);
        break;
      case 'y':
        FMT_INTCOPY(year % 100, 2, INTFMT_PADZERO);
        break;
      case 'Y':
        FMT_INTCOPY(year, 4, INTFMT_PADZERO);
        break;
      case 'z': {
        if (left < 5) {
          goto _out_of_size;
        }
        sprintf(dest_str, "%+.2d%02u", t->tm_gmtoff / 3600, (abs(t->tm_gmtoff) / 60) % 60);
        dest_str += 5;
        left -= 5;
        break;
      }
      case 'Z':
        FMT_STRCOPY(t->tm_zone);
        break;

      case 'n': // SU
        *dest_str++ = '\n';
        goto _fmt_chcopy;
      case 't': // SU
        *dest_str++ = '\t';
        goto _fmt_chcopy;
      // Copy stuff
      case '%':
        *dest_str++ = '%';
_fmt_chcopy:
        left--;
        break;
      case 's': // TZ // Old implementation didn't have it, skip it for code size.
      case '+': // TZ // Old implementation didn't have it, skip it for code size.
      default: // Old implementation just ignores invalid specifiers
        break;
    }
    fmt++;
  }
  if (left == 0) {
    goto _out_of_size;
  }
  // Finish him!!
  dest_str[0] = '\0';
  return maxsize - left;

_out_of_size:
  // Oops we're dead
  return 0;
}

size_t strftime(char * restrict s, size_t maxsize, const char* format, const struct tm* tim_p) {
  // Pass a NULL locale because firmware strftime is always localized
  return localized_strftime(s, maxsize, format, tim_p, NULL);
}
