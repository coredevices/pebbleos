/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "util/time/time.h"

// We divide the raw light sensor reading by this factor before storing it into AlgDlsMinuteData
#define ALG_RAW_LIGHT_SENSOR_DIVIDE_BY 16

// Nap constraints, also used by unit tests
// A sleep session in this range is always considered "primary" (not nap) sleep
// ... if it ends after this minute in the evening
#define ALG_PRIMARY_EVENING_MINUTE (21 * MINUTES_PER_HOUR)   // 9pm
// ... or starts before this minute in the morning
#define ALG_PRIMARY_MORNING_MINUTE  (12 * MINUTES_PER_HOUR)   // 12pm

// A sleep session outside of the primary range is considered a nap if it is less than
// this duration, otherwise it is considered a primary sleep session
#define ALG_MAX_NAP_MINUTES (3 * MINUTES_PER_HOUR)

// Max number of hours of past data we process to figure out sleep for "today". If a sleep
// cycle *ends* after midnight today, then we still count it as today's sleep. That means the
// start of the sleep cycle could have started more than 24 hours ago.
#define ALG_SLEEP_HISTORY_HOURS_FOR_TODAY   36
