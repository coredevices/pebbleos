/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "pbl/util/assert.h"
#include "pbl/util/logging.h"
#include "pbl/util/rand32.h"

#include <stdio.h>
#include <stdlib.h>

// Below are some default implementations for system-specific functions required by libutil.
// These functions assume a working C standard library is linked into the program.
// For programs where this isn't the case (e.g. the Pebble FW),
// alternate implementations need to be provided.
// The functions are defined as WEAK so they may be easily overridden.

// If you are getting link errors due to printf not being defined, you probably
// need to provide your own implementation of the functions below.

WEAK void util_log(const char *filename, int line, const char *string) {
  printf("%s:%d %s\n", filename, line, string);
}

WEAK void util_dbgserial_str(const char *string) {
  printf("%s\n", string);
}

WEAK NORETURN util_assertion_failed(const char *filename, int line) {
  util_log(filename, line, "*** UTIL ASSERT FAILED");
  exit(EXIT_FAILURE);
}

WEAK uint32_t rand32(void) {
  return ((uint32_t)rand() << 1) + (uint32_t)rand();
}
