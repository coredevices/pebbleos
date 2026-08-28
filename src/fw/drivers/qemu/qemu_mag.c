/* SPDX-FileCopyrightText: 2026 Philippe Loctaux */
/* SPDX-License-Identifier: Apache-2.0 */

#include <stdbool.h>

//! QEMU stub for the magnetometer rotated API.
//! Real hardware implements this in the MMC5603NJ driver.

void mag_set_rotated(bool rotated) {
  (void)rotated;
}
