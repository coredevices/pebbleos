/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

//! @param rand_out Storage for the 32-bit random number generated
//! @return True if a random number was successfully generated
bool rng_rand(uint32_t *rand_out);
