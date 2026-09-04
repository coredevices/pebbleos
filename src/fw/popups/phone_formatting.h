/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include <stddef.h>

//! Turns every word after the first one into an initial.
//! e.g. Katharine Claire Berry -> Katharine C. B.
//! @param full_name String containing the full name
//! @param destination buffer to copy the abbreviated name into.
//! @param length Size of the destination buffer.
void phone_format_caller_name(const char *full_name, char *destination, size_t length);

//! Forces 2 line formatting on international phone numbers as well as
//! most long distance phone numbers (where required by format and length)
//! e.g. +55 408-555-1212 becomes
//! +55 408
//! 555-1212
//! @param phone_number String containing original phone number
//! @param formatted_phone_number buffer to copy the formatted phone number into.
//! @param length Size of the destination buffer.
void phone_format_phone_number(const char *phone_number, char *formatted_phone_number,
                               size_t length);
