/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include <bluetooth/dis.h>

//! @param info Device info to serve. Must stay valid for as long as the stack runs.
void dis_service_init(const DisInfo *info);
