/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "process_management/app_manager.h"

#define QUICK_LAUNCH_NOTHING_UUID {0xde, 0x6d, 0xa1, 0x7f, 0x1a, 0x10, 0x47, 0x25, \
                                   0xad, 0xbb, 0x1e, 0xfc, 0x22, 0xe4, 0x3f, 0x04}

const PebbleProcessMd *quick_launch_nothing_get_app_info(void);
