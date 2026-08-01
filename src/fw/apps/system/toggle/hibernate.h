/* SPDX-FileCopyrightText: 2025 The PebbleOS Contributors */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "process_management/app_manager.h"

#ifdef CONFIG_HIBERNATE
#define HIBERNATE_TOGGLE_UUID {0x8b, 0x9a, 0x7c, 0x3e, 0x45, 0xd1, 0x4f, 0x89, \
                                0xb2, 0x56, 0xee, 0xcc, 0xdd, 0xbb, 0xaa, 0x01}

const PebbleProcessMd *hibernate_toggle_get_app_info(void);
#endif
