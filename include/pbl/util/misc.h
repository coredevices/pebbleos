/* SPDX-FileCopyrightText: 2025 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#define container_of(ptr, type, member) ((type *)((char *)(ptr) - (size_t)&(((type *)0)->member)))
