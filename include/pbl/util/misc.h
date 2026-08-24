/* SPDX-FileCopyrightText: 2025 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include <stdint.h>

#define container_of(ptr, type, member) ((type *)((char *)(ptr) - (size_t)&(((type *)0)->member)))

//! Caller's return address, for LR-tracking APIs. Unlike a named register
//! variable bound to "lr", this is well-defined under both gcc and clang.
#define PBL_RETURN_ADDRESS() ((uintptr_t)__builtin_return_address(0))
