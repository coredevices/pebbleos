/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "kernel/pebble_tasks.h"

typedef struct MemorySegment MemorySegment;
typedef struct PebbleProcessMd PebbleProcessMd;

//! Load the process image specified by app_md into memory.
//!
//! The memory that the process image is loaded into is split from the
//! destination memory segment. The destination memory segment must
//! already be zeroed out.
//!
//! Only the process' text, data and bss are loaded and split from the
//! memory segment. It is the caller's responsibility to set up the
//! process stack and heap.
//!
//! @return pointer to process's entry point function, or NULL if the
//!     process loading failed.
void * process_loader_load(const PebbleProcessMd *app_md, PebbleTask task,
                           MemorySegment *destination);
