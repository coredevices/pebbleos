/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "applib_resource.h"
#include "resource/resource.h"

//! Checks if a passed pointer refers to builtin or memory-mapped data and manages reference
//! counters as needed.
//! @note You might want to use \ref applib_resource_mmap_or_load() instead
//! @return true, if the passed pointer expresses memory-mapped data ans was successfully tracked
bool applib_resource_track_mmapped(const void *bytes);

//! True, if the passed pointer refers to builtin or memory-mapped data
bool applib_resource_is_mmapped(const void *bytes);

//! Checks if a passed pointer refers to builtin or memory-mapped data and manages reference
//! counters as needed.
//! @note You might want to use \ref applib_resource_munmap_or_free() instead
//! @return true, if the passed pointer expresses memory-mapped data ans was successfully
//!     untracked
bool applib_resource_munmap(const void *bytes);

//! Manages the reference counters for memory-mapped resources.
//! Should not be called by anyone but the process manager.
//! @return true, if any remaining resources were untracked
bool applib_resource_munmap_all();

//! Tries to load a resource as memory-mapped data. If this isn't supported on the system
//! or for a given resource if will try to allocate data and load it into RAM instead.
//! Have a look at \ref resource_load_byte_range_system for the discussion of arguments
//! @param app_num The app the resource belongs to
//! @param resource_id The ID of the resource
//! @param offset The offset in bytes into the resource
//! @param length The number of bytes to load
//! @param used_aligned True, if you want this function to allocate 7 extra bytes if it cannot mmap
//! @return NULL, if the resource couldn't be memory-mapped or allocated
void *applib_resource_mmap_or_load(ResAppNum app_num, uint32_t resource_id,
                                   size_t offset, size_t length, bool used_aligned);

//! Updates reference counters if bytes is memory-mapped,
//! alternatively it deallocates the data bytes points to.
void applib_resource_munmap_or_free(void *bytes);
