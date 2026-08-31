/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include <stdint.h>

#include "pbl/util/attributes.h"
#include "pebbleos/core_dump_structs.h"

// Size of RAM
// TODO: Do we have an equate for the total size of RAM somewhere else?
#if defined(CONFIG_BOARD_QEMU_EMERY) || defined(CONFIG_BOARD_QEMU_GABBRO)
#define COREDUMP_RAM_SIZE (276 * 1024)
#elif defined(CONFIG_BOARD_QEMU_FLINT)
#define COREDUMP_RAM_SIZE (256 * 1024)
#elif defined(CONFIG_BOARD_OBELIX) || defined(CONFIG_BOARD_GETAFIX)
// Main RAM on SF32LB52 extends to _heap_end (0x20045000).
// Thread stacks can be allocated above the 256KB mark.
#define COREDUMP_RAM_SIZE (276 * 1024)
#elif defined(CONFIG_BOARD_ASTERIX)
#define COREDUMP_RAM_SIZE (256 * 1024)
#endif

// LCPU (BLE coprocessor) RAM, in the LPSYS domain outside main RAM.
#if defined(CONFIG_SOC_SF32LB52)
#define COREDUMP_LCPU_RAM_START (0x20400000)
#define COREDUMP_LCPU_RAM_SIZE (64 * 1024)
#endif

// Max number of core dump images we can fit in our allocated space
#define CORE_DUMP_FLASH_START   FLASH_REGION_CD_BEGIN
#define CORE_DUMP_FLASH_END     FLASH_REGION_CD_END
#define CORE_DUMP_FLASH_SIZE    (CORE_DUMP_FLASH_END - CORE_DUMP_FLASH_START)
#if defined(CONFIG_SOC_NRF52)
#define CORE_DUMP_MAX_IMAGES 2
#elif defined(CONFIG_SOC_SF32LB52)
#define CORE_DUMP_MAX_IMAGES 1
#elif defined(CONFIG_QEMU)
#define CORE_DUMP_MAX_IMAGES 1
#else
#error "Unsupported micro family"
#endif

// Max size of a core dump image. The first image is found at CORE_DUMP_FLASH_START +
// SUBSECTOR_SIZE_BYTES.
#define CORE_DUMP_MAX_SIZE (((CORE_DUMP_FLASH_SIZE - SUBSECTOR_SIZE_BYTES) \
                             / CORE_DUMP_MAX_IMAGES) & SUBSECTOR_ADDR_MASK)

// Returned from prv_flash_start_address() if no valid region found
#define CORE_DUMP_FLASH_INVALID_ADDR 0xFFFFFFFF

// We don't overwrite an unread core-dump if it's less than CORE_DUMP_MIN_AGE seconds old and hasn't been
//  fetched from the watch yet.
#define CORE_DUMP_MIN_AGE_SECONDS     (60 * 60 * 24 * 1)       // 1 day

// --------------------------------------------------------------------------------------------
// Core dump flash storage structures. The first thing at CORE_DUMP_FLASH_START is a CoreDumpFlashHeader.
// SUBSECTOR_SIZE_BYTES after that is the CoreDumpFlashRegionHeader for the first region.
// Every CORE_DUMP_MAX_SIZE after the first region header is another CoreDumpFlashRegionHeader, up to a
// max of CORE_DUMP_MAX_IMAGES.
// Each of the bits in the 'unformatted' field start out at 1, they get cleared as we use up to
// CORE_DUMP_MAX_IMAGES regions. When all CORE_DUMP_MAX_IMAGES have been used at least once, we rotate and
// set the active one to have the highest last_used value.

// This comes first in flash, at CORE_DUMP_FLASH_START. It is NOT returned as part of the core dump
// binary image
#define CORE_DUMP_FLASH_HDR_MAGIC        0x464C5300
#define CORE_DUMP_ALL_UNFORMATTED        ((uint32_t)(~0))
typedef struct {
  uint32_t    magic;                // Set to CORE_DUMP_FLASH_HDR_MAGIC
  uint32_t    unformatted;          // set of 1 bit flags, bit n set means region n is still unformatted
} CoreDumpFlashHeader;

// This comes first in the front of each possibe flash region. It is NOT returned as part of the core dump
// image.
typedef struct {
  uint32_t    magic;                // set to CORE_DUMP_FLASH_HDR_MAGIC
  uint32_t    last_used;            // The region with the highest last_used count was the most recently used.
  // This value is always >= 1
  uint8_t     unread;               // non-zero if this core dump has not been read out yet
} CoreDumpFlashRegionHeader;

// The first item in a core dump image is a CoreDumpImageHeader. That is followed by one or more
// CoreDumpChunkHeader's, terminated by one with a key of CORE_DUMP_CHUNK_KEY_TERMINATOR
#define CORE_DUMP_MAGIC                   0xF00DCAFE
#define CORE_DUMP_VERSION                 1                   // Current version
typedef struct PACKED {
  uint32_t    magic;                // Set to CORE_DUMP_MAGIC

  uint32_t    core_number:8;        // See include/pebbleos/core_id.h
  uint32_t    version:24;           // Set to CORE_DUMP_VERSION

  uint32_t    time_stamp;           // rtc_get_time() when core dump was created
  uint8_t     serial_number[16];    // null terminated watch serial number string
  uint8_t     build_id[64];         // null terminated build ID of firmware string
} CoreDumpImageHeader;

// Chunk header for each chunk within the core dump
#define CORE_DUMP_CHUNK_KEY_TERMINATOR    0xFFFFFFFF
#define CORE_DUMP_CHUNK_KEY_RAM           1  // Deprecated
#define CORE_DUMP_CHUNK_KEY_THREAD        2
#define CORE_DUMP_CHUNK_KEY_EXTRA_REG     3
#define CORE_DUMP_CHUNK_KEY_MEMORY        4
typedef struct PACKED {
  uint32_t    key;          // CORE_DUMP_CHUNK_KEY_.*
  uint32_t    size;
  // uint8_t  data[size];
} CoreDumpChunkHeader;

// Header for dumped segments of memory, whether from RAM or peripheral space.
typedef struct PACKED {
  uint32_t start;   // start address of the chunk of dumped memory
  // uint8_t data[size - sizeof(CoreDumpMemoryHeader)];
} CoreDumpMemoryHeader;

void coredump_assert(int line);
#define CD_ASSERTN(expr) \
  do { \
    if (!(expr)) { \
      coredump_assert(__LINE__); \
    } \
  } while (0)
