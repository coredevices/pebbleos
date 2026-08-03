/*
 * SPDX-FileCopyrightText: 2026 Core Devices LLC
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

/* Minimal Mbed TLS configuration for PebbleOS. Only the primitives required
 * by the NimBLE host (SM pairing and GATT database hash) are enabled:
 * AES-128 (ECB/CMAC) and P-256 ECDH.
 */

#include <stddef.h>

/* Allocate from the kernel heap. */
void *kernel_calloc(size_t count, size_t size);
void kernel_free(void *ptr);

#define MBEDTLS_PLATFORM_C
#define MBEDTLS_PLATFORM_MEMORY
#define MBEDTLS_PLATFORM_CALLOC_MACRO kernel_calloc
#define MBEDTLS_PLATFORM_FREE_MACRO kernel_free

#define MBEDTLS_AES_C
#define MBEDTLS_AES_ROM_TABLES
#define MBEDTLS_CIPHER_C
#define MBEDTLS_CMAC_C

#define MBEDTLS_BIGNUM_C
#define MBEDTLS_ECP_C
#define MBEDTLS_ECP_DP_SECP256R1_ENABLED
#define MBEDTLS_ECDH_C

/* Favor small flash/RAM footprint over ECC speed, but keep the NIST fast
 * reduction: the generic division-based path makes one P-256 point multiply
 * slow enough to trip the task watchdog. */
#define MBEDTLS_ECP_NIST_OPTIM
#define MBEDTLS_ECP_WINDOW_SIZE 2
#define MBEDTLS_ECP_FIXED_POINT_OPTIM 0

#define MBEDTLS_NO_PLATFORM_ENTROPY
