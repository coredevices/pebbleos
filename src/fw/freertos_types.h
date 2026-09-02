/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once


typedef void * QueueHandle_t;

typedef void * TaskHandle_t;

typedef void (*TaskFunction_t)( void * );

typedef struct xTASK_PARAMETERS TaskParameters_t;

typedef struct xMEMORY_REGION MemoryRegion_t;
