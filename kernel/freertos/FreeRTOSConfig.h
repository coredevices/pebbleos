/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

// @nolint

/*
    FreeRTOS V7.1.0 - Copyright (C) 2011 Real Time Engineers Ltd.


    ***************************************************************************
     *                                                                       *
     *    FreeRTOS tutorial books are available in pdf and paperback.        *
     *    Complete, revised, and edited pdf reference manuals are also       *
     *    available.                                                         *
     *                                                                       *
     *    Purchasing FreeRTOS documentation will not only help you, by       *
     *    ensuring you get running as quickly as possible and with an        *
     *    in-depth knowledge of how to use FreeRTOS, it will also help       *
     *    the FreeRTOS project to continue with its mission of providing     *
     *    professional grade, cross platform, de facto standard solutions    *
     *    for microcontrollers - completely free of charge!                  *
     *                                                                       *
     *    >>> See http://www.FreeRTOS.org/Documentation for details. <<<     *
     *                                                                       *
     *    Thank you for using FreeRTOS, and thank you for your support!      *
     *                                                                       *
    ***************************************************************************


    This file is part of the FreeRTOS distribution.

    FreeRTOS is free software; you can redistribute it and/or modify it under
    the terms of the GNU General Public License (version 2) as published by the
    Free Software Foundation AND MODIFIED BY the FreeRTOS exception.
    >>>NOTE<<< The modification to the GPL is included to allow you to
    distribute a combined work that includes FreeRTOS without being obliged to
    provide the source code for proprietary components outside of the FreeRTOS
    kernel.  FreeRTOS is distributed in the hope that it will be useful, but
    WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
    or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
    more details. You should have received a copy of the GNU General Public
    License and the FreeRTOS license exception along with FreeRTOS; if not it
    can be viewed here: http://www.freertos.org/a00114.html and also obtained
    by writing to Richard Barry, contact details for whom are available on the
    FreeRTOS WEB site.

    1 tab == 4 spaces!

    http://www.FreeRTOS.org - Documentation, latest information, license and
    contact details.

    http://www.SafeRTOS.com - A version that is certified for use in safety
    critical systems.

    http://www.OpenRTOS.com - Commercial support, development, porting,
    licensing and training services.
*/

#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

/*-----------------------------------------------------------
 * Application specific definitions.
 *
 * These definitions should be adjusted for your particular hardware and
 * application requirements.
 *
 * THESE PARAMETERS ARE DESCRIBED WITHIN THE 'CONFIGURATION' SECTION OF THE
 * FreeRTOS API DOCUMENTATION AVAILABLE ON THE FreeRTOS.org WEB SITE.
 *
 * See http://www.freertos.org/a00110.html.
 *----------------------------------------------------------*/


#include <cmsis_core.h>
// Per-task newlib reentrancy needs newlib's struct _reent. picolibc uses
// TLS instead (and has no reent.h), the Pebble libc has no per-thread
// state, and the host unit-test build has no arm-newlib, so only pull it
// in for a newlib firmware.
#if defined(CONFIG_LIBC_NEWLIB) || defined(CONFIG_LIBC_NEWLIB_NANO)
#include <reent.h>
#endif

extern uint32_t SystemCoreClock;

#define configUSE_PREEMPTION     1
#define configUSE_IDLE_HOOK      0
#define configUSE_TICK_HOOK      0
#define configCPU_CLOCK_HZ       ((unsigned long)SystemCoreClock)
#define configTICK_RATE_HZ       ((TickType_t)CONFIG_KERNEL_TICK_HZ)
#define configMAX_PRIORITIES     ((unsigned portBASE_TYPE)CONFIG_KERNEL_NUM_PRIORITIES)
#define configMINIMAL_STACK_SIZE ((unsigned short)196)
#define configTOTAL_HEAP_SIZE    ((size_t)(30 * 1024))
#define configMAX_TASK_NAME_LEN  (16)
#define configUSE_TRACE_FACILITY 1
#define configUSE_16_BIT_TICKS   0
#define configIDLE_SHOULD_YIELD  1
#define configUSE_TICKLESS_IDLE  2   // Use STOP mode
#define configUSE_STATS_FORMATTING_FUNCTIONS 1

/* SF32LB52: SiFli's HAL programs MPU regions 0..4 and Pebble's static MPU
 * setup uses 5..7, so the per-thread regions live at 8..11. QEMU CM33 has
 * no vendor-reserved regions: Pebble's static regions take 0..3 and the
 * per-thread regions 4..7. */
#if defined(CONFIG_SOC_SF32LB52)
#define configFIRST_MPU_REGION 8
#define configLAST_MPU_REGION 11
#elif defined(CONFIG_QEMU) && defined(CONFIG_CORTEX_M33)
#define configFIRST_MPU_REGION 4
#define configLAST_MPU_REGION 7
#endif

#define configUSE_TIMERS 0
#define configTIMER_TASK_PRIORITY (configMAX_PRIORITIES - 1)
#define configTIMER_QUEUE_LENGTH 32
#define configTIMER_TASK_STACK_DEPTH (256 + 64)

/* Co-routine definitions. */
#define configUSE_CO_ROUTINES 		0
#define configMAX_CO_ROUTINE_PRIORITIES ( 2 )

#define configUSE_MUTEXES				0
#define configUSE_COUNTING_SEMAPHORES 	1
#define configUSE_ALTERNATIVE_API 		0
#define configCHECK_FOR_STACK_OVERFLOW	2
#define configUSE_RECURSIVE_MUTEXES		0
#define configQUEUE_REGISTRY_SIZE		0
#define configGENERATE_RUN_TIME_STATS	1
#if defined(CONFIG_LIBC_NEWLIB) || defined(CONFIG_LIBC_NEWLIB_NANO)
#define configUSE_NEWLIB_REENTRANT      1
#else
#define configUSE_NEWLIB_REENTRANT      0
#endif
#define configUSE_QUEUE_SETS            1
#define configUSE_LIGHT_MUTEXES         1
#define configUSE_RECURSIVE_LIGHT_MUTEXES 1
#define configUSE_TASK_NOTIFICATIONS    0
#define configENABLE_BACKWARD_COMPATIBILITY 0

/* Thread local storage for syscall information */
#define configNUM_THREAD_LOCAL_STORAGE_POINTERS 1

/* Set the following definitions to 1 to include the API function, or zero
to exclude the API function. */

#define INCLUDE_vTaskPrioritySet               1
#define INCLUDE_uxTaskPriorityGet              1
#define INCLUDE_vTaskDelete                    1
#define INCLUDE_vTaskCleanUpResources          0
#define INCLUDE_xTaskGetCurrentTaskHandle      1
#define INCLUDE_vTaskSuspend                   1
#define INCLUDE_vTaskDelayUntil                1
#define INCLUDE_vTaskDelay                     1
#define INCLUDE_eTaskGetState                  1
#define INCLUDE_xQueueGetMutexHolder           1
#define INCLUDE_pcTaskGetTaskName              1
#define INCLUDE_xTimerGetTimerDaemonTaskHandle 1
#define INCLUDE_xTaskGetSchedulerState         1
#define INCLUDE_uxTaskGetStackHighWaterMark    1
#define INCLUDE_xTaskGetIdleTaskHandle         1

/* This is the raw value as per the Cortex-M3 NVIC.  Values can be 255
(lowest) to 0 (1?) (highest). */
#define configKERNEL_INTERRUPT_PRIORITY      CONFIG_KERNEL_IRQ_PRIO_KERNEL
#define configMAX_SYSCALL_INTERRUPT_PRIORITY CONFIG_KERNEL_IRQ_PRIO_MAX_SYSCALL


/* This is the value being used as per the ST library which permits 16
priority values, 0 to 15.  This must correspond to the
configKERNEL_INTERRUPT_PRIORITY setting.  Here 15 corresponds to the lowest
NVIC value of 255. */
#define configLIBRARY_KERNEL_INTERRUPT_PRIORITY 15


#ifdef CONFIG_SOC_SF32LB52
#define xPortSysTickHandler RTOS_SysTick_Handler
#else
#define xPortSysTickHandler SysTick_Handler
#endif
#define xPortPendSVHandler  PendSV_Handler
#define vPortSVCHandler     SVC_Handler


/* Set this to add error checking code to the entry of each FreeRTOS function to ensure that the
   caller is in the correct state to make the call (ISR priority, critical section, etc.) */
#ifdef CONFIG_RELEASE
#define configCHECK_CALL_SAFETY 0
#else
#define configCHECK_CALL_SAFETY 1
#endif

/*-----------------------------------------------------------
 * Macros required to setup the timer for the run time stats.
 *-----------------------------------------------------------*/
/* Use the FreeRTOS tick counter as the run-time stats clock. It runs at
 * 1024 Hz and wraps every ~48.5 days on a 32-bit platform, so the task
 * accumulators never wrap within a heartbeat period. Sleep time is
 * accounted for by tickless idle, so the idle task's counter reflects
 * real sleep + idle-loop time. */
#define portCONFIGURE_TIMER_FOR_RUN_TIME_STATS()
#define portGET_RUN_TIME_COUNTER_VALUE() xTaskGetTickCount()

#include "pbl/os/assert.h"
#define configASSERT( x ) OS_ASSERT(x)

#if configCHECK_CALL_SAFETY
  #include "pbl/mcu/interrupts.h"
  bool vPortInCritical( void );
  #define configASSERT_SAFE_TO_CALL_FREERTOS_API()            \
    configASSERT(!vPortInCritical() && !mcu_state_is_isr())

  #define configASSERT_SAFE_TO_CALL_WAIT_FREERTOS_API(xTicksToWait)            \
    configASSERT((!vPortInCritical() || (vPortInCritical() && xTicksToWait == 0U)) && !mcu_state_is_isr())

  #define configASSERT_SAFE_TO_CALL_FREERTOS_FROMISR_API()            \
    configASSERT(!mcu_state_is_isr() || mcu_state_get_isr_priority() >= (configMAX_SYSCALL_INTERRUPT_PRIORITY >> (8U - __NVIC_PRIO_BITS)))

#else
  #define configASSERT_SAFE_TO_CALL_FREERTOS_API()           
  #define configASSERT_SAFE_TO_CALL_WAIT_FREERTOS_API(xTicksToWait)
  #define configASSERT_SAFE_TO_CALL_FREERTOS_FROMISR_API()

#endif


#endif /* FREERTOS_CONFIG_H */
