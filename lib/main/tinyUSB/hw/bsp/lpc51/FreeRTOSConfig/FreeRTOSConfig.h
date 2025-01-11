/*
 * FreeRTOS Kernel V10.0.0
 * Copyright (C) 2017 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software. If you wish to use our Amazon
 * FreeRTOS name, please do so in a fair use way that does not cause confusion.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * http://www.FreeRTOS.org
 * http://aws.amazon.com/freertos
 *
 * 1 tab == 4 spaces!
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

// skip if included from IAR assembler
#ifndef __IASMARM__
  #include "fsl_device_registers.h"
#endif

/* Cortex M23/M33 port configuration. */
#define configENABLE_MPU                        0
#if defined(__ARM_FP) && __ARM_FP >= 4
  #define configENABLE_FPU                      1
#else
  #define configENABLE_FPU                      0
#endif
#define configENABLE_TRUSTZONE                  0
#define configMINIMAL_SECURE_STACK_SIZE         (1024)

#define configUSE_PREEMPTION                    1
#define configUSE_PORT_OPTIMISED_TASK_SELECTION 0
#define configCPU_CLOCK_HZ                      SystemCoreClock
#define configTICK_RATE_HZ                      ( 1000 )
#define configMAX_PRIORITIES                    ( 5 )
#define configMINIMAL_STACK_SIZE                ( 128 )
#define configTOTAL_HEAP_SIZE                   ( configSUPPORT_DYNAMIC_ALLOCATION*4*1024 )
#define configMAX_TASK_NAME_LEN                 16
#define configUSE_16_BIT_TICKS                  0
#define configIDLE_SHOULD_YIELD                 1
#define configUSE_MUTEXES                       1
#define configUSE_RECURSIVE_MUTEXES             1
#define configUSE_COUNTING_SEMAPHORES           1
#define configQUEUE_REGISTRY_SIZE               2
#define configUSE_QUEUE_SETS                    0
#define configUSE_TIME_SLICING                  0
#define configUSE_NEWLIB_REENTRANT              0
#define configENABLE_BACKWARD_COMPATIBILITY     1
#define configSTACK_ALLOCATION_FROM_SEPARATE_HEAP   0

#define configSUPPORT_STATIC_ALLOCATION         1
#define configSUPPORT_DYNAMIC_ALLOCATION        0

/* Hook function related definitions. */
#define configUSE_IDLE_HOOK                    0
#define configUSE_TICK_HOOK                    0
#define configUSE_MALLOC_FAILED_HOOK           0 // cause nested extern warning
#define configCHECK_FOR_STACK_OVERFLOW         2
#define configCHECK_HANDLER_INSTALLATION       0

/* Run time and task stats gathering related definitions. */
#define configGENERATE_RUN_TIME_STATS          0
#define configRECORD_STACK_HIGH_ADDRESS        1
#define configUSE_TRACE_FACILITY               1 // legacy trace
#define configUSE_STATS_FORMATTING_FUNCTIONS   0

/* Co-routine definitions. */
#define configUSE_CO_ROUTINES                  0
#define configMAX_CO_ROUTINE_PRIORITIES        2

/* Software timer related definitions. */
#define configUSE_TIMERS                       1
#define configTIMER_TASK_PRIORITY              (configMAX_PRIORITIES-2)
#define configTIMER_QUEUE_LENGTH               32
#define configTIMER_TASK_STACK_DEPTH           configMINIMAL_STACK_SIZE

/* Optional functions - most linkers will remove unused functions anyway. */
#define INCLUDE_vTaskPrioritySet               0
#define INCLUDE_uxTaskPriorityGet              0
#define INCLUDE_vTaskDelete                    0
#define INCLUDE_vTaskSuspend                   1 // required for queue, semaphore, mutex to be blocked indefinitely with portMAX_DELAY
#define INCLUDE_xResumeFromISR                 0
#define INCLUDE_vTaskDelayUntil                1
#define INCLUDE_vTaskDelay                     1
#define INCLUDE_xTaskGetSchedulerState         0
#define INCLUDE_xTaskGetCurrentTaskHandle      1
#define INCLUDE_uxTaskGetStackHighWaterMark    0
#define INCLUDE_xTaskGetIdleTaskHandle         0
#define INCLUDE_xTimerGetTimerDaemonTaskHandle 0
#define INCLUDE_pcTaskGetTaskName              0
#define INCLUDE_eTaskGetState                  0
#define INCLUDE_xEventGroupSetBitFromISR       0
#define INCLUDE_xTimerPendFunctionCall         0

/* Define to trap errors during development. */
// Halt CPU (breakpoint) when hitting error, only apply for Cortex M3, M4, M7
#if defined(__ARM_ARCH_7M__) || defined (__ARM_ARCH_7EM__) || defined(__ARM_ARCH_8M_MAIN__) || defined(__ARM_ARCH_8_1M_MAIN__) || \
    defined(__ARM7M__) || defined (__ARM7EM__) || defined(__ARM8M_MAINLINE__) || defined(__ARM8EM_MAINLINE__)
  #define configASSERT(_exp) \
    do {\
      if ( !(_exp) ) { \
        volatile uint32_t* ARM_CM_DHCSR =  ((volatile uint32_t*) 0xE000EDF0UL); /* Cortex M CoreDebug->DHCSR */ \
        if ( (*ARM_CM_DHCSR) & 1UL ) {  /* Only halt mcu if debugger is attached */ \
          taskDISABLE_INTERRUPTS(); \
           __asm("BKPT #0\n"); \
        }\
      }\
    } while(0)
#endif

/* FreeRTOS hooks to NVIC vectors */
#define xPortPendSVHandler    PendSV_Handler
#define xPortSysTickHandler   SysTick_Handler
#define vPortSVCHandler       SVC_Handler

//--------------------------------------------------------------------+
// Interrupt nesting behavior configuration.
//--------------------------------------------------------------------+

// For Cortex-M specific: __NVIC_PRIO_BITS is defined in mcu header
#define configPRIO_BITS       2

/* The lowest interrupt priority that can be used in a call to a "set priority" function. */
#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY			  ((1<<configPRIO_BITS) - 1)

/* The highest interrupt priority that can be used by any interrupt service
routine that makes calls to interrupt safe FreeRTOS API functions.  DO NOT CALL
INTERRUPT SAFE FREERTOS API FUNCTIONS FROM ANY INTERRUPT THAT HAS A HIGHER
PRIORITY THAN THIS! (higher priorities are lower numeric values. */
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY	2

/* Interrupt priorities used by the kernel port layer itself.  These are generic
to all Cortex-M ports, and do not rely on any particular library functions. */
#define configKERNEL_INTERRUPT_PRIORITY 		          ( configLIBRARY_LOWEST_INTERRUPT_PRIORITY << (8 - configPRIO_BITS) )

/* !!!! configMAX_SYSCALL_INTERRUPT_PRIORITY must not be set to zero !!!!
See http://www.FreeRTOS.org/RTOS-Cortex-M3-M4.html. */
#define configMAX_SYSCALL_INTERRUPT_PRIORITY 	        ( configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY << (8 - configPRIO_BITS) )

#endif /* __FREERTOS_CONFIG__H */
