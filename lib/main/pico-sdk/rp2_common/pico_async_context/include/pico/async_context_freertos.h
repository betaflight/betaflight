/*
 * Copyright (c) 2022 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _PICO_ASYNC_CONTEXT_FREERTOS_H
#define _PICO_ASYNC_CONTEXT_FREERTOS_H

/** \file pico/async_context.h
 *  \defgroup async_context_freertos async_context_freertos
 *  \ingroup pico_async_context
 *  
 * \brief async_context_freertos provides an implementation of \ref async_context that handles asynchronous
 * work in a separate FreeRTOS task.
 */
#include "pico/async_context.h"

// FreeRTOS includes
#include "FreeRTOS.h"
#include "semphr.h"
#include "timers.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifndef ASYNC_CONTEXT_DEFAULT_FREERTOS_TASK_PRIORITY
#define ASYNC_CONTEXT_DEFAULT_FREERTOS_TASK_PRIORITY ( tskIDLE_PRIORITY + 4)
#endif

#ifndef ASYNC_CONTEXT_DEFAULT_FREERTOS_TASK_STACK_SIZE
#define ASYNC_CONTEXT_DEFAULT_FREERTOS_TASK_STACK_SIZE configMINIMAL_STACK_SIZE
#endif

typedef struct async_context_freertos async_context_freertos_t;

#if !defined(configNUMBER_OF_CORES) && defined(configNUM_CORES)
#if !portSUPPORT_SMP
#error configNUMBER_OF_CORES is the new name for configNUM_CORES
#else
// portSUPPORT_SMP was defined in old smp branch
#error configNUMBER_OF_CORES is the new name for configNUM_CORES, however it looks like you may need to define both as you are using an old SMP branch of FreeRTOS
#endif
#endif

/** 
 * \brief Configuration object for async_context_freertos instances.
 */
typedef struct async_context_freertos_config {
    /**
     * \brief Task priority for the async_context task
     */
    UBaseType_t task_priority;
    /**
     * \brief Stack size for the async_context task
     */
    configSTACK_DEPTH_TYPE task_stack_size;
    /**
     * \brief the core ID (see \ref portGET_CORE_ID()) to pin the task to.
     * This is only relevant in SMP mode.
     */
#if configUSE_CORE_AFFINITY && configNUMBER_OF_CORES > 1
    UBaseType_t task_core_id;
#endif
} async_context_freertos_config_t;

struct async_context_freertos {
    async_context_t core;
    SemaphoreHandle_t lock_mutex;
    SemaphoreHandle_t work_needed_sem;
    TimerHandle_t timer_handle;
    TaskHandle_t task_handle;
    uint8_t nesting;
    volatile bool task_should_exit;
};

/*!
 * \brief Initialize an async_context_freertos instance using the specified configuration
 * \ingroup async_context_freertos
 *
 * If this method succeeds (returns true), then the async_context is available for use
 * and can be de-initialized by calling async_context_deinit().
 * 
 * \param self a pointer to async_context_freertos structure to initialize
 * \param config the configuration object specifying characteristics for the async_context
 * \return true if initialization is successful, false otherwise
 */
bool async_context_freertos_init(async_context_freertos_t *self, async_context_freertos_config_t *config);

/*!
 * \brief Return a copy of the default configuration object used by \ref async_context_freertos_init_with_defaults() 
 * \ingroup async_context_freertos
 *
 * The caller can then modify just the settings it cares about, and call \ref async_context_freertos_init()
 * \return the default configuration object
 */
 static inline async_context_freertos_config_t async_context_freertos_default_config(void) {
    async_context_freertos_config_t config = {
            .task_priority = ASYNC_CONTEXT_DEFAULT_FREERTOS_TASK_PRIORITY,
            .task_stack_size = ASYNC_CONTEXT_DEFAULT_FREERTOS_TASK_STACK_SIZE,
#if configUSE_CORE_AFFINITY && configNUMBER_OF_CORES > 1
            .task_core_id = (UBaseType_t)-1, // none
#endif
    };
    return config;

}

/*!
 * \brief Initialize an async_context_freertos instance with default values
 * \ingroup async_context_freertos
 *
 * If this method succeeds (returns true), then the async_context is available for use
 * and can be de-initialized by calling async_context_deinit().
 * 
 * \param self a pointer to async_context_freertos structure to initialize
 * \return true if initialization is successful, false otherwise
 */
 static inline bool async_context_freertos_init_with_defaults(async_context_freertos_t *self) {
    async_context_freertos_config_t config = async_context_freertos_default_config();
    return async_context_freertos_init(self, &config);
}

#ifdef __cplusplus
}
#endif

#endif
