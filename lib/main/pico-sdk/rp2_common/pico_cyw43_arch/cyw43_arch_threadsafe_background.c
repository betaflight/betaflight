/*
 * Copyright (c) 2022 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#if PICO_CYW43_ARCH_THREADSAFE_BACKGROUND

#include "pico/cyw43_arch.h"
#include "pico/cyw43_driver.h"
#include "pico/async_context_threadsafe_background.h"

#if CYW43_LWIP
#include "pico/lwip_nosys.h"
#endif

#if CYW43_ENABLE_BLUETOOTH
#include "pico/btstack_cyw43.h"
#endif

#if CYW43_LWIP && !NO_SYS
#error PICO_CYW43_ARCH_THREADSAFE_BACKGROUND requires lwIP NO_SYS=1
#endif
#if CYW43_LWIP && MEM_LIBC_MALLOC
// would attempt to use malloc from IRQ context
#error MEM_LIBC_MALLOC is incompatible with PICO_CYW43_ARCH_THREADSAFE_BACKGROUND
#endif

static async_context_threadsafe_background_t cyw43_async_context_threadsafe_background;

async_context_t *cyw43_arch_init_default_async_context(void) {
    async_context_threadsafe_background_config_t config = async_context_threadsafe_background_default_config();
    if (async_context_threadsafe_background_init(&cyw43_async_context_threadsafe_background, &config))
        return &cyw43_async_context_threadsafe_background.core;
    return NULL;
}

int cyw43_arch_init(void) {
    async_context_t *context = cyw43_arch_async_context();
    if (!context) {
        context = cyw43_arch_init_default_async_context();
        if (!context) return PICO_ERROR_GENERIC;
        cyw43_arch_set_async_context(context);
    }
    bool ok = cyw43_driver_init(context);
#if CYW43_LWIP
    ok &= lwip_nosys_init(context);
#endif
#if CYW43_ENABLE_BLUETOOTH
    ok &= btstack_cyw43_init(context);
#endif
    if (!ok) {
        cyw43_arch_deinit();
        return PICO_ERROR_GENERIC;
    } else {
        return 0;
    }
}

void cyw43_arch_deinit(void) {
    async_context_t *context = cyw43_arch_async_context();
#if CYW43_ENABLE_BLUETOOTH
    btstack_cyw43_deinit(context);
#endif
    // there is a bit of a circular dependency here between lwIP and cyw43_driver. We
    // shut down cyw43_driver first as it has IRQs calling back into lwIP. Also lwIP itself
    // does not actually get shut down.
    // todo add a "pause" method to async_context if we need to provide some atomicity (we
    //      don't want to take the lock as these methods may invoke execute_sync()
    cyw43_driver_deinit(context);
#if CYW43_LWIP
    lwip_nosys_deinit(context);
#endif
    // if it is our context, then we de-init it.
    if (context == &cyw43_async_context_threadsafe_background.core) {
        async_context_deinit(context);
        cyw43_arch_set_async_context(NULL);
    }
}

#endif
