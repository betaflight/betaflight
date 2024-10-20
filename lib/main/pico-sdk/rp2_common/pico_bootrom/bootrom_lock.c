/*
 * Copyright (c) 2024 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "pico/bootrom/lock.h"
#include "pico/runtime_init.h"

#if PICO_BOOTROM_LOCKING_ENABLED
#if !PICO_RUNTIME_NO_INIT_BOOTROM_LOCKING_ENABLE
#include "hardware/sync.h"
void __weak runtime_init_bootrom_locking_enable(void) {
    bootrom_acquire_lock_blocking(BOOTROM_LOCK_ENABLE);
}
#endif

#if !PICO_RUNTIME_SKIP_INIT_BOOTROM_LOCKING_ENABLE
PICO_RUNTIME_INIT_FUNC_RUNTIME(runtime_init_bootrom_locking_enable, PICO_RUNTIME_INIT_BOOTROM_LOCKING_ENABLE);
#endif
#endif
