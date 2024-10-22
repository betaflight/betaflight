/*
 * Copyright (c) 2024 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#include "hardware/boot_lock.h"
#include "pico/runtime_init.h"

#if NUM_BOOT_LOCKS > 0
void boot_locks_reset(void) {
    GCC_Pragma("GCC unroll 1") // prevent GCC unrolling this loop which is 8 bytes per
    for (uint i = 0; i < NUM_BOOT_LOCKS; i++) {
        boot_unlock_unsafe(boot_lock_instance(i));
    }
}

boot_lock_t *boot_lock_init(uint lock_num) {
    assert(lock_num < NUM_BOOT_LOCKS);
    boot_lock_t *lock = boot_lock_instance(lock_num);
    boot_unlock_unsafe(lock);
    return lock;
}

#if !PICO_RUNTIME_NO_INIT_BOOT_LOCKS_RESET
#include "hardware/sync.h"
void __weak runtime_init_boot_locks_reset(void) {
    boot_locks_reset();
}
#endif

#if !PICO_RUNTIME_SKIP_INIT_BOOT_LOCKS_RESET
PICO_RUNTIME_INIT_FUNC_RUNTIME(runtime_init_boot_locks_reset, PICO_RUNTIME_INIT_BOOT_LOCKS_RESET);
#endif

#endif
