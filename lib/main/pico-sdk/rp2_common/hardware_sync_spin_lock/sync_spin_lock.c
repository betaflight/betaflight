/*
 * Copyright (c) 2024 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#include "hardware/sync/spin_lock.h"

void spin_locks_reset(void) {
    for (uint i = 0; i < NUM_SPIN_LOCKS; i++) {
        spin_unlock_unsafe(spin_lock_instance(i));
    }
}

spin_lock_t *spin_lock_init(uint lock_num) {
    assert(lock_num < NUM_SPIN_LOCKS);
    spin_lock_t *lock = spin_lock_instance(lock_num);
    spin_unlock_unsafe(lock);
    return lock;
}

#if PICO_USE_SW_SPIN_LOCKS
spin_lock_t _sw_spin_locks[NUM_SPIN_LOCKS];

#if __ARM_ARCH_8M_MAIN__ && !PICO_SW_SPIN_LOCKS_NO_EXTEXCLALL
#include "pico/runtime_init.h"
#include "hardware/structs/m33.h"

static void spinlock_set_extexclall(void) {
    // Force use of global exclusive monitor for all exclusive load/stores:
    // makes multicore exclusives work without adding MPU regions. For
    // something more exotic, like having multicore exclusives in internal
    // SRAM and also single-core exclusives in external PSRAM (not covered by
    // the global monitor on RP2350) you must clear this and add your own
    // Shareable regions.
    //
    // Setting PICO_SW_SPIN_LOCKS_NO_EXTEXCLALL == 1 will disable this code
    m33_hw->actlr |= M33_ACTLR_EXTEXCLALL_BITS;
}

// PICO_RUNTIME_INIT_SPIN_LOCKS_RESET is fine as resetting them does not require EXTEXCLALL
PICO_RUNTIME_INIT_FUNC_PER_CORE(spinlock_set_extexclall, PICO_RUNTIME_INIT_SPIN_LOCKS_RESET);
#endif
#endif

