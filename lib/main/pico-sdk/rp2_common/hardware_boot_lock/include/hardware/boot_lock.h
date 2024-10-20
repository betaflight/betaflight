/*
 * Copyright (c) 2024 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _HARDWARE_BOOT_LOCK_H
#define _HARDWARE_BOOT_LOCK_H

#include "pico.h"

// PICO_CONFIG: PARAM_ASSERTIONS_ENABLED_HARDWARE_BOOT_LOCK, Enable/disable assertions in the hardware_boot_lock module, type=bool, default=0, group=hardware_boot_lock
#ifndef PARAM_ASSERTIONS_ENABLED_HARDWARE_BOOT_LOCK
#define PARAM_ASSERTIONS_ENABLED_HARDWARE_BOOT_LOCK 0
#endif

#if NUM_BOOT_LOCKS > 0
#include "hardware/sync.h"
#include "hardware/structs/bootram.h"

/** \brief A boot lock identifier
 * \ingroup hardware_sync
 */
typedef volatile uint32_t boot_lock_t;

/*! \brief Get HW Bootlock instance from number
 *  \ingroup hardware_sync
 *
 * \param lock_num Bootlock ID
 * \return The bootlock instance
 */
__force_inline static boot_lock_t *boot_lock_instance(uint lock_num) {
    invalid_params_if(HARDWARE_BOOT_LOCK, lock_num >= NUM_BOOT_LOCKS);
    return (boot_lock_t *) (BOOTRAM_BASE + BOOTRAM_BOOTLOCK0_OFFSET + lock_num * 4);
}

/*! \brief Get HW Bootlock number from instance
 *  \ingroup hardware_sync
 *
 * \param lock The Bootlock instance
 * \return The Bootlock ID
 */
__force_inline static uint boot_lock_get_num(boot_lock_t *lock) {
    invalid_params_if(HARDWARE_BOOT_LOCK, (uint) lock < BOOTRAM_BASE + BOOTRAM_BOOTLOCK0_OFFSET ||
                            (uint) lock >= NUM_BOOT_LOCKS * sizeof(boot_lock_t) + BOOTRAM_BASE + BOOTRAM_BOOTLOCK0_OFFSET ||
                            ((uint) lock - BOOTRAM_BASE + BOOTRAM_BOOTLOCK0_OFFSET) % sizeof(boot_lock_t) != 0);
    return (uint) (lock - (boot_lock_t *) (BOOTRAM_BASE + BOOTRAM_BOOTLOCK0_OFFSET));
}

/*! \brief Acquire a boot lock without disabling interrupts (hence unsafe)
 *  \ingroup hardware_sync
 *
 * \param lock Bootlock instance
 */
__force_inline static void boot_lock_unsafe_blocking(boot_lock_t *lock) {
    // Note we don't do a wfe or anything, because by convention these boot_locks are VERY SHORT LIVED and NEVER BLOCK and run
    // with INTERRUPTS disabled (to ensure that)... therefore nothing on our core could be blocking us, so we just need to wait on another core
    // anyway which should be finished soon
    while (__builtin_expect(!*lock, 0)) { // read from bootlock register (tries to acquire the lock)
        tight_loop_contents();
    }
    __mem_fence_acquire();
}

/*! \brief try to acquire a boot lock without disabling interrupts (hence unsafe)
 *  \ingroup hardware_sync
 *
 * \param lock Bootlock instance
 */
__force_inline static bool boot_try_lock_unsafe(boot_lock_t *lock) {
    if (*lock) {
        __mem_fence_acquire();
        return true;
    }
    return false;
}

/*! \brief Release a boot lock without re-enabling interrupts
 *  \ingroup hardware_sync
 *
 * \param lock Bootlock instance
 */
__force_inline static void boot_unlock_unsafe(boot_lock_t *lock) {
    __mem_fence_release();
    *lock = 0; // write to bootlock register (release lock)
}

/*! \brief Acquire a boot lock safely
 *  \ingroup hardware_sync
 *
 * This function will disable interrupts prior to acquiring the bootlock
 *
 * \param lock Bootlock instance
 * \return interrupt status to be used when unlocking, to restore to original state
 */
__force_inline static uint32_t boot_lock_blocking(boot_lock_t *lock) {
    uint32_t save = save_and_disable_interrupts();
    boot_lock_unsafe_blocking(lock);
    return save;
}

/*! \brief Check to see if a bootlock is currently acquired elsewhere.
 *  \ingroup hardware_sync
 *
 * \param lock Bootlock instance
 */
inline static bool is_boot_locked(boot_lock_t *lock) {
    check_hw_size(boot_lock_t, 4);
    uint lock_num = boot_lock_get_num(lock);
    return 0 != (*(io_ro_32 *) (BOOTRAM_BASE + BOOTRAM_BOOTLOCK_STAT_OFFSET) & (1u << lock_num));
}

/*! \brief Release a boot lock safely
 *  \ingroup hardware_sync
 *
 * This function will re-enable interrupts according to the parameters.
 *
 * \param lock Bootlock instance
 * \param saved_irq Return value from the \ref boot_lock_blocking() function.
 *
 * \sa boot_lock_blocking()
 */
__force_inline static void boot_unlock(boot_lock_t *lock, uint32_t saved_irq) {
    boot_unlock_unsafe(lock);
    restore_interrupts_from_disabled(saved_irq);
}

/*! \brief Initialise a boot lock
 *  \ingroup hardware_sync
 *
 * The boot lock is initially unlocked
 *
 * \param lock_num The boot lock number
 * \return The boot lock instance
 */
boot_lock_t *boot_lock_init(uint lock_num);

/*! \brief Release all boot locks
 *  \ingroup hardware_sync
 */
void boot_locks_reset(void);

#endif
#endif