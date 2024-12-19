/*
 * Copyright (c) 2024 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _PICO_BOOTROM_LOCK_H
#define _PICO_BOOTROM_LOCK_H

#include "hardware/boot_lock.h"
#include "pico/bootrom_constants.h"

// PICO_CONFIG: PICO_BOOTROM_LOCKING_ENABLED, Enable/disable locking for bootrom functions that use shared resources. If this flag is enabled bootrom lock checking is turned on and BOOT locks are taken around the relevant bootrom functions, type=bool, default=1, group=pico_bootrom
#ifndef PICO_BOOTROM_LOCKING_ENABLED
#if NUM_BOOT_LOCKS > 0
#define PICO_BOOTROM_LOCKING_ENABLED 1
#endif
#endif

/**
 * \brief Try to acquire a bootrom lock
 *
 * If PICO_BOOTROM_LOCKING_ENABLED is false, this method returns true immediately
 *
 * \param lock_num the lock numbers - BOOTROM_LOCK_SHA_256, BOOTROM_LOCK_FLASH_OP or BOOTROM_LOCK_OTP
 * \return true if the lock was acquired
 */
static inline bool bootrom_try_acquire_lock(uint lock_num) {
#if PICO_BOOTROM_LOCKING_ENABLED
    // unsafe as this is a long term lock (so no irq disable)
    return boot_try_lock_unsafe(boot_lock_instance(lock_num));
#else
    (void)lock_num;
    return true;
#endif
}

/**
 * \brief Acquire a bootrom lock. If the lock is unavailable, block until it is available
 *
 * If PICO_BOOTROM_LOCKING_ENABLED is false, this method does nothing
 *
 * \param lock_num the lock numbers - BOOTROM_LOCK_SHA_256, BOOTROM_LOCK_FLASH_OP or BOOTROM_LOCK_OTP
 */
static inline void bootrom_acquire_lock_blocking(uint lock_num) {
#if PICO_BOOTROM_LOCKING_ENABLED
    // unsafe as this is a long term lock (so no irq disable)
    boot_lock_unsafe_blocking(boot_lock_instance(lock_num));
#else
    (void)lock_num;
#endif
}

/**
 * \brief Release a bootrom lock
 *
 * If PICO_BOOTROM_LOCKING_ENABLED is false, this method does nothing
 *
 * \param lock_num the lock numbers - BOOTROM_LOCK_SHA_256, BOOTROM_LOCK_FLASH_OP or BOOTROM_LOCK_OTP
 */
static inline void bootrom_release_lock(uint lock_num) {
#if PICO_BOOTROM_LOCKING_ENABLED
    boot_unlock_unsafe(boot_lock_instance(lock_num));
#else
    (void)lock_num;
#endif
}

#endif
