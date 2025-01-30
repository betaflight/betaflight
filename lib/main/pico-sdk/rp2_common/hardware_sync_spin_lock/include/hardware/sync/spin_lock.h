/*
 * Copyright (c) 2024 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _HARDWARE_SYNC_SPIN_LOCK_H
#define _HARDWARE_SYNC_SPIN_LOCK_H

#include "pico.h"
#include "hardware/sync.h"

// PICO_CONFIG: PICO_USE_SW_SPIN_LOCKS, Use software implementation for spin locks, type=bool, default=1 on RP2350 due to errata, group=hardware_sync
#ifndef PICO_USE_SW_SPIN_LOCKS
#if PICO_RP2350
#define PICO_USE_SW_SPIN_LOCKS 1
#endif
#endif

// PICO_CONFIG: PICO_SPINLOCK_ID_IRQ, Spinlock ID for IRQ protection, min=0, max=31, default=9, group=hardware_sync
#ifndef PICO_SPINLOCK_ID_IRQ
#define PICO_SPINLOCK_ID_IRQ 9
#endif

// PICO_CONFIG: PICO_SPINLOCK_ID_TIMER, Spinlock ID for Timer protection, min=0, max=31, default=10, group=hardware_sync
#ifndef PICO_SPINLOCK_ID_TIMER
#define PICO_SPINLOCK_ID_TIMER 10
#endif

// PICO_CONFIG: PICO_SPINLOCK_ID_HARDWARE_CLAIM, Spinlock ID for Hardware claim protection, min=0, max=31, default=11, group=hardware_sync
#ifndef PICO_SPINLOCK_ID_HARDWARE_CLAIM
#define PICO_SPINLOCK_ID_HARDWARE_CLAIM 11
#endif

// PICO_CONFIG: PICO_SPINLOCK_ID_RAND, Spinlock ID for Random Number Generator, min=0, max=31, default=12, group=hardware_sync
#ifndef PICO_SPINLOCK_ID_RAND
#define PICO_SPINLOCK_ID_RAND 12
#endif

// PICO_CONFIG: PICO_SPINLOCK_ID_ATOMIC, Spinlock ID for atomics, min=0, max=31, default=13, group=hardware_sync
#ifndef PICO_SPINLOCK_ID_ATOMIC
#define PICO_SPINLOCK_ID_ATOMIC 13
#endif

// PICO_CONFIG: PICO_SPINLOCK_ID_OS1, First Spinlock ID reserved for use by low level OS style software, min=0, max=31, default=14, group=hardware_sync
#ifndef PICO_SPINLOCK_ID_OS1
#define PICO_SPINLOCK_ID_OS1 14
#endif

// PICO_CONFIG: PICO_SPINLOCK_ID_OS2, Second Spinlock ID reserved for use by low level OS style software, min=0, max=31, default=15, group=hardware_sync
#ifndef PICO_SPINLOCK_ID_OS2
#define PICO_SPINLOCK_ID_OS2 15
#endif

// PICO_CONFIG: PICO_SPINLOCK_ID_STRIPED_FIRST, Lowest Spinlock ID in the 'striped' range, min=0, max=31, default=16, group=hardware_sync
#ifndef PICO_SPINLOCK_ID_STRIPED_FIRST
#define PICO_SPINLOCK_ID_STRIPED_FIRST 16
#endif

// PICO_CONFIG: PICO_SPINLOCK_ID_STRIPED_LAST, Highest Spinlock ID in the 'striped' range, min=0, max=31, default=23, group=hardware_sync
#ifndef PICO_SPINLOCK_ID_STRIPED_LAST
#define PICO_SPINLOCK_ID_STRIPED_LAST 23
#endif

// PICO_CONFIG: PICO_SPINLOCK_ID_CLAIM_FREE_FIRST, Lowest Spinlock ID in the 'claim free' range, min=0, max=31, default=24, group=hardware_sync
#ifndef PICO_SPINLOCK_ID_CLAIM_FREE_FIRST
#define PICO_SPINLOCK_ID_CLAIM_FREE_FIRST 24
#endif

#ifdef PICO_SPINLOCK_ID_CLAIM_FREE_END
#warning PICO_SPINLOCK_ID_CLAIM_FREE_END has been renamed to PICO_SPINLOCK_ID_CLAIM_FREE_LAST
#endif

// PICO_CONFIG: PICO_SPINLOCK_ID_CLAIM_FREE_LAST, Highest Spinlock ID in the 'claim free' range, min=0, max=31, default=31, group=hardware_sync
#ifndef PICO_SPINLOCK_ID_CLAIM_FREE_LAST
#define PICO_SPINLOCK_ID_CLAIM_FREE_LAST 31
#endif

/** \brief A spin lock identifier
 * \ingroup hardware_sync
 */
#if !PICO_USE_SW_SPIN_LOCKS
// Hardware lock flag in SIO:
typedef io_rw_32 spin_lock_t;
#else
#ifndef SW_SPIN_LOCK_TYPE
// Byte flag in memory:
#define SW_SPIN_LOCK_TYPE volatile uint8_t
#endif
typedef SW_SPIN_LOCK_TYPE spin_lock_t;
#endif

#if PICO_USE_SW_SPIN_LOCKS
#ifndef SW_SPIN_LOCK_INSTANCE
#define SW_SPIN_LOCK_INSTANCE(lock_num) ({             \
    extern spin_lock_t _sw_spin_locks[NUM_SPIN_LOCKS]; \
    &_sw_spin_locks[lock_num];                         \
    })
#endif

#ifndef SW_SPIN_LOCK_NUM
#define SW_SPIN_LOCK_NUM(lock) ({                          \
        extern spin_lock_t _sw_spin_locks[NUM_SPIN_LOCKS]; \
        (lock) - _sw_spin_locks;                           \
        })
#endif

#ifndef SW_SPIN_LOCK_IS_LOCKED
#define SW_SPIN_LOCK_IS_LOCKED(lock) ((bool) *(lock))
#endif

#ifndef SW_SPIN_LOCK_LOCK
#if __ARM_ARCH_8M_MAIN__
#define SW_SPIN_LOCK_LOCK(lock) ({                             \
    uint32_t _tmp0, _tmp1;                                     \
    pico_default_asm_volatile (                                \
    "1:\n"                                                     \
    "ldaexb %1, [%2]\n"                                        \
    "movs %0, #1\n" /* fill dependency slot */                 \
    "cmp %1, #0\n"                                             \
    /* Immediately retry if lock is seen to be taken */        \
    "bne 1b\n"                                                 \
    /* Attempt to claim */                                     \
    "strexb %1, %0, [%2]\n"                                    \
    "cmp %1, #0\n"                                             \
    /* Claim failed due to intervening write, so retry */      \
    "bne 1b\n"                                                 \
    : "=&r" (_tmp0), "=&r" (_tmp1) : "r" (lock)                \
    );                                                         \
    __mem_fence_acquire();                                     \
    })
#elif __riscv && (defined(__riscv_a) || defined(__riscv_zaamo))
#define SW_SPIN_LOCK_LOCK(lock) ({                                              \
    uint32_t _tmp0, _tmp1;                                                      \
    pico_default_asm_volatile (                                                 \
        /* Get word address, and bit mask for LSB of the */                     \
        /* correct byte within that word -- note shamt is modulo xlen: */       \
        "slli %1, %0, 3\n"                                                      \
        "bset %1, zero, %1\n"                                                   \
        "andi %0, %0, -4\n"                                                     \
        /* Repeatedly set the bit until we see that it was clear at the */      \
        /* point we set it. A set from 0 -> 1 is a successful lock take. */     \
    "1:"                                                                        \
        "amoor.w.aq %2, %1, (%0)\n"                                             \
        "and %2, %2, %1\n"                                                      \
        "bnez %2, 1b\n"                                                         \
        : "+r" (lock), "=r" (_tmp0), "=r" (_tmp1)                               \
    );                                                                          \
    __mem_fence_acquire();                                                      \
    })
#else
#error no SW_SPIN_TRY_LOCK available for PICO_USE_SW_SPIN_LOCK on this platform
#endif
#endif

#ifndef SW_SPIN_TRY_LOCK
#if __ARM_ARCH_8M_MAIN__
#define SW_SPIN_TRY_LOCK(lock) ({                              \
    uint32_t _tmp0, _tmp1;                                     \
    pico_default_asm_volatile (                                \
    "ldaexb %1, [%2]\n"                                        \
    "movs %0, #1\n" /* fill dependency slot */                 \
    "cmp %1, #0\n"                                             \
    /* Immediately give up if lock is seen to be taken */      \
    "bne 1f\n"                                                 \
    /* Otherwise attempt to claim, once. */                    \
    "strexb %1, %0, [%2]\n"                                    \
    "1:\n"                                                     \
    : "=&r" (_tmp0), "=&r" (_tmp1) : "r" (lock)                \
    );                                                         \
    __mem_fence_acquire();                                     \
    !_tmp1;                                                    \
    })
#elif __riscv && (defined(__riscv_a) || defined(__riscv_zaamo))
#define SW_SPIN_TRY_LOCK(lock) ({                                               \
    uint32_t _tmp0;                                                             \
    pico_default_asm_volatile (                                                 \
        /* Get word address, and bit mask for LSB of the */                     \
        /* correct byte within that word -- note shamt is modulo xlen: */       \
        "slli %1, %0, 3\n"                                                      \
        "bset %1, zero, %1\n"                                                   \
        "andi %0, %0, -4\n"                                                     \
        /* Set the bit. If it was clear at the point we set it, then we took */ \
        /* the lock. Otherwise the lock was already held, and we give up. */    \
        "amoor.w.aq %0, %1, (%0)\n"                                             \
        "and %1, %1, %0\n"                                                      \
        : "+r" (lock), "=r" (_tmp0)                                             \
    );                                                                          \
    __mem_fence_acquire();                                                      \
    !_tmp0;                                                                     \
    })
#else
#error no SW_SPIN_TRY_LOCK available for PICO_USE_SW_SPIN_LOCK on this platform
#endif
#endif

#ifndef SW_SPIN_LOCK_UNLOCK
#if __ARM_ARCH_8M_MAIN__
#define SW_SPIN_LOCK_UNLOCK(lock) ({                                        \
    /* Release-ordered store is available: use instead of separate fence */ \
    uint32_t zero = 0;                                                      \
    pico_default_asm_volatile(                                              \
        "stlb %0, [%1]\n"                                                   \
        : : "r" (zero), "r" (lock)                                          \
    );                                                                      \
    })
#elif __riscv
#define SW_SPIN_LOCK_UNLOCK(lock) ({                              \
    __mem_fence_release();                                        \
    *(lock) = 0; /* write to spinlock register (release lock) */  \
    })
#else
#error no SW_SPIN_TRY_LOCK available for PICO_USE_SW_SPIN_LOCK on this platform
#endif
#endif

#endif

/*! \brief Get HW Spinlock instance from number
 *  \ingroup hardware_sync
 *
 * \param lock_num Spinlock ID
 * \return The spinlock instance
 */
__force_inline static spin_lock_t *spin_lock_instance(uint lock_num) {
    invalid_params_if(HARDWARE_SYNC, lock_num >= NUM_SPIN_LOCKS);
#if PICO_USE_SW_SPIN_LOCKS
    return SW_SPIN_LOCK_INSTANCE(lock_num);
#else
    return (spin_lock_t *) (SIO_BASE + SIO_SPINLOCK0_OFFSET + lock_num * 4);
#endif
}

/*! \brief Get HW Spinlock number from instance
 *  \ingroup hardware_sync
 *
 * \param lock The Spinlock instance
 * \return The Spinlock ID
 */
__force_inline static uint spin_lock_get_num(spin_lock_t *lock) {
#if PICO_USE_SW_SPIN_LOCKS
    uint lock_num = SW_SPIN_LOCK_NUM(lock);
    invalid_params_if(HARDWARE_SYNC, lock_num >= (uint)NUM_SPIN_LOCKS);
    return lock_num;
#else
    invalid_params_if(HARDWARE_SYNC, (uint) lock < SIO_BASE + SIO_SPINLOCK0_OFFSET ||
                            (uint) lock >= NUM_SPIN_LOCKS * sizeof(spin_lock_t) + SIO_BASE + SIO_SPINLOCK0_OFFSET ||
                            ((uint) lock - SIO_BASE + SIO_SPINLOCK0_OFFSET) % sizeof(spin_lock_t) != 0);
    return (uint) (lock - (spin_lock_t *) (SIO_BASE + SIO_SPINLOCK0_OFFSET));
#endif
}

/*! \brief Acquire a spin lock without disabling interrupts (hence unsafe)
 *  \ingroup hardware_sync
 *
 * \param lock Spinlock instance
 */
__force_inline static void spin_lock_unsafe_blocking(spin_lock_t *lock) {
    // Note we don't do a wfe or anything, because by convention these spin_locks are VERY SHORT LIVED and NEVER BLOCK and run
    // with INTERRUPTS disabled (to ensure that)... therefore nothing on our core could be blocking us, so we just need to wait on another core
    // anyway which should be finished soon
#if PICO_USE_SW_SPIN_LOCKS
    SW_SPIN_LOCK_LOCK(lock);
#else
    while (__builtin_expect(!*lock, 0)) { // read from spinlock register (tries to acquire the lock)
        tight_loop_contents();
    }
    __mem_fence_acquire();
#endif
}

__force_inline static bool spin_try_lock_unsafe(spin_lock_t *lock) {
#if PICO_USE_SW_SPIN_LOCKS
    return SW_SPIN_TRY_LOCK(lock);
#else
    return *lock;
#endif
}
/*! \brief Release a spin lock without re-enabling interrupts
 *  \ingroup hardware_sync
 *
 * \param lock Spinlock instance
 */
__force_inline static void spin_unlock_unsafe(spin_lock_t *lock) {
#if PICO_USE_SW_SPIN_LOCKS
    SW_SPIN_LOCK_UNLOCK(lock);
#else
    __mem_fence_release();
    *lock = 0; // write to spinlock register (release lock)
#endif
}

/*! \brief Acquire a spin lock safely
 *  \ingroup hardware_sync
 *
 * This function will disable interrupts prior to acquiring the spinlock
 *
 * \param lock Spinlock instance
 * \return interrupt status to be used when unlocking, to restore to original state
 */
__force_inline static uint32_t spin_lock_blocking(spin_lock_t *lock) {
    uint32_t save = save_and_disable_interrupts();
    spin_lock_unsafe_blocking(lock);
    return save;
}

/*! \brief Check to see if a spinlock is currently acquired elsewhere.
 *  \ingroup hardware_sync
 *
 * \param lock Spinlock instance
 */
inline static bool is_spin_locked(spin_lock_t *lock) {
#if PICO_USE_SW_SPIN_LOCKS
    return SW_SPIN_LOCK_IS_LOCKED(lock);
#else
    check_hw_size(spin_lock_t, 4);
    uint lock_num = spin_lock_get_num(lock);
    return 0 != (*(io_ro_32 *) (SIO_BASE + SIO_SPINLOCK_ST_OFFSET) & (1u << lock_num));
#endif
}

/*! \brief Release a spin lock safely
 *  \ingroup hardware_sync
 *
 * This function will re-enable interrupts according to the parameters.
 *
 * \param lock Spinlock instance
 * \param saved_irq Return value from the \ref spin_lock_blocking() function.
 *
 * \sa spin_lock_blocking()
 */
__force_inline static void spin_unlock(spin_lock_t *lock, uint32_t saved_irq) {
    spin_unlock_unsafe(lock);
    restore_interrupts_from_disabled(saved_irq);
}

/*! \brief Initialise a spin lock
 *  \ingroup hardware_sync
 *
 * The spin lock is initially unlocked
 *
 * \param lock_num The spin lock number
 * \return The spin lock instance
 */
spin_lock_t *spin_lock_init(uint lock_num);

/*! \brief Release all spin locks
 *  \ingroup hardware_sync
 */
void spin_locks_reset(void);

#endif
