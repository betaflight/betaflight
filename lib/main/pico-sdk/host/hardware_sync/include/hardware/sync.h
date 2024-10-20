/*
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _HARDWARE_SYNC_H
#define _HARDWARE_SYNC_H

#include "pico.h"

#ifndef __cplusplus

#if (__STDC_VERSION__ >= 201112L)
#include <stdatomic.h>
#else
enum {
    memory_order_acquire, memory_order_release
};
static inline void atomic_thread_fence(uint x) {}
#endif

#else

#include <atomic>

#endif

// PICO_CONFIG: PICO_SPINLOCK_ID_ATOMIC, Spinlock ID for atomics, min=0, max=31, default=8, group=hardware_sync
#ifndef PICO_SPINLOCK_ID_ATOMIC
#define PICO_SPINLOCK_ID_ATOMIC 8
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

typedef struct _spin_lock_t spin_lock_t;

inline static void __mem_fence_acquire() {
#ifndef __cplusplus
    atomic_thread_fence(memory_order_acquire);
#else
    std::atomic_thread_fence(std::memory_order_acquire);
#endif
}

inline static void __mem_fence_release() {
#ifndef __cplusplus
    atomic_thread_fence(memory_order_release);
#else
    std::atomic_thread_fence(std::memory_order_release);
#endif
}

#ifdef __cplusplus
extern "C" {
#endif

void __sev();

void __wev();

void __wfi();

void __wfe();

uint32_t save_and_disable_interrupts();

void restore_interrupts(uint32_t status);

void restore_interrupts_from_disabled(uint32_t status);

uint spin_lock_get_num(spin_lock_t *lock);

spin_lock_t *spin_lock_instance(uint lock_num);

void spin_lock_unsafe_blocking(spin_lock_t *lock);

void spin_unlock_unsafe(spin_lock_t *lock);

uint32_t spin_lock_blocking(spin_lock_t *lock);

bool is_spin_locked(const spin_lock_t *lock);

void spin_unlock(spin_lock_t *lock, uint32_t saved_irq);

spin_lock_t *spin_lock_init(uint lock_num);

void clear_spin_locks(void);
#define spin_locks_reset() clear_spin_locks()

uint next_striped_spin_lock_num();

void spin_lock_claim(uint lock_num);
void spin_lock_claim_mask(uint32_t lock_num_mask);
void spin_lock_unclaim(uint lock_num);
int spin_lock_claim_unused(bool required);
uint spin_lock_num(spin_lock_t *lock);

#ifdef __cplusplus
}
#endif
#endif
