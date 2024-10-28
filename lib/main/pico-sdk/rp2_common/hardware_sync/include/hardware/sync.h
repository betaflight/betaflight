/*
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _HARDWARE_SYNC_H
#define _HARDWARE_SYNC_H

#include "pico.h"
#include "hardware/address_mapped.h"

#ifdef __riscv
#include "hardware/hazard3.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif

/** \file hardware/sync.h
 *  \defgroup hardware_sync hardware_sync
 *
 * \brief Low level hardware spin locks, barrier and processor event APIs
 *
 * Spin Locks
 * ----------
 *
 * The RP-series microcontrollers provide 32 hardware spin locks, which can be used to manage mutually-exclusive access to shared software
 * and hardware resources.
 *
 * Generally each spin lock itself is a shared resource,
 * i.e. the same hardware spin lock can be used by multiple higher level primitives (as long as the spin locks are neither held for long periods, nor
 * held concurrently with other spin locks by the same core - which could lead to deadlock). A hardware spin lock that is exclusively owned can be used
 * individually without more flexibility and without regard to other software. Note that no hardware spin lock may
 * be acquired re-entrantly (i.e. hardware spin locks are not on their own safe for use by both thread code and IRQs) however the default spinlock related
 * methods here (e.g. \ref spin_lock_blocking) always disable interrupts while the lock is held as use by IRQ handlers and user code is common/desirable,
 * and spin locks are only expected to be held for brief periods.
 *
 * \if rp2350_specific
 * RP2350 Warning. Due to erratum RP2350-E2, writes to new SIO registers above an offset of +0x180 alias the spinlocks, causing spurious lock releases.
 * This SDK by default uses atomic memory accesses to implement the hardware_sync_spin_lock API, as a workaround on RP2350 A2.
 * \endif
 *
 * The SDK uses the following default spin lock assignments, classifying which spin locks are reserved for exclusive/special purposes
 * vs those suitable for more general shared use:
 *
 * Number (ID) | Description
 * :---------: | -----------
 * 0-13        | Currently reserved for exclusive use by the SDK and other libraries. If you use these spin locks, you risk breaking SDK or other library functionality. Each reserved spin lock used individually has its own PICO_SPINLOCK_ID so you can search for those.
 * 14,15       | (\ref PICO_SPINLOCK_ID_OS1 and \ref PICO_SPINLOCK_ID_OS2). Currently reserved for exclusive use by an operating system (or other system level software) co-existing with the SDK.
 * 16-23       | (\ref PICO_SPINLOCK_ID_STRIPED_FIRST - \ref PICO_SPINLOCK_ID_STRIPED_LAST). Spin locks from this range are assigned in a round-robin fashion via \ref next_striped_spin_lock_num(). These spin locks are shared, but assigning numbers from a range reduces the probability that two higher level locking primitives using _striped_ spin locks will actually be using the same spin lock.
 * 24-31       | (\ref PICO_SPINLOCK_ID_CLAIM_FREE_FIRST - \ref PICO_SPINLOCK_ID_CLAIM_FREE_LAST). These are reserved for exclusive use and are allocated on a first come first served basis at runtime via \ref spin_lock_claim_unused()
 */

// PICO_CONFIG: PARAM_ASSERTIONS_ENABLED_HARDWARE_SYNC, Enable/disable assertions in the hardware_sync module, type=bool, default=0, group=hardware_sync
#ifndef PARAM_ASSERTIONS_ENABLED_HARDWARE_SYNC
#ifdef PARAM_ASSERTIONS_ENABLED_SYNC // backwards compatibility with SDK < 2.0.0
#define PARAM_ASSERTIONS_ENABLED_HARDWARE_SYNC PARAM_ASSERTIONS_ENABLED_SYNC
#else
#define PARAM_ASSERTIONS_ENABLED_HARDWARE_SYNC 0
#endif
#endif

/*! \brief Insert a NOP instruction in to the code path.
 *  \ingroup hardware_sync
 *
 * NOP does nothing for one cycle. On RP2350 Arm binaries this is forced to be
 * a 32-bit instruction to avoid dual-issue of NOPs.
 */
__force_inline static void __nop(void) {
#if !__ARM_ARCH_6M__
#ifdef __riscv
    __asm volatile ("nop");
#else
    __asm volatile ("nop.w");
#endif
#else
    __asm volatile ("nop");
#endif
}


/*! \brief Insert a SEV instruction in to the code path.
 *  \ingroup hardware_sync

 * The SEV (send event) instruction sends an event to both cores.
 */
#if !__has_builtin(__sev)
__force_inline static void __sev(void) {
#ifdef __riscv
    __hazard3_unblock();
#else
    pico_default_asm_volatile ("sev");
#endif
}
#endif

/*! \brief Insert a WFE instruction in to the code path.
 *  \ingroup hardware_sync
 *
 * The WFE (wait for event) instruction waits until one of a number of
 * events occurs, including events signalled by the SEV instruction on either core.
 */
#if !__has_builtin(__wfe)
__force_inline static void __wfe(void) {
#ifdef __riscv
    __hazard3_block();
#else
    pico_default_asm_volatile ("wfe");
#endif
}
#endif

/*! \brief Insert a WFI instruction in to the code path.
  *  \ingroup hardware_sync
*
 * The WFI (wait for interrupt) instruction waits for a interrupt to wake up the core.
 */
#if !__has_builtin(__wfi)
__force_inline static void __wfi(void) {
    pico_default_asm_volatile("wfi");
}
#endif

/*! \brief Insert a DMB instruction in to the code path.
 *  \ingroup hardware_sync
 *
 * The DMB (data memory barrier) acts as a memory barrier, all memory accesses prior to this
 * instruction will be observed before any explicit access after the instruction.
 */
__force_inline static void __dmb(void) {
#ifdef __riscv
    __asm volatile ("fence rw, rw" : : : "memory");
#else
    pico_default_asm_volatile ("dmb" : : : "memory");
#endif
}

/*! \brief Insert a DSB instruction in to the code path.
 *  \ingroup hardware_sync
 *
 * The DSB (data synchronization barrier) acts as a special kind of data
 * memory barrier (DMB). The DSB operation completes when all explicit memory
 * accesses before this instruction complete.
 */
__force_inline static void __dsb(void) {
#ifdef __riscv
    __asm volatile ("fence rw, rw" : : : "memory");
#else
    pico_default_asm_volatile ("dsb" : : : "memory");
#endif
}

/*! \brief Insert a ISB instruction in to the code path.
 *  \ingroup hardware_sync
 *
 * ISB acts as an instruction synchronization barrier. It flushes the pipeline of the processor,
 * so that all instructions following the ISB are fetched from cache or memory again, after
 * the ISB instruction has been completed.
 */
__force_inline static void __isb(void) {
#ifdef __riscv
    __asm volatile ("fence.i" : : : "memory");
#else
    pico_default_asm_volatile("isb" ::: "memory");
#endif
}

/*! \brief Acquire a memory fence
 *  \ingroup hardware_sync
 */
__force_inline static void __mem_fence_acquire(void) {
    // the original code below makes it hard for us to be included from C++ via a header
    // which itself is in an extern "C", so just use __dmb instead, which is what
    // is required on Cortex M0+
    __dmb();
//#ifndef __cplusplus
//    atomic_thread_fence(memory_order_acquire);
//#else
//    std::atomic_thread_fence(std::memory_order_acquire);
//#endif
}

/*! \brief Release a memory fence
 *  \ingroup hardware_sync
 *
 */
__force_inline static void __mem_fence_release(void) {
    // the original code below makes it hard for us to be included from C++ via a header
    // which itself is in an extern "C", so just use __dmb instead, which is what
    // is required on Cortex M0+
    __dmb();
//#ifndef __cplusplus
//    atomic_thread_fence(memory_order_release);
//#else
//    std::atomic_thread_fence(std::memory_order_release);
//#endif
}

/*! \brief Save and disable interrupts
 *  \ingroup hardware_sync
 *
 * \return The prior interrupt enable status for restoration later via restore_interrupts()
 */
__force_inline static uint32_t save_and_disable_interrupts(void) {
    uint32_t status;
#ifdef __riscv
    pico_default_asm_volatile (
        "csrrci %0, mstatus, 0x8\n"
        : "=r" (status) :: "memory"
    );
#else
    pico_default_asm_volatile (
            "mrs %0, PRIMASK\n"
            "cpsid i"
            : "=r" (status) :: "memory");
#endif
    return status;
}

/*! \brief Restore interrupts to a specified state
 *  \ingroup hardware_sync
 *
 * \param status Previous interrupt status from save_and_disable_interrupts()
  */
__force_inline static void restore_interrupts(uint32_t status) {
#ifdef __riscv
    __compiler_memory_barrier();
    if (status & 0x8) {
        riscv_set_csr(mstatus, 8);
    } else {
        riscv_clear_csr(mstatus, 8);
    }
    __compiler_memory_barrier();
#else
    pico_default_asm_volatile ("msr PRIMASK,%0"::"r" (status) : "memory" );
#endif
}

/*! \brief Restore interrupts to a specified state with restricted transitions
 *  \ingroup hardware_sync
 *
 * This method should only be used when the interrupt state is known to be disabled,
 * e.g. when paired with \ref save_and_disable_interrupts()
 *
 * \param status Previous interrupt status from save_and_disable_interrupts()
  */
__force_inline static void restore_interrupts_from_disabled(uint32_t status) {
#ifdef __riscv
    // on RISC-V this can enable interrupts, but not disable interrupts... which
    // is the common case and doesn't require a branch
    __compiler_memory_barrier();
    riscv_set_csr(mstatus, status & 8);
    __compiler_memory_barrier();
#else
    // on ARM, this behaves the same as restore_interrupts()
    pico_default_asm_volatile ("msr PRIMASK,%0"::"r" (status) : "memory" );
#endif
}

#include "hardware/sync/spin_lock.h"

/*! \brief Return a spin lock number from the _striped_ range
 *  \ingroup hardware_sync
 *
 * Returns a spin lock number in the range PICO_SPINLOCK_ID_STRIPED_FIRST to PICO_SPINLOCK_ID_STRIPED_LAST
 * in a round robin fashion. This does not grant the caller exclusive access to the spin lock, so the caller
 * must:
 *
 * -# Abide (with other callers) by the contract of only holding this spin lock briefly (and with IRQs disabled - the default via \ref spin_lock_blocking()),
 * and not whilst holding other spin locks.
 * -# Be OK with any contention caused by the - brief due to the above requirement - contention with other possible users of the spin lock.
 *
 * \return lock_num a spin lock number the caller may use (non exclusively)
 * \see PICO_SPINLOCK_ID_STRIPED_FIRST
 * \see PICO_SPINLOCK_ID_STRIPED_LAST
 */
uint next_striped_spin_lock_num(void);

/*! \brief Mark a spin lock as used
 *  \ingroup hardware_sync
 *
 * Method for cooperative claiming of hardware. Will cause a panic if the spin lock
 * is already claimed. Use of this method by libraries detects accidental
 * configurations that would fail in unpredictable ways.
 *
 * \param lock_num the spin lock number
 */
void spin_lock_claim(uint lock_num);

/*! \brief Mark multiple spin locks as used
 *  \ingroup hardware_sync
 *
 * Method for cooperative claiming of hardware. Will cause a panic if any of the spin locks
 * are already claimed. Use of this method by libraries detects accidental
 * configurations that would fail in unpredictable ways.
 *
 * \param lock_num_mask Bitfield of all required spin locks to claim (bit 0 == spin lock 0, bit 1 == spin lock 1 etc)
 */
void spin_lock_claim_mask(uint32_t lock_num_mask);

/*! \brief Mark a spin lock as no longer used
 *  \ingroup hardware_sync
 *
 * Method for cooperative claiming of hardware.
 *
 * \param lock_num the spin lock number to release
 */
void spin_lock_unclaim(uint lock_num);

/*! \brief Claim a free spin lock
 *  \ingroup hardware_sync
 *
 * \param required if true the function will panic if none are available
 * \return the spin lock number or -1 if required was false, and none were free
 */
int spin_lock_claim_unused(bool required);

/*! \brief Determine if a spin lock is claimed
 *  \ingroup hardware_sync
 *
 * \param lock_num the spin lock number
 * \return true if claimed, false otherwise
 * \see spin_lock_claim
 * \see spin_lock_claim_mask
 */
bool spin_lock_is_claimed(uint lock_num);

// no longer use __mem_fence_acquire here, as it is overkill on cortex M0+
#if PICO_C_COMPILER_IS_GNU
#define remove_volatile_cast(t, x) (t)(x)
#define remove_volatile_cast_no_barrier(t, x) (t)(x)
#else
#define remove_volatile_cast(t, x) ({__compiler_memory_barrier(); Clang_Pragma("clang diagnostic push"); Clang_Pragma("clang diagnostic ignored \"-Wcast-qual\""); (t)(x); Clang_Pragma("clang diagnostic pop"); })
#define remove_volatile_cast_no_barrier(t, x) ({ Clang_Pragma("clang diagnostic push"); Clang_Pragma("clang diagnostic ignored \"-Wcast-qual\""); (t)(x); Clang_Pragma("clang diagnostic pop"); })
#endif

#ifdef __cplusplus
}
#endif

#endif
