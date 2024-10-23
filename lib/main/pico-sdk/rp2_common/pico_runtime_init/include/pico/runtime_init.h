/*
 * Copyright (c) 2024 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _PICO_RUNTIME_INITS_H
#define _PICO_RUNTIME_INITS_H

#include "pico.h"
#include "pico/runtime.h"

#ifdef __cplusplus
extern "C" {
#endif

/** \file pico/runtime_init.h
 *  \defgroup pico_runtime_init pico_runtime_init
 *
 * \brief Main runtime initialization functions required to set up the runtime environment before entering main
 *
 * The runtime initialization is registration based:
 *
 * For each step of the initialization there is a 5 digit ordinal which indicates
 * the ordering (alphabetic increasing sort of the 5 digits) of the steps.
 *
 * e.g. for the step "bootrom_reset", there is:
 *
 * \code
 * #ifndef PICO_RUNTIME_INIT_BOOTROM_RESET
 * #define PICO_RUNTIME_INIT_BOOTROM_RESET   "00050"
 * #endif
 * \endcode
 *
 * The user can override the order if they wish, by redefining PICO_RUNTIME_INIT_BOOTROM_RESET
 *
 * For each step, the automatic initialization may be skipped by defining (in this case)
 * PICO_RUNTIME_SKIP_INIT_BOOTROM_RESET = 1. The user can then choose to either omit the step
 * completely or register their own replacement initialization.
 *
 * The default method used to perform the initialization is provided, in case the user
 * wishes to call it manually; in this case:
 *
 * \code
 * void runtime_init_bootrom_reset(void);
 * \endcode
 *
 * If PICO_RUNTIME_NO_INIT_BOOTOROM_RESET define is set (NO vs SKIP above), then the function
 * is not defined, allowing the user to provide a replacement (and also avoiding
 * cases where the default implementation won't compile due to missing dependencies)
 */

// must have no dependency on any other initialization code
#define PICO_RUNTIME_INIT_EARLIEST              "00001"

// -----------------------------------------------------------------------------------------------
// Reset of global bootrom state (can be skipped if boot path was via bootrom); not used on RP2040
// -----------------------------------------------------------------------------------------------
// PICO_CONFIG: PICO_RUNTIME_SKIP_INIT_BOOTROM_RESET, Skip calling of `runtime_init_bootrom_reset` function during runtime init, type=bool, default=1 on RP2040, group=pico_runtime_init
// PICO_CONFIG: PICO_RUNTIME_NO_INIT_BOOTROM_RESET, Do not include SDK implementation of `runtime_init_bootrom_reset` function, type=bool, default=1 on RP2040, group=pico_runtime_init

#ifndef PICO_RUNTIME_INIT_BOOTROM_RESET
#define PICO_RUNTIME_INIT_BOOTROM_RESET   "00050"
#endif

#ifndef PICO_RUNTIME_SKIP_INIT_BOOTROM_RESET
#if PICO_RP2040 || (!LIB_PICO_MULTICORE && PICO_NO_FLASH)
#define PICO_RUNTIME_SKIP_INIT_BOOTROM_RESET 1
#endif
#endif

#ifndef PICO_RUNTIME_NO_INIT_BOOTROM_RESET
#if PICO_RP2040 || (!LIB_PICO_MULTICORE && PICO_NO_FLASH)
#define PICO_RUNTIME_NO_INIT_BOOTROM_RESET 1
#endif
#endif

#ifndef __ASSEMBLER__
void runtime_init_bootrom_reset(void);
#endif

// ---------------------------------------------------------------------------------------
// Non-boot core eset of bootrom state, not needed if only using core 0 not used on RP2040
// ---------------------------------------------------------------------------------------
#ifndef PICO_RUNTIME_INIT_PER_CORE_BOOTROM_RESET
#define PICO_RUNTIME_INIT_PER_CORE_BOOTROM_RESET "00051"
#endif

// PICO_CONFIG: PICO_RUNTIME_SKIP_INIT_PER_CORE_BOOTROM_RESET, Skip calling of `runtime_init_per_core_bootrom_reset` function during per-core init, type=bool, default=1 on RP2040, group=pico_runtime_init
// PICO_CONFIG: PICO_RUNTIME_NO_INIT_PER_CORE_BOOTROM_RESET, Do not include SDK implementation of `runtime_init_per_core_bootrom_reset` function, type=bool, default=1 on RP2040, group=pico_runtime_init
#ifndef PICO_RUNTIME_SKIP_INIT_PER_CORE_BOOTROM_RESET
#if PICO_RP2040
#define PICO_RUNTIME_SKIP_INIT_PER_CORE_BOOTROM_RESET 1
#endif
#endif

#ifndef PICO_RUNTIME_NO_INIT_PER_CORE_BOOTROM_RESET
#if PICO_RP2040
#define PICO_RUNTIME_NO_INIT_PER_CORE_BOOTROM_RESET 1
#endif
#endif

#ifndef __ASSEMBLER__
void runtime_init_per_core_bootrom_reset(void);
#endif

// ---------------------------
// Hazard3 processor IRQ setup
// ---------------------------

// PICO_CONFIG: PICO_RUNTIME_SKIP_INIT_PER_CORE_H3_IRQ_REGISTERS, Skip calling of `runtime_init_per_core_h3_irq_registers` function during per-core init, type=bool, default=1 on non RISC-V, group=pico_runtime_init

#ifndef PICO_RUNTIME_INIT_PER_CORE_H3_IRQ_REGISTERS
#define PICO_RUNTIME_INIT_PER_CORE_H3_IRQ_REGISTERS "00060"
#endif

#ifndef PICO_RUNTIME_SKIP_INIT_PER_CORE_H3_IRQ_REGISTERS
#ifndef __riscv
#define PICO_RUNTIME_SKIP_INIT_PER_CORE_H3_IRQ_REGISTERS 1
#endif
#endif

// -------------------------------
// Earliest resets (no clocks yet)
// -------------------------------
#ifndef PICO_RUNTIME_INIT_EARLY_RESETS
#define PICO_RUNTIME_INIT_EARLY_RESETS          "00100"
#endif

// PICO_CONFIG: PICO_RUNTIME_SKIP_INIT_EARLY_RESETS, Skip calling of `runtime_init_early_resets` function during runtime init, type=bool, default=1 on RP2040, group=pico_runtime_init
// PICO_CONFIG: PICO_RUNTIME_NO_INIT_EARLY_RESETS, Do not include SDK implementation of `runtime_init_early_resets` function, type=bool, default=1 on RP2040, group=pico_runtime_init
#ifndef PICO_RUNTIME_SKIP_INIT_EARLY_RESETS
#define PICO_RUNTIME_SKIP_INIT_EARLY_RESETS 0
#endif

#ifndef PICO_RUNTIME_NO_INIT_EARLY_RESETS
#define PICO_RUNTIME_NO_INIT_EARLY_RESETS 0
#endif

#ifndef __ASSEMBLER__
void runtime_init_early_resets(void);
#endif

// --------------
// USB power down
// --------------
// PICO_CONFIG: PICO_RUNTIME_SKIP_INIT_USB_POWER_DOWN, Skip calling of `runtime_init_usb_power_down` function during runtime init, type=bool, default=0, group=pico_runtime_init
// PICO_CONFIG: PICO_RUNTIME_NO_INIT_USB_POWER_DOWN, Do not include SDK implementation of `runtime_init_usb_power_down` function, type=bool, default=0, group=pico_runtime_init

#ifndef PICO_RUNTIME_INIT_USB_POWER_DOWN
#define PICO_RUNTIME_INIT_USB_POWER_DOWN        "00101"
#endif

#ifndef PICO_RUNTIME_SKIP_INIT_USB_POWER_DOWN
#define PICO_RUNTIME_SKIP_INIT_USB_POWER_DOWN 0
#endif

#ifndef PICO_RUNTIME_NO_INIT_USB_POWER_DOWN
#define PICO_RUNTIME_NO_INIT_USB_POWER_DOWN 0
#endif

#ifndef __ASSEMBLER__
void runtime_init_usb_power_down(void);
#endif

// ------------------------------------
// per core co-processor initialization
// ------------------------------------

// PICO_CONFIG: PICO_RUNTIME_SKIP_INIT_PER_CORE_ENABLE_COPROCESSORS, Skip calling of `runtime_init_per_core_enable_coprocessors` function during per-core init, type=bool, default=1 on RP2040 or RISC-V, group=pico_runtime_init
// PICO_CONFIG: PICO_RUNTIME_NO_INIT_PER_CORE_ENABLE_COPROCESSORS, Do not include SDK implementation of `runtime_init_per_core_enable_coprocessors` function, type=bool, default=1 on RP2040 or RISC-V, group=pico_runtime_init

#ifndef PICO_RUNTIME_INIT_PER_CORE_ENABLE_COPROCESSORS
#define PICO_RUNTIME_INIT_PER_CORE_ENABLE_COPROCESSORS "00200"
#endif

#ifndef PICO_RUNTIME_SKIP_INIT_PER_CORE_ENABLE_COPROCESSORS
#if PICO_RP2040 || defined(__riscv)
#define PICO_RUNTIME_SKIP_INIT_PER_CORE_ENABLE_COPROCESSORS 1
#endif
#endif

#ifndef PICO_RUNTIME_NO_INIT_PER_CORE_ENABLE_COPROCESSORS
#if PICO_RP2040 || defined(__riscv)
#define PICO_RUNTIME_NO_INIT_PER_CORE_ENABLE_COPROCESSORS 1
#endif
#endif

#ifndef __ASSEMBLER__
void runtime_init_per_core_enable_coprocessors(void);
#endif

// AEABI init; this initialization is auto-injected byte pico_aeebi_mem_ops if present
#ifndef PICO_RUNTIME_INIT_AEABI_MEM_OPS
// on RP2040 we need to get memcpy and memset hooked up to bootrom
#define PICO_RUNTIME_INIT_AEABI_MEM_OPS         "00300"
#endif

// AEABI init; this initialization is auto-injected byte pico_aeebi_bit_ops if present
#ifndef PICO_RUNTIME_INIT_AEABI_BIT_OPS
#define PICO_RUNTIME_INIT_AEABI_BIT_OPS         "00275"
#endif

// AEABI init; this initialization is auto-injected byte pico_aeebi_float if present
#ifndef PICO_RUNTIME_INIT_AEABI_FLOAT
#define PICO_RUNTIME_INIT_AEABI_FLOAT           "00350"
#endif

// AEABI init; this initialization is auto-injected byte pico_aeebi_double if present
#ifndef PICO_RUNTIME_INIT_AEABI_DOUBLE
#define PICO_RUNTIME_INIT_AEABI_DOUBLE          "00350"
#endif

// ------------------------
// Initialization of clocks
// ------------------------
// PICO_CONFIG: PICO_RUNTIME_SKIP_INIT_CLOCKS, Skip calling of `runtime_init_clocks` function during runtime init, type=bool, default=0, group=pico_runtime_init
// PICO_CONFIG: PICO_RUNTIME_NO_INIT_CLOCKS, Do not include SDK implementation of `runtime_init_clocks` function, type=bool, default=0, group=pico_runtime_init

#ifndef PICO_RUNTIME_INIT_CLOCKS
// on RP2040 we need some of the AEABI init by this point to do clock math
#define PICO_RUNTIME_INIT_CLOCKS                "00500"
#endif

#ifndef PICO_RUNTIME_SKIP_INIT_CLOCKS
#define PICO_RUNTIME_SKIP_INIT_CLOCKS 0
#endif

#ifndef PICO_RUNTIME_NO_INIT_CLOCKS
#define PICO_RUNTIME_NO_INIT_CLOCKS 0
#endif
#ifndef __ASSEMBLER__
void runtime_init_clocks(void);

/*! \brief Initialise the clock hardware
 *  \ingroup pico_runtime_init
 *
 *  Must be called before any other clock function.
 */
static inline void clocks_init(void) {
    // backwards compatibility with earlier SDK
    runtime_init_clocks();
}
#endif

// ----------------------------------------
// Remaining h/w initialization post clocks
// ----------------------------------------
#ifndef PICO_RUNTIME_INIT_POST_CLOCK_RESETS
#define PICO_RUNTIME_INIT_POST_CLOCK_RESETS     "00600"
#endif

// PICO_RUNTIME_INIT_POST_CLOCKS_RESETS defaults to 0
#ifndef __ASSEMBLER__
void runtime_init_post_clock_resets(void);
#endif

// ----------------------------------------
// RP2040 IE disable for GPIO 26-29
// ----------------------------------------

// PICO_CONFIG: PICO_RUNTIME_SKIP_INIT_RP2040_GPIO_IE_DISABLE, Skip calling of `runtime_init_rp2040_gpio_ie_disable` function during runtime init, type=bool, default=0 on RP2040, group=pico_runtime_init
// PICO_CONFIG: PICO_RUNTIME_NO_INIT_RP2040_GPIO_IE_DISABLE, Do not include SDK implementation of `runtime_init_rp2040_gpio_ie_disable` function, type=bool, default=0 on RP2040, group=pico_runtime_init

#ifndef PICO_RUNTIME_INIT_RP2040_GPIO_IE_DISABLE
#define PICO_RUNTIME_INIT_RP2040_GPIO_IE_DISABLE "00700"
#endif

#ifndef PICO_RUNTIME_SKIP_INIT_RP2040_GPIO_IE_DISABLE
#if !PICO_RP2040 || PICO_IE_26_29_UNCHANGED_ON_RESET
#define PICO_RUNTIME_SKIP_INIT_RP2040_GPIO_IE_DISABLE 1
#endif
#endif
#ifndef PICO_RUNTIME_NO_INIT_RP2040_GPIO_IE_DISABLE
#if !PICO_RP2040
#define PICO_RUNTIME_NO_INIT_RP2040_GPIO_IE_DISABLE 1
#endif
#endif

#ifndef __ASSEMBLER__
void runtime_init_rp2040_gpio_ie_disable(void);
#endif

// -----------------------------
// Reset all spin SIO spin locks
// -----------------------------

// PICO_CONFIG: PICO_RUNTIME_SKIP_INIT_SPIN_LOCKS_RESET, Skip calling of `runtime_init_spin_locks_reset` function during runtime init, type=bool, default=0, group=pico_runtime_init
// PICO_CONFIG: PICO_RUNTIME_NO_INIT_SPIN_LOCKS_RESET, Do not include SDK implementation of `runtime_init_spin_locks_reset` function, type=bool, default=0, group=pico_runtime_init

#ifndef PICO_RUNTIME_INIT_SPIN_LOCKS_RESET
// clearing of all spin locks
#define PICO_RUNTIME_INIT_SPIN_LOCKS_RESET      "01000"
#endif

#ifndef PICO_RUNTIME_SKIP_INIT_SPIN_LOCKS_RESET
#define PICO_RUNTIME_SKIP_INIT_SPIN_LOCKS_RESET 0
#endif

#ifndef PICO_RUNTIME_NO_INIT_SPIN_LOCKS_RESET
#define PICO_RUNTIME_NO_INIT_SPIN_LOCKS_RESET 0
#endif

#ifndef __ASSEMBLER__
void runtime_init_spin_locks_reset(void);
#endif

// -----------------------------
// Reset all bootram boot locks
// -----------------------------

// PICO_CONFIG: PICO_RUNTIME_SKIP_INIT_BOOT_LOCKS_RESET, Skip calling of `runtime_init_boot_locks_reset` function during runtime init, type=bool, default=0, group=pico_runtime_init

#ifndef PICO_RUNTIME_INIT_BOOT_LOCKS_RESET
// clearing of all spin locks
#define PICO_RUNTIME_INIT_BOOT_LOCKS_RESET      "01000"
#endif

#ifndef PICO_RUNTIME_SKIP_INIT_BOOT_LOCKS_RESET
#define PICO_RUNTIME_SKIP_INIT_BOOT_LOCKS_RESET 0
#endif
#ifndef __ASSEMBLER__
void runtime_init_boot_locks_reset(void);
#endif

// ------------------------------
// Enable bootrom locking support
// ------------------------------
// PICO_CONFIG: PICO_RUNTIME_SKIP_INIT_BOOTROM_LOCKING_ENABLE, Skip calling of `runtime_init_bootrom_locking_enable` function during runtime init, type=bool, default=0, group=pico_runtime_init
#ifndef PICO_RUNTIME_INIT_BOOTROM_LOCKING_ENABLE
// clearing of all spin locks
#define PICO_RUNTIME_INIT_BOOTROM_LOCKING_ENABLE  "01010"
#endif

#ifndef PICO_RUNTIME_SKIP_INIT_BOOTROM_LOCKING_ENABLE
#define PICO_RUNTIME_SKIP_INIT_BOOTROM_LOCKING_ENABLE 0
#endif

#ifndef __ASSEMBLER__
void runtime_init_bootrom_locking_enable(void);
#endif

// PICO_RUNTIME_INIT_MUTEX is registered automatically by pico_sync
// PICO_CONFIG: PICO_RUNTIME_SKIP_INIT_MUTEX, Skip calling of `runtime_init_mutex` function during runtime init, type=bool, default=0, group=pico_runtime_init
// PICO_CONFIG: PICO_RUNTIME_NO_INIT_MUTEX, Do not include SDK implementation of `runtime_init_mutex` function, type=bool, default=0, group=pico_runtime_init

#ifndef PICO_RUNTIME_INIT_MUTEX
// depends on SPIN_LOCKS
// initialize auto_init mutexes
#define PICO_RUNTIME_INIT_MUTEX                 "01100"
#endif

#ifndef PICO_RUNTIME_SKIP_INIT_MUTEX
#define PICO_RUNTIME_SKIP_INIT_MUTEX 0
#endif

#ifndef PICO_RUNTIME_NO_INIT_MUTEX
#define PICO_RUNTIME_NO_INIT_MUTEX 0
#endif

// ------------------------------------------------------------
// Initialization of IRQs, added by hardware_irq
// ------------------------------------------------------------

#ifndef PICO_RUNTIME_INIT_PER_CORE_IRQ_PRIORITIES
#define PICO_RUNTIME_INIT_PER_CORE_IRQ_PRIORITIES "01200"
#endif

// PICO_RUNTIME_SKIP_INIT_PER_CORE_TLS_SETUP defaults to 0
#ifndef PICO_RUNTIME_INIT_PER_CORE_TLS_SETUP
#define PICO_RUNTIME_INIT_PER_CORE_TLS_SETUP                 "10060"
#endif

// PICO_CONFIG: PICO_RUNTIME_SKIP_INIT_INSTALL_RAM_VECTOR_TABLE, Skip calling of `runtime_init_install_ram_vector_table` function during runtime init, type=bool, default=0 unless RISC-V or RAM binary, group=pico_runtime_init
// PICO_CONFIG: PICO_RUNTIME_NO_INIT_INSTALL_RAM_VECTOR_TABLE, Do not include SDK implementation of `runtime_init_install_ram_vector_table` function, type=bool, default=0 unless RISC-V or RAM binary, group=pico_runtime_init
#ifndef PICO_RUNTIME_INIT_INSTALL_RAM_VECTOR_TABLE
#define PICO_RUNTIME_INIT_INSTALL_RAM_VECTOR_TABLE "10080"
#endif

// ------------------------------------------------------
// Copy of ROM vector table to RAM; not used on RISC-V or
// no_flash which has a RAM vector table anyway
// ------------------------------------------------------

#ifndef PICO_RUNTIME_SKIP_INIT_INSTALL_RAM_VECTOR_TABLE
#if PICO_NO_RAM_VECTOR_TABLE || PICO_NO_FLASH || defined(__riscv)
#define PICO_RUNTIME_SKIP_INIT_INSTALL_RAM_VECTOR_TABLE 1
#endif
#endif

#ifndef PICO_RUNTIME_NO_INIT_INSTALL_RAM_VECTOR_TABLE
#if PICO_NO_RAM_VECTOR_TABLE || PICO_NO_FLASH || defined(__riscv)
#define PICO_RUNTIME_NO_INIT_INSTALL_RAM_VECTOR_TABLE 1
#endif
#endif

// ------------------------------------------------------------
// Default alarm pool initialization, added by pico_time unless
// PICO_TIME_DEFAULT_ALARM_POOL_DISABLED == 1
// ------------------------------------------------------------
#ifndef PICO_RUNTIME_INIT_DEFAULT_ALARM_POOL
#define PICO_RUNTIME_INIT_DEFAULT_ALARM_POOL    "11000"
#endif

// PICO_CONFIG: PICO_RUNTIME_SKIP_INIT_DEFAULT_ALARM_POOL, Skip calling of `runtime_init_default_alarm_pool` function during runtime init, type=bool, default=1 if `PICO_TIME_DEFAULT_ALARM_POOL_DISABLED` is 1, group=pico_runtime_init
// PICO_CONFIG: PICO_RUNTIME_NO_INIT_DEFAULT_ALARM_POOL, Do not include SDK implementation of `runtime_init_default_alarm_pool` function, type=bool, default=1 if `PICO_TIME_DEFAULT_ALARM_POOL_DISABLED` is , group=pico_runtime_init
#ifndef PICO_RUNTIME_SKIP_INIT_DEFAULT_ALARM_POOL
#if PICO_TIME_DEFAULT_ALARM_POOL_DISABLED
#define PICO_RUNTIME_SKIP_INIT_DEFAULT_ALARM_POOL 1
#endif
#endif

#ifndef PICO_RUNTIME_NO_INIT_DEFAULT_ALARM_POOL
#if PICO_TIME_DEFAULT_ALARM_POOL_DISABLED
#define PICO_RUNTIME_NO_INIT_DEFAULT_ALARM_POOL 1
#endif
#endif

// ------------------------------------------------------------------------------------------------
// stack guard; these are a special case as they take a parameter; however the normal defines apply
// ------------------------------------------------------------------------------------------------

// PICO_CONFIG: PICO_RUNTIME_SKIP_INIT_PER_CORE_INSTALL_STACK_GUARD, Skip calling of `runtime_init_per_core_install_stack_guard` function during runtime init, type=bool, default=1 unless `PICO_USE_STACK_GUARDS` is 1, group=pico_runtime_init
// PICO_CONFIG: PICO_RUNTIME_NO_INIT_PER_CORE_INSTALL_STACK_GUARD, Do not include SDK implementation of `runtime_init_per_core_install_stack_guard` function, type=bool, default=1 unless `PICO_USE_STACK_GUARDS` is 1, group=pico_runtime_init

#ifndef PICO_RUNTIME_INIT_PER_CORE_INSTALL_STACK_GUARD
#define PICO_RUNTIME_INIT_PER_CORE_INSTALL_STACK_GUARD     "10050"
#endif

#ifndef PICO_RUNTIME_SKIP_INIT_PER_CORE_INSTALL_STACK_GUARD
#if !PICO_USE_STACK_GUARDS
#define PICO_RUNTIME_SKIP_INIT_PER_CORE_INSTALL_STACK_GUARD 1
#endif
#endif

#ifndef PICO_RUNTIME_NO_INIT_PER_CORE_INSTALL_STACK_GUARD
#if !PICO_USE_STACK_GUARDS
#define PICO_RUNTIME_NO_INIT_PER_CORE_INSTALL_STACK_GUARD 1
#endif
#endif

#ifndef __ASSEMBLER__
void runtime_init_per_core_install_stack_guard(void *stack_bottom);
// backwards compatibility
static __force_inline void runtime_install_stack_guard(void *stack_bottom) {
    runtime_init_per_core_install_stack_guard(stack_bottom);
}

#endif

#ifdef __cplusplus
}
#endif

#endif