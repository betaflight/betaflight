/*
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "pico/runtime_init.h"

// This file is sorted in the order of initialization

// -------------------------------------
// 00050 PICO_RUNTIME_INIT_BOOTROM_RESET
// -------------------------------------
#if !PICO_RUNTIME_NO_INIT_BOOTROM_RESET
#include "pico/bootrom.h"
void __weak runtime_init_bootrom_reset(void) {
    // todo can we tell if we came in thru the bootrom where this is not necessary (this is necessary for debugger)
    rom_bootrom_state_reset_fn state_reset = rom_func_lookup(ROM_FUNC_BOOTROM_STATE_RESET);
    state_reset(BOOTROM_STATE_RESET_GLOBAL_STATE);
}
#endif

#if !PICO_RUNTIME_SKIP_INIT_BOOTROM_RESET
PICO_RUNTIME_INIT_FUNC_RUNTIME(runtime_init_bootrom_reset, PICO_RUNTIME_INIT_BOOTROM_RESET);
#endif

// ----------------------------------------------
// 00051 PICO_RUNTIME_INIT_PER_CORE_BOOTROM_RESET
// ----------------------------------------------
#if !PICO_RUNTIME_NO_INIT_PER_CORE_BOOTROM_RESET
#include "pico/bootrom.h"
void __weak runtime_init_per_core_bootrom_reset(void) {
    // todo can we tell if we came in thru the bootrom where this is not necessary (this is necessary for debugger)
    rom_bootrom_state_reset_fn state_reset = rom_func_lookup(ROM_FUNC_BOOTROM_STATE_RESET);
    state_reset(BOOTROM_STATE_RESET_CURRENT_CORE);
}
#endif

#if !PICO_RUNTIME_SKIP_INIT_PER_CORE_BOOTROM_RESET
PICO_RUNTIME_INIT_FUNC_PER_CORE(runtime_init_per_core_bootrom_reset, PICO_RUNTIME_INIT_PER_CORE_BOOTROM_RESET);
#endif

// ------------------------------------
// 00060 PICO_RUNTIME_INIT_H3_IRQ_REGISTERS
// ------------------------------------
#if !PICO_RUNTIME_SKIP_INIT_PER_CORE_H3_IRQ_REGISTERS
extern void runtime_init_per_core_h3_irq_registers(void);
PICO_RUNTIME_INIT_FUNC_PER_CORE(runtime_init_per_core_h3_irq_registers, PICO_RUNTIME_INIT_PER_CORE_H3_IRQ_REGISTERS);
#endif

// ------------------------------------
// 00100 PICO_RUNTIME_INIT_EARLY_RESETS
// ------------------------------------
#if !PICO_RUNTIME_NO_INIT_EARLY_RESETS
#include "hardware/resets.h"
void __weak runtime_init_early_resets(void) {
    static_assert(NUM_RESETS <= 32, "");
    // Reset all peripherals to put system into a known state,
    // - except for QSPI pads and the XIP IO bank, as this is fatal if running from flash
    // - and the PLLs, as this is fatal if clock muxing has not been reset on this boot
    // - and USB, syscfg, as this disturbs USB-to-SWD on core 1
    reset_block_mask(~(
            (1u << RESET_IO_QSPI) |
            (1u << RESET_PADS_QSPI) |
            (1u << RESET_PLL_USB) |
            (1u << RESET_USBCTRL) |
            (1u << RESET_SYSCFG) |
            (1u << RESET_PLL_SYS)
    ));

    // Remove reset from peripherals which are clocked only by clk_sys and
    // clk_ref. Other peripherals stay in reset until we've configured clocks.
    unreset_block_mask_wait_blocking(RESETS_RESET_BITS & ~(
#if !PICO_RP2040
            (1u << RESET_HSTX) |
            #endif
            (1u << RESET_ADC) |
            #if PICO_RP2040
            (1u << RESET_RTC) |
            #endif
            (1u << RESET_SPI0) |
            (1u << RESET_SPI1) |
            (1u << RESET_UART0) |
            (1u << RESET_UART1) |
            (1u << RESET_USBCTRL)
    ));

}
#endif

#if !PICO_RUNTIME_SKIP_INIT_EARLY_RESETS
PICO_RUNTIME_INIT_FUNC_HW(runtime_init_early_resets, PICO_RUNTIME_INIT_EARLY_RESETS);
#endif

#if !PICO_RUNTIME_NO_INIT_USB_POWER_DOWN
#include "hardware/structs/usb.h"
void __weak runtime_init_usb_power_down(void) {
    // Ensure USB PHY is in low-power state -- must be cleared before beginning USB operations. Only
    // do this if USB appears to be in the reset state, to avoid breaking core1-as-debugger.
    if (usb_hw->sie_ctrl == USB_SIE_CTRL_RESET) {
        hw_set_bits(&usb_hw->sie_ctrl, USB_SIE_CTRL_TRANSCEIVER_PD_BITS);
    }
}
#endif

#if !PICO_RUNTIME_SKIP_INIT_USB_POWER_DOWN
PICO_RUNTIME_INIT_FUNC_HW(runtime_init_usb_power_down, PICO_RUNTIME_INIT_USB_POWER_DOWN);
#endif

#if !PICO_RUNTIME_NO_INIT_PER_CORE_ENABLE_COPROCESSORS
#include "hardware/gpio.h" // PICO_USE_GPIO_COPROCESSOR is defined here
#include "hardware/structs/m33.h"
// ----------------------------------------------------
// 00200 PICO_RUNTIME_INIT_PER_CORE_ENABLE_COPROCESSORS
// ----------------------------------------------------
void __weak runtime_init_per_core_enable_coprocessors(void) {
    // VFP copro (float)
    uint32_t cpacr = M33_CPACR_CP10_BITS;
#if HAS_DOUBLE_COPROCESSOR
    cpacr |= M33_CPACR_CP4_BITS;
#endif
#if PICO_USE_GPIO_COPROCESSOR
    cpacr |= M33_CPACR_CP0_BITS;
#endif
    arm_cpu_hw->cpacr |= cpacr;
#if HAS_DOUBLE_COPROCESSOR
    asm volatile ("mrc p4,#0,r0,c0,c0,#1" : : : "r0"); // clear engaged flag via RCMP
#endif
}
#endif

#if !PICO_RUNTIME_SKIP_INIT_PER_CORE_ENABLE_COPROCESSORS
PICO_RUNTIME_INIT_FUNC_PER_CORE(runtime_init_per_core_enable_coprocessors, PICO_RUNTIME_INIT_PER_CORE_ENABLE_COPROCESSORS);
#endif

// ----------------------------------------------------
// 00500 PICO_RUNTIME_INIT_CLOCKS
// ----------------------------------------------------
#if !PICO_RUNTIME_SKIP_INIT_CLOCKS
PICO_RUNTIME_INIT_FUNC_HW(runtime_init_clocks, PICO_RUNTIME_INIT_CLOCKS);
#endif

// ----------------------------------------------------
// 00600 PICO_RUNTIME_INIT_POST_CLOCK_RESETS
// ----------------------------------------------------
#if !PICO_RUNTIME_NO_INIT_POST_CLOCK_RESETS
#include "hardware/resets.h"
void __weak runtime_init_post_clock_resets(void) {
    // Peripheral clocks should now all be running
    static_assert(NUM_RESETS <= 32, "");
    unreset_block_mask_wait_blocking(RESETS_RESET_BITS);
}
#endif

#if !PICO_RUNTIME_SKIP_POST_CLOCK_RESETS
PICO_RUNTIME_INIT_FUNC_HW(runtime_init_post_clock_resets, PICO_RUNTIME_INIT_POST_CLOCK_RESETS);
#endif

// ----------------------------------------------------
// 00700 PICO_RUNTIME_INIT_RP2040_GPIO_IE_DISABLE
// ----------------------------------------------------

#if !PICO_RUNTIME_NO_INIT_RP2040_GPIO_IE_DISABLE
#include "hardware/structs/pads_bank0.h"
void __weak runtime_init_rp2040_gpio_ie_disable(void) {
#if PICO_RP2040 && !PICO_IE_26_29_UNCHANGED_ON_RESET
    // after resetting BANK0 we should disable IE on 26-29 as these may have mid-rail voltages when
    // ADC is in use (on RP2040 B2 and later, and non-RP2040 chips, ADC pins should already have
    // the correct reset state):
    pads_bank0_hw_t *pads_bank0_hw_clear = (pads_bank0_hw_t *)hw_clear_alias_untyped(pads_bank0_hw);
    pads_bank0_hw_clear->io[26] = pads_bank0_hw_clear->io[27] =
            pads_bank0_hw_clear->io[28] = pads_bank0_hw_clear->io[29] = PADS_BANK0_GPIO0_IE_BITS;
#endif
}
#endif

#if !PICO_RUNTIME_SKIP_INIT_RP2040_GPIO_IE_DISABLE
PICO_RUNTIME_INIT_FUNC_HW(runtime_init_rp2040_gpio_ie_disable, PICO_RUNTIME_INIT_RP2040_GPIO_IE_DISABLE);
#endif

#if !PICO_RUNTIME_NO_INIT_SPIN_LOCKS_RESET
#include "hardware/sync.h"
void __weak runtime_init_spin_locks_reset(void) {
    spin_locks_reset();
}
#endif

#if !PICO_RUNTIME_SKIP_INIT_SPIN_LOCKS_RESET
PICO_RUNTIME_INIT_FUNC_RUNTIME(runtime_init_spin_locks_reset, PICO_RUNTIME_INIT_SPIN_LOCKS_RESET);
#endif

// On RISC-V the vector table is not relocatable since it contains PC-relative
// jump instructions, so rather than copying it into a RAM-resident array, we
// just link it in an initialised RAM section. Note there is no requirement on
// RISC-V to have an initial flash-resident vector table at a well-known
// location, unlike Cortex-M which can take an NMI on cycle 0.
#ifndef __riscv

#if !PICO_RUNTIME_NO_INIT_INSTALL_RAM_VECTOR_TABLE
uint32_t __attribute__((section(".ram_vector_table"))) ram_vector_table[PICO_RAM_VECTOR_TABLE_SIZE];

#include "hardware/structs/scb.h"
void runtime_init_install_ram_vector_table(void) {
    // Note on RISC-V the RAM vector table is initialised during crt0
#if !(PICO_NO_RAM_VECTOR_TABLE || PICO_NO_FLASH) && !defined(__riscv)
#if !PICO_NO_STORED_VECTOR_TABLE
    __builtin_memcpy(ram_vector_table, (uint32_t *) scb_hw->vtor, sizeof(ram_vector_table));
#else
    __builtin_memcpy(ram_vector_table, (uint32_t *) scb_hw->vtor, MIN(VTABLE_FIRST_IRQ, sizeof(ram_vector_table)));
    for(uint i = VTABLE_FIRST_IRQ; i<count_of(ram_vector_table); i++) {
        ram_vector_table[i] = (uintptr_t)__unhandled_user_irq;
    }
#endif

    scb_hw->vtor = (uintptr_t) ram_vector_table;
#endif
}
#endif
#endif

#if !PICO_RUNTIME_SKIP_INIT_INSTALL_RAM_VECTOR_TABLE
// todo this wants to be per core if we decide to support per core vector tables
PICO_RUNTIME_INIT_FUNC_RUNTIME(runtime_init_install_ram_vector_table, PICO_RUNTIME_INIT_INSTALL_RAM_VECTOR_TABLE);
#endif
