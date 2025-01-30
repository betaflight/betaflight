/*
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _HARDWARE_GPIO_COPROC_H
#define _HARDWARE_GPIO_COPROC_H

#ifdef __riscv
#error "GPIO coprocessor port is not available on RISC-V"
#endif

#if PICO_RP2040
#error "GPIO coprocessor is not available on RP2040"
#endif

#if !HAS_GPIO_COPROCESSOR
#error "GPIO coprocessor is not available"
#endif

#include "pico.h"

// ----------------------------------------------------------------------------
// OUT mask write instructions

// Equivalent to sio_hw->gpio_out = x;
__force_inline static void gpioc_lo_out_put(uint32_t x) {
    pico_default_asm_volatile ("mcr p0, #0, %0, c0, c0" : : "r" (x));
}

// Equivalent to sio_hw->gpio_togl = x;
__force_inline static void gpioc_lo_out_xor(uint32_t x) {
    pico_default_asm_volatile ("mcr p0, #1, %0, c0, c0" : : "r" (x));
}

// Equivalent to sio_hw->gpio_set = x;
__force_inline static void gpioc_lo_out_set(uint32_t x) {
    pico_default_asm_volatile ("mcr p0, #2, %0, c0, c0" : : "r" (x));
}

// Equivalent to sio_hw->gpio_clr = x;
__force_inline static void gpioc_lo_out_clr(uint32_t x) {
    pico_default_asm_volatile ("mcr p0, #3, %0, c0, c0" : : "r" (x));
}

// Equivalent to sio_hw->gpio_hi_out = x;
__force_inline static void gpioc_hi_out_put(uint32_t x) {
    pico_default_asm_volatile ("mcr p0, #0, %0, c0, c1" : : "r" (x));
}

// Equivalent to sio_hw->gpio_hi_togl = x;
__force_inline static void gpioc_hi_out_xor(uint32_t x) {
    pico_default_asm_volatile ("mcr p0, #1, %0, c0, c1" : : "r" (x));
}

// Equivalent to sio_hw->gpio_hi_set = x;
__force_inline static void gpioc_hi_out_set(uint32_t x) {
    pico_default_asm_volatile ("mcr p0, #2, %0, c0, c1" : : "r" (x));
}

// Equivalent to sio_hw->gpio_hi_clr = x;
__force_inline static void gpioc_hi_out_clr(uint32_t x) {
    pico_default_asm_volatile ("mcr p0, #3, %0, c0, c1" : : "r" (x));
}

// Equivalent to these two operations performed on the same cycle:
// - sio_hw->gpio_out    = x & 0xffffffff;
// - sio_hw->gpio_hi_out = x >> 32;
__force_inline static void gpioc_hilo_out_put(uint64_t x) {
    pico_default_asm_volatile ("mcrr p0, #0, %0, %1, c0" : : "r" (x & 0xffffffffu), "r" (x >> 32));
}

// Equivalent to these two operations performed on the same cycle:
// - sio_hw->gpio_togl    = x & 0xffffffff;
// - sio_hw->gpio_hi_togl = x >> 32;
__force_inline static void gpioc_hilo_out_xor(uint64_t x) {
    pico_default_asm_volatile ("mcrr p0, #1, %0, %1, c0" : : "r" (x & 0xffffffffu), "r" (x >> 32));
}

// Equivalent to these two operations performed on the same cycle:
// - sio_hw->gpio_set    = x & 0xffffffff;
// - sio_hw->gpio_hi_set = x >> 32;
__force_inline static void gpioc_hilo_out_set(uint64_t x) {
    pico_default_asm_volatile ("mcrr p0, #2, %0, %1, c0" : : "r" (x & 0xffffffffu), "r" (x >> 32));
}

// Equivalent to these two operations performed on the same cycle:
// - sio_hw->gpio_clr    = x & 0xffffffff;
// - sio_hw->gpio_hi_clr = x >> 32;
__force_inline static void gpioc_hilo_out_clr(uint64_t x) {
    pico_default_asm_volatile ("mcrr p0, #3, %0, %1, c0" : : "r" (x & 0xffffffffu), "r" (x >> 32));
}

// ----------------------------------------------------------------------------
// OE mask write instructions

// Equivalent to sio_hw->gpio_oe = x;
__force_inline static void gpioc_lo_oe_put(uint32_t x) {
    pico_default_asm_volatile ("mcr p0, #0, %0, c0, c4" : : "r" (x));
}

// Equivalent to sio_hw->gpio_oe_togl = x;
__force_inline static void gpioc_lo_oe_xor(uint32_t x) {
    pico_default_asm_volatile ("mcr p0, #1, %0, c0, c4" : : "r" (x));
}

// Equivalent to sio_hw->gpio_oe_set = x;
__force_inline static void gpioc_lo_oe_set(uint32_t x) {
    pico_default_asm_volatile ("mcr p0, #2, %0, c0, c4" : : "r" (x));
}

// Equivalent to sio_hw->gpio_oe_clr = x;
__force_inline static void gpioc_lo_oe_clr(uint32_t x) {
    pico_default_asm_volatile ("mcr p0, #3, %0, c0, c4" : : "r" (x));
}

// Equivalent to sio_hw->gpio_hi_oe = x;
__force_inline static void gpioc_hi_oe_put(uint32_t x) {
    pico_default_asm_volatile ("mcr p0, #0, %0, c0, c5" : : "r" (x));
}

// Equivalent to sio_hw->gpio_hi_oe_togl = x;
__force_inline static void gpioc_hi_oe_xor(uint32_t x) {
    pico_default_asm_volatile ("mcr p0, #1, %0, c0, c5" : : "r" (x));
}

// Equivalent to sio_hw->gpio_hi_oe_set = x;
__force_inline static void gpioc_hi_oe_set(uint32_t x) {
    pico_default_asm_volatile ("mcr p0, #2, %0, c0, c5" : : "r" (x));
}

// Equivalent to sio_hw->gpio_hi_oe_clr = x;
__force_inline static void gpioc_hi_oe_clr(uint32_t x) {
    pico_default_asm_volatile ("mcr p0, #3, %0, c0, c5" : : "r" (x));
}

// Equivalent to these two operations performed on the same cycle:
// - sio_hw->gpio_oe    = x & 0xffffffff;
// - sio_hw->gpio_hi_oe = x >> 32;
__force_inline static void gpioc_hilo_oe_put(uint64_t x) {
    pico_default_asm_volatile ("mcrr p0, #0, %0, %1, c4" : : "r" (x & 0xffffffffu), "r" (x >> 32));
}

// Equivalent to these two operations performed on the same cycle:
// - sio_hw->gpio_oe_togl    = x & 0xffffffff;
// - sio_hw->gpio_hi_oe_togl = x >> 32;
__force_inline static void gpioc_hilo_oe_xor(uint64_t x) {
    pico_default_asm_volatile ("mcrr p0, #1, %0, %1, c4" : : "r" (x & 0xffffffffu), "r" (x >> 32));
}

// Equivalent to these two operations performed on the same cycle:
// - sio_hw->gpio_oe_set    = x & 0xffffffff;
// - sio_hw->gpio_hi_oe_set = x >> 32;
__force_inline static void gpioc_hilo_oe_set(uint64_t x) {
    pico_default_asm_volatile ("mcrr p0, #2, %0, %1, c4" : : "r" (x & 0xffffffffu), "r" (x >> 32));
}

// Equivalent to these two operations performed on the same cycle:
// - sio_hw->gpio_oe_clr    = x & 0xffffffff;
// - sio_hw->gpio_hi_oe_clr = x >> 32;
__force_inline static void gpioc_hilo_oe_clr(uint64_t x) {
    pico_default_asm_volatile ("mcrr p0, #3, %0, %1, c4" : : "r" (x & 0xffffffffu), "r" (x >> 32));
}

// ----------------------------------------------------------------------------
// Single-bit write instructions

// Write a 1-bit value to any output. Equivalent to:
//
//     if (val)
//         gpioc_hilo_out_set(1ull << pin);
//     else
//         gpioc_hilo_out_clr(1ull << pin);
__force_inline static void gpioc_bit_out_put(uint pin, bool val) {
    pico_default_asm_volatile ("mcrr p0, #4, %0, %1, c0" : : "r" (pin), "r" (val));
}

// Unconditionally toggle any single output. Equivalent to:
//
//     gpioc_hilo_out_xor(1ull << pin);
__force_inline static void gpioc_bit_out_xor(uint pin) {
    pico_default_asm_volatile ("mcr p0, #5, %0, c0, c0" : : "r" (pin));
}

// Unconditionally set any single output. Equivalent to:
//
//     gpioc_hilo_out_set(1ull << pin);
__force_inline static void gpioc_bit_out_set(uint pin) {
    pico_default_asm_volatile ("mcr p0, #6, %0, c0, c0" : : "r" (pin));
}

// Unconditionally clear any single output. Equivalent to:
//
//     gpioc_hilo_out_clr(1ull << pin);
__force_inline static void gpioc_bit_out_clr(uint pin) {
    pico_default_asm_volatile ("mcr p0, #7, %0, c0, c0" : : "r" (pin));
}

// Conditionally toggle any single output. Equivalent to:
//
//     gpioc_hilo_out_xor((uint64_t)val << pin);
__force_inline static void gpioc_bit_out_xor2(uint pin, bool val) {
    pico_default_asm_volatile ("mcrr p0, #5, %0, %1, c0" : : "r" (pin), "r" (val));
}

// Conditionally set any single output. Equivalent to:
//
//     gpioc_hilo_out_set((uint64_t)val << pin);
__force_inline static void gpioc_bit_out_set2(uint pin, bool val) {
    pico_default_asm_volatile ("mcrr p0, #6, %0, %1, c0" : : "r" (pin), "r" (val));
}

// Conditionally clear any single output. Equivalent to:
//
//     gpioc_hilo_out_clr((uint64_t)val << pin);
__force_inline static void gpioc_bit_out_clr2(uint pin, bool val) {
    pico_default_asm_volatile ("mcrr p0, #7, %0, %1, c0" : : "r" (pin), "r" (val));
}

// Write a 1-bit value to any output enable. Equivalent to:
//
//     if (val)
//         gpioc_hilo_oe_set(1ull << pin);
//     else
//         gpioc_hilo_oe_clr(1ull << pin);
__force_inline static void gpioc_bit_oe_put(uint pin, bool val) {
    pico_default_asm_volatile ("mcrr p0, #4, %0, %1, c4" : : "r" (pin), "r" (val));
}

// Unconditionally toggle any output enable. Equivalent to:
//
//     gpioc_hilo_oe_xor(1ull << pin);
__force_inline static void gpioc_bit_oe_xor(uint pin) {
    pico_default_asm_volatile ("mcr p0, #5, %0, c0, c4" : : "r" (pin));
}

// Unconditionally set any output enable (set to output). Equivalent to:
//
//     gpioc_hilo_oe_set(1ull << pin);
__force_inline static void gpioc_bit_oe_set(uint pin) {
    pico_default_asm_volatile ("mcr p0, #6, %0, c0, c4" : : "r" (pin));
}

// Unconditionally clear any output enable (set to input). Equivalent to:
//
//     gpioc_hilo_oe_clr(1ull << pin);
__force_inline static void gpioc_bit_oe_clr(uint pin) {
    pico_default_asm_volatile ("mcr p0, #7, %0, c0, c4" : : "r" (pin));
}

// Conditionally toggle any output enable. Equivalent to:
//
//     gpioc_hilo_oe_xor((uint64_t)val << pin);
__force_inline static void gpioc_bit_oe_xor2(uint pin, bool val) {
    pico_default_asm_volatile ("mcrr p0, #5, %0, %1, c4" : : "r" (pin), "r" (val));
}

// Conditionally set any output enable (set to output). Equivalent to:
//
//     gpioc_hilo_oe_set((uint64_t)val << pin);
__force_inline static void gpioc_bit_oe_set2(uint pin, bool val) {
    pico_default_asm_volatile ("mcrr p0, #6, %0, %1, c4" : : "r" (pin), "r" (val));
}

// Conditionally clear any output enable (set to input). Equivalent to:
//
//     gpioc_hilo_oe_clr((uint64_t)val << pin);
__force_inline static void gpioc_bit_oe_clr2(uint pin, bool val) {
    pico_default_asm_volatile ("mcrr p0, #7, %0, %1, c4" : : "r" (pin), "r" (val));
}

// ----------------------------------------------------------------------------
// Indexed mask write instructions -- write to a dynamically selected 32-bit
// GPIO register

// Write to a selected GPIO output register. Equivalent to:
//
//     if (reg_index == 0) {
//         gpioc_lo_out_put(val);
//     } else if (reg_index == 1) {
//         gpioc_hi_out_put(val);
//     } else {
//         // undefined
//     }
__force_inline static void gpioc_index_out_put(uint reg_index, uint32_t val) {
    pico_default_asm_volatile ("mcrr p0, #8, %1, %0, c0" : : "r" (reg_index), "r" (val));
}

// Toggle bits in a selected GPIO output register. Equivalent to:
//
//     if (reg_index == 0) {
//         gpioc_lo_out_xor(val);
//     } else if (reg_index == 1) {
//         gpioc_hi_out_xor(val);
//     } else {
//         // undefined
//     }
__force_inline static void gpioc_index_out_xor(uint reg_index, uint32_t mask) {
    pico_default_asm_volatile ("mcrr p0, #9, %1, %0, c0" : : "r" (reg_index), "r" (mask));
}

// Set bits in a selected GPIO output register. Equivalent to:
//
//     if (reg_index == 0) {
//         gpioc_lo_out_set(val);
//     } else if (reg_index == 1) {
//         gpioc_hi_out_set(val);
//     } else {
//         // undefined
//     }
__force_inline static void gpioc_index_out_set(uint reg_index, uint32_t mask) {
    pico_default_asm_volatile ("mcrr p0, #10, %1, %0, c0" : : "r" (reg_index), "r" (mask));
}

// Clear bits in a selected GPIO output register. Equivalent to:
//
//     if (reg_index == 0) {
//         gpioc_lo_out_clr(val);
//     } else if (reg_index == 1) {
//         gpioc_hi_out_clr(val);
//     } else {
//         // undefined
//     }
__force_inline static void gpioc_index_out_clr(uint reg_index, uint32_t mask) {
    pico_default_asm_volatile ("mcrr p0, #11, %1, %0, c0" : : "r" (reg_index), "r" (mask));
}

// Write to a selected GPIO output enable register. Equivalent to:
//
//     if (reg_index == 0) {
//         gpioc_lo_oe_put(val);
//     } else if (reg_index == 1) {
//         gpioc_hi_oe_put(val);
//     } else {
//         // undefined
//     }
__force_inline static void gpioc_index_oe_put(uint reg_index, uint32_t val) {
    pico_default_asm_volatile ("mcrr p0, #8, %1, %0, c4" : : "r" (reg_index), "r" (val));
}

// Toggle bits in a selected GPIO output enable register. Equivalent to:
//
//     if (reg_index == 0) {
//         gpioc_lo_oe_xor(val);
//     } else if (reg_index == 1) {
//         gpioc_hi_oe_xor(val);
//     } else {
//         // undefined
//     }
__force_inline static void gpioc_index_oe_xor(uint reg_index, uint32_t mask) {
    pico_default_asm_volatile ("mcrr p0, #9, %1, %0, c4" : : "r" (reg_index), "r" (mask));
}

// Set bits in a selected GPIO output enable register (set to output). Equivalent to:
//
//     if (reg_index == 0) {
//         gpioc_lo_oe_set(val);
//     } else if (reg_index == 1) {
//         gpioc_hi_oe_set(val);
//     } else {
//         // undefined
//     }
__force_inline static void gpioc_index_oe_set(uint reg_index, uint32_t mask) {
    pico_default_asm_volatile ("mcrr p0, #10, %1, %0, c4" : : "r" (reg_index), "r" (mask));
}

// Clear bits in a selected GPIO output enable register (set to input). Equivalent to:
//
//     if (reg_index == 0) {
//         gpioc_lo_oe_clr(val);
//     } else if (reg_index == 1) {
//         gpioc_hi_oe_clr(val);
//     } else {
//         // undefined
//     }
__force_inline static void gpioc_index_oe_clr(uint reg_index, uint32_t mask) {
    pico_default_asm_volatile ("mcrr p0, #11, %1, %0, c4" : : "r" (reg_index), "r" (mask));
}

// ----------------------------------------------------------------------------
// Read instructions

// Read back the lower 32-bit output register. Equivalent to:
//
//     return sio_hw->gpio_out;
__force_inline static uint32_t gpioc_lo_out_get(void) {
    uint32_t lo;
    pico_default_asm_volatile ("mrc p0, #0, %0, c0, c0" : "=r" (lo));
    return lo;
}

// Read back the upper 32-bit output register. Equivalent to:
//
//     return sio_hw->gpio_hi_out;
__force_inline static uint32_t gpioc_hi_out_get(void) {
    uint32_t hi;
    pico_default_asm_volatile ("mrc p0, #0, %0, c0, c1" : "=r" (hi));
    return hi;
}

// Read back two 32-bit output registers in a single operation. Equivalent to:
//
//     return sio_hw->gpio_out | ((uint64_t)sio_hw->gpio_hi_out << 32);
__force_inline static uint64_t gpioc_hilo_out_get(void) {
    uint32_t hi, lo;
    pico_default_asm_volatile ("mrrc p0, #0, %0, %1, c0" : "=r" (lo), "=r" (hi));
    return ((uint64_t)hi << 32) | lo;
}

// Read back the lower 32-bit output enable register. Equivalent to:
//
//     return sio_hw->gpio_oe;
__force_inline static uint32_t gpioc_lo_oe_get(void) {
    uint32_t lo;
    pico_default_asm_volatile ("mrc p0, #0, %0, c0, c4" : "=r" (lo));
    return lo;
}

// Read back the upper 32-bit output enable register. Equivalent to:
//
//     return sio_hw->gpio_hi_oe;
__force_inline static uint32_t gpioc_hi_oe_get(void) {
    uint32_t hi;
    pico_default_asm_volatile ("mrc p0, #0, %0, c0, c5" : "=r" (hi));
    return hi;
}

// Read back two 32-bit output enable registers in a single operation. Equivalent to:
//
//     return sio_hw->gpio_oe | ((uint64_t)sio_hw->gpio_hi_oe << 32);
__force_inline static uint64_t gpioc_hilo_oe_get(void) {
    uint32_t hi, lo;
    pico_default_asm_volatile ("mrrc p0, #0, %0, %1, c4" : "=r" (lo), "=r" (hi));
    return ((uint64_t)hi << 32) | lo;
}

// Sample the lower 32 GPIOs. Equivalent to:
//
//     return sio_hw->gpio_in;
__force_inline static uint32_t gpioc_lo_in_get(void) {
    uint32_t lo;
    pico_default_asm_volatile ("mrc p0, #0, %0, c0, c8" : "=r" (lo));
    return lo;
}

// Sample the upper 32 GPIOs. Equivalent to:
//
//     return sio_hw->gpio_hi_in;
__force_inline static uint32_t gpioc_hi_in_get(void) {
    uint32_t hi;
    pico_default_asm_volatile ("mrc p0, #0, %0, c0, c9" : "=r" (hi));
    return hi;
}

// Sample 64 GPIOs on the same cycle. Equivalent to:
//
//     return sio_hw->gpio_in | ((uint64_t)sio_hw->gpio_hi_in << 32);
__force_inline static uint64_t gpioc_hilo_in_get(void) {
    uint32_t hi, lo;
    pico_default_asm_volatile ("mrrc p0, #0, %0, %1, c8" : "=r" (lo), "=r" (hi));
    return ((uint64_t)hi << 32) | lo;
}

#endif
