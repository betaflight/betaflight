/*
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _PICO_RCP_H
#define _PICO_RCP_H

#include "pico.h"

/** \file hardware/rcp.h
 * \defgroup hardware_rcp hardware_rcp
 * \brief Inline functions and assembly macros for the Redundancy Coprocessor
 */

// ----------------------------------------------------------------------------
// RCP instructions (this header is Arm-only)
#if defined(PICO_RP2350) && !defined(__riscv)

#define RCP_MASK_TRUE   _u(0xa500a500)
#define RCP_MASK_FALSE  _u(0x00c300c3)
#define RCP_MASK_INTXOR _u(0x96009600)

// ----------------------------------------------------------------------------
// Macros and inline functions for use in C files
#ifndef __ASSEMBLER__

#define __rcpinline __force_inline

#ifdef __cplusplus
extern "C" {
#endif

#ifndef __riscv // should never do an rcp_asm in risc-v code
#if __ARM_ARCH_8M_MAIN__
#define rcp_asm pico_default_asm_volatile
#elif __ARM_ARCH_8M_BASE__
#define rcp_asm(...) ({pico_default_asm_volatile(".cpu cortex-m33\n" __VA_ARGS__); pico_default_asm_volatile(".cpu cortex-m23"); })
#elif __ARM_ARCH_6M__
#define rcp_asm(...) ({pico_default_asm_volatile(".cpu cortex-m33\n" __VA_ARGS__); pico_default_asm_volatile(".cpu cortex-m0plus"); })
#else
#error unknown ARM architecture
#endif

// Each macro has a default version (e.g. rcp_salt_core0) and a no-delay version
// (e.g. rcp_salt_core0_nodelay). The default version adds an up to 127-cycle
// pseudorandom delay on each coprocessor instruction, whereas the no-delay version
// does not have this delay. The difference in the generated code is whether an MCR
// (e.g.) or MCR2 opcode is emitted.

// ----------------------------------------------------------------------------
// RCP Canary instructions

// Initialise 64-bit salt value for core 0
static __rcpinline void rcp_salt_core0(uint64_t salt) {
    rcp_asm (
            "mcrr p7, #8, %0, %1, c0\n"
            : : "r" (salt & 0xffffffffu), "r" (salt >> 32)
            );
}

static __rcpinline void rcp_salt_core0_nodelay(uint64_t salt) {
    rcp_asm (
            "mcrr2 p7, #8, %0, %1, c0\n"
            : : "r" (salt & 0xffffffffu), "r" (salt >> 32)
            );
}

// Initialise 64-bit salt value for core 1
static __rcpinline void rcp_salt_core1(uint64_t salt) {
    rcp_asm (
            "mcrr p7, #8, %0, %1, c1\n"
            : : "r" (salt & 0xffffffffu), "r" (salt >> 32)
            );
}

static __rcpinline void rcp_salt_core1_nodelay(uint64_t salt) {
    rcp_asm (
            "mcrr2 p7, #8, %0, %1, c1\n"
            : : "r" (salt & 0xffffffffu), "r" (salt >> 32)
            );
}

// Get a 32-bit canary value. `tag` must be a constant expression.
#define rcp_canary_get(tag) ({ \
    uint32_t __canary_u32; \
    rcp_asm ( \
        "mrc p7, #0, %0, c%c1, c%c2, #1\n" \
        : "=r" (__canary_u32) \
        : "i" ((tag >> 4) & 0xf), "i" (tag & 0xf) \
    ); \
    __canary_u32; \
})

#define rcp_canary_get_nodelay(tag) ({ \
    uint32_t __canary_u32; \
    rcp_asm ( \
        "mrc2 p7, #0, %0, c%c1, c%c2, #1\n" \
        : "=r" (__canary_u32) \
        : "i" (((tag) >> 4) & 0xf), "i" ((tag) & 0xf) \
    ); \
    __canary_u32; \
})

// Assert that canary matches result of rcp_canary_get with the same tags:
#define rcp_canary_check(tag, canary) ({ \
    rcp_asm ( \
        "mcr p7, #0, %0, c%c1, c%c2, #1\n" \
        : : "r" (canary), \
        "i" (((tag) >> 4) & 0xf), "i" ((tag) & 0xf) \
    ); \
})

#define rcp_canary_check_nodelay(tag, canary) ({ \
    rcp_asm ( \
        "mcr2 p7, #0, %0, c%c1, c%c2, #1\n" \
        : : "r" (canary), \
        "i" (((tag) >> 4) & 0xf), "i" ((tag) & 0xf) \
    ); \
})

// Return true/false bit pattern for whether the salt for this core has been
// initialised. (Invoking with Rt=0xf will set the Arm N flag if initialised.)
// If the salt has not been initialised then any operation other than
// initialising the salt or checking the canary status is a hard error.
static __rcpinline uint32_t rcp_canary_status(void) {
    uint32_t ret;
    rcp_asm ("mrc p7, #1, %0, c0, c0, #0\n" : "=r" (ret));
    return ret;
}

static __rcpinline uint32_t rcp_canary_status_nodelay(void) {
    uint32_t ret;
    rcp_asm ("mrc2 p7, #1, %0, c0, c0, #0\n" : "=r" (ret));
    return ret;
}

// ----------------------------------------------------------------------------
// RCP Boolean instructions

// Assert b is a valid boolean (0xa500a500u or 0x00c300c3u)
static __rcpinline void rcp_bvalid(uint32_t b) {
    rcp_asm ("mcr p7, #1, %0, c0, c0, #0\n" : : "r" (b));
}

static __rcpinline void rcp_bvalid_nodelay(uint32_t b) {
    rcp_asm ("mcr2 p7, #1, %0, c0, c0, #0\n" : : "r" (b));
}

// Assert b is true (0xa500a500u)
static __rcpinline void rcp_btrue(uint32_t b) {
    rcp_asm ("mcr p7, #2, %0, c0, c0, #0\n" : : "r" (b));
}

static __rcpinline void rcp_btrue_nodelay(uint32_t b) {
    rcp_asm ("mcr2 p7, #2, %0, c0, c0, #0\n" : : "r" (b));
}

// Assert b is false (0x00c300c3u)
static __rcpinline void rcp_bfalse(uint32_t b) {
    rcp_asm ("mcr p7, #3, %0, c0, c0, #1\n" : : "r" (b));
}

static __rcpinline void rcp_bfalse_nodelay(uint32_t b) {
    rcp_asm ("mcr2 p7, #3, %0, c0, c0, #1\n" : : "r" (b));
}

// Assert b0 and b1 are both valid booleans
static __rcpinline void rcp_b2valid(uint32_t b0, uint32_t b1) {
    rcp_asm ("mcrr p7, #0, %0, %1, c8\n" : : "r" (b0), "r" (b1));
}

static __rcpinline void rcp_b2valid_nodelay(uint32_t b0, uint32_t b1) {
    rcp_asm ("mcrr2 p7, #0, %0, %1, c8\n" : : "r" (b0), "r" (b1));
}

// Assert b0 and b1 are both true
static __rcpinline void rcp_b2and(uint32_t b0, uint32_t b1) {
    rcp_asm ("mcrr p7, #1, %0, %1, c0\n" : : "r" (b0), "r" (b1));
}

static __rcpinline void rcp_b2and_nodelay(uint32_t b0, uint32_t b1) {
    rcp_asm ("mcrr2 p7, #1, %0, %1, c0\n" : : "r" (b0), "r" (b1));
}

// Assert b0 and b1 are valid, and at least one is true
static __rcpinline void rcp_b2or(uint32_t b0, uint32_t b1) {
    rcp_asm ("mcrr p7, #2, %0, %1, c0\n" : : "r" (b0), "r" (b1));
}

static __rcpinline void rcp_b2or_nodelay(uint32_t b0, uint32_t b1) {
    rcp_asm ("mcrr2 p7, #2, %0, %1, c0\n" : : "r" (b0), "r" (b1));
}

// Assert (b ^ mask) is a valid boolean
static __rcpinline void rcp_bxorvalid(uint32_t b, uint32_t mask) {
    rcp_asm ("mcrr p7, #3, %0, %1, c8\n" : : "r" (b), "r" (mask));
}

static __rcpinline void rcp_bxorvalid_nodelay(uint32_t b, uint32_t mask) {
    rcp_asm ("mcrr2 p7, #3, %0, %1, c8\n" : : "r" (b), "r" (mask));
}

// Assert (b ^ mask) is true
static __rcpinline void rcp_bxortrue(uint32_t b, uint32_t mask) {
    rcp_asm ("mcrr p7, #4, %0, %1, c0\n" : : "r" (b), "r" (mask));
}

static __rcpinline void rcp_bxortrue_nodelay(uint32_t b, uint32_t mask) {
    rcp_asm ("mcrr2 p7, #4, %0, %1, c0\n" : : "r" (b), "r" (mask));
}

// Assert (b ^ mask) is false
static __rcpinline void rcp_bxorfalse(uint32_t b, uint32_t mask) {
    rcp_asm ("mcrr p7, #5, %0, %1, c8\n" : : "r" (b), "r" (mask));
}

static __rcpinline void rcp_bxorfalse_nodelay(uint32_t b, uint32_t mask) {
    rcp_asm ("mcrr2 p7, #5, %0, %1, c8\n" : : "r" (b), "r" (mask));
}

// ----------------------------------------------------------------------------
// RCP Integer instructions

// Assert (x ^ parity) == 0x96009600u
static __rcpinline void rcp_ivalid(uint32_t x, uint32_t parity) {
    rcp_asm ("mcrr p7, #6, %0, %1, c8\n" : : "r" (x), "r" (parity));
}

static __rcpinline void rcp_ivalid_nodelay(uint32_t x, uint32_t parity) {
    rcp_asm ("mcrr2 p7, #6, %0, %1, c8\n" : : "r" (x), "r" (parity));
}

// Assert x == y
static __rcpinline void rcp_iequal(uint32_t x, uint32_t y) {
    rcp_asm ("mcrr p7, #7, %0, %1, c0\n" : : "r" (x), "r" (y));
}

static __rcpinline void rcp_iequal_nodelay(uint32_t x, uint32_t y) {
    rcp_asm ("mcrr2 p7, #7, %0, %1, c0\n" : : "r" (x), "r" (y));
}

// ----------------------------------------------------------------------------
// RCP Random instructions

// Return a random 8-bit value generated from the upper 24 bits of the 64-bit
// salt value. This is the same PRNG used for random delay values.
static __rcpinline uint8_t rcp_random_byte(void) {
    uint8_t ret;
    rcp_asm ("mrc2 p7, #2, %0, c0, c0, #0\n" : "=r" (ret));
    return ret;
}

// Note neither version of this has delay, since the PRNG for the random delay
// is the same as the one read by this MRC instruction (and it's only stepped
// once).
static __rcpinline uint8_t rcp_random_byte_nodelay(void) {
    uint8_t ret;
    rcp_asm ("mrc2 p7, #2, %0, c0, c0, #0\n" : "=r" (ret));
    return ret;
}

// ----------------------------------------------------------------------------
// RCP Sequence count instructions

// Directly write value cnt to the sequence counter.
// cnt must be a constant expression.
#define rcp_count_set(cnt) rcp_asm ( \
    "mcr p7, #4, r0, c%c0, c%c1, #0\n" \
    : : "i"(((cnt) >> 4) & 0xf), "i"((cnt) & 0xf) \
);

#define rcp_count_set_nodelay(cnt) rcp_asm ( \
    "mcr2 p7, #4, r0, c%c0, c%c1, #0\n" \
    : : "i"(((cnt) >> 4) & 0xf), "i"((cnt) & 0xf) \
);

// Check value cnt against the sequence counter, then increment the counter.
// cnt must be a constant expression.
#define rcp_count_check(cnt) rcp_asm ( \
    "mcr p7, #5, r0, c%c0, c%c1, #1\n" \
    : : "i"(((cnt) >> 4) & 0xf), "i"((cnt) & 0xf) \
);

#define rcp_count_check_nodelay(cnt) rcp_asm ( \
    "mcr2 p7, #5, r0, c%c0, c%c1, #1\n" \
    : : "i"(((cnt) >> 4) & 0xf), "i"((cnt) & 0xf) \
);

// ----------------------------------------------------------------------------
// RCP Panic instructions

// Stall the coprocessor port. If the coprocessor access goes away, assert NMI

static __rcpinline __attribute__((noreturn)) void rcp_panic(void) {
    rcp_asm("cdp p7, #0, c0, c0, c0, #1");
    __builtin_unreachable();
}

// There is no nodelay version.

#endif // !__riscv
// ----------------------------------------------------------------------------
// GAS macros for RCP instructions, for direct use in ASM files
#else // __ASSEMBLER__
#ifndef __riscv

// Assert b is a valid boolean (0xa500a500u or 0x00c300c3u)
.macro rcp_bvalid r
    mcr p7, #1, \r , c0, c0, #0
.endm

.macro rcp_bvalid_nodelay r
    mcr2 p7, #1, \r , c0, c0, #0
.endm

// Assert b is true (0xa500a500u)
.macro rcp_btrue r
    mcr p7, #2, \r , c0, c0, #0
.endm

.macro rcp_btrue_nodelay r
    mcr2 p7, #2, \r , c0, c0, #0
.endm

// Assert b is false (0x00c300c3u)
.macro rcp_bfalse r
    mcr p7, #3, \r , c0, c0, #1
.endm

.macro rcp_bfalse_nodelay r
    mcr2 p7, #3, \r , c0, c0, #1
.endm

// Assert b0 and b1 are both valid booleans
.macro rcp_b2valid b0, b1
    mcrr p7, #0, \b0 , \b1 , c8
.endm

.macro rcp_b2valid_nodelay b0, b1
    mcrr2 p7, #0, \b0 , \b1 , c8
.endm

// Assert b0 and b1 are both true
.macro rcp_b2and b0, b1
    mcrr p7, #1, \b0 , \b1 , c0
.endm

.macro rcp_b2and_nodelay b0, b1
    mcrr2 p7, #1, \b0 , \b1 , c0
.endm

// Assert b0 and b1 are valid, and at least one is true
.macro rcp_b2or b0, b1
    mcrr p7, #2, \b0 , \b1 , c0
.endm

.macro rcp_b2or_nodelay b0, b1
    mcrr2 p7, #2, \b0 , \b1 , c0
.endm

// Assert (b ^ mask) is a valid boolean
.macro rcp_bxorvalid b, mask
    mcrr p7, #3, \b , \mask , c8
.endm

.macro rcp_bxorvalid_nodelay b, mask
    mcrr2 p7, #3, \b , \mask , c8
.endm

// Assert (b ^ mask) is true
.macro rcp_bxortrue b, mask
    mcrr p7, #4, \b , \mask , c0
.endm

.macro rcp_bxortrue_nodelay b, mask
    mcrr2 p7, #4, \b , \mask , c0
.endm

// Assert (b ^ mask) is false
.macro rcp_bxorfalse b, mask
    mcrr p7, #5, \b , \mask , c8
.endm

.macro rcp_bxorfalse_nodelay b, mask
    mcrr2 p7, #5, \b , \mask , c8
.endm

// Assert (x ^ parity) == 0x96009600u
.macro rcp_ivalid x, parity
    mcrr p7, #6, \x , \parity , c8
.endm

.macro rcp_ivalid_nodelay x, parity
    mcrr2 p7, #6, \x , \parity , c8
.endm

// Assert x == y
.macro rcp_iequal x, y
    mcrr p7, #7, \x , \y , c0
.endm

.macro rcp_iequal_nodelay x, y
    mcrr2 p7, #7, \x , \y , c0
.endm

// They call this "metaprogramming" I think
.macro rcp_switch_u8_to_ch_cl macro_name, x, args:vararg
.if (\x) == 0
\macro_name  c0,  c0, \args
.elseif (\x) == 1
\macro_name  c0,  c1, \args
.elseif (\x) == 2
\macro_name  c0,  c2, \args
.elseif (\x) == 3
\macro_name  c0,  c3, \args
.elseif (\x) == 4
\macro_name  c0,  c4, \args
.elseif (\x) == 5
\macro_name  c0,  c5, \args
.elseif (\x) == 6
\macro_name  c0,  c6, \args
.elseif (\x) == 7
\macro_name  c0,  c7, \args
.elseif (\x) == 8
\macro_name  c0,  c8, \args
.elseif (\x) == 9
\macro_name  c0,  c9, \args
.elseif (\x) == 10
\macro_name  c0, c10, \args
.elseif (\x) == 11
\macro_name  c0, c11, \args
.elseif (\x) == 12
\macro_name  c0, c12, \args
.elseif (\x) == 13
\macro_name  c0, c13, \args
.elseif (\x) == 14
\macro_name  c0, c14, \args
.elseif (\x) == 15
\macro_name  c0, c15, \args
.elseif (\x) == 16
\macro_name  c1,  c0, \args
.elseif (\x) == 17
\macro_name  c1,  c1, \args
.elseif (\x) == 18
\macro_name  c1,  c2, \args
.elseif (\x) == 19
\macro_name  c1,  c3, \args
.elseif (\x) == 20
\macro_name  c1,  c4, \args
.elseif (\x) == 21
\macro_name  c1,  c5, \args
.elseif (\x) == 22
\macro_name  c1,  c6, \args
.elseif (\x) == 23
\macro_name  c1,  c7, \args
.elseif (\x) == 24
\macro_name  c1,  c8, \args
.elseif (\x) == 25
\macro_name  c1,  c9, \args
.elseif (\x) == 26
\macro_name  c1, c10, \args
.elseif (\x) == 27
\macro_name  c1, c11, \args
.elseif (\x) == 28
\macro_name  c1, c12, \args
.elseif (\x) == 29
\macro_name  c1, c13, \args
.elseif (\x) == 30
\macro_name  c1, c14, \args
.elseif (\x) == 31
\macro_name  c1, c15, \args
.elseif (\x) == 32
\macro_name  c2,  c0, \args
.elseif (\x) == 33
\macro_name  c2,  c1, \args
.elseif (\x) == 34
\macro_name  c2,  c2, \args
.elseif (\x) == 35
\macro_name  c2,  c3, \args
.elseif (\x) == 36
\macro_name  c2,  c4, \args
.elseif (\x) == 37
\macro_name  c2,  c5, \args
.elseif (\x) == 38
\macro_name  c2,  c6, \args
.elseif (\x) == 39
\macro_name  c2,  c7, \args
.elseif (\x) == 40
\macro_name  c2,  c8, \args
.elseif (\x) == 41
\macro_name  c2,  c9, \args
.elseif (\x) == 42
\macro_name  c2, c10, \args
.elseif (\x) == 43
\macro_name  c2, c11, \args
.elseif (\x) == 44
\macro_name  c2, c12, \args
.elseif (\x) == 45
\macro_name  c2, c13, \args
.elseif (\x) == 46
\macro_name  c2, c14, \args
.elseif (\x) == 47
\macro_name  c2, c15, \args
.elseif (\x) == 48
\macro_name  c3,  c0, \args
.elseif (\x) == 49
\macro_name  c3,  c1, \args
.elseif (\x) == 50
\macro_name  c3,  c2, \args
.elseif (\x) == 51
\macro_name  c3,  c3, \args
.elseif (\x) == 52
\macro_name  c3,  c4, \args
.elseif (\x) == 53
\macro_name  c3,  c5, \args
.elseif (\x) == 54
\macro_name  c3,  c6, \args
.elseif (\x) == 55
\macro_name  c3,  c7, \args
.elseif (\x) == 56
\macro_name  c3,  c8, \args
.elseif (\x) == 57
\macro_name  c3,  c9, \args
.elseif (\x) == 58
\macro_name  c3, c10, \args
.elseif (\x) == 59
\macro_name  c3, c11, \args
.elseif (\x) == 60
\macro_name  c3, c12, \args
.elseif (\x) == 61
\macro_name  c3, c13, \args
.elseif (\x) == 62
\macro_name  c3, c14, \args
.elseif (\x) == 63
\macro_name  c3, c15, \args
.elseif (\x) == 64
\macro_name  c4,  c0, \args
.elseif (\x) == 65
\macro_name  c4,  c1, \args
.elseif (\x) == 66
\macro_name  c4,  c2, \args
.elseif (\x) == 67
\macro_name  c4,  c3, \args
.elseif (\x) == 68
\macro_name  c4,  c4, \args
.elseif (\x) == 69
\macro_name  c4,  c5, \args
.elseif (\x) == 70
\macro_name  c4,  c6, \args
.elseif (\x) == 71
\macro_name  c4,  c7, \args
.elseif (\x) == 72
\macro_name  c4,  c8, \args
.elseif (\x) == 73
\macro_name  c4,  c9, \args
.elseif (\x) == 74
\macro_name  c4, c10, \args
.elseif (\x) == 75
\macro_name  c4, c11, \args
.elseif (\x) == 76
\macro_name  c4, c12, \args
.elseif (\x) == 77
\macro_name  c4, c13, \args
.elseif (\x) == 78
\macro_name  c4, c14, \args
.elseif (\x) == 79
\macro_name  c4, c15, \args
.elseif (\x) == 80
\macro_name  c5,  c0, \args
.elseif (\x) == 81
\macro_name  c5,  c1, \args
.elseif (\x) == 82
\macro_name  c5,  c2, \args
.elseif (\x) == 83
\macro_name  c5,  c3, \args
.elseif (\x) == 84
\macro_name  c5,  c4, \args
.elseif (\x) == 85
\macro_name  c5,  c5, \args
.elseif (\x) == 86
\macro_name  c5,  c6, \args
.elseif (\x) == 87
\macro_name  c5,  c7, \args
.elseif (\x) == 88
\macro_name  c5,  c8, \args
.elseif (\x) == 89
\macro_name  c5,  c9, \args
.elseif (\x) == 90
\macro_name  c5, c10, \args
.elseif (\x) == 91
\macro_name  c5, c11, \args
.elseif (\x) == 92
\macro_name  c5, c12, \args
.elseif (\x) == 93
\macro_name  c5, c13, \args
.elseif (\x) == 94
\macro_name  c5, c14, \args
.elseif (\x) == 95
\macro_name  c5, c15, \args
.elseif (\x) == 96
\macro_name  c6,  c0, \args
.elseif (\x) == 97
\macro_name  c6,  c1, \args
.elseif (\x) == 98
\macro_name  c6,  c2, \args
.elseif (\x) == 99
\macro_name  c6,  c3, \args
.elseif (\x) == 100
\macro_name  c6,  c4, \args
.elseif (\x) == 101
\macro_name  c6,  c5, \args
.elseif (\x) == 102
\macro_name  c6,  c6, \args
.elseif (\x) == 103
\macro_name  c6,  c7, \args
.elseif (\x) == 104
\macro_name  c6,  c8, \args
.elseif (\x) == 105
\macro_name  c6,  c9, \args
.elseif (\x) == 106
\macro_name  c6, c10, \args
.elseif (\x) == 107
\macro_name  c6, c11, \args
.elseif (\x) == 108
\macro_name  c6, c12, \args
.elseif (\x) == 109
\macro_name  c6, c13, \args
.elseif (\x) == 110
\macro_name  c6, c14, \args
.elseif (\x) == 111
\macro_name  c6, c15, \args
.elseif (\x) == 112
\macro_name  c7,  c0, \args
.elseif (\x) == 113
\macro_name  c7,  c1, \args
.elseif (\x) == 114
\macro_name  c7,  c2, \args
.elseif (\x) == 115
\macro_name  c7,  c3, \args
.elseif (\x) == 116
\macro_name  c7,  c4, \args
.elseif (\x) == 117
\macro_name  c7,  c5, \args
.elseif (\x) == 118
\macro_name  c7,  c6, \args
.elseif (\x) == 119
\macro_name  c7,  c7, \args
.elseif (\x) == 120
\macro_name  c7,  c8, \args
.elseif (\x) == 121
\macro_name  c7,  c9, \args
.elseif (\x) == 122
\macro_name  c7, c10, \args
.elseif (\x) == 123
\macro_name  c7, c11, \args
.elseif (\x) == 124
\macro_name  c7, c12, \args
.elseif (\x) == 125
\macro_name  c7, c13, \args
.elseif (\x) == 126
\macro_name  c7, c14, \args
.elseif (\x) == 127
\macro_name  c7, c15, \args
.elseif (\x) == 128
\macro_name  c8,  c0, \args
.elseif (\x) == 129
\macro_name  c8,  c1, \args
.elseif (\x) == 130
\macro_name  c8,  c2, \args
.elseif (\x) == 131
\macro_name  c8,  c3, \args
.elseif (\x) == 132
\macro_name  c8,  c4, \args
.elseif (\x) == 133
\macro_name  c8,  c5, \args
.elseif (\x) == 134
\macro_name  c8,  c6, \args
.elseif (\x) == 135
\macro_name  c8,  c7, \args
.elseif (\x) == 136
\macro_name  c8,  c8, \args
.elseif (\x) == 137
\macro_name  c8,  c9, \args
.elseif (\x) == 138
\macro_name  c8, c10, \args
.elseif (\x) == 139
\macro_name  c8, c11, \args
.elseif (\x) == 140
\macro_name  c8, c12, \args
.elseif (\x) == 141
\macro_name  c8, c13, \args
.elseif (\x) == 142
\macro_name  c8, c14, \args
.elseif (\x) == 143
\macro_name  c8, c15, \args
.elseif (\x) == 144
\macro_name  c9,  c0, \args
.elseif (\x) == 145
\macro_name  c9,  c1, \args
.elseif (\x) == 146
\macro_name  c9,  c2, \args
.elseif (\x) == 147
\macro_name  c9,  c3, \args
.elseif (\x) == 148
\macro_name  c9,  c4, \args
.elseif (\x) == 149
\macro_name  c9,  c5, \args
.elseif (\x) == 150
\macro_name  c9,  c6, \args
.elseif (\x) == 151
\macro_name  c9,  c7, \args
.elseif (\x) == 152
\macro_name  c9,  c8, \args
.elseif (\x) == 153
\macro_name  c9,  c9, \args
.elseif (\x) == 154
\macro_name  c9, c10, \args
.elseif (\x) == 155
\macro_name  c9, c11, \args
.elseif (\x) == 156
\macro_name  c9, c12, \args
.elseif (\x) == 157
\macro_name  c9, c13, \args
.elseif (\x) == 158
\macro_name  c9, c14, \args
.elseif (\x) == 159
\macro_name  c9, c15, \args
.elseif (\x) == 160
\macro_name c10,  c0, \args
.elseif (\x) == 161
\macro_name c10,  c1, \args
.elseif (\x) == 162
\macro_name c10,  c2, \args
.elseif (\x) == 163
\macro_name c10,  c3, \args
.elseif (\x) == 164
\macro_name c10,  c4, \args
.elseif (\x) == 165
\macro_name c10,  c5, \args
.elseif (\x) == 166
\macro_name c10,  c6, \args
.elseif (\x) == 167
\macro_name c10,  c7, \args
.elseif (\x) == 168
\macro_name c10,  c8, \args
.elseif (\x) == 169
\macro_name c10,  c9, \args
.elseif (\x) == 170
\macro_name c10, c10, \args
.elseif (\x) == 171
\macro_name c10, c11, \args
.elseif (\x) == 172
\macro_name c10, c12, \args
.elseif (\x) == 173
\macro_name c10, c13, \args
.elseif (\x) == 174
\macro_name c10, c14, \args
.elseif (\x) == 175
\macro_name c10, c15, \args
.elseif (\x) == 176
\macro_name c11,  c0, \args
.elseif (\x) == 177
\macro_name c11,  c1, \args
.elseif (\x) == 178
\macro_name c11,  c2, \args
.elseif (\x) == 179
\macro_name c11,  c3, \args
.elseif (\x) == 180
\macro_name c11,  c4, \args
.elseif (\x) == 181
\macro_name c11,  c5, \args
.elseif (\x) == 182
\macro_name c11,  c6, \args
.elseif (\x) == 183
\macro_name c11,  c7, \args
.elseif (\x) == 184
\macro_name c11,  c8, \args
.elseif (\x) == 185
\macro_name c11,  c9, \args
.elseif (\x) == 186
\macro_name c11, c10, \args
.elseif (\x) == 187
\macro_name c11, c11, \args
.elseif (\x) == 188
\macro_name c11, c12, \args
.elseif (\x) == 189
\macro_name c11, c13, \args
.elseif (\x) == 190
\macro_name c11, c14, \args
.elseif (\x) == 191
\macro_name c11, c15, \args
.elseif (\x) == 192
\macro_name c12,  c0, \args
.elseif (\x) == 193
\macro_name c12,  c1, \args
.elseif (\x) == 194
\macro_name c12,  c2, \args
.elseif (\x) == 195
\macro_name c12,  c3, \args
.elseif (\x) == 196
\macro_name c12,  c4, \args
.elseif (\x) == 197
\macro_name c12,  c5, \args
.elseif (\x) == 198
\macro_name c12,  c6, \args
.elseif (\x) == 199
\macro_name c12,  c7, \args
.elseif (\x) == 200
\macro_name c12,  c8, \args
.elseif (\x) == 201
\macro_name c12,  c9, \args
.elseif (\x) == 202
\macro_name c12, c10, \args
.elseif (\x) == 203
\macro_name c12, c11, \args
.elseif (\x) == 204
\macro_name c12, c12, \args
.elseif (\x) == 205
\macro_name c12, c13, \args
.elseif (\x) == 206
\macro_name c12, c14, \args
.elseif (\x) == 207
\macro_name c12, c15, \args
.elseif (\x) == 208
\macro_name c13,  c0, \args
.elseif (\x) == 209
\macro_name c13,  c1, \args
.elseif (\x) == 210
\macro_name c13,  c2, \args
.elseif (\x) == 211
\macro_name c13,  c3, \args
.elseif (\x) == 212
\macro_name c13,  c4, \args
.elseif (\x) == 213
\macro_name c13,  c5, \args
.elseif (\x) == 214
\macro_name c13,  c6, \args
.elseif (\x) == 215
\macro_name c13,  c7, \args
.elseif (\x) == 216
\macro_name c13,  c8, \args
.elseif (\x) == 217
\macro_name c13,  c9, \args
.elseif (\x) == 218
\macro_name c13, c10, \args
.elseif (\x) == 219
\macro_name c13, c11, \args
.elseif (\x) == 220
\macro_name c13, c12, \args
.elseif (\x) == 221
\macro_name c13, c13, \args
.elseif (\x) == 222
\macro_name c13, c14, \args
.elseif (\x) == 223
\macro_name c13, c15, \args
.elseif (\x) == 224
\macro_name c14,  c0, \args
.elseif (\x) == 225
\macro_name c14,  c1, \args
.elseif (\x) == 226
\macro_name c14,  c2, \args
.elseif (\x) == 227
\macro_name c14,  c3, \args
.elseif (\x) == 228
\macro_name c14,  c4, \args
.elseif (\x) == 229
\macro_name c14,  c5, \args
.elseif (\x) == 230
\macro_name c14,  c6, \args
.elseif (\x) == 231
\macro_name c14,  c7, \args
.elseif (\x) == 232
\macro_name c14,  c8, \args
.elseif (\x) == 233
\macro_name c14,  c9, \args
.elseif (\x) == 234
\macro_name c14, c10, \args
.elseif (\x) == 235
\macro_name c14, c11, \args
.elseif (\x) == 236
\macro_name c14, c12, \args
.elseif (\x) == 237
\macro_name c14, c13, \args
.elseif (\x) == 238
\macro_name c14, c14, \args
.elseif (\x) == 239
\macro_name c14, c15, \args
.elseif (\x) == 240
\macro_name c15,  c0, \args
.elseif (\x) == 241
\macro_name c15,  c1, \args
.elseif (\x) == 242
\macro_name c15,  c2, \args
.elseif (\x) == 243
\macro_name c15,  c3, \args
.elseif (\x) == 244
\macro_name c15,  c4, \args
.elseif (\x) == 245
\macro_name c15,  c5, \args
.elseif (\x) == 246
\macro_name c15,  c6, \args
.elseif (\x) == 247
\macro_name c15,  c7, \args
.elseif (\x) == 248
\macro_name c15,  c8, \args
.elseif (\x) == 249
\macro_name c15,  c9, \args
.elseif (\x) == 250
\macro_name c15, c10, \args
.elseif (\x) == 251
\macro_name c15, c11, \args
.elseif (\x) == 252
\macro_name c15, c12, \args
.elseif (\x) == 253
\macro_name c15, c13, \args
.elseif (\x) == 254
\macro_name c15, c14, \args
.elseif (\x) == 255
\macro_name c15, c15, \args
.else
.error "Value outside of range 0-255"
.endif
.endm

// Directly write 8-bit constant expression cnt to the sequence counter.
.macro rcp_count_set_impl h, l
mcr p7, #4, r0, \h , \l , #0
.endm
.macro rcp_count_set cnt
rcp_switch_u8_to_ch_cl rcp_count_set_impl, \cnt
.endm

.macro rcp_count_set_nodelay_impl h, l
mcr2 p7, #4, r0, \h , \l , #0
.endm
.macro rcp_count_set_nodelay cnt
rcp_switch_u8_to_ch_cl rcp_count_set_nodelay_impl, \cnt
.endm

// Check 8-bit constant expression cnt against the sequence counter, then
// increment the counter.
.macro rcp_count_check_impl h, l
    mcr p7, #5, r0, \h, \l, #1
.endm
.macro rcp_count_check cnt
rcp_switch_u8_to_ch_cl rcp_count_check_impl, \cnt
.endm

.macro rcp_count_check_nodelay_impl h, l
    mcr2 p7, #5, r0, \h, \l, #1
.endm
.macro rcp_count_check_nodelay cnt
rcp_switch_u8_to_ch_cl rcp_count_check_nodelay_impl, \cnt
.endm

// Get a 32-bit canary value. `tag` must be a constant expression.
.macro rcp_canary_get_impl h, l, x
   mrc p7, #0, \x, \h, \l, #1
.endm

.macro rcp_canary_get x, tag
rcp_switch_u8_to_ch_cl rcp_canary_get_impl \tag, \x
.endm

// Get a 32-bit canary value. `tag` must be a constant expression.
.macro rcp_canary_get_nodelay_impl h, l, x
   mrc2 p7, #0, \x, \h, \l, #1
.endm

.macro rcp_canary_get_nodelay x, tag
rcp_switch_u8_to_ch_cl rcp_canary_get_nodelay_impl \tag, \x
.endm

// Assert that canary matches result of rcp_canary_get with the same tags:
.macro rcp_canary_check_impl h, l, x
   mcr p7, #0, \x, \h, \l, #1
.endm

.macro rcp_canary_check x, tag
rcp_switch_u8_to_ch_cl rcp_canary_check_impl \tag, \x
.endm

.macro rcp_canary_check_nodelay_impl h, l, x
   mcr2 p7, #0, \x, \h, \l, #1
.endm

.macro rcp_canary_check_nodelay x, tag
rcp_switch_u8_to_ch_cl rcp_canary_check_nodelay_impl \tag, \x
.endm

.macro rcp_panic
    cdp p7, #0, c0, c0, c0, #1
.endm

#endif // !__riscv
#endif // __ASSEMBLER__
// ----------------------------------------------------------------------------

#ifdef __cplusplus
}
#endif
#endif
#endif
