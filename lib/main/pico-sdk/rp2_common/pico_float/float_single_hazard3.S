/*
 * Copyright (c) 2024 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "pico/asm_helper.S"
#include "hardware/hazard3.h"

// This file reimplements some common single-precision soft float routines
// from libgcc. It targets the RV32IMBZbkb dialect (plus optionally Xh3bextm)
// and is tuned for Hazard3 execution timings.

// Subnormal values are always flushed to zero on both input and output.
// Rounding is always to nearest (even on tie).

pico_default_asm_setup

.macro float_section name
#if PICO_FLOAT_IN_RAM
.section RAM_SECTION_NAME(\name), "ax"
#else
.section SECTION_NAME(\name), "ax"
#endif
.endm

float_section __addsf3
.global __subsf3
.p2align 2
__subsf3:
    binvi a1, a1, 31
.global __addsf3
__addsf3:
    // Unpack exponent:
    h3.bextmi a2, a0, 23, 8
    h3.bextmi a3, a1, 23, 8
    // Flush-to-zero => 0 + y = y applies, including nan, with the sole
    // exception of y being subnormal (which also needs to be flushed)
    beqz a2, __addsf_return_y_flushed
    // Don't have to handle this case for x + 0 = 0 because we already know x
    // is nonzero
    beqz a3, __addsf_return_x
    // Unpack significand, plus 3 extra zeroes for working space:
    slli a4, a0, 9
    slli a5, a1, 9
    // check nan/inf on input
    li t0, 255
    beq a2, t0, __addsf_x_nan_inf
    beq a3, t0, __addsf_y_nan_inf
    // (finish unpacking significand)
    srli a4, a4, 6
    srli a5, a5, 6

    // If we're still on the straight path then we are adding two normal
    // values. Add implicit one (1.xx...xx000)
    bseti a4, a4, 23 + 3
    bseti a5, a5, 23 + 3
    // Negate if sign bit is set
    bgez a0, 1f
    neg a4, a4
1:
    // (tuck this 16-bit here to avoid alignment penalty)
    li t1, 25
    bgez a1, 1f
    neg a5, a5
1:

    bltu a2, a3, __addsf_ye_gt_xe

    // The main body is repeated twice with different register assignments.
    // lhs is the more-significant addend:
.macro addsf_core packed_lhs, packed_rhs, sig_lhs, sig_rhs, exp_lhs, exp_rhs, rhs_is_x
    sub \packed_rhs, \exp_lhs, \exp_rhs
    // If there is a large exponent difference then there is no effect on lhs
.if \rhs_is_x
    bgeu \packed_rhs, t1, __addsf_return_y
.else
    bgeu \packed_rhs, t1, __addsf_return_x
.endif
    // Shift rhs down to correct relative significance
    sra \packed_lhs, \sig_rhs, \packed_rhs
    // Set sticky bit if ones were shifted out
    sll \packed_rhs, \packed_lhs, \packed_rhs
    sltu \packed_rhs, \packed_rhs, \sig_rhs
    or \packed_lhs, \packed_lhs, \packed_rhs
    // Add significands
    add \sig_lhs, \sig_lhs, \packed_lhs
    // Detect exact cancellation (may be beyond max normalisation shift; also
    // IEEE 754 requires +0 for exact cancellation, no matter input signs)
    beqz \sig_lhs, __addsf_return_0
    // Convert two's complement back to sign + magnitude
    srai \exp_rhs, \sig_lhs, 31
    xor \sig_lhs, \sig_lhs, \exp_rhs
    sub \sig_lhs, \sig_lhs, \exp_rhs
    // Renormalise significand: bit 31 is now implicit one
    clz \packed_lhs, \sig_lhs
    sll \sig_lhs, \sig_lhs, \packed_lhs
    // Adjust exponent
    addi \packed_lhs, \packed_lhs, -5
    sub \exp_lhs, \exp_lhs, \packed_lhs

    // Round to nearest, even on tie (bias upward if above odd number)
    bexti \packed_lhs, \sig_lhs, 8
    addi \sig_lhs, \sig_lhs, 127
    add \sig_lhs, \sig_lhs, \packed_lhs
    // Exponent may increase by one due to rounding up from all-ones; this is
    // detected by clearing of implicit one (there is a carry-out too)
    bgez \sig_lhs, 3f
4:
    // Detect underflow/overflow
    bgeu \exp_lhs, t0, 1f

    // Pack and return
    packh \exp_lhs, \exp_lhs, \exp_rhs
    slli \exp_lhs, \exp_lhs, 23
    slli \sig_lhs, \sig_lhs, 1
    srli \sig_lhs, \sig_lhs, 9
    add a0, \sig_lhs, \exp_lhs
    ret
1:
    bgez \exp_lhs, 2f
    // Signed zero on underflow
    slli a0, \exp_rhs, 31
    ret
2:
    // Signed infinity on overflow
    packh a0, t0, \exp_rhs
    slli a0, a0, 23
    ret
3:
    // Exponent increase due to rounding (uncommon)
    srli \sig_lhs, \sig_lhs, 1
    addi \exp_lhs, \exp_lhs, 1
    j 4b
.endm

__addsf_xe_gte_ye:
    addsf_core a0, a1, a4, a5, a2, a3, 0
.p2align 2
__addsf_ye_gt_xe:
    addsf_core a1, a0, a5, a4, a3, a2, 1

__addsf_x_nan_inf:
    // When at least one operand is nan, we must propagate at least one of
    // those nan payloads (sign of nan result is unspecified, which we take
    // advantage of by implementing x - y as x + -y). Check x nan vs inf:
    bnez a4, __addsf_return_x
__addsf_x_inf:
    // If x is +-inf, need to distinguish the following cases:
    bne  a3, t0, __addsf_return_x // y is neither inf nor nan   -> return x (propagate inf)
    bnez a5,     __addsf_return_y // y is nan:                  -> return y (propagate nan)
    xor a5, a0, a1
    srli a5, a5, 31
    beqz a5,     __addsf_return_x // y is inf of same sign      -> return either x or y (x is faster)
    li a0, -1                     // y is inf of different sign -> return nan
    ret

__addsf_y_nan_inf:
    // Mirror of __addsf_x_nan_inf
    bnez a5, __addsf_return_y
__addsf_y_inf:
    bne  a2, t0, __addsf_return_y
    bnez a4,     __addsf_return_x
    xor a4, a0, a1
    srli a4, a4, 31
    beqz a4,     __addsf_return_x
    li a0, -1
    ret

__addsf_return_y_flushed:
    bnez a3, 1f
    srli a1, a1, 23
    slli a1, a1, 23
1:
__addsf_return_y:
    mv a0, a1
__addsf_return_x:
    ret
__addsf_return_0:
    li a0, 0
    ret


float_section __mulsf3
.global __mulsf3
.p2align 2
__mulsf3:
    // Force y to be positive (by possibly negating x) *before* unpacking.
    // This allows many special cases to be handled without repacking.
    bgez a1, 1f
    binvi a0, a0, 31
1:
    // Unpack exponent:
    h3.bextmi a2, a0, 23, 8
    h3.bextmi a3, a1, 23, 8
    // Check special cases
    li t0, 255
    beqz a2, __mulsf_x_0
    beqz a3, __mulsf_y_0
    beq a2, t0, __mulsf_x_nan_inf
    beq a3, t0, __mulsf_y_nan_inf

    // Finish unpacking sign
    srai a6, a0, 31
    // Unpack significand (with implicit one in MSB)
    slli a4, a0, 8
    slli a5, a1, 8
    bseti a4, a4, 31
    bseti a5, a5, 31
    // Get full 64-bit multiply result in a4:a1 (one cycle each half)
    // Going from Q1.23 to Q2.46 (both left-justified)
    mul a1, a4, a5
    mulhu a4, a4, a5
    // Normalise (shift left by either 0 or 1) -- bit 8 is the LSB of the
    // final significand (ignoring rounding)
    clz a0, a4
    sll a4, a4, a0
    sub a2, a2, a0
    // After normalising we can calculate the final exponent, since rounding
    // cannot increase the exponent for multiplication (unlike addition)
    add a2, a2, a3
    // Subtract redundant bias term (127), add 1 for normalisation correction
    addi a2, a2, -126
    blez a2, __mulsf_underflow
    bge a2, t0, __mulsf_overflow

    // Gather sticky bits from low fraction:
    snez a1, a1
    or a4, a4, a1
    // Round to nearest, even on tie (aka bias upward if odd)
    bexti a1, a4, 8
    add a4, a4, a1
    addi a4, a4, 127
    // Pack it and ship it
    packh a2, a2, a6
    slli a2, a2, 23
    slli a4, a4, 1
    srli a4, a4, 9
    add a0, a4, a2
    ret

__mulsf_underflow:
    // Signed zero
    slli a0, a6, 31
    ret
__mulsf_overflow:
    // Signed inf
    packh a0, t0, a6
    slli a0, a0, 23
    ret

__mulsf_x_0:
    // 0 times nan    -> propagate nan
    // 0 times inf    -> generate nan
    // 0 times others -> 0 (need to flush significand too as we are FTZ)
    bne a3, t0, __mulsf_return_flushed_x
    slli a5, a1, 9
    beqz a5, 1f
    // Propagate nan from y
__mulsf_return_y:
    mv a0, a1
    ret
1:
    // Generate new nan
    li a0, -1
    ret

__mulsf_y_0:
    // Mirror image of x_0 except we still return x for signed 0, since the
    // signs were already resolved.
    bne a2, t0, __mulsf_return_flushed_x
    slli a1, a0, 9
    bnez a1, 1f
    li a0, -1
1:
    ret

__mulsf_return_flushed_x:
    // If we don't support subnormals we at least need to flush to a canonical
    // zero. This is just a sign bit in bit 31.
    srli a0, a0, 31
    slli a0, a0, 31
__mulsf_return_x:
    ret

__mulsf_x_nan_inf:
    // We know that y is not zero and is positive. So...
    //      x is nan    -> return x
    // else y is nan    -> return y
    // else y is inf    -> return x
    // else y is normal -> return x
    // (the order of the first two clauses is actually our free choice)
    slli a4, a0, 9
    bnez a4, __mulsf_return_x
    bne a3, t0, __mulsf_return_x
    slli a5, a1, 9
    bnez a5, __mulsf_return_y
    ret // return x

__mulsf_y_nan_inf:
    // We know that x is not zero, nan, nor inf. That just leaves normals.
    // y is nan -> return y
    // y is inf -> return inf * sgn(x) (since we already merged the signs)
    slli a5, a1, 9
    bnez a5, __mulsf_return_y
    srai a0, a0, 31
    packh a0, t0, a0
    slli a0, a0, 23
    ret


// This is a hack to improve soft float performance for the routines we don't
// implement (e.g. libm) in libraries built against a non-Zbb ISA dialect:
float_section __clz2si
.global __clz2si
__clz2si:
    clz a0, a0
    ret
