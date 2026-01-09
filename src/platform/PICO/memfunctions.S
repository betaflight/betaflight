/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "pico/asm_helper.S"

.section .text.memfunctions, "ax"

// Wrapped memset and memcpy functions, code copied from rp2040 bootrom source
// https://github.com/raspberrypi/pico-bootrom-rp2040/blob/master/bootrom/bootrom_misc.S

// memcpy function assuming r0 and r1 are both word aligned
wrapper_func memcpy_44
    mov ip, r0
    cmp r2, #8
    blo _memcpy_short
    push {r4-r6}
    b _memcpy_aligned

_memcpy_slow_lp:
    sub r2, #1
    ldrb r3, [r1, r2]
    strb r3, [r0, r2]
    bne _memcpy_slow_lp
    mov r0, ip
    bx lr

.align 2
wrapper_func memcpy
    mov ip, r0
    cmp r2, #8
    blo _memcpy_short
    sub r3, r0, r1
    lsl r3, #30
    bne _memcpy_slow_lp

    // r0 and r1 are co-aligned
    push {r4-r6}
    sub r1, r0
    mov r5, r0
    lsr r3, r0, #1
    bcc 1f

    // byte at odd address
    ldrb r4, [r0, r1]
    strb r4, [r0]
    add r0, #1
1:
    lsr r3, r0, #2
    bcc 1f

    // halfword on non word boundary
    ldrh r4, [r0, r1]
    strh r4, [r0]
    add r0, #2
1:
    // adjust length
    add r1, r0
    sub r5, r0
    add r2, r5

_memcpy_aligned:
    sub r2, #16
    bcc 5f
.align 2
    // 16 byte loop
1:
    ldmia r1!, {r3, r4, r5, r6}
    stmia r0!, {r3, r4, r5, r6}
    sub r2, #16
    bcs 1b
5:
    // 8 bytes remainder?
    lsl r2, #29
    bcc 1f
    ldmia r1!, {r3, r4}
    stmia r0!, {r3, r4}
1:
    // 4 bytes remainder?
    lsl r2, #1
    bcc 1f
    ldmia r1!, {r3}
    stmia r0!, {r3}
1:
    // early out for word aligned ending
    beq 2f
    // 2 bytes remainder?
    lsl r2, #1
    bcc 1f
    ldrh r3, [r1]
    strh r3, [r0]
    beq 2f
    add r1, #2
    add r0, #2
    // note fall thru into branch that wont take
1:
    // 1 bytes remainder?
    beq 2f
    ldrb r3, [r1]
    strb r3, [r0]

2:
    pop {r4-r6}
    mov r0, ip
    bx lr

_memcpy_short:
    adr r3, _memcpy_short_end
    lsl r2, #2
    sub r3, r2
    add r3, #1
    bx r3

.align 2
    ldrb r3, [r1, #6]
    strb r3, [r0, #6]
    ldrb r3, [r1, #5]
    strb r3, [r0, #5]
    ldrb r3, [r1, #4]
    strb r3, [r0, #4]
    ldrb r3, [r1, #3]
    strb r3, [r0, #3]
    ldrb r3, [r1, #2]
    strb r3, [r0, #2]
    ldrb r3, [r1, #1]
    strb r3, [r0, #1]
    ldrb r3, [r1, #0]
    strb r3, [r0, #0]
_memcpy_short_end:
    mov r0, ip
    bx lr

// memset function assuming r0 is word aligned
wrapper_func memset_4
    mov ip, r0
    uxtb r1, r1
    lsl  r3, r1, #8
    orr  r1, r3
    b _memset_word_boundary_with_hword_r1

.align 2
wrapper_func memset
    mov ip, r0
    cmp r2, #8
    blo _memset_short

    lsr r3, r0, #1
    bcc 1f
    // byte at odd address
    strb r1, [r0]
    add r0, #1
1:

    // do byte->hword early
    uxtb r1, r1
    lsl  r3, r1, #8
    orr  r1, r3
    lsr r3, r0, #2
    bcc 1f

    // halfword on non word boundary
    strh r1, [r0]
    add r0, #2
1:

    // adjust length
    mov r3, ip
    sub r3, r0
    add r2, r3

_memset_word_boundary_with_hword_r1:
    // at this point we already have r1 == 0000XXXX so extends to r1 = XXXXXXXX r3 = XXXXXXXX
    rev  r3, r1
    orr  r1, r3
    mov r3, r1

    sub r2, #16
    bcc 5f

    // extend to r4 = XXXXXXXX r5 = XXXXXXXX
    push {r4-r5}
    mov r4, r1
    mov r5, r1
.align 2
    // 16 byte loop
1:
    stmia r0!, {r1, r3, r4, r5}
    sub r2, #16
    bcs 1b
    pop {r4-r5}
5:
    // 8 bytes remainder?
    lsl r2, #29
    bcc 1f
    stmia r0!, {r1, r3}
1:
    // 4 bytes remainder?
    lsl r2, #1
    bcc 1f
    stmia r0!, {r1}
1:
    // early out for word aligned ending
    beq 2f
    // 2 bytes remainder?
    lsl r2, #1
    bcc 1f
    strh r1, [r0]
    beq 2f
    add r0, #2
    // note fall thru into branch that wont take
1:
    // 1 byte remainder?
    beq 2f
    strb r1, [r0]
2:
    mov r0, ip
    bx lr

_memset_short:
    adr r3, _memset_short_end
    sub r3, r2
    sub r3, r2
    add r3, #1
    bx r3

    strb r1, [r0, #6]
    strb r1, [r0, #5]
    strb r1, [r0, #4]
    strb r1, [r0, #3]
    strb r1, [r0, #2]
    strb r1, [r0, #1]
    strb r1, [r0, #0]
_memset_short_end:
    mov r0, ip
    bx lr
