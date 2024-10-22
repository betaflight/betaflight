/*
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "pico/asm_helper.S"
#include "pico/bootrom.h"
#include "pico/runtime_init.h"

pico_default_asm_setup

.macro bits_section name
#if PICO_BITS_IN_RAM
.section RAM_SECTION_NAME(\name), "ax"
#else
.section SECTION_NAME(\name), "ax"
#endif
.endm

#if PICO_RP2040
PICO_RUNTIME_INIT_FUNC_RUNTIME(__aeabi_bits_init, PICO_RUNTIME_INIT_AEABI_BIT_OPS)

.section .data.aeabi_bits_funcs
.global aeabi_bits_funcs, aeabi_bits_funcs_end
.equ BITS_FUNC_COUNT, 4
.align 4
aeabi_bits_funcs:
    .word ROM_FUNC_POPCOUNT32
    .word ROM_FUNC_CLZ32
    .word ROM_FUNC_CTZ32
    .word ROM_FUNC_REVERSE32
aeabi_bits_funcs_end:

.section .text
.thumb_func
__aeabi_bits_init:
    ldr r0, =aeabi_bits_funcs
    movs r1, #BITS_FUNC_COUNT
    ldr r3, =rom_funcs_lookup
    bx r3

.equ POPCOUNT32, 0
.equ CLZ32, 4
.equ CTZ32, 8
.equ REVERSE32, 12

bits_section clzsi
wrapper_func __clz
wrapper_func __clzl
wrapper_func __clzsi2
    ldr r3, =aeabi_bits_funcs
    ldr r3, [r3, #CLZ32]
    bx r3

bits_section ctzsi
wrapper_func __ctzsi2
    ldr r3, =aeabi_bits_funcs
    ldr r3, [r3, #CTZ32]
    bx r3

bits_section popcountsi
wrapper_func __popcountsi2
    ldr r3, =aeabi_bits_funcs
    ldr r3, [r3, #POPCOUNT32]
    bx r3

bits_section clzdi
wrapper_func __clzll
wrapper_func __clzdi2
    ldr r3, =aeabi_bits_funcs
    ldr r3, [r3, #CLZ32]
    cmp r1, #0
    bne 1f
    push {lr}
    blx r3
    adds r0, #32
    pop {pc}
1:
    mov r0, r1
    bx r3

bits_section ctzdi
wrapper_func __ctzdi2
    ldr r3, =aeabi_bits_funcs
    ldr r3, [r3, #CTZ32]
    cmp r0, #0
    beq 1f
    bx r3
1:
    push {lr}
    mov r0, r1
    blx r3
    adds r0, #32
    pop {pc}

bits_section popcountdi
wrapper_func __popcountdi2
    ldr r3, =aeabi_bits_funcs
    ldr r3, [r3, #POPCOUNT32]
    push {r1, r3, lr}
    blx r3
    mov ip, r0
    pop {r0, r3}
    blx r3
    mov r1, ip
    add r0, r1
    pop {pc}

bits_section __rev
regular_func __rev
regular_func __revl
regular_func reverse32
    ldr r3, =aeabi_bits_funcs
    ldr r3, [r3, #REVERSE32]
    bx r3

bits_section __revll
regular_func __revll
regular_func reverse64
    push {lr}
    ldr r3, =aeabi_bits_funcs
    ldr r3, [r3, #REVERSE32]
    push {r1, r3}
    blx r3
    mov ip, r0 // reverse32 preserves ip
    pop {r0, r3}
    blx r3
    mov r1, ip
    pop {pc}

#else // RP2350

bits_section __rev
regular_func __rev
regular_func __revl
regular_func reverse32
#ifdef __riscv
    rev8 a0, a0
    brev8 a0, a0
    ret
#else
    rbit r0, r0
    bx lr
#endif

bits_section __revll
regular_func __revll
regular_func reverse64
#ifdef __riscv
    rev8 a0, a0
    brev8 a0, a0
    rev8 a1, a1
    brev8 a1, a1
    mv a2, a0
    mv a0, a1
    mv a1, a2
    ret
#else
    mov r2, r1
    rbit r1, r0
    rbit r0, r2
    bx lr
#endif

// todo gcc riscv version is nuts; we'll provide both versions for simplicity in CMakeLists.txt
//  (also gcc arm version is a bit pants)
bits_section ctzdi
wrapper_func __ctzdi2
#ifdef __riscv
    beqz a0, 1f
    ctz a0, a0
    ret
1:
    ctz a0, a1
    addi a0, a0, 32
    ret
#else
    cbz r0, 1f
    rbit r0, r0
    clz r0, r0
    bx	lr
1:
    rbit r0, r1
    clz r0, r0
    adds r0, #32
    bx	lr
#endif

#endif