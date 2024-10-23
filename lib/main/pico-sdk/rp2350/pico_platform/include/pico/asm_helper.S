/*
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "pico.h"

#ifdef __riscv
// Get macros for convenient use of Hazard3 instructions without binutils support
#include "hardware/hazard3/instructions.h"
#endif

#if !PICO_ASSEMBLER_IS_CLANG
#define apsr_nzcv r15
#endif
// note we don't do this by default in this file for backwards comaptibility with user code
// that may include this file, but not use unified syntax. Note that this macro does equivalent
// setup to the pico_default_asm macro for inline assembly in C code.
.macro pico_default_asm_setup
#ifndef __riscv
.syntax unified
.cpu cortex-m33
.fpu fpv5-sp-d16
.thumb
#endif
.endm

// do not put align in here as it is used mid function sometimes
.macro regular_func x
.global \x
.type \x,%function
#ifndef __riscv
.thumb_func
#endif
\x:
.endm

.macro weak_func x
.weak \x
.type \x,%function
#ifndef __riscv
.thumb_func
#endif
\x:
.endm

.macro regular_func_with_section x
.section .text.\x
regular_func \x
.endm

// do not put align in here as it is used mid function sometimes
.macro wrapper_func x
regular_func WRAPPER_FUNC_NAME(\x)
.endm

.macro weak_wrapper_func x
weak_func WRAPPER_FUNC_NAME(\x)
.endm

.macro __pre_init_with_offset func, offset, priority_string1
.section .preinit_array.\priority_string1
.p2align 2
.word \func + \offset
.endm

// backwards compatibility
.macro __pre_init func, priority_string1
__pre_init_with_offset func, 0, priority_string1
.endm
