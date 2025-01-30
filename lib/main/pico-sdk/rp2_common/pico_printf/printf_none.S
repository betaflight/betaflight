/*
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "pico/asm_helper.S"
#include "pico/bootrom/sf_table.h"

pico_default_asm_setup

wrapper_func sprintf
wrapper_func snprintf
wrapper_func vsnprintf
// pico_stdio may itself overwrite printf, so we want to support that (hence weak), but override it
// here to in case pico_stdio is not used
weak_wrapper_func printf
regular_func printf_none_assert
#ifndef __riscv
    push {lr}       // keep stack trace sane
    ldr r0, =str
    bl panic
#else
    la a0, str
    call panic
#endif

str:
    .asciz "printf support is disabled"