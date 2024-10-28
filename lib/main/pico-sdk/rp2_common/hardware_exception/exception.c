/*
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "hardware/exception.h"
#include "hardware/sync.h"
#include "pico/platform/cpu_regs.h"

#ifndef exception_is_compile_time_default
static bool exception_is_compile_time_default(exception_handler_t handler) {
#ifdef __riscv
    extern char __unhandled_exception;
    return (uintptr_t)handler == (uintptr_t)__unhandled_exception;
#else
    extern char __default_isrs_start;
    extern char __default_isrs_end;
    return ((uintptr_t)handler) >= (uintptr_t)&__default_isrs_start &&
            ((uintptr_t)handler) < (uintptr_t)&__default_isrs_end;
#endif
}
#endif

static inline exception_handler_t *get_exception_table(void) {
#ifdef __riscv
    extern uintptr_t __riscv_exception_table;
    return (exception_handler_t *) &__riscv_exception_table;
#else
    return (exception_handler_t *) scb_hw->vtor;
#endif
}

static void set_raw_exception_handler_and_restore_interrupts(enum exception_number num, exception_handler_t handler, uint32_t save) {
    // update vtable (vtable_handler may be same or updated depending on cases, but we do it anyway for compactness)
    get_exception_table()[num] = handler;
    __dmb();
    restore_interrupts_from_disabled(save);
}

static inline void check_exception_param(__unused enum exception_number num) {
    invalid_params_if(HARDWARE_EXCEPTION, num < MIN_EXCEPTION_NUM || num > MAX_EXCEPTION_NUM);
}

exception_handler_t exception_get_vtable_handler(enum exception_number num) {
    check_exception_param(num);
    return get_exception_table()[num];
}

exception_handler_t exception_set_exclusive_handler(enum exception_number num, exception_handler_t handler) {
    check_exception_param(num);
#if !PICO_NO_RAM_VECTOR_TABLE
    uint32_t save = save_and_disable_interrupts();
    exception_handler_t current = exception_get_vtable_handler(num);
    hard_assert(handler == current || exception_is_compile_time_default(current));
    set_raw_exception_handler_and_restore_interrupts(num, handler, save);
#else
    panic_unsupported();
#endif
    return current;
}

void exception_restore_handler(enum exception_number num, exception_handler_t original_handler) {
    hard_assert(exception_is_compile_time_default(original_handler));
#if !PICO_NO_RAM_VECTOR_TABLE
    uint32_t save = save_and_disable_interrupts();
    set_raw_exception_handler_and_restore_interrupts(num, original_handler, save);
#else
    panic_unsupported();
#endif
}

#ifndef __riscv

static io_rw_32 *get_shpr0(uint num) {
    io_rw_32 *shpr0 = NULL;
#if __ARM_ARCH_6M__
    // only has shpr2-3
    if (num >= 8 && num < 16) shpr0 = (io_rw_32 *) (PPB_BASE + ARM_CPU_PREFIXED(SHPR2_OFFSET) - 8);
#elif __ARM_ARCH_8M_MAIN__
    // only has shpr1-3
    if (num >= 4 && num < 16) shpr0 = (io_rw_32 *)(PPB_BASE + ARM_CPU_PREFIXED(SHPR1_OFFSET) - 4);
#endif
    return shpr0;
}

bool exception_set_priority(uint num, uint8_t hardware_priority) {
    io_rw_32 *shpr0 = get_shpr0(num);
    if (shpr0) {
        // note that only 32 bit writes are supported
        shpr0[num / 4] = (shpr0[num/4] & ~(0xffu << (8 * (num & 3u)))) | (((uint32_t) hardware_priority) << (8 * (num & 3u)));
        return true;
    }
    return false;
}

uint exception_get_priority(uint num) {
    io_rw_32 *shpr0 = get_shpr0(num);
    if (shpr0) {
        // note that only 32 bit writes are supported
        return (uint8_t) (shpr0[num/4] >> (8 * (num & 3)));
    }
    return PICO_LOWEST_EXCEPTION_PRIORITY;
}
#endif