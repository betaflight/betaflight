/*
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#include <stdarg.h>
#include <stdio.h>

#include "pico.h"
#include "hardware/timer.h"

PICO_WEAK_FUNCTION_DEF(tight_loop_contents)
void PICO_WEAK_FUNCTION_IMPL_NAME(tight_loop_contents)() {

}


PICO_WEAK_FUNCTION_DEF(get_core_num)
uint PICO_WEAK_FUNCTION_IMPL_NAME(get_core_num)() {
    return 0;
}

void __noreturn panic_unsupported() {
    panic("not supported");
}

void panic(const char *fmt, ...) {
    va_list args;

    puts("*** PANIC ***\n");
    if (fmt) {
        va_start(args, fmt);
        vprintf(fmt, args);
        va_end(args);
    }

    puts("\n");

    __breakpoint();
}

void __breakpoint() {
    #ifdef _MSC_VER
        __debugbreak();
    #else
        __builtin_trap();
    #endif
}

PICO_WEAK_FUNCTION_DEF(busy_wait_at_least_cycles)
void PICO_WEAK_FUNCTION_IMPL_NAME(busy_wait_at_least_cycles)(uint32_t cycles) {
    // fairly arbitrary; we'll use 125Mhz as a reference
    busy_wait_us((cycles + 124)/125);
}
