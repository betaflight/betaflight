/*
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include <stdarg.h>
#include <sys/cdefs.h>
#include <unistd.h>
#include "pico.h"
#include "pico/platform/panic.h"

#if LIB_PICO_PRINTF_PICO
#include "pico/printf.h"
#else
#define weak_raw_printf printf
#define weak_raw_vprintf vprintf
#endif

void __attribute__((noreturn)) panic_unsupported(void) {
    panic("not supported");
}

// PICO_CONFIG: PICO_PANIC_FUNCTION, Name of a function to use in place of the stock panic function or empty string to simply breakpoint on panic, group=pico_runtime
// note the default is not "panic" it is undefined
#ifdef PICO_PANIC_FUNCTION
#define PICO_PANIC_FUNCTION_EMPTY (__CONCAT(PICO_PANIC_FUNCTION, 1) == 1)
#if !PICO_PANIC_FUNCTION_EMPTY
extern void __attribute__((noreturn)) __printflike(1, 0) PICO_PANIC_FUNCTION(__unused const char *fmt, ...);
#endif
// Use a forwarding method here as it is a little simpler than renaming the symbol as it is used from assembler
void __attribute__((naked, noreturn)) __printflike(1, 0) panic(__unused const char *fmt, ...) {
    // if you get an undefined reference here, you didn't define your PICO_PANIC_FUNCTION!
    pico_default_asm (
#ifdef __riscv

#if !PICO_PANIC_FUNCTION_EMPTY
            "jal " __XSTRING(PICO_PANIC_FUNCTION) "\n"
#endif
            "ebreak\n"
            "1: j 1b\n"

#else

            "push {lr}\n"
#if !PICO_PANIC_FUNCTION_EMPTY
            "bl " __XSTRING(PICO_PANIC_FUNCTION) "\n"
#endif
            "bkpt #0\n"
            "1: b 1b\n" // loop for ever as we are no return

#endif
        :
        :
        :
    );
}
#else
// todo consider making this try harder to output if we panic early
//  right now, print mutex may be uninitialised (in which case it deadlocks - although after printing "PANIC")
//  more importantly there may be no stdout/UART initialized yet
// todo we may want to think about where we print panic messages to; writing to USB appears to work
//  though it doesn't seem like we can expect it to... fine for now
void __attribute__((noreturn)) __printflike(1, 0) panic(const char *fmt, ...) {
    puts("\n*** PANIC ***\n");
    if (fmt) {
#if LIB_PICO_PRINTF_NONE
        puts(fmt);
#else
        va_list args;
        va_start(args, fmt);
#if PICO_PRINTF_ALWAYS_INCLUDED
        vprintf(fmt, args);
#else
        weak_raw_vprintf(fmt, args);
#endif
        va_end(args);
        puts("\n");
#endif
    }

    _exit(1);
}
#endif
