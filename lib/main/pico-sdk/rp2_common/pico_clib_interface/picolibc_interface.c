/*
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/time.h>

#include "pico.h"
#if LIB_PICO_STDIO
#include "pico/stdio.h"
#endif

#if PICO_ENTER_USB_BOOT_ON_EXIT
#include "pico/bootrom.h"
#endif

#include "pico/time.h"
#include "pico/runtime_init.h"

#if LIB_PICO_PRINTF_PICO
#include "pico/printf.h"
#else
#define weak_raw_printf printf
#define weak_raw_vprintf vprintf
#endif

static int picolibc_putc(char c, __unused FILE *file) {
#if LIB_PICO_STDIO
    stdio_putchar(c);
#endif
    return c;
}

static int picolibc_getc(__unused FILE *file) {
#if LIB_PICO_STDIO
    return stdio_getchar();
#endif
}

static int picolibc_flush(__unused FILE *file) {
#if LIB_PICO_STDIO
    stdio_flush();
#endif
    return 0;
}

static FILE __stdio = FDEV_SETUP_STREAM(picolibc_putc,
                                        picolibc_getc,
                                        picolibc_flush,
                                        _FDEV_SETUP_RW);

FILE *const stdin = &__stdio; __strong_reference(stdin, stdout); __strong_reference(stdin, stderr);

void __weak __assert_func(const char *file, int line, const char *func, const char *failedexpr) {
    weak_raw_printf("assertion \"%s\" failed: file \"%s\", line %d%s%s\n",
                    failedexpr, file, line, func ? ", function: " : "",
                    func ? func : "");

    _exit(1);
}


void __attribute__((noreturn)) __weak _exit(__unused int status) {
#if PICO_ENTER_USB_BOOT_ON_EXIT
    reset_usb_boot(0,0);
#else
    while (1) {
        __breakpoint();
    }
#endif
}

static int64_t epoch_time_us_since_boot;

__weak int gettimeofday (struct timeval *__restrict tv, __unused void *__restrict tz) {
    if (tv) {
        int64_t us_since_epoch = ((int64_t)to_us_since_boot(get_absolute_time())) - epoch_time_us_since_boot;
        tv->tv_sec = (time_t)(us_since_epoch / 1000000);
        tv->tv_usec = (suseconds_t)(us_since_epoch % 1000000);
    }
    return 0;
}

__weak int settimeofday(__unused const struct timeval *tv, __unused const struct timezone *tz) {
    if (tv) {
        int64_t us_since_epoch = tv->tv_sec * 1000000 + tv->tv_usec;
        epoch_time_us_since_boot = (int64_t)to_us_since_boot(get_absolute_time()) - us_since_epoch;
    }
    return 0;
}


void runtime_init(void) {
#ifndef NDEBUG
    if (__get_current_exception()) {
        // crap; started in exception handler
        __breakpoint();
    }
#endif

#if !PICO_RUNTIME_SKIP_INIT_PER_CORE_INSTALL_STACK_GUARD
    // install core0 stack guard
    extern char __StackBottom;
    runtime_init_per_core_install_stack_guard(&__StackBottom);
#endif

    // piolibc __libc_init_array does __preint_array and __init_array
    extern void __libc_init_array(void);
    __libc_init_array();
}

#if !PICO_RUNTIME_NO_INIT_PER_CORE_TLS_SETUP
__weak void runtime_init_pre_core_tls_setup(void) {
    // for now we just set the same global area on both cores
    // note: that this is superfluous with the stock picolibc it seems, since it is itself
    // using a version of __aeabi_read_tp that returns the same pointer on both cores
    extern void __tls_base;
    extern void _set_tls(void *tls);
    _set_tls(&__tls_base);
}
#endif

#if !PICO_RUNTIME_SKIP_INIT_PER_CORE_TLS_SETUP
PICO_RUNTIME_INIT_FUNC_PER_CORE(runtime_init_pre_core_tls_setup, PICO_RUNTIME_INIT_PER_CORE_TLS_SETUP);
#endif

//// naked as it must preserve everything except r0 and lr
//uint32_t __attribute__((naked)) WRAPPER_FUNC(__aeabi_read_tp)() {
//    // note for now we are just returning a shared instance on both cores
//    pico_default_asm_volatile(
//            "ldr r0, =__tls_base\n"
//            "bx lr\n"
//            );
//}