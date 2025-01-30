/*
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <math.h>
#include <stddef.h>
#include <sys/time.h>

#include <llvm-libc-types/ssize_t.h>

#include "pico/runtime_init.h"
#include "pico/stdio.h"
#include "pico/time.h"

#if PICO_ENTER_USB_BOOT_ON_EXIT
#include "pico/bootrom.h"
#endif

// These functions are defined by POSIX and not C standard.

static int64_t epoch_time_us_since_boot;

int gettimeofday(struct timeval *__restrict tv, __unused struct timezone *__restrict tz) {
    if (tv) {
        int64_t us_since_epoch = ((int64_t)to_us_since_boot(get_absolute_time())) - epoch_time_us_since_boot;
        tv->tv_sec = (time_t)(us_since_epoch / 1000000);
        tv->tv_usec = (suseconds_t)(us_since_epoch % 1000000);
    }
    return 0;
}

int settimeofday(__unused const struct timeval *tv, __unused const struct timezone *tz) {
    if (tv) {
        int64_t us_since_epoch = tv->tv_sec * 1000000 + tv->tv_usec;
        epoch_time_us_since_boot = (int64_t)to_us_since_boot(get_absolute_time()) - us_since_epoch;
    }
    return 0;
}

// TODO: This should be a thread-local variable.
int errno;

int *__llvm_libc_errno(void) { return &errno; }

struct __llvm_libc_stdio_cookie {};

struct __llvm_libc_stdio_cookie __llvm_libc_stdin_cookie;
struct __llvm_libc_stdio_cookie __llvm_libc_stdout_cookie;
struct __llvm_libc_stdio_cookie __llvm_libc_stderr_cookie;

ssize_t __llvm_libc_stdio_read(__unused void *cookie, char *buf, size_t size) {
    for (size_t i = 0; i < size; i++) {
        buf[i] = getchar_timeout_us(0);
    }
    return size;
}

ssize_t __llvm_libc_stdio_write(__unused void *cookie, const char *buf, size_t size) {
    // TODO: We would ideally use `stdio_put_string` from pico_stdio.
    for (size_t i = 0; i < size; i++) {
        putchar_raw(buf[i]);
    }
    return size;
}

void __cxa_finalize(__unused void *dso) {}

void __attribute__((noreturn)) __llvm_libc_exit(__unused int status) {
#if PICO_ENTER_USB_BOOT_ON_EXIT
    reset_usb_boot(0,0);
#else
    while (1) {
        __breakpoint();
    }
#endif
}

void _exit(int) __attribute__((noreturn, alias("__llvm_libc_exit")));

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

    // todo maybe we want to do this in the future, but it does stuff like register_tm_clones
    //      which we didn't do in previous SDKs
    //extern void __libc_init_array(void);
    //__libc_init_array();

    // ... so instead just do the __preinit_array
    runtime_run_initializers();
    // ... and the __init_array
    extern void (*__init_array_start)(void);
    extern void (*__init_array_end)(void);
    for (void (**p)(void) = &__init_array_start; p < &__init_array_end; ++p) {
        (*p)();
    }
}

