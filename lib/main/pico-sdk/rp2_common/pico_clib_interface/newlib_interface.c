/*
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include <stdarg.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/times.h>
#include <time.h>
#include <unistd.h>
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
#if LIB_PICO_STDIO
#include "pico/stdio.h"
#endif

#if PICO_ENTER_USB_BOOT_ON_EXIT
#include "pico/bootrom.h"
#endif

extern char __StackLimit; /* Set by linker.  */

#define STDIO_HANDLE_STDIN  0
#define STDIO_HANDLE_STDOUT 1
#define STDIO_HANDLE_STDERR 2

void __attribute__((noreturn)) __weak _exit(__unused int status) {
#if PICO_ENTER_USB_BOOT_ON_EXIT
    reset_usb_boot(0,0);
#else
    while (1) {
        __breakpoint();
    }
#endif
}

__weak void *_sbrk(int incr) {
    extern char end; /* Set by linker.  */
    static char *heap_end;
    char *prev_heap_end;

    if (heap_end == 0)
        heap_end = &end;

    prev_heap_end = heap_end;
    char *next_heap_end = heap_end + incr;

    if (__builtin_expect(next_heap_end > (&__StackLimit), false)) {
#if PICO_USE_OPTIMISTIC_SBRK
        if (heap_end == &__StackLimit) {
//        errno = ENOMEM;
            return (char *) -1;
        }
        next_heap_end = &__StackLimit;
#else
        return (char *) -1;
#endif
    }

    heap_end = next_heap_end;
    return (void *) prev_heap_end;
}

static int64_t epoch_time_us_since_boot;

__weak int _gettimeofday (struct timeval *__restrict tv, __unused void *__restrict tz) {
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

__weak int _times(struct tms *tms) {
#if CLOCKS_PER_SEC >= 1000000
    tms->tms_utime = (clock_t)(to_us_since_boot(get_absolute_time()) * (CLOCKS_PER_SEC / 1000000));
#else
    tms->tms_utime = (clock_t)(to_us_since_boot(get_absolute_time()) / (1000000 / CLOCKS_PER_SEC));
#endif
    tms->tms_stime = 0;
    tms->tms_cutime = 0;
    tms->tms_cstime = 0;
    return 0;
}

__weak pid_t _getpid(void) {
    return 0;
}

__weak int _kill(__unused pid_t pid, __unused int sig) {
    return -1;
}

int __attribute__((weak)) _read(int handle, char *buffer, int length) {
#if LIB_PICO_STDIO
    if (handle == STDIO_HANDLE_STDIN) {
        return stdio_get_until(buffer, length, at_the_end_of_time);
    }
#endif
    return -1;
}

int __attribute__((weak)) _write(int handle, char *buffer, int length) {
#if LIB_PICO_STDIO
    if (handle == STDIO_HANDLE_STDOUT || handle == STDIO_HANDLE_STDERR) {
        stdio_put_string(buffer, length, false, true);
        return length;
    }
#endif
    return -1;
}

int __attribute__((weak)) _open(__unused const char *fn, __unused int oflag, ...) {
    return -1;
}

int __attribute__((weak)) _close(__unused int fd) {
    return -1;
}

off_t __attribute__((weak)) _lseek(__unused int fd, __unused off_t pos, __unused int whence) {
    return -1;
}

int __attribute__((weak)) _fstat(__unused int fd, __unused struct stat *buf) {
    return -1;
}

int __attribute__((weak)) _isatty(int fd) {
    return fd == STDIO_HANDLE_STDIN || fd == STDIO_HANDLE_STDOUT || fd == STDIO_HANDLE_STDERR;
}

// exit is not useful... no desire to pull in __call_exitprocs
void exit(int status) {
    _exit(status);
}

// incorrect warning from GCC 6
GCC_Pragma("GCC diagnostic push")
GCC_Pragma("GCC diagnostic ignored \"-Wsuggest-attribute=format\"")
void __weak __assert_func(const char *file, int line, const char *func, const char *failedexpr) {
    weak_raw_printf("assertion \"%s\" failed: file \"%s\", line %d%s%s\n",
                    failedexpr, file, line, func ? ", function: " : "",
                    func ? func : "");

    _exit(1);
}
GCC_Pragma("GCC diagnostic pop")

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