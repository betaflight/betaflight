/*
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <string.h>
#include <stdio.h>
#include <stdarg.h>

#include "pico.h"
#if LIB_PICO_PRINTF_PICO
#include "pico/printf.h"
#endif
#include "pico/stdio.h"
#include "pico/stdio/driver.h"
#include "pico/time.h"
#if PICO_STDOUT_MUTEX
#include "pico/mutex.h"
#endif

#if LIB_PICO_STDIO_UART
#include "pico/stdio_uart.h"
#endif

#if LIB_PICO_STDIO_USB
#include "pico/stdio_usb.h"
#endif

#if LIB_PICO_STDIO_SEMIHOSTING
#include "pico/stdio_semihosting.h"
#endif

#if LIB_PICO_STDIO_RTT
#include "pico/stdio_rtt.h"
#endif

static stdio_driver_t *drivers;
static stdio_driver_t *filter;

#if PICO_STDOUT_MUTEX
auto_init_mutex(print_mutex);

bool stdout_serialize_begin(void) {
    return mutex_try_enter_block_until(&print_mutex, make_timeout_time_ms(PICO_STDIO_DEADLOCK_TIMEOUT_MS));
}

void stdout_serialize_end(void) {
    mutex_exit(&print_mutex);
}

#else
static bool stdout_serialize_begin(void) {
    return true;
}
static void stdout_serialize_end(void) {
}
#endif
static void stdio_out_chars_no_crlf(stdio_driver_t *driver, const char *s, int len) {
    driver->out_chars(s, len);
}

static void stdio_out_chars_crlf(stdio_driver_t *driver, const char *s, int len) {
#if PICO_STDIO_ENABLE_CRLF_SUPPORT
    if (!driver->crlf_enabled) {
        driver->out_chars(s, len);
        return;
    }
    int first_of_chunk = 0;
    static const char crlf_str[] = {'\r', '\n'};
    for (int i = 0; i < len; i++) {
        bool prev_char_was_cr = i > 0 ? s[i - 1] == '\r' : driver->last_ended_with_cr;
        if (s[i] == '\n' && !prev_char_was_cr) {
            if (i > first_of_chunk) {
                driver->out_chars(&s[first_of_chunk], i - first_of_chunk);
            }
            driver->out_chars(crlf_str, 2);
            first_of_chunk = i + 1;
        }
    }
    if (first_of_chunk < len) {
        driver->out_chars(&s[first_of_chunk], len - first_of_chunk);
    }
    if (len > 0) {
        driver->last_ended_with_cr = s[len - 1] == '\r';
    }
#else
    driver->out_chars(s, len);
#endif
}

int stdio_put_string(const char *s, int len, bool newline, bool cr_translation) {
    bool serialized = stdout_serialize_begin();
    if (!serialized) {
#if PICO_STDIO_IGNORE_NESTED_STDOUT
        return 0;
#endif
    }
    if (len == -1) len = (int)strlen(s);
    void (*out_func)(stdio_driver_t *, const char *, int) = cr_translation ? stdio_out_chars_crlf : stdio_out_chars_no_crlf;
    for (stdio_driver_t *driver = drivers; driver; driver = driver->next) {
        if (!driver->out_chars) continue;
        if (filter && filter != driver) continue;
        out_func(driver, s, len);
        if (newline) {
            const char c = '\n';
            out_func(driver, &c, 1);
        }
    }
    if (serialized) {
        stdout_serialize_end();
    }
    return len;
}

int stdio_get_until(char *buf, int len, absolute_time_t until) {
    do {
        // todo round robin might be nice on each call, but then again hopefully
        //  no source will starve the others
        for (stdio_driver_t *driver = drivers; driver; driver = driver->next) {
            if (filter && filter != driver) continue;
            if (driver->in_chars) {
                int read = driver->in_chars(buf, len);
                if (read > 0) {
                    return read;
                }
            }
        }
        if (time_reached(until)) {
            return PICO_ERROR_TIMEOUT;
        }
        // we sleep here in case the in_chars methods acquire mutexes or disable IRQs and
        // potentially starve out what they are waiting on (have seen this with USB)
        busy_wait_us(1);
    } while (true);
}

int stdio_putchar_raw(int c) {
    char cc = (char)c;
    stdio_put_string(&cc, 1, false, false);
    return c;
}

int stdio_puts_raw(const char *s) {
    int len = (int)strlen(s);
    stdio_put_string(s, len, true, false);
    stdio_flush();
    return len;
}

void stdio_set_driver_enabled(stdio_driver_t *driver, bool enable) {
    stdio_driver_t **prev = &drivers;
    while (*prev) {
        if (*prev == driver) {
            if (!enable) {
                *prev = driver->next;
                driver->next = NULL;
            }
            return;
        }
        prev = &(*prev)->next;
    }
    if (enable) {
        *prev = driver;
    }
}

void stdio_flush(void) {
    for (stdio_driver_t *d = drivers; d; d = d->next) {
        if (d->out_flush) d->out_flush();
    }
}

#if LIB_PICO_PRINTF_PICO
typedef struct stdio_stack_buffer {
    int used;
    char buf[PICO_STDIO_STACK_BUFFER_SIZE];
} stdio_stack_buffer_t;

static void stdio_stack_buffer_flush(stdio_stack_buffer_t *buffer) {
    if (buffer->used) {
        for (stdio_driver_t *d = drivers; d; d = d->next) {
            if (!d->out_chars) continue;
            if (filter && filter != d) continue;
            stdio_out_chars_crlf(d, buffer->buf, buffer->used);
        }
        buffer->used = 0;
    }
}

static void stdio_buffered_printer(char c, void *arg) {
    stdio_stack_buffer_t *buffer = (stdio_stack_buffer_t *)arg;
    if (buffer->used == PICO_STDIO_STACK_BUFFER_SIZE) {
        stdio_stack_buffer_flush(buffer);
    }
    buffer->buf[buffer->used++] = c;
}
#endif

bool stdio_init_all(void) {
    // todo add explicit custom, or registered although you can call stdio_enable_driver explicitly anyway
    // These are well known ones

    bool rc = false;
#if LIB_PICO_STDIO_UART
    stdio_uart_init();
    rc = true;
#endif

#if LIB_PICO_STDIO_SEMIHOSTING
    stdio_semihosting_init();
    rc = true;
#endif

#if LIB_PICO_STDIO_RTT
    stdio_rtt_init();
    rc = true;
#endif

#if LIB_PICO_STDIO_USB
    rc |= stdio_usb_init();
#endif
    return rc;
}

bool stdio_deinit_all(void) {
    // todo add explicit custom, or registered although you can call stdio_enable_driver explicitly anyway
    // These are well known ones

    // First flush, to make sure everything is printed
    stdio_flush();

    bool rc = false;
#if LIB_PICO_STDIO_UART
    stdio_uart_deinit();
    rc = true;
#endif

#if LIB_PICO_STDIO_SEMIHOSTING
    stdio_semihosting_deinit();
    rc = true;
#endif

#if LIB_PICO_STDIO_RTT
    stdio_rtt_deinit();
    rc = true;
#endif

#if LIB_PICO_STDIO_USB
    rc = stdio_usb_deinit();
#endif
    return rc;
}

int stdio_getchar_timeout_us(uint32_t timeout_us) {
    char buf[1];
    int rc = stdio_get_until(buf, sizeof(buf), make_timeout_time_us(timeout_us));
    if (rc < 0) return rc;
    assert(rc);
    return (uint8_t)buf[0];
}

void stdio_filter_driver(stdio_driver_t *driver) {
    filter = driver;
}

void stdio_set_translate_crlf(stdio_driver_t *driver, bool enabled) {
#if PICO_STDIO_ENABLE_CRLF_SUPPORT
    if (enabled && !driver->crlf_enabled) {
        driver->last_ended_with_cr = false;
    }
    driver->crlf_enabled = enabled;
#else
    // Suppress -Wunused-parameter
    (void)driver;
    (void)enabled;

    panic_unsupported();
#endif
}

void stdio_set_chars_available_callback(void (*fn)(void*), void *param) {
    for (stdio_driver_t *s = drivers; s; s = s->next) {
        if (s->set_chars_available_callback) s->set_chars_available_callback(fn, param);
    }
}

#if PICO_STDIO_SHORT_CIRCUIT_CLIB_FUNCS
#define PRIMARY_STDIO_FUNC(x) WRAPPER_FUNC(x)
#else
#define PRIMARY_STDIO_FUNC(x) stdio_ ## x
#endif

int PRIMARY_STDIO_FUNC(getchar)(void) {
    char buf[1];
    int len = stdio_get_until(buf, 1, at_the_end_of_time);
    if (len < 0) return len;
    assert(len == 1);
    return (uint8_t)buf[0];
}

int PRIMARY_STDIO_FUNC(putchar)(int c) {
    char cc = (char)c;
    stdio_put_string(&cc, 1, false, true);
    return c;
}

int PRIMARY_STDIO_FUNC(puts)(const char *s) {
    int len = (int)strlen(s);
    stdio_put_string(s, len, true, true);
    stdio_flush();
    return len;
}

int REAL_FUNC(vprintf)(const char *format, va_list va);

int PRIMARY_STDIO_FUNC(vprintf)(const char *format, va_list va) {
    bool serialzed = stdout_serialize_begin();
    if (!serialzed) {
#if PICO_STDIO_IGNORE_NESTED_STDOUT
        return 0;
#endif
    }
    int ret;
#if LIB_PICO_PRINTF_PICO
    struct stdio_stack_buffer buffer;
    buffer.used = 0;
    ret = vfctprintf(stdio_buffered_printer, &buffer, format, va);
    stdio_stack_buffer_flush(&buffer);
    stdio_flush();
#elif LIB_PICO_PRINTF_NONE
    ((void)format);
    ((void)va);
    extern void printf_none_assert(void);
    printf_none_assert();
    ret = 0;
#else
    ret = REAL_FUNC(vprintf)(format, va);
#endif
    if (serialzed) {
        stdout_serialize_end();
    }
    return ret;
}

int __printflike(1, 0) PRIMARY_STDIO_FUNC(printf)(const char* format, ...)
{
    va_list va;
    va_start(va, format);
    int ret = vprintf(format, va);
    va_end(va);
    return ret;
}

#if PICO_STDIO_SHORT_CIRCUIT_CLIB_FUNCS
// define the stdio_ versions to be the same as our wrappers
int stdio_getchar(void) __attribute__((alias(__XSTRING(WRAPPER_FUNC(getchar)))));
int stdio_putchar(int) __attribute__((alias(__XSTRING(WRAPPER_FUNC(putchar)))));
int stdio_puts(const char *s) __attribute__((alias(__XSTRING(WRAPPER_FUNC(puts)))));
int stdio_vprintf(const char *format, va_list va) __attribute__((alias(__XSTRING(WRAPPER_FUNC(vprintf)))));
int __printflike(1, 0) stdio_printf(const char* format, ...) __attribute__((alias(__XSTRING(WRAPPER_FUNC(printf)))));
#else
// todo there is no easy way to avoid the wrapper functions since they are in the CMake, so lets just forward for now

int REAL_FUNC(getchar)(void);
int REAL_FUNC(putchar)(int);
int REAL_FUNC(puts)(const char *s);
int __printflike(1, 0) REAL_FUNC(printf)(const char* format, ...);

int WRAPPER_FUNC(getchar)(void) {
    return REAL_FUNC(getchar)();
}
int WRAPPER_FUNC(putchar)(int c) {
    return REAL_FUNC(putchar)(c);
}
int WRAPPER_FUNC(puts)(const char *s) {
    return REAL_FUNC(puts)(s);
}
int WRAPPER_FUNC(vprintf)(const char *format, va_list va) {
    return REAL_FUNC(vprintf)(format, va);
}
int __printflike(1, 0) WRAPPER_FUNC(printf)(const char* format, ...) {
    va_list va;
    va_start(va, format);
    int ret = REAL_FUNC(vprintf)(format, va);
    va_end(va);
    return ret;
}
#endif


