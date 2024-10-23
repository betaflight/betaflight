/*
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _PICO_STDIO_H
#define _PICO_STDIO_H

/** \file stdio.h
*  \defgroup pico_stdio pico_stdio
* \brief Customized stdio support allowing for input and output from UART, USB, semi-hosting etc
*
* Note the API for adding additional input output devices is not yet considered stable
*/

#include "pico.h"

// PICO_CONFIG: PICO_STDOUT_MUTEX, Enable/disable mutex around stdout, type=bool, default=1, group=pico_stdio
#ifndef PICO_STDOUT_MUTEX
#define PICO_STDOUT_MUTEX 1
#endif

// PICO_CONFIG: PICO_STDIO_ENABLE_CRLF_SUPPORT, Enable/disable CR/LF output conversion support, type=bool, default=1, group=pico_stdio
#ifndef PICO_STDIO_ENABLE_CRLF_SUPPORT
#define PICO_STDIO_ENABLE_CRLF_SUPPORT 1
#endif

// PICO_CONFIG: PICO_STDIO_DEFAULT_CRLF, Default for CR/LF conversion enabled on all stdio outputs, type=bool, default=1, depends=PICO_STDIO_ENABLE_CRLF_SUPPORT, group=pico_stdio
#ifndef PICO_STDIO_DEFAULT_CRLF
#define PICO_STDIO_DEFAULT_CRLF 1
#endif

// PICO_CONFIG: PICO_STDIO_STACK_BUFFER_SIZE, Define printf buffer size (on stack)... this is just a working buffer not a max output size, min=0, max=512, default=128, group=pico_stdio
#ifndef PICO_STDIO_STACK_BUFFER_SIZE
#define PICO_STDIO_STACK_BUFFER_SIZE 128
#endif

// PICO_CONFIG: PICO_STDIO_DEADLOCK_TIMEOUT_MS, Time after which to assume stdio_usb is deadlocked by use in IRQ and give up, type=int, default=1000, group=pico_stdio
#ifndef PICO_STDIO_DEADLOCK_TIMEOUT_MS
#define PICO_STDIO_DEADLOCK_TIMEOUT_MS 1000
#endif

// PICO_CONFIG: PICO_STDIO_SHORT_CIRCUIT_CLIB_FUNCS, Directly replace common stdio functions such as putchar from the C-library to avoid pulling in lots of c library code for simple output, type=bool, default=1, advanced=true, group=pico_stdio
#ifndef PICO_STDIO_SHORT_CIRCUIT_CLIB_FUNCS
#define PICO_STDIO_SHORT_CIRCUIT_CLIB_FUNCS 1
#endif

#ifdef __cplusplus
extern "C" {
#endif

#include <stdarg.h>

typedef struct stdio_driver stdio_driver_t;

/*! \brief Initialize all of the present standard stdio types that are linked into the binary.
 * \ingroup pico_stdio
 *
 * Call this method once you have set up your clocks to enable the stdio support for UART, USB,
 * semihosting, and RTT based on the presence of the respective libraries in the binary.
 *
 * When stdio_usb is configured, this method can be optionally made to block, waiting for a connection
 * via the variables specified in \ref stdio_usb_init (i.e. \ref PICO_STDIO_USB_CONNECT_WAIT_TIMEOUT_MS)
 *
 * \return true if at least one output was successfully initialized, false otherwise.
 * \see stdio_uart, stdio_usb, stdio_semihosting, stdio_rtt
 */
bool stdio_init_all(void);

/*! \brief Deinitialize all of the present standard stdio types that are linked into the binary.
 * \ingroup pico_stdio
 *
 * This method currently only supports stdio_uart and stdio_semihosting
 *
 * \return true if all outputs was successfully deinitialized, false otherwise.
 * \see stdio_uart, stdio_usb, stdio_semihosting, stdio_rtt
 */
bool stdio_deinit_all(void);

/*! \brief Flushes any buffered output.
 * \ingroup pico_stdio
 */
void stdio_flush(void);

/*! \brief Return a character from stdin if there is one available within a timeout
 * \ingroup pico_stdio
 *
 * \param timeout_us the timeout in microseconds, or 0 to not wait for a character if none available.
 * \return the character from 0-255 or PICO_ERROR_TIMEOUT if timeout occurs
 */
int stdio_getchar_timeout_us(uint32_t timeout_us);

/*! \brief Alias for \ref stdio_getchar_timeout_us for backwards compatibility
 * \ingroup pico_stdio
 */
static inline int getchar_timeout_us(uint32_t timeout_us) {
    return stdio_getchar_timeout_us(timeout_us);
}

/*! \brief Adds or removes a driver from the list of active drivers used for input/output
 * \ingroup pico_stdio
 *
 * \note this method should always be called on an initialized driver and is not re-entrant
 * \param driver the driver
 * \param enabled true to add, false to remove
 */
void stdio_set_driver_enabled(stdio_driver_t *driver, bool enabled);

/*! \brief Control limiting of output to a single driver
 * \ingroup pico_stdio
 *
 * \note this method should always be called on an initialized driver
 *
 * \param driver if non-null then output only that driver will be used for input/output (assuming it is in the list of enabled drivers).
 *               if NULL then all enabled drivers will be used
 */
void stdio_filter_driver(stdio_driver_t *driver);

/*! \brief control conversion of line feeds to carriage return on transmissions
 * \ingroup pico_stdio
 *
 * \note this method should always be called on an initialized driver
 *
 * \param driver the driver
 * \param translate If true, convert line feeds to carriage return on transmissions
 */
void stdio_set_translate_crlf(stdio_driver_t *driver, bool translate);

/*! \brief putchar variant that skips any CR/LF conversion if enabled
 * \ingroup pico_stdio
 */
int stdio_putchar_raw(int c);

/*! \brief Alias for \ref stdio_putchar_raw for backwards compatibility
 * \ingroup pico_stdio
 */
static inline int putchar_raw(int c) {
    return stdio_putchar_raw(c);
}

/*! \brief puts variant that skips any CR/LF conversion if enabled
 * \ingroup pico_stdio
 */
int stdio_puts_raw(const char *s);

/*! \brief Alias for \ref stdio_puts_raw for backwards compatibility
 * \ingroup pico_stdio
 */
static inline int puts_raw(const char *s) {
    return stdio_puts_raw(s);
}

/*! \brief get notified when there are input characters available
 * \ingroup pico_stdio
 *
 * \param fn Callback function to be called when characters are available. Pass NULL to cancel any existing callback
 * \param param Pointer to pass to the callback
 */
void stdio_set_chars_available_callback(void (*fn)(void*), void *param);

/*! \brief Waits until a timeout to reard at least one character into a buffer
 * \ingroup pico_stdio
 *
 * This method returns as soon as input is available, but more characters may
 * be returned up to the end of the buffer.
 *
 * \param buf the buffer to read into
 * \param len the length of the buffer
 * \return the number of characters read or PICO_ERROR_TIMEOUT
 * \param until the time after which to return PICO_ERROR_TIMEOUT if no characters are available
 */
int stdio_get_until(char *buf, int len, absolute_time_t until);

/*! \brief Prints a buffer to stdout with optional newline and carriage return insertion
 * \ingroup pico_stdio
 *
 * This method returns as soon as input is available, but more characters may
 * be returned up to the end of the buffer.
 *
 * \param s the characters to print
 * \param len the length of s
 * \param newline true if a newline should be added after the string
 * \param cr_translation true if line feed to carriage return translation should be performed
 * \return the number of characters written
 */
int stdio_put_string(const char *s, int len, bool newline, bool cr_translation);

/*! \brief stdio_getchar Alias for \ref getchar that definitely does not go thru the implementation
 * in the standard C library even when \ref PICO_STDIO_SHORT_CIRCUIT_CLIB_FUNCS == 0
 *
 * \ingroup pico_stdio
 */
int stdio_getchar(void);

/*! \brief stdio_getchar Alias for \ref putchar that definitely does not go thru the implementation
 * in the standard C library even when \ref PICO_STDIO_SHORT_CIRCUIT_CLIB_FUNCS == 0
 *
 * \ingroup pico_stdio
 */
int stdio_putchar(int);

/*! \brief stdio_getchar Alias for \ref puts that definitely does not go thru the implementation
 * in the standard C library even when \ref PICO_STDIO_SHORT_CIRCUIT_CLIB_FUNCS == 0
 *
 * \ingroup pico_stdio
 */
int stdio_puts(const char *s);

/*! \brief stdio_getchar Alias for \ref vprintf that definitely does not go thru the implementation
 * in the standard C library even when \ref PICO_STDIO_SHORT_CIRCUIT_CLIB_FUNCS == 0
 *
 * \ingroup pico_stdio
 */
int stdio_vprintf(const char *format, va_list va);

/*! \brief stdio_getchar Alias for \ref printf that definitely does not go thru the implementation
 * in the standard C library even when \ref PICO_STDIO_SHORT_CIRCUIT_CLIB_FUNCS == 0
 *
 * \ingroup pico_stdio
 */
int __printflike(1, 0) stdio_printf(const char* format, ...);

#ifdef __cplusplus
}
#endif

#endif
