/*
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _PICO_STDLIB_H
#define _PICO_STDLIB_H

#include "pico.h"
#include "pico/stdio.h"
#include "pico/time.h"
#include "hardware/gpio.h"
#include "hardware/uart.h"

#ifdef __cplusplus
extern "C" {
#endif

/** \file stdlib.h
 *  \defgroup pico_stdlib pico_stdlib
 *
 * \brief Aggregation of a core subset of Raspberry Pi Pico SDK libraries used by most executables along with some additional
 * utility methods
 *
 * Including pico_stdlib gives you everything you need to get a basic program running
 * which prints to stdout or flashes a LED
 *
 * This library aggregates:
 *   - @ref hardware_divider
 *   - @ref hardware_gpio
 *   - @ref hardware_uart
 *   - @ref pico_runtime
 *   - @ref pico_platform
 *   - @ref pico_stdio
 *   - @ref pico_time
 *   - @ref pico_util
 *
 * There are some basic default values used by these functions that will default to
 * usable values, however, they can be customised in a board definition header via
 * config.h or similar
 */

// Note PICO_STDIO_UART, PICO_STDIO_USB, PICO_STDIO_SEMIHOSTING are set by the
// respective INTERFACE libraries, so these defines are set if the library
// is included for the target executable

#if LIB_PICO_STDIO_UART
#include "pico/stdio_uart.h"
#endif

#if LIB_PICO_STDIO_USB
#include "pico/stdio_usb.h"
#endif

#if LIB_PICO_STDIO_SEMIHOSTING
#include "pico/stdio_semihosting.h"
#endif

// PICO_CONFIG: PICO_DEFAULT_LED_PIN, Optionally define a pin that drives a regular LED on the board, default=Usually provided via board header, group=pico_stdlib

// PICO_CONFIG: PICO_DEFAULT_LED_PIN_INVERTED, 1 if LED is inverted or 0 if not, type=int, default=0, group=pico_stdlib
#ifndef PICO_DEFAULT_LED_PIN_INVERTED
#define PICO_DEFAULT_LED_PIN_INVERTED 0
#endif

// PICO_CONFIG: PICO_DEFAULT_WS2812_PIN, Optionally define a pin that controls data to a WS2812 compatible LED on the board, group=pico_stdlib
// PICO_CONFIG: PICO_DEFAULT_WS2812_POWER_PIN, Optionally define a pin that controls power to a WS2812 compatible LED on the board, group=pico_stdlib

/*! \brief Set up the default UART and assign it to the default GPIOs
 *  \ingroup pico_stdlib
 *
 * By default this will use UART 0, with TX to pin GPIO 0,
 * RX to pin GPIO 1, and the baudrate to 115200
 *
 * Calling this method also initializes stdin/stdout over UART if the
 * @ref pico_stdio_uart library is linked.
 *
 * Defaults can be changed using configuration defines,
 *  PICO_DEFAULT_UART_INSTANCE,
 *  PICO_DEFAULT_UART_BAUD_RATE
 *  PICO_DEFAULT_UART_TX_PIN
 *  PICO_DEFAULT_UART_RX_PIN
 */
void setup_default_uart(void);

#ifdef __cplusplus
}
#endif
#endif
