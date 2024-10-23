/*
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "pico/stdlib.h"
#if LIB_PICO_STDIO_UART
#include "pico/stdio_uart.h"
#else
#include "pico/binary_info.h"
#endif

void setup_default_uart(void) {
#if LIB_PICO_STDIO_UART
    stdio_uart_init();
#elif defined(PICO_DEFAULT_UART_BAUD_RATE) && defined(PICO_DEFAULT_UART_TX_PIN) && defined(PICO_DEFAULT_UART_RX_PIN)
    // this is mostly for backwards compatibility - stdio_uart_init is a bit more nuanced, and usually likely to be present
    uart_init(uart_default, PICO_DEFAULT_UART_BAUD_RATE);
    if (PICO_DEFAULT_UART_TX_PIN >= 0)
        gpio_set_function(PICO_DEFAULT_UART_TX_PIN, GPIO_FUNC_UART);
    if (PICO_DEFAULT_UART_RX_PIN >= 0)
        gpio_set_function(PICO_DEFAULT_UART_RX_PIN, GPIO_FUNC_UART);
    bi_decl_if_func_used(bi_2pins_with_func(PICO_DEFAULT_UART_RX_PIN, PICO_DEFAULT_UART_TX_PIN, GPIO_FUNC_UART));
#endif
}
