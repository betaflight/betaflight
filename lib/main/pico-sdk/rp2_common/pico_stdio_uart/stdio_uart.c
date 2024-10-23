/*
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "pico/stdio/driver.h"
#include "pico/stdio_uart.h"
#include "pico/binary_info.h"
#include "hardware/gpio.h"

static uart_inst_t *uart_instance;

#if PICO_STDIO_UART_SUPPORT_CHARS_AVAILABLE_CALLBACK
static void (*chars_available_callback)(void*);
static void *chars_available_param;
#endif

#if PICO_NO_BI_STDIO_UART
#define stdio_bi_decl_if_func_used(x)
#else
#define stdio_bi_decl_if_func_used bi_decl_if_func_used
#endif

#ifdef PICO_DEFAULT_UART_TX_PIN
#if (PICO_DEFAULT_UART_TX_PIN & 0x1) || (PICO_RP2040 && (PICO_DEFAULT_UART_TX_PIN & 0x2))
#error "Specified PICO_DEFAULT_UART_TX_PIN does not support UART TX"
#endif
#endif

#ifdef PICO_DEFAULT_UART_RX_PIN
#if !(PICO_DEFAULT_UART_RX_PIN & 0x1) || (PICO_RP2040 && (PICO_DEFAULT_UART_TX_PIN & 0x2))
#error "Specified PICO_DEFAULT_UART_RX_PIN does not support UART RX"
#endif
#endif

void stdio_uart_init(void) {
#ifdef uart_default
    int tx_pin = -1;
    int rx_pin = -1;
#ifdef PICO_DEFAULT_UART_TX_PIN
    tx_pin = PICO_DEFAULT_UART_TX_PIN;
#ifdef PICO_DEFAULT_UART_RX_PIN
    rx_pin = PICO_DEFAULT_UART_RX_PIN;
    stdio_bi_decl_if_func_used(bi_program_feature("UART stdin / stdout"));
#if PICO_DEFAULT_UART_TX_PIN == PICO_DEFAULT_UART_RX_PIN
    bi_decl_if_func_used(bi_2pins_with_func(PICO_DEFAULT_UART_RX_PIN, PICO_DEFAULT_UART_TX_PIN, uart_get_funcsel(uart_default, PICO_DEFAULT_UART_RX_PIN)));
#else
    bi_decl_if_func_used(bi_1pin_with_func(PICO_DEFAULT_UART_TX_PIN, UART_FUNCSEL_NUM(uart_default, PICO_DEFAULT_UART_TX_PIN)));
    bi_decl_if_func_used(bi_1pin_with_func(PICO_DEFAULT_UART_RX_PIN, UART_FUNCSEL_NUM(uart_default, PICO_DEFAULT_UART_RX_PIN)));
#endif
#else
    stdio_bi_decl_if_func_used(bi_program_feature("UART stdout"));
    bi_decl_if_func_used(bi_1pin_with_func(PICO_DEFAULT_UART_TX_PIN, GPIO_FUNC_UART));
#endif
#elif defined(PICO_DEFAULT_UART_RX_PIN)
    rx_pin = PICO_DEFAULT_UART_RX_PIN;
    stdio_bi_decl_if_func_used(bi_program_feature("UART stdin"));
    bi_decl_if_func_used(bi_1pin_with_func(PICO_DEFAULT_UART_RX_PIN, GPIO_FUNC_UART));
#endif
#if !defined(PICO_DEFAULT_UART_BAUD_RATE)
    panic("UART baud rate undefined");
#else
    stdio_uart_init_full(uart_default, PICO_DEFAULT_UART_BAUD_RATE, tx_pin, rx_pin);
#endif
#endif
}

void stdout_uart_init(void) {
#if defined(uart_default) && defined(PICO_DEFAULT_UART_TX_PIN)
    bi_decl_if_func_used(bi_1pin_with_func(PICO_DEFAULT_UART_TX_PIN, UART_FUNCSEL_NUM(uart_default, PICO_DEFAULT_UART_TX_PIN)));
#if !defined(PICO_DEFAULT_UART_BAUD_RATE)
    panic("UART baud rate undefined");
#else
    stdio_bi_decl_if_func_used(bi_program_feature("UART stdout"));
    stdio_uart_init_full(uart_default, PICO_DEFAULT_UART_BAUD_RATE, PICO_DEFAULT_UART_TX_PIN, -1);
#endif
#endif
}

void stdin_uart_init(void) {
#if defined(uart_default) && defined(PICO_DEFAULT_UART_RX_PIN)
    bi_decl_if_func_used(bi_1pin_with_func(PICO_DEFAULT_UART_RX_PIN, UART_FUNCSEL_NUM(uart_default, PICO_DEFAULT_UART_RX_PIN)));
#if !defined(PICO_DEFAULT_UART_BAUD_RATE)
    panic("UART baud rate undefined");
#else
    stdio_bi_decl_if_func_used(bi_program_feature("UART stdin"));
    stdio_uart_init_full(uart_default, PICO_DEFAULT_UART_BAUD_RATE, -1, PICO_DEFAULT_UART_RX_PIN);
#endif
#endif
}

void stdio_uart_init_full(struct uart_inst *uart, uint baud_rate, int tx_pin, int rx_pin) {
    uart_instance = uart;
    if (tx_pin >= 0) gpio_set_function((uint)tx_pin, UART_FUNCSEL_NUM(uart, tx_pin));
    if (rx_pin >= 0) gpio_set_function((uint)rx_pin, UART_FUNCSEL_NUM(uart, rx_pin));
    uart_init(uart_instance, baud_rate);
    stdio_set_driver_enabled(&stdio_uart, true);
}

void stdio_uart_deinit(void) {
#ifdef uart_default
    int tx_pin = -1;
    int rx_pin = -1;
#ifdef PICO_DEFAULT_UART_TX_PIN
    tx_pin = PICO_DEFAULT_UART_TX_PIN;
#endif
#ifdef PICO_DEFAULT_UART_RX_PIN
    rx_pin = PICO_DEFAULT_UART_RX_PIN;
#endif
    stdio_uart_deinit_full(uart_default, tx_pin, rx_pin);
#endif
}

void stdout_uart_deinit(void) {
#if defined(uart_default) && defined(PICO_DEFAULT_UART_TX_PIN)
    stdio_uart_deinit_full(uart_default, PICO_DEFAULT_UART_TX_PIN, -1);
#endif
}

void stdin_uart_deinit(void) {
#if defined(uart_default) && defined(PICO_DEFAULT_UART_RX_PIN)
    stdio_uart_deinit_full(uart_default, -1, PICO_DEFAULT_UART_RX_PIN);
#endif
}

void stdio_uart_deinit_full(struct uart_inst *uart, int tx_pin, int rx_pin) {
    uart_instance = uart;
    stdio_set_driver_enabled(&stdio_uart, false);
    uart_deinit(uart_instance);
#if PICO_RP2040
    ((void)tx_pin);
    ((void)rx_pin);
#else
    // Leave pads isolated
    if (tx_pin >= 0) hw_set_bits(&pads_bank0_hw->io[tx_pin], PADS_BANK0_GPIO0_ISO_BITS);
    if (rx_pin >= 0) hw_set_bits(&pads_bank0_hw->io[rx_pin], PADS_BANK0_GPIO0_ISO_BITS);
#endif
}

static void stdio_uart_out_chars(const char *buf, int length) {
    for (int i = 0; i <length; i++) {
        uart_putc(uart_instance, buf[i]);
    }
}

int stdio_uart_in_chars(char *buf, int length) {
    int i=0;
    while (i<length && uart_is_readable(uart_instance)) {
        buf[i++] = uart_getc(uart_instance);
    }
#if PICO_STDIO_UART_SUPPORT_CHARS_AVAILABLE_CALLBACK
    if (chars_available_callback) {
        // Re-enable interrupts after reading a character
        uart_set_irqs_enabled(uart_instance, true, false);
    }
#endif
    return i ? i : PICO_ERROR_NO_DATA;
}

#if PICO_STDIO_UART_SUPPORT_CHARS_AVAILABLE_CALLBACK
static void on_uart_rx(void) {
    if (chars_available_callback) {
        // Interrupts will go off until the uart is read, so disable them
        uart_set_irqs_enabled(uart_instance, false, false);
        chars_available_callback(chars_available_param);
    }
}

#include "hardware/irq.h"

static void stdio_uart_set_chars_available_callback(void (*fn)(void*), void *param) {
    uint irq_num = UART_IRQ_NUM(uart_instance);
    if (fn && !chars_available_callback) {
        irq_set_exclusive_handler(irq_num, on_uart_rx);
        irq_set_enabled(irq_num, true);
        uart_set_irqs_enabled(uart_instance, true, false);
    } else if (!fn && chars_available_callback) {
        uart_set_irqs_enabled(uart_instance, false, false);
        irq_set_enabled(irq_num, false);
        irq_remove_handler(irq_num, on_uart_rx);
    }
    chars_available_callback = fn;
    chars_available_param = param;
}
#endif

static void stdio_uart_out_flush(void) {
    uart_default_tx_wait_blocking();
}

stdio_driver_t stdio_uart = {
    .out_chars = stdio_uart_out_chars,
    .out_flush = stdio_uart_out_flush,
    .in_chars = stdio_uart_in_chars,
#if PICO_STDIO_UART_SUPPORT_CHARS_AVAILABLE_CALLBACK
    .set_chars_available_callback = stdio_uart_set_chars_available_callback,
#endif
#if PICO_STDIO_ENABLE_CRLF_SUPPORT
    .crlf_enabled = PICO_STDIO_UART_DEFAULT_CRLF
#endif
};
