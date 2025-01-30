/*
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "hardware/address_mapped.h"
#include "hardware/platform_defs.h"
#include "hardware/uart.h"

#include "hardware/structs/uart.h"
#include "hardware/resets.h"
#include "hardware/clocks.h"

static inline uint32_t uart_clock_get_hz(__unused uart_inst_t *inst) {
    return clock_get_hz(UART_CLOCK_NUM(inst));
}

#include "hardware/timer.h"

#include "pico/assert.h"
#include "pico.h"

check_hw_layout(uart_hw_t, fr, UART_UARTFR_OFFSET);
check_hw_layout(uart_hw_t, dmacr, UART_UARTDMACR_OFFSET);

#if PICO_UART_ENABLE_CRLF_SUPPORT
short uart_char_to_line_feed[NUM_UARTS];
#endif

/// \tag::uart_reset[]
static inline void uart_reset(uart_inst_t *uart) {
    reset_block_num(uart_get_reset_num(uart));
}

static inline void uart_unreset(uart_inst_t *uart) {
    unreset_block_num_wait_blocking(uart_get_reset_num(uart));
}
/// \end::uart_reset[]

/// \tag::uart_init[]
uint uart_init(uart_inst_t *uart, uint baudrate) {
    invalid_params_if(HARDWARE_UART, uart != uart0 && uart != uart1);

    if (uart_clock_get_hz(uart) == 0) {
        return 0;
    }

    uart_reset(uart);
    uart_unreset(uart);

#if PICO_UART_ENABLE_CRLF_SUPPORT
    uart_set_translate_crlf(uart, PICO_UART_DEFAULT_CRLF);
#endif

    // Any LCR writes need to take place before enabling the UART
    uint baud = uart_set_baudrate(uart, baudrate);

    // inline the uart_set_format() call, as we don't need the CR disable/re-enable
    // protection, and also many people will never call it again, so having
    // the generic function is not useful, and much bigger than this inlined
    // code which is only a handful of instructions.
    //
    // The UART_UARTLCR_H_FEN_BITS setting is combined as well as it is the same register
#if 0
    uart_set_format(uart, 8, 1, UART_PARITY_NONE);
    // Enable FIFOs (must be before setting UARTEN, as this is an LCR access)
    hw_set_bits(&uart_get_hw(uart)->lcr_h, UART_UARTLCR_H_FEN_BITS);
#else
    uint data_bits = 8;
    uint stop_bits = 1;
    uint parity = UART_PARITY_NONE;
    hw_write_masked(&uart_get_hw(uart)->lcr_h,
        ((data_bits - 5u) << UART_UARTLCR_H_WLEN_LSB) |
            ((stop_bits - 1u) << UART_UARTLCR_H_STP2_LSB) |
            (bool_to_bit(parity != UART_PARITY_NONE) << UART_UARTLCR_H_PEN_LSB) |
            (bool_to_bit(parity == UART_PARITY_EVEN) << UART_UARTLCR_H_EPS_LSB) |
            UART_UARTLCR_H_FEN_BITS,
        UART_UARTLCR_H_WLEN_BITS | UART_UARTLCR_H_STP2_BITS |
            UART_UARTLCR_H_PEN_BITS | UART_UARTLCR_H_EPS_BITS |
            UART_UARTLCR_H_FEN_BITS);
#endif

    // Enable the UART, both TX and RX
    uart_get_hw(uart)->cr = UART_UARTCR_UARTEN_BITS | UART_UARTCR_TXE_BITS | UART_UARTCR_RXE_BITS;
#if !PICO_UART_NO_DMACR_ENABLE
    // Always enable DREQ signals -- no harm in this if DMA is not listening
    uart_get_hw(uart)->dmacr = UART_UARTDMACR_TXDMAE_BITS | UART_UARTDMACR_RXDMAE_BITS;
#endif

    return baud;
}
/// \end::uart_init[]

void uart_deinit(uart_inst_t *uart) {
    invalid_params_if(HARDWARE_UART, uart != uart0 && uart != uart1);
    uart_reset(uart);
}

static uint32_t uart_disable_before_lcr_write(uart_inst_t *uart) {
    // Notes from PL011 reference manual:
    //
    // - Before writing the LCR, if the UART is enabled it needs to be
    //   disabled and any current TX + RX activity has to be completed
    //
    // - There is a BUSY flag which waits for the current TX char, but this is
    //   OR'd with TX FIFO !FULL, so not usable when FIFOs are enabled and
    //   potentially nonempty
    //
    // - FIFOs can't be set to disabled whilst a character is in progress
    //   (else "FIFO integrity is not guaranteed")
    //
    // Combination of these means there is no general way to halt and poll for
    // end of TX character, if FIFOs may be enabled. Either way, there is no
    // way to poll for end of RX character.
    //
    // So, insert a 15 Baud period delay before changing the settings.
    // 15 Baud is comfortably higher than start + max data + parity + stop.
    // Anything else would require API changes to permit a non-enabled UART
    // state after init() where settings can be changed safely.
    uint32_t cr_save = uart_get_hw(uart)->cr;

    if (cr_save & UART_UARTCR_UARTEN_BITS) {
        hw_clear_bits(&uart_get_hw(uart)->cr,
            UART_UARTCR_UARTEN_BITS | UART_UARTCR_TXE_BITS | UART_UARTCR_RXE_BITS);

        uint32_t current_ibrd = uart_get_hw(uart)->ibrd;
        uint32_t current_fbrd = uart_get_hw(uart)->fbrd;

        // Note: Maximise precision here. Show working, the compiler will mop this up.
        // Create a 16.6 fixed-point fractional division ratio; then scale to 32-bits.
        uint32_t brdiv_ratio = 64u * current_ibrd + current_fbrd;
        brdiv_ratio <<= 10;
        // 3662 is ~(15 * 244.14) where 244.14 is 16e6 / 2^16
        uint32_t scaled_freq = uart_clock_get_hz(uart) / 3662ul;
        uint32_t wait_time_us = brdiv_ratio / scaled_freq;
        busy_wait_us(wait_time_us);
    }

    return cr_save;
}

static void uart_write_lcr_bits_masked(uart_inst_t *uart, uint32_t values, uint32_t write_mask) {
    invalid_params_if(HARDWARE_UART, uart != uart0 && uart != uart1);

    // (Potentially) Cleanly handle disabling the UART before touching LCR
    uint32_t cr_save = uart_disable_before_lcr_write(uart);

    hw_write_masked(&uart_get_hw(uart)->lcr_h, values, write_mask);

    uart_get_hw(uart)->cr = cr_save;
}

/// \tag::uart_set_baudrate[]
uint uart_set_baudrate(uart_inst_t *uart, uint baudrate) {
    invalid_params_if(HARDWARE_UART, baudrate == 0);
    uint32_t baud_rate_div = (8 * uart_clock_get_hz(uart) / baudrate) + 1;
    uint32_t baud_ibrd = baud_rate_div >> 7;
    uint32_t baud_fbrd;

    if (baud_ibrd == 0) {
        baud_ibrd = 1;
        baud_fbrd = 0;
    } else if (baud_ibrd >= 65535) {
        baud_ibrd = 65535;
        baud_fbrd = 0;
    }  else {
        baud_fbrd = (baud_rate_div & 0x7f) >> 1;
    }

    uart_get_hw(uart)->ibrd = baud_ibrd;
    uart_get_hw(uart)->fbrd = baud_fbrd;

    // PL011 needs a (dummy) LCR_H write to latch in the divisors.
    // We don't want to actually change LCR_H contents here.
    uart_write_lcr_bits_masked(uart, 0, 0);

    // See datasheet
    return (4 * uart_clock_get_hz(uart)) / (64 * baud_ibrd + baud_fbrd);
}
/// \end::uart_set_baudrate[]

void uart_set_format(uart_inst_t *uart, uint data_bits, uint stop_bits, uart_parity_t parity) {
    invalid_params_if(HARDWARE_UART, data_bits < 5 || data_bits > 8);
    invalid_params_if(HARDWARE_UART, stop_bits != 1 && stop_bits != 2);
    invalid_params_if(HARDWARE_UART, parity != UART_PARITY_NONE && parity != UART_PARITY_EVEN && parity != UART_PARITY_ODD);

    uart_write_lcr_bits_masked(uart,
        ((data_bits - 5u) << UART_UARTLCR_H_WLEN_LSB) |
        ((stop_bits - 1u) << UART_UARTLCR_H_STP2_LSB) |
        (bool_to_bit(parity != UART_PARITY_NONE) << UART_UARTLCR_H_PEN_LSB) |
        (bool_to_bit(parity == UART_PARITY_EVEN) << UART_UARTLCR_H_EPS_LSB),
        UART_UARTLCR_H_WLEN_BITS |
        UART_UARTLCR_H_STP2_BITS |
        UART_UARTLCR_H_PEN_BITS |
        UART_UARTLCR_H_EPS_BITS);
}

void uart_set_fifo_enabled(uart_inst_t *uart, bool enabled) {

    uint32_t lcr_h_fen_bits = 0;

    if (enabled) {
        lcr_h_fen_bits = UART_UARTLCR_H_FEN_BITS;
    }

    uart_write_lcr_bits_masked(uart, lcr_h_fen_bits, UART_UARTLCR_H_FEN_BITS);
}

void uart_set_break(uart_inst_t *uart, bool en) {

    uint32_t lcr_h_brk_bits = 0;

    if (en) {
        lcr_h_brk_bits = UART_UARTLCR_H_BRK_BITS;
    }

    uart_write_lcr_bits_masked(uart, lcr_h_brk_bits, UART_UARTLCR_H_BRK_BITS);
}

void uart_set_translate_crlf(uart_inst_t *uart, bool crlf) {
#if PICO_UART_ENABLE_CRLF_SUPPORT
    uart_char_to_line_feed[uart_get_index(uart)] = crlf ? '\n' : 0x100;
#else
    panic_unsupported();
#endif
}

bool uart_is_readable_within_us(uart_inst_t *uart, uint32_t us) {
    uint32_t t = time_us_32();
    do {
        if (uart_is_readable(uart)) {
             return true;
        }
    } while ((time_us_32() - t) <= us);
    return false;
}
