/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

// Derived from pico-examples/pio/uart_tx.pio/

/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"

// ------- //
// uart_tx //
// ------- //

#define uart_tx_wrap_target 0
#define uart_tx_wrap 4
#define uart_tx_pio_version 0

#define uart_tx_offset_tx_await 1u
uint32_t tx_await_offset = uart_tx_offset_tx_await;

static const uint16_t uart_tx_program_instructions[] = {
            //     .wrap_target
    0xbe42, //  0: nop                    side 1 [6]
    0x98a0, //  1: pull   block           side 1
    0xf727, //  2: set    x, 7            side 0 [7]
    0x6001, //  3: out    pins, 1
    0x0643, //  4: jmp    x--, 3                 [6]
            //     .wrap
};

const struct pio_program uart_tx_program = {
    .instructions = uart_tx_program_instructions,
    .length = 5,
    .origin = -1,
    .pio_version = uart_tx_pio_version,
#if PICO_PIO_VERSION > 0
    .used_gpio_ranges = 0x0
#endif
};

static inline pio_sm_config uart_tx_program_get_default_config(uint offset) {
    pio_sm_config c = pio_get_default_sm_config();
    sm_config_set_wrap(&c, offset + uart_tx_wrap_target, offset + uart_tx_wrap);
    sm_config_set_sideset(&c, 2, true, false);
    return c;
}

void uart_tx_program_init(PIO pio, uint sm, uint offset, uint pin_tx, uint baud) {
    // Tell PIO to initially drive output-high on the selected pin, then map PIO
    // onto that pin with the IO muxes.
    pio_sm_set_enabled(pio, sm, false);
    pio_sm_set_pins_with_mask64(pio, sm, 1ull << pin_tx, 1ull << pin_tx);
    pio_sm_set_pindirs_with_mask64(pio, sm, 1ull << pin_tx, 1ull << pin_tx);
    // pio_gpio_init(pio, pin_tx); // Take care of assigning gpio function in uart_pio.c, not here.
    pio_sm_config c = uart_tx_program_get_default_config(offset);
    // OUT shifts to right, no autopull
    sm_config_set_out_shift(&c, true, false, 32);
    // We are mapping both OUT and side-set to the same pin, because sometimes
    // we need to assert user data onto the pin (with OUT) and sometimes
    // assert constant values (start/stop bit)
    sm_config_set_out_pins(&c, pin_tx, 1);
    sm_config_set_sideset_pins(&c, pin_tx);
    // We only need TX, so get an 8-deep FIFO!
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_TX);

    // SM transmits 1 bit per 8 execution cycles.
    float div = (float)clock_get_hz(clk_sys) / (8 * baud);
    sm_config_set_clkdiv(&c, div);
    pio_sm_init(pio, sm, offset, &c); // clears FIFO
    pio_sm_set_enabled(pio, sm, true);
}


