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

/*
 * Derived from pico-examples/pio/uart_rx.pio
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "platform.h"

#include "drivers/serial_impl.h"

#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"

// ------- //
// uart_rx //
// ------- //

#define uart_rx_wrap_target 0
#define uart_rx_wrap 8
#define uart_rx_pio_version 0

// ------------------------------------------------------------------ //
// The instruction array below is pioasm output; do not edit by hand. //
// ------------------------------------------------------------------ //

static const uint16_t uart_rx_program_instructions[] = {
            //     .wrap_target
    0x2020, //  0: wait   0 pin, 0
    0xea27, //  1: set    x, 7                   [10]
    0x4001, //  2: in     pins, 1
    0x0642, //  3: jmp    x--, 2                 [6]
    0x00c8, //  4: jmp    pin, 8
    0xc014, //  5: irq    nowait 4 rel
    0x20a0, //  6: wait   1 pin, 0
    0x0000, //  7: jmp    0
    0x8020, //  8: push   block
            //     .wrap
};

const struct pio_program uart_rx_program = {
    .instructions = uart_rx_program_instructions,
    .length = 9,
    .origin = -1,
    .pio_version = uart_rx_pio_version,
#if PICO_PIO_VERSION > 0
    .used_gpio_ranges = 0x0
#endif
};

static inline pio_sm_config uart_rx_program_get_default_config(uint offset) {
    pio_sm_config c = pio_get_default_sm_config();
    sm_config_set_wrap(&c, offset + uart_rx_wrap_target, offset + uart_rx_wrap);
    return c;
}

void uart_rx_program_init(PIO pio, uint sm, uint offset, uint pin, uint baud, serialPullMode_t pull) {
    pio_sm_set_enabled(pio, sm, false);
    pio_sm_set_consecutive_pindirs(pio, sm, pin, 1, false);
    pio_gpio_init(pio, pin);

    // The pull sets the idle level of the line, and in half duplex it is what holds the
    // line at that level whenever the tx program has released it. Inverted ports and
    // SmartAudio need a pulldown rather than a pullup.
    switch (pull) {
    case serialPullUp:
        gpio_set_pulls(pin, true, false);
        break;
    case serialPullDown:
        gpio_set_pulls(pin, false, true);
        break;
    case serialPullNone:
        gpio_set_pulls(pin, false, false);
        break;
    }

    pio_sm_config c = uart_rx_program_get_default_config(offset);
    sm_config_set_in_pins(&c, pin); // for WAIT, IN
    sm_config_set_jmp_pin(&c, pin); // for JMP
    // Shift to right, autopush disabled
    sm_config_set_in_shift(&c, true, false, 32);
    // Deeper FIFO as we're not doing any TX
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_RX);
    // SM transmits 1 bit per 8 execution cycles.
    float div = (float)clock_get_hz(clk_sys) / (8 * baud);
    sm_config_set_clkdiv(&c, div);
    pio_sm_init(pio, sm, offset, &c);
    pio_sm_set_enabled(pio, sm, true);
}
