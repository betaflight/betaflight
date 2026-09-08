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
 * Derived from pico-examples/pio/uart_tx.pio
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "platform.h"

#include "hardware/pio.h"
#include "hardware/clocks.h"

// ------- //
// uart_tx //
// ------- //

/*
 * The equivalent pioasm source for the program below:
 *
 *   .program uart_tx
 *   .side_set 1 opt
 *   .mov_status txfifo < 1
 *   .wrap_target
 *   tx_release:
 *       set    pindirs, 0             ; release the line (half duplex only, see below)
 *   tx_await:
 *       pull   block                  ; idle here
 *       set    pindirs, 1 side 0  [6] ; take the line AND assert the start bit
 *       set    x, 7                   ; start bit continues (7 + 1 = 8 cycles)
 *   bitloop:
 *       out    pins, 1                ; data bit, LSB first
 *       jmp    x-- bitloop        [6] ; 8 cycles per bit
 *       jmp    !y, tx_next side 1 [5] ; stop bit 1. y == 0 => single stop bit
 *       nop               side 1  [7] ; stop bit 2
 *   tx_next:
 *       mov    x, status              ; X = ~0 iff the tx FIFO is empty
 *       jmp    !x, tx_await           ; another byte queued: hold the line, skip the release
 *   .wrap
 *
 * Notes
 * Line direction: in half duplex the rx state machine sits on the same pin, so
 *   the pin must be released between frames.
 *   In full duplex the pin must stay driven, so the SET pin count is configured as 0,
 *   which makes both `set pindirs` instructions no-ops (side-set still applies).
 *
 * Stop bits: Y (set during initialisation) holds the number of extra stop bits,
 *   so 0 for one stop bit, 1 for two.
 *
 * The line is only released when there is nothing more to send.
 *
 */

#define uart_tx_wrap_target 0
#define uart_tx_wrap 9
#define uart_tx_pio_version 0

// Offset of the blocking PULL, i.e. where the state machine sits when idle (isTxComplete_pio)
#define uart_tx_offset_tx_await 1u
uint32_t tx_await_offset = uart_tx_offset_tx_await;

static const uint16_t uart_tx_program_instructions[] = {
            //     .wrap_target
    0xe080, //  0: set    pindirs, 0
    0x80a0, //  1: pull   block
    0xf681, //  2: set    pindirs, 1      side 0 [6]
    0xe027, //  3: set    x, 7
    0x6001, //  4: out    pins, 1
    0x0644, //  5: jmp    x--, 4                 [6]
    0x1d68, //  6: jmp    !y, 8           side 1 [5]
    0xbf42, //  7: nop                    side 1 [7]
    0xa025, //  8: mov    x, status
    0x0021, //  9: jmp    !x, 1
            //     .wrap
};

const struct pio_program uart_tx_program = {
    .instructions = uart_tx_program_instructions,
    .length = 10,
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

    // Make MOV ..., STATUS report whether the tx FIFO is empty.
    sm_config_set_mov_status(&c, STATUS_TX_LESSTHAN, 1);

    return c;
}

void uart_tx_program_init(PIO pio, uint sm, uint offset, uint pin_tx, uint baud, bool halfDuplex, bool twoStopBits) {
    // Tell PIO to initially drive output-high on the selected pin, then map PIO
    // onto that pin with the IO muxes.
    pio_sm_set_enabled(pio, sm, false);
    pio_sm_set_pins_with_mask64(pio, sm, 1ull << pin_tx, 1ull << pin_tx);
    // In half duplex the line starts released (an input, held at the idle level by the
    // pull configured on the rx side); the program takes it for the duration of a frame.
    pio_sm_set_pindirs_with_mask64(pio, sm, halfDuplex ? 0ull : (1ull << pin_tx), 1ull << pin_tx);
    // pio_gpio_init(pio, pin_tx); // Take care of assigning gpio function in uart_pio.c, not here.
    pio_sm_config c = uart_tx_program_get_default_config(offset);
    // OUT shifts to right, no autopull
    sm_config_set_out_shift(&c, true, false, 32);
    // We are mapping both OUT and side-set to the same pin, because sometimes
    // we need to assert user data onto the pin (with OUT) and sometimes
    // assert constant values (start/stop bit)
    sm_config_set_out_pins(&c, pin_tx, 1);
    sm_config_set_sideset_pins(&c, pin_tx);
    // SET drives the pin direction for the half duplex line turnaround. A pin count of
    // 0 in full duplex turns the two `set pindirs` instructions into no-ops, so the pin
    // is driven continuously.
    sm_config_set_set_pins(&c, pin_tx, halfDuplex ? 1 : 0);
    // We only need TX, so get an 8-deep FIFO!
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_TX);

    // SM transmits 1 bit per 8 execution cycles.
    float div = (float)clock_get_hz(clk_sys) / (8 * baud);
    sm_config_set_clkdiv(&c, div);
    pio_sm_init(pio, sm, offset, &c); // clears FIFO, and sets pc to offset
    // Number of extra stop bits. pio_sm_init does not touch X/Y, so this holds until
    // the next call here. Must be set while the state machine is stopped.
    pio_sm_exec(pio, sm, pio_encode_set(pio_y, twoStopBits ? 1 : 0));
    pio_sm_set_enabled(pio, sm, true);
}
