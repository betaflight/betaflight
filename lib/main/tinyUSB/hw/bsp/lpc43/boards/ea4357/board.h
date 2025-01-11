/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2023 Ha Thach (tinyusb.org)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 * This file is part of the TinyUSB stack.
 */

#ifndef _BOARD_EA4357_H
#define _BOARD_EA4357_H

#ifdef __cplusplus
extern "C" {
#endif

#include "pca9532.h"

// P9_1 joystick down
#define BUTTON_PORT     4
#define BUTTON_PIN      13
#define BUTTON_STATE_ACTIVE   0

#define UART_DEV        LPC_USART0
#define UART_PORT       0x0f
#define UART_PIN_TX     10
#define UART_PIN_RX     11

//static const struct {
//  uint8_t mux_port;
//  uint8_t mux_pin;
//
//  uint8_t gpio_port;
//  uint8_t gpio_pin;
//}buttons[] =
//{
//    {0x0a, 3, 4, 10 }, // Joystick up
//    {0x09, 1, 4, 13 }, // Joystick down
//    {0x0a, 2, 4, 9  }, // Joystick left
//    {0x09, 0, 4, 12 }, // Joystick right
//    {0x0a, 1, 4, 8  }, // Joystick press
//    {0x02, 7, 0, 7  }, // SW6
//};

static const PINMUX_GRP_T pinmuxing[] = {
    // Button ( Joystick down )
    { 0x9, 1, SCU_MODE_INBUFF_EN | SCU_MODE_INACT | SCU_MODE_FUNC0 | SCU_MODE_PULLUP },

    // UART
    { UART_PORT, UART_PIN_TX, SCU_MODE_PULLDOWN | SCU_MODE_FUNC1 },
    { UART_PORT, UART_PIN_RX, SCU_MODE_INACT | SCU_MODE_INBUFF_EN | SCU_MODE_ZIF_DIS | SCU_MODE_FUNC1 },

    // USB
};

/* Pin clock mux values, re-used structure, value in first index is meaningless */
//static const PINMUX_GRP_T pinclockmuxing[] = {
//    { 0, 0, SCU_MODE_INACT | SCU_MODE_INBUFF_EN | SCU_MODE_ZIF_DIS | SCU_MODE_HIGHSPEEDSLEW_EN | SCU_MODE_FUNC0 },
//    { 0, 1, SCU_MODE_INACT | SCU_MODE_INBUFF_EN | SCU_MODE_ZIF_DIS | SCU_MODE_HIGHSPEEDSLEW_EN | SCU_MODE_FUNC0 },
//    { 0, 2, SCU_MODE_INACT | SCU_MODE_INBUFF_EN | SCU_MODE_ZIF_DIS | SCU_MODE_HIGHSPEEDSLEW_EN | SCU_MODE_FUNC0 },
//    { 0, 3, SCU_MODE_INACT | SCU_MODE_INBUFF_EN | SCU_MODE_ZIF_DIS | SCU_MODE_HIGHSPEEDSLEW_EN | SCU_MODE_FUNC0 },
//};

#ifdef __cplusplus
}
#endif

#endif
