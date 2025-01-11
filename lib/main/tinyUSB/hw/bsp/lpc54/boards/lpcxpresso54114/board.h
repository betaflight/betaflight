/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2021, Ha Thach (tinyusb.org)
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

#ifndef BOARD_H_
#define BOARD_H_

#ifdef __cplusplus
 extern "C" {
#endif

// LED
#define LED_PORT              0
#define LED_PIN               29
#define LED_STATE_ON          0

// WAKE button
#define BUTTON_PORT           0
#define BUTTON_PIN            24
#define BUTTON_STATE_ACTIVE   0

// UART
#define UART_DEV              USART0
#define UART_RX_PINMUX        0, 0, IOCON_PIO_DIG_FUNC1_EN
#define UART_TX_PINMUX        0, 1, IOCON_PIO_DIG_FUNC1_EN

// USB0 VBUS
#define USB0_VBUS_PINMUX      1, 6, IOCON_PIO_DIG_FUNC7_EN

// XTAL
//#define XTAL0_CLK_HZ          (16 * 1000 * 1000U)

#ifdef __cplusplus
 }
#endif

#endif
