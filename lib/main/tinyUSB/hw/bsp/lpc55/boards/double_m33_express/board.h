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
#define LED_PIN               1
#define LED_STATE_ON          1

// WAKE button
#define BUTTON_PORT           0
#define BUTTON_PIN            5
#define BUTTON_STATE_ACTIVE   0

// Number of neopixels
#define NEOPIXEL_NUMBER       2
#define NEOPIXEL_PORT         0
#define NEOPIXEL_PIN          27
#define NEOPIXEL_CH           6
#define NEOPIXEL_TYPE         0

// UART
#define UART_DEV              USART0
#define UART_RX_PINMUX        0U, 29U, IOCON_PIO_DIG_FUNC1_EN
#define UART_TX_PINMUX        0U, 30U, IOCON_PIO_DIG_FUNC1_EN

// XTAL
#define XTAL0_CLK_HZ          (16 * 1000 * 1000U)

#ifdef __cplusplus
 }
#endif

#endif
