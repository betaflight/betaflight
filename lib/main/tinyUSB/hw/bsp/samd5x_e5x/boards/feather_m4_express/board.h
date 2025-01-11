/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2020, Ha Thach (tinyusb.org)
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

#define _PINNUM(port, pin)    ((port)*32 + (pin))

// LED
#define LED_PIN               23
#define LED_STATE_ON          1

// Button
#define BUTTON_PIN            16 // D5
#define BUTTON_STATE_ACTIVE   0

// UART
#define UART_TX_PIN           (32 + 17)
#define UART_RX_PIN           (32 + 16)

// SPI for USB host shield
#define MAX3421_SERCOM_ID       1  // SERCOM2
#define MAX3421_SERCOM_FUNCTION 2  // function C

#define MAX3421_SCK_PIN         _PINNUM(0, 17)
#define MAX3421_MOSI_PIN        _PINNUM(1, 23)
#define MAX3421_MISO_PIN        _PINNUM(1, 22)
#define MAX3421_TX_PAD          2 // MOSI = PAD_3, SCK = PAD_1
#define MAX3421_RX_PAD          2 // MISO = PAD_2

#define MAX3421_CS_PIN          20 // D10

#define MAX3421_INTR_PIN        19 // D9
#define MAX3421_INTR_EIC_ID     3  // EIC3

#ifdef __cplusplus
 }
#endif

#endif /* BOARD_H_ */
