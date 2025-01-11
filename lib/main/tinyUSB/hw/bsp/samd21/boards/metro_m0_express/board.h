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

// LED
#define LED_PIN               17
#define LED_STATE_ON          1

// Button: D5
#define BUTTON_PIN            15
#define BUTTON_STATE_ACTIVE   0

// UART
#define UART_RX_PIN           4
#define UART_TX_PIN           5

// SPI for USB host shield
#define MAX3421_SERCOM_ID       4       // SERCOM4
#define MAX3421_SERCOM_FUNCTION 3       // function D (Sercom Alt)

#define MAX3421_SCK_PIN         (32+11)
#define MAX3421_MOSI_PIN        (32+10)
#define MAX3421_MISO_PIN        12
#define MAX3421_TX_PAD          1 // MOSI = PAD_2, SCK = PAD_3
#define MAX3421_RX_PAD          0 // MISO = PAD_2

#define MAX3421_CS_PIN          18      // D10

#define MAX3421_INTR_PIN        7       // D9
#define MAX3421_INTR_EIC_ID     7       // EIC7


#ifdef __cplusplus
 }
#endif

#endif /* BOARD_H_ */
