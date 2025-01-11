/*
 * The MIT License (MIT)
 *
 * Copyright 2021 Bridgetek Pte Ltd
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

// Note: This definition file covers all MM900EV1B, MM900EV2B, MM900EV3B,
// MM900EV-Lite boards.
// Each of these boards has an FT900 device.

#ifdef __cplusplus
 extern "C" {
#endif

// UART to use on this board.
#ifndef BOARD_UART
#define BOARD_UART UART0
#endif

// UART is on connector CN1.
#ifndef BOARD_GPIO_UART0_TX
#define BOARD_GPIO_UART0_TX 48 // Pin 4 of CN1.
#endif
#ifndef BOARD_GPIO_UART0_RX
#define BOARD_GPIO_UART0_RX 49 // Pin 6 of CN1.
#endif

// LED is connected to pins 17 (signal) and 15 (GND) of CN1.
#ifndef BOARD_GPIO_LED
#define BOARD_GPIO_LED 35
#endif
#ifndef BOARD_GPIO_LED_STATE_ON
#define BOARD_GPIO_LED_STATE_ON 1
#endif
// Button is connected to pins 13 (signal) and 15 (GND) of CN1.
#ifndef BOARD_GPIO_BUTTON
#define BOARD_GPIO_BUTTON 56
#endif
// Button is pulled up and grounded for active.
#ifndef BOARD_GPIO_BUTTON_STATE_ACTIVE
#define BOARD_GPIO_BUTTON_STATE_ACTIVE 0
#endif

// Enable the Remote Wakeup signalling.
// Remote wakeup is wired to pin 40 of CN1.
#ifndef BOARD_GPIO_REMOTE_WAKEUP
#define BOARD_GPIO_REMOTE_WAKEUP 18
#endif

// USB VBus signal is connected directly to the FT900.
#ifndef BOARD_USBD_VBUS_DTC_PIN
#define BOARD_USBD_VBUS_DTC_PIN 3
#endif

#ifdef __cplusplus
 }
#endif

#endif
