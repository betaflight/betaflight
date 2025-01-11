/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2024, Brent Kowal (Analog Devices, Inc)
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

#include "gpio.h"
#include "mxc_sys.h"

#ifdef __cplusplus
extern "C" {
#endif

// LED
#define LED_PORT        MXC_GPIO0
#define LED_PIN         MXC_GPIO_PIN_14
#define LED_VDDIO       MXC_GPIO_VSSEL_VDDIOH
#define LED_STATE_ON    0

// Button
#define BUTTON_PORT         MXC_GPIO4
#define BUTTON_PIN          MXC_GPIO_PIN_0
#define BUTTON_PULL         MXC_GPIO_PAD_PULL_UP
#define BUTTON_STATE_ACTIVE 0

// UART Enable for EvKit's Integrated FTDI Adapter. Pin Mux handled by the HAL
#define UART_NUM 2

#ifdef __cplusplus
}
#endif

#endif /* BOARD_H_ */
