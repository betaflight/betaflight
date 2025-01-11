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

#ifndef EA4088QS__BOARD_H
#define EA4088QS__BOARD_H

#ifdef __cplusplus
extern "C" {
#endif

#define LED_PORT      2
#define LED_PIN       19

#define BUTTON_PORT   2
#define BUTTON_PIN    10
#define BUTTON_ACTIV_STATE 0

/* System oscillator rate and RTC oscillator rate */
const uint32_t OscRateIn = 12000000;
const uint32_t RTCOscRateIn = 32768;

/* Pin muxing configuration */
static const PINMUX_GRP_T pinmuxing[] = {
    // LED
    { 2, 19, (IOCON_FUNC0 | IOCON_MODE_INACT) },

    // Button
    { 2, 10, (IOCON_FUNC0 | IOCON_MODE_INACT | IOCON_MODE_PULLUP) },

    // USB1 as Host
    { 0, 29, (IOCON_FUNC1 | IOCON_MODE_INACT) }, // D+1
    { 0, 30, (IOCON_FUNC1 | IOCON_MODE_INACT) }, // D-1
    { 1, 18, (IOCON_FUNC1 | IOCON_MODE_INACT) }, // UP LED1
    { 1, 19, (IOCON_FUNC2 | IOCON_MODE_INACT) }, // PPWR1
//  {2, 14, (IOCON_FUNC2 | IOCON_MODE_INACT)}, // VBUS1
//  {2, 15, (IOCON_FUNC2 | IOCON_MODE_INACT)}, // OVRCR1

    // USB2 as Device
    { 0, 31, (IOCON_FUNC1 | IOCON_MODE_INACT) }, // D+2
    { 0, 13, (IOCON_FUNC1 | IOCON_MODE_INACT) }, // UP LED
    { 0, 14, (IOCON_FUNC3 | IOCON_MODE_INACT) }, // CONNECT2

    /* VBUS is not connected on this board, so leave the pin at default setting. */
    /*Chip_IOCON_PinMux(LPC_IOCON, 1, 30, IOCON_MODE_INACT, IOCON_FUNC2);*/ /* USB VBUS */
};

#ifdef __cplusplus
}
#endif

#endif
