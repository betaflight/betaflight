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

// PD_10
#define LED_PORT      6
#define LED_PIN       24

// P4_0
#define BUTTON_PORT   2
#define BUTTON_PIN    0

#define UART_DEV      LPC_USART3

static inline void board_lpc18_pinmux(void) {
  const PINMUX_GRP_T pinmuxing[] = {
    // ETM Trace
    #ifdef TRACE_ETM
    { 0xF, 4, SCU_MODE_FUNC2 | SCU_MODE_HIGHSPEEDSLEW_EN },
    { 0xF, 5, SCU_MODE_FUNC3 | SCU_MODE_HIGHSPEEDSLEW_EN },
    { 0xF, 6, SCU_MODE_FUNC3 | SCU_MODE_HIGHSPEEDSLEW_EN },
    { 0xF, 7, SCU_MODE_FUNC3 | SCU_MODE_HIGHSPEEDSLEW_EN },
    { 0xF, 8, SCU_MODE_FUNC3 | SCU_MODE_HIGHSPEEDSLEW_EN },
    #endif

    // LEDs
    { 0xD, 10, (SCU_MODE_INBUFF_EN | SCU_MODE_INACT | SCU_MODE_FUNC4) },
    { 0xD, 11, (SCU_MODE_INBUFF_EN | SCU_MODE_INACT | SCU_MODE_FUNC4 | SCU_MODE_PULLDOWN) },
    { 0xD, 12, (SCU_MODE_INBUFF_EN | SCU_MODE_INACT | SCU_MODE_FUNC4 | SCU_MODE_PULLDOWN) },
    { 0xD, 13, (SCU_MODE_INBUFF_EN | SCU_MODE_INACT | SCU_MODE_FUNC4 | SCU_MODE_PULLDOWN) },
    { 0xD, 14, (SCU_MODE_INBUFF_EN | SCU_MODE_INACT | SCU_MODE_FUNC4 | SCU_MODE_PULLDOWN) },
    { 0x9,  0, (SCU_MODE_INBUFF_EN | SCU_MODE_INACT | SCU_MODE_FUNC0 | SCU_MODE_PULLDOWN) },
    { 0x9,  1, (SCU_MODE_INBUFF_EN | SCU_MODE_INACT | SCU_MODE_FUNC0 | SCU_MODE_PULLDOWN) },
    { 0x9,  2, (SCU_MODE_INBUFF_EN | SCU_MODE_INACT | SCU_MODE_FUNC0 | SCU_MODE_PULLDOWN) },

    // Button
    { 0x4, 0, (SCU_MODE_INBUFF_EN | SCU_MODE_INACT | SCU_MODE_FUNC0 | SCU_MODE_PULLUP) },

    // UART
    { 2, 3, SCU_MODE_PULLDOWN | SCU_MODE_FUNC2 },
    { 2, 4, SCU_MODE_INACT | SCU_MODE_INBUFF_EN | SCU_MODE_ZIF_DIS | SCU_MODE_FUNC2 },

    // USB0
    { 0x6, 3, SCU_MODE_PULLUP | SCU_MODE_INBUFF_EN | SCU_MODE_FUNC1 },		                // P6_3 USB0_PWR_EN, USB0 VBus function
    { 0x9, 5, SCU_MODE_PULLUP | SCU_MODE_INBUFF_EN | SCU_MODE_FUNC2 },			              // P9_5 USB1_VBUS_EN, USB1 VBus function
    { 0x2, 5, SCU_MODE_INACT  | SCU_MODE_INBUFF_EN | SCU_MODE_ZIF_DIS | SCU_MODE_FUNC2 }, // P2_5 USB1_VBUS, MUST CONFIGURE THIS SIGNAL FOR USB1 NORMAL OPERATION
  };

  Chip_SCU_SetPinMuxing(pinmuxing, sizeof(pinmuxing) / sizeof(PINMUX_GRP_T));

  /* Pin clock mux values, re-used structure, value in first index is meaningless */
  const PINMUX_GRP_T pinclockmuxing[] = {
    { 0, 0,  (SCU_MODE_INACT | SCU_MODE_INBUFF_EN | SCU_MODE_ZIF_DIS | SCU_MODE_HIGHSPEEDSLEW_EN | SCU_MODE_FUNC0)},
    { 0, 1,  (SCU_MODE_INACT | SCU_MODE_INBUFF_EN | SCU_MODE_ZIF_DIS | SCU_MODE_HIGHSPEEDSLEW_EN | SCU_MODE_FUNC0)},
    { 0, 2,  (SCU_MODE_INACT | SCU_MODE_INBUFF_EN | SCU_MODE_ZIF_DIS | SCU_MODE_HIGHSPEEDSLEW_EN | SCU_MODE_FUNC0)},
    { 0, 3,  (SCU_MODE_INACT | SCU_MODE_INBUFF_EN | SCU_MODE_ZIF_DIS | SCU_MODE_HIGHSPEEDSLEW_EN | SCU_MODE_FUNC0)},
  };

  /* Clock pins only, group field not used */
  for (uint32_t i = 0; i < (sizeof(pinclockmuxing) / sizeof(pinclockmuxing[0])); i++) {
    Chip_SCU_ClockPinMuxSet(pinclockmuxing[i].pinnum, pinclockmuxing[i].modefunc);
  }
}

#ifdef __cplusplus
 }
#endif

#endif
