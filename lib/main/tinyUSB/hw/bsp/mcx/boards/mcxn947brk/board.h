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
#define LED_GPIO              GPIO3
#define LED_CLK               kCLOCK_Gpio3
#define LED_PIN               4 // red
#define LED_STATE_ON          0

// WAKE button (Dummy, use unused pin
#define BUTTON_GPIO           GPIO0
#define BUTTON_CLK            kCLOCK_Gpio0
#define BUTTON_PIN            6
#define BUTTON_STATE_ACTIVE   0

// UART
#define UART_DEV              LPUART4

static inline void board_uart_init_clock(void) {
  /* attach FRO 12M to FLEXCOMM4 */
  CLOCK_SetClkDiv(kCLOCK_DivFlexcom4Clk, 1u);
  CLOCK_AttachClk(kFRO12M_to_FLEXCOMM4);
  RESET_ClearPeripheralReset(kFC4_RST_SHIFT_RSTn);
}

//#define UART_RX_PINMUX        0, 24, IOCON_PIO_DIG_FUNC1_EN
//#define UART_TX_PINMUX        0, 25, IOCON_PIO_DIG_FUNC1_EN

// XTAL
#define XTAL0_CLK_HZ          (24 * 1000 * 1000U)

#ifdef __cplusplus
 }
#endif

#endif
