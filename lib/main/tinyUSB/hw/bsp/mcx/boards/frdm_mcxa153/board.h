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
#define LED_CLK               kCLOCK_GateGPIO3
#define LED_PIN               12 // red
#define LED_STATE_ON          0

// ISP button (Dummy, use unused pin
#define BUTTON_GPIO           GPIO3
#define BUTTON_CLK            kCLOCK_GateGPIO3
#define BUTTON_PIN            29 //sw2
#define BUTTON_STATE_ACTIVE   0

// UART
#define UART_DEV              LPUART0

static inline void board_uart_init_clock(void) {
  /* attach 12 MHz clock to LPUART0 (debug console) */
  CLOCK_SetClockDiv(kCLOCK_DivLPUART0, 1u);
  CLOCK_AttachClk(kFRO12M_to_LPUART0);

  RESET_PeripheralReset(kLPUART0_RST_SHIFT_RSTn);
}

// XTAL
#define XTAL0_CLK_HZ          (24 * 1000 * 1000U)

#ifdef __cplusplus
}
#endif

#endif
