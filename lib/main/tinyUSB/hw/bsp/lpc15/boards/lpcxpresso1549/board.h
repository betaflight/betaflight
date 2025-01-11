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

// XTAL
#define XTAL_OscRateIn      12000000
#define XTAL_RTCOscRateIn   32768

// LED
#define LED_PORT            0
#define LED_PIN             25

// Wake Switch
#define BUTTON_PORT         0
#define BUTTON_PIN          17

#define UART_PORT           LPC_USART0

static inline void board_lpc15_pinmux_swm_init(void)
{
  // Pinmux
  const PINMUX_GRP_T pinmuxing[] =
  {
    {0, 25, (IOCON_MODE_INACT    | IOCON_DIGMODE_EN)}, // PIO0_25-BREAK_CTRL-RED (low enable)
    {0, 13, (IOCON_MODE_INACT    | IOCON_DIGMODE_EN)}, // PIO0_13-ISP_RX
    {0, 18, (IOCON_MODE_INACT    | IOCON_DIGMODE_EN)}, // PIO0_18-ISP_TX
    {1, 11, (IOCON_MODE_PULLDOWN | IOCON_DIGMODE_EN)}, // PIO1_11-ISP_1 (VBUS)
  };

  // Pin Mux
  Chip_IOCON_SetPinMuxing(LPC_IOCON, pinmuxing, sizeof(pinmuxing) / sizeof(PINMUX_GRP_T));

  // SWIM
  Chip_SWM_MovablePortPinAssign(SWM_USB_VBUS_I , 1, 11);
  Chip_SWM_MovablePortPinAssign(SWM_UART0_RXD_I, 0, 13);
  Chip_SWM_MovablePortPinAssign(SWM_UART0_TXD_O, 0, 18);
}

#ifdef __cplusplus
 }
#endif

#endif
