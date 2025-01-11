/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2019, Ha Thach (tinyusb.org)
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

#include "fsl_device_registers.h"

#define USB_CLOCK_SOURCE kCLOCK_UsbSrcIrc48M

// LED
#define LED_PIN_CLOCK         kCLOCK_PortA
#define LED_GPIO              GPIOA
#define LED_PORT              PORTA
#define LED_PIN               2
#define LED_STATE_ON          1

// UART
#define UART_PORT             LPUART1
#define UART_PIN_RX           3u
#define UART_PIN_TX           0u

#define UART_CLOCK_SOURCE_HZ  CLOCK_GetFreq(kCLOCK_McgIrc48MClk)

static inline void BOARD_InitBootPins(void) {
  /* PORTC3 is configured as LPUART0_RX */
  PORT_SetPinMux(PORTC, 3U, kPORT_MuxAlt3);
  /* PORTA2 (pin 24) is configured as LPUART0_TX */
  PORT_SetPinMux(PORTE, 0U, kPORT_MuxAlt3);

  SIM->SOPT5 = ((SIM->SOPT5 &
                 /* Mask bits to zero which are setting */
                 (~(SIM_SOPT5_LPUART1TXSRC_MASK | SIM_SOPT5_LPUART1RXSRC_MASK)))
                /* LPUART0 Transmit Data Source Select: LPUART0_TX pin. */
                | SIM_SOPT5_LPUART1TXSRC(SOPT5_LPUART1TXSRC_LPUART_TX)
                /* LPUART0 Receive Data Source Select: LPUART_RX pin. */
                | SIM_SOPT5_LPUART1RXSRC(SOPT5_LPUART1RXSRC_LPUART_RX));
  CLOCK_SetLpuart1Clock(1);
}

#endif /* BOARD_H_ */
