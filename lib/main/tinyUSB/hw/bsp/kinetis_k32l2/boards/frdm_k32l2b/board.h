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
#define LED_PIN_CLOCK         kCLOCK_PortD
#define LED_GPIO              GPIOD
#define LED_PORT              PORTD
#define LED_PIN               5
#define LED_STATE_ON          0

// SW3 button1
#define BUTTON_PIN_CLOCK      kCLOCK_PortC
#define BUTTON_GPIO           GPIOC
#define BUTTON_PORT           PORTC
#define BUTTON_PIN            3
#define BUTTON_STATE_ACTIVE   0

// UART
#define UART_PORT             LPUART0
#define UART_PIN_CLOCK        kCLOCK_PortA
#define UART_PIN_PORT         PORTA
#define UART_PIN_RX           1u
#define UART_PIN_TX           2u
#define SOPT5_LPUART0RXSRC_LPUART_RX 0x00u /*!<@brief LPUART0 Receive Data Source Select: LPUART_RX pin */
#define SOPT5_LPUART0TXSRC_LPUART_TX 0x00u /*!<@brief LPUART0 Transmit Data Source Select: LPUART0_TX pin */
#define UART_CLOCK_SOURCE_HZ  CLOCK_GetFreq(kCLOCK_McgIrc48MClk)

static inline void BOARD_InitBootPins(void) {
  /* PORTA1 (pin 23) is configured as LPUART0_RX */
  PORT_SetPinMux(PORTA, 1U, kPORT_MuxAlt2);
  /* PORTA2 (pin 24) is configured as LPUART0_TX */
  PORT_SetPinMux(PORTA, 2U, kPORT_MuxAlt2);

  SIM->SOPT5 = ((SIM->SOPT5 &
                 /* Mask bits to zero which are setting */
                 (~(SIM_SOPT5_LPUART0TXSRC_MASK | SIM_SOPT5_LPUART0RXSRC_MASK)))
                /* LPUART0 Transmit Data Source Select: LPUART0_TX pin. */
                | SIM_SOPT5_LPUART0TXSRC(SOPT5_LPUART0TXSRC_LPUART_TX)
                /* LPUART0 Receive Data Source Select: LPUART_RX pin. */
                | SIM_SOPT5_LPUART0RXSRC(SOPT5_LPUART0RXSRC_LPUART_RX));

  BOARD_BootClockRUN();
  SystemCoreClockUpdate();
  CLOCK_SetLpuart0Clock(1);
}

#endif /* BOARD_H_ */
