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
 */

#ifndef BOARD_H
#define BOARD_H

//--------------------------------------------------------------------+
// MACRO TYPEDEF CONSTANT ENUM DECLARATION
//--------------------------------------------------------------------+
// LED
#define LED_PINMUX            IOMUXC_GPIO_AD_B0_09_GPIO1_IO09
#define LED_PORT              GPIOB
#define LED_PIN_CLOCK         kCLOCK_PortB
#define LED_PIN_PORT          PORTB
#define LED_PIN               19U
#define LED_PIN_FUNCTION      kPORT_MuxAsGpio
#define LED_STATE_ON          0

// Button
#define BUTTON_PORT           GPIOC
#define BUTTON_PIN_CLOCK      kCLOCK_PortC
#define BUTTON_PIN_PORT       PORTC
#define BUTTON_PIN            9U
#define BUTTON_PIN_FUNCTION   kPORT_MuxAsGpio
#define BUTTON_STATE_ACTIVE   0

// UART
#define UART_PORT             UART0
#define UART_PIN_CLOCK        kCLOCK_PortA
#define UART_PIN_PORT         PORTA
#define UART_PIN_RX           1u
#define UART_PIN_TX           2u
#define UART_PIN_FUNCTION     kPORT_MuxAlt2
#define SOPT5_UART0RXSRC_UART_RX      0x00u   /*!< UART0 receive data source select: UART0_RX pin */
#define SOPT5_UART0TXSRC_UART_TX      0x00u   /*!< UART0 transmit data source select: UART0_TX pin */

#endif
