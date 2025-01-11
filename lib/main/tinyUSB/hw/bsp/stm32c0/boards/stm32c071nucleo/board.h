/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2020, Ha Thach (tinyusb.org)
 * Copyright (c) 2023, HiFiPhile
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

// Green LED
#define GREEN_LED_PORT         GPIOA
#define GREEN_LED_PIN          GPIO_PIN_5
#define GREEN_LED_STATE_ON     1

// Blue LED
#define BLUE_LED_PORT          GPIOC
#define BLUE_LED_PIN           GPIO_PIN_9
#define BLUE_LED_STATE_ON      0

// Generic LED
#define LED_PORT GREEN_LED_PORT
#define LED_PIN GREEN_LED_PIN
#define LED_STATE_ON GREEN_LED_STATE_ON

// Button
#define BUTTON_PORT           GPIOC
#define BUTTON_PIN            GPIO_PIN_13
#define BUTTON_STATE_ACTIVE   0

// Enable UART serial communication with the ST-Link
#define UART_DEV              USART2
#define UART_GPIO_PORT        GPIOA
#define UART_GPIO_AF          GPIO_AF1_USART2
#define UART_TX_PIN           GPIO_PIN_2
#define UART_RX_PIN           GPIO_PIN_3

#endif /* BOARD_H_ */
