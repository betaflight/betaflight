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

#ifndef BOARD_MIMXRT1015_EVK_H_
#define BOARD_MIMXRT1015_EVK_H_

// required since iMXRT MCUX-SDK include this file for board size
#define BOARD_FLASH_SIZE (0x1000000U)

// LED
#define LED_PINMUX            IOMUXC_GPIO_SD_B1_01_GPIO3_IO21
#define LED_PORT              BOARD_INITPINS_USER_LED_PERIPHERAL
#define LED_PIN               BOARD_INITPINS_USER_LED_CHANNEL
#define LED_STATE_ON          0

// SW8 button
#define BUTTON_PINMUX         IOMUXC_GPIO_EMC_09_GPIO2_IO09
#define BUTTON_PORT           BOARD_INITPINS_USER_BUTTON_GPIO
#define BUTTON_PIN            BOARD_INITPINS_USER_BUTTON_GPIO_PIN
#define BUTTON_STATE_ACTIVE   0

// UART
#define UART_PORT             LPUART1
#define UART_CLK_ROOT         BOARD_BOOTCLOCKRUN_UART_CLK_ROOT
#define UART_RX_PINMUX        IOMUXC_GPIO_AD_B0_07_LPUART1_RX
#define UART_TX_PINMUX        IOMUXC_GPIO_AD_B0_06_LPUART1_TX

#endif
