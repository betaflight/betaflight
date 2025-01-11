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

#ifndef BOARD_MIMXRT1024_EVK_H_
#define BOARD_MIMXRT1024_EVK_H_

// required since iMXRT MCUX-SDK include this file for board size
// RT1020-EVK #define BOARD_FLASH_SIZE (0x800000U)
#define BOARD_FLASH_SIZE (0x400000U) // builtin flash of RT1024

// LED: IOMUXC_GPIO_AD_B1_08_GPIO1_IO24
#define LED_PORT              BOARD_INITPINS_USER_LED_GPIO
#define LED_PIN               BOARD_INITPINS_USER_LED_PIN
#define LED_STATE_ON          1

// SW8 button: IOMUXC_SNVS_WAKEUP_GPIO5_IO00
#define BUTTON_PORT           BOARD_INITPINS_USER_BUTTON_GPIO
#define BUTTON_PIN            BOARD_INITPINS_USER_BUTTON_PIN
#define BUTTON_STATE_ACTIVE   0

// UART: IOMUXC_GPIO_AD_B0_07_LPUART1_RX, IOMUXC_GPIO_AD_B0_06_LPUART1_TX
#define UART_PORT             LPUART1
#define UART_CLK_ROOT         BOARD_BOOTCLOCKRUN_UART_CLK_ROOT

#endif
