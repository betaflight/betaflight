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
#ifdef PICO_DEFAULT_LED_PIN
#define LED_PIN               PICO_DEFAULT_LED_PIN
#define LED_STATE_ON          (!(PICO_DEFAULT_LED_PIN_INVERTED))
#endif

// Button pin is BOOTSEL which is flash CS pin
#define BUTTON_BOOTSEL
#define BUTTON_STATE_ACTIVE   0

// UART
#if defined(PICO_DEFAULT_UART_TX_PIN) && defined(PICO_DEFAULT_UART_RX_PIN) && \
    defined(PICO_DEFAULT_UART) && defined(LIB_PICO_STDIO_UART)
#define UART_DEV              PICO_DEFAULT_UART
#define UART_TX_PIN           PICO_DEFAULT_UART_TX_PIN
#define UART_RX_PIN           PICO_DEFAULT_UART_RX_PIN
#endif

//--------------------------------------------------------------------+
// PIO_USB
// default to pin on Adafruit Feather rp2040 USB Host or Tester if defined
//--------------------------------------------------------------------+

// #define USE_ADAFRUIT_FEATHER_RP2040_USBHOST
#ifdef USE_ADAFRUIT_FEATHER_RP2040_USBHOST
#define PICO_DEFAULT_PIO_USB_DP_PIN       16
#define PICO_DEFAULT_PIO_USB_VBUSEN_PIN   18
#endif

#ifndef PICO_DEFAULT_PIO_USB_DP_PIN
#define PICO_DEFAULT_PIO_USB_DP_PIN       20
#endif

// VBUS enable pin and its active state
#ifndef PICO_DEFAULT_PIO_USB_VBUSEN_PIN
#define PICO_DEFAULT_PIO_USB_VBUSEN_PIN   22
#endif

// VBUS enable state
#ifndef PICO_DEFAULT_PIO_USB_VBUSEN_STATE
#define PICO_DEFAULT_PIO_USB_VBUSEN_STATE 1
#endif

//--------------------------------------------------------------------
// USB Host MAX3421E
//--------------------------------------------------------------------

#ifdef PICO_DEFAULT_SPI
#define MAX3421_SPI      PICO_DEFAULT_SPI // sdk v2
#else
#define MAX3421_SPI      PICO_DEFAULT_SPI_INSTANCE // sdk v1
#endif

#define MAX3421_SCK_PIN  PICO_DEFAULT_SPI_SCK_PIN
#define MAX3421_MOSI_PIN PICO_DEFAULT_SPI_TX_PIN
#define MAX3421_MISO_PIN PICO_DEFAULT_SPI_RX_PIN
#define MAX3421_CS_PIN   10
#define MAX3421_INTR_PIN 9

#ifdef __cplusplus
 }
#endif

#endif /* BOARD_H_ */
