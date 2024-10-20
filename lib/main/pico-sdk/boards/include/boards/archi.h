/*
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

// -----------------------------------------------------
// NOTE: THIS HEADER IS ALSO INCLUDED BY ASSEMBLER SO
//       SHOULD ONLY CONSIST OF PREPROCESSOR DIRECTIVES
// -----------------------------------------------------

// This header may be included by other board headers as "boards/archi.h"

// pico_cmake_set PICO_PLATFORM=rp2040

#ifndef _BOARDS_ARCHI_H
#define _BOARDS_ARCHI_H

// For board detection
#define ARCHI

// On some samples, the xosc can take longer to stabilize than is usual
#ifndef PICO_XOSC_STARTUP_DELAY_MULTIPLIER
#define PICO_XOSC_STARTUP_DELAY_MULTIPLIER 64
#endif

// --- BOARD SPECIFIC ---
#ifndef ARCHI_BUTTON1_PIN
#define ARCHI_BUTTON1_PIN 23
#endif

#ifndef ARCHI_BUTTON2_PIN
#define ARCHI_BUTTON2_PIN 10
#endif

#ifndef ARCHI_BUTTON3_PIN
#define ARCHI_BUTTON3_PIN 25
#endif

#ifndef ARCHI_BUTTON4_PIN
#define ARCHI_BUTTON4_PIN 19
#endif

#ifndef ARCHI_BUZZER_PIN
#define ARCHI_BUZZER_PIN 22
#endif

#ifndef ARCHI_NEOPIXEL_PIN
#define ARCHI_NEOPIXEL_PIN 24
#endif

// --- UART ---
#ifndef PICO_DEFAULT_UART
#define PICO_DEFAULT_UART 0
#endif
#ifndef PICO_DEFAULT_UART_TX_PIN
#define PICO_DEFAULT_UART_TX_PIN 16
#endif
#ifndef PICO_DEFAULT_UART_RX_PIN
#define PICO_DEFAULT_UART_RX_PIN 17
#endif

// --- LED ---
// no PICO_DEFAULT_LED_PIN

#ifndef PICO_DEFAULT_WS2812_PIN
#define PICO_DEFAULT_WS2812_PIN ARCHI_NEOPIXEL_PIN
#endif

// --- I2C ---
#ifndef PICO_DEFAULT_I2C
#define PICO_DEFAULT_I2C 1
#endif
#ifndef PICO_DEFAULT_I2C_SDA_PIN
#define PICO_DEFAULT_I2C_SDA_PIN 14
#endif
#ifndef PICO_DEFAULT_I2C_SCL_PIN
#define PICO_DEFAULT_I2C_SCL_PIN 15
#endif

// --- SPI ---
#ifndef PICO_DEFAULT_SPI
#define PICO_DEFAULT_SPI 0
#endif
#ifndef PICO_DEFAULT_SPI_SCK_PIN
#define PICO_DEFAULT_SPI_SCK_PIN 2
#endif
#ifndef PICO_DEFAULT_SPI_TX_PIN
#define PICO_DEFAULT_SPI_TX_PIN 3
#endif
#ifndef PICO_DEFAULT_SPI_RX_PIN
#define PICO_DEFAULT_SPI_RX_PIN 4
#endif
#ifndef PICO_DEFAULT_SPI_CSN_PIN
#define PICO_DEFAULT_SPI_CSN_PIN 5
#endif

// --- FLASH ---

// Use slower generic flash access
#define PICO_BOOT_STAGE2_CHOOSE_GENERIC_03H 1

#ifndef PICO_FLASH_SPI_CLKDIV
#define PICO_FLASH_SPI_CLKDIV 4
#endif

// pico_cmake_set_default PICO_FLASH_SIZE_BYTES = (4 * 1024 * 1024)
#ifndef PICO_FLASH_SIZE_BYTES
#define PICO_FLASH_SIZE_BYTES (4 * 1024 * 1024)
#endif

#endif
