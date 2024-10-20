/*
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

// -----------------------------------------------------
// NOTE: THIS HEADER IS ALSO INCLUDED BY ASSEMBLER SO
//       SHOULD ONLY CONSIST OF PREPROCESSOR DIRECTIVES
// -----------------------------------------------------

// This header may be included by other board headers as "boards/ilabs_challenger_rp2350_wifi_ble.h"

// pico_cmake_set PICO_PLATFORM=rp2350

#ifndef _BOARDS_ILABS_CHALLENGER_RP2350_WIFI_BLE_H
#define _BOARDS_ILABS_CHALLENGER_RP2350_WIFI_BLE_H

// For board detection
#define ILABS_CHALLENGER_RP2350_WIFI_BLE

// --- RP2350 VARIANT ---
#define PICO_RP2350A 1

// --- UART ---
#ifndef PICO_DEFAULT_UART
#define PICO_DEFAULT_UART 0
#endif
#ifndef PICO_DEFAULT_UART_TX_PIN
#define PICO_DEFAULT_UART_TX_PIN 12
#endif
#ifndef PICO_DEFAULT_UART_RX_PIN
#define PICO_DEFAULT_UART_RX_PIN 13
#endif

// PICO_DEFAULT_LED_PIN
#ifndef PICO_DEFAULT_LED_PIN
#define PICO_DEFAULT_LED_PIN 7
#endif

// no PICO_DEFAULT_WS2812_PIN

// --- I2C ---
#ifndef PICO_DEFAULT_I2C
#define PICO_DEFAULT_I2C 0
#endif
#ifndef PICO_DEFAULT_I2C_SDA_PIN
#define PICO_DEFAULT_I2C_SDA_PIN 20
#endif
#ifndef PICO_DEFAULT_I2C_SCL_PIN
#define PICO_DEFAULT_I2C_SCL_PIN 21
#endif

// --- SPI ---
#ifndef PICO_DEFAULT_SPI
#define PICO_DEFAULT_SPI 0
#endif
#ifndef PICO_DEFAULT_SPI_SCK_PIN
#define PICO_DEFAULT_SPI_SCK_PIN 18
#endif
#ifndef PICO_DEFAULT_SPI_TX_PIN
#define PICO_DEFAULT_SPI_TX_PIN 19
#endif
#ifndef PICO_DEFAULT_SPI_RX_PIN
#define PICO_DEFAULT_SPI_RX_PIN 16
#endif
#ifndef PICO_DEFAULT_SPI_CSN_PIN
#define PICO_DEFAULT_SPI_CSN_PIN 17
#endif

// --- FLASH ---
#define PICO_BOOT_STAGE2_CHOOSE_W25Q080 1

#ifndef PICO_FLASH_SPI_CLKDIV
#define PICO_FLASH_SPI_CLKDIV 2
#endif

// pico_cmake_set_default PICO_FLASH_SIZE_BYTES = (8 * 1024 * 1024)
#ifndef PICO_FLASH_SIZE_BYTES
#define PICO_FLASH_SIZE_BYTES (8 * 1024 * 1024)
#endif

#ifndef PICO_RP2350_A2_SUPPORTED
#define PICO_RP2350_A2_SUPPORTED 1
#endif

// Board specific helper macros
// Types of valid modules on the board
#define ESP32C6_MINI_1 1
#define ESP32C3_MINI_1 2

// Specifies the default module used on this board
#ifndef ILABS_ONBOARD_ESP_MODULE
#define ILABS_ONBOARD_ESP_MODULE ESP32C6_MINI_1
#endif

// ESP module control signals
#ifndef ILABS_DEFAULT_ESP_MODULE_RST_PIN
#define ILABS_DEFAULT_ESP_MODULE_RST_PIN 15
#endif

#ifndef ILABS_DEFAULT_ESP_MODULE_BOOT_MODE_PIN
#define ILABS_DEFAULT_ESP_MODULE_BOOT_MODE_PIN 14
#endif

// ESP module UART definitions
#ifndef ILABS_ESP_AT_UART
#define ILABS_ESP_AT_UART 1
#endif

#ifndef ILABS_DEFAULT_ESP_AT_TX_PIN
#define ILABS_DEFAULT_ESP_AT_TX_PIN 4
#endif

#ifndef ILABS_DEFAULT_ESP_AT_RX_PIN
#define ILABS_DEFAULT_ESP_AT_RX_PIN 5
#endif

// ESP module SPI definitions
#ifndef ILABS_DEFAULT_ESP_SPI
#define ILABS_DEFAULT_ESP_SPI 1
#endif

#ifndef ILABS_DEFAULT_ESP_SPI_MISO_PIN
#define ILABS_DEFAULT_ESP_SPI_MISO_PIN 8
#endif

#ifndef ILABS_DEFAULT_ESP_SPI_MOSI_PIN
#define ILABS_DEFAULT_ESP_SPI_MOSI_PIN 11
#endif

#ifndef ILABS_DEFAULT_ESP_SPI_SCK_PIN
#define ILABS_DEFAULT_ESP_SPI_SCK_PIN 10
#endif

#ifndef ILABS_DEFAULT_ESP_SPI_SS_PIN
#define ILABS_DEFAULT_ESP_SPI_SS_PIN 9
#endif

#ifndef ILABS_DEFAULT_ESP_SPI_HS_PIN
#define ILABS_DEFAULT_ESP_SPI_HS_PIN 22
#endif

#endif
