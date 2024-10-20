/*
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

// -----------------------------------------------------
// NOTE: THIS HEADER IS ALSO INCLUDED BY ASSEMBLER SO
//       SHOULD ONLY CONSIST OF PREPROCESSOR DIRECTIVES
// -----------------------------------------------------

#ifndef _BOARDS_HELLBENDER_2350A_DEVBOARD_H
#define _BOARDS_HELLBENDER_2350A_DEVBOARD_H

// pico_cmake_set PICO_PLATFORM=rp2350

// For board detection
#define HELLBENDER_2350A_DEVBOARD

// Pin aliases
#define HB_2350A_XIP_CS1_PIN 0
#define HB_2350A_IMU_INT_PIN 1
#define HB_2350A_USER_QWIIC_SDA_PIN 2
#define HB_2350A_USER_QWIIC_SCL_PIN 3
#define HB_2350A_RTC_INT_PIN 8
#define HB_2350A_SDCARD_CS_PIN 9
#define HB_2350A_LORA_IO1_PIN 10
#define HB_2350A_LORA_IO3_PIN 11
#define HB_2350A_LORA_CS_PIN 21
#define HB_2350A_RTC_CLKIN_PIN 22
#define HB_2350A_IMU_CLKOUT_PIN 23
#define HB_2350A_LORA_BUSY_PIN 24
#define HB_2350A_FUSB307_INT_PIN 25
#define HB_2350A_LORA_RESET_PIN 29

// --- UART ---
// Note, conflicts with HSTX range
#ifndef PICO_DEFAULT_UART
#define PICO_DEFAULT_UART 0
#endif
#ifndef PICO_DEFAULT_UART_TX_PIN
#define PICO_DEFAULT_UART_TX_PIN 12
#endif
#ifndef PICO_DEFAULT_UART_RX_PIN
#define PICO_DEFAULT_UART_RX_PIN 13
#endif

// no PICO_DEFAULT_LED_PIN
// no PICO_DEFAULT_WS2812_PIN

// --- I2C ---
#ifndef PICO_DEFAULT_I2C
#define PICO_DEFAULT_I2C 0
#endif
#ifndef PICO_DEFAULT_I2C_SDA_PIN
#define PICO_DEFAULT_I2C_SDA_PIN 4
#endif
#ifndef PICO_DEFAULT_I2C_SCL_PIN
#define PICO_DEFAULT_I2C_SCL_PIN 5
#endif

// --- SPI ---
#ifndef PICO_DEFAULT_SPI
#define PICO_DEFAULT_SPI 0
#endif
#ifndef PICO_DEFAULT_SPI_SCK_PIN
#define PICO_DEFAULT_SPI_SCK_PIN 6
#endif
#ifndef PICO_DEFAULT_SPI_TX_PIN
#define PICO_DEFAULT_SPI_TX_PIN 7
#endif
#ifndef PICO_DEFAULT_SPI_RX_PIN
#define PICO_DEFAULT_SPI_RX_PIN 20
#endif
#ifndef PICO_DEFAULT_SPI_CSN_PIN
#define PICO_DEFAULT_SPI_CSN_PIN 21
#endif

// --- FLASH ---

#define PICO_BOOT_STAGE2_CHOOSE_W25Q080 1

#ifndef PICO_FLASH_SPI_CLKDIV
#define PICO_FLASH_SPI_CLKDIV 2
#endif

// pico_cmake_set_default PICO_FLASH_SIZE_BYTES = (16 * 1024 * 1024)
#ifndef PICO_FLASH_SIZE_BYTES
#define PICO_FLASH_SIZE_BYTES (16 * 1024 * 1024)
#endif
// --- RP2350 VARIANT ---
#define PICO_RP2350A 1

#ifndef PICO_RP2350_A2_SUPPORTED
#define PICO_RP2350_A2_SUPPORTED 1
#endif

#endif
