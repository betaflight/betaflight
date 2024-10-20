/*
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

// -----------------------------------------------------
// NOTE: THIS HEADER IS ALSO INCLUDED BY ASSEMBLER SO
//       SHOULD ONLY CONSIST OF PREPROCESSOR DIRECTIVES
// -----------------------------------------------------

// FIXME delete this file before release (board file for Amethyst FPGA platform)

// pico_cmake_set PICO_PLATFORM=rp2350

#ifndef _BOARDS_AMETHYST_FPGA_H
#define _BOARDS_AMETHYST_FPGA_H

#if !PICO_RP2350
#error "Invalid PICO_PLATFORM for amethyst_fpga.h: must be rp2350 or rp2350-riscv"
#endif

// For board detection
#define RASPBERRYPI_AMETHYST_FPGA

#define PICO_NO_FPGA_CHECK 0

// --- UART ---
#ifndef PICO_DEFAULT_UART
#define PICO_DEFAULT_UART 0
#endif
#ifndef PICO_DEFAULT_UART_TX_PIN
#define PICO_DEFAULT_UART_TX_PIN 46
#endif
#ifndef PICO_DEFAULT_UART_RX_PIN
#define PICO_DEFAULT_UART_RX_PIN 47
#endif
// Match bootrom UART baud rate of 1 Mbaud:
#ifndef PICO_DEFAULT_UART_BAUD_RATE
#define PICO_DEFAULT_UART_BAUD_RATE 1000000
#endif

// --- LED ---
#ifndef PICO_DEFAULT_LED_PIN
#define PICO_DEFAULT_LED_PIN 25
#endif
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

#define PICO_SD_CLK_PIN 18
#define PICO_SD_CMD_PIN 19
#define PICO_SD_DAT0_PIN 20

#define PICO_ON_FPGA 1

#define PICO_SCANVIDEO_COLOR_PIN_COUNT 16
#define PICO_SCANVIDEO_DPI_PIXEL_RSHIFT 0u
#define PICO_SCANVIDEO_DPI_PIXEL_GSHIFT 6u
#define PICO_SCANVIDEO_DPI_PIXEL_BSHIFT 11u

#define PICO_SCANVIDEO_48MHZ 1
#define PICO_AUDIO_I2S_DATA_PIN 29
#define PICO_AUDIO_I2S_CLOCK_PIN_BASE 30

#define PICO_AUDIO_PWM_L_PIN 28
#define PICO_AUDIO_PWM_R_PIN 27

// --- FLASH ---

#define PICO_BOOT_STAGE2_CHOOSE_W25Q080 1

#ifndef PICO_FLASH_SPI_CLKDIV
#define PICO_FLASH_SPI_CLKDIV 2
#endif

// pico_cmake_set_default PICO_FLASH_SIZE_BYTES = (16 * 1024 * 1024)
#ifndef PICO_FLASH_SIZE_BYTES
#define PICO_FLASH_SIZE_BYTES (16 * 1024 * 1024)
#endif
#endif
