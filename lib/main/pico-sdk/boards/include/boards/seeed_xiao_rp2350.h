/*
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

// -----------------------------------------------------
// NOTE: THIS HEADER IS ALSO INCLUDED BY ASSEMBLER SO
//       SHOULD ONLY CONSIST OF PREPROCESSOR DIRECTIVES
// -----------------------------------------------------

// pico_cmake_set PICO_PLATFORM=rp2350

#ifndef _BOARDS_SEEED_XIAO_RP2350_H
#define _BOARDS_SEEED_XIAO_RP2350_H

// For board detection
#define SEEED_XIAO_RP2350

//------------- UART -------------//
#ifndef PICO_DEFAULT_UART
#define PICO_DEFAULT_UART 0
#endif

#ifndef PICO_DEFAULT_UART_TX_PIN
#define PICO_DEFAULT_UART_TX_PIN 0
#endif

#ifndef PICO_DEFAULT_UART_RX_PIN
#define PICO_DEFAULT_UART_RX_PIN 1
#endif

#ifndef PICO_UART1_TX_PIN
#define PICO_UART1_TX_PIN 20
#endif

#ifndef PICO_UART1_RX_PIN
#define PICO_UART1_RX_PIN 21
#endif

//------------- LED -------------//
#ifndef PICO_DEFAULT_LED_PIN
#define PICO_DEFAULT_LED_PIN 25
#endif

#ifndef PICO_DEFAULT_WS2812_PIN
#define PICO_DEFAULT_WS2812_PIN 22
#endif

#ifndef PICO_DEFAULT_WS2812_POWER_PIN
#define PICO_DEFAULT_WS2812_POWER_PIN 23
#endif

//------------- I2C -------------//
#ifndef PICO_DEFAULT_I2C
#define PICO_DEFAULT_I2C 1
#endif

#ifndef PICO_DEFAULT_I2C_SDA_PIN
#define PICO_DEFAULT_I2C_SDA_PIN 6
#endif

#ifndef PICO_DEFAULT_I2C_SCL_PIN
#define PICO_DEFAULT_I2C_SCL_PIN 7
#endif

#ifndef PICO_I2C0_SDA_PIN
#define PICO_I2C0_SDA_PIN 16
#endif

#ifndef PICO_I2C0_SCL_PIN
#define PICO_I2C0_SCL_PIN 17
#endif

//------------- SPI -------------//
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

#ifndef PICO_SPI1_CSN_PIN
#define PICO_SPI1_CSN_PIN 9
#endif

#ifndef PICO_SPI1_SCK_PIN
#define PICO_SPI1_SCK_PIN 10
#endif

#ifndef PICO_SPI1_TX_PIN
#define PICO_SPI1_TX_PIN 11
#endif

#ifndef PICO_SPI1_RX_PIN
#define PICO_SPI1_RX_PIN 12
#endif

//------------- ADC -------------//
#ifndef PICO_ADC_A0_PIN
#define PICO_ADC_A0_PIN 26
#endif 

#ifndef PICO_ADC_A1_PIN
#define PICO_ADC_A1_PIN 27
#endif 

#ifndef PICO_ADC_A2_PIN
#define PICO_ADC_A2_PIN 28
#endif 

#ifndef PICO_ADC_BAT_PIN
#define PICO_ADC_BAT_PIN 29
#endif 
#ifndef PICO_ADC_BAT_EN_PIN
#define PICO_ADC_BAT_EN_PIN 19
#endif

//------------- FLASH -------------//

#define PICO_BOOT_STAGE2_CHOOSE_W25Q080 1

#ifndef PICO_FLASH_SPI_CLKDIV
#define PICO_FLASH_SPI_CLKDIV 2
#endif

// pico_cmake_set_default PICO_FLASH_SIZE_BYTES = (4 * 1024 * 1024)
#ifndef PICO_FLASH_SIZE_BYTES
#define PICO_FLASH_SIZE_BYTES (4 * 1024 * 1024)
#endif

#ifndef PICO_RP2350_A2_SUPPORTED
#define PICO_RP2350_A2_SUPPORTED 1
#endif

#endif
