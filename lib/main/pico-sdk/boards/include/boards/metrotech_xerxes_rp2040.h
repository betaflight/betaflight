/*
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

// -----------------------------------------------------
// NOTE: THIS HEADER IS ALSO INCLUDED BY ASSEMBLER SO
//       SHOULD ONLY CONSIST OF PREPROCESSOR DIRECTIVES
// -----------------------------------------------------

// pico_cmake_set PICO_PLATFORM=rp2040

#ifndef _BOARDS_METROTECH_XERXES_RP2040_H
#define _BOARDS_METROTECH_XERXES_RP2040_H

#define USR_SW_PIN 18
#define USR_BTN_PIN 24
#define USR_LED_PIN 25

#define UART1_TX_PIN 8
#define UART1_RX_PIN 9

#define UART0_TX_PIN 16
#define UART0_RX_PIN 17

#define SPI0_MISO_PIN 0
#define SPI0_CSN_PIN 1
#define SPI0_CLK_PIN 2
#define SPI0_MOSI_PIN 3

#define I2C0_SDA_PIN 4
#define I2C0_SCL_PIN 5

#define EXT_3V3_EN_PIN 6

#define RS_EN_PIN 19
#define RS_TX_PIN UART0_TX_PIN
#define RS_RX_PIN UART0_RX_PIN

#define ADC0_PIN 26 // ADC0 and PWM5_A share the same pin
#define ADC1_PIN 27 // ADC1 and PWM5_B share the same pin
#define ADC2_PIN 28 // ADC2 and PWM6_A share the same pin
#define ADC3_PIN 29 // ADC3 and PWM6_B share the same pin

#define TMP0_PIN 22 // TMP0 and I2C1 share the same pin
#define TMP1_PIN 23 // TMP1 and I2C1 share the same pin

#define I2C1_SDA_PIN 22 // I2C1 and TMP0 share the same pin
#define I2C1_SCL_PIN 23 // I2C1 and TMP1 share the same pin

#define PWM0_A_PIN 0  // PWM0_A and SPI0_MISO share the same pin
#define PWM0_B_PIN 1  // PWM0_B and SPI0_CSN share the same pin
#define PWM1_A_PIN 2  // PWM1_A and SPI0_CLK share the same pin
#define PWM1_B_PIN 3  // PWM1_B and SPI0_MOSI share the same pin
#define PWM2_A_PIN 4  // PWM2_A and I2C0_SDA share the same pin
#define PWM2_B_PIN 5  // PWM2_B and I2C0_SCL share the same pin
#define PWM3_A_PIN 22 // PWM3_A and TMP0 share the same pin
#define PWM3_B_PIN 23 // PWM3_B and TMP1 share the same pin
#define PWM4_A_PIN 24 // PWM4_A and USR_BTN share the same pin
#define PWM4_B_PIN 25 // PWM4_B and USR_LED share the same pin
#define PWM5_A_PIN 26 // PWM5_A and ADC0 share the same pin
#define PWM5_B_PIN 27 // PWM5_B and ADC1 share the same pin
#define PWM6_A_PIN 28 // PWM6_A and ADC2 share the same pin
#define PWM6_B_PIN 29 // PWM6_B and ADC3 share the same pin

#define CLK_GPIN1_PIN 22  // CLK_GPIN1 and TMP0 share the same pin
#define CLK_GPOUT1_PIN 23 // CLK_GPOUT1 and TMP1 share the same pin

/// @brief Mask of pins that are used by the shield and not used by the board, eq. 0x3fc0033f
#define SHIELD_MASK 1 << SPI0_MISO_PIN | 1 << SPI0_CSN_PIN | 1 << SPI0_CLK_PIN | 1 << SPI0_MOSI_PIN | \
                        1 << I2C0_SDA_PIN | 1 << I2C0_SCL_PIN |                                       \
                        1 << UART1_TX_PIN | 1 << UART1_RX_PIN |                                       \
                        1 << ADC0_PIN | 1 << ADC1_PIN | 1 << ADC2_PIN | 1 << ADC3_PIN |               \
                        1 << TMP0_PIN | 1 << TMP1_PIN

// For board detection
#define XERXES_RP2040

// --- UART ---
#ifndef PICO_DEFAULT_UART
#define PICO_DEFAULT_UART 0
#endif
#ifndef PICO_DEFAULT_UART_TX_PIN
#define PICO_DEFAULT_UART_TX_PIN UART0_TX_PIN
#endif
#ifndef PICO_DEFAULT_UART_RX_PIN
#define PICO_DEFAULT_UART_RX_PIN UART0_RX_PIN
#endif

// --- LED ---
#ifndef PICO_DEFAULT_LED_PIN
#define PICO_DEFAULT_LED_PIN USR_LED_PIN
#endif
// no PICO_DEFAULT_WS2812_PIN

// --- I2C ---
#ifndef PICO_DEFAULT_I2C
#define PICO_DEFAULT_I2C 0
#endif
#ifndef PICO_DEFAULT_I2C_SDA_PIN
#define PICO_DEFAULT_I2C_SDA_PIN I2C0_SDA_PIN
#endif
#ifndef PICO_DEFAULT_I2C_SCL_PIN
#define PICO_DEFAULT_I2C_SCL_PIN I2C0_SCL_PIN
#endif

// --- SPI ---
#ifndef PICO_DEFAULT_SPI
#define PICO_DEFAULT_SPI 0
#endif
#ifndef PICO_DEFAULT_SPI_SCK_PIN
#define PICO_DEFAULT_SPI_SCK_PIN SPI0_CLK_PIN
#endif
#ifndef PICO_DEFAULT_SPI_TX_PIN
#define PICO_DEFAULT_SPI_TX_PIN SPI0_MOSI_PIN
#endif
#ifndef PICO_DEFAULT_SPI_RX_PIN
#define PICO_DEFAULT_SPI_RX_PIN SPI0_MISO_PIN
#endif
#ifndef PICO_DEFAULT_SPI_CSN_PIN
#define PICO_DEFAULT_SPI_CSN_PIN SPI0_CSN_PIN
#endif

// --- FLASH ---
#define PICO_BOOT_STAGE2_CHOOSE_W25Q080 1

#ifndef __CLKDIV
#define __CLKDIV 4
#endif // !__CLKDIV

#ifndef PICO_FLASH_SPI_CLKDIV
/**
 * @brief divisor for SPI clock (default 4), must be divisible by 2
 *
 * This is critical for some flash chips, and can be set to 4 for some,
 * but needs to be at least 32 for others. Set using -DCLKDIV=<N*2> in cmake.
 * @note Increasing this value will help with BOOTLOOPs, but will slow down the flash chip.
 */
#define PICO_FLASH_SPI_CLKDIV __CLKDIV
#endif // !PICO_FLASH_SPI_CLKDIV

#ifndef PICO_XOSC_STARTUP_DELAY_MULTIPLIER
#define PICO_XOSC_STARTUP_DELAY_MULTIPLIER 16
#endif // !PICO_XOSC_STARTUP_DELAY_MULTIPLIER

// pico_cmake_set_default PICO_FLASH_SIZE_BYTES = (16 * 1024 * 1024)
#ifndef PICO_FLASH_SIZE_BYTES
/**
 * @brief 16MiB, Flash size in bytes
 *
 * This is the size of the flash chip on the board, not the size of the flash chip that is used for the bootloader.
 */
#define PICO_FLASH_SIZE_BYTES (16 * 1024 * 1024)
#endif // !PICO_FLASH_SIZE_BYTES

// All boards have B1 RP2040
#ifndef PICO_RP2040_B0_SUPPORTED
#define PICO_RP2040_B0_SUPPORTED 0
#endif // !PICO_RP2040_B0_SUPPORTED

#endif // _BOARDS_METROTECH_XERXES_RP2040_H