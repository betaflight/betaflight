/*
 * Copyright (c) 2023 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

// -----------------------------------------------------
// NOTE: THIS HEADER IS ALSO INCLUDED BY ASSEMBLER SO
//       SHOULD ONLY CONSIST OF PREPROCESSOR DIRECTIVES
// -----------------------------------------------------

// pico_cmake_set PICO_PLATFORM=rp2040

#ifndef _BOARDS_POLOLU_ZUMO_2040_ROBOT_H
#define _BOARDS_POLOLU_ZUMO_2040_ROBOT_H

// For board detection
#define POLOLU_ZUMO_2040_ROBOT

#define PICO_DEFAULT_LED_PIN 25
#define PICO_BOOT_STAGE2_CHOOSE_W25Q080 1
#define PICO_FLASH_SPI_CLKDIV 2
// pico_cmake_set_default PICO_FLASH_SIZE_BYTES = (16 * 1024 * 1024)
#define PICO_FLASH_SIZE_BYTES (16 * 1024 * 1024)

// All boards have at least the B1 revision
#define PICO_RP2040_B0_SUPPORTED 0

#endif
