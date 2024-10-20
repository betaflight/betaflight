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

#ifndef _BOARDS_DEFCON32_BADGE_H
#define _BOARDS_DEFCON32_BADGE_H

// For board detection
#define DEFCON32_BADGE

// --- RP2350 VARIANT ---
#define PICO_RP2350A 1

#define DEFCON32_BADGE_SRAM_CS_PIN             0
#define DEFCON32_BADGE_TOUCH_INT_PIN           1
#define DEFCON32_BADGE_I2C_SDA_PIN             2
#define DEFCON32_BADGE_I2C_SDL_PIN             3
#define DEFCON32_BADGE_WS2812_PIN              4
#define DEFCON32_BADGE_DISPLAY_RS_PIN          5
#define DEFCON32_BADGE_DISPLAY_DO_PIN          6
#define DEFCON32_BADGE_IR_SD_PIN               7
#define DEFCON32_BADGE_DISPLAY_SCK_PIN         8
#define DEFCON32_BADGE_DISPLAY_CS_PIN          9
#define DEFCON32_BADGE_DISPLAY_BL_PIN          10
#define DEFCON32_BADGE_SYS_POWER_CONTROL_PIN   11
#define DEFCON32_BADGE_SPI_MISO_PIN            12
#define DEFCON32_BADGE_SD_CS_PIN               13
#define DEFCON32_BADGE_SPI_CK_PIN              14
#define DEFCON32_BADGE_SPI_MOSI_PIN            15
#define DEFCON32_BADGE_SW_RIGHT_PIN            16
#define DEFCON32_BADGE_SW_DOWN_PIN             17
#define DEFCON32_BADGE_SW_UP_PIN               18
#define DEFCON32_BADGE_SW_LEFT_PIN             19
#define DEFCON32_BADGE_SW_B_PIN                20
#define DEFCON32_BADGE_SW_A_PIN                21
#define DEFCON32_BADGE_SW_START_PIN            22
#define DEFCON32_BADGE_SW_SELECT_PIN           23
#define DEFCON32_BADGE_SW_FN_PIN               24
#define DEFCON32_BADGE_SPEAKER_OUT_PIN         25
#define DEFCON32_BADGE_IR_RX_PIN               27
#define DEFCON32_BADGE_IR_TX_PIN               28

// --- UART ---
// NOTE: since there is no UART on the badge, you should probably pass:
// -DPICO_BOARD=defcon32_badge -DPICO_STDIO_USB=1 -DPICO_STDIO_UART=0
// when building to set up stdio over USB CDC by default

// --- LED ---
// no PICO_DEFAULT_LED_PIN
#ifndef PICO_DEFAULT_WS2812_PIN
#define PICO_DEFAULT_WS2812_PIN DEFCON32_BADGE_WS2812_PIN
#endif

// --- I2C ---
#ifndef PICO_DEFAULT_I2C
#define PICO_DEFAULT_I2C 1
#endif
#ifndef PICO_DEFAULT_I2C_SDA_PIN
#define PICO_DEFAULT_I2C_SDA_PIN DEFCON32_BADGE_I2C_SDA_PIN
#endif
#ifndef PICO_DEFAULT_I2C_SCL_PIN
#define PICO_DEFAULT_I2C_SCL_PIN DEFCON32_BADGE_I2C_SDL_PIN
#endif

// --- SPI ---
#ifndef PICO_DEFAULT_SPI
#define PICO_DEFAULT_SPI 1
#endif
#ifndef PICO_DEFAULT_SPI_SCK_PIN
#define PICO_DEFAULT_SPI_SCK_PIN DEFCON32_BADGE_SPI_CK_PIN
#endif
#ifndef PICO_DEFAULT_SPI_TX_PIN
#define PICO_DEFAULT_SPI_TX_PIN DEFCON32_BADGE_SPI_MOSI_PIN
#endif
#ifndef PICO_DEFAULT_SPI_RX_PIN
#define PICO_DEFAULT_SPI_RX_PIN DEFCON32_BADGE_SPI_MISO_PIN
#endif
// multiple devices, so this doesn't make much sense
// no PICO_DEFAULT_SPI_CSN_PIN

#ifndef PICO_AUDIO_PWM_L_PIN
#define PICO_AUDIO_PWM_L_PIN DEFCON32_BADGE_SPEAKER_OUT_PIN
#endif

#ifndef PICO_AUDIO_PWM_MONO_PIN
#define PICO_AUDIO_PWM_MONO_PIN PICO_AUDIO_PWM_L_PIN
#endif

// --- FLASH ---

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


