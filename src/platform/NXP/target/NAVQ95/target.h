/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

/* target.h — NavQ95 (NXP i.MX 95, Cortex-M7) base target. */

#pragma once

#ifndef TARGET_BOARD_IDENTIFIER
#define TARGET_BOARD_IDENTIFIER "N095"
#endif

#ifndef USBD_PRODUCT_STRING
#define USBD_PRODUCT_STRING     "Betaflight - NXP NavQ95 (i.MX95 M7)"
#endif

#ifndef IMX95_M7
#define IMX95_M7
#endif

/*
 * Configuration storage in the FlexSPI boot NOR (persistent across power
 * cycles). CONFIG_IN_FLASH is platform-owned via config_streamer_impl.h:
 * NXP/config_flash_navq95.c implements configUnlock/Lock/ClearFlags/WriteWord
 * on the FlexSPI NOR driver; reads come from the memory-mapped NOR
 * (__config_start). Streamer word = one 256-byte NOR page; EEPROM_SIZE 8192 =
 * 2 x 4 KB sectors (see navq95_flexspi.ld __config_start/__config_end).
 */
#define CONFIG_IN_FLASH
#define EEPROM_SIZE                        8192
#define FLASH_CONFIG_STREAMER_BUFFER_SIZE  256
#define FLASH_CONFIG_BUFFER_TYPE           uint32_t

/*
 * FlexSPI NOR driver (MX25UM51345GXDI00 octal NOR, NXP/flexspi_nor_imx95.c).
 * NAVQ95_USE_FLEXSPI_NOR gates our self-contained erase/program driver (required
 * by CONFIG_IN_FLASH); we do NOT use USE_FLASH_MEMORY_MAPPED (STM32-OctoSPI
 * coupled). NAVQ95_NOR_PROBE = optional non-fatal boot NOR health probe.
 */
#define NAVQ95_USE_FLEXSPI_NOR
/* #define NAVQ95_NOR_PROBE */   /* boot NOR probe prints — off for release */

/* USB VCP logical port (CLI), backed by the LPUART5 console. */
#define USE_VCP

/*
 * Generic serial UARTs via drivers/serial_uart.c + the i.MX 95 platform layer
 * in NXP/serial_uart_navq95.c. UART5 == LPUART5 (RC/SBUS), UART7 == LPUART7
 * (MSP). Alt mux + pad + RX daisy + SBUS framing applied over SCMI by the
 * driver. SERIAL_UART_FIRST_INDEX = 1: no UART0.
 */
#define USE_UART
#define USE_UART5
#define USE_UART7
#define USE_USART7
#define SERIAL_UART_FIRST_INDEX 1

/*
 * UART5 pins MUST be non-NONE placeholders: SERIAL_TRAIT_PIN_CONFIG makes
 * serialInit() drop any UART with neither RX nor TX ioTag. serialUART() skips
 * IOInit for UART5, so these are never driven — the real LPUART5 pads are muxed
 * over SCMI. UART7 pads: TX = GPIO_IO08 = PA8, RX = GPIO_IO09 = PA9.
 */
#define UART5_TX_PIN            PA7
#define UART5_RX_PIN            PA6
#define UART7_TX_PIN            PA8
#define UART7_RX_PIN            PA9

/* MSP (+ CLI) over UART7 for the Configurator; reassignable from Ports tab. */
#define MSP_UART               SERIAL_PORT_UART7

/* Bake board RX defaults via targetConfiguration() (config_navq95.c). */
#define USE_TARGET_CONFIG

/*
 * Force STANDARD PWM: common_pre.h defines USE_DSHOT unconditionally, which
 * would default to DSHOT600 -> dshotPwmDevInit() (stubbed; DShot needs an
 * eDMA/FlexIO layer not implemented). pg/motor.c honours DEFAULT_MOTOR_PROTOCOL
 * above every fallback, so this routes to our TPM PWM driver (motorPwmDevInit,
 * NXP/pwm_navq95.c). MOTOR_PROTOCOL_PWM = enum in drivers/motor_types.h.
 */
#define DEFAULT_MOTOR_PROTOCOL  MOTOR_PROTOCOL_PWM

/*
 * Beeper: GPIO-driven (no PWM) on GPIO_IO23 = PB23 (pad row in io_navq95.c).
 * Active-high push-pull -> BEEPER_INVERTED (systemBeep(true) drives the pin
 * HIGH).
 */
#define USE_BEEPER
#define BEEPER_PIN              PB23
#define BEEPER_INVERTED

/*
 * IMU: ICM-45686 6-axis over LPSPI1 (polled register-level SPI). pg/gyrodev.c
 * builds the device config from these; EXTI/CLKIN/align default (NONE/CW0),
 * DRDY left NONE (polled reads, no data-ready IRQ yet).
 */
#define USE_SPI_GYRO
#define GYRO_1_SPI_INSTANCE     SPI1
#define GYRO_1_CS_PIN           PA11

/*
 * SD card over the i.MX 95 uSDHC controller (NXP/sdio_usdhc_navq95.c, ADMA2
 * polled). Per the NavQ95 schematic the microSD is wired to uSDHC2 via the SD2
 * pads (SD2_CLK/CMD/DATA0..3, 4-bit). Config storage stays in the FlexSPI NOR
 * (above), so an SD failure is non-fatal. NAVQ95_SD_SELFTEST runs a boot
 * clock-grant scan + SD2 pinmux probe + non-destructive card-identify (gated so
 * a wrong guess cannot hang the bus); leave it off for normal use.
 */
#define USE_SDCARD
#define USE_SDCARD_SDIO
#define SDIO_DEVICE             SDIODEV_1
#define SDIO_USE_4BIT           false
/* #define NAVQ95_SD_SELFTEST */

/*
 * NAVQ95_PWM_SELFTEST — BENCH-ONLY boot probe for the motor PWM: prints TPM
 * clock/pinmux/MOD/SC then drives a static 1500 us pulse on all channels for a
 * scope/logic-analyzer. *** PROPS OFF — drives the ESC signal lines. ***
 */
/* #define NAVQ95_PWM_SELFTEST */   /* boot PWM probe prints — off for normal use */

/*
 * I2C (Phase 1 bring-up): two LPI2C master instances, register-level polled
 * driver in NXP/bus_i2c_navq95.c. Option-A Betaflight bus numbering so the
 * CLI/Configurator bus number matches the silicon instance:
 *
 *   I2CDEV_1 == LPI2C1 (AONMIX,    base 0x44340000, CCM clock root 6)
 *              dedicated I2C1_SCL / I2C1_SDA pads (ALT0, no SELECT_INPUT daisy).
 *   I2CDEV_6 == LPI2C6 (WAKEUPMIX, base 0x426C0000, CCM clock root 91)
 *              shared GPIO_IO02 (SDA) / GPIO_IO03 (SCL) pads at ALT6, each with
 *              a SELECT_INPUT daisy. In Betaflight tag terms GPIO_IO02 = PA2 and
 *              GPIO_IO03 = PA3 (RGPIO1 bits 2/3).
 *
 * The LPI2C pad mux + functional-clock ungate are done over SCMI by the System
 * Manager inside the driver (the M7 owns neither IOMUXC nor the CCM), so the
 * pin TAGS here are cosmetic (CLI resource listing). The dedicated I2C1 pads
 * have no GPIO tag, so they are NONE.
 *
 * Clock speed is 100 kHz for bring-up (external pull-ups are present on both
 * buses, so no internal pull-up is enabled -> I2Cn_PULLUP left default false).
 * I2CDEV_COUNT = 6 (platform.h): enum I2CDEV_1..I2CDEV_6 map to indices 0..5.
 *
 * NOTE: USE_I2C itself is defined on the command line in mk/NAVQ95.mk (mirroring
 * USE_SPI), NOT here - defining it in both places collides under -Werror
 * ("USE_I2C" redefined). Only the per-device USE_I2C_DEVICE_n spellings and the
 * pin/clock config live in target.h.
 */
#define USE_I2C_DEVICE_1
#define I2C1_SCL_PIN            NONE
#define I2C1_SDA_PIN            NONE
#define I2C1_CLOCKSPEED         100

#define USE_I2C_DEVICE_6
#define I2C6_SCL_PIN            PA3
#define I2C6_SDA_PIN            PA2
#define I2C6_CLOCKSPEED         100

/*
 * Magnetometer (Phase 5): Bosch BMM350 on LPI2C1 (I2CDEV_1) at I2C address 0x14
 * (ADSEL = GND; 0x15 is ADSEL = VDDIO). Confirmed present on i2c1 by the bus
 * scan. The real BMM350 driver is enabled by removing -DUSE_VIRTUAL_MAG in
 * mk/NAVQ95.mk; with USE_MAG set and USE_VIRTUAL_MAG gone, common_post.h turns
 * on the mag driver list (incl. USE_MAG_BMM350) and compass_bmm350.c (already
 * in COMMON_SRC) is compiled. pg/compass.c's pgResetFn_compassConfig() seeds the
 * default bus/address from MAG_I2C_INSTANCE / MAG_I2C_ADDRESS below.
 *   MAG_I2C_INSTANCE -> I2CDEV_1 (LPI2C1)
 *   MAG_I2C_ADDRESS  -> 0x14     (BMM350, ADSEL=GND)
 * The BMM350 needs a 2.2 uF cap on CRST for magnetic reset / full heading
 * accuracy (per the driver header).
 */
#define MAG_I2C_INSTANCE        I2CDEV_1
#define MAG_I2C_ADDRESS         0x14

/*
 * Barometer (Phase 6): Bosch BMP581 on LPI2C1 (I2CDEV_1) at I2C address 0x46
 * (SDO = LOW; BMP5XX_I2C_ADDR_SECONDARY — 0x47 is SDO = HIGH / primary).
 * Confirmed present on i2c1 by the bus scan. The BMP581 I2C path is forced on
 * the command line in mk/NAVQ95.mk (USE_BARO + USE_BARO_BMP581 +
 * DEFAULT_BARO_BMP581 + DEFAULT_BARO_DEVICE=BARO_BMP581), because the default
 * baro list only ships the SPI BMP5xx variants and its auto-selector never
 * picks BMP5xx. pg/barometer.c's pgResetFn_barometerConfig() then seeds the
 * default bus/address from BARO_I2C_INSTANCE / DEFAULT_BARO_I2C_ADDRESS below.
 *   BARO_I2C_INSTANCE        -> I2CDEV_1 (LPI2C1)
 *   DEFAULT_BARO_I2C_ADDRESS -> 0x46     (BMP581, SDO=LOW)
 * NOTE: the baro PG reads DEFAULT_BARO_I2C_ADDRESS (not BARO_I2C_ADDRESS). If
 * left 0, bmp5xxDetect() auto-probes 0x47 then 0x46; we set 0x46 explicitly to
 * skip the (absent) 0x47 probe. barometer_bmp5xx.c reads CHIP_ID (BMP581=0x51).
 */
#define BARO_I2C_INSTANCE       I2CDEV_1
#define DEFAULT_BARO_I2C_ADDRESS 0x46

/* Pull in the platform abstraction (CMSIS includes, type shims, capabilities). */
#include "platform.h"

/*
 * 64-bit DWT CYCCNT time base. navq95_timebaseTick() folds the CYCCNT wrap and
 * is called from SysTick (1 kHz); navq95_timebaseInit() configures it after
 * cycleCounterInit(). micros()/millis()/microsISR() build on the same fold.
 */
void navq95_timebaseTick(void);
void navq95_timebaseInit(void);
