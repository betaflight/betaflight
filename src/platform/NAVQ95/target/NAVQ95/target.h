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

/*
 * target.h — NavQ95 (NXP i.MX 95, Cortex-M7) base target.
 */

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
 * Configuration storage in RAM (volatile). On NavQ95 the FlexSPI boot NOR is
 * owned by the System Manager/U-Boot (the M7 cannot drive the FlexSPI
 * controller to erase/program it), so a direct-M7 flash config backend does not
 * work. Board RX/sensor defaults come up every boot via targetConfiguration()
 * (config_navq95.c). The real systemReset() (SCMI warm reset, platform_stubs.c)
 * is implemented — `save` reboots cleanly, it just does not persist across
 * power cycles yet.
 */
#define CONFIG_IN_RAM
#define EEPROM_SIZE             8192

/*
 * Expose one logical serial port (USB VCP) so SERIAL_PORT_COUNT >= 1. The port
 * is backed by the LPUART5 console in platform_stubs.c and carries the CLI.
 * (The console was moved off LPUART2 to LPUART5 so RC/SBUS can own LPUART2's
 * dedicated, HW-proven pads.)
 */
#define USE_VCP

/*
 * Generic serial UARTs, via Betaflight's drivers/serial_uart.c core plus the
 * i.MX 95 platform layer in NAVQ95/serial_uart_navq95.c (uartHardware[] table).
 * On i.MX 95 every UART is an LPUART; like the i.MX RT reference port we map
 * LPUART instances onto the regular UART identifiers.
 *
 *   UART2 == LPUART2 : RC / SBUS receiver (dedicated UART2_RXD/TXD pads, ALT0).
 *   UART7 == LPUART7 : MSP for the Betaflight Configurator (shared GPIO_IO08/09
 *                      pads, ALT5 + RX SELECT_INPUT daisy).
 *
 * The actual LPUART alt mux + pad electrical + any RX SELECT_INPUT daisy are
 * applied over SCMI by the driver (the SM owns IOMUXC). SBUS framing (8E2 +
 * inverted RX) is applied in uartReconfigure() from the port options.
 *
 * SERIAL_UART_FIRST_INDEX = 1: UART numbering starts at 1 (no UART0).
 */
#define USE_UART
#define USE_UART2
#define USE_UART7
/*
 * Also define USE_USART2/USE_USART7: pg/serial_uart.c gates each UART entry of
 * its uartDmaopt[] table on the USE_USARTn spelling. Without them the table
 * could be empty and its `for (i = 0; i < ARRAYLEN(uartDmaopt); i++)` trips
 * -Werror=type-limits. The DMA opts default to DMA_OPT_UNUSED; inert otherwise.
 */
#define USE_USART2
#define USE_USART7
#define SERIAL_UART_FIRST_INDEX 1

/* UART2 == LPUART2 (RC/SBUS). Pads: TX = UART2_TXD, RX = UART2_RXD (dedicated,
 * ALT0, no daisy). The pin tags pick the uartHardware[] rows; the SCMI pinmux
 * uses the dedicated-pad path. */
#define UART2_TX_PIN            NONE
#define UART2_RX_PIN            NONE

/* UART7 == LPUART7 (MSP). Pads: TX = GPIO_IO08 = PA8, RX = GPIO_IO09 = PA9
 * (RGPIO1 bits 8/9, LPUART7 ALT5). */
#define UART7_TX_PIN            PA8
#define UART7_RX_PIN            PA9

/*
 * MSP over UART7. pg/msp.c claims serial ports for FUNCTION_MSP at config reset:
 * slot 0 is always the first port (the VCP console, so the CLI stays reachable),
 * and MSP_UART is claimed as an additional MSP slot. Defining it here makes the
 * Betaflight Configurator connect over UART7 (LPUART7, 115200 8N1) out of the
 * box, while the LPUART5 console keeps the CLI. Can still be reassigned from the
 * Configurator's Ports tab / `msp_2_uart` CLI setting.
 */
#define MSP_UART               SERIAL_PORT_UART7

/*
 * Bake the board RX defaults in via targetConfiguration() (config_navq95.c):
 * FEATURE_RX_SERIAL + SBUS provider (inverted) on UART2. Needed because
 * CONFIG_IN_RAM is volatile and does not persist across power cycles, so the
 * SBUS port must come up at boot. USE_TARGET_CONFIG makes resetConfig() call it
 * every boot.
 */
#define USE_TARGET_CONFIG

/*
 * Beeper (buzzer). GPIO-driven (no PWM): sound_beeper.c toggles the pin via the
 * IO HAL when BEEPER_PWM_HZ is 0 (the default). The pad is GPIO_IO23 = GPIO2_IO23
 * = Betaflight tag PB23 (RGPIO2[23]); its IOMUXC mux row lives in io_navq95.c
 * (navq95_padMux[]). beeperInit() -> IOConfigGPIO(PB23, ...) brings the pin up.
 *
 * Polarity (see pg/beeper_dev.c): the NavQ95 buzzer is driven active-high /
 * push-pull, so BEEPER_INVERTED is defined (IS_INVERTED=true, IS_OPEN_DRAIN=
 * false) — systemBeep(true) then drives the pin HIGH to sound the buzzer.
 */
#define USE_BEEPER
#define BEEPER_PIN              PB23
#define BEEPER_INVERTED

/*
 * Sensors: real 6-axis IMU (ICM-45686) over LPSPI1, polled register-level SPI.
 * MAG/BARO stay virtual (set in mk/NAVQ95.mk) until their I2C bus is brought up.
 * The gyro device config (bus + CS) is built by pg/gyrodev.c from these:
 *   GYRO_1_SPI_INSTANCE -> SPI1 (== LPSPI1), GYRO_1_CS_PIN -> GPIO1_IO11 (PA11).
 * EXTI/CLKIN/alignment default (NONE / CW0) in pg/gyrodev.c. DRDY/EXTI is left
 * NONE for now: the driver uses polled reads (no data-ready interrupt yet).
 */
#define USE_SPI_GYRO
#define GYRO_1_SPI_INSTANCE     SPI1
#define GYRO_1_CS_PIN           PA11

/*
 * I2C (Phase 1 bring-up): two LPI2C master instances, register-level polled
 * driver in NAVQ95/bus_i2c_navq95.c. Option-A Betaflight bus numbering so the
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
 * 64-bit DWT CYCCNT time base helper. navq95_timebaseTick() folds the CYCCNT
 * wrap and must be called periodically (from SysTick, once configured) so no
 * wrap is lost. navq95_timebaseInit() configures the Cortex-M7 SysTick at 1 kHz
 * to drive that fold; call it after cycleCounterInit(). The 32-bit
 * micros()/millis()/microsISR() primitives (declared in drivers/time.h) are
 * built on the same fold.
 */
void navq95_timebaseTick(void);
void navq95_timebaseInit(void);
