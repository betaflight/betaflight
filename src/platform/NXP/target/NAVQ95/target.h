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

// Configuration is stored in the FlexSPI boot NOR, two 4 KB sectors.
#define CONFIG_IN_FLASH
#define EEPROM_SIZE                        8192
#define FLASH_CONFIG_STREAMER_BUFFER_SIZE  256
#define FLASH_CONFIG_BUFFER_TYPE           uint32_t
#define NAVQ95_USE_FLEXSPI_NOR

// The VCP port is the LPUART2 console.
#define USE_VCP

#define USE_UART
#define USE_UART5
#define USE_UART7
#define USE_USART7
#define SERIAL_UART_FIRST_INDEX 1

// UART5 pads are muxed over SCMI; the pin tags are placeholders that are never driven.
#define UART5_TX_PIN            PA7
#define UART5_RX_PIN            PA6
#define UART7_TX_PIN            PA8
#define UART7_RX_PIN            PA9

#define MSP_UART                SERIAL_PORT_UART7

#define USE_TARGET_CONFIG

// DShot is not supported yet.
#define DEFAULT_MOTOR_PROTOCOL  MOTOR_PROTOCOL_PWM

#define USE_BEEPER
#define BEEPER_PIN              PB23
#define BEEPER_INVERTED

#define USE_SPI_GYRO
#define GYRO_1_SPI_INSTANCE     SPI1
#define GYRO_1_CS_PIN           PA11

#define USE_SDCARD
#define USE_SDCARD_SDIO
#define SDIO_DEVICE             SDIODEV_1
#define SDIO_USE_4BIT           true
#define DEFAULT_BLACKBOX_DEVICE BLACKBOX_DEVICE_SDCARD

// I2C1 uses dedicated pads with no GPIO tag.
#define USE_I2C_DEVICE_1
#define I2C1_SCL_PIN            NONE
#define I2C1_SDA_PIN            NONE
#define I2C1_CLOCKSPEED         100

#define USE_I2C_DEVICE_6
#define I2C6_SCL_PIN            PA3
#define I2C6_SDA_PIN            PA2
#define I2C6_CLOCKSPEED         100

#define MAG_I2C_INSTANCE        I2CDEV_1
#define MAG_I2C_ADDRESS         0x14

#define BARO_I2C_INSTANCE       I2CDEV_1
#define DEFAULT_BARO_I2C_ADDRESS 0x46

#include "platform.h"

void navq95_timebaseTick(void);
void navq95_timebaseInit(void);
