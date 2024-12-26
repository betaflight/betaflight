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

#ifndef RP2350B
#define RP2350B
#endif

#ifndef TARGET_BOARD_IDENTIFIER
#define TARGET_BOARD_IDENTIFIER "235B"
#endif

#ifndef USBD_PRODUCT_STRING
#define USBD_PRODUCT_STRING     "Betaflight RP2350B"
#endif

#define USE_IO
#define USE_UART0
#define USE_UART1

#define USE_SPI
#define USE_SPI_DEVICE_1
#define USE_SPI_DEVICE_2

#undef USE_SOFTSERIAL1
#undef USE_SOFTSERIAL2

#undef USE_VCP

#undef USE_TRANSPONDER
#undef USE_DMA
#undef USE_FLASH
#undef USE_FLASH_CHIP
#undef USE_SDCARD

#undef USE_TIMER
#undef USE_I2C
#undef USE_UART
#undef USE_DSHOT
#undef USE_RCC
#undef USE_CLI
#undef USE_PWM_OUTPUT
#undef USE_RX_PWM
#undef USE_RX_PPM
#undef USE_RX_SPI
#undef USE_RX_CC2500
#undef USE_SERIAL_4WAY_BLHELI_INTERFACE
#undef USE_SERIAL_4WAY_BLHELI_BOOTLOADER
#undef USE_SERIAL_4WAY_SK_BOOTLOADER
#undef USE_MULTI_GYRO
#undef USE_BARO



#undef USE_RANGEFINDER_HCSR04
#undef USE_CRSF
#undef USE_TELEMETRY_CRSF
#undef USE_RX_EXPRESSLRS
#undef USE_MAX7456
#undef USE_MAG
#undef USE_MAG_HMC5883
#undef USE_MAG_SPI_HMC5883
#undef USE_VTX_RTC6705
#undef USE_VTX_RTC6705_SOFTSPI
#undef USE_RX_SX1280
#undef USE_SRXL
#undef USE_TELEMETRY
#undef USE_OSD
#undef USE_SPEKTRUM
#undef USE_SPEKTRUM_BIND
#undef USE_MSP
#undef USE_MSP_UART
#undef USE_GPS
#undef USE_GPS_UBLOX
#undef USE_GPS_MTK
#undef USE_GPS_NMEA
#undef USE_GPS_SERIAL
#undef USE_GPS_SONAR
#undef USE_GPS_UBLOX7
#undef USE_GPS_UBLOX8
#undef USE_GPS_RESCUE

#undef USE_VTX
#undef USE_VTX_TRAMP
#undef USE_VTX_SMARTAUDIO
#undef USE_SPEKTRUM_VTX_CONTROL
#undef USE_VTX_COMMON
#undef USE_FLASH
#undef USE_FLASH_CHIP
#undef USE_FLASH_M25P16
#undef USE_FLASH_W25N01G
#undef USE_FLASH_W25N02K
#undef USE_FLASH_W25M
#undef USE_FLASH_W25M512
#undef USE_FLASH_W25M02G
#undef USE_FLASH_W25Q128FV
#undef USE_FLASH_PY25Q128HA
#undef USE_FLASH_W25Q64FV

#define FLASH_PAGE_SIZE                     0x4000
#define CONFIG_IN_FLASH
#define FLASH_CONFIG_STREAMER_BUFFER_SIZE   4
#define FLASH_CONFIG_BUFFER_ALIGN_TYPE      uint32_t

/* to be moved to a config file once target if working */
#define LED0_PIN             PA6
#define LED1_PIN             PA7

#define SPI0_SCK_PIN         PA5
#define SPI0_SDI_PIN         PA6
#define SPI0_SDO_PIN         PA7

#define SPI1_SCK_PIN         PA26
#define SPI1_SDI_PIN         PA24
#define SPI1_SDO_PIN         PA27

#define SDCARD_CS_PIN        PA25
#define FLASH_CS_PIN         PA25
#define MAX7456_SPI_CS_PIN   PA17

#define GYRO_1_CS_PIN        PA1
#define GYRO_2_CS_PIN        NONE

#define MAX7456_SPI_INSTANCE SPI1
#define SDCARD_SPI_INSTANCE  SPI1
#define GYRO_1_SPI_INSTANCE  SPI0

#define USE_GYRO
#define USE_GYRO_SPI_ICM42688P
#define USE_ACC
#define USE_ACC_SPI_ICM42688P
//#define USE_FLASH
//#define USE_FLASH_W25Q128FV
//#define USE_MAX7456

/*

SPI0_CS         P1
SPI0_SCLK       P2
SPI0_COPI       P3
SPI0_CIPO       P4
BUZZER          P5
LED0            P6
LED1            P7
UART1_TX        P8
UART1_RX        P9
I2C1_SDA        P10
I2C1_SCL        P11
UART0_TX        P12
UART0_RX        P13

OSD_CS          P17

UART2_TX        P20
UART2_RX        P21

GYRO_INT        P22

GYRO_CLK        P23

SPI1_CIPO       P24
SPI1_CS         P25
SPI1_SCLK       P26
SPI1_COPI       P27

MOTOR1          P28
MOTOR2          P29
MOTOR3          P30
MOTOR4          P31

SPARE1          P32
SPARE2          P33

UART3_TX        P34
UART3_RX        P35

DVTX_SBUS_RX    P36
TELEM_RX        P37
LED_STRIP       P38
RGB_LED         P39

VBAT_SENSE      P40
CURR_SENSE      P41
ADC_SPARE       P42

I2C0_SDA        P44
I2C0_SCL        P45

SPARE3          P47

*/